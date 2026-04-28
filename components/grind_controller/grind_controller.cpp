#include "grind_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <cmath>
#include <algorithm>
#include <driver/gpio.h>

namespace esphome {
namespace grind_controller {

static const char* const TAG = "grind_ctrl";

// ============================================================================
// Component lifecycle
// ============================================================================

void GrindController::setup() {
    ESP_LOGI(TAG, "GrindController setup");

    // Motor pin setup (RMT handles the actual output)
    motor_pin_->setup();
    motor_pin_->pin_mode(gpio::FLAG_OUTPUT);
    motor_pin_->digital_write(false);

    rmt_init();
    load_prefs();

    ESP_LOGI(TAG, "motor_latency=%.1f ms  chute_mode=%d  freshness=%.1f h",
             motor_latency_ms_, (int)chute_mode_, freshness_hours_);
}

void GrindController::loop() {
    // Autotune runs independently of the grind state machine.
    if (autotune_running_) {
        autotune_update();
    }

    if (!is_active()) {
        // In IDLE — publish weight + calibration status continuously
        static uint32_t last_idle_pub = 0;
        uint32_t now = millis();
        if (now - last_idle_pub >= 200) {
            last_idle_pub = now;
            if (s_current_weight_ && weight_sensor_)
                s_current_weight_->publish_state(weight_sensor_->get_display_weight());
            if (s_calibrated_ && weight_sensor_)
                s_calibrated_->publish_state(weight_sensor_->is_calibrated());
            if (s_motor_latency_)
                s_motor_latency_->publish_state(motor_latency_ms_);
            if (s_mechanical_)
                s_mechanical_->publish_state((float)mech_anomaly_count_);
        }
        return;
    }

    unsigned long now = millis();

    // Populate per-cycle values once
    float current_weight = weight_sensor_ ? weight_sensor_->get_weight_low_latency() : 0.0f;
    float flow_rate      = weight_sensor_ ? weight_sensor_->get_flow_rate() : 0.0f;

    if (control_loop_paused_) {
        publish_sensors();
        return;
    }

    monitor_mechanical_instability();

    switch (phase_) {
        case GrindPhase::INITIALIZING:
            // Immediately proceed to SETUP (no dual-core wait needed)
            switch_phase(GrindPhase::SETUP);
            break;

        case GrindPhase::SETUP:
            switch_phase(GrindPhase::TARING);
            break;

        case GrindPhase::TARING:
            if (weight_sensor_->start_nonblocking_tare())
                switch_phase(GrindPhase::TARE_CONFIRM);
            break;

        case GrindPhase::TARE_CONFIRM:
            if (!weight_sensor_->is_tare_in_progress() && weight_sensor_->is_settled()) {
                motor_start_continuous();
                time_grind_start_ms_ = now;
                if (mode_ == GrindMode::TIME)
                    switch_phase(GrindPhase::TIME_GRINDING);
                else
                    switch_phase(GrindPhase::PRIME);
            }
            break;

        case GrindPhase::PRIME: {
            if (!motor_on_) motor_start_continuous();
            bool reached = current_weight >= session_chute_amount_g_;
            bool timed_out = (now - phase_start_time_) >= GRIND_PRIME_MAX_DURATION_MS;
            if (reached || timed_out) {
                motor_stop();
                switch_phase(GrindPhase::PRIME_SETTLING);
            }
            break;
        }

        case GrindPhase::PRIME_SETTLING: {
            bool settled = weight_sensor_->check_settling_complete(GRIND_SCALE_PRECISION_SETTLING_TIME_MS);
            bool timed_out = (now - phase_start_time_) >= GRIND_SCALE_SETTLING_TIMEOUT_MS;
            if (settled || timed_out) {
                flow_start_confirmed_ = false;
                grind_latency_ms_ = 0;

                // Decide if we need purge confirmation
                bool show_purge = false;
                if (!purged_since_boot_) {
                    show_purge = true;
                } else {
                    uint64_t now_ms = (uint64_t)(millis());
                    uint64_t elapsed = now_ms - last_grind_runtime_ms_;
                    uint64_t threshold = (uint64_t)(freshness_hours_ * 3600000.0f);
                    show_purge = elapsed > threshold;
                }

                if (session_chute_mode_ == GrinderPurgeMode::PURGE && show_purge) {
                    timeout_pause_start_ = now;
                    switch_phase(GrindPhase::PURGE_CONFIRM);
                } else {
                    motor_start_continuous();
                    time_grind_start_ms_ = now;
                    switch_phase(GrindPhase::PREDICTIVE);
                }
            }
            break;
        }

        case GrindPhase::PURGE_CONFIRM:
            // Waiting for continue_from_purge() to be called by UI
            break;

        case GrindPhase::PREDICTIVE:
            run_predictive_phase();
            break;

        case GrindPhase::PULSE_DECISION:
            run_pulse_decision_phase();
            break;

        case GrindPhase::PULSE_EXECUTE:
            run_pulse_execute_phase();
            break;

        case GrindPhase::PULSE_SETTLING:
            run_pulse_settling_phase();
            break;

        case GrindPhase::FINAL_SETTLING:
            if (weight_sensor_->check_settling_complete(GRIND_SCALE_PRECISION_SETTLING_TIME_MS))
                final_measurement();
            break;

        case GrindPhase::TIME_GRINDING:
            run_time_grinding_phase();
            break;

        case GrindPhase::TIME_ADDITIONAL_PULSE:
            if (motor_pulse_complete())
                switch_phase(GrindPhase::COMPLETED);
            break;

        case GrindPhase::COMPLETED:
        case GrindPhase::TIMEOUT:
            // Terminal phases — wait for return_to_idle()
            break;

        default:
            break;
    }

    // Negative weight failsafe
    if (phase_ != GrindPhase::COMPLETED && phase_ != GrindPhase::TIMEOUT &&
        phase_ != GrindPhase::IDLE && phase_ != GrindPhase::TARING &&
        phase_ != GrindPhase::TARE_CONFIRM && motor_is_settled() &&
        current_weight < -1.0f) {
        motor_stop();
        timeout_phase_ = phase_;
        snprintf(last_error_msg_, sizeof(last_error_msg_), "Err: neg wt");
        switch_phase(GrindPhase::TIMEOUT);
    }
    // Global timeout
    else if (phase_ != GrindPhase::COMPLETED && phase_ != GrindPhase::TIMEOUT &&
             phase_ != GrindPhase::PURGE_CONFIRM && check_timeout()) {
        motor_stop();
        timeout_phase_ = phase_;
        snprintf(last_error_msg_, sizeof(last_error_msg_), "Timeout");
        switch_phase(GrindPhase::TIMEOUT);
    }

    publish_sensors();
}

void GrindController::dump_config() {
    ESP_LOGCONFIG(TAG, "GrindController:");
    LOG_PIN("  Motor pin: ", motor_pin_);
    ESP_LOGCONFIG(TAG, "  motor_latency: %.1f ms", motor_latency_ms_);
    ESP_LOGCONFIG(TAG, "  motor_settling: %u ms", motor_settling_time_ms_);
}

// ============================================================================
// Public API
// ============================================================================

void GrindController::start_grind_weight(float target_g, uint8_t profile_id) {
    if (!weight_sensor_ || weight_sensor_->has_hardware_fault()) {
        ESP_LOGE(TAG, "Cannot start grind — weight sensor fault");
        return;
    }
    target_weight_ = target_g;
    target_time_ms_ = 0;
    mode_ = GrindMode::WEIGHT;
    current_profile_id_ = profile_id;

    session_chute_mode_   = chute_mode_;
    session_chute_amount_g_ = chute_amount_g_;

    start_time_ = millis();
    pulse_attempts_ = 0;
    timeout_phase_ = GrindPhase::IDLE;
    timeout_pause_start_ = 0;
    timeout_offset_ms_ = 0;
    grind_latency_ms_ = 0;
    predictive_end_weight_ = 0;
    final_weight_ = 0;
    motor_stop_target_weight_ = GRIND_UNDERSHOOT_TARGET_G;
    flow_start_confirmed_ = false;
    pulse_flow_rate_ = 0;
    additional_pulse_count_ = 0;
    pulse_duration_ms_ = GRIND_TIME_PULSE_DURATION_MS;
    control_loop_paused_ = false;
    mech_anomaly_count_ = 0;
    mech_monitor_init_ = false;
    last_error_msg_[0] = '\0';

    switch_phase(GrindPhase::INITIALIZING);
    ESP_LOGI(TAG, "start_grind WEIGHT target=%.2fg profile=%d", target_g, profile_id);
}

void GrindController::start_grind_time(float target_time_s, uint8_t profile_id) {
    if (!weight_sensor_ || weight_sensor_->has_hardware_fault()) {
        ESP_LOGE(TAG, "Cannot start grind — weight sensor fault");
        return;
    }
    target_weight_ = 0;
    target_time_ms_ = (uint32_t)(target_time_s * 1000);
    mode_ = GrindMode::TIME;
    current_profile_id_ = profile_id;

    session_chute_mode_   = chute_mode_;
    session_chute_amount_g_ = chute_amount_g_;

    start_time_ = millis();
    pulse_attempts_ = 0;
    timeout_phase_ = GrindPhase::IDLE;
    timeout_pause_start_ = 0;
    timeout_offset_ms_ = 0;
    grind_latency_ms_ = 0;
    final_weight_ = 0;
    time_grind_start_ms_ = 0;
    additional_pulse_count_ = 0;
    pulse_duration_ms_ = GRIND_TIME_PULSE_DURATION_MS;
    control_loop_paused_ = false;
    mech_anomaly_count_ = 0;
    mech_monitor_init_ = false;
    last_error_msg_[0] = '\0';

    switch_phase(GrindPhase::INITIALIZING);
    ESP_LOGI(TAG, "start_grind TIME target=%.1fs profile=%d", target_time_s, profile_id);
}

void GrindController::stop_grind() {
    motor_stop();
    last_error_msg_[0] = '\0';
    switch_phase(GrindPhase::IDLE);
    ESP_LOGI(TAG, "Grind stopped by user");
}

void GrindController::continue_from_purge() {
    if (phase_ != GrindPhase::PURGE_CONFIRM) return;
    if (timeout_pause_start_ > 0) {
        timeout_offset_ms_ += millis() - timeout_pause_start_;
        timeout_pause_start_ = 0;
    }
    motor_start_continuous();
    time_grind_start_ms_ = millis();
    switch_phase(GrindPhase::PREDICTIVE);
    ESP_LOGI(TAG, "Purge confirmed, continuing to PREDICTIVE");
}

void GrindController::start_additional_pulse() {
    if (!can_pulse()) return;
    additional_pulse_count_++;
    start_time_ = millis();  // Reset timeout
    switch_phase(GrindPhase::TIME_ADDITIONAL_PULSE);
    motor_start_pulse(pulse_duration_ms_);
    ESP_LOGI(TAG, "Additional pulse #%d (%u ms)", additional_pulse_count_, pulse_duration_ms_);
}

void GrindController::request_tare() {
    if (weight_sensor_) {
        weight_sensor_->start_nonblocking_tare();
        ESP_LOGI(TAG, "Tare requested");
    }
}

void GrindController::start_calibration(float reference_weight_g) {
    if (!weight_sensor_) return;
    weight_sensor_->calibrate(reference_weight_g);
    if (s_calibrated_) s_calibrated_->publish_state(true);
    ESP_LOGI(TAG, "Calibration complete for %.2fg reference", reference_weight_g);
}

// ============================================================================
// Phase switch
// ============================================================================

void GrindController::switch_phase(GrindPhase new_phase) {
    if (phase_ == new_phase) return;
    ESP_LOGD(TAG, "PHASE: %s -> %s", phase_name(phase_), phase_name(new_phase));
    phase_ = new_phase;
    phase_start_time_ = millis();
    control_loop_paused_ = (phase_ == GrindPhase::PURGE_CONFIRM);

    if (new_phase == GrindPhase::IDLE) {
        motor_stop();
    }
    if (new_phase == GrindPhase::COMPLETED) {
        purged_since_boot_     = true;
        last_grind_runtime_ms_ = (uint64_t)millis();
        save_prefs();
        if (s_last_error_ && mode_ == GrindMode::WEIGHT)
            s_last_error_->publish_state(final_weight_ - target_weight_);
        if (s_last_duration_)
            s_last_duration_->publish_state((float)(millis() - start_time_));
        if (s_last_final_)
            s_last_final_->publish_state(final_weight_);
        if (s_pulse_count_)
            s_pulse_count_->publish_state((float)pulse_attempts_);
        if (s_mechanical_)
            s_mechanical_->publish_state((float)mech_anomaly_count_);
    }
    // Phase name publish drives YAML automations (HA event fire, page transitions)
    if (s_phase_name_) s_phase_name_->publish_state(phase_name(new_phase));
}

// ============================================================================
// Motor control (IDF RMT)
// ============================================================================

void GrindController::rmt_init() {
    rmt_tx_channel_config_t cfg = {};
    cfg.gpio_num            = (gpio_num_t)motor_pin_->get_pin();
    cfg.clk_src             = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz       = 1000000;  // 1 MHz → 1µs resolution
    cfg.mem_block_symbols   = 64;
    cfg.trans_queue_depth   = 4;

    if (rmt_new_tx_channel(&cfg, &rmt_channel_) == ESP_OK) {
        rmt_enable(rmt_channel_);
        rmt_initialized_ = true;
        ESP_LOGI(TAG, "RMT channel initialised on GPIO%d", motor_pin_->get_pin());
    } else {
        ESP_LOGE(TAG, "RMT init failed — motor control disabled");
    }
}

void GrindController::motor_start_continuous() {
    if (!rmt_initialized_) return;
    pulse_active_ = false;
    motor_start_time_ = millis();

    if (rmt_encoder_) { rmt_del_encoder(rmt_encoder_); rmt_encoder_ = nullptr; }
    rmt_copy_encoder_config_t enc_cfg = {};
    if (rmt_new_copy_encoder(&enc_cfg, &rmt_encoder_) != ESP_OK) return;

    rmt_symbol_word_t sym[1];
    sym[0].duration0 = 32767;
    sym[0].level0    = 1;
    sym[0].duration1 = 0;
    sym[0].level1    = 0;
    rmt_transmit_config_t tx = {};
    tx.loop_count = -1;
    rmt_transmit(rmt_channel_, rmt_encoder_, sym, sizeof(sym), &tx);
    motor_on_ = true;
}

void GrindController::motor_stop() {
    if (!rmt_initialized_) return;
    rmt_disable(rmt_channel_);
    rmt_enable(rmt_channel_);
    if (rmt_encoder_) { rmt_del_encoder(rmt_encoder_); rmt_encoder_ = nullptr; }
    motor_on_ = false;
    pulse_active_ = false;
}

void GrindController::motor_start_pulse(uint32_t duration_ms) {
    if (!rmt_initialized_) return;
    motor_start_time_ = millis();

    if (rmt_encoder_) { rmt_del_encoder(rmt_encoder_); rmt_encoder_ = nullptr; }
    rmt_copy_encoder_config_t enc_cfg = {};
    if (rmt_new_copy_encoder(&enc_cfg, &rmt_encoder_) != ESP_OK) return;

    uint32_t duration_us = duration_ms * 1000;
    rmt_symbol_word_t sym[2];
    rmt_transmit_config_t tx = {};

    if (duration_us <= 32767) {
        sym[0].level0    = 1;
        sym[0].duration0 = duration_us;
        sym[0].level1    = 0;
        sym[0].duration1 = 1;
        tx.loop_count = 0;
        pulse_active_ = true;
        motor_on_     = true;
        rmt_transmit(rmt_channel_, rmt_encoder_, sym, sizeof(rmt_symbol_word_t), &tx);
    } else {
        uint32_t base = 32767;
        uint32_t loops = (duration_us / base) - 1;
        uint32_t rem   = duration_us % base;
        sym[0].level0 = 1; sym[0].duration0 = base;
        sym[0].level1 = 1; sym[0].duration1 = rem > 0 ? rem : 1;
        sym[1].level0 = 0; sym[1].duration0 = 1;
        sym[1].level1 = 0; sym[1].duration1 = 0;
        tx.loop_count = (int)loops;
        pulse_active_ = true;
        motor_on_     = true;
        rmt_transmit(rmt_channel_, rmt_encoder_, sym, sizeof(sym), &tx);
    }
}

bool GrindController::motor_pulse_complete() {
    if (!pulse_active_) return true;
    bool done = (gpio_get_level((gpio_num_t)motor_pin_->get_pin()) == 0);
    if (done) { pulse_active_ = false; motor_on_ = false; }
    return done;
}

bool GrindController::motor_is_settled() const {
    if (motor_start_time_ == 0) return false;
    return (millis() - motor_start_time_) >= motor_settling_time_ms_;
}

// ============================================================================
// WeightGrindStrategy equivalent (inlined)
// ============================================================================

void GrindController::run_predictive_phase() {
    if (!weight_sensor_) return;
    unsigned long now = millis();

    if (!flow_start_confirmed_) {
        float fr = weight_sensor_->get_flow_rate(500);
        if (fr >= GRIND_FLOW_DETECTION_THRESHOLD_GPS) {
            grind_latency_ms_ = (float)(now - phase_start_time_);
            flow_start_confirmed_ = true;
            ESP_LOGI(TAG, "Flow confirmed latency=%.1fms fr=%.2fg/s", grind_latency_ms_, fr);
        }
    }

    if (flow_start_confirmed_) {
        const uint32_t calc_window = 1500;
        if (now > (phase_start_time_ + (unsigned long)grind_latency_ms_ + calc_window)) {
            float fr = weight_sensor_->get_flow_rate(calc_window);
            if (fr > GRIND_FLOW_DETECTION_THRESHOLD_GPS) {
                motor_stop_target_weight_ = (grind_latency_ms_ * GRIND_LATENCY_TO_COAST_RATIO /
                                             (float)SYS_MS_PER_SECOND) * fr;
            }
        }
    }

    float cw = weight_sensor_->get_weight_low_latency();
    if (motor_is_settled() &&
        cw >= (target_weight_ - motor_stop_target_weight_)) {
        motor_stop();
        predictive_end_weight_ = cw;
        pulse_flow_rate_ = weight_sensor_->get_flow_rate_95th_percentile(2500);
        switch_phase(GrindPhase::PULSE_SETTLING);
    }
}

float GrindController::clamped_pulse_flow_rate() const {
    float fr = pulse_flow_rate_;
    if (fr < GRIND_FLOW_RATE_MIN_SANE_GPS)  return GRIND_PULSE_FLOW_RATE_FALLBACK_GPS;
    if (fr > GRIND_FLOW_RATE_MAX_SANE_GPS)  return GRIND_FLOW_RATE_MAX_SANE_GPS;
    return fr;
}

float GrindController::calculate_pulse_duration_ms(float error_g) const {
    float fr  = clamped_pulse_flow_rate();
    float prod = (error_g / fr) * 1000.0f;
    float clamp = std::max(0.0f, std::min(prod, GRIND_MOTOR_MAX_PULSE_DURATION_MS));
    return motor_latency_ms_ + clamp;
}

void GrindController::run_pulse_decision_phase() {
    if (!weight_sensor_) return;
    float settled_weight;
    if (!weight_sensor_->check_settling_complete(GRIND_SCALE_PRECISION_SETTLING_TIME_MS, &settled_weight))
        return;

    float error = (target_weight_ - GRIND_ACCURACY_TOLERANCE_G) - settled_weight;

    if ((target_weight_ - settled_weight) < GRIND_ACCURACY_TOLERANCE_G ||
        pulse_attempts_ >= GRIND_MAX_PULSE_ATTEMPTS) {
        switch_phase(GrindPhase::FINAL_SETTLING);
        return;
    }

    current_pulse_duration_ms_ = calculate_pulse_duration_ms(error);
    pulse_history_[pulse_attempts_].start_weight = settled_weight;
    pulse_history_[pulse_attempts_].duration_ms  = current_pulse_duration_ms_;
    switch_phase(GrindPhase::PULSE_EXECUTE);
    motor_start_pulse((uint32_t)current_pulse_duration_ms_);
    pulse_attempts_++;
}

void GrindController::run_pulse_execute_phase() {
    if (motor_pulse_complete())
        switch_phase(GrindPhase::PULSE_SETTLING);
}

void GrindController::run_pulse_settling_phase() {
    if (!weight_sensor_) return;
    unsigned long now = millis();
    if (now - phase_start_time_ >= (unsigned long)grind_latency_ms_ + GRIND_MOTOR_SETTLING_TIME_MS) {
        if (weight_sensor_->check_settling_complete(GRIND_MOTOR_SETTLING_TIME_MS))
            switch_phase(GrindPhase::PULSE_DECISION);
    }
}

// ============================================================================
// TimeGrindStrategy equivalent (inlined)
// ============================================================================

void GrindController::run_time_grinding_phase() {
    unsigned long now = millis();
    if (time_grind_start_ms_ == 0) time_grind_start_ms_ = now;
    if (target_time_ms_ == 0 || (now - time_grind_start_ms_) >= target_time_ms_) {
        motor_stop();
        switch_phase(GrindPhase::FINAL_SETTLING);
    }
}

// ============================================================================
// Shared helpers
// ============================================================================

void GrindController::final_measurement() {
    final_weight_ = weight_sensor_->get_weight_high_latency();
    static const float NO_WEIGHT_THRESH = 0.2f;
    if (mode_ == GrindMode::WEIGHT && target_weight_ >= 1.0f &&
        final_weight_ < NO_WEIGHT_THRESH) {
        timeout_phase_ = GrindPhase::FINAL_SETTLING;
        snprintf(last_error_msg_, sizeof(last_error_msg_), "Err: no wt");
        switch_phase(GrindPhase::TIMEOUT);
        return;
    }
    switch_phase(GrindPhase::COMPLETED);
}

void GrindController::monitor_mechanical_instability() {
    if (!motor_on_) { mech_monitor_init_ = false; return; }
    float cw = weight_sensor_ ? weight_sensor_->get_weight_low_latency() : 0.0f;
    if (!mech_monitor_init_) { last_mech_weight_ = cw; mech_monitor_init_ = true; return; }
    float delta = cw - last_mech_weight_;
    unsigned long now = millis();
    if (delta <= -GRIND_MECHANICAL_DROP_THRESHOLD_G &&
        now - last_mech_event_ms_ >= GRIND_MECHANICAL_EVENT_COOLDOWN_MS) {
        mech_anomaly_count_++;
        last_mech_event_ms_ = now;
    }
    last_mech_weight_ = cw;
}

bool GrindController::check_timeout() const {
    unsigned long elapsed = millis() - start_time_;
    return (elapsed - timeout_offset_ms_) >= (GRIND_TIMEOUT_SEC * 1000UL);
}

int GrindController::get_progress_percent() const {
    if (mode_ == GrindMode::TIME) {
        if (target_time_ms_ == 0 || time_grind_start_ms_ == 0) return 0;
        unsigned long elapsed = millis() - time_grind_start_ms_;
        int p = (int)((float)elapsed / (float)target_time_ms_ * 100.0f);
        return std::max(0, std::min(100, p));
    }
    if (target_weight_ <= 0) return 0;
    float ground = (phase_ == GrindPhase::COMPLETED || phase_ == GrindPhase::TIMEOUT)
                   ? final_weight_
                   : (weight_sensor_ ? weight_sensor_->get_display_weight() : 0.0f);
    int p = (int)((ground / target_weight_) * 100.0f);
    return std::max(0, std::min(100, p));
}

float GrindController::get_current_weight() const {
    return weight_sensor_ ? weight_sensor_->get_display_weight() : 0.0f;
}

// ============================================================================
// Sensor publishing
// ============================================================================

void GrindController::publish_sensors() {
    if (s_current_weight_ && weight_sensor_)
        s_current_weight_->publish_state(weight_sensor_->get_display_weight());
    if (s_flow_rate_ && weight_sensor_)
        s_flow_rate_->publish_state(weight_sensor_->get_flow_rate());
    if (s_progress_)
        s_progress_->publish_state((float)get_progress_percent());
    if (s_motor_latency_)
        s_motor_latency_->publish_state(motor_latency_ms_);
    if (s_calibrated_ && weight_sensor_)
        s_calibrated_->publish_state(weight_sensor_->is_calibrated());
}

// ============================================================================
// Preferences persistence
// ============================================================================

void GrindController::load_prefs() {
    auto p_lat   = global_preferences->make_preference<float>(fnv1_hash("motor_lat_ms"));
    auto p_chute = global_preferences->make_preference<int>(fnv1_hash("chute_mode"));
    auto p_amt   = global_preferences->make_preference<float>(fnv1_hash("chute_amt_g"));
    auto p_fresh = global_preferences->make_preference<float>(fnv1_hash("freshness_h"));
    auto p_last  = global_preferences->make_preference<uint64_t>(fnv1_hash("last_grind"));
    auto p_swipe = global_preferences->make_preference<bool>(fnv1_hash("swipe_en"));

    float f; int i; uint64_t u; bool b;
    if (p_lat.load(&f)   && f >= GRIND_AUTOTUNE_LATENCY_MIN_MS && f <= GRIND_AUTOTUNE_LATENCY_MAX_MS)
        motor_latency_ms_ = f;
    if (p_chute.load(&i)) chute_mode_    = (GrinderPurgeMode)i;
    if (p_amt.load(&f))   chute_amount_g_ = std::max(GRIND_PURGE_AMOUNT_MIN_G, std::min(GRIND_PURGE_AMOUNT_MAX_G, f));
    if (p_fresh.load(&f)) freshness_hours_ = f;
    if (p_last.load(&u))  last_grind_runtime_ms_ = u;
    if (p_swipe.load(&b)) swipe_enabled_ = b;
}

void GrindController::save_prefs() {
    auto p_lat   = global_preferences->make_preference<float>(fnv1_hash("motor_lat_ms"));
    auto p_chute = global_preferences->make_preference<int>(fnv1_hash("chute_mode"));
    auto p_amt   = global_preferences->make_preference<float>(fnv1_hash("chute_amt_g"));
    auto p_fresh = global_preferences->make_preference<float>(fnv1_hash("freshness_h"));
    auto p_last  = global_preferences->make_preference<uint64_t>(fnv1_hash("last_grind"));
    auto p_swipe = global_preferences->make_preference<bool>(fnv1_hash("swipe_en"));

    p_lat.save(&motor_latency_ms_);
    int chute_int = (int)chute_mode_;
    p_chute.save(&chute_int);
    p_amt.save(&chute_amount_g_);
    p_fresh.save(&freshness_hours_);
    p_last.save(&last_grind_runtime_ms_);
    p_swipe.save(&swipe_enabled_);
}

// ============================================================================
// HA events: fired from automations.yaml on phase_name text_sensor transitions.
// ============================================================================

// ============================================================================
// Autotune (simplified — single-pulse latency probe)
// ============================================================================

void GrindController::start_autotune() {
    if (!weight_sensor_ || autotune_running_ || is_active()) return;
    autotune_running_ = true;
    at_current_ms_    = (GRIND_AUTOTUNE_LATENCY_MIN_MS + GRIND_AUTOTUNE_LATENCY_MAX_MS) / 2.0f;
    at_step_ms_       = (GRIND_AUTOTUNE_LATENCY_MAX_MS - GRIND_AUTOTUNE_LATENCY_MIN_MS) / 4.0f;
    at_iter_          = 0;
    at_total_iter_    = 0;
    at_success_       = 0;
    at_pre_weight_    = 0.0f;
    at_sub_           = AutoTuneSubPhase::IDLE;
    weight_sensor_->start_nonblocking_tare();
    ESP_LOGI(TAG, "AutoTune started, midpoint=%.1fms step=%.1fms", at_current_ms_, at_step_ms_);
}

void GrindController::cancel_autotune() {
    autotune_running_ = false;
    motor_stop();
    ESP_LOGI(TAG, "AutoTune cancelled");
}

void GrindController::autotune_update() {
    if (!autotune_running_ || !weight_sensor_) return;
    unsigned long now = millis();

    switch (at_sub_) {
        case AutoTuneSubPhase::IDLE: {
            // Snapshot pre-pulse weight, then fire the test pulse
            at_pre_weight_ = weight_sensor_->get_weight_high_latency();
            motor_start_pulse((uint32_t)at_current_ms_);
            at_phase_start_ = now;
            at_sub_ = AutoTuneSubPhase::PULSE;
            break;
        }
        case AutoTuneSubPhase::PULSE:
            if (motor_pulse_complete()) {
                at_phase_start_ = now;
                at_sub_ = AutoTuneSubPhase::SETTLING;
            }
            break;
        case AutoTuneSubPhase::SETTLING: {
            if (now - at_phase_start_ < GRIND_AUTOTUNE_COLLECTION_DELAY_MS) break;

            // Verification iteration complete — measure delta, decide
            float post_weight = weight_sensor_->get_weight_high_latency();
            float delta = post_weight - at_pre_weight_;
            if (delta >= GRIND_AUTOTUNE_WEIGHT_THRESHOLD_G) at_success_++;
            at_iter_++;
            at_total_iter_++;

            if (at_total_iter_ >= GRIND_AUTOTUNE_MAX_ITERATIONS) {
                ESP_LOGW(TAG, "AutoTune max iterations reached at latency=%.1fms", at_current_ms_);
                autotune_running_ = false;
                return;
            }

            if (at_iter_ >= GRIND_AUTOTUNE_VERIFICATION_PULSES) {
                float rate = (float)at_success_ / (float)at_iter_;
                if (rate >= GRIND_AUTOTUNE_SUCCESS_RATE) {
                    motor_latency_ms_ = at_current_ms_;
                    save_prefs();
                    if (s_motor_latency_) s_motor_latency_->publish_state(motor_latency_ms_);
                    ESP_LOGI(TAG, "AutoTune complete: latency=%.1fms", motor_latency_ms_);
                    autotune_running_ = false;
                    return;
                }
                // Binary search step: low success → longer pulse, high → shorter
                if (rate < 0.5f) at_current_ms_ += at_step_ms_;
                else             at_current_ms_ -= at_step_ms_;
                at_current_ms_ = std::max((float)GRIND_AUTOTUNE_LATENCY_MIN_MS,
                                 std::min((float)GRIND_AUTOTUNE_LATENCY_MAX_MS, at_current_ms_));
                at_step_ms_ = std::max(GRIND_AUTOTUNE_TARGET_ACCURACY_MS, at_step_ms_ / 2.0f);
                at_iter_    = 0;
                at_success_ = 0;
            }
            at_sub_ = AutoTuneSubPhase::IDLE;
            break;
        }
        default: break;
    }
}

// ============================================================================
// Phase name helper
// ============================================================================

const char* GrindController::phase_name(GrindPhase p) {
    switch (p) {
        case GrindPhase::IDLE:                  return "IDLE";
        case GrindPhase::INITIALIZING:          return "INITIALIZING";
        case GrindPhase::SETUP:                 return "SETUP";
        case GrindPhase::TARING:                return "TARING";
        case GrindPhase::TARE_CONFIRM:          return "TARE_CONFIRM";
        case GrindPhase::PRIME:                 return "PRIME";
        case GrindPhase::PRIME_SETTLING:        return "PRIME_SETTLING";
        case GrindPhase::PURGE_CONFIRM:         return "PURGE_CONFIRM";
        case GrindPhase::PREDICTIVE:            return "PREDICTIVE";
        case GrindPhase::PULSE_DECISION:        return "PULSE_DECISION";
        case GrindPhase::PULSE_EXECUTE:         return "PULSE_EXECUTE";
        case GrindPhase::PULSE_SETTLING:        return "PULSE_SETTLING";
        case GrindPhase::FINAL_SETTLING:        return "FINAL_SETTLING";
        case GrindPhase::TIME_GRINDING:         return "TIME_GRINDING";
        case GrindPhase::TIME_ADDITIONAL_PULSE: return "TIME_ADDITIONAL_PULSE";
        case GrindPhase::COMPLETED:             return "COMPLETED";
        case GrindPhase::TIMEOUT:               return "TIMEOUT";
        default:                                return "UNKNOWN";
    }
}

}  // namespace grind_controller
}  // namespace esphome
