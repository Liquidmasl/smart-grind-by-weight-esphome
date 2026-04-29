// weight_sensor.cpp — HX711 polling + filtering + persistence.
//
// HX711 bit-bang runs in a dedicated FreeRTOS task pinned to Core 0,
// while ESPHome's main loop / I2C / touchscreen all run on Core 1.
// InterruptLock around the bit-bang therefore disables interrupts on
// Core 0 only — the I2C ISR keeps firing on Core 1 and the touch driver
// is unaffected. Samples are pushed through a FreeRTOS queue and drained
// by loop() on Core 1, where they feed CircularBufferMath, tare logic
// and HA publishing.
#include "weight_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <cmath>
#include <cstring>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome {
namespace weight_sensor {

static const char* const TAG = "weight_sensor";

// ── HX711 low-level GPIO helpers ──────────────────────────────────────────────

bool WeightSensorComponent::hx711_is_ready() const {
    return dout_pin_->digital_read() == false;  // DOUT LOW = data ready
}

bool WeightSensorComponent::hx711_read_raw(int32_t& out) {
    if (!hx711_is_ready()) return false;

    // Full interrupt lock over the bit-bang: the HX711 sleeps if PD_SCK
    // stays high >~60 µs, and even hardware interrupts can push us past
    // that. vTaskSuspendAll() alone wasn't enough — task preemption was
    // covered but ISRs still produced corrupted reads.
    //
    // The earlier touch-probe-at-boot failures correlated with this lock
    // but were actually caused by setup_priority ordering (weight_sensor
    // and touchscreen both at DATA, non-deterministic). With weight_sensor
    // forced to HARDWARE priority (see header) the touch driver gets a
    // full 1-second clean-bus window before its probe, so the lock no
    // longer interferes with boot.
    int32_t raw = 0;
    {
        InterruptLock lock;
        for (int i = 0; i < 24; i++) {
            clk_pin_->digital_write(true);
            delayMicroseconds(1);
            raw = (raw << 1) | (dout_pin_->digital_read() ? 1 : 0);
            clk_pin_->digital_write(false);
            delayMicroseconds(1);
        }
        // Gain select pulses (1=128/A, 2=32/B, 3=64/A)
        for (int i = 0; i < gain_pulses_; i++) {
            clk_pin_->digital_write(true);
            delayMicroseconds(1);
            clk_pin_->digital_write(false);
            delayMicroseconds(1);
        }
    }
    // Sign extend 24-bit to 32-bit
    if (raw & 0x800000) raw |= 0xFF000000;
    out = raw;
    return true;
}

void WeightSensorComponent::hx711_set_gain(uint8_t gain) {
    if (gain <= 32)       gain_pulses_ = 2;
    else if (gain <= 64)  gain_pulses_ = 3;
    else                  gain_pulses_ = 1;
}

void WeightSensorComponent::hx711_power_up() {
    clk_pin_->digital_write(false);
    delayMicroseconds(100);
}

// ── Component lifecycle ───────────────────────────────────────────────────────

void WeightSensorComponent::setup() {
    ESP_LOGI(TAG, "Initialising HX711 (DOUT=%d CLK=%d gain=128 %dSPS)",
             dout_pin_->get_pin(), clk_pin_->get_pin(), sample_rate_sps_);

    dout_pin_->setup();
    clk_pin_->setup();
    clk_pin_->pin_mode(gpio::FLAG_OUTPUT);
    dout_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLDOWN);
    hx711_set_gain(128);
    hx711_power_up();

    // Load calibration from NVS
    load_calibration();

    // Wait for first sample (up to 1 s). This blocking window is also used
    // as a guaranteed startup delay for the FT3168 touch IC — the touchscreen
    // driver, which sits at the next priority tier down, won't start probing
    // I2C until this returns. See get_setup_priority() in the header.
    uint32_t deadline = millis() + 1000;
    while (millis() < deadline) {
        if (hx711_is_ready()) {
            int32_t dummy;
            hx711_read_raw(dummy);
            initialized_ = true;
            ESP_LOGI(TAG, "HX711 ready. cal_factor=%.1f tare_offset=%ld",
                     cal_factor_, (long)tare_offset_);
            // Don't early-return — let the rest of the setup window elapse so
            // the FT3168 still has the full delay budget before touch probes.
            uint32_t remaining = deadline - millis();
            if (remaining > 0) delay(remaining);
            break;
        }
        delay(10);
    }
    if (!initialized_) {
        ESP_LOGE(TAG, "HX711 not responding — check wiring");
        hardware_fault_ = true;
        return;
    }

    // Spawn the dedicated sampler task pinned to Core 0. ESPHome runs on
    // Core 1, so InterruptLock inside the task only blocks Core 0 — the
    // I2C/touchscreen ISRs on Core 1 keep firing.
    sample_queue_ = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(RawSample));
    if (sample_queue_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create sample queue");
        hardware_fault_ = true;
        return;
    }
    // ESPHome's main loop and the I2C ISR run on Core 0 (IDF default for
    // app_main on ESP32-S3). Pin the HX711 task to Core 1 so InterruptLock
    // here disables Core 1 IRQs only, leaving Core 0's I2C ISR alive for
    // the touchscreen driver.
    BaseType_t r = xTaskCreatePinnedToCore(
        &WeightSensorComponent::sampler_task_trampoline,
        "hx711_sampler",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &sampler_task_,
        1);                         // pin to Core 1 (opposite of ESPHome)
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sampler task");
        hardware_fault_ = true;
        return;
    }
    ESP_LOGI(TAG, "HX711 sampler task pinned to Core 1 (ESPHome on Core %d)",
             xPortGetCoreID());
}

// Trampoline → instance method
void WeightSensorComponent::sampler_task_trampoline(void* arg) {
    static_cast<WeightSensorComponent*>(arg)->sampler_task_run();
}

void WeightSensorComponent::sampler_task_run() {
    // Confirm we're actually on the core we asked for.
    ESP_LOGI(TAG, "HX711 sampler task running on Core %d", xPortGetCoreID());

    // Hold off until the rest of ESPHome's setup (notably the touchscreen
    // I2C probe at DATA priority) has finished. The lock is per-CPU and
    // shouldn't affect ISRs on the other core, but starting reads mid-
    // setup correlated with touch probe failures earlier. 2 s is plenty.
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Tight polling loop on the pinned core — wake every 5 ms, read when
    // HX711 signals ready, push to the queue. Heartbeat every 30 s so we
    // can confirm via wireless logs that the task is alive and on the
    // expected core (boot-time ESP_LOGI calls are not visible over the
    // ESPHome API, which only attaches a few seconds after boot).
    uint32_t samples_total = 0;
    uint32_t samples_dropped = 0;
    uint32_t last_heartbeat_ms = millis();
    for (;;) {
        if (hx711_is_ready()) {
            int32_t raw;
            if (hx711_read_raw(raw)) {
                RawSample s{ raw, (uint32_t)millis() };
                if (xQueueSend(sample_queue_, &s, 0) == pdTRUE) {
                    samples_total++;
                } else {
                    samples_dropped++;
                }
            }
        }
        uint32_t now = millis();
        if (now - last_heartbeat_ms >= 30000) {
            last_heartbeat_ms = now;
            ESP_LOGI(TAG, "sampler heartbeat: core=%d samples=%u dropped=%u",
                     xPortGetCoreID(), samples_total, samples_dropped);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void WeightSensorComponent::loop() {
    if (hardware_fault_ || sample_queue_ == nullptr) return;

    // Drain whatever the sampler task has queued. Each iteration runs the
    // same logic the old in-loop polling did, but the bit-bang itself is
    // safely on Core 0 now.
    RawSample s;
    while (xQueueReceive(sample_queue_, &s, 0) == pdTRUE) {
        raw_filter_.add_sample(s.raw, s.ts_ms);

        // Tare accumulation
        if (do_tare_) {
            tare_accumulator_ += s.raw;
            tare_sample_count_++;
            if (tare_sample_count_ >= TARE_SAMPLES) {
                tare_offset_ = (int32_t)(tare_accumulator_ / TARE_SAMPLES);
                do_tare_ = false;
                tare_sample_count_ = 0;
                tare_accumulator_ = 0;
                raw_filter_.reset_display_filter();
                save_calibration();
                ESP_LOGI(TAG, "Tare complete: offset=%ld", (long)tare_offset_);
            }
        }

        // Publish display weight to HA at the natural sensor rate.
        if (weight_out_ != nullptr) {
            static uint32_t last_publish = 0;
            if (s.ts_ms - last_publish >= 1000u / sample_rate_sps_) {
                last_publish = s.ts_ms;
                weight_out_->publish_state(get_display_weight());
            }
        }
    }
}

void WeightSensorComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "WeightSensor:");
    LOG_PIN("  DOUT: ", dout_pin_);
    LOG_PIN("  CLK:  ", clk_pin_);
    ESP_LOGCONFIG(TAG, "  cal_factor: %.3f", cal_factor_);
    ESP_LOGCONFIG(TAG, "  tare_offset: %ld", (long)tare_offset_);
    ESP_LOGCONFIG(TAG, "  calibrated: %s", is_calibrated() ? "YES" : "NO");
}

// ── Tare ──────────────────────────────────────────────────────────────────────

bool WeightSensorComponent::start_nonblocking_tare() {
    do_tare_ = true;
    tare_sample_count_ = 0;
    tare_accumulator_ = 0;
    return true;
}

// ── Settling ──────────────────────────────────────────────────────────────────

bool WeightSensorComponent::is_settled(uint32_t window_ms) {
    int32_t threshold_raw = weight_to_raw_threshold(0.010f);  // GRIND_SCALE_SETTLING_TOLERANCE_G
    return raw_filter_.is_settled(window_ms, threshold_raw);
}

bool WeightSensorComponent::check_settling_complete(uint32_t window_ms, float* settled_weight_out) {
    bool settled = is_settled(window_ms);
    if (settled && settled_weight_out != nullptr) {
        *settled_weight_out = get_weight_high_latency();
    }
    return settled;
}

// ── Weight getters ────────────────────────────────────────────────────────────

float WeightSensorComponent::get_weight_low_latency() const {
    return raw_to_weight(raw_filter_.get_raw_low_latency());
}

float WeightSensorComponent::get_display_weight() {
    return raw_to_weight(raw_filter_.get_display_raw());
}

float WeightSensorComponent::get_weight_high_latency() const {
    return raw_to_weight(raw_filter_.get_raw_high_latency());
}

float WeightSensorComponent::get_flow_rate(uint32_t window_ms) const {
    float raw_rate = raw_filter_.get_raw_flow_rate(window_ms);
    if (cal_factor_ == 0.0f) return 0.0f;
    return raw_rate / cal_factor_;
}

float WeightSensorComponent::get_flow_rate_95th_percentile(uint32_t window_ms) const {
    float raw_rate = raw_filter_.get_raw_flow_rate_95th_percentile(window_ms);
    if (cal_factor_ == 0.0f) return 0.0f;
    return raw_rate / cal_factor_;
}

float WeightSensorComponent::get_standard_deviation_g(uint32_t window_ms) const {
    float std_raw = raw_filter_.get_standard_deviation_raw(window_ms);
    if (cal_factor_ == 0.0f) return 0.0f;
    return std_raw / fabsf(cal_factor_);
}

// ── Calibration ───────────────────────────────────────────────────────────────

void WeightSensorComponent::calibrate(float known_weight) {
    int32_t raw_avg = raw_filter_.get_raw_high_latency();
    float new_factor = (float)(raw_avg - tare_offset_) / known_weight;
    cal_factor_ = new_factor;
    set_calibrated(true);
    save_calibration();
    ESP_LOGI(TAG, "Calibrated: known_weight=%.2f raw_avg=%ld factor=%.3f",
             known_weight, (long)raw_avg, cal_factor_);
}

void WeightSensorComponent::set_calibration_factor_runtime(float factor) {
    cal_factor_ = factor;
    cal_flag_cached_ = false;  // Invalidate cache
    ESP_LOGI(TAG, "cal_factor set to %.3f", factor);
}

bool WeightSensorComponent::is_calibrated() const {
    if (cal_flag_cached_) return cal_flag_value_;
    // Treat the factory default magnitude (|7050|) as "uncalibrated"; user
    // has calibrated once the factor is meaningfully different.
    cal_flag_value_ = (fabsf(fabsf(cal_factor_) - 7050.0f) > 1.0f);
    cal_flag_cached_ = true;
    return cal_flag_value_;
}

void WeightSensorComponent::set_calibrated(bool v) {
    cal_flag_value_  = v;
    cal_flag_cached_ = true;
}

void WeightSensorComponent::save_calibration() {
    auto pref_factor = global_preferences->make_preference<float>(fnv1_hash(PREF_KEY_CAL_FACTOR));
    auto pref_tare   = global_preferences->make_preference<int32_t>(fnv1_hash(PREF_KEY_TARE_OFFSET));
    pref_factor.save(&cal_factor_);
    pref_tare.save(&tare_offset_);
}

void WeightSensorComponent::load_calibration() {
    auto pref_factor = global_preferences->make_preference<float>(fnv1_hash(PREF_KEY_CAL_FACTOR));
    auto pref_tare   = global_preferences->make_preference<int32_t>(fnv1_hash(PREF_KEY_TARE_OFFSET));
    float f; int32_t t;
    if (pref_factor.load(&f)) cal_factor_  = f;
    if (pref_tare.load(&t))   tare_offset_ = t;
    ESP_LOGI(TAG, "Loaded calibration: factor=%.3f tare=%ld", cal_factor_, (long)tare_offset_);
}

void WeightSensorComponent::publish_weight() {
    if (weight_out_ != nullptr) {
        weight_out_->publish_state(get_display_weight());
    }
}

}  // namespace weight_sensor
}  // namespace esphome
