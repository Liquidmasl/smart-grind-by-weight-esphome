// weight_sensor.cpp — HX711 polling + filtering + persistence.
//
// InterruptLock around the bit-bang is required: the HX711 enters sleep
// if PD_SCK stays high for >~60µs, which can happen if ESPHome's main loop
// is preempted mid-read. Without the lock, ~1 in 50 reads gets garbage
// bits and the weight reading jumps wildly.
#include "weight_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <cmath>
#include <cstring>
#include <driver/gpio.h>

namespace esphome {
namespace weight_sensor {

static const char* const TAG = "weight_sensor";

// ── HX711 low-level GPIO helpers ──────────────────────────────────────────────

bool WeightSensorComponent::hx711_is_ready() const {
    return dout_pin_->digital_read() == false;  // DOUT LOW = data ready
}

bool WeightSensorComponent::hx711_read_raw(int32_t& out) {
    if (!hx711_is_ready()) return false;

    // Critical section: don't get preempted during the 24+gain bits. If we
    // are, PD_SCK stays high too long and the HX711 sleeps mid-read.
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

    // Wait for first sample (up to 300 ms)
    uint32_t deadline = millis() + 300;
    while (millis() < deadline) {
        if (hx711_is_ready()) {
            int32_t dummy;
            hx711_read_raw(dummy);
            initialized_ = true;
            ESP_LOGI(TAG, "HX711 ready. cal_factor=%.1f tare_offset=%ld",
                     cal_factor_, (long)tare_offset_);
            return;
        }
        delay(10);
    }
    ESP_LOGE(TAG, "HX711 not responding — check wiring");
    hardware_fault_ = true;
}

void WeightSensorComponent::loop() {
    if (hardware_fault_) return;

    if (!hx711_is_ready()) return;

    int32_t raw;
    if (!hx711_read_raw(raw)) return;

    uint32_t now = millis();
    raw_filter_.add_sample(raw, now);

    // Tare accumulation
    if (do_tare_) {
        tare_accumulator_ += raw;
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

    // Publish display weight to HA every loop cycle
    if (weight_out_ != nullptr) {
        // Publish at natural sensor rate (not every loop tick)
        static uint32_t last_publish = 0;
        if (now - last_publish >= 1000u / sample_rate_sps_) {
            last_publish = now;
            float w = get_display_weight();
            weight_out_->publish_state(w);
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
