#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "circular_buffer_math.h"
#include <atomic>
#include <cstdint>

namespace esphome {
namespace weight_sensor {

// ============================================================================
// WeightSensorComponent
// Owns the HX711 ADC (direct GPIO polling), CircularBufferMath filtering, and
// calibration/tare persistence.  Provides the same API as WeightSensor.cpp in
// the PlatformIO firmware so GrindController can be ported verbatim.
// ============================================================================
class WeightSensorComponent : public Component {
public:
    // ── ESPHome config setters ────────────────────────────────────────────
    void set_dout_pin(InternalGPIOPin* pin)   { dout_pin_ = pin; }
    void set_clk_pin(InternalGPIOPin* pin)    { clk_pin_ = pin; }
    void set_calibration_factor(float f)      { cal_factor_ = f; }
    void set_sample_rate_sps(uint8_t sps)     { sample_rate_sps_ = sps; }

    // Optional weight output to HA
    void set_weight_sensor(sensor::Sensor* s) { weight_out_ = s; }

    // ── Component lifecycle ───────────────────────────────────────────────
    void setup() override;
    void loop() override;
    // Setup runs BEFORE the touchscreen (DATA=200). Its blocking wait for the
    // HX711's first sample doubles as a deliberate delay so the FT3168 has
    // time to come up before the touchscreen driver probes it on the I2C bus.
    // Without this, touch and weight_sensor share priority and the order is
    // non-deterministic — random per-boot touch init failures.
    float get_setup_priority() const override { return setup_priority::HARDWARE_LATE; }
    void dump_config() override;

    // ── Same public API as original WeightSensor ──────────────────────────
    bool start_nonblocking_tare();
    bool is_tare_in_progress() const  { return do_tare_; }
    bool is_settled(uint32_t window_ms = 500);

    bool check_settling_complete(uint32_t window_ms, float* settled_weight_out = nullptr);

    float get_weight_low_latency() const;
    float get_display_weight();
    float get_weight_high_latency() const;
    float get_flow_rate(uint32_t window_ms = 200) const;
    float get_flow_rate_95th_percentile(uint32_t window_ms = 200) const;
    float get_standard_deviation_g(uint32_t window_ms = 500) const;

    void  set_calibration_factor_runtime(float factor);
    float get_calibration_factor() const { return cal_factor_; }
    int32_t get_zero_offset() const      { return tare_offset_; }

    bool is_calibrated() const;
    void set_calibrated(bool v);

    void save_calibration();
    void load_calibration();
    void calibrate(float known_weight);

    bool has_hardware_fault() const { return hardware_fault_; }

    // Direct CircularBufferMath access (for advanced use by GrindController)
    CircularBufferMath* get_raw_filter() { return &raw_filter_; }

    // Publish current display weight to HA sensor
    void publish_weight();

private:
    InternalGPIOPin* dout_pin_{nullptr};
    InternalGPIOPin* clk_pin_{nullptr};
    sensor::Sensor*  weight_out_{nullptr};

    float    cal_factor_{-7050.0f};
    int32_t  tare_offset_{0};
    uint8_t  sample_rate_sps_{10};

    CircularBufferMath raw_filter_;

    bool hardware_fault_{false};
    bool initialized_{false};

    // Tare state
    bool     do_tare_{false};
    uint8_t  tare_sample_count_{0};
    int64_t  tare_accumulator_{0};
    static const uint8_t TARE_SAMPLES = 5;

    // HX711 gain encoding (128=1, 64=3, 32=2)
    uint8_t gain_pulses_{1};

    // Preferences keys (bumped suffix forces a fresh NVS slot when wiring
    // semantics change — e.g. the cal-factor sign flip).
    ESPPreferences* prefs_{nullptr};
    static constexpr const char* PREF_KEY_CAL_FACTOR    = "cal_factor_v2";
    static constexpr const char* PREF_KEY_TARE_OFFSET   = "tare_offset_v2";
    static constexpr const char* PREF_KEY_CAL_FLAG      = "calibrated_v2";
    static constexpr const char* PREF_KEY_CAL_WEIGHT    = "cal_weight_v2";

    mutable bool cal_flag_cached_{false};
    mutable bool cal_flag_value_{false};

    // HX711 low-level
    bool  hx711_is_ready() const;
    bool  hx711_read_raw(int32_t& out);
    void  hx711_set_gain(uint8_t gain);
    void  hx711_power_up();

    float raw_to_weight(int32_t raw) const {
        if (cal_factor_ == 0.0f) return 0.0f;
        return (float)(raw - tare_offset_) / cal_factor_;
    }
    int32_t weight_to_raw_threshold(float weight_g) const {
        return (int32_t)(fabsf(weight_g * cal_factor_));
    }
};

}  // namespace weight_sensor
}  // namespace esphome
