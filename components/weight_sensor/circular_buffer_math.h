#pragma once

// CircularBufferMath — ported verbatim from src/hardware/circular_buffer_math/
// Operates on raw int32_t ADC readings with time-based windowing.

#include <algorithm>
#include <cstdint>

#define HW_LOADCELL_SAMPLE_RATE_SPS_CBM  10  // used internally for sample estimates

class CircularBufferMath {
public:
    struct RawDataReading {
        int32_t raw_value;
        float   confidence;
        uint32_t settle_time_ms;
        bool    timeout_occurred;
    };

private:
    struct AdcSample {
        int32_t  raw_value;
        uint32_t timestamp_ms;
    };

    static const uint16_t MAX_BUFFER_SIZE = 1024;

    AdcSample circular_buffer[MAX_BUFFER_SIZE];
    uint16_t  write_index;
    uint16_t  samples_count;

    int32_t display_filtered_raw;
    bool    display_filter_initialized;

    mutable uint32_t flow_stable_since_ms;
    mutable bool     flow_stability_initialized;

    int      get_samples_in_window(uint32_t window_ms, int32_t* out) const;
    int32_t  apply_outlier_rejection(const int32_t* samples, int count) const;
    float    calculate_standard_deviation(const int32_t* samples, int count) const;
    int32_t  get_latest_sample() const;
    int      calculate_max_samples_for_window(uint32_t window_ms) const;

public:
    CircularBufferMath();

    void    add_sample(int32_t raw_adc_value, uint32_t timestamp_ms);
    int32_t get_smoothed_raw(uint32_t window_ms) const;
    int32_t get_instant_raw() const;
    int32_t get_raw_low_latency() const;
    int32_t get_display_raw();
    int32_t get_raw_high_latency() const;
    bool    get_window_delta(uint32_t window_ms, int32_t* delta_out,
                             uint32_t* span_ms_out = nullptr,
                             int* samples_out = nullptr) const;

    uint16_t get_sample_count() const { return samples_count; }
    uint32_t get_buffer_time_span_ms() const;

    bool  is_settled(uint32_t window_ms, int32_t threshold_raw_units) const;
    float get_settling_confidence(uint32_t window_ms) const;

    float get_raw_flow_rate(uint32_t window_ms = 200) const;
    float get_raw_flow_rate_95th_percentile(uint32_t window_ms = 200) const;
    bool  raw_flowrate_is_stable(uint32_t window_ms = 100) const;

    float   get_standard_deviation_raw(uint32_t window_ms) const;
    int32_t get_min_raw(uint32_t window_ms) const;
    int32_t get_max_raw(uint32_t window_ms) const;

    void reset_display_filter();
    void clear_all_samples();
};
