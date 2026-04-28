// CircularBufferMath — ported from src/hardware/circular_buffer_math/
// millis() is available in both ESPHome framework targets.

#include "circular_buffer_math.h"
#include <math.h>
#include <algorithm>
#include <cstring>
#include <alloca.h>
#include "esphome/core/hal.h"   // millis()

#define SPS  HW_LOADCELL_SAMPLE_RATE_SPS_CBM

CircularBufferMath::CircularBufferMath()
    : write_index(0), samples_count(0),
      display_filtered_raw(0), display_filter_initialized(false),
      flow_stable_since_ms(0), flow_stability_initialized(false) {
    for (uint16_t i = 0; i < MAX_BUFFER_SIZE; i++) {
        circular_buffer[i].raw_value    = 0;
        circular_buffer[i].timestamp_ms = 0;
    }
}

void CircularBufferMath::add_sample(int32_t raw_adc_value, uint32_t timestamp_ms) {
    circular_buffer[write_index].raw_value    = raw_adc_value;
    circular_buffer[write_index].timestamp_ms = timestamp_ms;
    write_index = (write_index + 1) % MAX_BUFFER_SIZE;
    if (samples_count < MAX_BUFFER_SIZE) samples_count++;
}

int32_t CircularBufferMath::get_latest_sample() const {
    if (samples_count == 0) return 0;
    uint16_t idx = (write_index - 1 + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    return circular_buffer[idx].raw_value;
}

int CircularBufferMath::get_samples_in_window(uint32_t window_ms, int32_t* out) const {
    if (samples_count == 0) return 0;
    uint32_t current_time  = millis();
    uint32_t window_start  = current_time - window_ms;
    int collected = 0;
    for (int i = 0; i < samples_count; i++) {
        uint16_t idx = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        if (circular_buffer[idx].timestamp_ms >= window_start) {
            out[collected++] = circular_buffer[idx].raw_value;
        } else {
            break;
        }
    }
    return collected;
}

int32_t CircularBufferMath::apply_outlier_rejection(const int32_t* samples, int count) const {
    if (count == 0) return 0;
    if (count == 1) return samples[0];
    if (count == 2) return (samples[0] + samples[1]) / 2;

    int reject = 1;
    if (count <= 2 * reject) {
        int32_t* s = (int32_t*)alloca(count * sizeof(int32_t));
        memcpy(s, samples, count * sizeof(int32_t));
        std::sort(s, s + count);
        return s[count / 2];
    }
    int32_t* s = (int32_t*)alloca(count * sizeof(int32_t));
    memcpy(s, samples, count * sizeof(int32_t));
    std::sort(s, s + count);
    int n = count - 2 * reject;
    int64_t sum = 0;
    for (int i = reject; i < count - reject; i++) sum += s[i];
    return (int32_t)(sum / n);
}

int CircularBufferMath::calculate_max_samples_for_window(uint32_t window_ms) const {
    int est = (window_ms * SPS) / 1000 + 10;
    if (est > (int)samples_count) est = samples_count;
    if (est > MAX_BUFFER_SIZE)    est = MAX_BUFFER_SIZE;
    return est;
}

int32_t CircularBufferMath::get_smoothed_raw(uint32_t window_ms) const {
    if (samples_count == 0) return 0;
    int mx = calculate_max_samples_for_window(window_ms);
    int32_t* s = (int32_t*)alloca(mx * sizeof(int32_t));
    int n = get_samples_in_window(window_ms, s);
    if (n == 0) return get_latest_sample();
    return apply_outlier_rejection(s, n);
}

int32_t CircularBufferMath::get_instant_raw() const {
    return get_latest_sample();
}

int32_t CircularBufferMath::get_raw_low_latency() const {
    return get_smoothed_raw(100);
}

int32_t CircularBufferMath::get_display_raw() {
    int32_t current_raw = get_smoothed_raw(300);
    if (!display_filter_initialized) {
        display_filtered_raw = current_raw;
        display_filter_initialized = true;
        return display_filtered_raw;
    }
    int32_t deadband = 100;
    if (abs(current_raw - display_filtered_raw) < deadband)
        return display_filtered_raw;
    if (current_raw > display_filtered_raw) {
        display_filtered_raw = current_raw;
    } else {
        float alpha = 0.9f;  // SYS_DISPLAY_FILTER_ALPHA_DOWN
        display_filtered_raw = (int32_t)(alpha * current_raw + (1.0f - alpha) * display_filtered_raw);
    }
    return display_filtered_raw;
}

int32_t CircularBufferMath::get_raw_high_latency() const {
    return get_smoothed_raw(300);
}

uint32_t CircularBufferMath::get_buffer_time_span_ms() const {
    if (samples_count < 2) return 0;
    uint16_t oldest = (samples_count < MAX_BUFFER_SIZE) ? 0 : write_index;
    uint16_t newest = (write_index - 1 + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    return circular_buffer[newest].timestamp_ms - circular_buffer[oldest].timestamp_ms;
}

bool CircularBufferMath::get_window_delta(uint32_t window_ms, int32_t* delta_out,
                                          uint32_t* span_ms_out, int* samples_out) const {
    if (!delta_out || samples_count < 2) {
        if (delta_out) *delta_out = 0;
        if (span_ms_out) *span_ms_out = 0;
        if (samples_out) *samples_out = 0;
        return false;
    }
    uint32_t current_time = millis();
    uint32_t window_start = current_time - window_ms;
    int collected = 0;
    int32_t newest_raw = 0, oldest_raw = 0;
    uint32_t newest_ts = 0, oldest_ts = 0;
    for (int i = 0; i < samples_count; i++) {
        uint16_t idx = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        const AdcSample& sample = circular_buffer[idx];
        if (sample.timestamp_ms < window_start) break;
        if (collected == 0) { newest_raw = sample.raw_value; newest_ts = sample.timestamp_ms; }
        oldest_raw = sample.raw_value; oldest_ts = sample.timestamp_ms;
        collected++;
    }
    if (samples_out) *samples_out = collected;
    if (collected < 2) { *delta_out = 0; if (span_ms_out) *span_ms_out = 0; return false; }
    *delta_out = newest_raw - oldest_raw;
    if (span_ms_out) {
        *span_ms_out = (newest_ts >= oldest_ts) ? (newest_ts - oldest_ts)
                       : (UINT32_MAX - oldest_ts + newest_ts + 1);
    }
    return true;
}

bool CircularBufferMath::is_settled(uint32_t window_ms, int32_t threshold_raw_units) const {
    float std_dev = get_standard_deviation_raw(window_ms);
    return std_dev <= (float)threshold_raw_units;
}

float CircularBufferMath::get_settling_confidence(uint32_t window_ms) const {
    float std_dev = get_standard_deviation_raw(window_ms);
    float max_exp = 1000.0f;
    return std::max(0.0f, std::min(1.0f, 1.0f - std_dev / max_exp));
}

float CircularBufferMath::calculate_standard_deviation(const int32_t* samples, int count) const {
    if (count <= 1) return 0.0f;
    int64_t sum = 0;
    for (int i = 0; i < count; i++) sum += samples[i];
    float mean = (float)sum / count;
    float var = 0.0f;
    for (int i = 0; i < count; i++) { float d = samples[i] - mean; var += d * d; }
    return sqrtf(var / (count - 1));
}

float CircularBufferMath::get_standard_deviation_raw(uint32_t window_ms) const {
    int mx = calculate_max_samples_for_window(window_ms);
    if (mx == 0) return 0.0f;
    int32_t* s = (int32_t*)alloca(mx * sizeof(int32_t));
    int n = get_samples_in_window(window_ms, s);
    return calculate_standard_deviation(s, n);
}

float CircularBufferMath::get_raw_flow_rate(uint32_t window_ms) const {
    int mx = calculate_max_samples_for_window(window_ms);
    if (mx < 2) return 0.0f;
    int32_t*  sv = (int32_t*)alloca(mx * sizeof(int32_t));
    uint32_t* st = (uint32_t*)alloca(mx * sizeof(uint32_t));
    int collected = 0;
    uint32_t current_time = millis();
    uint32_t window_start = current_time - window_ms;
    for (int i = 0; i < (int)samples_count && collected < mx; i++) {
        uint16_t idx = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        if (circular_buffer[idx].timestamp_ms >= window_start) {
            sv[collected] = circular_buffer[idx].raw_value;
            st[collected] = circular_buffer[idx].timestamp_ms;
            collected++;
        } else break;
    }
    if (collected < 2) return 0.0f;
    int32_t  raw_change  = sv[0] - sv[collected - 1];
    uint32_t time_change = st[0] - st[collected - 1];
    if (time_change == 0) return 0.0f;
    return (float)raw_change * 1000.0f / (float)time_change;
}

float CircularBufferMath::get_raw_flow_rate_95th_percentile(uint32_t window_ms) const {
    const uint32_t MIN_SAMPLES = 10;
    const uint32_t SUB_WINDOW_MS = 300;
    const uint32_t STEP_MS = 100;
    const int MIN_SUB_WINDOWS = 4;
    const int MAX_SUB_WINDOWS = 32;
    const int MIN_SAMPLES_SUB = 3;

    if (samples_count < MIN_SAMPLES) return get_raw_flow_rate(window_ms);

    uint32_t min_window = (MIN_SAMPLES * 1000) / SPS;
    uint32_t eff_window = std::max(window_ms, min_window);
    int mx = calculate_max_samples_for_window(eff_window);
    if (mx < (int)MIN_SAMPLES) return get_raw_flow_rate(eff_window);

    int32_t*  sample_values = (int32_t*)alloca(mx * sizeof(int32_t));
    uint32_t* sample_times  = (uint32_t*)alloca(mx * sizeof(uint32_t));
    int collected = 0;
    uint32_t current_time = millis();
    uint32_t window_start = current_time - eff_window;
    for (int i = 0; i < (int)samples_count && collected < mx; i++) {
        uint16_t idx = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        if (circular_buffer[idx].timestamp_ms >= window_start) {
            sample_values[collected] = circular_buffer[idx].raw_value;
            sample_times[collected]  = circular_buffer[idx].timestamp_ms;
            collected++;
        } else break;
    }
    if (collected < (int)MIN_SAMPLES) return get_raw_flow_rate(eff_window);

    int num_sw = (eff_window > SUB_WINDOW_MS) ? 1 + (eff_window - SUB_WINDOW_MS) / STEP_MS : 1;
    num_sw = std::max(MIN_SUB_WINDOWS, std::min(MAX_SUB_WINDOWS, num_sw));
    float* flow_rates = (float*)alloca(num_sw * sizeof(float));
    int valid = 0;
    for (int i = 0; i < num_sw; i++) {
        uint32_t sw_end   = current_time - (i * STEP_MS);
        uint32_t sw_start = sw_end - SUB_WINDOW_MS;
        int ni = -1, oi = -1;
        for (int j = 0; j < collected; j++) {
            if (sample_times[j] <= sw_end) {
                if (ni == -1) ni = j;
                if (sample_times[j] >= sw_start) oi = j; else break;
            }
        }
        if (ni != -1 && oi != -1 && (oi - ni + 1) >= MIN_SAMPLES_SUB) {
            uint32_t td = sample_times[ni] - sample_times[oi];
            if (td > 0) {
                int32_t rd = sample_values[ni] - sample_values[oi];
                flow_rates[valid++] = (float)rd * 1000.0f / (float)td;
            }
        }
    }
    if (valid >= MIN_SAMPLES_SUB) {
        std::sort(flow_rates, flow_rates + valid);
        int idx95 = std::min((int)(valid * 0.95f), valid - 1);
        return flow_rates[idx95];
    }
    return get_raw_flow_rate(eff_window);
}

bool CircularBufferMath::raw_flowrate_is_stable(uint32_t window_ms) const {
    float cur  = get_raw_flow_rate(window_ms);
    float half = get_raw_flow_rate(window_ms / 2);
    float thr  = fabsf(cur) * 0.1f;
    return fabsf(cur - half) <= thr;
}

int32_t CircularBufferMath::get_min_raw(uint32_t window_ms) const {
    int mx = calculate_max_samples_for_window(window_ms);
    if (mx == 0) return 0;
    int32_t* s = (int32_t*)alloca(mx * sizeof(int32_t));
    int n = get_samples_in_window(window_ms, s);
    if (n == 0) return 0;
    int32_t mn = s[0];
    for (int i = 1; i < n; i++) if (s[i] < mn) mn = s[i];
    return mn;
}

int32_t CircularBufferMath::get_max_raw(uint32_t window_ms) const {
    int mx = calculate_max_samples_for_window(window_ms);
    if (mx == 0) return 0;
    int32_t* s = (int32_t*)alloca(mx * sizeof(int32_t));
    int n = get_samples_in_window(window_ms, s);
    if (n == 0) return 0;
    int32_t mx_val = s[0];
    for (int i = 1; i < n; i++) if (s[i] > mx_val) mx_val = s[i];
    return mx_val;
}

void CircularBufferMath::reset_display_filter() {
    display_filter_initialized = false;
    display_filtered_raw = 0;
}

void CircularBufferMath::clear_all_samples() {
    write_index = 0;
    samples_count = 0;
    display_filter_initialized = false;
    flow_stability_initialized = false;
    for (uint16_t i = 0; i < MAX_BUFFER_SIZE; i++) {
        circular_buffer[i].raw_value    = 0;
        circular_buffer[i].timestamp_ms = 0;
    }
}
