#pragma once

// ============================================================================
// Grind configuration constants — ported from src/config/ in the PlatformIO
// firmware.  Single header consumed by grind_controller and weight_sensor.
// ============================================================================

// ── Hardware ──────────────────────────────────────────────────────────────────
#define HW_LOADCELL_SAMPLE_RATE_SPS     10
#define HW_LOADCELL_SAMPLE_INTERVAL_MS  (1000 / HW_LOADCELL_SAMPLE_RATE_SPS)
#define HW_LOADCELL_CAL_MIN_ADC_VALUE   1000
#define HW_GRINDER_SETTLING_TIME_MS     500  // overridden by component config

// ── User profiles ─────────────────────────────────────────────────────────────
#define USER_PROFILE_COUNT              3
#define USER_PROFILE_NAME_MAX_LENGTH    8
#define USER_SINGLE_ESPRESSO_WEIGHT_G   9.0f
#define USER_DOUBLE_ESPRESSO_WEIGHT_G   18.0f
#define USER_CUSTOM_PROFILE_WEIGHT_G    21.5f
#define USER_SINGLE_ESPRESSO_TIME_S     5.0f
#define USER_DOUBLE_ESPRESSO_TIME_S     10.0f
#define USER_CUSTOM_PROFILE_TIME_S      12.0f
#define USER_MIN_TARGET_WEIGHT_G        5.0f
#define USER_MAX_TARGET_WEIGHT_G        1000.0f
#define USER_MIN_TARGET_TIME_S          0.5f
#define USER_MAX_TARGET_TIME_S          25.0f
#define USER_FINE_WEIGHT_ADJUSTMENT_G   0.1f
#define USER_FINE_TIME_ADJUSTMENT_S     0.1f
#define USER_CALIBRATION_REFERENCE_WEIGHT_G  100.0f
#define USER_DEFAULT_CALIBRATION_FACTOR      -7050.0f

// ── Screen & display ──────────────────────────────────────────────────────────
#define USER_SCREEN_AUTO_DIM_TIMEOUT_MS  300000
#define USER_SCREEN_BRIGHTNESS_NORMAL    1.0f
#define USER_SCREEN_BRIGHTNESS_DIMMED    0.35f
#define USER_WEIGHT_ACTIVITY_THRESHOLD_G 1.0f
#define HW_DISPLAY_MINIMAL_BRIGHTNESS_PERCENT 15

// ── Auto-grind trigger ────────────────────────────────────────────────────────
#define USER_AUTO_GRIND_TRIGGER_DELTA_G       50.0f
#define USER_AUTO_GRIND_TRIGGER_WINDOW_MS     5000
#define USER_AUTO_GRIND_TRIGGER_SETTLING_MS   1000
#define USER_AUTO_GRIND_REARM_DELAY_MS        1500

// ── Grind control ─────────────────────────────────────────────────────────────
#define GRIND_ACCURACY_TOLERANCE_G            0.03f
#define GRIND_TIMEOUT_SEC                     60
#define GRIND_MAX_PULSE_ATTEMPTS              10
#define GRIND_FLOW_DETECTION_THRESHOLD_GPS    0.5f
#define GRIND_UNDERSHOOT_TARGET_G             1.0f
#define GRIND_LATENCY_TO_COAST_RATIO          1.0f
#define GRIND_PRIME_TARGET_WEIGHT_G           1.0f
#define GRIND_PRIME_MAX_DURATION_MS           5000
#define GRIND_SCALE_SETTLING_TOLERANCE_G      0.010f
#define GRIND_TIME_PULSE_DURATION_MS          100

// ── Flow rate ─────────────────────────────────────────────────────────────────
#define GRIND_FLOW_RATE_MIN_SANE_GPS          1.0f
#define GRIND_FLOW_RATE_MAX_SANE_GPS          3.0f
#define GRIND_PULSE_FLOW_RATE_FALLBACK_GPS    1.5f

// ── Motor timing ──────────────────────────────────────────────────────────────
#define GRIND_MOTOR_RESPONSE_LATENCY_DEFAULT_MS  50.0f
#define GRIND_MOTOR_MAX_PULSE_DURATION_MS        250.0f
#define GRIND_MOTOR_SETTLING_TIME_MS             200

// ── Mechanical instability detection ─────────────────────────────────────────
#define GRIND_MECHANICAL_DROP_THRESHOLD_G        0.4f
#define GRIND_MECHANICAL_EVENT_COOLDOWN_MS       200
#define GRIND_MECHANICAL_EVENT_REQUIRED_COUNT    3

// ── Scale settling ────────────────────────────────────────────────────────────
#define GRIND_SCALE_PRECISION_SETTLING_TIME_MS   500
#define GRIND_SCALE_SETTLING_TIMEOUT_MS          10000
#define GRIND_TARE_SAMPLE_WINDOW_MS              500
#define GRIND_TARE_TIMEOUT_MS                    3000
#define GRIND_CALIBRATION_SAMPLE_WINDOW_MS       800
#define GRIND_CALIBRATION_TIMEOUT_MS             2000
#define GRIND_TARE_SAMPLE_COUNT  (GRIND_TARE_SAMPLE_WINDOW_MS / HW_LOADCELL_SAMPLE_INTERVAL_MS)
#define GRIND_CALIBRATION_SAMPLE_COUNT (GRIND_CALIBRATION_SAMPLE_WINDOW_MS / HW_LOADCELL_SAMPLE_INTERVAL_MS)

// ── Purge / prime ─────────────────────────────────────────────────────────────
#define GRIND_PURGE_MODE_DEFAULT              1  // PURGE
#define GRIND_PURGE_AMOUNT_DEFAULT_G          1.0f
#define GRIND_PURGE_AMOUNT_MIN_G              0.1f
#define GRIND_PURGE_AMOUNT_MAX_G              2.5f
#define GRIND_FRESHNESS_DEFAULT_HOURS         8.0f

// ── Auto-tune ─────────────────────────────────────────────────────────────────
#define GRIND_AUTOTUNE_LATENCY_MIN_MS         30.0f
#define GRIND_AUTOTUNE_LATENCY_MAX_MS         300.0f
#define GRIND_AUTOTUNE_PRIMING_PULSE_MS       1000
#define GRIND_AUTOTUNE_TARGET_ACCURACY_MS     5.0f
#define GRIND_AUTOTUNE_SUCCESS_RATE           0.80f
#define GRIND_AUTOTUNE_VERIFICATION_PULSES    5
#define GRIND_AUTOTUNE_MAX_ITERATIONS         50
#define GRIND_AUTOTUNE_COLLECTION_DELAY_MS    1500
#define GRIND_AUTOTUNE_SETTLING_TIMEOUT_MS    5000
#define GRIND_AUTOTUNE_WEIGHT_THRESHOLD_G     GRIND_SCALE_SETTLING_TOLERANCE_G

// ── System ────────────────────────────────────────────────────────────────────
#define SYS_MS_PER_SECOND                     1000
#define SYS_DISPLAY_FILTER_ALPHA_DOWN         0.9f
#define SYS_TASK_GRIND_CONTROL_INTERVAL_MS    20

// ── Grind modes ───────────────────────────────────────────────────────────────
enum class GrindMode {
    WEIGHT = 0,
    TIME   = 1,
};

enum class GrinderPurgeMode {
    PRIME = 0,
    PURGE = 1,
};

// ── Color palette (RGB888) ────────────────────────────────────────────────────
#define COLOR_PRIMARY     0xFF0000  // Red
#define COLOR_ACCENT      0x00AAFF  // Blue
#define COLOR_SUCCESS     0x00AA00  // Green
#define COLOR_WARNING     0xCC8800  // Orange
#define COLOR_BACKGROUND  0x000000  // Black
#define COLOR_TEXT        0xFFFFFF  // White
