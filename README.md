# Smart Grind-by-Weight — ESPHome Port

ESPHome reimplementation of the
[Smart Grind-by-Weight PlatformIO firmware](https://github.com/jaapp/smart-grind-by-weight)
by jaapp, targeting the same hardware. The grind algorithm and accuracy targets
are unchanged; the firmware is wrapped in ESPHome conventions so every value,
sensor, control, and calibration parameter is exposed to Home Assistant.

**Status:** functional. Display, touch, load cell, motor, predictive grind
algorithm, calibration, and HA integration all working. A few second-tier
features from the original are deferred or dropped — see
[`docs/PORT_GAPS.md`](docs/PORT_GAPS.md).

## Hardware

| Component | Notes |
|---|---|
| **Waveshare ESP32-S3-Touch-AMOLED-1.64** | 280×456 AMOLED, CO5300 controller (QSPI), FT3168 capacitive touch |
| **HX711** | 24-bit ADC for the load cell |
| **MAVIN / T70 load cell** | 0.3–1 kg range; mounting per original docs |
| **Motor relay** | Drives the grinder motor's existing control input. GPIO 18 active-high. |
| **1000 µF capacitor** | On 5 V rail — brownout protection during motor inrush |

Wiring is identical to the original; see the
[original docs](https://github.com/jaapp/smart-grind-by-weight/blob/main/docs/DOC.md)
for parts list, 3D-printed mounts, and step-by-step assembly.

### Pin map

| Function | GPIO |
|---|---|
| Display CS | 9 |
| Display SCK | 10 |
| Display D0–D3 | 11, 12, 13, 14 |
| Display RESET | 21 |
| Touch SDA | 47 |
| Touch SCL | 48 |
| HX711 DOUT | 3 |
| HX711 SCK | 2 |
| Motor relay | 18 |

## Architecture

```
grinder.yaml                       # main device config
├─ packages/
│  ├─ hardware.yaml                # GPIO, I2C, SPI, display, touch, weight, motor
│  ├─ ui.yaml                      # LVGL pages, widgets, styles
│  └─ automations.yaml             # HA-exposed numbers/buttons/switches/selects, intervals, scripts
└─ components/                     # custom external components (C++)
   ├─ grind_controller/            # phase machine + predictive grind algorithm + RMT motor pulses
   └─ weight_sensor/               # HX711 driver + filtering + flow-rate analysis (CircularBufferMath)
```

The grind algorithm (16-phase state machine, predictive flow detection,
pulse-correction loop, mechanical-anomaly detection, prime/purge handling) is
ported verbatim from the original PlatformIO C++ into the
`grind_controller` external component. The hardware abstraction lives in
`weight_sensor` (HX711 polling + signal filtering).

The display is the **built-in `mipi_spi` ESPHome platform** with
`model: CO5300`. No custom display driver — that was an early dead-end.
The touch IC (FT3168) speaks the FT5x06 register set, so the built-in
`platform: ft5x06` handles it.

## Setup

### Prerequisites

- Home Assistant with the **ESPHome add-on** (≥ 2025.8 — earlier versions
  don't ship the CO5300 init sequence in `mipi_spi`)
- HACS with **`apexcharts-card`** for the dashboard
- A 32-byte base64 string for `api_encryption_key` (HA generates one when you
  add the device)

### First flash (one-time, via USB)

The device has never had your Wi-Fi creds, so OTA can't reach it yet.

1. Copy this repo's contents into `/config/esphome/` on your HA host:
   ```bash
   rsync -av \
     --exclude='.git' --exclude='secrets.yaml' --exclude='.esphome' \
     /path/to/smart-grind-by-weight-esphome/ \
     root@homeassistant.local:/config/esphome/
   ```
2. Create `/config/esphome/secrets.yaml`:
   ```yaml
   wifi_ssid: "YourWifi"
   wifi_password: "..."
   ap_password: "..."
   api_encryption_key: "..."   # let HA generate when adding device
   ota_password: "..."
   ```
3. In the ESPHome dashboard, click `smart-grinder` → **Install** → "Plug into
   the computer running ESPHome Dashboard". Connect the board via USB-C to
   that machine.
4. After the first boot the device joins your Wi-Fi. From then on every
   subsequent change is `Install → Wirelessly` (~30 s).

### Iterating

```
edit on Mac  →  git push  →  git pull on HA  →  ESPHome dashboard Install
```

`secrets.yaml` stays on HA only — never committed. The `.gitignore`
excludes it along with `.esphome/build/` and macOS cruft.

## Calibration

Default `cal_factor` is `+7050` in `hardware.yaml`. Sign depends on load-cell
wiring direction — flip in YAML if reading is negative when weight is added.
Magnitude gets refined by the calibration procedure:

1. Set the **HA "Calibration Reference Weight (g)"** number to your reference
   (anything you can verify on a kitchen scale — two German 50-cent coins
   = 15.6 g works well).
2. Empty scale → press **Tare** (HA button or device menu).
3. Place the reference → press **Calibrate**.

`cal_factor` is persisted to NVS. Bumping the `_v2` suffix in
`weight_sensor.h::PREF_KEY_*` forces a fresh slot if the YAML default sign
ever changes.

## Home Assistant entities

ESPHome auto-creates HA entities under the device name `smart_grinder`. Pattern:
`<domain>.smart_grinder_<entity>`. Highlights:

**Live**: `sensor.smart_grinder_grind_weight`, `flow_rate`, `grind_progress`,
`grind_phase` (text)

**Buttons**: `tare_scale`, `start_grind`, `stop_grind`, `additional_pulse`,
`continue_from_purge`, `start_calibration`, `start_autotune`,
`grind_single`, `grind_double`, `grind_custom`

**Numbers**: `target_weight_g`, `target_time_s`, `motor_latency_ms`,
`calibration_factor`, `calibration_reference_weight_g`, `freshness_hours`,
`purge_amount_g`, `preset_single_g`, `preset_double_g`, `preset_custom_g`,
`screen_timeout`

**Selects**: `grind_mode` (Weight/Time), `chute_mode` (Prime/Purge)

**Switches**: `auto_grind_on_cup`, `return_on_removal`, `swipe_gestures`,
`screen` (on/off)

**Events fired** on every grind: `esphome.grind_started` and
`esphome.grind_completed` with full payload (profile, target, final, error,
pulse count, duration).

A starter Lovelace dashboard with live gauges, profile shortcuts, last-shot
results, error history, calibration controls and diagnostics lives in
[`docs/ha-dashboard.yaml`](docs/ha-dashboard.yaml).

## Differences from the original

The grind algorithm and accuracy targets are unchanged. What's different:

- **OTA + analytics** are HA-native, not BLE. Streamlit replaced by Lovelace
  + `apexcharts-card`.
- **Display offset** for the 1.64" panel is `offset_width: 20` (tuned visually).
- **HX711 noise mitigation** uses `vTaskSuspendAll()` over the bit-bang
  rather than full `InterruptLock` — the latter breaks the touch I2C probe
  at boot.
- **Setup ordering** is forced: `weight_sensor` runs at
  `setup_priority::HARDWARE` (800), one tier above the touchscreen at
  `DATA` (600). Its 1-second blocking HX711 init also acts as a guaranteed
  delay so the FT3168 has time to come up before the touchscreen probes.

Full gap list and what's worth backfilling: see [`docs/PORT_GAPS.md`](docs/PORT_GAPS.md).

## Credits

- Original [Smart Grind-by-Weight](https://github.com/jaapp/smart-grind-by-weight)
  by jaapp — the algorithm, the hardware design, the docs, and the months
  of iteration we benefited from.
- [openGBW](https://github.com/jb-xyz/openGBW) and
  [Coffee Grinder Smart Scale](https://besson.co/projects/coffee-grinder-smart-scale)
  — the lineage that the original drew from.
- ESPHome 2025.8+ for native `mipi_spi` CO5300 support that made this port
  practical without writing a custom display driver.
