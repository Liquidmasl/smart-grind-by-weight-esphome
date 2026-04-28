# Smart Grind-by-Weight — ESPHome Port

ESPHome 1:1 functional clone of the [Smart Grind-by-Weight PlatformIO firmware](../smart-grind-by-weight/), targeting the same hardware.  Every value, sensor, and calibration parameter is exposed to Home Assistant.

## Hardware

| Component | Part |
|---|---|
| MCU | Waveshare ESP32-S3 1.64" AMOLED (280×456 px) |
| Display IC | CO5300 / SH8501B, QSPI |
| Touch IC | FT3168, I2C 0x38 |
| Load cell ADC | HX711, 10 SPS, channel A gain 128 |
| Motor relay | Active-high GPIO |

### Pin map

| Signal | GPIO |
|---|---|
| Touch SDA | 47 |
| Touch SCL | 48 |
| Display CS | 9 |
| Display SCK | 10 |
| Display D0–D3 | 11, 12, 13, 14 |
| Display RESET | 21 |
| HX711 DOUT | 3 |
| HX711 CLK | 2 |
| Motor relay | 18 |

Wiring diagrams and BOM: see [`docs/DOC.md`](../smart-grind-by-weight/docs/DOC.md) in the sibling repo — hardware is identical.

## Features

- **Grind-by-weight** — predictive stop + up to 10 pulse corrections, ±0.03 g accuracy
- **Grind-by-time** — timed grind with additional-pulse completion button
- **Purge / Prime** — saturate grinder chute before main grind; shows confirmation popup when stale
- **Freshness tracking** — auto-purge if last grind was more than N hours ago (configurable)
- **Motor latency auto-tune** — binary search finds optimal motor response latency
- **Calibration** — zero + known-weight two-point calibration
- **LVGL touchscreen UI** — 7 screens matching the original firmware
- **Home Assistant integration** — all live values, last-grind stats, and controls exposed
- **OTA** — ESPHome native Wi-Fi OTA (replaces BLE OTA)
- **Grind events** — `grind_started` and `grind_completed` HA events with full payload

## What's NOT here (dropped by design)

| Feature | Replacement |
|---|---|
| BLE custom GATT server | ESPHome native OTA + Wi-Fi |
| Streamlit analytics tool | HA history graphs (see starter dashboard below) |
| Web flasher | ESPHome standard install flow |
| FreeRTOS dual-core tasks | ESPHome single-loop architecture |

## Repository layout

```
smart-grind-by-weight-esphome/
├── grinder.yaml              # Main ESPHome config entry point
├── secrets.yaml.example      # Copy to secrets.yaml and fill in
├── packages/
│   ├── hardware.yaml         # GPIO, I2C, display, weight sensor, grind controller
│   ├── ui.yaml               # LVGL screens (7 pages)
│   └── automations.yaml      # Numbers, selects, buttons, switches, interval callbacks
├── components/
│   ├── grind_controller/     # Grind algorithm (phases, predictive, pulse corrections)
│   ├── weight_sensor/        # HX711 + CircularBufferMath filtering
│   └── sh8501_amoled/        # QSPI display driver (CO5300 / SH8501B)
├── fonts/                    # Place Montserrat-Regular.ttf here
└── images/                   # Icons (currently unused)
```

## Getting started

### 1. Install ESPHome

```bash
pip install esphome
```

### 2. Add fonts

Download [Montserrat-Regular.ttf](https://fonts.google.com/specimen/Montserrat) and place it in `fonts/`.

### 3. Configure secrets

```bash
cp secrets.yaml.example secrets.yaml
# Edit secrets.yaml with your Wi-Fi, API key, and OTA password
```

### 4. First flash (USB)

```bash
esphome run grinder.yaml
```

Subsequent flashes can use OTA:
```bash
esphome upload grinder.yaml
```

### 5. Calibrate the scale

1. In Home Assistant → **Smart Grinder** → **Tare Scale** (empty portafilter basket)
2. Place a 100 g reference weight
3. Press **Start Calibration**
4. Note the `Calibration Factor` sensor value for backup

### 6. Run auto-tune

1. Place an empty portafilter under the grinder
2. In HA → **Start Autotune** (or via the device's Autotune screen)
3. Wait ~2 minutes; the controller will find the motor response latency and save it

## Migrating from the PlatformIO firmware

1. On the original device, open **Menu → Diagnostics → System Info** and note:
   - Calibration factor
   - Motor latency
2. Flash the ESPHome firmware
3. In HA, set **Calibration Factor** and **Motor Latency** numbers to the values you noted

## Home Assistant starter dashboard

Add this to your `configuration.yaml` or Lovelace YAML:

```yaml
# Template sensors for per-grind statistics
template:
  - sensor:
      - name: "Grind Accuracy"
        state: >
          {{ states('sensor.smart_grinder_last_grind_error') | float | abs | round(2) }}
        unit_of_measurement: g

# Example history graph card
type: history-graph
entities:
  - entity: sensor.smart_grinder_last_grind_error
    name: Error (g)
  - entity: sensor.smart_grinder_last_grind_duration
    name: Duration (ms)
hours_to_show: 168
```

For per-grind events, create an automation in HA that listens to `esphome.grind_completed`:

```yaml
trigger:
  - platform: event
    event_type: esphome.grind_completed
action:
  - service: logbook.log
    data:
      name: Grinder
      message: >
        Profile {{ trigger.event.data.profile_id }},
        {{ trigger.event.data.final_g }}g (error {{ trigger.event.data.error_g }}g),
        {{ trigger.event.data.pulse_count }} pulses,
        {{ trigger.event.data.duration_ms }}ms
```

## Architecture

The ESPHome port follows the same layered design as the original firmware but collapses the FreeRTOS dual-core split into a single-threaded `loop()`:

```
hardware.yaml           → pin declarations, ESPHome component config
components/weight_sensor → HX711 GPIO + CircularBufferMath (10 SPS)
components/grind_controller → 16-phase state machine, IDF RMT pulses,
                              predictive algorithm, auto-tune
components/sh8501_amoled → IDF QSPI SPI master display driver
packages/ui.yaml        → LVGL 9 declarative screens
packages/automations.yaml → numbers/selects/buttons exposed to HA,
                            auto-grind, return-on-removal, UI refresh
```

### Grind phases (identical to PlatformIO firmware)

`IDLE → INITIALIZING → SETUP → TARING → TARE_CONFIRM → PRIME → PRIME_SETTLING → [PURGE_CONFIRM →] PREDICTIVE → PULSE_SETTLING → PULSE_DECISION → PULSE_EXECUTE → … → FINAL_SETTLING → COMPLETED`

Time mode: `TARE_CONFIRM → TIME_GRINDING → FINAL_SETTLING → COMPLETED [→ TIME_ADDITIONAL_PULSE →]`

## Known limitations / open items

1. **Display driver init sequence** — The CO5300 register init in `sh8501_amoled.cpp` is based on publicly available Waveshare BSP references.  If your panel has a different revision, compare with the Arduino_GFX `Arduino_CO5300` driver and adjust `init_display_registers()`.

2. **LVGL version** — Requires ESPHome ≥ 2024.3 which ships LVGL 9.  The `on_swipe` handler syntax may vary; check the ESPHome LVGL docs for your version.

3. **FT3168 touchscreen** — ESPHome's `ft63x6` platform targets FT6336/FT6X36.  The FT3168 is register-compatible but verify the interrupt pin is wired and configured correctly.

4. **Motor relay active-low** — The hardware config assumes active-high relay.  If your relay is active-low, add `inverted: true` to `motor_pin` in `hardware.yaml`.

5. **A/B accuracy validation** — Run 10 grinds side-by-side with the original firmware to confirm ±0.03 g is maintained.  The algorithm is a direct port but single-threaded timing may differ slightly.
