# CLAUDE.md — context for future Claude sessions

This file is the durable handoff between sessions on this project. If you're
picking up cold, read this end-to-end before doing anything. README.md is
user-facing; this file is engineering-context.

## What this project is

ESPHome port of [`~/repos/smart-grind-by-weight`](../smart-grind-by-weight)
(jaapp's Arduino/PlatformIO firmware). Same hardware (Waveshare ESP32-S3-
Touch-AMOLED-1.64 + HX711 + MAVIN/T70 load cell + motor relay), same target
accuracy (±0.03 g), same predictive grind algorithm. Wrapped in ESPHome so
every value lands in Home Assistant as a sensor/number/button.

The original repo is the **authoritative reference** for algorithm logic and
constants. When in doubt, grep there:

```
grep -rn "thing_you_are_porting" ~/repos/smart-grind-by-weight/src/
```

## Repos / paths

- **This repo:** `/Users/marcel/repos/smart-grind-by-weight-esphome`
  - GitHub: `git@github.com:Liquidmasl/smart-grind-by-weight-esphome.git`
- **Original (reference only, do not modify):**
  `/Users/marcel/repos/smart-grind-by-weight`
- **On the HA host:** files go under `/config/esphome/`. The user does git
  pull on HA and clicks Install in the ESPHome dashboard.

⚠️ **Watch the cwd when running git commands.** It's easy to commit into
the wrong repo. Use `git -C /Users/marcel/repos/smart-grind-by-weight-esphome ...`
explicitly when in doubt.

## Workflow

User edits / ports happen on Marcel's Mac. The HA host pulls + installs:

```
edit → git push (this repo) → git pull on HA → ESPHome dashboard Install
```

Marcel has standing approval to git push without per-command confirmation
(see his global `~/.claude/CLAUDE.md`). Use `Assisted-By:` trailer, not
`Co-Authored-By:`.

`secrets.yaml` lives on HA only — gitignored, never commit. If the user ever
asks to put secrets in the repo, push back.

## Stack

- **Framework:** ESP-IDF (not Arduino — ESPHome's `esp32: framework: type:
  esp-idf`). Required for `mipi_spi`. ESPHome ≥ 2025.8 needed for the
  CO5300 init sequence.
- **Display:** built-in `mipi_spi` platform, `model: CO5300`, `bus_mode: quad`,
  `dimensions.offset_width: 20` (tuned visually for the 1.64" panel —
  others land off-center with a green strip), `mirror_x/y: true,
  swap_xy: false` for the physical orientation, `invert_colors: false`.
- **Touch:** built-in `ft5x06` platform at I2C 0x38. The chip is FT3168 but
  it's register-compatible.
- **LVGL:** ESPHome's `lvgl:` declarative YAML. Pages, widgets, styles all
  in `packages/ui.yaml`. Built-in `montserrat_*` fonts (sizes up to 48
  ship; 60 needed a TTF subset for the big weight readout).
- **Custom components** (under `components/`):
  - `grind_controller`: the 16-phase grind state machine, predictive
    flow logic, pulse correction, RMT-based motor pulse generation,
    autotune (currently simplified — see `docs/PORT_GAPS.md`),
    diagnostics, NVS persistence.
  - `weight_sensor`: HX711 polling, signal filtering via
    `CircularBufferMath`, calibration. Owns the load cell entirely.
- **HA controls** are template numbers/buttons/switches/selects in
  `packages/automations.yaml`.

## Critical context (things that took hours to figure out)

### Setup priority ordering — MANDATORY for boot reliability

`weight_sensor` runs at `setup_priority::HARDWARE` (800), the touchscreen
runs at `DATA` (600). Both originally lived at `DATA` and the order
between them was non-deterministic per boot — when touch ran first it
probed the FT3168 before the chip had stabilized, the FT5x06 driver
NACK'd and marked the component `FAILED` for the rest of the boot.
Random ~50/50 touch-vs-load-cell-only behavior.

`weight_sensor`'s `setup()` blocks for a full 1 second waiting for the
HX711's first sample, and **does not early-return** when the sample
arrives. That blocking window doubles as a deliberate delay before the
touchscreen probes the bus. **Don't** restore the early-return; touch
probes will start failing again.

`HARDWARE_LATE` does NOT exist in this ESPHome version — use `HARDWARE`.

### HX711 critical section + Core 1 task isolation

`hx711_read_raw()` runs in a dedicated FreeRTOS task pinned to **Core 1**
(ESPHome's main loop and the touchscreen I2C ISR live on Core 0). The
bit-bang wraps in `vTaskSuspendAll()` / `xTaskResumeAll()` to prevent
task-level preemption of the 24-bit read. Hardware interrupts keep
firing on every core — crucially, Core 0's I2C ISR for the FT5x06 touch
driver is never blocked.

Don't change this back to `InterruptLock` — that disables CPU interrupts
on the current core, and broke the touch driver every time we tried it.

**Important debugging history.** During iteration we thought `InterruptLock`
fixed weight noise that `vTaskSuspendAll` couldn't. That was wrong. The
"noise" we saw with `vTaskSuspendAll` was actually a corrupted `cal_factor`
(`-4.423` instead of the proper `~-25000`) amplifying tiny load-cell drift
by ~5760×. A proper calibration made the readings stable with the soft
lock. We chased the wrong root cause for hours — if the weight ever looks
like it's noise-amplified, **check `cal_factor` first** before assuming
bit-bang corruption.

Samples flow Core 1 → FreeRTOS queue (16 slots, `RawSample{int32_t raw,
uint32_t ts_ms}`) → drained by `loop()` on Core 0 into `CircularBufferMath`
+ tare accumulation + HA publish. The task starts after a 2 s delay so
ESPHome setup (touchscreen probe!) has clean air on Core 0 first.

A 30-second heartbeat log line (`sampler heartbeat: core=N samples=N
dropped=N`) is in there for diagnostic visibility — boot-time `ESP_LOGI`
calls aren't visible over the wireless API which only attaches several
seconds after boot.

### Calibration sign

`cal_factor` sign depends on which way the load cell strain-gauge bridge
is wired. The user's specific cell ended up with cal_factor ≈ -25430
after calibration; YAML default is +7050. The `is_calibrated()`
heuristic compares `fabsf(fabsf(cal_factor_) - 7050.0f) > 1.0f` so both
signs of the magnitude are treated as "uncalibrated default".

NVS preference keys are suffixed `_v2` because we changed semantics once
and a v1 negative value would override the user's calibration on every
boot. If you change the sign convention again, bump to `_v3`.

### AMOLED screen-off

`lvgl.pause:` does NOT blank an AMOLED — pixels keep displaying the last
frame. The "Screen off" switch and auto-off interval navigate to a
full-black `page_off` instead. Tapping `page_off` returns to
`page_ready`.

The user wanted brightness control and the `mipi_spi` `set_brightness()`
call killed the panel — sign/range of the argument unverified. Brightness
control is currently NOT wired (HA number commented out / removed).
Re-add carefully if requested; verify the API takes float 0..1 vs uint8
0..255 first.

### Display offset 20

The 1.64" CO5300 panel needs `dimensions.offset_width: 20`. We
binary-searched this visually:

- 0 → image shifted left, big green strip on right
- 22 → shifted right (overshoot), small green strip on left
- 18 → still slight strip on right
- 20 → centered ✓

`offset_height` is 0 — no vertical shift observed. Don't break this.

### FT3168 idle NACKs

The FT3168 NACKs I2C polls when no finger is touching. The original
firmware suppressed these with `DEBUG_SUPPRESS_TOUCH_I2C_ERRORS`. In
ESPHome they show up as `[W][ft5x06.touchscreen:059]: Failed to read
status` lines, which are harmless but noisy. To suppress, add
`ft5x06.touchscreen: ERROR` under `logger.logs:`.

## File map

```
grinder.yaml                       # device root: esp32, psram, packages, wifi/api/ota, time
packages/
├─ hardware.yaml                   # i2c, spi, weight_sensor (custom), grind_controller (custom),
│                                  # touchscreen (ft5x06), display (mipi_spi), font
├─ ui.yaml                         # lvgl: pages (page_ready[tabview], page_grinding,
│                                  # page_off, page_purge_confirm, page_menu,
│                                  # page_grind_settings, page_calibration,
│                                  # page_autotune, page_diagnostics)
└─ automations.yaml                # globals, number/select/button/switch templates,
                                   # 250 ms UI refresh interval (lvgl.label.update calls),
                                   # auto-grind interval, return-on-removal interval,
                                   # screen auto-off interval

components/grind_controller/
├─ __init__.py                     # ESPHome codegen + schema
├─ grind_config.h                  # all GRIND_*, USER_*, SYS_* constants ported from original
├─ grind_controller.h              # GrindController class + GrindPhase enum + autotune state
└─ grind_controller.cpp            # phase machine, predictive logic, pulse correction,
                                   # RMT motor control, autotune, NVS prefs

components/weight_sensor/
├─ __init__.py                     # codegen + schema
├─ weight_sensor.h                 # WeightSensorComponent + HX711 helpers
├─ weight_sensor.cpp               # setup/loop, HX711 bit-bang (vTaskSuspendAll), tare,
│                                  # calibration, NVS persistence
├─ circular_buffer_math.h          # ported verbatim from original
└─ circular_buffer_math.cpp        # raw-ADC ring buffer + windowed stats + flow rate

docs/
├─ PORT_GAPS.md                    # what's missing vs original (Backfill / Deferred / Dropped)
├─ ha-dashboard.yaml               # full Lovelace dashboard YAML (paste into HA Raw config editor)
└─ ESPHOME_PORT_PLAN.md            # original handoff plan (historical; mostly executed)

fonts/
└─ Montserrat-Regular.ttf          # subset to digits + g/s/. for the 60pt weight display
                                   # (NOT in the repo — user keeps it locally; gitignored
                                   # if it ever lands in the project root)
```

## Common pitfalls (don't repeat)

- **Wrong-cwd git commits.** Use `git -C <full path>` for every git command
  unless you've just `cd`d to the repo. Confirmed bite once already.
- **Manual file copying loses files.** Use `rsync` or `git pull`, not Finder
  drag-drop one file at a time.
- **Stale build cache.** ESPHome's "Clean Build Files" sometimes doesn't
  fully clear external_components. If you see `undefined reference to
  esphome::weight_sensor::*` from the linker, that's stale cache: `rm -rf
  /config/esphome/.esphome/build/smart-grinder` from HA terminal.
- **Linter clang errors in the IDE about missing ESPHome headers.** Those
  are local include-path issues only — the actual ESPHome compile uses
  different include paths. Ignore them.
- **`!secret` references in `grinder.yaml`.** Never paste the actual secret
  values into the YAML (the user has done this once and lost the keys
  during overwrite). Always reference via `!secret`.

## User preferences (project-specific reinforcement)

- **Terse responses.** Marcel reads diffs himself. No long preambles, no
  trailing summaries. State the change and the reason in one or two
  sentences.
- **Push back on bad ideas.** Marcel has explicitly said multiple times he
  prefers honest "this won't work because X" over a try-and-fail. If
  something is going to be expensive or risky, say so before doing it.
- **Don't fabricate values.** When tuning hardware constants (display
  offsets, calibration factors), if there's no authoritative source, say
  "let's binary search this in real time" rather than guessing a number.
- **Confidence checks before reflashing.** If you're about to push
  something speculative, give a percentage confidence and the failure
  mode. Saved hours of broken-flash cycles when used.
- **Stash → fix break → unstash.** When mid-iteration on something but a
  prior change broke the build, `git stash` the in-flight work, fix the
  break first, then unstash. Don't pile changes onto a broken build.

## When picking up a cold session

1. Read this file.
2. `git log --oneline -20` to see what's recent.
3. `cat docs/PORT_GAPS.md` to see the current backlog.
4. Ask Marcel what he wants to work on — don't assume.
5. If it touches the algorithm, cross-reference the original at
   `~/repos/smart-grind-by-weight/src/` first. Don't reinvent.
