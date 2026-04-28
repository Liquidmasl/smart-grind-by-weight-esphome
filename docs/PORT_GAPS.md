# Port gaps vs the original PlatformIO firmware

This file tracks features from the original `smart-grind-by-weight` firmware
that have not been carried over to the ESPHome port, organised by intent:

- **Backfill** — worth implementing in the ESPHome port
- **Deferred** — acceptable to leave out for now, low impact
- **Dropped** — deliberately removed because ESPHome / HA replaces them

The **functional core** (predictive grind algorithm, pulse correction, tare,
calibration, settling, mechanical-instability detection) is a faithful port.
What's listed here is mostly second-tier UX, polish, and two algorithm
behaviours that materially affect accuracy.

---

## Backfill — worth doing

### Real autotune algorithm

**What it is.** The "Tune Motor Latency" / "Tune Pulses" feature. Measures
how long the motor takes to actually start moving coffee after a pulse start
signal, so the per-pulse correction durations in the predictive phase land
accurately. The motor-response latency is the whole reason the algorithm
hits ±0.03 g instead of ±0.3 g.

**What we have.** A 50-line binary-search stub in
`components/grind_controller/grind_controller.cpp::autotune_update()`. It:

1. Picks a midpoint pulse duration between `GRIND_AUTOTUNE_LATENCY_MIN_MS`
   (30) and `MAX_MS` (300).
2. Fires N test pulses, counts how many produce ≥ a threshold weight delta.
3. Halves the search step each round; converges on the shortest pulse that
   reliably moves grounds.

It's directionally correct but has no priming phase, no rejection of
inconsistent samples, and no recovery from the grinder being empty.

**What the original has** (`src/controllers/autotune_controller.cpp`,
~700 lines). A four-phase state machine:

1. **Priming.** Pre-grinds a small amount before measurements start, so the
   chute is saturated and pulse responses are representative of in-use
   behaviour rather than first-pulse cold-start.
2. **Binary search.** Same logic as ours, but with explicit sub-phases
   (`PULSE_EXECUTE` → `MOTOR_SETTLING` → `COLLECTION_DELAY` → `SCALE_SETTLING`
   → `TARE`) that wait for each physical event to complete before reading
   the result. Ours just sleeps for `GRIND_AUTOTUNE_COLLECTION_DELAY_MS` and
   reads — vulnerable to noisy reads if the scale hasn't fully settled.
3. **Verification.** Once a candidate latency is found, fires N more pulses
   to confirm the success rate (`GRIND_AUTOTUNE_SUCCESS_RATE` = 0.80) before
   committing. Our version commits as soon as one round of N hits.
4. **Tare between iterations** so weight deltas are deltas, not running
   sums against drift.

**Effort to port.** ~4–6 hours. Most of the original code translates
verbatim; the integration with `WeightSensor::check_settling_complete()`
and the existing `motor_start_pulse()` is straightforward. New persistent
state per autotune run, a couple of new sensor publishes for progress %,
and the four-phase enum.

**Recommended approach.** Port `autotune_controller.{h,cpp}` from the
original into `components/grind_controller/` as `autotune.{h,cpp}`. Keep
the existing `start_autotune()` / `cancel_autotune()` API on
`GrindController` but delegate the state machine to the new file. Replace
`autotune_update()` with calls into the new controller's `update()`.

---

## Deferred — acceptable to leave for now

| Feature | What | Why deferred |
|---|---|---|
| **Edit screen** | Long-press a profile tile to rename / change target. | Users edit values from HA today via `number.smart_grinder_preset_*_g`. On-device editing is convenience-only. |
| **Profile name editing** | "SINGLE / DOUBLE / CUSTOM" labels are hardcoded. | Same as above — HA can override; no functional impact. |
| **Chart-mode grinding screen** | Real-time flow-rate chart variant of the grind view (alternative to the arc). | The arc gives the same functional info. The chart is cosmetic. HA's `apexcharts-card` already shows a richer live chart anyway. |
| **System Info screen** | Build #, IP, uptime, free heap, signal strength on-device. | All exposed to HA via existing sensors. Only useful when HA is unreachable. |
| **Lifetime Stats screen** | Total grinds, total mass ground, mean error, blade-life proxy. | Belongs in HA history (recorder retains the same data). On-device is duplication. |
| **Detailed diagnostics screen** | Tare quality readout, sustained-noise indicator, error history. | Existing HA diagnostics card covers the same ground. |
| **Status indicator overlays** | Background tint while motor active, blinking warning icon for diagnostic alerts. | Pure visual polish. |
| **Time-mode UI variant** | Different grind-screen visualization for time vs weight mode. | Time mode works functionally; the visualization just isn't optimized for it. |
| **Swipe gestures for mode switching** | Vertical swipes on `page_ready` to switch Weight ↔ Time. | The `swipe_enabled` toggle exists; the gesture handler isn't wired. HA select handles it. |
| **Per-profile NVS-stored target weights** | Original stores each profile's target separately. | We use 3 HA-restored numbers globally; same effect. |
| **Auto-grind sensitivity / debounce tuning** | More knobs for cup-detection thresholds. | Current defaults (50 g delta, 1.5 s rearm) work fine. |

---

## Dropped — deliberately not coming back

| Feature | Replaced by |
|---|---|
| BLE custom GATT (OTA + data export) | ESPHome native OTA over Wi-Fi + HA `esphome.grind_*` events |
| Streamlit analytics tool | HA dashboard with `apexcharts-card` (`docs/ha-dashboard.yaml`) |
| Custom web flasher | ESPHome dashboard install flow |
| Mock load cell debug build | Development-only; no end-user value |
| Bluetooth on-screen settings | Wi-Fi-only by design; no BT settings to expose |
| `OTA_UPDATE_FAILED` recovery screen | ESPHome handles OTA failures with its own safe-mode flow |
| Per-build firmware version on UI | HA exposes `sensor.smart_grinder_esphome_version` |

---

## How to use this file

When picking what to work on next: prefer items in **Backfill**. If a
**Deferred** item starts to bug you in daily use, promote it. Anything in
**Dropped** is not coming back without a strong reason — those decisions
were the whole point of moving to ESPHome.
