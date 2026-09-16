# `IMUWidget.py` — MPU-6500 IMU / attitude display

**File:** `IMUWidget.py`
**Class:** `IMUWidget(tk.Frame)`
**Fed by:** `DroneCockpitApp._update_loop` → `update_ui(ui_data)`, every
tick (~50 Hz, no throttling — see
[app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))

## Responsibility

Displays live accelerometer/gyroscope readings and derived attitude
health, using an explicit three-state aviation alerting model rather than
plain color-coded text.

## Aviation alerting model

Modeled directly on EASA CS-25 / FAA AC 25.1322 alerting conventions, with
contrast ratios called out in the source comments:

| State | Look | Contrast |
|---|---|---|
| **SAFE** | Dark cell background (`#0d1520`), green text (`#00ff88`) | — |
| **WARN** | Solid amber fill (`#C87000`), black text | Black-on-amber ≈ 4.6:1 (WCAG AA) |
| **CRIT** | Flashing red, alternating `#CC0000`/`#500000`, white text always | White-on-`#CC0000` ≈ 5.1:1 (WCAG AA); flash cadence 500 ms on / 500 ms off per DO-160/EASA convention |

All three states keep the numeric value fully legible — the point of the
model is that a pilot glancing at the panel never has to squint through a
color to read the number underneath it.

Flashing is centralized: a single shared `_tick()` loop walks a registry of
currently-flashing cells rather than each cell owning its own Tk `after()`
timer. Adding or removing a cell from the flash set is O(1) — a cell is
added when it crosses into CRIT and removed when it recovers.

## Scale factors

```python
GYRO_SCALE  = 16.4     # MSP counts → °/s
ACCEL_SCALE = 2048.0   # MSP counts → g
```

These mirror the C++ `IMUSensor::GYRO_SCALE`/`ACC_SCALE` constants exactly
(see [../modules/sensors.md](../modules/sensors.md#imusensor--raw--physical-unit-conversion))
— used here for any raw-count display this widget does independently of
the already-scaled fields `telemetry_worker.py` may provide. If one side
of this scale pair is ever "corrected", check the other: they encode the
same physical assumption (MPU-6500 at ±16g / ±2000°/s) and are meant to
agree.

## Alerting thresholds

```python
ANGLE_WARN_DEG = 15.0   ANGLE_CRIT_DEG = 30.0
GYRO_WARN_DPS  = 30.0   GYRO_CRIT_DPS  = 100.0
HOLD_WARN_SEC  = 2.0    HOLD_CRIT_SEC  = 4.0
ACC_LAT_WARN   = 0.26   ACC_LAT_CRIT   = 0.50
```

The `HOLD_*` pair implies a **debounce/hold-time** requirement before a
threshold crossing escalates alert state — a brief spike doesn't
immediately flash red; the condition has to persist.

## Layout tiers

Four progressive tiers chosen from available width/height, same pattern
used across every other widget in this app:

| Tier | Threshold |
|---|---|
| FULL | h ≥ 210, w ≥ 400 |
| MEDIUM | h ≥ 150, w ≥ 290 |
| COMPACT | h ≥ 90, w ≥ 190 |
| TINY | h < 90 or w < 190 |
