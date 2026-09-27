# Link-aware UI — USB cable vs ELRS radio

**Files:** `link_mode.py` (shared helper) and every instrument widget that
imports it: `IMUWidget.py`, `MagWidget.py`, `Drone3DView.py`,
`BaroWidget.py`, `GPSWidget.py`, `FCStatusWidget.py`, `ArmingWidget.py`
**Tests:** `TestScripts/test_link_mode.py` (UT-LINK-001 … 009)

## Why

The cockpit gets telemetry from one of two sources, and
`TelemetryWorker` switches between them automatically (see
[workers.md](workers.md#two-telemetry-sources--usb-vs-elrs)):

| | USB (DroneLink, MSP) | Radio (CrsfLink, CRSF over ELRS) |
|---|---|---|
| Attitude | every poll (~100 Hz) | ATTITUDE frame, typically several Hz |
| Raw gyro / accelerometer / magnetometer | yes | **no** |
| Battery, flight mode, armed | yes | yes (~1 Hz) |
| GPS position, speed, course, satellite count | yes | yes (~1 Hz) |
| GPS fix type, HDOP, satellite list | yes | **no** |
| Motor outputs, RC channels | yes | **no** |
| CPU load, loop time, I2C errors, PID profile | yes | **no** |
| Arming-disable reasons | bitmask | only "blocked yes/no" (`!ERR`) |
| Link quality | RSSI-derived | real ELRS uplink LQ |
| Calibration commands | yes | **no** (listen-only link) |

Before this change the widgets showed zeros for everything the radio
doesn't carry. Those zeros looked like real readings: the IMU flashed a
red vertical-G alarm at "0 g", the GPS said "3D FIX", motors read idle,
and the calibration buttons looked usable.

## The three rules

Every widget follows the same three rules, through `link_mode.py`:

1. **Not carried → grey placeholder.** If
   `link_mode.available(data, what)` is `False`, the widget shows
   `USB` / `USB ONLY` in the grey `C_NA_FG` colour. It never shows a zero
   or runs an alarm on that value.
2. **Old → muted amber "STALE".** `link_mode.stale_ms(data, group)`
   returns the age of a data group once it is past its limit
   (`STALE_ATTITUDE_MS` = 1.5 s for attitude, `STALE_SLOW_MS` = 3 s for
   GPS, battery, flight mode and baro). The **last value stays visible**;
   only its colour and label change. On USB this is always 0.
3. **Say where it comes from.** A small `USB` / `RADIO` tag
   (`link_mode.source_tag`) sits in the header of the IMU, FC status and
   altitude panels.

## What each widget does on the radio link

| Widget | Radio behaviour |
|---|---|
| `IMUWidget` | Rotation and G-force cells show `USB` with no alarms. Angle cells stay live and turn amber when stale. ADJUST HEADING and "Δ HDG" are disabled (the radio heading *is* the FC yaw). The latency footer becomes ATT AGE / ATT RATE / RADIO LQ. Gyro-offset calibration is abandoned. |
| `MagWidget` | Label "FC HEADING · RADIO", status "FC HDG · RADIO" (or `STALE n.ns`), XYZ bars show `USB`, both calibration buttons are disabled as "USB ONLY". |
| `Drone3DView` | Strip badge `FC·RF`, no mag ghost or Δ badge. When attitude is stale the ADI is hatched over with "ATTITUDE STALE n.ns" / "NO ATTITUDE DATA", so a frozen horizon can't be read as level flight. |
| `BaroWidget` | Source tag: `BARO · RADIO`, `GPS Δ HOME · RADIO` (CrsfLink's GPS-altitude fallback), `NO ALT · RADIO`, or `STALE n.ns`. |
| `GPSWidget` | Fix readout `RF FIX` (a fix inferred from the satellite count, never "3D FIX"), then `STALE` when GPS frames stop. HDOP reads `n/a`. The satellite table explains that the list is USB only and shows the satellite count. The last position stays on the map. |
| `FCStatusWidget` | CPU, LOOP, I2C ERR, PID, motors and RC channels show `USB`. Sensor pills the radio can't confirm read `MAG?` / `RNG?`. The battery badge shows `STALE` when old. LINK QUALITY reads `ELRS nn%`. |
| `ArmingWidget` | Evaluators can return `None` ("cannot verify on this link"), which gets a grey dot. MOTORS IDLE is `None` on radio. GYRO/ACC pass on received attitude. GPS FIX is satellite-count based. Betaflight's `!ERR` fails the RC LINK row. The banner never shows plain **READY TO ARM** while a check is unverified; it shows **READY — n CHECK(S) NEED USB**. |

## Also fixed along the way

The IMU widget ran the YAW heading through the roll/pitch tilt limits
whenever there was no magnetometer, so any heading above 30° flashed red.
A heading has no tilt limit: without a mag reference the YAW cell is now
neutral, the same as the widget's TINY layout already did.

## Adding a widget or a value

- Read the source and capabilities only through `link_mode` (`is_radio`,
  `available`, `stale_ms`), never `data["link_source"]` directly.
- If a value is missing on a link, add an `available_*` key in both
  `TelemetryWorker._build_ui_data()` and `to_dict()` in `Bindings.cpp`,
  instead of special-casing `"ELRS"` in the widget.
- Add a UT-LINK test with `_frame("ELRS", …)` from
  `TestScripts/test_link_mode.py`.
