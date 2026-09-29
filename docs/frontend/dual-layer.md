# Dual-layer panels — a USB face and a radio face per slot

**Files:** `dual_layer.py` (`DualLayerPanel`, `LayerSwitch`),
`radio_panels.py` (`RadioAttitudePanel`, `RadioFlightPanel`), wiring in `DroneCockpitUI.py`
(`_setup_ui`, `_update_link_status`, `_switch_link_layer`)
**Tests:** `TestScripts/test_dual_layer.py` (UT-LAYER-001 … 009)

## What it does

Two instrument panels have a second face designed for the ELRS radio
link, and a third, the magnetometer, adapts in place (see below). When the telemetry source changes, **every panel swaps face in place
within about one second**. Position, size, z-order, visibility and the
saved layout profile stay exactly as they were, so neither the pilot nor
the operator has to reselect or rearrange anything.

| Panel slot | USB face (cable, MSP) | Radio face (ELRS, CRSF) |
|---|---|---|
| `imu` | `IMUWidget`: rotation, G-force, angles, heading drift, MSP latency | `RadioAttitudePanel`: large ROLL / PITCH / HEADING with the same tilt alarms (amber ≥ 15°, flashing red ≥ 30°); footer with attitude age, frame rate, uplink LQ and RSSI behind the ⏱ toggle (off by default, one setting shared with the USB face's "Link latency") |
| `mag` | `MagWidget` | **the same `MagWidget`** — no separate face (see below) |
| `fc_status` | `FCStatusWidget`: arm/mode, battery, timer, sensors, CPU/loop/I2C, motors, RC | `RadioFlightPanel`: arm/mode, battery, the same flight timer, full **ELRS link statistics** (uplink/downlink LQ, RSSI per antenna, SNR, TX power, RF mode), frame rates, link-lost / reconnect / CRC counters |

**Magnetometer — one widget for both links.** A separate radio face was
tried and dropped: the pilot should see the *same* compass rose and heading
readout on both links, with the same responsive layouts (the heading box
moves beside the rose when there is room). On the radio link `MagWidget`:

- shows the FC heading ("FC HEADING · RADIO", status "FC HDG · RADIO" or
  STALE);
- draws a cyan **H** arrow towards home on the rose (on USB too, whenever
  the FC reports a home point);
- in the full layout, replaces the raw-field bar row (no raw
  magnetometer over CRSF) with a **HOME / NAVIGATION** row: distance,
  bearing, TURN L/R, ground speed, altitude — or **HOME NOT SET** until
  the drone is armed with a usable GPS fix;
- keeps both calibration buttons in place but disabled ("USB ONLY", or
  "DISARM FIRST" while armed).

Only the panel title changes ("RADIO — Heading & Home").

The other panels (ADI, altitude/VSI, GPS map, arming checklist, FPV) carry
the same kind of data on both links. They stay single-face and adapt
per frame through `link_mode.py` (see [link-mode.md](link-mode.md)).

The panel title changes with the face, for example "MPU-6500 — IMU /
Attitude" becomes "RADIO — Attitude & Link Quality" in blue.

## How the switch works

```
TelemetryWorker ── get_link_status()["active_source"]  ("USB" / "ELRS" / "NONE")
        │  every Tk tick (~50 Hz)
        ▼
LayerSwitch.observe(source, now) ── True once the new source has held for LAYER_HOLD_S
        │
        ▼
DroneCockpitApp._switch_link_layer(radio)
        ├─ DualLayerPanel.show_radio(radio)   × imu, fc_status       (grid swap)
        └─ DraggablePanel.set_title(...)       × imu, mag, fc_status
```

- **Debounce:** a new source has to stay active for `LAYER_HOLD_S`
  (1.0 s) before the faces switch. A link that flickers between USB and
  radio doesn't make the window jump back and forth. Measured in the real
  app with a simulated source change, the full switch takes **≈ 1.1 s**
  (hold time plus the worker's status refresh).
- **All panels at once:** there is one switch decision, applied to every
  dual panel in the same Tk callback.
- **No source ("NONE") never switches:** the current faces stay, with
  their last values, and the toolbar's radio indicator explains why.
- **During the hold second** the old face gets frames from the new source.
  That's safe, because every widget already adapts per frame (grey
  placeholders, STALE, and so on, per [link-mode.md](link-mode.md)).

## DualLayerPanel

```python
dual = DualLayerPanel(panel.content)
usb = IMUWidget(dual, ...)
radio = RadioAttitudePanel(dual)
dual.set_faces(usb, "update_ui", radio, "update_radio")
dual.feed(ui_data)          # update loop — the visible face only
dual.show_radio(True)       # from _switch_link_layer
```

Both faces are gridded in the same cell. The hidden one is
`grid_remove()`d, so it keeps its state and costs nothing to draw. Only
the visible face is updated. A hidden face can define
`on_hidden_data(data)` to track minimal state cheaply.

### The flight timer is shared

`FCStatusWidget` owns the cockpit's single flight timer. While its radio
face is showing, `on_hidden_data()` keeps its armed state current, so the
timer keeps running. `RadioFlightPanel` displays it through
`timer_provider=fc_status_view.flight_timer`. Switching links mid-flight
never restarts or duplicates the clock.

## Data used by the radio faces

Everything comes from the normal `ui_data` frame. On ELRS,
`TelemetryWorker` adds the link statistics (`_radio_link_fields()`):
`elrs_link_stats_valid`, `elrs_uplink_lq`, `elrs_uplink_rssi1_dbm`,
`elrs_uplink_rssi2_dbm`, `elrs_uplink_snr`, `elrs_downlink_lq`,
`elrs_downlink_rssi_dbm`, `elrs_downlink_snr`, `elrs_tx_power_mw`,
`elrs_rf_mode_index`, `elrs_active_antenna`, `age_link_stats_ms`,
`rate_battery_hz`, `rate_flight_mode_hz`, `rate_total_hz`,
`radio_link_lost_count`, `radio_reconnect_count`, `radio_crc_errors` and
`home_set`. On USB they are zero/False.

`MagWidget` uses the home point that `CrsfStateMapper` computes at
arming (`gps_dist_to_home_m`, `gps_bearing_to_home`, `home_set`). Before
arming with a usable GPS fix it shows **HOME NOT SET**.

## Adding a radio face to another panel

1. Write a `tk.Frame` subclass with `update_radio(self, data)`, reading
   only `ui_data` and the `link_mode` helpers.
2. In `_setup_ui`, wrap the panel's widget in a `DualLayerPanel`, add a
   title pair to `_LAYER_TITLES`, and feed the panel with `.feed()` in
   `_update_loop`.
3. Add it to the tuple in `_switch_link_layer`.
4. Add a UT-LAYER test.
