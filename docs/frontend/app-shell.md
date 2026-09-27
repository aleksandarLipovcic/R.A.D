# `DroneCockpitUI.py` — app shell & panel workspace

**File:** `DroneCockpitUI.py`
**Entry point:** `DroneCockpitApp` (constructed on `tk.Tk()`, run via
`root.mainloop()` at the bottom of the file)
**Depends on:** `DroneBackend.pyd`, every widget module (see
[../FRONTEND.md](../FRONTEND.md) for the full list), `telemetry_worker.py`,
`video_worker.py`, `detection_worker.py`, `osd_overlay_controls.py`,
`radio_link_indicator.py`

## Responsibility

This is the whole application: it owns the Tk root window, creates the
C++ link objects (`DroneLink`, `CrsfLink`, `VideoLink`, `DetectionLink`)
and their Python worker wrappers, builds every instrument panel, and runs the
single recurring update loop that feeds fresh data to each widget.

## Startup sequence (`DroneCockpitApp.__init__`)

1. `self.hub = DroneBackend.DroneLink()` is created, then `_setup_ui()`
   builds every panel widget — **before** anything is connected, so a
   connection failure never leaves half the UI missing.
   Right after the hub, `self.radio = DroneBackend.CrsfLink()` is
   created (or `None`, with a console note, if the `.pyd` has no
   `CrsfLink`). This is the ELRS telemetry source, see
   [../modules/crsflink.md](../modules/crsflink.md). Its battery cell
   count is set from `RADIO_BATTERY_CELLS` (default 4) before
   `start_auto()`.
2. `TelemetryWorker(self.hub, radio=self.radio)` is created (not started
   yet, see [workers.md](workers.md)).
3. `self.video_link = DroneBackend.VideoLink()` is created, `VideoWorker`
   is started immediately, and `_start_video_autoconnect()` kicks off — the
   FPV capture dongle is treated as an independent USB device with its own
   connect/retry loop, deliberately not gated on (or gating) the
   flight-controller serial connection.
4. `self.fpv_view.attach(self.video_link)` hands the FPV panel's native
   window handle to `VideoLink` so the C++ capture thread can start
   painting into it — safe to call before a capture device is even
   connected.
5. `self.video_link.set_telemetry_provider(self._get_osd_telemetry)` wires
   the OSD's data source — a **dedicated** trampoline, not the one used for
   detection (see [Telemetry trampolines](#telemetry-trampolines) below).
   Without this call, `VideoLink::drawOsdOverlay()` bails out immediately
   and the OSD never draws regardless of what's enabled in the settings
   panel.
6. `self.detection_link = DroneBackend.DetectionLink()` is created and
   wired: `set_video_link_source(self.video_link)`,
   `set_telemetry_provider(self._get_detection_telemetry)`, model path,
   input size, screenshot dir, detection interval, horizontal FOV, known
   object widths, triangulation thresholds, CUDA opt-in. Every one of the
   newer bindings is guarded with `hasattr(self.detection_link, "set_x")`
   before calling it — a `DroneBackend.pyd` built before a given binding
   existed degrades with a loud console warning instead of taking the
   entire app down over `import DroneBackend` succeeding but one method
   missing. The source comments spell out exactly what silently breaks per
   missing binding (wrong FOV skewing every bearing, no object-size
   ranging, no triangulation, CPU-only inference).
7. `DetectionWorker` is started.
8. `DetectionMapWidget` is **not** created here — it's built on first
   open (`_open_detection_window()`), so a pilot who never opens the
   detection window never pays for an unused `Toplevel` + `Treeview`.
9. Telemetry **no longer waits for the USB cable**:
   `self.radio.start_auto()` (non-blocking, reconnects forever),
   `self._worker.start()` and `_schedule_update()` run immediately, and
   `_auto_connect()` then starts the USB probe on a background thread.
   The instruments come alive from whichever source is live first.

## Panel workspace

Every instrument lives in a `DraggablePanel(tk.Frame)` on a free-form
`tk.Canvas` workspace rather than a fixed grid:

- **Drag** by the title bar, **resize** via edge/corner handles,
  **right-click** for a z-order context menu (raise/lower/snap-to-neighbor).
- `_snap_candidates()` / `_apply_snap()` implement edge-snapping against
  other panels' current live positions while dragging — the same kind of
  alignment-guide behavior `osd_overlay_controls.OsdDragOverlay` reuses for
  OSD element placement.
- `on_zorder` / `on_visibility` callbacks, passed to every `DraggablePanel`
  at construction, let `DroneCockpitApp` keep `VideoLink`'s native FPV
  window's OS-level Z-order/visibility in sync with the "fpv" panel's Tk
  state — necessary because that native window lives entirely outside Tk's
  widget tree, so Tk's own `lift()`/`lower()` calls can't touch it (see
  [../modules/videolink.md](../modules/videolink.md#native-direct-to-window-rendering)).

### Panel inventory

Built by a local `_panel(name, title)` factory inside `_setup_ui()`:

| Panel key | Title | Widget class |
|---|---|---|
| `imu` | MPU-6500 — IMU / Attitude | `IMUWidget` |
| `mag` | QMC5883L — Magnetometer | `MagWidget` |
| `adi` | ADI — Attitude Direction Indicator | `Drone3DView` |
| `baro` | ALT / VSI — Altitude & Vertical Speed | `BaroWidget` |
| `gps` | GPS Navigation — Nav / Satellites / Map | `GPSWidget` |
| `fc_status` | FC Status — Armed · Mode · Sensors · CPU | `FCStatusWidget` |
| `arming` | Arming Diagnostics — Pre-flight Checklist | `ArmingWidget` |
| `fpv` | FPV — Live Video Feed | `FPVWidget` |

`DetectionMapWidget` is a **ninth**, deliberately separate top-level window
(see [detection-map-widget.md](detection-map-widget.md)) — never a
`DraggablePanel`.

## Layout persistence — `LayoutStore` / `LayoutManagerDialog`

`LayoutStore` persists **named layout profiles** (not just one layout) to
disk, with an "active profile" pointer. `LayoutManagerDialog` lists saved
profiles with a live miniature preview per row (`_LayoutPreviewCanvas`),
plus rename/delete/save-current-as/overwrite actions.

Toolbar actions built in `_setup_ui()`:

| Button | Action |
|---|---|
| 🗂 Layouts ▾ | Opens `LayoutManagerDialog` |
| 🧩 Panels ▾ | Per-panel show/hide checkboxes |
| 🎯 Detections | Opens the detection/map window |
| 📐 Camera Angle | Manual (or future SimpleBGC-auto) camera mount angle, used for detection georeferencing, persisted with the active layout |
| ↺ Revert to Saved | Discards unsaved panel-position changes |
| 🏭 Factory Default | Resets all panels to built-in default positions (does not touch any saved profile) |
| 🔓 Save & Lock Layout | Saves the current layout and disables drag/resize |

Icon-only buttons are used throughout (each with a hover tooltip via the
internal `_Tooltip` class) — mixing wide unicode glyphs with long text
labels was causing Tk to mis-size/clip buttons on some platforms.

## Telemetry trampolines

Two small no-arg methods build a `DroneBackend.TelemetrySnapshot` from
`self._worker.get_active_state()` (the USB **or** ELRS state, whichever
currently feeds the instruments, or `None` → `valid = False`) and are handed to the C++ side as provider
callables. They look interchangeable — the C++ header comments even
suggest reusing one — but they are **deliberately not the same function**,
and merging them breaks the OSD:

| | `_get_detection_telemetry()` | `_get_osd_telemetry()` |
|---|---|---|
| Wired to | `DetectionLink.set_telemetry_provider()` | `VideoLink.set_telemetry_provider()` |
| Called from | `DetectionLink`'s own C++ inference thread, once per pass | `VideoLink`'s own C++ capture/paint thread, once per painted frame |
| `valid` means | GPS fix usable (`gps.position_usable`) — no fix ⇒ skip georeferencing | FC link is up at all — altitude/horizon/compass need no GPS fix |
| `altitude_m` source | `gps.altitude_m` | **barometer** (`state.baro_altitude_cm`, converted) — keeps the OSD altitude in agreement with `BaroWidget` instead of silently depending on GPS |
| `gimbal_pan/tilt_deg` | `_get_camera_mount_angle()` — a manual, toolbar-editable stand-in; the SimpleBGC gimbal is physically installed but not electrically wired yet, so there's no live readback to use | not read by the OSD |

Both are safe to call from a non-Tk thread: `get_active_state()` only
takes mutex-protected snapshot copies from `DroneLink` / `CrsfLink` (see
[../modules/dronelink.md](../modules/dronelink.md)). Note that each call
copies **both** states (radio and USB) to pick the source. At the OSD's
per-frame rate this is the main cost of the trampoline.

Two related helpers follow the same source rule:

- `_detection_gps_status()` explains why georeferencing is off. On the
  radio link, HDOP is reported as 9999, so the text says "HDOP n/a on
  radio link" and "need >=5 sats on radio link".
- `_get_current_drone_fix()` places the drone marker on the detection map.
  When no source is live but the radio reports `TELEMETRY_LOST`, it
  **keeps the last known position** from `CrsfLink` instead of removing
  the marker. Where the drone was when the link dropped is exactly what a
  rescue crew needs.

## The Tk update loop (`_update_loop`)

One recurring `self.root.after(UI_REFRESH_MS, self._update_loop)` job.
Each tick:

1. Updates the FPV status/FPS text from `VideoWorker.get_status()` — done
   **independently** of the telemetry connection check, so the video
   panel's status text doesn't blank out just because the separate MSP
   link hiccuped or is mid-reconnect.
2. Calls `_update_link_status()` on **every** tick, even with no live
   source. It feeds `worker.get_link_status()` to the
   [`RadioLinkIndicator`](radio-link-indicator.md) and its tooltips,
   picks up the result of a finished USB probe, and sets the USB label
   (`USB: connected COMx` / `link degraded` / `searching...` /
   `not connected (radio active)`). This is also where USB drop detection
   lives (see [Reconnect handling](#reconnect-handling)).
3. If `TelemetryWorker.is_connected` is false (neither USB nor ELRS is
   live), the instruments keep their last values and the loop just
   reschedules. The indicator and USB label explain why.
4. Drains `TelemetryWorker.get_frame()` (non-blocking; `None` if nothing
   new this tick).
5. Feeds the resulting `ui_data` dict to each widget **on a per-widget
   throttle** — a `_frame_counters` dict compared against a `_THROTTLE`
   divisor per key, so expensive widgets don't set the pace for cheap ones:

   | Widget | Rate |
   |---|---|
   | IMU, Baro, FC Status, Arming, Mag | every tick (~50 Hz) |
   | ADI (`Drone3DView`) | every 3rd tick (~17 Hz); also gets `source` and `stale_ms` from `link_mode` |
   | GPS (`GPSWidget`) | every 5th tick (~10 Hz) — the heaviest, map redraw |

Detection records are pumped on a **separate** `after()` job
(`_pump_detection_records()`), only while the detection window is open —
see [detection-map-widget.md](detection-map-widget.md).

## Reconnect handling

Only the **USB** link is handled here. `CrsfLink` finds and reconnects the
Pocket on its own worker thread, and the telemetry worker keeps running
through both.

- **`_auto_connect()`** starts one background `UsbProbe` thread (a no-op
  if a probe is already running or the hub is connected). The thread calls
  `DroneBackend.auto_detect_fc(exclude=[radio port])`, which does an MSP
  handshake and never picks the Pocket (see
  [../modules/serial-port-scan.md](../modules/serial-port-scan.md)). On an
  older `.pyd` it falls back to `auto_detect_f405()`. It then calls
  `hub.connect(port)` and, on success,
  `radio.set_excluded_ports([port])`. The `(port, ok)` result is handed
  back through `_usb_probe_result` and consumed on the Tk thread by
  `_update_link_status()`. A failed probe schedules a retry after
  `RECONNECT_MS` (2 s) via `_schedule_usb_retry()`.
- **Drop detection:** if the hub is connected but `usb_healthy` stays
  false for `USB_DROP_AFTER_S` (5 s), `_reconnect()` runs.
- **`_reconnect()`** calls `hub.disconnect()` (which joins the poll thread)
  on a background `UsbDrop` thread, clears the radio's excluded ports, and
  schedules a new probe. The `TelemetryWorker` is **not** replaced any
  more. It simply falls over to ELRS if the radio link is up.

## Shutdown (`shutdown()`)

Bound to `WM_DELETE_WINDOW`. Order matters here — persists the active
layout first, cancels both `after()` jobs, then stops things in dependency
order: `TelemetryWorker` → `hub.disconnect()` → `radio.disconnect()` (if
present) → `VideoWorker` →
`fpv_view.detach()` → `video_link.disconnect()` → `DetectionWorker` →
`detection_link.stop()` → `root.destroy()`.
