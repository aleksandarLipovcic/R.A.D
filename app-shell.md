# `DroneCockpitUI.py` — app shell & panel workspace

**File:** `DroneCockpitUI.py`
**Entry point:** `DroneCockpitApp` (constructed on `tk.Tk()`, run via
`root.mainloop()` at the bottom of the file)
**Depends on:** `DroneBackend.pyd`, every widget module (see
[../FRONTEND.md](../FRONTEND.md) for the full list), `telemetry_worker.py`,
`video_worker.py`, `detection_worker.py`, `osd_overlay_controls.py`

## Responsibility

This is the whole application: it owns the Tk root window, creates the
three C++ link objects (`DroneLink`, `VideoLink`, `DetectionLink`) and
their Python worker wrappers, builds every instrument panel, and runs the
single recurring update loop that feeds fresh data to each widget.

## Startup sequence (`DroneCockpitApp.__init__`)

1. `self.hub = DroneBackend.DroneLink()` is created, then `_setup_ui()`
   builds every panel widget — **before** anything is connected, so a
   connection failure never leaves half the UI missing.
2. `TelemetryWorker(self.hub)` is created (not started yet — see
   [workers.md](workers.md)).
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
9. `_auto_connect()` is called, which only starts `TelemetryWorker` and the
   Tk update pump once a real serial connection to the FC succeeds.

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

Two small no-arg methods build a `DroneBackend.TelemetrySnapshot` straight
from `hub.get_latest_state()` and are handed to the C++ side as provider
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

Both are safe to call from a non-Tk thread: `DroneLink.get_latest_state()`
is already a thread-safe, mutex-protected snapshot copy (see
[../modules/dronelink.md](../modules/dronelink.md)), so calling it twice
per cycle from two different C++ threads is just two small extra
mutex-guarded copies, not a correctness concern.

## The Tk update loop (`_update_loop`)

One recurring `self.root.after(UI_REFRESH_MS, self._update_loop)` job.
Each tick:

1. Updates the FPV status/FPS text from `VideoWorker.get_status()` — done
   **independently** of the telemetry connection check, so the video
   panel's status text doesn't blank out just because the separate MSP
   link hiccuped or is mid-reconnect.
2. Checks `TelemetryWorker.is_connected`; if the worker has lost the link,
   either triggers `_reconnect()` (hub itself is down) or shows a
   "Waiting for data..." status (hub is up, first frame hasn't arrived
   yet) and reschedules.
3. Drains `TelemetryWorker.get_frame()` (non-blocking; `None` if nothing
   new this tick).
4. Feeds the resulting `ui_data` dict to each widget **on a per-widget
   throttle** — a `_frame_counters` dict compared against a `_THROTTLE`
   divisor per key, so expensive widgets don't set the pace for cheap ones:

   | Widget | Rate |
   |---|---|
   | IMU, Baro, FC Status, Arming, Mag | every tick (~50 Hz) |
   | ADI (`Drone3DView`) | every 3rd tick (~17 Hz) |
   | GPS (`GPSWidget`) | every 5th tick (~10 Hz) — the heaviest, map redraw |

Detection records are pumped on a **separate** `after()` job
(`_pump_detection_records()`), only while the detection window is open —
see [detection-map-widget.md](detection-map-widget.md).

## Reconnect handling

`_reconnect()` (called from `_update_loop()` when the hub itself reports
disconnected) stops the current `TelemetryWorker`, disconnects the hub,
shows a "Link lost — reconnecting..." status, and **replaces**
`self._worker` with a brand-new `TelemetryWorker` instance (re-applying the
saved yaw trim) rather than reusing the old one — ensuring the worker's
internal state starts clean on every reconnect cycle. `_auto_connect()`
then retries via `DroneBackend.auto_detect_f405()` on a timer
(`RECONNECT_MS`) until a port opens.

## Shutdown (`shutdown()`)

Bound to `WM_DELETE_WINDOW`. Order matters here — persists the active
layout first, cancels both `after()` jobs, then stops things in dependency
order: `TelemetryWorker` → `hub.disconnect()` → `VideoWorker` →
`fpv_view.detach()` → `video_link.disconnect()` → `DetectionWorker` →
`detection_link.stop()` → `root.destroy()`.
