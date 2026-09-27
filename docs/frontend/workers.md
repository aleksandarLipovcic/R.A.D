# Worker threads — `telemetry_worker.py`, `video_worker.py`, `detection_worker.py`

These three modules follow one identical contract, stated explicitly at the
top of `telemetry_worker.py` and mirrored by the other two: **no `tkinter`
import, no widget reference, ever.** Each owns one daemon thread that talks
to exactly one C++ link object, and hands the Tk thread a plain Python
value (dict / tuple / list) through a small, deliberately non-blocking
interface. None of them call back into Tk directly — `DroneCockpitApp`'s
own `after()`-scheduled pump decides when to read what they've produced
(see [app-shell.md](app-shell.md#the-tk-update-loop-_update_loop)).

## `telemetry_worker.py` — `TelemetryWorker`

**Wraps:** `DroneBackend.DroneLink` (USB/MSP) **and**, optionally,
`DroneBackend.CrsfLink` (ELRS radio)
**Consumed by:** every instrument widget, via `DroneCockpitApp._update_loop`;
the toolbar's [`RadioLinkIndicator`](radio-link-indicator.md) and USB label,
via `get_link_status()`; the OSD/detection trampolines, via
`get_active_state()`

```
TelemetryWorker(hub, radio=None)
  .start() / .stop()
  .get_frame() -> dict | None        # non-blocking
  .is_connected -> bool               # property: a source is live
  .get_link_status() -> dict          # USB + ELRS status, always available
  .get_active_state() -> DroneState | None   # thread-safe, for C++ trampolines
  .set_yaw_trim(trim) / .get_yaw_trim()
  .get_last_state_snapshot() -> (raw_yaw, mag_heading, mag_valid)
  .last_error: str | None
```

Polls the active source's `get_latest_state()` at `POLL_HZ` (60) into a bounded
`queue.Queue(maxsize=3)`. A full queue drops the **oldest** frame before
inserting the new one — the Tk thread is guaranteed to eventually see the
freshest telemetry, never an ever-growing backlog. `get_frame()` drains the
whole queue in a loop and returns only the last item pulled (or `None` if
empty), so a busy Tk thread that skips a tick doesn't fall behind — it just
catches up to "now" on its next read.

Two small pieces of state are exposed beyond the queue, each behind its own
lock:

- **Yaw trim** — `set_yaw_trim()` (written from the Tk thread, e.g. a pilot
  heading-adjust control) / `get_yaw_trim()` (read inside the worker
  thread, applied before yaw is placed into `ui_data`).
- **Last raw state snapshot** (`_last_raw_yaw`, `_last_mag_heading`,
  `_last_mag_valid`) — exposed via `get_last_state_snapshot()` so the Tk
  thread can recompute a heading-trim delta without needing a full
  `ui_data` frame.

### Two telemetry sources — USB vs ELRS

`radio` is `None` when the loaded `DroneBackend.pyd` predates `CrsfLink`.
In that case the worker behaves as USB-only, and `get_link_status()`
reports `radio_status = "DISABLED"`.

On every poll, `_pick_source()` returns `(source, state, radio_state)`,
choosing in this order:

1. **USB** if the cable link is connected **and** `link_healthy`;
2. **ELRS** if `radio_state.radio.status_str` is `TELEMETRY_OK` or
   `DEGRADED` (`RADIO_LIVE`);
3. **USB** if the cable is connected but momentarily unhealthy;
4. **none**: no frame is produced and `is_connected` goes `False`.

`radio_state` is always returned, even while USB is active, so the radio
indicator keeps showing the real radio status. Both `get_latest_state()`
calls are mutex-protected C++ snapshots.

**`get_active_state()`** runs the same selection and returns only the
state (or `None`). This is what `_get_osd_telemetry()`,
`_get_detection_telemetry()`, `_detection_gps_status()` and
`_get_current_drone_fix()` call, so the OSD, georeferencing and map marker
always use **the same source the instruments show**. It is safe to call
from the C++ capture and inference threads.

**`get_link_status()`** returns a flat dict built by
`_build_link_status()`: `active_source`, `usb_connected`, `usb_healthy`,
`radio_present`, `radio_status`, `radio_port`, `link_stats_valid`,
uplink/downlink LQ/RSSI/SNR, `tx_power_mw`, `age_*_ms`, `rate_*_hz`,
`reconnect_count`, `link_lost_count`, `crc_errors`, `radio_armed`. The
worker replaces the whole dict at once (an atomic reference swap), so the
Tk thread reads it without a lock. It is rebuilt at `_STATUS_HZ` (10 Hz)
while frames flow, and on every idle pass (`_IDLE_S`, 0.1 s) while no
source is live, so the indicator reacts within about 100 ms.

### Connection handling (`_run`)

The worker never connects or reconnects anything itself. `CrsfLink`
reconnects on its own thread, and the USB probe / drop / re-probe cycle
belongs to `DroneCockpitApp` (`_auto_connect()`, `_reconnect()`, see
[app-shell.md](app-shell.md#reconnect-handling)). When no source is live,
the worker clears `_connected`, refreshes the link status and sleeps
`_IDLE_S`. `RECONNECT_S` is still defined but no longer used. If a backend
call raises, the message goes to `last_error`, `_connected` is cleared, and
the loop pauses 0.1 s before retrying.

### `_build_ui_data(state, source)` — the one and only translation point

This is the **single place** in the whole frontend that touches the raw
`DroneBackend.DroneState` pybind11 object. Every widget consumes the flat
dict it returns instead — no widget needs to know the C++ struct shape at
all. Notable transformations performed here rather than anywhere else:

- `roll`/`pitch` divided by 10 (MSP wire convention: transmitted as
  degrees × 10; `yaw` is already whole degrees).
- `yaw` has the pilot's trim subtracted (`(raw_yaw - yaw_trim + 360) % 360`)
  before being exposed as `"yaw"` — this is what `Drone3DView`'s compass
  tape and heading-divergence badge actually display.
- `gps.hdop` — raw MSP integer, where `9999` means "invalid" — is converted
  to a real HDOP float, or `99.0` as an explicit invalid sentinel.
- `state.sv_list` (a list of backend `SVInfoEntry` structs) is converted to
  a list of plain dicts (`gnss_id`, `sv_id`, `cno`, `used`, `quality`,
  `status`, `elev`, `azim`) for `GPSWidget`'s satellite table.
- `rssi` (raw 0–255) becomes `rc_link_quality` as a 0–100 percentage, with
  `-1` reserved as a distinct sentinel for "no signal at all" (`rssi == 0`)
  vs. a genuine 0% reading.
- `sensor_status` (a bitmask) is unpacked into individual
  `sensor_{acc,baro,mag,gps,rangefinder,gyro}_present` booleans so widgets
  never do their own bit tests.
- **Source and ELRS keys.** Every frame gets `link_source` (`"USB"` /
  `"ELRS"`) and the `available_raw_imu`, `available_raw_mag`,
  `available_motors`, `available_rc_channels`, `available_sat_list`,
  `available_hdop`, `available_arming_flags` and `available_cpu_load`
  flags. All are `False` on ELRS, so widgets can show "USB only" instead of
  zeros. It also gets `arming_blocked`, `gps_waiting`, `altitude_source`
  and `age_attitude_ms` / `age_gps_ms` / `age_battery_ms` (−1 = never;
  always 0 on USB). On ELRS with valid link statistics, `rc_link_quality`
  is the real uplink LQ %.
- Every field is read via `getattr(state, "field", default)` rather than
  direct attribute access — a `DroneBackend.pyd` built before some field
  was added degrades to a sane default instead of raising an
  `AttributeError` that would otherwise take the whole worker thread down.

## `video_worker.py` — `VideoWorker`

**Wraps:** `DroneBackend.VideoLink`
**Consumed by:** `FPVWidget.update_status()`

```
VideoWorker(video_link, poll_hz=15)
  .start() / .stop()
  .get_status() -> (connected: bool, fps: float, device_name: str)
  .is_connected: bool               # plain attribute, not a property
```

The single most important thing about this worker is what it **doesn't**
do: it never calls `video_link.get_latest_frame()`. Live video frames are
painted directly into a native window by `VideoLink`'s own C++ capture
thread (see [../modules/videolink.md](../modules/videolink.md)), so nothing
in Python needs the pixels for the live feed — fetching them anyway would
clone a full frame under a mutex shared with the very thread that's busy
painting it, for zero benefit. This worker polls only three cheap, atomic
getters — `is_connected()`, `get_measured_fps()`, `get_device_name()` — at
15 Hz by default, purely to drive the small status/FPS text bar in
`FPVWidget`.

> If a future feature genuinely needs frame data in Python (e.g. local
> recording, a separate post-processing pipeline), call
> `video_link.get_latest_frame()` directly from wherever that feature
> lives — don't route it back through this worker, which exists
> specifically to avoid that cost on the hot path.

## `detection_worker.py` — `DetectionWorker`

**Wraps:** `DroneBackend.DetectionLink`
**Consumed by:** `DetectionMapWidget` (via `DroneCockpitApp._pump_detection_records()`)

```
DetectionWorker(detection_link, poll_hz=4)
  .start() / .stop()
  .get_status() -> (detection_count, last_pass_duration_ms, last_pass_timestamp_ms)
  .get_new_records() -> list[DetectionRecord]   # returns and clears
```

Same "route heavy work through C++, poll cheap results from Python" split
as `VideoWorker`. Every call this worker makes into `DetectionLink` is a
fast, mutex-protected read of already-computed data:
`get_detection_count()`, `get_last_pass_duration_ms()`,
`get_last_pass_timestamp_ms()`, and — the one doing real incremental work —
`get_records_since(self._last_seen_id)`.

`get_new_records()` is a pop-and-clear: it returns whatever has accumulated
in `self._new_records` since the last call and empties the list under
lock, so `DetectionMapWidget` only ever processes genuinely new records
instead of re-rendering its entire pin list every poll tick. `_last_seen_id`
is advanced to the highest `id` seen in each incremental batch, so a
record already delivered is never re-fetched. This worker never touches
frames, never triggers inference, and never writes to disk — everything it
reads is a small struct copy DetectionLink already produced on its own C++
threads (see [../modules/detectionlink.md](../modules/detectionlink.md)).
