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

**Wraps:** `DroneBackend.DroneLink`
**Consumed by:** every instrument widget, via `DroneCockpitApp._update_loop`

```
TelemetryWorker(hub, poll_hz=60)
  .start() / .stop()
  .get_frame() -> dict | None      # non-blocking
  .is_connected -> bool             # property
  .set_yaw_trim(trim) / .get_yaw_trim()
  .get_last_state_snapshot() -> (raw_yaw, mag_heading, mag_valid)
  .last_error: str | None
```

Polls `hub.get_latest_state()` at `POLL_HZ` (60) into a bounded
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

### Connection handling (`_run`)

If `hub.is_connected()` is false, the worker just clears its own
`_connected` event and sleeps `RECONNECT_S` (2.0 s) before checking again —
it does **not** try to reconnect anything itself. Reconnect orchestration
(disconnecting the hub, replacing the worker instance, calling
`auto_detect_f405()` again) is `DroneCockpitApp`'s job
(`_reconnect()`/`_auto_connect()`), not this worker's. If `get_latest_state()`
raises, the exception message is stored in `last_error`, `_connected` is
cleared, and the loop pauses briefly (0.1 s) before retrying — deliberately
short so a spinning failure doesn't burn CPU, but the Tk thread ultimately
decides what a repeated failure means for the app's connection state.

### `_build_ui_data(state)` — the one and only translation point

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
