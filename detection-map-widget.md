# `DetectionMapWidget.py` — detection & map window

**File:** `DetectionMapWidget.py`
**Class:** `DetectionMapWidget(tk.Toplevel)`
**Opened by:** `DroneCockpitApp._open_detection_window()` — never created
at startup (see [app-shell.md](app-shell.md))
**Fed by:** `DroneCockpitApp._pump_detection_records()`, a separate
`after()`-scheduled job that only runs while this window is open, pulling
from `DetectionWorker.get_new_records()` (see [workers.md](workers.md))

## Responsibility

A deliberately **separate top-level window**, not another panel competing
for space in the main draggable workspace. Per the module docstring, this
is the whole point of the design: nothing this widget does — map redraws,
tile downloads, screenshot loads — can ever obstruct or contend with the
live FPV feed, which lives in its own panel entirely independent of this
window's existence.

Like `FPVWidget`, this widget mostly never touches raw pixel data. It
receives already-built `DetectionRecord` objects (small structs: class
name, confidence, lat/lon, range, a screenshot *path*, not image bytes)
and displays them with ordinary Tk widgets (a `Treeview` list + a Canvas
map). See [../modules/detectionlink.md](../modules/detectionlink.md) for
what a `DetectionRecord` actually contains.

## Public API (called from `DroneCockpitApp`)

| Method | Called when | Effect |
|---|---|---|
| `set_engine_status(running, detail)` | Window opens / detection engine state changes | Updates a purely informational status line — makes "the engine never started" visually distinct from "nothing detected yet", which would otherwise look identical from inside this window |
| `set_drone_telemetry(lat, lon, valid)` | Every poll tick, independent of detections | Updates the drone's own position marker on the map. See below — this used to be broken |
| `set_telemetry_status(valid, detail)` | Every poll tick | Updates a second, separate status line reflecting current GPS quality in near-real-time |
| `set_detection_link(detection_link)` | Once, right after the window is created | Wires the live annotated-feed poll loop — **without this call the live feed pane never starts at all** |
| `add_records(records)` | Whenever `DetectionWorker.get_new_records()` returns anything | Appends rows to the `Treeview`, updates track position history, triggers a map redraw |

### The `set_drone_telemetry()` fix

Documented directly in the source as a fix for a real prior bug: the drone
marker used to be derived by digging through `self._records` for the most
recent one with valid embedded telemetry, and the map refused to draw
anything at all unless `self._track_positions` was non-empty. That meant a
flight with zero detections — or zero *georeferenced* ones, e.g. before GPS
lock — never showed the drone's own position on the map at all, even
though the drone's position is known independently of anything being
spotted. `set_drone_telemetry()` decouples "where is the drone" from "has
anything been detected" entirely — it's now pushed every poll tick
regardless of detection activity.

Both `set_drone_telemetry()` and `add_records()` share a **first-fix-wins**
gate on `self._map_origin`: whichever source (drone telemetry or a
georeferenced detection) produces a usable position first is the one that
clears the "Waiting for GPS fix" placeholder state and anchors the map's
origin — deliberately either source can be the one that does it, whichever
happens first.

### `add_records()` cost

Explicitly kept cheap: a handful of `Treeview` inserts and canvas ovals per
call, no image decoding. Tile decoding (when a map tile needs it) happens
lazily inside `_draw_tile_mosaic()` on redraw, never inside `add_records()`
itself. Each record's `track_id` (falling back to `0` via `getattr` if an
older `DetectionLink` build predates that field) groups repeated sightings
of the same physical object into one position history
(`self._track_positions`), matching the tracker described in
[../modules/detectionlink.md](../modules/detectionlink.md#object-tracking--re-identification).

## The live annotated-feed pane

On by default, this pane polls
`DetectionLink.get_latest_annotated_frame_jpeg()` at ~5–6 fps
(`_poll_live_frame()`) — the same deliberate, narrow, clearly-labeled
exception to the "no pixels in Python" rule documented in
[../modules/detectionlink.md](../modules/detectionlink.md#live-preview-exception).
This is what's shown whenever no detection row is selected. Clicking a
detection row (`_on_select`) temporarily swaps the pane to that detection's
saved screenshot loaded from disk (`_load_preview(path)`) instead — the
live poll loop keeps running in the background the entire time regardless
of what's currently displayed, so returning to it (`_return_to_live()`,
triggered by a "back to live" button — `_show_back_to_live_button()` /
`_hide_back_to_live_button()`) is instant rather than needing to restart
polling.

## Map rendering

Backed by real XYZ tiles via `MapTiles` (see [map-tiles.md](map-tiles.md)),
with two switchable providers — "Satellite" (Esri World Imagery) and
"Topographic" (OpenTopoMap) — via radio buttons above the canvas
(`_on_map_provider_change`). Tiles are cached to disk per-provider on first
view (`_poll_tile_downloads()` mirrors `MapTiles.TileCache`'s
non-blocking pattern: draw a flat placeholder color for any cell not yet
cached, pick it up on the next redraw once the background download
lands). No network is required at all once an area's tiles are cached —
whether cached from a prior live flight or from a `prefetch_tiles.py` run
beforehand.

Every detection and the drone's own position are plotted directly in
standard Web Mercator pixel space (`_mercator_pixel`/`_mercator_lonlat`
module-level functions, `_fit_zoom()` for auto-zoom-to-fit) — the same
coordinate system the tiles themselves are drawn in, so pins line up with
the imagery exactly rather than needing a separately-reconciled
projection. A scale bar (`_draw_scale_bar`) and compass mark are drawn on
top of the tile mosaic. Standard interactions are supported: mouse-wheel
zoom (`_on_mouse_wheel`/`_zoom_at_point`, zooming toward the cursor
position rather than the map center), click-drag panning
(`_on_map_press`/`_on_map_drag`/`_on_map_release`), and a "center on drone"
button (`_on_center_on_drone`).
