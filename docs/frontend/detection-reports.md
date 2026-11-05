# `detection_report.py` — self-contained detection reports

**File:** `DroneCockpitUI/core/detection_report.py`
**Class:** `DetectionReportWriter` (+ pure helpers)
**Created by:** `DroneCockpitApp` at startup, next to `DetectionWorker`
**Fed by:** `DetectionWorker` — every new `DetectionRecord`, whether or not
the Detections window is open
**Opened from:** the Detections window, "Open report" on a selected object
(see [detection-map-widget.md](detection-map-widget.md))

## Why

`DetectionLink` saves an annotated full frame per record into
`DroneCockpitUI/detections/`. On its own that image cannot be used to
re-find the object: the position, time, confidence and dron telemetry live
only in the Detections window and are gone when the app closes. The report
puts all of it into files that stand on their own, for the copilot to
review during or after the flight.

## Output

```
DroneCockpitUI/detections/reports/<YYYY-MM-DD>/
    <HHMMSS>_<mmm>_d<id>_T<track>_<class>.jpg    collage
    <HHMMSS>_<mmm>_d<id>_T<track>_<class>.json   same data, machine-readable
    index.csv                                    one row per report that day (';', UTF-8 BOM, opens in Excel)
```

`DETECTION_REPORT_DIR` in `DroneCockpitUI.py` sets the location; `""`
disables reports. Time in names and folders is local time.

### Collage (1524 × 836 px, text in Serbian)

| Area | Content |
|---|---|
| Header | class (Serbian + model name), confidence, best confidence and sightings, local date/time with UTC offset, UTC time, detection id, object (track) id |
| Left | the camera frame with the box and a class/confidence tag, plus a zoomed crop of the object (up to 6×) |
| Right | satellite map (Esri World Imagery, same tile cache as the window): object with its ± radius circle, dron with heading arrow, dashed line dron → object, north arrow, scale bar, attribution |
| Bottom | object position (decimal, DMS, MGRS), accuracy radius, ranging method, distance and bearing from the dron; dron position, height above ground, heading, speed, pitch/roll, camera tilt/pan, GPS fix and satellites |

The map zoom is the tightest one (12–19) at which the object, its radius
circle and the dron all fit. Without an object position the map is centred
on the dron (zoom 17); without any valid telemetry the panel says so.

The JPEG carries standard EXIF GPS tags with the **object's** position (the
dron's if the object has none), so Windows Photos, Google Earth or QGIS can
place it on a map directly.

### JSON (`"format": "rad-detection-report/1"`)

All `DetectionRecord` fields plus derived ones: `time.local` / `time.utc`
(ISO 8601), `object.mgrs`, `object.google_maps` link, the full `drone`
telemetry, `source_frame` (the raw frame), `report_image` and `map`
(provider, zoom, missing tiles). Fields that do not apply are `null`, never
a fake `0` — an invalid telemetry snapshot gives `"latitude": null`.

## Threading and offline behaviour

`DetectionReportWriter` owns one daemon thread. `submit()` is a deque
append, so neither the Tk thread nor `DetectionWorker` ever waits on image
work, tile downloads or disk. A report takes ~0.1 s.

Tiles come from `MapTiles.CACHE_ROOT`. A missing tile is downloaded with a
4 s timeout via `MapTiles.download_tile()`; after one failure the writer
uses cached tiles only for 60 s, so flying without internet never stalls
the queue. The map then states how many tiles are missing. Pre-fetch the
area with `tools/prefetch_tiles.py` before an offline flight.

On close, `DroneCockpitApp` lets the writer finish what is queued (at most
5 s). A record that fails (unreadable frame, full disk) is logged to the
console and skipped; the rest continue.

## Raw frame names

`DetectionLink::saveScreenshot()` now names frames
`det_<YYYYMMDD>_<HHMMSS>_<id>_<classIndex>.jpg`. Detection ids restart at 1
every session, so the old `det_<id>_<class>.jpg` overwrote frames from
earlier flights. Needs a `DroneBackend.pyd` rebuild; reports work with
either naming, they use `screenshot_path` from the record.

Tests: `tests/unit/test_detection_report.py` (UT-DETREP-001 … 007).
