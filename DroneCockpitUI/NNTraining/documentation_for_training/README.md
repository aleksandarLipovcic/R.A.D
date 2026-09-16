# Project R.A.D. — Documentation

**R.A.D.** (Rescue and Detection) is a drone ground control station: a hybrid
C++/Python system that talks to a Betaflight flight controller over MSP,
captures and renders an analog FPV feed, and runs a real-time YOLO detection
pipeline that georeferences what it sees onto a map.

This folder documents the whole app: the C++ core (`DroneBackend.pyd`,
compiled via pybind11) and the Python/Tkinter cockpit UI
(`DroneCockpitUI.py`) that drives it.

## Reading order

1. **[ARCHITECTURE.md](ARCHITECTURE.md)** — the big picture: process layout
   (C++ core + Python UI + their worker threads), the "provider" decoupling
   pattern shared by every link class, the two telemetry trampolines, the
   Tk update loop, and how a frame/telemetry sample flows end to end.
2. **[modules/dronelink.md](modules/dronelink.md)** — `DroneLink`: the MSP
   serial link to the flight controller, `DroneState`, polling cadence,
   calibration, GPS UBX passthrough.
3. **[modules/sensors.md](modules/sensors.md)** — the sensor helper classes
   that turn raw MSP payloads into physical units: `GPSNeoM10`, `IMUSensor`,
   `MagQMC5883L`, `BaroBMP280`.
4. **[modules/videolink.md](modules/videolink.md)** — `VideoLink`: analog
   video capture, the link-state machine, native GDI rendering, and the
   software OSD overlay.
5. **[modules/detectionlink.md](modules/detectionlink.md)** — `DetectionLink`:
   the YOLO inference pipeline, dual preview/inference threads, object
   tracking, and the three georeferencing strategies.
6. **[modules/bindings.md](modules/bindings.md)** — `Bindings.cpp`: the
   pybind11 surface (`DroneBackend` module) exposed to Python, organized by
   the C++ class it wraps.
7. **[FRONTEND.md](FRONTEND.md)** — index for the Python/Tkinter cockpit UI;
   links out to one page per file/module under `frontend/` (app shell,
   workers, each instrument widget, detection map, tile cache, OSD
   controls, build tooling).
8. **[ML_PIPELINE.md](ML_PIPELINE.md)** — index for the standalone YOLO
   training pipeline that produces the `.onnx` model `DetectionLink`
   loads; links out to one page per script under `ml-pipeline/`.

## Module map

### C++ backend (`DroneBackend.pyd`)

| C++ class      | File(s)                              | Owns                                            | Exposed to Python as        |
|----------------|---------------------------------------|--------------------------------------------------|------------------------------|
| `DroneLink`    | `DroneLink.h/.cpp`                    | MSP serial worker thread, `DroneState`           | `DroneBackend.DroneLink`     |
| `GPSNeoM10`    | `GPSneoM10.h/.cpp`                    | Static GPS/UBX parsers + frame builders          | `GPSReading`, `SVInfoEntry`, `NavStatus`, `GPSConfig*` |
| `IMUSensor`    | `IMUSensor.h/.cpp`                    | Raw→scaled IMU unit conversion                   | `DroneBackend.IMUSensor`     |
| `MagQMC5883L`  | `MagQMC5883L.h/.cpp`                  | Heading computation, MSP checksum helper         | (used internally by `DroneLink::parseDebug`) |
| `BaroBMP280`   | `BaroBMP280.h/.cpp`                   | `MSP_ALTITUDE` parsing + unit scaling             | (used internally by `DroneLink::parseBaro`) |
| `VideoLink`    | `VideoLink.h/.cpp`                    | Capture thread, native render window, OSD overlay | `DroneBackend.VideoLink`     |
| `DetectionLink`| `Detectionlink.h/.cpp`                | Preview + inference threads, tracker, georeferencing | `DroneBackend.DetectionLink` |
| —              | `Bindings.cpp`                        | pybind11 module definition (`PYBIND11_MODULE(DroneBackend, m)`) | — |

### Python frontend

One file per module/widget under [`frontend/`](FRONTEND.md) — see
[FRONTEND.md](FRONTEND.md) for the full index. Quick map:

| Module | Doc page |
|---|---|
| `DroneCockpitUI.py` (entry point, `DroneCockpitApp`) | [frontend/app-shell.md](frontend/app-shell.md) |
| `telemetry_worker.py`, `video_worker.py`, `detection_worker.py` | [frontend/workers.md](frontend/workers.md) |
| `IMUWidget.py` | [frontend/imu-widget.md](frontend/imu-widget.md) |
| `MagWidget.py` | [frontend/mag-widget.md](frontend/mag-widget.md) |
| `Drone3DView.py` | [frontend/drone3d-view.md](frontend/drone3d-view.md) |
| `BaroWidget.py` | [frontend/baro-widget.md](frontend/baro-widget.md) |
| `GPSWidget.py` | [frontend/gps-widget.md](frontend/gps-widget.md) |
| `FCStatusWidget.py` | [frontend/fc-status-widget.md](frontend/fc-status-widget.md) |
| `ArmingWidget.py` | [frontend/arming-widget.md](frontend/arming-widget.md) |
| `FPVWidget.py` | [frontend/fpv-widget.md](frontend/fpv-widget.md) |
| `DetectionMapWidget.py` | [frontend/detection-map-widget.md](frontend/detection-map-widget.md) |
| `MapTiles.py`, `prefetch_tiles.py` | [frontend/map-tiles.md](frontend/map-tiles.md) |
| `osd_overlay_controls.py`, `osd_layout.json` | [frontend/osd-overlay-controls.md](frontend/osd-overlay-controls.md) |
| `Setup_Project.py`, `DroneTest.py` | [frontend/setup-tools.md](frontend/setup-tools.md) |

### ML training pipeline

A separate subproject (no runtime dependency on `DroneBackend.pyd` or
`DroneCockpitUI.py`) that produces the `.onnx` model + `.names` file
`DetectionLink.set_model_path()` loads. See
[ML_PIPELINE.md](ML_PIPELINE.md) for the full pipeline-stage diagram.

| Script | Doc page |
|---|---|
| `setup.py` | [ml-pipeline/setup.md](ml-pipeline/setup.md) |
| `class_map.py` | [ml-pipeline/class-map.md](ml-pipeline/class-map.md) |
| `prepare_datasets.py` | [ml-pipeline/prepare-datasets.md](ml-pipeline/prepare-datasets.md) |
| `train.py` | [ml-pipeline/train.md](ml-pipeline/train.md) |
| `mosaic_guard.py` | [ml-pipeline/mosaic-guard.md](ml-pipeline/mosaic-guard.md) |
| `predict_test.py` | [ml-pipeline/predict-test.md](ml-pipeline/predict-test.md) |
| `vram_diagnostic.py` | [ml-pipeline/vram-diagnostic.md](ml-pipeline/vram-diagnostic.md) |
| `verify_pipeline_assumptions.py` | [ml-pipeline/verify-pipeline-assumptions.md](ml-pipeline/verify-pipeline-assumptions.md) |
| `check_label_gaps.py` | [ml-pipeline/check-label-gaps.md](ml-pipeline/check-label-gaps.md) |
| `cross_reference_gaps.py` | [ml-pipeline/cross-reference-gaps.md](ml-pipeline/cross-reference-gaps.md) |
| `resolve_cross_class_conflicts.py` | [ml-pipeline/resolve-cross-class-conflicts.md](ml-pipeline/resolve-cross-class-conflicts.md) |
| `generate_pseudo_labels.py` | [ml-pipeline/generate-pseudo-labels.md](ml-pipeline/generate-pseudo-labels.md) |
| `review_labels.py` | [ml-pipeline/review-labels.md](ml-pipeline/review-labels.md) |
| `finetune_and_resolve_queue.py` | [ml-pipeline/finetune-and-resolve-queue.md](ml-pipeline/finetune-and-resolve-queue.md) |
| `dedupe_real_labels.py` | [ml-pipeline/dedupe-real-labels.md](ml-pipeline/dedupe-real-labels.md) |

## Screenshots

Add UI screenshots under `docs/images/` (e.g. `main-window.png`,
`gps-widget.png`, `detection-preview.png`) and reference them from the
relevant page — [FRONTEND.md](FRONTEND.md)/`modules/*.md` for widget-level
shots, this README for a single "main window" overview shot. No images are
included yet.

## Conventions used throughout this documentation

- **MSP** = MultiWii Serial Protocol, the wire protocol Betaflight speaks to
  a companion computer. Command IDs referenced here are Betaflight 4.5.x
  values (see `namespace MSP` in `DroneLink.h`).
- Struct/field names are given in their **C++ form**; the pybind11 layer
  renames everything to `snake_case` for the Python side (see
  [modules/bindings.md](modules/bindings.md)).
- "Provider pattern" refers to a recurring design choice in this codebase:
  a class exposes a `setXProvider(std::function<...>)` setter instead of
  taking a concrete dependency, so e.g. `DetectionLink` never needs to know
  `DroneLink` exists. This is explained once in
  [ARCHITECTURE.md](ARCHITECTURE.md) and referenced from every module page.
