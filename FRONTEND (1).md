# Frontend — the Python/Tkinter cockpit UI

**Entry point:** `DroneCockpitUI.py` (`DroneCockpitApp` class)
**Depends on:** `DroneBackend.pyd` (see [modules/bindings.md](modules/bindings.md)),
Tkinter, Pillow, numpy

This page is the index for the Python side. Read [ARCHITECTURE.md](ARCHITECTURE.md)
first for how this side relates to the C++ backend as a whole — the
process/thread diagram, the Tk update loop, and the two telemetry
trampolines are introduced there. Each page below covers one file (or a
small tightly-related group of files) in detail.

## App shell & workers

| Page | Covers |
|---|---|
| [frontend/app-shell.md](frontend/app-shell.md) | `DroneCockpitUI.py` — `DroneCockpitApp`: startup sequence, the draggable panel workspace, `LayoutStore`/`LayoutManagerDialog`, the two telemetry trampolines, the Tk update loop, reconnect handling, shutdown |
| [frontend/workers.md](frontend/workers.md) | `telemetry_worker.py`, `video_worker.py`, `detection_worker.py` — the three background daemon threads that bridge each C++ link object to the Tk thread |

## Instrument-panel widgets

| Page | Covers |
|---|---|
| [frontend/imu-widget.md](frontend/imu-widget.md) | `IMUWidget.py` — MPU-6500 display, three-state aviation alerting model |
| [frontend/mag-widget.md](frontend/mag-widget.md) | `MagWidget.py` — QMC5883L heading display, layout-safety invariants |
| [frontend/drone3d-view.md](frontend/drone3d-view.md) | `Drone3DView.py` — attitude direction indicator, dual mag/gyro compass cross-check |
| [frontend/baro-widget.md](frontend/baro-widget.md) | `BaroWidget.py` — altitude tape + VSI |
| [frontend/gps-widget.md](frontend/gps-widget.md) | `GPSWidget.py` — map/nav/satellite navigation display (largest frontend file) |
| [frontend/fc-status-widget.md](frontend/fc-status-widget.md) | `FCStatusWidget.py` — arm state, battery, sensors, motors, RC |
| [frontend/arming-widget.md](frontend/arming-widget.md) | `ArmingWidget.py` — pre-flight checklist |
| [frontend/fpv-widget.md](frontend/fpv-widget.md) | `FPVWidget.py` — native-rendered live video host + OSD/timer controls |

## Detection & mapping

| Page | Covers |
|---|---|
| [frontend/detection-map-widget.md](frontend/detection-map-widget.md) | `DetectionMapWidget.py` — the separate detection/map `Toplevel` window |
| [frontend/map-tiles.md](frontend/map-tiles.md) | `MapTiles.py` (disk tile cache) + `prefetch_tiles.py` (offline pre-download CLI) |

## OSD

| Page | Covers |
|---|---|
| [frontend/osd-overlay-controls.md](frontend/osd-overlay-controls.md) | `osd_overlay_controls.py` (settings panel + drag placement) and the persisted `osd_layout.json` format |

## Build & environment tooling

| Page | Covers |
|---|---|
| [frontend/setup-tools.md](frontend/setup-tools.md) | `Setup_Project.py` (dependency/build checker) and `DroneTest.py` (standalone smoke test — **stale API**, see that page) |
