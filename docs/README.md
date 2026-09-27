# Project R.A.D. — Documentation

**R.A.D.** (Rescue and Detection) is a drone ground control station: a hybrid
C++/Python system that talks to a Betaflight flight controller over MSP
(USB cable) and listens to its CRSF telemetry over the ExpressLRS radio link,
captures and renders an analog FPV feed, and runs a real-time YOLO detection
pipeline that georeferences what it sees onto a map.

This `docs/` folder documents the whole app: the C++ core (`DroneBackend.pyd`,
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
4. **[modules/crsflink.md](modules/crsflink.md)** — `CrsfLink`: the
   ELRS radio telemetry link via the RadioMaster Pocket (auto-detect,
   auto-reconnect, link status). Its pure helpers are in
   [modules/crsf-protocol.md](modules/crsf-protocol.md) (`CrsfProtocol`
   framing/decoders, `CrsfStateMapper` frames → `DroneState`), and COM
   port discovery is in
   [modules/serial-port-scan.md](modules/serial-port-scan.md). Sending
   commands from the laptop to the drone (gimbal, emergency, movement) is
   **not implemented yet**; the design proposal is in
   [modules/command-uplink.md](modules/command-uplink.md).
5. **[modules/videolink.md](modules/videolink.md)** — `VideoLink`: analog
   video capture, the link-state machine, native GDI rendering, and the
   software OSD overlay.
6. **[modules/detectionlink.md](modules/detectionlink.md)** — `DetectionLink`:
   the YOLO inference pipeline, dual preview/inference threads, object
   tracking, and the three georeferencing strategies.
7. **[modules/bindings.md](modules/bindings.md)** — `Bindings.cpp`: the
   pybind11 surface (`DroneBackend` module) exposed to Python, organized by
   the C++ class it wraps.
8. **[FRONTEND.md](FRONTEND.md)** — index for the Python/Tkinter cockpit UI;
   links out to one page per file/module under `frontend/` (app shell,
   workers, each instrument widget, detection map, tile cache, OSD
   controls, build tooling).

## Module map

### C++ backend (`DroneBackend.pyd`)

| C++ class      | File(s)                              | Owns                                            | Exposed to Python as        |
|----------------|---------------------------------------|--------------------------------------------------|------------------------------|
| `DroneLink`    | `DroneLink.h/.cpp`                    | MSP serial worker thread, `DroneState` (+ `RadioLinkStats`) | `DroneBackend.DroneLink`     |
| `CrsfLink`     | `CrsfLink.h/.cpp`                     | ELRS telemetry thread, Pocket auto-detect/reconnect, link status | `DroneBackend.CrsfLink`, `RadioLinkStatus`, `RadioLinkStats` |
| `Crsf::` / `CrsfStateMapper` | `CrsfProtocol.h/.cpp`, `CrsfStateMapper.h/.cpp` | CRSF framing, decoders, frames → `DroneState` (pure logic) | (used internally by `CrsfLink`) |
| —              | `SerialPortScan.h/.cpp`               | COM enumeration with USB names, MSP-verified FC detection | `SerialPortInfo`, `enumerate_serial_ports()`, `auto_detect_fc()` |
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
| `radio_link_indicator.py` | [frontend/radio-link-indicator.md](frontend/radio-link-indicator.md) |
| `link_mode.py` (USB vs radio behaviour of every widget) | [frontend/link-mode.md](frontend/link-mode.md) |
| `dual_layer.py`, `radio_panels.py` (radio faces of the IMU / Mag / FC Status panels) | [frontend/dual-layer.md](frontend/dual-layer.md) |
| `FPVWidget.py` | [frontend/fpv-widget.md](frontend/fpv-widget.md) |
| `DetectionMapWidget.py` | [frontend/detection-map-widget.md](frontend/detection-map-widget.md) |
| `MapTiles.py`, `prefetch_tiles.py` | [frontend/map-tiles.md](frontend/map-tiles.md) |
| `osd_overlay_controls.py`, `osd_layout.json` | [frontend/osd-overlay-controls.md](frontend/osd-overlay-controls.md) |
| `Setup_Project.py`, `DroneTest.py` | [frontend/setup-tools.md](frontend/setup-tools.md) |

## Screenshots

UI screenshots live in [`../Software_progress/`](../Software_progress/)
(e.g. `MainView.png`, `GPS_sensor.png`, `Detection_and_mapping.png`,
`FPV_with_osd.png`), and hardware build photos and drawings live in
[`../Hardware_progress/`](../Hardware_progress/). Reference them from the
relevant page with a relative path, e.g.
`![Main view](../../Software_progress/MainView.png)` from a page under
`frontend/`. There is no screenshot of the radio-link indicator yet.

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
