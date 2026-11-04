# Project R.A.D. (Recon Aerial Drone)

A search-and-rescue drone ground control station. A C++ core
(`DroneBackend.pyd`, pybind11) talks to a Betaflight flight controller over
MSP (USB cable) and over CRSF telemetry via ExpressLRS (RadioMaster Pocket),
captures and renders the analog FPV feed, and runs a YOLO detection pipeline
that georeferences what it sees onto a map. A Python/Tkinter cockpit
(`DroneCockpitUI/DroneCockpitUI.py`) drives it all.

## Repository layout

| Folder | Contents |
|---|---|
| [`DroneBackend/`](DroneBackend/) | C++ backend → `DroneBackend.pyd` (Visual Studio project): `src/core`, `src/sensors`, `src/radio`, `src/video`, `src/detection`, `src/bindings`, `tests/` |
| [`DroneCockpitUI/`](DroneCockpitUI/) | Python cockpit: `widgets/`, `workers/`, `core/`, `control/`, `config/`, `tools/`, `tests/` (unit / integration / system), `NNTraining/` model pipeline |
| [`Firmware/`](Firmware/) | Pocket Lua command bridge, Betaflight CLI backups |
| [`IMUTests/`](IMUTests/) | GoogleTest project for the C++ IMU parser |
| [`docs/`](docs/) | Software documentation, plus `doxygen/`, `hardware/` (photos, drawings), `screenshots/` |

## Documentation

Start at **[docs/README.md](docs/README.md)** for the reading order, then:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): the big picture (threads,
  telemetry sources, data flow)
- [docs/modules/](docs/modules/): one page per C++ unit
- [docs/FRONTEND.md](docs/FRONTEND.md): index of the Python UI pages
- [DroneCockpitUI/NNTraining/documentation_for_training/](DroneCockpitUI/NNTraining/documentation_for_training/README.md):
  the detection-model training pipeline
