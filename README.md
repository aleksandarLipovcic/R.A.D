# Project R.A.D. (Rescue and Detection)

A search-and-rescue drone ground control station. A C++ core
(`DroneBackend.pyd`, pybind11) talks to a Betaflight flight controller over
MSP (USB cable) and over CRSF telemetry via ExpressLRS (RadioMaster Pocket),
captures and renders the analog FPV feed, and runs a YOLO detection pipeline
that georeferences what it sees onto a map. A Python/Tkinter cockpit
(`DroneCockpitUI/DroneCockpitUI.py`) drives it all.

## Repository layout

| Folder | Contents |
|---|---|
| [`DroneBackend/`](DroneBackend/) | C++ backend → `DroneBackend.pyd` (Visual Studio project) |
| [`DroneCockpitUI/`](DroneCockpitUI/) | Python cockpit UI, test suites, `NNTraining/` model pipeline |
| [`IMUTests/`](IMUTests/) | GoogleTest project for the C++ IMU parser |
| [`docs/`](docs/) | Software documentation |
| [`Doxygen_conf/`](Doxygen_conf/) | Doxygen configuration + comment filter |
| [`Hardware_progress/`](Hardware_progress/) | Build photos and mechanical drawings |
| [`Software_progress/`](Software_progress/) | UI screenshots |

## Documentation

Start at **[docs/README.md](docs/README.md)** for the reading order, then:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): the big picture (threads,
  telemetry sources, data flow)
- [docs/modules/](docs/modules/): one page per C++ unit
- [docs/FRONTEND.md](docs/FRONTEND.md): index of the Python UI pages
- [DroneCockpitUI/NNTraining/documentation_for_training/](DroneCockpitUI/NNTraining/documentation_for_training/README.md):
  the detection-model training pipeline
