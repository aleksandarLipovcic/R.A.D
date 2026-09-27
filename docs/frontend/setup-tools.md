# `Setup_Project.py` & `DroneTest.py` — environment tooling

Neither script is imported by `DroneCockpitUI.py`. Both are meant to be run
directly by a developer, independent of the cockpit app itself.

## `Setup_Project.py`

**Run as:** `python Setup_Project.py`

A five-stage readiness checker for the whole build/run environment,
printed to the console. Does not launch the cockpit itself — its own final
line says "You are ready to run DroneCockpitApp.py" once done.

### 1. Python dependencies (`check_dependencies`)

Installs (via `install_and_import`, which `pip install`s on
`ImportError`) whatever's missing among:

| Package | Import name | Why it's needed |
|---|---|---|
| `pybind11` | `pybind11` | Needed to *build* the C++ extension |
| `pyserial` | `serial` | Hardware auto-detection (`serial.tools.list_ports`, used by `check_hardware()` below and by `DroneBackend.auto_detect_f405()`'s Python-side equivalents) |
| `Pillow` | `PIL` | Used by `FPVWidget.py`-adjacent code to convert `cv::Mat`/numpy frames into something Tkinter can render, where still needed |
| `numpy` | `numpy` | **Required at runtime**, not just at build time. `DroneBackend`'s pybind11 bindings return frames via `py::array_t<uint8_t>` (`matToNumpy()` in `Bindings.cpp`), and any code touching `VideoLink.get_latest_frame()`'s return value operates on that array directly. Without numpy installed in the interpreter actually running the app, that code crashes on the first frame with `ModuleNotFoundError`. The comment notes this was missing from an earlier dependency list even though it's pulled in *transitively* at build time by pybind11's `numpy.h` header — which installs nothing for you at runtime |

### 2. FPV pipeline performance options (`check_video_performance_deps`)

Optional, informational only — these change frame latency, not
correctness:

- **Pillow-SIMD** — a drop-in replacement for stock Pillow (same `PIL`
  import name) built with AVX2 SIMD instructions, commonly 4–6× faster
  specifically at `Image.resize()`, which is called on every single frame
  wherever Python-side frame resizing still happens — the single most
  expensive per-frame operation in that path outside of capture/decode
  itself. **Not auto-installed**, for two reasons stated in the source:
  it must *replace* Pillow (same import, conflicting package — could break
  other tooling expecting stock Pillow), and Pillow-SIMD's prebuilt wheels
  lag behind new CPython releases (this project runs Python 3.14, recent
  enough that no prebuilt wheel may exist yet, which would force a
  from-source build needing a C compiler + AVX2 support). Left as a manual
  `pip uninstall pillow && pip install pillow-simd` step.
- **numpy backend sanity check** — imports numpy and prints its version,
  confirming it's importable in the exact interpreter running the check.

### 3. OpenCV C++ build dependency (`check_opencv_cpp`)

This one only *detects*, it can't fix anything automatically:
`pip install opencv-python` only provides Python bindings, not the
headers/`.lib`/`.dll` files Visual Studio needs to compile and link
`DroneBackend.pyd`. On non-Windows platforms this check is skipped
entirely, with a note that `VideoLink.cpp` currently targets the Windows
Media Foundation API, so the project isn't expected to build elsewhere
anyway.

Search order for an existing OpenCV install: `OPENCV_DIR` env var, then
`C:\opencv`, `C:\tools\opencv`, `C:\Tools\opencv`. If found, prints the
exact Visual Studio project setting (`Additional Include Directories`) to
verify. If not found, prints the full manual setup procedure: download
from opencv.org, extract to `C:\opencv`, and the three VS project settings
needed (include dirs, lib dirs, linker input `opencv_world4xxd.lib` for
Debug / `opencv_world4xx.lib` for Release).

Also checks for an optional **parallel-processing plugin DLL**
(`opencv_core_parallel_{onetbb,tbb,openmp}4120_64d.dll`) — without one,
OpenCV falls back to single-threaded execution for internal
`parallel_for_` calls (color conversion, resize, some codec paths). The
source notes this has usually-small impact on this project's actual
latency-critical path (`cv::VideoCapture::read()` decoding MJPEG off the
USB capture card, since MSMF/DSHOW do their own internal decode threading
regardless), but is described as "a free win" for any additional
`cv::Mat` processing added later (overlays, OSD, CV-based tracking) — OSD
rendering already exists in `VideoLink`, so this is now directly relevant,
not just a future consideration.

### 4. Compiled C++ backend (`check_cpp_backend`)

Looks for the built extension at `x64/Release/DroneBackend.pyd` or
`x64/Debug/DroneBackend.pyd` relative to the script's parent directory.
Explicitly notes the module name is `DroneBackend` (from
`PYBIND11_MODULE(DroneBackend, m)` in `Bindings.cpp`, matching
`import DroneBackend` in the app) — **not** `DroneLink`, which is only a
C++ class exposed *inside* that module, a distinction worth stating since
it's an easy mix-up. If not found, points at rebuilding the VS project and
flags OpenCV header issues as the most common build failure cause.

### 5. Flight controller connection (`check_hardware`)

Uses `serial.tools.list_ports.comports()` and looks for `"STM"` or
`"USB Serial"` in a port's description — the SpeedyBee/XFlight F405 this
project targets usually identifies as an STM32 Virtual Com Port.

## `DroneTest.py`

**Run as:** `python DroneTest.py`

A standalone smoke-test script for the C++ bridge, independent of the full
cockpit UI. Performs portable path resolution (locates
`../x64/Debug/DroneBackend.pyd` relative to its own location, so it works
regardless of where the project folder is moved), verifies the `.pyd`
exists, appends the backend path to `sys.path` and calls
`os.add_dll_directory()` (required on Python 3.8+ on Windows to load a
binary extension with native DLL dependencies), then connects to a
hardcoded `COM3` and prints a live battery-voltage/roll/pitch readout at
20 Hz.

> **This script's API calls do not match the current bindings.** It calls
> `drone.getBatteryVoltage()` and `drone.getAttitude()` (returning a
> `[Roll, Pitch, Yaw]` list), but the bindings documented in
> [../modules/bindings.md](../modules/bindings.md) expose telemetry only
> through `DroneLink.get_latest_state()` → a `DroneState` snapshot object —
> there is no `getBatteryVoltage()`/`getAttitude()` method on the current
> `DroneLink` binding. This script is not imported by `DroneCockpitUI.py`
> and appears to predate the current `DroneState`-snapshot-based binding
> shape. Don't use it as a reference for how to call `DroneBackend` today —
> use `DroneTest.py` as a template at most for the path-resolution and
> `add_dll_directory()` pattern, not for the API calls themselves.
