import os
import subprocess
import sys
import platform


def install_and_import(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
        return True
    except ImportError:
        print(f"📦 Installing missing dependency: {package}...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            return True
        except Exception as e:
            print(f"❌ Failed to install {package}: {e}")
            return False


def check_dependencies():
    print("--- 1. Checking Python Dependencies ---")
    # pybind11 is needed for the C++ interface
    install_and_import("pybind11")
    # pyserial is used for hardware detection logic
    install_and_import("pyserial", "serial")
    # Pillow is used by FPVWidget.py to convert cv::Mat/numpy frames into
    # something Tkinter can render (Image, ImageTk)
    install_and_import("Pillow", "PIL")
    # numpy is REQUIRED at runtime, not just for building: DroneBackend's
    # pybind11 bindings return frames via py::array_t<uint8_t>
    # (matToNumpy() in the .cpp), and VideoWorker.get_frame() /
    # FPVWidget.update_fpv() both operate on that array directly (indexing,
    # slicing for BGR->RGB, .shape, .size). Without numpy installed in the
    # interpreter actually running main.py, the video thread crashes the
    # first time a frame arrives with "ModuleNotFoundError: No module
    # named 'numpy'" -- this was missing from the original dependency list
    # even though it's pulled in transitively at build time by pybind11's
    # numpy.h header, which doesn't install anything for you at runtime.
    install_and_import("numpy")
    print("✅ All Python dependencies are ready.")


def check_video_performance_deps():
    """
    Optional, but directly relevant to FPV latency: these don't change
    correctness, they change how many milliseconds it takes to get each
    frame from the capture card onto the pilot's screen.
    """
    print("\n--- 2. Checking FPV Pipeline Performance Options ---")

    # ---- Pillow-SIMD -----------------------------------------------------
    # FPVWidget.update_fpv() calls Image.resize() on every single frame
    # (every ~16-20ms at 50-60fps) to fit the capture frame to the panel.
    # Stock Pillow's resize is pure C but single-threaded/non-vectorized;
    # Pillow-SIMD is a drop-in replacement (same "PIL" import name) built
    # with AVX2 SIMD instructions and is commonly 4-6x faster specifically
    # on resize -- which is the single most expensive per-frame operation
    # in this pipeline outside of the capture/decode itself.
    #
    # NOT auto-installed here because:
    #   1. It must REPLACE Pillow (same import, conflicting package), so
    #      blindly installing it could break other tooling that expects
    #      stock Pillow behavior/version pinning.
    #   2. Pillow-SIMD's prebuilt wheels lag behind new CPython releases.
    #      This project is on Python 3.14 (very recent at time of writing)
    #      -- there is a real chance no prebuilt wheel exists yet, which
    #      would force a from-source build requiring a C compiler + AVX2
    #      support, and isn't worth the friction for most setups.
    #
    # Left as a manual opt-in:
    print("ℹ️  Pillow-SIMD can significantly speed up the per-frame resize")
    print("   in FPVWidget.py (often 4-6x faster than stock Pillow).")
    print("   Optional, manual step (only if you want to try it):")
    print("     pip uninstall pillow")
    print("     pip install pillow-simd")
    print("   If that fails to find a wheel for your Python version, stick")
    print("   with stock Pillow -- it already installed fine above.")

    # ---- numpy backend sanity check --------------------------------------
    try:
        import numpy as np
        print(f"✅ numpy {np.__version__} import OK "
              f"(this is what get_latest_frame() returns frames as)")
    except ImportError:
        print("❌ numpy still not importable -- video thread will crash on "
              "the first frame. Re-run this script or install manually: "
              "pip install numpy")


def check_opencv_cpp():
    """
    OpenCV for the C++ side (VideoLink.cpp / DroneBackend bindings) is NOT
    something pip can install -- 'pip install opencv-python' only gives you
    Python bindings, not the headers/.lib/.dll files Visual Studio needs to
    compile and link DroneBackend.pyd. This check only *detects* a local
    install and tells you what to do if it's missing; it can't fetch and
    wire up a C++ library automatically the way install_and_import() does
    for pure-Python packages.
    """
    print("\n--- 3. Checking OpenCV (C++ build dependency) ---")

    if platform.system() != "Windows":
        print("ℹ️  Non-Windows platform detected -- VideoLink.cpp currently "
              "targets the Windows Media Foundation API, so this project "
              "isn't expected to build here anyway. Skipping OpenCV check.")
        return

    candidates = []

    env_dir = os.environ.get("OPENCV_DIR")
    if env_dir:
        candidates.append(os.path.join(env_dir, "include", "opencv2", "opencv.hpp"))

    for base in (r"C:\opencv", r"C:\tools\opencv", r"C:\Tools\opencv"):
        candidates.append(os.path.join(base, "build", "include", "opencv2", "opencv.hpp"))

    found_path = None
    opencv_root = None
    for c in candidates:
        if os.path.exists(c):
            found_path = c
            break

    if found_path:
        include_dir = os.path.dirname(os.path.dirname(found_path))
        opencv_root = os.path.dirname(include_dir)          # .../build
        print(f"✅ Found OpenCV headers at: {include_dir}")
        print("   Double-check the DroneBackend project's VS settings point here:")
        print(f"   C/C++ > General > Additional Include Directories -> {include_dir}")
    else:
        print("❌ OpenCV C++ SDK not found in any common location "
              "(checked OPENCV_DIR env var, C:\\opencv, C:\\tools\\opencv).")
        print("   Action needed (one-time, manual):")
        print("   1. Download the prebuilt Windows package from https://opencv.org/releases/")
        print(r"   2. Extract it to C:\opencv (so headers land at C:\opencv\build\include)")
        print("   3. In the DroneBackend VS project properties (All Configurations, x64):")
        print(r"        Include dirs : C:\opencv\build\include")
        print(r"        Lib dirs     : C:\opencv\build\x64\vc16\lib")
        print("        Linker input : opencv_world4xxd.lib (Debug) / opencv_world4xx.lib (Release)")
        print("      (replace 4xx with your actual version number, e.g. 4100)")
        print(r"   4. Make sure opencv_world4xx(d).dll is on PATH or next to the built .pyd/.exe.")
        return

    # ---- Parallel-processing plugin check --------------------------------
    # This is exactly what your last run's log showed failing to load:
    #   opencv_core_parallel_onetbb4120_64d.dll => FAILED
    #   opencv_core_parallel_tbb4120_64d.dll    => FAILED
    #   opencv_core_parallel_openmp4120_64d.dll => FAILED
    # These are OPTIONAL plugin DLLs, distributed separately from the main
    # OpenCV Windows package since ~4.5. Without one of them, OpenCV falls
    # back to single-threaded execution for its internal parallel_for_
    # calls (color conversion, resize, some codec paths). For this
    # project's actual latency-critical path -- cv::VideoCapture::read()
    # decoding MJPEG frames off the USB capture card -- the impact is
    # usually small since MSMF/DSHOW do their own internal decode
    # threading regardless. But it's a free win if you do any additional
    # cv::Mat processing (e.g. adding an on-screen overlay, OSD, or CV-based
    # tracking on the FPV feed later), so worth having.
    bin_dir = os.path.join(opencv_root, "x64", "vc16", "bin")
    plugin_names = [
        "opencv_core_parallel_onetbb4120_64d.dll",
        "opencv_core_parallel_tbb4120_64d.dll",
        "opencv_core_parallel_openmp4120_64d.dll",
    ]
    have_plugin = any(os.path.exists(os.path.join(bin_dir, p)) for p in plugin_names)

    if have_plugin:
        print("✅ A parallel-processing plugin DLL is present -- OpenCV can "
              "multi-thread internal cv::Mat operations.")
    else:
        print("ℹ️  No parallel-processing plugin DLL found "
              f"(checked {bin_dir}).")
        print("   Not required -- capture will still work single-threaded, "
              "same as your last run.")
        print("   Optional speed-up if you later add per-frame CV processing")
        print("   (overlays, filters, tracking) on top of the raw FPV feed:")
        print("     Download 'opencv_parallel_openmp' or 'opencv_parallel_tbb'")
        print("     from the Extra/optional downloads section of the same")
        print("     https://opencv.org/releases/ page you got opencv_world from,")
        print(f"     and drop the .dll into: {bin_dir}")
        print("   OpenMP is the simplest to add (no extra runtime dependency")
        print("   beyond what MSVC already ships).")


def check_cpp_backend():
    print("\n--- 4. Checking C++ Threaded Backend ---")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(base_dir)

    # The compiled pybind11 module is DroneBackend (see PYBIND11_MODULE(DroneBackend, m)
    # in the bindings .cpp, and `import DroneBackend` in main.py) -- NOT DroneLink.
    # DroneLink is just a C++ class exposed *inside* that module.
    search_paths = [
        os.path.join(root_dir, "x64", "Release", "DroneBackend.pyd"),
        os.path.join(root_dir, "x64", "Debug", "DroneBackend.pyd"),
    ]

    found = False
    for p in search_paths:
        if os.path.exists(p):
            print(f"✅ Found compiled backend: {os.path.basename(p)} ({p})")
            found = True
            break

    if not found:
        print("⚠️  Warning: DroneBackend.pyd not found.")
        print("   Action: Rebuild the 'DroneBackend' project in Visual Studio (x64 Debug/Release).")
        print("   If the build fails with 'cannot open source file opencv2/opencv2.hpp',")
        print("   see the OpenCV check above -- that's almost always the cause.")


def check_hardware():
    print("\n--- 5. Checking Flight Controller (F405) Connection ---")
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())

    drone_found = False
    for port in ports:
        # SpeedyBee/XFlight F405 usually identifies as STM32 Virtual Com Port
        if "STM" in port.description or "USB Serial" in port.description:
            print(f"✅ Drone detected on {port.device} ({port.description})")
            drone_found = True
            break

    if not drone_found:
        print("❌ Flight Controller not detected.")
        print("   Note: Ensure the drone is powered (Battery or USB) and drivers are installed.")


if __name__ == "__main__":
    print("========================================")
    print("   Project R.A.D - System Readiness    ")
    print("========================================\n")

    check_dependencies()
    check_video_performance_deps()
    check_opencv_cpp()
    check_cpp_backend()
    check_hardware()

    print("\nSetup check complete. You are ready to run DroneCockpitApp.py.")