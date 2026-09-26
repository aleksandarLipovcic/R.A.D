"""
main.py  —  Drone Control Ground Station
=========================================
Free-form floating instrument panel layout with responsive widgets
and panel visibility control.

Toolbar:
  [Status]  [Layouts ▾]  [Layout: <name>]  [Panels ▾]  [↺ Revert to Saved]
  [⟳ Factory Default]  [🔓 Save & Lock Layout]

Architecture
------------
  Telemetry polling and ui_data dict construction run in a dedicated
  background thread (TelemetryWorker).  The Tk main thread only:
    • drains the worker's queue (non-blocking get_frame())
    • feeds the resulting dict to each widget
    • handles all Tk/UI events

  FPV video capture AND display run entirely independently of Tk.
  VideoLink owns a C++ capture thread (USB/analog dongle -> cv::Mat,
  MJPG, buffer size 1 for minimum latency) that is fully decoupled from
  the 100 Hz MSP telemetry loop. Critically, that same capture thread
  also paints each decoded frame directly into a native Win32 child
  window (see FPVWidget.attach()) -- live video pixels never cross the
  pybind11 boundary, never touch numpy/PIL, and never go through a Tk
  PhotoImage. VideoWorker only polls VideoLink's cheap atomic status
  fields (connected / fps / device name) for the small text overlay bar;
  it no longer fetches frame data at all.

  Widget refresh rates are throttled independently:
    • IMU / Baro / FC Status / Arming / Mag  →  every frame  (~50 Hz)
    • FPV status/FPS text                    →  every 5th frame (~10 Hz; video itself
                                                 is painted natively, entirely outside
                                                 this pump)
    • 3-D attitude view                      →  every 3rd frame (~17 Hz)
    • GPS (heaviest — map redraws)            →  every 5th frame (~10 Hz)

FIXES vs previous version
--------------------------
  1. GPS debug console output removed.
  2. Z-order now works correctly via lift() / lower().
  3. Click-to-front on any panel interaction.
  4. Telemetry offloaded to TelemetryWorker background thread.
  5. Multi-profile layout system: any number of named layouts can be
     saved, previewed, loaded, renamed, or deleted independently, and
     on-screen changes can be reverted to the last-saved version of the
     active profile without discarding it (see LayoutStore /
     LayoutManagerDialog below).
  6. FPV live video panel: VideoLink now paints decoded frames directly
     into a native child window hosted by FPVWidget, instead of pushing
     pixel data through Python/Tk every frame. This eliminates the
     numpy-array marshal, PIL resize, and Tk PhotoImage churn that were
     the main source of the extra latency versus reference tools like
     OBS. VideoWorker's job shrank accordingly -- it only polls cheap
     status fields now. Capture itself is still fully independent (own
     thread, own connect/retry loop) so a dropped or slow-to-appear
     capture device never blocks or is blocked by flight telemetry.
  7. DroneBackend.pyd DLL resolution fixed: previously only the backend's
     own output folder was registered with os.add_dll_directory(), so the
     loader could find DroneBackend.pyd itself but not its transitive
     dependency opencv_world4120d.dll (Python 3.8+ no longer falls back to
     searching PATH for extension-module dependencies). The OpenCV bin
     folder is now registered too, which is what was causing
     "ImportError: DLL load failed while importing DroneBackend: The
     specified module could not be found." See _register_dll_directories()
     below.
  8. Native FPV render-window Z-order / visibility now tracked explicitly.
     The FPV video is a native Win32 child HWND that VideoLink paints into
     directly from C++ (see FPVWidget.attach()) -- Tk's lift()/lower() only
     reorders Tk's own widget tree, so it never touched that HWND, which
     is why the live video used to appear to "punch through" on top of
     unrelated Tk panels regardless of click-to-front order. Every place
     that changes a Tk panel's raised/lowered/visible state now also fires
     a matching VideoLink.raise_window()/lower_window()/show_window()/
     hide_window() call (see DroneCockpitApp._on_panel_zorder /
     _on_panel_visibility) so the native window's OS-level stacking always
     matches what's on screen.
  9. Layout-preview dialog no longer draws panels that are hidden in the
     profile being previewed. Previously every panel that had ever been
     part of a profile was drawn (dimmed if hidden), using whatever rect
     it last had on screen -- often a stale/default position that had
     nothing to do with the panels actually shown in that profile, and
     which also skewed the preview's scale factor. See
     _LayoutPreviewCanvas.render() below.
  10. Object-detection & map window wired in. DetectionLink (C++, own
      thread) pulls frames from VideoLink and telemetry from DroneLink,
      runs YOLO inference, georeferences hits, and hands off
      DetectionRecords. DetectionWorker polls that off the Tk thread at a
      low, steady rate; DetectionMapWidget displays it in its own
      Toplevel window (deliberately separate from the FPV panel -- see
      that module's docstring) opened via the new "🎯 Detections" toolbar
      button. See DroneCockpitApp._get_detection_telemetry /
      _open_detection_window / _pump_detection_records below.
  11. Console cleanup now that CUDA bring-up is done: the one-off "which
      DroneBackend.pyd got imported" diagnostic is gone, and the routine
      LayoutStore load/save confirmations, viewport-clamp notices, and
      inference-backend confirmation are gated behind
      COCKPIT_VERBOSE_LOGGING=1 (off by default) instead of always
      printing. Genuine warnings/errors are unaffected -- see that flag's
      comment above _register_dll_directories() for the full rationale.
  12. Camera mount angle (mode + tilt/pan) now persists across restarts as
      part of the active layout profile, the same way panel geometry
      does -- see LayoutStore's on-disk format and
      DroneCockpitApp._current_snapshot()/_apply_layout() below. Previously
      it silently reset to CAMERA_MOUNT_TILT_DEG/CAMERA_MOUNT_PAN_DEG
      (both 0) every launch, which meant georeferencing was quietly wrong
      until you reopened the "📐 Camera Angle" dialog and re-entered
      whatever the camera was actually mounted at.
"""

import sys
import os
import json
import math
import threading
import time
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import simpledialog, messagebox

from telemetry_worker import TelemetryWorker
from radio_link_indicator import RadioLinkIndicator
from video_worker      import VideoWorker
from detection_worker  import DetectionWorker

from IMUWidget           import IMUWidget
from Drone3DView         import Drone3DView
from BaroWidget          import BaroWidget
from MagWidget           import MagWidget
from GPSWidget           import GPSWidget
from FCStatusWidget      import FCStatusWidget
from ArmingWidget        import ArmingWidget
from FPVWidget           import FPVWidget
from DetectionMapWidget  import DetectionMapWidget

# ── Path configuration ────────────────────────────────────────────────────────
script_dir   = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Release'))
sys.path.append(backend_path)

# DroneBackend.pyd is a native pybind11 extension module. On Python 3.8+,
# the DLL loader used to import extension modules does NOT search PATH for
# the module's own transitive dependencies -- it only searches directories
# explicitly registered via os.add_dll_directory() (plus the folder the
# .pyd itself lives in). This Release build of DroneBackend.pyd depends on
# opencv_world4130.dll (CUDA-enabled OpenCV 4.13.0, not the old debug
# opencv_world4120d.dll), so OpenCV's bin folder has to be registered here
# too, or import fails with a generic "DLL load failed" error that doesn't
# name the missing dependency. It also needs the CUDA and cuDNN runtime
# folders discoverable, for the same reason.
#
# EDIT THIS to match your local OpenCV 4.13.0 (CUDA) install location.
OPENCV_BIN_DIR = r"C:\openCVBuild\build\bin\Release"
CUDA_BIN_DIR   = r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.3\bin\x64"
CUDNN_BIN_DIR  = r"C:\Program Files\NVIDIA\CUDNN\v9.24\bin\13.3\x64"


def _register_dll_directories() -> None:
    """Register every folder DroneBackend.pyd needs its dependencies
    resolved from. Safe to call on any platform/Python version -- no-ops
    if os.add_dll_directory doesn't exist (non-Windows / old Python) or if
    a given folder doesn't exist on this machine.

    NOTE: os.add_dll_directory() alone is not enough for cuDNN 9.x.
    cudnn64_9.dll is a thin frontend that loads its own engine backends
    (cudnn_ops64_9.dll, cudnn_cnn64_9.dll, cudnn_graph64_9.dll, ...) at
    runtime via a classic, flag-less LoadLibrary call, which only
    searches PATH -- it never sees directories registered through
    add_dll_directory. Skipping the PATH prepend below produces exactly
    "Invalid handle. Cannot load symbol cudnnGetVersion": cudnn64_9.dll
    itself loads fine, but it can't pull in its own sibling DLLs."""
    if not hasattr(os, "add_dll_directory"):
        return

    valid_dirs = []
    for candidate in (backend_path, OPENCV_BIN_DIR, CUDA_BIN_DIR, CUDNN_BIN_DIR):
        if not os.path.isdir(candidate):
            print(f"[DroneBackend] DLL directory notice: "
                  f"'{candidate}' does not exist, skipping")
            continue
        try:
            os.add_dll_directory(candidate)
            valid_dirs.append(candidate)
        except Exception as e:
            print(f"[DroneBackend] DLL directory notice: {e}")

    # Belt-and-suspenders: also prepend to PATH for cuDNN's internal
    # flag-less LoadLibrary calls (see note above). Must happen before
    # `import DroneBackend` below.
    if valid_dirs:
        os.environ["PATH"] = os.pathsep.join(valid_dirs) + os.pathsep + os.environ["PATH"]


_register_dll_directories()

import DroneBackend

# ── Verbose diagnostic logging ──────────────────────────────────────────────
# Same convention as DETECTIONLINK_VERBOSE_LOGGING in Detectionlink.cpp:
# routine/diagnostic console output (LayoutStore load/save confirmations,
# viewport-clamp notices, which inference backend got engaged) is off by
# default now that the CUDA build is confirmed working and this app has
# moved past bring-up. Set COCKPIT_VERBOSE_LOGGING=1 in the environment to
# bring all of it back, e.g. while diagnosing a new DroneBackend.pyd build,
# a fresh CUDA/cuDNN install, or a layout-persistence bug. This does NOT
# affect genuine warnings/errors (missing bindings, failed DLL registration,
# a stale .pyd, DetectionLink/VideoLink start failures) -- those stay on
# regardless, since silencing them would hide real problems.
COCKPIT_VERBOSE_LOGGING = os.environ.get("COCKPIT_VERBOSE_LOGGING", "0") == "1"


def _vlog(msg: str) -> None:
    if COCKPIT_VERBOSE_LOGGING:
        print(msg)


# ── Constants ─────────────────────────────────────────────────────────────────
#
# UI_REFRESH_MS is how often the Tk loop wakes up to drain the queue and
# feed widgets.  The backend is polled at 60 Hz inside the worker thread,
# independently of this value.
#
UI_REFRESH_MS  = 20          # ~50 Hz Tk pump — keeps UI snappy
RECONNECT_MS   = 2000

# FPV status polling: how often VideoWorker checks VideoLink's cheap
# atomic status fields (connected / fps / device name) for the overlay
# bar. Live video itself is painted directly by VideoLink into a native
# window (see FPVWidget.attach()) and never touches this poll loop or the
# Tk thread at all -- this cadence only needs to be fast enough for a
# readable FPS counter, not fast enough for smooth video.
VIDEO_POLL_HZ  = 15
VIDEO_PROBE_RETRY_MS = 3000   # how often to retry connect_auto() if no device found yet

# ── Object detection (DetectionLink / DetectionWorker / DetectionMapWidget) ──
#
# Path to the ONNX detection model DetectionLink loads at start() (see
# DetectionLink::setModelPath()). A sibling "<same-stem>.names" file next
# to it -- one class name per line -- is picked up automatically if
# present; otherwise classes show up as "class_N" in the detection list.
#
# EDIT THIS to match your local model location.
DETECTION_MODEL_PATH = os.path.join(script_dir, "models", "yolo26m_main.onnx")

# Square side the model expects, letterboxed -- MUST match whatever
# imgsz export_model.py used (EXPORT_IMGSZ there). yolo26m_main was
# exported at 960 with a static (non-dynamic) input shape, so feeding it
# anything else isn't just less accurate, it's a hard shape mismatch:
# DetectionLink::runInference()'s net.forward() call throws a
# cv::Exception that is NOT caught anywhere on that worker thread, which
# terminates the whole process, not just detection. Keep this in sync
# with set_input_size() below and with EXPORT_IMGSZ in export_model.py.
DETECTION_INPUT_SIZE = 960

# Directory annotated detection screenshots are written to (created on
# first use). Leave as "" to disable screenshot saving -- detections are
# still recorded and georeferenced, just without an image to inspect.
DETECTION_SCREENSHOT_DIR = os.path.join(script_dir, "detections")

# How often DetectionLink runs a detection pass, independent of both the
# 100 Hz telemetry loop and the 30fps FPV capture loop -- see
# kDefaultDetectionIntervalMs in DetectionLink.cpp. Safe to change at
# runtime via DetectionLink.set_detection_interval_ms().
DETECTION_INTERVAL_MS = 250

# How often DetectionWorker polls DetectionLink's cheap status/records
# getters off the Tk thread, and how often the detection map window is
# pumped with any newly-arrived records.
DETECTION_POLL_HZ = 4

# Opt into DetectionLink's CUDA backend (see DetectionLink::setUseCuda()).
# This ONLY works if the OpenCV this DroneBackend.pyd was built against
# has CUDA/cuDNN support compiled in -- the stock opencv-python /
# vcpkg opencv4 port does NOT; you need a source (or vcpkg with the
# cuda+dnn-cuda features) build against the CUDA toolkit + cuDNN version
# installed on this machine. If that build doesn't exist, start() falls
# back to CPU silently -- check the printed "[DetectionLink] inference
# backend: ..." line right after start() (below) to see which one you
# actually got. Confirmed working on this rig's RTX 3060 6GB (laptop) --
# inference currently runs at DNN_TARGET_CUDA (FP32); avg_inference sits
# around 70-85ms at 960x960/YOLO26m on this card, which is on the slow
# side for FP32 CUDA via cv::dnn -- see the perf notes in Detectionlink.cpp
# (DNN_TARGET_CUDA_FP16 / a TensorRT export are the next lever if this
# interval ever needs to come down).
DETECTION_USE_CUDA = True

# Horizontal FOV of the actual camera in degrees, used for every
# pixel-offset -> ray-angle calculation in georeferencing (see
# DetectionLink::setHorizontalFovDeg() / computeWorldRay() in
# Detectionlink.cpp). This MUST match the physical camera, not a guess --
# Detectionlink.h's compiled-in default (90 deg) was never actually
# correct for this rig and was silently never overridden until now.
# Sourced from the E5-FPV night-vision module's own datasheet: "FOV: 100
# deg". Update this if the camera is ever swapped for a different
# model/lens -- check the new module's datasheet, don't assume 90 or
# reuse this value.
DETECTION_CAMERA_FOV_DEG = 100.0

# Known face-on real-world widths (meters) for object-size ranging (see
# DetectionLink::setKnownObjectWidth() / rangeByObjectSize() in
# Detectionlink.cpp) -- this is the ranging method that still works when
# ground-plane ranging can't (shallow ray -- see the georeferencing note
# in the DetectionLink setup code below for why that's the common case
# on this rigidly forward-facing, non-gimbaled camera). Without entries
# here, DetectionLink had NO fallback and many/most detections in level
# flight likely never got georeferenced at all.
#
# Keys MUST exactly match the class names in your model's sibling
# ".names" file (same stem as DETECTION_MODEL_PATH, one class name per
# line) -- open that file and confirm before trusting these. A key that
# doesn't match any real class name fails silently (that class just
# never gets object-size ranging), not with an error.
DETECTION_KNOWN_OBJECT_WIDTHS_M = {
    "person": 0.5,     # average shoulder width, face-on
    "vehicle": 1.8,     # average car width, face-on -- rename/split this
                        # key (e.g. "car"/"truck") if your .names file
                        # doesn't use a single combined "vehicle" class
}

# ── Camera mount angle (manual, NOT the SimpleBGC gimbal) ───────────────
# The SimpleBGC gimbal hardware is physically installed but not
# electrically wired (missing step-down modules) -- there is no live
# pan/tilt readback yet. In the meantime the camera is mounted at a
# FIXED angle chosen by hand before each flight (anywhere from 0 deg
# forward-level, through 90 deg straight down, to 180 deg backward-level,
# or negative for facing upward -- a single tilt hinge, no pan -- though
# the controls below expose a pan value too, ready for the day the
# gimbal's second axis is wired in).
#
# These two constants are now only the FALLBACK defaults -- used to seed
# the very first layout profile, and to fill in a saved profile that
# predates this field or has a missing/malformed "camera_angle" entry
# (see LayoutStore's on-disk format and DroneCockpitApp._apply_layout()).
# Once the app has run once, the mode/tilt/pan you set from the
# "📐 Camera Angle" toolbar button is saved into the active layout
# profile (on dialog close, and again at shutdown as a safety net) and
# restored from there on every future launch, the same way panel
# geometry is -- editing these two constants and restarting no longer
# has any effect once a profile with its own "camera_angle" exists.
#
# Wire snapshot.gimbal_pan_deg/gimbal_tilt_deg to a real SimpleBGC
# readback in _get_camera_mount_angle() once the gimbal is actually
# powered ("auto" mode already exists as a placeholder for exactly that,
# and its choice persists across restarts the same as Manual's values
# do); until then, georeferenced pins will drift off if the toolbar's
# Manual value doesn't match how the camera is actually physically
# angled right now.
#
# Tilting toward 90 (straight down) during search sweeps is the single
# biggest improvement available to ground-plane ranging accuracy -- see
# the DetectionLink setup code below and rangeByGroundPlane()'s doc
# comment in Detectionlink.cpp for why a steep ray matters so much more
# than which ranging method is technically "preferred".
CAMERA_MOUNT_PAN_DEG = 0.0     # startup default only -- see toolbar control
CAMERA_MOUNT_TILT_DEG = 0.0    # 0=forward-level, 90=down, 180=backward-level, negative=up

# ── Bearings-only triangulation tuning ───────────────────────────────
# See DetectionLink::setTriangulationMinBaselineM() /
# setTriangulationMinBearingSpreadDeg() in Detectionlink.h for the full
# rationale -- both exist to reject a triangulated fix computed from
# observations with too little real parallax between them (e.g. the
# drone flew straight at the object with no lateral offset), which would
# otherwise be numerically "valid" but dominated by GPS/heading noise.
# The header's own compiled-in defaults (5.0m / 5.0deg) are reasonable
# starting points and don't strictly need overriding here -- these are
# only broken out as named constants so they're easy to find and tune
# from Python without a rebuild once you've flown a few missions and
# have a feel for how noisy/clean your GPS+heading are in practice.
DETECTION_TRIANGULATION_MIN_BASELINE_M = 5.0
DETECTION_TRIANGULATION_MIN_BEARING_SPREAD_DEG = 5.0

BG_WORKSPACE   = "#1a1a2e"
PANEL_BG       = "#0f0f1a"
PANEL_TITLE_BG = "#16213e"
PANEL_TITLE_FG = "#00d4ff"
GRIP_COLOR     = "#2a2a4a"
GRIP_SIZE      = 8
MIN_PANEL_W    = 120
MIN_PANEL_H    = 60

SNAP_PX        = 14
SNAP_COLOR     = "#00d4ff"
GUIDE_DASH     = (4, 3)

# ── Layout persistence ────────────────────────────────────────────────────────
#   cockpit_layouts.json holds *all* named profiles + which one is active.
#   cockpit_layout.json (singular) is the old single-profile file; if the
#   new file doesn't exist yet but the old one does, it's migrated
#   automatically into a profile called "Default" the first time the app
#   runs after this update, so no existing layout is lost.
_LAYOUTS_FILE         = Path(script_dir) / "cockpit_layouts.json"
_LEGACY_LAYOUT_FILE   = Path(script_dir) / "cockpit_layout.json"
_DEFAULT_PROFILE_NAME = "Default"

# ── Per-widget throttle divisors (frames between updates) ────────────────────
#   1 = every Tk pump cycle, 3 = every third, etc.
_THROTTLE = {
    "imu":       1,
    "baro":      1,
    "fc_status": 1,
    "arming":    1,
    "mag":       1,
    "fpv":       5,   # status/FPS text only now — video itself is painted natively
                       # by VideoLink outside the Tk pump entirely, so this just
                       # needs to be readable (~10 Hz at a 20 ms pump), not fast
    "adi":       3,   # 3-D canvas — heavy redraw, 17 Hz is plenty
    "gps":       5,   # map widget — heaviest, 10 Hz is plenty
}

# ── Default panel layout (x, y, w, h) ────────────────────────────────────────
_DEFAULT_PANELS = {
    "imu":       {"x":  10, "y":  10, "w": 460, "h": 260, "visible": True},
    "mag":       {"x":  10, "y": 280, "w": 460, "h": 210, "visible": True},
    "adi":       {"x": 480, "y":  10, "w": 450, "h": 430, "visible": True},
    "baro":      {"x": 940, "y":  10, "w": 220, "h": 430, "visible": True},
    "gps":       {"x":  10, "y": 780, "w": 700, "h": 300, "visible": True},
    "fc_status": {"x":  10, "y":  10, "w": 440, "h": 140, "visible": True},
    "arming":    {"x":  10, "y": 160, "w": 440, "h": 300, "visible": True},
    "fpv":       {"x": 480, "y": 450, "w": 480, "h": 320, "visible": True},
}

_PANEL_LABELS = {
    "imu":       "IMU / Attitude",
    "mag":       "Magnetometer",
    "adi":       "ADI — Attitude Indicator",
    "baro":      "ALT / VSI — Altitude & Speed",
    "gps":       "GPS Navigation",
    "fc_status": "FC Status — Armed / Mode / Sensors",
    "arming":    "Arming Diagnostics",
    "fpv":       "FPV Feed — Live Capture",
}


# ── Tooltip ──────────────────────────────────────────────────────────────────

class _Tooltip:
    """
    Lightweight hover tooltip for compact, icon-only widgets.

    Shows a small borderless popup near the widget after a short hover
    delay; hides on mouse-leave or click. Attach with `_Tooltip(widget,
    "explanation")`. Call `.set_text(...)` afterwards to update the text
    for widgets whose meaning changes at runtime (e.g. a lock toggle).
    """

    _DELAY_MS = 450

    def __init__(self, widget: tk.Widget, text: str):
        self._widget = widget
        self._text = text
        self._after_id = None
        self._tip = None
        widget.bind("<Enter>", self._schedule, add=True)
        widget.bind("<Leave>", self._hide, add=True)
        widget.bind("<ButtonPress>", self._hide, add=True)

    def set_text(self, text: str) -> None:
        self._text = text

    def _schedule(self, event=None):
        self._cancel_pending()
        self._after_id = self._widget.after(self._DELAY_MS, self._show)

    def _cancel_pending(self):
        if self._after_id is not None:
            try:
                self._widget.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None

    def _show(self):
        self._after_id = None
        if self._tip is not None or not self._text:
            return
        x = self._widget.winfo_rootx() + self._widget.winfo_width() // 2
        y = self._widget.winfo_rooty() + self._widget.winfo_height() + 6
        self._tip = tk.Toplevel(self._widget)
        self._tip.wm_overrideredirect(True)
        try:
            self._tip.attributes("-topmost", True)
        except Exception:
            pass
        frame = tk.Frame(self._tip, bg="#00d4ff", padx=1, pady=1)
        frame.pack()
        tk.Label(
            frame, text=self._text,
            bg="#0f1428", fg="#dce6fa",
            font=("Consolas", 8), padx=8, pady=5,
            justify="left", wraplength=260,
        ).pack()
        self._tip.update_idletasks()
        tip_w = self._tip.winfo_width()
        tip_h = self._tip.winfo_height()

        # Clamp so the tooltip stays fully inside the toplevel window that
        # owns this widget (this is what was missing before — the popup
        # used to be positioned purely relative to the widget, so it could
        # spill past the edge of the application window). Falls back to
        # flipping above the widget if there isn't room below.
        owner = self._widget.winfo_toplevel()
        bounds_x0 = owner.winfo_rootx()
        bounds_y0 = owner.winfo_rooty()
        bounds_x1 = bounds_x0 + owner.winfo_width()
        bounds_y1 = bounds_y0 + owner.winfo_height()

        tip_x = x - tip_w // 2
        tip_x = max(bounds_x0 + 2, min(tip_x, bounds_x1 - tip_w - 2))

        tip_y = y
        if tip_y + tip_h > bounds_y1 - 2:
            tip_y = self._widget.winfo_rooty() - tip_h - 6
        tip_y = max(bounds_y0 + 2, min(tip_y, bounds_y1 - tip_h - 2))

        self._tip.wm_geometry(f"+{tip_x}+{tip_y}")

    def _hide(self, event=None):
        self._cancel_pending()
        if self._tip is not None:
            self._tip.destroy()
            self._tip = None


# ── DraggablePanel ─────────────────────────────────────────────────────────────

class DraggablePanel(tk.Frame):
    """
    Floating, draggable, resizable instrument panel on a Canvas workspace.
    """

    def __init__(self, workspace: tk.Canvas, name: str, title: str,
                 x: int, y: int, w: int, h: int,
                 locked_ref: list,
                 all_panels_ref: dict,
                 on_zorder=None,
                 on_visibility=None,
                 **kwargs):
        super().__init__(workspace, bg=PANEL_BG,
                         highlightthickness=1,
                         highlightbackground="#2a3a5a",
                         **kwargs)

        self._ws            = workspace
        self._locked_ref    = locked_ref
        self._all_panels    = all_panels_ref
        self._name          = name
        self._drag_x        = 0
        self._drag_y        = 0
        self._guide_lines   = []
        self._z_raised      = False

        # Optional callbacks so the owning app can keep anything that
        # lives *outside* Tk's widget tree (e.g. a native child HWND
        # painted into by a different thread/process boundary, such as
        # the FPV render window) in sync with this panel's raised/lowered
        # and shown/hidden state. Tk's lift()/lower()/itemconfigure only
        # reorder/toggle Tk's own widgets -- they have no effect on a
        # foreign HWND, so without an explicit hook like this, a native
        # child window keeps whatever OS-level Z-order/visibility it had
        # regardless of what the user does with the Tk panels around it.
        #   on_zorder(name: str, raised: bool)
        #   on_visibility(name: str, visible: bool)
        self._on_zorder     = on_zorder
        self._on_visibility = on_visibility

        self._item = workspace.create_window(x, y, anchor="nw",
                                             window=self, width=w, height=h)

        # Title bar
        self._title_bar = tk.Frame(self, bg=PANEL_TITLE_BG, cursor="fleur")
        self._title_bar.pack(fill="x", side="top")
        self._title_lbl = tk.Label(
            self._title_bar, text=f"  {title}",
            bg=PANEL_TITLE_BG, fg=PANEL_TITLE_FG,
            font=("Consolas", 9, "bold"), anchor="w",
        )
        self._title_lbl.pack(side="left", fill="x", expand=True)

        self._z_btn = tk.Label(
            self._title_bar, text="⬆⬇",
            bg=PANEL_TITLE_BG, fg="#2a4a6a",
            font=("Consolas", 8), cursor="hand2",
        )
        self._z_btn.pack(side="right", padx=(0, 4))

        # Content area
        self.content = tk.Frame(self, bg=PANEL_BG)
        self.content.pack(fill="both", expand=True)

        # Resize grips
        self._grip = tk.Frame(self, bg=GRIP_COLOR,
                              width=GRIP_SIZE, height=GRIP_SIZE,
                              cursor="size_nw_se")
        self._grip.place(relx=1.0, rely=1.0, anchor="se")

        self._right_edge = tk.Frame(self, bg=GRIP_COLOR,
                                    width=GRIP_SIZE, cursor="sb_h_double_arrow")
        self._right_edge.place(relx=1.0, rely=0, anchor="ne",
                               relheight=1.0, height=0)

        self._bottom_edge = tk.Frame(self, bg=GRIP_COLOR,
                                     height=GRIP_SIZE, cursor="sb_v_double_arrow")
        self._bottom_edge.place(relx=0, rely=1.0, anchor="sw",
                                relwidth=1.0, width=0)

        # ── Bindings ──────────────────────────────────────────────────────────
        for w_ in (self, self._title_bar, self._title_lbl, self.content):
            w_.bind("<ButtonPress-1>", self._on_click_raise, add=True)

        for w_ in (self._title_bar, self._title_lbl):
            w_.bind("<ButtonPress-1>",   self._drag_start)
            w_.bind("<B1-Motion>",       self._drag_motion)
            w_.bind("<ButtonRelease-1>", self._drag_end)
            w_.bind("<ButtonPress-3>",   self._show_context_menu)

        self._z_btn.bind("<ButtonPress-1>",  self._z_btn_click)
        self._z_btn.bind("<ButtonPress-3>",  self._show_context_menu)

        self._grip.bind("<ButtonPress-1>",        self._resize_start)
        self._grip.bind("<B1-Motion>",            self._resize_both)
        self._right_edge.bind("<ButtonPress-1>",  self._resize_start)
        self._right_edge.bind("<B1-Motion>",      self._resize_h)
        self._bottom_edge.bind("<ButtonPress-1>", self._resize_start)
        self._bottom_edge.bind("<B1-Motion>",     self._resize_v)

    # =========================================================================
    # Z-order
    # =========================================================================

    def _on_click_raise(self, event=None):
        self.raise_panel()

    def raise_panel(self):
        self.lift()
        self._ws.tag_raise(self._item)
        if self._on_zorder is not None:
            self._on_zorder(self._name, True)

    def lower_panel(self):
        self.lower()
        try:
            self._ws.tag_raise("grid")
        except Exception:
            pass
        if self._on_zorder is not None:
            self._on_zorder(self._name, False)

    def _z_btn_click(self, event):
        self._z_raised = not self._z_raised
        if self._z_raised:
            self.raise_panel()
            self._z_btn.config(fg=PANEL_TITLE_FG)
        else:
            self.lower_panel()
            self._z_btn.config(fg="#2a4a6a")

    # =========================================================================
    # Geometry helpers
    # =========================================================================

    def _get_xywh(self):
        x, y = self._ws.coords(self._item)
        w = int(float(self._ws.itemcget(self._item, "width")))
        h = int(float(self._ws.itemcget(self._item, "height")))
        return int(x), int(y), w, h

    def _get_wh(self):
        _, _, w, h = self._get_xywh()
        return w, h

    def _set_wh(self, w, h):
        self._ws.itemconfigure(self._item,
                               width=max(MIN_PANEL_W, w),
                               height=max(MIN_PANEL_H, h))

    def _set_xy(self, x, y):
        self._ws.coords(self._item, x, y)

    # =========================================================================
    # Drag with snap guides
    # =========================================================================

    def _drag_start(self, event):
        if self._locked_ref[0]: return
        self._drag_x, self._drag_y = event.x_root, event.y_root
        self.raise_panel()

    def _drag_motion(self, event):
        if self._locked_ref[0]: return
        dx = event.x_root - self._drag_x
        dy = event.y_root - self._drag_y
        self._drag_x, self._drag_y = event.x_root, event.y_root
        cx, cy, cw, ch = self._get_xywh()
        self._ws.coords(self._item, cx + dx, cy + dy)
        self._draw_snap_guides()

    def _drag_end(self, event):
        if self._locked_ref[0]: return
        self._apply_snap()
        self._clear_snap_guides()

    def _other_panels(self):
        return {n: p for n, p in self._all_panels.items() if p is not self}

    def _snap_candidates(self):
        sx, sy, sw, sh = self._get_xywh()
        candidates = []

        for name, other in self._other_panels().items():
            ox, oy, ow, oh = other._get_xywh()

            target_y = oy + oh
            if abs(sy - target_y) < SNAP_PX and abs(sx - ox) < SNAP_PX * 3:
                candidates.append({
                    "axis": "h", "guide_pos": target_y,
                    "snap_x": ox, "snap_y": target_y,
                    "match_w": ow, "match_h": None,
                    "label": f"Below  {_PANEL_LABELS.get(name, name)}",
                    "other_name": name, "direction": "below",
                })

            target_y2 = oy - sh
            if abs(sy - target_y2) < SNAP_PX and abs(sx - ox) < SNAP_PX * 3:
                candidates.append({
                    "axis": "h", "guide_pos": oy,
                    "snap_x": ox, "snap_y": target_y2,
                    "match_w": ow, "match_h": None,
                    "label": f"Above  {_PANEL_LABELS.get(name, name)}",
                    "other_name": name, "direction": "above",
                })

            target_x = ox + ow
            if abs(sx - target_x) < SNAP_PX and abs(sy - oy) < SNAP_PX * 3:
                candidates.append({
                    "axis": "v", "guide_pos": target_x,
                    "snap_x": target_x, "snap_y": oy,
                    "match_w": None, "match_h": oh,
                    "label": f"Right of  {_PANEL_LABELS.get(name, name)}",
                    "other_name": name, "direction": "right",
                })

            target_x2 = ox - sw
            if abs(sx - target_x2) < SNAP_PX and abs(sy - oy) < SNAP_PX * 3:
                candidates.append({
                    "axis": "v", "guide_pos": ox,
                    "snap_x": target_x2, "snap_y": oy,
                    "match_w": None, "match_h": oh,
                    "label": f"Left of  {_PANEL_LABELS.get(name, name)}",
                    "other_name": name, "direction": "left",
                })

            if abs(sx - ox) < SNAP_PX and abs(sy - oy) > SNAP_PX:
                candidates.append({
                    "axis": "v", "guide_pos": ox,
                    "snap_x": ox, "snap_y": sy,
                    "match_w": None, "match_h": None,
                    "label": f"Align left  {_PANEL_LABELS.get(name, name)}",
                    "other_name": name, "direction": "align_left",
                })

            if abs(sy - oy) < SNAP_PX and abs(sx - ox) > SNAP_PX:
                candidates.append({
                    "axis": "h", "guide_pos": oy,
                    "snap_x": sx, "snap_y": oy,
                    "match_w": None, "match_h": None,
                    "label": f"Align top  {_PANEL_LABELS.get(name, name)}",
                    "other_name": name, "direction": "align_top",
                })

        return candidates

    def _draw_snap_guides(self):
        self._clear_snap_guides()
        ws_w = self._ws.winfo_width()
        ws_h = self._ws.winfo_height()
        for c in self._snap_candidates():
            if c["axis"] == "h":
                line = self._ws.create_line(
                    0, c["guide_pos"], ws_w, c["guide_pos"],
                    fill=SNAP_COLOR, dash=GUIDE_DASH, width=1, tags="snap_guide")
            else:
                line = self._ws.create_line(
                    c["guide_pos"], 0, c["guide_pos"], ws_h,
                    fill=SNAP_COLOR, dash=GUIDE_DASH, width=1, tags="snap_guide")
            self._guide_lines.append(line)
            self._ws.tag_raise("snap_guide")

    def _clear_snap_guides(self):
        self._ws.delete("snap_guide")
        self._guide_lines.clear()

    def _apply_snap(self):
        candidates = self._snap_candidates()
        if not candidates:
            return
        sx, sy, sw, sh = self._get_xywh()

        def _dist(c):
            return abs(sx - c["snap_x"]) + abs(sy - c["snap_y"])

        best = min(candidates, key=_dist)
        if _dist(best) > SNAP_PX * 2:
            return

        new_x = best["snap_x"]
        new_y = best["snap_y"]
        new_w = best["match_w"] if best["match_w"] is not None else sw
        new_h = best["match_h"] if best["match_h"] is not None else sh

        self._set_xy(new_x, new_y)
        self._set_wh(new_w, new_h)

    # =========================================================================
    # Resize
    # =========================================================================

    def _resize_start(self, event):
        if self._locked_ref[0]: return
        self._drag_x, self._drag_y = event.x_root, event.y_root

    def _resize_both(self, event):
        if self._locked_ref[0]: return
        dx = event.x_root - self._drag_x
        dy = event.y_root - self._drag_y
        self._drag_x, self._drag_y = event.x_root, event.y_root
        cw, ch = self._get_wh()
        self._set_wh(cw + dx, ch + dy)

    def _resize_h(self, event):
        if self._locked_ref[0]: return
        dx = event.x_root - self._drag_x
        self._drag_x = event.x_root
        cw, ch = self._get_wh()
        self._set_wh(cw + dx, ch)

    def _resize_v(self, event):
        if self._locked_ref[0]: return
        dy = event.y_root - self._drag_y
        self._drag_y = event.y_root
        cw, ch = self._get_wh()
        self._set_wh(cw, ch + dy)

    # =========================================================================
    # Context menu
    # =========================================================================

    def _show_context_menu(self, event):
        menu = tk.Menu(self._ws, tearoff=0,
                       bg="#0f1428", fg="#c0d0f0",
                       activebackground="#1e3060",
                       activeforeground="#ffffff",
                       font=("Consolas", 9), bd=1, relief="solid")

        menu.add_command(label="  ⬆  Bring to Front", command=self.raise_panel)
        menu.add_command(label="  ⬇  Send to Back",   command=self.lower_panel)
        menu.add_separator()

        snap_added = False
        for name, other in self._other_panels().items():
            if not other.winfo_ismapped():
                continue
            label = _PANEL_LABELS.get(name, name)
            ox, oy, ow, oh = other._get_xywh()
            sx, sy, sw, sh = self._get_xywh()

            def _snap_below(o=other, ow=ow, oh=oh, ox=ox, oy=oy):
                self._set_xy(ox, oy + oh); self._set_wh(ow, sh)

            def _snap_right(o=other, ow=ow, oh=oh, ox=ox, oy=oy):
                self._set_xy(ox + ow, oy); self._set_wh(sw, oh)

            def _snap_above(o=other, ow=ow, oh=oh, ox=ox, oy=oy):
                self._set_xy(ox, oy - sh); self._set_wh(ow, sh)

            def _snap_left(o=other, ow=ow, oh=oh, ox=ox, oy=oy):
                self._set_xy(ox - sw, oy); self._set_wh(sw, oh)

            if not snap_added:
                menu.add_command(label=f"  ── Snap to: {label} ──",
                                 state="disabled")
                snap_added = True
            else:
                menu.add_separator()
                menu.add_command(label=f"  ── {label} ──", state="disabled")

            menu.add_command(label="      Below  (match width)",  command=_snap_below)
            menu.add_command(label="      Above  (match width)",  command=_snap_above)
            menu.add_command(label="      Right  (match height)", command=_snap_right)
            menu.add_command(label="      Left   (match height)", command=_snap_left)

        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()

    # =========================================================================
    # Public geometry API
    # =========================================================================

    def get_geometry(self) -> dict:
        x, y, w, h = self._get_xywh()
        return {"x": x, "y": y, "w": w, "h": h}

    def set_geometry(self, x: int, y: int, w: int, h: int) -> None:
        self._set_xy(x, y)
        self._set_wh(w, h)

    def show(self):
        self._ws.itemconfigure(self._item, state="normal")
        self.raise_panel()
        if self._on_visibility is not None:
            self._on_visibility(self._name, True)

    def hide(self):
        self._ws.itemconfigure(self._item, state="hidden")
        if self._on_visibility is not None:
            self._on_visibility(self._name, False)


# ── LayoutStore ─────────────────────────────────────────────────────────────

class LayoutStore:
    """
    Persists *multiple* named layout profiles to a single JSON file, instead
    of the old one-file-one-layout scheme.

    On-disk format (cockpit_layouts.json):
        {
          "active_profile": "Default",
          "profiles": {
            "<name>": {
                "geometry": "1400x960+20+20",
                "locked": false,
                "panels": {
                    "<panel_name>": {"x":.., "y":.., "w":.., "h":.., "visible":..},
                    ...
                },
                "camera_angle": {
                    "mode": "manual",   # or "auto"
                    "tilt_deg": 0.0,    # 0=forward, 90=down, 180=back, -=up
                    "pan_deg": 0.0
                }
            },
            ...
          }
        }

    "camera_angle" mirrors the "📐 Camera Angle" toolbar dialog's state
    (see DroneCockpitApp._current_snapshot()/_apply_layout()) -- saved and
    restored together with panel geometry so the camera mount angle used
    for detection georeferencing survives a restart the same way the rest
    of the layout does. A profile saved before this field existed simply
    has no "camera_angle" key; _apply_layout() falls back to the
    CAMERA_MOUNT_TILT_DEG/CAMERA_MOUNT_PAN_DEG module constants in that
    case rather than erroring.

    If the new file doesn't exist yet but a legacy single-profile
    cockpit_layout.json does, it is transparently imported as a profile
    named "Default" the first time the app starts — nothing is lost.
    """

    def __init__(self, path: Path, legacy_path: Optional[Path] = None):
        self._path = path
        self._legacy_path = legacy_path
        self._data = self._load()

    # ---- disk I/O ------------------------------------------------------

    def _load(self) -> dict:
        if self._path.exists():
            try:
                with open(self._path) as fh:
                    data = json.load(fh)
                if data.get("profiles"):
                    return data
            except Exception as e:
                print(f"[LayoutStore] load failed: {e}")

        if self._legacy_path and self._legacy_path.exists():
            try:
                with open(self._legacy_path) as fh:
                    legacy = json.load(fh)
                _vlog(f"[LayoutStore] migrating legacy layout file "
                      f"→ profile '{_DEFAULT_PROFILE_NAME}'")
                return {
                    "active_profile": _DEFAULT_PROFILE_NAME,
                    "profiles": {_DEFAULT_PROFILE_NAME: legacy},
                }
            except Exception as e:
                print(f"[LayoutStore] legacy migration failed: {e}")

        return {"active_profile": _DEFAULT_PROFILE_NAME, "profiles": {}}

    def save(self) -> None:
        try:
            with open(self._path, "w") as fh:
                json.dump(self._data, fh, indent=2)
            _vlog(f"[LayoutStore] saved → {self._path}")
        except Exception as e:
            print(f"[LayoutStore] save failed: {e}")

    # ---- profile access --------------------------------------------------

    def has_any(self) -> bool:
        return bool(self._data.get("profiles"))

    def profile_names(self) -> list:
        return sorted(self._data.get("profiles", {}).keys())

    def get_profile(self, name: str) -> Optional[dict]:
        return self._data.get("profiles", {}).get(name)

    def active_name(self) -> str:
        return self._data.get("active_profile", _DEFAULT_PROFILE_NAME)

    def set_active(self, name: str) -> None:
        self._data["active_profile"] = name
        self.save()

    def upsert_profile(self, name: str, snapshot: dict, make_active: bool = True) -> None:
        self._data.setdefault("profiles", {})[name] = snapshot
        if make_active:
            self._data["active_profile"] = name
        self.save()

    def delete_profile(self, name: str) -> None:
        self._data.get("profiles", {}).pop(name, None)
        if self._data.get("active_profile") == name:
            remaining = self.profile_names()
            self._data["active_profile"] = remaining[0] if remaining else _DEFAULT_PROFILE_NAME
        self.save()

    def rename_profile(self, old: str, new: str) -> None:
        profiles = self._data.get("profiles", {})
        if old not in profiles or old == new:
            return
        profiles[new] = profiles.pop(old)
        if self._data.get("active_profile") == old:
            self._data["active_profile"] = new
        self.save()


# ── Layout preview + manager dialog ─────────────────────────────────────────

class _LayoutPreviewCanvas(tk.Canvas):
    """Read-only rendition of a saved profile's panel geometry.

    Sizes and re-renders itself relative to whatever space it's actually
    given, so growing the Layout Profiles window (or its preview pane)
    enlarges the preview — and its labels — instead of leaving it pinned
    at a small fixed pixel size that becomes unreadable.

    Only panels that are *visible* in the profile being previewed are
    drawn. A panel that's toggled off still has a saved rect in the
    profile (often a stale one from before it was hidden, or its original
    default position), but since it isn't shown in the real workspace, it
    has no business appearing in the preview either -- and letting it into
    the bounds calculation used to skew/shrink the preview to fit panels
    that were never actually visible.
    """

    _MIN_W, _MIN_H = 260, 170  # floor size so it's never unusably tiny

    def __init__(self, master, **kwargs):
        super().__init__(master, width=self._MIN_W, height=self._MIN_H,
                         bg="#0a0a14", highlightthickness=1,
                         highlightbackground="#2a3a5a", **kwargs)
        self._last_profile: Optional[dict] = None
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event=None) -> None:
        # Re-render at the new size whenever the canvas is stretched.
        self.render(self._last_profile)

    def render(self, profile: Optional[dict]) -> None:
        self._last_profile = profile
        self.delete("all")

        pw = max(self.winfo_width(), self._MIN_W)
        ph = max(self.winfo_height(), self._MIN_H)

        if not profile:
            self.create_text(pw // 2, ph // 2,
                             text="No profile selected",
                             fill="#445566", font=("Consolas", 9))
            return

        all_panels = profile.get("panels", {})

        # Filter down to only the panels actually visible in this profile
        # *before* anything else (bounds calc, drawing) touches them --
        # this is the fix. Previously every panel that had ever been part
        # of the profile was drawn (dimmed if hidden) using whatever rect
        # it last had, which routinely didn't reflect the real workspace
        # and also skewed the scale factor used to fit the preview.
        panels = {n: g for n, g in all_panels.items() if g.get("visible", True)}

        if not all_panels:
            self.create_text(pw // 2, ph // 2,
                             text="(empty layout)",
                             fill="#445566", font=("Consolas", 9))
            return

        if not panels:
            self.create_text(pw // 2, ph // 2,
                             text="(no visible panels)",
                             fill="#445566", font=("Consolas", 9))
            return

        # Scale factor relative to the floor size — used to grow label
        # font sizes proportionally as the canvas gets bigger, instead of
        # keeping text fixed at a size that's only readable when small.
        size_ratio = min(pw / self._MIN_W, ph / self._MIN_H)
        label_font_px = max(6, min(13, round(6 * size_ratio)))
        lock_font_px  = max(7, min(12, round(7 * size_ratio)))

        max_x = max(p["x"] + p["w"] for p in panels.values())
        max_y = max(p["y"] + p["h"] for p in panels.values())
        margin = max(10, round(10 * size_ratio))
        scale = min(
            (pw - 2 * margin) / max(max_x, 1),
            (ph - 2 * margin) / max(max_y, 1),
        )

        for name, geo in panels.items():
            x0 = margin + geo["x"] * scale
            y0 = margin + geo["y"] * scale
            x1 = x0 + geo["w"] * scale
            y1 = y0 + geo["h"] * scale
            self.create_rectangle(x0, y0, x1, y1, fill="#16213e", outline="#00d4ff")
            label = _PANEL_LABELS.get(name, name)
            if (x1 - x0) > 20 and (y1 - y0) > 10:
                self.create_text(
                    (x0 + x1) / 2, (y0 + y1) / 2,
                    text=label, fill="#5588aa",
                    font=("Consolas", label_font_px), width=max(10, x1 - x0 - 4),
                )

        lock_txt = "🔒 locked" if profile.get("locked") else "🔓 unlocked"
        self.create_text(margin, ph - margin // 2 - lock_font_px,
                         anchor="w",
                         text=lock_txt, fill="#5a6a8a", font=("Consolas", lock_font_px))


class LayoutManagerDialog(tk.Toplevel):
    """
    Lists every saved layout profile with a live preview, plus controls to
    load / save / rename / delete profiles. This is the "preview before you
    commit" surface — selecting a profile in the list only updates the
    preview; nothing is applied to the real workspace until "Load" is
    pressed.
    """

    def __init__(self, master, store: LayoutStore, on_load, on_save_current_as, on_overwrite):
        super().__init__(master, bg="#0f0f1a")
        self.title("Layout Profiles")
        self.transient(master)

        self._store = store
        self._on_load = on_load
        self._on_save_current_as = on_save_current_as
        self._on_overwrite = on_overwrite

        # Build the widgets *before* fixing the window's size. Sizing the
        # Toplevel first (e.g. self.geometry("540x400") right after
        # creation, followed by resizable(False, False)) locks the window
        # to whatever the WM guessed at creation time — before the
        # listbox, preview canvas, and action buttons had actually been
        # packed. That's why the dialog could render clipped down to just
        # the listbox, with the preview and every load/save/rename/delete
        # control invisible off to the right. Building first, then forcing
        # an idle-task pass, ensures every widget's real requested size is
        # known before the window is locked to a fixed size.
        self._build_ui()
        self.update_idletasks()
        self.minsize(560, 420)
        self.geometry("620x440")
        self.resizable(True, True)

        self._refresh_list(select=self._store.active_name())

        self.grab_set()

    # ---- UI construction ---------------------------------------------------

    def _build_ui(self):
        left = tk.Frame(self, bg="#0f0f1a")
        left.pack(side="left", fill="y", padx=(10, 6), pady=10)

        tk.Label(left, text="Saved Layouts", bg="#0f0f1a", fg="#00d4ff",
                 font=("Consolas", 10, "bold")).pack(anchor="w")

        self._listbox = tk.Listbox(
            left, width=26, height=15,
            bg="#0a0a14", fg="#c0d0f0",
            selectbackground="#1e3060", selectforeground="#ffffff",
            highlightthickness=1, highlightbackground="#2a3a5a",
            font=("Consolas", 9), activestyle="none",
        )
        self._listbox.pack(fill="y")
        self._listbox.bind("<<ListboxSelect>>", self._on_select)
        self._listbox.bind("<Double-Button-1>", lambda e: self._load_selected())
        self._listbox.bind("<Delete>", lambda e: self._delete_selected())
        self._listbox.bind("<ButtonPress-3>", self._show_row_context_menu)

        right = tk.Frame(self, bg="#0f0f1a")
        right.pack(side="left", fill="both", expand=True, padx=(6, 10), pady=10)

        self._name_lbl = tk.Label(right, text="—", bg="#0f0f1a", fg="#ffffff",
                                  font=("Consolas", 10, "bold"))
        self._name_lbl.pack(anchor="w")

        # Everything below is packed to the *bottom* first, in the order it
        # should appear (bottom-most first), so each reserves its own fixed
        # slice of space. The preview canvas is packed last with
        # fill="both", expand=True — it then claims whatever vertical space
        # is left between the name label and these, so growing the dialog
        # actually grows the preview (and, via _LayoutPreviewCanvas, its
        # text) instead of leaving it pinned at a small fixed size.

        tk.Button(right, text="Close", command=self.destroy,
                 font=("Consolas", 9), bg="#1a1a2e", fg="#8899aa",
                 relief="raised", bd=1, cursor="hand2",
                 activebackground="#252540").pack(side="bottom", anchor="e", pady=(10, 0))

        self._mkbtn(right, "💾  Save Current Layout As New Profile…",
                    self._save_current_as).pack(side="bottom", fill="x")

        sep = tk.Frame(right, bg="#2a3a5a", height=1)
        sep.pack(side="bottom", fill="x", pady=10)

        btn_row2 = tk.Frame(right, bg="#0f0f1a")
        btn_row2.pack(side="bottom", fill="x", pady=(0, 4))
        self._mkbtn(btn_row2, "Rename…", self._rename_selected).pack(side="left", padx=(0, 6))
        self._mkbtn(btn_row2, "Delete", self._delete_selected, "#ff5566").pack(side="left", padx=(0, 6))

        btn_row1 = tk.Frame(right, bg="#0f0f1a")
        btn_row1.pack(side="bottom", fill="x", pady=(6, 4))
        self._mkbtn(btn_row1, "▶  Load", self._load_selected, "#00d4ff").pack(side="left", padx=(0, 6))
        self._mkbtn(btn_row1, "Overwrite w/ Current", self._overwrite_selected).pack(side="left", padx=(0, 6))

        self._preview = _LayoutPreviewCanvas(right)
        self._preview.pack(side="top", fill="both", expand=True, pady=(6, 6))

    def _mkbtn(self, parent, text, command, fg="#a0b8d8"):
        return tk.Button(parent, text=text, command=command,
                         font=("Consolas", 9), bg="#162040", fg=fg,
                         activebackground="#1e3060", activeforeground="#ffffff",
                         relief="raised", bd=1, padx=8, pady=3, cursor="hand2")

    # ---- data plumbing -------------------------------------------------

    def _refresh_list(self, select: Optional[str] = None):
        self._listbox.delete(0, "end")
        names = self._store.profile_names()
        active = self._store.active_name()
        for n in names:
            label = f"●  {n}" if n == active else f"    {n}"
            self._listbox.insert("end", label)

        if select and select in names:
            idx = names.index(select)
        elif names:
            idx = 0
        else:
            idx = None

        if idx is not None:
            self._listbox.selection_set(idx)
            self._listbox.see(idx)
            self._show_preview(names[idx])
        else:
            self._show_preview(None)

    def _selected_name(self) -> Optional[str]:
        sel = self._listbox.curselection()
        if not sel:
            return None
        return self._store.profile_names()[sel[0]]

    def _show_preview(self, name: Optional[str]):
        self._name_lbl.config(text=name or "—")
        self._preview.render(self._store.get_profile(name) if name else None)

    def _on_select(self, event=None):
        self._show_preview(self._selected_name())

    def _show_row_context_menu(self, event):
        """Right-click menu on a listbox row: select it under the cursor
        first, then offer the same actions as the buttons on the right."""
        idx = self._listbox.nearest(event.y)
        if idx < 0 or idx >= self._listbox.size():
            return
        self._listbox.selection_clear(0, "end")
        self._listbox.selection_set(idx)
        self._show_preview(self._selected_name())

        menu = tk.Menu(self, tearoff=0,
                       bg="#0f1428", fg="#c0d0f0",
                       activebackground="#1e3060", activeforeground="#ffffff",
                       font=("Consolas", 9), bd=1, relief="solid")
        menu.add_command(label="  ▶  Load", command=self._load_selected)
        menu.add_command(label="  Overwrite w/ Current", command=self._overwrite_selected)
        menu.add_separator()
        menu.add_command(label="  Rename…", command=self._rename_selected)
        menu.add_command(label="  Delete", command=self._delete_selected)
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()

    # ---- actions ---------------------------------------------------------

    def _load_selected(self):
        name = self._selected_name()
        if not name:
            return
        self._on_load(name)
        self._refresh_list(select=name)

    def _overwrite_selected(self):
        name = self._selected_name()
        if not name:
            return
        if not messagebox.askyesno(
            "Overwrite Layout",
            f"Overwrite saved profile '{name}' with the current on-screen layout?",
            parent=self,
        ):
            return
        self._on_overwrite(name)
        self._refresh_list(select=name)

    def _rename_selected(self):
        name = self._selected_name()
        if not name:
            return
        new_name = simpledialog.askstring(
            "Rename Layout", "New name:", initialvalue=name, parent=self)
        if not new_name or new_name == name:
            return
        if new_name in self._store.profile_names():
            messagebox.showerror("Name in use",
                                 f"A profile named '{new_name}' already exists.",
                                 parent=self)
            return
        self._store.rename_profile(name, new_name)
        self._refresh_list(select=new_name)

    def _delete_selected(self):
        name = self._selected_name()
        if not name:
            return
        if len(self._store.profile_names()) <= 1:
            messagebox.showwarning("Can't Delete",
                                   "At least one layout profile must remain.",
                                   parent=self)
            return
        if not messagebox.askyesno("Delete Layout",
                                   f"Delete saved profile '{name}'? This cannot be undone.",
                                   parent=self):
            return
        self._store.delete_profile(name)
        self._refresh_list()

    def _save_current_as(self):
        new_name = simpledialog.askstring("Save Layout", "Profile name:", parent=self)
        if not new_name:
            return
        if new_name in self._store.profile_names():
            if not messagebox.askyesno("Overwrite Existing?",
                                       f"'{new_name}' already exists. Overwrite it?",
                                       parent=self):
                return
        self._on_save_current_as(new_name)
        self._refresh_list(select=new_name)


# ── Application ───────────────────────────────────────────────────────────────

class DroneCockpitApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Drone Control Ground Station")
        self.root.configure(bg="#0d0d1a")
        self.root.resizable(True, True)

        # ── Camera mount angle (manual toolbar control) ──────────────────────
        # Source of truth for the current mode/tilt/pan while the app is
        # running, live-editable from the "📐 Camera Angle" toolbar button.
        # These three are just the IN-MEMORY startup values -- the real
        # source of truth on disk is the active layout profile's
        # "camera_angle" entry (see LayoutStore's on-disk format and
        # _apply_layout() below), which is what __init__ actually loads
        # into these fields a few lines down, via _apply_layout(). The
        # CAMERA_MOUNT_TILT_DEG/CAMERA_MOUNT_PAN_DEG module constants only
        # matter as a fallback for a profile with no saved camera_angle
        # yet (e.g. the very first run). Deliberately plain floats behind
        # a lock, NOT a Tk StringVar/DoubleVar: _get_detection_telemetry()
        # (which reads this via _get_camera_mount_angle()) runs on
        # DetectionLink's own C++ worker thread, not the Tk thread --
        # touching a Tk variable from there is not safe. The dialog's Tk
        # widgets update this lock-protected state from their callbacks
        # (Tk thread); this getter reads it back (whichever thread calls
        # it).
        self._camera_angle_lock = threading.Lock()
        # Handle to the single open "Camera Angle" Toplevel, or None when
        # closed -- see _show_camera_angle_dialog() for why this exists.
        self._camera_angle_win = None
        # "manual" or "auto" -- overwritten below by _apply_layout() with
        # whatever was last saved, if anything was. Genuinely should
        # default to "auto" once the SimpleBGC gimbal is electrically
        # wired and its IMU-based attitude readback exists -- there is no
        # such readback yet (see _get_camera_mount_angle()'s TODO below),
        # so "auto" would silently do nothing useful right now (it still
        # falls back to whatever the Manual tilt/pan were last set to).
        # "manual" is the correct fallback for THIS moment, not the
        # long-term intent -- flip CAMERA_MOUNT_* mode handling over to
        # defaulting "auto" once real feedback is wired in.
        self._camera_angle_mode = "manual"
        self._camera_tilt_deg = CAMERA_MOUNT_TILT_DEG
        self._camera_pan_deg = CAMERA_MOUNT_PAN_DEG

        self.hub = DroneBackend.DroneLink()
        self._update_job = None

        # ── ELRS radio telemetry (RadioMaster Pocket, USB-VCP Telem Mirror) ──
        # Second, independent telemetry source next to the USB cable. It
        # auto-detects the Pocket and reconnects on its own after an
        # unplug; TelemetryWorker picks USB or ELRS per poll. None if this
        # DroneBackend.pyd predates CrsfLink (the cockpit then runs USB-only).
        self.radio = DroneBackend.CrsfLink() if hasattr(DroneBackend, "CrsfLink") else None
        if self.radio is None:
            print("[CrsfLink] not in this DroneBackend build -- radio telemetry disabled")

        # USB auto-connect runs on a background thread (auto_detect_fc sends
        # an MSP probe to each port, which takes a few hundred ms per port --
        # too long for the Tk thread now that the UI must stay live while
        # flying on the radio link alone). Result is handed back through
        # this attribute and picked up by _update_loop on the Tk thread.
        self._usb_probe_running = False
        self._usb_probe_result = None      # (port, ok) from the last probe
        self._usb_retry_job = None
        self._usb_port = None
        self._usb_unhealthy_since = None   # monotonic time the cable link went bad

        # Yaw trim is now owned here on the Tk side; the worker gets a copy
        # via set_yaw_trim() whenever it changes.
        self._yaw_trim = 0.0

        self._locked_ref  = [False]
        self._vis_vars: dict[str, tk.BooleanVar] = {}
        self._panels:   dict[str, DraggablePanel] = {}

        # Tracks whether the detection/map Toplevel is currently open --
        # separate from _vis_vars because DetectionMapWidget is
        # deliberately not a DraggablePanel (see its module docstring),
        # but it still needs a checkbutton-style entry in the Panels ▾
        # menu so it's discoverable the same way the docked panels are.
        self._detection_vis_var = tk.BooleanVar(value=False)

        # Per-widget frame counters for throttling
        self._frame_counters: dict[str, int] = {k: 0 for k in _THROTTLE}

        # widget -> _Tooltip, so dynamic buttons (e.g. the lock toggle) can
        # update their hover text at runtime
        self._tooltips: dict[tk.Widget, _Tooltip] = {}

        # ── Multi-profile layout persistence ─────────────────────────────────
        self._layout_store = LayoutStore(_LAYOUTS_FILE, _LEGACY_LAYOUT_FILE)
        if not self._layout_store.has_any():
            self._layout_store.upsert_profile(_DEFAULT_PROFILE_NAME, self._default_snapshot())

        self._setup_ui()
        self._apply_layout(self._layout_store.get_profile(self._layout_store.active_name())
                            or self._default_snapshot())
        self._refresh_active_layout_label()

        # ── Background telemetry worker ────────────────────────────────────
        # Starts after UI is built so widgets exist before first frame arrives.
        self._worker = TelemetryWorker(self.hub, radio=self.radio)
        self._worker.set_yaw_trim(self._yaw_trim)

        # ── Background video (FPV) worker ────────────────────────────────────
        # Deliberately independent of the telemetry link/worker: the FPV
        # capture dongle is a different USB device with its own lifecycle,
        # so it gets its own connect/retry loop rather than being gated on
        # (or gating) the flight-controller serial connection.
        self.video_link = DroneBackend.VideoLink()
        self._video_worker = VideoWorker(self.video_link, poll_hz=VIDEO_POLL_HZ)
        self._video_worker.start()
        self._video_connect_job = None
        self._start_video_autoconnect()

        # Hand the FPV panel's native window handle to VideoLink so the C++
        # capture thread can start painting frames directly into it. This
        # is safe to do before a capture device is actually connected --
        # VideoLink simply won't paint anything into the window until
        # connect()/connect_auto() succeeds and frames start arriving.
        self.fpv_view.attach(self.video_link)

        # Wire the pilot-facing software OSD's telemetry source. Uses
        # _get_osd_telemetry() (below), a dedicated trampoline -- NOT
        # _get_detection_telemetry() -- because the two have different
        # validity/altitude semantics (see _get_osd_telemetry()'s
        # docstring). Without this call, VideoLink::drawOsdOverlay()
        # bails out immediately (no telemetryProvider_ set at all), so the
        # OSD never draws regardless of what's enabled/saved in the
        # settings panel.
        self.video_link.set_telemetry_provider(self._get_osd_telemetry)

        # The native FPV render window sits entirely outside Tk's widget
        # tree, so nothing about it is touched by raise_panel()/lower_panel()
        # or show()/hide() on the "fpv" DraggablePanel by default. Push it
        # to the bottom of the OS Z-order right away -- it'll only get
        # raised again if/when the FPV panel itself is brought to front
        # (see _on_panel_zorder, wired into every DraggablePanel via the
        # on_zorder/on_visibility callbacks passed in _panel() below).
        try:
            self.video_link.lower_window()
        except Exception:
            pass

        # ── Background object-detection worker ───────────────────────────────
        # DetectionLink runs entirely on its own C++ thread -- independent of
        # both the 100 Hz MSP telemetry loop and the FPV capture/paint
        # thread, same "own thread, own lifecycle" pattern as VideoLink. It
        # pulls frames straight from VideoLink in C++ (pixel data never
        # crosses into Python here, same rule the FPV path follows) and gets
        # telemetry via the small pybind11 trampoline below
        # (_get_detection_telemetry), which is safe to call from a
        # non-Python-owned thread because DroneLink.get_latest_state() is
        # already a thread-safe, mutex-protected snapshot.
        self.detection_link = DroneBackend.DetectionLink()
        self.detection_link.set_video_link_source(self.video_link)
        self.detection_link.set_telemetry_provider(self._get_detection_telemetry)
        self.detection_link.set_model_path(DETECTION_MODEL_PATH)

        # set_input_size() is only present in DroneBackend once
        # Bindings.cpp has been recompiled after the binding was added --
        # editing Bindings.cpp/Detectionlink.cpp source does NOT change
        # the already-built DroneBackend.pyd Python actually imports.
        # `import DroneBackend` failing here would take the *entire*
        # cockpit down over a detection-only config mismatch, so warn
        # loudly and keep going instead -- DetectionLink just falls back
        # to its compiled-in default inputSize_ (640), which only matters
        # if that also doesn't match your .onnx export (see
        # DETECTION_INPUT_SIZE / Detectionlink.h's setInputSize() docs).
        if hasattr(self.detection_link, "set_input_size"):
            self.detection_link.set_input_size(DETECTION_INPUT_SIZE)
        else:
            print(
                "[DetectionLink] WARNING: this build of DroneBackend has no "
                "set_input_size() -- it was compiled before that binding "
                "was added to Bindings.cpp. Rebuild the DroneBackend "
                "extension (.pyd) from the current source and replace the "
                "one this app is importing. Continuing with whatever "
                f"default input size is compiled in (see inputSize_ in "
                f"Detectionlink.h) -- detection will misbehave if that "
                f"doesn't match the {DETECTION_INPUT_SIZE}x{DETECTION_INPUT_SIZE} "
                "the current model was exported at."
            )

        self.detection_link.set_screenshot_dir(DETECTION_SCREENSHOT_DIR)
        self.detection_link.set_detection_interval_ms(DETECTION_INTERVAL_MS)

        # ── Georeferencing config -- was silently missing before ────────
        # Without these two calls, DetectionLink ran on its compiled-in
        # header defaults: horizontalFovDeg_=90 (Detectionlink.h) and NO
        # known object widths at all. On this rig specifically that meant
        # (a) every ray-angle/bearing calculation was wrong by however far
        # 90 deg differs from the camera's real FOV, and (b) object-size
        # ranging (rangeByObjectSize() in Detectionlink.cpp) was never
        # available, leaving ONLY ground-plane ranging -- which itself
        # needs a ray steeper than ~7 deg below horizontal
        # (minGroundRayComponent_, see setMinGroundRayComponent() in the
        # header) to be trusted. The camera's SimpleBGC gimbal isn't
        # electrically wired yet (see CAMERA_MOUNT_PAN_DEG/
        # CAMERA_MOUNT_TILT_DEG and _get_detection_telemetry() above), so
        # unless you've physically angled the camera down for this
        # flight, that 7-degree bar is rarely cleared in level forward
        # flight -- most detections likely never got a lat/lon at all.
        # Both gaps are closed here.
        if hasattr(self.detection_link, "set_horizontal_fov_deg"):
            # E5-FPV night-vision module datasheet: FOV: 100 deg. Update
            # this constant (see DETECTION_CAMERA_FOV_DEG above) if the
            # camera is ever swapped for a different model/lens.
            self.detection_link.set_horizontal_fov_deg(DETECTION_CAMERA_FOV_DEG)
        else:
            print(
                "[DetectionLink] WARNING: this build of DroneBackend has no "
                "set_horizontal_fov_deg() -- rebuild DroneBackend from "
                "current source to get it. Continuing on the compiled-in "
                "default of 90 deg, which does NOT match this camera's "
                f"real {DETECTION_CAMERA_FOV_DEG} deg FOV -- every "
                "georeferenced bearing will be off until this is rebuilt."
            )

        if hasattr(self.detection_link, "set_known_object_width"):
            # Enables rangeByObjectSize() as a fallback whenever
            # ground-plane ranging can't be trusted (shallow ray -- the
            # common case whenever the camera is mounted forward-level
            # rather than angled down -- see CAMERA_MOUNT_TILT_DEG
            # above). Values are face-on width in meters -- see
            # setKnownObjectWidth()'s doc comment in Detectionlink.h for
            # why width (not length/height) is what the bbox actually
            # measures at most viewing angles.
            #
            # IMPORTANT: these class-name strings MUST exactly match the
            # names in your model's sibling .names file (one class name
            # per line, same stem as DETECTION_MODEL_PATH) -- a mismatch
            # doesn't error, it just silently never matches, and that
            # class quietly falls back to ground-plane-only again. Open
            # the .names file next to yolo26m_main.onnx and confirm the
            # exact strings before relying on this; adjust the dict below
            # to match if your classes are named differently (e.g. "car"/
            # "truck" instead of a single "vehicle" class).
            for class_name, width_m in DETECTION_KNOWN_OBJECT_WIDTHS_M.items():
                self.detection_link.set_known_object_width(class_name, width_m)
        else:
            print(
                "[DetectionLink] WARNING: this build of DroneBackend has no "
                "set_known_object_width() -- rebuild DroneBackend from "
                "current source to get it. Object-size ranging will stay "
                "unavailable; only ground-plane ranging will ever produce "
                "a lat/lon, and only when the ray is steep enough."
            )

        # Bearings-only triangulation (rangeByTriangulation() in
        # Detectionlink.cpp) -- a third ranging method that needs neither
        # a steep ray nor an assumed object size, using instead a
        # tracked object's accumulated bearing history plus however far
        # the drone has moved between sightings. These two thresholds
        # guard against trusting a fix built from too little real
        # parallax (see setTriangulationMinBaselineM/
        # setTriangulationMinBearingSpreadDeg in Detectionlink.h) --
        # overriding them here is optional, the compiled-in defaults
        # (5.0m / 5.0deg) match these constants already, but they're
        # exposed as named constants above so they're easy to tune
        # without a rebuild once you've flown a few missions.
        if hasattr(self.detection_link, "set_triangulation_min_baseline_m"):
            self.detection_link.set_triangulation_min_baseline_m(
                DETECTION_TRIANGULATION_MIN_BASELINE_M)
        if hasattr(self.detection_link, "set_triangulation_min_bearing_spread_deg"):
            self.detection_link.set_triangulation_min_bearing_spread_deg(
                DETECTION_TRIANGULATION_MIN_BEARING_SPREAD_DEG)
        if not hasattr(self.detection_link, "set_triangulation_min_baseline_m"):
            print(
                "[DetectionLink] WARNING: this build of DroneBackend has no "
                "triangulation bindings -- rebuild DroneBackend from "
                "current source to get bearings-only triangulated ranging "
                "(range_method=='triangulated'). Ground-plane/object-size "
                "ranging still work as before without it."
            )

        # Same defensive hasattr pattern as set_input_size() above --
        # set_use_cuda()/is_using_cuda() are also newer bindings that a
        # stale .pyd might not have yet.
        if hasattr(self.detection_link, "set_use_cuda"):
            self.detection_link.set_use_cuda(DETECTION_USE_CUDA)
        elif DETECTION_USE_CUDA:
            print(
                "[DetectionLink] WARNING: this build of DroneBackend has no "
                "set_use_cuda() -- rebuild DroneBackend from current source "
                "to get it. Continuing on CPU."
            )

        # DetectionWorker only ever does cheap, mutex-protected reads off
        # DetectionLink (status counters + incremental record pulls) --
        # never frames, never inference, never disk I/O -- so it's safe to
        # start polling immediately regardless of whether the model
        # actually loaded below.
        self._detection_worker = DetectionWorker(self.detection_link, poll_hz=DETECTION_POLL_HZ)
        self._detection_worker.start()

        # DetectionMapWidget is intentionally NOT created here -- like the
        # rest of this app's optional panels it's opened on demand (see
        # _open_detection_window), so a pilot who never looks at it never
        # pays for a Toplevel + Treeview that's sitting there unused.
        self._detection_map_window: Optional[DetectionMapWidget] = None
        self._detection_pump_job = None

        # Tracked (not just printed) so the detection window itself can
        # show *why* the pin list is empty -- an engine that never started
        # and an engine that's running but hasn't seen anything yet look
        # identical from an empty Treeview alone. See
        # DetectionMapWidget.set_engine_status() and
        # _retry_detection_engine() below.
        self._detection_engine_running = False
        self._detection_engine_detail = ""
        self._retry_detection_engine(log_on_failure=True)

        # Telemetry no longer waits for the USB cable: the worker and the Tk
        # pump start immediately so instruments come alive from whichever
        # source (USB or ELRS) is available first.
        if self.radio is not None:
            self.radio.start_auto()      # non-blocking, auto-reconnects forever
        self._worker.start()
        self._schedule_update()
        self._auto_connect()             # USB probe on a background thread

    # =========================================================================
    # Native FPV window sync — keeps VideoLink's off-Tk render HWND's
    # OS-level Z-order/visibility matching whatever the Tk panels are doing.
    # =========================================================================

    def _on_panel_zorder(self, name: str, raised: bool) -> None:
        """
        Fired by any DraggablePanel's raise_panel()/lower_panel(). Tk's
        lift()/lower() only reorders Tk's own widgets, so without this the
        native FPV HWND stays pinned wherever Windows put it when it was
        created (typically the top of the parent's Z-order) regardless of
        which Tk panel the user actually brought to front -- which is what
        made the live video appear to bleed over unrelated panels.

        Rule: the native window should be at the bottom of the stack
        except while the FPV panel itself is the one on top.
        """
        video_link = getattr(self, "video_link", None)
        if video_link is None:
            return
        try:
            if name == "fpv":
                video_link.raise_window() if raised else video_link.lower_window()
            elif raised:
                # Some other panel just came to front -- make sure the
                # native window isn't sitting above it.
                video_link.lower_window()
        except Exception as e:
            print(f"[VideoLink] window z-order sync failed: {e}")

    def _on_panel_visibility(self, name: str, visible: bool) -> None:
        """
        Fired by DraggablePanel.show()/hide(). DraggablePanel.hide() only
        hides the *canvas item* (a Tk-level concept) -- it has no effect on
        a foreign HWND embedded inside it, so without this the native FPV
        window kept rendering even while its panel was toggled off.
        """
        if name != "fpv":
            return
        video_link = getattr(self, "video_link", None)
        if video_link is None:
            return
        try:
            video_link.show_window() if visible else video_link.hide_window()
        except Exception as e:
            print(f"[VideoLink] window visibility sync failed: {e}")

    # =========================================================================
    # FPV video connection
    # =========================================================================

    def _start_video_autoconnect(self) -> None:
        """
        VideoLink.connect_auto() is blocking (it opens and probes candidate
        devices), so it always runs on a background Python thread — never
        on the Tk thread. If no device is found yet, this reschedules
        itself so a dongle plugged in mid-session still gets picked up.
        """
        def _probe():
            ok = False
            try:
                ok = self.video_link.connect_auto()
            except Exception as e:
                print(f"[VideoLink] connect_auto() error: {e}")
            if not ok:
                # Retry later from the Tk thread (after() is not thread-safe
                # to call directly from here in all Tk builds, so hop back
                # via root.after with a zero delay is avoided — instead we
                # just schedule the same probe again from this same thread).
                self.root.after(VIDEO_PROBE_RETRY_MS, self._start_video_autoconnect)

        threading.Thread(target=_probe, daemon=True).start()

    # =========================================================================
    # Object detection (DetectionLink / DetectionWorker / DetectionMapWidget)
    # =========================================================================

    def _get_camera_mount_angle(self) -> tuple:
        """
        Thread-safe. Called from _get_detection_telemetry() (DetectionLink's
        C++ worker thread) AND from the Tk-thread dialog's live status
        label -- lock-protected plain state, no Tk widgets touched here.
        Returns (tilt_deg, pan_deg).

        TODO once the SimpleBGC gimbal is electrically wired: in "auto"
        mode, read its real IMU-based pan/tilt readback here instead of
        falling back to the manual values. There is no such readback yet,
        so "auto" is currently indistinguishable from "manual" except for
        the one-time warning shown when switching into it (see
        _set_camera_angle_mode()) -- it's wired up this way now so turning
        on real auto-tracking later is a one-line change in this function,
        not a UI rework.
        """
        with self._camera_angle_lock:
            return self._camera_tilt_deg, self._camera_pan_deg

    def _set_camera_mount_angle(self, tilt_deg: float = None, pan_deg: float = None) -> None:
        """Thread-safe setter, called from the Tk-thread dialog only."""
        with self._camera_angle_lock:
            if tilt_deg is not None:
                self._camera_tilt_deg = tilt_deg
            if pan_deg is not None:
                self._camera_pan_deg = pan_deg

    def _set_camera_angle_mode(self, mode: str) -> None:
        """Thread-safe mode switch, called from the Tk-thread dialog only."""
        if mode == "auto":
            messagebox.showinfo(
                "Camera Angle — Auto",
                "Auto mode has no real gimbal feedback to read yet -- the "
                "SimpleBGC gimbal isn't electrically wired (missing "
                "step-down modules). Auto will keep using the last Manual "
                "tilt/pan values until real stepper-position readback is "
                "wired into DroneLink. Switch back to Manual any time to "
                "set them directly.",
            )
        with self._camera_angle_lock:
            self._camera_angle_mode = mode
        self._refresh_camera_angle_label()

    def _refresh_camera_angle_label(self) -> None:
        """Updates the small toolbar status label -- Tk thread only. Safe
        to call any time the mode/tilt/pan changes, including from
        _show_camera_angle_dialog()'s live slider callbacks."""
        tilt_deg, pan_deg = self._get_camera_mount_angle()
        mode = self._camera_angle_mode
        if mode == "auto":
            text = "📐 Auto (unwired)"
        else:
            text = f"📐 Manual: {tilt_deg:+.0f}° tilt, {pan_deg:+.0f}° pan"
        lbl = getattr(self, "_camera_angle_lbl", None)
        if lbl is not None:
            lbl.config(text=text)

    def _show_camera_angle_dialog(self) -> None:
        """
        Toolbar-triggered popup for setting how the FPV camera is
        physically mounted -- see the "Camera mount angle" constants'
        comment near the top of this file for the full rationale. Sliders
        apply live (no separate Apply/OK step) so you can nudge tilt and
        immediately see the toolbar label update to confirm what's being
        fed into georeferencing.

        Singleton dialog: repeated clicks on the toolbar button used to
        spawn a brand new Toplevel every time (nothing ever tracked
        whether one was already open), so mashing the button stacked up
        N duplicate windows, each with its own pair of sliders all
        writing to the same underlying state. Data was never actually
        corrupted (every instance shares self._camera_tilt_deg/_pan_deg
        under the same lock) but it looked broken and was confusing to
        use. Now we keep a handle to the live window and just raise/focus
        it on repeat clicks instead of creating another one.
        """
        existing = getattr(self, "_camera_angle_win", None)
        if existing is not None and existing.winfo_exists():
            existing.deiconify()
            existing.lift()
            existing.focus_force()
            return

        win = tk.Toplevel(self.root)
        self._camera_angle_win = win

        def on_close():
            self._camera_angle_win = None
            # Persist mode/tilt/pan into the active layout profile now,
            # same as any other layout edit -- don't wait for app
            # shutdown (shutdown() also persists as a safety net, but a
            # crash or kill between now and then would otherwise lose
            # whatever was just set here).
            self._persist_active()
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", on_close)
        win.title("Camera Angle")
        win.configure(bg="#0f1428")
        win.resizable(False, False)
        win.transient(self.root)

        pad = dict(padx=14, pady=(10, 0))

        tk.Label(
            win, text="Camera mount angle", fg="#00d4ff", bg="#0f1428",
            font=("Consolas", 11, "bold"),
        ).pack(anchor="w", **pad)
        tk.Label(
            win,
            text="Used for detection georeferencing (DetectionLink).\n"
                 "Auto has no real gimbal feedback yet -- see the info "
                 "shown when you switch to it.",
            fg="#8098c0", bg="#0f1428", font=("Consolas", 8),
            justify="left",
        ).pack(anchor="w", padx=14, pady=(2, 8))

        # ── Mode toggle ──────────────────────────────────────────────
        mode_row = tk.Frame(win, bg="#0f1428")
        mode_row.pack(fill="x", padx=14, pady=(0, 10))

        mode_var = tk.StringVar(value=self._camera_angle_mode)
        interactive_widgets = []   # populated below as tilt/pan scale + presets are built

        def set_manual_widgets_enabled(enabled: bool):
            state = "normal" if enabled else "disabled"
            for w in interactive_widgets:
                w.configure(state=state)

        def apply_mode():
            self._set_camera_angle_mode(mode_var.get())
            set_manual_widgets_enabled(mode_var.get() == "manual")

        tk.Radiobutton(
            mode_row, text="🕹 Manual", variable=mode_var, value="manual",
            command=apply_mode, fg="#c0d0f0", bg="#0f1428",
            selectcolor="#1e3060", activebackground="#0f1428",
            activeforeground="#ffffff", font=("Consolas", 10),
        ).pack(side="left", padx=(0, 16))
        tk.Radiobutton(
            mode_row, text="⟳ Auto", variable=mode_var, value="auto",
            command=apply_mode, fg="#c0d0f0", bg="#0f1428",
            selectcolor="#1e3060", activebackground="#0f1428",
            activeforeground="#ffffff", font=("Consolas", 10),
        ).pack(side="left")

        # ── Manual controls ──────────────────────────────────────────
        manual_frame = tk.Frame(win, bg="#0f1428")
        manual_frame.pack(fill="x", padx=14, pady=(0, 6))

        def on_tilt(v):
            self._set_camera_mount_angle(tilt_deg=float(v))
            self._refresh_camera_angle_label()
            redraw_indicator()

        def on_pan(v):
            self._set_camera_mount_angle(pan_deg=float(v))
            self._refresh_camera_angle_label()
            redraw_indicator()

        tk.Label(
            manual_frame, text="Tilt (0=forward, 90=down, 180=back, -=up)",
            fg="#a0b8d8", bg="#0f1428", font=("Consolas", 8),
        ).pack(anchor="w")
        tilt_scale = tk.Scale(
            manual_frame, from_=-180, to=180, orient="horizontal",
            resolution=1, length=280, command=on_tilt,
            bg="#0f1428", fg="#c0d0f0", troughcolor="#1c2c54",
            highlightthickness=0, activebackground="#00d4ff",
            font=("Consolas", 8),
        )
        tilt_scale.set(self._camera_tilt_deg)
        tilt_scale.pack(fill="x", pady=(0, 6))
        interactive_widgets.append(tilt_scale)

        preset_row = tk.Frame(manual_frame, bg="#0f1428")
        preset_row.pack(fill="x", pady=(0, 10))
        for label, val in (("Up", -90), ("Forward", 0), ("Down", 90), ("Backward", 180)):
            preset_btn = tk.Button(
                preset_row, text=label, command=lambda v=val: tilt_scale.set(v),
                relief="flat", bg="#162040", fg="#a0b8d8",
                activebackground="#1e3060", activeforeground="#ffffff",
                font=("Consolas", 8), padx=6, pady=2, cursor="hand2",
            )
            preset_btn.pack(side="left", padx=(0, 6))
            interactive_widgets.append(preset_btn)

        tk.Label(
            manual_frame, text="Pan (0=centered)",
            fg="#a0b8d8", bg="#0f1428", font=("Consolas", 8),
        ).pack(anchor="w")
        pan_scale = tk.Scale(
            manual_frame, from_=-180, to=180, orient="horizontal",
            resolution=1, length=280, command=on_pan,
            bg="#0f1428", fg="#c0d0f0", troughcolor="#1c2c54",
            highlightthickness=0, activebackground="#00d4ff",
            font=("Consolas", 8),
        )
        pan_scale.set(self._camera_pan_deg)
        pan_scale.pack(fill="x", pady=(0, 10))
        interactive_widgets.append(pan_scale)

        # ── 3D-ish orientation indicator ───────────────────────────────
        # A schematic wireframe camera (box body + lens cone) rotated
        # live by the tilt/pan sliders above, drawn with a simple
        # isometric projection -- no extra dependency, just stdlib
        # `math` and Canvas line/polygon primitives. This is a sanity
        # check while setting the mount angle, not a real 3D renderer.
        # A dashed arrow marks the drone's fixed forward-flight
        # direction so tilt/pan reads as "relative to the nose", which
        # is what matters when interpreting georeferenced detections.
        indicator_frame = tk.Frame(win, bg="#0f1428")
        indicator_frame.pack(fill="x", padx=14, pady=(4, 4))

        _CANVAS_W, _CANVAS_H = 252, 190
        indicator_canvas = tk.Canvas(
            indicator_frame, width=_CANVAS_W, height=_CANVAS_H, bg="#0a0e1e",
            highlightthickness=1, highlightbackground="#1c2c54",
        )
        indicator_canvas.pack()

        tk.Label(
            indicator_frame,
            text="Dashed = drone nose (forward flight)   Solid cone = camera view",
            fg="#5a7098", bg="#0f1428", font=("Consolas", 7),
        ).pack(anchor="w", pady=(3, 0))

        _ISO_COS30 = math.cos(math.radians(30))
        _ISO_SIN30 = math.sin(math.radians(30))
        _CX, _CY, _SCALE = _CANVAS_W // 2, _CANVAS_H // 2 + 6, 46

        def _rotate(pt, tilt_deg, pan_deg):
            """Local camera-frame point (X=right, Y=forward, Z=up) ->
            world frame. Tilt (pitch about X) is applied first -- the
            camera nods up/down on its own mount -- then pan (yaw about
            Z) swings the whole tilted assembly left/right, matching a
            real pan-tilt head. Sign convention matches the slider
            labels: tilt 90=down, -90=up; must mirror
            _get_camera_mount_angle()'s tilt/pan meaning exactly, since
            this is purely a visual and never feeds georeferencing
            itself."""
            x, y, z = pt
            t = math.radians(tilt_deg)
            y, z = y * math.cos(t) + z * math.sin(t), -y * math.sin(t) + z * math.cos(t)
            p = math.radians(pan_deg)
            x, y = x * math.cos(p) - y * math.sin(p), x * math.sin(p) + y * math.cos(p)
            return (x, y, z)

        def _project(pt):
            x, y, z = pt
            sx = (x - y) * _ISO_COS30
            sy = (x + y) * _ISO_SIN30 - z
            return (_CX + sx * _SCALE, _CY - sy * _SCALE)

        # Camera body (box) + lens cone, in the camera's own local frame.
        hw, hd, hh = 0.42, 0.5, 0.34
        _BODY = [
            (-hw, -hd, -hh), (hw, -hd, -hh), (hw, -hd, hh), (-hw, -hd, hh),  # back face
            (-hw,  hd, -hh), (hw,  hd, -hh), (hw,  hd, hh), (-hw,  hd, hh),  # front face
        ]
        _BODY_EDGES = [
            (0, 1), (1, 2), (2, 3), (3, 0),   # back face
            (4, 5), (5, 6), (6, 7), (7, 4),   # front face
            (0, 4), (1, 5), (2, 6), (3, 7),   # connecting edges
        ]
        _LENS_TIP = (0, hd + 0.55, 0)
        _LENS_FRONT_CORNERS = (4, 5, 6, 7)  # front-face corners fan out to the tip

        def redraw_indicator(*_args):
            tilt_deg, pan_deg = self._get_camera_mount_angle()
            indicator_canvas.delete("all")

            # Fixed ground reference + dashed "nose" arrow -- neither
            # rotates; they're the frame the camera orientation is
            # judged against.
            ground = [(-1.3, -1.3, -hh - 0.02), (1.3, -1.3, -hh - 0.02),
                      (1.3, 1.3, -hh - 0.02), (-1.3, 1.3, -hh - 0.02)]
            gpts = [c for p in ground for c in _project(p)]
            indicator_canvas.create_polygon(*gpts, outline="#1c2c54", fill="", width=1)
            nose_from = _project((0, -1.3, -hh - 0.02))
            nose_to = _project((0, 1.55, -hh - 0.02))
            indicator_canvas.create_line(
                *nose_from, *nose_to, fill="#5a7098", width=2,
                dash=(4, 3), arrow="last",
            )

            # Rotated camera body.
            world = [_rotate(p, tilt_deg, pan_deg) for p in _BODY]
            proj = [_project(p) for p in world]
            for a, b in _BODY_EDGES:
                indicator_canvas.create_line(*proj[a], *proj[b], fill="#00d4ff", width=2)

            # Lens cone: front face corners fanning out to a tip point,
            # so the "pointy end" visually reads as where the lens looks.
            tip_proj = _project(_rotate(_LENS_TIP, tilt_deg, pan_deg))
            for idx in _LENS_FRONT_CORNERS:
                indicator_canvas.create_line(*proj[idx], *tip_proj, fill="#ffb020", width=1)
            indicator_canvas.create_oval(
                tip_proj[0] - 3, tip_proj[1] - 3, tip_proj[0] + 3, tip_proj[1] + 3,
                fill="#ffb020", outline="",
            )

        redraw_indicator()

        set_manual_widgets_enabled(self._camera_angle_mode == "manual")

        tk.Button(
            win, text="Close", command=on_close,
            relief="flat", bg="#162040", fg="#a0b8d8",
            activebackground="#1e3060", activeforeground="#ffffff",
            font=("Consolas", 9), padx=10, pady=4, cursor="hand2",
        ).pack(pady=(0, 12))

    def _get_detection_telemetry(self) -> "DroneBackend.TelemetrySnapshot":
        """
        Trampoline passed to DetectionLink.set_telemetry_provider(). Called
        directly from DetectionLink's own C++ worker thread once per
        detection pass -- NOT the Tk thread, and NOT via TelemetryWorker's
        queue (that queue is drained on the Tk thread only, at UI_REFRESH_MS,
        which would make every detection pass wait on Tk's pump for no
        reason). DroneLink.get_latest_state() is already a thread-safe,
        mutex-protected snapshot (see Bindings.cpp), so it's safe to call
        straight from here.

        NOTE: gimbal_pan_deg / gimbal_tilt_deg come from
        _get_camera_mount_angle() below, NOT from live gimbal telemetry --
        the SimpleBGC gimbal hardware is physically installed but not
        electrically wired yet (missing step-down modules), so there's no
        readback to use. That getter's "manual" mode is a live,
        toolbar-editable stand-in (see the "📐 Camera Angle" button) for
        however the camera is actually physically angled right now --
        keep it in sync with reality, there's no sensor to do that for
        you yet. Its "auto" mode is a placeholder for real SimpleBGC
        readback once the gimbal is powered.
        """
        snapshot = DroneBackend.TelemetrySnapshot()

        # Active source (USB cable or ELRS radio) -- the same one the
        # instruments show. None when neither has live telemetry.
        worker = getattr(self, "_worker", None)
        state = worker.get_active_state() if worker is not None else None
        if state is None:
            return snapshot   # valid=False by default -- DetectionLink skips georeferencing

        gps = state.gps
        snapshot.valid = bool(gps.position_usable)
        snapshot.latitude = gps.latitude
        snapshot.longitude = gps.longitude
        snapshot.altitude_m = float(gps.altitude_m)
        snapshot.heading_deg = float(state.yaw)
        snapshot.roll_deg = state.roll / 10.0
        snapshot.pitch_deg = state.pitch / 10.0
        tilt_deg, pan_deg = self._get_camera_mount_angle()
        snapshot.gimbal_tilt_deg = tilt_deg
        snapshot.gimbal_pan_deg = pan_deg
        return snapshot

    def _get_osd_telemetry(self) -> "DroneBackend.TelemetrySnapshot":
        """
        Trampoline passed to VideoLink.set_telemetry_provider() -- the
        pilot-facing software OSD's data source. Deliberately NOT the same
        callable as _get_detection_telemetry() above, even though
        VideoLink.h's own comment suggests reusing one: the two have
        genuinely different validity/altitude semantics and reusing the
        detection trampoline silently breaks the OSD.

        Why a separate one is needed:
          - _get_detection_telemetry() sets `valid = gps.position_usable`,
            which is correct for DetectionLink (no GPS fix -> lat/lon are
            garbage -> skip georeferencing) but wrong for the OSD: none of
            altitude/horizon/compass need a GPS fix at all (they come from
            the barometer, IMU, and magnetometer respectively). Reusing
            that flag here means the OSD would stay blank on the bench, or
            anywhere the FC hasn't gotten a GPS lock yet -- which is
            exactly what was happening. Here `valid` means "do we have a
            live FC link", not "do we have a GPS fix".
          - _get_detection_telemetry() also fills altitude_m from
            gps.altitude_m (GPS altitude). The OSD should read the
            barometer instead, same source BaroWidget already uses, so the
            two on-screen altitude readouts agree and don't depend on GPS
            at all.

        Called directly from VideoLink's own capture/paint thread, once
        per painted frame (see drawOsdOverlay() in VideoLink.cpp) -- NOT
        the Tk thread. hub.get_latest_state() is the same thread-safe,
        mutex-protected snapshot copy _get_detection_telemetry() uses, so
        calling it a second time here (once per detection pass, once per
        painted video frame) is safe -- just a small extra mutex-guarded
        copy, not a correctness concern.
        """
        snapshot = DroneBackend.TelemetrySnapshot()

        # Active source (USB cable or ELRS radio) -- the same one the
        # instruments show. None when neither has live telemetry.
        worker = getattr(self, "_worker", None)
        state = worker.get_active_state() if worker is not None else None
        if state is None:
            return snapshot   # valid=False by default -- OSD stays hidden until link is up

        # "Do we have a live FC link at all" -- NOT gated on GPS fix. This
        # guards against drawing anything when the link is down (a fresh,
        # never-populated snapshot would otherwise be all-zeros and
        # misleadingly rendered as "valid" data). Note this is coarser
        # than ideal: none of drawOsdAltitude/Horizon/Compass in
        # VideoLink.cpp currently check baro_valid/mag_valid individually
        # -- they render whatever's in the snapshot unconditionally. So if
        # the barometer or magnetometer itself isn't calibrated/valid yet
        # (state.baro_valid / state.mag_valid false) while the FC link is
        # otherwise up, the OSD will still show a number (e.g. "ALT 0.0 m")
        # with nothing telling the pilot it's not trustworthy. Flagging
        # this as a follow-up, not fixing it here -- it needs a small
        # VideoLink.cpp change (checking per-element validity before each
        # draw call) if you want it closed.
        snapshot.valid = True
        snapshot.altitude_m = float(getattr(state, "baro_altitude_cm", 0)) / 100.0
        snapshot.heading_deg = float(state.yaw)
        snapshot.roll_deg = state.roll / 10.0
        snapshot.pitch_deg = state.pitch / 10.0

        # Battery / RSSI -- straight off DroneState, same fields
        # TelemetryWorker already surfaces to the rest of the UI.
        snapshot.battery_voltage = float(state.battery_voltage)
        snapshot.battery_percentage = int(state.battery_percentage)
        snapshot.rssi = int(state.rssi)

        # Drives VideoLink's OSD flight timer (start on arm / pause on
        # disarm / resume on rearm -- see drawOsdTimer in VideoLink.cpp).
        snapshot.armed = bool(getattr(state, "armed", False))

        # GPS lock/sat count and home distance/speed -- same gps reading
        # DetectionLink's own trampoline above reads lat/lon from. Unlike
        # that one, the OSD doesn't gate `valid` on position_usable (see
        # this method's docstring), so these are read defensively in case
        # gps itself isn't populated yet.
        gps = getattr(state, "gps", None)
        if gps is not None:
            snapshot.gps_fix_type = int(getattr(gps, "fix_type", 0))
            snapshot.gps_num_sat = int(getattr(gps, "num_sat", 0))
            snapshot.home_distance_m = float(getattr(gps, "dist_to_home_m", 0.0))
            # GPSReading.ground_speed_cms is documented in cm/s despite the
            # underlying field name (groundSpeedMs) -- divide by 100 for
            # m/s, same conversion telemetry_worker.py's UI dict leaves to
            # its own consumers.
            snapshot.ground_speed_ms = float(getattr(gps, "ground_speed_cms", 0)) / 100.0

        return snapshot

    def _retry_detection_engine(self, log_on_failure: bool = False) -> None:
        """
        Attempts DetectionLink.start(). Safe to call more than once:
        DetectionLink.start() itself is a no-op that returns True
        immediately if it's already running, so this doubles as both the
        initial startup attempt (from __init__) and a manual retry (from
        _open_detection_window) for the common case where the model file
        didn't exist yet at app launch and was dropped into place after,
        without needing to restart the whole cockpit.
        """
        if self.detection_link.start():
            self._detection_engine_running = True
            self._detection_engine_detail = ""
            # Confirms which backend actually got engaged -- setUseCuda()
            # silently falls back to CPU if this build/machine's OpenCV
            # lacks CUDA DNN support. Routine confirmation is gated behind
            # COCKPIT_VERBOSE_LOGGING now that CUDA is confirmed working;
            # the unexpected-fallback case below still always prints,
            # since that's a real problem worth surfacing.
            if hasattr(self.detection_link, "is_using_cuda"):
                backend = "CUDA" if self.detection_link.is_using_cuda() else "CPU"
                _vlog(f"[DetectionLink] inference backend: {backend}")
                if DETECTION_USE_CUDA and backend == "CPU":
                    print(
                        "[DetectionLink] NOTE: DETECTION_USE_CUDA=True but "
                        "the engine fell back to CPU -- this DroneBackend.pyd "
                        "was likely built against an OpenCV without CUDA/"
                        "cuDNN support. A stock opencv-python/vcpkg opencv4 "
                        "build does not have this; you need a source (or "
                        "vcpkg with cuda+dnn-cuda features) build against "
                        "your installed CUDA toolkit + matching cuDNN."
                    )
            return

        self._detection_engine_running = False
        self._detection_engine_detail = (
            f"model not found at '{DETECTION_MODEL_PATH}'"
        )
        if log_on_failure:
            print("[DetectionLink] start() failed -- check that "
                  f"DETECTION_MODEL_PATH ('{DETECTION_MODEL_PATH}') points "
                  "at a real ONNX model and that a frame source is wired "
                  "up. Detection stays disabled; the rest of the cockpit "
                  "runs normally.")

    def _show_help_window(self) -> None:
        """
        Full "How to use this cockpit" reference, opened by clicking the
        ℹ toolbar button. Previously that button had no command attached
        at all -- it only carried a hover _Tooltip, so clicking it did
        nothing and the only guidance available was the couple of lines
        about dragging/z-order shown on mouse-over. This gives it a real
        click action plus much more complete content (panels, toolbar,
        instruments), while leaving the hover tooltip as a quick-glance
        summary.

        Reuses the same "build widgets first, then size/center" approach
        as LayoutManagerDialog/_open_detection_window so the window can't
        end up clipped or off-screen.
        """
        win = getattr(self, "_help_window", None)
        if win is not None and win.winfo_exists():
            win.deiconify()
            win.lift()
            win.focus_force()
            return

        win = tk.Toplevel(self.root, bg="#0f1428")
        self._help_window = win
        win.title("Cockpit Help")
        win.transient(self.root)

        header = tk.Frame(win, bg="#0f1428")
        header.pack(fill="x", padx=16, pady=(14, 6))
        tk.Label(
            header, text="ℹ  Drone Cockpit — Help & Controls",
            fg="#66d9ff", bg="#0f1428", font=("Consolas", 13, "bold"),
        ).pack(side="left")

        body = tk.Frame(win, bg="#0f1428")
        body.pack(fill="both", expand=True, padx=16, pady=(0, 8))

        text = tk.Text(
            body, wrap="word", bg="#0a0e1c", fg="#c0d0f0",
            font=("Consolas", 10), relief="flat", padx=14, pady=12,
            highlightthickness=1, highlightbackground="#26365a",
            cursor="arrow",
        )
        scroll = tk.Scrollbar(body, orient="vertical", command=text.yview)
        text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        text.pack(side="left", fill="both", expand=True)

        text.tag_configure("h1", foreground="#66d9ff",
                            font=("Consolas", 11, "bold"),
                            spacing3=4, spacing1=10)
        text.tag_configure("body", foreground="#c0d0f0",
                            font=("Consolas", 10), spacing3=2)
        text.tag_configure("bullet", foreground="#a0b8d8",
                            font=("Consolas", 10), lmargin1=14, lmargin2=28,
                            spacing3=1)

        def h1(t):
            text.insert("end", t + "\n", "h1")

        def body_line(t):
            text.insert("end", t + "\n", "body")

        def bullet(t):
            text.insert("end", "•  " + t + "\n", "bullet")

        h1("Panel layout")
        bullet("Drag a panel's title bar to move it anywhere in the workspace.")
        bullet("Click any panel to bring it to the front of the stack.")
        bullet("Right-click a title bar for snap-to-edge and z-order options.")
        bullet("Drag a panel's edge/corner to resize it (when unlocked).")

        h1("Toolbar")
        bullet("Layouts ▾ — switch, preview, save, rename or delete saved "
               "panel-layout profiles.")
        bullet("Layout: <name> — shows which saved profile is currently active.")
        bullet("Panels ▾ — show or hide individual instrument panels without "
               "affecting your saved layout.")
        bullet("Detections — opens the object-detection & map window in its "
               "own separate window, so it never overlaps the live FPV feed.")
        bullet("↺ Revert to Saved — discards unsaved on-screen changes and "
               "reloads the active profile's last-saved layout.")
        bullet("🏭 Factory Default — snaps all panels back to the built-in "
               "default positions (in memory only; does not touch any saved "
               "profile).")
        bullet("🔓 / 🔒 Save & Lock Layout — saves the current layout and "
               "locks panels so they can't be dragged or resized. Click "
               "again to unlock.")

        h1("Instrument panels")
        bullet("FC Status / Arming — flight-controller connection state, arm "
               "status, and pre-arm checklist.")
        bullet("IMU / Attitude — MPU-6500 roll/pitch/yaw rates and gyro/mag "
               "calibration controls, plus link round-trip time.")
        bullet("Magnetometer — QMC5883L compass heading with lock status "
               "and a CAL MAG calibration routine.")
        bullet("ADI — Attitude Indicator — artificial horizon showing bank, "
               "pitch and heading.")
        bullet("ALT / VSI — barometric altitude and vertical-speed strip, "
               "with QNH reference.")
        bullet("GPS / Map — satellite count, DOP, and a live map with "
               "position, altitude and bearing readouts.")
        bullet("FPV — Live Video — native low-latency camera feed from the "
               "aircraft, decoupled from the telemetry loop for minimum lag.")

        h1("Detections & Map window")
        bullet("Runs as its own top-level window so heavy map/detection "
               "redraws never compete with or overlap the FPV video.")
        bullet("Closing it does not stop detection — records keep "
               "accumulating in the background and resume streaming in as "
               "soon as you reopen it.")

        text.configure(state="disabled")

        close_btn = tk.Button(
            win, text="Close", command=win.destroy,
            bg="#162040", fg="#a0b8d8", activebackground="#1e3060",
            activeforeground="#ffffff", relief="flat", padx=16, pady=6,
            cursor="hand2", bd=0,
            highlightthickness=1, highlightbackground="#26365a",
        )
        close_btn.pack(pady=(0, 14))

        win.update_idletasks()
        win_w, win_h = 620, 620
        x = self.root.winfo_rootx() + (self.root.winfo_width() - win_w) // 2
        y = self.root.winfo_rooty() + (self.root.winfo_height() - win_h) // 2
        win.geometry(f"{win_w}x{win_h}+{max(0, x)}+{max(0, y)}")
        win.lift()
        win.focus_force()

    def _open_detection_window(self) -> None:
        """
        Open (or bring to front) the detection/map window. Deliberately
        NOT a DraggablePanel: DetectionMapWidget is its own Toplevel by
        design (see that module's docstring) precisely so nothing it does
        can ever obstruct or contend with the live FPV feed. Reachable
        from both the "🎯 Detections" toolbar button and the "Detections &
        Map" entry in the Panels ▾ menu (see _toggle_detection_window).
        """
        win = self._detection_map_window
        if win is not None and win.winfo_exists():
            win.deiconify()
            win.lift()
            win.focus_force()
            self._detection_vis_var.set(True)
            return

        # Give the engine one more chance here -- covers a pilot dropping
        # the model file into place after launch and then opening this
        # window, without having to restart the whole cockpit.
        if not self._detection_engine_running:
            self._retry_detection_engine()

        self._detection_map_window = DetectionMapWidget(self.root)
        self._detection_map_window.protocol(
            "WM_DELETE_WINDOW", self._close_detection_window)
        self._detection_map_window.set_engine_status(
            self._detection_engine_running, self._detection_engine_detail)

        # Without this, DetectionMapWidget._link stays None and the live
        # annotated-feed pane (and its poll loop) never starts -- see
        # DetectionMapWidget.set_detection_link()'s own docstring. This was
        # the one call missing between DetectionLink starting up above and
        # the window actually being able to show anything live.
        self._detection_map_window.set_detection_link(self.detection_link)

        # Center over the main window and force it to the front. A bare
        # Toplevel with no explicit position can land off in a corner or
        # behind the main window depending on the window manager, which
        # is easy to mistake for "the window never opened."
        self.root.update_idletasks()
        win_w, win_h = 900, 560
        x = self.root.winfo_rootx() + (self.root.winfo_width() - win_w) // 2
        y = self.root.winfo_rooty() + (self.root.winfo_height() - win_h) // 2
        self._detection_map_window.geometry(f"+{max(0, x)}+{max(0, y)}")
        self._detection_map_window.lift()
        self._detection_map_window.focus_force()

        self._detection_vis_var.set(True)
        self._pump_detection_records()

    def _close_detection_window(self) -> None:
        """Stop pumping records into the window and let it close normally.
        DetectionWorker/DetectionLink keep running and keep accumulating
        records regardless -- reopening the window just resumes draining
        them, nothing already collected is lost."""
        if self._detection_pump_job is not None:
            self.root.after_cancel(self._detection_pump_job)
            self._detection_pump_job = None
        win = self._detection_map_window
        self._detection_map_window = None
        self._detection_vis_var.set(False)
        if win is not None and win.winfo_exists():
            win.destroy()

    def _toggle_detection_window(self) -> None:
        """Bound to the 'Detections & Map' checkbutton in the Panels ▾
        menu -- mirrors _toggle_panel()'s open/close semantics for the
        regular docked panels, even though this one lives in its own
        Toplevel rather than the workspace canvas."""
        if self._detection_vis_var.get():
            self._open_detection_window()
        else:
            self._close_detection_window()

    def _get_telemetry_status_summary(self) -> tuple:
        """
        Returns (valid, detail) describing, in plain language, whether the
        NEXT detection pass will be able to georeference anything. Mirrors
        exactly the same gps.position_usable check _get_detection_telemetry()
        gates DetectionLink's snapshot on, so this can never disagree with
        the real reason pins are or aren't appearing on the map -- it
        exists purely so DetectionMapWidget can show that reason instead of
        an operator having to cross-reference the FC status panel themselves.
        """
        worker = getattr(self, "_worker", None)
        try:
            state = worker.get_active_state() if worker is not None else None
        except Exception:
            return False, "telemetry read failed"
        if state is None:
            return False, "no telemetry link (USB or radio)"
        gps = state.gps
        # The ELRS/CRSF path carries no HDOP (reported as 9999); there
        # position_usable is decided from the satellite count alone.
        if gps.hdop >= 9999:
            hdop_txt = "HDOP n/a on radio link"
            need_txt = "need >=5 sats on radio link"
        else:
            hdop_txt = f"HDOP {gps.hdop / 100.0:.1f}"
            need_txt = "need fix>=2D, >=4 sats, HDOP<5.0"
        if bool(gps.position_usable):
            return True, f"GPS usable ({gps.num_sat} sats, {hdop_txt})"
        return False, f"GPS not usable yet ({gps.num_sat} sats, {hdop_txt} -- {need_txt})"

    def _get_current_drone_fix(self) -> tuple:
        """
        Returns (latitude, longitude, valid) for the map's drone marker.
        Deliberately independent of DetectionLink/DetectionWorker entirely
        -- the map should be able to show where the drone is regardless of
        whether anything has ever been detected, so this reads straight
        from the same hub state _get_detection_telemetry() does rather
        than going through a detection record.

        NOTE: deliberately does NOT use gps.position_usable here.
        position_usable additionally requires HDOP<5.0 -- that threshold
        exists to gate whether a DETECTION gets georeferenced accurately
        enough to trust its computed lat/lon (see _get_detection_telemetry /
        _get_telemetry_status_summary), not whether the map has anything to
        center a basemap on. Reusing it here meant the map (including the
        already-downloaded/cached tile mosaic -- see
        DetectionMapWidget._redraw_map's _map_origin gate) stayed on
        "Waiting for GPS fix..." any time HDOP was above 5.0, even with a
        solid 3D fix and 4+ sats, which is exactly when the FC status
        panel's own GPS widget was already showing a lock. A noisy fix is
        still fine for "roughly where is the drone" -- it just isn't
        accurate enough to trust a detection's triangulated position
        against, which is what position_usable is actually for.
        """
        worker = getattr(self, "_worker", None)
        try:
            state = worker.get_active_state() if worker is not None else None
            # Radio link LOST: keep the marker at the LAST KNOWN position
            # instead of removing it -- for search & rescue, where the drone
            # was when the link dropped is exactly what the crew needs.
            radio = getattr(self, "radio", None)
            if state is None and radio is not None:
                rs = radio.get_latest_state()
                if rs.radio.status_str == "TELEMETRY_LOST":
                    state = rs
        except Exception:
            return 0.0, 0.0, False
        if state is None:
            return 0.0, 0.0, False
        gps = state.gps
        # Loose "do we have *any* real fix" check, independent of HDOP:
        # enough satellites for a fix to exist at all, and a coordinate
        # that isn't the zeroed-out default. If your GPS binding exposes
        # a fix_type field (2D/3D) that's a cleaner check than num_sat
        # alone -- swap it in here if so.
        has_fix = gps.num_sat >= 4 and (gps.latitude != 0.0 or gps.longitude != 0.0)
        return gps.latitude, gps.longitude, has_fix

    def _pump_detection_records(self) -> None:
        """
        Drains DetectionWorker's incremental new-records buffer into the
        open map window -- same "poll cheap results on the Tk thread"
        pattern the FPV status bar uses for VideoWorker. Only scheduled
        while the window is actually open, so a pilot who never opens it
        never pays for the Treeview churn (DetectionWorker itself keeps
        polling and buffering in the background regardless -- see its own
        poll loop -- so nothing is missed by the window being closed).
        """
        win = self._detection_map_window
        if win is None or not win.winfo_exists():
            self._detection_map_window = None
            self._detection_pump_job = None
            self._detection_vis_var.set(False)
            return

        records = self._detection_worker.get_new_records()
        if records:
            win.add_records(records)

        telemetry_valid, telemetry_detail = self._get_telemetry_status_summary()
        win.set_telemetry_status(telemetry_valid, telemetry_detail)

        drone_lat, drone_lon, drone_valid = self._get_current_drone_fix()
        win.set_drone_telemetry(drone_lat, drone_lon, drone_valid)

        self._detection_pump_job = self.root.after(
            int(1000 / DETECTION_POLL_HZ), self._pump_detection_records)

    # =========================================================================
    # Layout persistence
    # =========================================================================

    def _default_snapshot(self) -> dict:
        """Factory-default layout, used to seed the very first profile."""
        return {
            "geometry": None,
            "locked": False,
            "panels": {name: dict(geo) for name, geo in _DEFAULT_PANELS.items()},
            "camera_angle": {
                "mode": "manual",
                "tilt_deg": CAMERA_MOUNT_TILT_DEG,
                "pan_deg": CAMERA_MOUNT_PAN_DEG,
            },
        }

    def _current_snapshot(self) -> dict:
        """Capture the on-screen state (geometry, lock, all panel rects,
        and the "📐 Camera Angle" mode/tilt/pan) -- saved together as one
        profile so restoring a layout also restores how the camera was
        mounted when that layout was last used."""
        tilt_deg, pan_deg = self._get_camera_mount_angle()
        return {
            "geometry": self.root.geometry(),
            "locked":   self._locked_ref[0],
            "panels": {
                name: {
                    **panel.get_geometry(),
                    "visible": self._vis_vars[name].get(),
                }
                for name, panel in self._panels.items()
            },
            "camera_angle": {
                "mode": self._camera_angle_mode,
                "tilt_deg": tilt_deg,
                "pan_deg": pan_deg,
            },
        }

    def _persist_active(self) -> None:
        """Write the current on-screen layout back into the active profile."""
        self._layout_store.upsert_profile(self._layout_store.active_name(),
                                          self._current_snapshot())

    def _apply_layout(self, layout: dict) -> None:
        """Push a saved profile's geometry/visibility/lock state onto the UI."""
        geom = layout.get("geometry")
        if geom:
            try:
                self.root.geometry(geom)
            except Exception:
                pass
        else:
            sw = self.root.winfo_screenwidth()
            sh = self.root.winfo_screenheight()
            self.root.geometry(f"{min(1400, sw-40)}x{min(960, sh-60)}+20+20")

        saved_panels = layout.get("panels", {})
        for name, panel in self._panels.items():
            geo = saved_panels.get(name)
            if not geo or "x" not in geo:
                geo = _DEFAULT_PANELS.get(name, {"x": 10, "y": 10,
                                                  "w": 400, "h": 200,
                                                  "visible": True})
            panel.set_geometry(geo["x"], geo["y"], geo["w"], geo["h"])
            vis = geo.get("visible", True)
            self._vis_vars[name].set(vis)
            panel.show() if vis else panel.hide()

        locked = layout.get("locked", False)
        self._locked_ref[0] = locked
        self._update_lock_button(locked)

        # ── Restore camera mount angle (mode + tilt/pan) ──────────────────
        # Falls back to the module-level startup defaults for any profile
        # saved before this field existed, or with a missing/malformed
        # entry, so old layout files keep loading instead of erroring.
        cam = layout.get("camera_angle") or {}
        mode = cam.get("mode", "manual")
        if mode not in ("manual", "auto"):
            mode = "manual"
        tilt_deg = cam.get("tilt_deg", CAMERA_MOUNT_TILT_DEG)
        pan_deg = cam.get("pan_deg", CAMERA_MOUNT_PAN_DEG)
        with self._camera_angle_lock:
            self._camera_angle_mode = mode
            self._camera_tilt_deg = float(tilt_deg)
            self._camera_pan_deg = float(pan_deg)
        self._refresh_camera_angle_label()

        self.root.after_idle(self._clamp_all_panels)

    # =========================================================================
    # Viewport clamping
    # =========================================================================

    def _clamp_all_panels(self) -> None:
        for name in self._panels:
            self._clamp_panel(name)

    def _clamp_panel(self, name: str) -> None:
        self.root.update_idletasks()
        ws_w = self._workspace.winfo_width()
        ws_h = self._workspace.winfo_height()

        if ws_w < 10 or ws_h < 10:
            self.root.after(100, lambda: self._clamp_panel(name))
            return

        panel  = self._panels[name]
        geo    = panel.get_geometry()
        x, y, w, h = geo["x"], geo["y"], geo["w"], geo["h"]

        MARGIN_X = 60
        TITLE_H  = 26

        new_x = max(0, min(x, ws_w - MARGIN_X))
        new_y = max(0, min(y, ws_h - TITLE_H))

        if new_x != x or new_y != y:
            _vlog(f"[Viewport] clamping '{name}' from ({x},{y}) → ({new_x},{new_y})")
            panel.set_geometry(new_x, new_y, w, h)

    # =========================================================================
    # Lock / Unlock
    # =========================================================================

    def _update_lock_button(self, locked: bool) -> None:
        """Set the lock icon, colors, and tooltip text to match state."""
        if locked:
            self._lock_btn.config(text="🔒", relief="sunken",
                                  bg="#1a2a1a", fg="#00ff88")
            tip = "Locked — click to unlock and allow panels to be moved/resized again"
        else:
            self._lock_btn.config(text="🔓", relief="raised",
                                  bg="#162040", fg="#00d4ff")
            tip = ("Save & Lock Layout — saves the current layout and locks "
                   "panels from being dragged or resized")
        if self._lock_btn in self._tooltips:
            self._tooltips[self._lock_btn].set_text(tip)

    def _toggle_lock(self) -> None:
        self._locked_ref[0] = not self._locked_ref[0]
        self._update_lock_button(self._locked_ref[0])
        self._persist_active()

    def _reset_layout(self) -> None:
        """Snap panels back to the hardcoded factory defaults (in-memory
        only — does NOT touch any saved profile). Use 'Revert to Saved' to
        undo unsaved changes to the active profile instead."""
        if self._locked_ref[0]:
            return
        for name, panel in self._panels.items():
            geo = _DEFAULT_PANELS.get(name, {})
            if geo:
                panel.set_geometry(geo["x"], geo["y"], geo["w"], geo["h"])
        self.root.after_idle(self._clamp_all_panels)

    def _revert_to_saved(self) -> None:
        """Discard unsaved on-screen changes by reloading the active
        profile's last-saved state from disk."""
        active = self._layout_store.active_name()
        profile = self._layout_store.get_profile(active)
        if profile:
            self._apply_layout(profile)

    # =========================================================================
    # Layout profiles (multi-loadout management)
    # =========================================================================

    def _refresh_active_layout_label(self) -> None:
        self._active_layout_lbl.config(text=f"Layout: {self._layout_store.active_name()}")

    def _open_layout_manager(self) -> None:
        LayoutManagerDialog(
            self.root, self._layout_store,
            on_load=self._load_profile,
            on_save_current_as=self._save_current_as_profile,
            on_overwrite=self._overwrite_profile,
        )

    def _load_profile(self, name: str) -> None:
        profile = self._layout_store.get_profile(name)
        if profile is None:
            return
        self._apply_layout(profile)
        self._layout_store.set_active(name)
        self._refresh_active_layout_label()

    def _save_current_as_profile(self, name: str) -> None:
        self._layout_store.upsert_profile(name, self._current_snapshot(), make_active=True)
        self._refresh_active_layout_label()

    def _overwrite_profile(self, name: str) -> None:
        active = self._layout_store.active_name()
        self._layout_store.upsert_profile(name, self._current_snapshot(),
                                          make_active=(name == active))
        self._refresh_active_layout_label()

    def _show_layouts_menu(self) -> None:
        menu_kw = dict(bg="#0f1428", fg="#c0d0f0",
                       activebackground="#1e3060", activeforeground="#ffffff",
                       font=("Consolas", 10), bd=1, relief="solid")
        menu = tk.Menu(self.root, tearoff=0, **menu_kw)
        active = self._layout_store.active_name()
        names = self._layout_store.profile_names()

        for name in names:
            prefix = "●  " if name == active else "    "
            # Each profile is a cascade so Load and Delete are both reachable
            # right from the toolbar, without opening the manager dialog.
            sub = tk.Menu(menu, tearoff=0, **menu_kw)
            sub.add_command(label="▶  Load", command=lambda n=name: self._load_profile(n))
            sub.add_command(label="🗑  Delete",
                            command=lambda n=name: self._quick_delete_profile(n))
            menu.add_cascade(label=f"{prefix}{name}", menu=sub)

        menu.add_separator()
        menu.add_command(label="  🔎  Preview & Manage…", command=self._open_layout_manager)
        menu.add_command(label="  💾  Save Current As New…", command=self._prompt_save_current_as)

        btn = self._layouts_btn
        x = btn.winfo_rootx()
        y = btn.winfo_rooty() + btn.winfo_height()
        try:
            menu.tk_popup(x, y)
        finally:
            menu.grab_release()

    def _prompt_save_current_as(self) -> None:
        name = simpledialog.askstring("Save Layout", "Profile name:", parent=self.root)
        if not name:
            return
        if name in self._layout_store.profile_names():
            if not messagebox.askyesno("Overwrite Existing?",
                                       f"'{name}' already exists. Overwrite it?"):
                return
        self._save_current_as_profile(name)

    def _quick_delete_profile(self, name: str) -> None:
        if len(self._layout_store.profile_names()) <= 1:
            messagebox.showwarning("Can't Delete",
                                   "At least one layout profile must remain.")
            return
        if not messagebox.askyesno("Delete Layout",
                                   f"Delete saved profile '{name}'? This cannot be undone."):
            return
        was_active = (name == self._layout_store.active_name())
        self._layout_store.delete_profile(name)
        self._refresh_active_layout_label()
        # If we just deleted the profile that was on-screen, load whatever
        # is now active so the label and workspace stay in sync.
        if was_active:
            new_active = self._layout_store.get_profile(self._layout_store.active_name())
            if new_active:
                self._apply_layout(new_active)

    # =========================================================================
    # Panel visibility
    # =========================================================================

    def _show_panels_menu(self) -> None:
        menu = tk.Menu(self.root, tearoff=0,
                       bg="#0f1428", fg="#c0d0f0",
                       activebackground="#1e3060",
                       activeforeground="#ffffff",
                       selectcolor="#ffffff",
                       font=("Consolas", 10), bd=1, relief="solid")
        for name, label in _PANEL_LABELS.items():
            menu.add_checkbutton(
                label=f"  {label}",
                variable=self._vis_vars[name],
                command=lambda n=name: self._toggle_panel(n),
            )
        menu.add_separator()
        # Detections & Map isn't a docked DraggablePanel (see
        # DetectionMapWidget's module docstring for why it's kept as its
        # own Toplevel), but it still gets an entry here so it's
        # discoverable the same way the docked panels are, instead of
        # only being reachable via the toolbar icon.
        menu.add_checkbutton(
            label="  🎯 Detections & Map",
            variable=self._detection_vis_var,
            command=self._toggle_detection_window,
        )
        menu.add_separator()
        menu.add_command(label="  Show All", command=self._show_all)
        menu.add_command(label="  Hide All", command=self._hide_all)
        btn = self._panels_btn
        x = btn.winfo_rootx()
        y = btn.winfo_rooty() + btn.winfo_height()
        try:
            menu.tk_popup(x, y)
        finally:
            menu.grab_release()

    def _toggle_panel(self, name: str) -> None:
        if self._vis_vars[name].get():
            self._panels[name].show()
            self.root.after_idle(lambda n=name: self._clamp_panel(n))
        else:
            self._panels[name].hide()

    def _show_all(self):
        for name in self._panels:
            self._vis_vars[name].set(True)
            self._panels[name].show()
        self.root.after_idle(self._clamp_all_panels)

    def _hide_all(self):
        for name in self._panels:
            self._vis_vars[name].set(False)
            self._panels[name].hide()

    # =========================================================================
    # Heading trim
    # =========================================================================

    def _apply_heading_trim(self) -> None:
        """
        Called from the IMU widget button on the Tk thread.
        Reads the last-seen sensor snapshot from the worker (thread-safe)
        and updates the trim, then pushes the new value back to the worker.
        """
        raw_yaw, mag_heading, mag_valid = self._worker.get_last_state_snapshot()
        if not mag_valid:
            return
        current_trimmed = (raw_yaw - self._yaw_trim + 360) % 360
        drift = (current_trimmed - mag_heading + 540) % 360 - 180
        self._yaw_trim = (self._yaw_trim + drift + 360) % 360
        # Push updated trim to worker so subsequent frames use it
        self._worker.set_yaw_trim(self._yaw_trim)

    # =========================================================================
    # UI setup
    # =========================================================================

    def _mk_icon_btn(self, parent, icon: str, command, tooltip: str,
                      fg: str = "#a0b8d8", active_fg: str = "#ffffff",
                      label: str = "") -> tk.Button:
        """Build a toolbar button: an icon by itself, or an icon plus a
        short text label (for the Layouts/Panels dropdowns, so their
        purpose is obvious without needing the tooltip). Flat relief plus
        a hover highlight give it a more modern, less boxy feel than the
        old fixed 'raised' icon buttons."""
        base_bg  = "#162040"
        hover_bg = "#1c2c54"
        btn = tk.Button(
            parent, text=(f"{icon}  {label}" if label else icon), command=command,
            font=("Segoe UI", 10) if label else ("Segoe UI Emoji", 11),
            width=0 if label else 3,
            relief="flat", padx=10 if label else 2, pady=4 if label else 2,
            bg=base_bg, fg=fg,
            activebackground="#1e3060", activeforeground=active_fg,
            cursor="hand2", bd=0,
            highlightthickness=1, highlightbackground="#26365a",
        )
        btn.bind("<Enter>", lambda e, b=btn: b.config(bg=hover_bg, highlightbackground="#3a6ab0"))
        btn.bind("<Leave>", lambda e, b=btn: b.config(bg=base_bg, highlightbackground="#26365a"))
        self._tooltips[btn] = _Tooltip(btn, tooltip)
        return btn

    def _setup_ui(self) -> None:

        # ── Toolbar ───────────────────────────────────────────────────────────
        toolbar = tk.Frame(self.root, bg="#0d0d1a", pady=6)
        toolbar.pack(side="top", fill="x", padx=10)

        self.status_label = tk.Label(
            toolbar, text="Status: Searching...",
            fg="orange", bg="#0d0d1a",
            font=("Consolas", 10, "bold"),
        )
        self.status_label.pack(side="left", padx=(0, 16))

        # ELRS radio-link traffic light (green / yellow / red, blinks on
        # link loss). Hover for LQ, RSSI, update rates and reconnect counts.
        self.radio_indicator = RadioLinkIndicator(toolbar)
        self.radio_indicator.pack(side="left", padx=(0, 16))
        # Tooltips go on the indicator's child widgets: a tooltip on the Frame
        # itself would hide again as soon as the pointer moves onto a child.
        self._radio_tooltips = []
        for w_ in self.radio_indicator.hover_widgets():
            tip = _Tooltip(w_, "Radio link status")
            self._tooltips[w_] = tip
            self._radio_tooltips.append(tip)

        # Icon-only buttons throughout — mixing wide unicode glyphs with
        # long labels was causing Tkinter to mis-size/clip buttons on some
        # platforms. Every icon button gets a hover tooltip instead.

        self._layouts_btn = self._mk_icon_btn(
            toolbar, "🗂", self._show_layouts_menu,
            "Layouts — switch, preview, save, rename or delete panel-layout profiles",
            label="Layouts ▾",
        )
        self._layouts_btn.pack(side="left", padx=(0, 8))

        self._active_layout_lbl = tk.Label(
            toolbar, text="",
            fg="#66aadd", bg="#13203a",
            font=("Consolas", 9, "bold"),
            padx=8, pady=3,
            highlightthickness=1, highlightbackground="#2a4a76",
        )
        self._active_layout_lbl.pack(side="left", padx=(0, 16))

        self._panels_btn = self._mk_icon_btn(
            toolbar, "🧩", self._show_panels_menu,
            "Panels — show or hide individual instrument panels",
            label="Panels ▾",
        )
        self._panels_btn.pack(side="left", padx=(0, 6))

        self._detections_btn = self._mk_icon_btn(
            toolbar, "🎯", self._open_detection_window,
            "Detections — open the object-detection & map window. Runs in "
            "its own top-level window (see DetectionMapWidget) so it never "
            "competes with or overlaps the live FPV feed.",
            label="Detections",
        )
        self._detections_btn.pack(side="left", padx=(0, 6))

        self._camera_angle_btn = self._mk_icon_btn(
            toolbar, "📐", self._show_camera_angle_dialog,
            "Camera Angle — set how the FPV camera is physically mounted "
            "(Manual), or hand off to real gimbal feedback once the "
            "SimpleBGC gimbal is wired (Auto). Used for detection "
            "georeferencing. Saved with the active layout profile, so it "
            "persists across restarts.",
            label="Camera Angle",
        )
        self._camera_angle_btn.pack(side="left", padx=(0, 8))

        self._camera_angle_lbl = tk.Label(
            toolbar, text="",
            fg="#66aadd", bg="#13203a",
            font=("Consolas", 9, "bold"),
            padx=8, pady=3,
            highlightthickness=1, highlightbackground="#2a4a76",
        )
        self._camera_angle_lbl.pack(side="left", padx=(0, 16))
        self._refresh_camera_angle_label()

        info_btn = self._mk_icon_btn(
            toolbar, "ℹ", self._show_help_window,
            "Help — click for the full guide to panels, the toolbar, and "
            "the instruments (drag title bars to move panels, click to "
            "bring to front, right-click for snap-to and z-order).",
        )
        info_btn.pack(side="left")

        self._lock_btn = self._mk_icon_btn(
            toolbar, "🔓", self._toggle_lock,
            "Save & Lock Layout — saves the current layout and locks panels "
            "from being dragged or resized",
            fg="#00d4ff", active_fg="#00ffff",
        )
        self._lock_btn.pack(side="right", padx=(6, 0))

        self._reset_btn = self._mk_icon_btn(
            toolbar, "🏭", self._reset_layout,
            "Factory Default — reset all panels to the built-in default "
            "positions (does not touch any saved layout)",
        )
        self._reset_btn.pack(side="right", padx=(6, 0))

        self._revert_btn = self._mk_icon_btn(
            toolbar, "↺", self._revert_to_saved,
            "Revert to Saved — discard unsaved changes and reload the "
            "active layout's last-saved state",
        )
        self._revert_btn.pack(side="right", padx=(6, 0))

        # ── Workspace canvas ──────────────────────────────────────────────────
        self._workspace = tk.Canvas(
            self.root, bg=BG_WORKSPACE, highlightthickness=0,
        )
        self._workspace.pack(fill="both", expand=True, padx=4, pady=(0, 4))
        self._workspace.bind("<Configure>", self._on_workspace_resize)

        # ── Panel factory ─────────────────────────────────────────────────────
        def _panel(name: str, title: str) -> DraggablePanel:
            self._vis_vars[name] = tk.BooleanVar(value=True)
            geo = _DEFAULT_PANELS[name]
            p = DraggablePanel(
                self._workspace, name, title,
                x=geo["x"], y=geo["y"], w=geo["w"], h=geo["h"],
                locked_ref=self._locked_ref,
                all_panels_ref=self._panels,
                on_zorder=self._on_panel_zorder,
                on_visibility=self._on_panel_visibility,
            )
            self._panels[name] = p
            return p

        # ── IMU ───────────────────────────────────────────────────────────────
        p = _panel("imu", "MPU-6500  —  IMU / Attitude")
        self.imu_view = IMUWidget(p.content,
                                  on_adjust_heading=self._apply_heading_trim)
        self.imu_view.pack(fill="both", expand=True)

        # ── Magnetometer ──────────────────────────────────────────────────────
        p = _panel("mag", "QMC5883L  —  Magnetometer")
        self.mag_view = MagWidget(
            p.content,
            on_mag_calibrate=self.hub.start_mag_calibration,
            on_acc_calibrate=self.hub.start_acc_calibration,
        )
        self.mag_view.pack(fill="both", expand=True)

        # ── ADI ───────────────────────────────────────────────────────────────
        p = _panel("adi", "ADI  —  Attitude Direction Indicator")
        self.drone_3d = Drone3DView(p.content, width=420, height=340)
        self.drone_3d.pack(fill="both", expand=True)

        # ── Baro ──────────────────────────────────────────────────────────────
        p = _panel("baro", "ALT / VSI  —  Altitude & Vertical Speed")
        self.baro_view = BaroWidget(p.content)
        self.baro_view.pack(fill="both", expand=True)

        # ── GPS ───────────────────────────────────────────────────────────────
        p = _panel("gps", "GPS Navigation  —  Nav | Satellites | Map")
        self.gps_view = GPSWidget(p.content)
        self.gps_view.pack(fill="both", expand=True)

        # ── FC Status ─────────────────────────────────────────────────────────
        p = _panel("fc_status", "FC Status  —  Armed · Mode · Sensors · CPU")
        self.fc_status_view = FCStatusWidget(p.content)
        self.fc_status_view.pack(fill="both", expand=True)

        # ── Arming Diagnostics ────────────────────────────────────────────────
        p = _panel("arming", "Arming Diagnostics  —  Pre-flight Checklist")
        self.arming_view = ArmingWidget(p.content)
        self.arming_view.pack(fill="both", expand=True)

        # ── FPV Video ─────────────────────────────────────────────────────────
        p = _panel("fpv", "FPV  —  Live Video Feed")
        self.fpv_view = FPVWidget(p.content)
        self.fpv_view.pack(fill="both", expand=True)

    # ── Grid + resize handler ─────────────────────────────────────────────────

    def _on_workspace_resize(self, event=None) -> None:
        self._draw_grid(event)

    def _draw_grid(self, event=None) -> None:
        self._workspace.delete("grid")
        w = self._workspace.winfo_width()
        h = self._workspace.winfo_height()
        for x in range(0, w, 40):
            self._workspace.create_line(x, 0, x, h, fill="#1e1e34", tags="grid")
        for y in range(0, h, 40):
            self._workspace.create_line(0, y, w, y, fill="#1e1e34", tags="grid")
        self._workspace.tag_lower("grid")

    # =========================================================================
    # Connection
    # =========================================================================

    # USB (cable) link: probe → connect → watch → drop → probe again.
    # The telemetry worker and the Tk pump are NOT tied to this any more —
    # they run from startup and use whichever source (USB / ELRS) is live.
    USB_DROP_AFTER_S = 5.0   # cable link unhealthy this long → release port, re-probe

    def _auto_connect(self) -> None:
        """Start one background USB probe (no-op if one is already running)."""
        self._usb_retry_job = None
        if self._usb_probe_running or self.hub.is_connected():
            return
        self._usb_probe_running = True
        self.status_label.config(text="USB: searching...", fg="orange")

        def _probe():
            port, ok = "NOT_FOUND", False
            try:
                exclude = []
                if self.radio is not None and self.radio.get_port_name():
                    exclude.append(self.radio.get_port_name())
                if hasattr(DroneBackend, "auto_detect_fc"):
                    # MSP handshake — never mistakes the Pocket for the FC
                    port = DroneBackend.auto_detect_fc(exclude)
                else:
                    port = DroneBackend.auto_detect_f405()
                if port != "NOT_FOUND":
                    ok = bool(self.hub.connect(port))
                    if ok and self.radio is not None:
                        self.radio.set_excluded_ports([port])
            except Exception as e:
                print(f"[DroneLink] USB auto-connect error: {e}")
            self._usb_probe_result = (port, ok)   # picked up in _update_loop
            self._usb_probe_running = False

        threading.Thread(target=_probe, name="UsbProbe", daemon=True).start()

    def _schedule_usb_retry(self) -> None:
        if self._usb_retry_job is None and not self._usb_probe_running:
            self._usb_retry_job = self.root.after(RECONNECT_MS, self._auto_connect)

    def _reconnect(self) -> None:
        """
        The cable link has been dead for USB_DROP_AFTER_S (cable pulled, FC
        rebooted …). Release the port on a background thread — DroneLink's
        disconnect() joins its poll thread — then probe again. The telemetry
        worker keeps running and falls over to ELRS if the radio link is up.
        """
        self._usb_port = None
        self._usb_unhealthy_since = None
        self.status_label.config(text="USB: link lost — reconnecting...", fg="red")

        def _drop():
            try:
                self.hub.disconnect()
            except Exception as e:
                print(f"[DroneLink] disconnect error: {e}")
            if self.radio is not None:
                self.radio.set_excluded_ports([])

        t = threading.Thread(target=_drop, name="UsbDrop", daemon=True)
        t.start()
        self._usb_retry_job = self.root.after(RECONNECT_MS, self._auto_connect)

    def _update_link_status(self) -> None:
        """Tk thread, every pump tick: USB label + radio traffic light."""
        link = self._worker.get_link_status()

        # ── Radio indicator ──────────────────────────────────────────────────
        self.radio_indicator.update_status(link)
        for tip in self._radio_tooltips:
            tip.set_text(self.radio_indicator.detail_text or "Radio link status")

        # ── Result of a finished USB probe ───────────────────────────────────
        if self._usb_probe_result is not None:
            port, ok = self._usb_probe_result
            self._usb_probe_result = None
            if ok:
                self._usb_port = port
                self._usb_unhealthy_since = None
            else:
                if port != "NOT_FOUND":
                    print(f"[DroneLink] found {port} but connect failed (port busy?)")
                self._schedule_usb_retry()

        # ── USB label + drop detection ───────────────────────────────────────
        src = link.get("active_source", "NONE")
        if self.hub.is_connected():
            if link.get("usb_healthy"):
                self._usb_unhealthy_since = None
                self.status_label.config(
                    text=f"USB: connected {self._usb_port or ''}".rstrip(), fg="#00ff88")
            else:
                now = time.monotonic()
                if self._usb_unhealthy_since is None:
                    self._usb_unhealthy_since = now
                if now - self._usb_unhealthy_since >= self.USB_DROP_AFTER_S:
                    self._reconnect()
                else:
                    self.status_label.config(text="USB: link degraded", fg="orange")
        elif self._usb_probe_running:
            self.status_label.config(text="USB: searching...", fg="orange")
        else:
            # Not an error when flying: the radio carries telemetry then.
            self.status_label.config(
                text="USB: not connected" + (" (radio active)" if src == "ELRS" else ""),
                fg="#8090a8" if src == "ELRS" else "orange")
            self._schedule_usb_retry()

    # =========================================================================
    # Update loop  —  Tk main thread only
    # =========================================================================

    def _schedule_update(self) -> None:
        if self._update_job is None:
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    def _update_loop(self) -> None:
        self._update_job = None

        # ── FPV status/FPS text — independent of telemetry connection state ──
        # This runs every pump tick's throttle check regardless of whether
        # the flight controller link is up, connecting, or degraded: the
        # video panel shouldn't blank out just because MSP telemetry
        # hiccuped. Note this only updates the small text overlay now --
        # the actual video pixels are painted directly by VideoLink into a
        # native window and never pass through this loop at all.
        if self._frame_counters["fpv"] >= _THROTTLE["fpv"]:
            self._frame_counters["fpv"] = 0
            connected, fps, device_name = self._video_worker.get_status()
            self.fpv_view.update_status(connected, fps, device_name)
        self._frame_counters["fpv"] += 1

        # ── Link status: USB label + ELRS radio indicator ─────────────────────
        # Runs every tick, even when no telemetry source is live, so the
        # pilot always sees WHY the instruments aren't moving.
        self._update_link_status()

        # ── Connection health check ───────────────────────────────────────────
        # No live source (neither USB nor ELRS): instruments keep their last
        # values; the red radio indicator / USB label explain the situation.
        if not self._worker.is_connected:
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)
            return

        # ── Drain queue — get the freshest frame only ─────────────────────────
        ui_data = self._worker.get_frame()
        if ui_data is None:
            # Worker is alive but no new frame this tick — reschedule and yield
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)
            return

        # ── Feed widgets with per-widget throttling ───────────────────────────
        #
        # Increment every counter.  A widget only gets updated when its
        # counter reaches its divisor, then it resets to 0.
        # This keeps fast widgets (IMU) at full rate and slow/heavy ones
        # (GPS map, 3-D view) at a fraction — without stalling the Tk thread.
        #
        for key in self._frame_counters:
            if key == "fpv":
                continue   # already handled above, independent of telemetry
            self._frame_counters[key] += 1

        # ── Fast widgets — update every frame ────────────────────────────────
        if self._frame_counters["imu"] >= _THROTTLE["imu"]:
            self._frame_counters["imu"] = 0
            self.imu_view.update_ui(ui_data)

        if self._frame_counters["baro"] >= _THROTTLE["baro"]:
            self._frame_counters["baro"] = 0
            self.baro_view.update_baro(ui_data)

        if self._frame_counters["fc_status"] >= _THROTTLE["fc_status"]:
            self._frame_counters["fc_status"] = 0
            self.fc_status_view.update_fc_status(ui_data)

        if self._frame_counters["arming"] >= _THROTTLE["arming"]:
            self._frame_counters["arming"] = 0
            self.arming_view.update_arming(ui_data)

        if self._frame_counters["mag"] >= _THROTTLE["mag"]:
            self._frame_counters["mag"] = 0
            self.mag_view.update_mag(ui_data)

        # ── Medium widget — 3-D attitude view (~17 Hz) ────────────────────────
        if self._frame_counters["adi"] >= _THROTTLE["adi"]:
            self._frame_counters["adi"] = 0
            self.drone_3d.update_orientation(
                roll=ui_data["roll"],
                pitch=ui_data["pitch"],
                yaw=ui_data["yaw"],
                mag_heading=ui_data["mag_heading_deg"],
                mag_valid=ui_data["mag_valid"],
            )

        # ── Heavy widget — GPS map (~10 Hz) ──────────────────────────────────
        if self._frame_counters["gps"] >= _THROTTLE["gps"]:
            self._frame_counters["gps"] = 0
            self.gps_view.update_gps(ui_data)

        self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    # =========================================================================
    # Clean shutdown
    # =========================================================================

    def shutdown(self) -> None:
        self._persist_active()
        if self._update_job is not None:
            self.root.after_cancel(self._update_job)
        if self._detection_pump_job is not None:
            self.root.after_cancel(self._detection_pump_job)
        self._worker.stop()
        self.hub.disconnect()
        if self.radio is not None:
            self.radio.disconnect()
        self._video_worker.stop()
        self.fpv_view.detach()
        self.video_link.disconnect()
        self._detection_worker.stop()
        self.detection_link.stop()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app  = DroneCockpitApp(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()