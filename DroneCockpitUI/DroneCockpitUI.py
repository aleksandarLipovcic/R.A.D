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
"""

import sys
import os
import json
import threading
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import simpledialog, messagebox

from telemetry_worker import TelemetryWorker
from video_worker      import VideoWorker

from IMUWidget      import IMUWidget
from Drone3DView    import Drone3DView
from BaroWidget     import BaroWidget
from MagWidget      import MagWidget
from GPSWidget      import GPSWidget
from FCStatusWidget import FCStatusWidget
from ArmingWidget   import ArmingWidget
from FPVWidget      import FPVWidget

# ── Path configuration ────────────────────────────────────────────────────────
script_dir   = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Debug'))
sys.path.append(backend_path)

# DroneBackend.pyd is a native pybind11 extension module. On Python 3.8+,
# the DLL loader used to import extension modules does NOT search PATH for
# the module's own transitive dependencies -- it only searches directories
# explicitly registered via os.add_dll_directory() (plus the folder the
# .pyd itself lives in). DroneBackend.pyd depends on opencv_world4120d.dll,
# so OpenCV's bin folder has to be registered here too, or import fails
# with a generic "DLL load failed" error that doesn't name the missing
# dependency.
#
# EDIT THIS to match your local OpenCV install location if it differs.
OPENCV_BIN_DIR = r"C:\opencv\build\x64\vc16\bin"


def _register_dll_directories() -> None:
    """Register every folder DroneBackend.pyd needs its dependencies
    resolved from. Safe to call on any platform/Python version -- no-ops
    if os.add_dll_directory doesn't exist (non-Windows / old Python) or if
    a given folder doesn't exist on this machine."""
    if not hasattr(os, "add_dll_directory"):
        return

    for candidate in (backend_path, OPENCV_BIN_DIR):
        if not os.path.isdir(candidate):
            print(f"[DroneBackend] DLL directory notice: "
                  f"'{candidate}' does not exist, skipping")
            continue
        try:
            os.add_dll_directory(candidate)
        except Exception as e:
            print(f"[DroneBackend] DLL directory notice: {e}")


_register_dll_directories()

import DroneBackend

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
                }
            },
            ...
          }
        }

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
                print(f"[LayoutStore] migrating legacy layout file "
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
            print(f"[LayoutStore] saved → {self._path}")
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

        self.hub = DroneBackend.DroneLink()
        self._update_job = None

        # Yaw trim is now owned here on the Tk side; the worker gets a copy
        # via set_yaw_trim() whenever it changes.
        self._yaw_trim = 0.0

        self._locked_ref  = [False]
        self._vis_vars: dict[str, tk.BooleanVar] = {}
        self._panels:   dict[str, DraggablePanel] = {}

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
        self._worker = TelemetryWorker(self.hub)
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

        self._auto_connect()   # connects hub, then starts worker + pump

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
    # Layout persistence
    # =========================================================================

    def _default_snapshot(self) -> dict:
        """Factory-default layout, used to seed the very first profile."""
        return {
            "geometry": None,
            "locked": False,
            "panels": {name: dict(geo) for name, geo in _DEFAULT_PANELS.items()},
        }

    def _current_snapshot(self) -> dict:
        """Capture the on-screen state (geometry, lock, all panel rects)."""
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
            print(f"[Viewport] clamping '{name}' from ({x},{y}) → ({new_x},{new_y})")
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

        info_btn = self._mk_icon_btn(
            toolbar, "ℹ", None,
            "Drag a panel's title bar to move it.\n"
            "Click any panel to bring it to front.\n"
            "Right-click a title bar for snap-to and z-order options.",
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

    def _auto_connect(self) -> None:
        port = DroneBackend.auto_detect_f405()
        if port == "NOT_FOUND":
            self.status_label.config(
                text="Status: Searching for drone on COM ports...", fg="orange")
            self.root.after(RECONNECT_MS, self._auto_connect)
            return
        if self.hub.connect(port):
            self.status_label.config(
                text=f"Status: Connected — {port}", fg="#00ff88")
            # Start background worker and Tk pump only after a real connection
            self._worker.start()
            self._schedule_update()
        else:
            self.status_label.config(
                text=f"Status: Found {port} but connection failed (port busy?)",
                fg="red")
            self.root.after(RECONNECT_MS, self._auto_connect)

    def _reconnect(self) -> None:
        """
        Called when the Tk pump detects the worker has lost the connection.
        Stops the worker cleanly, disconnects the hub, then retries.
        """
        self._worker.stop()
        self.hub.disconnect()
        self.status_label.config(
            text="Status: Link lost — reconnecting...", fg="red")
        # Re-create a fresh worker so its internal state is clean
        self._worker = TelemetryWorker(self.hub)
        self._worker.set_yaw_trim(self._yaw_trim)
        self.root.after(RECONNECT_MS, self._auto_connect)

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

        # ── Connection health check ───────────────────────────────────────────
        # The worker monitors the backend independently; we just check its flag.
        if not self._worker.is_connected:
            # Only show "degraded" if the hub thinks it's still up but the
            # worker hasn't received valid data recently.
            if not self.hub.is_connected():
                self._reconnect()
                return
            # Hub says connected but worker hasn't delivered a frame yet —
            # this is normal during the first few ms after connect.
            self.status_label.config(text="Status: Waiting for data...",
                                     fg="orange")
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)
            return

        # ── Drain queue — get the freshest frame only ─────────────────────────
        ui_data = self._worker.get_frame()
        if ui_data is None:
            # Worker is alive but no new frame this tick — reschedule and yield
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)
            return

        # ── Update status label from frame data ───────────────────────────────
        if ui_data.get("link_healthy", True):
            self.status_label.config(text="Status: Connected", fg="#00ff88")
        else:
            self.status_label.config(text="Status: Link degraded", fg="orange")

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
        self._worker.stop()
        self.hub.disconnect()
        self._video_worker.stop()
        self.fpv_view.detach()
        self.video_link.disconnect()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app  = DroneCockpitApp(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()