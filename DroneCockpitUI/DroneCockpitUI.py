"""
main.py  —  Drone Control Ground Station
=========================================
Free-form floating instrument panel layout with responsive widgets
and panel visibility control.

Toolbar:
  [Status]  [Panels ▾]  [⟳ Reset Layout]  [🔓 Save & Lock Layout]

Revision notes
--------------
  • FCStatusWidget panel  — armed state, flight mode, sensor health,
                             CPU load, I²C errors, PID profile
  • ArmingWidget panel    — arming-disable flag checklist
  • Z-order control       — right-click title bar for Bring to Front /
                             Send to Back so buried panels are always reachable
  • Snap-alignment        — while dragging, ghost guides appear when a panel
                             edge aligns within SNAP_PX pixels of another.
  • Context menu          — right-click any title bar.
  • Viewport clamping     — panels always spawn inside the visible workspace.
                             Toggling a panel on re-centres it if it drifted
                             off-screen (e.g. old layout file, window resize).
"""

import sys
import os
import json
from pathlib import Path

import tkinter as tk

from IMUWidget      import IMUWidget
from Drone3DView    import Drone3DView
from BaroWidget     import BaroWidget
from MagWidget      import MagWidget
from GPSWidget      import GPSWidget
from FCStatusWidget import FCStatusWidget
from ArmingWidget   import ArmingWidget

# ── Path configuration ────────────────────────────────────────────────────────
script_dir   = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Debug'))
sys.path.append(backend_path)

if hasattr(os, 'add_dll_directory'):
    try:
        os.add_dll_directory(backend_path)
    except Exception as e:
        print(f"[DroneBackend] DLL directory notice: {e}")

import DroneBackend

# ── Constants ─────────────────────────────────────────────────────────────────
UI_REFRESH_MS  = 20
RECONNECT_MS   = 2000
BG_WORKSPACE   = "#1a1a2e"
PANEL_BG       = "#0f0f1a"
PANEL_TITLE_BG = "#16213e"
PANEL_TITLE_FG = "#00d4ff"
GRIP_COLOR     = "#2a2a4a"
GRIP_SIZE      = 8
MIN_PANEL_W    = 120
MIN_PANEL_H    = 60

SNAP_PX        = 14    # pixels within which snap guides fire
SNAP_COLOR     = "#00d4ff"
GUIDE_DASH     = (4, 3)

_LAYOUT_FILE = Path(script_dir) / "cockpit_layout.json"

# ── Default panel layout (x, y, w, h) ────────────────────────────────────────
# All positions use small, safe coordinates so every panel is guaranteed to be
# inside even a 1024×768 workspace on first launch.  The pilot can drag them
# wherever they like and save the layout.
#
# Row 1 (y=10):  IMU | ADI | BARO
# Row 2 (y=280): MAG
# Row 3 (y=500): GPS
# Row 4 (y=500): FC_STATUS | ARMING  — stacked below GPS on the right
#
# NOTE: fc_status and arming intentionally start at modest x/y values so they
# are never off-screen on a fresh install or after the layout file is deleted.
_DEFAULT_PANELS = {
    "imu":       {"x":  10, "y":  10, "w": 460, "h": 260, "visible": True},
    "mag":       {"x":  10, "y": 280, "w": 460, "h": 210, "visible": True},
    "adi":       {"x": 480, "y":  10, "w": 450, "h": 430, "visible": True},
    "baro":      {"x": 940, "y":  10, "w": 220, "h": 430, "visible": True},
    "gps":       {"x":  10, "y": 500, "w": 700, "h": 300, "visible": True},
    "fc_status": {"x":  10, "y":  10, "w": 440, "h": 140, "visible": True},
    "arming":    {"x":  10, "y": 160, "w": 440, "h": 300, "visible": True},
}

_PANEL_LABELS = {
    "imu":       "IMU / Attitude",
    "mag":       "Magnetometer",
    "adi":       "ADI — Attitude Indicator",
    "baro":      "ALT / VSI — Altitude & Speed",
    "gps":       "GPS Navigation",
    "fc_status": "FC Status — Armed / Mode / Sensors",
    "arming":    "Arming Diagnostics",
}


# ── DraggablePanel ─────────────────────────────────────────────────────────────

class DraggablePanel(tk.Frame):
    """
    Floating, draggable, resizable instrument panel on a Canvas workspace.
    """

    def __init__(self, workspace: tk.Canvas, name: str, title: str,
                 x: int, y: int, w: int, h: int,
                 locked_ref: list,
                 all_panels_ref: dict,
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

        self.bind("<ButtonPress-1>", lambda e: self.raise_panel())
        self._title_bar.bind("<ButtonPress-1>",
                             lambda e: (self.raise_panel(), None))

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
    # Z-order
    # =========================================================================

    def raise_panel(self):
        self._ws.tag_raise(self._item)

    def lower_panel(self):
        self._ws.tag_lower(self._item)
        self._ws.tag_raise("grid")

    def _z_btn_click(self, event):
        if not hasattr(self, "_z_toggle"):
            self._z_toggle = False
        self._z_toggle = not self._z_toggle
        if self._z_toggle:
            self.raise_panel()
            self._z_btn.config(fg=PANEL_TITLE_FG)
        else:
            self.lower_panel()
            self._z_btn.config(fg="#2a4a6a")

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
        self.raise_panel()   # always bring to front when made visible

    def hide(self):
        self._ws.itemconfigure(self._item, state="hidden")


# ── Application ───────────────────────────────────────────────────────────────

class DroneCockpitApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Drone Control Ground Station")
        self.root.configure(bg="#0d0d1a")
        self.root.resizable(True, True)

        self.hub = DroneBackend.DroneLink()
        self._update_job = None

        self._yaw_trim         = 0.0
        self._last_raw_yaw     = 0.0
        self._last_mag_heading = 0.0
        self._last_mag_valid   = False

        self._sv_debug_frames = 0
        self._sv_debug_limit  = 10

        self._locked_ref  = [False]
        self._vis_vars: dict[str, tk.BooleanVar] = {}
        self._panels:   dict[str, DraggablePanel] = {}

        self._layout = self._load_layout()
        self._setup_ui()
        self._apply_layout()
        self._auto_connect()

    # =========================================================================
    # Layout persistence
    # =========================================================================

    def _load_layout(self) -> dict:
        try:
            with open(_LAYOUT_FILE) as fh:
                return json.load(fh)
        except Exception:
            return {}

    def _save_layout(self) -> None:
        layout = {
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
        try:
            with open(_LAYOUT_FILE, "w") as fh:
                json.dump(layout, fh, indent=2)
            print(f"[Layout] saved → {_LAYOUT_FILE}")
        except Exception as e:
            print(f"[Layout] save failed: {e}")

    def _apply_layout(self) -> None:
        geom = self._layout.get("geometry")
        if geom:
            try:
                self.root.geometry(geom)
            except Exception:
                pass
        else:
            sw = self.root.winfo_screenwidth()
            sh = self.root.winfo_screenheight()
            self.root.geometry(f"{min(1400, sw-40)}x{min(960, sh-60)}+20+20")

        saved_panels = self._layout.get("panels", {})
        for name, panel in self._panels.items():
            geo = saved_panels.get(name)
            # ── KEY FIX: only trust saved geo if it belongs to a panel that
            # existed when the layout was saved (i.e. it has geometry keys).
            # New panels (fc_status, arming) fall through to defaults when the
            # layout file pre-dates their introduction.
            if not geo or "x" not in geo:
                geo = _DEFAULT_PANELS.get(name, {"x": 10, "y": 10,
                                                  "w": 400, "h": 200,
                                                  "visible": True})
            panel.set_geometry(geo["x"], geo["y"], geo["w"], geo["h"])
            vis = geo.get("visible", True)
            self._vis_vars[name].set(vis)
            panel.show() if vis else panel.hide()

        if self._layout.get("locked", False):
            self._locked_ref[0] = True
            self._lock_btn.config(text="🔒 Locked",
                                  relief="sunken", bg="#1a2a1a", fg="#00ff88")

        # ── Clamp every panel into the visible workspace once Tk has painted ─
        # after_idle fires after the event loop processes pending geometry
        # negotiation, so winfo_width/height return real pixel values.
        self.root.after_idle(self._clamp_all_panels)

    # =========================================================================
    # Viewport clamping  ← NEW
    # =========================================================================

    def _clamp_all_panels(self) -> None:
        """Push every panel that sits outside the visible workspace back in."""
        for name in self._panels:
            self._clamp_panel(name)

    def _clamp_panel(self, name: str) -> None:
        """
        Ensure the named panel's top-left corner is within the workspace so
        the pilot can always see and grab the title bar.

        Aviation rationale: an instrument the pilot cannot see is an instrument
        that does not exist.  Every panel must spawn inside the cockpit.
        """
        self.root.update_idletasks()
        ws_w = self._workspace.winfo_width()
        ws_h = self._workspace.winfo_height()

        # If the workspace hasn't been drawn yet (e.g. called too early),
        # retry after the next idle cycle.
        if ws_w < 10 or ws_h < 10:
            self.root.after(100, lambda: self._clamp_panel(name))
            return

        panel = self._panels[name]
        x, y, w, h = panel.get_geometry()["x"], panel.get_geometry()["y"], \
                     panel.get_geometry()["w"], panel.get_geometry()["h"]

        # Title bar must be reachable: keep at least 60 px of width and the
        # full 26 px title bar height inside the workspace.
        MARGIN_X = 60    # px of panel width that must remain visible
        TITLE_H  = 26    # px — approximate title bar height

        new_x = max(0, min(x, ws_w - MARGIN_X))
        new_y = max(0, min(y, ws_h - TITLE_H))

        if new_x != x or new_y != y:
            print(f"[Viewport] clamping '{name}' from ({x},{y}) → ({new_x},{new_y})")
            panel.set_geometry(new_x, new_y, w, h)

    # =========================================================================
    # Lock / Unlock
    # =========================================================================

    def _toggle_lock(self) -> None:
        self._locked_ref[0] = not self._locked_ref[0]
        if self._locked_ref[0]:
            self._lock_btn.config(text="🔒 Locked",
                                  relief="sunken", bg="#1a2a1a", fg="#00ff88")
        else:
            self._lock_btn.config(text="🔓 Save & Lock Layout",
                                  relief="raised", bg="#162040", fg="#00d4ff")
        self._save_layout()

    def _reset_layout(self) -> None:
        if self._locked_ref[0]:
            return
        for name, panel in self._panels.items():
            geo = _DEFAULT_PANELS.get(name, {})
            if geo:
                panel.set_geometry(geo["x"], geo["y"], geo["w"], geo["h"])
        # After reset, clamp again in case screen is smaller than defaults
        self.root.after_idle(self._clamp_all_panels)

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
            # ── KEY FIX: clamp into view whenever a panel is turned on ───────
            # The panel may have been created with default coords that were
            # off-screen, or the window may have been resized since last save.
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
        if not self._last_mag_valid:
            return
        current_trimmed = (self._last_raw_yaw - self._yaw_trim + 360) % 360
        drift = (current_trimmed - self._last_mag_heading + 540) % 360 - 180
        self._yaw_trim = (self._yaw_trim + drift + 360) % 360

    # =========================================================================
    # UI setup
    # =========================================================================

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

        self._panels_btn = tk.Button(
            toolbar, text="Panels ▾",
            command=self._show_panels_menu,
            font=("Consolas", 9, "bold"),
            relief="raised", padx=10, pady=4,
            bg="#162040", fg="#a0b8d8",
            activebackground="#1e3060", activeforeground="#ffffff",
            cursor="hand2", bd=1,
        )
        self._panels_btn.pack(side="left", padx=(0, 6))

        tk.Label(
            toolbar,
            text="Drag title bars to move  •  Right-click title for snap & z-order",
            fg="#446688", bg="#0d0d1a",
            font=("Consolas", 8),
        ).pack(side="left")

        self._lock_btn = tk.Button(
            toolbar, text="🔓 Save & Lock Layout",
            command=self._toggle_lock,
            font=("Consolas", 9, "bold"),
            relief="raised", padx=10, pady=4,
            bg="#162040", fg="#00d4ff",
            activebackground="#1e3060", activeforeground="#00ffff",
            cursor="hand2", bd=1,
        )
        self._lock_btn.pack(side="right", padx=(6, 0))

        tk.Button(
            toolbar, text="⟳ Reset Layout",
            command=self._reset_layout,
            font=("Consolas", 9),
            relief="raised", padx=8, pady=4,
            bg="#1a1a2e", fg="#8899aa",
            activebackground="#252540",
            cursor="hand2", bd=1,
        ).pack(side="right", padx=(6, 0))

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
            self._sv_debug_frames = 0
            self._schedule_update()
        else:
            self.status_label.config(
                text=f"Status: Found {port} but connection failed (port busy?)",
                fg="red")
            self.root.after(RECONNECT_MS, self._auto_connect)

    def _reconnect(self) -> None:
        self.hub.disconnect()
        self.status_label.config(
            text="Status: Link lost — reconnecting...", fg="red")
        self.root.after(RECONNECT_MS, self._auto_connect)

    # =========================================================================
    # Update loop
    # =========================================================================

    def _schedule_update(self) -> None:
        if self._update_job is None:
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    def _update_loop(self) -> None:
        self._update_job = None

        if not self.hub.is_connected():
            self._reconnect()
            return

        try:
            state = self.hub.get_latest_state()

            if not state.link_healthy:
                self.status_label.config(text="Status: Link degraded", fg="orange")
            else:
                self.status_label.config(text="Status: Connected", fg="#00ff88")

            raw_yaw   = float(state.yaw)
            mag_hdg   = getattr(state, "mag_heading_deg", 0.0)
            mag_valid = getattr(state, "mag_valid", False)

            self._last_raw_yaw     = raw_yaw
            self._last_mag_heading = mag_hdg
            self._last_mag_valid   = mag_valid

            trimmed_yaw = (raw_yaw - self._yaw_trim + 360) % 360

            # ── GPS sub-object ────────────────────────────────────────────────
            gps = getattr(state, "gps", None)
            if gps is not None:
                gps_hdop_raw  = getattr(gps, "hdop", 9999)
                gps_hdop_real = (gps_hdop_raw / 100.0
                                 if gps_hdop_raw != 9999 else 99.0)

                sv_list_raw   = list(state.sv_list)
                sv_info_valid = getattr(state, "sv_info_valid", False)
                sv_source     = getattr(state, "sv_source", "")

                if self._sv_debug_frames < self._sv_debug_limit:
                    self._sv_debug_frames += 1
                    print(
                        f"[GPS-SV] frame={self._sv_debug_frames}"
                        f"  sv_info_valid={sv_info_valid}"
                        f"  sv_source={sv_source!r}"
                        f"  sv_count={len(sv_list_raw)}"
                    )
                    for sv in sv_list_raw:
                        print(
                            f"         gnss_name="
                            f"{getattr(sv,'gnss_name','?'):8s}"
                            f"  svid={getattr(sv,'svid',0):3d}"
                            f"  cno={getattr(sv,'cno',0):3d}"
                            f"  used={getattr(sv,'used',False)!s:5s}"
                            f"  quality={getattr(sv,'quality',0)}"
                            f"  status={getattr(sv,'status_str','idle')}"
                        )

                sv_list = [
                    {
                        "gnss_id": getattr(sv, "gnss_name",  "?"),
                        "sv_id":   getattr(sv, "svid",        0),
                        "cno":     getattr(sv, "cno",         0),
                        "used":    getattr(sv, "used",        False),
                        "quality": getattr(sv, "quality",     0),
                        "status":  getattr(sv, "status_str",  "idle"),
                        "elev":    getattr(sv, "elev",        0),
                        "azim":    getattr(sv, "azim",        0),
                    }
                    for sv in sv_list_raw
                ]

                heartbeat = getattr(gps, "heartbeat",
                            getattr(gps, "gps_heartbeat", None))

                gps_data = {
                    "gps_fix_type":         getattr(gps, "fix_type",         0),
                    "gps_num_sat":          getattr(gps, "num_sat",           0),
                    "gps_hdop":             gps_hdop_real,
                    "gps_latitude":         getattr(gps, "latitude",          0.0),
                    "gps_longitude":        getattr(gps, "longitude",         0.0),
                    "gps_altitude_m":       float(getattr(gps, "altitude_m",  0)),
                    "gps_ground_speed_cms": getattr(gps, "ground_speed_cms",  0),
                    "gps_ground_course":    getattr(gps, "ground_course",     0),
                    "gps_dist_to_home_m":   float(getattr(gps, "dist_to_home_m", 0)),
                    "gps_bearing_to_home":  getattr(gps, "bearing_to_home",  0),
                    "gps_heartbeat":        heartbeat,
                    "gps_raw_valid":        getattr(gps, "raw_valid",         False),
                    "gps_comp_valid":       getattr(gps, "comp_valid",        False),
                    "gps_position_usable":  getattr(gps, "position_usable",   False),
                    "gps_sv_list":          sv_list,
                    "gps_sv_info_valid":    sv_info_valid,
                    "gps_sv_source":        sv_source,
                }
            else:
                gps_data = {
                    "gps_fix_type": 0, "gps_num_sat": 0, "gps_hdop": 99.0,
                    "gps_latitude": 0.0, "gps_longitude": 0.0,
                    "gps_altitude_m": 0.0, "gps_ground_speed_cms": 0,
                    "gps_ground_course": 0, "gps_dist_to_home_m": 0.0,
                    "gps_bearing_to_home": 0, "gps_heartbeat": None,
                    "gps_raw_valid": False, "gps_comp_valid": False,
                    "gps_position_usable": False,
                    "gps_sv_list": [], "gps_sv_info_valid": False,
                    "gps_sv_source": "",
                }

            # ── Assemble full telemetry dict ──────────────────────────────────
            ui_data = {
                "ax": state.ax, "ay": state.ay, "az": state.az,
                "gx": state.gx, "gy": state.gy, "gz": state.gz,
                "roll":  state.roll  / 10.0,
                "pitch": state.pitch / 10.0,
                "yaw":   trimmed_yaw,
                "voltage":          state.battery_voltage,
                "rssi":             state.rssi,
                "rtt_ms":           state.last_rtt_ms,
                "fc_cycle_ms":      state.fc_cycle_ms,
                "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
                "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
                "baro_valid":            getattr(state, "baro_valid",             False),
                "mag_x":                     getattr(state, "mag_x",             0),
                "mag_y":                     getattr(state, "mag_y",             0),
                "mag_z":                     getattr(state, "mag_z",             0),
                "mag_heading_deg":           mag_hdg,
                "mag_valid":                 mag_valid,
                "mag_cal_active":            getattr(state, "mag_cal_active",            False),
                "mag_cal_seconds_remaining": getattr(state, "mag_cal_seconds_remaining", 0),
                "acc_cal_active":            getattr(state, "acc_cal_active",            False),
                "acc_cal_seconds_remaining": getattr(state, "acc_cal_seconds_remaining", 0),
                "armed":                  getattr(state, "armed",             False),
                "flight_mode_flags":      getattr(state, "flight_mode_flags", 0),
                "flight_mode_name":       getattr(state, "flight_mode_name",  "ACRO [DISARMED]"),
                "sensor_status":          getattr(state, "sensor_status",     0),
                "sensor_acc_present":     bool(getattr(state, "sensor_status", 0) & (1 << 0)),
                "sensor_baro_present":    bool(getattr(state, "sensor_status", 0) & (1 << 1)),
                "sensor_mag_present":     bool(getattr(state, "sensor_status", 0) & (1 << 2)),
                "sensor_gps_present":     bool(getattr(state, "sensor_status", 0) & (1 << 3)),
                "sensor_rangefinder_present": bool(getattr(state, "sensor_status", 0) & (1 << 4)),
                "sensor_gyro_present":    bool(getattr(state, "sensor_status", 0) & (1 << 5)),
                "i2c_error_count":        getattr(state, "i2c_error_count",   0),
                "cpu_load_percent":       getattr(state, "cpu_load_percent",  0),
                "pid_profile":            getattr(state, "pid_profile",       0),
                "arming_disable_flags":   getattr(state, "arming_disable_flags", 0),
                "arming_disable_str":     getattr(state, "arming_disable_str",   ""),
                **gps_data,
            }

            self.imu_view.update_ui(ui_data)
            self.baro_view.update_baro(ui_data)
            self.drone_3d.update_orientation(
                roll=ui_data["roll"], pitch=ui_data["pitch"],
                yaw=ui_data["yaw"],
                mag_heading=ui_data["mag_heading_deg"],
                mag_valid=ui_data["mag_valid"],
            )
            self.mag_view.update_mag(ui_data)
            self.gps_view.update_gps(ui_data)
            self.fc_status_view.update_fc_status(ui_data)
            self.arming_view.update_arming(ui_data)

        except Exception as e:
            print(f"[update_loop] {e}")
            self._reconnect()
            return

        self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    # =========================================================================
    # Clean shutdown
    # =========================================================================

    def shutdown(self) -> None:
        self._save_layout()
        if self._update_job is not None:
            self.root.after_cancel(self._update_job)
        self.hub.disconnect()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app  = DroneCockpitApp(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()