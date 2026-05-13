"""
main.py  —  Drone Control Ground Station
=========================================
Free-form floating instrument panel layout with responsive widgets
and panel visibility control.

Toolbar:
  [Status]  [Panels ▾]  [⟳ Reset Layout]  [🔓 Save & Lock Layout]

"Panels ▾" opens a checkbutton menu — uncheck any instrument to hide it
from the workspace without destroying it.  Data keeps flowing so the
display is instantly current when you re-enable a panel.
Visibility is saved to cockpit_layout.json with position/size.

Panel resize behaviour:
  All widgets fill their content frame (fill="both" expand=True).
  BaroWidget now binds <Configure> on both canvases and redraws at the
  new pixel dimensions — so the altitude tape and VSI scale gracefully
  whether the panel is tall+narrow, wide+short, or anything in between.
"""

import sys
import os
import json
from pathlib import Path

import tkinter as tk
from tkinter import ttk

from IMUWidget   import IMUWidget
from Drone3DView import Drone3DView
from BaroWidget  import BaroWidget
from MagWidget   import MagWidget
from GPSWidget   import GPSWidget

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

_LAYOUT_FILE = Path(script_dir) / "cockpit_layout.json"

# Aviation-standard default layout (x, y, w, h)
_DEFAULT_PANELS = {
    "imu":  {"x":  10, "y":  10, "w": 460, "h": 260, "visible": True},
    "mag":  {"x":  10, "y": 280, "w": 460, "h": 210, "visible": True},
    "adi":  {"x": 480, "y":  10, "w": 450, "h": 430, "visible": True},
    "baro": {"x": 940, "y":  10, "w": 220, "h": 430, "visible": True},
    "gps":  {"x":  10, "y": 500, "w":1150, "h": 320, "visible": True},
}

_PANEL_LABELS = {
    "imu":  "IMU / Attitude",
    "mag":  "Magnetometer",
    "adi":  "ADI — Attitude Indicator",
    "baro": "ALT / VSI — Altitude & Speed",
    "gps":  "GPS Navigation",
}


# ── DraggablePanel ─────────────────────────────────────────────────────────────

class DraggablePanel(tk.Frame):
    """Floating, draggable, resizable instrument panel on a Canvas workspace."""

    def __init__(self, workspace: tk.Canvas, title: str,
                 x: int, y: int, w: int, h: int,
                 locked_ref: list, **kwargs):
        super().__init__(workspace, bg=PANEL_BG,
                         highlightthickness=1,
                         highlightbackground="#2a3a5a",
                         **kwargs)

        self._ws         = workspace
        self._locked_ref = locked_ref
        self._drag_x     = 0
        self._drag_y     = 0

        self._item = workspace.create_window(x, y, anchor="nw",
                                             window=self, width=w, height=h)

        # Title bar
        self._title_bar = tk.Frame(self, bg=PANEL_TITLE_BG, cursor="fleur")
        self._title_bar.pack(fill="x", side="top")
        tk.Label(
            self._title_bar, text=f"  {title}",
            bg=PANEL_TITLE_BG, fg=PANEL_TITLE_FG,
            font=("Consolas", 9, "bold"), anchor="w",
        ).pack(side="left", fill="x", expand=True)

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

        # Bindings
        for w_ in (self._title_bar,) + tuple(self._title_bar.winfo_children()):
            w_.bind("<ButtonPress-1>", self._drag_start)
            w_.bind("<B1-Motion>",     self._drag_motion)

        self._grip.bind("<ButtonPress-1>",   self._resize_start)
        self._grip.bind("<B1-Motion>",       self._resize_both)
        self._right_edge.bind("<ButtonPress-1>",  self._resize_start)
        self._right_edge.bind("<B1-Motion>",      self._resize_h)
        self._bottom_edge.bind("<ButtonPress-1>", self._resize_start)
        self._bottom_edge.bind("<B1-Motion>",     self._resize_v)

    def _get_wh(self):
        return (int(float(self._ws.itemcget(self._item, "width"))),
                int(float(self._ws.itemcget(self._item, "height"))))

    def _set_wh(self, w, h):
        self._ws.itemconfigure(self._item,
                               width=max(MIN_PANEL_W, w),
                               height=max(MIN_PANEL_H, h))

    def _drag_start(self, event):
        if self._locked_ref[0]: return
        self._drag_x, self._drag_y = event.x_root, event.y_root

    def _drag_motion(self, event):
        if self._locked_ref[0]: return
        dx = event.x_root - self._drag_x
        dy = event.y_root - self._drag_y
        self._drag_x, self._drag_y = event.x_root, event.y_root
        cx, cy = self._ws.coords(self._item)
        self._ws.coords(self._item, cx + dx, cy + dy)

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

    def get_geometry(self) -> dict:
        x, y = self._ws.coords(self._item)
        w, h = self._get_wh()
        return {"x": int(x), "y": int(y), "w": w, "h": h}

    def set_geometry(self, x: int, y: int, w: int, h: int) -> None:
        self._ws.coords(self._item, x, y)
        self._set_wh(w, h)

    def show(self):
        self._ws.itemconfigure(self._item, state="normal")

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

        self._locked_ref = [False]
        self._vis_vars: dict[str, tk.BooleanVar] = {}

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
            self.root.geometry(f"{min(1400, sw-40)}x{min(860, sh-60)}+20+20")

        saved_panels = self._layout.get("panels", {})
        for name, panel in self._panels.items():
            geo = saved_panels.get(name, _DEFAULT_PANELS.get(name, {}))
            if geo:
                panel.set_geometry(geo["x"], geo["y"], geo["w"], geo["h"])
            vis = geo.get("visible", True) if geo else True
            self._vis_vars[name].set(vis)
            panel.show() if vis else panel.hide()

        if self._layout.get("locked", False):
            self._locked_ref[0] = True
            self._lock_btn.config(text="🔒 Locked",
                                  relief="sunken", bg="#1a2a1a", fg="#00ff88")

    # ── Lock / Unlock ─────────────────────────────────────────────────────────

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

    # =========================================================================
    # Panel visibility
    # =========================================================================

    def _show_panels_menu(self) -> None:
        menu = tk.Menu(self.root, tearoff=0,
                       bg="#0f1428", fg="#c0d0f0",
                       activebackground="#1e3060",
                       activeforeground="#ffffff",
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
        else:
            self._panels[name].hide()

    def _show_all(self):
        for name in self._panels:
            self._vis_vars[name].set(True)
            self._panels[name].show()

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

        # Toolbar
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
            text="Drag title bars to move  •  Drag edges/corners to resize",
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

        # Workspace canvas
        self._workspace = tk.Canvas(
            self.root, bg=BG_WORKSPACE, highlightthickness=0,
        )
        self._workspace.pack(fill="both", expand=True, padx=4, pady=(0, 4))
        self._workspace.bind("<Configure>", self._draw_grid)

        # Create panels
        self._panels: dict[str, DraggablePanel] = {}

        def _panel(name: str, title: str) -> DraggablePanel:
            self._vis_vars[name] = tk.BooleanVar(value=True)
            geo = _DEFAULT_PANELS[name]
            return DraggablePanel(
                self._workspace, title,
                x=geo["x"], y=geo["y"], w=geo["w"], h=geo["h"],
                locked_ref=self._locked_ref,
            )

        p = _panel("imu", "MPU-6500  —  IMU / Attitude")
        self.imu_view = IMUWidget(p.content,
                                  on_adjust_heading=self._apply_heading_trim)
        self.imu_view.pack(fill="both", expand=True)
        self._panels["imu"] = p

        p = _panel("mag", "QMC5883L  —  Magnetometer")
        self.mag_view = MagWidget(
            p.content,
            on_mag_calibrate=self.hub.start_mag_calibration,
            on_acc_calibrate=self.hub.start_acc_calibration,
        )
        self.mag_view.pack(fill="both", expand=True)
        self._panels["mag"] = p

        p = _panel("adi", "ADI  —  Attitude Direction Indicator")
        self.drone_3d = Drone3DView(p.content, width=420, height=340)
        self.drone_3d.pack(fill="both", expand=True)
        self._panels["adi"] = p

        p = _panel("baro", "ALT / VSI  —  Altitude & Vertical Speed")
        self.baro_view = BaroWidget(p.content)
        self.baro_view.pack(fill="both", expand=True)
        self._panels["baro"] = p

        p = _panel("gps", "GPS Navigation  —  Nav | Satellites | Map")
        self.gps_view = GPSWidget(p.content)
        self.gps_view.pack(fill="both", expand=True)
        self._panels["gps"] = p

    # ── Grid ──────────────────────────────────────────────────────────────────

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
            mag_valid = getattr(state, "mag_valid",        False)

            self._last_raw_yaw     = raw_yaw
            self._last_mag_heading = mag_hdg
            self._last_mag_valid   = mag_valid

            trimmed_yaw = (raw_yaw - self._yaw_trim + 360) % 360

            gps = getattr(state, "gps", None)
            if gps is not None:
                gps_hdop_raw  = getattr(gps, "hdop", 9999)
                gps_hdop_real = gps_hdop_raw / 100.0 if gps_hdop_raw != 9999 else 99.0
                sv_list_raw   = getattr(gps, "sv_list", None) or []
                sv_list = [
                    {"gnss_id": getattr(sv, "gnss_id", "?"),
                     "sv_id":   getattr(sv, "sv_id",   0),
                     "cno":     getattr(sv, "cno",     0),
                     "used":    getattr(sv, "used",    False),
                     "quality": getattr(sv, "quality", 0)}
                    for sv in sv_list_raw
                ]
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
                    "gps_bearing_to_home":  getattr(gps, "bearing_to_home",   0),
                    "gps_heartbeat":        getattr(gps, "gps_heartbeat",     None),
                    "gps_raw_valid":        getattr(gps, "raw_valid",         False),
                    "gps_comp_valid":       getattr(gps, "comp_valid",        False),
                    "gps_position_usable":  getattr(gps, "position_usable",   False),
                    "gps_sv_list":          sv_list,
                }
            else:
                gps_data = {
                    "gps_fix_type": 0, "gps_num_sat": 0, "gps_hdop": 99.0,
                    "gps_latitude": 0.0, "gps_longitude": 0.0,
                    "gps_altitude_m": 0.0, "gps_ground_speed_cms": 0,
                    "gps_ground_course": 0, "gps_dist_to_home_m": 0.0,
                    "gps_bearing_to_home": 0, "gps_heartbeat": None,
                    "gps_raw_valid": False, "gps_comp_valid": False,
                    "gps_position_usable": False, "gps_sv_list": [],
                }

            ui_data = {
                "ax": state.ax, "ay": state.ay, "az": state.az,
                "gx": state.gx, "gy": state.gy, "gz": state.gz,
                "roll":  state.roll  / 10.0,
                "pitch": state.pitch / 10.0,
                "yaw":   trimmed_yaw,
                "voltage":     state.battery_voltage,
                "rssi":        state.rssi,
                "rtt_ms":      state.last_rtt_ms,
                "fc_cycle_ms": state.fc_cycle_ms,
                "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
                "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
                "baro_valid":            getattr(state, "baro_valid",             False),
                "mag_x":                     getattr(state, "mag_x",                     0),
                "mag_y":                     getattr(state, "mag_y",                     0),
                "mag_z":                     getattr(state, "mag_z",                     0),
                "mag_heading_deg":           mag_hdg,
                "mag_valid":                 mag_valid,
                "mag_cal_active":            getattr(state, "mag_cal_active",            False),
                "mag_cal_seconds_remaining": getattr(state, "mag_cal_seconds_remaining", 0),
                "acc_cal_active":            getattr(state, "acc_cal_active",            False),
                "acc_cal_seconds_remaining": getattr(state, "acc_cal_seconds_remaining", 0),
                **gps_data,
            }

            # Always update all widgets — hidden panels stay current
            self.imu_view.update_ui(ui_data)
            self.baro_view.update_baro(ui_data)
            self.drone_3d.update_orientation(
                roll=ui_data["roll"], pitch=ui_data["pitch"], yaw=ui_data["yaw"],
                mag_heading=ui_data["mag_heading_deg"], mag_valid=ui_data["mag_valid"],
            )
            self.mag_view.update_mag(ui_data)
            self.gps_view.update_gps(ui_data)

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