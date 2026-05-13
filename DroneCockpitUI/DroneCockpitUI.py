"""
main.py  —  Drone Control Ground Station
=========================================
Layout redesign: two nested tk.PanedWindows replace the old fixed grid so the
window no longer exceeds screen height.

  ┌─ toolbar (status + 🔒 Save Layout) ──────────────────────────────────┐
  │  h_paned (horizontal, resizable)                                      │
  │  ┌─ v_paned (vertical, left) ──────┬─ right frame ──────────────────┐│
  │  │  left_top (IMU + Mag)           │  Primary Flight Display         ││
  │  ├──────────────────────────────── │  (Drone3DView + BaroWidget)     ││
  │  │  GPSWidget (Nav | Sat | Map)    │                                  ││
  │  └─────────────────────────────────┴────────────────────────────────┘│
  └───────────────────────────────────────────────────────────────────────┘

Lock / Save Layout button:
  • Click once  → saves current sash positions + window geometry to
                  cockpit_layout.json and LOCKS the sashes (sash drag
                  is blocked while locked).
  • Click again → UNLOCKS sashes so you can resize freely.
  • On every app start the saved layout is restored automatically.

Satellite data:
  The update loop tries to read state.gps.sv_list from the backend.
  If the C++ side exposes it (each item has gnss_id, sv_id, cno, used,
  quality attributes) the Satellites tab in GPSWidget will populate.
  Add TODO: expose sv_list in DroneBackend.DroneLink bindings.
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
UI_REFRESH_MS  = 20       # 50 Hz display refresh
RECONNECT_MS   = 2000     # retry interval when searching for drone
BG_COLOR       = "#f0f0f0"

_LAYOUT_FILE = Path(script_dir) / "cockpit_layout.json"
_DEFAULT_LAYOUT: dict = {
    "geometry": None,   # e.g. "1300x780+100+50"
    "h_sash":   440,    # horizontal sash x-position (left pane width)
    "v_sash":   380,    # vertical sash y-position   (top pane height)
    "locked":   False,
}


# ── Application ───────────────────────────────────────────────────────────────

class DroneCockpitApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Drone Control Ground Station")
        self.root.configure(bg=BG_COLOR)
        self.root.resizable(True, True)

        self.hub = DroneBackend.DroneLink()
        self._update_job = None

        # ── Heading trim — single source of truth ─────────────────────────────
        self._yaw_trim         = 0.0
        self._last_raw_yaw     = 0.0
        self._last_mag_heading = 0.0
        self._last_mag_valid   = False

        # ── Layout state ──────────────────────────────────────────────────────
        self._layout_locked = False
        self._layout        = self._load_layout()

        self._setup_ui()
        self._apply_layout()   # geometry + sashes + lock state
        self._auto_connect()

    # =========================================================================
    # Layout persistence
    # =========================================================================

    def _load_layout(self) -> dict:
        try:
            with open(_LAYOUT_FILE) as fh:
                saved = json.load(fh)
            return {**_DEFAULT_LAYOUT, **saved}
        except Exception:
            return _DEFAULT_LAYOUT.copy()

    def _save_layout(self) -> None:
        try:
            h_sash = self._h_paned.sash_coord(0)[0]
        except Exception:
            h_sash = _DEFAULT_LAYOUT["h_sash"]
        try:
            v_sash = self._v_paned.sash_coord(0)[1]
        except Exception:
            v_sash = _DEFAULT_LAYOUT["v_sash"]

        layout = {
            "geometry": self.root.geometry(),
            "h_sash":   int(h_sash),
            "v_sash":   int(v_sash),
            "locked":   self._layout_locked,
        }
        try:
            with open(_LAYOUT_FILE, "w") as fh:
                json.dump(layout, fh, indent=2)
            print(f"[Layout] saved → {_LAYOUT_FILE}")
        except Exception as e:
            print(f"[Layout] save failed: {e}")

    def _apply_layout(self) -> None:
        """Restore window geometry and sash positions from saved layout."""
        geom = self._layout.get("geometry")
        if geom:
            try:
                self.root.geometry(geom)
            except Exception:
                pass
        else:
            # First run: fit to content but cap at screen size
            self.root.update_idletasks()
            w  = self.root.winfo_reqwidth()  + 32
            h  = min(self.root.winfo_reqheight() + 32,
                     self.root.winfo_screenheight() - 80)
            sw = self.root.winfo_screenwidth()
            sh = self.root.winfo_screenheight()
            self.root.geometry(f"{w}x{h}+{(sw - w) // 2}+{(sh - h) // 2}")

        # Apply lock state from saved layout
        if self._layout.get("locked", False):
            self._layout_locked = True
            self._lock_btn.config(text="🔒 Locked", relief="sunken",
                                  bg="#c8c8c8")
            self._apply_sash_lock()

        # Sash positions must be applied after the window has rendered
        self.root.after(200, self._apply_sash_positions)

    def _apply_sash_positions(self) -> None:
        h = int(self._layout.get("h_sash", _DEFAULT_LAYOUT["h_sash"]))
        v = int(self._layout.get("v_sash", _DEFAULT_LAYOUT["v_sash"]))
        try:
            self._h_paned.sash_place(0, h, 0)
        except Exception:
            pass
        try:
            self._v_paned.sash_place(0, 0, v)
        except Exception:
            pass

    # ── Lock / Unlock ─────────────────────────────────────────────────────────

    def _toggle_lock(self) -> None:
        self._layout_locked = not self._layout_locked
        if self._layout_locked:
            self._lock_btn.config(text="🔒 Locked",      relief="sunken",  bg="#c8c8c8")
        else:
            self._lock_btn.config(text="🔓 Save Layout", relief="raised",  bg="#e8e8e8")
        self._apply_sash_lock()
        self._save_layout()

    def _apply_sash_lock(self) -> None:
        """Bind or unbind the sash-drag block on both PanedWindows."""
        for pw in (self._h_paned, self._v_paned):
            if self._layout_locked:
                pw.bind("<B1-Motion>", self._block_sash_drag)
            else:
                try:
                    pw.unbind("<B1-Motion>")
                except Exception:
                    pass

    @staticmethod
    def _block_sash_drag(event) -> str:
        """Return 'break' to cancel the PanedWindow sash-drag class binding."""
        return "break"

    # =========================================================================
    # Heading trim
    # =========================================================================

    def _apply_heading_trim(self) -> None:
        if not self._last_mag_valid:
            return
        current_trimmed = (self._last_raw_yaw - self._yaw_trim + 360) % 360
        drift = (current_trimmed - self._last_mag_heading + 540) % 360 - 180
        self._yaw_trim = (self._yaw_trim + drift + 360) % 360
        print(f"[HeadingTrim] trim={self._yaw_trim:.1f}  "
              f"raw_yaw={self._last_raw_yaw:.1f}  "
              f"mag={self._last_mag_heading:.1f}  "
              f"corrected={(self._last_raw_yaw - self._yaw_trim + 360) % 360:.1f}")

    # =========================================================================
    # UI setup
    # =========================================================================

    def _setup_ui(self) -> None:

        # ── Toolbar ───────────────────────────────────────────────────────────
        toolbar = tk.Frame(self.root, bg=BG_COLOR, pady=5)
        toolbar.pack(side="top", fill="x", padx=10)

        self.status_label = tk.Label(
            toolbar,
            text="Status: Searching...",
            fg="orange", bg=BG_COLOR,
            font=("Arial", 10, "bold"),
        )
        self.status_label.pack(side="left")

        self._lock_btn = tk.Button(
            toolbar,
            text="🔓 Save Layout",
            command=self._toggle_lock,
            font=("Arial", 9, "bold"),
            relief="raised", padx=8, pady=3,
            bg="#e8e8e8", activebackground="#d0d0d0",
            cursor="hand2",
        )
        self._lock_btn.pack(side="right")

        tk.Label(toolbar,
                 text="Drag dividers to resize  •  ",
                 fg="#999", bg=BG_COLOR,
                 font=("Arial", 8)).pack(side="right")

        # ── Horizontal PanedWindow: [left widgets]  |  [PFD] ─────────────────
        self._h_paned = tk.PanedWindow(
            self.root,
            orient=tk.HORIZONTAL,
            sashwidth=7, sashrelief="raised",
            bg=BG_COLOR, bd=0,
        )
        self._h_paned.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # ── Left side: Vertical PanedWindow: [IMU+Mag]  /  [GPS] ─────────────
        self._v_paned = tk.PanedWindow(
            self._h_paned,
            orient=tk.VERTICAL,
            sashwidth=7, sashrelief="raised",
            bg=BG_COLOR, bd=0,
        )

        # Top-left pane — IMU widget + Magnetometer stacked
        left_top = tk.Frame(self._v_paned, bg=BG_COLOR)

        self.imu_view = IMUWidget(
            left_top,
            on_adjust_heading=self._apply_heading_trim,
        )
        self.imu_view.pack(fill="x")

        self.mag_view = MagWidget(
            left_top,
            on_mag_calibrate=self.hub.start_mag_calibration,
            on_acc_calibrate=self.hub.start_acc_calibration,
        )
        self.mag_view.pack(fill="x", pady=(8, 0))

        self._v_paned.add(left_top, minsize=60, stretch="never")

        # Bottom-left pane — GPS (Navigation | Satellites | Map)
        self.gps_view = GPSWidget(self._v_paned)
        self._v_paned.add(self.gps_view, minsize=60, stretch="always")

        self._h_paned.add(self._v_paned, minsize=220, stretch="never")

        # ── Right side — Primary Flight Display ───────────────────────────────
        right_frame = tk.Frame(self._h_paned, bg=BG_COLOR)

        pfd = ttk.LabelFrame(right_frame, text="Primary Flight Display")
        pfd.pack(fill="both", expand=True)

        adi_frame = tk.Frame(pfd, bg=BG_COLOR)
        adi_frame.pack(side="left", padx=(8, 4), pady=8)

        self.drone_3d = Drone3DView(adi_frame, width=420, height=340)
        self.drone_3d.pack()

        self.baro_view = BaroWidget(pfd)
        self.baro_view.pack(side="left", padx=(4, 8), pady=8, anchor="n")

        self._h_paned.add(right_frame, minsize=380, stretch="always")

    # =========================================================================
    # Connection
    # =========================================================================

    def _auto_connect(self) -> None:
        port = DroneBackend.auto_detect_f405()
        if port == "NOT_FOUND":
            self.status_label.config(
                text="Status: Searching for drone on COM ports...", fg="orange"
            )
            self.root.after(RECONNECT_MS, self._auto_connect)
            return
        if self.hub.connect(port):
            self.status_label.config(
                text=f"Status: Connected on {port}", fg="green"
            )
            self._schedule_update()
        else:
            self.status_label.config(
                text=f"Status: Found {port} but connection failed (port busy?)",
                fg="red",
            )
            self.root.after(RECONNECT_MS, self._auto_connect)

    def _reconnect(self) -> None:
        self.hub.disconnect()
        self.status_label.config(
            text="Status: Link lost — reconnecting...", fg="red"
        )
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
                self.status_label.config(text="Status: Connected",      fg="green")

            # ── Apply heading trim ────────────────────────────────────────────
            raw_yaw   = float(state.yaw)
            mag_hdg   = getattr(state, "mag_heading_deg", 0.0)
            mag_valid = getattr(state, "mag_valid",        False)

            self._last_raw_yaw     = raw_yaw
            self._last_mag_heading = mag_hdg
            self._last_mag_valid   = mag_valid

            trimmed_yaw = (raw_yaw - self._yaw_trim + 360) % 360

            # ── GPS fields ────────────────────────────────────────────────────
            gps = getattr(state, "gps", None)
            if gps is not None:
                gps_hdop_raw  = getattr(gps, "hdop", 9999)
                gps_hdop_real = gps_hdop_raw / 100.0 if gps_hdop_raw != 9999 else 99.0

                # ── Per-satellite data (optional — needs backend support) ──────
                # TODO: expose sv_list in your C++ DroneBackend bindings.
                # Each item should have: gnss_id (str), sv_id (int),
                #   cno (int, dB-Hz), used (bool), quality (int 0-7 or str).
                sv_list_raw = getattr(gps, "sv_list", None) or []
                sv_list = []
                for sv in sv_list_raw:
                    sv_list.append({
                        "gnss_id": getattr(sv, "gnss_id", "?"),
                        "sv_id":   getattr(sv, "sv_id",   0),
                        "cno":     getattr(sv, "cno",     0),
                        "used":    getattr(sv, "used",    False),
                        "quality": getattr(sv, "quality", 0),
                    })

                gps_data = {
                    "gps_fix_type":          getattr(gps, "fix_type",          0),
                    "gps_num_sat":           getattr(gps, "num_sat",            0),
                    "gps_hdop":              gps_hdop_real,
                    "gps_latitude":          getattr(gps, "latitude",           0.0),
                    "gps_longitude":         getattr(gps, "longitude",          0.0),
                    "gps_altitude_m":        float(getattr(gps, "altitude_m",   0)),
                    "gps_ground_speed_cms":  getattr(gps, "ground_speed_cms",   0),
                    "gps_ground_course":     getattr(gps, "ground_course",      0),
                    "gps_dist_to_home_m":    float(getattr(gps, "dist_to_home_m", 0)),
                    "gps_bearing_to_home":   getattr(gps, "bearing_to_home",    0),
                    "gps_heartbeat":         getattr(gps, "gps_heartbeat",      None),
                    "gps_raw_valid":         getattr(gps, "raw_valid",          False),
                    "gps_comp_valid":        getattr(gps, "comp_valid",         False),
                    "gps_position_usable":   getattr(gps, "position_usable",    False),
                    "gps_sv_list":           sv_list,  # per-satellite data
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
                # IMU raw counts
                "ax": state.ax, "ay": state.ay, "az": state.az,
                "gx": state.gx, "gy": state.gy, "gz": state.gz,
                # Attitude
                "roll":  state.roll  / 10.0,
                "pitch": state.pitch / 10.0,
                "yaw":   trimmed_yaw,
                # Link quality
                "voltage":     state.battery_voltage,
                "rssi":        state.rssi,
                "rtt_ms":      state.last_rtt_ms,
                "fc_cycle_ms": state.fc_cycle_ms,
                # Barometer
                "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
                "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
                "baro_valid":            getattr(state, "baro_valid",             False),
                # Magnetometer
                "mag_x":                     getattr(state, "mag_x",                     0),
                "mag_y":                     getattr(state, "mag_y",                     0),
                "mag_z":                     getattr(state, "mag_z",                     0),
                "mag_heading_deg":           mag_hdg,
                "mag_valid":                 mag_valid,
                "mag_cal_active":            getattr(state, "mag_cal_active",            False),
                "mag_cal_seconds_remaining": getattr(state, "mag_cal_seconds_remaining", 0),
                "acc_cal_active":            getattr(state, "acc_cal_active",            False),
                "acc_cal_seconds_remaining": getattr(state, "acc_cal_seconds_remaining", 0),
                # GPS
                **gps_data,
            }

            # Dispatch to all widgets
            self.imu_view.update_ui(ui_data)
            self.baro_view.update_baro(ui_data)
            self.drone_3d.update_orientation(
                roll        = ui_data["roll"],
                pitch       = ui_data["pitch"],
                yaw         = ui_data["yaw"],
                mag_heading = ui_data["mag_heading_deg"],
                mag_valid   = ui_data["mag_valid"],
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
        # Always persist the final window position/size on exit
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