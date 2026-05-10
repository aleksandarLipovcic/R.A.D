import sys
import os
import ctypes
import tkinter as tk
from tkinter import ttk
from IMUWidget  import IMUWidget
from Drone3DView import Drone3DView
from BaroWidget  import BaroWidget

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
UI_REFRESH_MS = 20      # 50 Hz display refresh
RECONNECT_MS  = 2000    # retry interval when searching for drone
BG_COLOR      = "#f0f0f0"


class DroneCockpitApp:
    def __init__(self, root: tk.Tk):
        self.root = root

        # ── Window title ──────────────────────────────────────────────────────
        self.root.title("Drone Control Ground Station")
        try:
            hwnd = ctypes.windll.user32.GetParent(self.root.winfo_id())
            ctypes.windll.user32.SetWindowTextW(hwnd, "Drone Control Ground Station")
        except Exception as e:
            print(f"Windows API title force failed: {e}")

        self.root.configure(bg=BG_COLOR)

        # C++ threaded backend — serial I/O runs in its own thread
        self.hub = DroneBackend.DroneLink()

        # Track whether the update loop is already scheduled
        self._update_job = None

        self._setup_ui()
        self._center_window()
        self._auto_connect()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _setup_ui(self):
        # ── Status bar ────────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg=BG_COLOR)
        top.pack(side="top", fill="x", pady=10)

        self.status_label = tk.Label(
            top,
            text="Status: Searching...",
            fg="orange", bg=BG_COLOR,
            font=("Arial", 10, "bold")
        )
        self.status_label.pack()

        # ── Main content (2 columns) ──────────────────────────────────────────
        main = tk.Frame(self.root, bg=BG_COLOR)
        main.pack(fill="both", expand=True, padx=20, pady=10)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        # ── Left column: IMU table (top) + Baro table (bottom) ───────────────
        left = tk.Frame(main, bg=BG_COLOR)
        left.grid(row=0, column=0, sticky="nsew", padx=10)

        self.imu_view = IMUWidget(left)
        self.imu_view.grid(row=0, column=0, sticky="nsew", pady=(0, 10))

        self.baro_view = BaroWidget(left)
        self.baro_view.grid(row=1, column=0, sticky="nsew")

        # ── Right column: ADI / Spatial Orientation ───────────────────────────
        visual = ttk.LabelFrame(main, text="Spatial Orientation")
        visual.grid(row=0, column=1, sticky="nsew", padx=10)
        self.drone_3d = Drone3DView(visual, width=400, height=350)
        self.drone_3d.pack(padx=10, pady=10, expand=True, fill="both")

    def _center_window(self):
        self.root.update_idletasks()
        w = self.root.winfo_reqwidth()  + 20
        h = self.root.winfo_reqheight() + 20
        x = (self.root.winfo_screenwidth()  // 2) - (w // 2)
        y = (self.root.winfo_screenheight() // 2) - (h // 2)
        self.root.geometry(f"{w}x{h}+{x}+{y}")

    # ── Connection ────────────────────────────────────────────────────────────

    def _auto_connect(self):
        """Scan COM ports (via C++ helper) and connect when the drone is found."""
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
                fg="red"
            )
            self.root.after(RECONNECT_MS, self._auto_connect)

    def _reconnect(self):
        """Called when telemetry is lost mid-session."""
        self.hub.disconnect()
        self.status_label.config(
            text="Status: Link lost - reconnecting...", fg="red"
        )
        self.root.after(RECONNECT_MS, self._auto_connect)

    # ── Update loop ───────────────────────────────────────────────────────────

    def _schedule_update(self):
        """Kick off the 50 Hz UI refresh cycle (idempotent)."""
        if self._update_job is None:
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    def _update_loop(self):
        """
        Pull the latest snapshot from the C++ background thread and push it
        into all widgets.  Runs entirely on the Tk main thread — no locking
        needed here because get_latest_state() does the locking inside C++.
        """
        self._update_job = None   # reset so _schedule_update can re-arm

        if not self.hub.is_connected():
            self._reconnect()
            return

        try:
            state = self.hub.get_latest_state()

            # ── Link health indicator ─────────────────────────────────────────
            if not state.link_healthy:
                self.status_label.config(text="Status: Link degraded", fg="orange")
            else:
                self.status_label.config(text="Status: Connected", fg="green")

            # ── Build shared ui_data dict ─────────────────────────────────────
            # roll/pitch arrive from FC as degrees * 10 — divide to plain degrees
            ui_data = {
                # IMU raw counts (scaled inside IMUWidget)
                "ax":          state.ax,
                "ay":          state.ay,
                "az":          state.az,
                "gx":          state.gx,
                "gy":          state.gy,
                "gz":          state.gz,
                # Attitude
                "roll":        state.roll  / 10.0,
                "pitch":       state.pitch / 10.0,
                "yaw":         float(state.yaw),
                # Link quality
                "voltage":     state.battery_voltage,
                "rssi":        state.rssi,
                "rtt_ms":      state.last_rtt_ms,
                "fc_cycle_ms": state.fc_cycle_ms,
                # Barometer — BMP280 via MSP_ALTITUDE
                # getattr fallback allows the GUI to run even if the C++ baro
                # integration is not yet compiled (shows "NO SIGNAL" safely).
                "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
                "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
                "baro_valid":            getattr(state, "baro_valid",             False),
            }

            # ── Push to widgets ───────────────────────────────────────────────
            self.imu_view.update_ui(ui_data)
            self.baro_view.update_baro(ui_data)
            self.drone_3d.update_orientation(
                roll=ui_data["roll"],
                pitch=ui_data["pitch"],
                yaw=ui_data["yaw"],
            )

        except Exception as e:
            print(f"[update_loop] {e}")
            self._reconnect()
            return

        # Re-arm for the next tick
        self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    # ── Clean shutdown ────────────────────────────────────────────────────────

    def shutdown(self):
        """Stop the C++ worker thread before destroying the window."""
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