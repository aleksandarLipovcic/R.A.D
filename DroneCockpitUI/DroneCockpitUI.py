import sys
import os
import ctypes
import tkinter as tk
from tkinter import ttk
from IMUWidget   import IMUWidget
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
            ctypes.windll.user32.SetWindowTextW(
                hwnd, "Drone Control Ground Station"
            )
        except Exception:
            pass

        self.root.configure(bg=BG_COLOR)
        # Allow Tk to resize the window to fit content, but not smaller
        self.root.resizable(True, True)

        self.hub = DroneBackend.DroneLink()
        self._update_job = None

        self._setup_ui()
        self._fit_window()       # auto-size after all widgets are built
        self._auto_connect()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _setup_ui(self):
        # ── Status bar ────────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg=BG_COLOR)
        top.pack(side="top", fill="x", pady=(8, 4))

        self.status_label = tk.Label(
            top,
            text="Status: Searching...",
            fg="orange", bg=BG_COLOR,
            font=("Arial", 10, "bold")
        )
        self.status_label.pack()

        # ── Main content frame ────────────────────────────────────────────────
        # Two columns side by side, anchored to top so rows don't stretch.
        main = tk.Frame(self.root, bg=BG_COLOR)
        main.pack(side="top", fill="both", expand=True, padx=16, pady=(4, 12))

        # ── Left column: IMU table ────────────────────────────────────────────
        # sticky="n" is critical — prevents this column from forcing the right
        # column to grow taller, which was stretching the ADI canvas.
        left = tk.Frame(main, bg=BG_COLOR)
        left.grid(row=0, column=0, sticky="n", padx=(0, 14))

        self.imu_view = IMUWidget(left)
        self.imu_view.grid(row=0, column=0, sticky="nsew")

        # ── Right column: Primary Flight Display ──────────────────────────────
        # Contains the ADI (left) and the altitude tape + VSI (right),
        # matching the layout of a real PFD (e.g. Garmin G1000).
        pfd = ttk.LabelFrame(main, text="Primary Flight Display")
        pfd.grid(row=0, column=1, sticky="n")

        # ADI canvas — fixed size, packed without fill/expand so it never
        # grows beyond the requested width/height.  Drone3DView reads its
        # actual pixel size via winfo_width/height on every frame so it will
        # still adapt if the user manually resizes the window.
        adi_frame = tk.Frame(pfd, bg=BG_COLOR)
        adi_frame.pack(side="left", padx=(8, 4), pady=8)

        self.drone_3d = Drone3DView(adi_frame, width=420, height=340)
        self.drone_3d.pack()   # no fill="both" — keeps the canvas at exact size

        # Altitude tape + VSI — immediately to the right of the ADI
        self.baro_view = BaroWidget(pfd)
        self.baro_view.pack(side="left", padx=(4, 8), pady=8, anchor="n")

    # ── Auto window sizing ────────────────────────────────────────────────────

    def _fit_window(self):
        """
        Size the window to exactly fit its content with a small margin.

        Two-pass approach:
          1. update_idletasks() — lets Tk calculate preferred widget sizes
          2. update()           — processes any pending layout events so
                                  winfo_reqwidth/height return final values
          3. geometry(WxH)      — lock the window to that size and centre it

        This eliminates the need for the user to manually drag the window
        corner after launch.
        """
        self.root.update_idletasks()
        self.root.update()

        w = self.root.winfo_reqwidth()  + 24
        h = self.root.winfo_reqheight() + 24

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        x  = (sw - w) // 2
        y  = (sh - h) // 2

        self.root.geometry(f"{w}x{h}+{x}+{y}")
        # Set minimum size = natural size so user can only make it larger
        self.root.minsize(w, h)

    # ── Connection ────────────────────────────────────────────────────────────

    def _auto_connect(self):
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
        self.hub.disconnect()
        self.status_label.config(
            text="Status: Link lost - reconnecting...", fg="red"
        )
        self.root.after(RECONNECT_MS, self._auto_connect)

    # ── Update loop ───────────────────────────────────────────────────────────

    def _schedule_update(self):
        if self._update_job is None:
            self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    def _update_loop(self):
        self._update_job = None

        if not self.hub.is_connected():
            self._reconnect()
            return

        try:
            state = self.hub.get_latest_state()

            if not state.link_healthy:
                self.status_label.config(text="Status: Link degraded",  fg="orange")
            else:
                self.status_label.config(text="Status: Connected",       fg="green")

            ui_data = {
                # IMU raw counts (divided by scale inside IMUWidget)
                "ax":          state.ax,
                "ay":          state.ay,
                "az":          state.az,
                "gx":          state.gx,
                "gy":          state.gy,
                "gz":          state.gz,
                # Attitude — FC sends degrees*10, divide here
                "roll":        state.roll  / 10.0,
                "pitch":       state.pitch / 10.0,
                "yaw":         float(state.yaw),
                # Link quality
                "voltage":     state.battery_voltage,
                "rssi":        state.rssi,
                "rtt_ms":      state.last_rtt_ms,
                "fc_cycle_ms": state.fc_cycle_ms,
                # Barometer — getattr fallback so GUI runs before C++ baro
                # integration is compiled (BaroWidget shows "NO SIG" safely)
                "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
                "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
                "baro_valid":            getattr(state, "baro_valid",             False),
            }

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

        self._update_job = self.root.after(UI_REFRESH_MS, self._update_loop)

    # ── Clean shutdown ────────────────────────────────────────────────────────

    def shutdown(self):
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