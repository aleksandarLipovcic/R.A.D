import sys
import os
import tkinter as tk
from tkinter import ttk
from IMUWidget   import IMUWidget
from Drone3DView import Drone3DView
from BaroWidget  import BaroWidget
from MagWidget   import MagWidget

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
        self.root.title("Drone Control Ground Station")
        self.root.configure(bg=BG_COLOR)
        self.root.resizable(True, True)

        self.hub = DroneBackend.DroneLink()
        self._update_job = None

        # ── Heading trim — single source of truth ─────────────────────────────
        # This offset is added to the raw FC gyro yaw before it reaches ANY
        # widget.  Because both IMUWidget and Drone3DView read "yaw" from the
        # same ui_data dict, they are always identical — there is no way for
        # them to disagree.
        #
        # _apply_heading_trim() is called by IMUWidget via the on_adjust_heading
        # callback.  It captures the current divergence (gyro − mag) and stores
        # it here.  On the very next 20 ms tick both widgets see the corrected
        # value simultaneously.
        #
        # _last_raw_yaw / _last_mag_heading are updated every tick so that when
        # the button fires between ticks it always has fresh values.
        self._yaw_trim         = 0.0   # degrees — subtracted from raw FC yaw
        self._last_raw_yaw     = 0.0
        self._last_mag_heading = 0.0
        self._last_mag_valid   = False

        self._setup_ui()
        self._fit_window()
        self._auto_connect()

    # ── Heading trim callback ─────────────────────────────────────────────────

    def _apply_heading_trim(self):
        """
        Called by IMUWidget when the pilot presses "Adjust Heading".

        Computes the signed shortest-path difference between the current raw
        gyro yaw and the magnetometer heading, and accumulates it into
        _yaw_trim.  After this call the trimmed yaw equals the mag heading
        at the moment of pressing — and BOTH widgets see the same value on
        the very next telemetry tick because the trim is applied in one place
        before ui_data is built.
        """
        if not self._last_mag_valid:
            return   # no mag reference — do nothing

        # How far is the (already trimmed) gyro yaw from the mag heading?
        current_trimmed = (self._last_raw_yaw - self._yaw_trim + 360) % 360
        drift = (current_trimmed - self._last_mag_heading + 540) % 360 - 180

        # Absorb that drift into the trim so the new trimmed yaw == mag heading
        self._yaw_trim = (self._yaw_trim + drift + 360) % 360
        print(f"[HeadingTrim] trim={self._yaw_trim:.1f}°  "
              f"raw_yaw={self._last_raw_yaw:.1f}°  "
              f"mag={self._last_mag_heading:.1f}°  "
              f"corrected={(self._last_raw_yaw - self._yaw_trim + 360) % 360:.1f}°")

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
        main = tk.Frame(self.root, bg=BG_COLOR)
        main.pack(side="top", fill="both", expand=True, padx=16, pady=(4, 12))

        # ── Col 0, Row 0: IMU table ───────────────────────────────────────────
        # on_adjust_heading fires _apply_heading_trim() in this class.
        # The trim lives here, not in IMUWidget, so all widgets share it.
        self.imu_view = IMUWidget(
            main,
            on_adjust_heading=self._apply_heading_trim,
        )
        self.imu_view.grid(row=0, column=0, sticky="new", padx=(0, 14))

        # ── Col 0, Row 1: Magnetometer + calibration widget ───────────────────
        self.mag_view = MagWidget(
            main,
            on_mag_calibrate=self.hub.start_mag_calibration,
            on_acc_calibrate=self.hub.start_acc_calibration,
        )
        self.mag_view.grid(row=1, column=0, sticky="new", padx=(0, 14), pady=(10, 0))

        # ── Col 1: Primary Flight Display ─────────────────────────────────────
        pfd = ttk.LabelFrame(main, text="Primary Flight Display")
        pfd.grid(row=0, column=1, rowspan=2, sticky="n")

        adi_frame = tk.Frame(pfd, bg=BG_COLOR)
        adi_frame.pack(side="left", padx=(8, 4), pady=8)

        self.drone_3d = Drone3DView(adi_frame, width=420, height=340)
        self.drone_3d.pack()

        self.baro_view = BaroWidget(pfd)
        self.baro_view.pack(side="left", padx=(4, 8), pady=8, anchor="n")

    # ── Auto window sizing ────────────────────────────────────────────────────

    def _fit_window(self):
        self.root.update_idletasks()
        self.root.update()
        w = self.root.winfo_reqwidth()  + 24
        h = self.root.winfo_reqheight() + 24
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        x  = (sw - w) // 2
        y  = (sh - h) // 2
        self.root.geometry(f"{w}x{h}+{x}+{y}")
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
                self.status_label.config(text="Status: Link degraded", fg="orange")
            else:
                self.status_label.config(text="Status: Connected",      fg="green")

            # ── Apply heading trim — single point of application ──────────────
            # Raw FC gyro yaw, then subtract the trim so trimmed_yaw == mag
            # heading at the moment "Adjust Heading" was last pressed.
            # _last_raw_yaw is cached here so _apply_heading_trim() always has
            # a fresh value when the button is pressed between ticks.
            raw_yaw   = float(state.yaw)
            mag_hdg   = getattr(state, "mag_heading_deg", 0.0)
            mag_valid = getattr(state, "mag_valid",        False)

            self._last_raw_yaw     = raw_yaw
            self._last_mag_heading = mag_hdg
            self._last_mag_valid   = mag_valid

            # This is the one and only place the trim is applied.
            # Both widgets receive this same value — no divergence possible.
            trimmed_yaw = (raw_yaw - self._yaw_trim + 360) % 360

            ui_data = {
                # ── IMU raw counts ────────────────────────────────────────────
                "ax":          state.ax,
                "ay":          state.ay,
                "az":          state.az,
                "gx":          state.gx,
                "gy":          state.gy,
                "gz":          state.gz,
                # ── Attitude ─────────────────────────────────────────────────
                "roll":        state.roll  / 10.0,
                "pitch":       state.pitch / 10.0,
                # "yaw" is the trimmed gyro yaw — same value for every widget.
                # IMUWidget shows it in the Angle/HDG cell.
                # Drone3DView scrolls the compass tape on it.
                # They will always agree because they both read this key.
                "yaw":         trimmed_yaw,
                # ── Link quality ──────────────────────────────────────────────
                "voltage":     state.battery_voltage,
                "rssi":        state.rssi,
                "rtt_ms":      state.last_rtt_ms,
                "fc_cycle_ms": state.fc_cycle_ms,
                # ── Barometer ─────────────────────────────────────────────────
                "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
                "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
                "baro_valid":            getattr(state, "baro_valid",             False),
                # ── Magnetometer ──────────────────────────────────────────────
                "mag_x":                     getattr(state, "mag_x",                     0),
                "mag_y":                     getattr(state, "mag_y",                     0),
                "mag_z":                     getattr(state, "mag_z",                     0),
                "mag_heading_deg":           mag_hdg,
                "mag_valid":                 mag_valid,
                # ── Magnetometer calibration ──────────────────────────────────
                "mag_cal_active":            getattr(state, "mag_cal_active",            False),
                "mag_cal_seconds_remaining": getattr(state, "mag_cal_seconds_remaining", 0),
                # ── Gyro/Accel calibration ────────────────────────────────────
                "acc_cal_active":            getattr(state, "acc_cal_active",            False),
                "acc_cal_seconds_remaining": getattr(state, "acc_cal_seconds_remaining", 0),
            }

            # All widgets receive the same ui_data dict.
            # "yaw" in that dict is the trimmed value — guaranteed consistent.
            self.imu_view.update_ui(ui_data)
            self.baro_view.update_baro(ui_data)
            self.drone_3d.update_orientation(
                roll        = ui_data["roll"],
                pitch       = ui_data["pitch"],
                yaw         = ui_data["yaw"],          # trimmed gyro yaw
                mag_heading = ui_data["mag_heading_deg"],
                mag_valid   = ui_data["mag_valid"],
            )
            self.mag_view.update_mag(ui_data)

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