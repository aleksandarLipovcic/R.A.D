import tkinter as tk
from tkinter import ttk
import time


class IMUWidget(ttk.LabelFrame):
    """
    Displays MPU-6500 sensor data in a tabular PFD-style grid.

    Expects update_ui() to be called with a plain dict from main.py:
        {
            "ax", "ay", "az"      — raw accel counts (int16)
            "gx", "gy", "gz"      — raw gyro  counts (int16)
            "roll", "pitch", "yaw" — degrees (already ÷10 from main)
            "voltage"             — volts (float)
            "rssi"                — 0-255
            "rtt_ms"              — round-trip time from DroneState
            "fc_cycle_ms"         — FC loop time from DroneState
        }
    """

    # MPU-6500 scale factors matching DroneLink.cpp / IMUSensor.cpp
    # ±4g  accel → LSB/g  = 8192
    # ±2000°/s   → LSB/°/s = 16.4
    GYRO_SCALE  = 16.4
    ACCEL_SCALE = 8192.0

    # Colour thresholds
    CLR_SAFE     = "#99FF99"
    CLR_WARN     = "#FFFF99"
    CLR_CRITICAL = "#FF4444"
    CLR_OFF      = "#E0E0E0"

    def __init__(self, parent):
        super().__init__(parent, text="MPU-6500 Long-Range Flight Hub", padding=10)

        self.gyro_offsets   = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.is_calibrating = False
        self.calib_samples  = []

        self._setup_header()
        self._setup_grid()
        self._setup_diag_frame()
        self.refresh_layout()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _setup_header(self):
        ctrl = tk.Frame(self)
        ctrl.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))

        self.show_diag = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            ctrl,
            text="Show thesis latency data",
            variable=self.show_diag,
            command=self.refresh_layout
        ).pack(side="left")

        self.calib_btn = ttk.Button(
            ctrl, text="Calibrate gyro", command=self._start_calibration
        )
        self.calib_btn.pack(side="left", padx=10)

    def _setup_grid(self):
        container = tk.Frame(self, bd=1, relief="solid", padx=5, pady=5)
        container.grid(row=1, column=0, columnspan=4, sticky="nsew")

        headers = ["Flight axis", "Rotation (°/s)", "G-force (g)", "Angle (°)"]
        for col, text in enumerate(headers):
            tk.Label(
                container, text=text, font=("Arial", 10, "bold")
            ).grid(row=0, column=col, padx=12, pady=5)

        self.axes = {}
        rows = [("roll", "Roll  (X)"), ("pitch", "Pitch (Y)"), ("yaw", "Yaw   (Z)")]
        for row_idx, (key, label) in enumerate(rows, start=1):
            tk.Label(
                container, text=f"{label}:", font=("Arial", 10)
            ).grid(row=row_idx, column=0, sticky="e")

            rot = tk.Label(container, text="0.00",
                           font=("Consolas", 12, "bold"), width=10, bg=self.CLR_SAFE)
            acc = tk.Label(container, text="0.00",
                           font=("Consolas", 12, "bold"), width=10, bg=self.CLR_SAFE)
            ang = tk.Label(container, text="0.0",
                           font=("Consolas", 12, "bold"), width=10, bg=self.CLR_SAFE)

            rot.grid(row=row_idx, column=1, padx=2, pady=2)
            acc.grid(row=row_idx, column=2, padx=2, pady=2)
            ang.grid(row=row_idx, column=3, padx=2, pady=2)

            self.axes[key] = {"rot": rot, "acc": acc, "ang": ang}

    def _setup_diag_frame(self):
        self.diag_frame = ttk.LabelFrame(self, text="Link performance breakdown")
        self.rtt_lbl    = ttk.Label(self.diag_frame, text="Total RTT:     — ms",
                                    font=("Consolas", 10))
        self.oneway_lbl = ttk.Label(self.diag_frame, text="Est. one-way:  — ms",
                                    font=("Consolas", 10))
        self.cycle_lbl  = ttk.Label(self.diag_frame, text="FC cycle:      — ms",
                                    font=("Consolas", 10))
        for lbl in (self.rtt_lbl, self.oneway_lbl, self.cycle_lbl):
            lbl.pack(anchor="w", padx=5, pady=1)

    def refresh_layout(self):
        if self.show_diag.get():
            self.diag_frame.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        else:
            self.diag_frame.grid_forget()

    # ── Calibration ───────────────────────────────────────────────────────────

    def _start_calibration(self):
        """Collect 50 gyro samples at rest and compute offset averages."""
        self.is_calibrating = True
        self.calib_samples  = []
        self.calib_btn.config(text="Calibrating...", state="disabled")

    def _finish_calibration(self, samples):
        n = len(samples)
        self.gyro_offsets = {
            "x": sum(s[0] for s in samples) / n,
            "y": sum(s[1] for s in samples) / n,
            "z": sum(s[2] for s in samples) / n,
        }
        self.is_calibrating = False
        self.calib_btn.config(text="Calibrate gyro", state="normal")

    # ── Main update ───────────────────────────────────────────────────────────

    def update_ui(self, data: dict):
        """
        Receive a plain dict from main.py and refresh all displays.
        All scaling from raw counts to physical units happens here so
        the C++ side stays unmodified if ranges change.
        """
        # Raw counts → physical units
        gx = data.get("gx", 0) / self.GYRO_SCALE
        gy = data.get("gy", 0) / self.GYRO_SCALE
        gz = data.get("gz", 0) / self.GYRO_SCALE
        ax = data.get("ax", 0) / self.ACCEL_SCALE
        ay = data.get("ay", 0) / self.ACCEL_SCALE
        az = data.get("az", 0) / self.ACCEL_SCALE

        # Gyro calibration collection
        if self.is_calibrating:
            self.calib_samples.append((gx, gy, gz))
            if len(self.calib_samples) >= 50:
                self._finish_calibration(self.calib_samples)
            return  # Don't update display during calibration

        # Apply gyro offsets
        gx -= self.gyro_offsets["x"]
        gy -= self.gyro_offsets["y"]
        gz -= self.gyro_offsets["z"]

        roll  = data.get("roll",  0.0)
        pitch = data.get("pitch", 0.0)
        yaw   = data.get("yaw",   0.0)

        # Update the three axis rows
        # roll row:  X gyro, X accel, roll angle from FC fusion
        # pitch row: Y gyro, Y accel, pitch angle from FC fusion
        # yaw row:   Z gyro, Z accel, yaw heading (shown as N/A — no reliable
        #            absolute reference without magnetometer fusion in MSP)
        rows = {
            "roll":  (gx, ax, roll,  False),
            "pitch": (gy, ay, pitch, False),
            "yaw":   (gz, az, yaw,   True),   # is_yaw=True disables angle cell
        }

        for key, (rot_val, acc_val, ang_val, is_yaw) in rows.items():
            w = self.axes[key]

            w["rot"].config(text=f"{rot_val:>7.2f}")

            acc_color = self._accel_color(acc_val, is_yaw)
            w["acc"].config(text=f"{acc_val:>7.3f}", bg=acc_color)

            if is_yaw:
                w["ang"].config(text="N/A", bg=self.CLR_OFF)
            else:
                w["ang"].config(
                    text=f"{ang_val:>7.1f}",
                    bg=self._angle_color(ang_val)
                )

        # Latency panel
        if self.show_diag.get():
            rtt      = data.get("rtt_ms",      0.0)
            fc_cycle = data.get("fc_cycle_ms", 0.0)
            self.rtt_lbl.config(   text=f"Total RTT:     {rtt:>6.2f} ms")
            self.oneway_lbl.config(text=f"Est. one-way:  {rtt/2:>6.2f} ms")
            self.cycle_lbl.config( text=f"FC cycle:      {fc_cycle:>6.2f} ms")

    # ── Colour helpers ────────────────────────────────────────────────────────

    def _accel_color(self, val: float, is_z_axis: bool) -> str:
        """
        Z axis should read ~1.0 g at rest (gravity).
        X/Y axes should read ~0.0 g at rest.
        """
        if is_z_axis:
            if   0.85 < val < 1.15: return self.CLR_SAFE
            elif 0.60 < val < 1.40: return self.CLR_WARN
            else:                   return self.CLR_CRITICAL
        else:
            a = abs(val)
            if   a < 0.20: return self.CLR_SAFE
            elif a < 0.60: return self.CLR_WARN
            else:          return self.CLR_CRITICAL

    def _angle_color(self, deg: float) -> str:
        a = abs(deg)
        if   a < 15: return self.CLR_SAFE
        elif a < 35: return self.CLR_WARN
        else:        return self.CLR_CRITICAL