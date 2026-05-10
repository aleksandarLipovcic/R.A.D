import tkinter as tk
from tkinter import ttk
import time


class IMUWidget(ttk.LabelFrame):
    """
    MPU-6500 sensor display for a 10-inch long-range quadcopter cockpit.

    ┌──────────────────────────────────────────────────────────────────────────┐
    │  COLUMN MEANINGS                                                         │
    │                                                                          │
    │  Rotation (°/s) — raw gyro rate with auto-reset peak-hold alarm         │
    │  G-force  (g)   — accelerometer in physical g units                     │
    │  Angle    (°)   — FC-fused attitude from MSP_ATTITUDE                   │
    └──────────────────────────────────────────────────────────────────────────┘

    ── ACCEL SCALE = 2048 ────────────────────────────────────────────────────
    Betaflight normalises MSP RAW_IMU accel to its internal ±16 g unit:
        1 LSB = 1/2048 g
    Verified: raw_az ≈ 2047 at rest → 2047/2048 ≈ 1.000 g ✓

    ── THRESHOLD DESIGN (10-inch long-range quad) ────────────────────────────
    LR quads fly gently; thresholds are LOWER than freestyle values.

    Angle (°):
        green  |°| <  20   normal hover / gentle cruise
        yellow |°| <  40   moderate bank — pilot awareness
        red    |°| ≥  40   aggressive for LR — critical

    Gyro peak-hold (°/s)  — LR drone turns gently, 10-20°/s is normal:
        green           |°/s| <  30    normal flight input
        yellow (2 s hold) |°/s| < 100  noticeable snap / gust
        red    (4 s hold) |°/s| ≥ 100  severe event — drone likely in trouble
        Both auto-reset after their hold window expires.
        "Calibrate gyro" resets gyro offsets only — not an alarm button.

    G-force lateral (ax, ay — Roll/Pitch rows) — physics-aligned with angle:
        green  |g| < sin(20°) = 0.34   within angle warn threshold
        yellow |g| < sin(40°) = 0.64   between warn and critical
        red    |g| ≥ sin(40°) = 0.64   at or past angle critical threshold ✓

    G-force vertical (az — Yaw row, ≈ 1 g at rest):
        green  0.85 < az < 1.15   roughly within 30° of level
        yellow 0.64 < az < 0.85   20°–50° tilt range
        red    az ≤ 0.64 or az ≥ 1.36   severe tilt (cos 50° ≈ 0.64)

    ── CONSISTENCY ───────────────────────────────────────────────────────────
    ANGLE_WARN_DEG / ANGLE_CRIT_DEG must match Drone3DView.WARN_ANGLE /
    CRITICAL_ANGLE so the table and the 3D model warn at the same moment.
    """

    # ── Scale factors ─────────────────────────────────────────────────────────
    GYRO_SCALE  = 16.4      # LSB/(°/s)  — Betaflight MSP RAW_IMU gyro
    ACCEL_SCALE = 2048.0    # LSB/g      — Betaflight internal ±16 g norm

    # ── Colours ───────────────────────────────────────────────────────────────
    CLR_SAFE     = "#99FF99"
    CLR_WARN     = "#FFFF99"
    CLR_CRITICAL = "#FF4444"
    CLR_OFF      = "#E0E0E0"

    # ── Angle thresholds — MUST match Drone3DView.WARN_ANGLE / CRITICAL_ANGLE ─
    ANGLE_WARN_DEG = 15.0   # LR quad: warn at 15°
    ANGLE_CRIT_DEG = 30.0   # LR quad: critical at 30°

    # ── Gyro peak-hold thresholds ─────────────────────────────────────────────
    GYRO_WARN_DPS  =  30.0   # LR drone: gentle turns are 10-20°/s; 30 = noticeable snap
    GYRO_CRIT_DPS  = 100.0   # crash/severe gust — LR drone should never reach this
    HOLD_WARN_SEC  =   2.0
    HOLD_CRIT_SEC  =   4.0

    # ── G-force thresholds ────────────────────────────────────────────────────
    ACC_LAT_WARN     = 0.26   # sin(15°) — aligns with ANGLE_WARN_DEG = 15°
    ACC_LAT_CRIT     = 0.50   # sin(30°) — aligns with ANGLE_CRIT_DEG = 30°
    ACC_VERT_LO_OK   = 0.87   # cos(30°) = 0.866 — green while within crit angle threshold
    ACC_VERT_HI_OK   = 1.13   # symmetric upper green bound
    ACC_VERT_LO_CR   = 0.64   # cos(50°) = 0.643 — warn-to-critical boundary
    ACC_VERT_HI_CR   = 1.36   # symmetric upper critical bound

    def __init__(self, parent):
        super().__init__(parent, text="MPU-6500 Long-Range Flight Hub", padding=10)

        self.gyro_offsets   = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.is_calibrating = False
        self.calib_samples  = []

        # Per-axis gyro state machine: "safe" | "warn" | "critical"
        self._gyro_state = {
            axis: {"state": "safe", "hold_until": 0.0}
            for axis in ("roll", "pitch", "yaw")
        }

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

        # tk.Button so we can set bg colour during calibration
        self.calib_btn = tk.Button(
            ctrl, text="Calibrate gyro",
            command=self._start_calibration,
            relief="raised", padx=6, pady=2
        )
        self.calib_btn.pack(side="left", padx=10)

    def _setup_grid(self):
        container = tk.Frame(self, bd=1, relief="solid", padx=5, pady=5)
        container.grid(row=1, column=0, columnspan=4, sticky="nsew")

        for col, text in enumerate(
            ["Flight axis", "Rotation (°/s)", "G-force (g)", "Angle (°)"]
        ):
            tk.Label(container, text=text, font=("Arial", 10, "bold")
                     ).grid(row=0, column=col, padx=12, pady=5)

        self.axes = {}
        for row_idx, (key, label) in enumerate(
            [("roll", "Roll  (X)"), ("pitch", "Pitch (Y)"), ("yaw", "Yaw   (Z)")],
            start=1
        ):
            tk.Label(container, text=f"{label}:", font=("Arial", 10)
                     ).grid(row=row_idx, column=0, sticky="e")

            rot = tk.Label(container, text="  0.00",
                           font=("Consolas", 12, "bold"), width=10, bg=self.CLR_SAFE)
            acc = tk.Label(container, text="  0.000",
                           font=("Consolas", 12, "bold"), width=10, bg=self.CLR_SAFE)
            ang = tk.Label(container, text="  0.0",
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
            self.diag_frame.grid(row=5, column=0, columnspan=4,
                                 sticky="ew", pady=(10, 0))
        else:
            self.diag_frame.grid_forget()

    # ── Calibration ───────────────────────────────────────────────────────────

    def _start_calibration(self):
        """Average 50 gyro samples at rest to compute zero-rate drift offsets."""
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
        gx = data.get("gx", 0) / self.GYRO_SCALE
        gy = data.get("gy", 0) / self.GYRO_SCALE
        gz = data.get("gz", 0) / self.GYRO_SCALE
        ax = data.get("ax", 0) / self.ACCEL_SCALE
        ay = data.get("ay", 0) / self.ACCEL_SCALE
        az = data.get("az", 0) / self.ACCEL_SCALE

        if self.is_calibrating:
            self.calib_samples.append((gx, gy, gz))
            if len(self.calib_samples) >= 50:
                self._finish_calibration(self.calib_samples)
            return

        gx -= self.gyro_offsets["x"]
        gy -= self.gyro_offsets["y"]
        gz -= self.gyro_offsets["z"]

        roll  = data.get("roll",  0.0)
        pitch = data.get("pitch", 0.0)
        now   = time.monotonic()

        # Row: (key, gyro_val, accel_val, angle_val|None, accel_is_vertical)
        # az on the Yaw row because az is the body Z = vertical = gravity axis
        rows = [
            ("roll",  gx, ax, roll,  False),
            ("pitch", gy, ay, pitch, False),
            ("yaw",   gz, az, None,  True ),
        ]

        for key, rot_val, acc_val, ang_val, is_vertical in rows:
            w = self.axes[key]
            w["rot"].config(text=f"{rot_val:>7.2f}",
                            bg=self._gyro_color(key, abs(rot_val), now))
            w["acc"].config(text=f"{acc_val:>7.3f}",
                            bg=self._accel_color(acc_val, is_vertical))
            if ang_val is None:
                w["ang"].config(text="  N/A", bg=self.CLR_OFF)
            else:
                w["ang"].config(text=f"{ang_val:>7.1f}",
                                bg=self._angle_color(ang_val))

        if self.show_diag.get():
            rtt      = data.get("rtt_ms",      0.0)
            fc_cycle = data.get("fc_cycle_ms", 0.0)
            self.rtt_lbl.config(   text=f"Total RTT:     {rtt:>6.2f} ms")
            self.oneway_lbl.config(text=f"Est. one-way:  {rtt / 2:>6.2f} ms")
            self.cycle_lbl.config( text=f"FC cycle:      {fc_cycle:>6.2f} ms")

    # ── Colour helpers ────────────────────────────────────────────────────────

    def _gyro_color(self, axis: str, abs_dps: float, now: float) -> str:
        """
        Auto-resetting three-state peak-hold.

        critical (≥250 °/s): red held for HOLD_CRIT_SEC, then safe directly
        warn     (≥ 80 °/s): yellow held for HOLD_WARN_SEC, then safe
        While red hold is active: new warn events don't downgrade to yellow.
        Calibrate gyro is a drift-zeroing tool, not an alarm reset.
        """
        s = self._gyro_state[axis]

        if abs_dps >= self.GYRO_CRIT_DPS:
            s["state"]      = "critical"
            s["hold_until"] = now + self.HOLD_CRIT_SEC

        elif abs_dps >= self.GYRO_WARN_DPS:
            if s["state"] != "critical":
                s["state"]      = "warn"
                s["hold_until"] = now + self.HOLD_WARN_SEC

        else:
            if now >= s["hold_until"]:
                s["state"]      = "safe"
                s["hold_until"] = 0.0

        if s["state"] == "critical": return self.CLR_CRITICAL
        if s["state"] == "warn":     return self.CLR_WARN
        return self.CLR_SAFE

    def _accel_color(self, val: float, is_vertical: bool) -> str:
        if is_vertical:
            if   self.ACC_VERT_LO_OK < val < self.ACC_VERT_HI_OK: return self.CLR_SAFE
            elif self.ACC_VERT_LO_CR < val < self.ACC_VERT_HI_CR: return self.CLR_WARN
            else:                                                  return self.CLR_CRITICAL
        else:
            a = abs(val)
            if   a < self.ACC_LAT_WARN: return self.CLR_SAFE
            elif a < self.ACC_LAT_CRIT: return self.CLR_WARN
            else:                       return self.CLR_CRITICAL

    def _angle_color(self, deg: float) -> str:
        a = abs(deg)
        if   a < self.ANGLE_WARN_DEG: return self.CLR_SAFE
        elif a < self.ANGLE_CRIT_DEG: return self.CLR_WARN
        else:                         return self.CLR_CRITICAL