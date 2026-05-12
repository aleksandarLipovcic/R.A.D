import tkinter as tk
from tkinter import ttk
import time
import math


class IMUWidget(ttk.LabelFrame):
    """
    MPU-6500 sensor display for a 10-inch long-range quadcopter cockpit.

    ┌──────────────────────────────────────────────────────────────────────────┐
    │  COLUMN MEANINGS                                                         │
    │                                                                          │
    │  Rotation (°/s) — raw gyro rate with auto-reset peak-hold alarm         │
    │  G-force  (g)   — accelerometer in physical g units                     │
    │  Angle    (°)   — FC-fused attitude (Roll/Pitch) or mag heading (Yaw)   │
    └──────────────────────────────────────────────────────────────────────────┘

    ── YAW ANGLE COLUMN ─────────────────────────────────────────────────────
    The Yaw row Angle cell now shows mag_heading_deg (0–360°) from the
    QMC5883L magnetometer instead of N/A.  Colour reflects the divergence
    between the FC gyro-integrated yaw and the magnetic heading:

        green   |Δ| <  15°   headings agree — normal flight
        yellow  |Δ| <  30°   noticeable drift — monitor
        red     |Δ| ≥  30°   significant divergence — heading adjustment needed

    The "Adjust Heading" button flashes red when drift ≥ 30°.
    Pressing it applies a one-shot trim so the displayed yaw matches the
    magnetometer heading at that instant.  This is NOT a sensor calibration —
    use the CALIBRATE MAG / CALIBRATE GYRO/ACC buttons in the mag widget for
    that.

    ── ACCEL SCALE = 2048 ────────────────────────────────────────────────────
    Betaflight normalises MSP RAW_IMU accel to its internal ±16 g unit:
        1 LSB = 1/2048 g

    ── THRESHOLD DESIGN (10-inch long-range quad) ────────────────────────────
    Angle (°):
        green  |°| <  15   normal hover / gentle cruise
        yellow |°| <  30   moderate bank
        red    |°| ≥  30   aggressive for LR

    Gyro peak-hold (°/s):
        green           |°/s| <  30
        yellow (2 s)    |°/s| < 100
        red    (4 s)    |°/s| ≥ 100

    G-force lateral (ax, ay):
        green  |g| < sin(15°) = 0.26
        yellow |g| < sin(30°) = 0.50
        red    |g| ≥ 0.50

    G-force vertical (az):
        green  0.87 < az < 1.13
        yellow 0.64 < az < 1.36
        red    outside yellow band
    """

    # ── Scale factors ─────────────────────────────────────────────────────────
    GYRO_SCALE  = 16.4      # LSB/(°/s)
    ACCEL_SCALE = 2048.0    # LSB/g

    # ── Colours ───────────────────────────────────────────────────────────────
    CLR_SAFE     = "#99FF99"
    CLR_WARN     = "#FFFF99"
    CLR_CRITICAL = "#FF4444"
    CLR_OFF      = "#E0E0E0"

    # ── Angle thresholds — must match Drone3DView ─────────────────────────────
    ANGLE_WARN_DEG = 15.0
    ANGLE_CRIT_DEG = 30.0

    # ── Gyro peak-hold ────────────────────────────────────────────────────────
    GYRO_WARN_DPS  =  30.0
    GYRO_CRIT_DPS  = 100.0
    HOLD_WARN_SEC  =   2.0
    HOLD_CRIT_SEC  =   4.0

    # ── G-force thresholds ────────────────────────────────────────────────────
    ACC_LAT_WARN   = 0.26   # sin(15°)
    ACC_LAT_CRIT   = 0.50   # sin(30°)
    ACC_VERT_LO_OK = 0.87
    ACC_VERT_HI_OK = 1.13
    ACC_VERT_LO_CR = 0.64
    ACC_VERT_HI_CR = 1.36

    # ── Heading drift thresholds ──────────────────────────────────────────────
    DRIFT_WARN_DEG = 15.0
    DRIFT_CRIT_DEG = 30.0

    def __init__(self, parent, on_adjust_heading=None):
        super().__init__(parent, text="MPU-6500 Long-Range Flight Hub", padding=10)
        self._on_adjust_heading = on_adjust_heading

        self.gyro_offsets   = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.is_calibrating = False
        self.calib_samples  = []

        # Per-axis gyro state machine
        self._gyro_state = {
            axis: {"state": "safe", "hold_until": 0.0}
            for axis in ("roll", "pitch", "yaw")
        }

        # Flash state for the Adjust Heading button
        self._flash_job  = None
        self._flash_on   = False
        self._flashing   = False

        # Cache for flash/drift logic
        self._last_mag_heading  = 0.0
        self._mag_valid         = False

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

        # "Adjust Heading" — fires on_adjust_heading callback (lives in main.py).
        # Flashes red when drift ≥ DRIFT_CRIT_DEG.
        self.adjust_btn = tk.Button(
            ctrl,
            text="Adjust Heading",
            command=self._on_adjust_pressed,
            relief="raised", padx=6, pady=2,
            bg="#E0E0E0", fg="#000000"
        )
        self.adjust_btn.pack(side="left", padx=10)

        # Small drift readout next to the button
        self._drift_lbl = tk.Label(
            ctrl, text="Δ HDG: ---",
            font=("Consolas", 8), fg="#555555"
        )
        self._drift_lbl.pack(side="left", padx=(0, 6))

    def _setup_grid(self):
        container = tk.Frame(self, bd=1, relief="solid", padx=5, pady=5)
        container.grid(row=1, column=0, columnspan=4, sticky="nsew")

        for col, text in enumerate(
            ["Flight axis", "Rotation (°/s)", "G-force (g)", "Angle / HDG (°)"]
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

    # ── Heading trim / adjust ─────────────────────────────────────────────────

    def _on_adjust_pressed(self):
        """
        Fire the on_adjust_heading callback provided by main.py.
        The trim is computed and stored in main.py so ALL widgets see the
        same already-trimmed yaw value on the very next telemetry tick.
        """
        if not self._mag_valid:
            return   # nothing to align to if mag has no signal
        if self._on_adjust_heading:
            self._on_adjust_heading()

    # ── Flash logic ───────────────────────────────────────────────────────────

    def _start_flash(self):
        if self._flashing:
            return
        self._flashing = True
        self._flash_on = True
        self._do_flash()

    def _stop_flash(self):
        if self._flash_job is not None:
            try:
                self.after_cancel(self._flash_job)
            except Exception:
                pass
            self._flash_job = None
        self._flashing = False
        self._flash_on = False
        self.adjust_btn.config(bg="#E0E0E0", fg="#000000",
                               relief="raised", text="Adjust Heading")

    def _do_flash(self):
        if not self._flashing:
            return
        self._flash_on = not self._flash_on
        if self._flash_on:
            self.adjust_btn.config(bg="#FF2222", fg="#FFFFFF",
                                   relief="sunken", text="⚠ Adjust Heading")
        else:
            self.adjust_btn.config(bg="#880000", fg="#FF9999",
                                   relief="raised", text="⚠ Adjust Heading")
        self._flash_job = self.after(500, self._do_flash)

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

        # ── Magnetometer / heading ────────────────────────────────────────────
        mag_heading  = data.get("mag_heading_deg", 0.0)
        mag_valid    = data.get("mag_valid", False)
        fc_yaw_raw   = data.get("yaw", 0.0)

        self._last_mag_heading = mag_heading
        self._mag_valid        = mag_valid

        # yaw is already trimmed by main.py — use directly
        fc_yaw_trimmed = fc_yaw_raw % 360

        # Signed shortest-path difference (−180 … +180)
        if mag_valid:
            drift = (fc_yaw_trimmed - mag_heading + 540) % 360 - 180
        else:
            drift = 0.0

        abs_drift = abs(drift)

        # ── Update drift label ────────────────────────────────────────────────
        if mag_valid:
            drift_txt = f"Δ HDG: {drift:>+6.1f}°"
            if   abs_drift >= self.DRIFT_CRIT_DEG: drift_clr = "#FF2222"
            elif abs_drift >= self.DRIFT_WARN_DEG: drift_clr = "#CCAA00"
            else:                                  drift_clr = "#227722"
            self._drift_lbl.config(text=drift_txt, fg=drift_clr)
        else:
            self._drift_lbl.config(text="Δ HDG: NO MAG", fg="#888888")

        # ── Flash control ────────────────────────────────────────────────────
        if mag_valid and abs_drift >= self.DRIFT_CRIT_DEG:
            self._start_flash()
        else:
            self._stop_flash()

        # ── IMU rows ─────────────────────────────────────────────────────────
        rows = [
            ("roll",  gx, ax, roll,  False),
            ("pitch", gy, ay, pitch, False),
            ("yaw",   gz, az, None,  True ),   # angle handled separately below
        ]

        for key, rot_val, acc_val, ang_val, is_vertical in rows:
            w = self.axes[key]
            w["rot"].config(text=f"{rot_val:>7.2f}",
                            bg=self._gyro_color(key, abs(rot_val), now))
            w["acc"].config(text=f"{acc_val:>7.3f}",
                            bg=self._accel_color(acc_val, is_vertical))

            if key == "yaw":
                # Always show FC gyro yaw (trimmed, 0-360°).
                # Cell colour reflects drift vs magnetometer:
                #   green  = headings agree  (drift < DRIFT_WARN_DEG)
                #   yellow = monitor         (drift < DRIFT_CRIT_DEG)
                #   red    = adjust needed   (drift ≥ DRIFT_CRIT_DEG)
                # When mag has no signal the colour falls back to the normal
                # gyro-rate colour so the gyro yaw is still always readable.
                yaw_display = fc_yaw_trimmed % 360
                if mag_valid:
                    cell_bg = self._drift_color(abs_drift)
                else:
                    cell_bg = self.CLR_SAFE   # no reference → neutral green
                w["ang"].config(
                    text=f"{yaw_display:>7.1f}",
                    bg=cell_bg
                )
            else:
                w["ang"].config(text=f"{ang_val:>7.1f}",
                                bg=self._angle_color(ang_val))

        if self.show_diag.get():
            rtt      = data.get("rtt_ms",      0.0)
            fc_cycle = data.get("fc_cycle_ms", 0.0)
            self.rtt_lbl.config(   text=f"Total RTT:     {rtt:>6.2f} ms")
            self.oneway_lbl.config(text=f"Est. one-way:  {rtt / 2:>6.2f} ms")
            self.cycle_lbl.config( text=f"FC cycle:      {fc_cycle:>6.2f} ms")

    # ── Calibration (kept for internal gyro offset, not exposed as button) ────

    def _start_calibration(self):
        self.is_calibrating = True
        self.calib_samples  = []

    def _finish_calibration(self, samples):
        n = len(samples)
        self.gyro_offsets = {
            "x": sum(s[0] for s in samples) / n,
            "y": sum(s[1] for s in samples) / n,
            "z": sum(s[2] for s in samples) / n,
        }
        self.is_calibrating = False

    # ── Colour helpers ────────────────────────────────────────────────────────

    def _gyro_color(self, axis: str, abs_dps: float, now: float) -> str:
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

    def _drift_color(self, abs_drift: float) -> str:
        """Cell background for the yaw heading cell based on mag/gyro divergence."""
        if   abs_drift >= self.DRIFT_CRIT_DEG: return self.CLR_CRITICAL
        elif abs_drift >= self.DRIFT_WARN_DEG: return self.CLR_WARN
        else:                                  return self.CLR_SAFE