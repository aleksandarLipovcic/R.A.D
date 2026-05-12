import tkinter as tk
from tkinter import ttk
import math


class MagWidget(ttk.LabelFrame):
    """
    QMC5883L Magnetometer display widget.

    Shows:
      - Compass rose with rotating card and fixed lubber line (magnetic heading)
      - X / Y / Z field strength bars with numeric readout
      - Heading readout box (degrees + cardinal)
      - Signal validity indicator
      - Magnetometer calibration button with live countdown
      - Gyro/Accel calibration button with live countdown

    ── MAG DATA SOURCE ──────────────────────────────────────────────────────────
    Betaflight has NO MSP_RAW_MAG command.  ID 130 = MSP_BATTERY_STATE.
    Raw mag X/Y/Z comes from MSP_DEBUG (254) when the FC has:
        set debug_mode = MAG_CALIB
        save
    This is the same source the Betaflight Configurator Sensors tab uses.
    If magValid is False, the most likely cause is that debug_mode is not
    set to MAG_CALIB in the Betaflight CLI.

    ── DATA DICT KEYS (from DroneState.to_dict() / main.py ui_data) ────────────
        "mag_x"                      int16   raw field X (ADC counts)
        "mag_y"                      int16   raw field Y
        "mag_z"                      int16   raw field Z
        "mag_heading_deg"            float   0–360°, tilt-uncorrected
        "mag_valid"                  bool    False → show NO SIG + hint
        "mag_cal_active"             bool    True while FC is calibrating mag
        "mag_cal_seconds_remaining"  int     countdown 30 → 0
        "acc_cal_active"             bool    True while FC is calibrating acc/gyro
        "acc_cal_seconds_remaining"  int     countdown 5 → 0

    ── CALLBACKS ────────────────────────────────────────────────────────────────
        on_mag_calibrate  — wired to DroneLink.start_mag_calibration()
        on_acc_calibrate  — wired to DroneLink.start_acc_calibration()
        Both are optional; if None the corresponding button is shown disabled.
    """

    # ── Colours — cockpit palette ─────────────────────────────────────────────
    C_BG         = "#0a0a0a"
    C_ROSE_BG    = "#111418"
    C_ROSE_RING  = "#2a3a2a"
    C_LUBBER     = "#FFD700"      # aviation gold
    C_CARDINAL   = "#FFFFFF"
    C_CARDINAL_N = "#FF4444"      # North always red
    C_INTER_TICK = "#445544"
    C_SAFE       = "#00FF44"
    C_WARN       = "#FFD700"
    C_CRIT       = "#FF3333"
    C_OFF        = "#444444"
    C_BAR_BG     = "#1a1a1a"
    C_TEXT       = "#CCCCCC"
    C_LABEL      = "#888888"
    C_NOSIG_FG   = "#FF3333"
    C_FRAME      = "#1e2a1e"
    C_CAL_BTN    = "#1a2a1a"
    C_ACC_BTN    = "#1a1a2a"
    C_CAL_ACTIVE = "#FFD700"
    C_HINT       = "#885500"

    # ── Mag field bar scale ───────────────────────────────────────────────────
    # QMC5883L full-scale at ±8 Gauss (BF default) = ±32768 counts
    # Bar auto-scaling: expands to 1.25x the largest axis seen so bars
    # fill a meaningful portion even at low field strength. BF MAG_CALIB
    # debug values are scaled counts, typical range +-500 to +-2000.
    MAG_SCALE_MIN = 800.0    # half-range floor — bars always visible
    _mag_scale    = 800.0    # runtime auto-scale (instance, not class)

    # ── Cardinal labels ───────────────────────────────────────────────────────
    CARDINALS = {
        0: "N",  45: "NE",  90: "E",  135: "SE",
        180: "S", 225: "SW", 270: "W", 315: "NW"
    }

    def __init__(self, parent, on_mag_calibrate=None, on_acc_calibrate=None):
        """
        on_mag_calibrate: callable()  → DroneLink.start_mag_calibration()
        on_acc_calibrate: callable()  → DroneLink.start_acc_calibration()
        """
        super().__init__(parent, text="QMC5883L Magnetometer", padding=8)
        self.configure(style="Mag.TLabelframe")
        self._last_heading    = 0.0
        self._valid           = False
        self._on_mag_cal      = on_mag_calibrate
        self._on_acc_cal      = on_acc_calibrate
        self._setup_ui()

    # ══════════════════════════════════════════════════════════════════════════
    # UI Construction
    # ══════════════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        # ── Top row: compass rose (left) + heading + cal buttons (right) ──────
        top = tk.Frame(self, bg=self.C_BG)
        top.pack(fill="x", pady=(0, 6))

        self._rose_size = 160
        self.rose_canvas = tk.Canvas(
            top,
            width=self._rose_size, height=self._rose_size,
            bg=self.C_BG, highlightthickness=0
        )
        self.rose_canvas.pack(side="left", padx=(0, 10))

        right = tk.Frame(top, bg=self.C_BG)
        right.pack(side="left", fill="y", expand=True)

        # ── Heading readout box ───────────────────────────────────────────────
        hdg_frame = tk.Frame(right, bg=self.C_FRAME,
                             relief="flat", bd=1,
                             highlightbackground="#2a4a2a",
                             highlightthickness=1)
        hdg_frame.pack(fill="x", pady=(10, 4))

        tk.Label(hdg_frame, text="MAGNETIC HEADING",
                 font=("Consolas", 7, "bold"),
                 fg=self.C_LABEL, bg=self.C_FRAME
                 ).pack(pady=(4, 0))

        self.hdg_value_lbl = tk.Label(
            hdg_frame, text="---.-°",
            font=("Consolas", 22, "bold"),
            fg=self.C_SAFE, bg=self.C_FRAME
        )
        self.hdg_value_lbl.pack()

        self.cardinal_lbl = tk.Label(
            hdg_frame, text="---",
            font=("Consolas", 12, "bold"),
            fg=self.C_WARN, bg=self.C_FRAME
        )
        self.cardinal_lbl.pack(pady=(0, 4))

        # ── Signal status ─────────────────────────────────────────────────────
        self.status_lbl = tk.Label(
            right, text="● NO SIGNAL",
            font=("Consolas", 9, "bold"),
            fg=self.C_NOSIG_FG, bg=self.C_BG
        )
        self.status_lbl.pack(pady=(2, 0))

        # ── Debug mode hint (shown when magValid is False) ────────────────────
        self._hint_lbl = tk.Label(
            right,
            text="Set  debug_mode = MAG_CALIB  in BF CLI",
            font=("Consolas", 7),
            fg=self.C_HINT, bg=self.C_BG,
            wraplength=160
        )
        self._hint_lbl.pack(pady=(1, 0))

        # ── Magnetometer calibration ──────────────────────────────────────────
        mag_cal_frame = tk.Frame(right, bg=self.C_BG)
        mag_cal_frame.pack(fill="x", pady=(6, 0))

        self._mag_cal_btn = tk.Button(
            mag_cal_frame,
            text="⊕  CALIBRATE MAG",
            font=("Consolas", 8, "bold"),
            fg=self.C_SAFE,
            bg=self.C_CAL_BTN,
            activeforeground=self.C_BG,
            activebackground=self.C_SAFE,
            relief="flat", bd=0, padx=6, pady=4,
            cursor="hand2",
            command=self._on_mag_cal_pressed,
            state="normal" if self._on_mag_cal else "disabled"
        )
        self._mag_cal_btn.pack(fill="x")

        self._mag_cal_status_lbl = tk.Label(
            mag_cal_frame, text="",
            font=("Consolas", 8),
            fg=self.C_CAL_ACTIVE, bg=self.C_BG
        )
        self._mag_cal_status_lbl.pack(fill="x", pady=(2, 0))

        # ── Gyro/Accel calibration ────────────────────────────────────────────
        acc_cal_frame = tk.Frame(right, bg=self.C_BG)
        acc_cal_frame.pack(fill="x", pady=(4, 0))

        self._acc_cal_btn = tk.Button(
            acc_cal_frame,
            text="⊕  CALIBRATE GYRO/ACC",
            font=("Consolas", 8, "bold"),
            fg="#44AAFF",
            bg=self.C_ACC_BTN,
            activeforeground=self.C_BG,
            activebackground="#44AAFF",
            relief="flat", bd=0, padx=6, pady=4,
            cursor="hand2",
            command=self._on_acc_cal_pressed,
            state="normal" if self._on_acc_cal else "disabled"
        )
        self._acc_cal_btn.pack(fill="x")

        self._acc_cal_status_lbl = tk.Label(
            acc_cal_frame, text="",
            font=("Consolas", 8),
            fg="#44AAFF", bg=self.C_BG
        )
        self._acc_cal_status_lbl.pack(fill="x", pady=(2, 0))

        # ── Raw field bars ────────────────────────────────────────────────────
        bar_frame = tk.Frame(self, bg=self.C_BG)
        bar_frame.pack(fill="x")

        tk.Label(bar_frame, text="RAW FIELD COMPONENTS",
                 font=("Consolas", 7, "bold"),
                 fg=self.C_LABEL, bg=self.C_BG
                 ).pack(anchor="w")

        self._bars = {}
        for axis, color in [("X", "#4488FF"), ("Y", "#FF8844"), ("Z", "#44FF88")]:
            row = tk.Frame(bar_frame, bg=self.C_BG)
            row.pack(fill="x", pady=1)

            tk.Label(row, text=f" {axis} ",
                     font=("Consolas", 9, "bold"),
                     fg=color, bg=self.C_BG, width=3
                     ).pack(side="left")

            bar_cv = tk.Canvas(row, width=160, height=14,
                               bg=self.C_BG, highlightthickness=0)
            bar_cv.pack(side="left", padx=(2, 6))

            val_lbl = tk.Label(row, text="     0",
                               font=("Consolas", 9),
                               fg=self.C_TEXT, bg=self.C_BG, width=7)
            val_lbl.pack(side="left")

            self._bars[axis] = {"canvas": bar_cv, "label": val_lbl, "color": color}

        self._draw_rose(0.0, valid=False)
        self._draw_bars(0, 0, 0)

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration button handlers
    # ══════════════════════════════════════════════════════════════════════════

    def _on_mag_cal_pressed(self):
        if self._on_mag_cal:
            self._on_mag_cal()
        self._mag_cal_btn.config(state="disabled", text="⊕  CALIBRATING MAG…")

    def _on_acc_cal_pressed(self):
        if self._on_acc_cal:
            self._on_acc_cal()
        self._acc_cal_btn.config(state="disabled", text="⊕  CALIBRATING…")

    # ══════════════════════════════════════════════════════════════════════════
    # Public update entry point
    # ══════════════════════════════════════════════════════════════════════════

    def update_mag(self, data: dict):
        """
        Call this every telemetry tick with the ui_data dict from main.py.
        Handles both mag and acc/gyro calibration state.
        """
        valid        = data.get("mag_valid",                  False)
        heading      = data.get("mag_heading_deg",            0.0)
        mx           = data.get("mag_x",                      0)
        my           = data.get("mag_y",                      0)
        mz           = data.get("mag_z",                      0)
        mag_cal_act  = data.get("mag_cal_active",             False)
        mag_cal_rem  = data.get("mag_cal_seconds_remaining",  0)
        acc_cal_act  = data.get("acc_cal_active",             False)
        acc_cal_rem  = data.get("acc_cal_seconds_remaining",  0)

        self._valid        = valid
        self._last_heading = heading

        self._draw_rose(heading, valid=valid)
        self._draw_bars(mx, my, mz)
        self._update_readout(heading, valid)
        self._update_mag_cal_ui(mag_cal_act, mag_cal_rem)
        self._update_acc_cal_ui(acc_cal_act, acc_cal_rem)

    # ══════════════════════════════════════════════════════════════════════════
    # Compass Rose
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_rose(self, heading: float, valid: bool):
        cv = self.rose_canvas
        cv.delete("all")

        S  = self._rose_size
        cx = S // 2
        cy = S // 2
        r  = S // 2 - 8

        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       fill=self.C_ROSE_BG, outline=self.C_ROSE_RING, width=2)

        if not valid:
            cv.create_text(cx, cy - 8, text="NO SIG",
                           fill=self.C_NOSIG_FG,
                           font=("Consolas", 11, "bold"))
            cv.create_text(cx, cy + 10, text="debug_mode?",
                           fill=self.C_HINT,
                           font=("Consolas", 7))
            return

        # Rotating card ticks
        for deg in range(0, 360, 5):
            is_card  = (deg % 90 == 0)
            is_inter = (deg % 45 == 0)
            tick_len = 10 if is_card else (7 if is_inter else 4)
            tick_w   = 2  if is_card else 1
            tick_clr = self.C_CARDINAL if is_card else (
                       self.C_INTER_TICK if is_inter else "#2a3a2a")

            screen_deg = deg - heading
            rad = math.radians(screen_deg - 90)
            ox = cx + r              * math.cos(rad)
            oy = cy + r              * math.sin(rad)
            ix = cx + (r - tick_len) * math.cos(rad)
            iy = cy + (r - tick_len) * math.sin(rad)
            cv.create_line(ix, iy, ox, oy, fill=tick_clr, width=tick_w)

        # Rotating cardinal letters
        label_r = r - 18
        for hdg_fixed, letter in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            screen_deg = hdg_fixed - heading
            rad = math.radians(screen_deg - 90)
            lx = cx + label_r * math.cos(rad)
            ly = cy + label_r * math.sin(rad)
            color = self.C_CARDINAL_N if letter == "N" else self.C_CARDINAL
            cv.create_text(lx, ly, text=letter, fill=color,
                           font=("Consolas", 9, "bold"))

        # Fixed lubber line — aircraft fore reference (does not rotate)
        cv.create_polygon(
            cx - 5, cy - r + 2,
            cx + 5, cy - r + 2,
            cx,     cy - r + 12,
            fill=self.C_LUBBER, outline=""
        )

        cv.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                       fill=self.C_LUBBER, outline=self.C_ROSE_BG, width=1)

    # ══════════════════════════════════════════════════════════════════════════
    # Field strength bars
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_bars(self, mx: int, my: int, mz: int):
        # Auto-scale: grow the range to 1.25x the largest axis value seen.
        # Never shrink below MAG_SCALE_MIN so bars stay readable.
        peak = max(abs(mx), abs(my), abs(mz), 1)
        self._mag_scale = max(self.MAG_SCALE_MIN, peak * 1.25)

        for axis, raw in [("X", mx), ("Y", my), ("Z", mz)]:
            entry = self._bars[axis]
            cv    = entry["canvas"]
            color = entry["color"]
            cv.delete("all")

            W = cv.winfo_width()  or 160
            H = cv.winfo_height() or 14
            mid = W // 2

            cv.create_rectangle(0, 2, W, H - 2,
                                 fill=self.C_BAR_BG, outline="#222222")
            cv.create_line(mid, 0, mid, H, fill="#333333", width=1)

            norm   = max(-1.0, min(1.0, raw / self._mag_scale))
            bar_px = int(abs(norm) * (mid - 2))

            if norm >= 0:
                cv.create_rectangle(mid,           3, mid + bar_px, H - 3,
                                    fill=color, outline="")
            else:
                cv.create_rectangle(mid - bar_px,  3, mid,          H - 3,
                                    fill=color, outline="")

            entry["label"].config(text=f"{raw:>6d}")

    # ══════════════════════════════════════════════════════════════════════════
    # Heading readout + signal status
    # ══════════════════════════════════════════════════════════════════════════

    def _update_readout(self, heading: float, valid: bool):
        if not valid:
            self.hdg_value_lbl.config(text="---.-°", fg=self.C_NOSIG_FG)
            self.cardinal_lbl.config( text="---",    fg=self.C_OFF)
            self.status_lbl.config(   text="● NO SIGNAL", fg=self.C_NOSIG_FG)
            self._hint_lbl.config(    fg=self.C_HINT)
            return

        self._hint_lbl.config(fg=self.C_BG)   # hide hint when valid

        hdg_norm = heading % 360.0
        nearest  = min(self.CARDINALS.keys(),
                       key=lambda k: abs((k - hdg_norm + 180) % 360 - 180))
        cardinal = self.CARDINALS[nearest]

        dev     = abs((nearest - hdg_norm + 180) % 360 - 180)
        hdg_clr = self.C_SAFE if dev < 5 else (self.C_WARN if dev < 20 else self.C_TEXT)

        self.hdg_value_lbl.config(text=f"{hdg_norm:05.1f}°", fg=hdg_clr)
        self.cardinal_lbl.config( text=cardinal,              fg=self.C_WARN)
        self.status_lbl.config(   text="● LOCK",             fg=self.C_SAFE)

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration UI state
    # ══════════════════════════════════════════════════════════════════════════

    def _update_mag_cal_ui(self, cal_active: bool, seconds_remaining: int):
        if cal_active:
            self._mag_cal_btn.config(state="disabled", text="⊕  CALIBRATING MAG…")
            self._mag_cal_status_lbl.config(
                text=f"Rotate on all axes — {seconds_remaining:2d}s remaining",
                fg=self.C_CAL_ACTIVE
            )
        else:
            self._mag_cal_btn.config(
                state="normal" if self._on_mag_cal else "disabled",
                text="⊕  CALIBRATE MAG"
            )
            self._mag_cal_status_lbl.config(text="")

    def _update_acc_cal_ui(self, cal_active: bool, seconds_remaining: int):
        if cal_active:
            self._acc_cal_btn.config(state="disabled", text="⊕  CALIBRATING…")
            self._acc_cal_status_lbl.config(
                text=f"Keep level & still — {seconds_remaining:2d}s remaining",
                fg="#44AAFF"
            )
        else:
            self._acc_cal_btn.config(
                state="normal" if self._on_acc_cal else "disabled",
                text="⊕  CALIBRATE GYRO/ACC"
            )
            self._acc_cal_status_lbl.config(text="")