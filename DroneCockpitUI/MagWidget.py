import tkinter as tk
import math


class MagWidget(tk.Frame):
    """
    QMC5883L Magnetometer display widget — aviation-standard, fully responsive.

    SAFETY INVARIANTS (never broken regardless of window size):
      ① Heading readout is always visible.
      ② Both calibration buttons are always accessible.

    Layout tiers (progressive enhancement — larger window adds content):
      MINI    (h<145 or w<285)  Heading box + two side-by-side buttons. No rose.
      COMPACT (h<225 or w<455)  Small rose (left) + Heading + Buttons (right).
      FULL    (anything larger)  Larger rose + Heading + Buttons + XYZ bars.

    Button anchor strategy (COMPACT + FULL):
      The right panel is split into two sub-frames:
        • top_frame  (row 0, weight=1) — heading + status. Expands/shrinks freely.
        • btn_frame  (row 2, weight=0) — MAG + ACC buttons. Fixed at bottom.
      As height decreases, the top section compresses; buttons never fall off.

    Column widths:
      Right panel column has minsize=152 px so "090.0°" at all supported fonts
      is never clipped regardless of how narrow the overall window becomes.
    """

    # ── Cockpit colour palette ───────────────────────────────────────────────
    C_BG         = "#0a0a10"
    C_ROSE_BG    = "#0d1117"
    C_ROSE_RING  = "#1e3040"
    C_LUBBER     = "#FFD700"
    C_CARDINAL   = "#c8d8e8"
    C_CARDINAL_N = "#FF4444"
    C_TICK_MINOR = "#1e2e3e"
    C_SAFE       = "#00d4ff"
    C_GREEN      = "#00ff88"
    C_WARN       = "#FFD700"
    C_CRIT       = "#FF4444"
    C_OFF        = "#444455"
    C_BAR_BG     = "#111118"
    C_TEXT       = "#a0b8c8"
    C_LABEL      = "#445566"
    C_NOSIG_FG   = "#FF4444"
    C_HINT       = "#664400"
    C_SEP        = "#1e2a3a"
    C_HDG_BG     = "#080f18"
    C_HDG_BORDER = "#1e4a6a"
    C_BTN_MAG_BG = "#082210"
    C_BTN_MAG_FG = "#00ff88"
    C_BTN_ACC_BG = "#080a20"
    C_BTN_ACC_FG = "#44aaff"

    MAG_SCALE_MIN = 800.0
    _mag_scale    = 800.0

    PAD = 6   # uniform outer padding (px)

    CARDINALS = {
        0:   "N",  45: "NE",  90:  "E",  135: "SE",
        180: "S", 225: "SW", 270:  "W",  315: "NW",
    }

    def __init__(self, parent, on_mag_calibrate=None, on_acc_calibrate=None):
        super().__init__(parent, bg=self.C_BG)
        self._last_heading = 0.0
        self._valid        = False
        self._on_mag_cal   = on_mag_calibrate
        self._on_acc_cal   = on_acc_calibrate
        self._current_tier = None
        self._tier_widgets = {}

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._container = tk.Frame(self, bg=self.C_BG)
        self._container.grid(row=0, column=0, sticky="nsew",
                             padx=self.PAD, pady=self.PAD)

        self.bind("<Configure>", self._on_resize)

    # ══════════════════════════════════════════════════════════════════════════
    # Resize / tier management
    # ══════════════════════════════════════════════════════════════════════════

    def _on_resize(self, event=None):
        w    = self.winfo_width()
        h    = self.winfo_height()
        tier = self._classify(w, h)
        if tier != self._current_tier:
            self._build_tier(tier)
            self._current_tier = tier
        self._redraw_dynamic()

    def _classify(self, w, h):
        if h < 145 or w < 285:
            return "mini"
        if h < 225 or w < 455:
            return "compact"
        return "full"

    # ══════════════════════════════════════════════════════════════════════════
    # Tier builders
    # ══════════════════════════════════════════════════════════════════════════

    def _clear_container(self):
        for widget in self._container.winfo_children():
            widget.destroy()
        self._tier_widgets = {}
        for i in range(10):
            self._container.columnconfigure(i, weight=0, minsize=0)
            self._container.rowconfigure(i,    weight=0, minsize=0)

    def _build_tier(self, tier):
        self._clear_container()
        {"full": self._build_full,
         "compact": self._build_compact,
         "mini": self._build_mini}[tier]()

    # ── FULL ─────────────────────────────────────────────────────────────────
    def _build_full(self):
        c = self._container
        c.columnconfigure(0, weight=3)
        c.columnconfigure(1, weight=2, minsize=152)   # heading never clips
        c.rowconfigure(0, weight=1)
        c.rowconfigure(1, weight=0)

        rose_cv = tk.Canvas(c, bg=self.C_BG, highlightthickness=0)
        rose_cv.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 4))
        rose_cv.bind("<Configure>", lambda e: self._draw_rose())
        self._tier_widgets["rose"] = rose_cv

        right = tk.Frame(c, bg=self.C_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 4))
        self._build_right_panel(right,
                                font_hdg=20, font_card=13,
                                font_status=9, font_btn=9,
                                tier="full")

        # XYZ bars span both columns
        bars_frame = tk.Frame(c, bg=self.C_BG)
        bars_frame.grid(row=1, column=0, columnspan=2,
                        sticky="ew", pady=(4, 0))
        bars_frame.columnconfigure(1, weight=1)
        tk.Label(bars_frame, text="RAW FIELD COMPONENTS",
                 font=("Consolas", 7, "bold"),
                 fg=self.C_LABEL, bg=self.C_BG,
                 ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 2))
        self._build_bars_into(bars_frame, start_row=1, bar_height=13)

    # ── COMPACT ───────────────────────────────────────────────────────────────
    def _build_compact(self):
        c = self._container
        c.columnconfigure(0, weight=3)
        c.columnconfigure(1, weight=2, minsize=152)   # heading never clips
        c.rowconfigure(0, weight=1)

        rose_cv = tk.Canvas(c, bg=self.C_BG, highlightthickness=0)
        rose_cv.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        rose_cv.bind("<Configure>", lambda e: self._draw_rose())
        self._tier_widgets["rose"] = rose_cv

        right = tk.Frame(c, bg=self.C_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        self._build_right_panel(right,
                                font_hdg=16, font_card=11,
                                font_status=8, font_btn=8,
                                tier="compact")

    # ── MINI ──────────────────────────────────────────────────────────────────
    def _build_mini(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.columnconfigure(1, weight=1)
        c.rowconfigure(0, weight=0)
        c.rowconfigure(1, weight=0)
        c.rowconfigure(2, weight=0)

        # Heading box — spans full width
        hdg_box = tk.Frame(c, bg=self.C_HDG_BG,
                           highlightbackground=self.C_HDG_BORDER,
                           highlightthickness=2)
        hdg_box.grid(row=0, column=0, columnspan=2,
                     sticky="ew", pady=(0, 3))
        hdg_box.columnconfigure(0, weight=1)

        hdg_val = tk.Label(hdg_box, text="---.-°",
                           font=("Consolas", 14, "bold"),
                           fg=self.C_CRIT, bg=self.C_HDG_BG, anchor="center")
        hdg_val.grid(row=0, column=0, sticky="ew", padx=6, pady=(4, 0))
        self._tier_widgets["hdg_val"] = hdg_val

        card_lbl = tk.Label(hdg_box, text="---",
                            font=("Consolas", 10, "bold"),
                            fg=self.C_WARN, bg=self.C_HDG_BG, anchor="center")
        card_lbl.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 4))
        self._tier_widgets["cardinal"] = card_lbl

        # Status
        status = tk.Label(c, text="⬤  NO SIGNAL",
                          font=("Consolas", 8, "bold"),
                          fg=self.C_NOSIG_FG, bg=self.C_BG, anchor="center")
        status.grid(row=1, column=0, columnspan=2, pady=(0, 3))
        self._tier_widgets["status"] = status

        # Buttons — side by side, tall touch target
        mag_btn = tk.Button(
            c, text="⊕ MAG",
            font=("Consolas", 9, "bold"),
            fg=self.C_BTN_MAG_FG, bg=self.C_BTN_MAG_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_MAG_FG,
            relief="flat", bd=0, padx=4, pady=10,
            cursor="hand2",
            command=self._on_mag_cal_pressed,
            state="normal" if self._on_mag_cal else "disabled")
        mag_btn.grid(row=2, column=0, sticky="ew", padx=(0, 2))
        self._tier_widgets["mag_btn"] = mag_btn

        acc_btn = tk.Button(
            c, text="⊕ GYRO",
            font=("Consolas", 9, "bold"),
            fg=self.C_BTN_ACC_FG, bg=self.C_BTN_ACC_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_ACC_FG,
            relief="flat", bd=0, padx=4, pady=10,
            cursor="hand2",
            command=self._on_acc_cal_pressed,
            state="normal" if self._on_acc_cal else "disabled")
        acc_btn.grid(row=2, column=1, sticky="ew", padx=(2, 0))
        self._tier_widgets["acc_btn"] = acc_btn

    # ══════════════════════════════════════════════════════════════════════════
    # Shared right panel — heading pinned top, buttons pinned bottom
    # ══════════════════════════════════════════════════════════════════════════

    def _build_right_panel(self, parent, font_hdg, font_card,
                           font_status, font_btn, tier):
        """
        Three-row layout so buttons are always visible at any height:

          row 0  weight=1   top_frame  — heading + status (shrinks, never hidden)
          row 1  weight=0   separator  — 1 px line
          row 2  weight=0   btn_frame  — MAG + ACC buttons (fixed, always visible)
        """
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)   # top section takes all spare height
        parent.rowconfigure(1, weight=0)   # separator is fixed
        parent.rowconfigure(2, weight=0)   # button section is fixed at bottom

        # ── Top sub-frame: heading + status ──────────────────────────────────
        top = tk.Frame(parent, bg=self.C_BG)
        top.grid(row=0, column=0, sticky="nsew")
        top.columnconfigure(0, weight=1)

        trow = 0
        if tier == "full":
            tk.Label(top, text="MAGNETIC HEADING",
                     font=("Consolas", 7, "bold"),
                     fg=self.C_LABEL, bg=self.C_BG,
                     ).grid(row=trow, column=0, pady=(2, 2))
            trow += 1

        # Heading box
        hdg_box = tk.Frame(top, bg=self.C_HDG_BG,
                           highlightbackground=self.C_HDG_BORDER,
                           highlightthickness=2)
        hdg_box.grid(row=trow, column=0, sticky="ew", ipady=3)
        hdg_box.columnconfigure(0, weight=1)
        trow += 1

        hdg_val = tk.Label(hdg_box, text="---.-°",
                           font=("Consolas", font_hdg, "bold"),
                           fg=self.C_CRIT, bg=self.C_HDG_BG, anchor="center")
        hdg_val.grid(row=0, column=0, sticky="ew", padx=4, pady=(4, 0))
        self._tier_widgets["hdg_val"] = hdg_val

        card_lbl = tk.Label(hdg_box, text="---",
                            font=("Consolas", font_card, "bold"),
                            fg=self.C_WARN, bg=self.C_HDG_BG, anchor="center")
        card_lbl.grid(row=1, column=0, sticky="ew", padx=4, pady=(0, 4))
        self._tier_widgets["cardinal"] = card_lbl

        # Status
        status = tk.Label(top, text="⬤  NO SIGNAL",
                          font=("Consolas", font_status, "bold"),
                          fg=self.C_NOSIG_FG, bg=self.C_BG, anchor="center")
        status.grid(row=trow, column=0, pady=(5, 2))
        self._tier_widgets["status"] = status
        trow += 1

        # Hint — full only, invisible when signal is valid
        if tier == "full":
            hint = tk.Label(top,
                            text="Set debug_mode = MAG_CALIB\nin Betaflight CLI",
                            font=("Consolas", 7),
                            fg=self.C_HINT, bg=self.C_BG, justify="center")
            hint.grid(row=trow, column=0, pady=(0, 2))
            self._tier_widgets["hint"] = hint

        # ── Separator ─────────────────────────────────────────────────────────
        tk.Frame(parent, bg=self.C_SEP, height=1).grid(
            row=1, column=0, sticky="ew", padx=2, pady=3)

        # ── Button sub-frame — pinned to bottom, always visible ───────────────
        btn = tk.Frame(parent, bg=self.C_BG)
        btn.grid(row=2, column=0, sticky="ew")
        btn.columnconfigure(0, weight=1)

        brow = 0

        label_mag = "⊕  CALIBRATE MAG" if tier == "full" else "⊕  CAL MAG"
        mag_btn = tk.Button(
            btn, text=label_mag,
            font=("Consolas", font_btn, "bold"),
            fg=self.C_BTN_MAG_FG, bg=self.C_BTN_MAG_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_MAG_FG,
            relief="flat", bd=0, padx=8, pady=9,
            cursor="hand2",
            command=self._on_mag_cal_pressed,
            state="normal" if self._on_mag_cal else "disabled")
        mag_btn.grid(row=brow, column=0, sticky="ew", pady=(0, 2))
        self._tier_widgets["mag_btn"] = mag_btn
        brow += 1

        if tier == "full":
            mag_lbl = tk.Label(btn, text="", font=("Consolas", 7),
                               fg=self.C_WARN, bg=self.C_BG, justify="center")
            mag_lbl.grid(row=brow, column=0)
            self._tier_widgets["mag_lbl"] = mag_lbl
            brow += 1

        label_acc = "⊕  CALIBRATE GYRO/ACC" if tier == "full" else "⊕  CAL GYRO"
        acc_btn = tk.Button(
            btn, text=label_acc,
            font=("Consolas", font_btn, "bold"),
            fg=self.C_BTN_ACC_FG, bg=self.C_BTN_ACC_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_ACC_FG,
            relief="flat", bd=0, padx=8, pady=9,
            cursor="hand2",
            command=self._on_acc_cal_pressed,
            state="normal" if self._on_acc_cal else "disabled")
        acc_btn.grid(row=brow, column=0, sticky="ew", pady=(0, 2))
        self._tier_widgets["acc_btn"] = acc_btn
        brow += 1

        if tier == "full":
            acc_lbl = tk.Label(btn, text="", font=("Consolas", 7),
                               fg=self.C_BTN_ACC_FG, bg=self.C_BG,
                               justify="center")
            acc_lbl.grid(row=brow, column=0)
            self._tier_widgets["acc_lbl"] = acc_lbl

    # ── Bar builder helper ─────────────────────────────────────────────────
    def _build_bars_into(self, parent, start_row=0, bar_height=13):
        bars = {}
        for i, (axis, color) in enumerate(
                [("X", "#4488FF"), ("Y", "#FF8844"), ("Z", "#44FF88")]):
            r = start_row + i

            tk.Label(parent, text=f" {axis} ",
                     font=("Consolas", 9, "bold"),
                     fg=color, bg=self.C_BG, width=3,
                     ).grid(row=r, column=0, sticky="w")

            bar_cv = tk.Canvas(parent, height=bar_height,
                               bg=self.C_BG, highlightthickness=0)
            bar_cv.grid(row=r, column=1, sticky="ew", padx=(4, 8))

            val_lbl = tk.Label(parent, text="    0",
                               font=("Consolas", 9),
                               fg=self.C_TEXT, bg=self.C_BG,
                               width=6, anchor="e")
            val_lbl.grid(row=r, column=2, sticky="e", padx=(0, 4))

            bars[axis] = {"canvas": bar_cv, "label": val_lbl, "color": color}

        self._tier_widgets["bars"] = bars

    # ══════════════════════════════════════════════════════════════════════════
    # Dynamic redraw dispatcher
    # ══════════════════════════════════════════════════════════════════════════

    def _redraw_dynamic(self):
        self._draw_rose()
        self._update_readout_widgets(self._last_heading, self._valid)

    # ══════════════════════════════════════════════════════════════════════════
    # Compass Rose — radius clamped so bottom overlay stays inside canvas
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_rose(self):
        cv = self._tier_widgets.get("rose")
        if cv is None:
            return

        cv.delete("all")
        W = cv.winfo_width()
        H = cv.winfo_height()
        if W < 20 or H < 20:
            return

        heading = self._last_heading
        valid   = self._valid
        cx = W // 2
        cy = H // 2

        # Overlay bottom ≈ cy + r×0.92 — clamp so it stays inside the canvas.
        r_from_size   = min(W, H) // 2 - 5
        r_from_bottom = int((H / 2 - 5) / 0.92)
        r = max(15, min(r_from_size, r_from_bottom))

        f_cardinal = max(8,  int(r * 0.17))
        f_numeral  = max(6,  int(r * 0.12))
        f_overlay  = max(8,  int(r * 0.15))
        f_nosig    = max(9,  int(r * 0.17))

        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       fill=self.C_ROSE_BG, outline=self.C_ROSE_RING, width=2)
        r2 = int(r * 0.88)
        cv.create_oval(cx - r2, cy - r2, cx + r2, cy + r2,
                       fill="", outline="#152535", width=1)

        if not valid:
            cv.create_text(cx, cy - int(r * 0.12), text="NO SIG",
                           fill=self.C_NOSIG_FG,
                           font=("Consolas", f_nosig, "bold"))
            if r > 40:
                cv.create_text(cx, cy + int(r * 0.22), text="debug_mode?",
                               fill=self.C_HINT,
                               font=("Consolas", max(6, f_nosig - 3)))
            return

        # Ticks
        tick_step = 5 if r > 70 else (10 if r > 40 else 30)
        for deg in range(0, 360, tick_step):
            is_major = (deg % 90 == 0)
            is_semi  = (deg % 45 == 0)
            is_10    = (deg % 10 == 0)

            if is_major:
                tick_len = max(5, int(r * 0.18))
                tick_w, tick_clr = 2, self.C_CARDINAL
            elif is_semi:
                tick_len = max(4, int(r * 0.13))
                tick_w, tick_clr = 1, "#3a6a8a"
            elif is_10:
                tick_len = max(3, int(r * 0.09))
                tick_w, tick_clr = 1, "#254555"
            else:
                tick_len = max(1, int(r * 0.05))
                tick_w, tick_clr = 1, self.C_TICK_MINOR

            rad = math.radians((deg - heading) - 90)
            ox  = cx + r              * math.cos(rad)
            oy  = cy + r              * math.sin(rad)
            ix  = cx + (r - tick_len) * math.cos(rad)
            iy  = cy + (r - tick_len) * math.sin(rad)
            cv.create_line(ix, iy, ox, oy, fill=tick_clr, width=tick_w)

        # Degree numerals every 30°
        if r > 55:
            num_r = int(r * 0.70)
            for deg in range(30, 360, 30):
                rad = math.radians((deg - heading) - 90)
                lx  = cx + num_r * math.cos(rad)
                ly  = cy + num_r * math.sin(rad)
                cv.create_text(lx, ly, text=str(deg // 10),
                               fill="#2a5a7a",
                               font=("Consolas", f_numeral))

        # Cardinals
        label_r = int(r * 0.72)
        for hdg_fixed, letter in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            if r < 40 and letter in ("E", "W"):
                continue
            rad   = math.radians((hdg_fixed - heading) - 90)
            lx    = cx + label_r * math.cos(rad)
            ly    = cy + label_r * math.sin(rad)
            color = self.C_CARDINAL_N if letter == "N" else self.C_CARDINAL
            cv.create_text(lx, ly, text=letter, fill=color,
                           font=("Consolas", f_cardinal, "bold"))

        # Lubber-line triangle (fixed at top, never rotates)
        tip_y  = cy - r + 2
        base_y = cy - r + max(7, int(r * 0.20))
        half_w = max(3, int(r * 0.07))
        cv.create_polygon(
            cx - half_w, base_y,
            cx + half_w, base_y,
            cx,          tip_y,
            fill=self.C_LUBBER, outline="#b09000", width=1)

        # Heading needle
        ptr_r = int(r * 0.55)
        arrow = (max(4, int(r * 0.10)),
                 max(5, int(r * 0.12)),
                 max(2, int(r * 0.04)))
        cv.create_line(cx, cy, cx, cy - ptr_r,
                       fill=self.C_LUBBER, width=2,
                       arrow=tk.LAST, arrowshape=arrow)

        # Centre dot
        dot = max(2, int(r * 0.05))
        cv.create_oval(cx - dot, cy - dot, cx + dot, cy + dot,
                       fill=self.C_LUBBER, outline=self.C_ROSE_BG, width=1)

        # Digital heading overlay — bottom of rose, always clamped inside canvas
        ov_y  = cy + int(r * 0.76)
        ov_hw = int(r * 0.46)
        ov_hh = max(9, int(r * 0.16))
        cv.create_rectangle(cx - ov_hw, ov_y - ov_hh,
                             cx + ov_hw, ov_y + ov_hh,
                             fill=self.C_HDG_BG,
                             outline=self.C_HDG_BORDER, width=1)
        cv.create_text(cx, ov_y,
                       text=f"{heading % 360:05.1f}°",
                       fill=self.C_SAFE,
                       font=("Consolas", f_overlay, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # Readout widget update
    # ══════════════════════════════════════════════════════════════════════════

    def _update_readout_widgets(self, heading: float, valid: bool):
        hdg_val  = self._tier_widgets.get("hdg_val")
        cardinal = self._tier_widgets.get("cardinal")
        status   = self._tier_widgets.get("status")
        hint     = self._tier_widgets.get("hint")

        if not valid:
            if hdg_val:  hdg_val.config(text="---.-°",      fg=self.C_CRIT)
            if cardinal: cardinal.config(text="---",         fg=self.C_OFF)
            if status:   status.config(text="⬤  NO SIGNAL", fg=self.C_NOSIG_FG)
            if hint:     hint.config(fg=self.C_HINT)
            return

        if hint:
            hint.config(fg=self.C_BG)   # invisible when valid

        hdg_norm = heading % 360.0
        nearest  = min(self.CARDINALS.keys(),
                       key=lambda k: abs((k - hdg_norm + 180) % 360 - 180))

        if hdg_val:  hdg_val.config(text=f"{hdg_norm:05.1f}°",    fg=self.C_SAFE)
        if cardinal: cardinal.config(text=self.CARDINALS[nearest], fg=self.C_WARN)
        if status:   status.config(text="⬤  LOCK",                fg=self.C_GREEN)

    # ══════════════════════════════════════════════════════════════════════════
    # Raw field bars
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_bars(self, mx: int, my: int, mz: int):
        bars = self._tier_widgets.get("bars")
        if not bars:
            return

        peak = max(abs(mx), abs(my), abs(mz), 1)
        self._mag_scale = max(self.MAG_SCALE_MIN, peak * 1.25)

        for axis, raw in [("X", mx), ("Y", my), ("Z", mz)]:
            if axis not in bars:
                continue
            entry = bars[axis]
            cv    = entry["canvas"]
            color = entry["color"]
            cv.delete("all")

            W   = cv.winfo_width()  or 120
            H   = cv.winfo_height() or 13
            mid = W // 2

            cv.create_rectangle(1, 1, W - 1, H - 1,
                                 fill=self.C_BAR_BG, outline="#1a2535")
            cv.create_line(mid, 1, mid, H - 1, fill="#1e2e40", width=1)

            norm   = max(-1.0, min(1.0, raw / self._mag_scale))
            bar_px = int(abs(norm) * (mid - 3))

            if norm >= 0:
                cv.create_rectangle(mid,          2, mid + bar_px, H - 2,
                                    fill=color, outline="")
            else:
                cv.create_rectangle(mid - bar_px, 2, mid,          H - 2,
                                    fill=color, outline="")

            entry["label"].config(text=f"{raw:>5d}")

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration button handlers
    # ══════════════════════════════════════════════════════════════════════════

    def _on_mag_cal_pressed(self):
        if self._on_mag_cal:
            self._on_mag_cal()
        btn = self._tier_widgets.get("mag_btn")
        if btn:
            btn.config(state="disabled", text="⊕  CALIBRATING MAG…")

    def _on_acc_cal_pressed(self):
        if self._on_acc_cal:
            self._on_acc_cal()
        btn = self._tier_widgets.get("acc_btn")
        if btn:
            btn.config(state="disabled", text="⊕  CALIBRATING…")

    # ══════════════════════════════════════════════════════════════════════════
    # Public update — called every telemetry tick
    # ══════════════════════════════════════════════════════════════════════════

    def update_mag(self, data: dict):
        valid       = data.get("mag_valid",                  False)
        heading     = data.get("mag_heading_deg",            0.0)
        mx          = data.get("mag_x",                      0)
        my          = data.get("mag_y",                      0)
        mz          = data.get("mag_z",                      0)
        mag_cal_act = data.get("mag_cal_active",             False)
        mag_cal_rem = data.get("mag_cal_seconds_remaining",  0)
        acc_cal_act = data.get("acc_cal_active",             False)
        acc_cal_rem = data.get("acc_cal_seconds_remaining",  0)

        self._valid        = valid
        self._last_heading = heading

        w    = self.winfo_width()
        h    = self.winfo_height()
        tier = self._classify(w, h)
        if tier != self._current_tier:
            self._build_tier(tier)
            self._current_tier = tier

        self._draw_rose()
        self._draw_bars(mx, my, mz)
        self._update_readout_widgets(heading, valid)
        self._update_mag_cal_ui(mag_cal_act, mag_cal_rem)
        self._update_acc_cal_ui(acc_cal_act, acc_cal_rem)

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration UI state
    # ══════════════════════════════════════════════════════════════════════════

    def _update_mag_cal_ui(self, cal_active: bool, seconds_remaining: int):
        mag_btn = self._tier_widgets.get("mag_btn")
        mag_lbl = self._tier_widgets.get("mag_lbl")
        if cal_active:
            if mag_btn:
                mag_btn.config(state="disabled", text="⊕  CALIBRATING MAG…")
            if mag_lbl:
                mag_lbl.config(
                    text=f"Rotate on all axes — {seconds_remaining:2d}s",
                    fg=self.C_WARN)
        else:
            if mag_btn:
                mag_btn.config(
                    state="normal" if self._on_mag_cal else "disabled",
                    text="⊕  CALIBRATE MAG" if self._current_tier == "full"
                    else "⊕  CAL MAG")
            if mag_lbl:
                mag_lbl.config(text="")

    def _update_acc_cal_ui(self, cal_active: bool, seconds_remaining: int):
        acc_btn = self._tier_widgets.get("acc_btn")
        acc_lbl = self._tier_widgets.get("acc_lbl")
        if cal_active:
            if acc_btn:
                acc_btn.config(state="disabled", text="⊕  CALIBRATING…")
            if acc_lbl:
                acc_lbl.config(
                    text=f"Keep level & still — {seconds_remaining:2d}s",
                    fg=self.C_BTN_ACC_FG)
        else:
            if acc_btn:
                acc_btn.config(
                    state="normal" if self._on_acc_cal else "disabled",
                    text="⊕  CALIBRATE GYRO/ACC" if self._current_tier == "full"
                    else "⊕  CAL GYRO")
            if acc_lbl:
                acc_lbl.config(text="")