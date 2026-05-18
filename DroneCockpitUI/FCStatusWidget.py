"""
FCStatusWidget.py  —  Flight Controller Status Panel
=====================================================
Priority-based responsive layout — critical info always visible.

LAYOUT FIXES (this version):
  1. P1 (ARM state, flight mode, battery strip, voltage bar, flight timer)
     is packed OUTSIDE the scroll area and is always visible regardless of
     window height.  Shrinking the window never hides the battery voltage or
     ARM state.
  2. P2 + P3 (battery detail, sensors, FC metrics, motors, RC channels) live
     inside a Canvas-backed scrollable frame.  A scrollbar appears when the
     content overflows vertically.
  3. Mousewheel scrolling is bound on every child widget inside the scroll
     area.
  4. Compact mode (motors/RC switch from bar graphs to dots) at width < 300
     is preserved.  An additional compact battery-strip mode at width < 270
     condenses the voltage display into a single row to save vertical space.
  5. Mode label wraps when text is too long for the available space.
  6. Sensors row wraps to multiple lines when too narrow to fit all pills.
"""

import tkinter as tk
import time

# ── Palette ───────────────────────────────────────────────────────────────────
_BG       = "#0f0f1a"
_BG2      = "#12121f"
_BG3      = "#0d1525"
_BORDER   = "#1e2a3a"
_LABEL_FG = "#4a6080"
_VALUE_FG = "#c8dff0"
_GREEN    = "#00ff88"
_ORANGE   = "#ffaa00"
_RED      = "#ff3355"
_DIM      = "#1a2a3a"
_DIM_FG   = "#2a4055"
_BLUE     = "#00aaff"

_F_MICRO  = ("Consolas",  7)
_F_TINY   = ("Consolas",  7, "bold")
_F_SMALL  = ("Consolas",  8, "bold")
_F_BODY   = ("Consolas",  9)
_F_VALUE  = ("Consolas", 10, "bold")
_F_ARMED  = ("Consolas", 14, "bold")
_F_MODE   = ("Consolas", 11, "bold")
_F_VOLT   = ("Consolas", 18, "bold")
_F_VOLT_C = ("Consolas", 11, "bold")   # compact voltage font
_F_TIMER  = ("Consolas", 13, "bold")
_F_WARN   = ("Consolas", 10, "bold")

# ARM channel threshold — single authoritative value.
# Must match the minimum of the ARM range in BF Modes tab (default 1800–2100).
ARM_THRESHOLD = 1800

_SENSORS = [
    ("ACC",  "sensor_acc_present"),
    ("BARO", "sensor_baro_present"),
    ("MAG",  "sensor_mag_present"),
    ("GPS",  "sensor_gps_present"),
    ("RNG",  "sensor_rangefinder_present"),
    ("GYRO", "sensor_gyro_present"),
]

_RC_CHANNELS = [
    ("ROLL", "rc_roll"),
    ("PTCH", "rc_pitch"),
    ("THRO", "rc_throttle"),
    ("YAW",  "rc_yaw"),
    ("ARM",  "rc_arm"),
]

_MOTOR_KEYS = ["motor_1_us", "motor_2_us", "motor_3_us", "motor_4_us"]

_CELL_CRIT = 3.40
_CELL_WARN = 3.65
_CELL_FULL = 4.20

_BAT_STATE_COLORS = {
    "OK":          (_GREEN,  _BG3),
    "WARNING":     (_ORANGE, _BG3),
    "CRITICAL":    (_RED,    _BG3),
    "NOT_PRESENT": (_DIM_FG, _BORDER),
    "INIT":        (_DIM_FG, _BORDER),
    "UNKNOWN":     (_DIM_FG, _BORDER),
}

# Width below which the battery strip switches to the compact single-row layout.
_COMPACT_BAT_W = 270
# Width below which motor/RC bars switch to dots.
_COMPACT_MOT_W = 300


def _sep(parent):
    tk.Frame(parent, bg=_BORDER, height=1).pack(fill="x", padx=6, pady=1)


def _sec_hdr(parent, text):
    f = tk.Frame(parent, bg=_BG)
    f.pack(fill="x", padx=6, pady=(3, 1))
    tk.Label(f, text=text, bg=_BG, fg=_LABEL_FG, font=_F_TINY).pack(side="left")
    tk.Frame(f, bg=_BORDER, height=1).pack(
        side="left", fill="x", expand=True, padx=(4, 0), pady=4)


class FCStatusWidget(tk.Frame):

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_BG, **kwargs)
        self._last_data: dict  = {}
        self._planned_sec: int = 600
        self._arm_start: float = 0.0
        self._was_armed: bool  = False
        self._flash_on: bool   = False
        self._width: int       = 500
        self._motor_mode       = "bars"
        self._rc_mode          = "bars"
        self._bat_compact      = False
        self._build_ui()
        self.bind("<Configure>", self._on_resize)
        self._tick()

    # =========================================================================
    # Public
    # =========================================================================

    def set_planned_duration(self, minutes: float):
        self._planned_sec = int(minutes * 60)
        try:
            self._plan_spin.delete(0, "end")
            self._plan_spin.insert(0, str(int(minutes)))
        except Exception:
            pass

    # =========================================================================
    # Build — P1 (always visible, NOT inside scroll area)
    # =========================================================================

    def _build_p1(self):
        """Build the always-visible top section (ARM, battery, timer)."""

        # ── P1a: ARM state + flight mode ──────────────────────────────────────
        p1 = tk.Frame(self, bg=_BG2,
                      highlightthickness=1, highlightbackground=_BORDER)
        p1.pack(fill="x", padx=6, pady=(6, 2))
        self._p1_frame = p1

        arm_blk = tk.Frame(p1, bg=_BG2)
        arm_blk.pack(side="left", padx=(8, 4), pady=6)
        self._arm_cv = tk.Canvas(arm_blk, width=16, height=16,
                                  bg=_BG2, highlightthickness=0)
        self._arm_cv.pack(side="left", padx=(0, 5))
        self._arm_dot = self._arm_cv.create_oval(1, 1, 15, 15,
                                                   fill=_RED, outline="")
        self._arm_lbl = tk.Label(arm_blk, text="DISARMED",
                                  bg=_BG2, fg=_RED, font=_F_ARMED)
        self._arm_lbl.pack(side="left")

        tk.Frame(p1, bg=_BORDER, width=1).pack(
            side="left", fill="y", padx=(10, 10), pady=4)

        # Mode block — allow text to wrap so long mode names don't get clipped
        mode_blk = tk.Frame(p1, bg=_BG2)
        mode_blk.pack(side="left", pady=6, fill="x", expand=True)
        tk.Label(mode_blk, text="MODE", bg=_BG2, fg=_LABEL_FG,
                 font=_F_SMALL).pack(anchor="w")
        self._mode_lbl = tk.Label(mode_blk, text="—", bg=_BG2,
                                   fg=_ORANGE, font=_F_MODE,
                                   anchor="w", justify="left",
                                   wraplength=1)   # updated on resize
        self._mode_lbl.pack(anchor="w", fill="x", expand=True)
        # Keep a reference to update wraplength on resize
        self._mode_blk = mode_blk

        # ── P1b: Battery strip (full layout) ──────────────────────────────────
        self._bat_strip = tk.Frame(self, bg=_BG3,
                                    highlightthickness=1,
                                    highlightbackground=_BORDER)
        self._bat_strip.pack(fill="x", padx=6, pady=(0, 2))

        # Full layout — big voltage number
        self._bat_full_row = tk.Frame(self._bat_strip, bg=_BG3)
        self._bat_full_row.pack(fill="x")

        vblk = tk.Frame(self._bat_full_row, bg=_BG3)
        vblk.pack(side="left", padx=(10, 0), pady=6)
        tk.Label(vblk, text="VBAT", bg=_BG3, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        vrow = tk.Frame(vblk, bg=_BG3)
        vrow.pack(anchor="w")
        self._big_volt = tk.Label(vrow, text="—", bg=_BG3,
                                   fg=_GREEN, font=_F_VOLT)
        self._big_volt.pack(side="left")
        tk.Label(vrow, text="V", bg=_BG3, fg=_LABEL_FG,
                 font=_F_VALUE).pack(side="left", anchor="s", padx=(2, 0))

        cblk = tk.Frame(self._bat_full_row, bg=_BG3)
        cblk.pack(side="left", padx=(12, 0), pady=6)
        tk.Label(cblk, text="PER CELL", bg=_BG3, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        self._cell_lbl = tk.Label(cblk, text="—  V", bg=_BG3,
                                   fg=_GREEN, font=_F_VALUE)
        self._cell_lbl.pack(anchor="w")

        rblk = tk.Frame(self._bat_full_row, bg=_BG3)
        rblk.pack(side="right", padx=(0, 10), pady=6)
        self._bat_state_lbl = tk.Label(rblk, text="INIT",
                                        bg=_BORDER, fg=_DIM_FG,
                                        font=_F_SMALL, padx=6, pady=2)
        self._bat_state_lbl.pack(anchor="e")
        self._bat_pct_lbl = tk.Label(rblk, text="—%", bg=_BG3,
                                      fg=_VALUE_FG, font=_F_VALUE)
        self._bat_pct_lbl.pack(anchor="e", pady=(2, 0))

        # Compact layout — single row, smaller font
        self._bat_compact_row = tk.Frame(self._bat_strip, bg=_BG3)
        # (not packed until _apply_compact_bat switches modes)

        cv_row = tk.Frame(self._bat_compact_row, bg=_BG3)
        cv_row.pack(side="left", padx=(8, 0), pady=4)
        self._big_volt_c = tk.Label(cv_row, text="—", bg=_BG3,
                                     fg=_GREEN, font=_F_VOLT_C)
        self._big_volt_c.pack(side="left")
        tk.Label(cv_row, text="V", bg=_BG3, fg=_LABEL_FG,
                 font=_F_TINY).pack(side="left", anchor="s")

        tk.Frame(self._bat_compact_row, bg=_BORDER, width=1).pack(
            side="left", fill="y", padx=6, pady=3)

        self._cell_lbl_c = tk.Label(self._bat_compact_row, text="— V/cell",
                                     bg=_BG3, fg=_GREEN, font=_F_TINY)
        self._cell_lbl_c.pack(side="left")

        self._bat_state_lbl_c = tk.Label(self._bat_compact_row, text="INIT",
                                          bg=_BORDER, fg=_DIM_FG,
                                          font=_F_TINY, padx=4, pady=1)
        self._bat_state_lbl_c.pack(side="right", padx=(0, 8), pady=4)

        # ── P1c: Voltage bar ──────────────────────────────────────────────────
        vbf = tk.Frame(self, bg=_BG)
        vbf.pack(fill="x", padx=6, pady=(0, 2))
        tk.Label(vbf, text="VBAT", bg=_BG, fg=_LABEL_FG,
                 font=_F_MICRO).pack(side="left", padx=(2, 3))
        self._vbat_outer = tk.Frame(vbf, bg=_BORDER, height=8)
        self._vbat_outer.pack(side="left", fill="x", expand=True)
        self._vbat_inner = tk.Frame(self._vbat_outer, bg=_GREEN, height=8)
        self._vbat_inner.place(x=0, y=0, relheight=1.0, width=0)
        self._vbat_outer.bind(
            "<Configure>", lambda e: self.after_idle(self._redraw_vbat))

        # ── P1d: Flight timer ─────────────────────────────────────────────────
        tf = tk.Frame(self, bg=_BG2,
                      highlightthickness=1, highlightbackground=_BORDER)
        tf.pack(fill="x", padx=6, pady=(0, 2))

        el_blk = tk.Frame(tf, bg=_BG2)
        el_blk.pack(side="left", padx=(8, 0), pady=5)
        tk.Label(el_blk, text="FLT TIME", bg=_BG2, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        self._elapsed_lbl = tk.Label(el_blk, text="00:00", bg=_BG2,
                                      fg=_DIM_FG, font=_F_TIMER)
        self._elapsed_lbl.pack(anchor="w")

        rm_blk = tk.Frame(tf, bg=_BG2)
        rm_blk.pack(side="left", padx=(16, 0), pady=5)
        tk.Label(rm_blk, text="REMAINING", bg=_BG2, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        self._remain_lbl = tk.Label(rm_blk, text="—:——", bg=_BG2,
                                     fg=_DIM_FG, font=_F_TIMER)
        self._remain_lbl.pack(anchor="w")

        plan_blk = tk.Frame(tf, bg=_BG2)
        plan_blk.pack(side="right", padx=(0, 8), pady=5)
        tk.Label(plan_blk, text="PLANNED", bg=_BG2, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="e")
        spin_row = tk.Frame(plan_blk, bg=_BG2)
        spin_row.pack(anchor="e")
        self._plan_spin = tk.Spinbox(
            spin_row, from_=1, to=120, width=3,
            bg=_BG, fg=_VALUE_FG, buttonbackground=_BORDER,
            font=_F_VALUE, relief="flat",
            highlightthickness=1, highlightbackground=_BORDER,
            command=self._on_plan_change,
        )
        self._plan_spin.delete(0, "end")
        self._plan_spin.insert(0, "10")
        self._plan_spin.pack(side="left")
        tk.Label(spin_row, text="min", bg=_BG2, fg=_LABEL_FG,
                 font=_F_BODY).pack(side="left", padx=(3, 0))
        self._plan_spin.bind("<Return>",   lambda e: self._on_plan_change())
        self._plan_spin.bind("<FocusOut>", lambda e: self._on_plan_change())

        # Time warning strip (conditional — shown/hidden by _tick)
        self._twarn_outer = tk.Frame(self, bg=_BG,
                                      highlightthickness=1,
                                      highlightbackground=_BG)
        self._twarn_lbl = tk.Label(self._twarn_outer, text="",
                                    bg=_BG, fg=_ORANGE, font=_F_WARN)
        self._twarn_lbl.pack(pady=3)

    # =========================================================================
    # Build — P2 + P3 (inside scroll area)
    # =========================================================================

    def _build_scrollable(self):
        """Build the scrollable section containing P2 and P3."""

        sc_container = tk.Frame(self, bg=_BG)
        sc_container.pack(fill="both", expand=True)

        self._scroll_canvas = tk.Canvas(sc_container, bg=_BG,
                                         highlightthickness=0, bd=0)
        self._vscroll = tk.Scrollbar(sc_container, orient="vertical",
                                      command=self._scroll_canvas.yview)
        self._scroll_canvas.configure(yscrollcommand=self._vscroll.set)

        self._vscroll.pack(side="right", fill="y")
        self._scroll_canvas.pack(side="left", fill="both", expand=True)

        self._scroll_inner = tk.Frame(self._scroll_canvas, bg=_BG)
        self._canvas_win = self._scroll_canvas.create_window(
            (0, 0), window=self._scroll_inner, anchor="nw")

        self._scroll_inner.bind("<Configure>", self._on_inner_configure)
        self._scroll_canvas.bind("<Configure>", self._on_canvas_configure)

        self._bind_mw(self._scroll_canvas)
        self._bind_mw(self._scroll_inner)

        p = self._scroll_inner   # shorthand

        # ── P2: Battery detail ────────────────────────────────────────────────
        _sep(p)
        _sec_hdr(p, "BATTERY DETAIL")
        bat_det = tk.Frame(p, bg=_BG)
        bat_det.pack(fill="x", padx=6, pady=(0, 3))
        self._bat_curr_lbl  = self._mini_stat(bat_det, "CURRENT", "A")
        self._bat_mah_lbl   = self._mini_stat(bat_det, "USED",   "mAh")
        self._bat_cells_lbl = self._mini_stat(bat_det, "CELLS",    "S")
        self._bind_mw(bat_det)

        # ── P2: Sensors ───────────────────────────────────────────────────────
        _sep(p)
        _sec_hdr(p, "SENSORS")
        # Pills are placed with grid so we can change ncols without re-parenting.
        self._sens_outer = tk.Frame(p, bg=_BG)
        self._sens_outer.pack(fill="x", padx=6, pady=(0, 3))
        self._sensor_refs: dict = {}
        self._sensor_pills: list = []
        self._sens_ncols: int = 0          # track current column count
        for sname, _ in _SENSORS:
            pill = tk.Frame(self._sens_outer, bg=_DIM,
                            highlightthickness=1, highlightbackground=_BORDER)
            cv = tk.Canvas(pill, width=9, height=9,
                           bg=_DIM, highlightthickness=0)
            cv.pack(side="left", padx=(3, 1), pady=2)
            dot = cv.create_oval(1, 1, 8, 8, fill=_DIM_FG, outline="")
            lbl = tk.Label(pill, text=sname, bg=_DIM, fg=_DIM_FG,
                           font=_F_TINY)
            lbl.pack(side="left", padx=(0, 4), pady=2)
            self._sensor_refs[sname] = (cv, dot, lbl, pill)
            self._sensor_pills.append(pill)
            for w in (pill, cv, lbl):
                self._bind_mw(w)
        # Initial grid layout — all 6 pills in one row
        self._relayout_sensors(6)
        self._bind_mw(self._sens_outer)
        self._sens_outer.bind("<Configure>", self._on_sens_configure)

        # ── P3: FC metrics ────────────────────────────────────────────────────
        _sep(p)
        _sec_hdr(p, "FC METRICS")
        met_row = tk.Frame(p, bg=_BG)
        met_row.pack(fill="x", padx=6, pady=(0, 3))
        cpu_blk = tk.Frame(met_row, bg=_BG)
        cpu_blk.pack(side="left", padx=(2, 14), fill="y")
        tk.Label(cpu_blk, text="CPU", bg=_BG, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        self._cpu_outer = tk.Frame(cpu_blk, bg=_BORDER, height=7, width=55)
        self._cpu_outer.pack(fill="x", anchor="w")
        self._cpu_outer.pack_propagate(False)
        self._cpu_inner = tk.Frame(self._cpu_outer, bg=_GREEN, height=7)
        self._cpu_inner.place(x=0, y=0, relheight=1.0, width=0)
        self._cpu_lbl = tk.Label(cpu_blk, text="0%", bg=_BG,
                                  fg=_GREEN, font=_F_VALUE)
        self._cpu_lbl.pack(anchor="w")
        self._lbl_cycle = self._mini_stat(met_row, "LOOP",    "ms")
        self._lbl_i2c   = self._mini_stat(met_row, "I2C ERR")
        self._lbl_pid   = self._mini_stat(met_row, "PID")
        self._bind_mw(met_row)

        # ── P3: Motors ────────────────────────────────────────────────────────
        _sep(p)
        mot_hdr = tk.Frame(p, bg=_BG)
        mot_hdr.pack(fill="x", padx=6, pady=(3, 1))
        tk.Label(mot_hdr, text="MOTORS", bg=_BG, fg=_LABEL_FG,
                 font=_F_TINY).pack(side="left")
        self._mot_unit_lbl = tk.Label(mot_hdr, text="(us)", bg=_BG,
                                       fg=_LABEL_FG, font=_F_MICRO)
        self._mot_unit_lbl.pack(side="left", padx=(3, 0))
        tk.Frame(mot_hdr, bg=_BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(4, 0), pady=4)

        self._mot_wrapper = tk.Frame(p, bg=_BG)
        self._mot_wrapper.pack(fill="x", padx=6, pady=(0, 3))
        self._bind_mw(self._mot_wrapper)

        self._mot_bar_f = tk.Frame(self._mot_wrapper, bg=_BG)
        self._mot_bar_f.pack(fill="x")

        self._motor_bars:   list = []
        self._motor_lbls:   list = []
        self._motor_outers: list = []
        for i in range(4):
            blk = tk.Frame(self._mot_bar_f, bg=_BG2,
                           highlightthickness=1, highlightbackground=_BORDER)
            blk.pack(side="left", expand=True, fill="x", padx=2, pady=1)
            tk.Label(blk, text=f"M{i+1}", bg=_BG2, fg=_LABEL_FG,
                     font=_F_MICRO).pack(anchor="w", padx=3, pady=(2, 0))
            bo = tk.Frame(blk, bg=_BORDER, height=36, width=18)
            bo.pack(padx=5, pady=(1, 1))
            bo.pack_propagate(False)
            bi = tk.Frame(bo, bg=_DIM_FG)
            bi.place(x=0, rely=1.0, relwidth=1.0, height=0, anchor="sw")
            vl = tk.Label(blk, text="—", bg=_BG2, fg=_VALUE_FG, font=_F_MICRO)
            vl.pack(pady=(0, 2))
            self._motor_outers.append(bo)
            self._motor_bars.append(bi)
            self._motor_lbls.append(vl)
            self._bind_mw(blk)

        self._mot_dot_f = tk.Frame(self._mot_wrapper, bg=_BG)

        self._motor_dots: list = []
        dot_row = tk.Frame(self._mot_dot_f, bg=_BG)
        dot_row.pack(fill="x", padx=6, pady=2)
        for i in range(4):
            blk = tk.Frame(dot_row, bg=_BG)
            blk.pack(side="left", padx=(0, 8))
            cv = tk.Canvas(blk, width=13, height=13,
                           bg=_BG, highlightthickness=0)
            cv.pack(side="left", padx=(0, 3))
            d = cv.create_oval(1, 1, 12, 12, fill=_DIM_FG, outline="")
            ll = tk.Label(blk, text=f"M{i+1}", bg=_BG, fg=_DIM_FG,
                          font=_F_TINY)
            ll.pack(side="left")
            self._motor_dots.append((cv, d, ll))
            for w in (blk, cv, ll):
                self._bind_mw(w)

        # ── P3: RC channels ───────────────────────────────────────────────────
        _sep(p)
        rc_hdr = tk.Frame(p, bg=_BG)
        rc_hdr.pack(fill="x", padx=6, pady=(3, 1))
        tk.Label(rc_hdr, text="RC CHANNELS", bg=_BG, fg=_LABEL_FG,
                 font=_F_TINY).pack(side="left")
        tk.Frame(rc_hdr, bg=_BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(4, 0), pady=4)

        self._rc_wrapper = tk.Frame(p, bg=_BG)
        self._rc_wrapper.pack(fill="x", padx=6, pady=(0, 3))
        self._bind_mw(self._rc_wrapper)

        self._rc_bar_f = tk.Frame(self._rc_wrapper, bg=_BG)
        self._rc_bar_f.pack(fill="x")

        self._rc_bars:   list = []
        self._rc_lbls:   list = []
        self._rc_outers: list = []
        for ch_name, _ in _RC_CHANNELS:
            blk = tk.Frame(self._rc_bar_f, bg=_BG2,
                           highlightthickness=1, highlightbackground=_BORDER)
            blk.pack(side="left", expand=True, fill="x", padx=2, pady=1)
            tk.Label(blk, text=ch_name, bg=_BG2, fg=_LABEL_FG,
                     font=_F_MICRO).pack(anchor="w", padx=3, pady=(2, 0))
            bo = tk.Frame(blk, bg=_BORDER, height=36, width=18)
            bo.pack(padx=5, pady=(1, 1))
            bo.pack_propagate(False)
            bi = tk.Frame(bo, bg=_BLUE)
            bi.place(x=0, rely=0.5, relwidth=1.0, height=0, anchor="w")
            vl = tk.Label(blk, text="—", bg=_BG2, fg=_VALUE_FG, font=_F_MICRO)
            vl.pack(pady=(0, 2))
            self._rc_outers.append(bo)
            self._rc_bars.append(bi)
            self._rc_lbls.append(vl)
            self._bind_mw(blk)

        self._rc_dot_f = tk.Frame(self._rc_wrapper, bg=_BG)

        self._rc_dots: list = []
        rdot_row = tk.Frame(self._rc_dot_f, bg=_BG)
        rdot_row.pack(fill="x", padx=6, pady=2)
        for ch_name, _ in _RC_CHANNELS:
            blk = tk.Frame(rdot_row, bg=_BG)
            blk.pack(side="left", padx=(0, 8))
            cv = tk.Canvas(blk, width=13, height=13,
                           bg=_BG, highlightthickness=0)
            cv.pack(side="left", padx=(0, 3))
            d = cv.create_oval(1, 1, 12, 12, fill=_DIM_FG, outline="")
            ll = tk.Label(blk, text=ch_name[:2], bg=_BG, fg=_DIM_FG,
                          font=_F_TINY)
            ll.pack(side="left")
            self._rc_dots.append((cv, d, ll))
            for w in (blk, cv, ll):
                self._bind_mw(w)

        lq_f = tk.Frame(p, bg=_BG)
        lq_f.pack(fill="x", padx=8, pady=(0, 5))
        tk.Label(lq_f, text="LINK QUALITY", bg=_BG, fg=_LABEL_FG,
                 font=_F_MICRO).pack(side="left")
        self._lq_lbl = tk.Label(lq_f, text="—%", bg=_BG,
                                  fg=_VALUE_FG, font=_F_TINY)
        self._lq_lbl.pack(side="left", padx=5)
        self._bind_mw(lq_f)

    def _build_ui(self):
        self._build_p1()
        self._build_scrollable()

    # =========================================================================
    # Helpers
    # =========================================================================

    def _mini_stat(self, parent, label, unit=""):
        blk = tk.Frame(parent, bg=_BG)
        blk.pack(side="left", padx=(0, 14), fill="y")
        tk.Label(blk, text=label, bg=_BG, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        vrow = tk.Frame(blk, bg=_BG)
        vrow.pack(anchor="w")
        v = tk.Label(vrow, text="—", bg=_BG, fg=_VALUE_FG, font=_F_VALUE)
        v.pack(side="left")
        if unit:
            tk.Label(vrow, text=unit, bg=_BG, fg=_LABEL_FG,
                     font=_F_MICRO).pack(side="left", padx=(2, 0))
        self._bind_mw(blk)
        return v

    # =========================================================================
    # Mousewheel helpers
    # =========================================================================

    def _bind_mw(self, widget):
        widget.bind("<MouseWheel>", self._on_mousewheel, add="+")
        widget.bind("<Button-4>",   self._on_mousewheel, add="+")
        widget.bind("<Button-5>",   self._on_mousewheel, add="+")

    def _on_mousewheel(self, event):
        if event.num == 4:
            self._scroll_canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self._scroll_canvas.yview_scroll(1, "units")
        else:
            self._scroll_canvas.yview_scroll(
                int(-1 * (event.delta / 120)), "units")

    # =========================================================================
    # Canvas / scroll event handlers
    # =========================================================================

    def _on_inner_configure(self, _event):
        self._scroll_canvas.configure(
            scrollregion=self._scroll_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self._scroll_canvas.itemconfig(self._canvas_win, width=event.width)

    # =========================================================================
    # Sensors wrap layout
    # =========================================================================

    def _on_sens_configure(self, event):
        """Re-grid sensor pills when the container width changes."""
        self.after_idle(lambda: self._relayout_sensors_by_width(event.width))

    def _relayout_sensors_by_width(self, available_w: int):
        """Choose the right number of columns based on available width and re-grid."""
        # Each pill is roughly 48 px wide including padding.
        PILL_W = 48
        ncols = max(3, min(len(self._sensor_pills),
                           max(1, available_w // PILL_W)))
        self._relayout_sensors(ncols)

    def _relayout_sensors(self, ncols: int):
        """Place all sensor pills into a grid with `ncols` columns.

        All pills remain children of _sens_outer — we never re-parent them,
        so there is no TclError. Only grid() coordinates change.
        """
        if ncols == self._sens_ncols:
            return
        self._sens_ncols = ncols

        for col in range(ncols):
            self._sens_outer.columnconfigure(col, weight=0)

        for idx, pill in enumerate(self._sensor_pills):
            row = idx // ncols
            col = idx % ncols
            pill.grid(row=row, column=col, padx=2, pady=1, sticky="w")

    # =========================================================================
    # Resize handler
    # =========================================================================

    def _on_resize(self, event):
        w = event.width
        if w == self._width:
            return
        self._width = w

        # ── Mode label wraplength ─────────────────────────────────────────────
        # Approximate available width for mode text: total - arm block (~140px) - padding
        mode_wrap = max(60, w - 160)
        self._mode_lbl.config(wraplength=mode_wrap)

        # ── Motor display mode ────────────────────────────────────────────────
        mode = "bars" if w >= _COMPACT_MOT_W else "dots"
        if mode != self._motor_mode:
            self._motor_mode = mode
            if mode == "bars":
                self._mot_dot_f.pack_forget()
                self._mot_bar_f.pack(fill="x")
                self._mot_unit_lbl.config(text="(us)")
            else:
                self._mot_bar_f.pack_forget()
                self._mot_dot_f.pack(fill="x")
                self._mot_unit_lbl.config(text="")

        # ── RC display mode ───────────────────────────────────────────────────
        if mode != self._rc_mode:
            self._rc_mode = mode
            if mode == "bars":
                self._rc_dot_f.pack_forget()
                self._rc_bar_f.pack(fill="x")
            else:
                self._rc_bar_f.pack_forget()
                self._rc_dot_f.pack(fill="x")

        # ── Battery strip compact mode ────────────────────────────────────────
        compact_bat = w < _COMPACT_BAT_W
        if compact_bat != self._bat_compact:
            self._bat_compact = compact_bat
            self._apply_compact_bat(compact_bat)

        self.after_idle(self._redraw_cpu)
        self.after_idle(self._redraw_vbat)

    def _apply_compact_bat(self, compact: bool):
        """Switch the battery strip between full (big volt) and compact layouts."""
        if compact:
            self._bat_full_row.pack_forget()
            self._bat_compact_row.pack(fill="x")
        else:
            self._bat_compact_row.pack_forget()
            self._bat_full_row.pack(fill="x")

    # =========================================================================
    # Redraw helpers
    # =========================================================================

    def _redraw_cpu(self):
        cpu = max(0, min(100, int(self._last_data.get("cpu_load_percent", 0))))
        w = self._cpu_outer.winfo_width()
        if w < 4:
            return
        fill_w = max(1, int(w * cpu / 100))
        col = _RED if cpu > 80 else _ORANGE if cpu > 60 else _GREEN
        self._cpu_inner.place(x=0, y=0, width=fill_w, relheight=1.0)
        self._cpu_inner.config(bg=col)
        self._cpu_lbl.config(text=f"{cpu}%", fg=col)

    def _redraw_vbat(self):
        d = self._last_data
        v = float(d.get("battery_voltage", 0.0))
        c = self._detect_cells(d)
        if c == 0 or v == 0:
            return
        cell_v = v / c
        ratio  = max(0.0, min(1.0,
                    (cell_v - _CELL_CRIT) / (_CELL_FULL - _CELL_CRIT)))
        w = self._vbat_outer.winfo_width()
        if w < 4:
            return
        col = _RED if ratio < 0.15 else _ORANGE if ratio < 0.35 else _GREEN
        self._vbat_inner.place(x=0, y=0, relheight=1.0,
                                width=max(1, int(w * ratio)))
        self._vbat_inner.config(bg=col)

    @staticmethod
    def _detect_cells(d: dict) -> int:
        c = int(d.get("battery_cell_count", 0))
        if c > 0:
            return c
        v = float(d.get("battery_voltage", 0.0))
        for n in range(1, 9):
            if v <= n * 4.25 + 0.1:
                return n
        return 0

    def _cell_color(self, cv: float) -> str:
        return _RED if cv < _CELL_CRIT else _ORANGE if cv < _CELL_WARN else _GREEN

    # =========================================================================
    # Flight timer
    # =========================================================================

    def _on_plan_change(self):
        try:
            self._planned_sec = max(60, int(float(self._plan_spin.get()) * 60))
        except ValueError:
            pass

    def _tick(self):
        armed = bool(self._last_data.get("armed", False))
        if armed and not self._was_armed:
            self._arm_start = time.monotonic()
        self._was_armed = armed

        elapsed = int(time.monotonic() - self._arm_start) if (armed and self._arm_start) else 0
        em, es  = divmod(elapsed, 60)
        self._elapsed_lbl.config(
            text=f"{em:02d}:{es:02d}",
            fg=_GREEN if armed else _DIM_FG)

        if self._planned_sec > 0:
            remaining = max(0, self._planned_sec - elapsed)
            rm, rs = divmod(remaining, 60)
            self._remain_lbl.config(text=f"{rm:02d}:{rs:02d}")

            if not armed:
                self._remain_lbl.config(fg=_DIM_FG)
                self._twarn_outer.pack_forget()
            elif remaining == 0:
                self._remain_lbl.config(fg=_RED)
                self._twarn_outer.pack(fill="x", padx=6, pady=(0, 2))
                self._twarn_outer.config(highlightbackground=_RED)
                self._twarn_lbl.config(
                    text="WARNING — ENDURANCE EXCEEDED — LAND IMMEDIATELY",
                    fg=_RED)
            elif remaining <= 30:
                self._flash_on = not self._flash_on
                fc = _RED if self._flash_on else _ORANGE
                self._remain_lbl.config(fg=fc)
                self._twarn_outer.pack(fill="x", padx=6, pady=(0, 2))
                self._twarn_outer.config(highlightbackground=fc)
                self._twarn_lbl.config(
                    text="WARNING — RETURN TO HOME IMMEDIATELY", fg=fc)
            elif remaining <= 120:
                self._remain_lbl.config(fg=_ORANGE)
                self._twarn_outer.pack(fill="x", padx=6, pady=(0, 2))
                self._twarn_outer.config(highlightbackground=_ORANGE)
                self._twarn_lbl.config(
                    text="CAUTION — PLAN YOUR RETURN TO HOME", fg=_ORANGE)
            else:
                self._remain_lbl.config(fg=_VALUE_FG)
                self._twarn_outer.pack_forget()
        else:
            self._remain_lbl.config(text="—:——", fg=_DIM_FG)
            self._twarn_outer.pack_forget()

        self.after(500, self._tick)

    # =========================================================================
    # Public update
    # =========================================================================

    def update_fc_status(self, data: dict):
        self._last_data = data

        # ── ARM + mode ────────────────────────────────────────────────────────
        armed = bool(data.get("armed", False))
        if armed:
            self._arm_cv.itemconfig(self._arm_dot, fill=_GREEN)
            self._arm_lbl.config(text="ARMED  ", fg=_GREEN)
            self._p1_frame.config(highlightbackground=_GREEN)
        else:
            self._arm_cv.itemconfig(self._arm_dot, fill=_RED)
            self._arm_lbl.config(text="DISARMED", fg=_RED)
            self._p1_frame.config(highlightbackground=_BORDER)

        mode = str(data.get("flight_mode_name", "—"))
        if "FAILSAFE" in mode or "RESCUE" in mode:
            mc = _RED
        elif "DISARMED" in mode:
            mc = _ORANGE
        elif armed:
            mc = _GREEN
        else:
            mc = _VALUE_FG
        self._mode_lbl.config(text=mode, fg=mc)

        # ── Battery P1 (both full and compact rows updated) ───────────────────
        volts  = float(data.get("battery_voltage", 0.0))
        amps   = float(data.get("battery_current", 0.0))
        mah    = float(data.get("battery_mah_drawn", 0.0))
        pct    = int(data.get("battery_percentage", -1))
        cells  = self._detect_cells(data)
        state  = str(data.get("battery_state", "INIT")).upper()
        cell_v = (volts / cells) if cells > 0 else 0.0
        vcol   = self._cell_color(cell_v) if volts > 0 else _DIM_FG

        volt_str = f"{volts:.1f}" if volts > 0 else "—"
        cell_str = f"{cell_v:.2f}  V" if cell_v > 0 else "—  V"
        cell_str_c = f"{cell_v:.2f} V/cell" if cell_v > 0 else "— V/cell"

        # Full layout labels
        self._big_volt.config(text=volt_str, fg=vcol)
        self._cell_lbl.config(text=cell_str, fg=vcol)
        # Compact layout labels
        self._big_volt_c.config(text=volt_str, fg=vcol)
        self._cell_lbl_c.config(text=cell_str_c, fg=vcol)

        pcol = _RED if 0 <= pct < 15 else _ORANGE if 0 <= pct < 30 else _VALUE_FG
        self._bat_pct_lbl.config(
            text=f"{pct}%" if pct >= 0 else "—%",
            fg=pcol if pct >= 0 else _DIM_FG)

        sfg, sbg = _BAT_STATE_COLORS.get(state, (_DIM_FG, _BORDER))
        self._bat_state_lbl.config(text=state, fg=sfg, bg=sbg)
        self._bat_state_lbl_c.config(text=state, fg=sfg, bg=sbg)
        self._bat_strip.config(
            highlightbackground=_RED   if state == "CRITICAL" else
                                _ORANGE if state == "WARNING"  else _BORDER)
        self.after_idle(self._redraw_vbat)

        # ── Battery P2 ────────────────────────────────────────────────────────
        self._bat_curr_lbl.config(
            text=f"{amps:.1f}",
            fg=_ORANGE if amps > 30 else _VALUE_FG)
        self._bat_mah_lbl.config(text=f"{mah:.0f}")
        self._bat_cells_lbl.config(text=str(cells) if cells > 0 else "—")

        # ── Sensors ───────────────────────────────────────────────────────────
        for sname, key in _SENSORS:
            present = bool(data.get(key, False))
            cv, dot, lbl, pill = self._sensor_refs[sname]
            if present:
                cv.itemconfig(dot, fill=_GREEN)
                cv.config(bg=_BG2)
                pill.config(bg=_BG2, highlightbackground=_GREEN)
                lbl.config(bg=_BG2, fg=_VALUE_FG)
            else:
                cv.itemconfig(dot, fill=_DIM_FG)
                cv.config(bg=_DIM)
                pill.config(bg=_DIM, highlightbackground=_BORDER)
                lbl.config(bg=_DIM, fg=_DIM_FG)

        # ── CPU / metrics ─────────────────────────────────────────────────────
        self.after_idle(self._redraw_cpu)
        self._lbl_cycle.config(text=f"{float(data.get('fc_cycle_ms', 0)):.2f}")
        i2c = int(data.get("i2c_error_count", 0))
        self._lbl_i2c.config(text=str(i2c), fg=_RED if i2c > 0 else _VALUE_FG)
        self._lbl_pid.config(text=str(int(data.get("pid_profile", 0)) + 1))

        # ── Motors ────────────────────────────────────────────────────────────
        for i, key in enumerate(_MOTOR_KEYS):
            us    = int(data.get(key, 0))
            valid = us >= 1000
            ratio = (us - 1000) / 1000.0 if valid else 0.0
            col   = _RED if ratio > 0.85 else _ORANGE if ratio > 0.6 else _GREEN

            if self._motor_mode == "bars":
                bo  = self._motor_outers[i]
                bi  = self._motor_bars[i]
                lbl = self._motor_lbls[i]
                h   = bo.winfo_height() or 36
                if valid:
                    bi.place(x=0, rely=1.0, relwidth=1.0,
                             height=max(1, int(h * ratio)), anchor="sw")
                    bi.config(bg=col)
                    lbl.config(text=str(us), fg=_VALUE_FG)
                else:
                    bi.place(x=0, rely=1.0, relwidth=1.0, height=1, anchor="sw")
                    bi.config(bg=_DIM_FG)
                    lbl.config(text="—", fg=_DIM_FG)
            else:
                cv, d, ll = self._motor_dots[i]
                cv.itemconfig(d, fill=col if valid else _DIM_FG)
                ll.config(fg=_VALUE_FG if valid else _DIM_FG)

        # ── RC channels ───────────────────────────────────────────────────────
        for i, (ch_name, key) in enumerate(_RC_CHANNELS):
            us    = int(data.get(key, 0))
            valid = us >= 1000
            ratio = (us - 1000) / 1000.0 if valid else 0.5

            if self._rc_mode == "bars":
                bo  = self._rc_outers[i]
                bi  = self._rc_bars[i]
                lbl = self._rc_lbls[i]
                h   = bo.winfo_height() or 36
                if valid:
                    if ch_name == "THRO":
                        bi.place(x=0, rely=1.0, relwidth=1.0,
                                 height=max(1, int(h * ratio)), anchor="sw")
                    else:
                        dev = ratio - 0.5
                        bi.place(x=0, rely=0.5, relwidth=1.0,
                                 height=max(1, int(abs(dev) * h)),
                                 anchor="sw" if dev >= 0 else "nw")
                    # ARM bar: green only at or above ARM_THRESHOLD (1800).
                    col = _GREEN if ch_name == "ARM" and us >= ARM_THRESHOLD else _BLUE
                    bi.config(bg=col)
                    lbl.config(text=str(us), fg=_VALUE_FG)
                else:
                    bi.place(x=0, rely=0.5, relwidth=1.0, height=1, anchor="w")
                    bi.config(bg=_DIM_FG)
                    lbl.config(text="—", fg=_DIM_FG)
            else:
                cv, d, ll = self._rc_dots[i]
                # Same ARM threshold in dot mode.
                col = _GREEN if ch_name == "ARM" and us >= ARM_THRESHOLD else _BLUE
                cv.itemconfig(d, fill=col if valid else _DIM_FG)
                ll.config(fg=_VALUE_FG if valid else _DIM_FG)

        # ── Link quality ──────────────────────────────────────────────────────
        lq = int(data.get("rc_link_quality", -1))
        if lq >= 0:
            self._lq_lbl.config(
                text=f"{lq}%",
                fg=_RED if lq < 30 else _ORANGE if lq < 70 else _GREEN)
        else:
            rc_count = int(data.get("rc_channel_count", 0))
            rc_roll  = int(data.get("rc_roll", 0))
            if rc_count > 0 and 800 <= rc_roll <= 2200:
                self._lq_lbl.config(text=f"{rc_count} CH  (CRSF)", fg=_GREEN)
            else:
                self._lq_lbl.config(text="—%", fg=_DIM_FG)