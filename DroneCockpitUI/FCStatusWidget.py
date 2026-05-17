"""
FCStatusWidget.py  —  Flight Controller Status Panel
=====================================================
Priority-based responsive layout — critical info always visible.

PRIORITY TIERS:
  P1 (always visible): ARM state, flight mode, battery voltage/cell V/state,
                        voltage bar, flight timer + warnings
  P2 (always visible): secondary battery stats, sensors
  P3 (collapses on narrow): CPU/metrics, motors, RC channels

Responsive width behaviour:
  ≥ 300 px  →  motors / RC as vertical bars with µs values
  < 300 px  →  motors / RC as coloured LED dots only (no bars)

Flight timer (always visible):
  • Counts up from arm event (elapsed flight time).
  • Pilot sets planned duration via the spinbox (default 10 min).
  • Remaining < 2 min  →  orange CAUTION banner.
  • Remaining < 30 s   →  flashing red WARNING banner.
  • Time expired       →  steady red ENDURANCE EXCEEDED.
  set_planned_duration(minutes)  also callable programmatically.

Feed via  update_fc_status(data_dict)  every UI tick (~20 ms).

Expected data keys:
  armed                   bool
  flight_mode_name        str
  battery_voltage         float  V
  battery_current         float  A
  battery_mah_drawn       float  mAh
  battery_cell_count      int    0 = auto-detect
  battery_percentage      int    0-100 or -1
  battery_state           str    "OK"|"WARNING"|"CRITICAL"|"NOT_PRESENT"|"INIT"
  cpu_load_percent        int    0-100
  fc_cycle_ms             float  ms
  i2c_error_count         int
  pid_profile             int    0-based
  sensor_{acc,baro,mag,gps,rangefinder,gyro}_present  bool
  motor_1_us … motor_4_us int    1000-2000
  rc_roll/pitch/throttle/yaw/arm  int  1000-2000
  rc_link_quality         int    0-100  (-1 = RSSI N/A)
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
_F_TIMER  = ("Consolas", 13, "bold")
_F_WARN   = ("Consolas", 10, "bold")

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

# FIX: complete battery-state → (fg, bg) colour map.
# Previously only OK/WARNING/CRITICAL/UNKNOWN were listed.
# "INIT" (BF default before detection) and "NOT_PRESENT" both fell through
# to the dict's default, rendering as dim UNKNOWN text even when the key
# was correctly set.  Now all five BatteryState strings are mapped.
_BAT_STATE_COLORS = {
    "OK":          (_GREEN,   _BG3),
    "WARNING":     (_ORANGE,  _BG3),
    "CRITICAL":    (_RED,     _BG3),
    "NOT_PRESENT": (_DIM_FG,  _BORDER),
    "INIT":        (_DIM_FG,  _BORDER),
    "UNKNOWN":     (_DIM_FG,  _BORDER),   # fallback label
}


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
        self._planned_sec: int = 600        # default 10 min
        self._arm_start: float = 0.0
        self._was_armed: bool  = False
        self._flash_on: bool   = False
        self._width: int       = 500
        self._motor_mode       = "bars"
        self._rc_mode          = "bars"
        self._build_ui()
        self.bind("<Configure>", self._on_resize)
        self._tick()

    # =========================================================================
    # Public
    # =========================================================================

    def set_planned_duration(self, minutes: float):
        """Set planned flight duration in minutes (0 = no limit)."""
        self._planned_sec = int(minutes * 60)
        try:
            self._plan_spin.delete(0, "end")
            self._plan_spin.insert(0, str(int(minutes)))
        except Exception:
            pass

    # =========================================================================
    # Build
    # =========================================================================

    def _build_ui(self):
        # ── P1a: ARM state + flight mode  (always visible) ────────────────────
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

        mode_blk = tk.Frame(p1, bg=_BG2)
        mode_blk.pack(side="left", pady=6)
        tk.Label(mode_blk, text="MODE", bg=_BG2, fg=_LABEL_FG,
                 font=_F_SMALL).pack(anchor="w")
        self._mode_lbl = tk.Label(mode_blk, text="—", bg=_BG2,
                                   fg=_ORANGE, font=_F_MODE)
        self._mode_lbl.pack(anchor="w")

        # ── P1b: Battery critical strip  (always visible) ─────────────────────
        bs = tk.Frame(self, bg=_BG3,
                      highlightthickness=1, highlightbackground=_BORDER)
        bs.pack(fill="x", padx=6, pady=(0, 2))
        self._bat_strip = bs

        vblk = tk.Frame(bs, bg=_BG3)
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

        cblk = tk.Frame(bs, bg=_BG3)
        cblk.pack(side="left", padx=(12, 0), pady=6)
        tk.Label(cblk, text="PER CELL", bg=_BG3, fg=_LABEL_FG,
                 font=_F_TINY).pack(anchor="w")
        self._cell_lbl = tk.Label(cblk, text="—  V", bg=_BG3,
                                   fg=_GREEN, font=_F_VALUE)
        self._cell_lbl.pack(anchor="w")

        rblk = tk.Frame(bs, bg=_BG3)
        rblk.pack(side="right", padx=(0, 10), pady=6)
        self._bat_state_lbl = tk.Label(rblk, text="INIT",
                                        bg=_BORDER, fg=_DIM_FG,
                                        font=_F_SMALL, padx=6, pady=2)
        self._bat_state_lbl.pack(anchor="e")
        self._bat_pct_lbl = tk.Label(rblk, text="—%", bg=_BG3,
                                      fg=_VALUE_FG, font=_F_VALUE)
        self._bat_pct_lbl.pack(anchor="e", pady=(2, 0))

        # Voltage bar
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

        # ── P1c: Flight timer  (always visible) ───────────────────────────────
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

        # Timer warning banner (hidden until needed)
        self._twarn_outer = tk.Frame(self, bg=_BG,
                                      highlightthickness=1,
                                      highlightbackground=_BG)
        self._twarn_lbl = tk.Label(self._twarn_outer, text="",
                                    bg=_BG, fg=_ORANGE, font=_F_WARN)
        self._twarn_lbl.pack(pady=3)

        # ── P2: Battery detail + sensors  (always visible) ────────────────────
        _sep(self)
        _sec_hdr(self, "BATTERY DETAIL")
        bat_det = tk.Frame(self, bg=_BG)
        bat_det.pack(fill="x", padx=6, pady=(0, 3))
        self._bat_curr_lbl  = self._mini_stat(bat_det, "CURRENT", "A")
        self._bat_mah_lbl   = self._mini_stat(bat_det, "USED",   "mAh")
        self._bat_cells_lbl = self._mini_stat(bat_det, "CELLS",    "S")

        _sep(self)
        _sec_hdr(self, "SENSORS")
        sens_row = tk.Frame(self, bg=_BG)
        sens_row.pack(fill="x", padx=6, pady=(0, 3))
        self._sensor_refs: dict[str, tuple] = {}
        for sname, _ in _SENSORS:
            pill = tk.Frame(sens_row, bg=_DIM,
                            highlightthickness=1, highlightbackground=_BORDER)
            pill.pack(side="left", padx=2, pady=1)
            cv = tk.Canvas(pill, width=9, height=9,
                           bg=_DIM, highlightthickness=0)
            cv.pack(side="left", padx=(3, 1), pady=2)
            dot = cv.create_oval(1, 1, 8, 8, fill=_DIM_FG, outline="")
            lbl = tk.Label(pill, text=sname, bg=_DIM, fg=_DIM_FG,
                           font=_F_TINY)
            lbl.pack(side="left", padx=(0, 4), pady=2)
            self._sensor_refs[sname] = (cv, dot, lbl, pill)

        # ── P3: FC metrics ────────────────────────────────────────────────────
        _sep(self)
        _sec_hdr(self, "FC METRICS")
        met_row = tk.Frame(self, bg=_BG)
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
        self._lbl_i2c   = self._mini_stat(met_row, "I²C ERR")
        self._lbl_pid   = self._mini_stat(met_row, "PID")

        # ── P3: Motors ────────────────────────────────────────────────────────
        _sep(self)
        mot_hdr = tk.Frame(self, bg=_BG)
        mot_hdr.pack(fill="x", padx=6, pady=(3, 1))
        tk.Label(mot_hdr, text="MOTORS", bg=_BG, fg=_LABEL_FG,
                 font=_F_TINY).pack(side="left")
        self._mot_unit_lbl = tk.Label(mot_hdr, text="(µs)", bg=_BG,
                                       fg=_LABEL_FG, font=_F_MICRO)
        self._mot_unit_lbl.pack(side="left", padx=(3, 0))
        tk.Frame(mot_hdr, bg=_BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(4, 0), pady=4)

        self._mot_wrapper = tk.Frame(self, bg=_BG)
        self._mot_wrapper.pack(fill="x", padx=6, pady=(0, 3))

        self._mot_bar_f = tk.Frame(self._mot_wrapper, bg=_BG)
        self._mot_bar_f.pack(fill="x")

        self._motor_bars:   list[tk.Frame] = []
        self._motor_lbls:   list[tk.Label] = []
        self._motor_outers: list[tk.Frame] = []
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

        self._mot_dot_f = tk.Frame(self._mot_wrapper, bg=_BG)

        self._motor_dots: list[tuple] = []
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

        # ── P3: RC channels ───────────────────────────────────────────────────
        _sep(self)
        rc_hdr = tk.Frame(self, bg=_BG)
        rc_hdr.pack(fill="x", padx=6, pady=(3, 1))
        tk.Label(rc_hdr, text="RC CHANNELS", bg=_BG, fg=_LABEL_FG,
                 font=_F_TINY).pack(side="left")
        tk.Frame(rc_hdr, bg=_BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(4, 0), pady=4)

        self._rc_wrapper = tk.Frame(self, bg=_BG)
        self._rc_wrapper.pack(fill="x", padx=6, pady=(0, 3))

        self._rc_bar_f = tk.Frame(self._rc_wrapper, bg=_BG)
        self._rc_bar_f.pack(fill="x")

        self._rc_bars:   list[tk.Frame] = []
        self._rc_lbls:   list[tk.Label] = []
        self._rc_outers: list[tk.Frame] = []
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

        self._rc_dot_f = tk.Frame(self._rc_wrapper, bg=_BG)

        self._rc_dots: list[tuple] = []
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

        lq_f = tk.Frame(self, bg=_BG)
        lq_f.pack(fill="x", padx=8, pady=(0, 5))
        tk.Label(lq_f, text="LINK QUALITY", bg=_BG, fg=_LABEL_FG,
                 font=_F_MICRO).pack(side="left")
        self._lq_lbl = tk.Label(lq_f, text="—%", bg=_BG,
                                  fg=_VALUE_FG, font=_F_TINY)
        self._lq_lbl.pack(side="left", padx=5)

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
        return v

    # =========================================================================
    # Resize handler
    # =========================================================================

    def _on_resize(self, event):
        w = event.width
        if w == self._width:
            return
        self._width = w

        mode = "bars" if w >= 300 else "dots"
        if mode != self._motor_mode:
            self._motor_mode = mode
            if mode == "bars":
                self._mot_dot_f.pack_forget()
                self._mot_bar_f.pack(fill="x")
                self._mot_unit_lbl.config(text="(µs)")
            else:
                self._mot_bar_f.pack_forget()
                self._mot_dot_f.pack(fill="x")
                self._mot_unit_lbl.config(text="")

        if mode != self._rc_mode:
            self._rc_mode = mode
            if mode == "bars":
                self._rc_dot_f.pack_forget()
                self._rc_bar_f.pack(fill="x")
            else:
                self._rc_bar_f.pack_forget()
                self._rc_dot_f.pack(fill="x")

        self.after_idle(self._redraw_cpu)
        self.after_idle(self._redraw_vbat)

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
    # Flight timer  (runs every 500 ms independently)
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
                    text="⚠  ENDURANCE EXCEEDED — LAND IMMEDIATELY", fg=_RED)
            elif remaining <= 30:
                self._flash_on = not self._flash_on
                fc = _RED if self._flash_on else _ORANGE
                self._remain_lbl.config(fg=fc)
                self._twarn_outer.pack(fill="x", padx=6, pady=(0, 2))
                self._twarn_outer.config(highlightbackground=fc)
                self._twarn_lbl.config(
                    text="⚠  WARNING — RETURN TO HOME IMMEDIATELY", fg=fc)
            elif remaining <= 120:
                self._remain_lbl.config(fg=_ORANGE)
                self._twarn_outer.pack(fill="x", padx=6, pady=(0, 2))
                self._twarn_outer.config(highlightbackground=_ORANGE)
                self._twarn_lbl.config(
                    text="◉  CAUTION — PLAN YOUR RETURN TO HOME", fg=_ORANGE)
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

        # ── Battery P1 ────────────────────────────────────────────────────────
        volts  = float(data.get("battery_voltage", 0.0))
        amps   = float(data.get("battery_current", 0.0))
        mah    = float(data.get("battery_mah_drawn", 0.0))
        pct    = int(data.get("battery_percentage", -1))
        cells  = self._detect_cells(data)
        # FIX: normalise state string; map all BatteryState enum values.
        # to_dict() emits "OK"/"WARNING"/"CRITICAL"/"NOT_PRESENT"/"INIT".
        # Previously only OK/WARNING/CRITICAL/UNKNOWN were in the colour map
        # so INIT (the BF power-on default) rendered as the fallback "UNKNOWN"
        # text with no useful colour context.
        state  = str(data.get("battery_state", "INIT")).upper()
        cell_v = (volts / cells) if cells > 0 else 0.0
        vcol   = self._cell_color(cell_v) if volts > 0 else _DIM_FG

        self._big_volt.config(text=f"{volts:.1f}" if volts > 0 else "—", fg=vcol)
        self._cell_lbl.config(
            text=f"{cell_v:.2f}  V" if cell_v > 0 else "—  V", fg=vcol)

        pcol = _RED if 0 <= pct < 15 else _ORANGE if 0 <= pct < 30 else _VALUE_FG
        self._bat_pct_lbl.config(
            text=f"{pct}%" if pct >= 0 else "—%",
            fg=pcol if pct >= 0 else _DIM_FG)

        sfg, sbg = _BAT_STATE_COLORS.get(state, (_DIM_FG, _BORDER))
        self._bat_state_lbl.config(text=state, fg=sfg, bg=sbg)
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
                    col = _GREEN if ch_name == "ARM" and us > 1700 else _BLUE
                    bi.config(bg=col)
                    lbl.config(text=str(us), fg=_VALUE_FG)
                else:
                    bi.place(x=0, rely=0.5, relwidth=1.0, height=1, anchor="w")
                    bi.config(bg=_DIM_FG)
                    lbl.config(text="—", fg=_DIM_FG)
            else:
                cv, d, ll = self._rc_dots[i]
                col = _GREEN if ch_name == "ARM" and us > 1700 else _BLUE
                cv.itemconfig(d, fill=col if valid else _DIM_FG)
                ll.config(fg=_VALUE_FG if valid else _DIM_FG)

        # ── Link quality ──────────────────────────────────────────────────────
        lq = int(data.get("rc_link_quality", -1))
        if lq >= 0:
            self._lq_lbl.config(
                text=f"{lq}%",
                fg=_RED if lq < 30 else _ORANGE if lq < 70 else _GREEN)
        else:
            # ELRS/CRSF path: show channel count instead of percentage
            rc_count = int(data.get("rc_channel_count", 0))
            rc_roll  = int(data.get("rc_roll", 0))
            if rc_count > 0 and 800 <= rc_roll <= 2200:
                self._lq_lbl.config(text=f"{rc_count} CH  (CRSF)", fg=_GREEN)
            else:
                self._lq_lbl.config(text="—%", fg=_DIM_FG)