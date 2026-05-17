"""
FCStatusWidget.py  —  Flight Controller Status Panel
=====================================================
Aviation-standard FC status display. Fully responsive — redraws on resize.

Displays:
  • ARMED / DISARMED   — large colour-coded LED + text
  • Flight mode        — ACRO / ANGLE / HORIZON / GPS RESCUE / FAILSAFE
  • Sensor health      — ACC  BARO  MAG  GPS  RANGE  GYRO  (LED pills)
  • CPU load           — bar that scales with widget width + % value
  • FC loop time       — ms
  • I²C error count    — red when non-zero
  • PID profile        — 1-based index

Feed via  update_fc_status(data_dict)  every UI tick.
"""

import tkinter as tk

# ── Palette — matches cockpit dark theme ─────────────────────────────────────
_BG       = "#0f0f1a"
_BG2      = "#12121f"
_BORDER   = "#1e2a3a"
_LABEL_FG = "#4a6080"
_VALUE_FG = "#c8dff0"
_GREEN    = "#00ff88"
_ORANGE   = "#ffaa00"
_RED      = "#ff3355"
_DIM      = "#1a2a3a"
_DIM_FG   = "#2a4055"

_F_TINY  = ("Consolas",  7, "bold")
_F_SMALL = ("Consolas",  8, "bold")
_F_BODY  = ("Consolas",  9)
_F_VALUE = ("Consolas", 10, "bold")
_F_ARMED = ("Consolas", 13, "bold")
_F_MODE  = ("Consolas", 11, "bold")

_SENSORS = [
    ("ACC",   "sensor_acc_present"),
    ("BARO",  "sensor_baro_present"),
    ("MAG",   "sensor_mag_present"),
    ("GPS",   "sensor_gps_present"),
    ("RANGE", "sensor_rangefinder_present"),
    ("GYRO",  "sensor_gyro_present"),
]


class FCStatusWidget(tk.Frame):
    """Responsive Flight Controller status panel."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_BG, **kwargs)
        self._last_data: dict = {}
        self._build_ui()
        # Redraw CPU bar whenever the widget is resized
        self.bind("<Configure>", lambda e: self.after_idle(self._redraw_cpu))

    # =========================================================================
    # Build UI  (called once; no teardown needed — pack manager handles it)
    # =========================================================================

    def _build_ui(self):

        # ── Row 1: ARM status + flight mode ──────────────────────────────────
        row1 = tk.Frame(self, bg=_BG2,
                        highlightthickness=1, highlightbackground=_BORDER)
        row1.pack(fill="x", padx=6, pady=(6, 2))

        # ARM LED
        self._arm_canvas = tk.Canvas(row1, width=18, height=18,
                                     bg=_BG2, highlightthickness=0)
        self._arm_canvas.pack(side="left", padx=(10, 4), pady=7)
        self._arm_dot = self._arm_canvas.create_oval(2, 2, 16, 16,
                                                      fill=_RED, outline="")

        # ARM text
        self._arm_lbl = tk.Label(row1, text="DISARMED",
                                  bg=_BG2, fg=_RED, font=_F_ARMED,
                                  width=9, anchor="w")
        self._arm_lbl.pack(side="left", pady=7)

        # Divider
        tk.Frame(row1, bg=_BORDER, width=1).pack(
            side="left", fill="y", padx=(8, 10), pady=4)

        # MODE label
        tk.Label(row1, text="MODE", bg=_BG2, fg=_LABEL_FG,
                 font=_F_SMALL).pack(side="left")
        self._mode_lbl = tk.Label(row1, text="ACRO [DISARMED]",
                                   bg=_BG2, fg=_ORANGE, font=_F_MODE)
        self._mode_lbl.pack(side="left", padx=(6, 10), pady=7)

        # ── Row 2: Sensor health LEDs ─────────────────────────────────────────
        tk.Frame(self, bg=_BORDER, height=1).pack(fill="x", padx=6)

        row2 = tk.Frame(self, bg=_BG)
        row2.pack(fill="x", padx=6, pady=(4, 2))

        tk.Label(row2, text="SENSORS", bg=_BG, fg=_LABEL_FG,
                 font=_F_SMALL).pack(side="left", padx=(2, 8))

        self._sensor_refs: dict[str, tuple] = {}
        for sname, _ in _SENSORS:
            pill = tk.Frame(row2, bg=_DIM,
                            highlightthickness=1, highlightbackground=_BORDER)
            pill.pack(side="left", padx=2, pady=2)

            cv = tk.Canvas(pill, width=10, height=10,
                           bg=_DIM, highlightthickness=0)
            cv.pack(side="left", padx=(4, 2), pady=3)
            dot = cv.create_oval(1, 1, 9, 9, fill=_DIM_FG, outline="")

            lbl = tk.Label(pill, text=sname, bg=_DIM, fg=_DIM_FG,
                           font=_F_TINY)
            lbl.pack(side="left", padx=(0, 5), pady=3)

            self._sensor_refs[sname] = (cv, dot, lbl, pill)

        # ── Row 3: Metrics (CPU bar + numerics) ───────────────────────────────
        tk.Frame(self, bg=_BORDER, height=1).pack(fill="x", padx=6)

        row3 = tk.Frame(self, bg=_BG)
        row3.pack(fill="x", padx=6, pady=(4, 6))

        # CPU bar block
        cpu_blk = tk.Frame(row3, bg=_BG)
        cpu_blk.pack(side="left", padx=(2, 16), fill="y")

        tk.Label(cpu_blk, text="CPU LOAD", bg=_BG, fg=_LABEL_FG,
                 font=_F_SMALL).pack(anchor="w")

        # bar_outer expands horizontally as panel grows
        self._bar_outer = tk.Frame(cpu_blk, bg=_BORDER, height=8)
        self._bar_outer.pack(fill="x", anchor="w")
        self._bar_outer.pack_propagate(False)
        self._bar_inner = tk.Frame(self._bar_outer, bg=_GREEN, height=8)
        self._bar_inner.place(x=0, y=0, relheight=1.0, width=0)

        self._cpu_pct_lbl = tk.Label(cpu_blk, text="0 %", bg=_BG,
                                      fg=_GREEN, font=_F_VALUE)
        self._cpu_pct_lbl.pack(anchor="w")

        # Numeric metrics
        def _num(label, unit=""):
            blk = tk.Frame(row3, bg=_BG)
            blk.pack(side="left", padx=(0, 16), fill="y")
            tk.Label(blk, text=label, bg=_BG, fg=_LABEL_FG,
                     font=_F_SMALL).pack(anchor="w")
            vrow = tk.Frame(blk, bg=_BG)
            vrow.pack(anchor="w")
            v = tk.Label(vrow, text="—", bg=_BG,
                         fg=_VALUE_FG, font=_F_VALUE)
            v.pack(side="left")
            if unit:
                tk.Label(vrow, text=unit, bg=_BG, fg=_LABEL_FG,
                         font=_F_BODY).pack(side="left", padx=(2, 0))
            return v

        self._lbl_cycle = _num("FC LOOP", "ms")
        self._lbl_i2c   = _num("I²C ERR")
        self._lbl_pid   = _num("PID PROF")

    # =========================================================================
    # CPU bar redraw — called on resize and on every data update
    # =========================================================================

    def _redraw_cpu(self):
        cpu = max(0, min(100, int(self._last_data.get("cpu_load_percent", 0))))
        w = self._bar_outer.winfo_width()
        if w < 4:
            return
        fill_w = max(1, int(w * cpu / 100))
        color = _RED if cpu > 80 else _ORANGE if cpu > 60 else _GREEN
        self._bar_inner.place(x=0, y=0, width=fill_w, relheight=1.0)
        self._bar_inner.config(bg=color)
        self._cpu_pct_lbl.config(text=f"{cpu} %", fg=color)

    # =========================================================================
    # Public update — called every 20 ms from main._update_loop
    # =========================================================================

    def update_fc_status(self, data: dict):
        self._last_data = data

        # ── ARM state ─────────────────────────────────────────────────────────
        armed = bool(data.get("armed", False))
        if armed:
            self._arm_canvas.itemconfig(self._arm_dot, fill=_GREEN)
            self._arm_lbl.config(text="ARMED  ", fg=_GREEN)
        else:
            self._arm_canvas.itemconfig(self._arm_dot, fill=_RED)
            self._arm_lbl.config(text="DISARMED", fg=_RED)

        # ── Flight mode ───────────────────────────────────────────────────────
        mode = str(data.get("flight_mode_name", "ACRO [DISARMED]"))
        if "FAILSAFE" in mode or "RESCUE" in mode:
            mc = _RED
        elif "DISARMED" in mode:
            mc = _ORANGE
        elif armed:
            mc = _GREEN
        else:
            mc = _VALUE_FG
        self._mode_lbl.config(text=mode, fg=mc)

        # ── Sensor LEDs ───────────────────────────────────────────────────────
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

        # ── CPU bar + metrics ─────────────────────────────────────────────────
        self._redraw_cpu()

        cycle = float(data.get("fc_cycle_ms", 0.0))
        self._lbl_cycle.config(text=f"{cycle:.2f}")

        i2c = int(data.get("i2c_error_count", 0))
        self._lbl_i2c.config(text=str(i2c),
                              fg=_RED if i2c > 0 else _VALUE_FG)

        pid = int(data.get("pid_profile", 0)) + 1
        self._lbl_pid.config(text=str(pid))