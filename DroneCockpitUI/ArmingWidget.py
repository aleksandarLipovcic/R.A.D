"""
ArmingWidget.py  —  Pre-Flight Checklist & Arming Diagnostics
==============================================================
Two-tier aviation-style checklist.

SECTION A — AUTOMATIC CHECKS  (telemetry-driven, updates every tick)
  • FC arming disable flags  • Gyro  • Accelerometer
  • Battery voltage           • RC link  • GPS fix  • Motors idle

SECTION B — PILOT SIGN-OFF  (manual checkboxes, pilot ticks before each flight)
  Each item is a clickable row; ticked rows turn green.
  RESET button clears all for the next flight.

BANNER  (bottom, always visible)
  Grey  "WAITING FOR DATA…"  — no telemetry yet
  Red   "BLOCKED — FIX AUTO CHECKS"  — one or more auto checks failing
  Orange "COMPLETE SIGN-OFF ITEMS"   — auto OK but manual items remain
  Green  "✔  READY TO ARM"           — everything passed

Counter in header shows items remaining at a glance.

Public API:
  update_arming(data_dict)   — call every UI tick with telemetry
  reset_checklist()          — clear all manual checkboxes

Telemetry keys used:
  arming_disable_flags  int   bitfield
  battery_voltage       float V
  battery_cell_count    int   0 = auto-detect
  rc_link_quality       int   0-100  (-1 = RSSI not available)
  rc_channel_count      int   number of active RC channels
  rc_roll               int   1000-2000 µs  (used as ELRS/CRSF presence probe)
  gps_fix_type          int   0=no fix, 1=2D, 2=3D   ← replaces gps_fix
  gps_num_sats          int   satellites in solution  ← was gps_num_sat
  gps_hdop              float horizontal DOP
  sensor_gyro_present   bool
  sensor_acc_present    bool
  motor_1_us … motor_4_us  int  1000-2000 µs
"""

import tkinter as tk

# ── Palette ───────────────────────────────────────────────────────────────────
_BG       = "#0f0f1a"
_BG2      = "#12121f"
_BG3      = "#0a0a14"
_BORDER   = "#1e2a3a"
_LABEL_FG = "#4a6080"
_VALUE_FG = "#c8dff0"
_GREEN    = "#00ff88"
_ORANGE   = "#ffaa00"
_RED      = "#ff3355"
_DIM      = "#1a2a3a"
_DIM_FG   = "#263a4a"
_TICK_OK  = "#00cc66"

_F_SECTION = ("Consolas",  8, "bold")
_F_ITEM    = ("Consolas",  9)
_F_ITEM_B  = ("Consolas",  9, "bold")
_F_DETAIL  = ("Consolas",  8)
_F_BANNER  = ("Consolas", 14, "bold")
_F_COUNTER = ("Consolas", 10, "bold")
_F_RESET   = ("Consolas",  8, "bold")

# ── Arming-flag bits ──────────────────────────────────────────────────────────
_ARMING_BITS = [
    (1 << 0,  "NO GYRO"),
    (1 << 1,  "FAILSAFE"),
    (1 << 2,  "RX FAILSAFE"),
    (1 << 3,  "BAD RX"),
    (1 << 4,  "BOX FAILSAFE"),
    (1 << 5,  "RUNAWAY TKOFF"),
    (1 << 6,  "CRASH DETECT"),
    (1 << 7,  "THROTTLE HIGH"),
    (1 << 8,  "NOT LEVEL"),
    (1 << 9,  "BOOT GRACE"),
    (1 << 10, "NO PREARM"),
    (1 << 11, "CPU OVERLOAD"),
    (1 << 12, "CALIBRATING"),
    (1 << 13, "CLI ACTIVE"),
    (1 << 14, "OSD MENU"),
    (1 << 15, "BST"),
    (1 << 16, "MSP OVERRIDE"),
    (1 << 17, "PARALYZE"),
    (1 << 18, "GPS NOT READY"),
    (1 << 21, "RESCUE SW"),
    (1 << 23, "DSHOT BITBANG"),
    (1 << 24, "ACC CAL NEEDED"),
    (1 << 25, "MOTOR PROTOCOL"),
    (1 << 26, "ARM SW OFF"),
]

# ── Auto-check evaluators ─────────────────────────────────────────────────────

def _check_arm_flags(data):
    flags = int(data.get("arming_disable_flags", 0))
    if flags == 0:
        return True, "ALL FLAGS CLEAR"
    active = [name for bit, name in _ARMING_BITS if flags & bit]
    detail = ", ".join(active[:3]) + ("…" if len(active) > 3 else "")
    return False, detail

def _check_gyro(data):
    ok = bool(data.get("sensor_gyro_present", False))
    return ok, "ONLINE" if ok else "NOT DETECTED"

def _check_acc(data):
    ok = bool(data.get("sensor_acc_present", False))
    return ok, "ONLINE" if ok else "NOT DETECTED"

def _check_battery(data):
    v = float(data.get("battery_voltage", 0.0))
    if v < 1.0:
        return False, "NO VOLTAGE READING"
    cells = int(data.get("battery_cell_count", 0))
    if cells == 0:
        for n in range(1, 9):
            if v <= n * 4.25 + 0.1:
                cells = n; break
    if cells == 0:
        return False, f"{v:.2f}V — CELL COUNT UNKNOWN"
    cv = v / cells
    if cv < 3.40:
        return False, f"{v:.2f}V  {cv:.2f}V/cell — CRITICAL"
    if cv < 3.65:
        return True,  f"{v:.2f}V  {cv:.2f}V/cell — LOW"
    return True,      f"{v:.2f}V  {cv:.2f}V/cell"

# FIX: RC link — ELRS/CRSF radios do NOT populate MSP_ANALOG's rssi byte,
# so rc_link_quality will be -1 even with a healthy link.  When rssi is
# unavailable we probe RC channel presence as a reliable fallback:
# if rc_channel_count > 0 and any stick channel is in the valid µs range
# (800-2200) the receiver is clearly active.
def _check_rc_link(data):
    lq = int(data.get("rc_link_quality", -1))

    if lq >= 0:
        # RSSI is available (PWM/SBUS analogue path)
        if lq < 30:
            return False, f"WEAK  {lq}%"
        return True, f"QUALITY  {lq}%"

    # RSSI not available — fall back to RC channel presence check.
    # rc_roll (channel 0) is always transmitted and will be in 800-2200 µs
    # if the receiver is active, regardless of radio protocol.
    rc_count = int(data.get("rc_channel_count", 0))
    rc_roll  = int(data.get("rc_roll", 0))
    if rc_count > 0 and 800 <= rc_roll <= 2200:
        return True, f"ACTIVE — {rc_count} CH  (RSSI N/A / CRSF)"
    return False, "NO SIGNAL"

# FIX: GPS check — previously used gps_fix (= GPSReading.positionUsable) which
# requires HDOP < 5.0 in addition to 3D fix and ≥4 satellites.  A perfectly
# usable GPS lock with HDOP just above 5.0 (e.g. 5.06) was incorrectly
# reported as NO FIX in the checklist while the GPS widget showed 3D FIX.
#
# Now uses gps_fix_type (raw fix quality from MSP_RAW_GPS) and gps_num_sats
# directly, matching what the GPS widget displays.
def _check_gps(data):
    fix_type = int(data.get("gps_fix_type", 0))
    # gps_num_sats and gps_num_sat are both exported by to_dict(); accept either
    sats = int(data.get("gps_num_sats", 0) or data.get("gps_num_sat", 0))
    hdop = float(data.get("gps_hdop", 99.0))

    if fix_type < 2:
        return False, f"NO 3D FIX  ({sats} sats)"
    if sats < 6:
        return False, f"3D FIX  {sats} SATS — NEED ≥6"
    hdop_str = f"  HDOP {hdop:.2f}" if hdop < 90 else ""
    return True, f"3D FIX  {sats} SATS{hdop_str}"

def _check_motors(data):
    # BF MSP_MOTOR returns 0 for all motors when disarmed — this is correct
    # behaviour, not a missing-data condition. We distinguish:
    #   • All zero AND armed      → real problem (motors should report idle ~1000 µs)
    #   • All zero AND disarmed   → normal; pass as "DISARMED / IDLE"
    #   • Non-zero values present → check they are in idle range (900–1100 µs)
    vals = [int(data.get(f"motor_{i}_us", 0)) for i in range(1, 5)]
    armed = bool(data.get("armed", False))
    if all(v == 0 for v in vals):
        if armed:
            return False, "NO MOTOR DATA (ARMED)"
        return True, "DISARMED — IDLE"
    bad = [i+1 for i, v in enumerate(vals) if v != 0 and not (900 <= v <= 1100)]
    if bad:
        return False, f"M{bad} OUT OF IDLE RANGE"
    return True, "ALL IDLE OK"

# (id, display label, evaluator)
_AUTO_CHECKS = [
    ("arm_flags", "FC ARMING FLAGS",    _check_arm_flags),
    ("gyro",      "GYRO",               _check_gyro),
    ("acc",       "ACCELEROMETER",      _check_acc),
    ("battery",   "BATTERY VOLTAGE",    _check_battery),
    ("rc_link",   "RC LINK",            _check_rc_link),
    ("gps",       "GPS FIX",            _check_gps),
    ("motors",    "MOTORS IDLE",        _check_motors),
]

# ── Manual sign-off items ─────────────────────────────────────────────────────
_MANUAL_ITEMS = [
    ("props",     "PROPELLERS INSTALLED & SECURE"),
    ("area",      "FLIGHT AREA CLEAR — NO PEOPLE / OBSTACLES"),
    ("battery_p", "BATTERY PHYSICALLY SECURED IN FRAME"),
    ("vtx",       "VTX FREQUENCY CONFIRMED / AUTHORISED"),
    ("failsafe",  "FAILSAFE TESTED & VERIFIED"),
    ("home",      "HOME POINT SET (if GPS mission)"),
    ("range",     "RC RANGE CHECK PASSED"),
    ("visual",    "AIRCRAFT VISUAL INSPECTION COMPLETE"),
]


class ArmingWidget(tk.Frame):

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_BG, **kwargs)
        self._auto_state:   dict[str, bool]          = {}
        self._auto_rows:    dict[str, tuple]          = {}
        self._manual_vars:  dict[str, tk.BooleanVar]  = {}
        self._manual_lbls:  dict[str, tk.Label]       = {}
        self._cb_widgets:   dict[str, tuple]          = {}
        self._has_data = False
        self._n_auto_cols = 0
        self._build_ui()
        self.bind("<Configure>", self._on_resize)

    # =========================================================================
    # Build
    # =========================================================================

    def _build_ui(self):
        # ── Header ────────────────────────────────────────────────────────────
        hdr = tk.Frame(self, bg=_BG2,
                       highlightthickness=1, highlightbackground=_BORDER)
        hdr.pack(fill="x", padx=6, pady=(6, 3))
        tk.Label(hdr, text="PRE-FLIGHT CHECKLIST",
                 bg=_BG2, fg=_LABEL_FG, font=_F_SECTION).pack(
            side="left", padx=10, pady=5)
        self._count_lbl = tk.Label(hdr, text="WAITING FOR DATA",
                                    bg=_BG2, fg=_ORANGE, font=_F_COUNTER)
        self._count_lbl.pack(side="right", padx=8, pady=5)

        # ── Section A: Automatic checks ───────────────────────────────────────
        self._build_section_hdr("A  AUTOMATIC CHECKS  (telemetry)")

        auto_outer = tk.Frame(self, bg=_BG2,
                              highlightthickness=1, highlightbackground=_BORDER)
        auto_outer.pack(fill="x", padx=6, pady=(0, 4))

        self._auto_inner = tk.Frame(auto_outer, bg=_BG2)
        self._auto_inner.pack(fill="x", padx=4, pady=4)

        self._auto_col_frames: list[tk.Frame] = []
        self._layout_auto_cols(1)

        # ── Section B: Pilot sign-off ─────────────────────────────────────────
        self._build_section_hdr("B  PILOT SIGN-OFF  (manual)")

        signoff = tk.Frame(self, bg=_BG3,
                           highlightthickness=1, highlightbackground=_BORDER)
        signoff.pack(fill="x", padx=6, pady=(0, 4))

        for item_id, item_text in _MANUAL_ITEMS:
            var = tk.BooleanVar(value=False)
            self._manual_vars[item_id] = var

            row = tk.Frame(signoff, bg=_BG3)
            row.pack(fill="x", padx=6, pady=1)

            cv = tk.Canvas(row, width=16, height=16,
                           bg=_BG3, highlightthickness=0, cursor="hand2")
            cv.pack(side="left", padx=(2, 4))
            box  = cv.create_rectangle(1, 1, 15, 15, outline=_LABEL_FG,
                                        fill=_BG3, width=1)
            tick = cv.create_text(8, 8, text="", fill=_GREEN,
                                   font=_F_ITEM_B)

            lbl = tk.Label(row, text=item_text, bg=_BG3, fg=_LABEL_FG,
                           font=_F_ITEM, anchor="w", cursor="hand2")
            lbl.pack(side="left", fill="x")
            self._manual_lbls[item_id] = lbl
            self._cb_widgets[item_id] = (var, cv, box, tick, lbl)

            def _toggle(e, v=var, cv=cv, box=box, tick=tick,
                        lbl=lbl, iid=item_id):
                v.set(not v.get())
                self._update_checkbox(v, cv, box, tick, lbl)
                self._refresh_banner()

            cv.bind("<Button-1>", _toggle)
            lbl.bind("<Button-1>", _toggle)

        # Reset button
        rst_row = tk.Frame(signoff, bg=_BG3)
        rst_row.pack(fill="x", padx=6, pady=(4, 6))
        tk.Button(
            rst_row, text="↺  RESET SIGN-OFF",
            bg=_BORDER, fg=_LABEL_FG, font=_F_RESET,
            activebackground=_DIM, activeforeground=_VALUE_FG,
            relief="flat", padx=8, pady=3,
            command=self.reset_checklist,
        ).pack(side="right")

        # ── Banner ────────────────────────────────────────────────────────────
        self._banner_frame = tk.Frame(self, bg=_BG,
                                       highlightthickness=1,
                                       highlightbackground=_BORDER)
        self._banner_frame.pack(fill="x", padx=6, pady=(2, 6))
        self._banner_lbl = tk.Label(
            self._banner_frame, text="WAITING FOR DATA…",
            bg=_BG, fg=_ORANGE, font=_F_BANNER)
        self._banner_lbl.pack(pady=8)

    def _build_section_hdr(self, text):
        f = tk.Frame(self, bg=_BG)
        f.pack(fill="x", padx=6, pady=(4, 1))
        tk.Label(f, text=text, bg=_BG, fg=_LABEL_FG,
                 font=_F_SECTION).pack(side="left")
        tk.Frame(f, bg=_BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(6, 0), pady=4)

    # =========================================================================
    # Custom checkbox draw helper
    # =========================================================================

    @staticmethod
    def _update_checkbox(var, cv, box, tick, lbl):
        if var.get():
            cv.itemconfig(box,  outline=_TICK_OK, fill=_BG3)
            cv.itemconfig(tick, text="✔", fill=_TICK_OK)
            lbl.config(fg=_TICK_OK)
        else:
            cv.itemconfig(box,  outline=_LABEL_FG, fill=_BG3)
            cv.itemconfig(tick, text="")
            lbl.config(fg=_LABEL_FG)

    # =========================================================================
    # Responsive auto-check column layout
    # =========================================================================

    def _on_resize(self, event):
        w = event.width
        n = 2 if w >= 420 else 1
        if n != self._n_auto_cols:
            self._layout_auto_cols(n)

    def _layout_auto_cols(self, n: int):
        self._n_auto_cols = n

        for cf in self._auto_col_frames:
            cf.destroy()
        self._auto_col_frames = []
        self._auto_rows = {}

        total         = len(_AUTO_CHECKS)
        rows_per_col  = (total + n - 1) // n

        for i in range(n):
            cf = tk.Frame(self._auto_inner, bg=_BG2)
            cf.grid(row=0, column=i, sticky="nw", padx=(0, 12))
            self._auto_inner.columnconfigure(i, weight=1)
            self._auto_col_frames.append(cf)

        for idx, (chk_id, label, _fn) in enumerate(_AUTO_CHECKS):
            col_i = min(idx // rows_per_col, n - 1)
            row_i = idx % rows_per_col
            col_frame = self._auto_col_frames[col_i]

            rf = tk.Frame(col_frame, bg=_BG2)
            rf.grid(row=row_i, column=0, sticky="w", pady=2, padx=2)

            cv = tk.Canvas(rf, width=10, height=10,
                           bg=_BG2, highlightthickness=0)
            cv.pack(side="left", padx=(2, 3))
            dot = cv.create_oval(1, 1, 9, 9, fill=_DIM_FG, outline="")

            name_lbl = tk.Label(rf, text=label, bg=_BG2, fg=_DIM_FG,
                                 font=_F_ITEM_B, anchor="w")
            name_lbl.pack(side="left", padx=(0, 4))

            detail_lbl = tk.Label(rf, text="WAITING…", bg=_BG2,
                                   fg=_LABEL_FG, font=_F_DETAIL, anchor="w")
            detail_lbl.pack(side="left")

            self._auto_rows[chk_id] = (rf, cv, dot, name_lbl, detail_lbl)

            if chk_id in self._auto_state:
                passed = self._auto_state[chk_id]
                if passed:
                    cv.itemconfig(dot, fill=_GREEN)
                    name_lbl.config(fg=_VALUE_FG)
                    detail_lbl.config(text="✔  OK", fg=_TICK_OK)
                else:
                    cv.itemconfig(dot, fill=_RED)
                    name_lbl.config(fg=_RED)
                    detail_lbl.config(text="✗  FAIL", fg=_ORANGE)

    # =========================================================================
    # Banner refresh
    # =========================================================================

    def _refresh_banner(self):
        if not self._has_data:
            self._count_lbl.config(text="WAITING FOR DATA", fg=_ORANGE)
            self._banner_lbl.config(text="WAITING FOR DATA…", fg=_ORANGE)
            self._banner_frame.config(highlightbackground=_BORDER)
            return

        auto_ok    = all(self._auto_state.get(cid, False)
                         for cid, _, _ in _AUTO_CHECKS)
        manual_ok  = all(var.get() for var in self._manual_vars.values())

        n_auto_bad   = sum(1 for cid, _, _ in _AUTO_CHECKS
                           if not self._auto_state.get(cid, False))
        n_manual_bad = sum(1 for v in self._manual_vars.values() if not v.get())
        remaining    = n_auto_bad + n_manual_bad

        if remaining == 0:
            self._count_lbl.config(text="ALL CHECKS PASSED", fg=_GREEN)
            self._banner_lbl.config(text="✔  READY TO ARM",
                                     fg=_GREEN, bg=_BG)
            self._banner_frame.config(highlightbackground=_GREEN)
        else:
            word = f"item{'s' if remaining != 1 else ''}"
            self._count_lbl.config(
                text=f"{remaining} {word} remaining", fg=_ORANGE)
            if not auto_ok:
                self._banner_lbl.config(
                    text="✗  BLOCKED — FIX AUTO CHECKS", fg=_RED, bg=_BG)
                self._banner_frame.config(highlightbackground=_RED)
            else:
                self._banner_lbl.config(
                    text="◉  COMPLETE SIGN-OFF ITEMS", fg=_ORANGE, bg=_BG)
                self._banner_frame.config(highlightbackground=_ORANGE)

    # =========================================================================
    # Public update
    # =========================================================================

    def update_arming(self, data: dict):
        self._has_data = True

        for chk_id, label, evaluator in _AUTO_CHECKS:
            passed, detail = evaluator(data)
            self._auto_state[chk_id] = passed

            rf, cv, dot, name_lbl, detail_lbl = self._auto_rows[chk_id]
            if passed:
                cv.itemconfig(dot, fill=_GREEN)
                name_lbl.config(fg=_VALUE_FG)
                detail_lbl.config(text=f"✔  {detail}", fg=_TICK_OK)
            else:
                cv.itemconfig(dot, fill=_RED)
                name_lbl.config(fg=_RED)
                detail_lbl.config(text=f"✗  {detail}", fg=_ORANGE)

        self._refresh_banner()

    # =========================================================================
    # Public reset
    # =========================================================================

    def reset_checklist(self):
        """Clear all manual sign-off items (call before each new flight)."""
        for item_id, (var, cv, box, tick, lbl) in self._cb_widgets.items():
            var.set(False)
            self._update_checkbox(var, cv, box, tick, lbl)
        self._refresh_banner()