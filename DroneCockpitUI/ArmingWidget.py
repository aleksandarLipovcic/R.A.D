"""
ArmingWidget.py  —  Pre-Flight Checklist & Arming Diagnostics
==============================================================
Two-tier aviation-style checklist.

SECTION A — AUTOMATIC CHECKS  (telemetry-driven, updates every tick)
  • FC arming disable flags  • Gyro  • Accelerometer
  • Battery voltage           • RC link  • GPS fix  • Motors idle

SECTION B — PILOT SIGN-OFF  (manual checkboxes, pilot ticks before each flight)

BANNER  (bottom, always visible — pinned outside scroll area)
  Grey   "WAITING FOR DATA…"       — no telemetry yet
  Red    "BLOCKED — FIX AUTO CHECKS" — one or more auto checks failing
  Orange "COMPLETE SIGN-OFF ITEMS"  — auto OK, manual items remain
  Green  "✔  READY TO ARM"         — everything passed

LAYOUT / SCROLL FIXES (this version):
  1. Header and READY-TO-ARM banner are pinned outside the scroll area so
     they are always visible regardless of window height.
  2. All checklist content (sections A + B) lives inside a Canvas-backed
     scrollable frame. Vertical shrinking of the window never hides items —
     a scrollbar appears automatically.
  3. Mousewheel scrolling works on all child widgets inside the scroll area
     (bound recursively after build).
  4. Responsive column layout (1-col / 2-col) is now driven by the Canvas
     <Configure> event so the column count uses the actual content width,
     not the outer-frame width (avoids a 1-pixel-off edge case with the
     scrollbar visible).
  5. Detail-label wraplength updates whenever the canvas is resized.

LOGIC FIXES (carried from previous version):
  • only_transient detection in update_arming() uses set-intersection test.
  • GPS_NOT_READY is orange (transient); CPU_OVERLOAD is red (hard error).
  • ARM channel highlight threshold is 1800 µs to match BF Modes default.
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

# Flags that self-clear and do not indicate a permanent hardware problem.
# Still shown as FAIL (FC won't arm), but coloured orange and labelled
# "waiting" so the pilot knows to wait rather than investigate hardware.
_TRANSIENT_FLAGS = {
    1 << 9:  "BOOT GRACE",    # clears ~30 s after power-on
    1 << 12: "CALIBRATING",   # clears when IMU cal finishes (~5 s)
    1 << 18: "GPS NOT READY", # clears once GPS has a 3D fix
}

# Pre-built set of transient flag names for fast membership tests.
_TRANSIENT_NAMES: frozenset = frozenset(_TRANSIENT_FLAGS.values())

# Flags that are hard errors with additional explanatory context.
_FLAG_HINTS = {
    "CPU OVERLOAD":   "→ disable GPS Rescue or reduce loop rate",
    "GPS NOT READY":  "→ waiting for GPS fix",
    "NO GYRO":        "→ hardware fault or FC not initialised",
    "RUNAWAY TKOFF":  "→ check PID tune / motor direction",
    "CRASH DETECT":   "→ power-cycle FC",
    "BOOT GRACE":     "→ wait ~30 s after power-on",
    "CALIBRATING":    "→ wait for IMU calibration",
    "MSP OVERRIDE":   "→ GCS is holding override — disconnect MSP",
    "MOTOR PROTOCOL": "→ check ESC protocol in BF Config tab",
}

# ── Auto-check evaluators ─────────────────────────────────────────────────────

def _check_arm_flags(data):
    """
    Evaluate MSP_STATUS_EX arming-disable bitmask.

    Three tiers:
      • flags == 0                      → ALL CLEAR (green)
      • only transient flags set        → WAITING   (orange, self-clearing)
      • any non-transient flag set      → BLOCKED   (red, needs action)
    """
    flags = int(data.get("arming_disable_flags", 0))
    if flags == 0:
        return True, "ALL FLAGS CLEAR"

    active     = [name for bit, name in _ARMING_BITS if flags & bit]
    active_set = set(active)
    transient  = active_set & _TRANSIENT_NAMES
    real       = [n for n in active if n not in _TRANSIENT_NAMES]

    if real:
        parts = []
        for name in real[:2]:
            hint = _FLAG_HINTS.get(name, "")
            parts.append(f"{name}{(' ' + hint) if hint else ''}")
        detail = " | ".join(parts)
        if len(real) > 2:
            detail += f" +{len(real)-2} more"
        return False, detail

    # Only transient flags — still blocking but will self-clear
    parts = []
    for name in sorted(transient):
        hint = _FLAG_HINTS.get(name, "")
        parts.append(f"{name}{(' ' + hint) if hint else ''}")
    detail = ", ".join(parts) + " (self-clearing)"
    return False, detail


def _check_gyro(data):
    ok = bool(data.get("sensor_gyro_present", False))
    return ok, "ONLINE" if ok else "NOT DETECTED"


def _check_acc(data):
    ok = bool(data.get("sensor_acc_present", False))
    return ok, "ONLINE" if ok else "NOT DETECTED"


def _check_battery(data):
    v     = float(data.get("battery_voltage", 0.0))
    cells = int(data.get("battery_cell_count", 0))
    state = str(data.get("battery_state", "INIT")).upper()

    if v < 1.0:
        if cells > 0:
            return False, f"0V — CHECK BF VBAT SCALE / METER TYPE ({cells}S)"
        if state == "NOT_PRESENT":
            return False, "BATTERY NOT DETECTED — CHECK CONNECTOR"
        return False, "NO VOLTAGE — BATTERY UNPLUGGED OR METER=NONE"

    if cells == 0:
        for n in range(1, 9):
            if v <= n * 4.25 + 0.1:
                cells = n
                break

    if cells == 0:
        return False, f"{v:.2f}V — CELL COUNT UNKNOWN"

    cv = v / cells
    if cv < 3.40:
        return False, f"{v:.2f}V  {cv:.2f}V/cell — CRITICAL"
    if cv < 3.65:
        return True,  f"{v:.2f}V  {cv:.2f}V/cell — LOW"
    return True,      f"{v:.2f}V  {cv:.2f}V/cell"


def _check_rc_link(data):
    lq       = int(data.get("rc_link_quality", -1))
    rc_count = int(data.get("rc_channel_count", 0))

    if lq >= 0:
        if lq < 30:
            return False, f"WEAK  {lq}%"
        return True, f"QUALITY  {lq}%"

    if rc_count == 0:
        return False, "NO CH — UART SERIAL RX NOT ENABLED IN BF"

    sticks = [
        int(data.get("rc_roll",     0)),
        int(data.get("rc_pitch",    0)),
        int(data.get("rc_throttle", 0)),
        int(data.get("rc_yaw",      0)),
    ]
    if any(900 <= v <= 2100 for v in sticks):
        return True, f"ACTIVE — {rc_count} CH  (CRSF/ELRS)"

    return False, f"FAILSAFE / TX OFF — {rc_count} CH DETECTED"


def _check_gps(data):
    fix_type = int(data.get("gps_fix_type", 0))
    sats     = int(data.get("gps_num_sats", 0) or data.get("gps_num_sat", 0))
    hdop     = float(data.get("gps_hdop", 99.0))

    if fix_type < 2:
        return False, f"NO 3D FIX  ({sats} sats)"
    if sats < 6:
        return False, f"3D FIX  {sats} SATS — NEED ≥ 6"
    hdop_str = f"  HDOP {hdop:.2f}" if hdop < 90 else ""
    return True, f"3D FIX  {sats} SATS{hdop_str}"


def _check_motors(data):
    vals  = [int(data.get(f"motor_{i}_us", 0)) for i in range(1, 5)]
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

# ── Layout breakpoint ─────────────────────────────────────────────────────────
_TWO_COL_MIN_WIDTH = 560


class ArmingWidget(tk.Frame):

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_BG, **kwargs)
        self._auto_state:   dict = {}
        self._auto_rows:    dict = {}
        self._manual_vars:  dict = {}
        self._manual_lbls:  dict = {}
        self._cb_widgets:   dict = {}
        self._has_data          = False
        self._n_auto_cols       = 0
        self._last_canvas_w     = 0
        self._build_ui()

    # =========================================================================
    # Build
    # =========================================================================

    def _build_ui(self):
        # ── Fixed header (always visible) ────────────────────────────────────
        hdr = tk.Frame(self, bg=_BG2,
                       highlightthickness=1, highlightbackground=_BORDER)
        hdr.pack(fill="x", padx=6, pady=(6, 3))
        tk.Label(hdr, text="PRE-FLIGHT CHECKLIST",
                 bg=_BG2, fg=_LABEL_FG, font=_F_SECTION).pack(
            side="left", padx=10, pady=5)
        self._count_lbl = tk.Label(hdr, text="WAITING FOR DATA",
                                    bg=_BG2, fg=_ORANGE, font=_F_COUNTER)
        self._count_lbl.pack(side="right", padx=8, pady=5)

        # ── Fixed banner (always visible — pinned to bottom) ─────────────────
        self._banner_frame = tk.Frame(self, bg=_BG,
                                       highlightthickness=1,
                                       highlightbackground=_BORDER)
        self._banner_frame.pack(side="bottom", fill="x", padx=6, pady=(2, 6))
        self._banner_lbl = tk.Label(
            self._banner_frame, text="WAITING FOR DATA…",
            bg=_BG, fg=_ORANGE, font=_F_BANNER)
        self._banner_lbl.pack(pady=8)

        # ── Scrollable content area ───────────────────────────────────────────
        # Canvas + Scrollbar sit in a plain container frame so that the
        # scrollbar stays flush with the canvas edge.
        sc_container = tk.Frame(self, bg=_BG)
        sc_container.pack(fill="both", expand=True)

        self._scroll_canvas = tk.Canvas(sc_container, bg=_BG,
                                         highlightthickness=0, bd=0)
        self._vscroll = tk.Scrollbar(sc_container, orient="vertical",
                                      command=self._scroll_canvas.yview)
        self._scroll_canvas.configure(yscrollcommand=self._vscroll.set)

        self._vscroll.pack(side="right", fill="y")
        self._scroll_canvas.pack(side="left", fill="both", expand=True)

        # Inner frame is the real container for all checklist widgets.
        self._scroll_inner = tk.Frame(self._scroll_canvas, bg=_BG)
        self._canvas_win = self._scroll_canvas.create_window(
            (0, 0), window=self._scroll_inner, anchor="nw")

        # Keep inner frame width == canvas width and scroll region updated.
        self._scroll_inner.bind("<Configure>", self._on_inner_configure)
        self._scroll_canvas.bind("<Configure>", self._on_canvas_configure)

        # Propagate mousewheel events from all children up to the canvas.
        self._bind_mw(self._scroll_canvas)
        self._bind_mw(self._scroll_inner)

        # ── Section A ─────────────────────────────────────────────────────────
        self._build_section_hdr("A  AUTOMATIC CHECKS  (telemetry)")

        auto_outer = tk.Frame(self._scroll_inner, bg=_BG2,
                              highlightthickness=1, highlightbackground=_BORDER)
        auto_outer.pack(fill="x", padx=6, pady=(0, 4))

        self._auto_inner = tk.Frame(auto_outer, bg=_BG2)
        self._auto_inner.pack(fill="x", padx=4, pady=4)

        self._auto_col_frames: list = []
        self._layout_auto_cols(1)

        # ── Section B ─────────────────────────────────────────────────────────
        self._build_section_hdr("B  PILOT SIGN-OFF  (manual)")

        signoff = tk.Frame(self._scroll_inner, bg=_BG3,
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
            tick = cv.create_text(8, 8, text="", fill=_GREEN, font=_F_ITEM_B)

            lbl = tk.Label(row, text=item_text, bg=_BG3, fg=_LABEL_FG,
                           font=_F_ITEM, anchor="w", cursor="hand2",
                           wraplength=0)
            lbl.pack(side="left", fill="x", expand=True)
            self._manual_lbls[item_id] = lbl
            self._cb_widgets[item_id]  = (var, cv, box, tick, lbl)

            def _toggle(e, v=var, cv=cv, box=box, tick=tick,
                        lbl=lbl, iid=item_id):
                v.set(not v.get())
                self._update_checkbox(v, cv, box, tick, lbl)
                self._refresh_banner()

            cv.bind("<Button-1>", _toggle)
            lbl.bind("<Button-1>", _toggle)

            # Scroll on all interactive child widgets inside the scroll area.
            for w in (cv, lbl, row):
                self._bind_mw(w)

        rst_row = tk.Frame(signoff, bg=_BG3)
        rst_row.pack(fill="x", padx=6, pady=(4, 6))
        rst_btn = tk.Button(
            rst_row, text="↺  RESET SIGN-OFF",
            bg=_BORDER, fg=_LABEL_FG, font=_F_RESET,
            activebackground=_DIM, activeforeground=_VALUE_FG,
            relief="flat", padx=8, pady=3,
            command=self.reset_checklist,
        )
        rst_btn.pack(side="right")
        self._bind_mw(rst_row)
        self._bind_mw(rst_btn)

    # =========================================================================
    # Section header (inside scroll area)
    # =========================================================================

    def _build_section_hdr(self, text):
        f = tk.Frame(self._scroll_inner, bg=_BG)
        f.pack(fill="x", padx=6, pady=(4, 1))
        tk.Label(f, text=text, bg=_BG, fg=_LABEL_FG,
                 font=_F_SECTION).pack(side="left")
        tk.Frame(f, bg=_BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(6, 0), pady=4)
        self._bind_mw(f)

    # =========================================================================
    # Mousewheel helpers
    # =========================================================================

    def _bind_mw(self, widget):
        """Bind all three mousewheel events so scrolling works on every OS."""
        widget.bind("<MouseWheel>", self._on_mousewheel, add="+")
        widget.bind("<Button-4>",   self._on_mousewheel, add="+")  # Linux up
        widget.bind("<Button-5>",   self._on_mousewheel, add="+")  # Linux down

    def _on_mousewheel(self, event):
        if event.num == 4:
            self._scroll_canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self._scroll_canvas.yview_scroll(1, "units")
        else:
            self._scroll_canvas.yview_scroll(
                int(-1 * (event.delta / 120)), "units")

    # =========================================================================
    # Canvas / scroll helpers
    # =========================================================================

    def _on_inner_configure(self, _event):
        """Keep scroll region in sync when the inner frame changes size."""
        self._scroll_canvas.configure(
            scrollregion=self._scroll_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        """
        Called when the Canvas is resized.
        1. Stretch the inner frame to fill the canvas width.
        2. Rebuild the auto-check column layout if we crossed the breakpoint.
        3. Update detail-label wraplengths.
        """
        cw = event.width
        # Make the inner frame fill the full canvas width.
        self._scroll_canvas.itemconfig(self._canvas_win, width=cw)

        if cw == self._last_canvas_w:
            return
        self._last_canvas_w = cw

        n = 2 if cw >= _TWO_COL_MIN_WIDTH else 1
        if n != self._n_auto_cols:
            self._layout_auto_cols(n)

        # Clamp detail text to the available column width so it wraps
        # gracefully instead of being clipped by the column boundary.
        usable = max(80, (cw // n) - 130)
        for _chk_id, (_rf, _cv, _dot, _name_lbl, detail_lbl) in self._auto_rows.items():
            detail_lbl.config(wraplength=usable)

    # =========================================================================
    # Checkbox helper
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

    def _layout_auto_cols(self, n: int):
        self._n_auto_cols = n

        # Destroy existing column frames and their canvas dot refs.
        for cf in self._auto_col_frames:
            cf.destroy()
        self._auto_col_frames = []
        self._auto_rows = {}

        total        = len(_AUTO_CHECKS)
        rows_per_col = (total + n - 1) // n

        for i in range(n):
            cf = tk.Frame(self._auto_inner, bg=_BG2)
            cf.grid(row=0, column=i, sticky="nw", padx=(0, 12))
            self._auto_inner.columnconfigure(i, weight=1)
            self._auto_col_frames.append(cf)
            self._bind_mw(cf)

        for idx, (chk_id, label, _fn) in enumerate(_AUTO_CHECKS):
            col_i     = min(idx // rows_per_col, n - 1)
            row_i     = idx % rows_per_col
            col_frame = self._auto_col_frames[col_i]

            rf = tk.Frame(col_frame, bg=_BG2)
            rf.grid(row=row_i, column=0, sticky="ew", pady=2, padx=2)
            col_frame.columnconfigure(0, weight=1)
            self._bind_mw(rf)

            cv = tk.Canvas(rf, width=10, height=10,
                           bg=_BG2, highlightthickness=0)
            cv.pack(side="left", padx=(2, 3))
            dot = cv.create_oval(1, 1, 9, 9, fill=_DIM_FG, outline="")
            self._bind_mw(cv)

            name_lbl = tk.Label(rf, text=label, bg=_BG2, fg=_DIM_FG,
                                 font=_F_ITEM_B, anchor="w")
            name_lbl.pack(side="left", padx=(0, 4))
            self._bind_mw(name_lbl)

            detail_lbl = tk.Label(rf, text="WAITING…", bg=_BG2,
                                   fg=_LABEL_FG, font=_F_DETAIL, anchor="w",
                                   wraplength=200, justify="left")
            detail_lbl.pack(side="left", fill="x", expand=True)
            self._bind_mw(detail_lbl)

            self._auto_rows[chk_id] = (rf, cv, dot, name_lbl, detail_lbl)

            # Restore state if the widget was rebuilt after a resize.
            if chk_id in self._auto_state:
                passed = self._auto_state[chk_id]
                col    = _GREEN if passed else _RED
                cv.itemconfig(dot, fill=col)
                name_lbl.config(fg=_VALUE_FG if passed else _RED)

    # =========================================================================
    # Banner
    # =========================================================================

    def _refresh_banner(self):
        if not self._has_data:
            self._count_lbl.config(text="WAITING FOR DATA", fg=_ORANGE)
            self._banner_lbl.config(text="WAITING FOR DATA…", fg=_ORANGE)
            self._banner_frame.config(highlightbackground=_BORDER)
            return

        auto_ok = all(self._auto_state.get(cid, False)
                      for cid, _, _ in _AUTO_CHECKS)

        n_auto_bad   = sum(1 for cid, _, _ in _AUTO_CHECKS
                           if not self._auto_state.get(cid, False))
        n_manual_bad = sum(1 for v in self._manual_vars.values() if not v.get())
        remaining    = n_auto_bad + n_manual_bad

        if remaining == 0:
            self._count_lbl.config(text="ALL CHECKS PASSED", fg=_GREEN)
            self._banner_lbl.config(text="✔  READY TO ARM", fg=_GREEN, bg=_BG)
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

            if chk_id not in self._auto_rows:
                continue

            rf, cv, dot, name_lbl, detail_lbl = self._auto_rows[chk_id]

            # ── only_transient detection ──────────────────────────────────────
            # Build the set of active flag names, then test whether that set
            # is non-empty AND every member is a known transient name.
            # This avoids the previous inverted-generator bug where all()
            # was vacuously True for empty filtered sequences.
            if chk_id == "arm_flags" and not passed:
                flags        = int(data.get("arming_disable_flags", 0))
                active_names = {name for bit, name in _ARMING_BITS if flags & bit}
                only_transient = bool(active_names) and active_names.issubset(_TRANSIENT_NAMES)
            else:
                only_transient = False

            if passed:
                cv.itemconfig(dot, fill=_GREEN)
                name_lbl.config(fg=_VALUE_FG)
                detail_lbl.config(text=f"✔  {detail}", fg=_TICK_OK)
            elif only_transient:
                cv.itemconfig(dot, fill=_ORANGE)
                name_lbl.config(fg=_ORANGE)
                detail_lbl.config(text=f"◉  {detail}", fg=_ORANGE)
            else:
                cv.itemconfig(dot, fill=_RED)
                name_lbl.config(fg=_RED)
                detail_lbl.config(text=f"✗  {detail}", fg=_ORANGE)

        self._refresh_banner()

    # =========================================================================
    # Public reset
    # =========================================================================

    def reset_checklist(self):
        for item_id, (var, cv, box, tick, lbl) in self._cb_widgets.items():
            var.set(False)
            self._update_checkbox(var, cv, box, tick, lbl)
        self._refresh_banner()