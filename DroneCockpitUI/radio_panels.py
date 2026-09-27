"""
radio_panels.py  —  radio-link faces for the dual-layer instrument panels
=========================================================================

On the ELRS radio link the drone sends far less than over the USB cable
(see link_mode.py). Instead of showing the USB layouts full of "USB"
placeholders, these three panels take the same slots in the cockpit and are
built around what the radio actually delivers:

  Slot        USB face          Radio face (this file)
  ─────────── ───────────────── ──────────────────────────────────────────
  imu         IMUWidget         RadioAttitudePanel — large roll / pitch /
                                heading readouts with tilt alarms, attitude
                                age / rate, uplink LQ and RSSI
  mag         MagWidget         RadioNavPanel — heading-up compass with the
                                direction HOME, distance / bearing / turn
                                cue, ground speed, altitude
  fc_status   FCStatusWidget    RadioFlightPanel — armed / mode, battery,
                                the shared flight timer, full ELRS link
                                statistics, frame rates and counters

DualLayerPanel (dual_layer.py) swaps them in place when the telemetry source
changes; nothing moves, nothing needs to be reselected.

Every face takes the same ui_data dict as the USB widgets (update_radio),
never touches DroneBackend, and marks old data as STALE while keeping the
last value — for search & rescue the last known state before a link loss
matters most. Main thread only.
"""

import math
import tkinter as tk

import link_mode

# ── Shared palette (matches the USB widgets) ─────────────────────────────────
BG        = "#0a0a10"
BG_HDR    = "#0d1520"
BG_CELL   = "#0d1520"
BORDER    = "#1e2a3a"
LABEL     = "#4a6a8a"
TEXT      = "#c8dff0"
GREEN     = "#00ff88"
AMBER_BG  = "#C87000"
AMBER     = "#ffaa00"
RED       = "#ff3355"
CRIT_A    = "#CC0000"
CRIT_B    = "#500000"
CYAN      = "#00d4ff"
DIM       = "#2a4055"

F_HDR   = ("Consolas", 8, "bold")
F_SMALL = ("Consolas", 8)
F_TINY  = ("Consolas", 7, "bold")
F_VALUE = ("Consolas", 10, "bold")

# Tilt limits — same values as IMUWidget / Drone3DView
TILT_WARN_DEG = 15.0
TILT_CRIT_DEG = 30.0

# ELRS link colour limits
LQ_WARN = 70          # CrsfLink DEGRADED threshold
LQ_CRIT = 30
RSSI_MIN_DBM = -120   # bar scale
RSSI_MAX_DBM = -40
RSSI_WARN_DBM = -90   # indicative only; depends on packet rate
RSSI_CRIT_DBM = -105


def _header(parent, title):
    """Title strip with a right-aligned tag label; returns the tag."""
    hdr = tk.Frame(parent, bg=BG_HDR, highlightthickness=1,
                   highlightbackground=BORDER)
    tk.Label(hdr, text=title, bg=BG_HDR, fg=LABEL, font=F_HDR
             ).pack(side="left", padx=8, pady=3)
    tag = tk.Label(hdr, text="RADIO", bg=BG_HDR, fg=link_mode.C_RADIO,
                   font=F_TINY)
    tag.pack(side="right", padx=8)
    return hdr, tag


def _set_tag(tag, data, group):
    old = link_mode.stale_ms(data, group)
    if old:
        tag.config(text=link_mode.stale_text(old), fg=link_mode.C_STALE_FG)
    else:
        txt, clr = link_mode.source_tag(data)
        tag.config(text=txt, fg=clr)
    return old


def _fmt_dist(m: float) -> str:
    return f"{m:.0f} m" if m < 1000 else f"{m / 1000:.2f} km"


def _tilt_state(deg: float) -> str:
    a = abs(deg)
    if a >= TILT_CRIT_DEG:
        return "crit"
    if a >= TILT_WARN_DEG:
        return "warn"
    return "safe"


def _lq_colour(lq: int) -> str:
    return RED if lq < LQ_CRIT else AMBER if lq < LQ_WARN else GREEN


def _rssi_colour(dbm: int) -> str:
    return RED if dbm <= RSSI_CRIT_DBM else AMBER if dbm <= RSSI_WARN_DBM else GREEN


class _Bar(tk.Canvas):
    """Thin horizontal level bar (0..1)."""

    def __init__(self, parent, height=9):
        super().__init__(parent, height=height, bg=BG, highlightthickness=0)
        self._ratio, self._colour = 0.0, DIM
        self.bind("<Configure>", lambda e: self._draw())

    def set(self, ratio: float, colour: str):
        self._ratio = max(0.0, min(1.0, ratio))
        self._colour = colour
        self._draw()

    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 4:
            return
        self.create_rectangle(0, 0, w - 1, h - 1, fill="#111827", outline=BORDER)
        if self._ratio > 0:
            self.create_rectangle(1, 1, max(2, int((w - 2) * self._ratio)), h - 2,
                                  fill=self._colour, outline="")


# =============================================================================
# imu slot — RadioAttitudePanel
# =============================================================================

class RadioAttitudePanel(tk.Frame):
    """
    Attitude on the radio link: ROLL / PITCH with the IMU widget's tilt
    alarms (amber ≥ 15°, flashing red ≥ 30°), HEADING (FC fused yaw, no
    alarm), and the numbers that say how trustworthy it is right now:
    attitude age, attitude frame rate, uplink LQ and RSSI.
    """

    _FLASH_MS = 500

    def __init__(self, parent):
        super().__init__(parent, bg=BG)
        self.columnconfigure(1, weight=1)
        hdr, self._tag = _header(self, "ATTITUDE  ·  RADIO LINK")
        hdr.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=(4, 2))

        self._values = {}
        for r, (key, name) in enumerate((("roll", "ROLL"), ("pitch", "PITCH"),
                                         ("hdg", "HEADING")), start=1):
            self.rowconfigure(r, weight=1)
            tk.Label(self, text=name, bg=BG, fg=LABEL, font=F_HDR, anchor="w"
                     ).grid(row=r, column=0, sticky="w", padx=(10, 6))
            v = tk.Label(self, text="---", bg=BG_CELL, fg=GREEN, anchor="e",
                         font=("Consolas", 20, "bold"), padx=10,
                         highlightthickness=1, highlightbackground=BORDER)
            v.grid(row=r, column=1, sticky="nsew", padx=(0, 6), pady=2)
            self._values[key] = v

        foot = tk.Frame(self, bg=BG_HDR, highlightthickness=1,
                        highlightbackground=BORDER)
        foot.grid(row=4, column=0, columnspan=2, sticky="ew", padx=4, pady=(2, 4))
        self._foot = {}
        for key, name in (("age", "ATT AGE"), ("rate", "ATT RATE"),
                          ("lq", "UPLINK LQ"), ("rssi", "RSSI")):
            blk = tk.Frame(foot, bg=BG_HDR)
            blk.pack(side="left", expand=True, fill="x", padx=4, pady=2)
            tk.Label(blk, text=name, bg=BG_HDR, fg=LABEL, font=F_TINY).pack()
            lbl = tk.Label(blk, text="---", bg=BG_HDR, fg=CYAN, font=F_VALUE)
            lbl.pack()
            self._foot[key] = lbl

        self._crit = set()
        self._flash_on = True
        self._flash_job = None
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event):
        # Three value rows share the height left after header + footer.
        size = int(max(10, min(40, (event.height - 90) / 3 * 0.45)))
        for v in self._values.values():
            v.config(font=("Consolas", size, "bold"))

    def _apply(self, key, text, state):
        lbl = self._values[key]
        self._crit.discard(key)
        if state == "stale":
            lbl.config(text=text, bg=BG_CELL, fg=link_mode.C_STALE_FG)
        elif state == "warn":
            lbl.config(text=text, bg=AMBER_BG, fg="#000000")
        elif state == "crit":
            self._crit.add(key)
            lbl.config(text=text, bg=CRIT_A if self._flash_on else CRIT_B,
                       fg="#FFFFFF")
        else:
            lbl.config(text=text, bg=BG_CELL, fg=GREEN)
        if self._crit and self._flash_job is None:
            self._flash_job = self.after(self._FLASH_MS, self._flash)

    def _flash(self):
        self._flash_job = None
        if not self._crit:
            return
        self._flash_on = not self._flash_on
        for key in self._crit:
            try:
                self._values[key].config(bg=CRIT_A if self._flash_on else CRIT_B)
            except tk.TclError:
                return
        self._flash_job = self.after(self._FLASH_MS, self._flash)

    def update_radio(self, data: dict):
        old = _set_tag(self._tag, data, "attitude")
        roll = float(data.get("roll", 0.0))
        pitch = float(data.get("pitch", 0.0))
        hdg = float(data.get("yaw", 0.0)) % 360.0
        self._apply("roll", f"{roll:+6.1f}°", "stale" if old else _tilt_state(roll))
        self._apply("pitch", f"{pitch:+6.1f}°", "stale" if old else _tilt_state(pitch))
        self._apply("hdg", f"{hdg:05.1f}°", "stale" if old else "safe")

        age = link_mode.age_ms(data, "attitude")
        self._foot["age"].config(
            text=f"{age} ms" if age >= 0 else "never",
            fg=link_mode.C_STALE_FG if old else CYAN)
        self._foot["rate"].config(text=f"{float(data.get('rate_attitude_hz', 0.0)):.1f} Hz")
        lq = int(data.get("rc_link_quality", -1))
        self._foot["lq"].config(text=f"{lq}%" if lq >= 0 else "---",
                                fg=_lq_colour(lq) if lq >= 0 else DIM)
        if data.get("elrs_link_stats_valid"):
            rssi = int(data.get("elrs_uplink_rssi1_dbm", 0))
            self._foot["rssi"].config(text=f"{rssi} dBm", fg=_rssi_colour(rssi))
        else:
            self._foot["rssi"].config(text="---", fg=DIM)

    def destroy(self):
        if self._flash_job is not None:
            try:
                self.after_cancel(self._flash_job)
            except Exception:
                pass
        super().destroy()


# =============================================================================
# mag slot — RadioNavPanel
# =============================================================================

class RadioNavPanel(tk.Frame):
    """
    Heading & home on the radio link. The ELRS link has no magnetometer
    data, but it does carry the FC's heading and GPS — and CrsfLink
    computes the home point at arming. For search & rescue the question in
    this slot becomes "which way is home, and how far?":

      • heading-up compass: aircraft symbol fixed pointing up, cardinals
        rotate, a cyan arrow points to HOME;
      • HDG, HOME distance, bearing, TURN L/R cue, ground speed, altitude.
    """

    def __init__(self, parent):
        super().__init__(parent, bg=BG)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        hdr, self._tag = _header(self, "HEADING & HOME  ·  RADIO LINK")
        hdr.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=(4, 2))

        self._cv = tk.Canvas(self, bg=BG, highlightthickness=0)
        self._cv.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        self._cv.bind("<Configure>", lambda e: self._draw())

        col = tk.Frame(self, bg=BG)
        col.grid(row=1, column=1, sticky="ns", padx=(0, 8), pady=4)
        self._rows = {}
        for key, name in (("hdg", "HDG"), ("dist", "HOME"), ("brg", "BRG HOME"),
                          ("turn", "TURN"), ("gs", "GND SPD"), ("alt", "ALT")):
            tk.Label(col, text=name, bg=BG, fg=LABEL, font=F_TINY, anchor="w"
                     ).pack(anchor="w")
            lbl = tk.Label(col, text="---", bg=BG, fg=TEXT, font=F_VALUE,
                           anchor="w")
            lbl.pack(anchor="w", pady=(0, 3))
            self._rows[key] = lbl

        self._hdg = 0.0
        self._home_rel = None     # relative bearing to home, or None
        self._hdg_stale = False

    # ── drawing ──────────────────────────────────────────────────────────────
    def _pt(self, cx, cy, r, ang_deg):
        a = math.radians(ang_deg)
        return cx + r * math.sin(a), cy - r * math.cos(a)

    def _draw(self):
        cv = self._cv
        cv.delete("all")
        w, h = cv.winfo_width(), cv.winfo_height()
        r = min(w, h) / 2 - 10
        if r < 20:
            return
        cx, cy = w / 2, h / 2
        cv.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#1e3040", width=2,
                       fill="#0d1117")
        # Rotating scale: heading-up, so a card at bearing B sits at B - hdg
        for deg in range(0, 360, 30):
            ang = deg - self._hdg
            x0, y0 = self._pt(cx, cy, r, ang)
            x1, y1 = self._pt(cx, cy, r - (12 if deg % 90 == 0 else 7), ang)
            cv.create_line(x0, y0, x1, y1, fill="#c8d8e8" if deg % 90 == 0 else "#3a4a5a",
                           width=2 if deg % 90 == 0 else 1)
            if deg % 90 == 0:
                tx, ty = self._pt(cx, cy, r - 24, ang)
                card = {0: "N", 90: "E", 180: "S", 270: "W"}[deg]
                cv.create_text(tx, ty, text=card, font=("Consolas", max(8, int(r / 7)), "bold"),
                               fill="#FF4444" if card == "N" else "#c8d8e8")
        # Lubber line
        cv.create_polygon(cx - 6, cy - r - 2, cx + 6, cy - r - 2, cx, cy - r + 8,
                          fill="#FFD700", outline="")
        # Home arrow
        if self._home_rel is not None:
            tip = self._pt(cx, cy, r * 0.78, self._home_rel)
            tail = self._pt(cx, cy, r * 0.25, self._home_rel)
            cv.create_line(*tail, *tip, fill=CYAN, width=4, arrow="last",
                           arrowshape=(14, 16, 6))
            hx, hy = self._pt(cx, cy, r * 0.9, self._home_rel)
            cv.create_text(hx, hy, text="H", fill=CYAN, font=("Consolas", 10, "bold"))
        else:
            cv.create_text(cx, cy + r * 0.45, text="HOME NOT SET",
                           fill=link_mode.C_NA_FG, font=F_HDR)
        # Aircraft symbol (always nose-up in a heading-up display)
        s = max(8, r / 6)
        cv.create_polygon(cx, cy - s, cx + s * 0.7, cy + s * 0.8, cx, cy + s * 0.4,
                          cx - s * 0.7, cy + s * 0.8,
                          fill=link_mode.C_STALE_FG if self._hdg_stale else "#FFD700",
                          outline="")

    # ── data ─────────────────────────────────────────────────────────────────
    def update_radio(self, data: dict):
        att_old = link_mode.stale_ms(data, "attitude")
        gps_old = link_mode.stale_ms(data, "gps")
        _set_tag(self._tag, data, "gps" if gps_old and not att_old else "attitude")

        self._hdg = float(data.get("yaw", 0.0)) % 360.0
        self._hdg_stale = bool(att_old)
        self._rows["hdg"].config(text=f"{self._hdg:05.1f}°",
                                 fg=link_mode.C_STALE_FG if att_old else CYAN)

        home_ok = bool(data.get("gps_comp_valid", False)) and \
            (bool(data.get("home_set", False)) or not link_mode.is_radio(data))
        gps_fg = link_mode.C_STALE_FG if gps_old else TEXT
        if home_ok:
            dist = float(data.get("gps_dist_to_home_m", 0.0))
            brg = (int(data.get("gps_bearing_to_home", 0)) + 360) % 360
            rel = (brg - self._hdg + 540.0) % 360.0 - 180.0
            self._home_rel = rel
            self._rows["dist"].config(text=_fmt_dist(dist), fg=gps_fg)
            self._rows["brg"].config(text=f"{brg:03d}°", fg=gps_fg)
            if abs(rel) <= 10:
                turn = "ON COURSE"
            else:
                turn = f"{'R' if rel > 0 else 'L'} {abs(rel):.0f}°"
            self._rows["turn"].config(text=turn, fg=GREEN if abs(rel) <= 10 else AMBER)
        else:
            self._home_rel = None
            for key in ("dist", "brg", "turn"):
                self._rows[key].config(text="---", fg=DIM)

        if data.get("gps_raw_valid", False):
            kmh = float(data.get("gps_ground_speed_cms", 0)) * 0.036
            self._rows["gs"].config(text=f"{kmh:.1f} km/h", fg=gps_fg)
        else:
            self._rows["gs"].config(text="---", fg=DIM)

        if data.get("baro_valid", False):
            alt = int(data.get("baro_altitude_cm", 0)) / 100.0
            src = str(data.get("altitude_source", "BARO")).upper()
            unit = " (GPS)" if src == "GPS" else ""
            self._rows["alt"].config(text=f"{alt:+.1f} m{unit}", fg=TEXT)
        else:
            self._rows["alt"].config(text="---", fg=DIM)
        self._draw()


# =============================================================================
# fc_status slot — RadioFlightPanel
# =============================================================================

class RadioFlightPanel(tk.Frame):
    """
    Flight & link on the radio link: armed state and flight mode, battery,
    the cockpit's single flight timer (owned by FCStatusWidget, so a USB ↔
    radio switch mid-flight never restarts it), and everything ELRS reports
    about the link itself — which on this link replaces the FC metrics,
    motors and RC channels the USB face shows.
    """

    def __init__(self, parent, timer_provider=None):
        super().__init__(parent, bg=BG)
        self._timer = timer_provider

        # ── ARM + mode ─────────────────────────────────────────────────────
        top = tk.Frame(self, bg=BG_HDR, highlightthickness=1,
                       highlightbackground=BORDER)
        top.pack(fill="x", padx=4, pady=(4, 2))
        self._top = top
        self._arm_cv = tk.Canvas(top, width=14, height=14, bg=BG_HDR,
                                 highlightthickness=0)
        self._arm_cv.pack(side="left", padx=(8, 4), pady=6)
        self._arm_dot = self._arm_cv.create_oval(1, 1, 13, 13, fill=RED, outline="")
        self._arm_lbl = tk.Label(top, text="DISARMED", bg=BG_HDR, fg=RED,
                                 font=("Consolas", 13, "bold"))
        self._arm_lbl.pack(side="left")
        self._tag = tk.Label(top, text="RADIO", bg=BG_HDR, fg=link_mode.C_RADIO,
                             font=F_TINY)
        self._tag.pack(side="right", padx=8)
        self._mode_lbl = tk.Label(top, text="—", bg=BG_HDR, fg=AMBER,
                                  font=("Consolas", 10, "bold"))
        self._mode_lbl.pack(side="left", padx=(14, 4))

        # ── Battery ────────────────────────────────────────────────────────
        bat = tk.Frame(self, bg=BG, highlightthickness=1,
                       highlightbackground=BORDER)
        bat.pack(fill="x", padx=4, pady=2)
        self._bat_frame = bat
        self._volt = tk.Label(bat, text="—", bg=BG, fg=GREEN,
                              font=("Consolas", 18, "bold"))
        self._volt.pack(side="left", padx=(8, 2), pady=4)
        tk.Label(bat, text="V", bg=BG, fg=LABEL, font=F_VALUE).pack(side="left", anchor="s", pady=6)
        mid = tk.Frame(bat, bg=BG)
        mid.pack(side="left", padx=12)
        self._cell = tk.Label(mid, text="— V/cell", bg=BG, fg=GREEN, font=F_VALUE)
        self._cell.pack(anchor="w")
        self._amps = tk.Label(mid, text="— A   — mAh", bg=BG, fg=TEXT, font=F_SMALL)
        self._amps.pack(anchor="w")
        right = tk.Frame(bat, bg=BG)
        right.pack(side="right", padx=8)
        self._bat_state = tk.Label(right, text="INIT", bg=BORDER, fg=DIM,
                                   font=F_HDR, padx=6)
        self._bat_state.pack(anchor="e")
        self._pct = tk.Label(right, text="—%", bg=BG, fg=TEXT, font=F_VALUE)
        self._pct.pack(anchor="e")

        # ── Timer ──────────────────────────────────────────────────────────
        self._timer_lbl = tk.Label(self, text="FLT --:--    REM --:--", bg=BG,
                                   fg=DIM, font=("Consolas", 11, "bold"), anchor="w")
        self._timer_lbl.pack(fill="x", padx=10, pady=(2, 2))

        # ── Link statistics ────────────────────────────────────────────────
        link = tk.Frame(self, bg=BG)
        link.pack(fill="x", padx=4, pady=(2, 0))
        link.columnconfigure(1, weight=1)
        tk.Label(link, text="ELRS LINK", bg=BG, fg=LABEL, font=F_TINY
                 ).grid(row=0, column=0, columnspan=3, sticky="w", padx=4)
        self._bars = {}
        for r, (key, name) in enumerate((("up", "UPLINK LQ"), ("down", "DNLINK LQ"),
                                         ("rssi1", "RSSI ANT1"), ("rssi2", "RSSI ANT2")),
                                        start=1):
            tk.Label(link, text=name, bg=BG, fg=LABEL, font=F_TINY, width=10,
                     anchor="w").grid(row=r, column=0, sticky="w", padx=4)
            bar = _Bar(link)
            bar.grid(row=r, column=1, sticky="ew", padx=4, pady=1)
            val = tk.Label(link, text="---", bg=BG, fg=TEXT, font=F_HDR, width=8,
                           anchor="e")
            val.grid(row=r, column=2, sticky="e", padx=4)
            self._bars[key] = (bar, val)

        self._link_line = tk.Label(self, text="", bg=BG, fg=TEXT, font=F_SMALL,
                                   anchor="w")
        self._link_line.pack(fill="x", padx=8, pady=(2, 0))
        self._rates_line = tk.Label(self, text="", bg=BG, fg=LABEL, font=F_SMALL,
                                    anchor="w")
        self._rates_line.pack(fill="x", padx=8)
        self._count_line = tk.Label(self, text="", bg=BG, fg=LABEL, font=F_SMALL,
                                    anchor="w")
        self._count_line.pack(fill="x", padx=8, pady=(0, 4))

    @staticmethod
    def _cells(data):
        c = int(data.get("battery_cell_count", 0))
        if c > 0:
            return c
        v = float(data.get("battery_voltage", 0.0))
        for n in range(1, 9):
            if v <= n * 4.25 + 0.1:
                return n
        return 0

    def update_radio(self, data: dict):
        _set_tag(self._tag, data, "flight_mode")

        # ARM + mode
        armed = bool(data.get("armed", False))
        self._arm_cv.itemconfig(self._arm_dot, fill=GREEN if armed else RED)
        self._arm_lbl.config(text="ARMED" if armed else "DISARMED",
                             fg=GREEN if armed else RED)
        self._top.config(highlightbackground=GREEN if armed else BORDER)
        mode = str(data.get("flight_mode_name", "—"))
        if "FAILSAFE" in mode or "RESCUE" in mode or "BLOCKED" in mode:
            mc = RED
        elif "DISARMED" in mode or "WAIT" in mode:
            mc = AMBER
        else:
            mc = GREEN if armed else TEXT
        if link_mode.stale_ms(data, "flight_mode"):
            mc = link_mode.C_STALE_FG
        self._mode_lbl.config(text=mode, fg=mc)

        # Battery
        v = float(data.get("battery_voltage", 0.0))
        cells = self._cells(data)
        cv = v / cells if cells else 0.0
        vcol = (RED if cv < 3.40 else AMBER if cv < 3.65 else GREEN) if cv else DIM
        self._volt.config(text=f"{v:.1f}" if v > 0 else "—", fg=vcol)
        self._cell.config(text=f"{cv:.2f} V/cell  {cells}S" if cv else "— V/cell", fg=vcol)
        self._amps.config(text=f"{float(data.get('battery_current', 0.0)):.1f} A   "
                               f"{float(data.get('battery_mah_drawn', 0.0)):.0f} mAh")
        pct = int(data.get("battery_percentage", -1))
        self._pct.config(text=f"{pct}%" if pct >= 0 else "—%")
        state = str(data.get("battery_state", "INIT")).upper()
        sfg, sbg = {"OK": (GREEN, BG_HDR), "WARNING": (AMBER, BG_HDR),
                    "CRITICAL": (RED, BG_HDR)}.get(state, (DIM, BORDER))
        if link_mode.stale_ms(data, "battery"):
            state, sfg, sbg = "STALE", BG, link_mode.C_STALE_FG
        self._bat_state.config(text=state, fg=sfg, bg=sbg)
        self._bat_frame.config(highlightbackground=RED if state == "CRITICAL" else
                               AMBER if state == "WARNING" else BORDER)

        # Timer (shared with the USB face)
        if self._timer is not None:
            elapsed, remaining, t_armed = self._timer()
            em, es = divmod(int(elapsed), 60)
            rem = "--:--" if remaining is None else "%02d:%02d" % divmod(int(remaining), 60)
            fg = (RED if remaining == 0 else AMBER if remaining is not None and remaining <= 120
                  else GREEN) if t_armed else DIM
            self._timer_lbl.config(text=f"FLT {em:02d}:{es:02d}    REM {rem}", fg=fg)

        # Link statistics
        valid = bool(data.get("elrs_link_stats_valid", False))
        ls_old = link_mode.stale_ms(data, "link_stats", link_mode.STALE_SLOW_MS)
        if valid and not ls_old:
            up = int(data.get("elrs_uplink_lq", 0))
            dn = int(data.get("elrs_downlink_lq", 0))
            r1 = int(data.get("elrs_uplink_rssi1_dbm", 0))
            r2 = int(data.get("elrs_uplink_rssi2_dbm", 0))
            span = RSSI_MAX_DBM - RSSI_MIN_DBM
            for key, ratio, colour, text in (
                    ("up", up / 100, _lq_colour(up), f"{up} %"),
                    ("down", dn / 100, _lq_colour(dn), f"{dn} %"),
                    ("rssi1", (r1 - RSSI_MIN_DBM) / span, _rssi_colour(r1), f"{r1} dBm"),
                    ("rssi2", (r2 - RSSI_MIN_DBM) / span, _rssi_colour(r2), f"{r2} dBm")):
                bar, val = self._bars[key]
                bar.set(ratio, colour)
                val.config(text=text, fg=colour)
            self._link_line.config(
                text=(f"SNR {int(data.get('elrs_uplink_snr', 0))} dB   "
                      f"TX {int(data.get('elrs_tx_power_mw', 0))} mW   "
                      f"ANT {int(data.get('elrs_active_antenna', 0)) + 1}   "
                      f"RF MODE {int(data.get('elrs_rf_mode_index', 0))}"),
                fg=TEXT)
        else:
            for bar, val in self._bars.values():
                bar.set(0, DIM)
                val.config(text="---", fg=DIM)
            self._link_line.config(
                text=link_mode.stale_text(ls_old) + " — NO LINK STATISTICS" if ls_old
                else "NO LINK STATISTICS", fg=link_mode.C_STALE_FG)

        self._rates_line.config(text=(
            f"RATES  ATT {float(data.get('rate_attitude_hz', 0)):.1f}  "
            f"GPS {float(data.get('rate_gps_hz', 0)):.1f}  "
            f"BAT {float(data.get('rate_battery_hz', 0)):.1f}  "
            f"MODE {float(data.get('rate_flight_mode_hz', 0)):.1f} Hz"))
        lost = int(data.get("radio_link_lost_count", 0))
        self._count_line.config(
            text=(f"LINK LOST {lost}   RECONNECT {int(data.get('radio_reconnect_count', 0))}"
                  f"   CRC ERR {int(data.get('radio_crc_errors', 0))}"),
            fg=AMBER if lost else LABEL)
