"""
GPSWidget.py — NEO-M10 GPS display for Project R.A.D Ground Control Station

Aviation-standard EFIS-style GPS panel.
Conforms to GCS display tolerances — no interpolation, raw telemetry only.

Layout (fits inside a ttk.LabelFrame, grid or pack):
  ┌─────────────────────────────────────────────────┐
  │  STATUS BAR  [FIX TYPE]  [SATS]  [HDOP]  [♥]   │
  ├──────────────────────┬──────────────────────────┤
  │  COORDINATES         │  COMPASS ROSE            │
  │  LAT / LON / ALT     │  (bearing to home)       │
  ├──────────────────────┴──────────────────────────┤
  │  SPEED   COURSE   DIST-HOME   BEARING-HOME       │
  └─────────────────────────────────────────────────┘

Usage:
    from GPSWidget import GPSWidget

    gps_view = GPSWidget(parent_frame)
    gps_view.pack(...)          # or .grid(...)

    # In your update loop — pass the dict from state.to_dict()
    gps_view.update_gps(ui_data)   # ui_data contains gps_* keys
"""

import tkinter as tk
import math

# ── Palette — EFIS dark cockpit ───────────────────────────────────────────────
C_BG         = "#0d1117"   # panel background
C_BORDER     = "#1e2733"   # subtle widget borders
C_CARD       = "#131a23"   # card / section background
C_LABEL      = "#4a6480"   # dim label text
C_VALUE      = "#e8f4fd"   # primary value text
C_GREEN      = "#00e676"   # 3-D fix / good signal
C_AMBER      = "#ffb300"   # 2-D fix / degraded
C_RED        = "#ff3d3d"   # no fix / invalid
C_CYAN       = "#00bcd4"   # accent / course vector
C_WHITE      = "#ffffff"

# Fix-type colours and labels (matches Betaflight 0/1/2)
FIX_META = {
    0: ("NO FIX",  C_RED),
    1: ("2D FIX",  C_AMBER),
    2: ("3D FIX",  C_GREEN),
}

# HDOP quality thresholds (HDOP is ×100 from C++, divided to raw in to_dict)
# raw HDOP: <1 = ideal, <2 = excellent, <5 = good, <10 = moderate, ≥10 = poor
def hdop_colour(hdop_raw: float) -> str:
    if hdop_raw <= 0 or hdop_raw >= 99.99:
        return C_RED
    if hdop_raw < 2.0:
        return C_GREEN
    if hdop_raw < 5.0:
        return C_AMBER
    return C_RED


class _CompassRose(tk.Canvas):
    """
    Minimal EFIS-style compass rose showing bearing-to-home.
    North-up fixed; home bearing needle rotates.
    """
    SIZE = 140   # canvas width = height

    def __init__(self, parent, **kwargs):
        super().__init__(
            parent,
            width=self.SIZE, height=self.SIZE,
            bg=C_BG, highlightthickness=0,
            **kwargs
        )
        self._bearing = 0
        self._valid   = False
        self._draw()

    # Public update ─────────────────────────────────────────────────────────
    def set_bearing(self, bearing_deg: float, valid: bool):
        self._bearing = bearing_deg
        self._valid   = valid
        self._draw()

    # Internal render ───────────────────────────────────────────────────────
    def _draw(self):
        self.delete("all")
        cx = cy = self.SIZE // 2
        r  = cx - 10

        # Outer ring
        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         outline=C_BORDER, width=1, fill=C_CARD)

        # Cardinal tick marks + labels
        cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
        for deg in range(0, 360, 10):
            rad    = math.radians(deg - 90)
            inner  = r - (8 if deg % 90 == 0 else 4)
            x0 = cx + inner * math.cos(rad)
            y0 = cy + inner * math.sin(rad)
            x1 = cx + r     * math.cos(rad)
            y1 = cy + r     * math.sin(rad)
            colour = C_VALUE if deg % 90 == 0 else C_LABEL
            self.create_line(x0, y0, x1, y1, fill=colour, width=1)

        for deg, label in cardinals.items():
            rad = math.radians(deg - 90)
            lx  = cx + (r - 18) * math.cos(rad)
            ly  = cy + (r - 18) * math.sin(rad)
            self.create_text(lx, ly, text=label,
                             fill=C_VALUE, font=("Courier", 8, "bold"))

        # Centre dot
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                         fill=C_CYAN, outline="")

        if not self._valid:
            self.create_text(cx, cy + 22, text="NO DATA",
                             fill=C_LABEL, font=("Courier", 7))
            return

        # Home bearing needle
        ned = math.radians(self._bearing - 90)
        tip_r   = r - 12
        base_r  = 12
        tip_x   = cx + tip_r  * math.cos(ned)
        tip_y   = cy + tip_r  * math.sin(ned)
        base_x  = cx - base_r * math.cos(ned)
        base_y  = cy - base_r * math.sin(ned)

        # Arrow head (filled triangle)
        perp = ned + math.pi / 2
        hw   = 5
        p1x  = tip_x
        p1y  = tip_y
        p2x  = base_x + hw * math.cos(perp)
        p2y  = base_y + hw * math.sin(perp)
        p3x  = base_x - hw * math.cos(perp)
        p3y  = base_y - hw * math.sin(perp)
        self.create_polygon(p1x, p1y, p2x, p2y, p3x, p3y,
                            fill=C_GREEN, outline="")

        # Tail line
        self.create_line(cx, cy, base_x, base_y,
                         fill=C_GREEN, width=1, dash=(3, 2))

        # Bearing label
        self.create_text(cx, self.SIZE - 10,
                         text=f"{self._bearing:+.0f}°",
                         fill=C_GREEN, font=("Courier", 8, "bold"))


class GPSWidget(tk.Frame):
    """
    Aviation-standard GPS telemetry panel for the Project R.A.D GCS.

    Call update_gps(data_dict) on every UI refresh tick.
    data_dict is the dict returned by DroneState.to_dict() — no conversion needed.
    """

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=C_BG, **kwargs)
        self._prev_heartbeat = None
        self._hb_flash       = False
        self._build()

    # ── Public API ────────────────────────────────────────────────────────────

    def update_gps(self, d: dict):
        """
        Refresh all GPS display elements.
        d — dict with gps_* keys (from DroneState.to_dict()).
        Safe to call even before C++ GPS integration is compiled:
        all keys are fetched with .get() and sensible defaults.
        """
        raw_valid  = d.get("gps_raw_valid",  False)
        comp_valid = d.get("gps_comp_valid", False)
        fix_type   = d.get("gps_fix_type",   0)
        num_sat    = d.get("gps_num_sat",    0)
        hdop_raw   = d.get("gps_hdop",       99.99)
        lat        = d.get("gps_latitude",   0.0)
        lon        = d.get("gps_longitude",  0.0)
        alt_m      = d.get("gps_altitude_m", 0.0)
        speed_kph  = d.get("gps_speed_kph",  0.0)
        course_deg = d.get("gps_course_deg", 0.0)
        dist_m     = d.get("gps_dist_home_m",  0)
        bearing_h  = d.get("gps_bearing_home", 0)
        heartbeat  = d.get("gps_heartbeat",    0)

        # ── Status bar ───────────────────────────────────────────────────────
        fix_label, fix_colour = FIX_META.get(fix_type, ("UNKNOWN", C_RED))
        self._fix_lbl.config(text=fix_label, fg=fix_colour)

        sat_colour = C_GREEN if num_sat >= 6 else (C_AMBER if num_sat >= 4 else C_RED)
        self._sat_lbl.config(text=f"SAT {num_sat:02d}", fg=sat_colour)

        hd_colour = hdop_colour(hdop_raw)
        hdop_text = f"HDOP {hdop_raw:.2f}" if hdop_raw < 99 else "HDOP ---"
        self._hdop_lbl.config(text=hdop_text, fg=hd_colour)

        # GPS heartbeat flash — toggles each new GPS frame from FC
        if heartbeat != self._prev_heartbeat and self._prev_heartbeat is not None:
            self._hb_flash = True
        self._prev_heartbeat = heartbeat
        hb_fg = C_GREEN if self._hb_flash else C_LABEL
        self._hb_lbl.config(fg=hb_fg)
        self._hb_flash = False   # reset until next toggle

        # ── Coordinates ──────────────────────────────────────────────────────
        if raw_valid and fix_type > 0:
            lat_str = self._fmt_dms(lat, is_lat=True)
            lon_str = self._fmt_dms(lon, is_lat=False)
            alt_str = f"{alt_m:.1f} m MSL"
            coord_fg = C_VALUE
        else:
            lat_str = lon_str = "---°--'--\"  -"
            alt_str = "--- m MSL"
            coord_fg = C_LABEL

        self._lat_val.config(text=lat_str, fg=coord_fg)
        self._lon_val.config(text=lon_str, fg=coord_fg)
        self._alt_val.config(text=alt_str, fg=coord_fg)

        # ── Speed & course ───────────────────────────────────────────────────
        if raw_valid:
            self._spd_val.config(text=f"{speed_kph:.1f}", fg=C_VALUE)
            self._crs_val.config(text=f"{course_deg:.1f}°", fg=C_VALUE)
        else:
            self._spd_val.config(text="---.-", fg=C_LABEL)
            self._crs_val.config(text="---°",  fg=C_LABEL)

        # ── Distance & bearing to home ───────────────────────────────────────
        if comp_valid:
            dist_str = f"{dist_m} m" if dist_m < 1000 else f"{dist_m/1000:.2f} km"
            self._dst_val.config(text=dist_str,         fg=C_VALUE)
            self._brg_val.config(text=f"{bearing_h:+.0f}°", fg=C_VALUE)
            self._compass.set_bearing(bearing_h, valid=True)
        else:
            self._dst_val.config(text="--- m",  fg=C_LABEL)
            self._brg_val.config(text="---°",   fg=C_LABEL)
            self._compass.set_bearing(0, valid=False)

    # ── Layout builder ────────────────────────────────────────────────────────

    def _build(self):
        PAD = 6

        # ── 1. Status bar ─────────────────────────────────────────────────────
        status = tk.Frame(self, bg=C_CARD, padx=PAD, pady=4)
        status.pack(fill="x", padx=PAD, pady=(PAD, 2))

        self._fix_lbl = tk.Label(status, text="NO FIX", bg=C_CARD,
                                 fg=C_RED, font=("Courier", 10, "bold"))
        self._fix_lbl.pack(side="left", padx=(0, 12))

        self._sat_lbl = tk.Label(status, text="SAT 00", bg=C_CARD,
                                 fg=C_RED, font=("Courier", 10, "bold"))
        self._sat_lbl.pack(side="left", padx=(0, 12))

        self._hdop_lbl = tk.Label(status, text="HDOP ---", bg=C_CARD,
                                  fg=C_LABEL, font=("Courier", 10))
        self._hdop_lbl.pack(side="left", padx=(0, 12))

        # GPS heartbeat indicator (♥ symbol)
        hb_frame = tk.Frame(status, bg=C_CARD)
        hb_frame.pack(side="right")
        tk.Label(hb_frame, text="GPS", bg=C_CARD,
                 fg=C_LABEL, font=("Courier", 8)).pack(side="left")
        self._hb_lbl = tk.Label(hb_frame, text=" ♥", bg=C_CARD,
                                 fg=C_LABEL, font=("Courier", 10, "bold"))
        self._hb_lbl.pack(side="left")

        # ── 2. Middle row: coordinates + compass ──────────────────────────────
        mid = tk.Frame(self, bg=C_BG)
        mid.pack(fill="x", padx=PAD, pady=2)

        # Coordinates card
        coord_card = tk.Frame(mid, bg=C_CARD, padx=PAD, pady=PAD)
        coord_card.pack(side="left", fill="both", expand=True, padx=(0, 4))

        self._lat_val = self._coord_row(coord_card, "LAT", row=0)
        self._lon_val = self._coord_row(coord_card, "LON", row=1)

        tk.Frame(coord_card, bg=C_BORDER, height=1).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=4)

        tk.Label(coord_card, text="ALTITUDE", bg=C_CARD,
                 fg=C_LABEL, font=("Courier", 8)).grid(
                     row=3, column=0, sticky="w")
        self._alt_val = tk.Label(coord_card, text="--- m MSL", bg=C_CARD,
                                  fg=C_LABEL, font=("Courier", 13, "bold"),
                                  anchor="e")
        self._alt_val.grid(row=3, column=1, sticky="e")

        # Compass rose card
        comp_card = tk.Frame(mid, bg=C_CARD, padx=PAD, pady=PAD)
        comp_card.pack(side="right", fill="y")

        tk.Label(comp_card, text="HOME BRG", bg=C_CARD,
                 fg=C_LABEL, font=("Courier", 7)).pack()
        self._compass = _CompassRose(comp_card)
        self._compass.pack()

        # ── 3. Bottom data strip ──────────────────────────────────────────────
        strip = tk.Frame(self, bg=C_CARD, padx=PAD, pady=6)
        strip.pack(fill="x", padx=PAD, pady=(2, PAD))

        metrics = [
            ("GND SPD", "km/h", "_spd"),
            ("COURSE",  "",     "_crs"),
            ("DIST HOME", "",   "_dst"),
            ("BRG HOME",  "",   "_brg"),
        ]
        for col, (label, unit, attr) in enumerate(metrics):
            cell = tk.Frame(strip, bg=C_CARD)
            cell.grid(row=0, column=col, padx=10)

            tk.Label(cell, text=label, bg=C_CARD,
                     fg=C_LABEL, font=("Courier", 7)).pack()

            val_lbl = tk.Label(cell, text="---", bg=C_CARD,
                               fg=C_LABEL, font=("Courier", 14, "bold"))
            val_lbl.pack()
            setattr(self, f"{attr}_val", val_lbl)

            if unit:
                tk.Label(cell, text=unit, bg=C_CARD,
                         fg=C_LABEL, font=("Courier", 7)).pack()

        strip.columnconfigure(list(range(len(metrics))), weight=1)

    # ── Helper: coordinate row ────────────────────────────────────────────────

    def _coord_row(self, parent, label: str, row: int) -> tk.Label:
        tk.Label(parent, text=label, bg=C_CARD,
                 fg=C_LABEL, font=("Courier", 8)).grid(
                     row=row, column=0, sticky="w", padx=(0, 8))
        val = tk.Label(parent, text='---°--\'--"  -', bg=C_CARD,
                       fg=C_LABEL, font=("Courier", 13, "bold"), anchor="e")
        val.grid(row=row, column=1, sticky="e")
        return val

    # ── Coordinate formatter — DD to DMS ─────────────────────────────────────

    @staticmethod
    def _fmt_dms(decimal_deg: float, is_lat: bool) -> str:
        """Convert decimal degrees to DMS string with hemisphere indicator."""
        if is_lat:
            hemi = "N" if decimal_deg >= 0 else "S"
        else:
            hemi = "E" if decimal_deg >= 0 else "W"

        dd = abs(decimal_deg)
        d  = int(dd)
        m  = int((dd - d) * 60)
        s  = (dd - d - m / 60) * 3600

        return f'{d:03d}°{m:02d}\'{s:05.2f}"  {hemi}'