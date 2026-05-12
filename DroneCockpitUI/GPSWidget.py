"""
GPSWidget.py

Aviation-style GPS navigation display widget for Project R.A.D GCS.

Displays data from MSP_RAW_GPS (106) and MSP_COMP_GPS (107) following
ICAO Annex 10 / DO-229E conventions:

  - Fix quality: NO FIX / 2D FIX / 3D FIX with colour coding
  - Satellite count
  - HDOP quality bar (colour-graded: green < 1.0, yellow < 2.0, red >= 2.0)
  - Latitude / Longitude in Degrees, Decimal-Minutes (DDM) — standard for
    aviation charts and the format used on VFR sectionals
  - Altitude MSL in feet (aviation standard) and metres
  - Ground speed in knots (kt) — standard aviation unit — and km/h
  - Ground track (COG) with a small compass-rose needle
  - Distance to home in metres / nautical miles
  - Bearing to home drawn as a pointer on a miniature compass arc
  - GPS heartbeat blink indicator — confirms fresh data is arriving
  - Position-usable gate indicator — warns when fix does not meet integrity
    thresholds (fixType>=2, numSat>=4, HDOP<5.0)

Usage:
    widget = GPSWidget(parent_frame)
    widget.grid(row=2, column=0, columnspan=2, sticky="ew", ...)
    widget.update_gps(ui_data)   # called every tick from DroneCockpitApp

ui_data keys consumed (all provided by DroneCockpitApp._update_loop):
    gps_fix_type          int    0/1/2
    gps_num_sat           int
    gps_latitude          float  decimal degrees
    gps_longitude         float  decimal degrees
    gps_altitude_m        float  metres MSL
    gps_ground_speed_cms  int    cm/s
    gps_ground_course     int    decidegrees (0-3599)
    gps_hdop              float  already divided by 100 (real HDOP value)
    gps_dist_to_home_m    float  metres
    gps_bearing_to_home   int    degrees (-180 to +180)
    gps_heartbeat         int    toggles 0/1
    gps_raw_valid         bool
    gps_comp_valid        bool
    gps_position_usable   bool
"""

import math
import tkinter as tk
from tkinter import ttk

# =============================================================================
# Colour palette — matches the dark-instrument aesthetic used elsewhere
# =============================================================================
BG          = "#1a1a2e"   # deep navy — widget background
BG_PANEL    = "#16213e"   # slightly darker panel sections
BG_CELL     = "#0f3460"   # individual data cells
BORDER      = "#e94560"   # accent border / active highlight (red-accent)
TEXT_HI     = "#eaeaea"   # primary text
TEXT_DIM    = "#8892a4"   # secondary / label text
TEXT_UNIT   = "#5a7a9a"   # unit suffix text

# Fix-type colours
COL_NO_FIX  = "#e94560"   # red
COL_2D      = "#f5a623"   # amber
COL_3D      = "#00d084"   # green

# HDOP quality colours
COL_HDOP_GOOD = "#00d084"
COL_HDOP_MED  = "#f5a623"
COL_HDOP_BAD  = "#e94560"

# Heartbeat colours
COL_HB_ON   = "#00d084"
COL_HB_OFF  = "#1e3050"

FONT_MONO   = ("Courier New", 10, "bold")
FONT_LABEL  = ("Courier New",  8)
FONT_VALUE  = ("Courier New", 11, "bold")
FONT_BIG    = ("Courier New", 13, "bold")
FONT_SMALL  = ("Courier New",  8)
FONT_TITLE  = ("Courier New",  9, "bold")


# =============================================================================
# Helpers
# =============================================================================

def _cms_to_kt(cms: float) -> float:
    """cm/s -> knots (1 kt = 51.4444 cm/s)"""
    return cms / 51.4444

def _cms_to_kmh(cms: float) -> float:
    return cms * 0.036

def _m_to_ft(m: float) -> float:
    return m * 3.28084

def _m_to_nm(m: float) -> float:
    """metres -> nautical miles"""
    return m / 1852.0

def _decimal_to_ddm(deg: float, is_lat: bool) -> str:
    """
    Convert decimal degrees to Degrees Decimal-Minutes string.
    e.g.  45.234567  ->  "45° 14.074' N"
    Standard format on VFR aviation charts.
    """
    hemi_pos = "N" if is_lat else "E"
    hemi_neg = "S" if is_lat else "W"
    hemi = hemi_pos if deg >= 0 else hemi_neg
    deg  = abs(deg)
    d    = int(deg)
    m    = (deg - d) * 60.0
    return f"{d:02d}\u00b0 {m:06.3f}' {hemi}"

def _hdop_colour(hdop: float) -> str:
    if hdop < 1.0:
        return COL_HDOP_GOOD
    if hdop < 2.0:
        return COL_HDOP_MED
    return COL_HDOP_BAD

def _fix_colour(fix_type: int) -> str:
    if fix_type == 2: return COL_3D
    if fix_type == 1: return COL_2D
    return COL_NO_FIX

def _fix_label(fix_type: int) -> str:
    return {0: "NO FIX", 1: "2D FIX", 2: "3D FIX"}.get(fix_type, "NO FIX")


# =============================================================================
# GPSWidget
# =============================================================================

class GPSWidget(tk.Frame):
    """
    Aviation-grade GPS display widget.

    Laid out as a two-row instrument panel:
      Row 0: Fix status bar (full width)
      Row 1: Left block  — coordinates + altitude + speed + track
             Right block — home vector (distance + bearing compass arc)
    """

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=BG, bd=1, relief="solid", **kwargs)
        self.configure(highlightbackground=BORDER, highlightthickness=1)

        self._prev_heartbeat = -1   # sentinel — first tick always blinks

        self._build_ui()

    # =========================================================================
    # UI construction
    # =========================================================================

    def _build_ui(self):
        # ----- Title bar -----------------------------------------------------
        title_row = tk.Frame(self, bg=BORDER)
        title_row.pack(fill="x", side="top")

        tk.Label(
            title_row,
            text="  GPS NAV  \u2014  NEO-M10",
            bg=BORDER, fg="#ffffff",
            font=FONT_TITLE, anchor="w",
        ).pack(side="left", padx=6, pady=2)

        # Heartbeat indicator (right side of title bar)
        self._hb_canvas = tk.Canvas(
            title_row, width=12, height=12,
            bg=BORDER, highlightthickness=0
        )
        self._hb_canvas.pack(side="right", padx=8, pady=2)
        self._hb_dot = self._hb_canvas.create_oval(
            1, 1, 11, 11, fill=COL_HB_OFF, outline=""
        )

        # ----- Fix status row ------------------------------------------------
        status_row = tk.Frame(self, bg=BG_PANEL)
        status_row.pack(fill="x", side="top", pady=(0, 1))

        # Fix type label
        self._fix_label = tk.Label(
            status_row,
            text="NO FIX",
            bg=BG_PANEL, fg=COL_NO_FIX,
            font=("Courier New", 11, "bold"),
            width=7, anchor="center",
        )
        self._fix_label.pack(side="left", padx=(8, 0), pady=3)

        tk.Label(status_row, text="|", bg=BG_PANEL,
                 fg=TEXT_DIM, font=FONT_LABEL).pack(side="left", padx=4)

        # Satellite count
        tk.Label(status_row, text="SAT", bg=BG_PANEL,
                 fg=TEXT_DIM, font=FONT_LABEL).pack(side="left")
        self._sat_label = tk.Label(
            status_row, text="--",
            bg=BG_PANEL, fg=TEXT_HI,
            font=FONT_MONO, width=3, anchor="w",
        )
        self._sat_label.pack(side="left", padx=(2, 8))

        # HDOP
        tk.Label(status_row, text="HDOP", bg=BG_PANEL,
                 fg=TEXT_DIM, font=FONT_LABEL).pack(side="left")
        self._hdop_label = tk.Label(
            status_row, text="-.--",
            bg=BG_PANEL, fg=TEXT_HI,
            font=FONT_MONO, width=5, anchor="w",
        )
        self._hdop_label.pack(side="left", padx=(2, 4))

        # HDOP quality bar (10 segments × 6 px = 60 px)
        self._hdop_bar = tk.Canvas(
            status_row, width=62, height=10,
            bg=BG_PANEL, highlightthickness=0
        )
        self._hdop_bar.pack(side="left", padx=(0, 8))
        self._hdop_segs = []
        for i in range(10):
            seg = self._hdop_bar.create_rectangle(
                i * 6 + 1, 1, i * 6 + 5, 9,
                fill="#1e3050", outline=""
            )
            self._hdop_segs.append(seg)

        # Position usable warning
        self._usable_label = tk.Label(
            status_row, text="POS UNUSABLE",
            bg=BG_PANEL, fg=COL_NO_FIX,
            font=("Courier New", 8, "bold"),
        )
        self._usable_label.pack(side="right", padx=8)

        # ----- Main data area ------------------------------------------------
        data_row = tk.Frame(self, bg=BG)
        data_row.pack(fill="both", expand=True, side="top",
                      padx=6, pady=(2, 6))

        # Left block: coordinates / altitude / speed / track
        left = tk.Frame(data_row, bg=BG)
        left.pack(side="left", fill="both", expand=True)

        self._coord_frame = self._make_section(left, "POSITION")
        self._coord_frame.pack(fill="x", pady=(0, 4))

        self._lat_val  = self._add_data_row(self._coord_frame, "LAT", "\u2014")
        self._lon_val  = self._add_data_row(self._coord_frame, "LON", "\u2014")
        self._alt_val  = self._add_data_row(self._coord_frame, "ALT", "\u2014")

        self._nav_frame = self._make_section(left, "NAVIGATION")
        self._nav_frame.pack(fill="x")

        self._spd_val  = self._add_data_row(self._nav_frame, "GS",  "\u2014")
        self._trk_val  = self._add_data_row(self._nav_frame, "TRK", "\u2014")

        # Right block: home vector + bearing compass
        right = tk.Frame(data_row, bg=BG)
        right.pack(side="right", fill="y", padx=(8, 0))

        self._home_frame = self._make_section(right, "HOME VECTOR")
        self._home_frame.pack(fill="x", pady=(0, 4))

        self._dist_val = self._add_data_row(self._home_frame, "DIST", "\u2014")
        self._brg_val  = self._add_data_row(self._home_frame, "BRG",  "\u2014")

        # Miniature bearing compass arc
        self._compass = tk.Canvas(
            right, width=90, height=90,
            bg=BG, highlightthickness=0
        )
        self._compass.pack(pady=(2, 0))
        self._draw_compass_rose()

    # =========================================================================
    # UI helpers
    # =========================================================================

    def _make_section(self, parent, title: str) -> tk.Frame:
        frame = tk.Frame(parent, bg=BG_CELL,
                         highlightbackground="#1e3060",
                         highlightthickness=1)
        tk.Label(
            frame, text=f" {title} ",
            bg=BG_CELL, fg=TEXT_DIM,
            font=FONT_SMALL, anchor="w",
        ).pack(fill="x", side="top", padx=2, pady=(2, 0))
        return frame

    def _add_data_row(self, parent: tk.Frame, label: str, initial: str):
        """
        Add a label + value row inside a section frame.
        Returns the value Label so the caller can update it.
        """
        row = tk.Frame(parent, bg=BG_CELL)
        row.pack(fill="x", padx=4, pady=1)

        tk.Label(
            row, text=f"{label:<4}",
            bg=BG_CELL, fg=TEXT_DIM,
            font=FONT_LABEL, anchor="w", width=4,
        ).pack(side="left")

        val = tk.Label(
            row, text=initial,
            bg=BG_CELL, fg=TEXT_HI,
            font=FONT_MONO, anchor="w",
        )
        val.pack(side="left", padx=(4, 0))
        return val

    # =========================================================================
    # Compass rose
    # =========================================================================

    def _draw_compass_rose(self):
        """Draw static compass markings; bearing needle is updated each tick."""
        cx, cy, r = 45, 45, 38
        c = self._compass

        # Outer ring
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      outline=TEXT_DIM, width=1)

        # Cardinal tick marks + labels
        cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
        for deg in range(0, 360, 30):
            rad  = math.radians(deg - 90)
            inner = r - (6 if deg % 90 == 0 else 3)
            x1 = cx + inner * math.cos(rad)
            y1 = cy + inner * math.sin(rad)
            x2 = cx + r     * math.cos(rad)
            y2 = cy + r     * math.sin(rad)
            c.create_line(x1, y1, x2, y2,
                          fill=TEXT_DIM if deg % 90 != 0 else TEXT_HI,
                          width=1)
            if deg in cardinals:
                lx = cx + (r - 14) * math.cos(rad)
                ly = cy + (r - 14) * math.sin(rad)
                c.create_text(lx, ly, text=cardinals[deg],
                              fill=TEXT_HI, font=FONT_SMALL)

        # Centre dot
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                      fill=TEXT_DIM, outline="")

        # Placeholder needle (home bearing pointer) — updated each tick
        self._needle = c.create_line(
            cx, cy,
            cx, cy - r + 8,
            fill=BG, width=2, arrow="last",
        )

        # Track needle (COG) — updated each tick
        self._track_needle = c.create_line(
            cx, cy,
            cx, cy - r + 14,
            fill=BG, width=2,
        )

    def _update_compass(self, bearing_deg: int, track_deg: float,
                        comp_valid: bool, raw_valid: bool):
        """
        Update the bearing needle (home direction) and track needle (COG).

        bearing_deg : bearing to home (-180 to +180), degrees
        track_deg   : ground track (COG), 0-360 degrees
        """
        cx, cy, r = 45, 45, 38
        c = self._compass

        # --- Home bearing pointer (red) ---
        if comp_valid:
            rad = math.radians(bearing_deg - 90)
            ex  = cx + (r - 8) * math.cos(rad)
            ey  = cy + (r - 8) * math.sin(rad)
            c.coords(self._needle, cx, cy, ex, ey)
            c.itemconfig(self._needle, fill=COL_NO_FIX)
        else:
            c.coords(self._needle, cx, cy, cx, cy)
            c.itemconfig(self._needle, fill=BG)

        # --- Ground track pointer (green) ---
        if raw_valid:
            rad = math.radians(track_deg - 90)
            ex  = cx + (r - 14) * math.cos(rad)
            ey  = cy + (r - 14) * math.sin(rad)
            c.coords(self._track_needle, cx, cy, ex, ey)
            c.itemconfig(self._track_needle, fill=COL_3D)
        else:
            c.coords(self._track_needle, cx, cy, cx, cy)
            c.itemconfig(self._track_needle, fill=BG)

    # =========================================================================
    # HDOP quality bar
    # =========================================================================

    def _update_hdop_bar(self, hdop: float):
        """
        Fill the HDOP bar proportionally.
        Bar represents HDOP 0.0 - 5.0 (= 10 segments of 0.5 each).
        Colour: green (good) -> amber -> red (poor).
        """
        # Clamp to 0-5 range for display; 9999 (unknown) shown as empty
        if hdop >= 99:
            filled = 0
            colour = COL_HDOP_BAD
        else:
            clamped = min(hdop, 5.0)
            filled  = round(clamped / 0.5)   # 0-10 segments
            colour  = _hdop_colour(hdop)

        for i, seg in enumerate(self._hdop_segs):
            self._hdop_bar.itemconfig(
                seg,
                fill=colour if i < filled else "#1e3050"
            )

    # =========================================================================
    # Public update method — called every tick from DroneCockpitApp
    # =========================================================================

    def update_gps(self, d: dict):
        """
        Refresh all GPS display elements from the ui_data dict.
        Designed to be called at 50 Hz; all operations are O(1).
        """
        raw_valid  = d.get("gps_raw_valid",       False)
        comp_valid = d.get("gps_comp_valid",       False)
        pos_usable = d.get("gps_position_usable",  False)
        fix_type   = d.get("gps_fix_type",         0)
        num_sat    = d.get("gps_num_sat",          0)
        hdop       = d.get("gps_hdop",             99.0)   # real HDOP value
        lat        = d.get("gps_latitude",         0.0)
        lon        = d.get("gps_longitude",        0.0)
        alt_m      = d.get("gps_altitude_m",       0.0)
        spd_cms    = d.get("gps_ground_speed_cms", 0)
        course     = d.get("gps_ground_course",    0)      # decidegrees
        dist_m     = d.get("gps_dist_to_home_m",   0.0)
        bearing    = d.get("gps_bearing_to_home",  0)
        heartbeat  = d.get("gps_heartbeat",        0)

        # -- Heartbeat indicator ----------------------------------------------
        if heartbeat != self._prev_heartbeat:
            self._hb_canvas.itemconfig(self._hb_dot, fill=COL_HB_ON)
            self._prev_heartbeat = heartbeat
        else:
            self._hb_canvas.itemconfig(self._hb_dot, fill=COL_HB_OFF)

        # -- Fix status -------------------------------------------------------
        col = _fix_colour(fix_type)
        self._fix_label.config(text=_fix_label(fix_type), fg=col)

        # -- Satellite count --------------------------------------------------
        self._sat_label.config(
            text=str(num_sat) if raw_valid else "--",
            fg=TEXT_HI if num_sat >= 4 else COL_2D,
        )

        # -- HDOP -------------------------------------------------------------
        if raw_valid and hdop < 99:
            self._hdop_label.config(
                text=f"{hdop:.2f}",
                fg=_hdop_colour(hdop),
            )
            self._update_hdop_bar(hdop)
        else:
            self._hdop_label.config(text="-.--", fg=TEXT_DIM)
            self._update_hdop_bar(99)

        # -- Position usable gate ---------------------------------------------
        if pos_usable:
            self._usable_label.config(text="POS OK", fg=COL_3D)
        elif raw_valid:
            self._usable_label.config(text="POS DEGRADED", fg=COL_2D)
        else:
            self._usable_label.config(text="POS UNUSABLE", fg=COL_NO_FIX)

        # -- Coordinates ------------------------------------------------------
        if pos_usable:
            self._lat_val.config(
                text=_decimal_to_ddm(lat, is_lat=True),
                fg=TEXT_HI,
            )
            self._lon_val.config(
                text=_decimal_to_ddm(lon, is_lat=False),
                fg=TEXT_HI,
            )
        else:
            self._lat_val.config(text="\u2014 \u00b0 \u2014 -  -", fg=TEXT_DIM)
            self._lon_val.config(text="\u2014 \u00b0 \u2014 -  -", fg=TEXT_DIM)

        # -- Altitude (feet primary, metres secondary) ------------------------
        # Aviation standard: altitude in feet MSL.
        if raw_valid and fix_type >= 1:
            alt_ft = _m_to_ft(alt_m)
            self._alt_val.config(
                text=f"{alt_ft:,.0f} ft  ({alt_m:.0f} m)",
                fg=TEXT_HI,
            )
        else:
            self._alt_val.config(text="\u2014  ft", fg=TEXT_DIM)

        # -- Ground speed (knots primary, km/h secondary) --------------------
        # Knots is the standard aviation unit for airspeed and groundspeed.
        if raw_valid:
            kt  = _cms_to_kt(spd_cms)
            kmh = _cms_to_kmh(spd_cms)
            self._spd_val.config(
                text=f"{kt:5.1f} kt  ({kmh:.0f} km/h)",
                fg=TEXT_HI,
            )
        else:
            self._spd_val.config(text="\u2014  kt", fg=TEXT_DIM)

        # -- Ground track (COG) -----------------------------------------------
        if raw_valid:
            trk_deg = course * 0.1   # decidegrees -> degrees
            self._trk_val.config(
                text=f"{trk_deg:05.1f}\u00b0 T",
                fg=TEXT_HI,
            )
        else:
            self._trk_val.config(text="\u2014\u00b0  T", fg=TEXT_DIM)
            trk_deg = 0.0

        # -- Distance to home -------------------------------------------------
        if comp_valid:
            nm = _m_to_nm(dist_m)
            self._dist_val.config(
                text=f"{dist_m:.0f} m  ({nm:.3f} NM)",
                fg=TEXT_HI,
            )
        else:
            self._dist_val.config(text="\u2014  m", fg=TEXT_DIM)

        # -- Bearing to home --------------------------------------------------
        if comp_valid:
            # Convert -180..+180 to 0..360 for display
            brg360 = bearing % 360
            self._brg_val.config(
                text=f"{brg360:03d}\u00b0 M",
                fg=TEXT_HI,
            )
        else:
            self._brg_val.config(text="\u2014\u00b0  M", fg=TEXT_DIM)

        # -- Compass rose needles ---------------------------------------------
        self._update_compass(
            bearing_deg=bearing,
            track_deg=trk_deg if raw_valid else 0.0,
            comp_valid=comp_valid,
            raw_valid=raw_valid,
        )