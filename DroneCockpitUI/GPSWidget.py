"""
GPSWidget.py  —  Aviation-grade GPS Navigation Widget
=====================================================

Layout philosophy
-----------------
  • The MAP is the primary instrument.  Navigation data (fix status, position,
    altitude, speed, track, home bearing/distance) is overlaid on top of the
    map in semi-transparent HUD panels — the pilot never leaves the map view.

  • A "SAT" toggle button (top-right of the map toolbar) opens/closes a
    satellite-signal panel that slides in alongside the map.  The map shrinks
    to accommodate it; closing it restores full width.

  • When the widget is too narrow or too short to render the map usefully
    (below MIN_MAP_W × MIN_MAP_H pixels) the map is hidden and a compact
    full-detail navigation panel is shown instead.  The SAT panel remains
    accessible in that mode via a separate "SAT" button in the header row.

Aviation HUD overlay layout (map mode)
---------------------------------------
  ┌─[FIX badge] [SAT ##] [HDOP #.##] [●]────────────[+][−][⊙ Ctr][SAT]─┐
  │                                                                       │
  │   ┌─ top-left ────────────────┐                                       │
  │   │  LAT  44.770029 N         │                                       │
  │   │  LON  17.210185 E         │                                       │
  │   │  ALT  6 ft  (2 m)        │                                       │
  │   └───────────────────────────┘                                       │
  │                                    ┌─ bottom-right ─────────────────┐ │
  │                                    │ GS  1.2 kt  (2.3 km/h)        │ │
  │                                    │ TRK 323.2°  [compass rose]     │ │
  │                                    │ DIST 0 m  BRG 000° [rose]     │ │
  │                                    └────────────────────────────────┘ │
  └───────────────────────────────────────────────────────────────────────┘

All update_gps(ui_data) keys are identical to the original widget.
"""

import tkinter as tk
from tkinter import ttk
import math
import threading
import queue
import urllib.request
import base64


# ── Thresholds ─────────────────────────────────────────────────────────────
MIN_MAP_W = 320   # px — below this width the map is hidden
MIN_MAP_H = 220   # px — below this height the map is hidden

# ── Unit conversion ─────────────────────────────────────────────────────────
_CMS_TO_KT  = 0.019438
_CMS_TO_KMH = 0.036
_M_TO_FT    = 3.28084

# ── Aviation colour palette ─────────────────────────────────────────────────
_C = {
    # structure
    "bg":           "#0d1117",
    "frame_bg":     "#141920",
    "border":       "#1e2730",
    "hud_bg":       "#0a0e14cc",   # semi-transparent (used as stipple workaround)
    # text
    "text":         "#e8edf4",
    "label":        "#6b7885",
    "unit":         "#8a9aaa",
    "dim":          "#3a4555",
    # status
    "green":        "#00e676",
    "amber":        "#ffb300",
    "red":          "#ff3d3d",
    "cyan":         "#00e5ff",
    "magenta":      "#ff4081",
    # instruments
    "compass_bg":   "#080c10",
    "compass_rim":  "#2a3545",
    "track_needle": "#00e676",
    "home_arrow":   "#00e5ff",
    "hb_on":        "#00e676",
    "hb_off":       "#1e2730",
    # map
    "map_bg":       "#1a2030",
    "map_btn":      "#1e2a38",
    "map_marker":   "#ffb300",
    "map_home":     "#00e5ff",
    # satellite
    "sv_row_even":  "#0d1117",
    "sv_row_odd":   "#111820",
    "sv_used":      "#00e676",
    "sv_locked":    "#ffb300",
    "sv_nodata":    "#2a3545",
}

# Fonts — Courier New keeps the avionics mono feel; sizes bumped for legibility
_FF  = "Courier New"
_FL  = (_FF,  8, "bold")   # micro label
_FLM = (_FF,  9, "bold")   # label medium
_FV  = (_FF, 14, "bold")   # primary value
_FVS = (_FF, 11, "bold")   # secondary value
_FU  = (_FF,  9)           # unit
_FHD = (_FF, 11, "bold")   # HUD overlay data
_FHL = (_FF,  7, "bold")   # HUD overlay micro-label

# ── Quality tables (Betaflight 4.5.3) ───────────────────────────────────────
_QUALITY_LABEL = [
    "idle", "searching", "acquired", "unusable",
    "code lock", "fully locked", "fully locked", "fully locked",
]
_QUALITY_COLOR = [
    "#2a3545", "#2a3545", "#ffb300", "#ff3d3d",
    "#ffb300", "#00e676", "#00e676", "#00e676",
]


# =============================================================================
# SVInfoEntry normaliser  (unchanged from original)
# =============================================================================

def _normalize_sv_list(raw_list) -> list:
    out = []
    for item in raw_list:
        try:
            if isinstance(item, dict):
                gnss_id = item.get("gnss_id") or item.get("gnss_name", "?")
                sv_id   = item.get("sv_id")
                if sv_id is None:
                    sv_id = item.get("svid", 0)
                status  = item.get("status") or item.get("status_str", "idle")
                out.append({
                    "gnss_id": gnss_id,
                    "sv_id":   sv_id,
                    "cno":     int(item.get("cno", 0)),
                    "used":    bool(item.get("used", False)),
                    "quality": int(item.get("quality", 0)),
                    "elev":    int(item.get("elev", 0)),
                    "azim":    int(item.get("azim", 0)),
                    "status":  status,
                })
            else:
                out.append({
                    "gnss_id": item.gnss_name,
                    "sv_id":   item.svid,
                    "cno":     int(item.cno),
                    "used":    bool(item.used),
                    "quality": int(item.quality),
                    "elev":    int(item.elev),
                    "azim":    int(item.azim),
                    "status":  item.status_str,
                })
        except Exception:
            continue
    return out


# =============================================================================
# Satellite panel  (scrollable canvas — standalone Frame, no Notebook)
# =============================================================================

class _SatPanel(tk.Frame):
    """
    Compact satellite signal-strength panel.
    Designed to live to the right of the map or as a standalone popup.
    Width is fixed at SAT_PANEL_W pixels.
    """
    SAT_PANEL_W = 310
    _ROW_H      = 22
    _CNO_MAX    = 50
    _BAR_MAX    = 72

    _GNSS_COLOR = {
        "GPS":     "#00e676",
        "SBAS":    "#ffb300",
        "Galileo": "#cc88ff",
        "BeiDou":  "#00e5ff",
        "IMES":    "#888888",
        "QZSS":    "#ff8844",
        "GLONASS": "#dd4444",
        "Unknown": "#2a3545",
    }
    _GNSS_ALIASES = {
        "beidou": "BeiDou", "bei dou": "BeiDou",
        "galileo": "Galileo", "glonass": "GLONASS",
        "gps": "GPS", "sbas": "SBAS", "qzss": "QZSS", "imes": "IMES",
    }

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["frame_bg"],
                         width=self.SAT_PANEL_W, **kwargs)
        self.pack_propagate(False)
        self._sv_data = []
        self._build()

    def _build(self):
        # Header
        hdr = tk.Frame(self, bg=_C["border"], height=1)
        hdr.pack(fill="x")

        title_row = tk.Frame(self, bg=_C["frame_bg"])
        title_row.pack(fill="x", padx=6, pady=(4, 2))
        tk.Label(title_row, text="SATELLITE CONSTELLATION",
                 fg=_C["label"], bg=_C["frame_bg"],
                 font=(_FF, 8, "bold")).pack(side="left")

        # Column header
        col_hdr = tk.Canvas(self, bg=_C["frame_bg"], height=16,
                            highlightthickness=0)
        col_hdr.pack(fill="x", padx=4)
        self._col_hdr = col_hdr
        self._draw_col_header()

        sep = tk.Frame(self, bg=_C["border"], height=1)
        sep.pack(fill="x")

        # Scrollable body
        body = tk.Frame(self, bg=_C["bg"])
        body.pack(fill="both", expand=True)
        self._cv = tk.Canvas(body, bg=_C["bg"], highlightthickness=0,
                             width=self.SAT_PANEL_W)
        sb = ttk.Scrollbar(body, orient="vertical", command=self._cv.yview)
        self._cv.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._cv.pack(side="left", fill="both", expand=True)
        self._cv.bind("<Configure>",  lambda _: self._redraw())
        self._cv.bind("<MouseWheel>", self._scroll)
        self._cv.bind("<Button-4>",   lambda _: self._cv.yview_scroll(-1, "units"))
        self._cv.bind("<Button-5>",   lambda _: self._cv.yview_scroll( 1, "units"))

    def _cols(self):
        return [("GNSS", 52), ("SV", 26), ("Signal", self._BAR_MAX + 6),
                ("dBHz", 32), ("Status", 60), ("Qual.", 58)]

    def _draw_col_header(self):
        cv = self._col_hdr
        cv.delete("all")
        x = 4
        for lbl, w in self._cols():
            cv.create_text(x, 8, text=lbl, anchor="w",
                           fill=_C["label"], font=(_FF, 7, "bold"))
            x += w

    def _scroll(self, event):
        self._cv.yview_scroll(int(-event.delta / 120), "units")

    def update_satellites(self, sv_list: list):
        self._sv_data = sv_list or []
        self._redraw()

    def _redraw(self):
        self._cv.delete("all")
        self._draw_col_header()
        w = max(self._cv.winfo_width(), self.SAT_PANEL_W - 14)

        if not self._sv_data:
            self._cv.create_text(w // 2, 40,
                text="No satellite data", fill=_C["dim"],
                font=(_FF, 9), justify="center")
            self._cv.configure(scrollregion=(0, 0, w, 80))
            return

        rows = sorted(self._sv_data, key=lambda s: (
            0 if (s.get("used") and int(s.get("cno", 0)) > 0) else
            1 if int(s.get("cno", 0)) > 0 else 2,
            -int(s.get("cno", 0))
        ))

        total_h = len(rows) * self._ROW_H + 2
        self._cv.configure(scrollregion=(0, 0, w, total_h))
        cols = self._cols()

        for i, sv in enumerate(rows):
            y      = i * self._ROW_H
            row_bg = _C["sv_row_odd"] if i % 2 else _C["sv_row_even"]
            self._cv.create_rectangle(0, y, w, y + self._ROW_H,
                                      fill=row_bg, outline="")
            x  = 4
            cy = y + self._ROW_H // 2
            cno     = int(sv.get("cno", 0))
            quality = min(int(sv.get("quality", 0)), 7)
            used    = bool(sv.get("used", False)) and cno > 0

            # GNSS
            gnss_raw = str(sv.get("gnss_id", "?"))
            gnss_key = self._GNSS_ALIASES.get(gnss_raw.lower(), gnss_raw)
            gcol     = self._GNSS_COLOR.get(gnss_key, self._GNSS_COLOR["Unknown"])
            self._cv.create_text(x, cy, text=gnss_key, anchor="w",
                                 fill=gcol, font=(_FF, 8, "bold"))
            x += cols[0][1]

            # SV
            self._cv.create_text(x, cy, text=str(sv.get("sv_id", "--")),
                                 anchor="w", fill=_C["text"], font=(_FF, 8))
            x += cols[1][1]

            # Bar
            bar_w  = int(min(cno / self._CNO_MAX, 1.0) * self._BAR_MAX)
            bar_col = (_C["sv_used"] if used else
                       _C["sv_locked"] if cno > 0 else _C["sv_nodata"])
            track_h = 5
            by      = cy - track_h // 2
            self._cv.create_rectangle(x, by, x + self._BAR_MAX, by + track_h,
                                      fill=_C["border"], outline="")
            if bar_w > 0:
                self._cv.create_rectangle(x, by, x + bar_w, by + track_h,
                                          fill=bar_col, outline="")
            x += cols[2][1]

            # dBHz
            self._cv.create_text(
                x, cy,
                text=f"{cno:2d}" if cno > 0 else "--",
                anchor="w",
                fill=_C["text"] if cno > 0 else _C["dim"],
                font=(_FF, 8))
            x += cols[3][1]

            # Status badge
            if used:
                b_txt, b_col = "USED", _C["sv_used"]
            elif cno > 0 and quality >= 4:
                b_txt, b_col = "unused", _C["sv_locked"]
            elif cno > 0:
                b_txt, b_col = "unused", _C["dim"]
            elif quality in (1, 2):
                b_txt, b_col = "search", _C["dim"]
            else:
                b_txt, b_col = "idle", _C["dim"]
            self._cv.create_text(x, cy, text=b_txt, anchor="w",
                                 fill=b_col, font=(_FF, 8, "bold"))
            x += cols[4][1]

            # Quality
            if cno == 0:
                q_str = "search" if quality in (1, 2) else "idle"
                q_col = _C["dim"]
            else:
                q_str = _QUALITY_LABEL[quality]
                q_col = _QUALITY_COLOR[quality]
            self._cv.create_text(x, cy, text=q_str, anchor="w",
                                 fill=q_col, font=(_FF, 7))


# =============================================================================
# Map canvas  (OSM tiles + HUD overlay)
# =============================================================================

class _MapCanvas(tk.Frame):
    """
    OSM tile map with aviation HUD overlay.
    Nav data panels are drawn directly on the canvas over the tiles so the
    pilot always sees both the map and the flight data simultaneously.
    """
    TILE_SIZE = 256
    _OSM_URL  = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    _UA       = "DroneCockpitGCS/1.0 (github educational project)"

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._zoom        = 15
        self._center_lat  = 44.77
        self._center_lon  = 17.21
        self._drone_lat   = None
        self._drone_lon   = None
        self._fix_valid   = False
        self._auto_center = True

        self._tile_img  = {}
        self._tile_queue = queue.Queue()
        self._tile_pend  = set()
        self._pan_last   = None

        # Nav data cache for HUD
        self._hud = {}

        self._build()
        self._poll_tiles()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build(self):
        # Toolbar
        bar = tk.Frame(self, bg=_C["frame_bg"])
        bar.pack(side="top", fill="x")

        btn_kw = dict(
            bg=_C["map_btn"], fg=_C["text"],
            relief="flat", padx=5, pady=2,
            font=(_FF, 9, "bold"),
            activebackground="#2a3545", activeforeground=_C["text"],
            cursor="hand2",
        )
        tk.Button(bar, text="＋", command=self._zoom_in,               **btn_kw).pack(side="left", padx=(2, 1))
        tk.Button(bar, text="－", command=self._zoom_out,              **btn_kw).pack(side="left", padx=(0, 1))
        tk.Button(bar, text="⊙ CTR", command=self._center_on_drone,   **btn_kw).pack(side="left", padx=(0, 2))

        self._zoom_lbl = tk.Label(bar, text=f"z{self._zoom}",
                                  fg=_C["label"], bg=_C["frame_bg"],
                                  font=(_FF, 8))
        self._zoom_lbl.pack(side="left", padx=4)

        # Map canvas
        self._cv = tk.Canvas(self, bg=_C["map_bg"],
                             highlightthickness=0, cursor="fleur")
        self._cv.pack(fill="both", expand=True)

        self._cv.bind("<Configure>",       self._on_resize)
        self._cv.bind("<ButtonPress-1>",   self._on_pan_start)
        self._cv.bind("<B1-Motion>",       self._on_pan)
        self._cv.bind("<ButtonRelease-1>", self._on_pan_end)
        self._cv.bind("<MouseWheel>",      self._on_scroll)
        self._cv.bind("<Button-4>",        lambda e: self._zoom_in())
        self._cv.bind("<Button-5>",        lambda e: self._zoom_out())

    # ── Tile maths ────────────────────────────────────────────────────────────

    @staticmethod
    def _deg2tile_f(lat, lon, zoom):
        n   = 2 ** zoom
        xf  = (lon + 180.0) / 360.0 * n
        lr  = math.radians(lat)
        yf  = (1.0 - math.asinh(math.tan(lr)) / math.pi) / 2.0 * n
        return xf, yf

    def _center_tile_f(self):
        return self._deg2tile_f(self._center_lat, self._center_lon, self._zoom)

    def _latlon_to_canvas(self, lat, lon):
        w  = self._cv.winfo_width()  or 400
        h  = self._cv.winfo_height() or 300
        cx, cy = self._center_tile_f()
        xf, yf = self._deg2tile_f(lat, lon, self._zoom)
        return (w / 2 + (xf - cx) * self.TILE_SIZE,
                h / 2 + (yf - cy) * self.TILE_SIZE)

    # ── Tile fetching ─────────────────────────────────────────────────────────

    def _request_tile(self, z, x, y):
        key = (z, x, y)
        if key in self._tile_img or key in self._tile_pend:
            return
        self._tile_pend.add(key)
        threading.Thread(target=self._fetch_tile, args=(z, x, y), daemon=True).start()

    def _fetch_tile(self, z, x, y):
        key = (z, x, y)
        url = self._OSM_URL.format(z=z, x=x, y=y)
        try:
            req  = urllib.request.Request(url, headers={"User-Agent": self._UA})
            data = urllib.request.urlopen(req, timeout=8).read()
            self._tile_queue.put((key, base64.b64encode(data).decode()))
        except Exception:
            self._tile_queue.put((key, None))
        finally:
            self._tile_pend.discard(key)

    def _poll_tiles(self):
        changed = False
        try:
            while True:
                key, b64 = self._tile_queue.get_nowait()
                if b64:
                    self._tile_img[key] = tk.PhotoImage(data=b64)
                    changed = True
        except queue.Empty:
            pass
        if changed:
            self._redraw()
        self.after(250, self._poll_tiles)

    # ── Redraw: tiles + HUD overlay ───────────────────────────────────────────

    def _redraw(self):
        if not self.winfo_ismapped():
            return
        w = self._cv.winfo_width()
        h = self._cv.winfo_height()
        if w < 4 or h < 4:
            return

        self._cv.delete("all")
        self._draw_tiles(w, h)
        self._draw_marker(w, h)
        self._draw_hud(w, h)

    def _draw_tiles(self, w, h):
        n          = 2 ** self._zoom
        cx_f, cy_f = self._center_tile_f()
        cx_ti      = int(cx_f)
        cy_ti      = int(cy_f)
        tiles_x    = w // self.TILE_SIZE + 3
        tiles_y    = h // self.TILE_SIZE + 3

        for dx in range(-tiles_x // 2 - 1, tiles_x // 2 + 2):
            for dy in range(-tiles_y // 2 - 1, tiles_y // 2 + 2):
                tx = (cx_ti + dx) % n
                ty = cy_ti + dy
                if ty < 0 or ty >= n:
                    continue
                px  = w / 2 + (cx_ti + dx - cx_f) * self.TILE_SIZE
                py  = h / 2 + (cy_ti + dy - cy_f) * self.TILE_SIZE
                key = (self._zoom, tx, ty)
                if key in self._tile_img:
                    self._cv.create_image(px, py, anchor="nw",
                                          image=self._tile_img[key])
                else:
                    self._cv.create_rectangle(
                        px, py, px + self.TILE_SIZE, py + self.TILE_SIZE,
                        fill="#1a2030", outline="#1e2730")
                    self._request_tile(self._zoom, tx, ty)

    def _draw_marker(self, w, h):
        if not (self._fix_valid and self._drone_lat is not None):
            # No-fix overlay
            self._cv.create_text(w // 2, h // 2,
                text="NO GPS FIX", fill=_C["red"],
                font=(_FF, 13, "bold"))
            return
        mx, my = self._latlon_to_canvas(self._drone_lat, self._drone_lon)
        r = 9
        # Accuracy ring
        self._cv.create_oval(mx - r*2, my - r*2, mx + r*2, my + r*2,
                             outline=_C["green"], width=1, dash=(3, 4))
        # Position dot
        self._cv.create_oval(mx - r, my - r, mx + r, my + r,
                             fill=_C["map_marker"], outline="#ffffff", width=1.5)
        # Track vector (uses hud track angle)
        trk = self._hud.get("trk", 0)
        rad = math.radians(trk - 90)
        vlen = 24
        self._cv.create_line(mx, my,
                             mx + math.cos(rad) * vlen,
                             my + math.sin(rad) * vlen,
                             fill=_C["green"], width=2,
                             arrow="last", arrowshape=(7, 9, 3))

    # ── HUD overlay panels ────────────────────────────────────────────────────

    def _hud_box(self, x, y, w_px, h_px, anchor="nw"):
        """Draw a semi-transparent HUD box; return (x0, y0) top-left."""
        # Tkinter canvas has no real transparency; use a dark stipple rectangle
        if anchor == "se":
            x0, y0 = x - w_px, y - h_px
        elif anchor == "ne":
            x0, y0 = x - w_px, y
        else:
            x0, y0 = x, y
        self._cv.create_rectangle(
            x0, y0, x0 + w_px, y0 + h_px,
            fill="#080c14", outline="#1e2a38",
            stipple="gray75",
        )
        # Opaque overlay for text readability (non-stippled, slightly smaller)
        self._cv.create_rectangle(
            x0 + 1, y0 + 1, x0 + w_px - 1, y0 + h_px - 1,
            fill="#0a0e14", outline="",
        )
        return x0, y0

    def _draw_hud(self, w, h):
        d = self._hud
        if not d:
            return

        # ── TOP-LEFT: position + altitude ────────────────────────────────────
        pad  = 6
        box_w, box_h = 220, 74
        bx, by = self._hud_box(pad, pad, box_w, box_h)

        raw_valid  = d.get("raw_valid", False)
        fix_type   = d.get("fix_type", 0)
        pos_usable = d.get("pos_usable", False)
        lat        = d.get("lat", 0.0)
        lon        = d.get("lon", 0.0)
        alt_m      = d.get("alt_m", 0.0)
        num_sat    = d.get("num_sat", 0)
        hdop       = d.get("hdop", 99.0)

        # Fix status pill
        if raw_valid and fix_type >= 2:
            fix_txt, fix_col = "3D FIX", _C["green"]
        elif raw_valid and fix_type == 1:
            fix_txt, fix_col = "2D FIX", _C["amber"]
        else:
            fix_txt, fix_col = "NO FIX", _C["red"]

        tx = bx + 6
        self._cv.create_text(tx, by + 11, text=fix_txt, anchor="w",
                             fill=fix_col, font=(_FF, 10, "bold"))

        # SAT / HDOP
        sat_col  = (_C["green"] if (raw_valid and num_sat >= 6)
                    else _C["amber"] if raw_valid else _C["dim"])
        hdop_col = (_C["green"] if hdop < 1.0
                    else _C["amber"] if hdop < 2.0 else _C["red"])
        self._cv.create_text(tx + 65, by + 11,
                             text=f"SAT {num_sat:2d}" if raw_valid else "SAT --",
                             anchor="w", fill=sat_col, font=_FHL)
        self._cv.create_text(tx + 115, by + 11,
                             text=f"HDOP {hdop:.2f}" if raw_valid else "HDOP --",
                             anchor="w", fill=hdop_col, font=_FHL)

        # Separator
        self._cv.create_line(bx + 2, by + 18, bx + box_w - 2, by + 18,
                             fill=_C["border"])

        # Lat/Lon
        if raw_valid and fix_type >= 1:
            col = _C["text"] if pos_usable else _C["amber"]
            lat_str = f"{abs(lat):10.6f} {'N' if lat >= 0 else 'S'}"
            lon_str = f"{abs(lon):10.6f} {'E' if lon >= 0 else 'W'}"
        else:
            col = _C["dim"]
            lat_str = "---.---------- -"
            lon_str = "---.---------- -"

        row_y = by + 30
        self._cv.create_text(tx, row_y, text="LAT", anchor="w",
                             fill=_C["label"], font=_FHL)
        self._cv.create_text(tx + 26, row_y, text=lat_str, anchor="w",
                             fill=col, font=_FHD)

        row_y += 18
        self._cv.create_text(tx, row_y, text="LON", anchor="w",
                             fill=_C["label"], font=_FHL)
        self._cv.create_text(tx + 26, row_y, text=lon_str, anchor="w",
                             fill=col, font=_FHD)

        # Altitude
        row_y += 16
        if raw_valid and fix_type >= 2:
            alt_str = f"ALT  {alt_m * _M_TO_FT:6.0f} ft  ({alt_m:.0f} m)"
            a_col   = _C["cyan"]
        else:
            alt_str = "ALT  ------"
            a_col   = _C["dim"]
        self._cv.create_text(tx, row_y, text=alt_str, anchor="w",
                             fill=a_col, font=_FHL)

        # ── BOTTOM-RIGHT: speed + track + home ───────────────────────────────
        br_w, br_h = 200, 110
        rx, ry = self._hud_box(w - pad, h - pad, br_w, br_h, anchor="se")

        gs_cms   = d.get("gs_cms", 0)
        trk      = d.get("trk", 0.0)
        dist_m   = d.get("dist_m", 0.0)
        brg      = d.get("brg", 0)
        comp_v   = d.get("comp_valid", False)

        tx2 = rx + 6
        cy2 = ry + 12

        # Ground speed
        if raw_valid:
            gs_kt  = gs_cms * _CMS_TO_KT
            gs_kmh = gs_cms * _CMS_TO_KMH
            self._cv.create_text(tx2, cy2, anchor="w",
                text=f"GS  {gs_kt:5.1f} kt  ({gs_kmh:.1f} km/h)",
                fill=_C["text"], font=_FHL)
        else:
            self._cv.create_text(tx2, cy2, anchor="w",
                text="GS  ---.- kt",
                fill=_C["dim"], font=_FHL)

        cy2 += 14
        # Track
        if raw_valid:
            self._cv.create_text(tx2, cy2, anchor="w",
                text=f"TRK {trk:05.1f}°",
                fill=_C["green"], font=_FHL)
        else:
            self._cv.create_text(tx2, cy2, anchor="w",
                text="TRK ---.-°", fill=_C["dim"], font=_FHL)

        cy2 += 8
        self._cv.create_line(rx + 2, cy2, rx + br_w - 2, cy2,
                             fill=_C["border"])
        cy2 += 8

        # Home dist / bearing
        if comp_v:
            self._cv.create_text(tx2, cy2, anchor="w",
                text=f"DIST  {int(dist_m):5d} m",
                fill=_C["cyan"], font=_FHL)
            cy2 += 14
            self._cv.create_text(tx2, cy2, anchor="w",
                text=f"BRG   {(brg+360)%360:03d}°",
                fill=_C["cyan"], font=_FHL)
        else:
            self._cv.create_text(tx2, cy2, anchor="w",
                text="DIST  ------ m",
                fill=_C["dim"], font=_FHL)
            cy2 += 14
            self._cv.create_text(tx2, cy2, anchor="w",
                text="BRG   ---°",
                fill=_C["dim"], font=_FHL)

        cy2 += 10
        self._cv.create_line(rx + 2, cy2, rx + br_w - 2, cy2,
                             fill=_C["border"])
        cy2 += 8

        # Mini compass roses (track + home) side by side
        rose_r = 24
        rose_gap = 10
        rose_cx1 = rx + rose_r + 6
        rose_cx2 = rx + rose_r * 2 + rose_gap + rose_r + 6
        rose_cy  = cy2 + rose_r + 2

        if rose_cy + rose_r + 4 < ry + br_h:
            # Track rose
            self._draw_hud_rose(rose_cx1, rose_cy, rose_r, "TRK",
                                trk if raw_valid else None, _C["track_needle"])
            # Home bearing rose
            self._draw_hud_rose(rose_cx2, rose_cy, rose_r, "HOME",
                                (brg + 360) % 360 if comp_v else None,
                                _C["home_arrow"])

    def _draw_hud_rose(self, cx, cy, r, label, angle_deg, needle_col):
        self._cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                             outline=_C["compass_rim"], width=1)
        self._cv.create_oval(cx - 2, cy - 2, cx + 2, cy + 2,
                             fill=_C["compass_rim"], outline="")
        # North tick
        self._cv.create_line(cx, cy - r + 3, cx, cy - r,
                             fill=_C["amber"], width=2)
        # Cardinal labels
        for lbl, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            rad = math.radians(deg - 90)
            rl  = r - 9
            self._cv.create_text(cx + math.cos(rad) * rl,
                                 cy + math.sin(rad) * rl,
                                 text=lbl, fill=_C["label"],
                                 font=(_FF, 5))
        # Needle
        if angle_deg is not None:
            rad = math.radians(angle_deg - 90)
            tip = r - 5
            self._cv.create_line(cx, cy,
                                 cx + math.cos(rad) * tip,
                                 cy + math.sin(rad) * tip,
                                 fill=needle_col, width=2,
                                 arrow="last", arrowshape=(5, 7, 2))
        # Label below
        self._cv.create_text(cx, cy + r + 6, text=label,
                             fill=_C["label"], font=(_FF, 6, "bold"))

    # ── Event handlers ────────────────────────────────────────────────────────

    def _on_resize(self, _): self._redraw()
    def _on_pan_end(self, _): self._pan_last = None

    def _on_pan_start(self, event):
        self._pan_last = (event.x, event.y)
        self._auto_center = False

    def _on_pan(self, event):
        if self._pan_last is None:
            return
        dx = event.x - self._pan_last[0]
        dy = event.y - self._pan_last[1]
        self._pan_last = (event.x, event.y)
        n = 2 ** self._zoom
        cx_f, cy_f = self._center_tile_f()
        new_xf = cx_f - dx / self.TILE_SIZE
        new_yf = cy_f - dy / self.TILE_SIZE
        self._center_lon = new_xf / n * 360.0 - 180.0
        self._center_lat = math.degrees(
            math.atan(math.sinh(math.pi * (1.0 - 2.0 * new_yf / n))))
        self._redraw()

    def _on_scroll(self, event):
        if event.delta > 0:
            self._zoom_in()
        else:
            self._zoom_out()

    def _zoom_in(self):
        if self._zoom < 19:
            self._zoom += 1
            self._zoom_lbl.config(text=f"z{self._zoom}")
            self._redraw()

    def _zoom_out(self):
        if self._zoom > 2:
            self._zoom -= 1
            self._zoom_lbl.config(text=f"z{self._zoom}")
            self._redraw()

    def _center_on_drone(self):
        if self._drone_lat is not None:
            self._center_lat = self._drone_lat
            self._center_lon = self._drone_lon
        self._auto_center = True
        self._redraw()

    # ── Public update ─────────────────────────────────────────────────────────

    def update_position(self, lat, lon, fix_valid):
        self._fix_valid = fix_valid
        if fix_valid and lat != 0.0:
            self._drone_lat = lat
            self._drone_lon = lon
            if self._auto_center:
                self._center_lat = lat
                self._center_lon = lon
        self._redraw()

    def update_hud(self, d: dict):
        """Cache nav data dict; redraws the HUD overlay."""
        self._hud = d
        self._redraw()


# =============================================================================
# Compact navigation panel  (shown when widget is too small for map)
# =============================================================================

class _NavPanel(tk.Frame):
    """Full-detail navigation readout — used when widget is too small for map."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._prev_heartbeat = None
        self._hb_state       = False
        self._build()

    def _sep(self):
        tk.Frame(self, bg=_C["border"], height=1).pack(fill="x", pady=3)

    def _lrow(self, parent, label, attr, default_text, font=_FV):
        row = tk.Frame(parent, bg=_C["bg"])
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, fg=_C["label"], bg=_C["bg"],
                 font=_FLM, width=5, anchor="w").pack(side="left")
        lbl = tk.Label(row, text=default_text, fg=_C["dim"], bg=_C["bg"], font=font)
        lbl.pack(side="left")
        setattr(self, attr, lbl)

    def _build(self):
        c = tk.Frame(self, bg=_C["bg"])
        c.pack(fill="both", expand=True, padx=8, pady=4)

        # Fix row
        fix_row = tk.Frame(c, bg=_C["bg"])
        fix_row.pack(fill="x", pady=(2, 0))
        self._fix_lbl = tk.Label(fix_row, text="NO FIX",
                                 fg=_C["red"], bg=_C["frame_bg"],
                                 font=(_FF, 11, "bold"),
                                 width=7, anchor="center",
                                 relief="flat", padx=4, pady=2)
        self._fix_lbl.pack(side="left")

        for txt, attr in (("SAT", "_sat_lbl"), ("HDOP", "_hdop_lbl")):
            f = tk.Frame(fix_row, bg=_C["bg"])
            f.pack(side="left", padx=(10, 0))
            tk.Label(f, text=txt, fg=_C["label"], bg=_C["bg"],
                     font=_FLM).pack(side="left")
            lbl = tk.Label(f, text=" --", fg=_C["dim"], bg=_C["bg"],
                           font=(_FF, 11, "bold"), width=5)
            lbl.pack(side="left")
            setattr(self, attr, lbl)

        self._hb_cv  = tk.Canvas(fix_row, width=12, height=12,
                                 bg=_C["bg"], highlightthickness=0)
        self._hb_cv.pack(side="right", padx=(0, 4))
        self._hb_dot = self._hb_cv.create_oval(1, 1, 11, 11,
                                               fill=_C["hb_off"], outline="")

        self._sep()

        # Position
        tk.Label(c, text="POSITION", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        self._lrow(c, "LAT", "_lat_lbl", "---.---------- -")
        self._lrow(c, "LON", "_lon_lbl", "---.---------- -")

        self._sep()

        # Altitude
        tk.Label(c, text="ALTITUDE MSL", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        alt_r = tk.Frame(c, bg=_C["bg"])
        alt_r.pack(fill="x")
        self._alt_ft_lbl = tk.Label(alt_r, text="------ ft",
                                    fg=_C["dim"], bg=_C["bg"], font=_FV)
        self._alt_ft_lbl.pack(side="left")
        self._alt_m_lbl  = tk.Label(alt_r, text="(------ m)",
                                    fg=_C["unit"], bg=_C["bg"], font=_FVS)
        self._alt_m_lbl.pack(side="left", padx=(8, 0))

        self._sep()

        # Ground vector
        tk.Label(c, text="GROUND VECTOR", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        inner = tk.Frame(c, bg=_C["bg"])
        inner.pack(fill="x")
        left  = tk.Frame(inner, bg=_C["bg"])
        left.pack(side="left", anchor="n")
        self._lrow(left, "GS",  "_gs_kt_lbl", "---.- kt")
        self._lrow(left, "  ",  "_gs_km_lbl", "(--- km/h)", font=_FVS)
        self._lrow(left, "TRK", "_trk_lbl",   "---.-°")

        self._trk_cv = tk.Canvas(inner, width=68, height=68,
                                 bg=_C["compass_bg"], highlightthickness=1,
                                 highlightbackground=_C["compass_rim"])
        self._trk_cv.pack(side="right", padx=(8, 0))
        self._draw_rose(self._trk_cv, 34, 34, 28, cardinals=True)
        self._trk_needle = self._trk_cv.create_line(
            34, 34, 34, 10, fill=_C["track_needle"],
            width=2, arrow="last", arrowshape=(7, 9, 3))

        self._sep()

        # Home
        tk.Label(c, text="HOME POINT", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        inner2 = tk.Frame(c, bg=_C["bg"])
        inner2.pack(fill="x")
        left2  = tk.Frame(inner2, bg=_C["bg"])
        left2.pack(side="left", anchor="n")
        self._lrow(left2, "DIST", "_dist_lbl", "------ m")
        self._lrow(left2, "BRG",  "_brg_lbl",  "---°")

        self._home_cv = tk.Canvas(inner2, width=68, height=68,
                                  bg=_C["compass_bg"], highlightthickness=1,
                                  highlightbackground=_C["compass_rim"])
        self._home_cv.pack(side="right", padx=(8, 0))
        self._draw_rose(self._home_cv, 34, 34, 28, cardinals=False)
        self._home_arrow = self._home_cv.create_line(
            34, 34, 34, 10, fill=_C["home_arrow"],
            width=2, arrow="last", arrowshape=(7, 9, 3))

    # ── Compass helpers ───────────────────────────────────────────────────────

    def _draw_rose(self, cv, cx, cy, r, cardinals=True):
        cv.create_oval(cx-r, cy-r, cx+r, cy+r,
                       outline=_C["compass_rim"], width=1)
        cv.create_oval(cx-2, cy-2, cx+2, cy+2,
                       fill=_C["compass_rim"], outline="")
        for deg in range(0, 360, 45):
            rad = math.radians(deg - 90)
            r1  = r - 5
            cv.create_line(cx + math.cos(rad)*r1, cy + math.sin(rad)*r1,
                           cx + math.cos(rad)*r,  cy + math.sin(rad)*r,
                           fill=_C["compass_rim"], width=1)
        cv.create_line(cx, cy - r + 5, cx, cy - r, fill=_C["amber"], width=2)
        if cardinals:
            rl = r - 11
            for lbl, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
                rad = math.radians(deg - 90)
                cv.create_text(cx + math.cos(rad)*rl, cy + math.sin(rad)*rl,
                               text=lbl, fill=_C["label"], font=(_FF, 6))

    def _point_needle(self, cv, needle_id, cx, cy, r, angle_deg):
        rad   = math.radians(angle_deg - 90)
        tip_r = r - 4
        cv.coords(needle_id, cx, cy,
                  cx + math.cos(rad)*tip_r, cy + math.sin(rad)*tip_r)

    # ── Public update ─────────────────────────────────────────────────────────

    def update(self, d: dict):
        raw_valid  = d.get("raw_valid", False)
        comp_valid = d.get("comp_valid", False)
        pos_usable = d.get("pos_usable", False)
        fix_type   = d.get("fix_type", 0)
        num_sat    = d.get("num_sat", 0)
        hdop       = d.get("hdop", 99.0)
        lat        = d.get("lat", 0.0)
        lon        = d.get("lon", 0.0)
        alt_m      = d.get("alt_m", 0.0)
        gs_cms     = d.get("gs_cms", 0)
        trk        = d.get("trk", 0.0)
        dist_m     = d.get("dist_m", 0.0)
        brg        = d.get("brg", 0)
        heartbeat  = d.get("heartbeat", None)

        # Fix badge
        if raw_valid and fix_type >= 2:
            self._fix_lbl.config(text="3D FIX", fg=_C["green"])
        elif raw_valid and fix_type == 1:
            self._fix_lbl.config(text="2D FIX", fg=_C["amber"])
        else:
            self._fix_lbl.config(text="NO FIX", fg=_C["red"])

        # SAT
        self._sat_lbl.config(
            text=f"{num_sat:3d}" if raw_valid else " --",
            fg=_C["green"] if (raw_valid and num_sat >= 6)
               else _C["amber"] if raw_valid else _C["dim"])

        # HDOP
        if raw_valid and hdop < 99.0:
            hcol = (_C["green"] if hdop < 1.0
                    else _C["amber"] if hdop < 2.0 else _C["red"])
            self._hdop_lbl.config(text=f"{hdop:5.2f}", fg=hcol)
        else:
            self._hdop_lbl.config(text=" -.--", fg=_C["dim"])

        # Heartbeat blink
        if heartbeat is not None and heartbeat != self._prev_heartbeat:
            self._hb_state       = not self._hb_state
            self._prev_heartbeat = heartbeat
        self._hb_cv.itemconfig(self._hb_dot,
                               fill=_C["hb_on"] if self._hb_state else _C["hb_off"])

        # Position
        if raw_valid and fix_type >= 1:
            col = _C["text"] if pos_usable else _C["amber"]
            self._lat_lbl.config(
                text=f"{abs(lat):10.6f} {'N' if lat >= 0 else 'S'}", fg=col)
            self._lon_lbl.config(
                text=f"{abs(lon):10.6f} {'E' if lon >= 0 else 'W'}", fg=col)
        else:
            self._lat_lbl.config(text="---.---------- -", fg=_C["dim"])
            self._lon_lbl.config(text="---.---------- -", fg=_C["dim"])

        # Altitude
        if raw_valid and fix_type >= 2:
            self._alt_ft_lbl.config(text=f"{alt_m * _M_TO_FT:7.0f} ft", fg=_C["text"])
            self._alt_m_lbl.config( text=f"({alt_m:6.0f} m)",             fg=_C["unit"])
        else:
            self._alt_ft_lbl.config(text="------ ft",  fg=_C["dim"])
            self._alt_m_lbl.config( text="(------ m)", fg=_C["unit"])

        # Ground speed
        if raw_valid:
            self._gs_kt_lbl.config(text=f"{gs_cms * _CMS_TO_KT:5.1f} kt",      fg=_C["text"])
            self._gs_km_lbl.config(text=f"({gs_cms * _CMS_TO_KMH:5.1f} km/h)", fg=_C["unit"])
        else:
            self._gs_kt_lbl.config(text="---.- kt",   fg=_C["dim"])
            self._gs_km_lbl.config(text="(--- km/h)", fg=_C["dim"])

        # Track
        if raw_valid:
            self._trk_lbl.config(text=f"{trk:05.1f}°", fg=_C["text"])
            self._point_needle(self._trk_cv, self._trk_needle, 34, 34, 28, trk)
        else:
            self._trk_lbl.config(text="---.-°", fg=_C["dim"])
            self._point_needle(self._trk_cv, self._trk_needle, 34, 34, 28, 0)

        # Home
        if comp_valid:
            brg360 = (brg + 360) % 360
            self._dist_lbl.config(text=f"{int(dist_m):6d} m", fg=_C["cyan"])
            self._brg_lbl.config( text=f"{brg360:03d}°",       fg=_C["cyan"])
            self._point_needle(self._home_cv, self._home_arrow, 34, 34, 28, brg360)
        else:
            self._dist_lbl.config(text="------ m", fg=_C["dim"])
            self._brg_lbl.config( text="---°",     fg=_C["dim"])
            self._point_needle(self._home_cv, self._home_arrow, 34, 34, 28, 0)


# =============================================================================
# Main GPS Widget
# =============================================================================

class GPSWidget(tk.Frame):
    """
    Aviation-grade GPS navigation widget.

    Layout modes (auto-selected by widget size):

      MAP MODE  (widget ≥ MIN_MAP_W × MIN_MAP_H)
        • Full-width map with navigation data overlaid as HUD panels.
        • "SAT ▶" button in the title bar toggles the satellite panel
          which appears to the RIGHT of the map (map shrinks, doesn't hide).

      NAV MODE  (widget too small for map)
        • Classic scrollable navigation readout (no map).
        • "SAT" button in the header toggles a floating satellite Toplevel.

    Public API
    ----------
    update_gps(ui_data)   — call on every telemetry tick; same key contract
                             as the original GPSWidget.
    """

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._sat_open     = False
        self._map_mode     = True        # current layout mode
        self._last_ui_data = {}
        self._sat_toplevel = None        # used only in nav-mode fallback

        self._build_ui()
        self.bind("<Configure>", self._on_configure)

    # =========================================================================
    # Build UI
    # =========================================================================

    def _build_ui(self):
        # ── Title / header bar ───────────────────────────────────────────────
        self._hdr = tk.Frame(self, bg=_C["frame_bg"], height=26)
        self._hdr.pack(fill="x")
        self._hdr.pack_propagate(False)

        tk.Label(self._hdr, text="GPS NAVIGATION",
                 fg=_C["label"], bg=_C["frame_bg"],
                 font=(_FF, 9, "bold")).pack(side="left", padx=8)

        self._sat_btn = tk.Button(
            self._hdr, text="SAT ▶",
            command=self._toggle_sat,
            bg=_C["map_btn"], fg=_C["amber"],
            relief="flat", padx=6, pady=1,
            font=(_FF, 8, "bold"),
            activebackground="#2a3545", activeforeground=_C["amber"],
            cursor="hand2",
        )
        self._sat_btn.pack(side="right", padx=6, pady=3)

        # Heartbeat indicator in header
        self._hb_cv  = tk.Canvas(self._hdr, width=10, height=10,
                                 bg=_C["frame_bg"], highlightthickness=0)
        self._hb_cv.pack(side="right", padx=(0, 4))
        self._hb_dot = self._hb_cv.create_oval(1, 1, 9, 9,
                                               fill=_C["hb_off"], outline="")
        self._prev_heartbeat = None
        self._hb_state       = False

        # Thin separator
        tk.Frame(self, bg=_C["border"], height=1).pack(fill="x")

        # ── Content area ─────────────────────────────────────────────────────
        self._content = tk.Frame(self, bg=_C["bg"])
        self._content.pack(fill="both", expand=True)

        self._build_map_layout()
        self._build_nav_layout()

        # Initial layout — start in map mode
        self._apply_map_mode()

    def _build_map_layout(self):
        """Map + optional side satellite panel."""
        self._map_frame = tk.Frame(self._content, bg=_C["bg"])

        self._map_canvas = _MapCanvas(self._map_frame)
        self._map_canvas.pack(side="left", fill="both", expand=True)

        self._sat_panel = _SatPanel(self._map_frame)
        # not packed initially

    def _build_nav_layout(self):
        """Compact nav panel (fallback for small widget sizes)."""
        self._nav_frame = tk.Frame(self._content, bg=_C["bg"])
        self._nav_panel = _NavPanel(self._nav_frame)
        self._nav_panel.pack(fill="both", expand=True)

    # =========================================================================
    # Layout mode switching
    # =========================================================================

    def _apply_map_mode(self):
        self._map_mode = True
        self._nav_frame.pack_forget()
        self._map_frame.pack(fill="both", expand=True)
        # Restore sat panel visibility
        if self._sat_open:
            self._sat_panel.pack(side="right", fill="y")
        self._sat_btn.config(state="normal")

    def _apply_nav_mode(self):
        self._map_mode = False
        self._map_frame.pack_forget()
        self._sat_panel.pack_forget()
        self._nav_frame.pack(fill="both", expand=True)
        self._sat_btn.config(state="normal")

    def _on_configure(self, event):
        w = event.width
        h = event.height - 27   # subtract header bar
        want_map = (w >= MIN_MAP_W and h >= MIN_MAP_H)
        if want_map != self._map_mode:
            if want_map:
                self._apply_map_mode()
            else:
                self._apply_nav_mode()
            # Re-push last known data into new layout
            if self._last_ui_data:
                self._push_data(self._last_ui_data)

    # =========================================================================
    # Satellite panel toggle
    # =========================================================================

    def _toggle_sat(self):
        self._sat_open = not self._sat_open
        if self._sat_open:
            self._sat_btn.config(text="SAT ◀")
            if self._map_mode:
                self._sat_panel.pack(side="right", fill="y")
            else:
                self._open_sat_toplevel()
        else:
            self._sat_btn.config(text="SAT ▶")
            if self._map_mode:
                self._sat_panel.pack_forget()
            else:
                self._close_sat_toplevel()

    def _open_sat_toplevel(self):
        """In nav-mode, satellites float in a Toplevel window."""
        if self._sat_toplevel and self._sat_toplevel.winfo_exists():
            return
        tl = tk.Toplevel(self)
        tl.title("Satellite Constellation")
        tl.configure(bg=_C["frame_bg"])
        tl.resizable(False, True)
        tl.geometry(f"{_SatPanel.SAT_PANEL_W + 20}x400")
        tl.protocol("WM_DELETE_WINDOW", self._close_sat_toplevel)

        panel = _SatPanel(tl)
        panel.pack(fill="both", expand=True, padx=4, pady=4)
        self._sat_toplevel_panel = panel
        self._sat_toplevel       = tl

        # Push latest data
        if self._last_ui_data:
            raw  = self._last_ui_data.get("gps_sv_list", [])
            norm = _normalize_sv_list(raw)
            panel.update_satellites(norm)

    def _close_sat_toplevel(self):
        self._sat_open = False
        self._sat_btn.config(text="SAT ▶")
        if self._sat_toplevel and self._sat_toplevel.winfo_exists():
            self._sat_toplevel.destroy()
        self._sat_toplevel = None

    # =========================================================================
    # Public update
    # =========================================================================

    def update_gps(self, ui_data: dict):
        self._last_ui_data = ui_data
        self._push_data(ui_data)

    def _push_data(self, ui_data: dict):
        raw_valid  = ui_data.get("gps_raw_valid",        False)
        comp_valid = ui_data.get("gps_comp_valid",       False)
        pos_usable = ui_data.get("gps_position_usable",  False)
        fix_type   = ui_data.get("gps_fix_type",          0)
        num_sat    = ui_data.get("gps_num_sat",           0)
        hdop       = ui_data.get("gps_hdop",              99.0)
        lat        = ui_data.get("gps_latitude",          0.0)
        lon        = ui_data.get("gps_longitude",         0.0)
        alt_m      = float(ui_data.get("gps_altitude_m",  0.0))
        gs_cms     = ui_data.get("gps_ground_speed_cms",  0)
        course_dd  = ui_data.get("gps_ground_course",     0)
        dist_m     = float(ui_data.get("gps_dist_to_home_m",  0.0))
        brg        = int(ui_data.get("gps_bearing_to_home",   0))
        heartbeat  = ui_data.get("gps_heartbeat",         None)
        trk        = (course_dd / 10.0) % 360.0

        # Common nav dict shared by all sub-widgets
        nav = dict(
            raw_valid=raw_valid, comp_valid=comp_valid,
            pos_usable=pos_usable, fix_type=fix_type,
            num_sat=num_sat, hdop=hdop,
            lat=lat, lon=lon, alt_m=alt_m,
            gs_cms=gs_cms, trk=trk,
            dist_m=dist_m, brg=brg,
            heartbeat=heartbeat,
        )

        # Heartbeat in header (shared across both modes)
        if heartbeat is not None and heartbeat != self._prev_heartbeat:
            self._hb_state       = not self._hb_state
            self._prev_heartbeat = heartbeat
        self._hb_cv.itemconfig(
            self._hb_dot,
            fill=_C["hb_on"] if self._hb_state else _C["hb_off"])

        if self._map_mode:
            self._map_canvas.update_position(lat, lon, raw_valid and fix_type >= 2)
            self._map_canvas.update_hud(nav)
        else:
            self._nav_panel.update(nav)

        # Satellite panel (in-frame or floating toplevel)
        raw_sv = ui_data.get("gps_sv_list", [])
        norm   = _normalize_sv_list(raw_sv)
        self._sat_panel.update_satellites(norm)
        if (self._sat_toplevel and self._sat_toplevel.winfo_exists()
                and hasattr(self, "_sat_toplevel_panel")):
            self._sat_toplevel_panel.update_satellites(norm)