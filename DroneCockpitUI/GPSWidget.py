"""
GPSWidget.py  —  Aviation-grade GPS Navigation Widget
=====================================================

FIXES applied (v7):
  1. Centering fix — drone marker is now centred in the *visible* map area
     (above the HUD overlay) rather than in the full canvas height.
     A new helper _hud_height(w, h) mirrors the HUD layout logic and returns
     the actual HUD box height so every coordinate-to-pixel conversion and the
     auto-centre pan can offset the effective vertical centre appropriately.

  2. No-GPS warning — the "NO GPS FIX" overlay is now a large, bold, attention-
     grabbing banner with a pulsing amber/red border, replacing the small dim
     text that was easy to miss.

  (all prior v6/v5/v4 fixes retained)
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
    "bg":           "#0d1117",
    "frame_bg":     "#141920",
    "border":       "#1e2730",
    "hud_bg":       "#0a0e14cc",
    "text":         "#e8edf4",
    "label":        "#6b7885",
    "unit":         "#8a9aaa",
    "dim":          "#3a4555",
    "green":        "#00e676",
    "amber":        "#ffb300",
    "red":          "#ff3d3d",
    "cyan":         "#00e5ff",
    "magenta":      "#ff4081",
    "compass_bg":   "#080c10",
    "compass_rim":  "#2a3545",
    "track_needle": "#00e676",
    "home_arrow":   "#00e5ff",
    "hb_on":        "#00e676",
    "hb_off":       "#1e2730",
    "map_bg":       "#1a2030",
    "map_btn":      "#1e2a38",
    "map_marker":   "#ffb300",
    "map_home":     "#00e5ff",
    "sv_row_even":  "#0d1117",
    "sv_row_odd":   "#111820",
    "sv_used":      "#00e676",
    "sv_locked":    "#ffb300",
    "sv_nodata":    "#2a3545",
}

# ── Fonts ───────────────────────────────────────────────────────────────────
_FF  = "Courier New"
_FL  = (_FF,  8, "bold")
_FLM = (_FF,  9, "bold")
_FV  = (_FF, 14, "bold")
_FVS = (_FF, 11, "bold")
_FU  = (_FF,  9)
_FHD = (_FF, 13, "bold")
_FHL = (_FF, 10, "bold")
_FTB = (_FF,  9, "bold")

# ── Quality tables (Betaflight 4.5.3) ───────────────────────────────────────
_QUALITY_LABEL = [
    "idle", "search", "acquir", "unusbl",
    "c-lock", "f-lock", "f-lock", "f-lock",
]
_QUALITY_COLOR = [
    "#2a3545", "#2a3545", "#ffb300", "#ff3d3d",
    "#ffb300", "#00e676", "#00e676", "#00e676",
]


# =============================================================================
# SVInfoEntry normaliser
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
# Satellite panel
# =============================================================================

class _SatPanel(tk.Frame):
    """Compact satellite signal-strength panel with sticky column header."""

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
        tk.Frame(self, bg=_C["border"], height=1).pack(fill="x")
        title_row = tk.Frame(self, bg=_C["frame_bg"])
        title_row.pack(fill="x", padx=6, pady=(4, 2))
        tk.Label(title_row, text="SATELLITE CONSTELLATION",
                 fg=_C["label"], bg=_C["frame_bg"],
                 font=(_FF, 8, "bold")).pack(side="left")

        self._col_hdr_frame = tk.Frame(self, bg=_C["frame_bg"])
        self._col_hdr_frame.pack(fill="x")
        self._col_hdr_cv = tk.Canvas(self._col_hdr_frame,
                                     bg=_C["frame_bg"], height=16,
                                     highlightthickness=0)
        self._col_hdr_cv.pack(fill="x", padx=4)
        self._draw_col_header()
        tk.Frame(self, bg=_C["border"], height=1).pack(fill="x")

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
        cv = self._col_hdr_cv
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

            gnss_raw = str(sv.get("gnss_id", "?"))
            gnss_key = self._GNSS_ALIASES.get(gnss_raw.lower(), gnss_raw)
            gcol     = self._GNSS_COLOR.get(gnss_key, self._GNSS_COLOR["Unknown"])
            self._cv.create_text(x, cy, text=gnss_key, anchor="w",
                                 fill=gcol, font=(_FF, 8, "bold"))
            x += cols[0][1]

            self._cv.create_text(x, cy, text=str(sv.get("sv_id", "--")),
                                 anchor="w", fill=_C["text"], font=(_FF, 8))
            x += cols[1][1]

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

            self._cv.create_text(
                x, cy,
                text=f"{cno:2d}" if cno > 0 else "--",
                anchor="w",
                fill=_C["text"] if cno > 0 else _C["dim"],
                font=(_FF, 8))
            x += cols[3][1]

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

    v7 centering fix:
      _hud_height(w, h) returns the pixel height of the HUD box drawn at the
      bottom of the canvas.  All coordinate <-> pixel conversions now use an
      effective canvas centre:
          eff_cy = (h - hud_h) / 2
      so the drone marker and auto-centre pan are relative to the visible map
      strip above the HUD, not the full canvas height.

      _center_on_drone() and the auto-centre path in update_position() both
      compute the compensated lat so the drone tile lands at eff_cy.

    v7 no-GPS warning:
      _draw_no_fix_warning() replaces the old single line of small text with a
      large pulsing banner centred in the visible map strip.
    """
    TILE_SIZE = 256
    _OSM_URL  = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    _UA       = "DroneCockpitGCS/1.0 (github educational project)"
    _ROSE_R   = 36

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._zoom        = 15
        self._center_lat  = 44.77
        self._center_lon  = 17.21
        self._drone_lat   = None
        self._drone_lon   = None
        self._fix_valid   = False
        self._auto_center = True

        self._tile_img   = {}
        self._tile_queue = queue.Queue()
        self._tile_pend  = set()
        self._pan_last   = None
        self._hud        = {}

        self._no_fix_pulse = False   # toggles every 600 ms for warning animation

        self._build()
        self._poll_tiles()
        self._pulse_no_fix()

    # ── Build ─────────────────────────────────────────────────────────────────

    def _build(self):
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

    def zoom_in(self):  self._zoom_in()
    def zoom_out(self): self._zoom_out()
    def center_drone(self): self._center_on_drone()

    # ── Pulsing no-fix warning ────────────────────────────────────────────────

    def _pulse_no_fix(self):
        self._no_fix_pulse = not self._no_fix_pulse
        if not self._fix_valid:
            self._redraw()
        self.after(600, self._pulse_no_fix)

    # ── HUD height mirror ─────────────────────────────────────────────────────

    def _hud_height(self, w: int, h: int) -> int:
        """
        Return the pixel height of the HUD box that _draw_hud will draw at the
        bottom of a (w x h) canvas.  Mirrors the layout decisions in _draw_hud
        so that centering can compensate correctly.  Returns 0 when no HUD data.
        """
        if not self._hud:
            return 0

        pad            = 6
        avail_h        = h - pad * 2
        rose_r         = self._ROSE_R
        rose_section_h = 5 + rose_r * 2 + 14 + 4   # sep_sp + diam + lbl_gap + margin

        if w >= 560:
            ip, rh, sep_sp = 10, 22, 5
            core_h     = ip + rh + sep_sp + rh + sep_sp
            show_roses = (avail_h - core_h - ip) >= rose_section_h
            return core_h + (rose_section_h if show_roses else 0) + ip

        elif w >= 380:
            ip, rh, row_gap, sep_sp = 8, 20, 3, 6
            avail_w    = w - pad * 2
            box_w      = max(min(avail_w, 600), 164 + 12 + 80 + ip * 2)
            ix0        = pad + ip
            ix1        = pad + box_w - ip
            single_row = (ix0 + 292 + 12 + 80) <= ix1
            if single_row:
                core_h = ip + rh + sep_sp + rh + ip
            else:
                core_h = ip + rh + row_gap + rh + sep_sp + rh + ip
            show_roses = (avail_h - core_h) >= (rose_section_h + 8)
            return core_h + (rose_section_h if show_roses else 0)

        else:
            ip, rh, sp = 6, 19, 2
            max_rows   = max(1, (avail_h - ip * 2 + sp) // (rh + sp))
            n_rows     = min(8, max_rows)
            return ip + n_rows * rh + (n_rows - 1) * sp + ip

    # ── Effective vertical centre ─────────────────────────────────────────────

    def _eff_center_y(self, w: int, h: int) -> float:
        """Centre Y of the visible map strip (above HUD overlay)."""
        return (h - self._hud_height(w, h)) / 2.0

    # ── Tile maths ────────────────────────────────────────────────────────────

    @staticmethod
    def _deg2tile_f(lat, lon, zoom):
        n  = 2 ** zoom
        xf = (lon + 180.0) / 360.0 * n
        lr = math.radians(lat)
        yf = (1.0 - math.asinh(math.tan(lr)) / math.pi) / 2.0 * n
        return xf, yf

    def _center_tile_f(self):
        return self._deg2tile_f(self._center_lat, self._center_lon, self._zoom)

    def _latlon_to_canvas(self, lat, lon):
        """Convert lat/lon → canvas pixel, using the visible-strip centre for Y."""
        w  = self._cv.winfo_width()  or 400
        h  = self._cv.winfo_height() or 300
        cx, cy = self._center_tile_f()
        xf, yf = self._deg2tile_f(lat, lon, self._zoom)
        return (w / 2.0 + (xf - cx) * self.TILE_SIZE,
                self._eff_center_y(w, h) + (yf - cy) * self.TILE_SIZE)

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

    # ── Redraw ────────────────────────────────────────────────────────────────

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
        """Draw OSM tiles centred on the visible map strip (above HUD)."""
        n          = 2 ** self._zoom
        cx_f, cy_f = self._center_tile_f()
        cx_ti      = int(cx_f)
        cy_ti      = int(cy_f)
        tiles_x    = w // self.TILE_SIZE + 3
        tiles_y    = h // self.TILE_SIZE + 3
        eff_cy     = self._eff_center_y(w, h)

        for dx in range(-tiles_x // 2 - 1, tiles_x // 2 + 2):
            for dy in range(-tiles_y // 2 - 1, tiles_y // 2 + 2):
                tx = (cx_ti + dx) % n
                ty = cy_ti + dy
                if ty < 0 or ty >= n:
                    continue
                px  = w / 2.0  + (cx_ti + dx - cx_f) * self.TILE_SIZE
                py  = eff_cy   + (cy_ti + dy - cy_f) * self.TILE_SIZE
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
            self._draw_no_fix_warning(w, h)
            return
        mx, my = self._latlon_to_canvas(self._drone_lat, self._drone_lon)
        r = 9

        # 1. Dark shadow halo — punches through any map tile colour
        self._cv.create_oval(mx - r - 4, my - r - 4, mx + r + 4, my + r + 4,
                             fill="#000000", outline="", stipple="gray50")

        # 2. Outer dashed range ring (cyan, slightly heavier)
        self._cv.create_oval(mx - r*2, my - r*2, mx + r*2, my + r*2,
                             outline=_C["cyan"], width=1.5, dash=(3, 4))

        # 3. Solid black backing disc — guarantees contrast for the dot itself
        self._cv.create_oval(mx - r - 1, my - r - 1, mx + r + 1, my + r + 1,
                             fill="#000000", outline="")

        # 4. Main marker — magenta is perceptually opposite to OSM's greens/yellows
        self._cv.create_oval(mx - r, my - r, mx + r, my + r,
                             fill=_C["magenta"], outline="#ffffff", width=2)

        # 5. Track arrow — white shadow then green on top for legibility
        trk  = self._hud.get("trk", 0)
        rad  = math.radians(trk - 90)
        vlen = 26
        tip_x = mx + math.cos(rad) * vlen
        tip_y = my + math.sin(rad) * vlen
        self._cv.create_line(mx, my, tip_x, tip_y,
                             fill="#000000", width=4,
                             arrow="last", arrowshape=(8, 10, 4))
        self._cv.create_line(mx, my, tip_x, tip_y,
                             fill=_C["green"], width=2,
                             arrow="last", arrowshape=(7, 9, 3))

    # ── Prominent no-fix warning ──────────────────────────────────────────────

    def _draw_no_fix_warning(self, w, h):
        """
        Large, bold, pulsing NO GPS SIGNAL banner centred in the visible map
        strip (above the HUD overlay).
        """
        hud_h  = self._hud_height(w, h)
        vis_h  = h - hud_h
        cx     = w // 2
        cy     = vis_h // 2

        # Alternate between red and amber for the pulsing border
        border_col = _C["red"] if self._no_fix_pulse else _C["amber"]

        bw, bh = min(w - 40, 290), 80
        x0     = cx - bw // 2
        y0     = cy - bh // 2
        x1     = cx + bw // 2
        y1     = cy + bh // 2

        # Outer glow ring (thick)
        self._cv.create_rectangle(x0 - 5, y0 - 5, x1 + 5, y1 + 5,
                                  fill="", outline=border_col, width=3)
        # Inner box
        self._cv.create_rectangle(x0, y0, x1, y1,
                                  fill="#110408", outline=border_col, width=2)

        # Main warning text
        self._cv.create_text(cx, y0 + 22,
                             text="\u26a0  NO GPS SIGNAL  \u26a0",
                             fill=border_col,
                             font=(_FF, 14, "bold"),
                             anchor="center")

        # Sub-label
        self._cv.create_text(cx, y0 + 48,
                             text="WAITING FOR FIX",
                             fill=_C["label"],
                             font=(_FF, 10, "bold"),
                             anchor="center")

        # Animated scanning dots
        dot_y   = y1 - 13
        n_dots  = 7
        spacing = max(bw // (n_dots + 1), 1)
        for i in range(n_dots):
            lit     = (i % 2 == (0 if self._no_fix_pulse else 1))
            dot_col = _C["amber"] if lit else _C["dim"]
            dx      = x0 + spacing * (i + 1)
            self._cv.create_oval(dx - 3, dot_y - 3, dx + 3, dot_y + 3,
                                 fill=dot_col, outline="")

    # ── HUD helpers ───────────────────────────────────────────────────────────

    def _hud_box(self, x0, y0, w_px, h_px):
        x1, y1 = x0 + w_px, y0 + h_px
        self._cv.create_rectangle(x0, y0, x1, y1,
            fill="#080c14", outline="#1e2a38", stipple="gray75")
        self._cv.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1,
            fill="#0a0e14", outline="")
        return x0, y0

    def _draw_hud_rose(self, cx, cy, r, label, angle_deg, needle_col):
        cv = self._cv
        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       outline=_C["compass_rim"], width=1)
        cv.create_oval(cx - 2, cy - 2, cx + 2, cy + 2,
                       fill=_C["compass_rim"], outline="")
        for deg in range(0, 360, 45):
            rad = math.radians(deg - 90)
            r1  = r - 5
            cv.create_line(cx + math.cos(rad) * r1, cy + math.sin(rad) * r1,
                           cx + math.cos(rad) * r,  cy + math.sin(rad) * r,
                           fill=_C["compass_rim"], width=1)
        cv.create_line(cx, cy - r + 5, cx, cy - r,
                       fill=_C["amber"], width=2)
        for lbl, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            rad = math.radians(deg - 90)
            rl  = r - 12
            cv.create_text(cx + math.cos(rad) * rl,
                           cy + math.sin(rad) * rl,
                           text=lbl, fill=_C["label"],
                           font=(_FF, 7, "bold"))
        if angle_deg is not None:
            rad = math.radians(angle_deg - 90)
            tip = r - 5
            cv.create_line(cx, cy,
                           cx + math.cos(rad) * tip,
                           cy + math.sin(rad) * tip,
                           fill=needle_col, width=2,
                           arrow="last", arrowshape=(6, 8, 3))
        cv.create_text(cx, cy + r + 9, text=label,
                       fill=_C["label"], font=(_FF, 8, "bold"))

    # ── HUD overlay ───────────────────────────────────────────────────────────

    def _draw_hud(self, w, h):
        d = self._hud
        if not d:
            return

        pad = 6
        cv  = self._cv

        raw_valid  = d.get("raw_valid",  False)
        fix_type   = d.get("fix_type",   0)
        pos_usable = d.get("pos_usable", False)
        lat        = d.get("lat",    0.0)
        lon        = d.get("lon",    0.0)
        alt_m      = d.get("alt_m",  0.0)
        gs_cms     = d.get("gs_cms", 0)
        trk        = d.get("trk",    0.0)
        dist_m     = d.get("dist_m", 0.0)
        brg        = d.get("brg",    0)
        comp_v     = d.get("comp_valid", False)

        if raw_valid and fix_type >= 2:
            fix_txt, fix_col = "3D FIX", _C["green"]
        elif raw_valid and fix_type == 1:
            fix_txt, fix_col = "2D FIX", _C["amber"]
        else:
            fix_txt, fix_col = "NO FIX", _C["red"]

        pos_col = _C["text"] if pos_usable else (_C["amber"] if raw_valid else _C["dim"])
        ok_pos  = raw_valid and fix_type >= 1
        ok_alt  = raw_valid and fix_type >= 2

        if ok_pos:
            lat_num = f"{abs(lat):010.6f}"
            lat_hem = "N" if lat >= 0 else "S"
            lon_num = f"{abs(lon):010.6f}"
            lon_hem = "E" if lon >= 0 else "W"
        else:
            lat_num = "---.------"
            lat_hem = "-"
            lon_num = "---.------"
            lon_hem = "-"

        alt_str = f"{alt_m * _M_TO_FT:.0f} ft" if ok_alt else "---- ft"
        alt_col = _C["cyan"]  if ok_alt    else _C["dim"]

        gs_kt   = gs_cms * _CMS_TO_KT if raw_valid else 0.0
        gs_col  = _C["text"]  if raw_valid else _C["dim"]
        trk_col = _C["green"] if raw_valid else _C["dim"]
        dist_col= _C["cyan"]  if comp_v    else _C["dim"]
        brg360  = (brg + 360) % 360

        gs_str   = f"{gs_kt:5.1f} kt"      if raw_valid else " --.- kt"
        trk_str  = f"{trk:06.2f}\u00b0"    if raw_valid else "---.-\u00b0"
        dist_str = f"{int(dist_m):5d} m"   if comp_v    else " --- m"
        brg_str  = f"{brg360:03d}\u00b0"   if comp_v    else "---\u00b0"

        avail_w = w - pad * 2
        avail_h = h - pad * 2

        rose_r         = self._ROSE_R
        rose_diam      = rose_r * 2
        lbl_gap        = 14
        rose_sep_sp    = 5
        rose_section_h = rose_sep_sp + rose_diam + lbl_gap + 4

        # ═══════════════════════════════════════════════════════════════════
        # FULL tier  (w >= 560)
        # ═══════════════════════════════════════════════════════════════════
        if w >= 560:
            ip      = 10
            rh      = 22
            sep_sp  = 5

            C_FIX   = 0
            C_LAT   = 58
            C_LAT_V = 90
            C_LAT_H = 190
            C_LON   = 208
            C_LON_V = 240
            C_LON_H = 346

            inner_needed = C_LON_H + 10 + 80
            box_w = max(min(avail_w, 680), inner_needed + ip * 2)

            core_h = ip + rh + sep_sp + rh + sep_sp
            show_roses = (avail_h - core_h - ip) >= rose_section_h
            box_h = core_h + (rose_section_h if show_roses else 0) + ip

            bx = pad
            by = max(pad, h - pad - box_h)
            self._hud_box(bx, by, box_w, box_h)

            ix0 = bx + ip
            ix1 = bx + box_w - ip
            r1y = by + ip + rh // 2

            cv.create_text(ix0 + C_FIX, r1y, text=fix_txt,
                           anchor="w", fill=fix_col, font=(_FF, 10, "bold"))
            cv.create_text(ix0 + C_LAT, r1y, text="LAT",
                           anchor="w", fill=_C["label"], font=(_FF, 9))
            cv.create_text(ix0 + C_LAT_V, r1y, text=lat_num,
                           anchor="w", fill=pos_col, font=_FHD)
            cv.create_text(ix0 + C_LAT_H, r1y, text=lat_hem,
                           anchor="w", fill=_C["unit"], font=(_FF, 9, "bold"))

            alt_block_x = ix1 - 80
            if ix0 + C_LON_H + 14 < alt_block_x:
                cv.create_text(ix0 + C_LON, r1y, text="LON",
                               anchor="w", fill=_C["label"], font=(_FF, 9))
                cv.create_text(ix0 + C_LON_V, r1y, text=lon_num,
                               anchor="w", fill=pos_col, font=_FHD)
                cv.create_text(ix0 + C_LON_H, r1y, text=lon_hem,
                               anchor="w", fill=_C["unit"], font=(_FF, 9, "bold"))

            cv.create_text(ix1, r1y, text=f"ALT  {alt_str}",
                           anchor="e", fill=alt_col, font=(_FF, 10, "bold"))

            sep1y = by + ip + rh + sep_sp // 2
            cv.create_line(bx + 3, sep1y, bx + box_w - 3, sep1y,
                           fill=_C["border"])
            r2y = sep1y + sep_sp // 2 + rh // 2 + 1

            half_x = bx + box_w // 2
            cv.create_text(ix0,       r2y, text="GS",
                           anchor="w", fill=_C["label"], font=(_FF, 9))
            cv.create_text(ix0 + 24,  r2y, text=gs_str,
                           anchor="w", fill=gs_col,  font=(_FF, 10, "bold"))
            cv.create_text(ix0 + 110, r2y, text="TRK",
                           anchor="w", fill=_C["label"], font=(_FF, 9))
            cv.create_text(ix0 + 138, r2y, text=trk_str,
                           anchor="w", fill=trk_col, font=(_FF, 10, "bold"))

            cv.create_line(half_x, sep1y + sep_sp // 2,
                           half_x, sep1y + sep_sp // 2 + rh + sep_sp,
                           fill=_C["border"])

            rx0 = half_x + ip
            cv.create_text(rx0,       r2y, text="DIST",
                           anchor="w", fill=_C["label"], font=(_FF, 9))
            cv.create_text(rx0 + 36,  r2y, text=dist_str,
                           anchor="w", fill=dist_col, font=(_FF, 10, "bold"))
            cv.create_text(rx0 + 116, r2y, text="BRG",
                           anchor="w", fill=_C["label"], font=(_FF, 9))
            cv.create_text(rx0 + 144, r2y, text=brg_str,
                           anchor="w", fill=dist_col, font=(_FF, 10, "bold"))

            if show_roses:
                sep2y = sep1y + sep_sp + rh + sep_sp // 2 + 1
                cv.create_line(bx + 3, sep2y, bx + box_w - 3, sep2y,
                               fill=_C["border"])
                rose_cy = sep2y + rose_sep_sp // 2 + rose_r + 4
                q1 = bx + box_w // 4
                q3 = bx + box_w - box_w // 4
                if q3 - q1 >= rose_diam * 2 + 20:
                    self._draw_hud_rose(q1, rose_cy, rose_r, "TRK",
                                        trk if raw_valid else None, _C["track_needle"])
                    self._draw_hud_rose(q3, rose_cy, rose_r, "HOME",
                                        brg360 if comp_v else None, _C["home_arrow"])

        # ═══════════════════════════════════════════════════════════════════
        # COMPACT tier  (380 <= w < 560)
        # ═══════════════════════════════════════════════════════════════════
        elif w >= 380:
            ip      = 8
            rh      = 20
            row_gap = 3
            sep_sp  = 6

            C_FIX    = 0
            C_LLBL   = 52
            C_NUM    = 76
            C_LAT_H  = 164
            C_LON    = 180
            C_LON_V  = 204
            C_LON_H  = 292
            ALT_W    = 80

            box_w = min(avail_w, 600)
            box_w = max(box_w, C_LAT_H + 12 + ALT_W + ip * 2)

            ix0 = pad + ip
            ix1 = pad + box_w - ip

            single_row = (ix0 + C_LON_H + 12 + ALT_W) <= ix1

            if single_row:
                core_h = ip + rh + sep_sp + rh + ip
            else:
                core_h = ip + rh + row_gap + rh + sep_sp + rh + ip

            show_roses = (avail_h - core_h) >= (rose_section_h + 8)
            box_h = core_h + (rose_section_h if show_roses else 0)

            bx = pad
            by = max(pad, h - pad - box_h)
            self._hud_box(bx, by, box_w, box_h)

            ix0 = bx + ip
            ix1 = bx + box_w - ip
            r1y = by + ip + rh // 2

            if single_row:
                cv.create_text(ix0 + C_FIX, r1y, text=fix_txt,
                               anchor="w", fill=fix_col, font=(_FF, 9, "bold"))
                cv.create_text(ix0 + C_LLBL, r1y, text="LAT",
                               anchor="w", fill=_C["label"], font=(_FF, 8))
                cv.create_text(ix0 + C_NUM, r1y, text=lat_num,
                               anchor="w", fill=pos_col, font=(_FF, 10, "bold"))
                cv.create_text(ix0 + C_LAT_H, r1y, text=lat_hem,
                               anchor="w", fill=_C["unit"], font=(_FF, 8, "bold"))
                cv.create_text(ix0 + C_LON, r1y, text="LON",
                               anchor="w", fill=_C["label"], font=(_FF, 8))
                cv.create_text(ix0 + C_LON_V, r1y, text=lon_num,
                               anchor="w", fill=pos_col, font=(_FF, 10, "bold"))
                cv.create_text(ix0 + C_LON_H, r1y, text=lon_hem,
                               anchor="w", fill=_C["unit"], font=(_FF, 8, "bold"))
                cv.create_text(ix1, r1y, text=f"ALT  {alt_str}",
                               anchor="e", fill=alt_col, font=(_FF, 9, "bold"))

                sep1y = by + ip + rh + sep_sp // 2
                r2y   = sep1y + sep_sp // 2 + rh // 2 + 1

            else:
                cv.create_text(ix0 + C_FIX, r1y, text=fix_txt,
                               anchor="w", fill=fix_col, font=(_FF, 9, "bold"))
                cv.create_text(ix0 + C_LLBL, r1y, text="LAT",
                               anchor="w", fill=_C["label"], font=(_FF, 8))
                cv.create_text(ix0 + C_NUM, r1y, text=lat_num,
                               anchor="w", fill=pos_col, font=(_FF, 10, "bold"))
                cv.create_text(ix0 + C_LAT_H, r1y, text=lat_hem,
                               anchor="w", fill=_C["unit"], font=(_FF, 8, "bold"))
                cv.create_text(ix1, r1y, text=f"ALT  {alt_str}",
                               anchor="e", fill=alt_col, font=(_FF, 9, "bold"))

                r2y_coord = r1y + rh + row_gap
                cv.create_text(ix0 + C_LLBL, r2y_coord, text="LON",
                               anchor="w", fill=_C["label"], font=(_FF, 8))
                cv.create_text(ix0 + C_NUM, r2y_coord, text=lon_num,
                               anchor="w", fill=pos_col, font=(_FF, 10, "bold"))
                cv.create_text(ix0 + C_LAT_H, r2y_coord, text=lon_hem,
                               anchor="w", fill=_C["unit"], font=(_FF, 8, "bold"))

                sep1y = by + ip + rh + row_gap + rh + sep_sp // 2
                r2y   = sep1y + sep_sp // 2 + rh // 2 + 1

            cv.create_line(bx + 3, sep1y, bx + box_w - 3, sep1y,
                           fill=_C["border"])

            half_x = bx + box_w // 2

            cv.create_text(ix0,       r2y, text="GS",
                           anchor="w", fill=_C["label"], font=(_FF, 8))
            cv.create_text(ix0 + 22,  r2y, text=gs_str,
                           anchor="w", fill=gs_col,  font=(_FF, 9, "bold"))
            cv.create_text(ix0 + 100, r2y, text="TRK",
                           anchor="w", fill=_C["label"], font=(_FF, 8))
            cv.create_text(ix0 + 124, r2y, text=trk_str,
                           anchor="w", fill=trk_col, font=(_FF, 9, "bold"))

            cv.create_line(half_x, sep1y + sep_sp // 2,
                           half_x, sep1y + sep_sp // 2 + rh + sep_sp,
                           fill=_C["border"])

            rx0 = half_x + ip
            cv.create_text(rx0,       r2y, text="DIST",
                           anchor="w", fill=_C["label"], font=(_FF, 8))
            cv.create_text(rx0 + 34,  r2y, text=dist_str,
                           anchor="w", fill=dist_col, font=(_FF, 9, "bold"))
            cv.create_text(rx0 + 104, r2y, text="BRG",
                           anchor="w", fill=_C["label"], font=(_FF, 8))
            cv.create_text(rx0 + 128, r2y, text=brg_str,
                           anchor="w", fill=dist_col, font=(_FF, 9, "bold"))

            if show_roses:
                sep2y = by + core_h - ip + rose_sep_sp // 2
                cv.create_line(bx + 3, sep2y, bx + box_w - 3, sep2y,
                               fill=_C["border"])
                rose_cy = sep2y + rose_sep_sp // 2 + rose_r + 4
                q1 = bx + box_w // 4
                q3 = bx + box_w - box_w // 4
                if q3 - q1 >= rose_diam * 2 + 20:
                    self._draw_hud_rose(q1, rose_cy, rose_r, "TRK",
                                        trk if raw_valid else None, _C["track_needle"])
                    self._draw_hud_rose(q3, rose_cy, rose_r, "HOME",
                                        brg360 if comp_v else None, _C["home_arrow"])

        # ═══════════════════════════════════════════════════════════════════
        # STACKED tier  (w < 380)
        # ═══════════════════════════════════════════════════════════════════
        else:
            ip    = 6
            rh    = 19
            sp    = 2
            lbl_w = 40
            box_w = min(avail_w, 220)

            all_rows = [
                ("",     fix_txt,  fix_col,  ""),
                ("LAT",  lat_num,  pos_col,  lat_hem),
                ("LON",  lon_num,  pos_col,  lon_hem),
                ("ALT",  alt_str,  alt_col,  ""),
                ("GS",   gs_str,   gs_col,   ""),
                ("TRK",  trk_str,  trk_col,  ""),
                ("DST",  dist_str, dist_col, ""),
                ("BRG",  brg_str,  dist_col, ""),
            ]

            max_rows = max(1, (avail_h - ip * 2 + sp) // (rh + sp))
            rows  = all_rows[:max_rows]
            n     = len(rows)
            box_h = ip + n * rh + (n - 1) * sp + ip

            bx = pad
            by = max(pad, h - pad - box_h)
            self._hud_box(bx, by, box_w, box_h)

            y_cur = by + ip

            for lbl, val, vcol, hem in rows:
                cy_r = y_cur + rh // 2
                vx   = bx + ip + (lbl_w if lbl else 0)
                if lbl:
                    cv.create_text(bx + ip, cy_r, text=lbl,
                                   anchor="w", fill=_C["label"], font=(_FF, 8))
                cv.create_text(vx, cy_r, text=val,
                               anchor="w", fill=vcol, font=(_FF, 9, "bold"))
                if hem:
                    cv.create_text(vx + 108, cy_r, text=hem,
                                   anchor="w", fill=_C["unit"], font=(_FF, 8))
                y_cur += rh + sp

    # ── Event handlers ────────────────────────────────────────────────────────

    def _on_resize(self, _): self._redraw()
    def _on_pan_end(self, _): self._pan_last = None

    def _on_pan_start(self, event):
        self._pan_last    = (event.x, event.y)
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
        if event.delta > 0: self._zoom_in()
        else:               self._zoom_out()

    def get_zoom(self): return self._zoom

    def _zoom_in(self):
        if self._zoom < 19:
            self._zoom += 1
            self._redraw()

    def _zoom_out(self):
        if self._zoom > 2:
            self._zoom -= 1
            self._redraw()

    def _set_center_for_drone(self, lat, lon, w, h):
        """
        Compute _center_lat/_center_lon so the drone's tile position lands at
        eff_center_y — the centre of the visible map strip above the HUD.

        _latlon_to_canvas uses eff_center_y = (h - hud_h) / 2 as its Y origin,
        so when cy_f == drone_yf the drone pixel Y is:
            eff_center_y + (drone_yf - drone_yf) * TILE_SIZE == eff_center_y  ✓

        No tile-space offset is needed; adding one was the source of the
        above-centre positioning bug.
        """
        drone_xf, drone_yf = self._deg2tile_f(lat, lon, self._zoom)
        n                   = 2 ** self._zoom
        self._center_lon    = drone_xf / n * 360.0 - 180.0
        self._center_lat    = math.degrees(
            math.atan(math.sinh(math.pi * (1.0 - 2.0 * drone_yf / n))))

    def _center_on_drone(self):
        if self._drone_lat is not None:
            w = self._cv.winfo_width()  or 400
            h = self._cv.winfo_height() or 300
            self._set_center_for_drone(self._drone_lat, self._drone_lon, w, h)
        self._auto_center = True
        self._redraw()

    def update_position(self, lat, lon, fix_valid):
        self._fix_valid = fix_valid
        if fix_valid and lat != 0.0:
            self._drone_lat = lat
            self._drone_lon = lon
            if self._auto_center:
                w = self._cv.winfo_width()  or 400
                h = self._cv.winfo_height() or 300
                self._set_center_for_drone(lat, lon, w, h)
        self._redraw()

    def update_hud(self, d: dict):
        self._hud = d
        self._redraw()


# =============================================================================
# Compact navigation panel  (shown when widget is too small for map)
# =============================================================================

class _NavPanel(tk.Frame):
    """
    Full-detail navigation readout — used when widget is too small for map.
    """

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._prev_heartbeat = None
        self._hb_state       = False
        self._last_d         = {}
        self._build_full()

    def set_sat_toggle_cmd(self, cmd):
        self._sat_btn.config(command=cmd)

    def update_sat_btn_text(self, text: str):
        self._sat_btn.config(text=text)

    def _sep(self, parent=None):
        p = parent or self
        tk.Frame(p, bg=_C["border"], height=1).pack(fill="x", pady=2)

    def _lrow(self, parent, label, attr, default_text, font=_FV):
        row = tk.Frame(parent, bg=_C["bg"])
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, fg=_C["label"], bg=_C["bg"],
                 font=_FLM, width=5, anchor="w").pack(side="left")
        lbl = tk.Label(row, text=default_text, fg=_C["dim"], bg=_C["bg"], font=font)
        lbl.pack(side="left")
        setattr(self, attr, lbl)

    _FIXROW_WIDE = 340

    def _build_full(self):
        c = tk.Frame(self, bg=_C["bg"])
        c.pack(fill="both", expand=True, padx=8, pady=4)

        self._fix_row_a = tk.Frame(c, bg=_C["bg"])
        self._fix_row_a.pack(fill="x", pady=(2, 0))

        self._fix_lbl = tk.Label(self._fix_row_a, text="NO FIX",
                                 fg=_C["red"], bg=_C["frame_bg"],
                                 font=(_FF, 11, "bold"),
                                 width=7, anchor="center",
                                 relief="flat", padx=4, pady=2)
        self._fix_lbl.pack(side="left")

        for txt, attr in (("SAT", "_sat_lbl"), ("HDOP", "_hdop_lbl")):
            f = tk.Frame(self._fix_row_a, bg=_C["bg"])
            f.pack(side="left", padx=(10, 0))
            tk.Label(f, text=txt, fg=_C["label"], bg=_C["bg"],
                     font=_FLM).pack(side="left")
            lbl = tk.Label(f, text=" --", fg=_C["dim"], bg=_C["bg"],
                           font=(_FF, 11, "bold"), width=5)
            lbl.pack(side="left")
            setattr(self, attr, lbl)

        self._hb_cv  = tk.Canvas(self._fix_row_a, width=12, height=12,
                                 bg=_C["bg"], highlightthickness=0)
        self._hb_cv.pack(side="left", padx=(8, 0))
        self._hb_dot = self._hb_cv.create_oval(1, 1, 11, 11,
                                               fill=_C["hb_off"], outline="")

        self._sat_btn = tk.Button(
            self._fix_row_a, text="SAT▶",
            bg=_C["map_btn"], fg=_C["amber"],
            relief="flat", padx=5, pady=1,
            font=(_FF, 8, "bold"),
            activebackground="#2a3545", activeforeground=_C["amber"],
            cursor="hand2",
        )
        self._sat_btn.pack(side="right", padx=(0, 2))

        self._fix_row_b    = tk.Frame(c, bg=_C["bg"])
        self._fix_row_wide = True
        c.bind("<Configure>", self._on_nav_configure, add="+")
        self._nav_c = c

        self._sep(c)
        tk.Label(c, text="POSITION", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        self._lrow(c, "LAT", "_lat_lbl", "---.---------- -")
        self._lrow(c, "LON", "_lon_lbl", "---.---------- -")

        self._sep(c)
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

        self._sep(c)
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

        self._sep(c)
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

    def _on_nav_configure(self, event):
        self._adapt_fix_rows(event.width)

    def _adapt_fix_rows(self, w):

        want_wide = w >= self._FIXROW_WIDE

        if want_wide == self._fix_row_wide:
            return

        self._fix_row_wide = want_wide

        # reset layout
        self._hb_cv.pack_forget()
        self._sat_btn.pack_forget()
        self._fix_row_b.pack_forget()

        # ─────────────────────────────────────────────
        # WIDE LAYOUT
        # everything stays in row A
        # ─────────────────────────────────────────────
        if want_wide:

            self._hb_cv.pack(
                side="left",
                padx=(8, 0),
                pady=2
            )

            self._sat_btn.pack(
                side="right",
                padx=(0, 2),
                pady=2
            )

        # ─────────────────────────────────────────────
        # COMPACT LAYOUT
        # second row visible
        # ─────────────────────────────────────────────
        else:

            self._fix_row_b.pack(
                fill="x",
                after=self._fix_row_a
            )

            self._hb_cv.pack(
                side="left",
                padx=(2, 0),
                pady=2
            )

            self._sat_btn.pack(
                side="left",
                padx=(8, 0),
                pady=2
            )

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

    def update(self, d: dict):
        self._last_d = d
        self._apply(d)

    def _apply(self, d: dict):
        raw_valid  = d.get("raw_valid",  False)
        comp_valid = d.get("comp_valid", False)
        pos_usable = d.get("pos_usable", False)
        fix_type   = d.get("fix_type",   0)
        num_sat    = d.get("num_sat",    0)
        hdop       = d.get("hdop",       99.0)
        lat        = d.get("lat",        0.0)
        lon        = d.get("lon",        0.0)
        alt_m      = d.get("alt_m",      0.0)
        gs_cms     = d.get("gs_cms",     0)
        trk        = d.get("trk",        0.0)
        dist_m     = d.get("dist_m",     0.0)
        brg        = d.get("brg",        0)
        heartbeat  = d.get("heartbeat",  None)

        if raw_valid and fix_type >= 2:
            self._fix_lbl.config(text="3D FIX", fg=_C["green"])
        elif raw_valid and fix_type == 1:
            self._fix_lbl.config(text="2D FIX", fg=_C["amber"])
        else:
            self._fix_lbl.config(text="NO FIX", fg=_C["red"])

        self._sat_lbl.config(
            text=f"{num_sat:3d}" if raw_valid else " --",
            fg=_C["green"] if (raw_valid and num_sat >= 6)
               else _C["amber"] if raw_valid else _C["dim"])

        if raw_valid and hdop < 99.0:
            hcol = (_C["green"] if hdop < 1.0
                    else _C["amber"] if hdop < 2.0 else _C["red"])
            self._hdop_lbl.config(text=f"{hdop:5.2f}", fg=hcol)
        else:
            self._hdop_lbl.config(text=" -.--", fg=_C["dim"])

        if heartbeat is not None and heartbeat != self._prev_heartbeat:
            self._hb_state       = not self._hb_state
            self._prev_heartbeat = heartbeat
        self._hb_cv.itemconfig(self._hb_dot,
                               fill=_C["hb_on"] if self._hb_state else _C["hb_off"])

        if raw_valid and fix_type >= 1:
            col = _C["text"] if pos_usable else _C["amber"]
            self._lat_lbl.config(
                text=f"{abs(lat):10.6f} {'N' if lat >= 0 else 'S'}", fg=col)
            self._lon_lbl.config(
                text=f"{abs(lon):10.6f} {'E' if lon >= 0 else 'W'}", fg=col)
        else:
            self._lat_lbl.config(text="---.---------- -", fg=_C["dim"])
            self._lon_lbl.config(text="---.---------- -", fg=_C["dim"])

        if raw_valid and fix_type >= 2:
            self._alt_ft_lbl.config(text=f"{alt_m * _M_TO_FT:7.0f} ft", fg=_C["text"])
            self._alt_m_lbl.config( text=f"({alt_m:6.0f} m)",             fg=_C["unit"])
        else:
            self._alt_ft_lbl.config(text="------ ft",  fg=_C["dim"])
            self._alt_m_lbl.config( text="(------ m)", fg=_C["dim"])

        if raw_valid:
            self._gs_kt_lbl.config(text=f"{gs_cms * _CMS_TO_KT:5.1f} kt",      fg=_C["text"])
            self._gs_km_lbl.config(text=f"({gs_cms * _CMS_TO_KMH:5.1f} km/h)", fg=_C["unit"])
        else:
            self._gs_kt_lbl.config(text="---.- kt",   fg=_C["dim"])
            self._gs_km_lbl.config(text="(--- km/h)", fg=_C["dim"])

        if raw_valid:
            self._trk_lbl.config(text=f"{trk:05.1f}°", fg=_C["text"])
            self._point_needle(self._trk_cv, self._trk_needle, 34, 34, 28, trk)
        else:
            self._trk_lbl.config(text="---.-°", fg=_C["dim"])
            self._point_needle(self._trk_cv, self._trk_needle, 34, 34, 28, 0)

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

    TOOLBAR (34 px tall) — visible ONLY in MAP mode.
    Contents: [+] [−] [⊙ CTR] | SAT n | HDOP x.xx  ●  [SAT▶]

    In NAV mode the toolbar is hidden entirely; a SAT▶ button is embedded
    directly in the nav panel's fix row instead.

    Layout modes (auto-selected by widget size):
      MAP MODE  — Full-width map with HUD overlay; SAT panel slides in right.
      NAV MODE  — Compact scrollable nav readout (no map, no toolbar).

    Public API
    ----------
    update_gps(ui_data)
    """

    _TB_LABELS_MIN = 360

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._sat_open     = False
        self._map_mode     = True
        self._last_ui_data = {}
        self._sat_toplevel = None
        self._toolbar_w    = -1

        self._prev_heartbeat = None
        self._hb_state       = False

        self._build_ui()
        self.bind("<Configure>", self._on_configure)

    def _build_ui(self):
        self._toolbar = tk.Frame(self, bg=_C["frame_bg"], height=34)
        self._toolbar.pack_propagate(False)

        btn_kw = dict(
            bg=_C["map_btn"], fg=_C["text"],
            relief="flat", padx=5, pady=2,
            font=_FTB,
            activebackground="#2a3545", activeforeground=_C["text"],
            cursor="hand2",
        )

        self._btn_plus = tk.Button(self._toolbar, text="+",
                                   command=self._zoom_in, **btn_kw)
        self._btn_plus.pack(side="left", padx=(4, 1), pady=3)

        self._btn_minus = tk.Button(self._toolbar, text="−",
                                    command=self._zoom_out, **btn_kw)
        self._btn_minus.pack(side="left", padx=(0, 1), pady=3)

        self._btn_ctr = tk.Button(self._toolbar, text="⊙",
                                  command=self._center_on_drone, **btn_kw)
        self._btn_ctr.pack(side="left", padx=(0, 0), pady=3)

        self._ctr_lbl = tk.Label(self._toolbar, text="CTR",
                                 fg=_C["text"], bg=_C["map_btn"],
                                 font=_FTB, padx=3, pady=2)
        self._ctr_lbl.pack(side="left", padx=(0, 4), pady=3)

        self._sep_v1 = tk.Frame(self._toolbar, bg=_C["border"], width=1)
        self._sep_v1.pack(side="left", fill="y", pady=4)

        self._sat_txt_lbl = tk.Label(self._toolbar, text="SAT",
                                     fg=_C["label"], bg=_C["frame_bg"],
                                     font=_FTB)

        self._sat_count_lbl = tk.Label(self._toolbar, text=" --",
                                       fg=_C["dim"], bg=_C["frame_bg"],
                                       font=(_FF, 10, "bold"), width=3)
        self._sat_count_lbl.pack(side="left", padx=(4, 0))

        self._sep_v2 = tk.Frame(self._toolbar, bg=_C["border"], width=1)
        self._sep_v2.pack(side="left", fill="y", pady=4, padx=(4, 0))

        self._hdop_txt_lbl = tk.Label(self._toolbar, text="HDOP",
                                      fg=_C["label"], bg=_C["frame_bg"],
                                      font=_FTB)

        self._hdop_lbl = tk.Label(self._toolbar, text=" --.--",
                                  fg=_C["dim"], bg=_C["frame_bg"],
                                  font=(_FF, 10, "bold"), width=6)
        self._hdop_lbl.pack(side="left", padx=(4, 4))

        self._sat_btn = tk.Button(
            self._toolbar, text="SAT▶",
            command=self._toggle_sat,
            bg=_C["map_btn"], fg=_C["amber"],
            relief="flat", padx=5, pady=2,
            font=(_FF, 8, "bold"),
            activebackground="#2a3545", activeforeground=_C["amber"],
            cursor="hand2",
        )
        self._sat_btn.pack(side="right", padx=(0, 5), pady=3)

        self._hb_cv  = tk.Canvas(self._toolbar, width=10, height=10,
                                 bg=_C["frame_bg"], highlightthickness=0)
        self._hb_cv.pack(side="right", padx=(0, 4))
        self._hb_dot = self._hb_cv.create_oval(1, 1, 9, 9,
                                               fill=_C["hb_off"], outline="")

        self._toolbar_border = tk.Frame(self, bg=_C["border"], height=1)

        self._content = tk.Frame(self, bg=_C["bg"])
        self._content.pack(fill="both", expand=True)

        self._build_map_layout()
        self._build_nav_layout()
        self._apply_map_mode()

    def _build_map_layout(self):
        self._map_frame  = tk.Frame(self._content, bg=_C["bg"])
        self._map_canvas = _MapCanvas(self._map_frame)
        self._map_canvas.pack(side="left", fill="both", expand=True)
        self._sat_panel  = _SatPanel(self._map_frame)

    def _build_nav_layout(self):
        self._nav_frame = tk.Frame(self._content, bg=_C["bg"])
        self._nav_panel = _NavPanel(self._nav_frame)
        self._nav_panel.pack(fill="both", expand=True)
        self._nav_panel.set_sat_toggle_cmd(self._toggle_sat)

    def _adapt_toolbar(self, w):
        if w == self._toolbar_w:
            return
        self._toolbar_w = w
        show_labels = w >= self._TB_LABELS_MIN
        self._sat_txt_lbl.pack_forget()
        self._hdop_txt_lbl.pack_forget()
        if show_labels:
            self._sat_txt_lbl.pack(in_=self._toolbar, side="left",
                                   padx=(4, 2), after=self._sep_v1)
            self._hdop_txt_lbl.pack(in_=self._toolbar, side="left",
                                    padx=(4, 2), after=self._sep_v2)

    def _zoom_in(self):
        if self._map_mode: self._map_canvas.zoom_in()

    def _zoom_out(self):
        if self._map_mode: self._map_canvas.zoom_out()

    def _center_on_drone(self):
        if self._map_mode: self._map_canvas.center_drone()

    def _update_toolbar_status(self, raw_valid, fix_type, num_sat, hdop):
        sat_col = (_C["green"] if (raw_valid and num_sat >= 6)
                   else _C["amber"] if raw_valid else _C["dim"])
        self._sat_count_lbl.config(
            text=f"{num_sat:3d}" if raw_valid else " --",
            fg=sat_col)
        if raw_valid and hdop < 99.0:
            hdop_col = (_C["green"] if hdop < 1.0
                        else _C["amber"] if hdop < 2.0 else _C["red"])
            self._hdop_lbl.config(text=f"{hdop:6.2f}", fg=hdop_col)
        else:
            self._hdop_lbl.config(text=" --.--", fg=_C["dim"])

    def _apply_map_mode(self):
        self._map_mode = True
        self._toolbar.pack(fill="x", before=self._content)
        self._toolbar_border.pack(fill="x", before=self._content)
        self._nav_frame.pack_forget()
        self._map_frame.pack(fill="both", expand=True)
        if self._sat_open:
            self._sat_panel.pack(side="right", fill="y")
        self._toolbar_w = -1
        self._adapt_toolbar(self.winfo_width())

    def _apply_nav_mode(self):
        self._map_mode = False
        self._toolbar.pack_forget()
        self._toolbar_border.pack_forget()
        self._map_frame.pack_forget()
        self._sat_panel.pack_forget()
        self._nav_frame.pack(fill="both", expand=True)

    def _on_configure(self, event):
        w = event.width
        h = event.height
        if self._map_mode:
            self._adapt_toolbar(w)
        want_map = (w >= MIN_MAP_W and h >= MIN_MAP_H + 35)
        if want_map != self._map_mode:
            if want_map: self._apply_map_mode()
            else:        self._apply_nav_mode()
            if self._last_ui_data:
                self._push_data(self._last_ui_data)

    def _sat_fits_inline(self) -> bool:
        w = self.winfo_width()
        return (w - _SatPanel.SAT_PANEL_W) >= MIN_MAP_W

    def _toggle_sat(self):
        self._sat_open = not self._sat_open
        btn_text = "SAT◀" if self._sat_open else "SAT▶"
        self._sat_btn.config(text=btn_text)
        self._nav_panel.update_sat_btn_text(btn_text)
        if self._sat_open:
            if self._map_mode:
                if self._sat_fits_inline():
                    self._sat_panel.pack(side="right", fill="y")
                else:
                    self._open_sat_toplevel()
            else:
                self._open_sat_toplevel()
        else:
            if self._map_mode:
                self._sat_panel.pack_forget()
            self._close_sat_toplevel()

    def _open_sat_toplevel(self):
        if self._sat_toplevel and self._sat_toplevel.winfo_exists():
            return
        tl = tk.Toplevel(self)
        tl.title("Satellite Constellation")
        tl.configure(bg=_C["frame_bg"])
        tl.resizable(False, True)
        try:
            wx = self.winfo_rootx()
            wy = self.winfo_rooty()
            ww = self.winfo_width()
            wh = self.winfo_height()
            tl.geometry(f"{_SatPanel.SAT_PANEL_W + 20}x{min(wh, 520)}"
                        f"+{wx + ww - _SatPanel.SAT_PANEL_W - 24}+{wy}")
        except Exception:
            tl.geometry(f"{_SatPanel.SAT_PANEL_W + 20}x400")
        tl.protocol("WM_DELETE_WINDOW", self._close_sat_toplevel)
        panel = _SatPanel(tl)
        panel.pack(fill="both", expand=True, padx=4, pady=4)
        self._sat_toplevel_panel = panel
        self._sat_toplevel       = tl
        if self._last_ui_data:
            raw  = self._last_ui_data.get("gps_sv_list", [])
            norm = _normalize_sv_list(raw)
            panel.update_satellites(norm)

    def _close_sat_toplevel(self):
        if self._sat_toplevel and self._sat_toplevel.winfo_exists():
            self._sat_toplevel.destroy()
        self._sat_toplevel = None
        if self._sat_open:
            self._sat_open = False
            btn_text = "SAT▶"
            self._sat_btn.config(text=btn_text)
            self._nav_panel.update_sat_btn_text(btn_text)

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

        nav = dict(
            raw_valid=raw_valid, comp_valid=comp_valid,
            pos_usable=pos_usable, fix_type=fix_type,
            num_sat=num_sat, hdop=hdop,
            lat=lat, lon=lon, alt_m=alt_m,
            gs_cms=gs_cms, trk=trk,
            dist_m=dist_m, brg=brg,
            heartbeat=heartbeat,
        )

        if heartbeat is not None and heartbeat != self._prev_heartbeat:
            self._hb_state       = not self._hb_state
            self._prev_heartbeat = heartbeat
        self._hb_cv.itemconfig(
            self._hb_dot,
            fill=_C["hb_on"] if self._hb_state else _C["hb_off"])

        if self._map_mode:
            self._update_toolbar_status(raw_valid, fix_type, num_sat, hdop)
            self._map_canvas.update_position(lat, lon, raw_valid and fix_type >= 2)
            self._map_canvas.update_hud(nav)
        else:
            self._nav_panel.update(nav)

        raw_sv = ui_data.get("gps_sv_list", [])
        norm   = _normalize_sv_list(raw_sv)
        self._sat_panel.update_satellites(norm)
        if (self._sat_toplevel and self._sat_toplevel.winfo_exists()
                and hasattr(self, "_sat_toplevel_panel")):
            self._sat_toplevel_panel.update_satellites(norm)