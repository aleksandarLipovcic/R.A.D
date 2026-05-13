"""
GPSWidget.py
============
Aviation-grade GPS navigation widget — three tabbed views:

  [Navigation]  Classic numeric readout (lat/lon, alt, speed, track, home)
  [Satellites]  Per-satellite signal-strength bars (Betaflight-style)
  [Map]         Live OpenStreetMap tile map with drone position marker

All three tabs are updated on every call to update_gps(ui_data).

Map notes
---------
• Tiles are fetched from tile.openstreetmap.org in daemon threads and
  cached in memory.  The GCS machine needs internet access.
• Pan with left-click drag.  Zoom with scroll wheel or +/- buttons.
• "⊙ Center" snaps the view back to the drone.
• If the GPS has no fix the map shows a "Waiting for GPS fix" overlay.

Satellite tab notes
-------------------
• Populated when ui_data["gps_sv_list"] is a non-empty list.
• Each item must be a dict with keys:
      gnss_id (str)   e.g. "GPS", "GLONASS", "BeiDou", "SBAS"
      sv_id   (int)   satellite PRN / slot number
      cno     (int)   carrier-to-noise ratio in dB-Hz  (0 = no signal)
      used    (bool)  contributing to the current fix
      quality (int 0-7 or str)  0=idle … 7=fully locked
• If the list is empty a "No satellite data — upgrade backend" hint is shown.
"""

import tkinter as tk
from tkinter import ttk
import math
import threading
import queue
import urllib.request
import base64


# ── Unit conversion constants ─────────────────────────────────────────────────
_CMS_TO_KT  = 0.019438
_CMS_TO_KMH = 0.036
_M_TO_FT    = 3.28084

# ── Colour palette ────────────────────────────────────────────────────────────
_C = {
    "bg":           "#1a1e24",
    "frame_bg":     "#21262e",
    "border":       "#2e3540",
    "text":         "#d8dde6",
    "label":        "#5a6370",
    "unit":         "#7a8898",
    "green":        "#39e07a",
    "amber":        "#f0b429",
    "red":          "#f04040",
    "cyan":         "#29c7e0",
    "compass_bg":   "#151920",
    "compass_rim":  "#3a4555",
    "track_needle": "#39e07a",
    "home_arrow":   "#29c7e0",
    "no_data":      "#3a4555",
    "hb_on":        "#39e07a",
    "hb_off":       "#2e3540",
    # satellite colours
    "sv_row_even":  "#1a1e24",
    "sv_row_odd":   "#1f242c",
    "sv_used":      "#39e07a",
    "sv_locked":    "#f0b429",
    "sv_nodata":    "#3a4555",
    # map colours
    "map_bg":       "#2a3040",
    "map_btn":      "#2e3540",
    "map_marker":   "#f0b429",
    "map_home":     "#29c7e0",
    "map_acc_ring": "#39e07a",
}

_FF   = "Courier New"
_FL   = (_FF,  8, "bold")
_FV   = (_FF, 13, "bold")
_FVS  = (_FF, 10, "bold")
_FU   = (_FF,  8)
_FFIX = (_FF, 11, "bold")


# =============================================================================
# Map canvas widget
# =============================================================================

class _MapCanvas(tk.Frame):
    """
    Lightweight OSM tile map.
    Tiles are 256×256 PNG from tile.openstreetmap.org, fetched in daemon
    threads and cached as tk.PhotoImage objects (kept alive in a dict).
    """
    TILE_SIZE = 256
    _OSM_URL  = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    _UA       = "DroneCockpitGCS/1.0 (github educational project)"

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._zoom        = 15
        self._center_lat  = 44.77    # sensible default until first fix
        self._center_lon  = 17.21
        self._drone_lat   = None
        self._drone_lon   = None
        self._fix_valid   = False
        self._auto_center = True     # follow drone until user pans

        self._tile_img    = {}       # {(z,x,y): PhotoImage}  keep refs!
        self._tile_queue  = queue.Queue()
        self._tile_pend   = set()

        self._pan_last    = None

        self._build()
        self._poll_tiles()

    # ── Build UI ──────────────────────────────────────────────────────────────

    def _build(self):
        ctrl = tk.Frame(self, bg=_C["bg"])
        ctrl.pack(side="top", fill="x", padx=2, pady=2)

        btn_kw = dict(
            bg=_C["map_btn"], fg=_C["text"],
            relief="flat", padx=6, pady=2,
            font=(_FF, 9, "bold"),
            activebackground="#3a4555", activeforeground=_C["text"],
            cursor="hand2",
        )
        tk.Button(ctrl, text="＋", command=self._zoom_in,          **btn_kw).pack(side="left", padx=(0, 2))
        tk.Button(ctrl, text="－", command=self._zoom_out,         **btn_kw).pack(side="left", padx=(0, 2))
        tk.Button(ctrl, text="⊙ Center", command=self._center_on_drone, **btn_kw).pack(side="left")

        self._zoom_lbl = tk.Label(
            ctrl, text=f"z{self._zoom}",
            fg=_C["label"], bg=_C["bg"], font=_FL)
        self._zoom_lbl.pack(side="right", padx=4)

        self._cv = tk.Canvas(self, bg=_C["map_bg"],
                             highlightthickness=0, cursor="fleur")
        self._cv.pack(fill="both", expand=True)

        self._cv.bind("<Configure>",       self._on_resize)
        self._cv.bind("<ButtonPress-1>",   self._on_pan_start)
        self._cv.bind("<B1-Motion>",       self._on_pan)
        self._cv.bind("<ButtonRelease-1>", self._on_pan_end)
        self._cv.bind("<MouseWheel>",      self._on_scroll)
        self._cv.bind("<Button-4>",        lambda e: self._zoom_in())   # Linux
        self._cv.bind("<Button-5>",        lambda e: self._zoom_out())  # Linux

        # Overlay text (shown when no fix)
        self._overlay = self._cv.create_text(
            10, 10, anchor="nw",
            text="Waiting for GPS fix…",
            fill=_C["amber"], font=(_FF, 10))

    # ── Tile math ─────────────────────────────────────────────────────────────

    @staticmethod
    def _deg2tile_f(lat, lon, zoom):
        """Return fractional tile coordinates for lat/lon at zoom."""
        n     = 2 ** zoom
        x_f   = (lon + 180.0) / 360.0 * n
        lat_r = math.radians(lat)
        y_f   = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
        return x_f, y_f

    def _center_tile_f(self):
        return self._deg2tile_f(self._center_lat, self._center_lon, self._zoom)

    def _latlon_to_canvas(self, lat, lon):
        """Map lat/lon to canvas pixel coordinates."""
        w  = self._cv.winfo_width()  or 400
        h  = self._cv.winfo_height() or 300
        cx, cy = self._center_tile_f()
        xf, yf = self._deg2tile_f(lat, lon, self._zoom)
        px = w / 2 + (xf - cx) * self.TILE_SIZE
        py = h / 2 + (yf - cy) * self.TILE_SIZE
        return px, py

    # ── Tile fetching ─────────────────────────────────────────────────────────

    def _request_tile(self, z, x, y):
        key = (z, x, y)
        if key in self._tile_img or key in self._tile_pend:
            return
        self._tile_pend.add(key)
        threading.Thread(
            target=self._fetch_tile, args=(z, x, y), daemon=True
        ).start()

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
        """Drain the tile queue and redraw if any new tiles arrived."""
        changed = False
        try:
            while True:
                key, b64 = self._tile_queue.get_nowait()
                if b64:
                    # PhotoImage must be created on the main thread
                    self._tile_img[key] = tk.PhotoImage(data=b64)
                    changed = True
        except queue.Empty:
            pass
        if changed:
            self._redraw()
        self.after(250, self._poll_tiles)

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _redraw(self):
        if not self.winfo_ismapped():
            return
        w = self._cv.winfo_width()
        h = self._cv.winfo_height()
        if w < 4 or h < 4:
            return

        self._cv.delete("tile", "marker")

        n    = 2 ** self._zoom
        cx_f, cy_f = self._center_tile_f()
        cx_ti = int(cx_f)
        cy_ti = int(cy_f)

        tiles_x = w // self.TILE_SIZE + 3
        tiles_y = h // self.TILE_SIZE + 3

        for dx in range(-tiles_x // 2 - 1, tiles_x // 2 + 2):
            for dy in range(-tiles_y // 2 - 1, tiles_y // 2 + 2):
                tx = (cx_ti + dx) % n
                ty = cy_ti + dy
                if ty < 0 or ty >= n:
                    continue
                px = w / 2 + (cx_ti + dx - cx_f) * self.TILE_SIZE
                py = h / 2 + (cy_ti + dy - cy_f) * self.TILE_SIZE
                key = (self._zoom, tx, ty)
                if key in self._tile_img:
                    self._cv.create_image(px, py, anchor="nw",
                                          image=self._tile_img[key],
                                          tags="tile")
                else:
                    self._cv.create_rectangle(
                        px, py,
                        px + self.TILE_SIZE, py + self.TILE_SIZE,
                        fill="#2a3040", outline="#3a4555", tags="tile")
                    self._request_tile(self._zoom, tx, ty)

        # Drone marker
        if self._fix_valid and self._drone_lat is not None:
            mx, my = self._latlon_to_canvas(self._drone_lat, self._drone_lon)
            r = 8
            self._cv.create_oval(mx - r, my - r, mx + r, my + r,
                                 fill=_C["map_marker"], outline="#ffffff",
                                 width=1.5, tags="marker")
            self._cv.create_line(mx, my - r, mx, my - r - 12,
                                 fill="#ffffff", width=2,
                                 arrow="last", arrowshape=(6, 8, 3),
                                 tags="marker")

        # Overlay
        if self._fix_valid:
            self._cv.itemconfig(self._overlay, state="hidden")
        else:
            self._cv.itemconfig(self._overlay,
                                text="Waiting for GPS fix…", state="normal")
            self._cv.tag_raise(self._overlay)

    # ── Event handlers ────────────────────────────────────────────────────────

    def _on_resize(self, _event):
        self._redraw()

    def _on_pan_start(self, event):
        self._pan_last    = (event.x, event.y)
        self._auto_center = False

    def _on_pan(self, event):
        if self._pan_last is None:
            return
        dx = event.x - self._pan_last[0]
        dy = event.y - self._pan_last[1]
        self._pan_last = (event.x, event.y)
        n      = 2 ** self._zoom
        cx_f, cy_f = self._center_tile_f()
        new_xf = cx_f - dx / self.TILE_SIZE
        new_yf = cy_f - dy / self.TILE_SIZE
        self._center_lon = new_xf / n * 360.0 - 180.0
        self._center_lat = math.degrees(
            math.atan(math.sinh(math.pi * (1.0 - 2.0 * new_yf / n))))
        self._redraw()

    def _on_pan_end(self, _event):
        self._pan_last = None

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

    def update_position(self, lat: float, lon: float, fix_valid: bool):
        self._fix_valid = fix_valid
        if fix_valid and lat != 0.0:
            self._drone_lat = lat
            self._drone_lon = lon
            if self._auto_center:
                self._center_lat = lat
                self._center_lon = lon
        self._redraw()


# =============================================================================
# Satellite signal canvas widget
# =============================================================================

class _SatCanvas(tk.Frame):
    """
    Scrollable, canvas-drawn satellite signal-strength display.
    Mimics the Betaflight GPS signal strength panel.
    """
    _ROW_H   = 23
    _CNO_MAX = 55        # typical maximum C/N0 in dB-Hz
    _BAR_MAX = 88        # pixel width of the full-scale signal bar

    # GNSS constellation short-name colours
    _GNSS_COLOR = {
        "GPS":     "#39e07a",
        "GLONASS": "#f0b429",
        "BeiDou":  "#29c7e0",
        "Galileo": "#cc88ff",
        "SBAS":    "#aaaaaa",
        "QZSS":    "#ff8844",
    }

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._sv_data = []
        self._build()

    def _build(self):
        # Fixed header row
        hdr = tk.Canvas(self, bg=_C["frame_bg"], height=18,
                        highlightthickness=0)
        hdr.pack(fill="x")
        self._header_cv = hdr
        self._draw_header(hdr)

        # Scrollable body
        body = tk.Frame(self, bg=_C["bg"])
        body.pack(fill="both", expand=True)

        self._cv = tk.Canvas(body, bg=_C["bg"], highlightthickness=0)
        sb = ttk.Scrollbar(body, orient="vertical",
                           command=self._cv.yview)
        self._cv.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._cv.pack(side="left", fill="both", expand=True)

        self._cv.bind("<Configure>",  lambda _e: self._redraw())
        self._cv.bind("<MouseWheel>", self._on_scroll)
        self._cv.bind("<Button-4>",   lambda _e: self._cv.yview_scroll(-1, "units"))
        self._cv.bind("<Button-5>",   lambda _e: self._cv.yview_scroll( 1, "units"))

    def _draw_header(self, cv):
        cv.delete("all")
        w = cv.winfo_width() or 360
        col_fg = _C["label"]
        font   = (_FF, 7, "bold")
        x = 6
        for label, width in self._columns():
            cv.create_text(x, 9, text=label, anchor="w",
                           fill=col_fg, font=font)
            x += width

    @staticmethod
    def _columns():
        return [
            ("GNSS",   62),
            ("SV",     32),
            ("Signal", 96),
            ("dBHz",   38),
            ("Status", 60),
            ("Quality",80),
        ]

    def _on_scroll(self, event):
        self._cv.yview_scroll(int(-event.delta / 120), "units")

    def update_satellites(self, sv_list: list):
        self._sv_data = sv_list or []
        self._redraw()

    def _redraw(self):
        self._cv.delete("all")
        self._draw_header(self._header_cv)

        w = max(self._cv.winfo_width(), 360)

        if not self._sv_data:
            self._cv.create_text(
                w // 2, 50,
                text="No satellite data available\n"
                     "(expose sv_list in DroneBackend to populate)",
                fill=_C["no_data"], font=(_FF, 9),
                justify="center")
            self._cv.configure(scrollregion=(0, 0, w, 100))
            return

        rows = sorted(self._sv_data,
                      key=lambda s: (not s.get("used", False),
                                     -int(s.get("cno", 0))))
        total_h = len(rows) * self._ROW_H + 2
        self._cv.configure(scrollregion=(0, 0, w, total_h))

        cols = self._columns()

        for i, sv in enumerate(rows):
            y      = i * self._ROW_H
            row_bg = _C["sv_row_odd"] if i % 2 else _C["sv_row_even"]
            self._cv.create_rectangle(0, y, w, y + self._ROW_H,
                                      fill=row_bg, outline="")

            x = 6
            cy = y + self._ROW_H // 2

            # ── GNSS constellation ────────────────────────────────────────────
            gnss  = str(sv.get("gnss_id", "?"))[:8]
            gcol  = self._GNSS_COLOR.get(gnss, _C["text"])
            self._cv.create_text(x, cy, text=gnss, anchor="w",
                                 fill=gcol, font=(_FF, 8, "bold"))
            x += cols[0][1]

            # ── SV ID ─────────────────────────────────────────────────────────
            self._cv.create_text(x, cy, text=str(sv.get("sv_id", "--")),
                                 anchor="w", fill=_C["text"], font=(_FF, 9))
            x += cols[1][1]

            # ── Signal bar ────────────────────────────────────────────────────
            cno     = int(sv.get("cno", 0))
            used    = bool(sv.get("used", False))
            bar_pct = min(cno / self._CNO_MAX, 1.0)
            bar_w   = int(bar_pct * self._BAR_MAX)
            bar_col = (_C["sv_used"] if used
                       else _C["sv_locked"] if cno > 12
                       else _C["sv_nodata"])
            track_h = 6
            by      = cy - track_h // 2
            # track
            self._cv.create_rectangle(
                x, by, x + self._BAR_MAX, by + track_h,
                fill=_C["border"], outline="")
            # fill
            if bar_w > 0:
                self._cv.create_rectangle(
                    x, by, x + bar_w, by + track_h,
                    fill=bar_col, outline="")
            x += cols[2][1]

            # ── CNO value ─────────────────────────────────────────────────────
            cno_txt = f"{cno:2d}" if cno > 0 else "--"
            self._cv.create_text(x, cy, text=cno_txt, anchor="w",
                                 fill=_C["text"], font=(_FF, 9))
            x += cols[3][1]

            # ── Used / unused badge ───────────────────────────────────────────
            used_txt = "USED"   if used else "unused"
            used_col = _C["sv_used"] if used else _C["no_data"]
            self._cv.create_text(x, cy, text=used_txt, anchor="w",
                                 fill=used_col, font=(_FF, 8, "bold"))
            x += cols[4][1]

            # ── Quality ───────────────────────────────────────────────────────
            qual = sv.get("quality", 0)
            if isinstance(qual, int):
                _Q = ["idle", "searching", "acquired", "detected",
                      "code lock", "carrier lock", "fully locked", "fully locked"]
                qual = _Q[min(qual, 7)]
            qual_str = str(qual)[:14]
            if "fully" in qual_str.lower():
                qcol = _C["green"]
            elif "lock" in qual_str.lower() or "carrier" in qual_str.lower():
                qcol = _C["amber"]
            elif "code" in qual_str.lower():
                qcol = _C["amber"]
            else:
                qcol = _C["no_data"]
            self._cv.create_text(x, cy, text=qual_str, anchor="w",
                                 fill=qcol, font=(_FF, 8))


# =============================================================================
# Main GPS widget (Notebook with three tabs)
# =============================================================================

class GPSWidget(tk.Frame):
    """
    Self-contained GPS widget with three tabbed views.

    update_gps(ui_data) must be called on every telemetry tick.

    Expected ui_data keys (all optional):
        (navigation)
        gps_fix_type          int
        gps_num_sat           int
        gps_hdop              float   real HDOP (pre-divided)
        gps_latitude          float   decimal degrees
        gps_longitude         float   decimal degrees
        gps_altitude_m        float   MSL metres
        gps_ground_speed_cms  int     cm/s
        gps_ground_course     int     decidegrees
        gps_dist_to_home_m    float   metres
        gps_bearing_to_home   int     degrees (−180…+180)
        gps_heartbeat         int     toggles on each new GPS frame
        gps_raw_valid         bool
        gps_comp_valid        bool
        gps_position_usable   bool
        (satellite tab)
        gps_sv_list           list[dict]  see _SatCanvas docstring
    """

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_C["bg"], **kwargs)
        self._prev_heartbeat = None
        self._hb_state       = False
        self._build_ui()

    # =========================================================================
    # Build UI
    # =========================================================================

    def _build_ui(self):
        style = ttk.Style()
        style.configure("GPS.TLabelframe",
                        background=_C["bg"], bordercolor=_C["border"])
        style.configure("GPS.TLabelframe.Label",
                        background=_C["bg"], foreground=_C["label"],
                        font=(_FF, 9, "bold"))
        style.configure("GPSNB.TNotebook",
                        background=_C["bg"], borderwidth=0)
        style.configure("GPSNB.TNotebook.Tab",
                        background=_C["frame_bg"],
                        foreground=_C["label"],
                        font=(_FF, 8, "bold"),
                        padding=(8, 3))
        style.map("GPSNB.TNotebook.Tab",
                  background=[("selected", _C["border"])],
                  foreground=[("selected", _C["text"])])

        outer = ttk.LabelFrame(self, text="GPS Navigation",
                               style="GPS.TLabelframe")
        outer.pack(fill="both", expand=True, padx=2, pady=2)

        nb = ttk.Notebook(outer, style="GPSNB.TNotebook")
        nb.pack(fill="both", expand=True, padx=4, pady=(2, 4))

        # ── Tab 1: Navigation ─────────────────────────────────────────────────
        nav_frame = tk.Frame(nb, bg=_C["bg"])
        nb.add(nav_frame, text=" Navigation ")
        self._build_navigation(nav_frame)

        # ── Tab 2: Satellites ─────────────────────────────────────────────────
        sat_frame = tk.Frame(nb, bg=_C["bg"])
        nb.add(sat_frame, text=" Satellites ")
        self._sat_canvas = _SatCanvas(sat_frame)
        self._sat_canvas.pack(fill="both", expand=True)

        # ── Tab 3: Map ────────────────────────────────────────────────────────
        map_frame = tk.Frame(nb, bg=_C["bg"])
        nb.add(map_frame, text=" Map ")
        self._map_canvas = _MapCanvas(map_frame)
        self._map_canvas.pack(fill="both", expand=True)

    # =========================================================================
    # Navigation tab content  (same layout as the original GPSWidget)
    # =========================================================================

    def _build_navigation(self, p):
        c = tk.Frame(p, bg=_C["bg"])
        c.pack(fill="both", expand=True, padx=6, pady=4)

        self._build_fix_row(c)
        self._sep(c)
        self._build_position(c)
        self._sep(c)
        self._build_altitude(c)
        self._sep(c)
        self._build_ground_vector(c)
        self._sep(c)
        self._build_home(c)

    def _sep(self, parent):
        tk.Frame(parent, bg=_C["border"], height=1).pack(fill="x", pady=3)

    # ── Fix status row ────────────────────────────────────────────────────────

    def _build_fix_row(self, p):
        row = tk.Frame(p, bg=_C["bg"])
        row.pack(fill="x", pady=(2, 0))

        self._fix_lbl = tk.Label(
            row, text="NO FIX", fg=_C["red"], bg=_C["frame_bg"],
            font=_FFIX, width=7, anchor="center",
            relief="flat", padx=4, pady=2)
        self._fix_lbl.pack(side="left")

        sat_f = tk.Frame(row, bg=_C["bg"])
        sat_f.pack(side="left", padx=(10, 0))
        tk.Label(sat_f, text="SAT", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(side="left")
        self._sat_lbl = tk.Label(sat_f, text=" --",
                                 fg=_C["no_data"], bg=_C["bg"],
                                 font=_FVS, width=3)
        self._sat_lbl.pack(side="left")

        hdop_f = tk.Frame(row, bg=_C["bg"])
        hdop_f.pack(side="left", padx=(10, 0))
        tk.Label(hdop_f, text="HDOP", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(side="left")
        self._hdop_lbl = tk.Label(hdop_f, text=" -.--",
                                  fg=_C["no_data"], bg=_C["bg"],
                                  font=_FVS, width=5)
        self._hdop_lbl.pack(side="left")

        self._hb_cv = tk.Canvas(row, width=12, height=12,
                                bg=_C["bg"], highlightthickness=0)
        self._hb_cv.pack(side="right", padx=(0, 4))
        self._hb_dot = self._hb_cv.create_oval(
            1, 1, 11, 11, fill=_C["hb_off"], outline="")

    # ── Position ──────────────────────────────────────────────────────────────

    def _build_position(self, p):
        tk.Label(p, text="POSITION", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        lat_r = tk.Frame(p, bg=_C["bg"])
        lat_r.pack(fill="x")
        tk.Label(lat_r, text="LAT", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=4, anchor="w").pack(side="left")
        self._lat_lbl = tk.Label(lat_r, text="---.---------- -",
                                 fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._lat_lbl.pack(side="left")

        lon_r = tk.Frame(p, bg=_C["bg"])
        lon_r.pack(fill="x")
        tk.Label(lon_r, text="LON", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=4, anchor="w").pack(side="left")
        self._lon_lbl = tk.Label(lon_r, text="---.---------- -",
                                 fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._lon_lbl.pack(side="left")

    # ── Altitude ──────────────────────────────────────────────────────────────

    def _build_altitude(self, p):
        tk.Label(p, text="ALTITUDE MSL", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        alt_r = tk.Frame(p, bg=_C["bg"])
        alt_r.pack(fill="x")
        self._alt_ft_lbl = tk.Label(alt_r, text="------ ft",
                                    fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._alt_ft_lbl.pack(side="left")
        self._alt_m_lbl = tk.Label(alt_r, text="(------ m)",
                                   fg=_C["unit"], bg=_C["bg"], font=_FVS)
        self._alt_m_lbl.pack(side="left", padx=(8, 0))

    # ── Ground vector ─────────────────────────────────────────────────────────

    def _build_ground_vector(self, p):
        tk.Label(p, text="GROUND VECTOR", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        inner = tk.Frame(p, bg=_C["bg"])
        inner.pack(fill="x")
        left = tk.Frame(inner, bg=_C["bg"])
        left.pack(side="left", anchor="n")

        gs_kt_r = tk.Frame(left, bg=_C["bg"])
        gs_kt_r.pack(anchor="w")
        tk.Label(gs_kt_r, text="GS", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=3, anchor="w").pack(side="left")
        self._gs_kt_lbl = tk.Label(gs_kt_r, text="---.- kt",
                                   fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._gs_kt_lbl.pack(side="left")

        gs_km_r = tk.Frame(left, bg=_C["bg"])
        gs_km_r.pack(anchor="w")
        tk.Label(gs_km_r, text="  ", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=3, anchor="w").pack(side="left")
        self._gs_km_lbl = tk.Label(gs_km_r, text="(--- km/h)",
                                   fg=_C["unit"], bg=_C["bg"], font=_FVS)
        self._gs_km_lbl.pack(side="left")

        trk_r = tk.Frame(left, bg=_C["bg"])
        trk_r.pack(anchor="w", pady=(4, 0))
        tk.Label(trk_r, text="TRK", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=3, anchor="w").pack(side="left")
        self._trk_lbl = tk.Label(trk_r, text="---.-°",
                                 fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._trk_lbl.pack(side="left")

        self._trk_cv = tk.Canvas(inner, width=68, height=68,
                                 bg=_C["compass_bg"], highlightthickness=1,
                                 highlightbackground=_C["compass_rim"])
        self._trk_cv.pack(side="right", padx=(8, 0))
        self._draw_rose(self._trk_cv, 34, 34, 28, cardinals=True)
        self._trk_needle = self._trk_cv.create_line(
            34, 34, 34, 10, fill=_C["track_needle"],
            width=2, arrow="last", arrowshape=(7, 9, 3))

    # ── Home ──────────────────────────────────────────────────────────────────

    def _build_home(self, p):
        tk.Label(p, text="HOME POINT", fg=_C["label"], bg=_C["bg"],
                 font=_FL).pack(anchor="w")
        inner = tk.Frame(p, bg=_C["bg"])
        inner.pack(fill="x", pady=(0, 2))
        left = tk.Frame(inner, bg=_C["bg"])
        left.pack(side="left", anchor="n")

        dist_r = tk.Frame(left, bg=_C["bg"])
        dist_r.pack(anchor="w")
        tk.Label(dist_r, text="DIST", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=4, anchor="w").pack(side="left")
        self._dist_lbl = tk.Label(dist_r, text="------ m",
                                  fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._dist_lbl.pack(side="left")

        brg_r = tk.Frame(left, bg=_C["bg"])
        brg_r.pack(anchor="w")
        tk.Label(brg_r, text="BRG", fg=_C["label"], bg=_C["bg"],
                 font=_FL, width=4, anchor="w").pack(side="left")
        self._brg_lbl = tk.Label(brg_r, text="---°",
                                 fg=_C["no_data"], bg=_C["bg"], font=_FV)
        self._brg_lbl.pack(side="left")

        self._home_cv = tk.Canvas(inner, width=68, height=68,
                                  bg=_C["compass_bg"], highlightthickness=1,
                                  highlightbackground=_C["compass_rim"])
        self._home_cv.pack(side="right", padx=(8, 0))
        self._draw_rose(self._home_cv, 34, 34, 28, cardinals=False)
        self._home_arrow = self._home_cv.create_line(
            34, 34, 34, 10, fill=_C["home_arrow"],
            width=2, arrow="last", arrowshape=(7, 9, 3))

    # =========================================================================
    # Canvas helpers
    # =========================================================================

    def _draw_rose(self, cv, cx, cy, r, cardinals=True):
        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       outline=_C["compass_rim"], width=1)
        cv.create_oval(cx - 2, cy - 2, cx + 2, cy + 2,
                       fill=_C["compass_rim"], outline="")
        for deg in range(0, 360, 45):
            rad = math.radians(deg - 90)
            r1  = r - 5
            cv.create_line(
                cx + math.cos(rad) * r1, cy + math.sin(rad) * r1,
                cx + math.cos(rad) * r,  cy + math.sin(rad) * r,
                fill=_C["compass_rim"], width=1)
        cv.create_line(cx, cy - r + 5, cx, cy - r,
                       fill=_C["amber"], width=2)
        if cardinals:
            r_lbl = r - 11
            for lbl, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
                rad = math.radians(deg - 90)
                cv.create_text(
                    cx + math.cos(rad) * r_lbl,
                    cy + math.sin(rad) * r_lbl,
                    text=lbl, fill=_C["label"], font=(_FF, 6))

    def _point_needle(self, cv, needle_id, cx, cy, r, angle_deg):
        rad   = math.radians(angle_deg - 90)
        tip_r = r - 4
        cv.coords(needle_id,
                  cx, cy,
                  cx + math.cos(rad) * tip_r,
                  cy + math.sin(rad) * tip_r)

    # =========================================================================
    # Public update method
    # =========================================================================

    def update_gps(self, ui_data: dict):
        """Refresh all three tabs from the latest telemetry data dict."""
        self._update_navigation(ui_data)
        self._update_satellites(ui_data)
        self._update_map(ui_data)

    # ── Navigation tab update ─────────────────────────────────────────────────

    def _update_navigation(self, ui_data: dict):
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

        # Fix badge
        if raw_valid and fix_type >= 2:
            self._fix_lbl.config(text="3D FIX", fg=_C["green"])
        elif raw_valid and fix_type == 1:
            self._fix_lbl.config(text="2D FIX", fg=_C["amber"])
        else:
            self._fix_lbl.config(text="NO FIX", fg=_C["red"])

        # Satellites
        if raw_valid:
            self._sat_lbl.config(
                text=f"{num_sat:3d}",
                fg=_C["green"] if num_sat >= 6 else _C["amber"])
        else:
            self._sat_lbl.config(text=" --", fg=_C["no_data"])

        # HDOP
        if raw_valid and hdop < 99.0:
            hcol = (_C["green"] if hdop < 1.0 else
                    _C["amber"] if hdop < 2.0 else _C["red"])
            self._hdop_lbl.config(text=f"{hdop:5.2f}", fg=hcol)
        else:
            self._hdop_lbl.config(text=" -.--", fg=_C["no_data"])

        # Heartbeat blink
        if heartbeat is not None and heartbeat != self._prev_heartbeat:
            self._hb_state       = not self._hb_state
            self._prev_heartbeat = heartbeat
        self._hb_cv.itemconfig(
            self._hb_dot,
            fill=_C["hb_on"] if self._hb_state else _C["hb_off"])

        # Position
        if raw_valid and fix_type >= 1:
            lat_ch = "N" if lat >= 0 else "S"
            lon_ch = "E" if lon >= 0 else "W"
            col    = _C["text"] if pos_usable else _C["amber"]
            self._lat_lbl.config(text=f"{abs(lat):10.6f} {lat_ch}", fg=col)
            self._lon_lbl.config(text=f"{abs(lon):10.6f} {lon_ch}", fg=col)
        else:
            self._lat_lbl.config(text="---.---------- -", fg=_C["no_data"])
            self._lon_lbl.config(text="---.---------- -", fg=_C["no_data"])

        # Altitude
        if raw_valid and fix_type >= 2:
            alt_ft = alt_m * _M_TO_FT
            self._alt_ft_lbl.config(text=f"{alt_ft:7.0f} ft", fg=_C["text"])
            self._alt_m_lbl.config( text=f"({alt_m:6.0f} m)",  fg=_C["unit"])
        else:
            self._alt_ft_lbl.config(text="------ ft",  fg=_C["no_data"])
            self._alt_m_lbl.config( text="(------ m)", fg=_C["unit"])

        # Ground speed
        if raw_valid:
            gs_kt  = gs_cms * _CMS_TO_KT
            gs_kmh = gs_cms * _CMS_TO_KMH
            self._gs_kt_lbl.config(text=f"{gs_kt:5.1f} kt",      fg=_C["text"])
            self._gs_km_lbl.config(text=f"({gs_kmh:5.1f} km/h)", fg=_C["unit"])
        else:
            self._gs_kt_lbl.config(text="---.- kt",   fg=_C["no_data"])
            self._gs_km_lbl.config(text="(--- km/h)", fg=_C["unit"])

        # Ground track
        if raw_valid:
            trk = (course_dd / 10.0) % 360.0
            self._trk_lbl.config(text=f"{trk:05.1f}°", fg=_C["text"])
            self._point_needle(self._trk_cv, self._trk_needle, 34, 34, 28, trk)
        else:
            self._trk_lbl.config(text="---.-°", fg=_C["no_data"])
            self._point_needle(self._trk_cv, self._trk_needle, 34, 34, 28, 0)

        # Home
        if comp_valid:
            brg360 = (brg + 360) % 360
            self._dist_lbl.config(text=f"{int(dist_m):6d} m", fg=_C["cyan"])
            self._brg_lbl.config( text=f"{brg360:03d}°",       fg=_C["cyan"])
            self._point_needle(self._home_cv, self._home_arrow,
                               34, 34, 28, brg360)
        else:
            self._dist_lbl.config(text="------ m", fg=_C["no_data"])
            self._brg_lbl.config( text="---°",     fg=_C["no_data"])
            self._point_needle(self._home_cv, self._home_arrow, 34, 34, 28, 0)

    # ── Satellite tab update ──────────────────────────────────────────────────

    def _update_satellites(self, ui_data: dict):
        self._sat_canvas.update_satellites(ui_data.get("gps_sv_list", []))

    # ── Map tab update ────────────────────────────────────────────────────────

    def _update_map(self, ui_data: dict):
        raw_valid  = ui_data.get("gps_raw_valid",       False)
        fix_type   = ui_data.get("gps_fix_type",         0)
        lat        = ui_data.get("gps_latitude",         0.0)
        lon        = ui_data.get("gps_longitude",        0.0)
        fix_valid  = raw_valid and fix_type >= 2
        self._map_canvas.update_position(lat, lon, fix_valid)