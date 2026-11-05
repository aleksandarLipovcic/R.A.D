"""
detection_report.py — self-contained detection reports (collage + JSON)
=========================================================================
DetectionLink (C++) saves one annotated full frame per DetectionRecord into
detections/. On its own that image is not enough to re-find the object
later: it says nothing about WHERE on the map it was, WHEN, or how sure the
model was. Everything else lives only in the copilot's Detections window and
is gone when the app closes.

This module turns every DetectionRecord into a report that stands on its own:

    detections/reports/<YYYY-MM-DD>/
        <HHMMSS_mmm>_d<id>_T<track>_<class>.jpg    collage (see below)
        <HHMMSS_mmm>_d<id>_T<track>_<class>.json   the same data, machine-readable
        index.csv                                  one row per report that day

Collage layout (all text in Serbian, for the copilot):

    +------------------------------------------+----------------------+
    | header: class, confidence, date/time, ids                       |
    +------------------------------------------+----------------------+
    | camera frame with the box                | satellite map        |
    |   + zoomed crop of the object (inset)    |   object + ± radius  |
    |                                          |   dron + heading     |
    |                                          |   scale bar, north   |
    +------------------------------------------+----------------------+
    | object position (decimal, DMS, MGRS), ranging, dron telemetry   |
    +------------------------------------------------------------------+

The JPEG also carries the object's position as standard EXIF GPS tags, so
any photo viewer / Google Earth / QGIS can place it on a map directly.

Threading: DetectionReportWriter owns ONE daemon thread. DetectionWorker
only hands it record objects (submit() is a deque append); image work,
tile downloads and disk writes all happen on the writer thread, never on
the Tk thread or DetectionWorker's poll thread.

Map tiles come from the same on-disk cache the Detections window uses
(MapTiles.CACHE_ROOT). Missing tiles are downloaded here with a short
timeout; after a failed download the writer stops trying for a while
(OFFLINE_BACKOFF_S) and builds collages from cached tiles only, so flying
without internet never stalls the queue. Pre-fetch the area with
tools/prefetch_tiles.py before an offline flight.
"""

from __future__ import annotations

import collections
import csv
import datetime as _dt
import json
import math
import os
import threading
import time

from core import MapTiles

try:
    from PIL import Image, ImageDraw, ImageFont
    _PIL_AVAILABLE = True
except ImportError:  # pragma: no cover - Pillow is a hard dependency of the UI
    _PIL_AVAILABLE = False


# ── Layout / look ─────────────────────────────────────────────────────────────

FRAME_W, FRAME_H = 960, 540        # camera panel (frame letterboxed into it)
MAP_W, MAP_H = 540, 540            # map panel, square
HEADER_H = 64
INFO_H = 200
PAD = 8
COLLAGE_W = FRAME_W + MAP_W + 3 * PAD
COLLAGE_H = HEADER_H + FRAME_H + INFO_H + 4 * PAD
JPEG_QUALITY = 90

MAP_PROVIDER = "satellite"
MIN_ZOOM, MAX_ZOOM = 12, 19
DEFAULT_ZOOM_NO_OBJECT = 17        # dron-only map when the object has no position
TILE_TIMEOUT_S = 4.0
OFFLINE_BACKOFF_S = 60.0           # after a failed tile download, cache-only for this long

BG = (18, 20, 22)
PANEL = (30, 33, 36)
TEXT = (230, 232, 235)
MUTED = (150, 156, 162)
ACCENT = (0, 224, 255)             # same cyan as the selected pin in DetectionMapWidget
OBJECT_COL = (255, 170, 0)         # same amber as an object pin in DetectionMapWidget
DRONE_COL = (0, 255, 136)
WARN = (255, 90, 70)

# Serbian names for the model's classes; unknown classes fall back to the raw name.
CLASS_NAMES_SR = {
    "person": "osoba",
    "car": "automobil",
    "large_vehicle": "veliko vozilo",
    "motorcycle": "motocikl",
    "other_vehicle": "ostalo vozilo",
}

RANGE_METHODS_SR = {
    "ground_plane": "presjek pravca kamere sa zemljom",
    "triangulated": "triangulacija iz više položaja",
    "object_size": "po prividnoj veličini objekta",
    "coarse": "gruba procjena (smjer kamere)",
}

GPS_FIX_SR = {0: "nema fiksa", 1: "2D fiks", 2: "3D fiks"}


# ── Pure helpers (unit-tested) ────────────────────────────────────────────────

def mercator_pixel(lat: float, lon: float, zoom: int):
    """Same Web Mercator convention as DetectionMapWidget._mercator_pixel."""
    lat = max(min(lat, 85.05112878), -85.05112878)
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom * MapTiles.TILE_SIZE
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def meters_per_pixel(lat: float, zoom: int) -> float:
    return 156543.03392 * math.cos(math.radians(lat)) / (2.0 ** zoom)


def choose_zoom(points, radius_m: float, w: int, h: int, margin: int = 60) -> int:
    """
    Highest zoom (<= MAX_ZOOM) at which every (lat, lon) in `points`, plus a
    circle of radius_m around the FIRST point (the object), fits inside a
    w x h map with `margin` pixels to spare on each side.
    """
    lat0 = points[0][0]
    for z in range(MAX_ZOOM, MIN_ZOOM - 1, -1):
        xs, ys = zip(*(mercator_pixel(la, lo, z) for la, lo in points))
        r_px = radius_m / meters_per_pixel(lat0, z)
        ox, oy = xs[0], ys[0]
        span_x = max(max(xs), ox + r_px) - min(min(xs), ox - r_px)
        span_y = max(max(ys), oy + r_px) - min(min(ys), oy - r_px)
        if span_x <= w - 2 * margin and span_y <= h - 2 * margin:
            return z
    return MIN_ZOOM


def format_dms(value: float, is_lat: bool) -> str:
    hemi = ("N" if value >= 0 else "S") if is_lat else ("E" if value >= 0 else "W")
    v = abs(value)
    d = int(v)
    m_full = (v - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    if s >= 59.995:          # avoid 59.999" rounding up to 60.00"
        s = 0.0
        m += 1
        if m == 60:
            m = 0
            d += 1
    return f"{d}°{m:02d}'{s:05.2f}\" {hemi}"


def to_mgrs(lat: float, lon: float, precision: int = 5) -> str:
    """
    WGS84 lat/lon -> MGRS string, e.g. '33T XL 12345 67890' (1 m precision).
    Valid between 80°S and 84°N (UPS polar zones are not handled -- not a
    real limitation for this drone). Includes the Norway/Svalbard zone
    exceptions for completeness.
    """
    if not (-80.0 <= lat < 84.0):
        return "—"
    zone = int((lon + 180.0) // 6) + 1
    if 56.0 <= lat < 64.0 and 3.0 <= lon < 12.0:
        zone = 32
    if 72.0 <= lat < 84.0:
        if 0.0 <= lon < 9.0:
            zone = 31
        elif 9.0 <= lon < 21.0:
            zone = 33
        elif 21.0 <= lon < 33.0:
            zone = 35
        elif 33.0 <= lon < 42.0:
            zone = 37

    # WGS84 -> UTM (Krüger series, standard USGS formulation).
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    k0 = 0.9996
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    phi = math.radians(lat)
    lam = math.radians(lon)
    n_ = a / math.sqrt(1 - e2 * math.sin(phi) ** 2)
    t = math.tan(phi) ** 2
    c = ep2 * math.cos(phi) ** 2
    a_ = math.cos(phi) * (lam - lon0)
    m = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * phi
             - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * phi)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * phi)
             - (35 * e2 ** 3 / 3072) * math.sin(6 * phi))
    easting = k0 * n_ * (a_ + (1 - t + c) * a_ ** 3 / 6
                         + (5 - 18 * t + t ** 2 + 72 * c - 58 * ep2) * a_ ** 5 / 120) + 500000.0
    northing = k0 * (m + n_ * math.tan(phi) * (a_ ** 2 / 2
                                               + (5 - t + 9 * c + 4 * c ** 2) * a_ ** 4 / 24
                                               + (61 - 58 * t + t ** 2 + 600 * c - 330 * ep2) * a_ ** 6 / 720))
    if lat < 0:
        northing += 10000000.0

    band = "CDEFGHJKLMNPQRSTUVWXX"[int((lat + 80) // 8)]
    set_no = (zone - 1) % 6
    col_letters = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")[set_no % 3]
    row_letters = "ABCDEFGHJKLMNPQRSTUV"
    col = col_letters[int(easting // 100000) - 1]
    row_offset = 5 if zone % 2 == 0 else 0
    row = row_letters[(int(northing // 100000) + row_offset) % 20]

    div = 10 ** (5 - precision)
    e = int(easting % 100000) // div
    nn = int(northing % 100000) // div
    return f"{zone}{band} {col}{row} {e:0{precision}d} {nn:0{precision}d}"


def class_label(class_name: str) -> str:
    sr = CLASS_NAMES_SR.get(class_name)
    return f"{sr} ({class_name})" if sr else class_name


def report_basename(rec) -> str:
    """File stem of a record's report; also used by DetectionMapWidget to find it."""
    t = _dt.datetime.fromtimestamp(rec.timestamp_ms / 1000.0)
    safe_cls = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(rec.class_name))
    return (f"{t:%H%M%S}_{rec.timestamp_ms % 1000:03d}_d{rec.id}"
            f"_T{getattr(rec, 'track_id', 0)}_{safe_cls}")


def report_paths(rec, report_dir: str):
    """(jpg_path, json_path) of a record's report under report_dir/<date>/."""
    t = _dt.datetime.fromtimestamp(rec.timestamp_ms / 1000.0)
    folder = os.path.join(report_dir, f"{t:%Y-%m-%d}")
    stem = os.path.join(folder, report_basename(rec))
    return stem + ".jpg", stem + ".json"


def record_to_dict(rec) -> dict:
    """Everything a DetectionRecord carries, as plain JSON-serialisable data."""
    tel = getattr(rec, "telemetry", None)
    ts = rec.timestamp_ms / 1000.0
    local = _dt.datetime.fromtimestamp(ts).astimezone()
    utc = _dt.datetime.fromtimestamp(ts, _dt.timezone.utc)
    geo = bool(rec.georeferenced)

    def g(obj, name, default=None):
        return getattr(obj, name, default) if obj is not None else default

    d = {
        "format": "rad-detection-report/1",
        "detection_id": rec.id,
        "track_id": getattr(rec, "track_id", 0),
        "class_name": rec.class_name,
        "class_name_sr": CLASS_NAMES_SR.get(rec.class_name, rec.class_name),
        "confidence": round(float(rec.confidence), 4),
        "best_confidence": round(float(getattr(rec, "best_confidence", 0.0)), 4),
        "sightings": getattr(rec, "sightings", 0),
        "time": {
            "timestamp_ms": rec.timestamp_ms,
            "local": local.isoformat(timespec="milliseconds"),
            "utc": utc.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        },
        "object": {
            "georeferenced": geo,
            "latitude": rec.latitude if geo else None,
            "longitude": rec.longitude if geo else None,
            "mgrs": to_mgrs(rec.latitude, rec.longitude) if geo else None,
            "uncertainty_m": round(getattr(rec, "uncertainty_m", 0.0), 1) if geo else None,
            "range_method": rec.range_method or None,
            "distance_m": round(rec.distance_m, 1) if geo else None,
            "bearing_deg": round(rec.bearing_deg, 1) if geo else None,
            "google_maps": (f"https://www.google.com/maps?q={rec.latitude:.7f},{rec.longitude:.7f}"
                            if geo else None),
        },
        "bbox_px": {"x": getattr(rec, "bbox_x", 0), "y": getattr(rec, "bbox_y", 0),
                    "w": getattr(rec, "bbox_w", 0), "h": getattr(rec, "bbox_h", 0)},
        "drone": {
            "telemetry_valid": bool(g(tel, "valid", False)),
            "latitude": g(tel, "latitude"),
            "longitude": g(tel, "longitude"),
            "altitude_agl_m": g(tel, "altitude_m"),
            "heading_deg": g(tel, "heading_deg"),
            "roll_deg": g(tel, "roll_deg"),
            "pitch_deg": g(tel, "pitch_deg"),
            "gimbal_pan_deg": g(tel, "gimbal_pan_deg"),
            "gimbal_tilt_deg": g(tel, "gimbal_tilt_deg"),
            "ground_speed_ms": g(tel, "ground_speed_ms"),
            "gps_fix_type": g(tel, "gps_fix_type"),
            "gps_num_sat": g(tel, "gps_num_sat"),
            "battery_voltage": g(tel, "battery_voltage"),
            "armed": g(tel, "armed"),
        },
        "source_frame": rec.screenshot_path or None,
    }
    if not d["drone"]["telemetry_valid"]:
        # An invalid snapshot's fields are zeros/garbage -- don't let them
        # look like a position somebody could act on.
        d["drone"] = {k: (v if k == "telemetry_valid" else None) for k, v in d["drone"].items()}
    # Plain floats/ints only (pybind11 values already are, SimpleNamespace in tests too).
    for k, v in list(d["drone"].items()):
        if isinstance(v, float):
            d["drone"][k] = round(v, 7 if k in ("latitude", "longitude") else 2)
    return d


CSV_COLUMNS = ["vrijeme_lokalno", "vrijeme_utc", "detekcija_id", "objekat_id", "klasa",
               "pouzdanost", "najbolja_pouzdanost", "vidjen_puta", "lat", "lon", "mgrs",
               "nesigurnost_m", "metoda", "udaljenost_m", "azimut_deg",
               "dron_lat", "dron_lon", "dron_visina_m", "dron_kurs_deg", "izvjestaj"]


def csv_row(d: dict, jpg_path: str) -> list:
    o, dr = d["object"], d["drone"]
    return [d["time"]["local"], d["time"]["utc"], d["detection_id"], d["track_id"],
            d["class_name"], d["confidence"], d["best_confidence"], d["sightings"],
            o["latitude"], o["longitude"], o["mgrs"], o["uncertainty_m"], o["range_method"],
            o["distance_m"], o["bearing_deg"], dr["latitude"], dr["longitude"],
            dr["altitude_agl_m"], dr["heading_deg"], os.path.basename(jpg_path)]


# ── Fonts ─────────────────────────────────────────────────────────────────────

_FONT_CANDIDATES = {
    # Windows names first (looked up in C:\Windows\Fonts by name), then Linux
    # (test machines). All of these carry č ć š ž đ.
    "regular": ("segoeui.ttf", "arial.ttf", "DejaVuSans.ttf",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    "bold": ("segoeuib.ttf", "arialbd.ttf", "DejaVuSans-Bold.ttf",
             "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
    "mono": ("consola.ttf", "cour.ttf", "DejaVuSansMono.ttf",
             "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"),
}
_font_cache = {}


def _font(kind: str, size: int):
    key = (kind, size)
    if key not in _font_cache:
        f = None
        for name in _FONT_CANDIDATES[kind]:
            try:
                f = ImageFont.truetype(name, size)
                break
            except OSError:
                continue
        _font_cache[key] = f or ImageFont.load_default()
    return _font_cache[key]


# ── Collage drawing ───────────────────────────────────────────────────────────

class _TileSource:
    """Cache-first tile access for the writer thread, with an offline back-off."""

    def __init__(self, provider: str = MAP_PROVIDER):
        self.provider = provider
        self._offline_until = 0.0
        self._mem = collections.OrderedDict()   # small LRU of decoded tiles

    def get(self, z: int, x: int, y: int):
        key = (z, x, y)
        img = self._mem.get(key)
        if img is not None:
            self._mem.move_to_end(key)
            return img
        path = MapTiles.TileCache.tile_path(self.provider, z, x, y)
        if not os.path.exists(path):
            if time.monotonic() < self._offline_until:
                return None
            if not MapTiles.download_tile(self.provider, z, x, y, timeout=TILE_TIMEOUT_S):
                self._offline_until = time.monotonic() + OFFLINE_BACKOFF_S
                return None
        try:
            img = Image.open(path).convert("RGB")
        except (OSError, ValueError):
            return None
        self._mem[key] = img
        if len(self._mem) > 96:
            self._mem.popitem(last=False)
        return img


def _render_map(rec, tiles: _TileSource):
    """Returns (PIL image MAP_W x MAP_H, zoom or None, tiles_missing)."""
    img = Image.new("RGB", (MAP_W, MAP_H), (17, 26, 20))
    draw = ImageDraw.Draw(img)
    tel = getattr(rec, "telemetry", None)
    drone_ok = bool(tel is not None and getattr(tel, "valid", False)
                    and (tel.latitude != 0.0 or tel.longitude != 0.0))
    obj_ok = bool(rec.georeferenced)

    if not obj_ok and not drone_ok:
        msg = "Nema GPS pozicije\n(telemetrija nije bila ispravna)"
        draw.multiline_text((MAP_W / 2, MAP_H / 2), msg, fill=MUTED, font=_font("bold", 22),
                            anchor="mm", align="center")
        return img, None, 0

    radius = float(getattr(rec, "uncertainty_m", 0.0) or 0.0) if obj_ok else 0.0
    if obj_ok:
        points = [(rec.latitude, rec.longitude)] + ([(tel.latitude, tel.longitude)] if drone_ok else [])
        zoom = choose_zoom(points, radius, MAP_W, MAP_H)
        c_lat = (min(p[0] for p in points) + max(p[0] for p in points)) / 2
        c_lon = (min(p[1] for p in points) + max(p[1] for p in points)) / 2
    else:
        zoom = DEFAULT_ZOOM_NO_OBJECT
        c_lat, c_lon = tel.latitude, tel.longitude

    cx, cy = mercator_pixel(c_lat, c_lon, zoom)
    left, top = cx - MAP_W / 2, cy - MAP_H / 2
    n_tiles = 2 ** zoom
    ts = MapTiles.TILE_SIZE
    missing = 0
    for ty in range(int(math.floor(top / ts)), int(math.floor((top + MAP_H) / ts)) + 1):
        if not 0 <= ty < n_tiles:
            continue
        for tx in range(int(math.floor(left / ts)), int(math.floor((left + MAP_W) / ts)) + 1):
            tile = tiles.get(zoom, tx % n_tiles, ty)
            if tile is None:
                missing += 1
                continue
            img.paste(tile, (int(tx * ts - left), int(ty * ts - top)))

    def px(lat, lon):
        x, y = mercator_pixel(lat, lon, zoom)
        return x - left, y - top

    if drone_ok and obj_ok:
        dx, dy = px(tel.latitude, tel.longitude)
        ox, oy = px(rec.latitude, rec.longitude)
        _dashed_line(draw, (dx, dy), (ox, oy), fill=(255, 255, 255), width=2)

    if obj_ok:
        ox, oy = px(rec.latitude, rec.longitude)
        r_px = radius / meters_per_pixel(rec.latitude, zoom)
        if r_px >= 3:
            overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
            od = ImageDraw.Draw(overlay)
            od.ellipse((ox - r_px, oy - r_px, ox + r_px, oy + r_px),
                       fill=OBJECT_COL + (55,), outline=OBJECT_COL + (230,), width=2)
            img.paste(overlay, (0, 0), overlay)
            draw = ImageDraw.Draw(img)
        draw.ellipse((ox - 8, oy - 8, ox + 8, oy + 8), fill=OBJECT_COL, outline=(0, 0, 0), width=2)
        draw.line((ox - 14, oy, ox - 9, oy), fill=(0, 0, 0), width=2)
        draw.line((ox + 9, oy, ox + 14, oy), fill=(0, 0, 0), width=2)

    if drone_ok:
        dx, dy = px(tel.latitude, tel.longitude)
        h = math.radians(getattr(tel, "heading_deg", 0.0) or 0.0)
        pts = []
        for ang, rr in ((0.0, 18), (2.5, 13), (math.pi, 6), (-2.5, 13)):
            pts.append((dx + rr * math.sin(h + ang), dy - rr * math.cos(h + ang)))
        draw.polygon(pts, fill=DRONE_COL, outline=(0, 0, 0))

    # Legend, north arrow, scale bar, attribution.
    # Legend symbols are drawn as shapes, not font glyphs: not every
    # Windows font has ● / ▲, and a missing glyph renders as an empty box.
    if obj_ok:
        _label(draw, (30, 10), "objekat", OBJECT_COL)
        draw.ellipse((13, 13, 25, 25), fill=OBJECT_COL, outline=(0, 0, 0))
    else:
        _label(draw, (10, 10), "objekat bez pozicije", WARN)
    if drone_ok:
        _label(draw, (30, 36), "dron (smjer kursa)", DRONE_COL)
        draw.polygon([(19, 38), (13, 51), (25, 51)], fill=DRONE_COL, outline=(0, 0, 0))
    nx, ny = MAP_W - 30, 44
    draw.polygon([(nx, ny - 26), (nx - 10, ny), (nx + 10, ny)], fill=(255, 255, 255), outline=(0, 0, 0))
    draw.text((nx, ny + 12), "S", fill=(255, 255, 255), font=_font("bold", 18), anchor="mm",
              stroke_width=2, stroke_fill=(0, 0, 0))
    _scale_bar(draw, meters_per_pixel(c_lat, zoom))
    attribution = "© " + MapTiles.PROVIDERS[tiles.provider]["attribution"]
    draw.text((MAP_W - 6, MAP_H - 4), attribution, fill=(220, 220, 220), font=_font("regular", 11),
              anchor="rd", stroke_width=1, stroke_fill=(0, 0, 0))
    if missing:
        _label(draw, (10, MAP_H - 84), f"{missing} pločica karte nedostaje (offline?)", WARN)
    return img, zoom, missing


def _label(draw, xy, text, color):
    f = _font("bold", 15)
    x, y = xy
    l, t, r, b = draw.textbbox((x, y), text, font=f)
    draw.rectangle((l - 5, t - 3, r + 5, b + 3), fill=(0, 0, 0))
    draw.text((x, y), text, fill=color, font=f)


def _dashed_line(draw, p0, p1, fill, width, dash=10, gap=7):
    (x0, y0), (x1, y1) = p0, p1
    length = math.hypot(x1 - x0, y1 - y0)
    if length < 1:
        return
    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    pos = 0.0
    while pos < length:
        end = min(pos + dash, length)
        draw.line((x0 + ux * pos, y0 + uy * pos, x0 + ux * end, y0 + uy * end), fill=fill, width=width)
        pos = end + gap


def _scale_bar(draw, m_per_px: float):
    target = 120 * m_per_px
    nice = 1.0
    for step in (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000):
        if step <= target:
            nice = step
    bar_px = nice / m_per_px
    x0, y0 = 14, MAP_H - 24
    draw.rectangle((x0 - 6, y0 - 22, x0 + bar_px + 6, y0 + 8), fill=(0, 0, 0))
    draw.line((x0, y0, x0 + bar_px, y0), fill=(255, 255, 255), width=3)
    draw.line((x0, y0 - 6, x0, y0 + 4), fill=(255, 255, 255), width=2)
    draw.line((x0 + bar_px, y0 - 6, x0 + bar_px, y0 + 4), fill=(255, 255, 255), width=2)
    label = f"{nice / 1000:g} km" if nice >= 1000 else f"{nice:g} m"
    draw.text((x0 + bar_px / 2, y0 - 6), label, fill=(255, 255, 255), font=_font("regular", 13), anchor="md")


def _render_frame(rec):
    panel = Image.new("RGB", (FRAME_W, FRAME_H), (0, 0, 0))
    draw = ImageDraw.Draw(panel)
    path = rec.screenshot_path
    try:
        src = Image.open(path).convert("RGB") if path else None
    except (OSError, ValueError):
        src = None
    if src is None:
        draw.text((FRAME_W / 2, FRAME_H / 2), "Snimak kadra nije dostupan", fill=MUTED,
                  font=_font("bold", 22), anchor="mm")
        return panel

    scale = min(FRAME_W / src.width, FRAME_H / src.height)
    fw, fh = max(1, int(src.width * scale)), max(1, int(src.height * scale))
    ox, oy = (FRAME_W - fw) // 2, (FRAME_H - fh) // 2
    panel.paste(src.resize((fw, fh), Image.LANCZOS), (ox, oy))
    draw = ImageDraw.Draw(panel)

    bx, by = getattr(rec, "bbox_x", 0), getattr(rec, "bbox_y", 0)
    bw, bh = getattr(rec, "bbox_w", 0), getattr(rec, "bbox_h", 0)
    if bw > 0 and bh > 0:
        # Label above the box the C++ side already drew.
        tx, ty = ox + bx * scale, max(oy + by * scale, 24)
        tag = f"{CLASS_NAMES_SR.get(rec.class_name, rec.class_name)} {rec.confidence:.2f}"
        f = _font("bold", 16)
        l, t, r, b = draw.textbbox((tx, ty - 4), tag, font=f, anchor="ld")
        draw.rectangle((l - 3, t - 2, r + 3, b + 2), fill=(255, 220, 0))   # same colour as the C++ box (BGR 0,220,255)
        draw.text((tx, ty - 4), tag, fill=(0, 0, 0), font=f, anchor="ld")

        # Zoomed crop inset: the object at ~3x, bottom-right corner (or
        # bottom-left if the object itself sits in the bottom-right).
        m = max(bw, bh) * 0.6 + 8
        cx, cy = bx + bw / 2, by + bh / 2
        crop = src.crop((int(max(0, cx - bw / 2 - m)), int(max(0, cy - bh / 2 - m)),
                         int(min(src.width, cx + bw / 2 + m)), int(min(src.height, cy + bh / 2 + m))))
        inset_max = 220
        k = min(inset_max / crop.width, inset_max / crop.height, 6.0)
        if k < 1.5:
            return panel        # object already big in the frame -- an inset adds nothing
        crop = crop.resize((max(1, int(crop.width * k)), max(1, int(crop.height * k))), Image.LANCZOS)
        on_right = (ox + cx * scale) < FRAME_W * 0.62 or (oy + cy * scale) < FRAME_H * 0.55
        ix = FRAME_W - crop.width - 12 if on_right else 12
        iy = FRAME_H - crop.height - 12
        draw.rectangle((ix - 3, iy - 22, ix + crop.width + 2, iy + crop.height + 2), fill=ACCENT)
        draw.text((ix + 2, iy - 20), f"uvećano {k:.1f}x", fill=(0, 0, 0), font=_font("bold", 13))
        panel.paste(crop, (ix, iy))
    return panel


def _info_lines(d: dict):
    """Two columns of (label, value) rows for the bottom info band."""
    o, dr = d["object"], d["drone"]
    left = []
    if o["georeferenced"]:
        left += [
            ("Pozicija objekta", f"{o['latitude']:.7f}, {o['longitude']:.7f}"),
            ("", f"{format_dms(o['latitude'], True)}   {format_dms(o['longitude'], False)}"),
            ("MGRS", o["mgrs"]),
            ("Tačnost", f"± {o['uncertainty_m']:.0f} m"),
            ("Metoda", RANGE_METHODS_SR.get(o["range_method"], o["range_method"] or "—")),
            ("Od drona", f"{o['distance_m']:.0f} m, azimut {o['bearing_deg']:.0f}°"),
        ]
    else:
        left += [("Pozicija objekta", "nije određena (nema ispravne telemetrije)")]

    right = []
    if dr["telemetry_valid"]:
        right += [
            ("Dron", f"{dr['latitude']:.7f}, {dr['longitude']:.7f}"),
            ("Visina", f"{dr['altitude_agl_m']:.1f} m iznad zemlje"),
            ("Kurs / brzina", f"{dr['heading_deg']:.0f}°  /  {dr['ground_speed_ms'] or 0:.1f} m/s"),
            ("Nagib / valjanje", f"{dr['pitch_deg']:.1f}°  /  {dr['roll_deg']:.1f}°"),
            ("Kamera (nagib/zakret)", f"{dr['gimbal_tilt_deg']:.0f}°  /  {dr['gimbal_pan_deg']:.0f}°"),
            ("GPS", f"{GPS_FIX_SR.get(dr['gps_fix_type'], '?')}, {dr['gps_num_sat']} sat."),
        ]
    else:
        right += [("Dron", "telemetrija nije bila ispravna")]
    return left, right


def build_collage(rec, tiles: _TileSource, d: dict = None):
    """Builds the collage for one record. Returns (PIL image, map zoom, missing tiles)."""
    d = d or record_to_dict(rec)
    img = Image.new("RGB", (COLLAGE_W, COLLAGE_H), BG)
    draw = ImageDraw.Draw(img)

    # Header
    draw.rectangle((0, 0, COLLAGE_W, HEADER_H + PAD), fill=PANEL)
    title = f"{class_label(rec.class_name).upper()}   ·   pouzdanost {rec.confidence:.2f}"
    best = d["best_confidence"]
    if best and d["sightings"] > 1:
        title += f"  (najbolja {best:.2f}, viđen {d['sightings']}x)"
    draw.text((PAD + 6, PAD + 4), title, fill=OBJECT_COL, font=_font("bold", 24))
    local = _dt.datetime.fromisoformat(d["time"]["local"])
    tz = f"{local:%z}"                       # '+0100' -> 'UTC+01:00'
    when = (f"{local:%d.%m.%Y.  %H:%M:%S}.{local.microsecond // 1000:03d}"
            f"  (lokalno, UTC{tz[:3]}:{tz[3:]})")
    draw.text((PAD + 6, PAD + 36), when, fill=TEXT, font=_font("regular", 16))
    ids = f"Project R.A.D.  ·  detekcija #{d['detection_id']}  ·  objekat T{d['track_id']}"
    draw.text((COLLAGE_W - PAD - 6, PAD + 8), ids, fill=MUTED, font=_font("regular", 15), anchor="ra")
    draw.text((COLLAGE_W - PAD - 6, PAD + 36), "UTC " + d["time"]["utc"][11:23], fill=MUTED,
              font=_font("regular", 15), anchor="ra")

    # Frame + map
    y_mid = HEADER_H + 2 * PAD
    img.paste(_render_frame(rec), (PAD, y_mid))
    map_img, zoom, missing = _render_map(rec, tiles)
    img.paste(map_img, (2 * PAD + FRAME_W, y_mid))

    # Info band
    y_info = y_mid + FRAME_H + PAD
    draw.rectangle((PAD, y_info, COLLAGE_W - PAD, y_info + INFO_H), fill=PANEL)
    left, right = _info_lines(d)
    fl, fv = _font("regular", 15), _font("mono", 17)
    for col_x, rows in ((PAD + 14, left), (PAD + 14 + (COLLAGE_W - 2 * PAD) // 2, right)):
        y = y_info + 12
        for label, value in rows:
            if label:
                draw.text((col_x, y + 2), label, fill=MUTED, font=fl)
            value = str(value)
            max_w = (COLLAGE_W - 2 * PAD) // 2 - 190 - 40
            while value and draw.textlength(value, font=fv) > max_w:
                value = value[:-2] + "…"
            draw.text((col_x + 190, y), value, fill=TEXT, font=fv)
            y += 30
    draw.line((PAD + (COLLAGE_W - 2 * PAD) // 2 - 10, y_info + 12,
               PAD + (COLLAGE_W - 2 * PAD) // 2 - 10, y_info + INFO_H - 12), fill=(60, 64, 68), width=1)
    return img, zoom, missing


def _exif_with_gps(d: dict):
    """EXIF block carrying the object's position (or the dron's, if the object has none)."""
    exif = Image.Exif()
    exif[0x010E] = (f"R.A.D. detekcija #{d['detection_id']} T{d['track_id']} "
                    f"{d['class_name']} {d['confidence']:.2f}")          # ImageDescription
    exif[0x0131] = "Project R.A.D. kontrolna stanica"                   # Software
    local = _dt.datetime.fromisoformat(d["time"]["local"])
    exif[0x0132] = f"{local:%Y:%m:%d %H:%M:%S}"                          # DateTime

    o, dr = d["object"], d["drone"]
    if o["georeferenced"]:
        lat, lon = o["latitude"], o["longitude"]
    elif dr["telemetry_valid"]:
        lat, lon = dr["latitude"], dr["longitude"]
    else:
        return exif

    from PIL.TiffImagePlugin import IFDRational

    def dms(v):
        v = abs(v)
        deg = int(v)
        mins = int((v - deg) * 60)
        secs = (v - deg - mins / 60) * 3600
        return (IFDRational(deg, 1), IFDRational(mins, 1), IFDRational(round(secs * 10000), 10000))

    utc = _dt.datetime.fromisoformat(d["time"]["utc"].replace("Z", "+00:00"))
    gps = {
        0: b"\x02\x03\x00\x00",                       # GPSVersionID 2.3.0.0
        1: "N" if lat >= 0 else "S", 2: dms(lat),
        3: "E" if lon >= 0 else "W", 4: dms(lon),
        7: (IFDRational(utc.hour, 1), IFDRational(utc.minute, 1),
            IFDRational(utc.second * 1000 + utc.microsecond // 1000, 1000)),
        18: "WGS-84",
        29: f"{utc:%Y:%m:%d}",
    }
    exif[0x8825] = gps
    return exif


# ── Writer thread ─────────────────────────────────────────────────────────────

class DetectionReportWriter:
    """
    Background writer: submit() DetectionRecords from any thread, reports
    appear under report_dir/<date>/. Never raises into the caller; a record
    that fails (unreadable frame, full disk, ...) is logged and skipped.
    """

    def __init__(self, report_dir: str, provider: str = MAP_PROVIDER, log=print):
        self.report_dir = report_dir
        self._tiles = _TileSource(provider)
        self._log = log
        self._queue = collections.deque()
        self._cv = threading.Condition()
        self._running = False
        self._thread = None
        self._csv_lock = threading.Lock()
        self.written = 0
        self.failed = 0

    @property
    def available(self) -> bool:
        return _PIL_AVAILABLE

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="DetectionReportWriter", daemon=True)
        self._thread.start()

    def stop(self, flush_timeout_s: float = 5.0):
        """Stops after finishing what is queued, waiting at most flush_timeout_s."""
        with self._cv:
            self._running = False
            self._cv.notify_all()
        if self._thread:
            self._thread.join(flush_timeout_s)
            self._thread = None

    def submit(self, records):
        if not records:
            return
        with self._cv:
            self._queue.extend(records)
            self._cv.notify()

    def pending(self) -> int:
        with self._cv:
            return len(self._queue)

    def _loop(self):
        while True:
            with self._cv:
                while self._running and not self._queue:
                    self._cv.wait(0.5)
                if not self._queue:
                    return          # stopped and drained
                rec = self._queue.popleft()
            try:
                self.write_report(rec)
                self.written += 1
            except Exception as exc:     # keep going no matter what one record does
                self.failed += 1
                self._log(f"[DetectionReport] report for detection {getattr(rec, 'id', '?')} failed: {exc}")

    def write_report(self, rec):
        """Writes collage + JSON + index row for one record. Returns the JPEG path."""
        jpg_path, json_path = report_paths(rec, self.report_dir)
        os.makedirs(os.path.dirname(jpg_path), exist_ok=True)
        d = record_to_dict(rec)

        if _PIL_AVAILABLE:
            img, zoom, missing = build_collage(rec, self._tiles, d)
            d["map"] = {"provider": self._tiles.provider, "zoom": zoom, "missing_tiles": missing}
            d["report_image"] = os.path.basename(jpg_path)
            tmp = jpg_path + ".part"
            img.save(tmp, "JPEG", quality=JPEG_QUALITY, exif=_exif_with_gps(d))
            os.replace(tmp, jpg_path)

        tmp = json_path + ".part"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=2)
        os.replace(tmp, json_path)

        index = os.path.join(os.path.dirname(jpg_path), "index.csv")
        with self._csv_lock:
            new = not os.path.exists(index)
            # utf-8-sig so Excel opens the Serbian headers correctly.
            with open(index, "a", newline="", encoding="utf-8-sig" if new else "utf-8") as f:
                w = csv.writer(f, delimiter=";")
                if new:
                    w.writerow(CSV_COLUMNS)
                w.writerow(csv_row(d, jpg_path))
        return jpg_path
