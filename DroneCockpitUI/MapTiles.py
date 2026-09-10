"""
MapTiles.py — disk-cached XYZ tile fetcher for DetectionMapWidget
===================================================================
Separated from DetectionMapWidget.py because this is a self-contained
concern (talk to a tile server, cache bytes on disk, hand back a local
path) that doesn't know or care about detections, tracks, or Tk beyond
"don't block the caller's thread". Kept intentionally dumb: no tile
stitching, no lat/lon math -- that stays in DetectionMapWidget, which
already owns the projection/canvas logic.

Two providers are wired up:
  - "satellite": Esri World Imagery (ArcGIS Online), z/y/x URL scheme
  - "topo":       OpenTopoMap -- contour-line topographic style, the
                  closest free equivalent to a "military map" look

Both are free, no-API-key tile sources with usage policies that require
a descriptive User-Agent and reasonable request volume (see _USER_AGENT
below) -- fine for a single ground-control app, not for redistributing
or hammering the servers, which the on-disk cache also helps with.
"""

import os
import threading
import urllib.request

TILE_SIZE = 256

# Directory this file lives in -- used as the anchor for a project-relative
# cache path, so the cache travels with the project folder (copy/move
# "Project R.A.D" anywhere, cache included) instead of living in the
# Windows user profile where it'd be left behind on a fresh machine/user.
_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))

PROVIDERS = {
    "satellite": {
        "label": "Satellite",
        "url_template": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        "max_zoom": 19,
        "attribution": "Esri, Maxar, Earthstar Geographics",
    },
    "topo": {
        "label": "Topographic",
        "url_template": "https://tile.opentopomap.org/{z}/{x}/{y}.png",
        "max_zoom": 17,
        "attribution": "OpenTopoMap (CC-BY-SA), SRTM",
    },
}

_USER_AGENT = "DroneCockpitUI/1.0 (personal SAR ground-control project; contact: n/a)"

# Public so prefetch_tiles.py (offline pre-caching, run before a flight) and
# TileCache below write into the exact same on-disk layout -- there is only
# ever one cache location, whether a tile arrives via a live redraw's
# on-demand fetch or a pre-flight bulk prefetch.
#
# Modular/portable by design:
#   1. DRONECOCKPIT_TILE_CACHE env var, if set, always wins -- lets you point
#      at an external drive or a shared/network cache without touching code.
#   2. Otherwise, defaults to a "tile_cache" folder next to this file, i.e.
#      inside the project tree (Project R.A.D/DroneCockpitUI/tile_cache).
#      That means the cache is part of the project folder: copy/zip/move
#      the project anywhere (another machine, a fresh Windows install, a
#      USB stick with no internet) and the downloaded tiles come with it
#      instead of being stranded in C:\Users\<name>\.dronecockpitui.
CACHE_ROOT = os.environ.get(
    "DRONECOCKPIT_TILE_CACHE",
    os.path.join(_MODULE_DIR, "tile_cache"),
)


class TileCache:
    """
    On-disk tile cache + background downloader.

    get_cached() is the only method the UI thread calls, and it never
    blocks on the network: it returns a local file path immediately if
    the tile is already on disk, or None while kicking off a background
    download thread if it isn't. This mirrors the polling style already
    used for the live annotated feed elsewhere in this app (see
    DetectionMapWidget._poll_live_frame) rather than calling back into
    Tk from a worker thread directly -- the caller is expected to poll
    `ready_queue` on the Tk thread (e.g. via self.after(...)) and
    redraw once new tiles land.
    """

    def __init__(self):
        self._pending = set()
        self._pending_lock = threading.Lock()
        # Thread-safe handoff of "this tile finished downloading" back to
        # whichever Tk widget is polling us -- see MapTiles module
        # docstring for why this is a queue instead of a direct callback.
        import queue
        self.ready_queue = queue.Queue()
        os.makedirs(CACHE_ROOT, exist_ok=True)

    @staticmethod
    def tile_path(provider, z, x, y):
        return os.path.join(CACHE_ROOT, provider, str(z), str(x), f"{y}.png")

    def get_cached(self, provider, z, x, y):
        """
        Returns a local file path if this tile is already cached on
        disk. If it isn't, queues a background download (deduplicated
        against any already-in-flight request for the same tile) and
        returns None -- the caller should draw a placeholder for this
        cell and pick the tile up on a later redraw once it's ready.
        """
        path = self.tile_path(provider, z, x, y)
        if os.path.exists(path):
            return path
        self._request_download(provider, z, x, y)
        return None

    def _request_download(self, provider, z, x, y):
        key = (provider, z, x, y)
        with self._pending_lock:
            if key in self._pending:
                return
            self._pending.add(key)
        threading.Thread(target=self._download, args=key, daemon=True).start()

    def _download(self, provider, z, x, y):
        cfg = PROVIDERS[provider]
        url = cfg["url_template"].format(z=z, x=x, y=y)
        dest = self.tile_path(provider, z, x, y)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": _USER_AGENT})
            with urllib.request.urlopen(req, timeout=6) as resp:
                data = resp.read()
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            # Write to a temp file and rename into place so a half-written
            # tile (killed app, dropped connection mid-write) never gets
            # read back as "cached" by get_cached()'s os.path.exists check.
            tmp = dest + ".part"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, dest)
            self.ready_queue.put((provider, z, x, y))
        except Exception:
            # Offline, rate-limited, tile doesn't exist at this z/x/y, etc.
            # Fail quiet -- the widget keeps showing its flat-color
            # placeholder for this cell and will naturally retry the next
            # time this tile is needed (e.g. pan/zoom, or app restart).
            pass
        finally:
            with self._pending_lock:
                self._pending.discard((provider, z, x, y))