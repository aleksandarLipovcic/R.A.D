# `MapTiles.py` & `prefetch_tiles.py` — tile cache & offline pre-download

## `MapTiles.py`

**File:** `MapTiles.py`
**Classes:** `TileCache`
**Used by:** `GPSWidget._MapCanvas` (see [gps-widget.md](gps-widget.md)),
`DetectionMapWidget` (see [detection-map-widget.md](detection-map-widget.md)),
`prefetch_tiles.py` (below)

### Responsibility

A disk-cached XYZ tile fetcher, deliberately factored out of both map
widgets because it's a self-contained concern — talk to a tile server,
cache bytes on disk, hand back a local path — that doesn't need to know
anything about detections, tracks, or Tk beyond "don't block the caller's
thread". Kept intentionally dumb: no tile stitching, no lat/lon math —
that stays in whichever widget owns the projection/canvas logic.

### Providers

| Key | Label | Source | Max zoom |
|---|---|---|---|
| `satellite` | Satellite | Esri World Imagery (ArcGIS Online) | 19 |
| `topo` | Topographic | OpenTopoMap — contour-line style, the closest free equivalent to a "military map" look | 17 |

Both are free, no-API-key tile sources whose usage policies require a
descriptive `User-Agent` (`_USER_AGENT` in the source) and reasonable
request volume — acceptable for a single ground-control app, not for
redistributing tiles or hammering the servers, which the on-disk cache also
naturally helps avoid.

### `TileCache.get_cached(provider, z, x, y)`

The **only** method either UI thread calls, and it never blocks on the
network:

- Returns a local file path immediately if the tile is already on disk.
- Otherwise kicks off a background download thread (deduplicated against
  any already-in-flight request for the exact same tile via
  `_pending`/`_pending_lock`) and returns `None` — the caller draws a flat
  placeholder color for that cell and picks the tile up on a later redraw
  once it's ready.

Callers are expected to poll `ready_queue` (a `queue.Queue`) on the Tk
thread — e.g. via `self.after(...)` — and redraw once new tiles land. This
mirrors the polling style already used elsewhere in the app (see
`DetectionMapWidget._poll_live_frame`) rather than calling back into Tk
from a worker thread directly, which Tk does not support safely.

Downloads (`_download`) write to a `.part` temp file first and
`os.replace()` into place atomically — a half-written tile (app killed,
connection dropped mid-write) can never be read back as "cached" by
`get_cached()`'s `os.path.exists` check. Failures (offline, rate-limited,
tile doesn't exist at that z/x/y) fail quiet: the widget keeps showing its
placeholder and will naturally retry the next time that tile is needed
(pan/zoom, or app restart).

### `CACHE_ROOT` resolution — portability by design

```python
CACHE_ROOT = os.environ.get(
    "DRONECOCKPIT_TILE_CACHE",
    os.path.join(_MODULE_DIR, "tile_cache"),
)
```

1. The `DRONECOCKPIT_TILE_CACHE` environment variable always wins, if set —
   lets an operator point at an external drive or a shared/network cache
   without touching code.
2. Otherwise defaults to a `tile_cache/` folder **next to `MapTiles.py`
   itself**, i.e. inside the project tree
   (`Project R.A.D/DroneCockpitUI/tile_cache`). This means the cache is
   part of the project folder: copying, zipping, or moving the whole
   project — to another machine, a fresh Windows install, a USB stick with
   no internet — brings the downloaded tiles along instead of stranding
   them in `C:\Users\<name>\.dronecockpitui`, where a fresh user profile
   would never see them again.

This is the same path both live on-demand fetches (during a flight) and
`prefetch_tiles.py`'s bulk pre-download write into — there is only ever one
cache location on disk, regardless of which mechanism populated a given
tile.

## `prefetch_tiles.py`

**File:** `prefetch_tiles.py`
**Type:** standalone CLI script, not imported by `DroneCockpitUI.py`

### Responsibility

Bulk-downloads every tile needed to cover a bounded circular area, for a
range of zoom levels, into the exact same on-disk cache
(`MapTiles.CACHE_ROOT`) the live map widgets read from — run this **before**
heading somewhere with no signal, while there's still internet (home wifi,
a cafe, etc.), so the live map works with zero network calls once out in
the field.

Deliberately a **bounded** operation, not "download the world": give it a
center point + radius (a realistic search-area size, a few km across) and
a zoom range, and it downloads exactly the tiles covering that circle at
those zooms — nothing outside it. Already-cached tiles are skipped by
default, so re-running the script later (e.g. to widen the radius or add a
zoom level) only fetches what's newly missing.

### CLI usage

```bash
# Estimate only, no download:
python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
    --provider both --min-zoom 13 --max-zoom 17 --dry-run

# Actually download (prompts for confirmation first):
python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
    --provider satellite --min-zoom 13 --max-zoom 17

# Skip the confirmation prompt (e.g. for scripting):
... --yes

# Refresh imagery that's already cached but looks stale — re-fetches and
# overwrites every matching tile instead of skipping ones already on disk
# (the only way a stale tile ever gets replaced; normal runs never touch
# an existing file):
python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
    --provider satellite --min-zoom 13 --max-zoom 17 --force
```

| Flag | Meaning |
|---|---|
| `--lat` / `--lon` / `--radius-km` | Required — center point and radius of the area to cover |
| `--provider` | `satellite` / `topo` / `both` (default `both`) |
| `--min-zoom` / `--max-zoom` | Zoom range (defaults 13–17); automatically capped at each provider's own max supported zoom |
| `--dry-run` | Print the plan/tile-count estimate and exit without downloading anything |
| `--yes` | Skip the interactive confirmation prompt |
| `--force` | Re-download and overwrite tiles even if already cached — the only way to refresh stale imagery |
| `--source` | `live` (default, Esri's continuously-updated current mosaic — newest available, but undated) or `wayback` (a specific numbered Esri World Imagery Wayback release, so the exact capture date is known). Only affects `satellite`; `topo` is always fetched live |
| `--wayback-release` | Required when `--source wayback` is set — the numbered Wayback release ID, looked up at https://livingatlas.arcgis.com/wayback/ |

When `--source wayback` is used, the script swaps `MapTiles.PROVIDERS["satellite"]["url_template"]`
in-place to point at the Wayback endpoint for that release, so every
existing code path — including `DetectionMapWidget`'s live on-demand
fetches during a later run — picks up the change with no other code
edits, and tiles still land in the same `tile_cache/satellite/` folder.
