"""
prefetch_tiles.py — pre-download map tiles for offline flights
=================================================================
Run this BEFORE heading out somewhere with no signal, while you still
have internet (home wifi, a cafe, etc). It downloads every tile needed
to cover a circle around a center point, for a range of zoom levels,
into the SAME on-disk cache DetectionMapWidget reads from at flight
time (MapTiles.CACHE_ROOT) -- so once this finishes, the map in
DetectionMapWidget just works with zero network calls for that area.

This is a bulk, bounded operation, not "download the world": you give
it a center point + radius (a realistic search-area size, a few km
across) and a zoom range, and it downloads exactly the tiles that
cover that circle at those zooms -- nothing outside it. Already-cached
tiles are skipped, so re-running this later (e.g. to widen the radius
or add a zoom level) only fetches what's missing.

USAGE
-----
Estimate only, no download:
    python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
        --provider both --min-zoom 13 --max-zoom 17 --dry-run

Actually download (prompts for confirmation first, since high zoom /
large radius can mean a LOT of tiles):
    python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
        --provider satellite --min-zoom 13 --max-zoom 17
Command:
python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 --provider both --min-zoom 13 --max-zoom 17 --dry-run

Skip the confirmation prompt (e.g. for scripting):
    ... --yes

Refresh imagery that's already cached but looks out of date (re-fetches
and overwrites every matching tile instead of skipping ones already on
disk -- this is the only way stale tiles get replaced, since normal
runs never touch a tile that already exists):
    python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
        --provider satellite --min-zoom 13 --max-zoom 17 --force

Pull a specific dated Esri World Imagery "Wayback" release instead of
the live mosaic (see WAYBACK below for what this is and how to get a
release ID). Writes into the same satellite cache DetectionMapWidget
already reads, so this is a drop-in refresh -- combine with --force to
replace tiles you already have:
    python prefetch_tiles.py --lat 44.7722 --lon 17.1910 --radius-km 5 \
        --provider satellite --min-zoom 13 --max-zoom 17 \
        --source wayback --wayback-release 71943 --force

WAYBACK (getting a more recent satellite capture)
---------------------------------------------------
The live "satellite" provider is already Esri's continuously-updated
current World Imagery mosaic -- but it's undated, so there's no way to
tell how old a given tile's capture is, and a re-run normally skips
anything already cached even if Esri has since updated that tile.
Esri also publishes "Wayback": the same World Imagery archive, but as
numbered, dated releases. To use it:
  1. Open https://livingatlas.arcgis.com/wayback/ in a browser.
  2. Pan/search to your area of interest and use the timeline to find
     the most recent release with a local change.
  3. Note the release ID shown for that version (e.g. 71943).
  4. Pass it via --source wayback --wayback-release <ID>.
This only affects the "satellite" provider -- "topo" (OpenTopoMap) has
no Wayback equivalent and is always fetched live.

A ROUGH SENSE OF SCALE
-----------------------
Tile count roughly QUADRUPLES with each extra zoom level, and scales
with the SQUARE of the radius. A 5km-radius circle from z13 to z17 is
on the order of a few thousand tiles (tens of MB); the same circle
pushed to z19 can be 50-100x that. This script always prints the tile
count and a rough size estimate before it downloads anything -- read
it before saying yes to a wide zoom range.
"""

import argparse
import math
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed

import MapTiles

# Rough average bytes-per-tile used only for the pre-download size
# estimate shown to the user -- real tiles vary a lot by terrain/zoom,
# this is not meant to be precise, just enough to catch "oh, that's way
# too much" before it happens.
_APPROX_BYTES_PER_TILE = {"satellite": 22_000, "topo": 14_000}

# How many tiles to fetch at once. Kept modest and fixed (not
# configurable) to stay polite to free tile servers that have no API
# key or rate-limit agreement backing this -- see MapTiles.py's
# docstring about Esri/OpenTopoMap usage policies.
_CONCURRENCY = 6

# How many failed-tile errors to print in full when a run finishes with
# failures. Printing every single one would be noise on a bad run with
# thousands of failures; a handful is enough to see the pattern (same
# HTTP status / same exception type / same provider) and diagnose it.
_MAX_ERROR_SAMPLES = 5


def _deg2tile(lat, lon, zoom):
    """Standard slippy-map lon/lat -> integer tile x/y at a given zoom."""
    lat = max(min(lat, 85.05112878), -85.05112878)
    lat_rad = math.radians(lat)
    n = 2 ** zoom
    xtile = int((lon + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n)
    return xtile, ytile


def _bbox_for_radius(center_lat, center_lon, radius_km):
    """
    Simple equirectangular bounding box around a center point -- fine at
    the few-km scale this tool is for (same flat-earth-at-this-scale
    assumption used elsewhere in this app, e.g. DetectionLink's ranging).
    """
    delta_lat = radius_km / 111.32
    delta_lon = radius_km / (111.32 * max(math.cos(math.radians(center_lat)), 0.01))
    return (center_lat - delta_lat, center_lat + delta_lat,
            center_lon - delta_lon, center_lon + delta_lon)


def _tiles_for_zoom(min_lat, max_lat, min_lon, max_lon, zoom):
    x_min, y_min = _deg2tile(max_lat, min_lon, zoom)  # NW corner
    x_max, y_max = _deg2tile(min_lat, max_lon, zoom)  # SE corner
    n = 2 ** zoom
    tiles = []
    for y in range(y_min, y_max + 1):
        if y < 0 or y >= n:
            continue
        for x in range(x_min, x_max + 1):
            tiles.append((x % n, y))  # wrap longitude at +/-180
    return tiles


def _plan(providers, min_lat, max_lat, min_lon, max_lon, min_zoom, max_zoom):
    """Returns {provider: [(z, x, y), ...]} for every tile the plan needs,
    already deduplicated against what's already on disk."""
    plan = {}
    for provider in providers:
        max_supported = MapTiles.PROVIDERS[provider]["max_zoom"]
        needed = []
        for zoom in range(min_zoom, min(max_zoom, max_supported) + 1):
            for x, y in _tiles_for_zoom(min_lat, max_lat, min_lon, max_lon, zoom):
                needed.append((zoom, x, y))
        plan[provider] = needed
    return plan


def _print_estimate(plan, force=False):
    grand_total = 0
    grand_bytes = 0
    for provider, tiles in plan.items():
        already_cached = sum(
            1 for (z, x, y) in tiles
            if __import__("os").path.exists(MapTiles.TileCache.tile_path(provider, z, x, y))
        )
        to_fetch = len(tiles) if force else len(tiles) - already_cached
        approx_mb = to_fetch * _APPROX_BYTES_PER_TILE.get(provider, 18_000) / (1024 * 1024)
        grand_total += to_fetch
        grand_bytes += to_fetch * _APPROX_BYTES_PER_TILE.get(provider, 18_000)
        cached_label = "already cached (will be re-fetched, --force)" if force else "already cached"
        print(f"  {MapTiles.PROVIDERS[provider]['label']:<12} "
              f"{len(tiles):>6} tiles total, {already_cached:>6} {cached_label}, "
              f"{to_fetch:>6} to fetch  (~{approx_mb:.1f} MB)")
    print(f"  {'TOTAL':<12} {grand_total:>6} tiles to fetch  "
          f"(~{grand_bytes / (1024 * 1024):.1f} MB)")
    return grand_total


def _download_all(plan, force=False):
    cache = MapTiles.TileCache()
    if force:
        to_fetch = [(provider, z, x, y)
                    for provider, tiles in plan.items()
                    for (z, x, y) in tiles]
    else:
        to_fetch = [(provider, z, x, y)
                    for provider, tiles in plan.items()
                    for (z, x, y) in tiles
                    if not __import__("os").path.exists(MapTiles.TileCache.tile_path(provider, z, x, y))]
    if not to_fetch:
        print("Nothing to fetch -- already fully cached.")
        return

    total = len(to_fetch)
    done = 0
    failed = 0
    # (item, exception) pairs, capped at _MAX_ERROR_SAMPLES -- see fetch_one.
    error_samples = []
    start = time.monotonic()

    def fetch_one(item):
        provider, z, x, y = item
        cfg = MapTiles.PROVIDERS[provider]
        url = cfg["url_template"].format(z=z, x=x, y=y)
        dest = MapTiles.TileCache.tile_path(provider, z, x, y)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": MapTiles._USER_AGENT})
            with urllib.request.urlopen(req, timeout=8) as resp:
                data = resp.read()
            import os
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            tmp = dest + ".part"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, dest)
            return True
        except Exception as exc:
            return exc

    with ThreadPoolExecutor(max_workers=_CONCURRENCY) as pool:
        futures = {pool.submit(fetch_one, item): item for item in to_fetch}
        for future in as_completed(futures):
            item = futures[future]
            result = future.result()
            done += 1
            if result is not True:
                failed += 1
                if len(error_samples) < _MAX_ERROR_SAMPLES:
                    error_samples.append((item, result))
            if done % 50 == 0 or done == total:
                elapsed = time.monotonic() - start
                rate = done / elapsed if elapsed > 0 else 0
                print(f"\r  {done}/{total} tiles ({failed} failed)  "
                      f"[{rate:.1f} tiles/s]", end="", flush=True)
    print()
    if failed:
        print(f"Done, with {failed} tile(s) that failed to download "
              f"(offline momentarily, rate-limited, or no imagery at that "
              f"tile) -- re-run this script later to retry just those; "
              f"already-cached tiles are skipped automatically.")
        print(f"\nSample of {len(error_samples)} failure(s) "
              f"(out of {failed} total) to help diagnose:")
        for (provider, z, x, y), exc in error_samples:
            print(f"  [{provider} z{z}/{x}/{y}] {type(exc).__name__}: {exc}")
        if failed == total:
            print("\nAll tiles failed -- this usually means something systematic "
                  "(bad URL template, blocked/rejected User-Agent, DNS/firewall, "
                  "or a bad --wayback-release ID) rather than per-tile issues. "
                  "Check the sample errors above.")
    else:
        print("Done -- all tiles cached.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("USAGE")[0],
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lat", type=float, required=True, help="Center latitude")
    parser.add_argument("--lon", type=float, required=True, help="Center longitude")
    parser.add_argument("--radius-km", type=float, required=True, help="Radius around the center point, in km")
    parser.add_argument("--provider", choices=["satellite", "topo", "both"], default="both")
    parser.add_argument("--min-zoom", type=int, default=13)
    parser.add_argument("--max-zoom", type=int, default=17,
                         help="Capped automatically at each provider's max supported zoom")
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    parser.add_argument("--dry-run", action="store_true", help="Print the plan/estimate and exit without downloading")
    parser.add_argument("--force", action="store_true",
                         help="Re-download tiles even if already cached, overwriting them in place. "
                              "Use this to refresh old imagery -- normal runs skip anything already "
                              "on disk, so a stale tile from months ago is never replaced unless you "
                              "pass this flag. Only helps if the source actually has newer imagery "
                              "for that tile; it forces a re-fetch, it doesn't guarantee freshness.")
    parser.add_argument("--source", choices=["live", "wayback"], default="live",
                         help="Where satellite tiles come from. 'live' (default) is Esri's "
                              "continuously-updated current World Imagery mosaic -- newest available, "
                              "but undated. 'wayback' pulls a specific numbered Esri World Imagery "
                              "Wayback release instead, so you know exactly what capture date you "
                              "got (requires --wayback-release). Only affects 'satellite'; 'topo' is "
                              "always fetched live -- see WAYBACK in the module docstring / --help.")
    parser.add_argument("--wayback-release", type=int, default=None,
                         help="Esri World Imagery Wayback release ID to pull from when --source wayback "
                              "is set. Look this up at https://livingatlas.arcgis.com/wayback/ for your "
                              "area of interest.")
    args = parser.parse_args()

    if args.min_zoom > args.max_zoom:
        parser.error("--min-zoom must be <= --max-zoom")
    if args.source == "wayback" and args.wayback_release is None:
        parser.error("--source wayback requires --wayback-release <ID> "
                      "(look it up at https://livingatlas.arcgis.com/wayback/)")

    providers = ["satellite", "topo"] if args.provider == "both" else [args.provider]
    min_lat, max_lat, min_lon, max_lon = _bbox_for_radius(args.lat, args.lon, args.radius_km)

    if args.source == "wayback":
        if "satellite" not in providers:
            print("Note: --source wayback has no effect since 'satellite' isn't in --provider.\n")
        else:
            # Same z/y/x scheme as the live endpoint, just with the release ID as an
            # extra path segment -- swap it in place so every existing code path
            # (including DetectionMapWidget's live on-demand fetches) picks it up
            # with no other changes, and tiles land in the same tile_cache/satellite
            # folder as before.
            MapTiles.PROVIDERS["satellite"]["url_template"] = (
                "https://wayback.maptiles.arcgis.com/arcgis/rest/services/"
                f"World_Imagery/MapServer/tile/{args.wayback_release}/{{z}}/{{y}}/{{x}}"
            )
            print(f"Source: Esri World Imagery Wayback release {args.wayback_release} "
                  f"(writes into the same tile_cache/satellite/ folder)\n")

    print(f"Area: {args.radius_km:.1f}km radius around ({args.lat:.5f}, {args.lon:.5f})")
    print(f"Zoom: {args.min_zoom}-{args.max_zoom}   Providers: {', '.join(providers)}")
    print(f"Cache directory: {MapTiles.CACHE_ROOT}")
    if args.force:
        print("Mode: --force -- ALL matching tiles will be re-downloaded and overwritten, "
              "including ones already cached.")
    print()

    plan = _plan(providers, min_lat, max_lat, min_lon, max_lon, args.min_zoom, args.max_zoom)
    total_to_fetch = _print_estimate(plan, force=args.force)

    if args.dry_run:
        return

    if total_to_fetch == 0:
        print("\nNothing to fetch -- already fully cached.")
        return

    if not args.yes:
        verb = "Re-download and overwrite" if args.force else "Download"
        answer = input(f"\n{verb} {total_to_fetch} tiles? [y/N] ").strip().lower()
        if answer != "y":
            print("Cancelled.")
            return

    print()
    _download_all(plan, force=args.force)


if __name__ == "__main__":
    main()