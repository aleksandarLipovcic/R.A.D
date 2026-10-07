"""
DetectionMapWidget.py — Supplementary detection/map display window
=======================================================================
This is deliberately a SEPARATE top-level window from the pilot's FPV
panel, not another pane fighting for space in the same layout -- the
whole point (per the paralel-detection-window design) is that nothing
this widget does can ever obstruct or contend with the live video feed.

Like FPVWidget, this widget mostly never touches raw pixel data. It
receives already-built DetectionRecord objects (small structs: class
name, confidence, lat/lon, range, a screenshot *path*) from
DetectionWorker and displays them with ordinary Tk widgets. Two places
open actual image bytes, both on demand rather than every poll tick:
loading the saved screenshot off disk when the pilot clicks a specific
detection, and the live annotated-feed pane (on by default), which
polls DetectionLink::getLatestAnnotatedFrameJpeg() at a fixed ~5-6fps --
a deliberate, narrow, clearly-labeled exception to the no-pixels rule,
never the pilot's primary video path. The live feed is what's shown
whenever nothing is selected; clicking a detection row temporarily
swaps this pane to that detection's saved screenshot instead (see
_on_select / _return_to_live), and the live poll keeps running in the
background the whole time so going back to it is instant.

The map panel is now backed by real XYZ tiles (see MapTiles.py), with
two switchable providers -- "Satellite" (Esri World Imagery) and
"Topographic" (OpenTopoMap, a contour-line style closest to a free
military-map look) -- selected via the radio buttons above the canvas.
Tiles are cached to disk per-provider the first time an area is viewed
(see MapTiles.TileCache): if a tile is already cached it's read
straight off disk and drawn immediately; if it isn't, a background
download is kicked off (never blocking the Tk thread) and that cell is
left as a flat placeholder color until the tile lands and a redraw
picks it up (see _poll_tile_downloads). No network at all is required
once an area's tiles are cached from a prior flight. Every detection
and the drone's own position are plotted directly in standard Web
Mercator pixel space at whatever zoom the current view auto-fits to
(see _fit_zoom/_mercator_pixel) -- the same coordinate system the
tiles themselves are drawn in, so pins line up with the imagery exactly
rather than needing a separate projection reconciled against it. A
scale bar and north-up compass mark are still drawn on top (see
_draw_scale_bar), since even with real imagery a quick "how far is
that" glance shouldn't require the detection list. The detection list
remains the primary, trustworthy source of exact values (lat/lon,
distance, bearing) for any single sighting.

ONE ROW PER OBJECT: DetectionLink writes a record when an object is first
confirmed, when it has moved, and as a periodic refresh -- a parked car
watched for three minutes therefore produces many records. The list
shows each object (track_id) once and updates that row in place: class,
BEST confidence, number of sightings, last-seen time, fused position and
its uncertainty radius ("± m"). Selecting the row shows the screenshot
of the most confident sighting. The map draws the fused position with a
circle of that radius -- the area to send a search team to.

Records are grouped by track_id (see DetectionLink's tracker): only the
newest sighting of a given track is drawn as the bright "current" pin;
earlier sightings of that same track are drawn as a small fading trail
so a single object logged 3 times over a minute of movement reads as
one moving pin with a breadcrumb trail, not three unrelated dots.

SEARCH (report, section 15: detections as an index over the video)
-------------------------------------------------------------------
Two kinds of search, both above the list:

* Object filter -- class (or "vehicle" = any vehicle class), minimum best
  confidence, minimum number of sightings, "last N minutes", only objects
  with a map position, and free text (object number or class). The list
  AND the map show only matching objects; "Showing X of Y" says how many
  are hidden.
* Scene search -- frames with "N or more" / "exactly N" objects of a class
  at the same time (e.g. >= 2 persons, >= 5 vehicles, 0 objects). It runs
  on DetectionLink's frame index (get_frame_index_since: the confirmed
  objects of every detection pass), not on the records -- records are only
  written when an object is new, moved or on a refresh, so they cannot say
  what was visible together. Matching passes less than ~1 s apart are
  merged into one scene (the rule from section 15.2). Selecting a scene
  limits the list and map to the objects in it; "Clear" removes it.
  Quick buttons: person, >= 2 persons, vehicle, >= 2 vehicles, no objects.

The frame index needs a DroneBackend built with get_frame_index_since();
with an older build the scene search says so and the object filter still
works.

VIEW STATE — auto-follow vs. manual look-around
------------------------------------------------
By default the map is in "follow" mode (self._follow_drone = True): every
redraw re-fits the view to whatever's currently on the map (all track
points + the drone), exactly as before. The pilot can now break out of
that to look around: dragging the map, using the +/- zoom buttons, or
the mouse wheel/scroll all switch to manual mode (self._follow_drone =
False), at which point self._map_zoom / self._map_center are held fixed
across redraws instead of being recomputed from the data every time.
"Center on drone" flips follow mode back on, which snaps the view back
to the auto-fit framing (and keeps it following from then on).

Auto-fit is capped at _AUTO_FIT_MAX_ZOOM regardless of a provider's own
maximum, which matters more than it sounds: with a single point on the
map (e.g. just the drone, no georeferenced detections yet), the old
"walk zoom down from max until the bbox fits" search degenerates to a
zero-size box that always "fits", so it returned the provider's raw
max_zoom immediately -- 19 for Esri satellite vs. 17 for OpenTopoMap in
this app's config, even though both providers were prefetched (see
prefetch_tiles.py) over the SAME zoom range. That's why satellite could
come up blank ("no imagery at this zoom") right after a prefetch while
topographic loaded fine at the same location: the two providers'
auto-fit zoom silently diverged. Capping the auto-fit ceiling to match
what prefetch_tiles.py actually downloads keeps both providers landing
on a zoom level that's actually cached; the manual +/- controls can
still go deeper than the cap on either provider if the pilot wants to
push past it (kicking off a live download if online, or showing the
flat placeholder if not).
"""

import math
import queue
import time
import tkinter as tk
from tkinter import ttk

import MapTiles

try:
    from PIL import Image, ImageTk
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False


# Ceiling used only for the auto-fit ("follow drone") zoom search in
# _fit_zoom -- deliberately independent of any one provider's own
# max_zoom. Matches prefetch_tiles.py's own --max-zoom default, so a
# pilot who prefetched with the default range gets a follow-mode view
# that's actually cached on both providers instead of one of them
# silently landing above what was downloaded (see module docstring).
# Manual zoom (the +/- buttons, mouse wheel) is NOT capped by this --
# it can still go up to a provider's real max_zoom.
_AUTO_FIT_MAX_ZOOM = 17

# Class choices for search. "vehicle" = any of VEHICLE_CLASSES (the
# car <-> large_vehicle confusion makes the vehicle subclass unreliable on
# the FPV camera, see the report's section 14).
VEHICLE_CLASSES = ("car", "large_vehicle", "motorcycle", "other_vehicle")
SEARCH_CLASSES = ("person",) + VEHICLE_CLASSES
_TIME_WINDOWS = {"all time": None, "last 1 min": 60, "last 5 min": 300,
                 "last 15 min": 900, "last 60 min": 3600}
MAX_FRAME_INDEX = 50000     # same cap as DetectionLink::kMaxFrameIndexEntries


def class_matches(class_name: str, wanted: str) -> bool:
    """wanted: 'any', 'vehicle' or a class name."""
    if wanted in ("any", "all", ""):
        return True
    if wanted == "vehicle":
        return class_name in VEHICLE_CLASSES
    return class_name == wanted


def find_scenes(frames, wanted: str = "person", op: str = ">=", n: int = 1, gap_ms: int = None):
    """
    Scene search over the frame index (report, section 15.2).

    frames: [(timestamp_ms, ((track_id, class_name), ...)), ...] in time order.
    A frame matches when the number of objects of class `wanted` in it is
    >= n (op ">=") or == n (op "="). Matching frames less than gap_ms apart
    are merged into one scene (default: 1 s, or 3 detection intervals if
    passes are slower than that).

    Returns [{"start_ms", "end_ms", "frames", "max_count", "track_ids"}],
    oldest first; track_ids are the objects of class `wanted` seen in the
    scene (every object for "any").
    """
    if not frames:
        return []
    if gap_ms is None:
        steps = sorted(b[0] - a[0] for a, b in zip(frames, frames[1:]))
        median = steps[len(steps) // 2] if steps else 0
        gap_ms = max(1000, 3 * median)
    scenes = []
    cur = None
    for ts, objects in frames:
        ids = [tid for tid, cls in objects if class_matches(cls, wanted)]
        count = len(ids)
        hit = count >= n if op == ">=" else count == n
        if not hit:
            continue
        if cur is not None and ts - cur["end_ms"] <= gap_ms:
            cur["end_ms"] = ts
            cur["frames"] += 1
            cur["max_count"] = max(cur["max_count"], count)
            cur["track_ids"].update(ids)
        else:
            cur = {"start_ms": ts, "end_ms": ts, "frames": 1, "max_count": count,
                   "track_ids": set(ids)}
            scenes.append(cur)
    return scenes


def _mercator_pixel(lat, lon, zoom):
    """
    Standard Web Mercator lon/lat -> global pixel coordinates at a given
    zoom level (tile_size=256, same convention every XYZ tile server
    uses). This is the ONE coordinate system both the tile mosaic
    (_draw_tile_mosaic) and the pin/drone markers (_redraw_map's
    to_canvas) are placed in, so nothing can drift out of alignment with
    the imagery underneath it.
    """
    lat = max(min(lat, 85.05112878), -85.05112878)  # Mercator is undefined at the poles
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n * MapTiles.TILE_SIZE
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n * MapTiles.TILE_SIZE
    return x, y


def _mercator_lonlat(x, y, zoom):
    """
    Inverse of _mercator_pixel: global Web Mercator pixel coordinates at
    a given zoom -> (lat, lon). Needed for panning (turning a canvas drag
    delta back into a lat/lon center) and for cursor-centered zoom
    (finding what lat/lon is under the mouse before changing zoom so the
    same point can be kept under it afterward).
    """
    n = 2.0 ** zoom
    lon = x / (n * MapTiles.TILE_SIZE) * 360.0 - 180.0
    y_frac = y / (n * MapTiles.TILE_SIZE)
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * y_frac)))
    lat = math.degrees(lat_rad)
    return lat, lon


class DetectionMapWidget(tk.Toplevel):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.title("Detections & map")
        self.geometry("1100x760")   # room for the search panel above the list
        self.configure(bg="#1a1a1a")

        # In-memory record store. Kept here (not just in DetectionWorker)
        # because this widget owns the display-facing view of the data --
        # e.g. filtering by class, re-selecting an old pin -- independent
        # of the worker's own incremental "new since last poll" bookkeeping.
        self._records = []          # all DetectionRecord objects seen so far
        self._selected_photo = None  # keep a reference so Tk doesn't GC it

        # ── Map state ─────────────────────────────────────────────────
        self._map_origin = None   # (lat, lon) of the first-ever georeferenced fix, or None -- only used as
                                   # the "has anything arrived yet" gate for the "Waiting for GPS fix" message
        self._drone_telemetry = None   # (lat, lon) or None -- see set_drone_telemetry()
        # track_id -> list of (lat, lon) in arrival order, so a single
        # object's repeated sightings draw as one pin + trail instead of
        # a pile of unrelated dots (see module docstring).
        self._track_positions = {}
        # track key -> {"latest": rec, "best": rec, "count": n, "radius": m}
        # (one Treeview row per object, see add_records / module docstring)
        self._tracks = {}
        self._track_order = {}       # key -> arrival index (list order is stable under filtering)
        self._visible_keys = set()   # keys currently shown in the list (and on the map)
        # Frame index from DetectionLink (see add_frame_index / find_scenes):
        # [(timestamp_ms, ((track_id, class_name), ...)), ...]
        self._frames = []
        self._frame_index_supported = None   # None = unknown yet, set by set_frame_index_supported
        self._scenes = []                    # result of the last scene search
        self._scene_filter = None            # set of track ids of the selected scene, or None
        self._map_zoom = None      # zoom the map is currently drawn at (auto-fit or manual, see _follow_drone)
        self._map_center = None    # (lat, lon) the map is currently centered on

        # ── Manual look-around state ─────────────────────────────────
        # True (the default): every redraw re-fits zoom/center to the
        # data, same as before this feature existed. Panning, the +/-
        # buttons, or the mouse wheel set this False and freeze
        # _map_zoom/_map_center until "Center on drone" sets it back.
        self._follow_drone = True
        self._drag_start = None            # (canvas_x, canvas_y) at drag start, or None
        self._drag_start_center_px = None  # _map_center's Mercator pixel coords at drag start

        # ── Tile provider / cache state ──────────────────────────────
        self._tile_cache = MapTiles.TileCache()
        self._map_provider = "satellite"
        # Decoded-tile cache, keyed (provider, zoom, x, y) -> PIL.Image.
        # Unbounded for now (cleared wholesale on provider switch) --
        # fine for a single flight's worth of panning/zooming; if this
        # ever shows up as a memory problem on long sessions, cap it with
        # an LRU instead of clearing it all at once.
        self._tile_image_cache = {}
        self._mosaic_photo = None  # keep a ref so Tk doesn't GC the composited basemap image
        self._tile_poll_job = None
        self._resize_job = None

        self._build_layout()
        self._poll_tile_downloads()

    # =====================================================================
    # Layout
    # =====================================================================

    def _build_layout(self):
        # ── Engine status banner ─────────────────────────────────────────
        # An empty pin list looks identical whether nothing has been
        # detected yet or the detection engine never started at all (e.g.
        # the ONNX model path was wrong) -- that ambiguity used to only
        # get resolved by reading the console. set_engine_status() (called
        # by the owning app right after this window is created, and again
        # on any later retry) makes that state visible right here instead.
        self._status_var = tk.StringVar(value="●  Detection engine: unknown")
        self._status_lbl = tk.Label(
            self, textvariable=self._status_var,
            fg="#888888", bg="#1a1a1a",
            font=("Segoe UI", 9, "bold"), anchor="w",
        )
        self._status_lbl.pack(fill="x", padx=8, pady=(8, 0))

        # Second status line, driven by set_telemetry_status() (called by
        # the owning app's poll loop -- see _pump_detection_records in
        # DroneCockpitUI.py). An empty map is ambiguous for a second reason
        # beyond the engine-status line above: the engine can be running,
        # actively detecting things, and still never plot a single pin if
        # GPS isn't good enough for DetectionLink to georeference anything
        # (see gps.position_usable / _get_detection_telemetry in the main
        # app). Surfacing that here means a pilot never has to cross-check
        # the FC status panel just to understand why the map is blank.
        self._telemetry_status_var = tk.StringVar(value="●  Telemetry: unknown")
        self._telemetry_status_lbl = tk.Label(
            self, textvariable=self._telemetry_status_var,
            fg="#888888", bg="#1a1a1a",
            font=("Segoe UI", 9), anchor="w",
        )
        self._telemetry_status_lbl.pack(fill="x", padx=8, pady=(2, 0))

        paned = ttk.PanedWindow(self, orient="horizontal")
        paned.pack(fill="both", expand=True, pady=(6, 0))

        # ── Left: detection list ────────────────────────────────────
        left = tk.Frame(paned, bg="#1a1a1a")
        paned.add(left, weight=1)

        tk.Label(left, text="Detections", fg="#ffffff", bg="#1a1a1a",
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=8, pady=(8, 2))

        self._build_search_panel(left)

        # Scene results above the object list, in a vertical PanedWindow:
        # drag the sash between the two tables to give either more rows.
        left_split = ttk.PanedWindow(left, orient="vertical")
        left_split.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        scene_frame = tk.Frame(left_split, bg="#1a1a1a")
        list_frame = tk.Frame(left_split, bg="#1a1a1a")
        left_split.add(scene_frame, weight=1)
        left_split.add(list_frame, weight=3)

        self._scene_list = ttk.Treeview(scene_frame, columns=("when", "dur", "max", "objs"),
                                        show="headings", height=3, selectmode="browse")
        for col, text, width, anchor in (("when", "Scene", 120, "w"), ("dur", "Length", 60, "e"),
                                         ("max", "Max at once", 75, "e"), ("objs", "Objects", 60, "e")):
            self._scene_list.heading(col, text=text)
            self._scene_list.column(col, width=width, anchor=anchor)
        scene_scroll = ttk.Scrollbar(scene_frame, orient="vertical", command=self._scene_list.yview)
        self._scene_list.configure(yscrollcommand=scene_scroll.set)
        scene_scroll.pack(side="right", fill="y")
        self._scene_list.pack(side="left", fill="both", expand=True)
        self._scene_list.bind("<<TreeviewSelect>>", self._on_scene_select)

        columns = ("track", "class", "conf", "seen", "time", "geo", "acc")
        self._tree = ttk.Treeview(list_frame, columns=columns, show="headings", selectmode="browse")
        self._tree.heading("track", text="Obj.")
        self._tree.heading("class", text="Class")
        self._tree.heading("conf", text="Best")
        self._tree.heading("seen", text="Seen")
        self._tree.heading("time", text="Last seen")
        self._tree.heading("geo", text="Lat / Lon")
        self._tree.heading("acc", text="± m")
        self._tree.column("track", width=40, anchor="center")
        self._tree.column("class", width=80)
        self._tree.column("conf", width=45, anchor="e")
        self._tree.column("seen", width=45, anchor="e")
        self._tree.column("time", width=70, anchor="center")
        self._tree.column("geo", width=140)
        self._tree.column("acc", width=50, anchor="e")
        tree_scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self._tree.yview)
        self._tree.configure(yscrollcommand=tree_scroll.set)
        tree_scroll.pack(side="right", fill="y")
        self._tree.pack(side="left", fill="both", expand=True)
        self._tree.bind("<<TreeviewSelect>>", self._on_select)
        for col in columns:
            self._tree.heading(col, command=lambda c=col: self._sort_by(c))
        self._sort_col = None
        self._sort_desc = False

        # ── Right: pin map + screenshot preview ───────────────────────
        right = tk.Frame(paned, bg="#1a1a1a")
        paned.add(right, weight=2)
        # Map above the preview, in a vertical PanedWindow: drag the sash
        # to give the map or the screenshot/live feed more height.
        right_split = ttk.PanedWindow(right, orient="vertical")
        right_split.pack(fill="both", expand=True)
        map_frame = tk.Frame(right_split, bg="#1a1a1a")
        preview_frame = tk.Frame(right_split, bg="#1a1a1a")
        right_split.add(map_frame, weight=1)
        right_split.add(preview_frame, weight=1)

        map_header = tk.Frame(map_frame, bg="#1a1a1a")
        map_header.pack(fill="x", padx=8, pady=(8, 0))
        tk.Label(map_header, text="Map", fg="#ffffff", bg="#1a1a1a",
                 font=("Segoe UI", 11, "bold")).pack(side="left")

        # Satellite / topographic toggle. Each provider's tiles are
        # cached to disk independently (see MapTiles.TileCache) --
        # switching back and forth never re-downloads anything that was
        # already fetched under either one.
        self._map_provider_var = tk.StringVar(value=self._map_provider)
        for key, cfg in MapTiles.PROVIDERS.items():
            tk.Radiobutton(
                map_header, text=cfg["label"], value=key, variable=self._map_provider_var,
                command=self._on_map_provider_change, fg="#cccccc", bg="#1a1a1a",
                selectcolor="#333333", activebackground="#1a1a1a", activeforeground="#ffffff",
                indicatoron=False, padx=10, relief="flat", borderwidth=1,
            ).pack(side="right", padx=(4, 0))

        # Zoom +/- and "Center on drone" -- packed after the provider
        # loop above so they land just to its left (see _build_layout
        # packing order notes: repeated side="right" packs stack inward
        # from the right edge). Center on drone re-enables follow mode
        # (see _on_center_on_drone); +/- and drag/scroll disable it
        # (see _adjust_zoom / _zoom_at_point / _on_map_press).
        zoom_controls = tk.Frame(map_header, bg="#1a1a1a")
        zoom_controls.pack(side="right", padx=(4, 10))

        self._center_btn = tk.Button(
            zoom_controls, text="\u2299 Center on drone", command=self._on_center_on_drone,
            fg="#cccccc", bg="#2a2a2a", activebackground="#3a3a3a", activeforeground="#ffffff",
            relief="flat", padx=8, borderwidth=1,
        )
        self._center_btn.pack(side="left", padx=(0, 8))

        self._zoom_out_btn = tk.Button(
            zoom_controls, text="\u2212", command=lambda: self._adjust_zoom(-1),
            fg="#cccccc", bg="#2a2a2a", activebackground="#3a3a3a", activeforeground="#ffffff",
            relief="flat", width=2, borderwidth=1,
        )
        self._zoom_out_btn.pack(side="left")

        self._zoom_in_btn = tk.Button(
            zoom_controls, text="+", command=lambda: self._adjust_zoom(1),
            fg="#cccccc", bg="#2a2a2a", activebackground="#3a3a3a", activeforeground="#ffffff",
            relief="flat", width=2, borderwidth=1,
        )
        self._zoom_in_btn.pack(side="left", padx=(2, 0))

        self._map_canvas = tk.Canvas(map_frame, bg="#111a14", height=260, highlightthickness=0)
        self._map_canvas.pack(fill="both", expand=True, padx=8, pady=(4, 2))
        self._map_canvas.bind("<Configure>", self._on_map_canvas_resize)

        # Look-around controls: drag to pan, wheel/scroll to zoom toward
        # the cursor. All of these hand off to the same manual-view state
        # that the +/- buttons use (see _on_map_press/_on_map_drag and
        # _on_mouse_wheel).
        self._map_canvas.bind("<ButtonPress-1>", self._on_map_press)
        self._map_canvas.bind("<B1-Motion>", self._on_map_drag)
        self._map_canvas.bind("<ButtonRelease-1>", self._on_map_release)
        self._map_canvas.bind("<MouseWheel>", self._on_mouse_wheel)  # Windows / macOS
        self._map_canvas.bind("<Button-4>", self._on_mouse_wheel)    # Linux (X11) scroll up
        self._map_canvas.bind("<Button-5>", self._on_mouse_wheel)    # Linux (X11) scroll down

        self._map_attribution_var = tk.StringVar(value="")
        tk.Label(map_frame, textvariable=self._map_attribution_var,
                 fg="#888888", bg="#1a1a1a", font=("Segoe UI", 8)).pack(anchor="w", padx=8)

        # ── Live annotated feed (primary view for this pane) ──────────
        # On by default: this pane's job, day to day, is to show what
        # the detection engine sees RIGHT NOW with its boxes drawn on
        # it -- not a table of numbers, an actual moving picture -- so
        # the pilot doesn't have to open the FPV panel and mentally
        # cross-reference it against the detection list. Clicking a
        # detection row swaps this pane to that detection's saved
        # screenshot until the pilot clicks "Back to live" (or another
        # row); the live poll keeps running underneath the whole time
        # (see _poll_live_frame), so switching back is instant, not a
        # re-buffer.
        live_row = tk.Frame(preview_frame, bg="#1a1a1a")
        live_row.pack(fill="x", padx=8, pady=(4, 0))
        self._live_enabled = tk.BooleanVar(value=True)
        self._live_check = tk.Checkbutton(
            live_row, text="Live annotated feed (~20-25 fps)", variable=self._live_enabled,
            command=self._on_live_toggle, fg="#cccccc", bg="#1a1a1a",
            selectcolor="#1a1a1a", activebackground="#1a1a1a", activeforeground="#ffffff",
        )
        self._live_check.pack(side="left", anchor="w")

        self._preview_status_var = tk.StringVar(value="●  LIVE")
        self._preview_status_lbl = tk.Label(
            live_row, textvariable=self._preview_status_var, fg="#00ff88", bg="#1a1a1a",
            font=("Segoe UI", 9, "bold"),
        )
        self._preview_status_lbl.pack(side="left", padx=(12, 0))

        # Only packed (visible) while a saved screenshot is being shown
        # instead of the live feed -- see _show_back_to_live_button() /
        # _hide_back_to_live_button().
        self._back_to_live_btn = tk.Button(
            live_row, text="\u25c0 Back to live", command=self._return_to_live,
            fg="#000000", bg="#00e0ff", relief="flat", padx=6,
        )
        self._back_to_live_btn_visible = False

        self._preview_label = tk.Label(preview_frame, bg="#000000")
        self._preview_label.pack(fill="both", expand=True, padx=8, pady=8)

        self._detail_lbl = tk.Label(preview_frame, text="Live feed -- click a detection for its saved screenshot",
                                     fg="#cccccc", bg="#1a1a1a", font=("Segoe UI", 9), justify="left")
        self._detail_lbl.pack(anchor="w", padx=8, pady=(0, 8))

        # Set by the owning app via set_detection_link() -- needed for
        # the live feed above; everything else in this widget only ever
        # sees DetectionRecord structs, never the link.
        self._link = None
        self._live_job = None
        self._live_photo = None        # most recent decoded live frame -- kept alive even while viewing a saved shot
        self._viewing_saved_id = None  # detection id currently shown in place of the live feed, or None

    # =====================================================================
    # Search (see module docstring, "SEARCH")
    # =====================================================================

    def _build_search_panel(self, parent) -> None:
        lbl = dict(fg="#cccccc", bg="#1a1a1a", font=("Segoe UI", 9))
        spin = dict(width=5, bg="#2a2a2a", fg="#ffffff", insertbackground="#ffffff",
                    buttonbackground="#2a2a2a", relief="flat", font=("Segoe UI", 9))
        btn = dict(fg="#cccccc", bg="#2a2a2a", activebackground="#3a3a3a",
                   activeforeground="#ffffff", relief="flat", padx=6, borderwidth=1,
                   font=("Segoe UI", 9))
        classes = ("all", "vehicle") + SEARCH_CLASSES

        box = tk.Frame(parent, bg="#1a1a1a")
        box.pack(fill="x", padx=8, pady=(0, 4))

        # Row 1: object filter
        r1 = tk.Frame(box, bg="#1a1a1a")
        r1.pack(fill="x")
        tk.Label(r1, text="Class", **lbl).pack(side="left")
        self._flt_class = tk.StringVar(value="all")
        ttk.Combobox(r1, textvariable=self._flt_class, values=classes, width=12,
                     state="readonly").pack(side="left", padx=(2, 6))
        tk.Label(r1, text="Conf \u2265", **lbl).pack(side="left")
        self._flt_conf = tk.StringVar(value="0.00")
        tk.Spinbox(r1, from_=0.0, to=1.0, increment=0.05, format="%.2f",
                   textvariable=self._flt_conf, **spin).pack(side="left", padx=(2, 6))
        tk.Label(r1, text="Seen \u2265", **lbl).pack(side="left")
        self._flt_seen = tk.StringVar(value="0")
        tk.Spinbox(r1, from_=0, to=100000, increment=1, textvariable=self._flt_seen,
                   **spin).pack(side="left", padx=(2, 0))

        # Row 2: time window, map-only, text
        r2 = tk.Frame(box, bg="#1a1a1a")
        r2.pack(fill="x", pady=(3, 0))
        self._flt_window = tk.StringVar(value="all time")
        ttk.Combobox(r2, textvariable=self._flt_window, values=tuple(_TIME_WINDOWS), width=11,
                     state="readonly").pack(side="left")
        self._flt_geo = tk.BooleanVar(value=False)
        tk.Checkbutton(r2, text="on map only", variable=self._flt_geo, selectcolor="#1a1a1a",
                       activebackground="#1a1a1a", activeforeground="#ffffff",
                       **lbl).pack(side="left", padx=(6, 6))
        tk.Label(r2, text="Find", **lbl).pack(side="left")
        self._flt_text = tk.StringVar(value="")
        tk.Entry(r2, textvariable=self._flt_text, width=10, bg="#2a2a2a", fg="#ffffff",
                 insertbackground="#ffffff", relief="flat",
                 font=("Segoe UI", 9)).pack(side="left", padx=(2, 6))
        tk.Button(r2, text="Reset", command=self.reset_search, **btn).pack(side="left")

        # Row 3: scene search over the frame index
        r3 = tk.Frame(box, bg="#1a1a1a")
        r3.pack(fill="x", pady=(5, 0))
        tk.Label(r3, text="Frames with", **lbl).pack(side="left")
        self._q_op = tk.StringVar(value=">=")
        ttk.Combobox(r3, textvariable=self._q_op, values=(">=", "="), width=3,
                     state="readonly").pack(side="left", padx=(2, 2))
        self._q_n = tk.StringVar(value="1")
        tk.Spinbox(r3, from_=0, to=100, increment=1, textvariable=self._q_n,
                   **spin).pack(side="left", padx=(0, 2))
        self._q_class = tk.StringVar(value="person")
        ttk.Combobox(r3, textvariable=self._q_class, values=("any", "vehicle") + SEARCH_CLASSES,
                     width=12, state="readonly").pack(side="left", padx=(0, 2))
        tk.Label(r3, text="at once", **lbl).pack(side="left")
        tk.Button(r3, text="Search", command=self.run_scene_search, **btn).pack(side="left", padx=(6, 0))

        # Row 4: quick queries (the report's section 15.2 examples)
        r4 = tk.Frame(box, bg="#1a1a1a")
        r4.pack(fill="x", pady=(3, 0))
        tk.Label(r4, text="Quick:", **lbl).pack(side="left")
        for text, q in (("Person", ("person", ">=", 1)), ("\u2265 2 persons", ("person", ">=", 2)),
                        ("Vehicle", ("vehicle", ">=", 1)), ("\u2265 2 vehicles", ("vehicle", ">=", 2)),
                        ("No objects", ("any", "=", 0))):
            tk.Button(r4, text=text, command=lambda q=q: self.run_scene_search(*q),
                      **btn).pack(side="left", padx=(4, 0))

        self._search_status = tk.StringVar(value="")
        status_row = tk.Frame(box, bg="#1a1a1a")
        status_row.pack(fill="x", pady=(2, 0))
        tk.Label(status_row, textvariable=self._search_status, anchor="w", justify="left",
                 fg="#e0d060", bg="#1a1a1a", font=("Segoe UI", 9)).pack(side="left", fill="x", expand=True)
        self._clear_scene_btn = tk.Button(status_row, text="Clear scene", command=self.clear_scene, **btn)

        for var in (self._flt_class, self._flt_conf, self._flt_seen, self._flt_window,
                    self._flt_geo, self._flt_text):
            var.trace_add("write", lambda *_: self._apply_filters())
        self._window_refresh_job = None

    def _int(self, var, default=0):
        try:
            return int(float(var.get()))
        except (tk.TclError, ValueError):
            return default

    def _float(self, var, default=0.0):
        try:
            return float(var.get())
        except (tk.TclError, ValueError):
            return default

    def _matches(self, key, info, now_ms) -> bool:
        rec = info["latest"]
        if not class_matches(rec.class_name, self._flt_class.get()):
            return False
        if info["best_conf"] < self._float(self._flt_conf) - 1e-9:
            return False
        if info["seen"] < self._int(self._flt_seen):
            return False
        window_s = _TIME_WINDOWS.get(self._flt_window.get())
        if window_s is not None and info["last_ms"] < now_ms - window_s * 1000:
            return False
        if self._flt_geo.get() and not rec.georeferenced:
            return False
        text = self._flt_text.get().strip().lower()
        if text:
            tid = str(getattr(rec, "track_id", 0))
            if text.lstrip("#").isdigit():
                if text.lstrip("#") != tid:
                    return False
            elif text not in rec.class_name.lower():
                return False
        if self._scene_filter is not None and getattr(rec, "track_id", 0) not in self._scene_filter:
            return False
        return True

    def _filters_active(self) -> bool:
        return (self._flt_class.get() != "all" or self._float(self._flt_conf) > 0
                or self._int(self._flt_seen) > 0 or _TIME_WINDOWS.get(self._flt_window.get())
                or self._flt_geo.get() or bool(self._flt_text.get().strip())
                or self._scene_filter is not None)

    def _row_values(self, key, info):
        rec = info["latest"]
        radius = info["radius"]
        t = time.strftime("%H:%M:%S", time.localtime(info["last_ms"] / 1000.0))
        geo = f"{rec.latitude:.5f}, {rec.longitude:.5f}" if rec.georeferenced else "no GPS fix"
        acc = f"±{radius:.0f}" if rec.georeferenced and radius > 0 else ("?" if rec.georeferenced else "-")
        return (getattr(rec, "track_id", 0), rec.class_name, f"{info['best_conf']:.2f}",
                info["seen"], t, geo, acc)

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def _sync_row(self, key, now_ms) -> None:
        """Insert / update / remove one object's row according to the filters."""
        info = self._tracks[key]
        iid = f"t{key}"
        if self._matches(key, info, now_ms):
            values = self._row_values(key, info)
            if self._tree.exists(iid):
                self._tree.item(iid, values=values)
            else:
                order = self._track_order[key]
                index = "end"
                if self._sort_col is None:
                    for pos, child in enumerate(self._tree.get_children()):
                        if self._track_order.get(self._key_from_iid(child), -1) > order:
                            index = pos
                            break
                self._tree.insert("", index, iid=iid, values=values)
            self._visible_keys.add(key)
        else:
            if self._tree.exists(iid):
                self._tree.delete(iid)
            self._visible_keys.discard(key)

    def _apply_filters(self) -> None:
        """Re-evaluate every object against the filters (list + map)."""
        now_ms = self._now_ms()
        for key in sorted(self._tracks, key=lambda k: self._track_order[k]):
            self._sync_row(key, now_ms)
        if self._sort_col is not None:
            self._resort()
        self._update_search_status()
        self._redraw_map()
        # "last N min" drops objects as time passes: re-check periodically.
        if self._window_refresh_job is not None:
            self.after_cancel(self._window_refresh_job)
            self._window_refresh_job = None
        if _TIME_WINDOWS.get(self._flt_window.get()):
            self._window_refresh_job = self.after(5000, self._apply_filters)

    def _update_search_status(self) -> None:
        total, shown = len(self._tracks), len(self._visible_keys)
        parts = []
        if self._filters_active():
            parts.append(f"Showing {shown} of {total} objects")
        if self._scene_filter is not None:
            parts.append(self._scene_desc)
        self._search_status.set("  ·  ".join(parts))
        if self._scene_filter is not None:
            self._clear_scene_btn.pack(side="right")
        else:
            self._clear_scene_btn.pack_forget()

    def reset_search(self) -> None:
        self._scene_filter = None
        self._flt_class.set("all")
        self._flt_conf.set("0.00")
        self._flt_seen.set("0")
        self._flt_window.set("all time")
        self._flt_geo.set(False)
        self._flt_text.set("")
        self._apply_filters()

    def _sort_by(self, col) -> None:
        if self._sort_col == col:
            self._sort_desc = not self._sort_desc
        else:
            self._sort_col, self._sort_desc = col, col in ("conf", "seen", "time")
        self._resort()

    def _resort(self) -> None:
        cols = ("track", "class", "conf", "seen", "time", "geo", "acc")
        idx = cols.index(self._sort_col)

        def key(iid):
            v = self._tree.item(iid, "values")[idx]
            try:
                return (0, float(str(v).lstrip("±")))
            except ValueError:
                return (1, str(v))
        for pos, iid in enumerate(sorted(self._tree.get_children(), key=key, reverse=self._sort_desc)):
            self._tree.move(iid, "", pos)

    # ── Frame index / scenes ────────────────────────────────────────

    def set_frame_index_supported(self, supported: bool) -> None:
        """Called by the owning app: False if this DroneBackend has no frame index."""
        self._frame_index_supported = supported

    def add_frame_index(self, entries) -> None:
        """
        entries: [(timestamp_ms, ((track_id, class_name), ...)), ...] from
        DetectionWorker.get_new_frame_index(). Also keeps each object's
        "last seen" current between records.
        """
        if not entries:
            return
        self._frame_index_supported = True
        self._frames.extend(entries)
        if len(self._frames) > MAX_FRAME_INDEX:
            del self._frames[:len(self._frames) - MAX_FRAME_INDEX]
        now_ms = self._now_ms()
        touched = set()
        for ts, objects in entries:
            for tid, _cls in objects:
                info = self._tracks.get(tid)
                if info is not None and ts > info["last_ms"]:
                    info["last_ms"] = ts
                    touched.add(tid)
        for key in touched:
            self._sync_row(key, now_ms)

    def run_scene_search(self, wanted=None, op=None, n=None) -> list:
        if wanted is not None:
            self._q_class.set(wanted)
            self._q_op.set(op)
            self._q_n.set(str(n))
        wanted, op, n = self._q_class.get(), self._q_op.get(), self._int(self._q_n, 1)
        self._scene_list.delete(*self._scene_list.get_children())
        if not self._frames:
            self._scenes = []
            self._search_status.set(
                "Scene search needs a DroneBackend with get_frame_index_since() -- rebuild it"
                if self._frame_index_supported is False else "No detection passes yet")
            return []
        self._scenes = find_scenes(self._frames, wanted, op, n)
        for i, sc in enumerate(self._scenes):
            t0 = time.strftime("%H:%M:%S", time.localtime(sc["start_ms"] / 1000.0))
            t1 = time.strftime("%H:%M:%S", time.localtime(sc["end_ms"] / 1000.0))
            dur = (sc["end_ms"] - sc["start_ms"]) / 1000.0
            self._scene_list.insert("", "end", iid=f"s{i}",
                                    values=(f"{t0} \u2013 {t1}" if t1 != t0 else t0, f"{dur:.1f} s",
                                            sc["max_count"], len(sc["track_ids"])))
        total_s = sum((sc["end_ms"] - sc["start_ms"]) / 1000.0 for sc in self._scenes)
        span_s = (self._frames[-1][0] - self._frames[0][0]) / 1000.0
        what = "objects" if wanted == "any" else (wanted + "s" if wanted != "person" else "persons")
        self._search_status.set(
            f"{len(self._scenes)} scene(s) with {op} {n} {what} at once  ·  "
            f"{total_s:.0f} s of {span_s:.0f} s searched  ·  select a scene to show its objects")
        return self._scenes

    def _on_scene_select(self, _event) -> None:
        sel = self._scene_list.selection()
        if not sel:
            return
        sc = self._scenes[int(sel[0][1:])]
        self._scene_filter = set(sc["track_ids"])
        t0 = time.strftime("%H:%M:%S", time.localtime(sc["start_ms"] / 1000.0))
        self._scene_desc = (f"Scene {t0} ({(sc['end_ms'] - sc['start_ms']) / 1000.0:.0f} s, "
                            f"max {sc['max_count']} at once)")
        self._apply_filters()

    def clear_scene(self) -> None:
        self._scene_filter = None
        if self._scene_list.selection():
            self._scene_list.selection_remove(self._scene_list.selection())
        self._apply_filters()

    # =====================================================================
    # Public API — called from the main app's poll loop
    # =====================================================================

    def set_engine_status(self, running: bool, detail: str = "") -> None:
        """
        Called by the owning app (see DroneCockpitApp._open_detection_window
        / ._retry_detection_engine in main.py) whenever DetectionLink's
        running state is known or changes. Purely informational -- doesn't
        affect whether records are accepted -- but makes "the engine never
        started" visually distinct from "nothing detected yet", which
        otherwise look identical from inside this window.
        """
        if running:
            self._status_var.set("●  Detection engine running")
            self._status_lbl.config(fg="#00ff88")
        else:
            msg = "●  Detection engine stopped"
            if detail:
                msg += f"  —  {detail}"
            self._status_var.set(msg)
            self._status_lbl.config(fg="#ff5566")

    def set_drone_telemetry(self, latitude: float, longitude: float, valid: bool) -> None:
        """
        Called every poll tick (see _pump_detection_records in
        DroneCockpitUI.py) with the drone's current position, independent
        of whether anything has been detected. Previously the drone marker
        was derived by digging through self._records for the most recent
        one with valid embedded telemetry, and the map refused to draw
        anything at all unless self._track_positions was non-empty -- so a
        flight with zero detections (or zero *georeferenced* ones, e.g.
        before GPS lock) never showed the drone's position at all, even
        though it's always known independent of anything being spotted.
        This makes the map's own knowledge of "where is the drone" not
        depend on the detection pipeline having found something first.
        """
        if not valid:
            self._drone_telemetry = None
            self._redraw_map()
            return
        if self._map_origin is None:
            # Same "first fix to arrive wins" gate add_records() already
            # uses for the first georeferenced detection -- now either
            # source can be the one that clears the "Waiting for GPS fix"
            # placeholder, whichever happens first.
            self._map_origin = (latitude, longitude)
        self._drone_telemetry = (latitude, longitude)
        self._redraw_map()

    def set_telemetry_status(self, valid: bool, detail: str = "") -> None:
        """
        Called by the owning app's poll loop (see _pump_detection_records /
        _get_telemetry_status_summary in DroneCockpitUI.py) on every tick,
        so this reflects current GPS quality in close to real time -- not
        just whatever it was when the window opened. Purely informational,
        same as set_engine_status(): never affects whether records are
        accepted or drawn, only what the second status line says.
        """
        if valid:
            self._telemetry_status_var.set(f"●  Telemetry: {detail}" if detail else "●  Telemetry: usable")
            self._telemetry_status_lbl.config(fg="#00ff88")
        else:
            msg = "●  Telemetry: not usable -- new detections won't be georeferenced"
            if detail:
                msg += f"  —  {detail}"
            self._telemetry_status_var.set(msg)
            self._telemetry_status_lbl.config(fg="#ffaa00")

    def set_detection_link(self, detection_link) -> None:
        """
        Called once by the owning app right after this window is created
        (alongside set_engine_status), so the live feed has something to
        poll. Optional -- if never called, the checkbox simply has
        nothing to show.

        Since the live feed defaults to on (see _build_layout) but there
        is no link yet at __init__ time, kick polling off here rather
        than waiting for the pilot to toggle the checkbox off and back
        on just to get it started.
        """
        self._link = detection_link
        if self._live_enabled.get() and self._live_job is None:
            self._poll_live_frame()

    def add_records(self, records) -> None:
        """
        Feed newly-arrived DetectionRecords in (typically the list
        returned by DetectionWorker.get_new_records() each tick). Cheap:
        a handful of Treeview inserts and canvas ovals, no image
        decoding happens here (tile decoding, when needed, happens
        lazily inside _draw_tile_mosaic on redraw, not here).
        """
        if not records:
            return
        now_ms = self._now_ms()
        changed = []
        for rec in records:
            self._records.append(rec)
            track_id = getattr(rec, "track_id", 0)
            key = track_id if track_id else f"r{rec.id}"   # untracked (old build): one row per record
            info = self._tracks.get(key)
            if info is None:
                info = {"latest": rec, "best": rec, "count": 0, "radius": 0.0,
                        "first_ms": rec.timestamp_ms, "last_ms": rec.timestamp_ms}
                self._tracks[key] = info
                self._track_order[key] = len(self._track_order)
            info["latest"] = rec
            info["count"] += 1
            info["last_ms"] = max(info["last_ms"], rec.timestamp_ms)
            if rec.confidence >= info["best"].confidence:
                info["best"] = rec   # its screenshot is shown when the row is selected
            info["best_conf"] = max(getattr(rec, "best_confidence", 0.0), info["best"].confidence)
            info["seen"] = getattr(rec, "sightings", 0) or info["count"]
            info["radius"] = getattr(rec, "uncertainty_m", 0.0) if rec.georeferenced else 0.0
            changed.append(key)

            if rec.georeferenced:
                if self._map_origin is None:
                    self._map_origin = (rec.latitude, rec.longitude)
                self._track_positions.setdefault(key, []).append((rec.latitude, rec.longitude))

        for key in dict.fromkeys(changed):
            self._sync_row(key, now_ms)
        if self._sort_col is not None:
            self._resort()
        self._update_search_status()
        self._redraw_map()

        # Keep the list scrolled to the newest detection.
        children = self._tree.get_children()
        if children and self._sort_col is None:
            self._tree.see(children[-1])

    # =====================================================================
    # Internals — map / tiles
    # =====================================================================

    def _on_map_provider_change(self) -> None:
        self._map_provider = self._map_provider_var.get()
        # Different provider = different tile bytes at the same z/x/y --
        # drop the decoded-image cache so we don't accidentally paint the
        # old provider's tiles under the new provider's label/attribution.
        # The on-disk cache itself is untouched (each provider has its own
        # subdirectory), so nothing already downloaded is lost.
        self._tile_image_cache.clear()
        # If the pilot is in manual mode above the new provider's own
        # max_zoom (e.g. zoomed to 19 on satellite, then switched to a
        # topo provider that tops out lower), clamp down rather than
        # asking for tiles that don't exist at any zoom.
        if self._map_zoom is not None:
            max_zoom = MapTiles.PROVIDERS[self._map_provider]["max_zoom"]
            self._map_zoom = min(self._map_zoom, max_zoom)
        self._redraw_map()

    def _on_map_canvas_resize(self, _event) -> None:
        # Debounced: dragging the window edge fires many <Configure>
        # events in a row, and a full tile-mosaic rebuild on every one of
        # them would be wasted work (and visibly janky). Wait for resizing
        # to pause for 120ms before actually redrawing.
        if self._resize_job is not None:
            self.after_cancel(self._resize_job)
        self._resize_job = self.after(120, self._redraw_map)

    def _on_center_on_drone(self) -> None:
        """
        Bound to the "Center on drone" button. Re-enables follow mode,
        which snaps the view back to the same auto-fit framing
        _redraw_map has always used (drone + every track point) and keeps
        it following from here on, until the pilot pans/zooms again.
        """
        self._follow_drone = True
        self._redraw_map()

    def _adjust_zoom(self, delta: int) -> None:
        """Bound to the +/- buttons. Zooms toward the current view center."""
        if self._map_zoom is None or self._map_center is None:
            return  # no GPS fix yet -- nothing on screen to zoom
        w = self._map_canvas.winfo_width() or 400
        h = self._map_canvas.winfo_height() or 260
        self._zoom_at_point(w / 2.0, h / 2.0, delta)

    def _zoom_at_point(self, canvas_x: float, canvas_y: float, delta: int) -> None:
        """
        Changes zoom by `delta` while keeping whatever lat/lon is under
        (canvas_x, canvas_y) fixed on screen -- standard "zoom toward
        cursor" behavior, used by both the mouse wheel and the +/-
        buttons (which zoom toward the canvas center). Unlike the
        auto-fit search in _fit_zoom, this is allowed to go all the way
        up to the provider's real max_zoom, not just _AUTO_FIT_MAX_ZOOM --
        the pilot asking to zoom in further is a deliberate choice to
        push past what's cached, not a bbox-fitting accident.
        """
        if self._map_zoom is None or self._map_center is None:
            return
        max_zoom = MapTiles.PROVIDERS[self._map_provider]["max_zoom"]
        old_zoom = self._map_zoom
        new_zoom = max(1, min(max_zoom, old_zoom + delta))
        if new_zoom == old_zoom:
            return

        w = self._map_canvas.winfo_width() or 400
        h = self._map_canvas.winfo_height() or 260
        center_lat, center_lon = self._map_center
        cpx, cpy = _mercator_pixel(center_lat, center_lon, old_zoom)
        cursor_px = cpx + (canvas_x - w / 2.0)
        cursor_py = cpy + (canvas_y - h / 2.0)
        cursor_lat, cursor_lon = _mercator_lonlat(cursor_px, cursor_py, old_zoom)

        # Re-place that same lat/lon at the new zoom, then back-solve the
        # center that keeps it under the same canvas point.
        new_cursor_px, new_cursor_py = _mercator_pixel(cursor_lat, cursor_lon, new_zoom)
        new_center_px = new_cursor_px - (canvas_x - w / 2.0)
        new_center_py = new_cursor_py - (canvas_y - h / 2.0)
        new_center_lat, new_center_lon = _mercator_lonlat(new_center_px, new_center_py, new_zoom)

        self._map_zoom = new_zoom
        self._map_center = (new_center_lat, new_center_lon)
        self._follow_drone = False
        self._redraw_map()

    def _on_mouse_wheel(self, event) -> None:
        # Windows/macOS deliver a signed event.delta (positive = zoom in);
        # X11/Linux instead sends separate Button-4 (up/zoom in) and
        # Button-5 (down/zoom out) events with no delta attribute.
        delta = 1 if getattr(event, "num", None) == 4 else -1 if getattr(event, "num", None) == 5 else \
            (1 if getattr(event, "delta", 0) > 0 else -1)
        self._zoom_at_point(event.x, event.y, delta)

    def _on_map_press(self, event) -> None:
        if self._map_zoom is None or self._map_center is None:
            return  # no GPS fix yet -- nothing to drag
        self._drag_start = (event.x, event.y)
        center_lat, center_lon = self._map_center
        self._drag_start_center_px = _mercator_pixel(center_lat, center_lon, self._map_zoom)
        self._map_canvas.config(cursor="fleur")

    def _on_map_drag(self, event) -> None:
        if self._drag_start is None:
            return
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        cpx, cpy = self._drag_start_center_px
        # Dragging right/down should reveal area to the left/above, i.e.
        # the center point moves opposite to the drag direction.
        new_lat, new_lon = _mercator_lonlat(cpx - dx, cpy - dy, self._map_zoom)
        self._map_center = (new_lat, new_lon)
        self._follow_drone = False
        self._redraw_map()

    def _on_map_release(self, _event) -> None:
        self._drag_start = None
        self._drag_start_center_px = None
        self._map_canvas.config(cursor="")

    def _poll_tile_downloads(self) -> None:
        """
        Background tile downloads (see MapTiles.TileCache) hand finished
        tiles back via a thread-safe queue rather than calling into Tk
        directly from a worker thread -- same reasoning as the live-feed
        poll elsewhere in this file: only the Tk thread should ever touch
        Tk state. This drains that queue on a timer and triggers exactly
        one redraw if anything new landed, rather than one redraw per tile.
        """
        any_ready = False
        while True:
            try:
                self._tile_cache.ready_queue.get_nowait()
            except queue.Empty:
                break
            any_ready = True
        if any_ready:
            self._redraw_map()
        self._tile_poll_job = self.after(300, self._poll_tile_downloads)

    def _fit_zoom(self, min_lat, max_lat, min_lon, max_lon, canvas_w, canvas_h, pad) -> int:
        """
        Highest zoom level (most detail) at which the given lat/lon
        bounding box still fits inside the canvas, capped at the current
        provider's max supported zoom. Standard "fit bounds" search: walk
        zoom down from max until the box's Mercator-pixel footprint fits.
        """
        max_zoom = min(MapTiles.PROVIDERS[self._map_provider]["max_zoom"], _AUTO_FIT_MAX_ZOOM)
        avail_w = max(canvas_w - 2 * pad, 40)
        avail_h = max(canvas_h - 2 * pad, 40)
        for zoom in range(max_zoom, 0, -1):
            x0, y0 = _mercator_pixel(max_lat, min_lon, zoom)  # NW corner
            x1, y1 = _mercator_pixel(min_lat, max_lon, zoom)  # SE corner
            if (x1 - x0) <= avail_w and (y1 - y0) <= avail_h:
                return zoom
        return 1

    def _draw_tile_mosaic(self, zoom, center_lat, center_lon, w, h) -> None:
        """
        Composites just enough tiles to cover the canvas into one PIL
        image and draws it as the map background. Tiles already on disk
        (see MapTiles.TileCache.get_cached) are pasted in immediately;
        missing ones are left as the flat placeholder color and a
        background download is queued for them -- _poll_tile_downloads
        picks those up and triggers a follow-up redraw once they land.
        """
        cpx, cpy = _mercator_pixel(center_lat, center_lon, zoom)
        top_left_x = cpx - w / 2.0
        top_left_y = cpy - h / 2.0

        if not _PIL_AVAILABLE:
            self._map_canvas.create_rectangle(0, 0, w, h, fill="#111a14", outline="")
            self._map_canvas.create_text(
                12, h - 14, anchor="w", fill="#888888",
                text="Pillow not installed -- showing flat background instead of map tiles",
                font=("Segoe UI", 8))
            return

        n_tiles = 2 ** zoom
        tile_x_min = int(math.floor(top_left_x / MapTiles.TILE_SIZE))
        tile_x_max = int(math.floor((top_left_x + w) / MapTiles.TILE_SIZE))
        tile_y_min = int(math.floor(top_left_y / MapTiles.TILE_SIZE))
        tile_y_max = int(math.floor((top_left_y + h) / MapTiles.TILE_SIZE))

        mosaic = Image.new("RGB", (max(int(w), 1), max(int(h), 1)), "#111a14")
        for ty in range(tile_y_min, tile_y_max + 1):
            if ty < 0 or ty >= n_tiles:
                continue  # off the top/bottom of the world -- nothing to paste
            for tx in range(tile_x_min, tile_x_max + 1):
                wrapped_tx = tx % n_tiles  # longitude wraps at +/-180; latitude (ty) never does
                cache_key = (self._map_provider, zoom, wrapped_tx, ty)
                tile_img = self._tile_image_cache.get(cache_key)
                if tile_img is None:
                    tile_path = self._tile_cache.get_cached(self._map_provider, zoom, wrapped_tx, ty)
                    if tile_path is not None:
                        try:
                            tile_img = Image.open(tile_path).convert("RGB")
                            self._tile_image_cache[cache_key] = tile_img
                        except (OSError, ValueError):
                            tile_img = None
                if tile_img is not None:
                    paste_x = int(tx * MapTiles.TILE_SIZE - top_left_x)
                    paste_y = int(ty * MapTiles.TILE_SIZE - top_left_y)
                    mosaic.paste(tile_img, (paste_x, paste_y))
                # else: not cached yet (download now queued, see get_cached)
                # or the tile failed to load -- leave this cell as the flat
                # placeholder color; _poll_tile_downloads will trigger a
                # redraw once/if it arrives.

        self._mosaic_photo = ImageTk.PhotoImage(mosaic)
        self._map_canvas.create_image(0, 0, anchor="nw", image=self._mosaic_photo)

    def _redraw_map(self) -> None:
        """
        Full repaint of the map canvas: basemap tiles (_draw_tile_mosaic)
        plus track pins/trails and the drone marker on top, all in the
        same Web Mercator pixel space (_mercator_pixel) so nothing can
        drift out of alignment with the imagery. Cheap enough to call on
        every new record, every drone telemetry tick, every selection
        change, and every tile that finishes downloading -- a flight is a
        handful of points and, at any one time, a handful of tiles, not
        per-frame data.
        """
        self._map_canvas.delete("all")

        if self._map_origin is None:
            # Neither a detection nor a drone GPS fix has ever arrived --
            # there is no coordinate yet to center a map on. This is the
            # only case with genuinely nothing to show.
            self._map_canvas.create_rectangle(0, 0, 400, 260, fill="#111a14", outline="")
            self._map_canvas.create_text(
                12, 14, anchor="nw", fill="#888888",
                text="Waiting for GPS fix...", font=("Segoe UI", 9))
            return

        w = self._map_canvas.winfo_width() or 400
        h = self._map_canvas.winfo_height() or 260
        pad = 34

        if self._follow_drone:
            # Points the view must fit: every track pin/trail point, plus
            # the drone's own current position if we have one -- a drone
            # that has flown well away from its one detection (or that
            # has zero detections at all) still needs to stay on screen.
            all_points = [p for key, positions in self._track_positions.items()
                          if key in self._visible_keys for p in positions]
            if self._drone_telemetry is not None:
                all_points.append(self._drone_telemetry)
            if not all_points:
                all_points = [self._map_origin]  # nothing plottable yet, but still center the basemap somewhere real

            lats = [p[0] for p in all_points]
            lons = [p[1] for p in all_points]
            min_lat, max_lat = min(lats), max(lats)
            min_lon, max_lon = min(lons), max(lons)
            center_lat = (min_lat + max_lat) / 2.0
            center_lon = (min_lon + max_lon) / 2.0

            zoom = self._fit_zoom(min_lat, max_lat, min_lon, max_lon, w, h, pad)
            self._map_zoom = zoom
            self._map_center = (center_lat, center_lon)
        else:
            # Manual look-around mode (drag / zoom buttons / scroll wheel
            # already set these) -- hold the view still instead of
            # re-fitting it to the data on every redraw.
            zoom = self._map_zoom
            center_lat, center_lon = self._map_center

        self._draw_tile_mosaic(zoom, center_lat, center_lon, w, h)

        center_px, center_py = _mercator_pixel(center_lat, center_lon, zoom)

        def to_canvas(lat, lon):
            px, py = _mercator_pixel(lat, lon, zoom)
            return w / 2.0 + (px - center_px), h / 2.0 + (py - center_py)

        # Which track (if any) the pilot currently has selected in the
        # list, so its pin/trail can be highlighted on the map too.
        selected_track_id = None
        focus_iid = self._tree.focus()
        if focus_iid:
            selected_track_id = self._key_from_iid(focus_iid)

        if not self._track_positions:
            self._map_canvas.create_rectangle(8, 8, 210, 26, fill="#000000", stipple="gray50", outline="")
            self._map_canvas.create_text(
                14, 17, anchor="w", fill="#dddddd",
                text="No georeferenced detections yet", font=("Segoe UI", 9))

        meters_per_pixel = 156543.03392804097 * math.cos(math.radians(center_lat)) / (2 ** zoom)
        px_per_m = (1.0 / meters_per_pixel) if meters_per_pixel > 0 else 0.0

        for track_id, positions in self._track_positions.items():
            if track_id not in self._visible_keys:
                continue                       # hidden by the search filter
            is_selected = (track_id == selected_track_id)

            # Uncertainty area of the object's (fused) position: where to
            # search. Outline only, so the imagery underneath stays visible.
            radius_m = self._tracks.get(track_id, {}).get("radius", 0.0)
            if radius_m > 0 and px_per_m > 0 and positions:
                cx, cy = to_canvas(*positions[-1])
                rpx = max(4.0, radius_m * px_per_m)
                self._map_canvas.create_oval(
                    cx - rpx, cy - rpx, cx + rpx, cy + rpx,
                    outline="#00e0ff" if is_selected else "#ffaa00",
                    width=2 if is_selected else 1, dash=(4, 3))

            if len(positions) > 1:
                coords = []
                for lat, lon in positions:
                    x, y = to_canvas(lat, lon)
                    coords.extend([x, y])
                self._map_canvas.create_line(
                    *coords, fill="#2fb8c9" if is_selected else "#e0d060", width=2)

            last_idx = len(positions) - 1
            for idx, (lat, lon) in enumerate(positions):
                x, y = to_canvas(lat, lon)
                if idx == last_idx:
                    # Most recent sighting of this track -- the "current"
                    # pin, drawn bright and larger, with a dark outline so
                    # it stays visible against both light and dark imagery.
                    color = "#00e0ff" if is_selected else "#ffaa00"
                    radius = 6
                else:
                    # Older sighting of the SAME object -- a faded
                    # breadcrumb, not a separate detection to read on
                    # its own.
                    color = "#998833"
                    radius = 3
                self._map_canvas.create_oval(
                    x - radius, y - radius, x + radius, y + radius,
                    fill=color, outline="#000000", width=1)

        # Drone marker: driven by set_drone_telemetry() (continuous, from
        # the app's own poll loop) rather than digging through
        # self._records for one with valid embedded telemetry -- that
        # meant a flight with zero detections, or zero *georeferenced*
        # ones, never showed the drone at all even though its position is
        # always known independent of anything being spotted.
        if self._drone_telemetry is not None:
            dx, dy = to_canvas(*self._drone_telemetry)
            self._map_canvas.create_polygon(
                dx, dy - 8, dx - 7, dy + 6, dx + 7, dy + 6,
                fill="#00ff88", outline="#003318", width=1)
            self._map_canvas.create_text(
                dx, dy + 17, text="drone", fill="#00ff88", font=("Segoe UI", 7, "bold"))

        # North-up compass mark -- fixed, since the map is always drawn
        # north-up (standard Web Mercator orientation, same as the tiles).
        self._map_canvas.create_text(
            w - 24, 16, text="N ↑", fill="#ffffff", font=("Segoe UI", 9, "bold"))

        meters_per_pixel = 156543.03392804097 * math.cos(math.radians(center_lat)) / (2 ** zoom)
        scale_px_per_m = (1.0 / meters_per_pixel) if meters_per_pixel > 0 else 1.0
        self._draw_scale_bar(scale_px_per_m, h, pad)

        cfg = MapTiles.PROVIDERS[self._map_provider]
        self._map_attribution_var.set(f"{cfg['label']} — {cfg['attribution']}  ·  zoom {zoom}")

    def _draw_scale_bar(self, scale_px_per_m: float, canvas_h: int, pad: int) -> None:
        if scale_px_per_m <= 0:
            return
        target_px = 70.0
        meters_guess = max(target_px / scale_px_per_m, 0.1)
        # Round to a "nice" 1/2/5 * 10^n meter figure near the target
        # pixel length, same convention most map UIs use for scale bars.
        magnitude = 10 ** math.floor(math.log10(meters_guess))
        nice_m = magnitude
        for mult in (1, 2, 5, 10):
            candidate = mult * magnitude
            nice_m = candidate
            if candidate >= meters_guess:
                break
        bar_px = nice_m * scale_px_per_m

        x0 = pad * 0.6
        y0 = canvas_h - 16
        self._map_canvas.create_line(x0, y0, x0 + bar_px, y0, fill="#ffffff", width=2)
        self._map_canvas.create_line(x0, y0 - 4, x0, y0 + 4, fill="#ffffff")
        self._map_canvas.create_line(x0 + bar_px, y0 - 4, x0 + bar_px, y0 + 4, fill="#ffffff")
        label = f"{nice_m:.0f} m" if nice_m >= 1 else f"{nice_m:.1f} m"
        self._map_canvas.create_text(x0 + bar_px / 2.0, y0 - 10, text=label,
                                      fill="#ffffff", font=("Segoe UI", 8))

    # =====================================================================
    # Internals — detection list / preview
    # =====================================================================

    def _key_from_iid(self, iid: str):
        """Treeview row id 't<key>' -> self._tracks key (int track id or 'r<id>')."""
        key = iid[1:] if iid.startswith("t") else iid
        return int(key) if key.isdigit() else key

    def _on_select(self, _event) -> None:
        selection = self._tree.selection()
        if not selection:
            return
        info = self._tracks.get(self._key_from_iid(selection[0]))
        if info is None:
            return
        rec, best = info["latest"], info["best"]

        if rec.georeferenced:
            radius = getattr(rec, "uncertainty_m", 0.0)
            range_line = (f"within ±{radius:.0f} m of the pin  ·  last fix ~{rec.distance_m:.0f} m "
                          f"at {rec.bearing_deg:.0f}° (via {rec.range_method})")
        else:
            range_line = "no position (no valid GPS / telemetry at the time)"

        seen = getattr(rec, "sightings", 0) or info["count"]
        first = time.strftime("%H:%M:%S", time.localtime(info["first_ms"] / 1000.0))
        last = time.strftime("%H:%M:%S", time.localtime(info["last_ms"] / 1000.0))
        self._detail_lbl.config(
            text=f"{rec.class_name}  ·  object {getattr(rec, 'track_id', 0)}  ·  seen {seen}x  ·  "
                 f"best {best.confidence:.2f}  ·  in view {first} \u2013 {last}\n"
                 f"{range_line}\n"
                 f"screenshot of the best sighting: {best.screenshot_path or 'none saved'}"
        )

        # Swap the pane over to the BEST sighting's saved screenshot. The
        # live poll (_poll_live_frame) keeps running underneath -- it
        # just stops writing into _preview_label while _viewing_saved_id
        # is set, so "Back to live" is instant instead of a re-fetch.
        self._viewing_saved_id = best.id
        self._preview_status_var.set(f"●  SAVED FRAME  (object {getattr(rec, 'track_id', 0)}, "
                                     f"best sighting #{best.id})")
        self._preview_status_lbl.config(fg="#ffaa00")
        self._show_back_to_live_button()
        self._load_preview(best.screenshot_path)
        self._redraw_map()  # re-highlight this object's pin on the map

    def _return_to_live(self) -> None:
        """
        Leaves the saved-screenshot view and hands the preview pane back
        to the live feed -- bound to the "Back to live" button shown
        while a detection's screenshot is being displayed (see
        _on_select).
        """
        self._viewing_saved_id = None
        if self._tree.selection():
            self._tree.selection_remove(self._tree.selection())
        self._hide_back_to_live_button()

        if self._live_enabled.get():
            self._preview_status_var.set("●  LIVE")
            self._preview_status_lbl.config(fg="#00ff88")
            if self._live_photo is not None:
                self._preview_label.config(image=self._live_photo, text="")
            else:
                self._preview_label.config(image="", text="(waiting for live feed...)",
                                            fg="#888888", bg="#000000")
        else:
            self._preview_status_var.set("●  LIVE (paused)")
            self._preview_status_lbl.config(fg="#888888")
            self._preview_label.config(image="", text="(live feed paused)",
                                        fg="#888888", bg="#000000")

        self._detail_lbl.config(text="Live feed -- click a detection for its saved screenshot")
        self._redraw_map()  # clear the track highlight tied to the deselected row

    def _show_back_to_live_button(self) -> None:
        if not self._back_to_live_btn_visible:
            self._back_to_live_btn.pack(side="right", padx=(0, 4))
            self._back_to_live_btn_visible = True

    def _hide_back_to_live_button(self) -> None:
        if self._back_to_live_btn_visible:
            self._back_to_live_btn.pack_forget()
            self._back_to_live_btn_visible = False

    def _load_preview(self, path: str) -> None:
        if not path or not _PIL_AVAILABLE:
            self._preview_label.config(image="", text="(no preview available)",
                                        fg="#888888", bg="#000000")
            return
        try:
            img = Image.open(path)
            img.thumbnail((640, 360))
            self._selected_photo = ImageTk.PhotoImage(img)
            self._preview_label.config(image=self._selected_photo, text="")
        except (OSError, ValueError):
            self._preview_label.config(image="", text="(screenshot unavailable)",
                                        fg="#888888", bg="#000000")

    # =====================================================================
    # Live feed (on by default, ~5-6fps, see set_detection_link/_build_layout)
    # =====================================================================

    def _on_live_toggle(self) -> None:
        if self._live_enabled.get():
            if not _PIL_AVAILABLE or self._link is None:
                self._live_enabled.set(False)
                return
            if self._viewing_saved_id is None:
                self._preview_status_var.set("●  LIVE")
                self._preview_status_lbl.config(fg="#00ff88")
            if self._live_job is None:
                self._poll_live_frame()
        else:
            if self._live_job is not None:
                self.after_cancel(self._live_job)
                self._live_job = None
            # Only touch the visible pane if it isn't currently showing a
            # saved screenshot -- pausing the live feed shouldn't yank
            # the pilot out of a screenshot they clicked into.
            if self._viewing_saved_id is None:
                self._preview_status_var.set("●  LIVE (paused)")
                self._preview_status_lbl.config(fg="#888888")
                self._preview_label.config(image="", text="(live feed paused)",
                                            fg="#888888", bg="#000000")

    def _poll_live_frame(self) -> None:
        if not self._live_enabled.get() or self._link is None:
            return

        try:
            jpeg_bytes = self._link.get_latest_annotated_frame_jpeg()
        except AttributeError:
            # Binding not wired up yet on the C++/pybind11 side -- fail
            # quiet rather than spamming the console every tick.
            jpeg_bytes = None

        if jpeg_bytes:
            try:
                import io
                img = Image.open(io.BytesIO(bytes(jpeg_bytes)))
                img.thumbnail((640, 360))

                # Always keep the decoded frame around, even while a
                # saved screenshot is on screen, so "Back to live"
                # (_return_to_live) has something current to show
                # immediately instead of a blank pane for one tick.
                self._live_photo = ImageTk.PhotoImage(img)
                if self._viewing_saved_id is None:
                    self._preview_label.config(image=self._live_photo, text="")
            except (OSError, ValueError):
                pass

        # 40ms (~25fps): matches DetectionLink's own preview-tick cadence
        # (kPreviewTickMs in Detectionlink.cpp), which now redraws and
        # republishes the annotated frame every tick regardless of how
        # often actual inference runs (detectionIntervalMs_, still its own
        # separate slower cadence -- see detectionLoop()'s comments). This
        # used to be 180ms (~5-6fps) back when the two were the same loop
        # at the same cadence, so polling faster just re-fetched a stale
        # frame; now the backend genuinely has a fresh one this often.
        # Raise this if JPEG-decode/PhotoImage cost on the Tk thread ever
        # turns out to matter more than feed smoothness on your hardware.
        self._live_job = self.after(40, self._poll_live_frame)