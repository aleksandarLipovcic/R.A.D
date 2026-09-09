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

The map panel is still not a real satellite/tile basemap -- plugging in
actual imagery is future work (the "terrain matching / satellite
basemap" piece discussed separately) and needs an internet connection
or a pre-downloaded tile cache this app doesn't assume. What it DOES do
now is a proper local metric projection: every georeferenced detection
is converted to meters-north/meters-east from a fixed origin (the first
georeferenced sighting of the flight) using an equirectangular
tangent-plane approximation -- same flat-earth assumption class
DetectionLink itself uses for its ranging, fine at the sub-few-km scale
this is for. Unlike the old version (which independently rescaled
lat-span and lon-span to fill the canvas every time a pin arrived, so
"close together" or "far apart" on screen didn't mean anything in real
meters and the whole picture could silently reflow/rescale on every new
detection), this uses ONE shared scale for both axes, a visible scale
bar, and a north-up compass mark, so relative distance and direction on
screen are actually meaningful now -- an approximate pinpoint on a
real, if still tile-less, map, per the original ask. The detection list
remains the primary, trustworthy source of exact values (lat/lon,
distance, bearing) for any single sighting.

Records are grouped by track_id (see DetectionLink's tracker): only the
newest sighting of a given track is drawn as the bright "current" pin;
earlier sightings of that same track are drawn as a small fading trail
so a single object logged 3 times over a minute of movement reads as
one moving pin with a breadcrumb trail, not three unrelated dots.
"""

import math
import time
import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image, ImageTk
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False

_EARTH_RADIUS_M = 6378137.0


class DetectionMapWidget(tk.Toplevel):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.title("Detections & map")
        self.geometry("900x560")
        self.configure(bg="#1a1a1a")

        # In-memory record store. Kept here (not just in DetectionWorker)
        # because this widget owns the display-facing view of the data --
        # e.g. filtering by class, re-selecting an old pin -- independent
        # of the worker's own incremental "new since last poll" bookkeeping.
        self._records = []          # all DetectionRecord objects seen so far
        self._selected_photo = None  # keep a reference so Tk doesn't GC it

        # ── Map projection state ─────────────────────────────────────
        # origin is fixed the first time a georeferenced record arrives
        # and never moves again for the life of this window -- keeping
        # it fixed (rather than recentering on new bounds every pin, like
        # the old version did) is what makes the map stop silently
        # reflowing/rescaling every time a new detection comes in.
        self._map_origin = None   # (lat, lon) or None until first georeferenced record
        self._drone_telemetry = None   # (lat, lon) or None -- see set_drone_telemetry()
        # track_id -> list of (east_m, north_m) in arrival order, so a
        # single object's repeated sightings draw as one pin + trail
        # instead of a pile of unrelated dots (see module docstring).
        self._track_positions = {}

        self._build_layout()

    def _project_local_m(self, lat, lon):
        """
        Equirectangular tangent-plane projection relative to
        self._map_origin: returns (east_m, north_m). Same flat-earth
        assumption DetectionLink itself uses for ranging -- fine at the
        sub-few-km scale a single flight covers, not meant for anything
        further.
        """
        origin_lat, origin_lon = self._map_origin
        lat_rad = math.radians((lat + origin_lat) / 2.0)
        north_m = math.radians(lat - origin_lat) * _EARTH_RADIUS_M
        east_m = math.radians(lon - origin_lon) * _EARTH_RADIUS_M * math.cos(lat_rad)
        return east_m, north_m

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

        columns = ("track", "class", "conf", "time", "geo", "range")
        self._tree = ttk.Treeview(left, columns=columns, show="headings", selectmode="browse")
        self._tree.heading("track", text="Track")
        self._tree.heading("class", text="Class")
        self._tree.heading("conf", text="Conf.")
        self._tree.heading("time", text="Time")
        self._tree.heading("geo", text="Lat / Lon")
        self._tree.heading("range", text="Range")
        self._tree.column("track", width=50, anchor="center")
        self._tree.column("class", width=80)
        self._tree.column("conf", width=55, anchor="e")
        self._tree.column("time", width=80, anchor="center")
        self._tree.column("geo", width=150)
        self._tree.column("range", width=90, anchor="e")
        self._tree.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._tree.bind("<<TreeviewSelect>>", self._on_select)

        # ── Right: pin map (placeholder) + screenshot preview ────────
        right = tk.Frame(paned, bg="#1a1a1a")
        paned.add(right, weight=2)

        self._map_canvas = tk.Canvas(right, bg="#0d1f14", height=260, highlightthickness=0)
        self._map_canvas.pack(fill="x", padx=8, pady=(8, 4))
        tk.Label(right, text="North-up, true-to-scale local plot (see scale bar) -- not satellite imagery yet",
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
        live_row = tk.Frame(right, bg="#1a1a1a")
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

        self._preview_label = tk.Label(right, bg="#000000")
        self._preview_label.pack(fill="both", expand=True, padx=8, pady=8)

        self._detail_lbl = tk.Label(right, text="Live feed -- click a detection for its saved screenshot",
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
            # Same "fixed for the rest of the flight" origin add_records()
            # already uses for the first georeferenced detection -- now
            # either source can be the one that establishes it, whichever
            # happens first.
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
        decoding happens here.
        """
        if not records:
            return
        for rec in records:
            self._records.append(rec)
            t = time.strftime("%H:%M:%S", time.localtime(rec.timestamp_ms / 1000.0))
            geo = f"{rec.latitude:.5f}, {rec.longitude:.5f}" if rec.georeferenced else "n/a"
            rng = f"{rec.distance_m:.0f}m" if rec.georeferenced else "n/a"
            # track_id groups repeated sightings of the same physical
            # object (see DetectionLink's tracker) -- getattr fallback in
            # case an older DetectionLink build without the field is
            # still in use, so this widget doesn't hard-crash on it.
            track_id = getattr(rec, "track_id", 0)
            self._tree.insert("", "end", iid=str(rec.id),
                               values=(track_id, rec.class_name, f"{rec.confidence:.2f}", t, geo, rng))

            if rec.georeferenced:
                if self._map_origin is None:
                    # Fixed for the rest of the flight -- see
                    # _project_local_m()'s docstring for why a fixed
                    # origin (vs. recentering every pin) matters.
                    self._map_origin = (rec.latitude, rec.longitude)
                pos = self._project_local_m(rec.latitude, rec.longitude)
                self._track_positions.setdefault(track_id, []).append(pos)

            self._redraw_map()

        # Keep the list scrolled to the newest detection.
        children = self._tree.get_children()
        if children:
            self._tree.see(children[-1])

    # =====================================================================
    # Internals
    # =====================================================================

    def _redraw_map(self) -> None:
        """
        Full repaint of the map canvas from self._track_positions AND
        self._drone_telemetry (see _project_local_m/add_records/
        set_drone_telemetry). Cheap enough to call on every new record,
        every drone telemetry tick, and every selection change -- this is
        a handful of points per flight, not per-frame data.
        """
        self._map_canvas.delete("all")

        if self._map_origin is None:
            # Neither a detection nor a drone GPS fix has ever arrived --
            # there is no reference point yet to draw anything relative
            # to. This is the only case with genuinely nothing to show.
            self._map_canvas.create_text(
                12, 14, anchor="nw", fill="#888888",
                text="Waiting for GPS fix...", font=("Segoe UI", 9))
            return

        w = self._map_canvas.winfo_width() or 400
        h = self._map_canvas.winfo_height() or 260
        pad = 34

        # Points that must stay in view: every track pin/trail point,
        # PLUS the drone's own current position if we have one --
        # previously only detections drove the extent, so a drone that
        # had flown well away from its one detection (or that had zero
        # detections at all) could end up off-canvas or not drawn.
        all_points = [p for positions in self._track_positions.values() for p in positions]
        if self._drone_telemetry is not None:
            all_points.append(self._project_local_m(*self._drone_telemetry))

        if not all_points:
            # We have an origin (a fix arrived at some point) but nothing
            # -- not even the drone right now -- to plot yet.
            self._map_canvas.create_text(
                12, 14, anchor="nw", fill="#888888",
                text="No georeferenced detections yet", font=("Segoe UI", 9))
            return

        eastings = [p[0] for p in all_points]
        northings = [p[1] for p in all_points]
        # A single shared scale for BOTH axes (unlike the old lat-range /
        # lon-range-independent scaling) so distance and direction on
        # screen actually correspond to real meters. Floor the extent at
        # 5m so a single early detection doesn't zoom in to a meaningless
        # degree before there's anything to compare it against.
        extent_m = max(max(eastings) - min(eastings), max(northings) - min(northings), 5.0)
        usable_px = max(min(w, h) - 2 * pad, 40)
        scale_px_per_m = usable_px / extent_m

        cx, cy = w / 2.0, h / 2.0

        def to_canvas(east_m, north_m):
            # North-up: canvas y decreases as north increases.
            return cx + east_m * scale_px_per_m, cy - north_m * scale_px_per_m

        # Which track (if any) the pilot currently has selected in the
        # list, so its pin/trail can be highlighted on the map too.
        selected_track_id = None
        focus_iid = self._tree.focus()
        if focus_iid:
            focused_rec = next((r for r in self._records if str(r.id) == focus_iid), None)
            if focused_rec is not None:
                selected_track_id = getattr(focused_rec, "track_id", 0)

        if not self._track_positions:
            # We have an origin and/or a drone fix (otherwise we'd have
            # returned above), just nothing detected/georeferenced yet --
            # still draw the drone marker, scale bar and compass below,
            # just with this small note instead of any track pins.
            self._map_canvas.create_text(
                12, 14, anchor="nw", fill="#888888",
                text="No georeferenced detections yet", font=("Segoe UI", 9))

        for track_id, positions in self._track_positions.items():
            is_selected = (track_id == selected_track_id)

            if len(positions) > 1:
                coords = []
                for east_m, north_m in positions:
                    x, y = to_canvas(east_m, north_m)
                    coords.extend([x, y])
                self._map_canvas.create_line(
                    *coords, fill="#2fb8c9" if is_selected else "#665f33", width=1)

            last_idx = len(positions) - 1
            for idx, (east_m, north_m) in enumerate(positions):
                x, y = to_canvas(east_m, north_m)
                if idx == last_idx:
                    # Most recent sighting of this track -- the "current"
                    # pin, drawn bright and larger.
                    color = "#00e0ff" if is_selected else "#ffaa00"
                    radius = 5
                else:
                    # Older sighting of the SAME object -- a faded
                    # breadcrumb, not a separate detection to read on
                    # its own.
                    color = "#777744"
                    radius = 2
                self._map_canvas.create_oval(
                    x - radius, y - radius, x + radius, y + radius, fill=color, outline="")

        # Drone marker: driven by set_drone_telemetry() (continuous, from
        # the app's own poll loop) rather than digging through
        # self._records for one with valid embedded telemetry -- that
        # meant a flight with zero detections, or zero *georeferenced*
        # ones, never showed the drone at all even though its position is
        # always known independent of anything being spotted.
        if self._drone_telemetry is not None:
            de, dn = self._project_local_m(*self._drone_telemetry)
            dx, dy = to_canvas(de, dn)
            self._map_canvas.create_polygon(
                dx, dy - 7, dx - 6, dy + 5, dx + 6, dy + 5, fill="#00ff88", outline="")
            self._map_canvas.create_text(
                dx, dy + 15, text="drone", fill="#00ff88", font=("Segoe UI", 7))

        # North-up compass mark -- fixed, since the map itself is always
        # drawn north-up (see to_canvas() above).
        self._map_canvas.create_text(
            w - 22, 16, text="N ↑", fill="#aaaaaa", font=("Segoe UI", 9, "bold"))

        self._draw_scale_bar(scale_px_per_m, h, pad)

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
        self._map_canvas.create_line(x0, y0, x0 + bar_px, y0, fill="#cccccc", width=2)
        self._map_canvas.create_line(x0, y0 - 4, x0, y0 + 4, fill="#cccccc")
        self._map_canvas.create_line(x0 + bar_px, y0 - 4, x0 + bar_px, y0 + 4, fill="#cccccc")
        label = f"{nice_m:.0f} m" if nice_m >= 1 else f"{nice_m:.1f} m"
        self._map_canvas.create_text(x0 + bar_px / 2.0, y0 - 10, text=label,
                                      fill="#cccccc", font=("Segoe UI", 8))

    def _on_select(self, _event) -> None:
        selection = self._tree.selection()
        if not selection:
            return
        rec_id = int(selection[0])
        rec = next((r for r in self._records if r.id == rec_id), None)
        if rec is None:
            return

        if rec.georeferenced:
            range_line = (f"~{rec.distance_m:.0f}m at {rec.bearing_deg:.0f} deg "
                          f"(via {rec.range_method})")
        else:
            range_line = "not georeferenced this pass (no valid telemetry / ray)"

        self._detail_lbl.config(
            text=f"{rec.class_name}  ({rec.confidence:.2f})\n"
                 f"heading {rec.telemetry.heading_deg:.0f} deg, "
                 f"gimbal tilt {rec.telemetry.gimbal_tilt_deg:.0f} deg\n"
                 f"{range_line}\n"
                 f"{rec.screenshot_path or 'no screenshot saved'}"
        )

        # Swap the pane over to this detection's saved screenshot. The
        # live poll (_poll_live_frame) keeps running underneath -- it
        # just stops writing into _preview_label while _viewing_saved_id
        # is set, so "Back to live" is instant instead of a re-fetch.
        self._viewing_saved_id = rec.id
        self._preview_status_var.set(f"●  SAVED FRAME  (detection #{rec.id})")
        self._preview_status_lbl.config(fg="#ffaa00")
        self._show_back_to_live_button()
        self._load_preview(rec.screenshot_path)
        self._redraw_map()  # re-highlight this record's track on the map

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

    # ── TEMPORARY perf diagnostic ────────────────────────────────────────
    # These accumulate across calls and get flushed to the console every
    # _DBG_PRINT_EVERY ticks (~1s at the intended 40ms cadence). Three
    # numbers matter most here:
    #   avg_wall_interval -- how much real wall-clock time actually
    #     elapsed between polls. If this is way above the requested 40ms,
    #     something on the Tk thread (this call included) is stalling the
    #     mainloop and eating into after()'s own scheduling, not just this
    #     callback's own work.
    #   avg_get_call_ms -- time spent inside the single call to
    #     get_latest_annotated_frame_jpeg(). This crosses the pybind11
    #     boundary synchronously; if it's large, see the perf comment on
    #     that function in Detectionlink.cpp (GIL release).
    #   avg_decode_ms / avg_photoimage_ms -- PIL decode + thumbnail, and
    #     Tk PhotoImage construction, both on this thread.
    # Delete this whole block (and the _dbg_* attrs) once the bottleneck
    # is found and fixed.
    _DBG_PRINT_EVERY = 25

    def _poll_live_frame(self) -> None:
        if not self._live_enabled.get() or self._link is None:
            return

        import time
        now = time.monotonic()
        if not hasattr(self, "_dbg_last_poll_at"):
            self._dbg_last_poll_at = now
            self._dbg_tick = 0
            self._dbg_interval_accum = 0.0
            self._dbg_get_accum = 0.0
            self._dbg_decode_accum = 0.0
            self._dbg_photo_accum = 0.0
            self._dbg_empty_count = 0
            self._dbg_last_jpeg_len = -1
        wall_interval_ms = (now - self._dbg_last_poll_at) * 1000.0
        self._dbg_last_poll_at = now
        self._dbg_interval_accum += wall_interval_ms

        get_start = time.monotonic()
        try:
            jpeg_bytes = self._link.get_latest_annotated_frame_jpeg()
        except AttributeError:
            # Binding not wired up yet on the C++/pybind11 side -- fail
            # quiet rather than spamming the console every tick.
            jpeg_bytes = None
        self._dbg_get_accum += (time.monotonic() - get_start) * 1000.0

        if jpeg_bytes:
            same_len_as_last = (len(jpeg_bytes) == self._dbg_last_jpeg_len)
            self._dbg_last_jpeg_len = len(jpeg_bytes)
            if same_len_as_last:
                # Same byte length twice in a row is a decent (not
                # perfect) proxy for "the backend republished the exact
                # same annotated frame again" -- worth knowing if the
                # C++ side is falling behind its own 40ms tick.
                self._dbg_empty_count += 1
            try:
                import io
                decode_start = time.monotonic()
                img = Image.open(io.BytesIO(bytes(jpeg_bytes)))
                img.thumbnail((640, 360))
                self._dbg_decode_accum += (time.monotonic() - decode_start) * 1000.0

                photo_start = time.monotonic()
                # Always keep the decoded frame around, even while a
                # saved screenshot is on screen, so "Back to live"
                # (_return_to_live) has something current to show
                # immediately instead of a blank pane for one tick.
                self._live_photo = ImageTk.PhotoImage(img)
                if self._viewing_saved_id is None:
                    self._preview_label.config(image=self._live_photo, text="")
                self._dbg_photo_accum += (time.monotonic() - photo_start) * 1000.0
            except (OSError, ValueError):
                pass

        self._dbg_tick += 1
        if self._dbg_tick >= self._DBG_PRINT_EVERY:
            n = self._dbg_tick
            achieved_fps = 1000.0 / (self._dbg_interval_accum / n) if self._dbg_interval_accum else 0.0
            print(
                f"[DetectionMapWidget][perf] avg_wall_interval={self._dbg_interval_accum / n:.1f}ms "
                f"(~{achieved_fps:.1f}fps, target=25fps) "
                f"avg_get_call={self._dbg_get_accum / n:.1f}ms "
                f"avg_decode={self._dbg_decode_accum / n:.1f}ms "
                f"avg_photoimage={self._dbg_photo_accum / n:.1f}ms "
                f"repeated_frames={self._dbg_empty_count}/{n}"
            )
            self._dbg_tick = 0
            self._dbg_interval_accum = self._dbg_get_accum = 0.0
            self._dbg_decode_accum = self._dbg_photo_accum = 0.0
            self._dbg_empty_count = 0

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