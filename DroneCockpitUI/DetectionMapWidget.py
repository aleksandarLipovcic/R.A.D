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
detection, and the opt-in "live detection preview" toggle, which polls
DetectionLink::getLatestAnnotatedFrameJpeg() at a slow, fixed ~2fps --
a deliberate, narrow, clearly-labeled exception to the no-pixels rule,
never the pilot's primary video path.

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

        # ── Live view toggle ────────────────────────────────────────
        # Off by default: this pane shows the most recent frame the
        # detection engine processed with its boxes drawn on it, pulled
        # at a slow, fixed rate (see _poll_live_frame). It is a
        # deliberate, narrow exception to "this widget never touches raw
        # pixel data" -- see DetectionLink::getLatestAnnotatedFrameJpeg()
        # -- kept opt-in and throttled so it can never compete with or
        # be confused for the pilot's primary FPV feed.
        live_row = tk.Frame(right, bg="#1a1a1a")
        live_row.pack(fill="x", padx=8, pady=(4, 0))
        self._live_enabled = tk.BooleanVar(value=False)
        self._live_check = tk.Checkbutton(
            live_row, text="Show live detection preview (~2 fps)", variable=self._live_enabled,
            command=self._on_live_toggle, fg="#cccccc", bg="#1a1a1a",
            selectcolor="#1a1a1a", activebackground="#1a1a1a", activeforeground="#ffffff",
        )
        self._live_check.pack(anchor="w")

        self._preview_label = tk.Label(right, bg="#000000")
        self._preview_label.pack(fill="both", expand=True, padx=8, pady=8)

        self._detail_lbl = tk.Label(right, text="Select a detection to view its screenshot",
                                     fg="#cccccc", bg="#1a1a1a", font=("Segoe UI", 9), justify="left")
        self._detail_lbl.pack(anchor="w", padx=8, pady=(0, 8))

        # Set by the owning app via set_detection_link() -- only needed
        # for the live-preview toggle above; everything else in this
        # widget only ever sees DetectionRecord structs, never the link.
        self._link = None
        self._live_job = None

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

    def set_detection_link(self, detection_link) -> None:
        """
        Called once by the owning app right after this window is created
        (alongside set_engine_status), so the live-preview toggle has
        something to poll. Optional -- if never called, the checkbox is
        simply a no-op.
        """
        self._link = detection_link

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
        Full repaint of the map canvas from self._track_positions (see
        _project_local_m/add_records). Cheap enough to call on every new
        record and on every selection change -- this is a handful of
        tracks' worth of points per flight, not per-frame data.
        """
        self._map_canvas.delete("all")

        if self._map_origin is None or not self._track_positions:
            self._map_canvas.create_text(
                12, 14, anchor="nw", fill="#888888",
                text="No georeferenced detections yet", font=("Segoe UI", 9))
            return

        w = self._map_canvas.winfo_width() or 400
        h = self._map_canvas.winfo_height() or 260
        pad = 34

        all_points = [p for positions in self._track_positions.values() for p in positions]
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

        # Drone marker: projected position from the most recent record
        # that had valid telemetry, so the pilot can see roughly where
        # the drone was relative to what it's found, not just the
        # object's position in isolation.
        drone_rec = next((r for r in reversed(self._records) if r.telemetry.valid), None)
        if drone_rec is not None:
            de, dn = self._project_local_m(drone_rec.telemetry.latitude, drone_rec.telemetry.longitude)
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
        self._load_preview(rec.screenshot_path)
        self._redraw_map()  # re-highlight this record's track on the map

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
    # Live preview (opt-in, ~2fps, see set_detection_link/_build_layout)
    # =====================================================================

    def _on_live_toggle(self) -> None:
        if self._live_enabled.get():
            if not _PIL_AVAILABLE or self._link is None:
                self._live_enabled.set(False)
                return
            self._poll_live_frame()
        else:
            if self._live_job is not None:
                self.after_cancel(self._live_job)
                self._live_job = None

    def _poll_live_frame(self) -> None:
        if not self._live_enabled.get() or self._link is None:
            return
        try:
            jpeg_bytes = self._link.get_latest_annotated_frame_jpeg()
        except AttributeError:
            # Binding not wired up yet on the C++/pybind11 side -- fail
            # quiet rather than spamming the console every 500ms.
            jpeg_bytes = None

        if jpeg_bytes:
            try:
                import io
                img = Image.open(io.BytesIO(bytes(jpeg_bytes)))
                img.thumbnail((640, 360))
                self._live_photo = ImageTk.PhotoImage(img)
                self._preview_label.config(image=self._live_photo, text="")
            except (OSError, ValueError):
                pass

        # 500ms -- deliberately slow. This is a supplementary situational
        # pane, not a video feed; polling faster only burns CPU/JPEG
        # encode time the capture and inference threads need more.
        self._live_job = self.after(500, self._poll_live_frame)