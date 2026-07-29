"""
DetectionMapWidget.py — Supplementary detection/map display window
=======================================================================
This is deliberately a SEPARATE top-level window from the pilot's FPV
panel, not another pane fighting for space in the same layout -- the
whole point (per the paralel-detection-window design) is that nothing
this widget does can ever obstruct or contend with the live video feed.

Like FPVWidget, this widget never touches raw pixel data. It receives
already-built DetectionRecord objects (small structs: class name,
confidence, lat/lon, a screenshot *path*) from DetectionWorker and
displays them with ordinary Tk widgets. The one image load per selection
(via PIL, reading the saved screenshot off disk) is the only place this
file opens actual image bytes, and it happens on demand -- only when the
pilot clicks a specific detection to inspect it -- not on every poll
tick.

The map panel is a placeholder grid, not a real georeferenced basemap --
plugging in actual satellite tiles/projection is future work (this is
the "terrain matching / satellite basemap" piece discussed separately).
For now it plots pins in a simple local-offset scatter so pins are at
least spatially comparable to each other during a flight, with the
detection list as the primary, trustworthy source of truth.
"""

import time
import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image, ImageTk
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False


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

        self._build_layout()

    # =====================================================================
    # Layout
    # =====================================================================

    def _build_layout(self):
        paned = ttk.PanedWindow(self, orient="horizontal")
        paned.pack(fill="both", expand=True)

        # ── Left: detection list ────────────────────────────────────
        left = tk.Frame(paned, bg="#1a1a1a")
        paned.add(left, weight=1)

        tk.Label(left, text="Detections", fg="#ffffff", bg="#1a1a1a",
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=8, pady=(8, 2))

        columns = ("class", "conf", "time", "geo")
        self._tree = ttk.Treeview(left, columns=columns, show="headings", selectmode="browse")
        self._tree.heading("class", text="Class")
        self._tree.heading("conf", text="Conf.")
        self._tree.heading("time", text="Time")
        self._tree.heading("geo", text="Lat / Lon")
        self._tree.column("class", width=90)
        self._tree.column("conf", width=55, anchor="e")
        self._tree.column("time", width=80, anchor="center")
        self._tree.column("geo", width=170)
        self._tree.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._tree.bind("<<TreeviewSelect>>", self._on_select)

        # ── Right: pin map (placeholder) + screenshot preview ────────
        right = tk.Frame(paned, bg="#1a1a1a")
        paned.add(right, weight=2)

        self._map_canvas = tk.Canvas(right, bg="#0d1f14", height=260, highlightthickness=0)
        self._map_canvas.pack(fill="x", padx=8, pady=(8, 4))
        tk.Label(right, text="Pin positions are relative to the flight area, not a real basemap yet",
                 fg="#888888", bg="#1a1a1a", font=("Segoe UI", 8)).pack(anchor="w", padx=8)

        self._preview_label = tk.Label(right, bg="#000000")
        self._preview_label.pack(fill="both", expand=True, padx=8, pady=8)

        self._detail_lbl = tk.Label(right, text="Select a detection to view its screenshot",
                                     fg="#cccccc", bg="#1a1a1a", font=("Segoe UI", 9), justify="left")
        self._detail_lbl.pack(anchor="w", padx=8, pady=(0, 8))

    # =====================================================================
    # Public API — called from the main app's poll loop
    # =====================================================================

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
            self._tree.insert("", "end", iid=str(rec.id),
                               values=(rec.class_name, f"{rec.confidence:.2f}", t, geo))
            self._plot_pin(rec)

        # Keep the list scrolled to the newest detection.
        children = self._tree.get_children()
        if children:
            self._tree.see(children[-1])

    # =====================================================================
    # Internals
    # =====================================================================

    def _plot_pin(self, rec) -> None:
        if not rec.georeferenced or not self._records:
            return
        # Placeholder projection: scatter relative to the min/max lat/lon
        # seen so far, scaled into the canvas -- purely a "does this
        # cluster with other detections" view until real tiles are wired
        # in. Recomputed on every new pin since the bounds can grow.
        geo_records = [r for r in self._records if r.georeferenced]
        if len(geo_records) < 2:
            self._map_canvas.delete("all")
            self._map_canvas.create_oval(200 - 4, 130 - 4, 200 + 4, 130 + 4,
                                          fill="#ffaa00", outline="")
            return

        lats = [r.latitude for r in geo_records]
        lons = [r.longitude for r in geo_records]
        lat_range = max(max(lats) - min(lats), 1e-6)
        lon_range = max(max(lons) - min(lons), 1e-6)

        w = self._map_canvas.winfo_width() or 400
        h = self._map_canvas.winfo_height() or 260
        pad = 30

        self._map_canvas.delete("all")
        for r in geo_records:
            x = pad + (r.longitude - min(lons)) / lon_range * (w - 2 * pad)
            y = pad + (max(lats) - r.latitude) / lat_range * (h - 2 * pad)
            color = "#00e0ff" if str(r.id) == self._tree.focus() else "#ffaa00"
            self._map_canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill=color, outline="")

    def _on_select(self, _event) -> None:
        selection = self._tree.selection()
        if not selection:
            return
        rec_id = int(selection[0])
        rec = next((r for r in self._records if r.id == rec_id), None)
        if rec is None:
            return

        self._detail_lbl.config(
            text=f"{rec.class_name}  ({rec.confidence:.2f})\n"
                 f"heading {rec.telemetry.heading_deg:.0f} deg, "
                 f"gimbal tilt {rec.telemetry.gimbal_tilt_deg:.0f} deg\n"
                 f"{rec.screenshot_path or 'no screenshot saved'}"
        )
        self._load_preview(rec.screenshot_path)
        self._plot_pin(rec)  # re-highlight selected pin

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