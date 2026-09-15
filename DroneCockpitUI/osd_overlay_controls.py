"""
osd_overlay_controls.py — Software OSD settings + drag placement
====================================================================
Front-end for the OSD overlay VideoLink now draws itself (see
VideoLink.h's "Software OSD overlay" section and VideoLink.cpp's
drawOsdOverlay()/drawOsdAltitude()/drawOsdHorizon()/drawOsdCompass()).
This module owns none of the actual drawing -- it only flips
enabled/anchor/position state on the DroneBackend.VideoLink instance and
persists that state to disk. The overlay itself is rendered natively, in
the same off-screen buffer as the video frame, so nothing drawn from here
ever touches a pixel DetectionLink sees.

Two pieces:

  OsdSettingsPanel   A small popup: one checkbox row per element, a 3x3
                      anchor-grid button per element for the common
                      "corner / center" placements, a master enable +
                      lock checkbox, Save/Load, and an "Edit Positions"
                      button that hands off to OsdDragOverlay for anything
                      that isn't one of the 9 presets.

  OsdDragOverlay      A borderless, color-keyed Toplevel placed exactly
                      over the FPV video host frame while (and only
                      while) the pilot is in "edit positions" mode. Shows
                      a ghost box per enabled element (approximate size --
                      real sizing happens in GDI on the C++ side, this is
                      a placement aid, not a pixel-exact preview) plus
                      alignment guides (center cross, rule-of-thirds,
                      corner margins) that light up and snap the drag
                      when the pointer is close, the same way
                      PowerPoint/Figma-style guides work. A drag that
                      never gets close enough to snap commits as a free
                      CUSTOM position instead.

Wiring into FPVWidget:

    from osd_overlay_controls import OsdSettingsPanel

    settings_btn = tk.Button(fpv._status_bar, text="OSD", ...,
                              command=lambda: OsdSettingsPanel(
                                  fpv, video_link, fpv._video_host).show())

video_link.set_telemetry_provider(...) should be wired ONCE, at app
startup, to the exact same callable already passed to
DetectionLink.set_telemetry_provider() (e.g.
DroneCockpitApp._get_detection_telemetry) -- see attach_osd_telemetry()
at the bottom of this file for a one-line helper that does that.
"""

import json
import os
import tkinter as tk
from tkinter import ttk


def _osd_anchor_enum():
    """Resolve DroneBackend.OsdAnchor lazily -- NOT at this module's import
    time. This module is imported via FPVWidget, which main.py imports
    (line ~133) well before it appends DroneBackend's search path and
    calls `import DroneBackend` itself (line ~196). Importing OsdAnchor
    eagerly at the top of this file would race that setup and fail every
    single run, permanently binding OsdAnchor to a sentinel with no way
    to recover once main.py's own import succeeds later -- which is
    exactly the bug that took down every instrument panel, not just the
    OSD one: the failure landed inside FPVWidget.attach(), which runs
    after _setup_ui() has already built every widget but before
    root.mainloop() is ever reached, so the whole app died before a
    single frame was ever drawn.
    By the time anything in this file actually needs the enum -- opening
    the settings panel, loading a saved layout -- `import DroneBackend`
    is a sys.modules cache hit, not a re-run of the module or a reload of
    the DLL, so calling this on every use costs nothing."""
    import DroneBackend
    return DroneBackend.OsdAnchor


# =============================================================================
# Persistence
# =============================================================================
# Kept deliberately dumb (plain JSON, no schema versioning yet) and
# entirely on the Python side -- VideoLink only exposes get/set on
# individual elements, it has no idea a file exists. Point DEFAULT_PATH
# wherever the app already keeps the angle-panel settings if there's an
# existing settings directory; this default just sits next to the app.
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "osd_layout.json")

def save_layout(video_link, path: str = DEFAULT_PATH) -> None:
    """Read every element's current layout off VideoLink and write it to disk."""
    data = {"overlay_enabled": video_link.is_osd_overlay_enabled(),
            "locked": video_link.is_osd_locked(),
            "elements": {}}
    for elem_id in video_link.get_osd_element_ids():
        layout = video_link.get_osd_element_layout(elem_id)
        data["elements"][elem_id] = {
            "enabled": layout.enabled,
            "anchor": layout.anchor.name,
            "margin_x": layout.margin_x,
            "margin_y": layout.margin_y,
            "custom_fx": layout.custom_fx,
            "custom_fy": layout.custom_fy,
        }
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp_path, path)   # atomic-ish swap, avoids a half-written file on crash


def load_layout(video_link, path: str = DEFAULT_PATH) -> bool:
    """Push a previously saved layout back onto VideoLink. Returns False if
    there's nothing saved yet (first run) -- that's not an error, VideoLink's
    own constructor defaults already cover that case."""
    if not os.path.exists(path):
        return False
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    video_link.set_osd_overlay_enabled(bool(data.get("overlay_enabled", True)))
    video_link.set_osd_locked(bool(data.get("locked", False)))
    for elem_id, elem in data.get("elements", {}).items():
        if elem_id not in video_link.get_osd_element_ids():
            continue   # a saved id from an older build that's since been removed
        video_link.set_osd_element_enabled(elem_id, bool(elem.get("enabled", True)))
        anchor_name = elem.get("anchor", "TOP_LEFT")
        if anchor_name == "CUSTOM":
            video_link.set_osd_element_custom_position(
                elem_id, float(elem.get("custom_fx", 0.5)), float(elem.get("custom_fy", 0.5)))
        else:
            Anchor = _osd_anchor_enum()
            anchor = getattr(Anchor, anchor_name, Anchor.TOP_LEFT)
            video_link.set_osd_element_anchor(
                elem_id, anchor, int(elem.get("margin_x", 16)), int(elem.get("margin_y", 16)))
    return True


def attach_osd_telemetry(video_link, telemetry_provider) -> None:
    """One-line wiring helper: telemetry_provider is the exact same no-arg
    callable already passed to DetectionLink.set_telemetry_provider() --
    reuse it, don't build a second trampoline that queries DroneLink again."""
    video_link.set_telemetry_provider(telemetry_provider)


# =============================================================================
# Settings panel
# =============================================================================

_ELEMENT_LABELS = {
    "altitude": "Altitude",
    "horizon": "Artificial horizon + sidebars",
    "compass": "Compass",
    "battery": "Battery voltage / %",
    "rssi": "RSSI / signal strength",
    "gps": "GPS lock + satellite count",
    "timer": "Flight timer",
    "home_distance": "Distance to home + speed",
}

# 3x3 grid button layout -> (OsdAnchor enum name, grid row, grid col)
_ANCHOR_GRID = [
    ("TOP_LEFT", 0, 0), ("TOP_CENTER", 0, 1), ("TOP_RIGHT", 0, 2),
    ("MIDDLE_LEFT", 1, 0), ("CENTER", 1, 1), ("MIDDLE_RIGHT", 1, 2),
    ("BOTTOM_LEFT", 2, 0), ("BOTTOM_CENTER", 2, 1), ("BOTTOM_RIGHT", 2, 2),
]


class OsdSettingsPanel:
    """A small Toplevel: per-element enable checkbox + a 3x3 anchor-grid
    button, plus master enable/lock and save/load. Opened fresh each time
    (not a persistent widget) so it always reflects VideoLink's current
    state rather than a stale snapshot from whenever it was first built."""

    def __init__(self, parent_widget, video_link, video_host_widget, layout_path: str = DEFAULT_PATH,
                 on_close=None):
        self._video_link = video_link
        self._video_host = video_host_widget
        self._layout_path = layout_path
        self._parent = parent_widget
        self._drag_overlay = None
        self._on_close_cb = on_close

        self._win = tk.Toplevel(parent_widget)
        self._win.title("OSD Overlay")
        self._win.configure(bg="#1a1a1a")
        # Was resizable(False, False) with every element row stacked
        # straight into self._win -- fine for the original handful of
        # elements, but stopped fitting once the list grew (and never fit
        # on a shorter screen anyway). Now resizable, with the master
        # toggle row and the Save/Load/Close row pinned in place and the
        # element list (below) the one section that scrolls.
        self._win.resizable(True, True)
        self._win.minsize(300, 220)
        self._win.geometry("360x420")
        # Keep it on top of the main window but not system-modal -- the
        # pilot may want to glance at the live video while adjusting this.
        self._win.transient(parent_widget.winfo_toplevel())
        self._win.grid_columnconfigure(0, weight=1)
        self._win.grid_rowconfigure(2, weight=1)   # the scrollable element list, set below

        self._enabled_vars = {}
        self._anchor_btns = {}   # elem_id -> {anchor_name: Button}

        row = 0
        master_frame = tk.Frame(self._win, bg="#1a1a1a")
        master_frame.grid(row=row, column=0, columnspan=2, sticky="w", padx=10, pady=(10, 4))
        self._master_var = tk.BooleanVar(value=video_link.is_osd_overlay_enabled())
        tk.Checkbutton(master_frame, text="Show OSD overlay", variable=self._master_var,
                        command=self._on_master_toggle, bg="#1a1a1a", fg="#ffffff",
                        selectcolor="#333333", activebackground="#1a1a1a").pack(side="left")
        self._lock_var = tk.BooleanVar(value=video_link.is_osd_locked())
        tk.Checkbutton(master_frame, text="Lock positions", variable=self._lock_var,
                        command=self._on_lock_toggle, bg="#1a1a1a", fg="#ffffff",
                        selectcolor="#333333", activebackground="#1a1a1a").pack(side="left", padx=(16, 0))
        row += 1

        ttk.Separator(self._win, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", padx=10, pady=4)
        row += 1

        # -----------------------------------------------------------
        # Scrollable element list. Canvas + inner Frame is the standard
        # Tk pattern -- ttk has no native scrollable frame. The canvas
        # sits in the row grid_rowconfigure(2) above made stretchy, so it
        # grows/shrinks with the window; a Scrollbar covers whatever
        # doesn't fit. Master toggle row and the button row below stay
        # outside this frame, so they never scroll out of view.
        # -----------------------------------------------------------
        list_container = tk.Frame(self._win, bg="#1a1a1a")
        list_container.grid(row=row, column=0, columnspan=2, sticky="nsew", padx=(10, 0), pady=2)
        list_container.grid_rowconfigure(0, weight=1)
        list_container.grid_columnconfigure(0, weight=1)

        self._canvas = tk.Canvas(list_container, bg="#1a1a1a", highlightthickness=0)
        self._canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = tk.Scrollbar(list_container, orient="vertical", command=self._canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._canvas.configure(yscrollcommand=scrollbar.set)

        self._elements_frame = tk.Frame(self._canvas, bg="#1a1a1a")
        self._elements_window = self._canvas.create_window((0, 0), window=self._elements_frame, anchor="nw")
        self._elements_frame.bind("<Configure>", self._on_elements_frame_configure)
        self._canvas.bind("<Configure>", self._on_canvas_configure)
        # Wheel scroll only while the pointer is actually over the list --
        # bind_all on <Enter>/unbind_all on <Leave> so this panel doesn't
        # steal scroll events meant for whatever's behind/around it.
        self._canvas.bind("<Enter>", lambda e: self._canvas.bind_all("<MouseWheel>", self._on_mousewheel))
        self._canvas.bind("<Leave>", lambda e: self._canvas.unbind_all("<MouseWheel>"))
        row += 1

        elem_row = 0
        for elem_id in video_link.get_osd_element_ids():
            elem_row = self._build_element_row(elem_id, elem_row)

        row += 1
        btn_frame = tk.Frame(self._win, bg="#1a1a1a")
        btn_frame.grid(row=row, column=0, columnspan=2, pady=(6, 10))
        tk.Button(btn_frame, text="Edit Positions (drag)", command=self._toggle_edit_mode
                  ).pack(side="left", padx=4)
        tk.Button(btn_frame, text="Save", command=self._on_save).pack(side="left", padx=4)
        tk.Button(btn_frame, text="Load", command=self._on_load).pack(side="left", padx=4)
        tk.Button(btn_frame, text="Close", command=self._on_close).pack(side="left", padx=4)

        self._win.protocol("WM_DELETE_WINDOW", self._on_close)

    # -------------------------------------------------------------------
    # Scroll region upkeep
    # -------------------------------------------------------------------
    def _on_elements_frame_configure(self, _event):
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        # Keep the inner frame exactly as wide as the visible canvas, so
        # rows never get clipped on one side or leave a dead strip on the
        # other when the window is resized. Height is left alone -- the
        # scrollbar is what handles that dimension.
        self._canvas.itemconfigure(self._elements_window, width=event.width)

    def _on_mousewheel(self, event):
        self._canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def show(self):
        self._win.deiconify()
        self._win.lift()

    # -------------------------------------------------------------------
    # Per-element row: checkbox + 3x3 anchor grid
    # -------------------------------------------------------------------
    def _build_element_row(self, elem_id: str, row: int) -> int:
        label = _ELEMENT_LABELS.get(elem_id, elem_id)
        layout = self._video_link.get_osd_element_layout(elem_id)

        var = tk.BooleanVar(value=layout.enabled)
        self._enabled_vars[elem_id] = var
        tk.Checkbutton(self._elements_frame, text=label, variable=var,
                        command=lambda e=elem_id: self._on_enabled_toggle(e),
                        bg="#1a1a1a", fg="#ffffff", selectcolor="#333333",
                        activebackground="#1a1a1a", anchor="w"
                        ).grid(row=row, column=0, sticky="w", padx=10)

        grid_frame = tk.Frame(self._elements_frame, bg="#1a1a1a")
        grid_frame.grid(row=row, column=1, padx=(0, 10), pady=2)
        self._anchor_btns[elem_id] = {}
        current_anchor = layout.anchor.name
        for anchor_name, r, c in _ANCHOR_GRID:
            is_current = (anchor_name == current_anchor)
            btn = tk.Button(
                grid_frame, text="●" if is_current else "○", width=2,
                fg="#00ff88" if is_current else "#666666",
                bg="#111111", relief="flat",
                command=lambda e=elem_id, a=anchor_name: self._on_anchor_pick(e, a))
            btn.grid(row=r, column=c, padx=1, pady=1)
            self._anchor_btns[elem_id][anchor_name] = btn

        return row + 1

    def _refresh_anchor_grid(self, elem_id: str):
        layout = self._video_link.get_osd_element_layout(elem_id)
        current_anchor = layout.anchor.name
        for anchor_name, btn in self._anchor_btns[elem_id].items():
            is_current = (anchor_name == current_anchor)
            btn.configure(text="●" if is_current else "○",
                          fg="#00ff88" if is_current else "#666666")

    # -------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------
    def _on_master_toggle(self):
        self._video_link.set_osd_overlay_enabled(self._master_var.get())

    def _on_lock_toggle(self):
        self._video_link.set_osd_locked(self._lock_var.get())

    def _on_enabled_toggle(self, elem_id: str):
        self._video_link.set_osd_element_enabled(elem_id, self._enabled_vars[elem_id].get())

    def _on_anchor_pick(self, elem_id: str, anchor_name: str):
        anchor = getattr(_osd_anchor_enum(), anchor_name)
        # 16px margin default for every preset -- matches VideoLink's own
        # constructor defaults, so picking a corner here looks the same
        # as the app's very first run.
        self._video_link.set_osd_element_anchor(elem_id, anchor, 16, 16)
        self._refresh_anchor_grid(elem_id)

    def _on_save(self):
        save_layout(self._video_link, self._layout_path)

    def _on_load(self):
        load_layout(self._video_link, self._layout_path)
        # Reflect whatever just got loaded back into every control.
        self._master_var.set(self._video_link.is_osd_overlay_enabled())
        self._lock_var.set(self._video_link.is_osd_locked())
        for elem_id in self._video_link.get_osd_element_ids():
            layout = self._video_link.get_osd_element_layout(elem_id)
            self._enabled_vars[elem_id].set(layout.enabled)
            self._refresh_anchor_grid(elem_id)

    def _toggle_edit_mode(self):
        if self._drag_overlay is not None:
            self._drag_overlay.close()
            self._drag_overlay = None
            return
        if self._video_link.is_osd_locked():
            # Refuse quietly rather than opening an overlay the pilot can't
            # actually drag anything in -- Lock is meant to prevent exactly
            # this, not just prevent the drag committing at the end.
            return
        self._drag_overlay = OsdDragOverlay(
            self._parent, self._video_link, self._video_host,
            on_close=lambda: self._on_edit_mode_closed())

    def _on_edit_mode_closed(self):
        self._drag_overlay = None
        for elem_id in self._video_link.get_osd_element_ids():
            self._refresh_anchor_grid(elem_id)

    def _on_close(self):
        if self._drag_overlay is not None:
            self._drag_overlay.close()
        self._win.destroy()
        if self._on_close_cb is not None:
            self._on_close_cb()

    def is_open(self) -> bool:
        """True if this panel's window is still alive. Lets a caller that
        keeps a reference (e.g. FPVWidget) tell an already-open panel from
        one that was already closed/destroyed, instead of re-creating a
        second Toplevel on top of it."""
        try:
            return bool(self._win.winfo_exists())
        except tk.TclError:
            return False


# =============================================================================
# Drag-to-position overlay
# =============================================================================
#
# The live video is a foreign native HWND (see FPVWidget/VideoLink's
# attach_to_window), not a Tk widget -- Tk never sees mouse events over
# it. So dragging isn't done on the video itself: this opens a plain Tk
# Toplevel sized and positioned to exactly cover the video host, using
# Windows' -transparentcolor so the live video underneath stays fully
# visible while this window's Canvas captures the mouse and draws ghost
# boxes + guides on top. Positions are committed to VideoLink only on
# mouse release; nothing here talks to VideoLink per mouse-move.

_TRANSPARENT_KEY = "#010203"   # arbitrary color unlikely to appear in real UI chrome
_SNAP_PX = 18                  # how close to a guide before it snaps
_GHOST_SIZE = {                # approximate footprint for the drag ghost box --
    "altitude": (110, 30),     # cosmetic only, real sizing happens in GDI
    "horizon": (200, 200),
    "compass": (260, 30),
    "battery": (130, 30),
    "rssi": (110, 30),
    "gps": (90, 30),
    "timer": (80, 30),
    "home_distance": (170, 30),
}


class OsdDragOverlay:
    def __init__(self, parent_widget, video_link, video_host_widget, on_close=None):
        self._video_link = video_link
        self._video_host = video_host_widget
        self._on_close = on_close
        self._drag_elem = None
        self._drag_offset = (0, 0)

        self._win = tk.Toplevel(parent_widget)
        self._win.overrideredirect(True)   # no titlebar/border -- must sit pixel-exact on the video
        self._win.attributes("-topmost", True)
        self._win.configure(bg=_TRANSPARENT_KEY)
        self._win.attributes("-transparentcolor", _TRANSPARENT_KEY)

        self._sync_geometry()
        # Video host resizing (panel dragged/resized) while in edit mode
        # should keep this overlay pixel-locked to it, same as VideoLink's
        # own render window does via resize_window().
        self._configure_binding = video_host_widget.bind("<Configure>", lambda e: self._sync_geometry(), add="+")

        self._canvas = tk.Canvas(self._win, bg=_TRANSPARENT_KEY, highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)
        self._canvas.bind("<ButtonPress-1>", self._on_press)
        self._canvas.bind("<B1-Motion>", self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)

        self._redraw()

    def _sync_geometry(self):
        self._video_host.update_idletasks()
        x = self._video_host.winfo_rootx()
        y = self._video_host.winfo_rooty()
        w = self._video_host.winfo_width()
        h = self._video_host.winfo_height()
        self._win.geometry(f"{w}x{h}+{x}+{y}")
        self._w, self._h = w, h

    # -------------------------------------------------------------------
    # Ghost boxes + guides
    # -------------------------------------------------------------------
    def _element_center_px(self, elem_id: str):
        layout = self._video_link.get_osd_element_layout(elem_id)
        if layout.anchor.name == "CUSTOM":
            return layout.custom_fx * self._w, layout.custom_fy * self._h
        # Re-derive an approximate on-screen center for the 8 presets --
        # mirrors VideoLink::resolveOsdOrigin's corner/edge math closely
        # enough for a drag starting point (exact GDI sizing isn't
        # available here, and doesn't need to be: the first mouse-move
        # already replaces this with the pointer's own position).
        gw, gh = _GHOST_SIZE.get(elem_id, (120, 30))
        mx, my = layout.margin_x, layout.margin_y
        xs = {"TOP_LEFT": mx, "MIDDLE_LEFT": mx, "BOTTOM_LEFT": mx,
              "TOP_CENTER": self._w / 2, "CENTER": self._w / 2, "BOTTOM_CENTER": self._w / 2,
              "TOP_RIGHT": self._w - mx, "MIDDLE_RIGHT": self._w - mx, "BOTTOM_RIGHT": self._w - mx}
        ys = {"TOP_LEFT": my, "TOP_CENTER": my, "TOP_RIGHT": my,
              "MIDDLE_LEFT": self._h / 2, "CENTER": self._h / 2, "MIDDLE_RIGHT": self._h / 2,
              "BOTTOM_LEFT": self._h - my, "BOTTOM_CENTER": self._h - my, "BOTTOM_RIGHT": self._h - my}
        anchor = layout.anchor.name
        cx = xs.get(anchor, self._w / 2)
        cy = ys.get(anchor, self._h / 2)
        # xs/ys above give the box's near-edge, not center -- nudge inward
        # by half the ghost size for left/top-anchored, outward for
        # right/bottom-anchored, so the drawn ghost roughly matches where
        # GDI will actually paint the real element.
        if "LEFT" in anchor:
            cx += gw / 2
        elif "RIGHT" in anchor:
            cx -= gw / 2
        if "TOP" in anchor:
            cy += gh / 2
        elif "BOTTOM" in anchor:
            cy -= gh / 2
        return cx, cy

    def _redraw(self, drag_pos=None):
        self._canvas.delete("all")
        # Alignment guides -- thirds + center cross, always shown while in
        # edit mode so the pilot can see snap targets before even starting
        # a drag, not just once one is in progress.
        for frac in (1 / 3, 2 / 3):
            self._canvas.create_line(self._w * frac, 0, self._w * frac, self._h,
                                      fill="#444444", dash=(3, 3))
            self._canvas.create_line(0, self._h * frac, self._w, self._h * frac,
                                      fill="#444444", dash=(3, 3))
        self._canvas.create_line(self._w / 2, 0, self._w / 2, self._h, fill="#666666", dash=(2, 4))
        self._canvas.create_line(0, self._h / 2, self._w, self._h / 2, fill="#666666", dash=(2, 4))

        for elem_id in self._video_link.get_osd_element_ids():
            layout = self._video_link.get_osd_element_layout(elem_id)
            gw, gh = _GHOST_SIZE.get(elem_id, (120, 30))
            if drag_pos is not None and elem_id == self._drag_elem:
                cx, cy = drag_pos
            else:
                cx, cy = self._element_center_px(elem_id)
            color = "#00ff88" if layout.enabled else "#555555"
            self._canvas.create_rectangle(cx - gw / 2, cy - gh / 2, cx + gw / 2, cy + gh / 2,
                                           outline=color, width=2, dash=() if layout.enabled else (4, 2))
            self._canvas.create_text(cx, cy, text=_ELEMENT_LABELS.get(elem_id, elem_id),
                                      fill=color, font=("Consolas", 9))

        # Highlight whichever guide the active drag is currently snapped
        # to, so the snap feels deliberate rather than a mystery jump.
        if drag_pos is not None:
            snap = self._nearest_snap(*drag_pos)
            if snap is not None:
                sx, sy = snap
                self._canvas.create_line(sx, 0, sx, self._h, fill="#ffcc00", width=2)
                self._canvas.create_line(0, sy, self._w, sy, fill="#ffcc00", width=2)

    # -------------------------------------------------------------------
    # Snapping
    # -------------------------------------------------------------------
    def _nearest_snap(self, px, py):
        """Returns the (x, y) guide line position closest to (px, py) if
        within _SNAP_PX on both axes, else None. Only the center cross and
        the two thirds lines are treated as snap TARGETS (matching the
        common corner/center placements the pilot asked for) -- the
        margin-based corner anchors are handled separately in
        _resolve_anchor_from_snap below, since they depend on which
        element is being placed (ghost size)."""
        xs = [self._w / 3, self._w / 2, 2 * self._w / 3]
        ys = [self._h / 3, self._h / 2, 2 * self._h / 3]
        near_x = min(xs, key=lambda x: abs(x - px))
        near_y = min(ys, key=lambda y: abs(y - py))
        if abs(near_x - px) <= _SNAP_PX and abs(near_y - py) <= _SNAP_PX:
            return near_x, near_y
        return None

    def _resolve_anchor_from_position(self, px, py):
        """Maps a released drag position to one of the 9 presets if it's
        close enough to a natural corner/edge/center, else None (meaning:
        commit as CUSTOM instead)."""
        third_x = self._w / 3
        third_y = self._h / 3
        # Horizontal zone.
        if px < third_x - _SNAP_PX:
            hz = "LEFT"
        elif px > 2 * third_x + _SNAP_PX:
            hz = "RIGHT"
        elif abs(px - self._w / 2) <= _SNAP_PX:
            hz = "CENTER"
        else:
            return None
        # Vertical zone.
        if py < third_y - _SNAP_PX:
            vz = "TOP"
        elif py > 2 * third_y + _SNAP_PX:
            vz = "BOTTOM"
        elif abs(py - self._h / 2) <= _SNAP_PX:
            vz = "MIDDLE"
        else:
            return None

        if hz == "CENTER" and vz == "MIDDLE":
            return "CENTER"
        if vz == "MIDDLE":
            return f"MIDDLE_{hz}"
        if hz == "CENTER":
            return f"{vz}_CENTER"
        return f"{vz}_{hz}"

    # -------------------------------------------------------------------
    # Mouse handling
    # -------------------------------------------------------------------
    def _on_press(self, event):
        if self._video_link.is_osd_locked():
            return
        # Pick whichever enabled element's ghost the press landed inside,
        # topmost (last-drawn) first.
        for elem_id in reversed(self._video_link.get_osd_element_ids()):
            layout = self._video_link.get_osd_element_layout(elem_id)
            if not layout.enabled:
                continue
            cx, cy = self._element_center_px(elem_id)
            gw, gh = _GHOST_SIZE.get(elem_id, (120, 30))
            if abs(event.x - cx) <= gw / 2 and abs(event.y - cy) <= gh / 2:
                self._drag_elem = elem_id
                self._drag_offset = (event.x - cx, event.y - cy)
                return

    def _on_drag(self, event):
        if self._drag_elem is None:
            return
        ox, oy = self._drag_offset
        self._redraw(drag_pos=(event.x - ox, event.y - oy))

    def _on_release(self, event):
        if self._drag_elem is None:
            return
        ox, oy = self._drag_offset
        px, py = event.x - ox, event.y - oy
        px = max(0, min(self._w, px))
        py = max(0, min(self._h, py))

        anchor_name = self._resolve_anchor_from_position(px, py)
        if anchor_name is not None:
            anchor = getattr(_osd_anchor_enum(), anchor_name)
            self._video_link.set_osd_element_anchor(self._drag_elem, anchor, 16, 16)
        else:
            self._video_link.set_osd_element_custom_position(
                self._drag_elem, px / self._w, py / self._h)

        self._drag_elem = None
        self._redraw()

    def close(self):
        try:
            self._video_host.unbind("<Configure>", self._configure_binding)
        except Exception:
            pass
        self._win.destroy()
        if self._on_close:
            self._on_close()