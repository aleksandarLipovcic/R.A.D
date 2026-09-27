# `osd_overlay_controls.py` — OSD settings UI & persistence

**File:** `osd_overlay_controls.py`
**Classes:** `OsdSettingsPanel`, `OsdDragOverlay`
**Persists to:** `osd_layout.json`
**Wraps:** `DroneBackend.VideoLink`'s OSD element API (see
[../modules/videolink.md](../modules/videolink.md#software-osd-overlay))
**Opened from:** `FPVWidget`'s "OSD" button (see [fpv-widget.md](fpv-widget.md#osd-settings-entry-point))

## Responsibility

The front-end for the OSD overlay that `VideoLink` draws itself, natively,
in the same off-screen buffer as the video frame. This module owns **none**
of the actual drawing — it only flips enabled/anchor/position state on the
`DroneBackend.VideoLink` instance and persists that state to disk. Nothing
drawn from here ever touches a pixel `DetectionLink` sees, since the OSD is
composited after the frame is captured (see
[../modules/videolink.md](../modules/videolink.md)).

## Two pieces

### `OsdSettingsPanel`

A small popup: one checkbox row per OSD element, a 3×3 anchor-grid button
per element for the nine preset placements
(`TOP_LEFT`…`BOTTOM_RIGHT`, `CENTER`), a master enable + lock checkbox,
Save/Load buttons, and an "Edit Positions" button that hands off to
`OsdDragOverlay` for anything that isn't one of the nine presets.

Element labels shown in the panel:

| id | Label |
|---|---|
| `altitude` | Altitude |
| `horizon` | Artificial horizon + sidebars |
| `compass` | Compass |
| `battery` | Battery voltage / % |
| `rssi` | RSSI / signal strength |
| `gps` | GPS lock + satellite count |
| `timer` | Flight timer |
| `home_distance` | Distance to home + speed |

### `OsdDragOverlay`

A borderless, color-keyed `Toplevel` placed exactly over the FPV video
host frame while — and only while — the pilot is in "edit positions" mode.
Shows a ghost box per enabled element (approximate size; real sizing
happens in GDI on the C++ side, so this is a placement aid, not a
pixel-exact preview) plus alignment guides — a center cross, rule-of-thirds
lines, corner margins — that light up and snap the drag when the pointer
gets close, the same way PowerPoint/Figma-style guides work. A drag that
never gets close enough to any guide to snap commits as a free `CUSTOM`
position instead, stored as fractional coordinates (`custom_fx`/`custom_fy`)
rather than pixels.

## Persistence — `save_layout()` / `load_layout()`

Deliberately dumb: plain JSON, no schema versioning yet, entirely on the
Python side — `VideoLink` only exposes get/set on individual elements; it
has no concept of a file existing at all.

- `save_layout(video_link, path=DEFAULT_PATH)` — reads every element's
  current layout off `VideoLink` (`get_osd_element_ids()` +
  `get_osd_element_layout(id)` per element) plus the master
  enabled/locked flags, and writes it out. Uses the same atomic
  write-then-`os.replace()` temp-file pattern as `MapTiles`'s tile
  downloads, avoiding a half-written file on a crash mid-save.
- `load_layout(video_link, path=DEFAULT_PATH)` — pushes a previously saved
  layout back onto `VideoLink`. Returns `False` if nothing is saved yet
  (first run) — explicitly **not** treated as an error, since `VideoLink`'s
  own constructor defaults already cover that case. Silently skips any
  saved element id no longer present in
  `video_link.get_osd_element_ids()`, tolerating a saved file from an
  older build with different element ids.

`DEFAULT_PATH` resolves to `osd_layout.json` next to this module — no
existing settings directory convention was assumed for this to slot into.

## The lazy-enum-resolution bug (and why `_osd_anchor_enum()` exists)

`_osd_anchor_enum()` resolves `DroneBackend.OsdAnchor` **lazily, on every
call**, rather than importing it once at module load time. The doc comment
in the source explains why this matters, in detail:

This module is imported via `FPVWidget`, which `main.py` imports well
before it appends `DroneBackend`'s search path and calls
`import DroneBackend` itself. Importing `OsdAnchor` eagerly at the top of
this file would race that setup and fail on **every single run**,
permanently binding `OsdAnchor` to a broken sentinel value with no way to
recover once `main.py`'s own import later succeeds. This was, per the
source comment, exactly the bug that took down **every instrument panel**,
not just the OSD one: the failure landed inside `FPVWidget.attach()`, which
runs after `_setup_ui()` has already built every widget but before
`root.mainloop()` is ever reached — so the whole app died before a single
frame was ever drawn.

By the time anything in this file actually needs the enum — opening the
settings panel, loading a saved layout — `import DroneBackend` is a
`sys.modules` cache hit, not a re-run of the module or a DLL reload, so
calling `_osd_anchor_enum()` on every use costs nothing measurable.

## `attach_osd_telemetry()` — a note on actual wiring

This helper is a one-line convenience:

```python
def attach_osd_telemetry(video_link, telemetry_provider) -> None:
    video_link.set_telemetry_provider(telemetry_provider)
```

The module docstring suggests wiring `video_link`'s telemetry provider to
"the exact same callable already passed to
`DetectionLink.set_telemetry_provider()`". **The actual app wiring in
`DroneCockpitUI.py` does not do this** — it uses two separate trampolines
(`_get_osd_telemetry` and `_get_detection_telemetry`) with different
validity/altitude semantics, and this helper function is not actually
called from `DroneCockpitApp.__init__`. See
[app-shell.md](app-shell.md#telemetry-trampolines) for exactly why sharing
one callable breaks the OSD. Treat this helper and its docstring as
aspirational/legacy rather than a description of the current wiring.

## `osd_layout.json` — the persisted file

Top-level shape:

```json
{
  "overlay_enabled": true,
  "locked": true,
  "elements": {
    "<element_id>": {
      "enabled": true,
      "anchor": "TOP_LEFT",
      "margin_x": 16,
      "margin_y": 16,
      "custom_fx": 0.5,
      "custom_fy": 0.5
    }
  }
}
```

The checked-in copy has all eight elements enabled, spread across all nine
anchor presets (`altitude`→`TOP_LEFT`, `horizon`→`CENTER`,
`compass`→`BOTTOM_CENTER`, `battery`→`BOTTOM_LEFT`, `rssi`→`TOP_RIGHT`,
`gps`→`MIDDLE_LEFT`, `timer`→`TOP_CENTER`,
`home_distance`→`MIDDLE_RIGHT`) — i.e. no two elements share an anchor in
the default saved layout, so the anchor-stacking logic described in
[../modules/videolink.md](../modules/videolink.md#software-osd-overlay)
isn't actually exercised by this particular saved file.
