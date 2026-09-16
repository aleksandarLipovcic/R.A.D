# `GPSWidget.py` — aviation-grade GPS navigation widget

**File:** `GPSWidget.py`
**Class:** `GPSWidget(tk.Frame)`, composed of `_MapCanvas`, `_SatPanel`,
`_NavPanel`
**Fed by:** `DroneCockpitApp._update_loop` → `update_gps(ui_data)`, every
5th tick (~10 Hz — the **heaviest** widget in the app, hence the lowest
refresh rate; see [app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))
**Public API:** `update_gps(ui_data)` — the only method the app shell calls

This is the largest single frontend file in the project. It's built from
four internal pieces working together, switching between two whole-widget
layout modes depending on available size.

## Two layout modes

| Mode | Toolbar | Contents |
|---|---|---|
| **MAP mode** | 34px toolbar visible: `[+] [−] [⊙ CTR] │ SAT n │ HDOP x.xx ● [SAT▶]` | Full-width map with a HUD overlay; the satellite panel slides in from the right when toggled |
| **NAV mode** | Hidden entirely | Compact scrollable numeric readout, no map, no toolbar — a `SAT▶` button is embedded directly in the nav panel's own fix row instead |

Mode selection is automatic, driven by the widget's own size
(`_on_configure` → `_apply_map_mode()`/`_apply_nav_mode()`), not a
user-facing toggle — a small panel naturally falls back to the
numbers-only NAV mode where a map wouldn't be legible anyway.

## `_MapCanvas` — the live tile map

- Backed by the same `MapTiles.TileCache` used by `DetectionMapWidget` (see
  [map-tiles.md](map-tiles.md)) — one shared on-disk cache, whichever
  widget asks for a tile first.
- `_request_tile()`/`_fetch_tile()`/`_poll_tiles()` follow the same
  non-blocking pattern as `MapTiles.TileCache.get_cached()`: ask for a
  tile, get a path immediately if cached or `None` while a background
  fetch is kicked off, and redraw once `_poll_tiles()` notices new tiles
  have landed.
- Pan (`_on_pan_start`/`_on_pan`/`_on_pan_end`) and scroll-wheel zoom
  (`_on_scroll`) are supported directly on the canvas, plus
  `zoom_in()`/`zoom_out()`/`center_drone()` convenience methods the
  toolbar buttons call.
- A **HUD overlay** — altitude/heading roses — is drawn on top of the map
  itself (`_draw_hud`, `_draw_hud_rose`), inside a HUD box whose height is
  computed by `_hud_height(w, h)` and factored into `_eff_center_y()` so
  the drone marker is centered in the *visible* map area (the region above
  the HUD box), not the full canvas height — an explicit v7 fix noted in
  the module docstring.
- **No-GPS-fix warning**: `_draw_no_fix_warning()` plus `_pulse_no_fix()`
  draw a large, bold, attention-grabbing banner with a pulsing amber/red
  border when there's no fix — explicitly described as replacing an
  earlier version's small, easy-to-miss dim text.

## `_SatPanel` — satellite table

A scrollable per-satellite table (`update_satellites(sv_list)`) showing
GNSS constellation, SVID, C/N₀ (signal strength), quality, and lock status
per satellite — fed the already-normalized list `telemetry_worker.py`
builds from the backend's `sv_list` (see
[workers.md](workers.md#_build_ui_datastate--the-one-and-only-translation-point)).
`_cols()`/`_draw_col_header()`/`_redraw()` handle the actual per-row
rendering.

## `_NavPanel` — numeric readouts

Fix type, satellite count, HDOP, ground speed, distance/bearing to home,
and similar numeric telemetry, laid out as label/value rows
(`_lrow()`) with a rose-based needle display (`_draw_rose`/
`_point_needle`) for course/bearing. `_adapt_fix_rows(w)` reflows the row
layout to fit narrower widths.

## `GPSWidget` container logic

- `_sat_fits_inline()` decides whether the satellite panel can be shown
  inline (slid in beside the map) or needs its own `Toplevel`
  (`_open_sat_toplevel()`/`_close_sat_toplevel()`) — purely a function of
  available width, re-evaluated on every resize.
- `_update_toolbar_status(raw_valid, fix_type, num_sat, hdop)` keeps the
  compact toolbar summary (`SAT n`, `HDOP x.xx`, and a small heartbeat dot
  that blinks on `gps_heartbeat` changes) in sync without needing the full
  nav panel to be visible.
- `_normalize_sv_list(raw_list)` (module-level function) defensively
  accepts either shape of satellite data — raw backend structs or
  already-normalized dicts — so the widget doesn't care whether it's fed
  directly from a backend object or from `telemetry_worker.py`'s
  pre-processed `ui_data`.

## Units

```python
_CMS_TO_KT  = 0.019438   # cm/s -> knots
_CMS_TO_KMH = 0.036      # cm/s -> km/h
_M_TO_FT    = 3.28084    # meters -> feet
```

`ground_speed_cms`/altitude fields arrive in the backend's native units
(cm/s, meters) and are converted here at display time — the underlying
`ui_data` dict keeps the raw MSP-native units, matching the convention
`telemetry_worker.py` uses throughout (see
[workers.md](workers.md#_build_ui_datastate--the-one-and-only-translation-point)).
