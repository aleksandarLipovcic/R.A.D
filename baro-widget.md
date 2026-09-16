# `BaroWidget.py` — altitude tape & vertical speed indicator

**File:** `BaroWidget.py`
**Class:** `BaroWidget(tk.Frame)`
**Fed by:** `DroneCockpitApp._update_loop` → `update_baro(ui_data)`, every
tick (~50 Hz — see [app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))

## Responsibility

An aviation Primary Flight Display (PFD)-style vertical altitude tape
paired with a vertical speed indicator (VSI), reading from the barometer
fields `telemetry_worker.py` exposes (`baro_altitude_cm`,
`baro_vario_cm_per_sec`, `baro_valid` — sourced from `BaroBMP280` on the
C++ side, see [../modules/sensors.md](../modules/sensors.md#barobmp280--altitude--vertical-speed)).

## Layout

```
┌─────────────────────────┬──────┐
│                         │ V/S  │
│   ALTITUDE TAPE  (~75%) │ (25%)│
│                         │      │
└─────────────────────────┴──────┘
```

The VSI column has a **fixed minimum width and a maximum cap**, so it can
never dominate the panel regardless of how wide the whole widget gets —
the altitude tape simply takes whatever width is left over. Both canvases
bind `<Configure>` and redraw entirely from the actual current pixel
size — nothing about the layout is a hardcoded pixel constant.

## Tape geometry (percent-of-width constants)

All positions on the altitude tape canvas are expressed as a fraction of
the tape canvas's own width, recomputed on every resize rather than fixed
in pixels:

| Constant | Position | Meaning |
|---|---|---|
| `SPINE` | 92% | Vertical scale line, right side |
| `MAJ_L` | 84% | Major tick start |
| `MIN_L` | 89% | Minor tick start |
| `TIP_X` | 76% | Pentagon pointer tip |
| `SHLDR` | 66% | Pentagon shoulder |
| `BOX_L` | 3% | Left wall of the numeric readout box |

VSI canvas: fixed width = `clamp(W_total * 0.22, 44, 80)` px; the vertical
axis sits at 40% of the VSI's own width, with numeric labels to its right.

## Scale constants

```python
PX_PER_M_BASE = 6.0    # base pixels-per-meter at REF_H
REF_H         = 340.0  # reference canvas height the base scale was tuned for
LABEL_STEP    = 10      # meters between tape labels
```
