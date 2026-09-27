# `MagWidget.py` — QMC5883L magnetometer display

**File:** `MagWidget.py`
**Class:** `MagWidget(tk.Frame)`
**Fed by:** `DroneCockpitApp._update_loop` → `update_mag(ui_data)`, every
tick (~50 Hz — see [app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))
**Wired to:** `hub.start_mag_calibration` / `hub.start_acc_calibration` as
`on_mag_calibrate` / `on_acc_calibrate` callbacks (see
[../modules/dronelink.md](../modules/dronelink.md#calibration))

## Responsibility

Displays live compass heading plus raw magnetometer X/Y/Z, and hosts the
two calibration trigger buttons for magnetometer and accelerometer
calibration.

## Aviation safety invariants

The docstring calls out three properties the widget's layout logic is
built to **never** violate, regardless of window size:

1. The heading readout (degrees) is always fully visible, never clipped.
2. The lock/signal status indicator is always visible.
3. Both calibration buttons are always fully accessible and clickable.

A hard minimum widget size (`_MIN_W`/`_MIN_H`) exists specifically to
guarantee these three things are always present and never overlap or crop
each other — the MINI layout tier (below) is sized to exactly fit that
minimum.

## Layout tiers (progressive enhancement)

| Tier | Layout |
|---|---|
| **FULL** | Compass rose + heading/status/buttons column + raw XYZ bars (needs plenty of both width and height) |
| **COMPACT** | Rose + heading/status/buttons column, no XYZ bars (decent width and height, not enough room for bars) |
| **SIDE** | No rose. Heading box on the left, buttons stacked vertically on the right — used when the panel is wide but short, since putting the heading box beside the buttons (rather than above them) means neither has to compete with the other for vertical space |
| **MINI** | No rose. Heading box on top, both buttons in a row below — the fallback tier the hard minimum size is chosen to guarantee always fits |

## Heading-box sizing fix

The source documents a specific overflow bug and its fix: in every tier,
the heading box's grid row/column is given `weight=1` while the
status/hint/button rows are given `weight=0` with an **explicit minsize**.
That means the heading box only ever receives whatever space is left over
*after* the status text and both buttons have already claimed their
guaranteed minimum space — so a larger heading font can never push the
calibration buttons out of view or off-screen. `grid_propagate(False)` is
additionally set on the heading box itself so it never grows past the size
its row/column assigns it, no matter how large a font is requested —
previously the box would size itself to fit the font first and crowd out
everything else.
