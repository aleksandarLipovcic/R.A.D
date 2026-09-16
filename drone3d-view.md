# `Drone3DView.py` — attitude direction indicator (ADI)

**File:** `Drone3DView.py`
**Class:** `Drone3DView(tk.Canvas)`
**Fed by:** `DroneCockpitApp._update_loop` → `update_orientation(roll,
pitch, yaw, mag_heading, mag_valid)`, every 3rd tick (~17 Hz — the
"medium" throttle tier, see
[app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))

## Responsibility

A professional-style Attitude Direction Indicator (artificial horizon),
described in the source as built to a 10-inch long-range quadcopter /
aviation-instrument standard — not a simplified hobbyist gauge.

## Betaflight sign convention

Documented explicitly in the class docstring because getting any one of
these backwards silently inverts the display in a way that's easy to miss
during a bench test and dangerous to discover in flight:

**From `MSP_ATTITUDE` (verified):**
- `roll > 0` → right side drops
- `pitch > 0` → nose drops
- `yaw > 0` → nose turns right (clockwise from above)

**ADI rendering behavior (consequently):**
- `roll > 0` → horizon tilts: right end goes **down**
- `pitch > 0` → horizon shifts **up** on screen (nose drops → more sky
  becomes visible, the same visual logic as a real ADI)

## Dual-source compass strip

This is the widget's most distinctive feature — an aviation cross-check
pattern applied to a hobby platform:

- The compass tape's **primary** source is `mag_heading_deg` (the
  magnetometer heading) — described as the drift-free reference, directly
  analogous to the magnetic compass on a real aircraft.
- A secondary **ghost marker** (cyan diamond) shows where the FC's
  gyro-integrated yaw sits on the same tape. When the two overlap, the
  aircraft's heading tracking is trustworthy; when they diverge, the pilot
  sees the gap widen in real time rather than trusting a single
  drift-prone source silently.
- A numeric **divergence badge** (a Δ value) is printed inside the heading
  box whenever `|mag − gyro| > 5°`, giving a continuous quantitative
  cross-check rather than relying on eyeballing the tape.
- The heading box's outline is color-coded by that same divergence:

  | Divergence | Color | Meaning |
  |---|---|---|
  | `< 15°` | green | Headings agree |
  | `< 30°` | yellow | Monitor |
  | `≥ 30°` | red | Heading adjustment required |

This is why `telemetry_worker.py`'s pilot-adjustable **yaw trim** exists
(see [workers.md](workers.md#telemetry_workerpy--telemetryworker)) — the
gyro-integrated yaw this widget compares against is exactly the trimmed
value the worker produces, so correcting the trim is how a pilot brings
the two sources back into agreement after a mismatch is noticed here.
