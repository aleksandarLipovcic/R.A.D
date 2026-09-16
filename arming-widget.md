# `ArmingWidget.py` — pre-flight checklist & arming diagnostics

**File:** `ArmingWidget.py`
**Class:** `ArmingWidget(tk.Frame)`
**Fed by:** `DroneCockpitApp._update_loop` → `update_arming(ui_data)`,
every tick (~50 Hz — see
[app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))

## Responsibility

A two-tier aviation-style checklist gating whether the pilot should arm.

## Section A — automatic checks (telemetry-driven)

Updates every tick, straight from live data: gyro, accelerometer, battery
voltage, RC link, GPS fix, motors idle.

## Section B — pilot sign-off (manual)

Ordinary checkboxes the pilot ticks by hand before each flight — not
derived from telemetry at all, since some pre-flight items (propellers
secured, area clear, etc.) have no sensor to check them.

## Status banner

Pinned at the bottom, outside the scroll area, always visible:

| Color | Text | Meaning |
|---|---|---|
| Grey | "WAITING FOR DATA…" | No telemetry yet |
| Red | "BLOCKED — FIX AUTO CHECKS" | One or more Section A checks failing |
| Orange | "COMPLETE SIGN-OFF ITEMS" | Auto checks OK, manual items remain |
| Green | "✔ READY TO ARM" | Everything passed |

## Deliberate omission: FC arming-disable-flags check

Section A intentionally does **not** include a check against the raw
`arming_disable_flags` bitmask from `MSP_STATUS_EX` (see
[../modules/dronelink.md](../modules/dronelink.md)), even though that data
is available. The reasoning, stated directly in the source:

> The raw arming-disable bitmask always contains `MSP_OVERRIDE`,
> `CLI_ACTIVE`, and `OSD_MENU` whenever the FC is connected over USB. These
> flags cannot be cleared by the pilot without physically unplugging the
> cable. Individual hardware health items (gyro, acc, I2C) are already
> covered by their own dedicated checks below.

In other words: surfacing the raw flags directly would make the checklist
permanently red while bench-testing over USB (a false alarm baked into the
connection method itself), and the flags that actually matter for
flight-worthiness are already broken out into their own dedicated, more
specific checks elsewhere in Section A.

## Layout / scroll fixes

- Header and status banner are pinned outside the scroll area — always
  visible regardless of window height.
- Sections A and B live inside a `Canvas`-backed scrollable frame; vertical
  shrinking never hides checklist items, only scrolls them.
- Mousewheel scrolling is bound on every child widget inside the scroll
  area (not just the canvas), so scrolling works regardless of which child
  the pointer is over.
- Column layout (1-column vs. 2-column) is driven by the Canvas's own
  `<Configure>` event, adapting to available width.
- Detail-label wraplength is recalculated on every canvas resize so text
  never overflows its cell.
