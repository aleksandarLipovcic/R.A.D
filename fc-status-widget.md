# `FCStatusWidget.py` — flight controller status panel

**File:** `FCStatusWidget.py`
**Class:** `FCStatusWidget(tk.Frame)`
**Fed by:** `DroneCockpitApp._update_loop` → `update_fc_status(ui_data)`,
every tick (~50 Hz — see
[app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))

## Responsibility

Displays arm state, flight mode, battery detail, sensor presence, FC
performance metrics (CPU load, I2C errors), motor outputs, and RC channel
values — everything `telemetry_worker.py`'s `_build_ui_data()` exposes
under the FC-status/battery/motor/RC keys (see
[workers.md](workers.md#_build_ui_datastate--the-one-and-only-translation-point)).

## Priority-tiered layout

The defining design decision, called out explicitly in the source as a fix
over a previous version:

- **P1** — arm state, flight mode, battery strip, voltage bar, flight
  timer — is packed **outside** the scrollable area. It is always visible
  regardless of window height; shrinking the panel can never hide the
  battery voltage or arm state, which are the two pieces of information a
  pilot most needs in a glance.
- **P2 + P3** — battery detail, sensor-presence pills, FC metrics, motors,
  RC channels — live inside a `Canvas`-backed scrollable frame, with a
  scrollbar that appears only when content actually overflows vertically.
  Mousewheel scrolling is bound on every child widget inside the scroll
  area, not just the canvas itself, so scrolling works no matter which
  child the pointer happens to be over.

## Responsive behavior

- **Compact mode** (width < 300 px): motor and RC channel displays switch
  from bar graphs to compact dots.
- **Compact battery-strip mode** (width < 270 px): the voltage display
  further condenses into a single row to save vertical space, layered on
  top of the general compact mode above.
- The flight-mode label wraps when its text is too long for the available
  space, and the sensor-presence row wraps to multiple lines when too
  narrow to fit every pill on one line.
