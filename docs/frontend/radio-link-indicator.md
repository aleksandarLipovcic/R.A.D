# `radio_link_indicator.py` — ELRS radio-link traffic light

**File:** `radio_link_indicator.py`
**Class:** `RadioLinkIndicator(tk.Frame)` plus a pure function, `classify()`
**Lives in:** the main toolbar, right after the USB status label
**Fed by:** `DroneCockpitApp._update_link_status()` →
`update_status(worker.get_link_status())` on **every** Tk tick (~50 Hz),
even when no telemetry frames arrive (see
[app-shell.md](app-shell.md#the-tk-update-loop-_update_loop))

```
●  RADIO: OK  LQ 100%   ▶ ELRS
```

## Responsibility

Shows the pilot, at a glance, whether telemetry is coming back over the
ExpressLRS radio and which source currently feeds the instruments. It
reads a plain status dict (built by `TelemetryWorker._build_link_status()`,
see [workers.md](workers.md#two-telemetry-sources--usb-vs-elrs)) and has no
reference to `DroneBackend`.

## Colour logic — `classify(st) -> (color, headline, detail, blink)`

| Colour | Condition (`st["radio_status"]`) | Headline |
|---|---|---|
| **Grey** | `DISABLED`: this `DroneBackend.pyd` has no `CrsfLink` | `RADIO: N/A` |
| **Red** | `NO_RADIO`: Pocket not found on USB | `RADIO: NOT CONNECTED` |
| **Red** | `WAITING`: Pocket found, no telemetry from the drone | `RADIO: NO DRONE LINK` |
| **Red, blinking 1 Hz** | `TELEMETRY_LOST`: telemetry stopped | `RADIO: LINK LOST` |
| **Yellow** | `DEGRADED` (LQ < 70 % or horizon stale), **or** GPS / battery / flight-mode data missing or older than `STALE_MS` (3 s) | `RADIO: <first two problems>` |
| **Green** | `TELEMETRY_OK` and every data group is fresh | `RADIO: OK  LQ nn%` |

The `detail` text becomes the hover tooltip. For red states it explains
what to check (USB mode *USB Serial (VCP)* and *USB-VCP = Telem Mirror*,
drone powered/bound, Betaflight telemetry enabled). For green and yellow it
shows port, uplink LQ / RSSI ant. 1/2 / SNR, downlink LQ, TX power, frame
rates, reconnect count and link-loss count.

The small `▶ USB` / `▶ ELRS` / `▶ NONE` tag shows
`st["active_source"]`: the source feeding the instruments **right now**.
USB wins while the cable link is healthy. `NONE` is shown in red.

`classify()` has no Tk dependency, so it can be unit-tested headless with
hand-built dicts.

## Widget API

| Member | Purpose |
|---|---|
| `RadioLinkIndicator(master)` | Builds a dot `Canvas`, a headline `Label` and a source `Label` |
| `update_status(st)` | Runs `classify()`, applies the blink phase, and reconfigures Tk **only when the (colour, headline, source) key changed**, so the ~50 Hz calls cost almost nothing |
| `detail_text` | Latest tooltip text. The app copies it into the tooltips each tick |
| `hover_widgets()` | Child widgets to attach tooltips to. A tooltip on the `Frame` itself would disappear as soon as the pointer moved onto a child |

Main thread only, like every other widget.

## Known duplication

The yellow thresholds (`lq < 70`, attitude age `> 1500` ms) are hard-coded
here **and** in `CrsfLink` (`setDegradedLq`, `setInstrumentStaleMs`). If
one is tuned, tune the other too, or pass the thresholds through the
status dict.
