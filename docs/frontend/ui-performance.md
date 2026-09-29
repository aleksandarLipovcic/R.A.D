# UI responsiveness — how the Tk thread is kept free

**Why this page exists:** the cockpit felt slow. Dragging or resizing panels
lagged, and old images of moved panels (black bands, stale outlines) stayed
on screen. The cause was the **Tk main thread being too busy**. Tk repaints
moved or resized areas in its *idle* phase, which only runs when no timer
is due. When the telemetry pump took about as long as its own interval,
the repaint work kept being postponed.

## Measured (full app, simulated backend, same machine)

| | Before | After |
|---|---|---|
| UI tick cost (average) | 18.8 ms | 2.9 ms |
| Real UI tick rate | ~24 Hz | 50 Hz |
| `MagWidget` update | 9.4 ms | 0.2 ms |
| GPS map update | ~31 ms (two full redraws) | 0.2 ms + one batched ~2.8 ms redraw |
| Tile-download threads started in 6 s (offline) | 366 | 0 |
| Repaint (idle) delay, 95th percentile / worst | 14 ms / 274 ms | 2–5 ms / < 10 ms |
| ADI (primary flight instrument) refresh | 17 Hz | 25 Hz |

The absolute numbers differ on the Windows laptop, but the proportions hold.

## The rules

1. **Fixed-rate pump with guaranteed idle time.**
   `DroneCockpitApp._reschedule_update()` schedules the next tick
   `UI_REFRESH_MS` (20 ms) after the *start* of the current one, never less
   than `UI_MIN_IDLE_MS` (5 ms) after its end. The old code waited 20 ms
   *after* the work, so the real rate depended on how slow the work was.
2. **Update each instrument at the rate a human can use** (`_THROTTLE`):

   | Widget | Rate |
   |---|---|
   | ADI, IMU, altitude/VSI | 25 Hz |
   | Magnetometer / Heading & Home | ~17 Hz |
   | FC status, GPS | 10 Hz |
   | Arming checklist | 5 Hz |

   Text changing faster than about 10 Hz can't be read. It only costs CPU.
3. **Redraw only what changed.**
   - `MagWidget` redraws the rose and readout only when the heading (0.1°),
     validity, link or size changed, and the raw bars only when the values
     changed.
   - `RadioNavPanel` does the same at 0.5° resolution.
   - The satellite table skips identical lists.
4. **Never create Tk font objects per update.** The heading font fit
   measures text per font size **once** (`MagWidget._hdg_text_extent`,
   also used by `RadioNavPanel`). It used to create about 12 font objects
   per update.
5. **One batched redraw per idle cycle for canvases.** The GPS map's
   `_request_redraw()` merges position, overlay, pulse, pan, zoom and
   tile-arrival requests into one redraw. The tile layer is rebuilt only
   when the view changes (size, zoom, centre, overlay height, loaded
   tiles). Otherwise only the marker and overlay items are replaced.
6. **Bounded background work.** Map tiles are downloaded by a fixed pool
   of 4 threads. A tile that failed (offline, server error) is not retried
   for 30 s. Before this, every redraw started new threads for every
   missing tile, and those threads competed with the Tk thread for
   Python's global lock (GIL).
7. **Batch pointer motion.** `DraggablePanel` records the target position
   and size on every `<B1-Motion>` and applies them at most every 33 ms
   (`_GEOM_APPLY_MS`), plus once exactly on release. Applying on every
   event re-laid-out the whole panel dozens to hundreds of times a second.

## Measuring it again

The measurements came from the full `DroneCockpitApp` running against a
simulated `DroneBackend` under a virtual display. The harness wraps each
widget's update method with a timer and probes the idle delay with
`after_idle`. When adding a widget, measure its update call. Anything above
~2 ms at its update rate is worth a "redraw only what changed" check.
