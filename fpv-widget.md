# `FPVWidget.py` — live FPV video panel (native-rendered)

**File:** `FPVWidget.py`
**Class:** `FPVWidget(tk.Frame)`
**Wraps:** `DroneBackend.VideoLink` (see
[../modules/videolink.md](../modules/videolink.md))
**Fed by:** `DroneCockpitApp._update_loop` → `update_status(connected, fps,
device_name)`, every 5th tick (~10 Hz — see
[app-shell.md](app-shell.md#the-tk-update-loop-_update_loop)); status data
itself comes from `VideoWorker` (see
[workers.md](workers.md#video_workerpy--videoworker))

## Responsibility

This widget does **not** draw the live video feed — that's the entire
point of its design. `VideoLink` paints decoded frames directly into a
native Win32 child window from its own C++ capture thread. `FPVWidget`'s
job is narrower:

1. Own a plain Tk `Frame` ("host") whose HWND (via `winfo_id()`) is handed
   to `VideoLink` once, at `attach()` time.
2. Forward the host frame's `<Configure>` events to
   `VideoLink.resize_window()` so the native surface tracks panel
   resizing/dragging.
3. Show a lightweight status/FPS bar using ordinary Tk labels — cheap (a
   couple of `label.config()` calls per tick) and, critically, this bar
   never touches per-frame pixel data, which was the entire point of
   moving display off the Python/Tk path in the first place.

There is no `update_fpv(frame, ...)` method that takes pixel data — this
widget never sees a frame, ever. It receives only periodic status via
`update_status()`.

## `attach(video_link)`

Hands the host frame's native window handle to `VideoLink` so the C++
capture thread can start painting directly into it. Safe to call right
after construction — `winfo_id()` forces the underlying platform window to
be created if it doesn't exist yet, it doesn't require the window to be
mapped/visible first. Also safe to call before a capture device is
connected: `VideoLink` simply won't paint anything until frames start
arriving.

After attaching, `attach()` also restores the last-saved OSD layout by
calling `osd_overlay_controls.load_layout()` inside a `try/except` block.
This is deliberately tolerant: a missing or corrupt `osd_layout.json`, or
one saved by an older build with different element ids, must never be able
to take the rest of the cockpit down with it. This call runs from
`DroneCockpitApp.__init__`, **after** `_setup_ui()` has already built every
instrument panel but **before** `root.mainloop()` starts — so an uncaught
exception here would abort the constructor and kill the whole app before a
single frame is ever drawn, taking down every other widget with it. The
source notes this is exactly what an earlier eager-default-argument bug in
`load_layout()` actually did in production.

## Why the status bar sits above the video, not on top of it

The status bar is a real Tk strip **above** the video area, not an overlay
drawn on top of it. The native child window `VideoLink` paints into is
opaque and is always composited above ordinary Tk canvas/label content in
the same screen region — text placed visually "on top" of the video area
would simply never be visible once `VideoLink` starts painting there.

## OSD settings entry point

The "OSD" button opens `osd_overlay_controls.OsdSettingsPanel` (see
[osd-overlay-controls.md](osd-overlay-controls.md)) against whichever
`VideoLink` is currently attached, and is disabled until `attach()` has
run. Clicking it while a panel is already open brings the existing one to
the front rather than stacking a duplicate `Toplevel`
(`_osd_panel.is_open()` check in `_open_osd_settings()`).

## Flight timer reset

The "Reset Timer" button calls `video_link.reset_flight_timer()`, gated
behind a confirmation dialog (`messagebox.askyesno`) since it discards
accumulated flight time with no undo — same reasoning as the delete-layout
confirmations elsewhere in the cockpit. The OSD timer itself
(`VideoLink::drawOsdTimer`, see
[../modules/videolink.md](../modules/videolink.md#software-osd-overlay))
starts on first arm, pauses on disarm, and resumes from where it left off
on rearm — it never resets on its own, which is why this button is the
only way back to 0:00. Also disabled until `attach()` has run, for the
same reason as the OSD button.

## `detach()`

Called on shutdown — destroys the native render window
(`video_link.detach_window()`), wrapped in a `try/except` since shutdown
ordering means the link may already be in a torn-down state by the time
this runs.
