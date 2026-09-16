# `VideoLink` — analog FPV video capture & rendering

**Files:** `VideoLink.h`, `VideoLink.cpp`
**Depends on:** OpenCV (`cv::VideoCapture`, Media Foundation backend on
Windows), Win32 GDI
**Exposed to Python as:** `DroneBackend.VideoLink` (see
[bindings.md](bindings.md#videolink))

## Responsibility

`VideoLink` captures frames from a USB analog-video capture dongle,
maintains an honest connection/health state for the pilot-facing UI, and
paints frames — plus a software OSD overlay — directly into a native Win32
window, bypassing Python/Tk entirely for the live feed.

## Device discovery & connection

- `enumerateDevices()` — static, lists all Media Foundation capture devices
  with friendly names. Cheap metadata-only call; Media Foundation itself is
  started exactly once per process rather than per call, since
  `MFStartup()`/`MFShutdown()` is itself non-trivial and was previously
  being paid on every reconnect-recovery poll.
- `connectAuto()` — tries every enumerated device that doesn't look like a
  built-in laptop webcam (`looksLikeIntegratedWebcam()`), in order, until
  one opens and delivers a frame.
- `connect(deviceIndex)` — connects to a specific device.
- Reconnect logic (documented at length in the source) juggles MSMF vs.
  DSHOW backend ordering, uses a params-based `cap.open()` to avoid
  post-open property-set latency, tracks a `consecutiveFailures` staircase
  for diagnosis, and listens for Win32 device-arrival notifications for
  fast recovery (see below).

## Link-state machine

`isConnected()` alone was found insufficient for a pilot-facing UI — a
dropped device used to leave the capture loop silently retrying forever
with no externally visible change. `getLinkState()` is now the
authoritative signal:

| State | Meaning |
|---|---|
| `Disconnected` | No successful `connect()` yet, or `disconnect()` was called |
| `Searching` | `connectAuto()`/`connect()` is probing for the first frame |
| `Connected` | Frames are flowing and look like real video |
| `SignalLost` | A working connection stopped delivering frames (unplug, driver fault). Capture thread stays alive and keeps retrying — both on a timer and immediately on a Windows device-arrival event |
| `NoVideoInput` | The capture card itself is fine and still delivering frames at its normal rate, but the frames are blank/idle (analog RX has no signal, or the drone's OSD chip is drawing over a dead camera input). Detected from **frame content**, not from `cap.read()` failing |

`getMsSinceLastFrame()` is offered as a second, independent staleness check
Python can use even if a future change ever leaves the state machine wrong
— justified in the code as: for a video feed that is the pilot's eyes, it's
worth having both rather than trusting one flag.

`idleColorFraction(frame)` is the mechanism behind `NoVideoInput`: the
fraction of sampled pixels within a tolerance of the frame's own mean
color. It's deliberately cheap (sampled, not per-pixel) and returns a
continuous fraction rather than a bool specifically so the caller can apply
**hysteresis** — a stricter threshold to *enter* `NoVideoInput`, a more
lenient one required to *leave* it — preventing rapid flicker from an
ambiguous band (e.g. changing OSD telemetry digits on an otherwise blank
background).

## Native direct-to-window rendering

`attachToWindow(parentHwnd, x, y, w, h)` creates a genuine Win32 child
`HWND` under a Tk frame's HWND (`widget.winfo_id()` on the Python side) and
paints every captured frame straight into it from the capture thread — no
pybind11 marshaling, no numpy, no Tk `PhotoImage` for the live feed.
`getLatestFrame()` remains available separately for anything else that
still needs frame data in Python (e.g. recording).

Because the render window is a real foreign HWND outside Tk's widget tree,
Tk's own `lift()`/`lower()`/`tag_raise()` calls cannot touch it, and Windows
pins a newly created child window at the top of its Z-order by default.
`raiseWindow()` / `lowerWindow()` / `showWindow()` / `hideWindow()` exist
specifically so the Python side can keep this window's OS-level
stacking/visibility in sync with the corresponding Tk panel — **these must
be exposed in the pybind11 bindings**, or the sync silently becomes a no-op
and the video window gets stuck on top of everything.

An **off-screen back buffer** (`backBufferDc_`/`backBufferBmp_`) is
composited first (video + `NoVideoInput` warning box/border/text), then
copied to the visible window with one `BitBlt`. This exists because
compositing multiple separate GDI calls directly onto the visible window DC
let the screen (or a screen capture) sample mid-sequence — e.g. after the
warning box landed but before its second line of text — which is what
produced a visible flicker between one and two lines of warning text.

### Fast reconnect via OS device-arrival notifications

A dedicated hidden message-only window + thread
(`startDeviceNotifications()`/`notifyThreadMain()`) registers for Windows'
`RegisterDeviceNotification`/`WM_DEVICECHANGE` capture-device-interface
arrival events, for the entire lifetime of the `VideoLink` object (not tied
to one `connect()` session). The instant Windows reports a capture device
plugged back in, `signalDeviceArrival()` flags `captureLoop()`'s
`SignalLost` branch to retry immediately instead of waiting out its normal
poll interval — the same technique OBS and other capture software use.
`tryReconnect()` re-resolves the device index from `deviceName_` on every
attempt rather than trusting `lastDeviceIndex_`, since the OS-assigned index
for a physical device isn't guaranteed stable across a USB replug.

## Software OSD overlay

Replaces the flight controller's own analog OSD (now disabled at the FC).
The FC's OSD used to be optically burned into the analog signal *ahead of*
the capture card, so it was physically impossible to strip back out
downstream — and it was confusing the aerial-detection YOLO model with
false positives. The software overlay lives one layer higher, drawn into
the same off-screen back buffer `paintFrame()` already composites into, but
crucially **after** the frame is captured into `latestFrame_` — so it can
never leak into `getLatestFrame()` / `DetectionLink`'s input.

- Elements are independently enabled/positioned by string id (starting set:
  `"altitude"`, `"horizon"`, `"compass"` — matching what the model was
  reacting to falsely; add more by extending `kOsdElementIds` +
  `drawOsdOverlay()`).
- `OsdAnchor` — 9 presets (`TopLeft` … `BottomRight`, `Center`) plus
  `Custom` for free placement via `customFx`/`customFy` fractional
  coordinates (used when the pilot drags an element to a non-preset spot).
- **Anchor stacking**: two elements sharing an anchor (e.g. altitude +
  battery, both `BottomRight`) used to paint on top of each other.
  `resolveOsdOrigin()` now tracks a per-anchor accumulator
  (`osdStackUsedPx_`, zeroed every frame) so later elements at the same
  anchor are pushed clear of earlier ones — top-row anchors grow downward,
  bottom-row grow upward, middle-row grows downward. This only works
  because every element calls `resolveOsdOrigin` **exactly once per frame**
  in a fixed dispatch order; a hypothetical element that measures-then-places
  (calls it twice) would silently consume two stack slots and leave a gap
  — measure with a local `RECT` instead (see `layoutOsdReadout()`).
  `stack = false` opts an element out entirely; only `"horizon"` uses this,
  since it's a large ladder+sidebar backdrop meant to sit *under* the text
  readouts, not in line with them.
- `setTelemetryProvider()` — same `std::function<TelemetrySnapshot()>`
  pattern as `DetectionLink` (see [ARCHITECTURE.md](../ARCHITECTURE.md#3-the-provider-decoupling-pattern)).
  The header comment suggests wiring the *same* callable passed to
  `DetectionLink::setTelemetryProvider()`, but the actual Python wiring
  (`DroneCockpitApp`, see [FRONTEND.md](FRONTEND.md#telemetry-trampolines))
  deliberately uses **two separate trampolines** — the OSD's `valid`/
  `altitude_m` semantics differ from DetectionLink's (GPS-fix-gated vs.
  FC-link-gated; GPS altitude vs. barometer altitude), and sharing one
  breaks the OSD. See [ARCHITECTURE.md](../ARCHITECTURE.md#3-the-provider-decoupling-pattern)
  for the detail.
- Flight timer (`drawOsdTimer`): starts on first arm, **pauses (holds)** on
  disarm, resumes from where it left off on the next arm — an
  accumulated-total-plus-current-segment model rather than a single start
  timestamp, so a disarm/rearm cycle doesn't restart the clock. Only
  `resetFlightTimer()` zeroes it, via a cross-thread flag
  (`osdTimerResetRequested_`) consumed on the capture thread. All other
  timer state is touched only from `drawOsdTimer()` on the capture thread,
  so it needs no lock.

## Threading & synchronization summary

| State | Written by | Read by | Guard |
|---|---|---|---|
| `latestFrame` | capture thread | any thread (`getLatestFrame()`) | `frameMutex` |
| `linkState_`, `lastFrameTimeMs_` | capture thread | any thread | `std::atomic` |
| `renderHwnd_`, `renderHdc_` | Python/Tk thread (attach/detach) | capture thread (every paint) | `std::atomic` |
| `osdLayout_` (map) | Python/Tk thread (settings panel, drag overlay) | capture thread (`paintFrameDirect`) | `osdMutex_` (a map isn't atomic-friendly, but writes are rare — only on user interaction) |
| OSD timer state | capture thread only | capture thread only | none needed — single-thread-owned |
| `osdStackUsedPx_` | capture thread only (per-frame scratch, zeroed each frame) | capture thread only | none needed |
