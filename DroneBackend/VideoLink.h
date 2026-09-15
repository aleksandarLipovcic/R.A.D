#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <map>
#include <array>   // std::array -- osdStackUsedPx_ (per-anchor OSD stacking accumulator)
#include <functional>
#include <cstdint>
#include <chrono>

// Forward declaration only -- the full definition (lat/lon/altitude_m/
// heading_deg/roll_deg/pitch_deg/gimbal_*/battery_*/rssi/gps_*/
// home_distance_m/ground_speed_ms) lives in DetectionLink.h. The OSD
// overlay reuses this struct verbatim instead of inventing a second
// "current telemetry" type: it's the same aircraft state, just consumed
// by a second renderer. See setTelemetryProvider() below.
struct TelemetrySnapshot;

// Windows native window/DC handles for direct-to-window rendering.
// Forward-declared instead of including <windows.h> here to keep this
// header light for translation units that don't need Win32 types --
// only VideoLink.cpp needs the full Windows.h.
#ifndef HWND
struct HWND__;
typedef HWND__* HWND;
#endif
#ifndef HDC
struct HDC__;
typedef HDC__* HDC;
#endif
#ifndef HBITMAP
struct HBITMAP__;
typedef HBITMAP__* HBITMAP;
#endif

struct CaptureDeviceInfo {
    int index;
    std::string name;
};

// Link state as seen by the capture thread. Python polls this (via
// getLinkState()) to decide whether to show the live feed or a
// "SIGNAL LOST" overlay -- see getMsSinceLastFrame() below for the
// independent fallback check.
//
//   Disconnected -> no connect() has succeeded yet, or disconnect()
//                   was called explicitly. Capture thread is not running.
//   Searching    -> connectAuto()/connect() is probing devices, trying
//                   to get the very first frame.
//   Connected    -> frames are actively flowing and look like real video.
//   SignalLost   -> a previously-working connection stopped delivering
//                   frames entirely (device unplugged, driver fault,
//                   etc). The capture thread is still alive and
//                   retrying in the background -- both on a polling
//                   timer AND immediately whenever Windows reports the
//                   capture device has been plugged back in (see
//                   signalDeviceArrival() / notifyThreadMain() in the
//                   .cpp). No Python action is needed to trigger
//                   recovery, only to reflect the state in the UI.
//   NoVideoInput -> the USB capture card itself is fine and still
//                   delivering frames at its normal rate, but the
//                   frames are the card's own idle/blank pattern
//                   rather than real video from the drone -- e.g. the
//                   analog RX has no signal, or the drone's OSD chip
//                   is overlaying text on a blank background because
//                   its camera input is dead. This is invisible to
//                   SignalLost (cap.read() keeps succeeding) and is
//                   instead detected from frame content -- see
//                   idleColorFraction() in the .cpp. A warning banner is
//                   painted directly onto the native render window
//                   while in this state (see paintFrame()).
enum class LinkState { Disconnected, Searching, Connected, SignalLost, NoVideoInput };

class VideoLink {
public:
    VideoLink();
    ~VideoLink();

    // Enumerate all video capture devices visible to Windows (Media
    // Foundation), with their friendly names -- e.g. "USB Video Device",
    // "Integrated Webcam", etc. Index order matches what OpenCV's
    // CAP_MSMF backend expects for VideoCapture(index). Cheap -- this is
    // metadata-only, it never opens a capture pipeline. Media Foundation
    // itself is started exactly once per process (see the .cpp) rather
    // than on every call, since MFStartup()/MFShutdown() is itself a
    // non-trivial cost that was previously being paid on every single
    // reconnect-recovery poll.
    static std::vector<CaptureDeviceInfo> enumerateDevices();

    // Tries every enumerated device that doesn't look like a built-in
    // laptop webcam, in order, until one opens and delivers a frame.
    bool connectAuto();
    bool connect(int deviceIndex);
    void disconnect();
    bool isConnected() const { return connected.load(); }

    // ── Signal-loss detection ────────────────────────────────────────
    //
    // isConnected() alone isn't enough for a pilot-facing UI: it used to
    // never go false when the device dropped mid-stream, because the
    // capture loop just kept retrying cap.read() forever without
    // updating any externally-visible state. getLinkState() is the
    // authoritative state; getMsSinceLastFrame() is a second,
    // independent signal Python can use as a fallback staleness check
    // even if a future change ever leaves the state machine wrong --
    // for a video feed that's a drone pilot's eyes, it's worth having
    // both rather than relying on a single flag.
    LinkState getLinkState() const { return linkState_.load(); }
    uint64_t getMsSinceLastFrame() const;

    cv::Mat getLatestFrame();
    uint64_t getFrameCount() const { return frameCount.load(); }
    double getMeasuredFps() const { return measuredFps.load(); }
    std::string getDeviceName() const { return deviceName_; }

    void setPreferredResolution(int w, int h) { prefW = w; prefH = h; }

    // Requested capture frame rate, applied via CAP_PROP_FPS on connect().
    // Left unset, most UVC capture dongles/backends silently negotiate a
    // lower default (often 30 fps) even when they support more -- this is
    // a hint, not a guarantee; the device may clamp it to whatever mode it
    // actually supports at the requested resolution.
    void setPreferredFps(double fps) { prefFps = fps; }

    // ── Native direct-to-window rendering ───────────────────────────────
    //
    // Attaches a child GDI render window under parentHwnd (a Tk frame's
    // HWND, obtained via widget.winfo_id() on the Python side) and paints
    // every captured frame straight into it from the capture thread --
    // no pybind11 marshaling, no numpy, no Tk PhotoImage for the live
    // feed. getLatestFrame() remains available and unaffected for
    // anything else (e.g. recording) that still needs frame data in
    // Python.
    void attachToWindow(intptr_t parentHwnd, int x, int y, int w, int h);
    void resizeWindow(int w, int h);
    void detachWindow();
    bool isWindowAttached() const { return renderHwnd_.load() != nullptr; }

    // ── Native window Z-order / visibility control ──────────────────────
    //
    // The render window created by attachToWindow() is a genuine Win32
    // child HWND that lives entirely outside Tk's widget tree. Tk's own
    // lift()/lower()/tag_raise() calls (and itemconfigure(state=...))
    // only ever reorder/toggle Tk's own widgets against each other --
    // they have no way to touch a foreign HWND, and Windows leaves a
    // newly created child window pinned at the top of its parent's
    // Z-order until something explicitly repositions it. Without an
    // explicit hook, that means the render window can end up permanently
    // stacked above every other panel regardless of which Tk panel the
    // user clicks to bring to front -- the live video appears to "punch
    // through" panels that never even touch pixel data.
    //
    // These four calls let the Python side keep this window's OS-level
    // stacking/visibility in sync with whatever it's doing to the
    // corresponding Tk panel (see DroneCockpitApp._on_panel_zorder /
    // _on_panel_visibility on the Python side). Make sure they're
    // exposed in the pybind11 module -- if they're missing there, the
    // sync becomes a silent no-op on the Python side and the window gets
    // stuck at its default (topmost) Z-order.
    //
    // The same window is also how a signal-loss indication reaches the
    // screen at the C++ level (see paintNoSignalFrame in the .cpp) --
    // raiseWindow()/hideWindow() remain the right tool for a Python-side
    // "SIGNAL LOST" panel drawn in Tk above this HWND, if you want a
    // second, UI-toolkit-level indicator in addition to the in-window one.
    void raiseWindow();
    void lowerWindow();
    void showWindow();
    void hideWindow();
    bool isWindowVisible() const { return windowVisible_.load(); }

    // ── Software OSD overlay ─────────────────────────────────────────
    //
    // Replaces the flight controller's own analog OSD (Betaflight, now
    // disabled at the FC) so the pilot doesn't lose the readouts: this
    // draws them into the SAME off-screen back buffer paintFrame() already
    // composites into (see backBufferDc_), on top of the video but BEFORE
    // the BitBlt to screen -- so it costs nothing extra vs. the FC-baked
    // OSD in terms of extra round trips, and it is drawn AFTER the frame
    // is captured into latestFrame_ (see captureLoop()), so it can never
    // leak into getLatestFrame()/DetectionLink's input. That separation is
    // the entire point: the FC's OSD was optically burned into the analog
    // signal ahead of the capture card, so it was physically impossible to
    // strip back out downstream and confused the aerial-detection model.
    // This overlay lives one layer higher, purely in the render path, so
    // it never touches a single pixel DetectionLink ever sees.
    //
    // Each element is independently enabled/positioned by id. The
    // starting set (see kOsdElementIds in the .cpp) is
    // "altitude" / "horizon" / "compass", matching the three things the
    // NN was picking up false positives from in the FC OSD -- add more
    // ids to kOsdElementIds + a case in drawOsdOverlay() to extend this
    // (e.g. a "battery" element pulled from a second small provider, since
    // TelemetrySnapshot itself doesn't carry battery voltage).
    enum class OsdAnchor {
        TopLeft, TopCenter, TopRight,
        MiddleLeft, Center, MiddleRight,
        BottomLeft, BottomCenter, BottomRight,
        Custom   // free-placed; see customFx/customFy below
    };

    struct OsdElementLayout {
        bool enabled = true;
        OsdAnchor anchor = OsdAnchor::TopLeft;
        // Pixel inset from whichever edge(s) the anchor names -- ignored
        // for Center and for Custom.
        int marginX = 16;
        int marginY = 16;
        // Fraction (0..1) of the panel where this element's own CENTER
        // sits, only used when anchor == Custom (i.e. the pilot dragged
        // it somewhere that didn't snap to a preset -- see
        // OsdDragOverlay.commit() on the Python side).
        float customFx = 0.5f;
        float customFy = 0.5f;
    };

    // Enable/disable and position one OSD element. id must be one of
    // kOsdElementIds (see .cpp) -- unknown ids are ignored (logged in
    // verbose mode) rather than silently creating a stray entry, so a
    // typo'd id from the Python settings panel fails visibly instead of
    // just never drawing.
    void setOsdElementEnabled(const std::string& id, bool enabled);
    void setOsdElementAnchor(const std::string& id, OsdAnchor anchor, int marginX, int marginY);
    void setOsdElementCustomPosition(const std::string& id, float fx, float fy);
    // Snapshot of one element's current layout, for the settings panel to
    // read back on open and for save/load (persistence itself is done in
    // Python -- see osd_overlay_controls.py -- this just exposes state).
    OsdElementLayout getOsdElementLayout(const std::string& id) const;
    std::vector<std::string> getOsdElementIds() const;

    // Master on/off -- flips every element at once without forgetting
    // their individual enabled flags (e.g. a single OSD panel checkbox
    // "Show OSD overlay"), and a lock that OsdDragOverlay checks before
    // allowing a drag to start, so a pilot mid-flight can't nudge an
    // element by brushing the video panel.
    void setOsdOverlayEnabled(bool enabled) { osdOverlayEnabled_.store(enabled); }
    bool isOsdOverlayEnabled() const { return osdOverlayEnabled_.load(); }
    void setOsdLocked(bool locked) { osdLocked_.store(locked); }
    bool isOsdLocked() const { return osdLocked_.load(); }

    // Same std::function-provider pattern as DetectionLink::setTelemetryProvider
    // (see Bindings.cpp) -- a plain Python callable returning a
    // TelemetrySnapshot, invoked once per painted frame on the capture
    // thread, immediately before compositing. Wire the SAME callable
    // that's already passed to DetectionLink.set_telemetry_provider() on
    // the Python side (DroneCockpitApp._get_detection_telemetry() or
    // equivalent) -- there's no reason for two separate trampolines
    // producing the same snapshot.
    void setTelemetryProvider(std::function<TelemetrySnapshot()> provider);

    // Resets the OSD flight timer (see osdTimerAccumulated_ etc. below)
    // back to 0:00. Safe to call from any thread -- it just raises
    // osdTimerResetRequested_, which drawOsdTimer() consumes on the
    // capture/paint thread that actually owns the timer state. Wire this
    // to a cockpit "reset timer" button. If the timer is currently
    // running (drone still armed) when this is called, it keeps running
    // from zero rather than stopping -- only a disarm ever stops it.
    void resetFlightTimer();

    // Called by the OS device-notification window (see notifyThreadMain
    // in the .cpp) the instant Windows reports a capture-class USB
    // device has been plugged back in. Not intended to be called from
    // Python/application code -- it just flags captureLoop()'s
    // SignalLost branch to retry immediately instead of waiting out its
    // poll interval, the same technique OBS and other capture software
    // use to get near-instant reconnects instead of relying on polling.
    void signalDeviceArrival();

private:
    cv::VideoCapture cap;
    std::thread captureThread;
    std::atomic<bool> connected{ false };
    std::atomic<bool> keepRunning{ false };
    mutable std::mutex frameMutex;
    cv::Mat latestFrame;
    std::atomic<uint64_t> frameCount{ 0 };
    std::atomic<double> measuredFps{ 0.0 };
    std::string deviceName_;
    int prefW = 720, prefH = 480;
    double prefFps = 60.0;   // requested via CAP_PROP_FPS in connect()

    // Link state machine driven entirely by captureLoop(). See LinkState
    // above for the meaning of each value.
    std::atomic<LinkState> linkState_{ LinkState::Disconnected };

    // Epoch milliseconds (steady_clock) of the last successfully decoded
    // frame. 0 means "never received a frame". Read by
    // getMsSinceLastFrame(), written only from the capture thread.
    std::atomic<int64_t> lastFrameTimeMs_{ 0 };

    // Device index used by the current/most recent connect(), so the
    // capture thread's own reconnect-on-signal-loss logic knows which
    // index to retry without any help from Python. Set in connect();
    // read only from the capture thread. tryReconnect() re-resolves
    // this from deviceName_ on every attempt, since the OS-assigned
    // index for a given physical device is not guaranteed to stay the
    // same across a USB replug.
    int lastDeviceIndex_ = -1;

    // renderHwnd_ is written from the Python/Tk thread (attach/detach)
    // and read from the capture thread (paintFrame) every frame, so it's
    // an atomic rather than plain HWND.
    std::atomic<HWND> renderHwnd_{ nullptr };

    // renderHdc_ is a single DC acquired once (via GetDC, right after the
    // window is created with CS_OWNDC) and reused for every subsequent
    // StretchDIBits call, instead of paying a GetDC/ReleaseDC round trip
    // per frame. CS_OWNDC guarantees this DC stays valid and exclusively
    // ours for the lifetime of the window, so caching it is safe. It is
    // released exactly once, in detachWindow(), before the window itself
    // is destroyed.
    std::atomic<HDC> renderHdc_{ nullptr };

    // windowVisible_ mirrors the last showWindow()/hideWindow() call so
    // paintFrame() can skip the GDI blit entirely while the panel is
    // hidden, instead of continuing to draw into an invisible window.
    std::atomic<bool> windowVisible_{ true };

    // ── Off-screen back buffer ──────────────────────────────────────
    //
    // paintFrame() composites the background video frame plus the
    // NoVideoInput warning box/border/text into this off-screen memory
    // DC first, then copies the finished result to the visible window
    // with a single BitBlt. Without this, those were several separate
    // GDI calls straight onto the visible window DC with nothing
    // keeping them atomic with respect to the monitor's refresh -- the
    // screen (or a screen capture) could sample the window mid-sequence,
    // e.g. after the box/border landed but before the second line of
    // text had been emitted, which is what produced the box appearing
    // to flicker between showing one line and two. Created lazily
    // in paintFrame() (capture thread only) and recreated whenever the
    // window's size changes; torn down in detachWindow().
    std::atomic<HDC> backBufferDc_{ nullptr };
    std::atomic<HBITMAP> backBufferBmp_{ nullptr };
    std::atomic<int> backBufferW_{ 0 };
    std::atomic<int> backBufferH_{ 0 };

    // ── OS device-arrival notification ──────────────────────────────
    //
    // A dedicated hidden message-only window + thread that registers
    // for Windows' capture-device-interface arrival notifications
    // (RegisterDeviceNotification / WM_DEVICECHANGE), running for the
    // entire lifetime of the VideoLink object -- not tied to any one
    // connect() session -- so a replug is caught the instant Windows
    // reports it, whether or not we happen to be mid-recovery at that
    // moment. See notifyThreadMain() in the .cpp for the implementation;
    // it's kept out of this header entirely to avoid pulling Win32
    // window-message types into a header other translation units include.
    std::thread notifyThread_;
    std::atomic<bool> deviceArrivalSignal_{ false };
    std::atomic<bool> notifyThreadReady_{ false };
    std::atomic<unsigned long> notifyThreadId_{ 0 };

    void startDeviceNotifications();
    void stopDeviceNotifications();
    void notifyThreadMain();

    void captureLoop();
    static bool looksLikeIntegratedWebcam(const std::string& name);

    // Fraction (0.0-1.0) of sampled frame pixels that fall within
    // kIdleColorMatchTolerance of the frame's own mean color -- a rough
    // "how uniform/blank does this frame look" measure. See
    // LinkState::NoVideoInput above. Runs once per captured frame on the
    // capture thread, so it's deliberately cheap (sampled, not
    // per-pixel).
    //
    // Returns a continuous fraction rather than a bool specifically so
    // captureLoop() can apply two different thresholds (stricter to
    // enter NoVideoInput, more lenient required to leave it) -- that
    // hysteresis is what stops a frame landing in the ambiguous band
    // between them (e.g. from OSD telemetry digits changing on an
    // otherwise-blank background) from flipping the warning on and off
    // rapidly.
    static double idleColorFraction(const cv::Mat& frame);

    // ── Software OSD overlay state ───────────────────────────────────
    //
    // Written from the Python/Tk thread (settings panel + drag overlay
    // callbacks), read from the capture thread inside paintFrameDirect --
    // same cross-thread shape as renderHwnd_/renderHdc_ above, but a
    // std::map isn't atomic-friendly, so this one small mutex guards the
    // whole layout table instead. Contention is a non-issue: writes only
    // happen on user interaction (a checkbox click, a drag release), not
    // per-frame.
    mutable std::mutex osdMutex_;
    std::map<std::string, OsdElementLayout> osdLayout_;
    std::atomic<bool> osdOverlayEnabled_{ true };
    std::atomic<bool> osdLocked_{ false };
    std::function<TelemetrySnapshot()> telemetryProvider_;

    // Resolves one element's layout to a top-left pixel origin for a box
    // of size (elemW, elemH) inside a destW x destH panel. Shared by
    // every element's draw* helper below so anchor math lives in exactly
    // one place.
    //
    // ANCHOR STACKING. Two elements sharing one of the 9 presets used to
    // resolve to the identical origin and paint directly on top of each
    // other -- "altitude" and "battery" both on BottomRight rendered as
    // an unreadable smear. They now STACK instead: the first element
    // resolved for a given anchor sits exactly where it always did, and
    // each subsequent one is pushed clear of the ones before it. Top-row
    // anchors grow downward, bottom-row anchors grow upward, and the
    // middle row grows downward (so the first element stays vertically
    // centered and later ones hang below it).
    //
    // This is done here, rather than in each draw* helper, because of a
    // property that holds today and MUST keep holding: every element
    // calls resolveOsdOrigin EXACTLY ONCE per frame, in the fixed order
    // drawOsdOverlay() dispatches them (the kOsdElementIds order). That
    // makes this function the single point where "how much of this
    // anchor is already spoken for" can be both read and advanced, and
    // it is why no draw* helper needed changing to get stacking. The
    // flip side: a future element that calls this twice for one frame
    // (e.g. to measure, then to place) would silently consume two stack
    // slots and leave a gap. Measure with a local RECT instead, the way
    // layoutOsdReadout() does.
    //
    // `stack` opts an element out of that bookkeeping entirely -- it
    // neither offsets nor consumes a slot. Only "horizon" passes false:
    // it is a large square ladder+sidebar backdrop rather than a line of
    // text, so queueing a compass readout below it would push the
    // compass hundreds of pixels off its anchor. The horizon is meant to
    // be drawn under the readouts, not in line with them.
    void resolveOsdOrigin(const OsdElementLayout& layout, int destW, int destH,
        int elemW, int elemH, int& outX, int& outY, bool stack = true);

    // Shared box-sizing + origin resolution for every simple single-line
    // OSD readout (battery/RSSI/GPS/timer/home-distance) -- measures
    // `text` with DT_CALCRECT and resolves its anchor via
    // resolveOsdOrigin. Out-params (not a returned RECT) to match
    // resolveOsdOrigin's own style and because this header deliberately
    // never includes <windows.h> / RECT. Same shape drawOsdAltitude/
    // Compass already used inline; pulled out once the new elements all
    // needed the identical few lines.
    //
    // No longer const: it forwards to resolveOsdOrigin, which now
    // advances the per-anchor stacking accumulator (see above).
    void layoutOsdReadout(HDC hdc, const wchar_t* text, const OsdElementLayout& layout,
        int destW, int destH, int& outX, int& outY, int& outW, int& outH);

    // The actual per-element GDI drawing, run from paintFrameDirect after
    // the video blit. Takes the telemetry snapshot already fetched once
    // for this frame (not re-fetched per element) and destW/destH of the
    // panel being painted into.
    void drawOsdOverlay(HDC targetHdc, int destW, int destH);
    void drawOsdAltitude(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdHorizon(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdCompass(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdBattery(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdRssi(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdGps(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdTimer(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);
    void drawOsdHomeDistance(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
        const TelemetrySnapshot& t);

    // Flight timer state. Semantics: starts on the drone's first arm,
    // pauses (holds) on disarm, resumes from where it left off on the
    // next arm, and is only ever zeroed by an explicit resetFlightTimer()
    // call (cockpit button). This is deliberately an
    // accumulated-total-plus-current-segment model rather than a single
    // start timestamp, so a disarm/rearm cycle doesn't restart the clock.
    //
    // osdTimerAccumulated_/osdTimerRunning_/osdTimerSegmentStart_/
    // osdTimerPrevArmed_ are touched only from drawOsdTimer(), which only
    // ever runs on VideoLink's own capture/paint thread, so they need no
    // lock/atomic despite being read+written across frames -- same
    // reasoning the old single-timestamp version relied on.
    std::chrono::steady_clock::duration osdTimerAccumulated_{ 0 }; // total time from all completed armed segments
    bool osdTimerRunning_ = false;             // true while the current armed segment is counting
    std::chrono::steady_clock::time_point osdTimerSegmentStart_;  // start of the current armed segment
    bool osdTimerPrevArmed_ = false;           // t.armed as of the previous drawOsdTimer() call, for edge detection

    // The one piece of timer state that DOES cross threads: set from
    // resetFlightTimer() (called from the Tk/UI thread on a button
    // click), consumed and cleared inside drawOsdTimer() on the capture
    // thread. Everything above stays single-threaded; this flag is the
    // only hand-off.
    std::atomic<bool> osdTimerResetRequested_{ false };

    // Per-anchor stacking accumulator: how many vertical pixels each of
    // the 9 presets has already handed out THIS frame. Indexed by
    // static_cast<int>(OsdAnchor); the Custom slot exists only to keep
    // the indexing trivial and is never read (free-placed elements are
    // exactly the ones the pilot positioned by hand, so second-guessing
    // them with an auto-offset would fight the drag they just did).
    //
    // Needs no lock or atomic for the same reason the timer state above
    // doesn't: it is written and read only inside resolveOsdOrigin(),
    // which runs only from the draw* helpers, which run only from
    // drawOsdOverlay(), which runs only from paintFrameDirect() on
    // VideoLink's own capture/paint thread. It is zeroed at the top of
    // every drawOsdOverlay() call -- it is per-frame scratch, not
    // persistent layout state, so nothing carries over between frames
    // and a toggled-off element frees its slot on the very next paint.
    std::array<int, 10> osdStackUsedPx_{};

    void createRenderWindow(HWND parent, int x, int y, int w, int h);
    void paintFrame(const cv::Mat& frame);

    // The actual background-blit + NoVideoInput-banner drawing, factored
    // out so it can target either the off-screen back buffer (the
    // normal path) or the window DC directly (fallback if the back
    // buffer ever fails to allocate). See backBufferDc_ above for why
    // this is composited off-screen rather than drawn straight to the
    // window.
    void paintFrameDirect(HDC targetHdc, int destW, int destH, const cv::Mat& frame);

    // Paints a solid indicator + "NO SIGNAL" text directly into the
    // render window from the capture thread, at the moment linkState_
    // transitions to SignalLost (and again on each reconnect attempt,
    // in case the window got covered/uncovered by another app in the
    // meantime -- this window doesn't implement WM_PAINT repainting, so
    // nothing else will refresh it while no frames are flowing).
    void paintNoSignalFrame();

    // Attempts to re-open the current capture device and read one frame.
    // Returns true and leaves cap opened/streaming on success. Called
    // from captureLoop() while linkState_ == SignalLost. Re-resolves the
    // device index from deviceName_ via enumerateDevices() on every call
    // -- see lastDeviceIndex_ and the .cpp for why.
    bool tryReconnect();
};