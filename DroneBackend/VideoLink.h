#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <cstdint>

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
//                   retrying in the background; no Python action is
//                   needed to trigger recovery, only to reflect the
//                   state in the UI.
//   NoVideoInput -> the USB capture card itself is fine and still
//                   delivering frames at its normal rate, but the
//                   frames are the card's own idle/blank pattern
//                   rather than real video from the drone -- e.g. the
//                   analog RX has no signal, or the drone's OSD chip
//                   is overlaying text on a blank background because
//                   its camera input is dead. This is invisible to
//                   SignalLost (cap.read() keeps succeeding) and is
//                   instead detected from frame content -- see
//                   frameLooksIdle() in the .cpp. A warning banner is
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
    // metadata-only, it never opens a capture pipeline, so it's also
    // used by tryReconnect() as a fast pre-check before attempting the
    // much more expensive cap.open().
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

    void captureLoop();
    static bool looksLikeIntegratedWebcam(const std::string& name);

    // Heuristic "is this frame an idle/blank capture-card pattern rather
    // than real video" check -- see LinkState::NoVideoInput above and
    // the implementation in the .cpp for the sampling/tolerance details
    // and tuning notes. Runs once per captured frame on the capture
    // thread, so it's deliberately cheap (sampled, not per-pixel).
    static bool frameLooksIdle(const cv::Mat& frame);

    void createRenderWindow(HWND parent, int x, int y, int w, int h);
    void paintFrame(const cv::Mat& frame);

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