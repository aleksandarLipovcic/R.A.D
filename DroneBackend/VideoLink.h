#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>

// Windows native window handle for direct-to-window rendering.
// Forward-declared instead of including <windows.h> here to keep this
// header light for translation units that don't need Win32 types --
// only VideoLink.cpp needs the full Windows.h.
#ifndef HWND
struct HWND__;
typedef HWND__* HWND;
#endif

struct CaptureDeviceInfo {
    int index;
    std::string name;
};

class VideoLink {
public:
    VideoLink();
    ~VideoLink();

    // Enumerate all video capture devices visible to Windows (Media
    // Foundation), with their friendly names -- e.g. "USB Video Device",
    // "Integrated Webcam", etc. Index order matches what OpenCV's
    // CAP_MSMF backend expects for VideoCapture(index).
    static std::vector<CaptureDeviceInfo> enumerateDevices();

    // Tries every enumerated device that doesn't look like a built-in
    // laptop webcam, in order, until one opens and delivers a frame.
    bool connectAuto();
    bool connect(int deviceIndex);
    void disconnect();
    bool isConnected() const { return connected.load(); }

    cv::Mat getLatestFrame();
    uint64_t getFrameCount() const { return frameCount.load(); }
    double getMeasuredFps() const { return measuredFps.load(); }
    std::string getDeviceName() const { return deviceName_; }

    void setPreferredResolution(int w, int h) { prefW = w; prefH = h; }

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
    // _on_panel_visibility on the Python side).
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

    // renderHwnd_ is written from the Python/Tk thread (attach/detach)
    // and read from the capture thread (paintFrame) every frame, so it's
    // an atomic rather than plain HWND.
    std::atomic<HWND> renderHwnd_{ nullptr };

    // windowVisible_ mirrors the last showWindow()/hideWindow() call so
    // paintFrame() can skip the GDI blit entirely while the panel is
    // hidden, instead of continuing to draw into an invisible window.
    std::atomic<bool> windowVisible_{ true };

    void captureLoop();
    static bool looksLikeIntegratedWebcam(const std::string& name);

    void createRenderWindow(HWND parent, int x, int y, int w, int h);
    void paintFrame(const cv::Mat& frame);
};