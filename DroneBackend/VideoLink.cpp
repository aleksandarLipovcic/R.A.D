#include "VideoLink.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>

// Silences OpenCV's own internal MSMF/DSHOW logger (the
// "OnReadSample() is called with error status", "can't grab frame",
// "backend is generally available but can't be used to capture by
// index" lines). These come straight from OpenCV on every failed
// cap.read()/cap.open() call, completely independent of anything we
// print ourselves -- during a SignalLost recovery loop (tryReconnect()
// retried every kReconnectIntervalMs) that alone was enough to flood
// the console for as long as the dongle stayed disconnected, drowning
// out our own single-line state-transition logs below.
#include <opencv2/core/utils/logger.hpp>

// Media Foundation device enumeration (Windows-native device names +
// friendly indices -- this is what actually lets us tell "USB capture
// dongle" apart from "Integrated Webcam" before we ever open anything).
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")

namespace {
    // Number of consecutive failed cap.read() calls before we declare
    // the signal lost. At the loop's 5ms failure-path sleep, this is
    // roughly 75-100ms of real signal dropout before we react -- long
    // enough to ride out a single dropped USB packet without flapping
    // the UI, short enough that a genuine disconnect is caught almost
    // immediately.
    constexpr int kFailuresBeforeSignalLost = 15;

    // Minimum spacing between cap.open() retry attempts while the
    // signal is lost. Calling cap.open() in a tight loop against a
    // vanished device is wasted CPU and, on some MSMF backends, can
    // itself generate additional warning spam -- this keeps retries
    // frequent enough to feel instant to the pilot when the dongle is
    // replugged, without hammering the driver in between.
    constexpr int kReconnectIntervalMs = 400;

    // Number of consecutive good frames required at connect() time
    // before we declare the link genuinely stable and start the
    // capture thread. A single successful read is not proof the
    // stream is actually flowing -- some MSMF devices/drivers hand
    // back one valid frame from a half-initialized pipeline and then
    // immediately fail for the next several hundred ms to a few
    // seconds while D3D11 acceleration / the source reader finishes
    // spinning up. Without this check, connect() reports success,
    // the UI shows "connected", and the pilot is staring at a black
    // window while the capture thread silently churns through
    // kFailuresBeforeSignalLost failures and drops into the slower
    // SignalLost/tryReconnect() recovery path instead. Catching it
    // here means a not-actually-ready device just fails connect()
    // outright, so the (much faster) top-level connectAuto()/connect()
    // retry is what handles it instead of a multi-second SignalLost
    // stall.
    constexpr int kStableFramesRequired = 3;
    constexpr int kStableFrameSpacingMs = 15;
}

VideoLink::VideoLink() {
    static bool logLevelSet = false;
    if (!logLevelSet) {
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
        logLevelSet = true;
    }
}
VideoLink::~VideoLink() {
    disconnect();
    detachWindow();
}

// =============================================================================
// enumerateDevices()
// =============================================================================

/*static*/
std::vector<CaptureDeviceInfo> VideoLink::enumerateDevices() {
    std::vector<CaptureDeviceInfo> result;

    HRESULT hr = MFStartup(MF_VERSION);
    if (FAILED(hr)) return result;

    IMFAttributes* pAttributes = nullptr;
    if (SUCCEEDED(MFCreateAttributes(&pAttributes, 1))) {
        pAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);

        IMFActivate** ppDevices = nullptr;
        UINT32 count = 0;
        if (SUCCEEDED(MFEnumDeviceSources(pAttributes, &ppDevices, &count))) {
            for (UINT32 i = 0; i < count; ++i) {
                WCHAR* friendlyName = nullptr;
                UINT32 len = 0;
                if (SUCCEEDED(ppDevices[i]->GetAllocatedString(
                    MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &friendlyName, &len))) {
                    int needed = WideCharToMultiByte(CP_UTF8, 0, friendlyName, -1,
                        nullptr, 0, nullptr, nullptr);
                    std::string name(needed > 0 ? needed - 1 : 0, '\0');
                    if (needed > 0) {
                        WideCharToMultiByte(CP_UTF8, 0, friendlyName, -1,
                            &name[0], needed, nullptr, nullptr);
                    }
                    result.push_back({ static_cast<int>(i), name });
                    CoTaskMemFree(friendlyName);
                }
                ppDevices[i]->Release();
            }
            CoTaskMemFree(ppDevices);
        }
        pAttributes->Release();
    }

    MFShutdown();
    return result;
}

/*static*/
bool VideoLink::looksLikeIntegratedWebcam(const std::string& name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](unsigned char c) { return std::tolower(c); });

    static const char* excludePatterns[] = {
        "integrated", "built-in", "builtin", "facetime",
        "internal camera", "hd camera", "webcam"
    };
    for (auto pat : excludePatterns)
        if (lower.find(pat) != std::string::npos)
            return true;
    return false;
}

// =============================================================================
// connectAuto() / connect()
// =============================================================================

bool VideoLink::connectAuto() {
    linkState_.store(LinkState::Searching);

    auto devices = enumerateDevices();

    std::vector<CaptureDeviceInfo> candidates;
    for (auto& d : devices)
        if (!looksLikeIntegratedWebcam(d.name))
            candidates.push_back(d);

    if (candidates.empty())
        candidates = devices;

    for (auto& d : candidates) {
        std::cout << "[VideoLink] trying capture device [" << d.index << "] "
            << d.name << std::endl;
        if (connect(d.index)) {
            deviceName_ = d.name;
            std::cout << "[VideoLink] connected: " << d.name << std::endl;
            return true;
        }
    }

    linkState_.store(LinkState::Disconnected);
    return false;
}

bool VideoLink::connect(int deviceIndex) {
    disconnect();

    linkState_.store(LinkState::Searching);
    lastDeviceIndex_ = deviceIndex;

    cap.open(deviceIndex, cv::CAP_MSMF);
    if (!cap.isOpened())
        cap.open(deviceIndex, cv::CAP_DSHOW);   // fallback backend
    if (!cap.isOpened()) {
        linkState_.store(LinkState::Disconnected);
        return false;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, prefW);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, prefH);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);   // no internal queue -- always freshest frame
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // Ask for the highest frame rate we care about. Most backends/devices
    // will silently clamp this to whatever mode they actually support at
    // the requested resolution, so it's safe to just ask high -- but
    // without asking at all, several UVC dongles default to 30 fps even
    // when a 60 fps mode exists at the same resolution.
    cap.set(cv::CAP_PROP_FPS, prefFps);

    // A single successful read() here isn't sufficient proof the stream
    // is actually stable -- see kStableFramesRequired above. Require a
    // short run of consecutive good frames before declaring connect()
    // successful and starting the capture thread.
    cv::Mat testFrame;
    for (int i = 0; i < kStableFramesRequired; ++i) {
        if (!cap.read(testFrame) || testFrame.empty()) {
            cap.release();
            linkState_.store(LinkState::Disconnected);
            return false;
        }
        if (i + 1 < kStableFramesRequired)
            std::this_thread::sleep_for(std::chrono::milliseconds(kStableFrameSpacingMs));
    }

    connected.store(true);
    keepRunning.store(true);
    linkState_.store(LinkState::Connected);
    lastFrameTimeMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    captureThread = std::thread(&VideoLink::captureLoop, this);
    return true;
}

void VideoLink::disconnect() {
    keepRunning.store(false);
    if (captureThread.joinable())
        captureThread.join();
    if (cap.isOpened())
        cap.release();
    connected.store(false);
    linkState_.store(LinkState::Disconnected);
}

// =============================================================================
// Native window rendering
// =============================================================================

static LRESULT CALLBACK VideoLinkWndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    // We paint from the capture thread via a cached DC outside of
    // WM_PAINT, so this proc just needs to suppress the default
    // background erase (which would otherwise cause visible flicker
    // between frames) and defer everything else to Windows' default
    // handling.
    if (msg == WM_ERASEBKGND)
        return 1;
    return DefWindowProcW(hwnd, msg, wp, lp);
}

void VideoLink::createRenderWindow(HWND parent, int x, int y, int w, int h) {
    static bool classRegistered = false;
    static const wchar_t* clsName = L"VideoLinkRenderWnd";

    if (!classRegistered) {
        WNDCLASSW wc = {};
        wc.lpfnWndProc = VideoLinkWndProc;
        wc.hInstance = GetModuleHandleW(nullptr);
        wc.lpszClassName = clsName;
        wc.hbrBackground = nullptr;   // we own all painting
        // CS_OWNDC gives this window a private, persistent device context
        // instead of one borrowed from a shared DC cache on every
        // GetDC()/ReleaseDC() pair. That lets us acquire the DC exactly
        // once below and reuse it for every frame's StretchDIBits call,
        // instead of paying that acquire/release cost 30-60+ times a
        // second.
        wc.style = CS_OWNDC;
        RegisterClassW(&wc);
        classRegistered = true;
    }

    // WS_CLIPSIBLINGS is required so this window clips correctly against
    // its Tk-owned sibling HWNDs once Z-order between them is meaningful
    // (see raiseWindow()/lowerWindow() below). Without it, overlapping
    // regions between this window and a sibling can paint incorrectly
    // even after Z-order itself is fixed.
    HWND hwnd = CreateWindowExW(
        0, clsName, L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
        x, y, w, h, parent, nullptr, GetModuleHandleW(nullptr), nullptr);

    renderHwnd_.store(hwnd);
    windowVisible_.store(true);

    if (hwnd) {
        HDC hdc = GetDC(hwnd);
        if (hdc) {
            // COLORONCOLOR is a cheap nearest-neighbor-ish stretch filter.
            // HALFTONE (the previous mode) does per-pixel area averaging
            // for higher quality, but it's dramatically more expensive --
            // for a live low-latency feed the extra smoothing isn't worth
            // the CPU time, and skipping it is what keeps the capture
            // thread able to keep pace with the device's native frame
            // rate instead of the blit itself becoming the bottleneck.
            // COLORONCOLOR also doesn't need SetBrushOrgEx to avoid the
            // sub-pixel misalignment HALFTONE is prone to, so that call
            // is gone too. Set once here since the mode doesn't change
            // per frame.
            SetStretchBltMode(hdc, COLORONCOLOR);
        }
        renderHdc_.store(hdc);
    }
}

void VideoLink::attachToWindow(intptr_t parentHwnd, int x, int y, int w, int h) {
    detachWindow();   // clean up any previous render window first
    createRenderWindow(reinterpret_cast<HWND>(parentHwnd), x, y, w, h);
}

void VideoLink::resizeWindow(int w, int h) {
    HWND hwnd = renderHwnd_.load();
    if (hwnd)
        SetWindowPos(hwnd, nullptr, 0, 0, w, h, SWP_NOMOVE | SWP_NOZORDER);
}

void VideoLink::detachWindow() {
    HWND hwnd = renderHwnd_.exchange(nullptr);
    HDC hdc = renderHdc_.exchange(nullptr);
    if (hdc && hwnd)
        ReleaseDC(hwnd, hdc);
    if (hwnd)
        DestroyWindow(hwnd);
}

// =============================================================================
// Native window Z-order / visibility control
// =============================================================================
//
// This render window is a plain Win32 child HWND with no relationship to
// Tk's widget tree, so Tk's lift()/lower() never touch it and it stays
// wherever Windows last put it in the parent's Z-order (by default, the
// top -- newly created child windows are inserted there). These four
// calls are the explicit bridge the Python side uses to keep this
// window's actual OS-level stacking/visibility matching the Tk panel
// that hosts it.

void VideoLink::raiseWindow() {
    HWND hwnd = renderHwnd_.load();
    if (hwnd) {
        SetWindowPos(hwnd, HWND_TOP, 0, 0, 0, 0,
            SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
    }
}

void VideoLink::lowerWindow() {
    HWND hwnd = renderHwnd_.load();
    if (hwnd) {
        SetWindowPos(hwnd, HWND_BOTTOM, 0, 0, 0, 0,
            SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
    }
}

void VideoLink::showWindow() {
    HWND hwnd = renderHwnd_.load();
    windowVisible_.store(true);
    if (hwnd)
        ShowWindow(hwnd, SW_SHOW);
}

void VideoLink::hideWindow() {
    HWND hwnd = renderHwnd_.load();
    windowVisible_.store(false);
    if (hwnd)
        ShowWindow(hwnd, SW_HIDE);
}

void VideoLink::paintFrame(const cv::Mat& frame) {
    // Skip the blit entirely while the panel is hidden -- there's no
    // point spending a StretchDIBits round trip on a window Windows
    // isn't compositing anyway, and this also guarantees the window can
    // never appear to "come back" mid-frame while hidden.
    if (!windowVisible_.load())
        return;

    HWND hwnd = renderHwnd_.load();
    HDC hdc = renderHdc_.load();
    if (!hwnd || !hdc || frame.empty())
        return;

    RECT rc;
    if (!GetClientRect(hwnd, &rc))
        return;
    int destW = rc.right - rc.left;
    int destH = rc.bottom - rc.top;
    if (destW <= 0 || destH <= 0)
        return;

    // cv::Mat from cv::VideoCapture is BGR8 -- GDI's DIB_RGB_COLORS with
    // biBitCount=24 expects BGR byte order too, so no channel swap needed.
    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = frame.cols;
    bmi.bmiHeader.biHeight = -frame.rows;   // negative = top-down DIB
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 24;
    bmi.bmiHeader.biCompression = BI_RGB;

    // hdc is the window's own cached, persistent DC (CS_OWNDC, acquired
    // once in createRenderWindow) -- no GetDC()/ReleaseDC() per frame,
    // and the stretch mode was already set once when the DC was acquired.
    StretchDIBits(hdc,
        0, 0, destW, destH,
        0, 0, frame.cols, frame.rows,
        frame.data, &bmi, DIB_RGB_COLORS, SRCCOPY);
}

// =============================================================================
// Signal-loss indicator + reconnect
// =============================================================================

void VideoLink::paintNoSignalFrame() {
    // Bypasses the windowVisible_ check on purpose: if the panel is
    // hidden, showWindow()/hideWindow() (driven by Python) already
    // governs whether anything is on screen at all -- painting here
    // just makes sure that *when* it becomes visible again, it shows
    // "NO SIGNAL" rather than the last live frame.
    HWND hwnd = renderHwnd_.load();
    HDC hdc = renderHdc_.load();
    if (!hwnd || !hdc)
        return;

    RECT rc;
    if (!GetClientRect(hwnd, &rc))
        return;

    static HBRUSH noSignalBrush = CreateSolidBrush(RGB(90, 0, 0));
    FillRect(hdc, &rc, noSignalBrush);

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(255, 255, 255));
    HFONT font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    HFONT oldFont = (HFONT)SelectObject(hdc, font);

    const wchar_t* msg = L"NO SIGNAL";
    DrawTextW(hdc, msg, -1, &rc, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

    SelectObject(hdc, oldFont);
}

bool VideoLink::tryReconnect() {
    if (lastDeviceIndex_ < 0)
        return false;

    cap.release();

    bool opened = cap.open(lastDeviceIndex_, cv::CAP_MSMF);
    if (!opened)
        opened = cap.open(lastDeviceIndex_, cv::CAP_DSHOW);
    if (!opened)
        return false;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, prefW);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, prefH);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FPS, prefFps);

    cv::Mat probe;
    if (!cap.read(probe) || probe.empty()) {
        cap.release();
        return false;
    }
    return true;
}

// =============================================================================
// captureLoop()  —  runs on its own thread, decoupled entirely from Python
// =============================================================================

void VideoLink::captureLoop() {
    // Nudge the OS scheduler to favor this thread. The capture thread's
    // job is read-decode-blit in a tight loop; if it gets pre-empted for
    // long stretches by the Tk pump or other background threads, frames
    // queue up in the device/driver instead of being drained immediately,
    // which is exactly the kind of latency BUFFERSIZE=1 is meant to
    // avoid. ABOVE_NORMAL is a conservative bump that reduces scheduling
    // jitter without risking starving the rest of the app the way
    // THREAD_PRIORITY_TIME_CRITICAL could on a loaded system.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);

    auto fpsWindowStart = std::chrono::steady_clock::now();
    uint64_t framesInWindow = 0;
    int consecutiveFailures = 0;
    auto lastReconnectAttempt = std::chrono::steady_clock::time_point{};

    while (keepRunning.load()) {
        // ── Recovery mode: a prior connection dropped, cap is dead ────
        // We stay in this branch, retrying at kReconnectIntervalMs,
        // until either a new frame comes through or keepRunning goes
        // false (explicit disconnect()).
        if (linkState_.load() == LinkState::SignalLost) {
            auto now = std::chrono::steady_clock::now();
            auto sinceLastAttempt = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastReconnectAttempt).count();

            if (sinceLastAttempt < kReconnectIntervalMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            lastReconnectAttempt = now;

            if (tryReconnect()) {
                std::cout << "[VideoLink] signal recovered on device "
                    << lastDeviceIndex_ << std::endl;
                consecutiveFailures = 0;
                connected.store(true);
                linkState_.store(LinkState::Connected);
                fpsWindowStart = std::chrono::steady_clock::now();
                framesInWindow = 0;
            }
            else {
                // Still gone -- repaint the indicator in case the window
                // was covered/uncovered by another app since the last
                // attempt (this window has no WM_PAINT repaint path, so
                // nothing else will refresh it while no frames flow).
                paintNoSignalFrame();
            }
            continue;
        }

        // ── Normal streaming ──────────────────────────────────────────
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            if (consecutiveFailures == 0) {
                // Log the transition once, not on every failed grab --
                // this is the fix for the endless
                // "can't grab frame" console spam.
                std::cout << "[VideoLink] grab failures starting on device "
                    << lastDeviceIndex_ << ", monitoring..." << std::endl;
            }
            ++consecutiveFailures;

            if (consecutiveFailures >= kFailuresBeforeSignalLost) {
                std::cout << "[VideoLink] signal lost on device "
                    << lastDeviceIndex_ << " after " << consecutiveFailures
                    << " consecutive failed grabs -- entering recovery mode"
                    << std::endl;
                connected.store(false);
                linkState_.store(LinkState::SignalLost);
                cap.release();
                paintNoSignalFrame();
                lastReconnectAttempt = std::chrono::steady_clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        consecutiveFailures = 0;

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            latestFrame = frame;   // header swap -- see original comment
        }
        lastFrameTimeMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

        // Paint directly to the native window, if one is attached. This
        // runs on the capture thread itself -- the frame that was just
        // decoded is on screen within one StretchDIBits call, with no
        // Python/Tk round trip and no extra clone.
        paintFrame(frame);

        frameCount.fetch_add(1);
        ++framesInWindow;

        auto now = std::chrono::steady_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - fpsWindowStart).count();
        if (elapsedMs >= 1000) {
            measuredFps.store(framesInWindow * 1000.0 / elapsedMs);
            framesInWindow = 0;
            fpsWindowStart = now;
        }
    }
    connected.store(false);
    linkState_.store(LinkState::Disconnected);
}

cv::Mat VideoLink::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frameMutex);
    return latestFrame.clone();
}

uint64_t VideoLink::getMsSinceLastFrame() const {
    int64_t last = lastFrameTimeMs_.load();
    if (last == 0)
        return UINT64_MAX;   // never received a frame yet
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return static_cast<uint64_t>(now - last);
}