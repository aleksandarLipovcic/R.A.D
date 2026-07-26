#include "VideoLink.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <mutex>

// Silences OpenCV's own internal MSMF/DSHOW logger (the
// "OnReadSample() is called with error status", "can't grab frame",
// "backend is generally available but can't be used to capture by
// index" lines). These come straight from OpenCV on every failed
// cap.read()/cap.open() call, completely independent of anything we
// print ourselves -- during recovery it alone was enough to flood the
// console for as long as the dongle stayed disconnected, drowning out
// our own single-line state-transition logs below.
#include <opencv2/core/utils/logger.hpp>

// Media Foundation device enumeration (Windows-native device names +
// friendly indices -- this is what actually lets us tell "USB capture
// dongle" apart from "Integrated Webcam" before we ever open anything).
#include <windows.h>
#include <dbt.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "user32.lib")

namespace {
    // Number of consecutive failed cap.read() calls before we declare
    // the signal lost. At the loop's 5ms failure-path sleep, this is
    // roughly 75-100ms of real signal dropout before we react -- long
    // enough to ride out a single dropped USB packet without flapping
    // the UI, short enough that a genuine disconnect is caught almost
    // immediately.
    constexpr int kFailuresBeforeSignalLost = 15;

    // Fallback polling interval while SignalLost, used only when the
    // OS device-arrival notification doesn't fire (e.g. some other
    // process ate the notification, or the specific hardware doesn't
    // surface a clean arrival event for some reason). With the arrival
    // notification in place this is a safety net, not the primary
    // recovery path, so it no longer needs to be aggressive.
    constexpr int kReconnectIntervalMs = 300;

    // ── "No drone video" idle-frame detection ──────────────────────────
    //
    // How much of a frame must be close to that frame's own mean color
    // for it to be treated as an idle/blank capture-card pattern rather
    // than real video. This is a dominant-color-fraction test rather
    // than raw stddev specifically so it still works when the flight
    // controller's OSD chip is overlaying text on a blank background
    // (no camera signal) -- the OSD text only covers a small fraction
    // of pixels, so the frame is still mostly one color even though it
    // isn't perfectly flat.
    //
    // Two thresholds, not one, deliberately -- a single threshold meant
    // a frame whose fraction hovered right around it (e.g. OSD telemetry
    // digits changing frame to frame on the idle card's background)
    // could flip NoVideoInput on and off every few frames, which is
    // exactly what a flickering warning box looks like. With hysteresis,
    // entering NoVideoInput requires clearly-idle frames (>= enter) and
    // leaving it requires clearly-NOT-idle frames (< exit); anything
    // in between the two doesn't count as progress in either direction,
    // so a borderline frame can't undo an in-progress transition.
    //
    // NOT calibrated against real hardware output -- tune these against
    // your actual capture card's idle pattern. Temporarily log the
    // computed fraction from idleColorFraction() to see real numbers,
    // then adjust so real video reliably stays well below kIdleExitFraction
    // and the idle screen reliably stays well above kIdleEnterFraction.
    constexpr double kIdleEnterFraction = 0.85;
    constexpr double kIdleExitFraction = 0.65;
    // Per-channel distance (0-255) from the frame's mean color within
    // which a sampled pixel still counts as "matching" the dominant
    // color for the fraction above.
    constexpr int kIdleColorMatchTolerance = 14;
    // Sample every Nth pixel in each dimension instead of every pixel --
    // this runs once per captured frame on the capture thread, so it
    // needs to stay cheap. Stride 6 on a 720x480 frame is ~9600 samples,
    // well under a millisecond.
    constexpr int kIdleSampleStride = 6;
    // Consecutive idle/live frames required before flipping state, to
    // avoid flapping the UI on a single unlucky frame (e.g. a genuinely
    // dark real scene, or one corrupted frame). Leaving NoVideoInput
    // requires a notably longer sustained run than entering it -- a
    // pilot briefly seeing a stale "no video" banner linger for an
    // extra fraction of a second after real video resumes is a much
    // smaller problem than the banner flickering in and out.
    constexpr int kIdleFramesToConfirm = 15;  // ~250ms at 60fps
    constexpr int kLiveFramesToConfirm = 25;  // ~400ms at 60fps
    // Minimum time between two consecutive Connected<->NoVideoInput
    // flips, as a second, time-based (not just frame-count-based) guard
    // against rapid toggling.
    constexpr int kStateChangeCooldownMs = 500;

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
    // kFailuresBeforeSignalLost failures and drops into recovery
    // instead. Catching it here means a not-actually-ready device
    // just fails connect() outright, so the (much faster) top-level
    // connectAuto()/connect() retry handles it instead of a
    // multi-second SignalLost stall.
    constexpr int kStableFramesRequired = 3;
    constexpr int kStableFrameSpacingMs = 15;

    // Device-interface class GUID for video capture devices
    // (KSCATEGORY_VIDEO_INPUT_DEVICE). Defined here as a raw GUID
    // literal instead of pulling in ks.h/ksmedia.h (and the
    // INITGUID/DEFINE_GUID machinery that comes with it), purely to
    // avoid any risk of colliding with a differently-configured
    // definition elsewhere in the app. This is what we register for
    // via RegisterDeviceNotification so the OS tells us immediately
    // when a capture-class USB device is plugged back in, instead of
    // us having to poll for it -- the same mechanism OBS and other
    // capture software use for near-instant reconnects.
    constexpr GUID kVideoInputDeviceGuid = {
        0xa799a800, 0xa46d, 0x11d0, { 0xa1, 0x8c, 0x00, 0xa0, 0x24, 0x02, 0xdd, 0xff }
    };
}

// =============================================================================
// Device-arrival notification window (free function, anonymous namespace)
// =============================================================================
//
// Deliberately NOT a VideoLink member function -- WNDPROC has a fixed
// Win32 callback signature, and keeping it as a free function here
// means VideoLink.h never has to know about HWND/UINT/WPARAM/LPARAM at
// all. It reaches back into the owning VideoLink instance (stashed in
// GWLP_USERDATA right after the window is created) only through the
// public, trivial signalDeviceArrival() setter.
namespace {
    LRESULT CALLBACK DeviceNotifyWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
        if (msg == WM_DEVICECHANGE) {
            if (wParam == DBT_DEVICEARRIVAL) {
                auto* self = reinterpret_cast<VideoLink*>(
                    GetWindowLongPtrW(hwnd, GWLP_USERDATA));
                if (self)
                    self->signalDeviceArrival();
            }
            return TRUE;
        }
        return DefWindowProcW(hwnd, msg, wParam, lParam);
    }
}

VideoLink::VideoLink() {
    static bool logLevelSet = false;
    if (!logLevelSet) {
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
        logLevelSet = true;
    }
    startDeviceNotifications();
}
VideoLink::~VideoLink() {
    stopDeviceNotifications();
    disconnect();
    detachWindow();
}

void VideoLink::signalDeviceArrival() {
    deviceArrivalSignal_.store(true);
}

// =============================================================================
// Device-arrival notification thread
// =============================================================================

void VideoLink::startDeviceNotifications() {
    if (notifyThread_.joinable())
        return;   // already running

    notifyThreadReady_.store(false);
    notifyThread_ = std::thread(&VideoLink::notifyThreadMain, this);

    // Wait briefly for the notification window/registration to actually
    // be up before returning, so a connect() called immediately after
    // construction can't race the notification system coming online.
    for (int i = 0; i < 100 && !notifyThreadReady_.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void VideoLink::stopDeviceNotifications() {
    if (!notifyThread_.joinable())
        return;

    unsigned long tid = notifyThreadId_.load();
    if (tid != 0)
        PostThreadMessageW(tid, WM_QUIT, 0, 0);
    notifyThread_.join();
}

void VideoLink::notifyThreadMain() {
    // Message-only window (HWND_MESSAGE parent): never visible, never
    // needs to pump paint/activation messages, but still receives
    // WM_DEVICECHANGE just fine since RegisterDeviceNotification
    // delivers directly to the HWND we register -- it's a targeted
    // message, not a broadcast that requires a "real" top-level window.
    static bool classRegistered = false;
    static const wchar_t* clsName = L"VideoLinkDeviceNotifyWnd";
    if (!classRegistered) {
        WNDCLASSW wc = {};
        wc.lpfnWndProc = DeviceNotifyWndProc;
        wc.hInstance = GetModuleHandleW(nullptr);
        wc.lpszClassName = clsName;
        RegisterClassW(&wc);
        classRegistered = true;
    }

    HWND hwnd = CreateWindowExW(
        0, clsName, L"", 0, 0, 0, 0, 0,
        HWND_MESSAGE, nullptr, GetModuleHandleW(nullptr), nullptr);

    if (!hwnd) {
        // Couldn't create the notification window -- fall back silently
        // to poll-only recovery (captureLoop()'s kReconnectIntervalMs
        // timer still works regardless). Still have to flag ready so
        // stopDeviceNotifications() doesn't spin waiting forever.
        notifyThreadReady_.store(true);
        return;
    }

    SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

    DEV_BROADCAST_DEVICEINTERFACE_W filter = {};
    filter.dbcc_size = sizeof(filter);
    filter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    filter.dbcc_classguid = kVideoInputDeviceGuid;

    HDEVNOTIFY hDevNotify = RegisterDeviceNotificationW(
        hwnd, &filter, DEVICE_NOTIFY_WINDOW_HANDLE);
    // hDevNotify may come back null on failure -- if so we still run the
    // message loop (harmless) and just never get WM_DEVICECHANGE, so
    // recovery quietly falls back to polling only.

    notifyThreadId_.store(GetCurrentThreadId());
    notifyThreadReady_.store(true);

    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    if (hDevNotify)
        UnregisterDeviceNotification(hDevNotify);
    DestroyWindow(hwnd);
}

// =============================================================================
// enumerateDevices()
// =============================================================================

/*static*/
std::vector<CaptureDeviceInfo> VideoLink::enumerateDevices() {
    std::vector<CaptureDeviceInfo> result;

    // Media Foundation is started exactly once for the process lifetime
    // instead of on every call. MFStartup()/MFShutdown() is itself a
    // non-trivial cost (well beyond just opening a capture pipeline),
    // and this used to run on EVERY enumerateDevices() call -- including
    // every single reconnect-recovery poll -- which was a large part of
    // why recovery could take many seconds even with a short retry
    // interval. Never calling MFShutdown() leaks nothing meaningful
    // (it's released at process exit); this mirrors how OBS and other
    // long-running capture apps keep MF initialized for their whole
    // lifetime rather than repeatedly tearing it down.
    static std::once_flag mfInitFlag;
    std::call_once(mfInitFlag, []() { MFStartup(MF_VERSION); });

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

/*static*/
double VideoLink::idleColorFraction(const cv::Mat& frame) {
    if (frame.empty() || frame.channels() != 3)
        return 0.0;

    cv::Scalar meanVal = cv::mean(frame);
    const int mB = static_cast<int>(meanVal[0]);
    const int mG = static_cast<int>(meanVal[1]);
    const int mR = static_cast<int>(meanVal[2]);

    long matched = 0, sampled = 0;
    for (int y = 0; y < frame.rows; y += kIdleSampleStride) {
        const uint8_t* row = frame.ptr<uint8_t>(y);
        for (int x = 0; x < frame.cols; x += kIdleSampleStride) {
            const uint8_t* px = row + static_cast<size_t>(x) * 3;
            if (std::abs(static_cast<int>(px[0]) - mB) <= kIdleColorMatchTolerance &&
                std::abs(static_cast<int>(px[1]) - mG) <= kIdleColorMatchTolerance &&
                std::abs(static_cast<int>(px[2]) - mR) <= kIdleColorMatchTolerance) {
                ++matched;
            }
            ++sampled;
        }
    }
    if (sampled == 0)
        return 0.0;

    // Uncomment while tuning against real hardware:
    // std::cout << "[VideoLink] idle-fraction=" << (double)matched / sampled << std::endl;

    return static_cast<double>(matched) / sampled;
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

    // Cache the friendly name for this device index so tryReconnect()
    // can re-locate it by name after a replug (the index isn't
    // guaranteed to stay the same). connectAuto() already sets this
    // after connect() returns, but connect() can also be called
    // directly (e.g. from a device-picker UI), so resolve it here too
    // rather than leaving deviceName_ stale/empty in that path.
    for (auto& d : enumerateDevices()) {
        if (d.index == deviceIndex) {
            deviceName_ = d.name;
            break;
        }
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

    HDC backDc = backBufferDc_.exchange(nullptr);
    HBITMAP backBmp = backBufferBmp_.exchange(nullptr);
    if (backBmp)
        DeleteObject(backBmp);
    if (backDc)
        DeleteDC(backDc);
    backBufferW_.store(0);
    backBufferH_.store(0);
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

    // ── Off-screen back buffer ───────────────────────────────────────
    // Composite background + banner into an off-screen memory DC first,
    // then copy the finished result to the screen with one BitBlt --
    // see backBufferDc_ in the header for why this matters (fixes the
    // "box appears to flicker between one line and two lines" symptom,
    // which was actually the screen sampling a partially-drawn window,
    // not the warning state itself flapping).
    HDC backDc = backBufferDc_.load();
    HBITMAP backBmp = backBufferBmp_.load();
    if (!backDc || !backBmp || backBufferW_.load() != destW || backBufferH_.load() != destH) {
        if (backBmp) {
            DeleteObject(backBmp);
            backBmp = nullptr;
        }
        if (backDc) {
            DeleteDC(backDc);
            backDc = nullptr;
        }

        backDc = CreateCompatibleDC(hdc);
        if (backDc) {
            backBmp = CreateCompatibleBitmap(hdc, destW, destH);
            if (backBmp) {
                SelectObject(backDc, backBmp);
                SetStretchBltMode(backDc, COLORONCOLOR);
            }
            else {
                DeleteDC(backDc);
                backDc = nullptr;
            }
        }

        backBufferDc_.store(backDc);
        backBufferBmp_.store(backBmp);
        backBufferW_.store(destW);
        backBufferH_.store(destH);
    }

    if (!backDc || !backBmp) {
        // Off-screen buffer failed to allocate (should be rare) --
        // fall back to drawing straight to the window DC rather than
        // dropping the frame. Tearing risk returns in this fallback
        // case only.
        paintFrameDirect(hdc, destW, destH, frame);
        return;
    }

    paintFrameDirect(backDc, destW, destH, frame);
    BitBlt(hdc, 0, 0, destW, destH, backDc, 0, 0, SRCCOPY);
}

void VideoLink::paintFrameDirect(HDC targetHdc, int destW, int destH, const cv::Mat& frame) {
    // cv::Mat from cv::VideoCapture is BGR8 -- GDI's DIB_RGB_COLORS with
    // biBitCount=24 expects BGR byte order too, so no channel swap needed.
    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = frame.cols;
    bmi.bmiHeader.biHeight = -frame.rows;   // negative = top-down DIB
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 24;
    bmi.bmiHeader.biCompression = BI_RGB;

    StretchDIBits(targetHdc,
        0, 0, destW, destH,
        0, 0, frame.cols, frame.rows,
        frame.data, &bmi, DIB_RGB_COLORS, SRCCOPY);

    // ── "No drone video" warning banner ─────────────────────────────
    // The box is sized FROM the actual measured text extent (via
    // DT_CALCRECT), not the other way around -- previously the box size
    // was picked first (as a fraction of the panel) and the font size
    // derived from that, which meant at some panel sizes the two-line
    // message needed more vertical room than the box had, and
    // DrawTextW's default rect-clipping silently cut the second line off
    // entirely. Measuring first guarantees the box is always big enough
    // to hold the whole message.
    if (linkState_.load() == LinkState::NoVideoInput) {
        SetBkMode(targetHdc, TRANSPARENT);
        SetTextColor(targetHdc, RGB(255, 255, 255));

        // Font size is scaled off the panel height directly (not off a
        // box height that hasn't been decided yet), clamped to a
        // sensible legible range. Cached and only rebuilt when it
        // actually changes (e.g. panel resize) to avoid GDI object
        // churn every frame at 60fps.
        static HFONT cachedFont = nullptr;
        static int cachedFontHeight = 0;
        int fontHeight = -((std::max)(16, (std::min)(34, destH / 16)));
        if (!cachedFont || cachedFontHeight != fontHeight) {
            if (cachedFont)
                DeleteObject(cachedFont);
            cachedFont = CreateFontW(
                fontHeight, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
            cachedFontHeight = fontHeight;
        }
        HGDIOBJ oldFont = SelectObject(targetHdc, cachedFont);

        const wchar_t* msg = L"NO DRONE VIDEO\nCHECK VTX / ANTENNA / CAMERA";
        const int paddingX = 26, paddingY = 20;

        // Cap the wrap width at 72% of the panel (matches the previous
        // visual proportions) so the box doesn't stretch edge-to-edge
        // on wide panels, then measure the real height that text needs
        // at that width and this font.
        int wrapWidth = (std::max)(static_cast<int>(destW * 0.72) - paddingX * 2, 160);
        RECT calcRect = { 0, 0, wrapWidth, 0 };
        DrawTextW(targetHdc, msg, -1, &calcRect, DT_CENTER | DT_WORDBREAK | DT_CALCRECT);
        int textW = calcRect.right - calcRect.left;
        int textH = calcRect.bottom - calcRect.top;

        int boxW = (std::min)(textW + paddingX * 2, destW - 20);
        int boxH = (std::min)(textH + paddingY * 2, destH - 20);
        boxW = (std::max)(boxW, 160);
        boxH = (std::max)(boxH, 50);

        RECT box;
        box.left = (destW - boxW) / 2;
        box.right = box.left + boxW;
        box.top = (destH - boxH) / 2;
        box.bottom = box.top + boxH;

        static HBRUSH warnBrush = CreateSolidBrush(RGB(190, 95, 0));
        FillRect(targetHdc, &box, warnBrush);

        // Border makes the box read as a distinct overlay rather than
        // blending into whatever idle background is behind it.
        HGDIOBJ oldBrush = SelectObject(targetHdc, GetStockObject(NULL_BRUSH));
        HPEN borderPen = CreatePen(PS_SOLID, 3, RGB(255, 200, 120));
        HGDIOBJ oldPen = SelectObject(targetHdc, borderPen);
        Rectangle(targetHdc, box.left, box.top, box.right, box.bottom);
        SelectObject(targetHdc, oldPen);
        SelectObject(targetHdc, oldBrush);
        DeleteObject(borderPen);

        RECT textRect;
        textRect.left = box.left + (boxW - textW) / 2;
        textRect.right = textRect.left + textW;
        textRect.top = box.top + (boxH - textH) / 2;
        textRect.bottom = textRect.top + textH;
        // DT_NOCLIP as a belt-and-suspenders safety net: even if the
        // measured size is ever off by a pixel or two (font hinting,
        // DPI rounding), the message stays fully visible instead of
        // silently losing a line the way the old fixed-box version did.
        DrawTextW(targetHdc, msg, -1, &textRect, DT_CENTER | DT_WORDBREAK | DT_NOCLIP);

        SelectObject(targetHdc, oldFont);
    }
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

    // Cheaply confirm the device is actually enumerated by Windows right
    // now, and re-resolve its current index by friendly name, BEFORE
    // ever calling the expensive cap.open(). enumerateDevices() is pure
    // Media Foundation metadata -- it never opens a capture pipeline --
    // and with MFStartup() now only paid once per process (see
    // enumerateDevices()) this is genuinely cheap on every retry.
    //
    // Also transparently handles the device's index changing on replug,
    // which a fixed lastDeviceIndex_ retry would otherwise silently fail
    // against forever.
    if (!deviceName_.empty()) {
        int resolvedIndex = -1;
        for (auto& d : enumerateDevices()) {
            if (d.name == deviceName_) {
                resolvedIndex = d.index;
                break;
            }
        }
        if (resolvedIndex < 0)
            return false;   // not currently present -- don't attempt open()
        lastDeviceIndex_ = resolvedIndex;
    }

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
    int idleStreak = 0;
    int liveStreak = 0;
    // Deliberately default-constructed (== epoch, far in the past) so
    // the very first NoVideoInput transition isn't blocked by the
    // cooldown -- see kStateChangeCooldownMs.
    auto lastNoVideoStateChange = std::chrono::steady_clock::time_point{};

    while (keepRunning.load()) {
        // ── Recovery mode: a prior connection dropped, cap is dead ────
        // We retry either when the poll interval elapses, OR the instant
        // deviceArrivalSignal_ is set by the OS notification thread --
        // whichever comes first. The polling interval is now purely a
        // fallback safety net; the notification is what gets real-world
        // recovery time down from "up to several seconds of polling
        // against a not-yet-open()able device" to "as soon as Windows
        // says the device exists again", matching OBS-style near-instant
        // reconnects.
        if (linkState_.load() == LinkState::SignalLost) {
            bool arrivalSignaled = deviceArrivalSignal_.exchange(false);
            auto now = std::chrono::steady_clock::now();
            auto sinceLastAttempt = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastReconnectAttempt).count();

            if (!arrivalSignaled && sinceLastAttempt < kReconnectIntervalMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            lastReconnectAttempt = now;

            if (tryReconnect()) {
                std::cout << "[VideoLink] signal recovered on device "
                    << lastDeviceIndex_ << std::endl;
                consecutiveFailures = 0;
                idleStreak = 0;
                liveStreak = 0;
                lastNoVideoStateChange = std::chrono::steady_clock::now();
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
                deviceArrivalSignal_.store(false);   // discard any stale signal
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

        // ── "No drone video" detection ──────────────────────────────
        // The USB link is fine (we just read a frame successfully), but
        // it may be the capture card's own idle/blank pattern rather
        // than real video -- see LinkState::NoVideoInput in the header.
        // Only flips state between Connected and NoVideoInput; leaves
        // Disconnected/Searching/SignalLost alone (those are handled by
        // the read-failure branch above, not here).
        //
        // Hysteresis (see kIdleEnterFraction/kIdleExitFraction) plus a
        // minimum dwell time between flips (kStateChangeCooldownMs) is
        // what fixes the warning box flickering on borderline frames --
        // a single threshold meant a frame right at the boundary (e.g.
        // OSD telemetry digits changing on the idle background) could
        // toggle the state every few frames.
        {
            double frac = idleColorFraction(frame);
            LinkState curState = linkState_.load();
            auto nowTs = std::chrono::steady_clock::now();
            auto sinceStateChange = std::chrono::duration_cast<std::chrono::milliseconds>(
                nowTs - lastNoVideoStateChange).count();

            if (curState == LinkState::Connected) {
                if (frac >= kIdleEnterFraction) {
                    ++idleStreak;
                    liveStreak = 0;
                }
                else {
                    idleStreak = 0;
                }
                if (idleStreak >= kIdleFramesToConfirm &&
                    sinceStateChange >= kStateChangeCooldownMs) {
                    std::cout << "[VideoLink] capture card connected but no "
                        "drone video detected (idle/blank frame) on device "
                        << lastDeviceIndex_ << std::endl;
                    linkState_.store(LinkState::NoVideoInput);
                    lastNoVideoStateChange = nowTs;
                    idleStreak = 0;
                }
            }
            else if (curState == LinkState::NoVideoInput) {
                if (frac < kIdleExitFraction) {
                    ++liveStreak;
                    idleStreak = 0;
                }
                else {
                    liveStreak = 0;
                }
                if (liveStreak >= kLiveFramesToConfirm &&
                    sinceStateChange >= kStateChangeCooldownMs) {
                    std::cout << "[VideoLink] drone video acquired on device "
                        << lastDeviceIndex_ << std::endl;
                    linkState_.store(LinkState::Connected);
                    lastNoVideoStateChange = nowTs;
                    liveStreak = 0;
                }
            }
            else {
                // Disconnected/Searching/SignalLost -- not our concern
                // here, just keep the streaks from carrying stale state
                // into whichever of Connected/NoVideoInput comes next.
                idleStreak = 0;
                liveStreak = 0;
            }
        }

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