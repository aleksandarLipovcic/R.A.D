#include "VideoLink.h"
// Full definition of TelemetrySnapshot (VideoLink.h only forward-declares
// it) -- adjust this include if TelemetrySnapshot ever moves out of
// DetectionLink.h; it's defined there today per Bindings.cpp's own
// include order (VideoLink.h then DetectionLink.h).
#include "DetectionLink.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
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

// ── Debug console logging ────────────────────────────────────────────
//
// Every line below used to print unconditionally -- useful in isolation
// while diagnosing the connect/reconnect timing issues this file went
// through, but once other subsystems are running alongside this one it
// drowns out everything else in the console. Nothing has been deleted:
// every std::cout call is still here verbatim, just routed through this
// macro so it can be silenced or restored with one line instead of
// hand-editing dozens of call sites.
//
// Set VIDEOLINK_VERBOSE_LOGGING to 1 (or define it via a build-system
// flag, e.g. a debug-config preprocessor define) to bring every one of
// these lines back exactly as they were.
#ifndef VIDEOLINK_VERBOSE_LOGGING
#define VIDEOLINK_VERBOSE_LOGGING 0
#endif

#if VIDEOLINK_VERBOSE_LOGGING
#define VL_LOG(...) do { std::cout __VA_ARGS__ << std::endl; } while (0)
#else
#define VL_LOG(...) do { } while (0)
#endif

namespace {
    // ── Capture-open configuration (SLONWAKE 5.8G FPV OTG Receiver) ───
    //
    // The three points below were each individually validated against
    // real hardware (multiple connect/disconnect/reconnect cycles, full
    // console timing logs) before being made permanent here:
    //
    //  - No FOURCC request. This dongle has no onboard MJPG encoder (a
    //    cheap analog-capture bridge chip) and always negotiates YUY2
    //    regardless of what's requested -- confirmed by
    //    logActualNegotiatedFormat() reporting FOURCC="YUY2" identically
    //    whether MJPG was requested or not. Requesting it cost ~700ms
    //    per connect/reconnect for a format never actually granted.
    //
    //  - No FPS request. The receiver's own spec is a 720x480 @ 30fps
    //    analog source; the measured fps heartbeat in every capture run
    //    (with or without an explicit FPS request) landed at ~30fps.
    //    Requesting CAP_PROP_FPS was not changing real throughput, only
    //    making cap.get(CAP_PROP_FPS) parrot back whatever was last
    //    set -- and cost another ~700ms per connect/reconnect to ask
    //    for.
    //
    //  - Width/height/buffersize passed directly into cap.open(index,
    //    backend, params) instead of as separate sequential cap.set()
    //    calls afterward. DSHOW batches pending format changes and only
    //    commits/rebuilds the filter graph once it has enough info to
    //    do so; passing everything at open() collapsed what used to be
    //    two graph rebuilds (one implicit in open(), one triggered by
    //    the first cap.set() call) into one.
    //
    // Combined effect measured on this hardware: connect() dropped from
    // ~3000ms to ~800ms, and the reconnect-after-replug segment (device
    // found by the OS -> Connected) dropped from ~2800ms to ~650ms.
    //
    // If a future capture device needs FOURCC/FPS negotiation after
    // all, reintroduce those two cap.set() calls in applyOpenParams()
    // below and confirm via logActualNegotiatedFormat() that the
    // request is actually being honored before assuming it's doing
    // anything -- don't trust cap.get() alone, it can echo back a
    // requested value the hardware silently ignored.

    // Wall-clock "HH:MM:SS.mmm" timestamp prefix for every log line, so
    // connect/disconnect/reconnect events across the whole lifecycle can
    // be correlated against real time (and against when you physically
    // pulled/replugged the cable) instead of only against
    // elapsedMs()-since-some-earlier-call-in-this-function numbers,
    // which don't tell you anything about gaps *between* calls or about
    // what happened on a different thread in between.
    std::string nowStr() {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
        std::time_t t = system_clock::to_time_t(now);
        std::tm tmBuf{};
        localtime_s(&tmBuf, &t);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03lld",
            tmBuf.tm_hour, tmBuf.tm_min, tmBuf.tm_sec,
            static_cast<long long>(ms.count()));
        return std::string(buf);
    }

    // Human-readable LinkState name for transition logging (e.g.
    // "Connected -> SignalLost"), so the log itself narrates the state
    // machine instead of you having to infer it from which message
    // happened to print.
    const char* linkStateName(LinkState s) {
        switch (s) {
        case LinkState::Disconnected: return "Disconnected";
        case LinkState::Searching: return "Searching";
        case LinkState::Connected: return "Connected";
        case LinkState::SignalLost: return "SignalLost";
        case LinkState::NoVideoInput: return "NoVideoInput";
        }
        return "?";
    }

    // Logs a state transition with both old and new state names, and
    // performs the store -- use this instead of calling
    // linkState_.store() directly so every transition is guaranteed to
    // be visible in the log, not just the ones someone remembered to
    // hand-annotate with a VL_LOG(...) call.
    void setLinkState(std::atomic<LinkState>& stateVar, LinkState newState, const char* why) {
        LinkState old = stateVar.load();
        if (old == newState)
            return;
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] STATE " << linkStateName(old)
            << " -> " << linkStateName(newState) << "  (" << why << ")");
        stateVar.store(newState);
    }

    // Number of consecutive failed cap.read() calls before we declare
    // the signal lost. At the loop's 5ms failure-path sleep, this is
    // roughly 75-100ms of real signal dropout before we react -- long
    // enough to ride out a single dropped USB packet without flapping
    // the UI, short enough that a genuine disconnect is caught almost
    // immediately.
    //
    // NOTE: this streak count is diagnostic/secondary now -- see
    // kSignalLostTimeoutMs below. On unplug, DSHOW can intermittently
    // hand back a stale cached "successful" read mid-teardown, which
    // resets a pure consecutive-failure counter to 0 before it ever
    // reaches this threshold. That's what produced the endless
    // "grab failures starting... monitoring" loop that never actually
    // declared SignalLost. The time-based check is what actually
    // guards against that; this constant still gates the fast path for
    // a clean, uninterrupted failure run.
    constexpr int kFailuresBeforeSignalLost = 15;

    // Time-based signal-loss trigger, driven off lastFrameTimeMs_
    // (which only advances on a genuinely successful, non-empty read).
    // Unlike kFailuresBeforeSignalLost, this can't be reset by a single
    // stray "successful" read landing in the middle of a real dropout --
    // it only resets when a frame that actually decoded successfully
    // comes in. Set a bit above the failure-count path's own ~75-100ms
    // budget (see comment above) so ordinary single-packet blips still
    // ride it out untouched, but a real disconnect is still caught
    // quickly.
    constexpr int64_t kSignalLostTimeoutMs = 250;

    // Threshold for treating a "successful" cap.read() as dead anyway.
    // Confirmed by logging: on this hardware, after a physical unplug,
    // DSHOW's dying filter graph doesn't reliably make cap.read() return
    // false at all -- it keeps returning true with a stale/cached frame,
    // just gated behind an internal ~1000ms stall per call (consistent
    // with some internal DirectShow timeout). Since that failure mode
    // never enters the `!readOk` branch, kSignalLostTimeoutMs above never
    // even gets evaluated for it, and the link can be dead indefinitely
    // while still reporting Connected/NoVideoInput with a frozen frame.
    // A single legitimate frame at any supported capture rate (15-60fps)
    // should never take anywhere near this long, so treat any read this
    // slow as a dead link regardless of its boolean result, and route it
    // through the same failure-counting/time-based SignalLost path as a
    // genuine read failure instead of accepting the stale frame as live.
    constexpr int64_t kStalledReadMs = 300;

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

    // Generic USB device interface class (GUID_DEVINTERFACE_USB_DEVICE).
    // Registered as a SECOND, earlier trigger alongside the video-class
    // GUID above: on some capture card/driver combinations, the raw USB
    // device arrival can be reported by Windows meaningfully earlier
    // than the point where the video-capture-class interface itself
    // finishes registering (the class driver/KS filter graph binding
    // takes its own extra time on top of the bus-level arrival). Either
    // notification firing is treated identically -- see
    // DeviceNotifyWndProc below -- so this just gives the earliest
    // possible signal, whichever class happens to report first for a
    // given card.
    constexpr GUID kUsbDeviceInterfaceGuid = {
        0xa5dcbf10, 0x6530, 0x11d2, { 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed }
    };

    // After ANY arrival signal, retry this often (instead of the normal
    // kReconnectIntervalMs) for kPostArrivalBurstMs -- the arrival event
    // means the device exists again at the bus/driver level, but it may
    // still take the capture pipeline itself a short moment to become
    // openable, so a short fast-retry burst right after the signal
    // catches that moment much sooner than waiting out the steady-state
    // poll interval.
    constexpr int kBurstReconnectIntervalMs = 100;
    constexpr int kPostArrivalBurstMs = 4000;

    // Small helper used by connect()/tryReconnect()'s step-by-step
    // timing logs -- returns milliseconds since `from`.
    long long elapsedMs(std::chrono::steady_clock::time_point from) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - from).count();
    }
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

namespace {
    // The full set of OSD element ids the overlay knows how to draw.
    // setOsdElementEnabled()/setOsdElementAnchor() reject anything not in
    // this list. Add a new id here + a case in VideoLink::drawOsdOverlay()
    // together -- keeping them in one place is what makes an unknown id
    // from the Python side a loud no-op instead of a silently-ignored typo.
    const char* kOsdElementIds[] = { "altitude", "horizon", "compass",
        "battery", "rssi", "gps", "timer", "home_distance" };
}

VideoLink::VideoLink() {
    static bool logLevelSet = false;
    if (!logLevelSet) {
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
        logLevelSet = true;
    }
    // Seed every known element with a sane default layout up front so
    // getOsdElementLayout()/the settings panel always has something to
    // read, even before the pilot has touched anything this session.
    // Defaults mirror where these three sat in the Betaflight OSD
    // screenshot: altitude top-left, horizon dead center, compass
    // bottom-center.
    {
        std::lock_guard<std::mutex> lock(osdMutex_);
        osdLayout_["altitude"] = OsdElementLayout{ true, OsdAnchor::TopLeft, 16, 16, 0.5f, 0.5f };
        osdLayout_["horizon"] = OsdElementLayout{ true, OsdAnchor::Center, 0, 0, 0.5f, 0.5f };
        osdLayout_["compass"] = OsdElementLayout{ true, OsdAnchor::BottomCenter, 16, 16, 0.5f, 0.5f };
        // New elements default to the four corners/edges the three above
        // don't already occupy, mirroring the spread-out layout of a
        // typical Betaflight/INAV OSD (each stat gets its own spot,
        // nothing overlapping) -- the pilot can drag any of these
        // elsewhere via the settings panel same as the original three.
        osdLayout_["timer"] = OsdElementLayout{ true, OsdAnchor::TopCenter, 16, 16, 0.5f, 0.5f };
        osdLayout_["rssi"] = OsdElementLayout{ true, OsdAnchor::TopRight, 16, 16, 0.5f, 0.5f };
        osdLayout_["gps"] = OsdElementLayout{ true, OsdAnchor::MiddleLeft, 16, 16, 0.5f, 0.5f };
        osdLayout_["home_distance"] = OsdElementLayout{ true, OsdAnchor::MiddleRight, 16, 16, 0.5f, 0.5f };
        osdLayout_["battery"] = OsdElementLayout{ true, OsdAnchor::BottomLeft, 16, 16, 0.5f, 0.5f };
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

    DEV_BROADCAST_DEVICEINTERFACE_W videoFilter = {};
    videoFilter.dbcc_size = sizeof(videoFilter);
    videoFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    videoFilter.dbcc_classguid = kVideoInputDeviceGuid;
    HDEVNOTIFY hVideoNotify = RegisterDeviceNotificationW(
        hwnd, &videoFilter, DEVICE_NOTIFY_WINDOW_HANDLE);

    DEV_BROADCAST_DEVICEINTERFACE_W usbFilter = {};
    usbFilter.dbcc_size = sizeof(usbFilter);
    usbFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    usbFilter.dbcc_classguid = kUsbDeviceInterfaceGuid;
    HDEVNOTIFY hUsbNotify = RegisterDeviceNotificationW(
        hwnd, &usbFilter, DEVICE_NOTIFY_WINDOW_HANDLE);

    // Logged once at startup so it's obvious from the console whether
    // fast reconnect has a working OS signal to rely on at all, or is
    // silently falling all the way back to plain polling for this
    // particular system/hardware.
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] device arrival notifications: video-class="
        << (hVideoNotify ? "ok" : "FAILED") << ", usb-class="
        << (hUsbNotify ? "ok" : "FAILED"));

    notifyThreadId_.store(GetCurrentThreadId());
    notifyThreadReady_.store(true);

    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    if (hVideoNotify)
        UnregisterDeviceNotification(hVideoNotify);
    if (hUsbNotify)
        UnregisterDeviceNotification(hUsbNotify);
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
    //
    // OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS is forced off here, before
    // MFStartup() ever runs, because it's read once by OpenCV's MSMF
    // backend at that point. With it left at its default (enabled),
    // MSMF tries to negotiate a hardware-accelerated (DXVA) decode/
    // transform path for whatever FOURCC we ask for (MJPG here) as part
    // of every cap.open()/cap.set() call -- and on several USB capture
    // dongle drivers that hardware-transform enumeration is what was
    // actually costing the multi-second stalls seen in connect()/
    // tryReconnect() (8-10s+ in the property-set calls alone), not the
    // device I/O itself. Forcing software-only negotiation is the
    // standard workaround for this exact symptom. Setting it here
    // (rather than relying on an external env var / main()) guarantees
    // it's in place no matter which code path triggers MFStartup()
    // first.
    static std::once_flag mfInitFlag;
    std::call_once(mfInitFlag, []() {
        _putenv_s("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0");
        MFStartup(MF_VERSION);
        });

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
    // std::cout << "[" << nowStr() << "] [VideoLink] idle-fraction=" << (double)matched / sampled << std::endl;

    return static_cast<double>(matched) / sampled;
}

// =============================================================================
// connectAuto() / connect()
// =============================================================================

bool VideoLink::connectAuto() {
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] connectAuto() called");
    setLinkState(linkState_, LinkState::Searching, "connectAuto start");

    auto devices = enumerateDevices();

    std::vector<CaptureDeviceInfo> candidates;
    for (auto& d : devices)
        if (!looksLikeIntegratedWebcam(d.name))
            candidates.push_back(d);

    if (candidates.empty())
        candidates = devices;

    for (auto& d : candidates) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] trying capture device [" << d.index << "] "
            << d.name);
        if (connect(d.index)) {
            deviceName_ = d.name;
            VL_LOG(<< "[" << nowStr() << "] [VideoLink] connected: " << d.name);
            return true;
        }
    }

    VL_LOG(<< "[" << nowStr() << "] [VideoLink] connectAuto() exhausted all "
        << candidates.size() << " candidate device(s), none connected");
    setLinkState(linkState_, LinkState::Disconnected, "connectAuto: no candidate connected");
    return false;
}

namespace {
    // Every requested capture property is only a *request* -- the
    // driver is free to silently ignore it and negotiate something else
    // entirely (different FOURCC, different resolution, different
    // color format) if the device doesn't actually support what we
    // asked for. This reads back what was ACTUALLY negotiated so we can
    // compare it against what another app (e.g. OBS) ends up using for
    // the same physical device -- a FOURCC or resolution mismatch here
    // is a strong, concrete lead for a color-rendering difference
    // between the two apps, rather than guessing at gamma/colorspace
    // theories with no data to back them.
    void logActualNegotiatedFormat(cv::VideoCapture& cap, const char* context) {
        int fourccInt = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
        char fourccStr[5] = {
            static_cast<char>(fourccInt & 0xFF),
            static_cast<char>((fourccInt >> 8) & 0xFF),
            static_cast<char>((fourccInt >> 16) & 0xFF),
            static_cast<char>((fourccInt >> 24) & 0xFF),
            '\0'
        };
        double w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        double h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap.get(cv::CAP_PROP_FPS);
        double convertRgb = cap.get(cv::CAP_PROP_CONVERT_RGB);
        VL_LOG(<< "[" << nowStr() << "] [VideoLink]   " << context
            << ": actual negotiated format -- FOURCC=\"" << fourccStr
            << "\" (0x" << std::hex << fourccInt << std::dec << ")"
            << " " << w << "x" << h << " @ " << fps << "fps"
            << ", CONVERT_RGB=" << convertRgb);
    }

    // ── Backend order: DSHOW first, MSMF fallback ───────────────────
    //
    // An earlier version of this function raced CAP_MSMF and CAP_DSHOW
    // against each other concurrently (separate threads, each opening
    // its own cv::VideoCapture on the same device index) to dodge
    // MSMF's 5+ second cap.open() on this hardware. That backfired: the
    // loser was detached rather than cancelled -- cv::VideoCapture::
    // open() has no cancellation hook -- so for up to ~5s after every
    // connect, a second backend was still holding (or trying to grab)
    // the SAME physical device while the winner was actively streaming.
    // That's consistent with what testing showed: FPS dropping from 60
    // to 30 (bandwidth/negotiation contention between the two opens),
    // and reconnect breaking entirely after a replug (an MSMF
    // cap.open() has no timeout and can hang indefinitely if the device
    // disappears mid-open, and a hung/half-initialized MSMF source can
    // leave the capture-class interface unable to be reopened by
    // anything until that call unblocks -- which it may never do
    // against a device that's gone).
    //
    // Fix: never have two backends holding the device at once. DSHOW
    // reliably opens this device in ~300-400ms (see the earlier
    // connect()/tryReconnect() timing logs), so there's no real need to
    // race it against MSMF -- just try DSHOW first and only fall back
    // to MSMF if DSHOW fails outright. Same "try A, fall back to B on
    // failure" shape the original code had (MSMF first, DSHOW
    // fallback), just with the order swapped to match which backend is
    // actually fast and reliable here, and it guarantees strictly
    // sequential, single-owner device access at all times.
    //
    // `params` is passed straight into cap.open() (rather than applied
    // afterward via sequential cap.set() calls) so DSHOW can negotiate
    // width/height/buffersize as part of a single filter-graph build --
    // see buildOpenParams() below and the capture-open configuration
    // notes near the top of this file for why this replaced the older
    // sequential cap.set() approach.
    bool openPreferDshow(cv::VideoCapture& cap, int deviceIndex, const std::vector<int>& params) {
        auto tDshow = std::chrono::steady_clock::now();
        bool opened = cap.open(deviceIndex, cv::CAP_DSHOW, params);
        VL_LOG(<< "[" << nowStr() << "] [VideoLink]     cap.open(CAP_DSHOW) took "
            << elapsedMs(tDshow) << "ms, result=" << opened);

        if (!opened) {
            auto tMsmf = std::chrono::steady_clock::now();
            opened = cap.open(deviceIndex, cv::CAP_MSMF, params);
            VL_LOG(<< "[" << nowStr() << "] [VideoLink]     cap.open(CAP_MSMF) took "
                << elapsedMs(tMsmf) << "ms, result=" << opened);
        }
        return opened;
    }

    // Capture properties requested at open() time. Deliberately omits
    // FOURCC and FPS -- see the capture-open configuration notes near
    // the top of this file for why both were dropped after hardware
    // testing. prefFps is intentionally unused here; setPreferredFps()
    // remains a valid no-op-today call in case a future capture device
    // needs it reinstated.
    std::vector<int> buildOpenParams(int prefW, int prefH) {
        return {
            cv::CAP_PROP_FRAME_WIDTH, prefW,
            cv::CAP_PROP_FRAME_HEIGHT, prefH,
            cv::CAP_PROP_BUFFERSIZE, 1,
        };
    }
}

bool VideoLink::connect(int deviceIndex) {
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] connect(" << deviceIndex
        << ") called, current state=" << linkStateName(linkState_.load()));
    disconnect();

    setLinkState(linkState_, LinkState::Searching, "connect() start");
    lastDeviceIndex_ = deviceIndex;

    auto tConnectStart = std::chrono::steady_clock::now();

    openPreferDshow(cap, deviceIndex, buildOpenParams(prefW, prefH));

    if (!cap.isOpened()) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] connect() failed: no backend could open device "
            << deviceIndex << " after " << elapsedMs(tConnectStart) << "ms");
        setLinkState(linkState_, LinkState::Disconnected, "connect() failed: no backend opened");
        return false;
    }

    logActualNegotiatedFormat(cap, "connect");

    // A single successful read() here isn't sufficient proof the stream
    // is actually stable -- see kStableFramesRequired above. Require a
    // short run of consecutive good frames before declaring connect()
    // successful and starting the capture thread.
    cv::Mat testFrame;
    for (int i = 0; i < kStableFramesRequired; ++i) {
        auto tRead = std::chrono::steady_clock::now();
        bool ok = cap.read(testFrame) && !testFrame.empty();
        VL_LOG(<< "[" << nowStr() << "] [VideoLink]   connect: stability read " << (i + 1) << "/"
            << kStableFramesRequired << " took " << elapsedMs(tRead)
            << "ms, result=" << ok);
        if (!ok) {
            cap.release();
            VL_LOG(<< "[" << nowStr() << "] [VideoLink] connect() failed: stability check failed after "
                << elapsedMs(tConnectStart) << "ms total");
            setLinkState(linkState_, LinkState::Disconnected, "connect() failed: stability check");
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

    VL_LOG(<< "[" << nowStr() << "] [VideoLink] connect() succeeded in " << elapsedMs(tConnectStart)
        << "ms total");

    connected.store(true);
    keepRunning.store(true);
    setLinkState(linkState_, LinkState::Connected, "connect() succeeded");
    lastFrameTimeMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] connect(): starting capture thread");
    captureThread = std::thread(&VideoLink::captureLoop, this);
    return true;
}

void VideoLink::disconnect() {
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] disconnect() called, current state="
        << linkStateName(linkState_.load()) << ", captureThread joinable="
        << captureThread.joinable());

    keepRunning.store(false);

    if (captureThread.joinable()) {
        auto tJoin = std::chrono::steady_clock::now();
        captureThread.join();
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] disconnect(): capture thread joined after "
            << elapsedMs(tJoin) << "ms");
    }
    else {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] disconnect(): no capture thread to join");
    }

    if (cap.isOpened()) {
        auto tRel = std::chrono::steady_clock::now();
        cap.release();
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] disconnect(): cap.release() took "
            << elapsedMs(tRel) << "ms");
    }

    connected.store(false);
    setLinkState(linkState_, LinkState::Disconnected, "disconnect() complete");
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

// =============================================================================
// Software OSD overlay
// =============================================================================
//
// See the module comment on this block in VideoLink.h for why this exists
// (replacing the FC's analog OSD, which was optically baked into every
// frame ahead of the capture card and confusing the aerial-detection
// model) and why it's safe by construction: everything here draws into
// the render-window back buffer only, one layer above the plain
// StretchDIBits blit in paintFrameDirect(), and nowhere near latestFrame_
// (see captureLoop(), which stores latestFrame_ well before paintFrame()
// is even called).

namespace {
    bool isKnownOsdElementId(const std::string& id) {
        for (const char* known : kOsdElementIds)
            if (id == known)
                return true;
        return false;
    }
}

void VideoLink::setOsdElementEnabled(const std::string& id, bool enabled) {
    if (!isKnownOsdElementId(id)) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] setOsdElementEnabled: unknown id '"
            << id << "' ignored");
        return;
    }
    std::lock_guard<std::mutex> lock(osdMutex_);
    osdLayout_[id].enabled = enabled;
}

void VideoLink::setOsdElementAnchor(const std::string& id, OsdAnchor anchor, int marginX, int marginY) {
    if (!isKnownOsdElementId(id)) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] setOsdElementAnchor: unknown id '"
            << id << "' ignored");
        return;
    }
    std::lock_guard<std::mutex> lock(osdMutex_);
    OsdElementLayout& layout = osdLayout_[id];
    layout.anchor = anchor;
    layout.marginX = marginX;
    layout.marginY = marginY;
}

void VideoLink::setOsdElementCustomPosition(const std::string& id, float fx, float fy) {
    if (!isKnownOsdElementId(id)) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] setOsdElementCustomPosition: unknown id '"
            << id << "' ignored");
        return;
    }
    // Clamp defensively -- a drag that ends outside the video panel
    // (pilot's mouse slid past the edge before releasing) should clamp to
    // the edge, not place the element off-screen where it can never be
    // grabbed again without falling back to a preset anchor.
    fx = (std::max)(0.0f, (std::min)(1.0f, fx));
    fy = (std::max)(0.0f, (std::min)(1.0f, fy));
    std::lock_guard<std::mutex> lock(osdMutex_);
    OsdElementLayout& layout = osdLayout_[id];
    layout.anchor = OsdAnchor::Custom;
    layout.customFx = fx;
    layout.customFy = fy;
}

VideoLink::OsdElementLayout VideoLink::getOsdElementLayout(const std::string& id) const {
    std::lock_guard<std::mutex> lock(osdMutex_);
    auto it = osdLayout_.find(id);
    return (it != osdLayout_.end()) ? it->second : OsdElementLayout{};
}

std::vector<std::string> VideoLink::getOsdElementIds() const {
    return std::vector<std::string>(std::begin(kOsdElementIds), std::end(kOsdElementIds));
}

void VideoLink::setTelemetryProvider(std::function<TelemetrySnapshot()> provider) {
    telemetryProvider_ = std::move(provider);
}

void VideoLink::resetFlightTimer() {
    // Called from whatever thread the cockpit button lives on (Tk/UI
    // thread) -- just raise the flag. The actual mutation of
    // osdTimerAccumulated_/osdTimerRunning_/osdTimerSegmentStart_ happens
    // inside drawOsdTimer() on the capture/paint thread, which is the
    // only thread allowed to touch that state.
    osdTimerResetRequested_.store(true, std::memory_order_relaxed);
}

void VideoLink::resolveOsdOrigin(const OsdElementLayout& layout, int destW, int destH,
    int elemW, int elemH, int& outX, int& outY) const {
    if (layout.anchor == OsdAnchor::Custom) {
        outX = static_cast<int>(layout.customFx * destW) - elemW / 2;
        outY = static_cast<int>(layout.customFy * destH) - elemH / 2;
    }
    else {
        // Horizontal placement.
        switch (layout.anchor) {
        case OsdAnchor::TopLeft: case OsdAnchor::MiddleLeft: case OsdAnchor::BottomLeft:
            outX = layout.marginX;
            break;
        case OsdAnchor::TopCenter: case OsdAnchor::Center: case OsdAnchor::BottomCenter:
            outX = (destW - elemW) / 2;
            break;
        default:   // TopRight / MiddleRight / BottomRight
            outX = destW - elemW - layout.marginX;
            break;
        }
        // Vertical placement.
        switch (layout.anchor) {
        case OsdAnchor::TopLeft: case OsdAnchor::TopCenter: case OsdAnchor::TopRight:
            outY = layout.marginY;
            break;
        case OsdAnchor::MiddleLeft: case OsdAnchor::Center: case OsdAnchor::MiddleRight:
            outY = (destH - elemH) / 2;
            break;
        default:   // BottomLeft / BottomCenter / BottomRight
            outY = destH - elemH - layout.marginY;
            break;
        }
    }
    // Keep fully on-panel regardless of how the position was derived --
    // a stale custom position from a previous, larger panel size (video
    // host resized smaller since it was saved) shouldn't push an element
    // partly off the visible video.
    outX = (std::max)(0, (std::min)(outX, (std::max)(0, destW - elemW)));
    outY = (std::max)(0, (std::min)(outY, (std::max)(0, destH - elemH)));
}

void VideoLink::layoutOsdReadout(HDC hdc, const wchar_t* text, const OsdElementLayout& layout,
    int destW, int destH, int& outX, int& outY, int& outW, int& outH) const {
    RECT calc = { 0, 0, 0, 0 };
    DrawTextW(hdc, text, -1, &calc, DT_CALCRECT | DT_SINGLELINE);
    outW = calc.right - calc.left + 12;
    outH = calc.bottom - calc.top + 8;
    resolveOsdOrigin(layout, destW, destH, outW, outH, outX, outY);
}

void VideoLink::drawOsdOverlay(HDC targetHdc, int destW, int destH) {
    if (!osdOverlayEnabled_.load() || !telemetryProvider_)
        return;

    TelemetrySnapshot t = telemetryProvider_();
    if (!t.valid)
        return;   // GPS/attitude not trustworthy yet -- nothing sane to draw

    // Snapshot the whole layout table once per frame instead of locking
    // per-element -- three short reads under one lock beats three lock
    // acquisitions, and guarantees all three elements are drawn against
    // the same consistent layout even if a setter runs concurrently.
    std::map<std::string, OsdElementLayout> layout;
    {
        std::lock_guard<std::mutex> lock(osdMutex_);
        layout = osdLayout_;
    }

    auto it = layout.find("altitude");
    if (it != layout.end() && it->second.enabled)
        drawOsdAltitude(targetHdc, it->second, destW, destH, t);

    it = layout.find("horizon");
    if (it != layout.end() && it->second.enabled)
        drawOsdHorizon(targetHdc, it->second, destW, destH, t);

    it = layout.find("compass");
    if (it != layout.end() && it->second.enabled)
        drawOsdCompass(targetHdc, it->second, destW, destH, t);

    it = layout.find("battery");
    if (it != layout.end() && it->second.enabled)
        drawOsdBattery(targetHdc, it->second, destW, destH, t);

    it = layout.find("rssi");
    if (it != layout.end() && it->second.enabled)
        drawOsdRssi(targetHdc, it->second, destW, destH, t);

    it = layout.find("gps");
    if (it != layout.end() && it->second.enabled)
        drawOsdGps(targetHdc, it->second, destW, destH, t);

    it = layout.find("timer");
    if (it != layout.end() && it->second.enabled)
        drawOsdTimer(targetHdc, it->second, destW, destH, t);

    it = layout.find("home_distance");
    if (it != layout.end() && it->second.enabled)
        drawOsdHomeDistance(targetHdc, it->second, destW, destH, t);
}

namespace {
    // Shared look for every OSD text readout -- Consolas, matches
    // FPVWidget's status-bar font choice on the Python side so the
    // software OSD reads as part of the same UI, not a mismatched
    // add-on. Cached per requested pixel height the same way
    // paintFrameDirect already caches its NoVideoInput font.
    HFONT getOsdFont(int pixelHeight) {
        static HFONT cachedFont = nullptr;
        static int cachedHeight = 0;
        if (!cachedFont || cachedHeight != pixelHeight) {
            if (cachedFont)
                DeleteObject(cachedFont);
            cachedFont = CreateFontW(
                -pixelHeight, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_MODERN, L"Consolas");
            cachedHeight = pixelHeight;
        }
        return cachedFont;
    }

    // Standard analog-FPV-OSD look: bold WHITE glyphs with a 1px BLACK
    // outline and no filled backing plate. White reads over both bright
    // sky and dark terrain (unlike the old green, which all but
    // disappears over green fields/crops -- exactly the case a
    // search-and-rescue bird spends most of its time over), and the black
    // outline is what keeps it legible over bright/washed-out sky without
    // needing a solid box that eats a chunk of the frame. This is drawn
    // by painting the same text 8 times in black at a 1px ring of offsets
    // and then once more in white dead-center, same trick every FC OSD
    // (Betaflight/INAV) font renderer uses.
    void drawOutlinedText(HDC hdc, const wchar_t* text, const RECT& rect, UINT format,
        COLORREF color = RGB(255, 255, 255)) {
        static const int dx[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
        static const int dy[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
        SetTextColor(hdc, RGB(0, 0, 0));
        for (int i = 0; i < 8; ++i) {
            RECT r = rect;
            OffsetRect(&r, dx[i], dy[i]);
            DrawTextW(hdc, text, -1, &r, format);
        }
        SetTextColor(hdc, color);
        RECT r = rect;
        DrawTextW(hdc, text, -1, &r, format);
    }


    // Same outline trick for line art (horizon ladder, sidebar ticks,
    // compass tick) -- stroke the geometry once in black at width+2, then
    // again on top in white at the normal width, so the ladder/ticks stay
    // visible against bright sky the same way the text does instead of
    // depending on a single flat color.
    HPEN makeOsdPen(int width, COLORREF color) { return CreatePen(PS_SOLID, width, color); }
}

void VideoLink::drawOsdAltitude(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    wchar_t buf[32];
    swprintf_s(buf, L"ALT %.1f m", t.altitudeM);

    int fontPx = (std::max)(14, (std::min)(28, destH / 20));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    RECT calc = { 0, 0, 0, 0 };
    DrawTextW(hdc, buf, -1, &calc, DT_CALCRECT | DT_SINGLELINE);
    int elemW = calc.right - calc.left + 12;
    int elemH = calc.bottom - calc.top + 8;

    int x, y;
    resolveOsdOrigin(layout, destW, destH, elemW, elemH, x, y);

    // No backing plate -- plain outlined white text on the raw video,
    // matching how every standard FC (Betaflight/INAV) OSD renders its
    // readouts: just glyphs, no boxes eating into the frame.
    int oldBkMode = SetBkMode(hdc, TRANSPARENT);
    RECT textRect = { x + 6, y + 4, x + elemW - 6, y + elemH - 4 };
    drawOutlinedText(hdc, buf, textRect, DT_SINGLELINE | DT_VCENTER);

    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
}

void VideoLink::drawOsdHorizon(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    // Ladder size scales with panel height, same clamp shape used
    // elsewhere in this file so it stays legible on a small panel without
    // taking over a large one.
    int elemW = (std::max)(180, (std::min)(360, destW / 3));
    int elemH = elemW;   // square hit-box; the drawn ladder is narrower

    int originX, originY;
    resolveOsdOrigin(layout, destW, destH, elemW, elemH, originX, originY);
    int cx = originX + elemW / 2;
    int cy = originY + elemH / 2;

    // Pitch ladder offset (pixels per degree) and roll rotation. Positive
    // pitch = nose up = horizon line moves DOWN on screen (matches every
    // conventional attitude indicator).
    const double pxPerDeg = elemH / 60.0;   // +/-30 deg of pitch visible
    double pitchOffsetPx = t.pitchDeg * pxPerDeg;
    double rollRad = -t.rollDeg * 3.14159265358979 / 180.0;
    double cosR = std::cos(rollRad), sinR = std::sin(rollRad);

    auto rotate = [&](double lx, double ly, POINT& out) {
        // Rotate a point given in "local ladder space" (origin at the
        // pitch-shifted horizon center) by roll, then translate to screen.
        double rx = lx * cosR - ly * sinR;
        double ry = lx * sinR + ly * cosR;
        out.x = static_cast<LONG>(cx + rx);
        out.y = static_cast<LONG>(cy + pitchOffsetPx + ry);
        };

    // Draw every stroke twice: a wider black pass first, then a
    // narrower white pass on top of it. That gives every line a 1px
    // black outline (same technique as the text) so the ladder and
    // sidebars stay visible over bright sky AND over green fields/crops
    // -- the old solid green all but vanished over vegetation, which is
    // most of what a search-and-rescue bird flies over.
    auto strokeGeometry = [&](int width, COLORREF color) {
        HPEN pen = makeOsdPen(width, color);
        HGDIOBJ oldPen = SelectObject(hdc, pen);

        // Horizon line itself.
        POINT hl, hr;
        rotate(-elemW * 0.35, 0, hl);
        rotate(elemW * 0.35, 0, hr);
        MoveToEx(hdc, hl.x, hl.y, nullptr);
        LineTo(hdc, hr.x, hr.y);

        // Center reference chevron (fixed to the airframe, does NOT
        // rotate with roll -- this is what pilots actually read against
        // the moving horizon line).
        POINT chevron[3] = {
            { static_cast<LONG>(cx - 10), static_cast<LONG>(cy + 6) },
            { static_cast<LONG>(cx),      static_cast<LONG>(cy) },
            { static_cast<LONG>(cx + 10), static_cast<LONG>(cy + 6) },
        };
        Polyline(hdc, chevron, 3);

        // Sidebars -- vertical tick columns either side, the exact
        // element the NN was picking up false positives from in the FC
        // OSD. Purely cosmetic tick marks, fixed to the airframe like
        // the chevron.
        for (int side = -1; side <= 1; side += 2) {
            int sx = cx + side * static_cast<int>(elemW * 0.42);
            for (int i = -3; i <= 3; ++i) {
                int ty = cy + i * (elemH / 10);
                int tickLen = (i == 0) ? 14 : 8;
                MoveToEx(hdc, sx - tickLen / 2, ty, nullptr);
                LineTo(hdc, sx + tickLen / 2, ty);
            }
        }

        SelectObject(hdc, oldPen);
        DeleteObject(pen);
        };
    strokeGeometry(4, RGB(0, 0, 0));
    strokeGeometry(2, RGB(255, 255, 255));
}

void VideoLink::drawOsdCompass(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    int elemW = (std::max)(220, (std::min)(420, destW / 2));
    int elemH = (std::max)(24, (std::min)(40, destH / 16));

    int x, y;
    resolveOsdOrigin(layout, destW, destH, elemW, elemH, x, y);

    int fontPx = (std::max)(14, (std::min)(22, elemH - 10));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    // No backing plate -- outlined white letters directly over the video,
    // consistent with altitude/horizon above.
    int oldBkMode = SetBkMode(hdc, TRANSPARENT);

    // Simple scrolling tape: one compass letter every 45 degrees, offset
    // so the letter nearest current heading sits under the center tick --
    // same idea as the FC OSD's compass strip, but generated from a fixed
    // string instead of glyphs baked into the video, so nothing here can
    // ever be mistaken for a ground object by the detector (it never
    // reaches DetectionLink's frames at all).
    static const wchar_t* dirs[8] = { L"N", L"NE", L"E", L"SE", L"S", L"SW", L"W", L"NW" };
    double pxPerDeg = elemW / 90.0;   // +/-45 deg of heading visible across the tape
    for (int i = -1; i <= 8; ++i) {
        double dirHeading = ((i % 8) + 8) % 8 * 45.0;
        double delta = dirHeading - t.headingDeg;
        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;
        int tx = x + elemW / 2 + static_cast<int>(delta * pxPerDeg);
        if (tx < x - 20 || tx > x + elemW + 20)
            continue;
        RECT letterRect = { tx - 14, y, tx + 14, y + elemH };
        drawOutlinedText(hdc, dirs[((i % 8) + 8) % 8], letterRect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    }

    // Center tick showing exactly where current heading falls on the
    // tape -- kept as a distinct amber accent (outlined the same way)
    // so it still reads as "the pointer", not just another white line.
    auto strokeTick = [&](int width, COLORREF color) {
        HPEN tickPen = makeOsdPen(width, color);
        HGDIOBJ oldPen = SelectObject(hdc, tickPen);
        MoveToEx(hdc, x + elemW / 2, y, nullptr);
        LineTo(hdc, x + elemW / 2, y + elemH);
        SelectObject(hdc, oldPen);
        DeleteObject(tickPen);
        };
    strokeTick(4, RGB(0, 0, 0));
    strokeTick(2, RGB(255, 200, 0));

    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
}

void VideoLink::drawOsdBattery(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    wchar_t buf[32];
    swprintf_s(buf, L"BATT %.1fV %d%%", t.batteryVoltage, static_cast<int>(t.batteryPercentage));

    int fontPx = (std::max)(14, (std::min)(28, destH / 20));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    int x, y, elemW, elemH;
    layoutOsdReadout(hdc, buf, layout, destW, destH, x, y, elemW, elemH);

    // Below ~20% left, flip the whole readout to an amber warning color
    // (same accent the compass heading tick uses) instead of white -- a
    // low-battery call is exactly the kind of thing that shouldn't blend
    // in with every other white readout.
    COLORREF color = (t.batteryPercentage > 0 && t.batteryPercentage <= 20)
        ? RGB(255, 80, 40) : RGB(255, 255, 255);

    int oldBkMode = SetBkMode(hdc, TRANSPARENT);
    RECT textRect = { x + 6, y + 4, x + elemW - 6, y + elemH - 4 };
    drawOutlinedText(hdc, buf, textRect, DT_SINGLELINE | DT_VCENTER, color);
    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
}

void VideoLink::drawOsdRssi(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    // Shown as a 0-100% figure -- easier for a pilot to read at a glance
    // than the raw 0-255 link-quality byte the FC actually reports.
    int pct = static_cast<int>(t.rssi) * 100 / 255;
    wchar_t buf[24];
    swprintf_s(buf, L"RSSI %d%%", pct);

    int fontPx = (std::max)(14, (std::min)(28, destH / 20));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    int x, y, elemW, elemH;
    layoutOsdReadout(hdc, buf, layout, destW, destH, x, y, elemW, elemH);

    COLORREF color = (pct <= 20) ? RGB(255, 80, 40) : RGB(255, 255, 255);

    int oldBkMode = SetBkMode(hdc, TRANSPARENT);
    RECT textRect = { x + 6, y + 4, x + elemW - 6, y + elemH - 4 };
    drawOutlinedText(hdc, buf, textRect, DT_SINGLELINE | DT_VCENTER, color);
    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
}

void VideoLink::drawOsdGps(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    wchar_t buf[24];
    // fixType >= 2 is GPSReading::positionUsable's own "3D fix" bar --
    // matches the threshold DroneCockpitUI already uses to decide whether
    // lat/lon are trustworthy, so the OSD's notion of "locked" agrees
    // with the rest of the app's.
    bool locked = t.gpsFixType >= 2;
    if (locked)
        swprintf_s(buf, L"GPS %d", static_cast<int>(t.gpsNumSat));
    else
        swprintf_s(buf, L"GPS: NO FIX");

    int fontPx = (std::max)(14, (std::min)(28, destH / 20));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    int x, y, elemW, elemH;
    layoutOsdReadout(hdc, buf, layout, destW, destH, x, y, elemW, elemH);

    COLORREF color = locked ? RGB(255, 255, 255) : RGB(255, 80, 40);

    int oldBkMode = SetBkMode(hdc, TRANSPARENT);
    RECT textRect = { x + 6, y + 4, x + elemW - 6, y + elemH - 4 };
    drawOutlinedText(hdc, buf, textRect, DT_SINGLELINE | DT_VCENTER, color);
    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
}

void VideoLink::drawOsdTimer(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    // Flight timer semantics: starts counting on the drone's first arm,
    // pauses on disarm, resumes from where it left off on the next arm,
    // and is only ever zeroed by an explicit reset (resetFlightTimer(),
    // wired to a cockpit button) -- rearming never restarts it on its
    // own.
    //
    // Consume any pending reset first, before the edge-detection below,
    // so a reset that lands mid-armed-segment doesn't get immediately
    // re-accumulated by this same call.
    if (osdTimerResetRequested_.exchange(false, std::memory_order_relaxed)) {
        osdTimerAccumulated_ = std::chrono::steady_clock::duration::zero();
        if (osdTimerRunning_) {
            // Still armed right through the reset -- keep counting, just
            // from zero, instead of freezing until the next arm.
            osdTimerSegmentStart_ = std::chrono::steady_clock::now();
        }
    }

    // Edge-detect arm/disarm off the telemetry snapshot. Gated on
    // t.valid the same way the rest of the OSD effectively is elsewhere
    // -- an invalid/no-link snapshot defaults armed=false, which would
    // otherwise look like a disarm on every dropped telemetry frame.
    // Treat "no link this frame" as "no change" instead.
    if (t.valid) {
        if (t.armed && !osdTimerPrevArmed_) {
            // Rising edge: (re-)arm -- start a fresh segment on top of
            // whatever's already accumulated.
            osdTimerSegmentStart_ = std::chrono::steady_clock::now();
            osdTimerRunning_ = true;
        }
        else if (!t.armed && osdTimerPrevArmed_) {
            // Falling edge: disarm -- fold the just-finished segment into
            // the running total and stop counting until the next arm.
            osdTimerAccumulated_ += std::chrono::steady_clock::now() - osdTimerSegmentStart_;
            osdTimerRunning_ = false;
        }
        osdTimerPrevArmed_ = t.armed;
    }

    auto total = osdTimerAccumulated_;
    if (osdTimerRunning_)
        total += std::chrono::steady_clock::now() - osdTimerSegmentStart_;
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(total).count();
    int mm = static_cast<int>(elapsed / 60);
    int ss = static_cast<int>(elapsed % 60);

    wchar_t buf[16];
    swprintf_s(buf, L"%d:%02d", mm, ss);

    int fontPx = (std::max)(14, (std::min)(28, destH / 20));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    int x, y, elemW, elemH;
    layoutOsdReadout(hdc, buf, layout, destW, destH, x, y, elemW, elemH);

    int oldBkMode = SetBkMode(hdc, TRANSPARENT);
    RECT textRect = { x + 6, y + 4, x + elemW - 6, y + elemH - 4 };
    drawOutlinedText(hdc, buf, textRect, DT_SINGLELINE | DT_VCENTER);
    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
}

void VideoLink::drawOsdHomeDistance(HDC hdc, const OsdElementLayout& layout, int destW, int destH,
    const TelemetrySnapshot& t) {
    wchar_t buf[40];
    swprintf_s(buf, L"HOME %.0fm  %.1fm/s", t.homeDistanceM, t.groundSpeedMs);

    int fontPx = (std::max)(14, (std::min)(28, destH / 20));
    HFONT font = getOsdFont(fontPx);
    HGDIOBJ oldFont = SelectObject(hdc, font);

    int x, y, elemW, elemH;
    layoutOsdReadout(hdc, buf, layout, destW, destH, x, y, elemW, elemH);

    int oldBkMode = SetBkMode(hdc, TRANSPARENT);
    RECT textRect = { x + 6, y + 4, x + elemW - 6, y + elemH - 4 };
    drawOutlinedText(hdc, buf, textRect, DT_SINGLELINE | DT_VCENTER);
    SetBkMode(hdc, oldBkMode);
    SelectObject(hdc, oldFont);
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

    // ── Software OSD overlay ─────────────────────────────────────────
    // Drawn straight after the video blit, before the NoVideoInput
    // banner below, so the banner (when present) stays on top of and
    // fully legible over any OSD element it happens to overlap.
    drawOsdOverlay(targetHdc, destW, destH);

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
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] tryReconnect() called: lastDeviceIndex_="
        << lastDeviceIndex_ << " deviceName_=\"" << deviceName_
        << "\" cap.isOpened()=" << cap.isOpened());

    if (lastDeviceIndex_ < 0) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] tryReconnect() aborting: "
            "lastDeviceIndex_ is invalid (-1), nothing to reconnect to");
        return false;
    }

    auto attemptStart = std::chrono::steady_clock::now();

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
        auto tEnum = std::chrono::steady_clock::now();
        auto devices = enumerateDevices();
        int resolvedIndex = -1;
        for (auto& d : devices) {
            if (d.name == deviceName_) {
                resolvedIndex = d.index;
                break;
            }
        }
        VL_LOG(<< "[" << nowStr() << "] [VideoLink]   reconnect: enumerateDevices() took "
            << elapsedMs(tEnum) << "ms, device " << (resolvedIndex >= 0 ? "found" : "NOT found"));
        if (resolvedIndex < 0)
            return false;   // not currently present -- don't attempt open()
        lastDeviceIndex_ = resolvedIndex;
    }

    auto tRelease = std::chrono::steady_clock::now();
    cap.release();
    VL_LOG(<< "[" << nowStr() << "] [VideoLink]   reconnect: cap.release() took "
        << elapsedMs(tRelease) << "ms");

    // DSHOW first, MSMF fallback -- see openPreferDshow() above for why
    // this replaced the earlier concurrent MSMF/DSHOW race (that race
    // left a second backend holding the same device for seconds after
    // every connect, which is what broke both FPS and post-replug
    // reconnect).
    bool opened = openPreferDshow(cap, lastDeviceIndex_, buildOpenParams(prefW, prefH));
    if (!opened) {
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] reconnect attempt failed: no backend could "
            "open the device (" << elapsedMs(attemptStart) << "ms total)");
        return false;
    }

    logActualNegotiatedFormat(cap, "reconnect");

    auto tRead = std::chrono::steady_clock::now();
    cv::Mat probe;
    bool gotFrame = cap.read(probe) && !probe.empty();
    VL_LOG(<< "[" << nowStr() << "] [VideoLink]   reconnect: probe cap.read() took "
        << elapsedMs(tRead) << "ms, result=" << gotFrame);

    if (!gotFrame) {
        cap.release();
        VL_LOG(<< "[" << nowStr() << "] [VideoLink] reconnect attempt failed: opened but first "
            "read() failed (" << elapsedMs(attemptStart) << "ms total)");
        return false;
    }

    VL_LOG(<< "[" << nowStr() << "] [VideoLink] reconnect attempt succeeded in "
        << elapsedMs(attemptStart) << "ms total");
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
    // Timestamp of the moment we entered SignalLost, purely for the
    // "total downtime" number logged on recovery below -- this is the
    // number that actually matters to a pilot, as opposed to any single
    // attempt's timing.
    auto signalLostSince = std::chrono::steady_clock::time_point{};
    // While now() < burstUntil, retry every kBurstReconnectIntervalMs
    // instead of the steady-state kReconnectIntervalMs -- see
    // kPostArrivalBurstMs above for why.
    auto burstUntil = std::chrono::steady_clock::time_point{};
    // Diagnostics only: counts reconnect attempts within the current
    // SignalLost episode, and throttles the "still waiting" heartbeat so
    // the log shows the loop is alive without printing every 20ms tick.
    int reconnectAttemptNum = 0;
    auto lastHeartbeat = std::chrono::steady_clock::time_point{};

    VL_LOG(<< "[" << nowStr() << "] [VideoLink] captureLoop() thread started");

    while (keepRunning.load()) {
        // ── Recovery mode: a prior connection dropped, cap is dead ────
        // We retry either when the poll interval elapses, OR the instant
        // deviceArrivalSignal_ is set by the OS notification thread --
        // whichever comes first. An arrival signal also opens a short
        // fast-retry burst window (kPostArrivalBurstMs at
        // kBurstReconnectIntervalMs) rather than a single immediate
        // attempt, since the bus-level "device exists again" event can
        // still be a little ahead of the capture pipeline actually being
        // openable. Outside of that, kReconnectIntervalMs polling is
        // the fallback safety net for whenever the notification doesn't
        // fire at all for a given card/driver -- see the "video-class=
        // /usb-class=" line logged at startup to check that on your
        // hardware.
        if (linkState_.load() == LinkState::SignalLost) {
            if (signalLostSince == std::chrono::steady_clock::time_point{}) {
                signalLostSince = std::chrono::steady_clock::now();
                reconnectAttemptNum = 0;
                VL_LOG(<< "[" << nowStr() << "] [VideoLink] entering recovery loop for device "
                    << lastDeviceIndex_ << " (\"" << deviceName_ << "\")");
            }

            bool arrivalSignaled = deviceArrivalSignal_.exchange(false);
            auto now = std::chrono::steady_clock::now();

            if (arrivalSignaled) {
                auto sinceLost = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - signalLostSince).count();
                VL_LOG(<< "[" << nowStr() << "] [VideoLink] device arrival notification received "
                    << sinceLost << "ms after signal was lost -- starting fast retry");
                burstUntil = now + std::chrono::milliseconds(kPostArrivalBurstMs);
            }

            int effectiveIntervalMs = (now < burstUntil)
                ? kBurstReconnectIntervalMs : kReconnectIntervalMs;
            auto sinceLastAttempt = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastReconnectAttempt).count();

            if (!arrivalSignaled && sinceLastAttempt < effectiveIntervalMs) {
                // Heartbeat every ~2s so the log shows the recovery loop
                // is still alive and what it's waiting on, without
                // printing on every 20ms poll tick.
                auto sinceHeartbeat = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lastHeartbeat).count();
                if (sinceHeartbeat >= 2000) {
                    auto totalDownMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - signalLostSince).count();
                    VL_LOG(<< "[" << nowStr() << "] [VideoLink] recovery loop waiting: "
                        << "downtime=" << totalDownMs << "ms, attempts-so-far=" << reconnectAttemptNum
                        << ", nextAttemptIn=" << (effectiveIntervalMs - sinceLastAttempt) << "ms"
                        << ", burstActive=" << (now < burstUntil));
                    lastHeartbeat = now;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            lastReconnectAttempt = now;
            ++reconnectAttemptNum;

            VL_LOG(<< "[" << nowStr() << "] [VideoLink] recovery attempt #" << reconnectAttemptNum
                << " starting (arrivalSignaled=" << arrivalSignaled
                << ", burstActive=" << (now < burstUntil) << ")");

            bool reconnected = tryReconnect();

            VL_LOG(<< "[" << nowStr() << "] [VideoLink] recovery attempt #" << reconnectAttemptNum
                << " result: " << (reconnected ? "SUCCESS" : "failed, will retry"));

            if (reconnected) {
                auto downtimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - signalLostSince).count();
                VL_LOG(<< "[" << nowStr() << "] [VideoLink] signal recovered on device "
                    << lastDeviceIndex_ << " -- total downtime " << downtimeMs << "ms across "
                    << reconnectAttemptNum << " attempt(s)");
                consecutiveFailures = 0;
                idleStreak = 0;
                liveStreak = 0;
                lastNoVideoStateChange = std::chrono::steady_clock::now();
                signalLostSince = std::chrono::steady_clock::time_point{};
                burstUntil = std::chrono::steady_clock::time_point{};
                reconnectAttemptNum = 0;
                connected.store(true);
                setLinkState(linkState_, LinkState::Connected, "recovery attempt succeeded");
                fpsWindowStart = std::chrono::steady_clock::now();
                framesInWindow = 0;
                // A stale/possibly-garbage frame timestamp from before
                // the drop must not be allowed to make the very next
                // read-failure look like it's already been going on for
                // a long time -- reset the "last good frame" clock the
                // instant we're back, not just the failure counters.
                lastFrameTimeMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
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
        auto tReadStart = std::chrono::steady_clock::now();
        bool readOk = cap.read(frame) && !frame.empty();
        auto readDurationMs = elapsedMs(tReadStart);
        // A single cap.read() call taking anywhere near 1000ms is exactly
        // what a 30fps -> 1fps collapse looks like at the source -- this
        // makes that visible directly rather than inferring it from the
        // once-a-second measuredFps number. Threshold is well above a
        // normal frame interval (33ms at 30fps, 16ms at 60fps) so this
        // only fires on genuinely abnormal stalls, not routine jitter.
        if (readDurationMs >= 100) {
            VL_LOG(<< "[" << nowStr() << "] [VideoLink] SLOW cap.read() took "
                << readDurationMs << "ms (state=" << linkStateName(linkState_.load())
                << ", ok=" << readOk << ")");
        }

        // A read that took kStalledReadMs+ is treated as dead regardless
        // of its boolean result -- see kStalledReadMs above. Confirmed on
        // this hardware: after unplug, cap.read() kept returning true
        // with a stale cached frame, gated behind a ~1000ms internal
        // stall each call, and NEVER actually failed -- so relying on
        // `!readOk` alone left the link silently frozen forever with no
        // path to SignalLost.
        bool stalled = readOk && readDurationMs >= kStalledReadMs;
        if (stalled) {
            VL_LOG(<< "[" << nowStr() << "] [VideoLink] cap.read() returned \"success\" but took "
                << readDurationMs << "ms -- treating as a dead link, NOT a live frame");
        }

        if (!readOk || stalled) {
            // ── Signal-loss trigger: streak count AND elapsed time ─────
            //
            // On unplug, DSHOW's filter graph teardown can intermittently
            // hand back one stale/cached "successful" read in between a
            // run of real failures, before the pipeline is actually gone
            // for good. A pure consecutive-failure counter gets wiped
            // back to 0 by that single straggler and never reaches
            // kFailuresBeforeSignalLost -- which is exactly what produced
            // the repeating "grab failures starting... monitoring" loop
            // that never actually declared SignalLost.
            //
            // lastFrameTimeMs_ only advances on a read that genuinely
            // succeeded AND was fast enough to be live (see `stalled`
            // above), so "time since last good frame" keeps climbing
            // straight through both a flaky boolean-failure straggler
            // AND a stalled-but-"successful" read, instead of getting
            // reset by either. Trip on EITHER a clean, uninterrupted
            // failure streak (fast path, ~75-100ms) OR this elapsed-time
            // check (robust path, catches both flaky-success and
            // stalled-success cases) -- whichever fires first.
            if (consecutiveFailures == 0) {
                // Log the transition once per genuine streak start, not
                // on every failed grab and not every time a stray
                // success resets the streak counter -- this is the fix
                // for both the endless "can't grab frame" OpenCV spam
                // (already silenced above) and our own repeating
                // "monitoring..." log.
                VL_LOG(<< "[" << nowStr() << "] [VideoLink] grab failures/stalls starting on device "
                    << lastDeviceIndex_ << ", monitoring...");
            }
            ++consecutiveFailures;

            int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            int64_t lastGoodMs = lastFrameTimeMs_.load();
            int64_t sinceLastGoodFrame = (lastGoodMs == 0) ? 0 : (nowMs - lastGoodMs);
            bool timeBasedLoss = sinceLastGoodFrame >= kSignalLostTimeoutMs;

            if (consecutiveFailures >= kFailuresBeforeSignalLost || timeBasedLoss) {
                VL_LOG(<< "[" << nowStr() << "] [VideoLink] signal lost on device "
                    << lastDeviceIndex_ << " after " << consecutiveFailures
                    << " consecutive failed/stalled grabs (" << sinceLastGoodFrame
                    << "ms since last good frame) -- entering recovery mode");
                connected.store(false);
                setLinkState(linkState_, LinkState::SignalLost, "read-failure threshold tripped");
                deviceArrivalSignal_.store(false);   // discard any stale signal
                signalLostSince = std::chrono::steady_clock::now();
                burstUntil = std::chrono::steady_clock::time_point{};
                {
                    auto tRel = std::chrono::steady_clock::now();
                    cap.release();
                    VL_LOG(<< "[" << nowStr() << "] [VideoLink]   signal-lost: cap.release() took "
                        << elapsedMs(tRel) << "ms");
                }
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
                    setLinkState(linkState_, LinkState::NoVideoInput,
                        "idle/blank frame detected");
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
                    setLinkState(linkState_, LinkState::Connected, "real video reacquired");
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
        auto windowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - fpsWindowStart).count();
        if (windowMs >= 1000) {
            double fps = framesInWindow * 1000.0 / windowMs;
            measuredFps.store(fps);
            // Mean B/G/R of this frame -- concrete numbers to compare
            // against a color-picker reading of the same moment in OBS
            // (or against the frame's own dominant color from
            // idleColorFraction) instead of eyeballing screenshots.
            // cv::mean() is cheap and this only runs once/second.
            cv::Scalar meanColor = cv::mean(frame);
            VL_LOG(<< "[" << nowStr() << "] [VideoLink] fps heartbeat: " << fps
                << " fps (" << framesInWindow << " frames / " << windowMs
                << "ms), state=" << linkStateName(linkState_.load())
                << ", totalFrames=" << frameCount.load()
                << ", meanBGR=(" << static_cast<int>(meanColor[0]) << ","
                << static_cast<int>(meanColor[1]) << "," << static_cast<int>(meanColor[2]) << ")");
            framesInWindow = 0;
            fpsWindowStart = now;
        }
    }
    connected.store(false);
    setLinkState(linkState_, LinkState::Disconnected, "captureLoop() exiting (keepRunning=false)");
    VL_LOG(<< "[" << nowStr() << "] [VideoLink] captureLoop() thread exiting");
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