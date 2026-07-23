#include "VideoLink.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>

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

VideoLink::VideoLink() {}
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
    return false;
}

bool VideoLink::connect(int deviceIndex) {
    disconnect();

    cap.open(deviceIndex, cv::CAP_MSMF);
    if (!cap.isOpened())
        cap.open(deviceIndex, cv::CAP_DSHOW);   // fallback backend
    if (!cap.isOpened())
        return false;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, prefW);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, prefH);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);   // no internal queue -- always freshest frame
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    cv::Mat testFrame;
    if (!cap.read(testFrame) || testFrame.empty()) {
        cap.release();
        return false;
    }

    connected.store(true);
    keepRunning.store(true);
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
}

// =============================================================================
// Native window rendering
// =============================================================================

static LRESULT CALLBACK VideoLinkWndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    // We paint from the capture thread via GetDC() outside of WM_PAINT,
    // so this proc just needs to suppress the default background erase
    // (which would otherwise cause visible flicker between frames) and
    // defer everything else to Windows' default handling.
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
    // point spending a GetDC/StretchDIBits/ReleaseDC round trip on a
    // window Windows isn't compositing anyway, and this also guarantees
    // the window can never appear to "come back" mid-frame while hidden.
    if (!windowVisible_.load())
        return;

    HWND hwnd = renderHwnd_.load();
    if (!hwnd || frame.empty())
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

    HDC hdc = GetDC(hwnd);
    if (!hdc)
        return;
    SetStretchBltMode(hdc, HALFTONE);
    // HALFTONE mode ignores the brush origin unless it's set explicitly --
    // omitting this line is a common source of a 1px vertical misalignment.
    SetBrushOrgEx(hdc, 0, 0, nullptr);

    StretchDIBits(hdc,
        0, 0, destW, destH,
        0, 0, frame.cols, frame.rows,
        frame.data, &bmi, DIB_RGB_COLORS, SRCCOPY);

    ReleaseDC(hwnd, hdc);
}

// =============================================================================
// captureLoop()  —  runs on its own thread, decoupled entirely from Python
// =============================================================================

void VideoLink::captureLoop() {
    auto fpsWindowStart = std::chrono::steady_clock::now();
    uint64_t framesInWindow = 0;

    while (keepRunning.load()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            latestFrame = frame;   // header swap -- see original comment
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
}

cv::Mat VideoLink::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frameMutex);
    return latestFrame.clone();
}