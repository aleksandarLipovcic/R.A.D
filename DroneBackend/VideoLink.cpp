#include "VideoLink.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>

// Media Foundation device enumeration (Windows-native device names +
// friendly indices -- this is what actually lets us tell "USB capture
// dongle" apart from "Integrated Webcam" before we ever open anything).
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")

VideoLink::VideoLink() {}
VideoLink::~VideoLink() { disconnect(); }

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

    // Fall back to trying everything if filtering removed all devices --
    // better to land on the laptop webcam than to refuse to connect at all.
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
// captureLoop()  —  runs on its own thread, decoupled entirely from Python
// =============================================================================

void VideoLink::captureLoop() {
    auto fpsWindowStart = std::chrono::steady_clock::now();
    uint64_t framesInWindow = 0;

    while (keepRunning.load()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            // Don't spin the CPU on a dropped USB frame, but keep the
            // retry window short -- signal can come back within a few ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            latestFrame = frame;   // header swap -- underlying buffer is a
            // fresh allocation from cap.read(), so
            // this is safe without an extra clone here
        }

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
    return latestFrame.clone();   // caller gets an independent copy across the pybind boundary
}