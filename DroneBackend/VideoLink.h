#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>

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

    void captureLoop();
    static bool looksLikeIntegratedWebcam(const std::string& name);
};