#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <cstdint>

// Forward declaration only -- DetectionLink never includes VideoLink.h.
// The relationship is the same one-way "give me a frame, I don't need
// to know what you are" decoupling VideoLink already uses toward
// DroneLink. See setVideoLinkSource() below.
class VideoLink;

// ── Telemetry snapshot ──────────────────────────────────────────────
//
// Everything georeference() needs to turn a pixel-space detection into
// a lat/lon. DetectionLink has zero knowledge of DroneLink, MSP, or any
// specific telemetry source -- the application wires this in via
// setTelemetryProvider(). This mirrors VideoLink's own decoupling from
// DroneLink: swap the telemetry source without DetectionLink.h/.cpp
// ever changing.
//
// Angle conventions (fix these against your own DroneLink/gimbal code
// before trusting output -- get this wrong and every pin lands offset
// in a consistent, easy-to-miss way, the same class of bug the
// atan2(y,x)->atan2(x,y) magnetometer swap was):
//   headingDeg    : compass heading, 0 = north, increasing clockwise
//   rollDeg       : positive = right wing down
//   pitchDeg      : positive = nose up
//   gimbalPanDeg  : relative to body forward, positive = pan right
//   gimbalTiltDeg : 0 = forward/level, +90 = straight down
struct TelemetrySnapshot {
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitudeM = 0.0;       // height above the ground directly below (AGL)
    double headingDeg = 0.0;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double gimbalPanDeg = 0.0;
    double gimbalTiltDeg = 0.0;
};

// One detected + georeferenced object -- this is what becomes a pin.
struct DetectionRecord {
    uint64_t id = 0;
    int64_t timestampMs = 0;      // wall clock ms, epoch, when the frame was grabbed
    std::string className;
    float confidence = 0.0f;

    // Pixel-space box in the source frame, kept so the screenshot can be
    // re-cropped or the ray re-derived later without re-running inference.
    int bboxX = 0, bboxY = 0, bboxW = 0, bboxH = 0;

    double latitude = 0.0;
    double longitude = 0.0;
    bool georeferenced = false;   // false if telemetry wasn't valid for this pass

    std::string screenshotPath;   // empty if screenshot saving is disabled/failed
    TelemetrySnapshot telemetry;  // stored verbatim, for later re-derivation/debugging
};

class DetectionLink {
public:
    DetectionLink();
    ~DetectionLink();

    // ── Wiring (decoupled sources) ───────────────────────────────────

    // Preferred: hands DetectionLink a raw VideoLink pointer so every
    // detection pass pulls a frame via VideoLink::getLatestFrame()
    // directly in C++ -- pixel data never crosses into Python, same
    // rule your capture path already follows. DetectionLink does not
    // take ownership and does not call anything on VideoLink except
    // getLatestFrame().
    void setVideoLinkSource(VideoLink* videoLink);

    // Escape hatch for testing, or for a frame source that isn't a
    // VideoLink (e.g. a recorded file during post-flight replay).
    void setFrameSource(std::function<cv::Mat()> frameSource);

    // Telemetry provider -- called once per detection pass, immediately
    // before inference, so the snapshot is as time-close to the frame
    // as practical. Wire this from wherever DroneLink's telemetry
    // already lives on the C++ side; if telemetry is only reachable
    // from Python today, bind a small pybind11 trampoline instead (see
    // the bindings snippet).
    void setTelemetryProvider(std::function<TelemetrySnapshot()> telemetryProvider);

    // ── Configuration (call before start()) ───────────────────────────

    // Path to an ONNX object-detection model (tested against Ultralytics
    // YOLOv8/v11 export layout -- see runInference() in the .cpp for the
    // exact output shape assumed). If a sibling text file with the same
    // stem and a ".names" extension exists (one class name per line), it
    // is loaded automatically; otherwise classes are labeled "class_N".
    void setModelPath(const std::string& path);

    // Directory detection screenshots are written to (created if
    // missing). Leave empty to disable screenshot saving -- detections
    // are still recorded, just without an image.
    void setScreenshotDir(const std::string& dir);

    // Horizontal field of view of the camera feeding this link, in
    // degrees. Needed to turn a pixel offset into a ray angle for
    // georeferencing -- get this from the same camera datasheet you
    // used for the FOV assumptions in VideoLink's capture config.
    void setHorizontalFovDeg(double fovDeg) { horizontalFovDeg_.store(fovDeg); }

    // How often a detection pass runs, independent of the 30fps capture
    // loop -- see kDefaultDetectionIntervalMs in the .cpp. Safe to
    // change at runtime.
    void setDetectionIntervalMs(int ms) { detectionIntervalMs_.store(ms); }

    // Raw model confidence below which a detection is discarded before
    // it's ever turned into a DetectionRecord.
    void setConfidenceThreshold(float t) { confidenceThreshold_.store(t); }

    // ── Lifecycle ──────────────────────────────────────────────────

    // Loads the model and starts the worker thread. Returns false (and
    // does not start the thread) if the model failed to load or no
    // frame source has been set.
    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }

    // ── Cheap, atomic status -- safe to poll from Python at UI rate ──
    uint64_t getDetectionCount() const { return detectionCount_.load(); }
    double getLastPassDurationMs() const { return lastPassDurationMs_.load(); }
    int64_t getLastPassTimestampMs() const { return lastPassTimestampMs_.load(); }

    // ── Detection records ──────────────────────────────────────────

    // Full copy of every record collected since start() (or since
    // clearRecords()). Mutex-protected, same pattern as
    // VideoLink::getLatestFrame() -- fine to call at UI refresh rate
    // (a few Hz), not meant to be called every frame.
    std::vector<DetectionRecord> getAllRecords() const;

    // Only records with id > sinceId, so the UI can poll incrementally
    // instead of re-rendering the whole pin list every tick. Pass 0 to
    // get everything.
    std::vector<DetectionRecord> getRecordsSince(uint64_t sinceId) const;

    void clearRecords();

private:
    void detectionLoop();

    struct RawDetection {
        int classIndex = -1;
        float confidence = 0.0f;
        int x = 0, y = 0, w = 0, h = 0;   // pixel space, original frame
    };

    std::vector<RawDetection> runInference(const cv::Mat& frame);

    // Composes camera ray (from pixel offset + horizontal FOV) with
    // gimbal pan/tilt, body roll/pitch, and heading, then intersects the
    // resulting world-space ray with a flat ground plane altitudeM below
    // the drone. Flat-earth assumption -- fine at the few-km ranges this
    // is designed for; swap in a DEM lookup here later if slope error
    // matters for your use case. Returns false if telemetry.valid is
    // false or the ray points above the horizon (shouldn't happen for a
    // downward-biased gimbal, but a bug upstream could produce one).
    bool georeference(const RawDetection& raw, int frameW, int frameH,
        const TelemetrySnapshot& telemetry, double& outLat, double& outLon) const;

    std::string saveScreenshot(const cv::Mat& frame, const RawDetection& raw, uint64_t id) const;
    std::string classNameFor(int classIndex) const;

    std::function<cv::Mat()> frameSource_;
    std::function<TelemetrySnapshot()> telemetryProvider_;
    std::string modelPath_;
    std::string screenshotDir_;
    std::vector<std::string> classNames_;

    std::atomic<int> detectionIntervalMs_{ 250 };     // see kDefaultDetectionIntervalMs
    std::atomic<float> confidenceThreshold_{ 0.4f };
    std::atomic<double> horizontalFovDeg_{ 90.0 };

    std::thread worker_;
    std::atomic<bool> running_{ false };
    std::atomic<uint64_t> detectionCount_{ 0 };
    std::atomic<double> lastPassDurationMs_{ 0.0 };
    std::atomic<int64_t> lastPassTimestampMs_{ 0 };

    mutable std::mutex recordsMutex_;
    std::vector<DetectionRecord> records_;
    uint64_t nextId_ = 1;

    // Opaque inference-runtime state (cv::dnn::Net) -- kept out of the
    // public interface so DetectionLink.h stays cheap to include from
    // files that never touch inference directly.
    struct InferenceImpl;
    std::unique_ptr<InferenceImpl> impl_;
};