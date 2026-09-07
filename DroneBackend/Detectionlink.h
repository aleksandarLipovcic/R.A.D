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
#include <unordered_map>

// Forward declaration only -- DetectionLink never includes VideoLink.h.
// The relationship is the same one-way "give me a frame, I don't need
// to know what you are" decoupling VideoLink already uses toward
// DroneLink. See setVideoLinkSource() below.
class VideoLink;

// ── Telemetry snapshot ──────────────────────────────────────────────
//
// Everything the ranging methods (see computeWorldRay() in the .cpp)
// need to turn a pixel-space detection into
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

    // Identity assigned by the tracker in detectionLoop(), NOT a fresh
    // value per pass. Every DetectionRecord sharing the same trackId is
    // (as far as position/IoU-based tracking can tell) the SAME physical
    // object observed at a different moment -- see the "Object tracking
    // / re-identification" block below runInference() for how this is
    // assigned and its limitations (it is not appearance-based re-ID; an
    // object that leaves frame and comes back gets a new trackId). Use
    // this to group a pin's position history instead of assuming one
    // DetectionRecord == one physical sighting worth showing separately.
    uint64_t trackId = 0;

    // Pixel-space box in the source frame, kept so the screenshot can be
    // re-cropped or the ray re-derived later without re-running inference.
    int bboxX = 0, bboxY = 0, bboxW = 0, bboxH = 0;

    double latitude = 0.0;
    double longitude = 0.0;
    bool georeferenced = false;   // false if telemetry wasn't valid for this pass

    // How latitude/longitude above were derived, and how far away the
    // object was judged to be (meters). "ground_plane": ray/ground-plane
    // intersection using altitude + attitude + gimbal (accurate when the
    // gimbal is steeply downward and altitude is trustworthy, unstable
    // as the ray flattens toward the horizon). "object_size": apparent
    // pixel size vs. a configured real-world size for the class (works
    // at any elevation angle and doesn't need altitude at all, but is
    // only as good as the assumed real-world size and viewing angle).
    // Empty string if georeferenced is false.
    std::string rangeMethod;
    double distanceM = 0.0;       // 0 if not georeferenced
    double bearingDeg = 0.0;      // compass bearing from drone to object, 0 if not georeferenced

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

    // Path to an ONNX object-detection model. Tested against Ultralytics'
    // end-to-end/NMS-baked export layout used by YOLO26 (output shape
    // [1, maxDetections, 6] -- confidence/class already resolved, NMS
    // already applied inside the graph). See runInference() in the .cpp
    // for the exact column layout assumed and how to double check it
    // against your own export. If a sibling text file with the same
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

    // Square input side the ONNX model expects (letterboxed). MUST match
    // whatever imgsz the .onnx was exported at -- this does NOT have to
    // match training imgsz (you can train at 960 and export/infer at a
    // smaller size for speed), but it must match the export, or every
    // box coordinate coming out of runInference() will be silently wrong
    // (no crash -- just boxes and therefore georeferencing that are off
    // by a scale factor). Default 640; yolom_main_run was trained at
    // 960, so if you export at 960 call setInputSize(960) too.
    void setInputSize(int size) { inputSize_.store(size > 0 ? size : 640); }

    // Opt into a CUDA backend/target for inference (needs an OpenCV
    // build with CUDA support -- the stock pip/vcpkg opencv-python wheel
    // does NOT have this; you need a source build against your CUDA
    // toolkit). If unavailable or the build lacks CUDA support, start()
    // silently falls back to CPU rather than failing -- check
    // isUsingCuda() after start() to confirm which one you actually got.
    void setUseCuda(bool enabled) { useCuda_.store(enabled); }
    bool isUsingCuda() const { return usingCuda_.load(); }

    // Real-world width, in meters, of one object class as it would be
    // measured face-on (e.g. a person's shoulder width ~0.5m, a car's
    // width ~1.8m -- NOT length, since a car is usually seen more
    // side-on than head-on from above and width-across-frame is what the
    // bbox actually measures for most viewing angles). Only classes with
    // an entry here get object-size-based ranging; classes without one
    // fall back to ground-plane-only (or go ungeoreferenced if the ray
    // is too shallow for that either). Safe to call at runtime.
    void setKnownObjectWidth(const std::string& className, double widthMeters);
    void clearKnownObjectWidths();

    // Ray-downward-component threshold below which ground-plane
    // intersection is considered too unreliable to trust on its own
    // (ray nearly parallel to the ground -- small altitude/attitude
    // errors turn into huge position errors). Default 0.12 (~7 degrees
    // below horizontal). Below this, object-size ranging is preferred
    // when available for that class; if not available, ground-plane is
    // still used as a last resort rather than dropping the detection.
    void setMinGroundRayComponent(double v) { minGroundRayComponent_.store(v); }

    // ── Object tracking / re-identification (call before or during start()) ──
    //
    // Detection passes run every detectionIntervalMs_ (default 250ms) --
    // fast enough that the same physical object is almost always caught
    // on several consecutive passes while it's in frame. Without the
    // tracker below, that produced one DetectionRecord (and one saved
    // screenshot) PER PASS for what is visually a single sighting -- a
    // car sitting in frame for 10 seconds at 4 passes/sec turned into 40
    // near-identical records. The tracker below is a lightweight,
    // position-only re-identification: it matches each pass's raw boxes
    // against the previous pass's tracked boxes by IoU (+ same class),
    // NOT by any learned appearance embedding. That means:
    //   - it re-identifies an object continuously visible pass-to-pass
    //     reliably, since consecutive-pass boxes overlap heavily at 4Hz;
    //   - it CANNOT re-identify an object that leaves frame and comes
    //     back later, or two same-class objects that cross paths -- both
    //     get treated as distinct tracks (new trackId). True appearance
    //     re-ID would need an embedding model and is out of scope here;
    //     this is the "as close as possible" version with what
    //     runInference() already produces.
    // A matched, continuing track does NOT get a new DetectionRecord
    // every pass -- only when setTrackMoveThresholdM() worth of map
    // movement has accumulated, or setTrackRefreshIntervalMs() has
    // elapsed since its last record (so a long-dwelling static object
    // still gets a periodically refreshed confidence/timestamp/screenshot
    // rather than looking stale forever). A brand-new track always gets
    // an initial record immediately, same as today's behavior.

    // Minimum IoU (0-1) between a raw detection's box and a track's most
    // recently matched box, same class, for them to be considered the
    // same object. Default 0.3. Lower catches faster-moving objects at
    // the cost of more false merges between nearby same-class objects;
    // raise it if two separate objects passing near each other are
    // incorrectly being tracked as one.
    void setTrackIouThreshold(double v) { trackIouThreshold_.store(v); }

    // How many consecutive passes a track is allowed to go unmatched
    // (object briefly occluded, a missed detection, momentary confidence
    // dip below threshold) before it's dropped. Default 6 (~1.5s at the
    // default 250ms interval). Once dropped, the object reappearing
    // starts a brand new track/trackId -- see the re-ID limitation above.
    void setTrackMaxMissedPasses(int n) { trackMaxMissedPasses_.store(n); }

    // Minimum ground movement, in meters, between a track's last-recorded
    // position and its current one before a new DetectionRecord is
    // written for it. Default 3.0m. Only compared when both positions
    // are georeferenced; an ungeoreferenced continuing track falls back
    // to the refresh-interval trigger below instead.
    void setTrackMoveThresholdM(double m) { trackMoveThresholdM_.store(m); }

    // Even a perfectly static, continuously-tracked object gets a fresh
    // DetectionRecord (and screenshot) at least this often, in
    // milliseconds, so it doesn't silently vanish from "what's new"
    // views and its confidence/telemetry snapshot doesn't go stale.
    // Default 10000 (10s).
    void setTrackRefreshIntervalMs(int ms) { trackRefreshIntervalMs_.store(ms); }

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

    // ── Live preview (opt-in exception to "no pixels in Python") ─────

    // JPEG-encodes the most recent frame this link processed, with every
    // detection from that pass drawn on it. Empty vector if no pass has
    // completed yet. This is a deliberate, narrow exception to the "raw
    // pixel data never crosses into Python" rule the rest of the app
    // follows -- it exists only to feed an optional, low-rate (poll at
    // ~1-3Hz, not per-tick) "live detections" preview pane, and is NOT
    // the pilot's primary video path (that stays on VideoLink/FPVWidget
    // exactly as before). Costs one JPEG encode per call.
    std::vector<uchar> getLatestAnnotatedFrameJpeg() const;

private:
    void detectionLoop();

    struct RawDetection {
        int classIndex = -1;
        float confidence = 0.0f;
        int x = 0, y = 0, w = 0, h = 0;   // pixel space, original frame
    };

    std::vector<RawDetection> runInference(const cv::Mat& frame);

    // Result of a single ranging attempt -- kept separate from the
    // simpler georeference() this replaces so ground-plane and
    // object-size distances can each be computed and compared instead
    // of the first one found silently winning.
    struct RangeEstimate {
        bool valid = false;
        double lat = 0.0, lon = 0.0;
        double distanceM = 0.0;
        double bearingDeg = 0.0;
        std::string method;   // "ground_plane" or "object_size"
    };

    // Builds the world-space (NED) unit ray for one detection: composes
    // the pixel offset + horizontal FOV into a camera-space ray, then
    // gimbal pan/tilt, then body roll/pitch/heading. Shared by both
    // ranging methods below so they always agree on direction and only
    // differ on how far along that direction the object actually is.
    bool computeWorldRay(const RawDetection& raw, int frameW, int frameH,
        const TelemetrySnapshot& telemetry, double& outUnitX, double& outUnitY, double& outUnitZ) const;

    // Ray/ground-plane intersection: scales the ray until its down (Z)
    // component covers the drone's AGL altitude. Flat-earth assumption
    // -- fine at the few-km ranges this is designed for; swap in a DEM
    // lookup here later if slope error matters. Degrades as the ray
    // flattens toward the horizon (small unitZ means a tiny altitude
    // error turns into a huge range error) -- see
    // setMinGroundRayComponent().
    RangeEstimate rangeByGroundPlane(const TelemetrySnapshot& telemetry,
        double unitX, double unitY, double unitZ) const;

    // Apparent-size ranging: distance = (known real width in meters *
    // focal length in pixels) / (bounding box width in pixels), using
    // the same pinhole model implied by horizontalFovDeg. Independent of
    // altitude entirely, and works at any gimbal angle including
    // straight-ahead FPV shots where ground-plane intersection can't
    // return a result at all. Only available for classes registered via
    // setKnownObjectWidth(); accuracy is bounded by how close the real
    // object is to the assumed width and how face-on it's viewed.
    RangeEstimate rangeByObjectSize(const RawDetection& raw, int frameW,
        const TelemetrySnapshot& telemetry, double unitX, double unitY, double unitZ) const;

    std::string saveScreenshot(const cv::Mat& frame, const RawDetection& raw, uint64_t id) const;
    std::string classNameFor(int classIndex) const;

    // Bundles the outcome of running both ranging methods for one raw
    // detection -- computed ONCE per raw detection per pass, up front,
    // then used both to decide (in updateTracks(), via move-threshold
    // comparison against a track's last recorded position) whether this
    // sighting is "new" enough to log, and -- if so -- to fill in the
    // DetectionRecord without re-deriving the ray/ranging a second time.
    struct GeoCandidate {
        bool georeferenced = false;
        double lat = 0.0, lon = 0.0;
        double distanceM = 0.0;
        double bearingDeg = 0.0;
        std::string method;
    };
    GeoCandidate computeGeoCandidate(const RawDetection& raw, int frameW, int frameH,
        const TelemetrySnapshot& telemetry) const;

    // ── Tracking state (see setTrackIouThreshold() etc. above) ────────
    // Owned and touched ONLY by the worker thread inside detectionLoop()
    // -- unlike records_/lastAnnotatedFrame_, this is never read from
    // Python or any other thread, so it needs no mutex of its own.
    struct Track {
        uint64_t trackId = 0;
        int classIndex = -1;
        int lastX = 0, lastY = 0, lastW = 0, lastH = 0;  // last matched pixel-space box
        bool lastGeoreferenced = false;
        double lastLat = 0.0, lastLon = 0.0;             // position at the last WRITTEN record
        int64_t lastSeenMs = 0;       // last pass this track was matched
        int64_t lastRecordMs = 0;     // last pass a DetectionRecord was written for this track
        int missedPasses = 0;
    };
    std::vector<Track> tracks_;
    uint64_t nextTrackId_ = 1;

    // IoU between a raw detection's box and a track's last matched box.
    static double trackIou(const RawDetection& raw, const Track& track);

    // Matches this pass's raw detections against tracks_ (greedy,
    // highest-IoU-first, same class required), updates tracks_ in place,
    // ages out/removes tracks that exceeded trackMaxMissedPasses_, and
    // returns, for each raw detection (by index into rawDetections,
    // parallel array), the trackId it was assigned (existing or freshly
    // created) plus whether a new DetectionRecord should be written for
    // it this pass. geoCandidates must be the same size as
    // rawDetections (index-parallel) -- see setTrackMoveThresholdM() /
    // setTrackRefreshIntervalMs() for what "should [record]" means for a
    // continuing (already-tracked) detection; a brand new track is
    // always recorded.
    struct TrackAssignment {
        uint64_t trackId = 0;
        bool shouldRecord = false;
    };
    std::vector<TrackAssignment> updateTracks(
        const std::vector<RawDetection>& rawDetections,
        const std::vector<GeoCandidate>& geoCandidates,
        int64_t nowMs);

    std::atomic<double> trackIouThreshold_{ 0.3 };
    std::atomic<int> trackMaxMissedPasses_{ 6 };
    std::atomic<double> trackMoveThresholdM_{ 3.0 };
    std::atomic<int> trackRefreshIntervalMs_{ 10000 };

    std::function<cv::Mat()> frameSource_;
    std::function<TelemetrySnapshot()> telemetryProvider_;
    std::string modelPath_;
    std::string screenshotDir_;
    std::vector<std::string> classNames_;

    std::atomic<int> detectionIntervalMs_{ 250 };     // see kDefaultDetectionIntervalMs
    std::atomic<float> confidenceThreshold_{ 0.4f };
    std::atomic<double> horizontalFovDeg_{ 90.0 };
    std::atomic<int> inputSize_{ 640 };
    std::atomic<bool> useCuda_{ false };
    std::atomic<bool> usingCuda_{ false };
    std::atomic<double> minGroundRayComponent_{ 0.12 };

    mutable std::mutex classWidthsMutex_;
    std::unordered_map<std::string, double> knownObjectWidthsM_;

    std::thread worker_;
    std::atomic<bool> running_{ false };
    std::atomic<uint64_t> detectionCount_{ 0 };
    std::atomic<double> lastPassDurationMs_{ 0.0 };
    std::atomic<int64_t> lastPassTimestampMs_{ 0 };

    mutable std::mutex recordsMutex_;
    std::vector<DetectionRecord> records_;
    uint64_t nextId_ = 1;

    mutable std::mutex frameMutex_;
    cv::Mat lastAnnotatedFrame_;   // most recent pass, all boxes drawn

    // Opaque inference-runtime state (cv::dnn::Net) -- kept out of the
    // public interface so DetectionLink.h stays cheap to include from
    // files that never touch inference directly.
    struct InferenceImpl;
    std::unique_ptr<InferenceImpl> impl_;
};