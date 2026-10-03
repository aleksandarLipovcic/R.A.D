// Must come before ANY #include -- both of these only take effect if
// defined before the header that would otherwise omit/macro-clobber the
// symbols below is first pulled in (directly or transitively, e.g. via
// VideoLink.h -> ... -> <windows.h>):
//   _USE_MATH_DEFINES -- MSVC's <cmath>/<math.h> only define M_PI (and
//                         the other math constants) when this is set;
//                         without it, M_PI below is simply undeclared
//                         (MSVC error C2065).
//   NOMINMAX          -- <windows.h> defines `min`/`max` as raw
//                         preprocessor macros unless this is set, which
//                         silently swallows the `min`/`max` tokens out of
//                         every std::min(...)/std::max(...) call in this
//                         translation unit -- e.g. std::max(0.0f, x1)
//                         becomes std::(0.0f, x1), hence MSVC error
//                         C2589 ("illegal token on right side of '::'")
//                         at the std::min/std::max call sites below.
#define _USE_MATH_DEFINES
#define NOMINMAX

#include "DetectionLink.h"
#include "VideoLink.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <cctype>
#include <map>

#ifdef _WIN32
#include <windows.h>
#endif

// ── Verbose diagnostic logging ──────────────────────────────────────
// Same convention as VIDEOLINK_VERBOSE_LOGGING in VideoLink.cpp: every
// [DetectionLink][perf]/[DetectionLink] DEBUG line below (preview-thread
// startup notice, per-tick preview/inference timing, the one-time raw
// model-space row dump, and the JPEG-encode timing in
// getLatestAnnotatedFrameJpeg()) is compiled out by default now that
// CUDA is confirmed working and these aren't needed for normal runs.
// Set DETECTIONLINK_VERBOSE_LOGGING to 1 (or define it via a build-system
// flag) to bring all of them back exactly as they were, e.g. when
// diagnosing a new model export, a new capture device, or a perf
// regression. This does NOT affect the one-time cv::Exception warning
// in runInference() below (search "a detection pass failed") -- that's
// a genuine error surfaced to stderr regardless of this flag, not a
// diagnostic print, since silencing it would hide real failures.
#ifndef DETECTIONLINK_VERBOSE_LOGGING
#define DETECTIONLINK_VERBOSE_LOGGING 0
#endif

#if DETECTIONLINK_VERBOSE_LOGGING
#define DL_LOG(...) do { std::cout __VA_ARGS__ << std::endl; } while (0)
#define DL_LOG_ERR(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#else
#define DL_LOG(...) do { } while (0)
#define DL_LOG_ERR(...) do { } while (0)
#endif

namespace {
    // Detection does not need anywhere near 30fps -- it exists to give
    // the pilot supplementary situational awareness, not to drive the
    // live view. 250ms (4 passes/sec) is a starting point: fast enough
    // that a car or person crossing the frame gets caught, slow enough
    // that inference cost never competes with the capture thread for
    // CPU. Tune per your actual model's inference time -- if a single
    // pass takes 150ms on your hardware, going below ~300ms just means
    // back-to-back passes with no idle time, which is fine but worth
    // knowing.
    constexpr int kDefaultDetectionIntervalMs = 250;

    // Cap on how many past BearingObservations a single object carries
    // (see ObjectState::bearingHistory in the header). Triangulation only ever
    // needs a handful of well-spread observations -- keeping every
    // sighting for a track's whole lifetime would grow unbounded for a
    // long-dwelling object and cost more per-pass compute (every kept
    // observation is one more line in the least-squares solve) for no
    // accuracy benefit past a point. Oldest is dropped first once this
    // is exceeded.
    constexpr size_t kMaxBearingHistoryPerTrack = 12;

    // Ground-plane ranging needs a real height above the take-off point
    // and a ray that is not nearly level (see rangeByGroundPlane()).
    constexpr double kMinGroundPlaneAltitudeM = 2.0;
    constexpr double kMaxGroundPlaneRangeM = 400.0;

    // Object position fusion (see ObjectState in the header): older
    // sightings fade with this half-life, so a moving object's pin
    // follows it and a static one keeps converging.
    constexpr double kFusionHalfLifeS = 15.0;

    constexpr double kEarthRadiusM = 6378137.0;

    double degToRad(double d) { return d * M_PI / 180.0; }
    double radToDeg(double r) { return r * 180.0 / M_PI; }

    // Minimal 3x3 matrix / 3-vector helpers -- deliberately not pulling
    // in Eigen just for this. If Eigen is already a dependency elsewhere
    // in the project, swap these for Eigen::Matrix3d/Vector3d with no
    // change to the math itself.
    struct Vec3 { double x, y, z; };
    struct Mat3 { double m[3][3]; };

    Vec3 matVec(const Mat3& a, const Vec3& v) {
        return {
            a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
            a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z,
        };
    }
    Mat3 matMul(const Mat3& a, const Mat3& b) {
        Mat3 r{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    r.m[i][j] += a.m[i][k] * b.m[k][j];
        return r;
    }
    // Rotation about Z (heading/yaw), world frame convention: X=north,
    // Y=east, Z=down (NED). Positive yaw rotates north toward east,
    // matching compass heading increasing clockwise.
    Mat3 rotZ(double deg) {
        double r = degToRad(deg), c = std::cos(r), s = std::sin(r);
        return { { { c, -s, 0 }, { s, c, 0 }, { 0, 0, 1 } } };
    }
    // Rotation about Y (pitch), positive pitch = nose up = camera tilts
    // toward the sky, so this is a negative rotation of the forward axis
    // toward -Z (up) in NED.
    Mat3 rotY(double deg) {
        double r = degToRad(deg), c = std::cos(r), s = std::sin(r);
        return { { { c, 0, s }, { 0, 1, 0 }, { -s, 0, c } } };
    }
    // Rotation about X (roll), positive roll = right wing down.
    Mat3 rotX(double deg) {
        double r = degToRad(deg), c = std::cos(r), s = std::sin(r);
        return { { { 1, 0, 0 }, { 0, c, -s }, { 0, s, c } } };
    }
}

// Opaque inference state -- cv::dnn::Net kept out of the header.
struct DetectionLink::InferenceImpl {
    cv::dnn::Net net;
    bool loaded = false;
};

DetectionLink::DetectionLink() : impl_(std::make_unique<InferenceImpl>()) {}
DetectionLink::~DetectionLink() { stop(); }

void DetectionLink::setVideoLinkSource(VideoLink* videoLink) {
    if (!videoLink) {
        frameSource_ = nullptr;
        return;
    }
    // Captured by pointer, not by value -- DetectionLink does not own
    // VideoLink and does not outlive the caller's responsibility to
    // call stop() before destroying the VideoLink instance.
    frameSource_ = [videoLink]() { return videoLink->getLatestFrame(); };
}

void DetectionLink::setFrameSource(std::function<cv::Mat()> frameSource) {
    frameSource_ = std::move(frameSource);
}

void DetectionLink::setTelemetryProvider(std::function<TelemetrySnapshot()> telemetryProvider) {
    telemetryProvider_ = std::move(telemetryProvider);
}

void DetectionLink::setModelPath(const std::string& path) {
    modelPath_ = path;

    // Auto-load a sibling ".names" file if present: "yolov8n.onnx" ->
    // "yolov8n.names", one class name per line, index == line number.
    classNames_.clear();
    std::filesystem::path p(path);
    p.replace_extension(".names");
    std::ifstream namesFile(p);
    if (namesFile.is_open()) {
        std::string line;
        while (std::getline(namesFile, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            classNames_.push_back(line);
        }
    }
}

void DetectionLink::setScreenshotDir(const std::string& dir) {
    screenshotDir_ = dir;
    if (!dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
    }
}

bool DetectionLink::start() {
    if (running_.load())
        return true;
    if (!frameSource_ || modelPath_.empty())
        return false;

    try {
        impl_->net = cv::dnn::readNetFromONNX(modelPath_);
    }
    catch (const cv::Exception&) {
        return false;
    }
    if (impl_->net.empty())
        return false;
    impl_->loaded = true;

    // CPU by default. If setUseCuda(true) was called, try CUDA first --
    // this only actually works if OpenCV itself was built with CUDA/cuDNN
    // support (the stock opencv-python wheel is NOT; you need a source
    // build against your installed CUDA toolkit). readNetFromONNX
    // succeeding above says nothing about that -- the failure mode here
    // is setPreferableTarget silently doing nothing or forward() throwing
    // on first use, so we do a tiny dummy forward() to actually prove the
    // CUDA path works before committing to it, and fall back to CPU
    // (which always works) rather than leaving the app dead in the water
    // on a machine/build without CUDA DNN support.
    usingCuda_.store(false);
    usingCudaFp16_.store(false);
    if (useCuda_.load()) {
        // A dummy forward() is the only real proof a target works (see
        // above). FP16 is tried first when enabled: on RTX-class GPUs it is
        // typically 1.5-2x faster with practically identical detections;
        // its output is also checked for NaN/Inf, which a model with
        // FP16-overflowing layers would produce, before it is trusted.
        auto probe = [&](int target) -> bool {
            try {
                impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                impl_->net.setPreferableTarget(target);
                cv::Mat dummy(inputSize_.load(), inputSize_.load(), CV_8UC3, cv::Scalar(114, 114, 114));
                cv::Mat blob = cv::dnn::blobFromImage(dummy, 1.0 / 255.0,
                    cv::Size(inputSize_.load(), inputSize_.load()), cv::Scalar(), true, false);
                impl_->net.setInput(blob);
                cv::Mat out = impl_->net.forward();
                return !out.empty() && cv::checkRange(out);
            }
            catch (const cv::Exception&) {
                return false;
            }
        };
        if (useCudaFp16_.load() && probe(cv::dnn::DNN_TARGET_CUDA_FP16)) {
            usingCuda_.store(true);
            usingCudaFp16_.store(true);
        }
        else if (probe(cv::dnn::DNN_TARGET_CUDA)) {
            usingCuda_.store(true);
        }
        else {
            impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
    }
    else {
        impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    // Tracker: fresh state per start() (trackIds keep counting up, so an
    // id is never reused within a session). Vehicle classes the model
    // confuses with each other form one group -- a car detected as
    // other_vehicle on the next pass is still the same object.
    {
        std::map<int, int> groups;
        for (size_t i = 0; i < classNames_.size(); ++i) {
            std::string n = classNames_[i];
            std::transform(n.begin(), n.end(), n.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (n == "person" || n == "people" || n == "pedestrian")
                groups[static_cast<int>(i)] = 0;
            else if (n.find("car") != std::string::npos || n.find("vehicle") != std::string::npos ||
                     n.find("truck") != std::string::npos || n.find("bus") != std::string::npos ||
                     n.find("motor") != std::string::npos || n.find("van") != std::string::npos)
                groups[static_cast<int>(i)] = 1;
        }
        tracker_.setClassGroups(groups);
        tracker_.reset();
        objects_.clear();
        archive_.clear();
    }

    running_.store(true);
    previewWorker_ = std::thread(&DetectionLink::previewLoop, this);
    inferenceWorker_ = std::thread(&DetectionLink::inferenceLoop, this);
    return true;
}

void DetectionLink::stop() {
    running_.store(false);
    if (previewWorker_.joinable())
        previewWorker_.join();
    if (inferenceWorker_.joinable())
        inferenceWorker_.join();
}

void DetectionLink::previewLoop() {
    // Was a TEMPORARY "which .pyd actually loaded" diagnostic during the
    // CUDA bring-up; kept but now gated behind DETECTIONLINK_VERBOSE_LOGGING
    // rather than deleted, since it's still the fastest way to catch a
    // stale/locked .pyd not actually being replaced by a rebuild. Re-enable
    // the macro above if a "rebuild" ever appears to have no effect.
    DL_LOG(<< "[DetectionLink] preview thread active (kPreviewTickMs=40); "
        "inference thread runs independently (detectionIntervalMs="
        << detectionIntervalMs_.load() << ")");

    // Deliberately genuinely lightweight (frame grab + draw + JPEG-free
    // clone/rectangle/putText, no model, no georeferencing, no mutex
    // contention beyond boxesMutex_/frameMutex_ which inferenceLoop only
    // holds briefly) so this thread can actually sustain kPreviewTickMs
    // regardless of what inferenceLoop is doing. Runs BELOW_NORMAL, like
    // inferenceLoop, and idles entirely while nobody polls the preview.
    constexpr int kPreviewTickMs = 40;   // ~25fps ceiling for the live preview pane
    constexpr int64_t kPreviewIdleAfterMs = 2000;   // nobody polled the preview -> idle
#ifdef _WIN32
    // Operator convenience only -- the pilot's view is VideoLink's own
    // ABOVE_NORMAL capture/paint path. Below normal so it always yields.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif

    // Draws `boxes` onto a clone of `frame` and publishes it as the live
    // preview.
    auto publishPreviewFrame = [this](const cv::Mat& frame, const std::vector<RawDetection>& boxes) {
        cv::Mat annotated = frame.clone();
        for (const auto& raw : boxes) {
            cv::rectangle(annotated, cv::Rect(raw.x, raw.y, raw.w, raw.h), cv::Scalar(0, 220, 255), 2);
            std::string label = classNameFor(raw.classIndex) + " " +
                std::to_string(static_cast<int>(raw.confidence * 100)) + "%";
            cv::putText(annotated, label, cv::Point(raw.x, std::max(0, raw.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 220, 255), 1);
        }
        std::lock_guard<std::mutex> lock(frameMutex_);
        lastAnnotatedFrame_ = annotated;
        };

    // Sleeps off whatever time is left in this tick's kPreviewTickMs
    // budget. If the tick itself (frame grab + draw) already overran the
    // budget, loops straight back around instead of sleeping negative
    // time.
    auto throttleToTick = [](std::chrono::steady_clock::time_point tickStart, int tickBudgetMs) {
        double tickMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - tickStart).count();
        int sleepMs = tickBudgetMs - static_cast<int>(tickMs);
        if (sleepMs > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        };

    // ── TEMPORARY perf-diagnostic counters ────────────────────────────
    // Printed once every kDebugPrintEveryTicks ticks (~1s at the 40ms
    // target). boxes_age_ms is the important new number here: how stale
    // the overlay is relative to wall-clock "now" -- this should track
    // roughly detectionIntervalMs_ (plus one inference-pass duration) if
    // everything is healthy, and should NOT keep climbing unbounded; if
    // it does, inferenceLoop has fallen permanently behind. Delete this
    // whole block once the bottleneck is found and fixed.
    constexpr int kDebugPrintEveryTicks = 25;
    int dbgTickCounter = 0;
    double dbgGrabMsAccum = 0.0, dbgPublishMsAccum = 0.0, dbgTotalMsAccum = 0.0;
    int dbgEmptyFrameTicksInWindow = 0;

    while (running_.load()) {
        auto tickStart = std::chrono::steady_clock::now();

        // No frame copies / drawing while the preview pane is closed: a
        // full-resolution clone 25x per second is pure load otherwise.
        {
            const int64_t wallNow = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (wallNow - lastPreviewRequestMs_.load() > kPreviewIdleAfterMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        auto dbgGrabStart = std::chrono::steady_clock::now();
        cv::Mat frame = frameSource_ ? frameSource_() : cv::Mat();
        double dbgGrabMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - dbgGrabStart).count();
        dbgGrabMsAccum += dbgGrabMs;

        if (frame.empty()) {
            dbgEmptyFrameTicksInWindow++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::vector<RawDetection> boxes;
        {
            std::lock_guard<std::mutex> lock(boxesMutex_);
            boxes = latestBoxes_;   // whatever inferenceLoop most recently finished, if anything
        }

        auto dbgPublishStart = std::chrono::steady_clock::now();
        publishPreviewFrame(frame, boxes);
        double dbgPublishMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - dbgPublishStart).count();
        dbgPublishMsAccum += dbgPublishMs;
        dbgTotalMsAccum += std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - tickStart).count();

        if (++dbgTickCounter >= kDebugPrintEveryTicks) {
            // Gated behind DETECTIONLINK_VERBOSE_LOGGING -- see macro def
            // near the top of this file. Still tracked/reset every window
            // regardless (the accumulators cost nothing meaningful), only
            // the print itself is compiled out.
            int64_t nowWallMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            int64_t lastInfMs = lastInferenceCompletedAtMs_.load();
            DL_LOG(<< "[DetectionLink][perf][preview] "
                << "avg_grab=" << (dbgGrabMsAccum / dbgTickCounter) << "ms "
                << "avg_publish=" << (dbgPublishMsAccum / dbgTickCounter) << "ms "
                << "avg_tick=" << (dbgTotalMsAccum / dbgTickCounter) << "ms "
                << "(target=" << kPreviewTickMs << "ms, ~"
                << (1000.0 / std::max(0.001, dbgTotalMsAccum / dbgTickCounter)) << "fps) "
                << "boxes_age_ms=" << (lastInfMs > 0 ? (nowWallMs - lastInfMs) : -1)
                << " empty_frames=" << dbgEmptyFrameTicksInWindow);
            dbgTickCounter = 0; dbgGrabMsAccum = dbgPublishMsAccum = dbgTotalMsAccum = 0.0;
            dbgEmptyFrameTicksInWindow = 0;
        }

        throttleToTick(tickStart, kPreviewTickMs);
    }
}

void DetectionLink::inferenceLoop() {
#ifdef _WIN32
    // Deliberately the mirror image of VideoLink's capture thread, which
    // raises itself to ABOVE_NORMAL. This is the thread doing the actual
    // model forward pass -- the heavy CPU consumer -- so it, not
    // previewLoop, is the one that must never win a scheduling contest
    // against the thread painting the pilot's live feed.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif

    // Runs independently of previewLoop entirely: grabs its own frame,
    // runs the model + tracker + record-writing at detectionIntervalMs_
    // cadence (or however long a pass actually takes, if that's longer
    // than the interval -- see the back-to-back-passes note on
    // kDefaultDetectionIntervalMs), and hands off only the resulting
    // boxes to previewLoop via boxesMutex_/latestBoxes_. Critically,
    // previewLoop can NEVER be blocked by this loop no matter how slow a
    // given pass is -- a multi-second forward() call (e.g. an
    // unoptimized Debug build, or CPU-only inference at a large
    // inputSize) now only means the overlay goes stale for that long, it
    // does NOT freeze the live pane itself the way the old single-thread
    // version did.
    auto lastInferenceAt = std::chrono::steady_clock::now() -
        std::chrono::milliseconds(detectionIntervalMs_.load());

    // ── TEMPORARY perf-diagnostic counters ────────────────────────────
    // Printed once every kDebugPrintEveryPasses passes -- a small number
    // (not time-based) since a single pass can legitimately take
    // anywhere from milliseconds (CUDA, small inputSize, Release build)
    // to multiple seconds (CPU, large inputSize, Debug build), and we
    // still want feedback at a reasonable cadence either way. Delete
    // this whole block once the bottleneck is found and fixed.
    constexpr int kDebugPrintEveryPasses = 5;
    int dbgPassCounter = 0;
    double dbgInferMsAccum = 0.0, dbgPassMsAccum = 0.0;
    int dbgRecordsAccum = 0;

    // Duty-cycle limit (setMaxDutyCycle): earliest start of the next pass.
    auto restUntil = std::chrono::steady_clock::now();
    int slowTiledPasses = 0;

    while (running_.load()) {
        auto now = std::chrono::steady_clock::now();
        if (now < restUntil) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(restUntil - now).count();
            std::this_thread::sleep_for(std::chrono::milliseconds(std::min<int64_t>(std::max<int64_t>(left, 1), 20)));
            continue;
        }
        int64_t msSinceInference = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastInferenceAt).count();
        int intervalMs = detectionIntervalMs_.load();
        if (msSinceInference < intervalMs) {
            // Not due yet -- sleep a short, bounded amount rather than
            // busy-spinning, but don't oversleep past when we're due.
            int remaining = static_cast<int>(intervalMs - msSinceInference);
            std::this_thread::sleep_for(std::chrono::milliseconds(std::min(remaining, 20)));
            continue;
        }

        auto passStart = std::chrono::steady_clock::now();
        lastInferenceAt = passStart;

        cv::Mat frame = frameSource_ ? frameSource_() : cv::Mat();
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        TelemetrySnapshot telemetry = telemetryProvider_ ? telemetryProvider_() : TelemetrySnapshot{};

        auto dbgInferStart = std::chrono::steady_clock::now();
        auto rawAll = runDetection(frame);
        double dbgInferMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - dbgInferStart).count();
        dbgInferMsAccum += dbgInferMs;

        // ── Tracking ─────────────────────────────────────────────────
        // Everything down to the low-confidence floor goes to the tracker:
        // confident detections may start tracks, weak ones may only keep a
        // confirmed track alive through a dip (see ObjectTracker.h). Only
        // CONFIRMED tracks ever produce records.
        syncTrackerConfig();
        const float lowConf = tracker_.config().lowConfidence;   // set by syncTrackerConfig()
        std::vector<RawDetection> candidates;
        std::vector<trk::Detection> trackerInput;
        candidates.reserve(rawAll.size());
        trackerInput.reserve(rawAll.size());
        for (const auto& raw : rawAll) {
            if (raw.confidence < lowConf)
                continue;
            candidates.push_back(raw);
            trackerInput.push_back({ raw.classIndex, raw.confidence,
                                     cv::Rect2f(static_cast<float>(raw.x), static_cast<float>(raw.y),
                                                static_cast<float>(raw.w), static_cast<float>(raw.h)) });
        }
        const int64_t trackerNowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        trk::UpdateResult tracked = tracker_.update(frame, trackerInput, trackerNowMs);

        // Tracks the tracker gave up on go to the archive (geographic
        // re-ID); archived objects older than the re-ID window are dropped.
        for (const auto& gone : tracked.removed) {
            auto it = objects_.find(gone.id);
            if (it == objects_.end())
                continue;
            if (it->second.recorded && it->second.seenGeo) {
                ArchivedObject arc;
                arc.state = std::move(it->second);
                arc.group = gone.group;
                arc.hist = gone.hist;
                arc.archivedMs = nowMs;
                archive_.push_back(std::move(arc));
            }
            objects_.erase(it);
        }
        {
            const int64_t windowMs = trackReidWindowMs_.load();
            archive_.erase(std::remove_if(archive_.begin(), archive_.end(),
                [&](const ArchivedObject& a) { return nowMs - a.archivedMs > windowMs; }),
                archive_.end());
        }

        // Georeference every tracked detection ONCE, up front -- with the
        // object's bearing history, so rangeByTriangulation() gets its
        // parallax -- both for the record policy and the record itself.
        std::vector<RawDetection> rawDetections;       // what the preview draws
        int recordsThisPass = 0;
        for (size_t i = 0; i < candidates.size(); ++i) {
            const trk::Assignment& as = tracked.assignments[i];
            if (as.trackId == 0)
                continue;                               // weak and unmatched: noise
            RawDetection raw = candidates[i];
            if (as.votedClass >= 0)
                raw.classIndex = as.votedClass;         // stable class, no car/other_vehicle flicker
            rawDetections.push_back(raw);
            if (!as.confirmed)
                continue;                               // tentative: not a record (yet)

            ObjectState& obj = objects_[as.trackId];
            const GeoCandidate geo = computeGeoCandidate(raw, frame.cols, frame.rows, telemetry,
                                                         &obj.bearingHistory);
            if (geo.rayValid) {
                BearingObservation obs;
                obs.timestampMs = nowMs;
                obs.droneLat = telemetry.latitude;
                obs.droneLon = telemetry.longitude;
                obs.unitNorth = geo.rayUnitNorth;
                obs.unitEast = geo.rayUnitEast;
                obj.bearingHistory.push_back(obs);
                if (obj.bearingHistory.size() > kMaxBearingHistoryPerTrack)
                    obj.bearingHistory.erase(obj.bearingHistory.begin());
            }

            bool shouldRecord;
            if (obj.trackId == 0) {
                // First confirmed sighting: an object seen before (out of
                // view longer than the lost memory) keeps its old trackId
                // and is treated as continuing; otherwise it is new.
                const trk::TrackInfo* ti = tracker_.find(as.trackId);
                if (geo.georeferenced && ti &&
                    reidentify(obj, ti->group, ti->hist, geo.lat, geo.lon, nowMs)) {
                    shouldRecord = true;   // decided below, after fusion
                }
                else {
                    obj.trackId = nextTrackId_++;
                    shouldRecord = true;
                }
            }
            else {
                shouldRecord = true;       // decided below, after fusion
            }
            const bool firstRecord = !obj.recorded;
            obj.sightings++;
            obj.bestConfidence = std::max(obj.bestConfidence, candidates[i].confidence);
            GeoCandidate fusedGeo = geo;
            if (geo.georeferenced) {
                fuseObjectPosition(obj, geo, nowMs);
                fusedGeo.lat = obj.fusedLat();
                fusedGeo.lon = obj.fusedLon();
                fusedGeo.uncertaintyM = obj.fusedRadiusM();
                obj.seenGeo = true;
                obj.seenLat = fusedGeo.lat;
                obj.seenLon = fusedGeo.lon;
            }
            shouldRecord = firstRecord || shouldRecordAgain(obj, fusedGeo, nowMs);
            if (!shouldRecord)
                continue;   // same object as an already-recorded sighting, nothing new to log yet

            DetectionRecord rec;
            rec.timestampMs = nowMs;
            rec.className = classNameFor(raw.classIndex);
            rec.confidence = candidates[i].confidence;
            rec.bboxX = raw.x; rec.bboxY = raw.y; rec.bboxW = raw.w; rec.bboxH = raw.h;
            rec.telemetry = telemetry;
            rec.trackId = obj.trackId;
            rec.sightings = obj.sightings;
            rec.bestConfidence = obj.bestConfidence;

            if (fusedGeo.georeferenced) {
                // Position = fusion of all this object's sightings so far;
                // distance/bearing/method = this pass's own measurement.
                rec.latitude = fusedGeo.lat;
                rec.longitude = fusedGeo.lon;
                rec.uncertaintyM = fusedGeo.uncertaintyM;
                rec.distanceM = geo.distanceM;
                rec.bearingDeg = geo.bearingDeg;
                rec.rangeMethod = geo.method;
                rec.georeferenced = true;
            }

            // The NEXT pass's move-threshold/refresh-interval checks are
            // measured from this record.
            obj.recorded = true;
            obj.lastRecordMs = nowMs;
            obj.lastGeoreferenced = rec.georeferenced;
            obj.lastLat = rec.latitude;
            obj.lastLon = rec.longitude;

            // The screenshot (full-frame clone + JPEG write to disk) is
            // saved OUTSIDE recordsMutex_, so a Python poll of
            // getRecordsSince() -- which holds the GIL -- never waits on
            // disk I/O. Only this thread assigns ids and appends, so the
            // records stay in id order.
            {
                std::lock_guard<std::mutex> lock(recordsMutex_);
                rec.id = nextId_++;
            }
            if (!screenshotDir_.empty())
                rec.screenshotPath = saveScreenshot(frame, raw, rec.id);
            {
                std::lock_guard<std::mutex> lock(recordsMutex_);
                records_.push_back(std::move(rec));
            }
            detectionCount_.fetch_add(1);
            recordsThisPass++;
        }
        dbgRecordsAccum += recordsThisPass;

        // Hand off this pass's boxes to previewLoop -- the ONLY point of
        // contact between the two threads besides the (already-existing)
        // frameMutex_/recordsMutex_.
        {
            std::lock_guard<std::mutex> lock(boxesMutex_);
            latestBoxes_ = rawDetections;
        }
        lastInferenceCompletedAtMs_.store(nowMs);

        double durationMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - passStart).count();
        lastPassDurationMs_.store(durationMs);
        lastPassTimestampMs_.store(nowMs);

        // Never back-to-back: rest at least durationMs x (1/duty - 1).
        {
            const double duty = maxDutyCycle_.load();
            restUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds(
                static_cast<int64_t>(durationMs * (1.0 / duty - 1.0)));
        }
        // Tiling costs ~5x per pass; if this machine can't afford it, drop
        // back to full-frame detection rather than eat into the headroom
        // the live video needs.
        if (isTilingActive()) {
            int budget = tilingBudgetMs_.load();
            if (budget <= 0)
                budget = std::max(500, 2 * intervalMs);
            slowTiledPasses = durationMs > budget ? slowTiledPasses + 1 : 0;
            if (slowTiledPasses >= 3) {
                tilingSuspended_.store(true);
                slowTiledPasses = 0;
                fprintf(stderr, "[DetectionLink] tiled passes took %.0f ms (> %d ms budget) 3 times "
                                "in a row -- tiling suspended, full-frame detection continues.\n",
                        durationMs, budget);
            }
        }
        dbgPassMsAccum += durationMs;

        if (++dbgPassCounter >= kDebugPrintEveryPasses) {
            // Gated behind DETECTIONLINK_VERBOSE_LOGGING -- see macro def
            // near the top of this file. This was the line used to
            // confirm backend=CUDA during bring-up; re-enable if you ever
            // need to re-verify which backend is actually engaged, or to
            // profile inference time after a model/export change.
            DL_LOG(<< "[DetectionLink][perf][inference] "
                << "avg_inference=" << (dbgInferMsAccum / dbgPassCounter) << "ms "
                << "avg_pass_total=" << (dbgPassMsAccum / dbgPassCounter) << "ms "
                << "(interval_target=" << intervalMs << "ms) "
                << "records=" << dbgRecordsAccum
                << (isUsingCuda() ? " backend=CUDA" : " backend=CPU"));
            dbgPassCounter = 0; dbgInferMsAccum = dbgPassMsAccum = 0.0; dbgRecordsAccum = 0;
        }
    }
}

std::vector<DetectionLink::RawDetection> DetectionLink::runInference(const cv::Mat& frame) {
    std::vector<RawDetection> results;
    if (!impl_->loaded)
        return results;

    // ── Letterbox resize to a square inputSize x inputSize input,
    // preserving aspect ratio with gray padding -- standard YOLO
    // preprocessing. scale/padX/padY are needed to map boxes back to
    // original frame coordinates after inference.
    const int inputSize = inputSize_.load();
    int origW = frame.cols, origH = frame.rows;

    // inferenceLoop() already skips frame.empty() (cols==0 || rows==0 ||
    // no data) before calling here, but that's not quite the same thing
    // as "safe to divide by" -- guard explicitly anyway, since a 0 on
    // either side turns scale into +-inf, scaledW/H into NaN, and
    // static_cast<int> of a NaN is undefined behaviour in C++ (MSVC's
    // debug CRT traps this and calls abort() -- the exact "Debug Error!"
    // dialog this comment is here because of).
    if (origW <= 0 || origH <= 0)
        return results;

    double scale = std::min(static_cast<double>(inputSize) / origW,
        static_cast<double>(inputSize) / origH);
    int scaledW = std::max(1, static_cast<int>(origW * scale));
    int scaledH = std::max(1, static_cast<int>(origH * scale));
    int padX = (inputSize - scaledW) / 2;
    int padY = (inputSize - scaledH) / 2;

    // A fresh analog 5.8GHz receiver commonly hands back frames in an
    // unexpected format for the first bit while it's still hunting for
    // signal lock (the solid-blue "no signal" frame is the visible sign
    // of this) -- not necessarily BGR, not necessarily 3-channel.
    // blobFromImage below assumes exactly that. Normalize here rather
    // than let a channel-count mismatch throw deeper in the pipeline.
    cv::Mat bgrFrame;
    if (frame.channels() == 3) {
        bgrFrame = frame;
    }
    else if (frame.channels() == 1) {
        cv::cvtColor(frame, bgrFrame, cv::COLOR_GRAY2BGR);
    }
    else if (frame.channels() == 4) {
        cv::cvtColor(frame, bgrFrame, cv::COLOR_BGRA2BGR);
    }
    else {
        return results;   // unrecognized format for this pass; try again next tick
    }

    // Everything from here down touches OpenCV/the DNN graph directly.
    // Any of resize/blobFromImage/setInput/forward can throw --
    // historically only forward() was guarded, which still let a
    // resize/blobFromImage exception (e.g. from the channel-format hiccup
    // above, before this normalization existed) escape uncaught out of
    // this function, up through inferenceLoop() on its own worker
    // thread, and straight to std::terminate()/abort() -- killing the
    // *entire* cockpit process over what should only ever cost a single
    // skipped detection pass.
    cv::Mat output;
    try {
        cv::Mat resized;
        cv::resize(bgrFrame, resized, cv::Size(scaledW, scaledH));
        cv::Mat letterboxed(inputSize, inputSize, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(letterboxed(cv::Rect(padX, padY, scaledW, scaledH)));

        cv::Mat blob = cv::dnn::blobFromImage(letterboxed, 1.0 / 255.0,
            cv::Size(inputSize, inputSize), cv::Scalar(), true, false);
        impl_->net.setInput(blob);

        // This is also where a mismatch between inputSize_ and the size
        // the ONNX model was actually exported at (see setInputSize()'s
        // doc comment in the header) throws -- a configuration mistake,
        // not a reason to bring down the whole cockpit either.
        output = impl_->net.forward();
    }
    catch (const cv::Exception& e) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            fprintf(stderr,
                "[DetectionLink] a detection pass failed (%s) -- most "
                "likely either an odd/unsupported frame from the video "
                "source, or inputSize_ (%d) not matching the size this "
                "ONNX model was exported at (see setInputSize()). "
                "Detection passes will be skipped whenever this recurs; "
                "the rest of the cockpit keeps running.\n",
                e.what(), inputSize);
        }
        return results;
    }

    // Two ONNX layouts are accepted, told apart by the output shape:
    //
    //   END-TO-END [1, maxDetections, 6] -- YOLO26's default export (and
    //   export_model.py's): NMS is inside the graph. Each row is
    //   [x1, y1, x2, y2, confidence, classIndex], corner format, in *input*
    //   (letterboxed) pixel space; unused rows are zero-padded.
    //
    //   RAW [1, 4 + numClasses, numAnchors] -- `end2end=False` exports
    //   (train.py --export-only, older YOLO versions): per anchor
    //   [cx, cy, w, h, score_0 .. score_nc-1] with sigmoid already applied,
    //   channel-major. Needs a class argmax + NMS here (class-aware,
    //   IoU 0.7, max 300 -- Ultralytics' own predict defaults), so it gives
    //   the same boxes model.predict() would.
    //
    // Anything else is reported once and the pass is skipped. If boxes
    // ever come out wrong in a consistent way, turn on
    // DETECTIONLINK_VERBOSE_LOGGING to print raw rows (corner vs centre
    // format, pixel vs normalized coordinates are the usual suspects).
    struct ModelBox { float x1, y1, x2, y2, conf; int cls; };
    std::vector<ModelBox> boxes;

    const int d1 = output.dims == 3 ? output.size[1] : (output.dims == 2 ? output.size[0] : 0);
    const int d2 = output.dims == 3 ? output.size[2] : (output.dims == 2 ? output.size[1] : 0);
    const float* data = reinterpret_cast<const float*>(output.data);
    const bool endToEnd = d2 == 6 && d1 >= 6;
    const bool rawLayout = !endToEnd && d1 > 4 && d2 > d1;
    if (output.type() != CV_32F || (!endToEnd && !rawLayout)) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            fprintf(stderr,
                "[DetectionLink] unsupported model output layout (dims=%d, %d x %d) -- "
                "expected [1, N, 6] (end-to-end) or [1, 4+classes, anchors] (raw). "
                "Re-export the model with export_model.py.\n",
                output.dims, d1, d2);
        }
        return results;
    }

    if (endToEnd) {
        boxes.reserve(static_cast<size_t>(d1));
        for (int i = 0; i < d1; ++i) {
            const float* r = data + static_cast<size_t>(i) * 6;
            if (r[4] < 0.001f)   // padding rows; the real threshold is applied by the caller
                continue;
            boxes.push_back({ r[0], r[1], r[2], r[3], r[4], static_cast<int>(std::lround(r[5])) });
        }
    }
    else {
        const int numClasses = d1 - 4;
        const int numAnchors = d2;
        constexpr float kRawScoreFloor = 0.01f;   // callers filter far higher; this only bounds NMS work
        std::vector<cv::Rect2d> nmsRects;
        std::vector<float> nmsScores;
        std::vector<ModelBox> cand;
        for (int a = 0; a < numAnchors; ++a) {
            int best = 0;
            float bestScore = data[static_cast<size_t>(4) * numAnchors + a];
            for (int c = 1; c < numClasses; ++c) {
                const float v = data[static_cast<size_t>(4 + c) * numAnchors + a];
                if (v > bestScore) { bestScore = v; best = c; }
            }
            if (bestScore < kRawScoreFloor)
                continue;
            const float cx = data[a], cy = data[numAnchors + a];
            const float w = data[static_cast<size_t>(2) * numAnchors + a];
            const float h = data[static_cast<size_t>(3) * numAnchors + a];
            cand.push_back({ cx - w * 0.5f, cy - h * 0.5f, cx + w * 0.5f, cy + h * 0.5f, bestScore, best });
            // Class-aware NMS in one call: every class is shifted to its own
            // far-away region, so boxes of different classes never overlap.
            const double off = static_cast<double>(best) * (inputSize + 1) * 4.0;
            nmsRects.emplace_back(cx - w * 0.5 + off, cy - h * 0.5 + off, w, h);
            nmsScores.push_back(bestScore);
        }
        std::vector<int> keep;
        cv::dnn::NMSBoxes(nmsRects, nmsScores, kRawScoreFloor, 0.7f, keep, 1.0f, 300);
        boxes.reserve(keep.size());
        for (int k : keep)
            boxes.push_back(cand[static_cast<size_t>(k)]);
    }

#if DETECTIONLINK_VERBOSE_LOGGING
    static std::atomic<int> debugRowsLogged{ 0 };
    for (const auto& b : boxes) {
        if (debugRowsLogged.load() >= 8)
            break;
        const int n = debugRowsLogged.fetch_add(1);
        DL_LOG_ERR("[DetectionLink] DEBUG row %d (%s): model-space x1=%.2f y1=%.2f x2=%.2f "
                   "y2=%.2f conf=%.3f cls=%d (inputSize=%d)\n",
                   n, endToEnd ? "end-to-end" : "raw", b.x1, b.y1, b.x2, b.y2, b.conf, b.cls, inputSize);
    }
#endif

    results.reserve(boxes.size());
    for (const auto& b : boxes) {
        // Undo letterbox: from inputSize-space back to original frame.
        const float ox1 = (b.x1 - padX) / static_cast<float>(scale);
        const float oy1 = (b.y1 - padY) / static_cast<float>(scale);
        const float ox2 = (b.x2 - padX) / static_cast<float>(scale);
        const float oy2 = (b.y2 - padY) / static_cast<float>(scale);

        RawDetection d;
        d.classIndex = b.cls;
        d.confidence = b.conf;
        d.x = static_cast<int>(std::max(0.0f, ox1));
        d.y = static_cast<int>(std::max(0.0f, oy1));
        d.w = static_cast<int>(std::max(0.0f, std::min(ox2, static_cast<float>(origW)) - d.x));
        d.h = static_cast<int>(std::max(0.0f, std::min(oy2, static_cast<float>(origH)) - d.y));
        if (d.w > 0 && d.h > 0)
            results.push_back(d);
    }
    return results;
}

std::vector<DetectionLink::RawDetection> DetectionLink::runDetection(const cv::Mat& frame) {
    std::vector<RawDetection> all = runInference(frame);
    if (!isTilingActive() || frame.cols < 64 || frame.rows < 64)
        return all;
    const size_t nFull = all.size();   // [0, nFull) = full-frame boxes

    // Four corner tiles, 60 % of the frame each (20 % overlap in the
    // middle) -- the same layout fpv_eval.py's "tiles" variant measured.
    const int tw = frame.cols * 6 / 10, th = frame.rows * 6 / 10;
    const int xs[2] = { 0, frame.cols - tw }, ys[2] = { 0, frame.rows - th };
    // A box ending within this margin of an INNER tile edge is cut off.
    const int edgeX = std::max(4, tw / 50), edgeY = std::max(4, th / 50);
    for (int y0 : ys) {
        for (int x0 : xs) {
            const cv::Rect roi(x0, y0, tw, th);
            for (RawDetection d : runInference(frame(roi))) {
                // The full frame (and the neighbouring tile) has this
                // object whole -- a box cut by an inner tile edge would be
                // a wrong, smaller duplicate.
                const bool cutL = x0 > 0 && d.x < edgeX;
                const bool cutT = y0 > 0 && d.y < edgeY;
                const bool cutR = x0 + tw < frame.cols && d.x + d.w > tw - edgeX;
                const bool cutB = y0 + th < frame.rows && d.y + d.h > th - edgeY;
                if (cutL || cutT || cutR || cutB)
                    continue;
                d.x += x0;
                d.y += y0;
                // A fragment of a big object the full frame already has
                // whole: mostly inside a larger same-class full-frame box.
                const double area = static_cast<double>(d.w) * d.h;
                bool fragment = false;
                for (size_t i = 0; i < nFull && !fragment; ++i) {
                    const RawDetection& f = all[i];
                    if (f.classIndex != d.classIndex ||
                        static_cast<double>(f.w) * f.h <= area)
                        continue;
                    const int ix = std::max(0, std::min(d.x + d.w, f.x + f.w) - std::max(d.x, f.x));
                    const int iy = std::max(0, std::min(d.y + d.h, f.y + f.h) - std::max(d.y, f.y));
                    fragment = area > 0 && static_cast<double>(ix) * iy >= 0.7 * area;
                }
                if (!fragment)
                    all.push_back(d);
            }
        }
    }

    // One box per object: class-aware NMS over full-frame + tile boxes
    // (each class shifted to its own region so classes never suppress
    // each other), keeping the most confident.
    std::vector<cv::Rect> rects;
    std::vector<float> scores;
    rects.reserve(all.size());
    scores.reserve(all.size());
    const int shift = std::max(frame.cols, frame.rows) * 2;
    for (const auto& d : all) {
        const int off = std::max(0, d.classIndex) * shift;
        rects.emplace_back(d.x + off, d.y + off, d.w, d.h);
        scores.push_back(d.confidence);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(rects, scores, 0.0f, 0.5f, keep);
    std::vector<RawDetection> merged;
    merged.reserve(keep.size());
    for (int k : keep)
        merged.push_back(all[static_cast<size_t>(k)]);
    return merged;
}

DetectionLink::GeoCandidate DetectionLink::computeGeoCandidate(const RawDetection& raw,
    int frameW, int frameH, const TelemetrySnapshot& telemetry,
    const std::vector<BearingObservation>* history) const {
    GeoCandidate geo;

    // Ground-plane, triangulated, and object-size ranging always agree
    // on *direction* (same world ray) and only differ on *how far* --
    // see rangeByGroundPlane/rangeByTriangulation/rangeByObjectSize for
    // why each one degrades in different situations.
    double ux = 0.0, uy = 0.0, uz = 0.0;
    if (!computeWorldRay(raw, frameW, frameH, telemetry, ux, uy, uz))
        return geo;

    // Recorded regardless of whether any ranging method below succeeds
    // -- this is what lets inferenceLoop() log a bearing observation for
    // triangulation on every pass a track is seen, not only the passes
    // that happened to get georeferenced some other way.
    geo.rayValid = true;
    geo.rayUnitNorth = ux;
    geo.rayUnitEast = uy;

    RangeEstimate ground = rangeByGroundPlane(telemetry, ux, uy, uz);
    RangeEstimate triangulated;
    if (history && !history->empty())
        triangulated = rangeByTriangulation(*history, ux, uy, telemetry);
    RangeEstimate bySize = rangeByObjectSize(raw, frameW, telemetry, ux, uy, uz);

    // Preference order, each one degrading in a different, complementary
    // situation:
    //   1. ground-plane, when the ray is steep enough to trust (see
    //      setMinGroundRayComponent()) -- no assumed object size, no
    //      dependence on track history length.
    //   2. triangulated, when this track has enough accumulated parallax
    //      (see setTriangulationMinBaselineM/setTriangulationMinBearingSpreadDeg)
    //      -- also no assumed object size, and this is exactly the case
    //      ground-plane can't handle: a shallow/forward-facing ray. This
    //      is what lets a distance estimate keep refining pass-to-pass
    //      as the drone moves, without needing the object's real-world
    //      size at all.
    //   3. object-size, when this class has a known width configured --
    //      the fallback for a shallow ray with NOT enough parallax yet
    //      (track just started, or the drone is flying straight at the
    //      object with no lateral offset -- triangulation's own
    //      degenerate case).
    // If more than one produced a result, prefer in that order; if only
    // one did, use whichever that is -- some georeference is better than
    // none as long as we're honest (via geo.method) about which one
    // produced it.
    const RangeEstimate* chosen = nullptr;
    if (ground.valid) chosen = &ground;
    else if (triangulated.valid) chosen = &triangulated;
    else if (bySize.valid) chosen = &bySize;

    RangeEstimate coarse;
    if (!chosen && coarseDefaultRangeM_.load() > 0.0) {
        coarse = rangeCoarse(telemetry, ux, uy, uz);
        if (coarse.valid)
            chosen = &coarse;
    }

    if (chosen) {
        geo.lat = chosen->lat;
        geo.lon = chosen->lon;
        geo.distanceM = chosen->distanceM;
        geo.bearingDeg = chosen->bearingDeg;
        geo.method = chosen->method;
        geo.uncertaintyM = chosen->uncertaintyM;
        geo.georeferenced = true;
    }
    return geo;
}

bool DetectionLink::computeWorldRay(const RawDetection& raw, int frameW, int frameH,
    const TelemetrySnapshot& telemetry, double& outUnitX, double& outUnitY, double& outUnitZ) const {
    if (!telemetry.valid || frameW <= 0 || frameH <= 0)
        return false;

    // Pixel center of the detection box.
    double u = raw.x + raw.w / 2.0;
    double v = raw.y + raw.h / 2.0;

    double hFov = horizontalFovDeg_.load();
    double vFov = radToDeg(2.0 * std::atan(std::tan(degToRad(hFov / 2.0)) *
        static_cast<double>(frameH) / frameW));

    // Angle of this pixel off the camera's own optical axis.
    double angleXDeg = ((u - frameW / 2.0) / (frameW / 2.0)) * (hFov / 2.0);
    double angleYDeg = ((v - frameH / 2.0) / (frameH / 2.0)) * (vFov / 2.0);

    // Camera-space ray, camera looking along +X when the gimbal is
    // level/forward: X=forward, Y=right, Z=down.
    Vec3 rayCam{ 1.0, std::tan(degToRad(angleXDeg)), std::tan(degToRad(angleYDeg)) };

    // Gimbal: pan rotates about the (down) Z axis, tilt rotates about
    // the (right) Y axis -- tiltDeg=90 means the ray's forward component
    // rotates toward +Z (down), which is what we want for a straight-down
    // shot.
    Mat3 gimbalTilt = rotY(-telemetry.gimbalTiltDeg);   // negative: tilt-down rotates +X toward +Z
    Mat3 gimbalPan = rotZ(telemetry.gimbalPanDeg);
    Vec3 rayBody = matVec(matMul(gimbalPan, gimbalTilt), rayCam);

    // Body to world (NED): apply roll, then pitch, then yaw/heading.
    Mat3 attitude = matMul(rotZ(telemetry.headingDeg),
        matMul(rotY(telemetry.pitchDeg), rotX(telemetry.rollDeg)));
    Vec3 rayWorld = matVec(attitude, rayBody);

    double mag = std::sqrt(rayWorld.x * rayWorld.x + rayWorld.y * rayWorld.y + rayWorld.z * rayWorld.z);
    if (mag < 1e-9)
        return false;

    outUnitX = rayWorld.x / mag;
    outUnitY = rayWorld.y / mag;
    outUnitZ = rayWorld.z / mag;
    return true;
}

namespace {
    // Shared by both ranging methods: given a unit world-space ray from a
    // known lat/lon and a ground distance in meters along that ray's
    // horizontal component, produce the resulting lat/lon and compass
    // bearing. northM/eastM are the ray's horizontal components already
    // scaled to the chosen range.
    void offsetLatLon(double originLat, double originLon, double northM, double eastM,
        double& outLat, double& outLon, double& outBearingDeg) {
        double latRad = degToRad(originLat);
        outLat = originLat + radToDeg(northM / kEarthRadiusM);
        outLon = originLon + radToDeg(eastM / (kEarthRadiusM * std::cos(latRad)));
        outBearingDeg = radToDeg(std::atan2(eastM, northM));
        if (outBearingDeg < 0.0)
            outBearingDeg += 360.0;
    }

    // Inverse of offsetLatLon() above -- same flat-earth approximation,
    // consistent by construction so a round trip through both functions
    // is exact. Used by rangeByTriangulation() to bring a track's PAST
    // bearing observations (each stamped with the drone's lat/lon at the
    // time) into the CURRENT pass's local north/east frame, so all of a
    // track's observations can be compared/solved in one common frame.
    void latLonToLocalMeters(double originLat, double originLon, double lat, double lon,
        double& outNorthM, double& outEastM) {
        double originLatRad = degToRad(originLat);
        outNorthM = degToRad(lat - originLat) * kEarthRadiusM;
        outEastM = degToRad(lon - originLon) * kEarthRadiusM * std::cos(originLatRad);
    }
}

// ── Uncertainty model ─────────────────────────────────────────────────
// All radii are ~2 sigma, meters. Error sources that matter at search-and-
// rescue scale: altitude (baro / GPS-relative, a few meters), the camera
// tilt angle (manually set until the gimbal reports it -- easily 3-5 deg
// off), and heading (magnetometer / GPS course, ~5 deg).
namespace {
    constexpr double kAltSigmaBaseM = 3.0;      // + 5 % of altitude
    constexpr double kTiltSigmaDeg = 4.0;
    constexpr double kHeadingSigmaDeg = 5.0;
}

double DetectionLink::groundPlaneUncertaintyM(double altitudeM, double unitZ, double slantM) {
    // Horizontal distance D = h / tan(d), d = depression angle of the ray.
    const double dep = std::asin(std::min(1.0, std::max(1e-3, unitZ)));
    const double D = altitudeM / std::tan(dep);
    const double sigH = kAltSigmaBaseM + 0.05 * altitudeM;
    const double sigDep = degToRad(kTiltSigmaDeg);
    const double sD = std::hypot(D / altitudeM * sigH,                      // altitude error
                                 altitudeM / (std::sin(dep) * std::sin(dep)) * sigDep);  // tilt error
    const double sLat = slantM * degToRad(kHeadingSigmaDeg);               // heading error
    return std::max(5.0, 2.0 * std::hypot(sD, sLat));
}

DetectionLink::RangeEstimate DetectionLink::rangeCoarse(const TelemetrySnapshot& telemetry,
    double unitX, double unitY, double unitZ) const {
    RangeEstimate est;
    const double defRange = coarseDefaultRangeM_.load();
    const double maxRange = std::max(defRange, coarseMaxRangeM_.load());
    const double horiz = std::hypot(unitX, unitY);
    double dist = defRange;
    if (unitZ > 0.02 && telemetry.altitudeM >= kMinGroundPlaneAltitudeM)
        dist = std::min(maxRange, telemetry.altitudeM / unitZ * horiz);   // clamped ground-plane distance
    if (horiz < 0.15) {
        // Looking (nearly) straight down: the object is below the drone.
        est.lat = telemetry.latitude;
        est.lon = telemetry.longitude;
        est.bearingDeg = 0.0;
        est.distanceM = std::max(0.0, telemetry.altitudeM);
        est.uncertaintyM = std::max(30.0, telemetry.altitudeM);
    }
    else {
        const double n = unitX / horiz * dist, e = unitY / horiz * dist;
        offsetLatLon(telemetry.latitude, telemetry.longitude, n, e, est.lat, est.lon, est.bearingDeg);
        est.distanceM = dist;
        // Along the ray the distance is a guess (+-dist); across it the
        // heading error. Never claim better than 50 m for a coarse fix.
        est.uncertaintyM = std::max(50.0, std::max(dist, 2.0 * dist * degToRad(kHeadingSigmaDeg)));
    }
    est.method = "coarse";
    est.valid = true;
    return est;
}

DetectionLink::RangeEstimate DetectionLink::rangeByGroundPlane(const TelemetrySnapshot& telemetry,
    double unitX, double unitY, double unitZ) const {
    RangeEstimate est;

    // Ray must point meaningfully downward (positive Z in NED) to hit
    // the ground plane at all -- see setMinGroundRayComponent() for why
    // "meaningfully" matters, not just ">0".
    if (unitZ <= minGroundRayComponent_.load())
        return est;

    // Scale the unit ray until its Z component covers the drone's AGL
    // altitude, giving north/east ground offsets and slant range in
    // meters. Flat-earth assumption -- fine at the few-km ranges this is
    // designed for; swap in a DEM lookup here later if slope error
    // matters for your use case.
    // Altitude is height above the take-off point (see TelemetrySnapshot):
    // below ~2 m (on the ground, just after take-off, no baro/GPS altitude)
    // the intersection is meaningless -- it would put the object under
    // the drone at distance 0.
    if (telemetry.altitudeM < kMinGroundPlaneAltitudeM)
        return est;
    double t = telemetry.altitudeM / unitZ;   // = slant range, since unit vector has length 1
    double northM = unitX * t;
    double eastM = unitY * t;
    if (std::hypot(northM, eastM) > kMaxGroundPlaneRangeM)
        return est;   // ray nearly level: a few degrees of error = hundreds of meters

    offsetLatLon(telemetry.latitude, telemetry.longitude, northM, eastM, est.lat, est.lon, est.bearingDeg);
    est.distanceM = t;
    est.method = "ground_plane";
    est.uncertaintyM = groundPlaneUncertaintyM(telemetry.altitudeM, unitZ, t);
    est.valid = true;
    return est;
}

DetectionLink::RangeEstimate DetectionLink::rangeByTriangulation(
    const std::vector<BearingObservation>& history,
    double curUnitNorth, double curUnitEast, const TelemetrySnapshot& telemetry) const {
    RangeEstimate est;

    // Every observation (history + this pass's own) reduced to: an
    // observer position in meters, local to THIS pass's drone position,
    // plus a horizontal-only unit bearing direction from there. Ray
    // observations whose horizontal component is too small to normalize
    // (camera pointed almost straight down/up for that sighting) are
    // skipped -- they carry no usable bearing.
    struct Obs { double oN, oE, dN, dE; };
    std::vector<Obs> obs;
    obs.reserve(history.size() + 1);

    auto addObs = [&](double lat, double lon, double uN, double uE) {
        double horizMag = std::sqrt(uN * uN + uE * uE);
        if (horizMag < 1e-6)
            return;
        double oN = 0.0, oE = 0.0;
        latLonToLocalMeters(telemetry.latitude, telemetry.longitude, lat, lon, oN, oE);
        obs.push_back({ oN, oE, uN / horizMag, uE / horizMag });
        };

    for (const auto& h : history)
        addObs(h.droneLat, h.droneLon, h.unitNorth, h.unitEast);
    addObs(telemetry.latitude, telemetry.longitude, curUnitNorth, curUnitEast);   // oN=oE=0 for this one

    if (obs.size() < 2)
        return est;   // nothing to intersect yet -- first sighting of this track

    // Degeneracy check: require real parallax, not just elapsed
    // distance. A drone can rack up meters of movement while flying
    // essentially straight at (or at a constant bearing past) the
    // object -- large baseline, ~zero angular spread, and a
    // numerically "valid" solve that's actually amplifying GPS/heading
    // noise into a huge range error. Check every pair, not just
    // first-vs-last, since a track's history can wander non-monotonically
    // (e.g. an orbit, or a missed-then-reacquired pass).
    double maxBaselineM = 0.0, maxSpreadDeg = 0.0;
    for (size_t i = 0; i < obs.size(); ++i) {
        for (size_t j = i + 1; j < obs.size(); ++j) {
            double dN = obs[i].oN - obs[j].oN, dE = obs[i].oE - obs[j].oE;
            maxBaselineM = std::max(maxBaselineM, std::sqrt(dN * dN + dE * dE));
            double dot = obs[i].dN * obs[j].dN + obs[i].dE * obs[j].dE;
            dot = std::max(-1.0, std::min(1.0, dot));
            maxSpreadDeg = std::max(maxSpreadDeg, radToDeg(std::acos(dot)));
        }
    }
    if (maxBaselineM < triangulationMinBaselineM_.load() ||
        maxSpreadDeg < triangulationMinBearingSpreadDeg_.load())
        return est;   // not enough parallax yet -- rangeByObjectSize() is the fallback for this case

    // Least-squares intersection of all observation lines
    // (P = O_i + t * D_i): minimize the sum of squared perpendicular
    // distances from the solved point to every line. Each observation
    // contributes the 2x2 projector onto the perpendicular of its own
    // direction, (I - D_i * D_i^T); summing those gives a single 2x2
    // normal-equations system, solved directly (no iteration needed at
    // this dimensionality).
    double A00 = 0.0, A01 = 0.0, A11 = 0.0, b0 = 0.0, b1 = 0.0;
    for (const auto& o : obs) {
        double p00 = 1.0 - o.dN * o.dN;
        double p01 = -o.dN * o.dE;
        double p11 = 1.0 - o.dE * o.dE;
        A00 += p00; A01 += p01; A11 += p11;
        b0 += p00 * o.oN + p01 * o.oE;
        b1 += p01 * o.oN + p11 * o.oE;
    }
    double det = A00 * A11 - A01 * A01;
    if (std::abs(det) < 1e-6)
        return est;   // degenerate despite the spread check above -- be conservative

    double targetN = (A11 * b0 - A01 * b1) / det;
    double targetE = (A00 * b1 - A01 * b0) / det;

    double distanceM = std::sqrt(targetN * targetN + targetE * targetE);
    if (!(distanceM > 0.5) || distanceM > 5000.0)
        return est;   // reject a nonsensical/blown-up solve rather than pass it through as real

    offsetLatLon(telemetry.latitude, telemetry.longitude, targetN, targetE, est.lat, est.lon, est.bearingDeg);
    est.distanceM = distanceM;
    est.method = "triangulated";
    est.uncertaintyM = std::max(10.0, 0.2 * est.distanceM);
    est.valid = true;
    return est;
}

DetectionLink::RangeEstimate DetectionLink::rangeByObjectSize(const RawDetection& raw, int frameW,
    const TelemetrySnapshot& telemetry, double unitX, double unitY, double unitZ) const {
    RangeEstimate est;
    if (raw.w <= 0 || frameW <= 0)
        return est;

    double knownWidthM = 0.0;
    {
        std::lock_guard<std::mutex> lock(classWidthsMutex_);
        auto it = knownObjectWidthsM_.find(classNameFor(raw.classIndex));
        if (it == knownObjectWidthsM_.end())
            return est;   // no configured real-world size for this class
        knownWidthM = it->second;
    }
    if (knownWidthM <= 0.0)
        return est;

    // Pinhole camera model consistent with the same horizontalFovDeg used
    // for the ray itself: focalLengthPx is the distance (in pixels) at
    // which the sensor's horizontal FOV cone has unit half-width, i.e.
    // frameW/2 = focalLengthPx * tan(hFov/2).
    double hFov = horizontalFovDeg_.load();
    double focalLengthPx = (frameW / 2.0) / std::tan(degToRad(hFov / 2.0));

    // Slant range to the object: real_width_m * focal_length_px / bbox_width_px.
    double slantRangeM = (knownWidthM * focalLengthPx) / static_cast<double>(raw.w);

    // Project along the same unit ray used for ground-plane ranging --
    // this is what makes the two methods agree on *direction* and only
    // differ on *how far*. No altitude or ground-plane assumption needed
    // at all, which is exactly why this still works when unitZ is small
    // (near-horizontal FPV shots) or telemetry.altitudeM is untrustworthy.
    double northM = unitX * slantRangeM;
    double eastM = unitY * slantRangeM;

    offsetLatLon(telemetry.latitude, telemetry.longitude, northM, eastM, est.lat, est.lon, est.bearingDeg);
    est.distanceM = slantRangeM;
    est.method = "object_size";
    // Real width vs. assumed width and the viewing angle: ~35 %.
    est.uncertaintyM = std::max(10.0, 0.35 * est.distanceM);
    est.valid = true;
    return est;
}

void DetectionLink::setKnownObjectWidth(const std::string& className, double widthMeters) {
    std::lock_guard<std::mutex> lock(classWidthsMutex_);
    knownObjectWidthsM_[className] = widthMeters;
}

void DetectionLink::clearKnownObjectWidths() {
    std::lock_guard<std::mutex> lock(classWidthsMutex_);
    knownObjectWidthsM_.clear();
}

std::vector<uchar> DetectionLink::getLatestAnnotatedFrameJpeg() const {
    // Called from Python (DetectionMapWidget's Tk-thread poll, ~25 Hz) via
    // pybind11, synchronously on the calling thread. The binding releases
    // the GIL around this call (see Bindings.cpp), so the lock, resize and
    // imencode below never block the Tk mainloop or the other Python
    // threads. The timing counters are printed only with
    // DETECTIONLINK_VERBOSE_LOGGING.
    static std::atomic<int> dbgCallCounter{ 0 };
    constexpr int kDebugPrintEveryCalls = 25;
    auto dbgStart = std::chrono::steady_clock::now();

    lastPreviewRequestMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    std::vector<uchar> jpeg;
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        if (lastAnnotatedFrame_.empty())
            return jpeg;
        frame = lastAnnotatedFrame_;   // cv::Mat copy is a cheap header copy; data is shared
    }
    double dbgCopyMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now() - dbgStart).count();

    // Encode cost scales with pixel count, and DetectionMapWidget
    // immediately thumbnails whatever comes back down to 640x360 anyway
    // (see _poll_live_frame) -- so encoding at the full source-camera
    // resolution 25x/sec is pure waste: more imencode time here, more
    // bytes copied across the pybind11 boundary, more JPEG-decode time on
    // the Python side, for detail that gets thrown away one line later.
    // Downscaling here (to the same target size, aspect-preserved) should
    // cut this function's cost by roughly (original_pixels/target_pixels)
    // -- often several times over for a >960px-wide source frame.
    auto dbgResizeStart = std::chrono::steady_clock::now();
    cv::Mat toEncode = frame;
    constexpr int kPreviewMaxWidth = 640;
    if (frame.cols > kPreviewMaxWidth) {
        double scale = static_cast<double>(kPreviewMaxWidth) / frame.cols;
        cv::resize(frame, toEncode, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    double dbgResizeMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now() - dbgResizeStart).count();

    auto dbgEncodeStart = std::chrono::steady_clock::now();
    cv::imencode(".jpg", toEncode, jpeg);
    double dbgEncodeMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now() - dbgEncodeStart).count();

    int n = dbgCallCounter.fetch_add(1) + 1;
    if (n % kDebugPrintEveryCalls == 0) {
        // Gated behind DETECTIONLINK_VERBOSE_LOGGING -- see macro def
        // near the top of this file. Re-enable if the live-detections
        // preview pane ever feels slow and you need to see where the
        // time in this call is actually going.
        double totalMs = dbgCopyMs + dbgResizeMs + dbgEncodeMs;
        DL_LOG(<< "[DetectionLink][perf] getLatestAnnotatedFrameJpeg: "
            << "src=" << frame.cols << "x" << frame.rows
            << " encoded=" << toEncode.cols << "x" << toEncode.rows
            << " lock+copy=" << dbgCopyMs << "ms resize=" << dbgResizeMs
            << "ms imencode=" << dbgEncodeMs << "ms total=" << totalMs
            << "ms jpeg_bytes=" << jpeg.size());
    }
    return jpeg;
}

std::string DetectionLink::saveScreenshot(const cv::Mat& frame, const RawDetection& raw, uint64_t id) const {
    std::ostringstream name;
    name << "det_" << id << "_" << raw.classIndex << ".jpg";
    std::filesystem::path outPath = std::filesystem::path(screenshotDir_) / name.str();

    // Full frame saved, not just the crop -- a crop alone loses the
    // surrounding context that's often what actually helps a pilot
    // re-locate the object later. Draw the box on a copy so the saved
    // image shows what was detected without mutating the frame the
    // caller still owns.
    cv::Mat annotated = frame.clone();
    cv::rectangle(annotated, cv::Rect(raw.x, raw.y, raw.w, raw.h), cv::Scalar(0, 220, 255), 2);

    if (!cv::imwrite(outPath.string(), annotated))
        return "";
    return outPath.string();
}

std::string DetectionLink::classNameFor(int classIndex) const {
    if (classIndex >= 0 && classIndex < static_cast<int>(classNames_.size()))
        return classNames_[classIndex];
    return "class_" + std::to_string(classIndex);
}

namespace {
    // Straight-line ground distance between two lat/lon points, meters.
    // Equirectangular approximation -- same flat-earth assumption class
    // already used by offsetLatLon()/rangeByGroundPlane() above, which is
    // fine at the short (few-meter to few-hundred-meter) distances this
    // is used for (deciding "has this tracked object moved far enough on
    // the map to deserve a new record"), not meant for long-range
    // great-circle work.
    double approxGroundDistanceM(double lat1, double lon1, double lat2, double lon2) {
        constexpr double kEarthRadiusM = 6378137.0;
        double latRad = (lat1 + lat2) * 0.5 * M_PI / 180.0;
        double dLatM = (lat2 - lat1) * M_PI / 180.0 * kEarthRadiusM;
        double dLonM = (lon2 - lon1) * M_PI / 180.0 * kEarthRadiusM * std::cos(latRad);
        return std::sqrt(dLatM * dLatM + dLonM * dLonM);
    }
}

void DetectionLink::setClassConfidenceThreshold(const std::string& className, float threshold) {
    std::lock_guard<std::mutex> lock(classThresholdsMutex_);
    classConfidenceThresholds_[className] = threshold;
}

void DetectionLink::clearClassConfidenceThresholds() {
    std::lock_guard<std::mutex> lock(classThresholdsMutex_);
    classConfidenceThresholds_.clear();
}

void DetectionLink::syncTrackerConfig() {
    trk::Config& c = tracker_.config();
    c.highConfidence = confidenceThreshold_.load();
    c.classHighConfidence.clear();
    float lowestHigh = c.highConfidence;
    {
        std::lock_guard<std::mutex> lock(classThresholdsMutex_);
        for (size_t i = 0; i < classNames_.size(); ++i) {
            auto it = classConfidenceThresholds_.find(classNames_[i]);
            if (it == classConfidenceThresholds_.end())
                continue;
            c.classHighConfidence[static_cast<int>(i)] = it->second;
            lowestHigh = std::min(lowestHigh, it->second);
        }
    }
    c.lowConfidence = std::min(trackLowConfidence_.load(), lowestHigh);
    c.confirmHits = std::max(1, trackConfirmHits_.load());
    c.maxLostMs = std::max(0, trackLostMemoryMs_.load());
    c.minLostPasses = std::max(0, trackMaxMissedPasses_.load());
    c.minIou = static_cast<float>(trackIouThreshold_.load());
    c.compensateCameraMotion = trackCameraMotion_.load();
}

bool DetectionLink::reidentify(ObjectState& obj, int group, const cv::Mat& hist,
                               double lat, double lon, int64_t nowMs) {
    const double radius = trackReidRadiusM_.load();
    if (radius <= 0.0)
        return false;
    int best = -1;
    double bestDist = radius;
    for (size_t i = 0; i < archive_.size(); ++i) {
        const ArchivedObject& a = archive_[i];
        if (a.group != group || nowMs - a.archivedMs > trackReidWindowMs_.load())
            continue;
        const double d = approxGroundDistanceM(a.state.seenLat, a.state.seenLon, lat, lon);
        if (d > bestDist)
            continue;
        // Appearance must not contradict: a red car where a white one was
        // parked is a different car. (No histogram = position only.)
        const double sim = trk::ObjectTracker::histSimilarity(a.hist, hist);
        if (sim >= 0.0 && sim < 0.5)
            continue;
        best = static_cast<int>(i);
        bestDist = d;
    }
    if (best < 0)
        return false;
    // The archived bearing history is from the object's old sightings --
    // still the same object, still useful parallax.
    std::vector<BearingObservation> history = std::move(obj.bearingHistory);
    obj = std::move(archive_[static_cast<size_t>(best)].state);
    obj.bearingHistory.insert(obj.bearingHistory.end(), history.begin(), history.end());
    while (obj.bearingHistory.size() > kMaxBearingHistoryPerTrack)
        obj.bearingHistory.erase(obj.bearingHistory.begin());
    archive_.erase(archive_.begin() + best);
    return true;
}

void DetectionLink::fuseObjectPosition(ObjectState& obj, const GeoCandidate& geo, int64_t nowMs) {
    const double r = std::max(5.0, geo.uncertaintyM);
    const double w = 1.0 / (r * r);
    if (!obj.fusedValid) {
        obj.fusedValid = true;
        obj.anchorLat = geo.lat;
        obj.anchorLon = geo.lon;
        obj.sumW = obj.sumN = obj.sumE = 0.0;
        obj.minRadiusM = r;
        obj.bestIsCoarse = geo.method == "coarse";
        obj.fusedAtMs = nowMs;
    }
    // Fade older sightings (moving objects), then add this one.
    const double dtS = std::max<int64_t>(0, nowMs - obj.fusedAtMs) / 1000.0;
    const double decay = std::pow(0.5, dtS / kFusionHalfLifeS);
    obj.sumW *= decay; obj.sumN *= decay; obj.sumE *= decay;
    obj.minRadiusM /= std::max(decay, 1e-3);   // the old best fix fades too
    if (r <= obj.minRadiusM) {
        obj.minRadiusM = r;
        obj.bestIsCoarse = geo.method == "coarse";
    }
    double n = 0.0, e = 0.0;
    latLonToLocalMeters(obj.anchorLat, obj.anchorLon, geo.lat, geo.lon, n, e);
    obj.sumW += w; obj.sumN += w * n; obj.sumE += w * e;
    obj.fusedAtMs = nowMs;
}

double DetectionLink::ObjectState::fusedRadiusM() const {
    if (!fusedValid || sumW <= 0.0)
        return 0.0;
    // Independent errors would shrink as 1/sqrt(sum w); mount/heading
    // biases are shared between sightings, so never claim better than
    // half the best single fix (and never under 5 m). Coarse fixes all
    // share the same distance GUESS -- averaging them does not shrink it.
    const double floorFactor = bestIsCoarse ? 1.0 : 0.5;
    return std::max({ 5.0, 1.0 / std::sqrt(sumW), floorFactor * minRadiusM });
}

double DetectionLink::ObjectState::fusedLat() const {
    double lat = anchorLat, lon = anchorLon, brg = 0.0;
    if (fusedValid && sumW > 0.0)
        offsetLatLon(anchorLat, anchorLon, sumN / sumW, sumE / sumW, lat, lon, brg);
    return lat;
}

double DetectionLink::ObjectState::fusedLon() const {
    double lat = anchorLat, lon = anchorLon, brg = 0.0;
    if (fusedValid && sumW > 0.0)
        offsetLatLon(anchorLat, anchorLon, sumN / sumW, sumE / sumW, lat, lon, brg);
    return lon;
}

bool DetectionLink::shouldRecordAgain(const ObjectState& obj, const GeoCandidate& geo,
                                      int64_t nowMs) const {
    // A continuing object only earns a fresh DetectionRecord once it has
    // moved far enough on the map since its last recorded position, or
    // once the periodic refresh interval has elapsed -- this is the
    // actual "don't log the same object every frame" behavior; tracking
    // alone just keeps the identity (trackId) stable across passes.
    if (!obj.recorded)
        return true;
    if (nowMs - obj.lastRecordMs >= trackRefreshIntervalMs_.load())
        return true;
    if (obj.lastGeoreferenced && geo.georeferenced) {
        const double movedM = approxGroundDistanceM(obj.lastLat, obj.lastLon, geo.lat, geo.lon);
        return movedM >= trackMoveThresholdM_.load();
    }
    // Georeferencing just became available (or was lost): a real state
    // change a pilot reviewing the log would want to see. Neither
    // georeferenced: nothing to compare, rely on the refresh interval.
    return obj.lastGeoreferenced != geo.georeferenced;
}

std::vector<DetectionRecord> DetectionLink::getAllRecords() const {
    std::lock_guard<std::mutex> lock(recordsMutex_);
    return records_;
}

std::vector<DetectionRecord> DetectionLink::getRecordsSince(uint64_t sinceId) const {
    std::lock_guard<std::mutex> lock(recordsMutex_);
    std::vector<DetectionRecord> result;
    for (const auto& r : records_)
        if (r.id > sinceId)
            result.push_back(r);
    return result;
}

void DetectionLink::clearRecords() {
    std::lock_guard<std::mutex> lock(recordsMutex_);
    records_.clear();
}