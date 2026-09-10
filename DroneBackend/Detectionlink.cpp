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

    // Cap on how many past BearingObservations a single Track carries
    // (see Track::bearingHistory in the header). Triangulation only ever
    // needs a handful of well-spread observations -- keeping every
    // sighting for a track's whole lifetime would grow unbounded for a
    // long-dwelling object and cost more per-pass compute (every kept
    // observation is one more line in the least-squares solve) for no
    // accuracy benefit past a point. Oldest is dropped first once this
    // is exceeded.
    constexpr size_t kMaxBearingHistoryPerTrack = 12;

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
    if (useCuda_.load()) {
        try {
            impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            cv::Mat dummy(inputSize_.load(), inputSize_.load(), CV_8UC3, cv::Scalar(114, 114, 114));
            cv::Mat blob = cv::dnn::blobFromImage(dummy, 1.0 / 255.0,
                cv::Size(inputSize_.load(), inputSize_.load()), cv::Scalar(), true, false);
            impl_->net.setInput(blob);
            impl_->net.forward();
            usingCuda_.store(true);
        }
        catch (const cv::Exception&) {
            impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            usingCuda_.store(false);
        }
    }
    else {
        impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
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
    // regardless of what inferenceLoop is doing. Left at default/NORMAL
    // priority -- it's not the heavy CPU consumer, inferenceLoop is (see
    // that function's own THREAD_PRIORITY_BELOW_NORMAL).
    constexpr int kPreviewTickMs = 40;   // ~25fps ceiling for the live preview pane

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

    while (running_.load()) {
        auto now = std::chrono::steady_clock::now();
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
        auto rawAll = runInference(frame);
        double dbgInferMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - dbgInferStart).count();
        dbgInferMsAccum += dbgInferMs;

        // Filter by confidence BEFORE handing detections to the tracker --
        // a sub-threshold row shouldn't be allowed to start, extend, or
        // keep alive a track any more than it should have produced a
        // DetectionRecord under the old code.
        std::vector<RawDetection> rawDetections;
        rawDetections.reserve(rawAll.size());
        for (const auto& raw : rawAll)
            if (raw.confidence >= confidenceThreshold_.load())
                rawDetections.push_back(raw);

        // Peek-match this pass's boxes against tracks_ from previous
        // passes BEFORE computing geo candidates -- purely so a matched
        // track's accumulated bearingHistory can be handed to
        // rangeByTriangulation() below. matchRawToTracks() is pure/
        // read-only (see its header comment), so this is safe to call
        // again inside updateTracks() afterward with the exact same
        // result; we pass this result straight through instead of
        // letting updateTracks() recompute it, so there's only one
        // source of truth for "which track did this raw detection match"
        // this pass.
        std::vector<int> matchedTrackIdx = matchRawToTracks(rawDetections);

        // Georeference every surviving raw detection ONCE, up front --
        // both so updateTracks() below can compare a continuing track's
        // fresh position against its last-recorded one (the actual
        // "has it moved far enough to log again" check), and so the
        // DetectionRecord-building code further down doesn't redo the
        // same ray/ranging math a second time.
        std::vector<GeoCandidate> geoCandidates;
        geoCandidates.reserve(rawDetections.size());
        for (size_t ri = 0; ri < rawDetections.size(); ++ri) {
            const std::vector<BearingObservation>* history = nullptr;
            if (matchedTrackIdx[ri] >= 0)
                history = &tracks_[static_cast<size_t>(matchedTrackIdx[ri])].bearingHistory;
            geoCandidates.push_back(
                computeGeoCandidate(rawDetections[ri], frame.cols, frame.rows, telemetry, history));
        }

        // Commit this pass's matches/new-tracks/aging, using the SAME
        // matchedTrackIdx computed above -- see updateTracks()'s header
        // comment for why it takes this instead of recomputing its own.
        // assignments[i] corresponds to rawDetections[i]: which track it
        // belongs to (existing or brand new), and whether that track's
        // state (new track, moved enough, or refresh interval elapsed)
        // warrants writing a DetectionRecord this pass -- see
        // setTrackIouThreshold() etc. in the header for the full
        // rationale and its position-only re-ID limitations.
        auto assignments = updateTracks(rawDetections, geoCandidates, matchedTrackIdx, telemetry, nowMs);

        int recordsThisPass = 0;
        for (size_t i = 0; i < rawDetections.size(); ++i) {
            const auto& raw = rawDetections[i];
            const auto& assignment = assignments[i];

            if (!assignment.shouldRecord)
                continue;   // same object as an already-recorded sighting, nothing new to log yet

            const GeoCandidate& geo = geoCandidates[i];

            DetectionRecord rec;
            rec.timestampMs = nowMs;
            rec.className = classNameFor(raw.classIndex);
            rec.confidence = raw.confidence;
            rec.bboxX = raw.x; rec.bboxY = raw.y; rec.bboxW = raw.w; rec.bboxH = raw.h;
            rec.telemetry = telemetry;
            rec.trackId = assignment.trackId;

            if (geo.georeferenced) {
                rec.latitude = geo.lat;
                rec.longitude = geo.lon;
                rec.distanceM = geo.distanceM;
                rec.bearingDeg = geo.bearingDeg;
                rec.rangeMethod = geo.method;
                rec.georeferenced = true;
            }

            // updateTracks() already matched/created the track and
            // decided shouldRecord using this exact geo candidate; now
            // that we're actually committing a record for it, stamp the
            // track with this sighting's position/time so the NEXT
            // pass's move-threshold/refresh-interval checks are measured
            // from here, not from the sighting before it.
            for (auto& t : tracks_) {
                if (t.trackId == assignment.trackId) {
                    t.lastRecordMs = nowMs;
                    t.lastGeoreferenced = rec.georeferenced;
                    t.lastLat = rec.latitude;
                    t.lastLon = rec.longitude;
                    break;
                }
            }

            {
                std::lock_guard<std::mutex> lock(recordsMutex_);
                rec.id = nextId_++;
                if (!screenshotDir_.empty())
                    rec.screenshotPath = saveScreenshot(frame, raw, rec.id);
                records_.push_back(rec);
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

    // YOLO26 (and Ultralytics' "end-to-end" exports generally) bakes NMS
    // directly into the graph: output shape [1, maxDetections, 6], one
    // row per candidate slot, already deduplicated by the model itself.
    // This is NOT the older [1, 4+numClasses, numBoxes] "raw logits"
    // layout the previous version of this function assumed (that one
    // needed a manual per-class argmax + cv::dnn::NMSBoxes pass; this one
    // needs neither, since NMS already happened inside the model) --
    // confirmed against your own export.py output ("output shape(s)
    // (1, 300, 6)"), not guessed.
    //
    // Each row is [x1, y1, x2, y2, confidence, classIndex] in *input*
    // (letterboxed, inputSize x inputSize) pixel space, corner format --
    // not center+size. Rows beyond however many real detections the
    // model found are zero-padded (confidence ~0) up to the fixed 300,
    // so filter by confidence rather than assuming a "real" row count.
    //
    // Verify this column order against your own export in Netron (the
    // export script prints a netron.app link) if boxes ever come out
    // wrong -- the two usual suspects for "wrong in a consistent way"
    // are corner-vs-center+size here, and normalized-vs-pixel
    // coordinates (pixel-space values land in roughly 0-inputSize;
    // normalized ones in 0-1 -- easy to tell apart by printing one row's
    // raw values on a test frame).
    cv::Mat out = output.reshape(1, output.size[1]);   // -> [maxDetections, 6]

    // One-time diagnostic (now gated behind DETECTIONLINK_VERBOSE_LOGGING --
    // see macro def near the top of this file): prints the first handful
    // of real (non-padding) rows' raw model-space values before any of
    // the corner/pixel-space assumptions below are applied to them. Was
    // used to confirm the [x1,y1,x2,y2,conf,cls] layout and pixel-space
    // (not normalized) coordinates against this export -- that's now
    // confirmed for yolo26m_main.onnx, so it's off by default. If a
    // future re-export ever produces wrong/invisible boxes again,
    // re-enable the macro and check:
    //   - x1/y1/x2/y2 all sitting in roughly 0.0-1.0 -> the export is
    //     normalized, not pixel-space; every ox1/oy1/ox2/oy2 below needs
    //     an extra "* inputSize" before the letterbox-undo math.
    //   - x2 < x1 or y2 < y1 fairly often -> this likely isn't
    //     corner-format [x1,y1,x2,y2] at all; re-check in Netron whether
    //     it's actually [cx,cy,w,h] (center + size) instead.
    //   - values in roughly 0-inputSize range with x2>x1, y2>y1 -> this
    //     assumption is fine; the degenerate box is coming from somewhere
    //     else (e.g. a genuinely near-zero-confidence garbage row).
    static std::atomic<int> debugRowsLogged{ 0 };

    for (int i = 0; i < out.rows; ++i) {
        float x1 = out.at<float>(i, 0);
        float y1 = out.at<float>(i, 1);
        float x2 = out.at<float>(i, 2);
        float y2 = out.at<float>(i, 3);
        float conf = out.at<float>(i, 4);
        int classIndex = static_cast<int>(std::round(out.at<float>(i, 5)));

        if (conf < 0.001f)   // padding rows; the real threshold is applied by the caller
            continue;

#if DETECTIONLINK_VERBOSE_LOGGING
        if (debugRowsLogged.load() < 8) {
            int n = debugRowsLogged.fetch_add(1);
            DL_LOG_ERR(
                "[DetectionLink] DEBUG row %d: model-space x1=%.2f y1=%.2f "
                "x2=%.2f y2=%.2f conf=%.3f cls=%d (inputSize=%d) -- see "
                "runInference()'s comment above this loop for how to read "
                "these.\n",
                n, x1, y1, x2, y2, conf, classIndex, inputSize);
        }
#endif

        // Undo letterbox: from inputSize-space back to original frame.
        float ox1 = (x1 - padX) / static_cast<float>(scale);
        float oy1 = (y1 - padY) / static_cast<float>(scale);
        float ox2 = (x2 - padX) / static_cast<float>(scale);
        float oy2 = (y2 - padY) / static_cast<float>(scale);

        RawDetection d;
        d.classIndex = classIndex;
        d.confidence = conf;
        d.x = static_cast<int>(std::max(0.0f, ox1));
        d.y = static_cast<int>(std::max(0.0f, oy1));
        d.w = static_cast<int>(std::max(0.0f, ox2 - ox1));
        d.h = static_cast<int>(std::max(0.0f, oy2 - oy1));
        results.push_back(d);
    }
    return results;
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
    // -- this is what lets updateTracks() log a bearing observation for
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

    if (chosen) {
        geo.lat = chosen->lat;
        geo.lon = chosen->lon;
        geo.distanceM = chosen->distanceM;
        geo.bearingDeg = chosen->bearingDeg;
        geo.method = chosen->method;
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
    double t = telemetry.altitudeM / unitZ;   // = slant range, since unit vector has length 1
    double northM = unitX * t;
    double eastM = unitY * t;

    offsetLatLon(telemetry.latitude, telemetry.longitude, northM, eastM, est.lat, est.lon, est.bearingDeg);
    est.distanceM = t;
    est.method = "ground_plane";
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
    // ── TEMPORARY perf diagnostic ──────────────────────────────────────
    // This function is called from Python (DetectionMapWidget's Tk-thread
    // poll, ~25Hz) via pybind11, and runs SYNCHRONOUSLY on whichever
    // thread calls it. If the pybind11 wrapper for this function does not
    // release the GIL (py::gil_scoped_release) for the duration of the
    // call, every millisecond spent inside this function -- lock, copy,
    // AND imencode -- blocks the entire Python interpreter: the Tk
    // mainloop, DetectionWorker's poll thread, everything. That would
    // show up exactly as "the whole feed" bogging down, even though the
    // native FPV window itself is painted independently by VideoLink.
    // Check the Bindings.cpp entry for get_latest_annotated_frame_jpeg
    // and add py::call_guard<py::gil_scoped_release>() (or an explicit
    // py::gil_scoped_release inside the wrapper) if it's missing -- that
    // alone can be the difference between this stalling everything else
    // and running truly in parallel with it.
    static std::atomic<int> dbgCallCounter{ 0 };
    constexpr int kDebugPrintEveryCalls = 25;
    auto dbgStart = std::chrono::steady_clock::now();

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

double DetectionLink::trackIou(const RawDetection& raw, const Track& track) {
    int ax2 = raw.x + raw.w, ay2 = raw.y + raw.h;
    int bx2 = track.lastX + track.lastW, by2 = track.lastY + track.lastH;

    int ix1 = std::max(raw.x, track.lastX);
    int iy1 = std::max(raw.y, track.lastY);
    int ix2 = std::min(ax2, bx2);
    int iy2 = std::min(ay2, by2);

    int iw = std::max(0, ix2 - ix1);
    int ih = std::max(0, iy2 - iy1);
    double interArea = static_cast<double>(iw) * ih;
    if (interArea <= 0.0)
        return 0.0;

    double rawArea = static_cast<double>(raw.w) * raw.h;
    double trackArea = static_cast<double>(track.lastW) * track.lastH;
    double unionArea = rawArea + trackArea - interArea;
    return unionArea > 0.0 ? interArea / unionArea : 0.0;
}

std::vector<int> DetectionLink::matchRawToTracks(const std::vector<RawDetection>& rawDetections) const {
    std::vector<int> matched(rawDetections.size(), -1);
    std::vector<bool> rawTaken(rawDetections.size(), false);
    std::vector<bool> trackTaken(tracks_.size(), false);

    // Greedy IoU matching: consider every (raw detection, track) pair
    // with matching class and IoU above threshold, take the best pair
    // first, remove both from consideration, repeat. Simple O(n*m) --
    // fine at the handful of simultaneous detections this app expects;
    // swap for a proper Hungarian assignment only if that stops being
    // true. Pure/read-only: does not touch tracks_ at all, so it's safe
    // to call from inferenceLoop() as a "preview" match before
    // updateTracks() runs its own (identical, since it's the same
    // function) real assignment -- see this function's declaration in
    // the header for why that split exists.
    double iouThreshold = trackIouThreshold_.load();
    while (true) {
        double bestIou = iouThreshold;
        int bestRawIdx = -1, bestTrackIdx = -1;
        for (size_t ri = 0; ri < rawDetections.size(); ++ri) {
            if (rawTaken[ri])
                continue;
            for (size_t ti = 0; ti < tracks_.size(); ++ti) {
                if (trackTaken[ti])
                    continue;
                if (tracks_[ti].classIndex != rawDetections[ri].classIndex)
                    continue;
                double v = trackIou(rawDetections[ri], tracks_[ti]);
                if (v > bestIou) {
                    bestIou = v;
                    bestRawIdx = static_cast<int>(ri);
                    bestTrackIdx = static_cast<int>(ti);
                }
            }
        }
        if (bestRawIdx < 0)
            break;   // no pair left clears the IoU threshold

        rawTaken[bestRawIdx] = true;
        trackTaken[bestTrackIdx] = true;
        matched[bestRawIdx] = bestTrackIdx;
    }
    return matched;
}

std::vector<DetectionLink::TrackAssignment> DetectionLink::updateTracks(
    const std::vector<RawDetection>& rawDetections,
    const std::vector<GeoCandidate>& geoCandidates,
    const std::vector<int>& matchedTrackIdx,
    const TelemetrySnapshot& telemetry, int64_t nowMs) {

    std::vector<TrackAssignment> assignments(rawDetections.size());
    const size_t originalTrackCount = tracks_.size();   // tracks_ grows below as new tracks are appended
    std::vector<bool> trackMatched(originalTrackCount, false);

    for (size_t ri = 0; ri < rawDetections.size(); ++ri) {
        int ti = matchedTrackIdx[ri];
        if (ti < 0)
            continue;   // handled in the "brand-new track" loop below

        trackMatched[static_cast<size_t>(ti)] = true;

        Track& track = tracks_[static_cast<size_t>(ti)];
        const RawDetection& raw = rawDetections[ri];
        track.lastX = raw.x; track.lastY = raw.y; track.lastW = raw.w; track.lastH = raw.h;
        track.lastSeenMs = nowMs;
        track.missedPasses = 0;

        // A continuing track only earns a fresh DetectionRecord once it
        // has moved far enough on the map since its last recorded
        // position, or once the periodic refresh interval has elapsed --
        // this is the actual "don't log the same object every frame"
        // behavior; matching alone just keeps the track's identity
        // (trackId) stable across passes.
        bool shouldRecord = false;
        int64_t sinceLastRecordMs = nowMs - track.lastRecordMs;
        const GeoCandidate& geo = geoCandidates[ri];

        if (sinceLastRecordMs >= trackRefreshIntervalMs_.load()) {
            shouldRecord = true;
        }
        else if (track.lastGeoreferenced && geo.georeferenced) {
            // Both this sighting and the last recorded one have a real
            // position -- the actual "has this object moved enough on
            // the map" check.
            double movedM = approxGroundDistanceM(track.lastLat, track.lastLon, geo.lat, geo.lon);
            shouldRecord = movedM >= trackMoveThresholdM_.load();
        }
        else if (track.lastGeoreferenced != geo.georeferenced) {
            // Georeferencing just became available (or was lost) for a
            // continuing track -- worth a fresh record either way, since
            // it's a real state change a pilot reviewing the log would
            // want to see, not something the refresh interval alone
            // would reliably catch.
            shouldRecord = true;
        }
        // else: neither this sighting nor the last record is
        // georeferenced -- nothing meaningful to compare positionally,
        // so leave shouldRecord false and rely purely on the refresh
        // interval above to keep this track's entry from going stale.

        // Append this pass's bearing observation regardless of whether
        // shouldRecord ended up true -- an un-recorded sighting is still
        // a real ray the drone took, and is exactly the parallax a
        // FUTURE pass's rangeByTriangulation() needs. Only appended when
        // computeGeoCandidate() actually got a ray (geo.rayValid) -- see
        // that function for the one case it can't (telemetry invalid or
        // a zero-size frame).
        if (geo.rayValid) {
            BearingObservation obs;
            obs.timestampMs = nowMs;
            obs.droneLat = telemetry.latitude;
            obs.droneLon = telemetry.longitude;
            obs.unitNorth = geo.rayUnitNorth;
            obs.unitEast = geo.rayUnitEast;
            track.bearingHistory.push_back(obs);
            if (track.bearingHistory.size() > kMaxBearingHistoryPerTrack)
                track.bearingHistory.erase(track.bearingHistory.begin());
        }

        assignments[ri].trackId = track.trackId;
        assignments[ri].shouldRecord = shouldRecord;
    }

    // Unmatched raw detections start brand-new tracks -- always recorded
    // immediately, same as the pre-tracking behavior for a first sighting.
    for (size_t ri = 0; ri < rawDetections.size(); ++ri) {
        if (matchedTrackIdx[ri] >= 0)
            continue;
        Track track;
        track.trackId = nextTrackId_++;
        track.classIndex = rawDetections[ri].classIndex;
        track.lastX = rawDetections[ri].x; track.lastY = rawDetections[ri].y;
        track.lastW = rawDetections[ri].w; track.lastH = rawDetections[ri].h;
        track.lastSeenMs = nowMs;
        track.missedPasses = 0;

        const GeoCandidate& geo = geoCandidates[ri];
        if (geo.rayValid) {
            BearingObservation obs;
            obs.timestampMs = nowMs;
            obs.droneLat = telemetry.latitude;
            obs.droneLon = telemetry.longitude;
            obs.unitNorth = geo.rayUnitNorth;
            obs.unitEast = geo.rayUnitEast;
            track.bearingHistory.push_back(obs);
        }

        tracks_.push_back(track);

        assignments[ri].trackId = track.trackId;
        assignments[ri].shouldRecord = true;
    }

    // Age out tracks that went too many consecutive passes unmatched --
    // see setTrackMaxMissedPasses(). If the same physical object shows
    // up again later it will be assigned a brand new trackId (the
    // position-only re-ID limitation documented in the header). Only
    // the tracks that existed BEFORE this pass's brand-new ones were
    // appended above are eligible here -- a track created this very
    // pass is trivially "matched" (missedPasses already 0) and isn't in
    // trackMatched at all.
    int maxMissed = trackMaxMissedPasses_.load();
    for (size_t ti = 0; ti < originalTrackCount; ++ti) {
        if (!trackMatched[ti])
            tracks_[ti].missedPasses++;
    }
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [maxMissed](const Track& t) { return t.missedPasses > maxMissed; }),
        tracks_.end());

    return assignments;
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