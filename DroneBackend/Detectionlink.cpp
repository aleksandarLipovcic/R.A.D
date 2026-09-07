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

#ifdef _WIN32
#include <windows.h>
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
    worker_ = std::thread(&DetectionLink::detectionLoop, this);
    return true;
}

void DetectionLink::stop() {
    running_.store(false);
    if (worker_.joinable())
        worker_.join();
}

void DetectionLink::detectionLoop() {
#ifdef _WIN32
    // Deliberately the mirror image of VideoLink's capture thread, which
    // raises itself to ABOVE_NORMAL. Detection is supplementary -- it
    // must never win a scheduling contest against the thread painting
    // the pilot's live feed.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif

    while (running_.load()) {
        auto passStart = std::chrono::steady_clock::now();

        cv::Mat frame = frameSource_ ? frameSource_() : cv::Mat();
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(detectionIntervalMs_.load()));
            continue;
        }

        TelemetrySnapshot telemetry = telemetryProvider_ ? telemetryProvider_() : TelemetrySnapshot{};

        auto rawAll = runInference(frame);

        int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Filter by confidence BEFORE handing detections to the tracker --
        // a sub-threshold row shouldn't be allowed to start, extend, or
        // keep alive a track any more than it should have produced a
        // DetectionRecord under the old code.
        std::vector<RawDetection> rawDetections;
        rawDetections.reserve(rawAll.size());
        for (const auto& raw : rawAll)
            if (raw.confidence >= confidenceThreshold_.load())
                rawDetections.push_back(raw);

        // Georeference every surviving raw detection ONCE, up front --
        // both so updateTracks() below can compare a continuing track's
        // fresh position against its last-recorded one (the actual
        // "has it moved far enough to log again" check), and so the
        // DetectionRecord-building code further down doesn't redo the
        // same ray/ranging math a second time.
        std::vector<GeoCandidate> geoCandidates;
        geoCandidates.reserve(rawDetections.size());
        for (const auto& raw : rawDetections)
            geoCandidates.push_back(computeGeoCandidate(raw, frame.cols, frame.rows, telemetry));

        // Match this pass's boxes against tracks_ from previous passes.
        // assignments[i] corresponds to rawDetections[i]: which track it
        // belongs to (existing or brand new), and whether that track's
        // state (new track, moved enough, or refresh interval elapsed)
        // warrants writing a DetectionRecord this pass -- see
        // setTrackIouThreshold() etc. in the header for the full
        // rationale and its position-only re-ID limitations.
        auto assignments = updateTracks(rawDetections, geoCandidates, nowMs);

        cv::Mat passAnnotated;   // built lazily, only if at least one detection survives the threshold

        for (size_t i = 0; i < rawDetections.size(); ++i) {
            const auto& raw = rawDetections[i];
            const auto& assignment = assignments[i];

            // Always draw every currently-tracked box on the live preview
            // frame, whether or not it earns a new DetectionRecord this
            // pass -- the live view is meant to show what the model sees
            // right now, independent of the dedup/re-ID bookkeeping below.
            if (passAnnotated.empty())
                passAnnotated = frame.clone();
            cv::rectangle(passAnnotated, cv::Rect(raw.x, raw.y, raw.w, raw.h), cv::Scalar(0, 220, 255), 2);
            std::string label = classNameFor(raw.classIndex) + " " +
                std::to_string(static_cast<int>(raw.confidence * 100)) + "%";
            cv::putText(passAnnotated, label, cv::Point(raw.x, std::max(0, raw.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 220, 255), 1);

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
        }

        if (!passAnnotated.empty()) {
            std::lock_guard<std::mutex> lock(frameMutex_);
            lastAnnotatedFrame_ = passAnnotated;
        }

        double durationMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now() - passStart).count();
        lastPassDurationMs_.store(durationMs);
        lastPassTimestampMs_.store(nowMs);

        int intervalMs = detectionIntervalMs_.load();
        int remainingMs = intervalMs - static_cast<int>(durationMs);
        if (remainingMs > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(remainingMs));
        // If inference itself took longer than the configured interval,
        // loop straight back around rather than sleeping negative time --
        // this is the signal to raise detectionIntervalMs_ or move to a
        // smaller/faster model, not something to silently absorb.
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

    // detectionLoop() already skips frame.empty() (cols==0 || rows==0 ||
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
    // this function, up through detectionLoop() on its own worker
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

    // One-time diagnostic: print the first handful of real (non-padding)
    // rows' raw model-space values before any of the corner/pixel-space
    // assumptions below are applied to them. If boxes are coming out
    // wrong/invisible, this tells you which assumption is actually wrong
    // instead of guessing:
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

        if (debugRowsLogged.load() < 8) {
            int n = debugRowsLogged.fetch_add(1);
            fprintf(stderr,
                "[DetectionLink] DEBUG row %d: model-space x1=%.2f y1=%.2f "
                "x2=%.2f y2=%.2f conf=%.3f cls=%d (inputSize=%d) -- see "
                "runInference()'s comment above this loop for how to read "
                "these.\n",
                n, x1, y1, x2, y2, conf, classIndex, inputSize);
        }

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
    int frameW, int frameH, const TelemetrySnapshot& telemetry) const {
    GeoCandidate geo;

    // Ground-plane and object-size ranging always agree on *direction*
    // (same world ray) and only differ on *how far* -- see
    // rangeByGroundPlane/rangeByObjectSize for why each one degrades in
    // different situations.
    double ux = 0.0, uy = 0.0, uz = 0.0;
    if (!computeWorldRay(raw, frameW, frameH, telemetry, ux, uy, uz))
        return geo;

    RangeEstimate ground = rangeByGroundPlane(telemetry, ux, uy, uz);
    RangeEstimate bySize = rangeByObjectSize(raw, frameW, telemetry, ux, uy, uz);

    // Prefer ground-plane when the ray is steep enough for it to be
    // trustworthy (see setMinGroundRayComponent()); it needs no
    // assumption about the object's real-world size. Otherwise prefer
    // object-size if this class has a known width configured. If only
    // one of the two produced a result, use whichever that is -- some
    // georeference is better than none as long as we're honest about
    // which method (and therefore which failure mode) produced it.
    const RangeEstimate* chosen = nullptr;
    if (ground.valid) chosen = &ground;
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
    std::vector<uchar> jpeg;
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        if (lastAnnotatedFrame_.empty())
            return jpeg;
        frame = lastAnnotatedFrame_;   // cv::Mat copy is a cheap header copy; data is shared
    }
    cv::imencode(".jpg", frame, jpeg);
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

std::vector<DetectionLink::TrackAssignment> DetectionLink::updateTracks(
    const std::vector<RawDetection>& rawDetections,
    const std::vector<GeoCandidate>& geoCandidates, int64_t nowMs) {

    std::vector<TrackAssignment> assignments(rawDetections.size());
    std::vector<bool> rawMatched(rawDetections.size(), false);
    const size_t originalTrackCount = tracks_.size();   // tracks_ grows below as new tracks are appended
    std::vector<bool> trackMatched(originalTrackCount, false);

    // Greedy IoU matching: consider every (raw detection, track) pair
    // with matching class and IoU above threshold, take the best pair
    // first, remove both from consideration, repeat. Simple O(n*m) --
    // fine at the handful of simultaneous detections this app expects;
    // swap for a proper Hungarian assignment only if that stops being
    // true.
    double iouThreshold = trackIouThreshold_.load();
    while (true) {
        double bestIou = iouThreshold;
        int bestRawIdx = -1, bestTrackIdx = -1;
        for (size_t ri = 0; ri < rawDetections.size(); ++ri) {
            if (rawMatched[ri])
                continue;
            for (size_t ti = 0; ti < tracks_.size(); ++ti) {
                if (trackMatched[ti])
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

        rawMatched[bestRawIdx] = true;
        trackMatched[bestTrackIdx] = true;

        Track& track = tracks_[bestTrackIdx];
        const RawDetection& raw = rawDetections[bestRawIdx];
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
        const GeoCandidate& geo = geoCandidates[bestRawIdx];

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

        assignments[bestRawIdx].trackId = track.trackId;
        assignments[bestRawIdx].shouldRecord = shouldRecord;
    }

    // Unmatched raw detections start brand-new tracks -- always recorded
    // immediately, same as the pre-tracking behavior for a first sighting.
    for (size_t ri = 0; ri < rawDetections.size(); ++ri) {
        if (rawMatched[ri])
            continue;
        Track track;
        track.trackId = nextTrackId_++;
        track.classIndex = rawDetections[ri].classIndex;
        track.lastX = rawDetections[ri].x; track.lastY = rawDetections[ri].y;
        track.lastW = rawDetections[ri].w; track.lastH = rawDetections[ri].h;
        track.lastSeenMs = nowMs;
        track.missedPasses = 0;
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