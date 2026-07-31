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

    // Standard YOLO letterbox input size. Must match whatever size the
    // ONNX model was exported/trained at.
    constexpr int kInputSize = 640;

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

    // CPU by default -- switch to a CUDA/DNN_TARGET_CUDA backend here if
    // your ground-station laptop has a supported GPU and you've built
    // OpenCV with CUDA support; left conservative since that's an extra
    // build dependency, not something to assume.
    impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

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

        auto rawDetections = runInference(frame);

        int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        for (const auto& raw : rawDetections) {
            if (raw.confidence < confidenceThreshold_.load())
                continue;

            DetectionRecord rec;
            rec.timestampMs = nowMs;
            rec.className = classNameFor(raw.classIndex);
            rec.confidence = raw.confidence;
            rec.bboxX = raw.x; rec.bboxY = raw.y; rec.bboxW = raw.w; rec.bboxH = raw.h;
            rec.telemetry = telemetry;

            double lat = 0.0, lon = 0.0;
            if (georeference(raw, frame.cols, frame.rows, telemetry, lat, lon)) {
                rec.latitude = lat;
                rec.longitude = lon;
                rec.georeferenced = true;
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

    // ── Letterbox resize to a square kInputSize x kInputSize input,
    // preserving aspect ratio with gray padding -- standard YOLO
    // preprocessing. scale/padX/padY are needed to map boxes back to
    // original frame coordinates after inference.
    int origW = frame.cols, origH = frame.rows;
    double scale = std::min(static_cast<double>(kInputSize) / origW,
        static_cast<double>(kInputSize) / origH);
    int scaledW = static_cast<int>(origW * scale);
    int scaledH = static_cast<int>(origH * scale);
    int padX = (kInputSize - scaledW) / 2;
    int padY = (kInputSize - scaledH) / 2;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(scaledW, scaledH));
    cv::Mat letterboxed(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterboxed(cv::Rect(padX, padY, scaledW, scaledH)));

    cv::Mat blob = cv::dnn::blobFromImage(letterboxed, 1.0 / 255.0,
        cv::Size(kInputSize, kInputSize), cv::Scalar(), true, false);
    impl_->net.setInput(blob);

    cv::Mat output = impl_->net.forward();

    // Ultralytics YOLOv8/v11 ONNX export shape: [1, 4+numClasses, 8400]
    // (box coords transposed ahead of class scores, one column per
    // candidate box). Reshape to [4+numClasses, 8400] for easy column
    // access. If your export uses the older [1, 8400, 4+numClasses]
    // layout instead, transpose before the loop below -- check
    // output.size with a one-off print if boxes come out nonsensical.
    cv::Mat out = output.reshape(1, output.size[1]);
    int numClasses = out.rows - 4;
    int numBoxes = out.cols;

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classIds;

    for (int i = 0; i < numBoxes; ++i) {
        float cx = out.at<float>(0, i);
        float cy = out.at<float>(1, i);
        float w = out.at<float>(2, i);
        float h = out.at<float>(3, i);

        int bestClass = -1;
        float bestScore = 0.0f;
        for (int c = 0; c < numClasses; ++c) {
            float s = out.at<float>(4 + c, i);
            if (s > bestScore) { bestScore = s; bestClass = c; }
        }
        if (bestScore < 0.05f)   // cheap pre-filter before NMS; real threshold applied by caller
            continue;

        // Undo letterbox: from kInputSize-space back to original frame.
        float x1 = (cx - w / 2.0f - padX) / static_cast<float>(scale);
        float y1 = (cy - h / 2.0f - padY) / static_cast<float>(scale);
        float bw = w / static_cast<float>(scale);
        float bh = h / static_cast<float>(scale);

        boxes.emplace_back(cv::Rect(
            static_cast<int>(std::max(0.0f, x1)), static_cast<int>(std::max(0.0f, y1)),
            static_cast<int>(bw), static_cast<int>(bh)));
        scores.push_back(bestScore);
        classIds.push_back(bestClass);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, 0.05f, 0.45f, keep);

    for (int idx : keep) {
        RawDetection d;
        d.classIndex = classIds[idx];
        d.confidence = scores[idx];
        d.x = boxes[idx].x; d.y = boxes[idx].y; d.w = boxes[idx].width; d.h = boxes[idx].height;
        results.push_back(d);
    }
    return results;
}

bool DetectionLink::georeference(const RawDetection& raw, int frameW, int frameH,
    const TelemetrySnapshot& telemetry, double& outLat, double& outLon) const {
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

    // Ray must point downward (positive Z in NED) to hit the ground.
    if (rayWorld.z <= 1e-6)
        return false;

    // Scale the ray until its Z component covers the drone's AGL
    // altitude, giving north/east ground offsets in meters.
    double t = telemetry.altitudeM / rayWorld.z;
    double northM = rayWorld.x * t;
    double eastM = rayWorld.y * t;

    double latRad = degToRad(telemetry.latitude);
    outLat = telemetry.latitude + radToDeg(northM / kEarthRadiusM);
    outLon = telemetry.longitude + radToDeg(eastM / (kEarthRadiusM * std::cos(latRad)));
    return true;
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