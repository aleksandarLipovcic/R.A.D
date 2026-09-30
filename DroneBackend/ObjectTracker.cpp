#include "ObjectTracker.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <tuple>

namespace trk {

namespace {
    constexpr int64_t kMaxPredictMs = 1500;   // never extrapolate further than this
    constexpr float   kHistEma      = 0.30f;  // weight of a new appearance sample

    cv::Point2f centerOf(const cv::Rect2f& r) {
        return { r.x + r.width * 0.5f, r.y + r.height * 0.5f };
    }
}

double boxIou(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float ix = std::max(0.f, std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x));
    const float iy = std::max(0.f, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y));
    const double inter = static_cast<double>(ix) * iy;
    const double uni = static_cast<double>(a.area()) + b.area() - inter;
    return uni > 0.0 ? inter / uni : 0.0;
}

// =============================================================================
// Appearance
// =============================================================================
cv::Mat ObjectTracker::computeHist(const cv::Mat& frameBgr, const cv::Rect2f& box) {
    if (frameBgr.empty() || frameBgr.channels() != 3)
        return {};
    cv::Rect r(cv::Point(static_cast<int>(std::floor(box.x)), static_cast<int>(std::floor(box.y))),
               cv::Point(static_cast<int>(std::ceil(box.x + box.width)),
                         static_cast<int>(std::ceil(box.y + box.height))));
    r &= cv::Rect(0, 0, frameBgr.cols, frameBgr.rows);
    if (r.area() < 16)
        return {};
    // Inner 80 % of the box: less background around the object.
    const int dx = r.width / 10, dy = r.height / 10;
    if (r.width - 2 * dx >= 4 && r.height - 2 * dy >= 4)
        r = cv::Rect(r.x + dx, r.y + dy, r.width - 2 * dx, r.height - 2 * dy);

    cv::Mat hsv;
    cv::cvtColor(frameBgr(r), hsv, cv::COLOR_BGR2HSV);
    const int channels[] = { 0, 1 };
    const int bins[] = { 16, 4 };
    const float hRange[] = { 0.f, 180.f }, sRange[] = { 0.f, 256.f };
    const float* ranges[] = { hRange, sRange };
    cv::Mat hist;
    cv::calcHist(&hsv, 1, channels, cv::Mat(), hist, 2, bins, ranges);
    cv::normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);
    return hist;
}

double ObjectTracker::histSimilarity(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty() || a.size() != b.size())
        return -1.0;
    const double d = cv::compareHist(a, b, cv::HISTCMP_BHATTACHARYYA);   // 0 = identical
    return std::max(0.0, 1.0 - d);
}

// =============================================================================
// Camera motion
// =============================================================================
bool ObjectTracker::estimateCameraShift(const cv::Mat& frame, const std::vector<Detection>& dets,
                                        cv::Point2f& shift) {
    shift = { 0.f, 0.f };
    if (frame.empty()) {
        prevGray_.release();
        return false;
    }
    const double scale = std::min(1.0, static_cast<double>(cfg_.motionWorkWidth) / frame.cols);
    cv::Mat gray, small, f32;
    if (frame.channels() == 3)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frame;
    cv::resize(gray, small, cv::Size(), scale, scale, cv::INTER_AREA);
    small.convertTo(f32, CV_32F);

    std::vector<cv::Rect2f> boxes;
    boxes.reserve(dets.size());
    for (const auto& d : dets)
        boxes.push_back(d.box);

    bool ok = false;
    if (!prevGray_.empty() && prevGray_.size() == f32.size() && prevScale_ == scale) {
        // Objects must not drive the estimate: blank the boxes of BOTH passes
        // (a moving object sits in a different place in each) in both images.
        cv::Mat a = prevGray_.clone(), b = f32.clone();
        const cv::Scalar ma = cv::mean(a), mb = cv::mean(b);
        const cv::Rect imgRect(0, 0, f32.cols, f32.rows);
        auto blank = [&](const cv::Rect2f& r) {
            const float padX = 0.15f * r.width, padY = 0.15f * r.height;
            cv::Rect s(cv::Point(static_cast<int>(std::floor((r.x - padX) * scale)),
                                 static_cast<int>(std::floor((r.y - padY) * scale))),
                       cv::Point(static_cast<int>(std::ceil((r.x + r.width + padX) * scale)),
                                 static_cast<int>(std::ceil((r.y + r.height + padY) * scale))));
            s &= imgRect;
            if (s.area() > 0) { a(s).setTo(ma); b(s).setTo(mb); }
        };
        for (const auto& r : prevBoxes_) blank(r);
        for (const auto& r : boxes) blank(r);

        cv::Mat window;
        cv::createHanningWindow(window, f32.size(), CV_32F);
        double response = 0.0;
        // Shift of the current frame relative to the previous one: scene
        // content that was at p is now at p + s.
        const cv::Point2d s = cv::phaseCorrelate(a, b, window, &response);
        const bool plausible = std::abs(s.x) < 0.4 * f32.cols && std::abs(s.y) < 0.4 * f32.rows;
        if (response >= cfg_.motionMinResponse && plausible) {
            shift = { static_cast<float>(s.x / scale), static_cast<float>(s.y / scale) };
            ok = true;
        }
    }
    prevGray_ = f32;
    prevScale_ = scale;
    prevBoxes_ = std::move(boxes);
    return ok;
}

// =============================================================================
// Helpers
// =============================================================================
int ObjectTracker::groupOf(int classIndex) const {
    auto it = groups_.find(classIndex);
    return it != groups_.end() ? it->second : 1000 + classIndex;
}

const TrackInfo* ObjectTracker::find(uint64_t id) const {
    for (const auto& t : tracks_)
        if (t.id == id)
            return &t;
    return nullptr;
}

void ObjectTracker::reset() {
    tracks_.clear();
    prevGray_.release();
    prevBoxes_.clear();
}

cv::Rect2f ObjectTracker::predictBox(const TrackInfo& t, int64_t nowMs) const {
    const float dt = static_cast<float>(std::min<int64_t>(std::max<int64_t>(0, nowMs - t.lastSeenMs),
                                                          kMaxPredictMs));
    cv::Rect2f p = t.box;
    p.x += t.vx * dt;
    p.y += t.vy * dt;
    return p;
}

double ObjectTracker::gateFor(const TrackInfo& t, int64_t nowMs) const {
    const double lostSec = std::max<int64_t>(0, nowMs - t.lastSeenMs) / 1000.0;
    // Seen once: no velocity to predict with, so a fast vehicle can be
    // several of its own widths away on the next pass.
    const double base = t.hits <= 1 ? std::max(cfg_.gateBase, cfg_.gateNoVelocity) : cfg_.gateBase;
    return base + cfg_.gatePerLostSec * std::min(3.0, lostSec);
}

double ObjectTracker::score(const TrackInfo& t, const cv::Rect2f& pred, const Detection& d,
                            const cv::Mat& dHist, int64_t nowMs, bool strict) const {
    if (t.group != groupOf(d.classIndex))
        return -1.0;
    const double predArea = std::max(1.f, pred.area());
    const double ratio = d.box.area() / predArea;
    if (ratio < 0.25 || ratio > 4.0)
        return -1.0;

    const double iou = boxIou(pred, d.box);
    const cv::Point2f cp = centerOf(pred), cd = centerOf(d.box);
    const double size = std::max(4.0, std::sqrt(predArea));
    const double dn = std::hypot(cd.x - cp.x, cd.y - cp.y) / size;   // in object sizes
    const double gate = gateFor(t, nowMs);

    if (iou < cfg_.minIou && dn > gate)
        return -1.0;
    if (strict && iou < 0.3 && dn > 0.5 * gate)
        return -1.0;

    const double distTerm = std::max(0.0, 1.0 - dn / gate);
    const double app = histSimilarity(t.hist, dHist);
    // The wide no-velocity gate only for a detection that also looks like
    // the track -- two unrelated false alarms must not chain into a track.
    if (t.hits <= 1 && iou < cfg_.minIou && dn > cfg_.gateBase && app >= 0.0 && app < 0.6)
        return -1.0;
    double num = cfg_.wIou * iou + cfg_.wDist * distTerm;
    double den = cfg_.wIou + cfg_.wDist;
    if (app >= 0.0) {
        num += cfg_.wAppearance * app;
        den += cfg_.wAppearance;
    }
    double s = num / den;
    if (d.classIndex != t.votedClass)
        s -= cfg_.classChangePenalty;
    return s;
}

void ObjectTracker::applyMatch(TrackInfo& t, const Detection& d, const cv::Mat& dHist,
                               int64_t nowMs) {
    const int64_t dt = nowMs - t.lastSeenMs;
    if (dt > 0) {
        // Scene-relative motion: t.box was already moved by the camera
        // shift, so what is left is the object's own movement.
        const cv::Point2f c0 = centerOf(t.box), c1 = centerOf(d.box);
        const float mvx = (c1.x - c0.x) / static_cast<float>(dt);
        const float mvy = (c1.y - c0.y) / static_cast<float>(dt);
        if (t.hits <= 1) { t.vx = mvx; t.vy = mvy; }
        else { t.vx = 0.5f * t.vx + 0.5f * mvx; t.vy = 0.5f * t.vy + 0.5f * mvy; }
    }
    t.box = d.box;
    t.hits++;
    t.missedPasses = 0;
    t.lastSeenMs = nowMs;
    t.classVotes[d.classIndex] += d.confidence;
    t.votedClass = std::max_element(t.classVotes.begin(), t.classVotes.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->first;
    if (!dHist.empty()) {
        if (t.hist.empty() || t.hist.size() != dHist.size())
            t.hist = dHist.clone();
        else
            cv::addWeighted(t.hist, 1.0 - kHistEma, dHist, kHistEma, 0.0, t.hist);
    }
}

// =============================================================================
// update()
// =============================================================================
UpdateResult ObjectTracker::update(const cv::Mat& frame, const std::vector<Detection>& dets,
                                   int64_t nowMs) {
    UpdateResult res;
    res.assignments.resize(dets.size());

    // 1. Camera motion: move every track with the image.
    if (cfg_.compensateCameraMotion) {
        cv::Point2f s;
        res.cameraShiftValid = estimateCameraShift(frame, dets, s);
        if (res.cameraShiftValid) {
            res.cameraShift = s;
            for (auto& t : tracks_) { t.box.x += s.x; t.box.y += s.y; }
        }
    }

    // 2. Appearance of every usable detection, predictions of every track.
    std::vector<cv::Mat> hists(dets.size());
    for (size_t i = 0; i < dets.size(); ++i)
        if (dets[i].confidence >= cfg_.lowConfidence)
            hists[i] = computeHist(frame, dets[i].box);
    std::vector<cv::Rect2f> preds(tracks_.size());
    for (size_t k = 0; k < tracks_.size(); ++k)
        preds[k] = predictBox(tracks_[k], nowMs);

    std::vector<bool> detTaken(dets.size(), false), trackTaken(tracks_.size(), false);
    std::vector<TrackState> stateBefore(tracks_.size());
    for (size_t k = 0; k < tracks_.size(); ++k)
        stateBefore[k] = tracks_[k].state;

    auto runStage = [&](bool lowStage) {
        std::vector<std::tuple<double, size_t, size_t>> cand;
        for (size_t i = 0; i < dets.size(); ++i) {
            if (detTaken[i])
                continue;
            const float c = dets[i].confidence;
            const bool high = c >= highFor(dets[i].classIndex);
            if (lowStage ? (high || c < cfg_.lowConfidence) : !high)
                continue;
            for (size_t k = 0; k < tracks_.size(); ++k) {
                if (trackTaken[k])
                    continue;
                if (lowStage && tracks_[k].state == TrackState::Tentative)
                    continue;   // weak detections only extend real tracks
                const double s = score(tracks_[k], preds[k], dets[i], hists[i], nowMs, lowStage);
                if (s >= cfg_.minScore)
                    cand.emplace_back(s, i, k);
            }
        }
        std::sort(cand.begin(), cand.end(),
                  [](const auto& a, const auto& b) { return std::get<0>(a) > std::get<0>(b); });
        for (const auto& [s, i, k] : cand) {
            if (detTaken[i] || trackTaken[k])
                continue;
            detTaken[i] = trackTaken[k] = true;
            TrackInfo& t = tracks_[k];
            applyMatch(t, dets[i], hists[i], nowMs);
            if (t.state == TrackState::Tentative &&
                (t.hits >= cfg_.confirmHits || dets[i].confidence >= cfg_.instantConfirm))
                t.state = TrackState::Confirmed;
            else if (t.state == TrackState::Lost)
                t.state = TrackState::Confirmed;
            Assignment& a = res.assignments[i];
            a.trackId = t.id;
            a.confirmed = t.state == TrackState::Confirmed;
            a.newlyConfirmed = a.confirmed && stateBefore[k] == TrackState::Tentative;
            a.lowConfidenceMatch = lowStage;
            a.votedClass = t.votedClass;
        }
    };
    runStage(false);   // confident detections vs every track
    runStage(true);    // weak detections may keep confirmed tracks alive

    // 3. Unmatched tracks age; tentative ones die fast, confirmed ones get
    //    a time-based grace period.
    std::vector<TrackInfo> kept;
    kept.reserve(tracks_.size() + dets.size());
    for (size_t k = 0; k < tracks_.size(); ++k) {
        TrackInfo& t = tracks_[k];
        if (!trackTaken[k]) {
            t.missedPasses++;
            const int64_t unseen = nowMs - t.lastSeenMs;
            if (t.state == TrackState::Tentative) {
                if (t.missedPasses >= 2 || unseen > cfg_.tentativeMaxMs)
                    continue;                                  // silently dropped
            }
            else {
                t.state = TrackState::Lost;
                if (unseen > cfg_.maxLostMs && t.missedPasses >= cfg_.minLostPasses) {
                    res.removed.push_back(t);
                    continue;
                }
            }
        }
        kept.push_back(std::move(t));
    }
    tracks_ = std::move(kept);

    // 4. Confident, unmatched detections start new tracks.
    for (size_t i = 0; i < dets.size(); ++i) {
        if (detTaken[i] || dets[i].confidence < highFor(dets[i].classIndex))
            continue;
        TrackInfo t;
        t.id = nextId_++;
        t.group = groupOf(dets[i].classIndex);
        t.votedClass = dets[i].classIndex;
        t.box = dets[i].box;
        t.firstSeenMs = t.lastSeenMs = nowMs;
        t.hits = 1;
        t.classVotes[dets[i].classIndex] = dets[i].confidence;
        t.hist = hists[i];
        t.state = dets[i].confidence >= cfg_.instantConfirm || cfg_.confirmHits <= 1
                      ? TrackState::Confirmed : TrackState::Tentative;
        Assignment& a = res.assignments[i];
        a.trackId = t.id;
        a.confirmed = t.state == TrackState::Confirmed;
        a.newlyConfirmed = a.confirmed;
        a.votedClass = t.votedClass;
        tracks_.push_back(std::move(t));
    }
    return res;
}

} // namespace trk
