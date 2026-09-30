#pragma once
// =============================================================================
// ObjectTracker — multi-object tracking for the YOLO detections
//
// Keeps ONE identity per physical object across detection passes, so the
// same parked car / walking person / passing vehicle is not logged as a
// "new detection" on every pass. Real-time: a few milliseconds per pass for
// a handful of objects, OpenCV core + imgproc only, no Win32, no threads —
// unit-tested on any machine (tests/tracker_sim.cpp).
//
// Why the old IoU-only matcher lost identities (and what replaces it):
//
//   Detection runs every 250 ms at best (seconds per pass on CPU). Between
//   passes a crossing car moves more than its own width, and a drone yaw
//   shifts the WHOLE image — the new box no longer overlaps the old one,
//   so IoU = 0 → new track → new record. Class flicker (car ↔
//   other_vehicle) and one weak pass below the confidence threshold did the
//   same.
//
//   1. Camera-motion compensation: global image shift between passes via
//      cv::phaseCorrelate on a small grayscale copy; every track is moved
//      by it before matching. The detection boxes of both passes are masked
//      out first — otherwise a large moving vehicle on a plain background
//      is mistaken for a camera pan and drags every track with it.
//   2. Motion prediction: each track has a velocity (alpha-beta filter,
//      real time based), so it is predicted to where it should be now.
//   3. Association score = IoU + size-normalised centre distance +
//      colour-histogram appearance, with a gate that widens the longer a
//      track has not been seen. Greedy best-first assignment.
//   4. Class groups: classes that the model confuses (all vehicles) may
//      match each other with a penalty; the reported class is a
//      confidence-weighted vote over the track's history.
//   5. Two stages (ByteTrack): confident detections are matched first;
//      low-confidence detections may then only EXTEND an existing
//      confirmed track (a weak pass no longer kills it) — they never start
//      a new one.
//   6. Life cycle: TENTATIVE → CONFIRMED after `confirmHits` matches (or at
//      once when very confident) — single-frame false positives never
//      become records. A confirmed track that is not seen is LOST and still
//      predicted/matchable for `maxLostMs` (time, not passes), then
//      removed (reported in UpdateResult::removed so the caller can archive
//      it for geographic re-identification).
// =============================================================================
#include <opencv2/core.hpp>
#include <cstdint>
#include <map>
#include <vector>

namespace trk {

struct Detection {
    int   classIndex = -1;
    float confidence = 0.0f;
    cv::Rect2f box;              // pixel space of the frame passed to update()
};

struct Config {
    float highConfidence   = 0.45f;  // may start tracks (DetectionLink: its confidence threshold)
    float lowConfidence    = 0.15f;  // may only extend confirmed tracks
    float instantConfirm   = 0.70f;  // confidence that confirms a new track immediately
    int   confirmHits      = 2;      // matches needed to confirm otherwise
    int64_t tentativeMaxMs = 2500;   // an unconfirmed track dies if unmatched this long
    int64_t maxLostMs      = 5000;   // a confirmed track survives unseen this long
    int   minLostPasses    = 6;      // ...and at least this many passes (slow CPU passes)

    float minIou           = 0.10f;  // below this, only centre distance can match
    float gateBase         = 1.5f;   // centre-distance gate, in object sizes
    float gatePerLostSec   = 1.0f;   // gate widening per second unseen (max +3)
    float gateNoVelocity   = 3.5f;   // gate of a track seen once (no velocity yet)
    float minScore         = 0.20f;  // association score needed to match
    float classChangePenalty = 0.10f;
    float wIou = 0.45f, wDist = 0.35f, wAppearance = 0.20f;

    bool  compensateCameraMotion = true;
    int   motionWorkWidth  = 320;    // phase correlation works on this width
    double motionMinResponse = 0.08; // below: shift estimate ignored
};

enum class TrackState { Tentative, Confirmed, Lost };

struct TrackInfo {
    uint64_t id = 0;
    TrackState state = TrackState::Tentative;
    int   group = 0;
    int   votedClass = -1;           // confidence-weighted majority class
    cv::Rect2f box;                  // last matched (or predicted, if lost) box
    float vx = 0.0f, vy = 0.0f;      // px per ms, scene-relative
    int   hits = 0;
    int   missedPasses = 0;
    int64_t firstSeenMs = 0, lastSeenMs = 0;
    cv::Mat hist;                    // appearance (HSV histogram), may be empty
    std::map<int, float> classVotes;
};

struct Assignment {
    uint64_t trackId = 0;            // 0 = detection not assigned to any track
    bool confirmed = false;          // track is confirmed (records allowed)
    bool newlyConfirmed = false;     // confirmed on THIS pass (first record)
    bool lowConfidenceMatch = false; // matched in the second (low-confidence) stage
    int  votedClass = -1;
};

struct UpdateResult {
    std::vector<Assignment> assignments;   // index-parallel to the detections
    std::vector<TrackInfo>  removed;       // tracks deleted this pass
    cv::Point2f cameraShift{ 0.f, 0.f };   // estimated global image shift (px)
    bool cameraShiftValid = false;
};

class ObjectTracker {
public:
    explicit ObjectTracker(const Config& cfg = Config()) : cfg_(cfg) {}

    Config& config() { return cfg_; }
    const Config& config() const { return cfg_; }

    // classIndex → group. Classes in the same group may match each other
    // (with classChangePenalty). Unlisted classes form their own group.
    void setClassGroups(const std::map<int, int>& groups) { groups_ = groups; }

    // One detection pass. `frame` (BGR, may be empty → no camera-motion
    // compensation / appearance) is the frame the detections came from;
    // `nowMs` any monotonic millisecond clock.
    UpdateResult update(const cv::Mat& frame, const std::vector<Detection>& detections,
                        int64_t nowMs);

    const std::vector<TrackInfo>& tracks() const { return tracks_; }
    const TrackInfo* find(uint64_t id) const;
    void reset();

    // Appearance similarity 0..1 (1 = identical), -1 if either is empty.
    static double histSimilarity(const cv::Mat& a, const cv::Mat& b);
    static cv::Mat computeHist(const cv::Mat& frameBgr, const cv::Rect2f& box);

private:
    int groupOf(int classIndex) const;
    cv::Rect2f predictBox(const TrackInfo& t, int64_t nowMs) const;
    double gateFor(const TrackInfo& t, int64_t nowMs) const;
    double score(const TrackInfo& t, const cv::Rect2f& pred, const Detection& d,
                 const cv::Mat& dHist, int64_t nowMs, bool strict) const;
    void applyMatch(TrackInfo& t, const Detection& d, const cv::Mat& dHist, int64_t nowMs);
    bool estimateCameraShift(const cv::Mat& frame, const std::vector<Detection>& dets,
                             cv::Point2f& shift);

    Config cfg_;
    std::map<int, int> groups_;
    std::vector<TrackInfo> tracks_;
    uint64_t nextId_ = 1;
    cv::Mat prevGray_;               // for phase correlation (float, work size)
    double prevScale_ = 1.0;         // work size / frame size
    std::vector<cv::Rect2f> prevBoxes_;  // detections of the previous pass (masked out)
};

double boxIou(const cv::Rect2f& a, const cv::Rect2f& b);

} // namespace trk
