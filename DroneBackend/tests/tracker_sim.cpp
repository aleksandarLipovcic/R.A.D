// =============================================================================
// tracker_sim.cpp — synthetic benchmark: old IoU-only matcher vs ObjectTracker
//
// Renders a textured ground seen by a (possibly panning) drone camera with
// coloured moving objects, simulates an imperfect detector (box jitter,
// misses, confidence dips, class flicker, false alarms) and feeds the SAME
// detections to both trackers. Reports, per scenario:
//
//   ids/object   distinct track IDs given to each real object (ideal 1.00)
//   new records  distinct IDs that produced a "new object" record
//                (old: every new ID; new: every confirmed ID) vs real objects
//   impure       IDs that were given to more than one real object (swaps)
//   FP records   records created by false alarms
//
// Build (Linux, OpenCV 4):
//   g++ -std=c++17 -O2 -I.. tracker_sim.cpp ../ObjectTracker.cpp \
//       $(pkg-config --cflags --libs opencv4) -o tracker_sim && ./tracker_sim
// Exit code 0 only if the new tracker is at least as good as the old one on
// every scenario and within the per-scenario limits.
// =============================================================================
#include "ObjectTracker.h"
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {

// ── Old matcher: faithful copy of DetectionLink's matchRawToTracks /
//    updateTracks (greedy IoU >= 0.3, identical class, detections below the
//    confidence threshold dropped, deleted after 6 missed passes). ─────────
struct OldTrack { uint64_t id; int cls; cv::Rect2f box; int missed; };
struct OldTracker {
    std::vector<OldTrack> tracks;
    uint64_t next = 1;
    float confThr = 0.45f, iouThr = 0.3f;
    int maxMissed = 6;
    // returns id per detection (0 = filtered out), and whether it's new
    std::vector<std::pair<uint64_t, bool>> update(const std::vector<trk::Detection>& d) {
        std::vector<std::pair<uint64_t, bool>> out(d.size(), { 0, false });
        std::vector<bool> taken(tracks.size(), false), used(d.size(), false);
        while (true) {
            double best = iouThr; int bi = -1, bk = -1;
            for (size_t i = 0; i < d.size(); ++i) {
                if (used[i] || d[i].confidence < confThr) continue;
                for (size_t k = 0; k < tracks.size(); ++k) {
                    if (taken[k] || tracks[k].cls != d[i].classIndex) continue;
                    double v = trk::boxIou(d[i].box, tracks[k].box);
                    if (v > best) { best = v; bi = int(i); bk = int(k); }
                }
            }
            if (bi < 0) break;
            used[bi] = taken[bk] = true;
            tracks[bk].box = d[bi].box; tracks[bk].missed = 0;
            out[bi] = { tracks[bk].id, false };
        }
        size_t orig = tracks.size();
        for (size_t i = 0; i < d.size(); ++i) {
            if (used[i] || d[i].confidence < confThr) continue;
            tracks.push_back({ next, d[i].classIndex, d[i].box, 0 });
            out[i] = { next++, true };
        }
        std::vector<OldTrack> kept;
        for (size_t k = 0; k < tracks.size(); ++k) {
            if (k < orig && !taken[k]) tracks[k].missed++;
            if (tracks[k].missed <= maxMissed) kept.push_back(tracks[k]);
        }
        tracks = kept;
        return out;
    }
};

// ── World ─────────────────────────────────────────────────────────────────
constexpr int W = 1280, H = 720;
enum Cls { PERSON = 0, CAR = 1, LARGE = 2, MOTO = 3, OTHER = 4 };

struct Obj {
    int id; int cls;
    cv::Point2f pos;      // world px (top-left)
    cv::Point2f vel;      // world px per ms
    cv::Size2f size;
    cv::Scalar color;
    int64_t hideFrom = -1, hideTo = -1;   // occlusion window (ms)
    int64_t appearAt = 0, leaveAt = 1LL << 40;
};

struct Scenario {
    std::string name;
    int64_t durationMs = 20000, passMs = 250;
    cv::Point2f camVel{ 0, 0 };          // camera pan, world px per ms
    float missProb = 0.05f, dipProb = 0.0f, flickerProb = 0.0f, fpPerPass = 0.0f;
    float jitter = 0.03f;
    std::vector<Obj> objs;
    double maxIdsPerObj = 1.3;           // pass limit for the new tracker
};

cv::Mat makeGround(std::mt19937& rng) {
    cv::Mat g(2600, 6200, CV_8UC3);
    cv::randu(g, cv::Scalar(40, 60, 40), cv::Scalar(120, 150, 110));
    cv::GaussianBlur(g, g, cv::Size(0, 0), 3.0);
    std::uniform_int_distribution<int> x(0, 6199), y(0, 2599), s(20, 160), c(30, 200);
    for (int i = 0; i < 1400; ++i)   // fields, paths, bushes: texture to lock on to
        cv::rectangle(g, cv::Rect(x(rng), y(rng), s(rng), s(rng)),
                      cv::Scalar(c(rng), c(rng), c(rng)), cv::FILLED);
    cv::GaussianBlur(g, g, cv::Size(0, 0), 1.2);
    return g;
}

struct Metrics {
    double idsPerObj = 0; int newRecords = 0, realObjs = 0, impure = 0, fpRecords = 0;
};

struct Result { Metrics oldM, newM; double msPerPass = 0; };

Result run(const Scenario& sc, unsigned seed) {
    std::mt19937 rng(seed);
    cv::Mat ground = makeGround(rng);
    std::uniform_real_distribution<float> U(0.f, 1.f);
    std::normal_distribution<float> N(0.f, 1.f);

    OldTracker oldT;
    trk::ObjectTracker newT;
    newT.setClassGroups({ { PERSON, 0 }, { CAR, 1 }, { LARGE, 1 }, { MOTO, 1 }, { OTHER, 1 } });

    std::map<int, std::set<uint64_t>> oldIds, newIds;      // gt -> ids
    std::map<uint64_t, std::set<int>> oldOwners, newOwners; // id -> gt
    std::set<uint64_t> oldNew, newConfirmed, oldFp, newFp;
    std::set<int> seen;
    double trackMs = 0; int passes = 0;

    cv::Point2f cam(1400, 800);
    for (int64_t t = 0; t <= sc.durationMs; t += sc.passMs) {
        cv::Point2f camNow = cam + sc.camVel * static_cast<float>(t);
        cv::Rect view(static_cast<int>(camNow.x), static_cast<int>(camNow.y), W, H);
        cv::Mat frame = ground(view).clone();

        std::vector<trk::Detection> dets; std::vector<int> gtOf;
        for (const auto& o : sc.objs) {
            if (t < o.appearAt || t > o.leaveAt) continue;
            cv::Point2f p = o.pos + o.vel * static_cast<float>(t - o.appearAt) - camNow;
            cv::Rect2f r(p, o.size);
            cv::Rect ri(r); ri &= cv::Rect(0, 0, W, H);
            if (ri.area() < 0.4 * r.area()) continue;            // mostly out of view
            cv::rectangle(frame, ri, o.color, cv::FILLED);
            cv::rectangle(frame, cv::Rect(ri.x + ri.width / 4, ri.y + ri.height / 4,
                          ri.width / 2, ri.height / 3), o.color * 0.6, cv::FILLED);
            if (o.hideFrom >= 0 && t >= o.hideFrom && t <= o.hideTo) continue;   // occluded
            if (U(rng) < sc.missProb) continue;
            float conf = 0.62f + 0.15f * U(rng);
            if (U(rng) < sc.dipProb) conf = 0.22f + 0.15f * U(rng);           // weak pass
            int cls = o.cls;
            if (o.cls != PERSON && U(rng) < sc.flickerProb) cls = (U(rng) < 0.5f) ? OTHER : LARGE;
            const float j = sc.jitter;
            cv::Rect2f b(r.x + N(rng) * j * r.width, r.y + N(rng) * j * r.height,
                         r.width * (1 + N(rng) * j), r.height * (1 + N(rng) * j));
            dets.push_back({ cls, conf, b }); gtOf.push_back(o.id); seen.insert(o.id);
        }
        // false alarms: random single-pass boxes
        int nfp = static_cast<int>(sc.fpPerPass) + (U(rng) < (sc.fpPerPass - int(sc.fpPerPass)) ? 1 : 0);
        for (int f = 0; f < nfp; ++f) {
            dets.push_back({ CAR, 0.5f + 0.15f * U(rng),
                             cv::Rect2f(U(rng) * (W - 80), U(rng) * (H - 60), 70, 45) });
            gtOf.push_back(-1);
        }

        auto o = oldT.update(dets);
        const int64 t0 = cv::getTickCount();
        auto n = newT.update(frame, dets, t);
        trackMs += (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency(); passes++;

        for (size_t i = 0; i < dets.size(); ++i) {
            const int g = gtOf[i];
            if (o[i].first) {
                if (o[i].second) { oldNew.insert(o[i].first); if (g < 0) oldFp.insert(o[i].first); }
                if (g >= 0) { oldIds[g].insert(o[i].first); oldOwners[o[i].first].insert(g); }
            }
            if (n.assignments[i].trackId && n.assignments[i].confirmed) {
                if (n.assignments[i].newlyConfirmed) {
                    newConfirmed.insert(n.assignments[i].trackId);
                    if (g < 0) newFp.insert(n.assignments[i].trackId);
                }
                if (g >= 0) { newIds[g].insert(n.assignments[i].trackId);
                              newOwners[n.assignments[i].trackId].insert(g); }
            }
        }
    }
    auto summarize = [&](std::map<int, std::set<uint64_t>>& ids, std::map<uint64_t, std::set<int>>& own,
                         std::set<uint64_t>& recs, std::set<uint64_t>& fp) {
        Metrics m; double s = 0;
        for (int g : seen) s += std::max<size_t>(1, ids[g].size());
        m.realObjs = static_cast<int>(seen.size());
        m.idsPerObj = seen.empty() ? 0 : s / seen.size();
        m.newRecords = static_cast<int>(recs.size());
        for (auto& [id, g] : own) if (g.size() > 1) m.impure++;
        m.fpRecords = static_cast<int>(fp.size());
        return m;
    };
    Result r;
    r.oldM = summarize(oldIds, oldOwners, oldNew, oldFp);
    r.newM = summarize(newIds, newOwners, newConfirmed, newFp);
    r.msPerPass = passes ? trackMs / passes : 0;
    return r;
}

Obj car(int id, float x, float y, float vx, float vy, cv::Scalar c, int cls = CAR) {
    Obj o; o.id = id; o.cls = cls; o.pos = { x, y }; o.vel = { vx, vy };
    o.size = cls == PERSON ? cv::Size2f(18, 42) : cv::Size2f(70, 38); o.color = c; return o;
}

} // namespace

int main() {
    const cv::Scalar red(40, 40, 220), white(235, 235, 235), blue(200, 90, 30),
                     yellow(40, 210, 230), dark(30, 30, 30), orange(20, 120, 240);
    std::vector<Scenario> scs;
    // Absolute world positions: camera starts at (1400, 800).
    { Scenario s; s.name = "crossing cars, 4 Hz";
      s.objs = { car(1, 1400, 1100, 0.30f, 0, red), car(2, 2650, 1300, -0.25f, 0, white),
                 car(3, 1500, 1350, 0.18f, 0.02f, blue) };
      scs.push_back(s); }
    { Scenario s; s.name = "static objects, drone yaw/pan";
      s.camVel = { 0.16f, 0.03f };
      s.objs = { car(1, 1900, 1100, 0, 0, red), car(2, 2300, 1200, 0, 0, white),
                 car(3, 2600, 1000, 0, 0, yellow, PERSON), car(4, 2100, 1350, 0, 0, blue) };
      scs.push_back(s); }
    { Scenario s; s.name = "class flicker + confidence dips";
      s.flickerProb = 0.3f; s.dipProb = 0.25f;
      s.objs = { car(1, 1700, 1100, 0.05f, 0, red), car(2, 2200, 1250, -0.04f, 0, white),
                 car(3, 2000, 1000, 0.02f, 0.01f, yellow, PERSON) };
      scs.push_back(s); }
    { Scenario s; s.name = "occlusion 2 s (tree / building)";
      Obj a = car(1, 1500, 1100, 0.12f, 0, red); a.hideFrom = 6000; a.hideTo = 8000;
      Obj b = car(2, 2500, 1250, -0.10f, 0, white); b.hideFrom = 11000; b.hideTo = 13000;
      s.objs = { a, b }; scs.push_back(s); }
    { Scenario s; s.name = "two cars passing each other";
      s.objs = { car(1, 1500, 1150, 0.20f, 0, red), car(2, 2600, 1170, -0.20f, 0, white) };
      s.maxIdsPerObj = 1.5; scs.push_back(s); }
    { Scenario s; s.name = "slow CPU passes (1.5 s) + pan";
      s.passMs = 1500; s.durationMs = 40000; s.camVel = { 0.04f, 0 };
      s.objs = { car(1, 1600, 1100, 0.03f, 0, red), car(2, 2300, 1250, 0, 0, orange),
                 car(3, 2000, 1000, 0.008f, 0, yellow, PERSON) };
      scs.push_back(s); }
    { Scenario s; s.name = "false alarms (0.3 per pass)";
      s.fpPerPass = 0.3f;
      s.objs = { car(1, 1800, 1100, 0.05f, 0, red), car(2, 2200, 1200, 0, 0, dark) };
      scs.push_back(s); }

    // ── Stress scenarios: looking for the limits ─────────────────────────
    { Scenario s; s.name = "parking lot: 12 identical cars";
      s.camVel = { 0.05f, 0.0f }; s.missProb = 0.10f;
      for (int i = 0; i < 12; ++i)
          s.objs.push_back(car(i + 1, 1700 + (i % 6) * 85.f, 1100 + (i / 6) * 60.f, 0, 0, white));
      s.maxIdsPerObj = 1.3; scs.push_back(s); }
    { Scenario s; s.name = "fast vehicles (0.6 px/ms = 2x width/pass)";
      s.objs = { car(1, 1300, 1100, 0.6f, 0, red), car(2, 2800, 1300, -0.55f, 0.05f, blue) };
      s.durationMs = 6000; s.maxIdsPerObj = 1.5; scs.push_back(s); }
    { Scenario s; s.name = "20% missed detections + dips";
      s.missProb = 0.20f; s.dipProb = 0.2f;
      s.objs = { car(1, 1600, 1100, 0.15f, 0, red), car(2, 2400, 1250, -0.1f, 0, white),
                 car(3, 2000, 1000, 0.03f, 0.01f, yellow, PERSON) };
      scs.push_back(s); }
    { Scenario s; s.name = "sharp drone turn (0.5 px/ms pan)";
      s.camVel = { 0.5f, 0.0f }; s.durationMs = 6000;
      s.objs = { car(1, 2000, 1100, 0, 0, red), car(2, 2600, 1250, 0, 0, white),
                 car(3, 3200, 1000, 0, 0, yellow, PERSON), car(4, 3800, 1200, 0, 0, blue) };
      s.maxIdsPerObj = 1.5; scs.push_back(s); }
    { Scenario s; s.name = "long occlusion 7 s (> 5 s memory)";
      Obj a = car(1, 1500, 1100, 0.05f, 0, red); a.hideFrom = 5000; a.hideTo = 12000;
      s.objs = { a }; s.maxIdsPerObj = 2.0; scs.push_back(s); }
    { Scenario s; s.name = "pedestrians crossing (same colour)";
      s.objs = { car(1, 1900, 1100, 0.03f, 0, dark, PERSON), car(2, 2150, 1105, -0.03f, 0, dark, PERSON),
                 car(3, 2000, 1180, 0.0f, -0.02f, dark, PERSON) };
      s.maxIdsPerObj = 1.5; scs.push_back(s); }

    printf("%-34s | %-26s | %-26s | %s\n", "scenario", "OLD ids/obj  new-rec  swaps",
           "NEW ids/obj  new-rec  swaps", "FP rec old/new  ms/pass");
    bool ok = true;
    for (const auto& sc : scs) {
        Result avg; int runs = 5; Metrics o{}, n{}; double ms = 0;
        for (int r = 0; r < runs; ++r) {
            Result x = run(sc, 1000 + r * 17);
            o.idsPerObj += x.oldM.idsPerObj / runs; n.idsPerObj += x.newM.idsPerObj / runs;
            o.newRecords += x.oldM.newRecords; n.newRecords += x.newM.newRecords;
            o.impure += x.oldM.impure; n.impure += x.newM.impure;
            o.fpRecords += x.oldM.fpRecords; n.fpRecords += x.newM.fpRecords;
            o.realObjs = n.realObjs = x.oldM.realObjs; ms += x.msPerPass / runs;
        }
        printf("%-34s | %6.2f   %5.1f/%-2d %5.1f   | %6.2f   %5.1f/%-2d %5.1f   | %5.1f / %-5.1f   %5.2f\n",
               sc.name.c_str(), o.idsPerObj, o.newRecords / double(runs), o.realObjs,
               o.impure / double(runs), n.idsPerObj, n.newRecords / double(runs), n.realObjs,
               n.impure / double(runs), o.fpRecords / double(runs), n.fpRecords / double(runs), ms);
        if (n.idsPerObj > sc.maxIdsPerObj || n.idsPerObj > o.idsPerObj + 1e-9 ||
            n.impure > o.impure + runs || n.fpRecords > o.fpRecords) {
            printf("   ^^^ FAIL\n"); ok = false;
        }
    }
    printf(ok ? "\nALL SCENARIOS PASS\n" : "\nSOME SCENARIOS FAIL\n");
    return ok ? 0 : 1;
}
