// =============================================================================
// detectionlink_run -- runs the REAL DetectionLink (model + parser + tracker)
// on one image outside the cockpit and prints the records it writes.
//
// Use it to check a new ONNX export before flying it: both layouts
// (end-to-end [1,N,6] and raw [1,4+nc,anchors]) must give the same boxes as
// Ultralytics' predict() on the same image.
//
//   Linux (OpenCV >= 4.10 with dnn; videoio not needed):
//     g++ -std=c++17 -O2 -I.. detectionlink_run.cpp ../Detectionlink.cpp ../ObjectTracker.cpp \
//         $(pkg-config --cflags --libs opencv4) -lpthread -o detectionlink_run
//   (case-sensitive file systems: ln -s Detectionlink.h ../DetectionLink.h)
//
//   ./detectionlink_run ../../DroneCockpitUI/models/yolo26m_main.onnx frame.jpg 960
//
// Static image for ~2.5 s = ~50 passes: each object should appear ONCE
// (one confirmed track, no repeated records).
// =============================================================================
#include "DetectionLink.h"
#include "VideoLink.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

cv::Mat VideoLink::getLatestFrame() { return {}; }   // link-only stub: frames come from setFrameSource

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("usage: %s model.onnx image.jpg [input_size=640]\n", argv[0]);
        return 2;
    }
    cv::Mat img = cv::imread(argv[2]);
    if (img.empty()) { printf("cannot read %s\n", argv[2]); return 1; }
    DetectionLink dl;
    dl.setModelPath(argv[1]);
    dl.setInputSize(argc > 3 ? std::atoi(argv[3]) : 640);
    dl.setConfidenceThreshold(0.25f);
    dl.setDetectionIntervalMs(50);
    dl.setFrameSource([img]() { return img.clone(); });
    if (!dl.start()) { printf("start failed (model not loadable?)\n"); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    dl.stop();
    printf("last pass %.1f ms\n", dl.getLastPassDurationMs());
    for (const auto& r : dl.getAllRecords())
        printf("%-14s %.3f  x1=%4d y1=%4d x2=%4d y2=%4d  track=%llu\n", r.className.c_str(), r.confidence,
               r.bboxX, r.bboxY, r.bboxX + r.bboxW, r.bboxY + r.bboxH, (unsigned long long)r.trackId);
    return 0;
}
