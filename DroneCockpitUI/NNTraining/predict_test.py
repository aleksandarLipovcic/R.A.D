"""
predict_test.py — Visual sanity check before trusting the model in flight
=======================================================================
Metrics on VisDrone/UAVDT/SARD's own val splits don't tell you how the
model behaves on YOUR actual camera/altitude/lighting -- none of those
three are the real E5-FPV analog feed. Run this against a real test-
flight video and review the annotated output plus the per-frame CSV log
below before trusting the model in the air.

Usage:
    python predict_test.py --weights runs/detect/yolo26s_960-4/weights/best.pt \\
        --source path/to/test_flight.mp4 --conf 0.35

    # Hide a specific class's output entirely (e.g. if a class comes
    # back near-non-functional in a validation run -- see NOTE below):
    python predict_test.py --weights best.pt --source test_flight.mp4 \\
        --hide-classes other_vehicle

WHAT THIS PRODUCES for a video source:
  - The usual Ultralytics annotated video, saved under --out.
  - detections.csv under --out: one row per frame, with a count per
    class and that frame's mean detection confidence per class. This is
    the thing to actually open and scroll/plot afterward -- eyeballing
    a printed line per frame doesn't scale past a handful of images,
    and a video test-flight clip can be hundreds to thousands of
    frames. Frame numbers in the CSV match the video's frame order
    (starting at 0), not wall-clock timestamps -- divide by the
    source video's FPS yourself if you need time offsets.
  - A per-class AGGREGATE summary printed at the end (total detections,
    how many frames that class appeared in, mean confidence) -- the
    number to actually look at first for "is this model behaving,"
    rather than scrolling per-frame output.
  - A count of frames with ZERO detections. On a real flight clip with
    people/vehicles continuously in frame, a high zero-detection count
    is itself a signal worth investigating (missed detections, or the
    --conf threshold is too high for this footage) even before you
    look at any individual frame.

NOTE on --hide-classes: earlier versions of this project trained
against a taxonomy that included xView-derived classes (shed,
parking_lot) which validated as essentially unlearned (mAP50 ~0.001-
0.016) -- hidden by default at the time so a pilot never saw a
"detection" that was really just noise. xView is now inactive and
those classes aren't part of the current unified taxonomy (person/car/
large_vehicle/motorcycle/other_vehicle), so nothing is hidden by
default anymore. Per the current unified-taxonomy baseline (5-epoch
smoke test, blended val mAP50-95): person 0.215, car 0.517,
large_vehicle 0.373, motorcycle 0.154, other_vehicle 0.090 -- all of
these are weak-but-functional at this early stage, not non-functional
like shed/parking_lot were, so there's no clear candidate to hide by
default anymore. If a class ends up looking genuinely broken on real
footage (visibly wrong boxes, not just low recall), pass
--hide-classes explicitly for that test rather than trusting a
stale default -- e.g. --hide-classes other_vehicle given it's
currently the weakest class.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

from ultralytics import YOLO

# Nothing hidden by default -- see the NOTE in the module docstring for
# why the old shed/parking_lot default no longer applies to the current
# unified taxonomy. Pass --hide-classes explicitly per test run instead.
DEFAULT_HIDDEN = []


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--weights", required=True,
                    help="Path to best.pt (or best.onnx) to test.")
    p.add_argument("--source", required=True,
                    help="Folder of images, a single image, or a video "
                         "file/stream -- anything Ultralytics predict() "
                         "accepts. For a real test-flight clip, pass the "
                         "video file directly.")
    p.add_argument("--imgsz", type=int, default=960)
    p.add_argument("--conf", type=float, default=0.35,
                    help="Confidence threshold. Start higher than "
                         "training-time defaults (0.25) for a first look "
                         "-- easier to judge real precision without a "
                         "flood of low-confidence boxes.")
    p.add_argument("--hide-classes", nargs="*", default=DEFAULT_HIDDEN,
                    help=f"Class names to exclude from output entirely. "
                         f"Default: none (see NOTE in module docstring). "
                         f"Pass one or more class names to hide them for "
                         f"this test run.")
    p.add_argument("--out", default="runs/predict_test",
                    help="Where annotated output and detections.csv get "
                         "saved.")
    p.add_argument("--quiet", action="store_true",
                    help="Suppress the per-frame console line. Useful "
                         "for a long video where hundreds/thousands of "
                         "printed lines aren't reviewable anyway -- the "
                         "aggregate summary and detections.csv still get "
                         "written regardless of this flag.")
    return p.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.weights)

    all_names = model.names  # dict[int, str]
    keep_ids = [i for i, n in all_names.items() if n not in args.hide_classes]
    if args.hide_classes:
        print(f"Hiding classes: {args.hide_classes}")

    results = model.predict(
        source=args.source,
        imgsz=args.imgsz,
        conf=args.conf,
        classes=keep_ids or None,
        save=True,
        project=str(Path(args.out).parent),
        name=Path(args.out).name,
        exist_ok=True,
        stream=True,  # process frame-by-frame instead of loading a full
                       # video's results into memory at once -- matters
                       # once "video" means a real multi-minute clip
                       # rather than a handful of test images
    )

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "detections.csv"

    class_names_sorted = [all_names[i] for i in sorted(all_names.keys())]

    # Per-class aggregate accumulators, built up frame-by-frame below.
    total_count = defaultdict(int)
    frames_present = defaultdict(int)
    conf_sum = defaultdict(float)
    n_frames = 0
    n_zero_detection_frames = 0

    print(f"\n{'=' * 60}")
    if not args.quiet:
        print("Per-frame detection counts (spot-check these against "
              "what you'd actually expect to see):")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["frame", "source_name", "total_detections"]
            + [f"{name}_count" for name in class_names_sorted]
            + [f"{name}_mean_conf" for name in class_names_sorted]
        )

        for frame_idx, r in enumerate(results):
            n_frames += 1
            frame_counts = defaultdict(int)
            frame_conf_sum = defaultdict(float)

            n = len(r.boxes) if r.boxes is not None else 0
            if r.boxes is not None and n > 0:
                classes = r.boxes.cls.tolist()
                confs = r.boxes.conf.tolist()
                for c, conf in zip(classes, confs):
                    name = all_names[int(c)]
                    frame_counts[name] += 1
                    frame_conf_sum[name] += conf
                    total_count[name] += 1
                    conf_sum[name] += conf
                for name in frame_counts:
                    frames_present[name] += 1
            else:
                n_zero_detection_frames += 1

            if not args.quiet:
                print(f"  frame {frame_idx} ({Path(r.path).name}): "
                      f"{n} detections -- {dict(frame_counts)}")

            row = [frame_idx, Path(r.path).name, n]
            row += [frame_counts.get(name, 0) for name in class_names_sorted]
            row += [
                round(frame_conf_sum[name] / frame_counts[name], 4)
                if frame_counts.get(name) else ""
                for name in class_names_sorted
            ]
            writer.writerow(row)

    print(f"\n{'=' * 60}")
    print(f"Aggregate summary across {n_frames} frames")
    print(f"{'=' * 60}")
    for name in class_names_sorted:
        if total_count[name] == 0:
            print(f"  {name:15s}  0 detections across any frame")
            continue
        mean_conf = conf_sum[name] / total_count[name]
        pct_frames = 100 * frames_present[name] / n_frames if n_frames else 0
        print(f"  {name:15s}  {total_count[name]:>6} detections in "
              f"{frames_present[name]:>5}/{n_frames} frames "
              f"({pct_frames:5.1f}%)  mean_conf={mean_conf:.3f}")

    zero_pct = 100 * n_zero_detection_frames / n_frames if n_frames else 0
    print(f"\n  Frames with zero detections: {n_zero_detection_frames}/"
          f"{n_frames} ({zero_pct:.1f}%)")
    if zero_pct > 20:
        print("  NOTE: that's a substantial share of empty frames. On "
              "real flight footage with people/vehicles continuously in "
              "view, this is worth checking directly -- either the model "
              "is missing real detections, or --conf is set too high for "
              "this footage's actual confidence distribution.")

    print(f"\nAnnotated video saved under: {args.out}")
    print(f"Per-frame detection log: {csv_path}")
    print("Look specifically for: false positives on background clutter, "
          "missed obvious cars/people, and whether box tightness looks "
          "usable for a pilot overlay (not just 'technically overlapping "
          "the object'). The CSV is the better tool for spotting patterns "
          "across the whole clip (e.g. a stretch of frames with a "
          "confidence dip, or a class that only fires at certain "
          "altitudes/ranges) than scrolling annotated frames one by one.")


if __name__ == "__main__":
    main()