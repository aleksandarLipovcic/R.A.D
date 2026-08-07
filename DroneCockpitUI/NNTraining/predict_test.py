"""
predict_test.py — Visual sanity check before trusting the model in flight
=======================================================================
Metrics on VisDrone/xView's own val splits don't tell you how the model
behaves on YOUR actual camera/altitude/lighting -- neither dataset is a
drone-native match for R.A.D.'s real footage. Run this against a handful
of real frames (or a short test-flight clip) and eyeball the boxes.

Usage:
    python predict_test.py --weights runs/detect/yolo26s_960-2/weights/best.pt \\
        --source path/to/test_images_or_video --conf 0.35

    # Hide shed/parking_lot -- see NOTE below on why, until they're
    # retrained with better xView representation:
    python predict_test.py --weights best.pt --source test_clip.mp4 \\
        --hide-classes shed parking_lot
"""

import argparse
from pathlib import Path

from ultralytics import YOLO

# NOTE: shed (mAP50 0.001, recall 0.0) and parking_lot (mAP50 0.016,
# recall 0.018) are currently near-non-functional per the last validation
# run -- essentially unlearned, not just weak. Showing their output to a
# pilot as if it were a real detection risks eroding trust in the whole
# overlay. Default here is to hide both until retrained with more/better
# xView representation; override with --hide-classes if that changes.
DEFAULT_HIDDEN = ["shed", "parking_lot"]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--weights", required=True,
                    help="Path to best.pt (or best.onnx) to test.")
    p.add_argument("--source", required=True,
                    help="Folder of images, a single image, or a video "
                         "file/stream -- anything Ultralytics predict() "
                         "accepts.")
    p.add_argument("--imgsz", type=int, default=960)
    p.add_argument("--conf", type=float, default=0.35,
                    help="Confidence threshold. Start higher than "
                         "training-time defaults (0.25) for a first look "
                         "-- easier to judge real precision without a "
                         "flood of low-confidence boxes.")
    p.add_argument("--hide-classes", nargs="*", default=DEFAULT_HIDDEN,
                    help=f"Class names to exclude from output entirely. "
                         f"Default: {DEFAULT_HIDDEN} (see NOTE above). "
                         f"Pass --hide-classes with no arguments to show "
                         f"everything.")
    p.add_argument("--out", default="runs/predict_test",
                    help="Where annotated output gets saved.")
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
    )

    print(f"\n{'=' * 60}")
    print("Per-frame/image detection counts (spot-check these against "
          "what you'd actually expect to see):")
    for r in results:
        n = len(r.boxes) if r.boxes is not None else 0
        classes_seen = {}
        if r.boxes is not None:
            for c in r.boxes.cls.tolist():
                name = all_names[int(c)]
                classes_seen[name] = classes_seen.get(name, 0) + 1
        print(f"  {Path(r.path).name}: {n} detections -- {classes_seen}")

    print(f"\nAnnotated output saved under: {args.out}")
    print("Look specifically for: false positives on background clutter, "
          "missed obvious cars/people, and whether box tightness looks "
          "usable for a pilot overlay (not just 'technically overlapping "
          "the object').")


if __name__ == "__main__":
    main()