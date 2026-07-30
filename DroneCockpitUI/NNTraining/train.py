"""
train.py — Fine-tune YOLO26n on VisDrone for Project R.A.D detection
=======================================================================
Defaults are tuned for a 6GB laptop RTX 3060. If `nvidia-smi` shows more
VRAM available, see the comments below for what to bump.

Usage:
    python train.py                    # full run, defaults below
    python train.py --imgsz 960        # try higher res if VRAM allows
    python train.py --epochs 50        # shorter smoke-test run first

A short smoke-test run (--epochs 5 or so) is worth doing before committing
to a full 100-epoch run, just to confirm the pipeline runs end-to-end
(dataset downloads, GPU is actually used, export succeeds) before spending
hours on it.
"""

import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="yolo26n.pt",
                    help="Base pretrained weights. yolo26n = fastest/lowest "
                         "VRAM, yolo26s = more accurate but needs more VRAM "
                         "and time. Stick with n first; only move to s once "
                         "n's accuracy is the proven bottleneck, not "
                         "training convenience.")
    p.add_argument("--data", default="VisDrone.yaml",
                    help="Auto-downloads VisDrone on first run.")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640,
                    help="640 first. VisDrone objects are small even at "
                         "native resolution, and your drone altitude makes "
                         "this worse -- 960 or 1280 will likely help "
                         "accuracy once the pipeline is proven, at the cost "
                         "of VRAM and training time. Don't jump straight "
                         "to 1280 on a 6GB card without testing 640 first.")
    p.add_argument("--batch", default=-1,
                    help="-1 = Ultralytics auto-batch (picks a safe batch "
                         "size for your VRAM automatically). Set an "
                         "explicit small int (e.g. 8) if auto-batch still "
                         "OOMs.")
    p.add_argument("--device", default=0,
                    help="GPU index. 0 = first (only, on a laptop) GPU. "
                         "Use 'cpu' only to sanity-check the script runs, "
                         "never for a real training run.")
    p.add_argument("--workers", type=int, default=4,
                    help="Dataloader worker processes. Lower this (e.g. 2) "
                         "if you hit RAM/CPU bottlenecks, since this runs "
                         "alongside everything else on the same laptop.")
    p.add_argument("--cache", default="disk",
                    help="'disk' caches decoded images to local disk "
                         "between epochs (fast re-reads, avoids re-decoding "
                         "JPEGs every epoch, and avoids blowing out RAM the "
                         "way cache='ram' would on a laptop). Set to False "
                         "if disk space is tight.")
    p.add_argument("--export-only", action="store_true",
                    help="Skip training, just export an existing "
                         "runs/detect/train/weights/best.pt (use --weights "
                         "to point at a specific checkpoint).")
    p.add_argument("--weights", default=None,
                    help="Checkpoint to export when --export-only is set. "
                         "Defaults to the most recent "
                         "runs/detect/train*/weights/best.pt found.")
    return p.parse_args()


def find_latest_best_pt() -> Path:
    candidates = sorted(Path(".").glob("runs/detect/train*/weights/best.pt"),
                         key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(
            "No runs/detect/train*/weights/best.pt found. Run training "
            "first, or pass --weights explicitly.")
    return candidates[-1]


def export_for_cpp(weights_path: Path):
    """
    Exports with end2end=False (the one-to-many head) so the ONNX output
    shape stays (1, nc+4, N) -- what DetectionLink::runInference() already
    parses via manual class-argmax + cv::dnn::NMSBoxes. This deliberately
    avoids the newer NMS-free one-to-one export path for now; see
    SETUP_AND_TRAINING.md for why.
    """
    print(f"\nExporting {weights_path} to ONNX (one-to-many head, "
          f"NMS still required on the C++ side -- matches current "
          f"runInference())...")
    model = YOLO(str(weights_path))
    onnx_path = model.export(format="onnx", opset=17, simplify=True,
                              end2end=False, nms=False)
    onnx_path = Path(onnx_path)

    # Write the sibling .names file DetectionLink::setModelPath() auto-loads.
    names_path = onnx_path.with_suffix(".names")
    class_names = model.names  # dict[int, str], index order matters
    with open(names_path, "w") as f:
        for idx in sorted(class_names.keys()):
            f.write(class_names[idx] + "\n")

    print(f"\nExported model:  {onnx_path}")
    print(f"Class names file: {names_path}")
    print("\nNote on class merging: VisDrone's 'pedestrian' and 'people' "
          "classes are kept separate here. To treat both as one 'person' "
          "class in the cockpit app, either edit the .names file above so "
          "both lines read 'person', or merge them in "
          "DetectionLink::classNameFor() / the Python display layer -- "
          "whichever is easier to keep in sync with future re-exports.")
    return onnx_path


def main():
    args = parse_args()

    if args.export_only:
        weights = Path(args.weights) if args.weights else find_latest_best_pt()
        export_for_cpp(weights)
        return

    model = YOLO(args.model)

    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        cache=args.cache,
        amp=True,          # mixed precision -- large VRAM/speed win, on by
                            # default but stated explicitly here since it
                            # matters this much on a 6GB card
        patience=20,        # early stop if val mAP plateaus for 20 epochs,
                            # saves time on a laptop GPU
        project="runs/detect",
        name="train",
    )

    best_pt = Path(model.trainer.best)
    print(f"\nTraining complete. Best checkpoint: {best_pt}")

    export_for_cpp(best_pt)


if __name__ == "__main__":
    main()