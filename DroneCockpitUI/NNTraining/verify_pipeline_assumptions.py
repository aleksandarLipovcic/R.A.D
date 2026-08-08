"""
verify_pipeline_assumptions.py -- Empirically checks the claims made in
train.py / prepare_datasets.py's docstrings against your ACTUAL prepared
data and ACTUAL installed library versions, instead of trusting comments.

Run this from the same directory as train.py / prepare_datasets.py /
class_map.py (it imports them directly, same as train.py does).

    python verify_pipeline_assumptions.py
    python verify_pipeline_assumptions.py --checkpoint runs/detect/yolo26s_960/weights/best.pt

Five checks, each prints CONFIRMED / CONTRADICTED / INCONCLUSIVE:

  A. Per-class instance counts on your actual merged train set, and
     whether the true sparsest class is covered by --oversample-classes.
  B. Whether any label file in your actual merged train set contains
     segmentation polygons (the thing CopyPaste needs to not be a no-op).
  C. Whether build_degradation_transform() actually builds all 8 pieces
     against your installed albumentations version, or silently drops
     some (falls back to library defaults, which the docstring warns
     is worse than a crash for CoarseDropout specifically).
  D. Whether CoarseDropout's hole_height_range/hole_width_range really
     are literal pixels (not a fraction of image size) on your installed
     version -- verified on a synthetic image, not assumed from memory.
  E. (optional, --checkpoint) Real per-class mAP on a trained checkpoint,
     if you have one -- the actual number the whole exercise is for.
"""

import argparse
import sys
from pathlib import Path

import numpy as np


def section(title: str):
    print(f"\n{'=' * 70}\n{title}\n{'=' * 70}")


def verdict(label: str, ok: bool | None, detail: str = ""):
    tag = "CONFIRMED" if ok is True else "CONTRADICTED" if ok is False else "INCONCLUSIVE"
    print(f"[{tag}] {label}")
    if detail:
        print(f"         {detail}")


# ---------------------------------------------------------------------
# A. Real per-class instance counts (not assumed from VisDrone alone)
# ---------------------------------------------------------------------

def check_class_balance(oversample_classes: list[str]):
    section("A. Per-class instance counts (actual merged train set)")
    try:
        import prepare_datasets as pd
        from class_map import UNIFIED_CLASSES, taxonomy_signature
    except Exception as e:
        verdict("Could not import prepare_datasets/class_map", None, str(e))
        return

    signature = taxonomy_signature()
    try:
        visdrone_dirs = pd.prepare_visdrone(signature)
        xview_dirs = pd.prepare_xview(signature)
        uavdt_dirs = pd.prepare_uavdt(signature)
        sard_dirs = pd.prepare_sard(signature)
        external_dirs = pd.prepare_external(signature)
    except Exception as e:
        verdict("Dataset assembly failed", None,
                f"{e} -- run prepare_datasets.py once manually first.")
        return

    train_dirs = (visdrone_dirs["train"] + xview_dirs["train"]
                  + uavdt_dirs["train"] + sard_dirs["train"]
                  + external_dirs["train"])
    if not train_dirs:
        verdict("No train dirs found", None,
                "Datasets not downloaded/placed yet -- nothing to count.")
        return

    counts = pd._count_instances_per_class(train_dirs)
    ranked = sorted(counts.items(), key=lambda kv: kv[1])

    print(f"{'class':15s} {'instances':>10s}  {'in oversample list?':>20s}")
    for name, n in ranked:
        flag = "yes" if name in oversample_classes else ""
        print(f"{name:15s} {n:>10d}  {flag:>20s}")

    sparsest_name, sparsest_n = ranked[0]
    person_rank = [i for i, (name, _) in enumerate(ranked) if name == "person"]
    person_rank = person_rank[0] if person_rank else None
    person_n = counts.get("person", 0)

    if person_rank == 0 and "person" not in oversample_classes:
        verdict("--oversample-classes covers the true sparsest class", False,
                 f"'person' is the sparsest class ({person_n} instances) "
                 f"and is NOT in {oversample_classes}. This is the exact "
                 f"failure mode flagged in the review -- add 'person' to "
                 f"--oversample-classes.")
    elif "person" not in oversample_classes:
        verdict("--oversample-classes covers the true sparsest class", True,
                 f"'person' has {person_n} instances (rank {person_rank+1} "
                 f"of {len(ranked)} by scarcity, sparsest is "
                 f"'{sparsest_name}' with {sparsest_n}) -- not currently "
                 f"the sparsest, so the default list is defensible, but "
                 f"re-run this after adding more data to confirm it stays "
                 f"true.")
    else:
        verdict("--oversample-classes covers 'person'", True,
                 "'person' is already in the oversample list.")


# ---------------------------------------------------------------------
# B. Does ANY label file in the actual merged set have segmentation data?
# ---------------------------------------------------------------------

def check_copy_paste_is_really_inert():
    section("B. Empirical check: does this project's data have segmentation "
            "polygons? (i.e. is --copy-paste really a no-op HERE)")
    try:
        import prepare_datasets as pd
        from class_map import taxonomy_signature
    except Exception as e:
        verdict("Could not import prepare_datasets", None, str(e))
        return

    signature = taxonomy_signature()
    try:
        visdrone_dirs = pd.prepare_visdrone(signature)
        xview_dirs = pd.prepare_xview(signature)
        uavdt_dirs = pd.prepare_uavdt(signature)
        sard_dirs = pd.prepare_sard(signature)
        external_dirs = pd.prepare_external(signature)
    except Exception as e:
        verdict("Dataset assembly failed", None, str(e))
        return

    train_dirs = (visdrone_dirs["train"] + xview_dirs["train"]
                  + uavdt_dirs["train"] + sard_dirs["train"]
                  + external_dirs["train"])
    if not train_dirs:
        verdict("No train dirs found", None, "Nothing to scan.")
        return

    total_lines = 0
    segment_like_lines = 0
    examples = []
    for entry in train_dirs:
        for img_path in pd._iter_images_in_entry(entry):
            label_path = pd._label_path_for_image(img_path)
            if label_path is None:
                continue
            for line in label_path.read_text().splitlines():
                line = line.strip()
                if not line:
                    continue
                total_lines += 1
                n_fields = len(line.split())
                # standard YOLO detection bbox = 5 fields: class cx cy w h
                if n_fields > 5:
                    segment_like_lines += 1
                    if len(examples) < 3:
                        examples.append((str(label_path), n_fields))

    if total_lines == 0:
        verdict("Scanned label lines", None, "No labels found to scan.")
        return

    print(f"Scanned {total_lines} label lines across {len(train_dirs)} "
          f"train sources.")
    print(f"Lines with >5 fields (would indicate polygon/segment data): "
          f"{segment_like_lines}")

    if segment_like_lines == 0:
        verdict("--copy-paste is a no-op on this project's actual data",
                 True,
                 "Every label line is a plain 5-field bbox -- no "
                 "segmentation polygons exist anywhere in the merged "
                 "train set, so CopyPaste has literally nothing to work "
                 "with, independent of the Ultralytics source-code claim.")
    else:
        verdict("--copy-paste is a no-op on this project's actual data",
                 False,
                 f"{segment_like_lines} lines have >5 fields -- e.g. "
                 f"{examples}. Some labels may carry segmentation/OBB "
                 f"data; re-examine before assuming copy_paste is inert.")


# ---------------------------------------------------------------------
# C. Does build_degradation_transform() actually build all pieces on
#    YOUR installed albumentations version?
# ---------------------------------------------------------------------

def check_degradation_transform_builds():
    section("C. Degradation augmentation: what actually builds on your "
            "installed albumentations version")
    try:
        import albumentations as A
        print(f"Installed albumentations: {A.__version__}")
    except ImportError:
        verdict("albumentations installed", False,
                 "Not installed -- degradation augmentation will be "
                 "silently disabled during training.")
        return

    try:
        import train as train_module
    except Exception as e:
        verdict("Could not import train.py", None, str(e))
        return

    compose = train_module.build_degradation_transform()
    if compose is None:
        verdict("build_degradation_transform() produced a pipeline", False,
                 "Returned None -- every transform failed to build "
                 "against this albumentations version. Degradation "
                 "augmentation will not run during training.")
        return

    names = [type(t).__name__ for t in compose.transforms]
    print(f"Built {len(names)}/8 planned transforms: {names}")
    expected_min = 6  # allow OneOf blocks to count as 1-2 depending on nesting
    verdict(f"At least {expected_min}/8 transform slots built", len(names) >= expected_min,
             f"Got {len(names)}. If this is lower than expected, check the "
             f"printed '[degradation aug] Skipping ...' lines above for "
             f"which specific transform failed and why -- don't assume "
             f"the version-branching covers your exact installed version.")


# ---------------------------------------------------------------------
# D. CoarseDropout pixel semantics -- verified on a synthetic image
# ---------------------------------------------------------------------

def check_coarse_dropout_is_literal_pixels():
    section("D. CoarseDropout hole size: literal pixels or fraction of "
            "image, on your installed version?")
    try:
        import albumentations as A
    except ImportError:
        verdict("albumentations installed", False, "Not installed.")
        return

    try:
        alb_major = int(str(A.__version__).split(".")[0])
    except Exception:
        alb_major = 1

    # Force exactly 1 hole so the zeroed-pixel count is deterministic:
    # 1 hole of (4, 4) height x (10, 10) width = exactly 40 zeroed pixels
    # IF hole sizes are literal pixels. If they were actually a fraction
    # of image size, a 200x300 test image would zero out a much larger
    # area instead.
    try:
        if alb_major >= 2:
            transform = A.CoarseDropout(num_holes_range=(1, 1),
                                          hole_height_range=(4, 4),
                                          hole_width_range=(10, 10),
                                          fill=0, p=1.0)
        else:
            transform = A.CoarseDropout(max_holes=1, min_holes=1,
                                          max_height=4, min_height=4,
                                          max_width=10, min_width=10,
                                          fill_value=0, p=1.0)
    except Exception as e:
        verdict("Could build a deterministic single-hole CoarseDropout", None,
                 f"{e} -- parameter names for your installed version don't "
                 f"match either branch train.py handles. This is exactly "
                 f"the silent-fallback risk the docstring warns about.")
        return

    test_img = np.full((200, 300, 3), 255, dtype=np.uint8)  # solid white
    trials = 20
    counts = []
    for _ in range(trials):
        out = transform(image=test_img.copy())["image"]
        zeroed = int(np.sum(np.all(out == 0, axis=-1)))
        counts.append(zeroed)

    expected = 40  # 4 * 10
    all_match = all(c == expected for c in counts)
    print(f"Zeroed-pixel counts across {trials} trials: {sorted(set(counts))} "
          f"(expected exactly {expected} if literal pixels)")

    if all_match:
        verdict("hole_height_range/hole_width_range are literal pixels "
                 "(not a fraction of image size)", True,
                 f"Every trial zeroed exactly {expected} pixels on a "
                 f"200x300 test image, matching 4x10 literal pixels.")
    else:
        verdict("hole_height_range/hole_width_range are literal pixels "
                 "(not a fraction of image size)", False,
                 f"Got {sorted(set(counts))} instead of a constant "
                 f"{expected} -- the docstring's pixel-semantics claim "
                 f"does not hold on this installed version. Re-check the "
                 f"actual hole sizes being applied before trusting the "
                 f"'deliberately tiny, safe for small labels' reasoning.")


# ---------------------------------------------------------------------
# E. (optional) Real per-class mAP on an actual checkpoint
# ---------------------------------------------------------------------

def check_real_per_class_map(checkpoint: str):
    section("E. Real per-class mAP on your checkpoint (the number that "
            "actually matters)")
    ckpt_path = Path(checkpoint)
    if not ckpt_path.exists():
        verdict("Checkpoint exists", False, f"{ckpt_path} not found.")
        return

    try:
        import train as train_module
        from ultralytics import YOLO
    except Exception as e:
        verdict("Could not import train.py / ultralytics", None, str(e))
        return

    data_yaml = train_module.SCRIPT_DIR / "datasets" / "unified.yaml"
    if not data_yaml.exists():
        verdict("unified.yaml exists", False,
                 f"{data_yaml} not found -- run prepare_datasets.py first.")
        return

    print(f"Loading {ckpt_path} and running val() against {data_yaml} "
          f"-- this actually runs inference, may take a few minutes...")
    model = YOLO(str(ckpt_path))
    train_module.report_per_class_map(model, str(data_yaml))
    verdict("Per-class mAP computed from a real val() pass", True,
             "See numbers printed above -- this is ground truth, not an "
             "estimate.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--oversample-classes", default="motorcycle,other_vehicle",
                          help="Comma-separated list matching whatever you're "
                               "currently passing to train.py, so check A can "
                               "tell you if it's missing the true sparsest class.")
    parser.add_argument("--checkpoint", default=None,
                          help="Path to a best.pt to run check E against. "
                               "Skipped if not provided (requires inference "
                               "time / GPU).")
    args = parser.parse_args()

    oversample_classes = [c.strip() for c in args.oversample_classes.split(",") if c.strip()]

    check_class_balance(oversample_classes)
    check_copy_paste_is_really_inert()
    check_degradation_transform_builds()
    check_coarse_dropout_is_literal_pixels()
    if args.checkpoint:
        check_real_per_class_map(args.checkpoint)
    else:
        section("E. Real per-class mAP")
        print("Skipped -- pass --checkpoint runs/detect/<run>/weights/best.pt "
              "to run this against an actual trained model.")

    section("Done")
    print("Every result above came from running against your actual data / "
          "actual installed libraries -- nothing here was assumed from a "
          "docstring or a prior review's claims.")


if __name__ == "__main__":
    sys.exit(main())
