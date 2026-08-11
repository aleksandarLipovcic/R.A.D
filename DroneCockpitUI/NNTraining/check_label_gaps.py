"""
check_label_gaps.py -- Quantifies the missing-annotation risk in Project R.A.D's
merged VisDrone + UAVDT + SARD training set.

WHY THIS EXISTS:
UAVDT only labels car/bus/truck (-> car/large_vehicle in the unified taxonomy).
person, motorcycle, and other_vehicle are NEVER labeled in UAVDT, even if
they're visible in a frame. SARD only labels person -- car/large_vehicle/
motorcycle/other_vehicle are never labeled there either. Because YOLO's
classification loss treats every unlabeled region as confirmed background, a
real (but unlabeled) instance of a "missing" class in these sources actively
punishes the model for correctly detecting something it learned from VisDrone.

This script does NOT fix anything or touch any label file. It runs a general
COCO-pretrained YOLO26 checkpoint over each source's images, looking ONLY for
the classes that source structurally never labels, and reports how often it
finds them -- so the next step (pseudo-label the gaps, mask the loss, or do
nothing because the risk is negligible) is a decision backed by actual counts
and actual images, not a guess.

Which classes are "missing" per source is derived directly from
VISDRONE_REMAP / EXTERNAL_REMAPS in class_map.py -- not hardcoded here -- so
this stays correct automatically if the taxonomy or remap tables change, and
extends to any future dataset added the same way (e.g. AU-AIR).

Output:
  - Console summary per source/class/threshold.
  - datasets/label_gap_report.json -- full stats, machine-readable.
  - datasets/label_gap_review/<source>/ -- annotated sample images (mix of
    highest-confidence + random hits) for manual eyeballing before trusting
    any of this.

CAVEAT (read before trusting a "clean" result): the scanning model is
COCO-pretrained on mostly ground-level photos. Its recall on small, aerial,
top-down objects is worse than a model trained on this project's own drone
footage. Every count here is a LOWER BOUND on the real gap, not a ceiling.
Zero hits means "this pass didn't catch anything," not "there is no gap."

FIX (2026-08-11): model.predict() was previously called with source=<python
list of path strings> in both scan_source() and save_review_images(). A raw
python list source is NOT streamed the way a directory or a .txt list file
is -- Ultralytics' check_source() routes list/tuple sources through
autocast_list(), which eagerly Image.open()s every element up front and
returns in-memory PIL images with no real path preserved. Once that happens,
Results.path for each result falls back to Ultralytics' synthetic
placeholder names ("image0.jpg", "image1.jpg", "image2.jpg", ... cycling per
batch) instead of the actual file path.

That silently corrupted scan_source()'s own hit records (hits[cls].append((
conf, area_frac, r.path)) stored a fake cycling name, not the real image),
which means the "images_with_detection" dedup counts in the printed report
and label_gap_report.json were undercounted/wrong on any prior run of this
script -- not just cosmetic. It also crashed save_review_images() outright,
since it re-predicts on those already-fake names and Ultralytics then tries
to actually open a file literally called "image4.jpg" in the cwd.

Fix: both call sites now write the source paths to an on-disk .txt list
(one absolute path per line, same convention _auto_split_by_sequence() in
prepare_datasets.py already uses for UAVDT) and pass that file's path
(a single string) as source=. That keeps Ultralytics on the real streaming
loader path, which does preserve real file paths in Results.path. Any
label_gap_report.json written before this fix should be treated as
unreliable and regenerated.

Usage:
    python check_label_gaps.py                     # full scan, default thresholds
    python check_label_gaps.py --limit 300          # quick pass, 300 imgs/source
    python check_label_gaps.py --model yolo26l.pt   # stronger scanning model
    python check_label_gaps.py --conf-thresholds 0.25 0.4 0.6 0.8
"""

import argparse
import json
import random
import statistics
import time
from collections import defaultdict
from pathlib import Path

import cv2
from ultralytics import YOLO

import prepare_datasets
from class_map import UNIFIED_CLASSES, VISDRONE_REMAP, EXTERNAL_REMAPS, taxonomy_signature

# Which COCO class NAMES correspond to each unified class, for scanning with a
# COCO-pretrained model. Resolved to indices at runtime against the loaded
# model's own model.names rather than hardcoded indices -- COCO ordering is
# consistent across Ultralytics checkpoints, but resolving by name is safer
# than assuming that holds forever.
COCO_NAMES_FOR_UNIFIED = {
    "person": ["person"],
    "car": ["car"],
    "large_vehicle": ["bus", "truck"],
    "motorcycle": ["motorcycle"],
    "other_vehicle": ["bicycle"],  # closest COCO analog to tricycle/etc.
}

# source_name -> (prepare_fn, remap_table). prepare_fn(signature) returns
# {"train": [...], "val": [...], "test": [...]} the exact same way
# prepare_datasets.py itself resolves them internally (handles UAVDT's
# auto-split txt lists vs SARD's real directories transparently) -- reusing
# it here means this script can't drift out of sync with what actually feeds
# training. VisDrone isn't in this registry: its remap table covers all 5
# unified classes, so it has no missing-class risk to check.
SOURCE_REGISTRY = {
    "UAVDT": (prepare_datasets.prepare_uavdt, EXTERNAL_REMAPS["uavdt"]),
    "SARD": (prepare_datasets.prepare_sard, EXTERNAL_REMAPS["sard"]),
}


def missing_classes_for(remap_table: dict) -> list[str]:
    """Any UNIFIED_CLASSES name this source's remap table never produces --
    i.e. a class that can appear in that source's real footage but is
    structurally guaranteed to never have a ground-truth box there."""
    valid = {v for v in remap_table.values() if v is not None}
    return [c for c in UNIFIED_CLASSES if c not in valid]


def gather_images(dirs_dict: dict, splits=("train", "val")) -> list[Path]:
    paths: list[Path] = []
    for split in splits:
        for entry in dirs_dict.get(split, []):
            paths.extend(prepare_datasets._iter_images_in_entry(entry))
    return paths


def resolve_coco_indices(model_names: dict, class_names: list[str]) -> list[int]:
    name_to_idx = {v: k for k, v in model_names.items()}
    idxs = []
    for name in class_names:
        if name in name_to_idx:
            idxs.append(name_to_idx[name])
        else:
            print(f"  [warn] COCO class '{name}' not found on this checkpoint "
                  f"-- skipping.")
    return idxs


def _write_source_list(paths, out_path: Path) -> Path:
    """
    Writes an Ultralytics-compatible source list file: one absolute image
    path per line. Callers must pass str(out_path) (a single string) as
    model.predict()'s source= argument -- NOT a python list of paths.

    Why: a python list/tuple source is routed by Ultralytics'
    check_source() -> autocast_list(), which eagerly Image.open()s every
    element up front and returns in-memory PIL images with no real path
    kept. Results.path then falls back to synthetic placeholder names
    ("image0.jpg", "image1.jpg", ... cycling per batch) instead of the
    real file path -- silently breaking any code (like this script) that
    keys results by their source path. Passing a .txt list file instead
    keeps Ultralytics on its real streaming loader, which does preserve
    the actual path in Results.path. Same convention prepare_datasets.py's
    _auto_split_by_sequence() already uses for UAVDT's auto-split lists.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(str(p) for p in paths) + "\n")
    return out_path


def scan_source(model: YOLO, source_name: str, image_paths: list[Path],
                 missing_classes: list[str], conf_floor: float, device,
                 batch: int) -> dict[str, list[tuple]]:
    """One inference pass over image_paths, looking only for the COCO classes
    that correspond to this source's missing UNIFIED_CLASSES, at conf_floor
    (the lowest threshold we care about -- higher thresholds are computed
    afterward from these same detections, no need to rerun inference)."""
    print(f"\n{'=' * 70}\n{source_name}: scanning {len(image_paths)} images "
          f"for {missing_classes}\n{'=' * 70}")

    coco_names = []
    for c in missing_classes:
        coco_names.extend(COCO_NAMES_FOR_UNIFIED.get(c, []))
    class_idxs = resolve_coco_indices(model.names, coco_names)
    if not class_idxs:
        print(f"  Nothing to scan for -- skipping {source_name}.")
        return {}

    hits: dict[str, list[tuple]] = defaultdict(list)  # unified_class -> [(conf, area_frac, path)]
    scanned = 0
    t0 = time.time()

    # See _write_source_list()'s docstring: a .txt list file (not a raw
    # python list) is required to keep Results.path pointing at the real
    # file instead of a synthetic "imageN.jpg" placeholder.
    list_path = prepare_datasets.DATASETS_DIR / f"_scan_{source_name.lower()}.txt"
    _write_source_list(image_paths, list_path)

    for r in model.predict(source=str(list_path), classes=class_idxs, conf=conf_floor,
                            device=device, batch=batch, verbose=False, stream=True):
        scanned += 1
        if scanned % 200 == 0:
            rate = scanned / (time.time() - t0)
            print(f"  ...{scanned}/{len(image_paths)} ({rate:.1f} img/s)")

        if r.boxes is None or len(r.boxes) == 0:
            continue

        for cls_id, conf, xywhn in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(),
                                         r.boxes.xywhn.tolist()):
            coco_name = model.names[int(cls_id)]
            for uni_name, coco_list in COCO_NAMES_FOR_UNIFIED.items():
                if uni_name in missing_classes and coco_name in coco_list:
                    area_frac = xywhn[2] * xywhn[3]
                    hits[uni_name].append((conf, area_frac, r.path))

    print(f"  Done: {scanned} images in {time.time() - t0:.0f}s.")
    return dict(hits)


def summarize_hits(hits: dict, images_scanned: int, thresholds: list[float],
                    missing_classes: list[str]) -> dict:
    summary = {"images_scanned": images_scanned, "thresholds": thresholds,
               "per_class": {}}
    for cls_name in missing_classes:
        entries = hits.get(cls_name, [])
        confs = [c for c, _, _ in entries]
        areas = [a for _, a, _ in entries]

        by_threshold = {}
        for t in thresholds:
            filtered = [(c, p) for c, a, p in entries if c >= t]
            by_threshold[str(t)] = {
                "total_detections": len(filtered),
                "images_with_detection": len({p for _, p in filtered}),
            }

        summary["per_class"][cls_name] = {
            "total_detections_at_floor": len(entries),
            "images_with_detection_at_floor": len({p for _, _, p in entries}),
            "confidence_mean": statistics.mean(confs) if confs else None,
            "confidence_median": statistics.median(confs) if confs else None,
            "box_area_frac_mean": statistics.mean(areas) if areas else None,
            "box_area_frac_median": statistics.median(areas) if areas else None,
            "by_threshold": by_threshold,
        }
    return summary


def print_source_report(source_name: str, missing_classes: list[str], summary: dict) -> None:
    print(f"\n{'-' * 70}\n{source_name} -- missing classes: {missing_classes}\n{'-' * 70}")
    for cls_name in missing_classes:
        s = summary["per_class"][cls_name]
        print(f"  {cls_name:15s}  {s['total_detections_at_floor']:>5} raw hits "
              f"across {s['images_with_detection_at_floor']:>5}/"
              f"{summary['images_scanned']} images")
        for t in summary["thresholds"]:
            bt = s["by_threshold"][str(t)]
            print(f"      conf>={t:<4}  {bt['total_detections']:>5} detections, "
                  f"{bt['images_with_detection']:>5} images")
        if s["confidence_mean"] is not None:
            print(f"      confidence mean/median: {s['confidence_mean']:.2f} / "
                  f"{s['confidence_median']:.2f}   "
                  f"box-area-frac mean/median: {s['box_area_frac_mean']:.4f} / "
                  f"{s['box_area_frac_median']:.4f}")


def save_review_images(model: YOLO, hits: dict, missing_classes: list[str],
                        conf_floor: float, device, batch: int, review_dir: Path,
                        source_name: str, max_review: int) -> None:
    """Saves annotated copies of a mix of the highest-confidence hits and a
    random sample of the rest, so a human can actually look at what this
    scan is calling a 'person in UAVDT' or 'car in SARD' before anyone
    trusts it enough to pseudo-label from it."""
    per_image_max_conf: dict[str, float] = {}
    for cls_name in missing_classes:
        for conf, _, path in hits.get(cls_name, []):
            per_image_max_conf[path] = max(per_image_max_conf.get(path, 0.0), conf)

    if not per_image_max_conf:
        print(f"  No hits to review for {source_name}.")
        return

    ranked = sorted(per_image_max_conf.items(), key=lambda kv: kv[1], reverse=True)
    top_n = max_review // 2
    chosen = [p for p, _ in ranked[:top_n]]
    remainder = [p for p, _ in ranked[top_n:]]
    random.shuffle(remainder)
    chosen += remainder[:max_review - len(chosen)]

    out_dir = review_dir / source_name.lower()
    out_dir.mkdir(parents=True, exist_ok=True)

    coco_names = [n for c in missing_classes for n in COCO_NAMES_FOR_UNIFIED.get(c, [])]
    class_idxs = resolve_coco_indices(model.names, coco_names)

    # Same fix as scan_source(): write `chosen` (real paths recovered from
    # the FIRST pass's hit records) out to a .txt list file instead of
    # passing the python list directly to source=. This is also why
    # scan_source() itself had to be fixed first -- if hits still carried
    # synthetic "imageN.jpg" paths from an autocast_list() pass, `chosen`
    # here would just be a list of fake names and this fix alone wouldn't
    # help.
    list_path = out_dir / "_review_list.txt"
    _write_source_list(chosen, list_path)

    print(f"  Saving {len(chosen)} annotated review images to {out_dir}/ ...")
    for i, r in enumerate(model.predict(source=str(list_path), classes=class_idxs,
                                          conf=conf_floor, device=device,
                                          batch=min(batch, 8), verbose=False,
                                          stream=True)):
        conf_tag = per_image_max_conf.get(r.path, 0.0)
        out_path = out_dir / f"{i:03d}_conf{conf_tag:.2f}_{Path(r.path).stem}.jpg"
        cv2.imwrite(str(out_path), r.plot())


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default="yolo26m.pt",
                    help="COCO-pretrained checkpoint to scan with. Bigger = "
                         "better recall/precision for this one-off diagnostic "
                         "pass (inference-only, none of train.py's VRAM "
                         "constraints apply) -- try yolo26l.pt/yolo26x.pt for "
                         "a stronger pass if you don't mind the auto-download.")
    p.add_argument("--limit", type=int, default=None,
                    help="Randomly sample at most this many images per source "
                         "instead of scanning everything. Good for a fast "
                         "first pass before committing to a full scan.")
    p.add_argument("--conf-thresholds", type=float, nargs="+",
                    default=[0.25, 0.5, 0.7],
                    help="Confidence thresholds to report counts at. The "
                         "LOWEST value is also the actual conf floor used "
                         "during inference -- everything above it is computed "
                         "from that same pass, not rerun.")
    p.add_argument("--device", default=0)
    p.add_argument("--batch", type=int, default=16,
                    help="Inference batch size. Much lighter than training "
                         "(no gradients/optimizer state), so this can go well "
                         "above train.py's --batch.")
    p.add_argument("--splits", nargs="+", default=["train", "val"],
                    choices=["train", "val", "test"])
    p.add_argument("--max-review-images", type=int, default=40,
                    help="Per source: how many annotated sample images to "
                         "save for manual eyeballing (half highest-confidence, "
                         "half random).")
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def main():
    args = parse_args()
    random.seed(args.seed)
    conf_floor = min(args.conf_thresholds)

    print(f"Loading {args.model} (COCO-pretrained -- used only to scan for "
          f"classes each source doesn't label; this is NOT the unified-"
          f"taxonomy model)...")
    model = YOLO(args.model)

    signature = taxonomy_signature()
    datasets_dir = prepare_datasets.DATASETS_DIR
    review_dir = datasets_dir / "label_gap_review"
    report_path = datasets_dir / "label_gap_report.json"

    report = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "model": args.model,
        "conf_thresholds": args.conf_thresholds,
        "splits_scanned": args.splits,
        "sources": {},
        "notes": (
            "Counts are a LOWER BOUND on real missing instances -- the "
            "scanning model is COCO-pretrained on mostly ground-level "
            "photos, so recall on small/aerial objects is worse than a "
            "model trained on this project's own drone footage. Zero hits "
            "is not proof a gap is safe, just that this pass didn't catch "
            "anything -- check label_gap_review/ before trusting a clean "
            "result at face value."
        ),
    }

    for source_name, (prepare_fn, remap_table) in SOURCE_REGISTRY.items():
        missing = missing_classes_for(remap_table)
        if not missing:
            print(f"\n{source_name}: fully annotated across the unified "
                  f"taxonomy, nothing to check.")
            continue

        dirs = prepare_fn(signature)
        images = gather_images(dirs, splits=tuple(args.splits))
        if args.limit and len(images) > args.limit:
            images = random.sample(images, args.limit)

        if not images:
            print(f"\n{source_name}: no images found for splits {args.splits} "
                  f"-- was prepare_datasets.py run yet?")
            continue

        hits = scan_source(model, source_name, images, missing, conf_floor,
                             args.device, args.batch)
        summary = summarize_hits(hits, len(images), args.conf_thresholds, missing)
        save_review_images(model, hits, missing, conf_floor, args.device,
                             args.batch, review_dir, source_name,
                             args.max_review_images)

        report["sources"][source_name] = {"missing_classes": missing, **summary}
        print_source_report(source_name, missing, summary)

    datasets_dir.mkdir(parents=True, exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\n{'=' * 70}")
    print(f"Full report: {report_path}")
    print(f"Review images: {review_dir}/")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()