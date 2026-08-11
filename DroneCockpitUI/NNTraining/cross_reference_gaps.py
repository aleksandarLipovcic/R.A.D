"""
cross_reference_gaps.py -- Cross-references YOLO26l against Grounding DINO
on the same "missing-class" images check_label_gaps.py flagged, to get a
second, architecturally-independent opinion before trusting anything as a
pseudo-label ground truth for Project R.A.D's UAVDT/SARD label gaps.

WHY A SECOND MODEL, AND WHY THIS ONE:
check_label_gaps.py's review images showed two DIFFERENT failure modes:
  - UAVDT: YOLO26l has real false positives on "person" (over-triggering on
    small/aerial content it wasn't trained for).
  - SARD: YOLO26l is under-recalling "car" -- missing real cars that are
    actually there.
A second YOLO checkpoint (e.g. yolo26x.pt) wouldn't help much with either
problem: same anchor-free architecture, same COCO-only training data, so
its blind spots correlate heavily with yolo26l's. What actually reduces
CORRELATED error is a model that differs on both axes -- architecture AND
training data. Grounding DINO is a DETR-style transformer with language
grounding, trained on a broader mix of detection + grounding datasets, and
is open-vocabulary -- it's prompted directly with our own unified class
names ("person", "car", "truck", "bus", "motorcycle", "bicycle") instead of
going through the COCO-name indirection check_label_gaps.py needs.

WHAT THIS SCRIPT DOES:
  1. Reuses check_label_gaps.py's SOURCE_REGISTRY / missing_classes_for /
     gather_images so the exact same image set (and the same per-source
     missing-class list) is scanned by both models -- no drift between the
     two passes.
  2. Runs YOLO26l via the SAME .txt-source-list fix check_label_gaps.py
     uses (see that module's docstring for why a raw python list source
     silently breaks path identity in Ultralytics).
  3. Runs Grounding DINO by loading each image directly with PIL and
     tracking path->result ourselves -- deliberately NOT going through
     Ultralytics' loader at all on this side, so there's no equivalent
     path-identity risk to worry about for DINO's detections.
  4. Matches the two models' detections per image, per class, by IoU
     (greedy best-match), and buckets every detection into:
       - "agree"      -- both models found it (IoU >= --iou-threshold).
                          Highest-trust bucket; primary pseudo-label
                          candidates.
       - "yolo_only"   -- only YOLO26l found it. Likely includes some of
                          the false positives you already saw in UAVDT --
                          review before trusting.
       - "dino_only"   -- only Grounding DINO found it. This is the
                          practical answer to SARD's under-recall problem:
                          cars YOLO missed that a differently-trained model
                          independently caught.
  5. Saves annotated review images per source (YOLO boxes in red, DINO
     boxes in blue, agreed pairs additionally outlined in green) so you can
     eyeball agreement/disagreement visually, the same way
     check_label_gaps.py's review images let you eyeball raw YOLO hits.
  6. Writes datasets/cross_reference_report.json (aggregate stats) and
     datasets/cross_reference_candidates.json (the actual "agree" boxes,
     in absolute pixel xyxy, ready to be consumed by a future
     pseudo-labeling script -- NOT applied to any label file by this
     script; this script only measures and reviews, same "don't touch
     labels" contract check_label_gaps.py follows).

REQUIREMENTS:
    pip install transformers pillow --break-system-packages
(torch/ultralytics/opencv are already required by check_label_gaps.py /
train.py). First run downloads the Grounding DINO checkpoint from the
Hugging Face Hub (~170MB for -tiny, ~440MB for -base).

USAGE:
    # Start small -- Grounding DINO is much slower per-image than YOLO,
    # so sanity-check timing before committing to a full run.
    python cross_reference_gaps.py --limit 200

    # Full run once timing looks acceptable.
    python cross_reference_gaps.py

    # Stronger (slower) Grounding DINO checkpoint.
    python cross_reference_gaps.py --dino-model IDEA-Research/grounding-dino-base

    # Loosen/tighten the "same object" IoU bar for the agree bucket.
    python cross_reference_gaps.py --iou-threshold 0.4

FIX (2026-08-11): run_dino() was calling
processor.post_process_grounded_object_detection(..., box_threshold=...,
text_threshold=...) and then reading res["labels"] for the class name
strings. Current transformers ships the unified zero-shot-object-detection
post-processing signature (the same standardization that touched OWL-ViT/
OWLv2/OmDet-Turbo), which renamed GroundingDinoProcessor's confidence-score
kwarg from box_threshold to threshold:

    def post_process_grounded_object_detection(
        self, outputs, input_ids=None,
        threshold: float = 0.25,        # was box_threshold
        text_threshold: float = 0.25,
        target_sizes=None, text_labels=None,
    )

Passing box_threshold now raises TypeError: got an unexpected keyword
argument 'box_threshold'. Separately, res["labels"] on that same call now
raises a FutureWarning ("will return integer ids ... since v4.51.0. Use
text_labels instead") -- it still returns strings today, but only
text_labels is the forward-compatible key for the class-name strings
_label_to_unified() expects.

Fix: call post_process_grounded_object_detection(..., threshold=
box_threshold, text_threshold=text_threshold, ...) -- the local variable
and CLI flag are still named box_threshold for continuity with
check_label_gaps.py's conf_floor naming and to keep --dino-box-threshold
self-explanatory on the command line; only the kwarg passed to the
processor changed. And read res["text_labels"] instead of res["labels"]
for the detected class-name strings.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO

import prepare_datasets
from class_map import taxonomy_signature
from check_label_gaps import (
    SOURCE_REGISTRY,
    COCO_NAMES_FOR_UNIFIED,
    missing_classes_for,
    gather_images,
    resolve_coco_indices,
    _write_source_list,
)

try:
    from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
except ImportError:
    AutoProcessor = None
    AutoModelForZeroShotObjectDetection = None


# Reuse the exact same unified-class -> plain-English-name mapping
# check_label_gaps.py already built for COCO -- Grounding DINO is
# open-vocabulary and takes the same plain names as a text prompt, so
# there's no separate phrase table to maintain or drift out of sync.
DINO_PHRASES_FOR_UNIFIED = COCO_NAMES_FOR_UNIFIED

# Colors for annotated review images (BGR, since we draw with cv2).
COLOR_YOLO = (0, 0, 255)      # red
COLOR_DINO = (255, 128, 0)    # blue
COLOR_AGREE = (0, 220, 0)     # green


# ---------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------

def compute_iou(box_a, box_b) -> float:
    """box_a/box_b: [x1, y1, x2, y2] absolute pixel coords."""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def greedy_match(dets_a: list, dets_b: list, iou_thr: float):
    """
    dets_a/dets_b: list of dicts with keys 'cls', 'conf', 'box' (xyxy).
    Greedy best-IoU matching, class-constrained (a detection can only
    match another detection of the same unified class). Not globally
    optimal (Hungarian would be), but detection counts per image here are
    small (single digits), so greedy is a reasonable, simple choice --
    the same tradeoff Ultralytics' own NMS-adjacent code makes elsewhere.

    Returns (agreed_pairs, only_a, only_b) where agreed_pairs is a list of
    (det_a, det_b, iou) and only_a/only_b are the unmatched leftovers.
    """
    used_b = set()
    agreed_pairs = []
    matched_a_idx = set()

    for i, da in enumerate(dets_a):
        best_j, best_iou = None, 0.0
        for j, db in enumerate(dets_b):
            if j in used_b or db["cls"] != da["cls"]:
                continue
            iou = compute_iou(da["box"], db["box"])
            if iou > best_iou:
                best_iou, best_j = iou, j
        if best_j is not None and best_iou >= iou_thr:
            agreed_pairs.append((da, dets_b[best_j], best_iou))
            used_b.add(best_j)
            matched_a_idx.add(i)

    only_a = [d for i, d in enumerate(dets_a) if i not in matched_a_idx]
    only_b = [d for j, d in enumerate(dets_b) if j not in used_b]
    return agreed_pairs, only_a, only_b


# ---------------------------------------------------------------------
# YOLO pass -- mirrors check_label_gaps.scan_source(), but keeps pixel
# xyxy boxes (needed for IoU against DINO's pixel boxes) instead of the
# normalized xywhn check_label_gaps.py stores.
# ---------------------------------------------------------------------

def run_yolo(model: YOLO, source_name: str, image_paths: list[Path],
             missing_classes: list[str], conf_floor: float, device,
             batch: int) -> dict[str, list[dict]]:
    print(f"\n  [YOLO26] scanning {len(image_paths)} images for "
          f"{missing_classes}...")

    coco_names = []
    for c in missing_classes:
        coco_names.extend(COCO_NAMES_FOR_UNIFIED.get(c, []))
    class_idxs = resolve_coco_indices(model.names, coco_names)
    if not class_idxs:
        print("    Nothing to scan for -- skipping.")
        return {}

    list_path = prepare_datasets.DATASETS_DIR / f"_xref_{source_name.lower()}.txt"
    _write_source_list(image_paths, list_path)

    results: dict[str, list[dict]] = {}
    scanned, t0 = 0, time.time()
    for r in model.predict(source=str(list_path), classes=class_idxs,
                            conf=conf_floor, device=device, batch=batch,
                            verbose=False, stream=True):
        scanned += 1
        if scanned % 500 == 0:
            rate = scanned / (time.time() - t0)
            print(f"    ...{scanned}/{len(image_paths)} ({rate:.1f} img/s)")

        dets = []
        if r.boxes is not None and len(r.boxes) > 0:
            for cls_id, conf, xyxy in zip(r.boxes.cls.tolist(),
                                            r.boxes.conf.tolist(),
                                            r.boxes.xyxy.tolist()):
                coco_name = model.names[int(cls_id)]
                for uni_name, coco_list in COCO_NAMES_FOR_UNIFIED.items():
                    if uni_name in missing_classes and coco_name in coco_list:
                        dets.append({"cls": uni_name, "conf": float(conf),
                                     "box": [float(v) for v in xyxy]})
        results[r.path] = dets

    print(f"    Done: {scanned} images in {time.time() - t0:.0f}s.")
    return results


# ---------------------------------------------------------------------
# Grounding DINO pass -- loads images directly via PIL, so path identity
# is whatever we pass in, never at risk of an Ultralytics-loader-style
# silent substitution.
# ---------------------------------------------------------------------

def build_dino_prompt(missing_classes: list[str]) -> tuple[str, dict]:
    """
    Grounding DINO expects a single lowercase text prompt with candidate
    phrases separated by " . " (a trailing period matters for some
    checkpoints' tokenization -- included for safety). Returns the prompt
    plus a reverse lookup (phrase -> unified class name) used to map
    returned labels back to our taxonomy.
    """
    phrase_to_unified = {}
    phrases = []
    for uni_name in missing_classes:
        for phrase in DINO_PHRASES_FOR_UNIFIED.get(uni_name, []):
            phrases.append(phrase)
            phrase_to_unified[phrase] = uni_name
    prompt = " . ".join(phrases) + " ."
    return prompt, phrase_to_unified


def _label_to_unified(label: str, phrase_to_unified: dict) -> str | None:
    """
    Grounding DINO's returned 'text_labels' are free-text spans matched
    against the prompt, not guaranteed to be an exact phrase (e.g. it may
    return 'car' cleanly, but can also return a partial/merged span
    depending on tokenization). Try exact match first, then substring
    containment either direction, rather than assuming exact equality
    always holds.
    """
    label = label.strip().lower().strip(".")
    if label in phrase_to_unified:
        return phrase_to_unified[label]
    for phrase, uni_name in phrase_to_unified.items():
        if phrase in label or label in phrase:
            return uni_name
    return None


def run_dino(processor, model, device, source_name: str,
             image_paths: list[Path], missing_classes: list[str],
             box_threshold: float, text_threshold: float,
             batch: int) -> dict[str, list[dict]]:
    print(f"\n  [Grounding DINO] scanning {len(image_paths)} images for "
          f"{missing_classes}...")

    prompt, phrase_to_unified = build_dino_prompt(missing_classes)
    results: dict[str, list[dict]] = {}
    scanned, t0 = 0, time.time()

    for start in range(0, len(image_paths), batch):
        chunk = image_paths[start:start + batch]
        pil_images = []
        valid_paths = []
        for p in chunk:
            try:
                pil_images.append(Image.open(p).convert("RGB"))
                valid_paths.append(p)
            except Exception as e:
                print(f"    [warn] Could not open {p} ({e}) -- skipping.")

        if not pil_images:
            continue

        inputs = processor(images=pil_images, text=[prompt] * len(pil_images),
                            return_tensors="pt").to(device)
        with torch.no_grad():
            outputs = model(**inputs)

        target_sizes = torch.tensor([img.size[::-1] for img in pil_images])
        # NOTE: current transformers renamed this kwarg from
        # `box_threshold` to `threshold` as part of standardizing the
        # zero-shot-object-detection post-processing API across models
        # (GroundingDINO / OWL-ViT / OWLv2 / OmDet-Turbo). See this
        # module's FIX docstring at the top for details. The local
        # variable/CLI flag here keep the name `box_threshold` for
        # continuity -- only the kwarg passed below changed.
        post = processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids,
            threshold=box_threshold, text_threshold=text_threshold,
            target_sizes=target_sizes)

        for path, res in zip(valid_paths, post):
            dets = []
            boxes = res["boxes"].tolist()
            scores = res["scores"].tolist()
            # Use text_labels (string class names), not labels -- current
            # transformers already emits a FutureWarning on res["labels"]
            # ("will return integer ids ... since v4.51.0. Use text_labels
            # instead"), and text_labels is the key that's actually
            # guaranteed to keep returning strings going forward.
            labels = res["text_labels"]
            for box, score, label in zip(boxes, scores, labels):
                uni_name = _label_to_unified(label, phrase_to_unified)
                if uni_name is None:
                    continue
                dets.append({"cls": uni_name, "conf": float(score),
                             "box": [float(v) for v in box]})
            results[str(path)] = dets

        scanned += len(pil_images)
        if scanned % 200 < batch:
            rate = scanned / (time.time() - t0)
            print(f"    ...{scanned}/{len(image_paths)} ({rate:.1f} img/s)")

    print(f"    Done: {scanned} images in {time.time() - t0:.0f}s.")
    return results


# ---------------------------------------------------------------------
# Matching + review image output
# ---------------------------------------------------------------------

def draw_box(img, box, color, label):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(img, label, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, color, 1, cv2.LINE_AA)


def compare_and_review(source_name: str, image_paths: list[Path],
                        yolo_dets: dict[str, list[dict]],
                        dino_dets: dict[str, list[dict]],
                        iou_threshold: float, review_dir: Path,
                        max_review: int) -> dict:
    """
    Matches YOLO vs DINO detections per image, aggregates counts per
    class/bucket, and writes annotated review images for a sample of
    images that had ANY detection from either model (prioritizing images
    with disagreement, since those are the most informative to look at).
    """
    per_image_results = {}
    agree_count, yolo_only_count, dino_only_count = 0, 0, 0
    per_class_counts = {}
    candidates = []  # "agree" boxes -- future pseudo-label material

    # Build a path-keyed lookup that's robust to str/Path mismatches.
    yolo_by_str = {str(k): v for k, v in yolo_dets.items()}
    dino_by_str = {str(k): v for k, v in dino_dets.items()}

    for img_path in image_paths:
        key = str(img_path)
        da = yolo_by_str.get(key, [])
        db = dino_by_str.get(key, [])
        if not da and not db:
            continue

        agreed_pairs, only_yolo, only_dino = greedy_match(da, db, iou_threshold)

        agree_count += len(agreed_pairs)
        yolo_only_count += len(only_yolo)
        dino_only_count += len(only_dino)

        for pair in agreed_pairs:
            det_a, det_b, iou = pair
            per_class_counts.setdefault(det_a["cls"], {"agree": 0, "yolo_only": 0, "dino_only": 0})
            per_class_counts[det_a["cls"]]["agree"] += 1
            candidates.append({
                "image": key, "cls": det_a["cls"],
                "yolo_conf": det_a["conf"], "dino_conf": det_b["conf"],
                "iou": iou, "box": det_a["box"],
            })
        for d in only_yolo:
            per_class_counts.setdefault(d["cls"], {"agree": 0, "yolo_only": 0, "dino_only": 0})
            per_class_counts[d["cls"]]["yolo_only"] += 1
        for d in only_dino:
            per_class_counts.setdefault(d["cls"], {"agree": 0, "yolo_only": 0, "dino_only": 0})
            per_class_counts[d["cls"]]["dino_only"] += 1

        per_image_results[key] = {
            "agree": len(agreed_pairs), "yolo_only": len(only_yolo),
            "dino_only": len(only_dino),
        }

    # Prioritize review images with disagreement (only_yolo/only_dino) --
    # those are the informative ones; pure agreement is nice to spot-check
    # but less diagnostic.
    ranked = sorted(
        per_image_results.items(),
        key=lambda kv: (kv[1]["yolo_only"] + kv[1]["dino_only"], kv[1]["agree"]),
        reverse=True,
    )
    chosen = [Path(p) for p, _ in ranked[:max_review]]

    out_dir = review_dir / f"{source_name.lower()}_cross"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n  Saving {len(chosen)} cross-reference review images to "
          f"{out_dir}/ ...")

    for i, img_path in enumerate(chosen):
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        key = str(img_path)
        da = yolo_by_str.get(key, [])
        db = dino_by_str.get(key, [])
        agreed_pairs, only_yolo, only_dino = greedy_match(da, db, iou_threshold)

        for d in only_yolo:
            draw_box(img, d["box"], COLOR_YOLO, f"YOLO {d['cls']} {d['conf']:.2f}")
        for d in only_dino:
            draw_box(img, d["box"], COLOR_DINO, f"DINO {d['cls']} {d['conf']:.2f}")
        for det_a, det_b, iou in agreed_pairs:
            draw_box(img, det_a["box"], COLOR_AGREE,
                     f"AGREE {det_a['cls']} y{det_a['conf']:.2f}/d{det_b['conf']:.2f} iou{iou:.2f}")

        out_path = out_dir / f"{i:03d}_{Path(img_path).stem}.jpg"
        cv2.imwrite(str(out_path), img)

    summary = {
        "images_with_any_detection": len(per_image_results),
        "agree_total": agree_count,
        "yolo_only_total": yolo_only_count,
        "dino_only_total": dino_only_count,
        "per_class": per_class_counts,
    }
    return summary, candidates


# ---------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--yolo-model", default="yolo26l.pt",
                    help="COCO-pretrained YOLO checkpoint (same one "
                         "check_label_gaps.py used, for a like-for-like "
                         "comparison).")
    p.add_argument("--dino-model", default="IDEA-Research/grounding-dino-tiny",
                    help="Hugging Face Grounding DINO checkpoint. Use "
                         "IDEA-Research/grounding-dino-base for stronger "
                         "(slower) detection.")
    p.add_argument("--limit", type=int, default=None,
                    help="Randomly sample at most this many images per "
                         "source. Grounding DINO is much slower per-image "
                         "than YOLO -- start small (e.g. 200) to check "
                         "timing before running the full set.")
    p.add_argument("--yolo-conf", type=float, default=0.25,
                    help="YOLO confidence floor for this pass.")
    p.add_argument("--dino-box-threshold", type=float, default=0.25,
                    help="Grounding DINO box confidence floor. (Passed to "
                         "the processor as `threshold` -- current "
                         "transformers renamed this kwarg from "
                         "box_threshold; the CLI flag name is kept as-is "
                         "for continuity.)")
    p.add_argument("--dino-text-threshold", type=float, default=0.25,
                    help="Grounding DINO text-match confidence floor.")
    p.add_argument("--iou-threshold", type=float, default=0.5,
                    help="Minimum IoU for two models' boxes to count as "
                         "'agree' on the same object.")
    p.add_argument("--device", default=0,
                    help="YOLO device (int index or 'cpu').")
    p.add_argument("--dino-device", default=None,
                    help="Torch device string for Grounding DINO, e.g. "
                         "'cuda:0' or 'cpu'. Defaults to cuda:0 if "
                         "available, else cpu.")
    p.add_argument("--yolo-batch", type=int, default=16)
    p.add_argument("--dino-batch", type=int, default=4,
                    help="Grounding DINO is far more memory-hungry per "
                         "image than YOLO -- keep this modest.")
    p.add_argument("--splits", nargs="+", default=["train", "val"],
                    choices=["train", "val", "test"])
    p.add_argument("--max-review-images", type=int, default=60)
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def main():
    if AutoProcessor is None:
        raise SystemExit(
            "transformers is not installed. Run:\n"
            "    pip install transformers pillow --break-system-packages\n"
            "then rerun this script.")

    import random
    args = parse_args()
    random.seed(args.seed)

    dino_device = args.dino_device or ("cuda:0" if torch.cuda.is_available() else "cpu")

    print(f"Loading {args.yolo_model} (YOLO side of the comparison)...")
    yolo_model = YOLO(args.yolo_model)

    print(f"Loading {args.dino_model} (Grounding DINO side, device="
          f"{dino_device})...")
    dino_processor = AutoProcessor.from_pretrained(args.dino_model)
    dino_model = AutoModelForZeroShotObjectDetection.from_pretrained(
        args.dino_model).to(dino_device)
    dino_model.eval()

    signature = taxonomy_signature()
    datasets_dir = prepare_datasets.DATASETS_DIR
    review_dir = datasets_dir / "label_gap_review"
    report_path = datasets_dir / "cross_reference_report.json"
    candidates_path = datasets_dir / "cross_reference_candidates.json"

    report = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "yolo_model": args.yolo_model,
        "dino_model": args.dino_model,
        "iou_threshold": args.iou_threshold,
        "yolo_conf": args.yolo_conf,
        "dino_box_threshold": args.dino_box_threshold,
        "dino_text_threshold": args.dino_text_threshold,
        "splits_scanned": args.splits,
        "sources": {},
        "notes": (
            "'agree' = both models detected the same class with IoU >= "
            "iou_threshold -- highest-trust bucket, primary pseudo-label "
            "candidates (see cross_reference_candidates.json). "
            "'yolo_only' includes likely false positives (e.g. UAVDT "
            "person over-triggering seen in the single-model scan) -- "
            "review before trusting. 'dino_only' is the practical answer "
            "to under-recall (e.g. SARD car) -- objects YOLO missed that "
            "an architecturally different, differently-trained model "
            "independently caught -- also review before trusting, since "
            "it has no agreement boost."
        ),
    }
    all_candidates = {}

    for source_name, (prepare_fn, remap_table) in SOURCE_REGISTRY.items():
        missing = missing_classes_for(remap_table)
        if not missing:
            print(f"\n{source_name}: fully annotated, nothing to cross-reference.")
            continue

        print(f"\n{'=' * 70}\n{source_name} -- missing classes: {missing}\n{'=' * 70}")

        dirs = prepare_fn(signature)
        images = gather_images(dirs, splits=tuple(args.splits))
        if args.limit and len(images) > args.limit:
            images = random.sample(images, args.limit)
        if not images:
            print(f"  No images found for splits {args.splits} -- skipping.")
            continue

        yolo_dets = run_yolo(yolo_model, source_name, images, missing,
                              args.yolo_conf, args.device, args.yolo_batch)
        dino_dets = run_dino(dino_processor, dino_model, dino_device,
                              source_name, images, missing,
                              args.dino_box_threshold, args.dino_text_threshold,
                              args.dino_batch)

        summary, candidates = compare_and_review(
            source_name, images, yolo_dets, dino_dets, args.iou_threshold,
            review_dir, args.max_review_images)

        print(f"\n  {source_name} summary:")
        print(f"    agree:      {summary['agree_total']}")
        print(f"    yolo_only:  {summary['yolo_only_total']}")
        print(f"    dino_only:  {summary['dino_only_total']}")
        for cls_name, counts in summary["per_class"].items():
            print(f"      {cls_name:15s} agree={counts['agree']:>5}  "
                  f"yolo_only={counts['yolo_only']:>5}  "
                  f"dino_only={counts['dino_only']:>5}")

        report["sources"][source_name] = summary
        all_candidates[source_name] = candidates

    datasets_dir.mkdir(parents=True, exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    with open(candidates_path, "w") as f:
        json.dump(all_candidates, f, indent=2, default=str)

    print(f"\n{'=' * 70}")
    print(f"Full report: {report_path}")
    print(f"Agree-bucket candidates (future pseudo-label input): {candidates_path}")
    print(f"Review images: {review_dir}/<source>_cross/")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()