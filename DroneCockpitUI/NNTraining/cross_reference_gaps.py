"""
cross_reference_gaps.py -- Cross-references a VisDrone-finetuned YOLO26l
against a VisDrone-finetuned RF-DETR-Small on the same "missing-class"
images check_label_gaps.py flagged, to get a second, architecturally-
independent opinion before trusting anything as a pseudo-label ground
truth for Project R.A.D's UAVDT/SARD label gaps.

=======================================================================
V2 REWRITE (2026-08-11): swapped BOTH models for domain-matched ones.
=======================================================================
The v1 script (COCO-pretrained yolo26l.pt vs. open-vocabulary
IDEA-Research/grounding-dino-tiny) ran into exactly the failure mode its
own FIX-2 note predicted: dino_only outnumbered agree by ~10-17x on both
sources (UAVDT: 81567 vs 4838, SARD: 837 vs 1275), and the confidence
diagnostic showed dino_only detections sitting right on the 0.25 floor
(UAVDT person dino_only mean=0.342, barely above yolo_only's 0.348) --
i.e. exactly diagnosis (a) from that note: genuine low-confidence noise
from an open-vocabulary model reaching for approximate text-prompt
matches on a domain (aerial, top-down, tiny objects) neither COCO nor
Grounding DINO's grounding-pretraining mix represents well. No amount of
IoU/NMS tuning fixes a domain mismatch.

This rewrite replaces both sides with checkpoints actually finetuned on
VisDrone2019-DET (the same aerial domain UAVDT/SARD/Project R.A.D. live
in), from the same author's HF model zoo:
  - YOLO side:  dronefreak/visdrone-yolov26l  (Ultralytics YOLO26l,
    finetuned on VisDrone -- same checkpoint family the rest of this
    project already uses, just domain-adapted instead of COCO-generic).
  - "DINO" side: dronefreak/visdrone-rfdetr-small (RF-DETR Small, a
    DETR-family transformer with a DINOv2 backbone, finetuned on
    VisDrone). Still architecturally independent from YOLO (bipartite-
    matching transformer vs. anchor-free CNN -- the property that made
    a second opinion worth having in the first place), but now CLOSED-
    SET and domain-matched instead of open-vocabulary and domain-
    mismatched. Per its model card, mAP50 on VisDrone test is 33.25 vs.
    the COCO-pretrained rt_detr_l baseline's 21.68 -- confirms the
    domain-matched pretraining is doing real work, not just a hunch.
    (dronefreak/visdrone-rfdetr-medium scores higher still -- 36.82
    mAP50 -- at ~2x the inference cost; swap via --rfdetr-size medium
    if v1's dino_only volume was the concern and speed isn't.)

Practical consequences of the swap:
  1. Both models now speak VisDrone's native 10/11-class taxonomy
     (pedestrian/people/bicycle/car/van/truck/tricycle/awning-tricycle/
     bus/motor[/others]), NOT COCO's 80 classes. Neither
     COCO_NAMES_FOR_UNIFIED (check_label_gaps.py) nor DINO_PHRASES_FOR_
     UNIFIED (v1 of this script) apply anymore -- see
     VISDRONE_NAMES_FOR_UNIFIED below, built by inverting class_map.py's
     own VISDRONE_REMAP (imported directly), so it's guaranteed to match
     the taxonomy the rest of the project actually trains on and can't
     drift out of sync if VISDRONE_REMAP is ever edited.

  2. RF-DETR is closed-set, so all of v1's prompt-engineering (build_
     dino_prompt) and free-text-label matching (_label_to_unified) is
     GONE. Classes are read directly off the loaded checkpoint via
     model.class_names[class_id] (confirmed against Roboflow's current
     RFDETR class reference), same mechanism as YOLO's model.names.
  3. RF-DETR is a DETR-family detector trained with bipartite (Hungarian)
     matching, which is specifically designed to not need NMS the way
     anchor-based/anchor-free CNN detectors do -- it doesn't produce the
     dense overlapping-anchor duplicates NMS exists to collapse. apply_
     nms() is kept (still correct, still harmless) but is OFF by default
     on the RF-DETR side now (--rfdetr-nms-iou, default None = skip).
     Re-enable with e.g. --rfdetr-nms-iou 0.6 if review images show
     literal duplicate boxes; unlike v1, this is now expected to be a
     no-op most of the time rather than doing real work.
  4. transformers is no longer a dependency. New requirement: rfdetr
     (see REQUIREMENTS). huggingface_hub is used for both checkpoints.
  5. Confidence floors: --yolo-conf and --rfdetr-threshold both default
     to 0.25, same floor v1 used -- kept identical on purpose so the
     confidence diagnostic below is comparable to v1's numbers and you
     can see directly whether the domain-matched swap moved the needle
     versus just re-tuning a threshold would have.

WHAT THIS SCRIPT STILL DOES (unchanged from v1):
  1. Reuses check_label_gaps.py's SOURCE_REGISTRY / missing_classes_for /
     gather_images, so both models scan the exact same image set.
  2. Runs the YOLO side via the same .txt-source-list fix check_label_
     gaps.py uses (see that module's docstring).
  3. Runs the RF-DETR side by loading images directly with PIL.
  4. Matches detections per image/class by IoU (greedy best-match) into
     agree / yolo_only / rfdetr_only buckets.
  5. Saves annotated review images (YOLO in red, RF-DETR in blue, agreed
     pairs additionally outlined in green).
  6. Writes cross_reference_report.json, cross_reference_candidates.json
     (agree-bucket only), and cross_reference_full.json (every bucket,
     every image -- see FIX 3 in the v1 history below).
  This script still only measures and reviews -- no label file is ever
  touched here.

REQUIREMENTS:
    pip install rfdetr huggingface_hub pillow --break-system-packages
(torch/ultralytics/opencv are already required by check_label_gaps.py /
train.py.) First run downloads both checkpoints from the HF Hub
(~50-90MB for YOLO26l, ~120MB for RF-DETR-Small).

USAGE:
    # Start small -- sanity-check timing before committing to a full run.
    python cross_reference_gaps.py --limit 200

    # Full run once timing looks acceptable.
    python cross_reference_gaps.py

    # Stronger (slower) RF-DETR checkpoint.
    python cross_reference_gaps.py --rfdetr-size medium

    # Loosen/tighten the "same object" IoU bar for the agree bucket.
    python cross_reference_gaps.py --iou-threshold 0.4

-----------------------------------------------------------------------
v1 HISTORY (kept for context on why the matching/output plumbing below
looks the way it does):

FIX (2026-08-11): run_dino() was calling post_process_grounded_object_
detection(..., box_threshold=..., text_threshold=...) and reading
res["labels"]; current transformers renamed the kwarg to threshold and
the field to text_labels. (Moot now -- Grounding DINO/transformers path
is removed entirely in this v2 rewrite.)

FIX 2 (2026-08-11): Grounding DINO's post-processing doesn't apply NMS,
so apply_nms() was added. On a --limit 200 run it only removed ~2% of
raw detections and barely moved the agree/dino_only ratio -- NOT the
dominant cause of the ~10x mismatch. Confirmed by the full run (see V2
REWRITE note above): it was domain mismatch, not duplicate boxes.

FIX 3 (2026-08-11): compare_and_review() now returns/persists the FULL
per-image, per-bucket detection list (not just the agree bucket), to
cross_reference_full.json -- the substrate a future build_review_queue.py
/ relabel_tool.py needs to let a human triage every mismatch.
-----------------------------------------------------------------------
"""

import argparse
import json
import statistics
import time
from pathlib import Path

import cv2
import numpy as np
from huggingface_hub import hf_hub_download
from PIL import Image
from ultralytics import YOLO

import prepare_datasets
from class_map import taxonomy_signature, VISDRONE_REMAP
from check_label_gaps import (
    SOURCE_REGISTRY,
    missing_classes_for,
    gather_images,
    _write_source_list,
)

try:
    import rfdetr as rfdetr_pkg
except ImportError:
    rfdetr_pkg = None


# =======================================================================
# VisDrone's native class names -> this project's unified taxonomy.
#
# Derived directly from class_map.py's own VISDRONE_REMAP (imported
# above) rather than hand-duplicated here -- so this can never drift out
# of sync if VISDRONE_REMAP is ever edited. (An earlier draft of this
# script reconstructed this table by hand without seeing class_map.py;
# it happened to match VISDRONE_REMAP exactly once compared, but
# deriving it directly removes the need to trust that.)
#
# Both dronefreak checkpoints were trained on VisDrone2019-DET's stock
# 10-class taxonomy (11 including the always-empty "others" class, which
# RF-DETR's checkpoint retains and YOLO's does not -- see each model
# card's Classes section). "others" has zero annotated instances in
# VisDrone's own training set (per the RF-DETR-Small model card's Known
# Limitations) and has no entry in VISDRONE_REMAP, so it's correctly
# left unmapped/dropped below, not guessed into other_vehicle.
# =======================================================================
def _build_visdrone_names_for_unified() -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for visdrone_name, uni_name in VISDRONE_REMAP.items():
        if uni_name is None:
            continue
        out.setdefault(uni_name, []).append(visdrone_name)
    return out


VISDRONE_NAMES_FOR_UNIFIED = _build_visdrone_names_for_unified()

# HF Hub repo IDs. Confirmed live via the HF collection page
# (huggingface.co/collections/dronefreak/visdrone-detection-model-zoo)
# on 2026-08-11 -- NOTE the YOLO model card's own "Load Model" code
# snippet has the repo_id backwards ("dronefreak/yolov26l-visdrone");
# the actual working repo (and the one every other dronefreak VisDrone
# YOLO model card uses consistently in its collection listing) is
# "dronefreak/visdrone-yolov26l", used below. If hf_hub_download 404s,
# that's the first thing to check on HF's side.
YOLO_REPO = "dronefreak/visdrone-yolov26l"
YOLO_WEIGHTS_FILENAME = "best.pt"
RFDETR_REPO_TEMPLATE = "dronefreak/visdrone-rfdetr-{size}"
RFDETR_CHECKPOINT_FILENAME = "checkpoint_best_total.pth"
RFDETR_SIZE_TO_CLASS = {
    "nano": "RFDETRNano",
    "small": "RFDETRSmall",
    "medium": "RFDETRMedium",
}

# Colors for annotated review images (BGR, since we draw with cv2).
COLOR_YOLO = (0, 0, 255)      # red
COLOR_RFDETR = (255, 128, 0)  # blue
COLOR_AGREE = (0, 220, 0)     # green


def build_reverse_taxonomy_map() -> dict:
    """VisDrone class name (lowercase) -> unified class name."""
    reverse = {}
    for uni_name, visdrone_names in VISDRONE_NAMES_FOR_UNIFIED.items():
        for vname in visdrone_names:
            reverse[vname.lower()] = uni_name
    return reverse


def print_taxonomy_mapping():
    print("\n  VisDrone -> unified class mapping in use (derived from "
          "class_map.py's VISDRONE_REMAP):")
    for uni_name, visdrone_names in VISDRONE_NAMES_FOR_UNIFIED.items():
        print(f"    {uni_name:15s} <- {', '.join(visdrone_names)}")


# ---------------------------------------------------------------------
# Geometry (unchanged from v1)
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


def apply_nms(dets: list[dict], iou_thr: float = 0.6) -> list[dict]:
    """
    Greedy per-class NMS. Kept from v1 for anyone who re-enables it via
    --rfdetr-nms-iou, but OFF by default in this rewrite -- RF-DETR's
    bipartite-matching training objective means it doesn't produce the
    dense overlapping-anchor duplicates NMS exists to clean up, unlike
    the open-vocabulary Grounding DINO v1 used this for.
    """
    by_cls: dict[str, list[dict]] = {}
    for d in dets:
        by_cls.setdefault(d["cls"], []).append(d)

    kept: list[dict] = []
    for cls_name, cls_dets in by_cls.items():
        cls_dets = sorted(cls_dets, key=lambda d: d["conf"], reverse=True)
        used = [False] * len(cls_dets)
        for i, di in enumerate(cls_dets):
            if used[i]:
                continue
            kept.append(di)
            for j in range(i + 1, len(cls_dets)):
                if used[j]:
                    continue
                if compute_iou(di["box"], cls_dets[j]["box"]) >= iou_thr:
                    used[j] = True
    return kept


def greedy_match(dets_a: list, dets_b: list, iou_thr: float):
    """Greedy best-IoU matching, class-constrained. Same as v1."""
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
# YOLO pass -- now against the VisDrone-finetuned checkpoint, not COCO.
# ---------------------------------------------------------------------

def resolve_class_indices(model_names: dict, wanted_names: list[str]) -> list[int]:
    """model_names: {idx: name} as returned by an Ultralytics model.
    wanted_names: lowercase VisDrone class names to look up."""
    name_to_idx = {v.lower(): k for k, v in model_names.items()}
    idxs = []
    for name in wanted_names:
        if name in name_to_idx:
            idxs.append(name_to_idx[name])
        else:
            print(f"  [warn] VisDrone class '{name}' not found on this "
                  f"checkpoint's model.names -- skipping. (Checkpoint "
                  f"reports: {sorted(model_names.values())})")
    return idxs


def run_yolo(model: YOLO, source_name: str, image_paths: list[Path],
             missing_classes: list[str], conf_floor: float, device,
             batch: int, reverse_taxonomy: dict) -> dict[str, list[dict]]:
    print(f"\n  [YOLO26-VisDrone] scanning {len(image_paths)} images for "
          f"{missing_classes}...")

    visdrone_names = []
    for c in missing_classes:
        visdrone_names.extend(VISDRONE_NAMES_FOR_UNIFIED.get(c, []))
    class_idxs = resolve_class_indices(model.names, visdrone_names)
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
                visdrone_name = model.names[int(cls_id)].lower()
                uni_name = reverse_taxonomy.get(visdrone_name)
                if uni_name is None or uni_name not in missing_classes:
                    continue
                dets.append({"cls": uni_name, "conf": float(conf),
                             "box": [float(v) for v in xyxy]})
        results[r.path] = dets

    print(f"    Done: {scanned} images in {time.time() - t0:.0f}s.")
    return results


# ---------------------------------------------------------------------
# RF-DETR pass -- closed-set, no prompt engineering needed. Loads
# images directly via PIL, same path-identity reasoning as v1's DINO
# pass (never goes through Ultralytics' loader, so no equivalent risk).
# ---------------------------------------------------------------------

def run_rfdetr(model, source_name: str, image_paths: list[Path],
               missing_classes: list[str], threshold: float, batch: int,
               reverse_taxonomy: dict, nms_iou: float | None) -> dict[str, list[dict]]:
    print(f"\n  [RF-DETR-VisDrone] scanning {len(image_paths)} images for "
          f"{missing_classes}...")

    class_names = model.class_names  # list[str], embedded in the finetuned checkpoint
    results: dict[str, list[dict]] = {}
    scanned, t0 = 0, time.time()
    raw_total, deduped_total = 0, 0

    for start in range(0, len(image_paths), batch):
        chunk = image_paths[start:start + batch]
        pil_images, valid_paths = [], []
        for p in chunk:
            try:
                pil_images.append(Image.open(p).convert("RGB"))
                valid_paths.append(p)
            except Exception as e:
                print(f"    [warn] Could not open {p} ({e}) -- skipping.")

        if not pil_images:
            continue

        # predict() returns a single Detections for one image, or a
        # list of Detections (one per input, same order) for a list --
        # confirmed against Roboflow's current RFDETR class reference.
        batch_result = model.predict(pil_images, threshold=threshold)
        if not isinstance(batch_result, list):
            batch_result = [batch_result]

        for path, det in zip(valid_paths, batch_result):
            dets = []
            for box, score, cls_id in zip(det.xyxy.tolist(),
                                            det.confidence.tolist(),
                                            det.class_id.tolist()):
                visdrone_name = class_names[int(cls_id)].lower()
                uni_name = reverse_taxonomy.get(visdrone_name)
                if uni_name is None or uni_name not in missing_classes:
                    continue
                dets.append({"cls": uni_name, "conf": float(score),
                             "box": [float(v) for v in box]})
            raw_total += len(dets)
            if nms_iou is not None:
                dets = apply_nms(dets, iou_thr=nms_iou)
            deduped_total += len(dets)
            results[str(path)] = dets

        scanned += len(pil_images)
        if scanned % 200 < batch:
            rate = scanned / (time.time() - t0)
            print(f"    ...{scanned}/{len(image_paths)} ({rate:.1f} img/s)")

    if nms_iou is not None:
        dropped = raw_total - deduped_total
        pct = (dropped / raw_total * 100) if raw_total else 0.0
        print(f"    Done: {scanned} images in {time.time() - t0:.0f}s. "
              f"NMS (iou>={nms_iou}) removed {dropped}/{raw_total} raw "
              f"detections ({pct:.0f}%) as near-duplicates.")
    else:
        print(f"    Done: {scanned} images in {time.time() - t0:.0f}s. "
              f"({raw_total} raw detections; NMS skipped -- RF-DETR "
              f"doesn't need it by design, see module docstring.)")
    return results


# ---------------------------------------------------------------------
# Matching + review image output (unchanged from v1, aside from
# renaming the "dino"/"DINO" naming to "rfdetr"/"RF-DETR")
# ---------------------------------------------------------------------

def draw_box(img, box, color, label):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(img, label, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, color, 1, cv2.LINE_AA)


def compare_and_review(source_name: str, image_paths: list[Path],
                        yolo_dets: dict[str, list[dict]],
                        rfdetr_dets: dict[str, list[dict]],
                        iou_threshold: float, review_dir: Path,
                        max_review: int):
    """Same three-bucket (agree/yolo_only/rfdetr_only) matching and full-
    record persistence as v1's compare_and_review (see FIX 3 in the
    module docstring's v1 history)."""
    per_image_results = {}
    agree_count, yolo_only_count, rfdetr_only_count = 0, 0, 0
    per_class_counts = {}
    candidates = []
    full_detections = {}

    yolo_by_str = {str(k): v for k, v in yolo_dets.items()}
    rfdetr_by_str = {str(k): v for k, v in rfdetr_dets.items()}

    for img_path in image_paths:
        key = str(img_path)
        da = yolo_by_str.get(key, [])
        db = rfdetr_by_str.get(key, [])
        if not da and not db:
            continue

        agreed_pairs, only_yolo, only_rfdetr = greedy_match(da, db, iou_threshold)

        agree_count += len(agreed_pairs)
        yolo_only_count += len(only_yolo)
        rfdetr_only_count += len(only_rfdetr)

        agree_entries = []
        for det_a, det_b, iou in agreed_pairs:
            per_class_counts.setdefault(det_a["cls"], {"agree": 0, "yolo_only": 0, "rfdetr_only": 0})
            per_class_counts[det_a["cls"]]["agree"] += 1
            candidates.append({
                "image": key, "cls": det_a["cls"],
                "yolo_conf": det_a["conf"], "rfdetr_conf": det_b["conf"],
                "iou": iou, "box": det_a["box"],
            })
            agree_entries.append({
                "cls": det_a["cls"], "box": det_a["box"],
                "yolo_conf": det_a["conf"], "rfdetr_conf": det_b["conf"],
                "iou": iou,
            })

        for d in only_yolo:
            per_class_counts.setdefault(d["cls"], {"agree": 0, "yolo_only": 0, "rfdetr_only": 0})
            per_class_counts[d["cls"]]["yolo_only"] += 1
        for d in only_rfdetr:
            per_class_counts.setdefault(d["cls"], {"agree": 0, "yolo_only": 0, "rfdetr_only": 0})
            per_class_counts[d["cls"]]["rfdetr_only"] += 1

        per_image_results[key] = {
            "agree": len(agreed_pairs), "yolo_only": len(only_yolo),
            "rfdetr_only": len(only_rfdetr),
        }
        full_detections[key] = {
            "agree": agree_entries,
            "yolo_only": [{"cls": d["cls"], "box": d["box"], "conf": d["conf"]}
                           for d in only_yolo],
            "rfdetr_only": [{"cls": d["cls"], "box": d["box"], "conf": d["conf"]}
                             for d in only_rfdetr],
        }

    ranked = sorted(
        per_image_results.items(),
        key=lambda kv: (kv[1]["yolo_only"] + kv[1]["rfdetr_only"], kv[1]["agree"]),
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
        db = rfdetr_by_str.get(key, [])
        agreed_pairs, only_yolo, only_rfdetr = greedy_match(da, db, iou_threshold)

        for d in only_yolo:
            draw_box(img, d["box"], COLOR_YOLO, f"YOLO {d['cls']} {d['conf']:.2f}")
        for d in only_rfdetr:
            draw_box(img, d["box"], COLOR_RFDETR, f"RFDETR {d['cls']} {d['conf']:.2f}")
        for det_a, det_b, iou in agreed_pairs:
            draw_box(img, det_a["box"], COLOR_AGREE,
                     f"AGREE {det_a['cls']} y{det_a['conf']:.2f}/r{det_b['conf']:.2f} iou{iou:.2f}")

        out_path = out_dir / f"{i:03d}_{Path(img_path).stem}.jpg"
        cv2.imwrite(str(out_path), img)

    summary = {
        "images_with_any_detection": len(per_image_results),
        "agree_total": agree_count,
        "yolo_only_total": yolo_only_count,
        "rfdetr_only_total": rfdetr_only_count,
        "per_class": per_class_counts,
    }
    return summary, candidates, full_detections


def print_confidence_diagnostic(source_name: str, full_detections: dict) -> None:
    """Same as v1, renamed dino_only -> rfdetr_only. Now that both sides
    are domain-matched, expect this to look far less like v1's (means
    hugging the 0.25 floor) -- if it still does, that's a real signal
    something's off (see module docstring), not an artifact of the old
    open-vocab mismatch."""
    by_class: dict[str, dict[str, list[float]]] = {}
    for entry in full_detections.values():
        for bucket in ("yolo_only", "rfdetr_only"):
            for d in entry[bucket]:
                by_class.setdefault(d["cls"], {"yolo_only": [], "rfdetr_only": []})
                by_class[d["cls"]][bucket].append(d["conf"])

    if not by_class:
        return

    print(f"\n  {source_name} confidence diagnostic (mismatch buckets only):")
    for cls_name, buckets in by_class.items():
        for bucket_name, confs in buckets.items():
            if not confs:
                continue
            print(f"    {cls_name:15s} {bucket_name:10s} n={len(confs):>5}  "
                  f"mean={statistics.mean(confs):.3f}  "
                  f"median={statistics.median(confs):.3f}  "
                  f"min={min(confs):.3f}  max={max(confs):.3f}")


# ---------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--yolo-repo", default=YOLO_REPO)
    p.add_argument("--rfdetr-size", default="small", choices=list(RFDETR_SIZE_TO_CLASS))
    p.add_argument("--limit", type=int, default=None)
    p.add_argument("--yolo-conf", type=float, default=0.25)
    p.add_argument("--rfdetr-threshold", type=float, default=0.25)
    p.add_argument("--rfdetr-nms-iou", type=float, default=None,
                    help="Off by default -- RF-DETR shouldn't need NMS. "
                         "Set e.g. 0.6 if review images show literal "
                         "duplicate boxes.")
    p.add_argument("--iou-threshold", type=float, default=0.5)
    p.add_argument("--device", default=0)
    p.add_argument("--yolo-batch", type=int, default=16)
    p.add_argument("--rfdetr-batch", type=int, default=4)
    p.add_argument("--splits", nargs="+", default=["train", "val"],
                    choices=["train", "val", "test"])
    p.add_argument("--max-review-images", type=int, default=60)
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def main():
    if rfdetr_pkg is None:
        raise SystemExit(
            "rfdetr is not installed. Run:\n"
            "    pip install rfdetr huggingface_hub pillow --break-system-packages\n"
            "then rerun this script.")

    import random
    args = parse_args()
    random.seed(args.seed)

    reverse_taxonomy = build_reverse_taxonomy_map()
    print_taxonomy_mapping()

    print(f"\nDownloading/loading {args.yolo_repo} (YOLO side)...")
    yolo_weights = hf_hub_download(repo_id=args.yolo_repo,
                                     filename=YOLO_WEIGHTS_FILENAME)
    yolo_model = YOLO(yolo_weights)
    print(f"  YOLO checkpoint classes: {sorted(yolo_model.names.values())}")

    rfdetr_repo = RFDETR_REPO_TEMPLATE.format(size=args.rfdetr_size)
    rfdetr_class = getattr(rfdetr_pkg, RFDETR_SIZE_TO_CLASS[args.rfdetr_size])
    print(f"\nDownloading/loading {rfdetr_repo} (RF-DETR side)...")
    rfdetr_weights = hf_hub_download(repo_id=rfdetr_repo,
                                       filename=RFDETR_CHECKPOINT_FILENAME)
    rfdetr_model = rfdetr_class(pretrain_weights=rfdetr_weights)
    print(f"  RF-DETR checkpoint classes: {sorted(rfdetr_model.class_names)}")

    signature = taxonomy_signature()
    datasets_dir = prepare_datasets.DATASETS_DIR
    review_dir = datasets_dir / "label_gap_review"
    report_path = datasets_dir / "cross_reference_report.json"
    candidates_path = datasets_dir / "cross_reference_candidates.json"
    full_path = datasets_dir / "cross_reference_full.json"

    report = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "yolo_repo": args.yolo_repo,
        "rfdetr_repo": rfdetr_repo,
        "iou_threshold": args.iou_threshold,
        "rfdetr_nms_iou": args.rfdetr_nms_iou,
        "yolo_conf": args.yolo_conf,
        "rfdetr_threshold": args.rfdetr_threshold,
        "splits_scanned": args.splits,
        "visdrone_to_unified_mapping": VISDRONE_NAMES_FOR_UNIFIED,
        "sources": {},
        "notes": (
            "v2: both sides are VisDrone-domain-finetuned checkpoints "
            "(dronefreak's HF model zoo), replacing v1's COCO-pretrained "
            "YOLO + open-vocabulary Grounding DINO pairing that produced "
            "a ~10-17x agree/dino_only mismatch traced to domain-mismatch "
            "noise, not a matching bug. 'agree' = both models detected "
            "the same class with IoU >= iou_threshold -- highest-trust "
            "bucket, primary pseudo-label candidates (see cross_"
            "reference_candidates.json). visdrone_to_unified_mapping "
            "above is derived directly from class_map.py's VISDRONE_REMAP "
            "at run time, not hand-maintained here. See cross_reference_"
            "full.json for the complete per-image, per-bucket record."
        ),
    }
    all_candidates = {}
    all_full_detections = {}

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
                              args.yolo_conf, args.device, args.yolo_batch,
                              reverse_taxonomy)
        rfdetr_dets = run_rfdetr(rfdetr_model, source_name, images, missing,
                                   args.rfdetr_threshold, args.rfdetr_batch,
                                   reverse_taxonomy, args.rfdetr_nms_iou)

        summary, candidates, full_detections = compare_and_review(
            source_name, images, yolo_dets, rfdetr_dets, args.iou_threshold,
            review_dir, args.max_review_images)

        print(f"\n  {source_name} summary:")
        print(f"    agree:        {summary['agree_total']}")
        print(f"    yolo_only:    {summary['yolo_only_total']}")
        print(f"    rfdetr_only:  {summary['rfdetr_only_total']}")
        for cls_name, counts in summary["per_class"].items():
            print(f"      {cls_name:15s} agree={counts['agree']:>5}  "
                  f"yolo_only={counts['yolo_only']:>5}  "
                  f"rfdetr_only={counts['rfdetr_only']:>5}")

        print_confidence_diagnostic(source_name, full_detections)

        report["sources"][source_name] = summary
        all_candidates[source_name] = candidates
        all_full_detections[source_name] = full_detections

    datasets_dir.mkdir(parents=True, exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    with open(candidates_path, "w") as f:
        json.dump(all_candidates, f, indent=2, default=str)
    with open(full_path, "w") as f:
        json.dump(all_full_detections, f, indent=2, default=str)

    print(f"\n{'=' * 70}")
    print(f"Full report: {report_path}")
    print(f"Agree-bucket candidates: {candidates_path}")
    print(f"Complete per-image record (all buckets): {full_path}")
    print(f"Review images: {review_dir}/<source>_cross/")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()