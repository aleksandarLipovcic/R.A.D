"""
cross_reference_gaps.py -- Cross-references FOUR independent detectors
against each other on the "missing-class" images check_label_gaps.py
flagged, so pseudo-label candidates for Project R.A.D's UAVDT/SARD label
gaps carry a vote count instead of a single agree/disagree bit.

=======================================================================
V3 REWRITE (2026-08-12): back to four models, vote-counted instead of
pairwise agree/disagree.
=======================================================================
V2 (the two-model VisDrone-domain-matched YOLO+RF-DETR pairing) fixed
the domain-mismatch noise problem V1 had, but a domain-matched pair can
still both miss the same thing for the same reason -- e.g. a SARD frame
where two VisDrone-tuned detectors both undercall a cluttered top-down
car cluster because VisDrone itself doesn't have much of that exact
scene composition in its training distribution. A COCO-general detector
and an open-vocabulary detector don't share that blind spot, so they're
back in as two more independent voters:

  1. yolo_visdrone  -- dronefreak/visdrone-yolov26l (unchanged from V2)
  2. rfdetr_visdrone -- dronefreak/visdrone-rfdetr-small (unchanged from V2)
  3. yolo_coco       -- stock COCO-pretrained yolo26l.pt. Broader, more
                        generic training distribution -- exactly the
                        property that lets it catch scenes the two
                        VisDrone-domain specialists both miss. Re-added
                        from V1's COCO_NAMES_FOR_UNIFIED idea (that table
                        previously lived in check_label_gaps.py; it's
                        defined locally here now, see COCO_NAMES_FOR_
                        UNIFIED below, so this script doesn't depend on
                        that module having it).
  4. dino            -- IDEA-Research/grounding-dino-tiny, open-
                        vocabulary, text-prompted. Re-added from V1.
                        Slow (V1's original log: ~73 min on UAVDT, ~67
                        min on SARD) -- this is the expensive part of a
                        full run. Use --skip-dino / --limit while
                        iterating, drop them for the real pass.

VOTE-BASED TIERING (replaces V2's agree/yolo_only/rfdetr_only buckets):
Detections from all four models are clustered per image/class by IoU
(see cluster_detections()) into "vote clusters" -- one cluster per
physical object, carrying which of the four models found it and at what
confidence. generate_pseudo_labels.py (downstream) tiers on vote_count:
  1/4 votes -> always review (no second opinion at all)
  2/4 votes -> review UNLESS it clears a confidence floor (same spirit
               as V2's agree-bucket threshold -- two independent
               detections is real signal, but not automatically enough)
  3-4/4 votes -> auto-accept unconditionally -- that level of
               independent, architecturally-diverse consensus doesn't
               need a confidence floor on top of it.
This script itself does no accept/reject -- it only measures, clusters,
and outputs the full vote record. See generate_pseudo_labels.py for the
tiering logic that consumes cross_reference_candidates.json / cross_
reference_full.json below.

PHASE-LOADED MODEL EXECUTION (VRAM management):
All four checkpoints loaded simultaneously would stack on top of each
other in VRAM on a 6GB 3060 -- exactly the kind of WDDM shared-memory
spillover that's bitten this project before (see NNTraining/train.py's
mitigations). So this script does NOT load all four up front. Instead:
for each model in turn, load it, scan EVERY source's images, unload it
(del + torch.cuda.empty_cache()), then move to the next model. Only one
model's weights are resident in VRAM at any given time. Clustering
happens only after all four phases have produced their per-image
detection dicts. This also means each model is loaded exactly once
total (not once per source), which is strictly less loading overhead
than the old per-source-then-per-model nesting would have been anyway.

WHAT THIS SCRIPT STILL DOES (unchanged from V2):
  1. Reuses check_label_gaps.py's SOURCE_REGISTRY / missing_classes_for /
     gather_images, so all four models scan the exact same image set.
  2. Runs YOLO-family models via the same .txt-source-list fix check_
     label_gaps.py uses (see that module's docstring).
  3. Saves annotated review images -- one model per color, cluster
     outline color keyed to vote_count tier.
  4. Writes cross_reference_report.json, cross_reference_candidates.json
     (vote_count >= 2 clusters only -- i.e. anything with at least one
     independent second opinion), and cross_reference_full.json (every
     cluster, every image, including vote_count == 1 -- the substrate
     ALWAYS_REVIEW classes in generate_pseudo_labels.py need).
  This script still only measures and reviews -- no label file is ever
  touched here.

FIX (2026-08-12): compare_and_review() called out_dir.mkdir(parents=True,
exist_ok=True) for label_gap_review/<source>_cross/ but never cleared it
first -- the same staleness bug as check_label_gaps.py's
save_review_images() (see that module's matching 2026-08-12 FIX note).
Review images from a previous run (a different --iou-threshold /
--skip-coco / --skip-dino / --limit, or just a run that produced more
low-vote clusters and filled more numbered slots) could survive on disk
mixed in with a newer run's images, with no way to tell which is
current just by looking at the folder.

This affects ONLY the rendered .jpg review images under
label_gap_review/<source>_cross/ -- it does NOT affect
cross_reference_report.json / cross_reference_candidates.json /
cross_reference_full.json. Each of those is a single json.dump() call
under mode "w", executed once at the very end of main() after all four
phases have finished, and built entirely from this run's in-memory
`dets` dict -- nothing in that path ever reads a previous run's output
back off disk. In particular, the expensive part of a run (the four
phase-loaded model scans themselves, ~3hr with DINO included) is
unaffected by this bug and does NOT need to be repeated to get correct
JSON output; only the review-image folder needs a fresh look, and
regenerating it does not require rerunning the scans -- it's produced
from the same in-memory clusters right after they're computed.

Fixed via clean_review_subdir(), imported from check_label_gaps.py
(defined there, reused here, rather than duplicated) -- see that
function's docstring for exactly what it does and its safety guard.

REQUIREMENTS:
    pip install rfdetr huggingface_hub pillow transformers torch \
        --break-system-packages
(ultralytics/opencv/numpy are already required by check_label_gaps.py /
train.py.) First run downloads all four checkpoints (~50-90MB YOLO26l x2,
~120MB RF-DETR-Small, ~170MB Grounding DINO tiny).

USAGE:
    # Start small -- sanity-check timing before committing to a full run,
    # and skip DINO while you're iterating on everything else (it's the
    # slow one -- add it back for the real pass).
    python cross_reference_gaps.py --limit 200 --skip-dino

    # Full run, all four models. Budget ~70 extra minutes per source for
    # DINO on top of the other three's ~5-10 minutes combined.
    python cross_reference_gaps.py

    # Drop the COCO-general detector too, if VisDrone-domain-vs-DINO is
    # the comparison you actually want for a given pass.
    python cross_reference_gaps.py --skip-coco

    # Loosen/tighten the "same object" IoU bar used for clustering.
    python cross_reference_gaps.py --iou-threshold 0.4

-----------------------------------------------------------------------
V2 HISTORY (kept for context -- the two-model VisDrone-domain-matched
pairing this rewrite builds on top of):

V1 ran COCO-pretrained yolo26l.pt against open-vocabulary Grounding
DINO and got dino_only outnumbering agree by ~10-17x on both sources --
traced to domain-mismatch noise (DINO reaching for approximate
text-prompt matches on a domain neither COCO nor its own grounding-
pretraining mix represents well), not a matching bug. V2 swapped BOTH
models for VisDrone-finetuned checkpoints (dronefreak's HF model zoo)
and confirmed the confidence-diagnostic noise mostly went away. This V3
rewrite keeps V2's two VisDrone-domain models AND brings V1's two
general-purpose models back -- the point isn't that V1's pairing was
wrong, it's that a domain specialist pair and a domain-general pair
catch different blind spots, so having all four as independent voters
beats picking one pairing over the other.
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
    clean_review_subdir,
)

try:
    import rfdetr as rfdetr_pkg
except ImportError:
    rfdetr_pkg = None

try:
    import torch
    from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
except ImportError:
    torch = None
    AutoProcessor = None
    AutoModelForZeroShotObjectDetection = None


# =======================================================================
# Model identity -- used as dict keys throughout (votes dicts, review
# image colors, CLI flags) so adding/removing a model later is a matter
# of editing MODEL_TAGS + the corresponding load/run/unload code, not
# hunting down every place a bucket name was hardcoded.
# =======================================================================
MODEL_TAGS = ["yolo_visdrone", "rfdetr_visdrone", "yolo_coco", "dino"]

MODEL_DISPLAY = {
    "yolo_visdrone": "YOLO26-VisDrone",
    "rfdetr_visdrone": "RF-DETR-VisDrone",
    "yolo_coco": "YOLO26-COCO",
    "dino": "Grounding-DINO",
}

# BGR colors for annotated review images (cv2 draws in BGR).
MODEL_COLOR = {
    "yolo_visdrone": (0, 0, 255),      # red
    "rfdetr_visdrone": (255, 128, 0),  # blue
    "yolo_coco": (0, 255, 255),        # yellow
    "dino": (255, 0, 255),             # magenta
}

# Cluster outline color keyed to vote_count -- drawn as a heavier box
# around the representative detection so the review image shows both
# "which models fired" (thin per-model boxes) and "how much consensus"
# (thick outline) at a glance.
VOTE_TIER_COLOR = {
    1: (128, 128, 128),  # gray  -- single model, always-review territory
    2: (0, 165, 255),    # orange -- two votes, threshold-gated
    3: (0, 220, 0),      # green -- three votes, auto-accept
    4: (0, 255, 0),      # bright green -- unanimous, auto-accept
}


# =======================================================================
# VisDrone's native class names -> unified taxonomy. Derived from
# class_map.py's own VISDRONE_REMAP (see V2's docstring for why -- can't
# drift out of sync if VISDRONE_REMAP is ever edited).
# =======================================================================
def _build_visdrone_names_for_unified() -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for visdrone_name, uni_name in VISDRONE_REMAP.items():
        if uni_name is None:
            continue
        out.setdefault(uni_name, []).append(visdrone_name)
    return out


VISDRONE_NAMES_FOR_UNIFIED = _build_visdrone_names_for_unified()

# COCO's 80-class taxonomy -> unified. Hand-maintained here (NOT derived
# from a REMAP table the way VisDrone's is, since COCO isn't one of this
# project's training sources -- it's only ever used as a second-opinion
# voter in this script). Only classes with a real COCO equivalent are
# listed; anything with no COCO analogue (this project has none missing
# an equivalent given the five unified classes) is simply absent.
COCO_NAMES_FOR_UNIFIED = {
    "person": ["person"],
    "car": ["car"],
    "large_vehicle": ["bus", "truck"],
    "motorcycle": ["motorcycle"],
    "other_vehicle": ["bicycle"],
}

# Grounding DINO is open-vocabulary -- there's no fixed class list to
# remap, instead each unified class gets one or more free-text phrases
# it's prompted with, and whatever phrase comes back in the model's
# output is matched back to the unified class that phrase belongs to.
# Keep phrases short and concrete -- DINO's grounding performance is
# noticeably worse on multi-word or abstract phrases.
DINO_PHRASES_FOR_UNIFIED = {
    "person": ["person", "pedestrian"],
    "car": ["car"],
    "large_vehicle": ["bus", "truck", "van"],
    "motorcycle": ["motorcycle", "motorbike"],
    "other_vehicle": ["bicycle", "tricycle"],
}


def build_reverse_taxonomy_map(names_for_unified: dict[str, list[str]]) -> dict[str, str]:
    """native class name (lowercase) -> unified class name, for any of
    the native-name tables above (VisDrone or COCO)."""
    reverse = {}
    for uni_name, native_names in names_for_unified.items():
        for nname in native_names:
            reverse[nname.lower()] = uni_name
    return reverse


def build_dino_phrase_map(missing_classes: list[str]) -> dict[str, str]:
    """phrase (lowercase) -> unified class name, restricted to the
    classes actually being scanned for on this source."""
    out = {}
    for uni_name in missing_classes:
        for phrase in DINO_PHRASES_FOR_UNIFIED.get(uni_name, []):
            out[phrase.lower()] = uni_name
    return out


def build_dino_prompt(missing_classes: list[str]) -> str:
    """Grounding DINO wants a single string of period-separated, lower-
    cased phrases, e.g. 'person. car. bus. truck.'"""
    phrases = []
    for uni_name in missing_classes:
        phrases.extend(DINO_PHRASES_FOR_UNIFIED.get(uni_name, []))
    # de-dupe while preserving order (a phrase like "truck" could appear
    # under large_vehicle only, so dupes aren't expected, but be safe).
    seen = set()
    ordered = [p for p in phrases if not (p in seen or seen.add(p))]
    return ". ".join(ordered) + "."


def _label_to_unified(text_label: str, phrase_to_unified: dict[str, str]) -> str | None:
    """DINO's returned text_labels are usually the matched phrase
    verbatim, but can come back as a fragment or with stray punctuation
    depending on transformers version -- try exact match first, then
    substring containment against the known phrase set."""
    key = text_label.strip().lower().strip(".")
    if key in phrase_to_unified:
        return phrase_to_unified[key]
    for phrase, uni_name in phrase_to_unified.items():
        if phrase in key or key in phrase:
            return uni_name
    return None


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


def apply_nms(dets: list[dict], iou_thr: float = 0.6) -> list[dict]:
    """Greedy per-class NMS for a single model's raw output, kept
    available via --rfdetr-nms-iou / --dino-nms-iou. Off by default for
    RF-DETR (bipartite matching doesn't need it) and for DINO (grounding
    decode doesn't produce dense overlaps the way anchor-based detectors
    can). On by default for nothing -- these tend to be no-ops now that
    the models are cleaner; only enable if review images show literal
    duplicate boxes from a specific model."""
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


def cluster_detections(dets_by_model: dict[str, list[dict]], iou_thr: float) -> list[dict]:
    """
    Clusters detections from ALL FOUR models (for one image) into vote
    clusters, replacing V2's pairwise greedy_match. Each cluster
    represents one physical object as agreed by some subset of models.

    Algorithm: flatten every model's detections for this image, group by
    class, then greedily seed a cluster from the highest-remaining-
    confidence detection and absorb the best-IoU detection from every
    OTHER model that clears iou_thr against the seed (each model
    contributes at most one detection per cluster). Repeat until every
    detection is claimed. This generalizes V2's two-model greedy_match
    to N models without needing N-way exact correspondence.

    Returns a list of cluster dicts:
      cls, box (the seed/representative box -- highest-confidence
      member), rep_model, votes ({model_tag: conf-or-None}), vote_count,
      member_ious ({model_tag: iou-vs-representative-or-None}).
    """
    flat = []
    for model_tag, dets in dets_by_model.items():
        for d in dets:
            flat.append({**d, "model": model_tag})

    by_cls: dict[str, list[dict]] = {}
    for d in flat:
        by_cls.setdefault(d["cls"], []).append(d)

    clusters = []
    for cls_name, cls_dets in by_cls.items():
        remaining = sorted(cls_dets, key=lambda d: d["conf"], reverse=True)
        used = [False] * len(remaining)
        for i, di in enumerate(remaining):
            if used[i]:
                continue
            used[i] = True
            members = {di["model"]: di}
            for j in range(i + 1, len(remaining)):
                if used[j]:
                    continue
                dj = remaining[j]
                if dj["model"] in members:
                    continue  # this model already has a member in this cluster
                if compute_iou(di["box"], dj["box"]) >= iou_thr:
                    used[j] = True
                    members[dj["model"]] = dj

            votes = {m: (members[m]["conf"] if m in members else None) for m in MODEL_TAGS}
            member_ious = {
                m: (compute_iou(di["box"], members[m]["box"]) if m in members and m != di["model"] else
                    (1.0 if m == di["model"] else None))
                for m in MODEL_TAGS
            }
            clusters.append({
                "cls": cls_name,
                "box": di["box"],
                "rep_model": di["model"],
                "votes": votes,
                "vote_count": len(members),
                "member_ious": member_ious,
            })
    return clusters


# ---------------------------------------------------------------------
# YOLO pass -- shared by BOTH yolo_visdrone and yolo_coco (they only
# differ in which checkpoint is loaded and which native-name table
# resolves classes-of-interest to model.names indices).
# ---------------------------------------------------------------------

def resolve_class_indices(model_names: dict, wanted_names: list[str]) -> list[int]:
    """model_names: {idx: name} as returned by an Ultralytics model.
    wanted_names: lowercase native class names to look up."""
    name_to_idx = {v.lower(): k for k, v in model_names.items()}
    idxs = []
    for name in wanted_names:
        if name in name_to_idx:
            idxs.append(name_to_idx[name])
        else:
            print(f"  [warn] native class '{name}' not found on this "
                  f"checkpoint's model.names -- skipping. (Checkpoint "
                  f"reports: {sorted(model_names.values())})")
    return idxs


def run_yolo(model: YOLO, model_tag: str, source_name: str, image_paths: list[Path],
             missing_classes: list[str], conf_floor: float, device,
             batch: int, reverse_taxonomy: dict,
             names_for_unified: dict[str, list[str]],
             list_path_suffix: str) -> dict[str, list[dict]]:
    print(f"\n  [{MODEL_DISPLAY[model_tag]}] scanning {len(image_paths)} "
          f"images for {missing_classes}...")

    native_names = []
    for c in missing_classes:
        native_names.extend(names_for_unified.get(c, []))
    class_idxs = resolve_class_indices(model.names, native_names)
    if not class_idxs:
        print("    Nothing to scan for -- skipping.")
        return {}

    list_path = prepare_datasets.DATASETS_DIR / f"_xref_{source_name.lower()}_{list_path_suffix}.txt"
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
                native_name = model.names[int(cls_id)].lower()
                uni_name = reverse_taxonomy.get(native_name)
                if uni_name is None or uni_name not in missing_classes:
                    continue
                dets.append({"cls": uni_name, "conf": float(conf),
                             "box": [float(v) for v in xyxy]})
        results[r.path] = dets

    print(f"    Done: {scanned} images in {time.time() - t0:.0f}s.")
    return results


# ---------------------------------------------------------------------
# RF-DETR pass -- unchanged from V2 aside from the model_tag threading.
# ---------------------------------------------------------------------

def run_rfdetr(model, source_name: str, image_paths: list[Path],
               missing_classes: list[str], threshold: float, batch: int,
               reverse_taxonomy: dict, nms_iou: float | None) -> dict[str, list[dict]]:
    print(f"\n  [{MODEL_DISPLAY['rfdetr_visdrone']}] scanning "
          f"{len(image_paths)} images for {missing_classes}...")

    class_names = model.class_names
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

        batch_result = model.predict(pil_images, threshold=threshold)
        if not isinstance(batch_result, list):
            batch_result = [batch_result]

        for path, det in zip(valid_paths, batch_result):
            dets = []
            for box, score, cls_id in zip(det.xyxy.tolist(),
                                            det.confidence.tolist(),
                                            det.class_id.tolist()):
                native_name = class_names[int(cls_id)].lower()
                uni_name = reverse_taxonomy.get(native_name)
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
              f"({raw_total} raw detections; NMS skipped.)")
    return results


# ---------------------------------------------------------------------
# Grounding DINO pass -- reconstructed from V1 (removed in V2, brought
# back here). Uses transformers' zero-shot object detection API.
# NOTE: post_process_grounded_object_detection's kwarg/field names have
# changed across transformers versions before (V1 hit exactly this --
# see V2's docstring FIX note: box_threshold/text_threshold ->
# threshold/text_threshold, and res["labels"] -> res["text_labels"]).
# If this errors on your installed transformers version, that renamed-
# argument issue is the first thing to check.
# ---------------------------------------------------------------------

def run_dino(processor, model, device_str: str, source_name: str,
             image_paths: list[Path], missing_classes: list[str],
             box_threshold: float, text_threshold: float, batch: int,
             nms_iou: float | None) -> dict[str, list[dict]]:
    print(f"\n  [{MODEL_DISPLAY['dino']}] scanning {len(image_paths)} "
          f"images for {missing_classes}...")

    phrase_to_unified = build_dino_phrase_map(missing_classes)
    prompt = build_dino_prompt(missing_classes)
    print(f"    Prompt: \"{prompt}\"")

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

        inputs = processor(images=pil_images, text=[prompt] * len(pil_images),
                            return_tensors="pt").to(device_str)
        with torch.no_grad():
            outputs = model(**inputs)

        target_sizes = [(img.size[1], img.size[0]) for img in pil_images]  # (h, w)
        post = processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids, threshold=box_threshold,
            text_threshold=text_threshold, target_sizes=target_sizes)

        for path, res in zip(valid_paths, post):
            dets = []
            for box, score, text_label in zip(res["boxes"].tolist(),
                                                res["scores"].tolist(),
                                                res["text_labels"]):
                uni_name = _label_to_unified(text_label, phrase_to_unified)
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
              f"({raw_total} raw detections; NMS skipped.)")
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
                        dets_by_model_and_image: dict[str, dict[str, list[dict]]],
                        iou_threshold: float, review_dir: Path,
                        max_review: int):
    """
    dets_by_model_and_image: {model_tag: {image_key: [dets]}} for all
    four models on this source. Clusters every image's detections into
    vote clusters, saves annotated review images for the images with
    the most single/low-vote clusters (the ones most worth a human
    glance), and returns (summary, all_clusters_flat, full_detections).

    full_detections: {image_key: [cluster, ...]} -- EVERY cluster,
    EVERY vote_count, keyed by image. This is what ALWAYS_REVIEW classes
    in generate_pseudo_labels.py pull vote_count==1 items from.

    all_clusters_flat: same clusters, flattened to a single list with
    "image" added to each, filtered to vote_count >= 2 only -- these are
    the multi-model-corroborated candidates (cross_reference_
    candidates.json).
    """
    full_detections: dict[str, list[dict]] = {}
    candidates: list[dict] = []
    vote_count_totals = {1: 0, 2: 0, 3: 0, 4: 0}
    per_class_vote_totals: dict[str, dict[int, int]] = {}

    by_model_str = {m: {str(k): v for k, v in d.items()}
                     for m, d in dets_by_model_and_image.items()}

    per_image_low_vote_count = {}  # image_key -> count of vote_count<=2 clusters, for picking review images

    for img_path in image_paths:
        key = str(img_path)
        dets_by_model = {m: by_model_str.get(m, {}).get(key, []) for m in MODEL_TAGS}
        if not any(dets_by_model.values()):
            continue

        clusters = cluster_detections(dets_by_model, iou_threshold)
        if not clusters:
            continue

        full_detections[key] = clusters
        low_vote = sum(1 for c in clusters if c["vote_count"] <= 2)
        per_image_low_vote_count[key] = low_vote

        for c in clusters:
            vote_count_totals[c["vote_count"]] += 1
            per_class_vote_totals.setdefault(c["cls"], {1: 0, 2: 0, 3: 0, 4: 0})
            per_class_vote_totals[c["cls"]][c["vote_count"]] += 1
            if c["vote_count"] >= 2:
                candidates.append({**c, "image": key})

    ranked = sorted(per_image_low_vote_count.items(), key=lambda kv: kv[1], reverse=True)
    chosen = [Path(k) for k, _ in ranked[:max_review]]

    out_dir = review_dir / f"{source_name.lower()}_cross"
    clean_review_subdir(out_dir)
    print(f"\n  Saving {len(chosen)} cross-reference review images to "
          f"{out_dir}/ ...")

    for i, img_path in enumerate(chosen):
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        key = str(img_path)
        # NOTE: only the representative box is drawn (thick, vote-tier
        # colored) -- cluster_detections() only retains each non-
        # representative member's confidence and its IoU against the
        # representative, not that member's own box coordinates, so a
        # true per-model overlay isn't available without re-deriving it.
        # The label lists which models contributed by name/confidence,
        # which is enough to spot-check without needing four overlapping
        # boxes per cluster cluttering the image.
        for c in full_detections.get(key, []):
            contributors = "+".join(
                f"{m}:{c['votes'][m]:.2f}" for m in MODEL_TAGS if c["votes"][m] is not None)
            draw_box(img, c["box"], VOTE_TIER_COLOR[c["vote_count"]],
                     f"{c['cls']} v{c['vote_count']}/4 ({contributors})")
        out_path = out_dir / f"{i:03d}_{Path(img_path).stem}.jpg"
        cv2.imwrite(str(out_path), img)

    summary = {
        "vote_count_totals": vote_count_totals,
        "per_class": per_class_vote_totals,
    }
    return summary, candidates, full_detections


def print_confidence_diagnostic(source_name: str, full_detections: dict) -> None:
    """For vote_count==1 clusters (the ones with zero corroboration),
    breaks confidence down by WHICH single model found it. A model that
    systematically fires vote_count==1 with low confidence on a class is
    probably contributing noise there, not signal -- worth knowing
    before leaning on that model's single-model hits for anything (e.g.
    an ALWAYS_REVIEW class, where single-model hits are exactly what
    gets queued)."""
    by_model_cls: dict[str, dict[str, list[float]]] = {}
    for clusters in full_detections.values():
        for c in clusters:
            if c["vote_count"] != 1:
                continue
            m = c["rep_model"]
            by_model_cls.setdefault(m, {}).setdefault(c["cls"], []).append(c["votes"][m])

    if not by_model_cls:
        return

    print(f"\n  {source_name} single-vote (vote_count==1) confidence diagnostic:")
    for m in MODEL_TAGS:
        cls_confs = by_model_cls.get(m, {})
        for cls_name, confs in cls_confs.items():
            print(f"    {MODEL_DISPLAY[m]:18s} {cls_name:15s} n={len(confs):>5}  "
                  f"mean={statistics.mean(confs):.3f}  "
                  f"median={statistics.median(confs):.3f}  "
                  f"min={min(confs):.3f}  max={max(confs):.3f}")


# ---------------------------------------------------------------------
# Model load/unload -- phase-based, one model resident in VRAM at a
# time. See module docstring's "PHASE-LOADED MODEL EXECUTION" section.
# ---------------------------------------------------------------------

def empty_cuda_cache():
    """Call AFTER the caller has already `del`'d its own references to the
    model object(s) -- del inside a helper function only removes that
    helper's local parameter binding, not the caller's variable, so it
    would silently fail to free anything if done that way. The caller
    must del its own variable first; this just releases the cache once
    nothing references the weights anymore."""
    if torch is not None and torch.cuda.is_available():
        torch.cuda.empty_cache()


def resolve_torch_device(device) -> str:
    if torch is None:
        return "cpu"
    if isinstance(device, str) and device.lower() == "cpu":
        return "cpu"
    if not torch.cuda.is_available():
        return "cpu"
    return f"cuda:{device}" if str(device).isdigit() else str(device)


# ---------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--yolo-repo", default="dronefreak/visdrone-yolov26l")
    p.add_argument("--rfdetr-size", default="small",
                    choices=["nano", "small", "medium"])
    p.add_argument("--yolo-coco-weights", default="yolo26l.pt",
                    help="Stock Ultralytics checkpoint name/path -- "
                         "auto-downloaded from Ultralytics' own hub if "
                         "not already present locally, same as train.py "
                         "does for a fresh base checkpoint.")
    p.add_argument("--dino-repo", default="IDEA-Research/grounding-dino-tiny")
    p.add_argument("--skip-coco", action="store_true",
                    help="Skip the COCO-general YOLO voter.")
    p.add_argument("--skip-dino", action="store_true",
                    help="Skip the Grounding DINO voter -- this is the "
                         "slow one (V1 log: ~70min/source). Use while "
                         "iterating, drop for the real pass.")
    p.add_argument("--limit", type=int, default=None)
    p.add_argument("--yolo-conf", type=float, default=0.25,
                    help="Confidence floor for BOTH yolo_visdrone and "
                         "yolo_coco -- kept as one flag since it's the "
                         "same floor V1/V2 always used for the YOLO "
                         "side; use --yolo-coco-conf to override just "
                         "the COCO one if you want them to differ.")
    p.add_argument("--yolo-coco-conf", type=float, default=None,
                    help="Overrides --yolo-conf for the COCO voter only. "
                         "Defaults to --yolo-conf's value if unset.")
    p.add_argument("--rfdetr-threshold", type=float, default=0.25)
    p.add_argument("--rfdetr-nms-iou", type=float, default=None)
    p.add_argument("--dino-box-threshold", type=float, default=0.25)
    p.add_argument("--dino-text-threshold", type=float, default=0.20)
    p.add_argument("--dino-nms-iou", type=float, default=None)
    p.add_argument("--iou-threshold", type=float, default=0.5,
                    help="IoU bar for clustering detections from "
                         "different models into the same vote cluster.")
    p.add_argument("--device", default=0)
    p.add_argument("--yolo-batch", type=int, default=16)
    p.add_argument("--rfdetr-batch", type=int, default=4)
    p.add_argument("--dino-batch", type=int, default=4)
    p.add_argument("--splits", nargs="+", default=["train", "val"],
                    choices=["train", "val", "test"])
    p.add_argument("--max-review-images", type=int, default=60)
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def main():
    if rfdetr_pkg is None:
        raise SystemExit(
            "rfdetr is not installed. Run:\n"
            "    pip install rfdetr huggingface_hub pillow transformers torch "
            "--break-system-packages\nthen rerun this script.")
    import random
    args = parse_args()
    random.seed(args.seed)

    if not args.skip_dino and (torch is None or AutoProcessor is None):
        raise SystemExit(
            "transformers/torch not installed, and --skip-dino was not "
            "passed. Run:\n"
            "    pip install transformers torch --break-system-packages\n"
            "or rerun with --skip-dino to proceed without the DINO voter.")

    device_str = resolve_torch_device(args.device)
    yolo_coco_conf = args.yolo_coco_conf if args.yolo_coco_conf is not None else args.yolo_conf

    visdrone_reverse = build_reverse_taxonomy_map(VISDRONE_NAMES_FOR_UNIFIED)
    coco_reverse = build_reverse_taxonomy_map(COCO_NAMES_FOR_UNIFIED)

    signature = taxonomy_signature()
    datasets_dir = prepare_datasets.DATASETS_DIR
    review_dir = datasets_dir / "label_gap_review"
    report_path = datasets_dir / "cross_reference_report.json"
    candidates_path = datasets_dir / "cross_reference_candidates.json"
    full_path = datasets_dir / "cross_reference_full.json"

    # ---- Gather image sets + per-source missing classes up front, once,
    # so every phase below scans the exact same images without re-doing
    # dataset prep / gathering four times. ----
    sources = {}
    for source_name, (prepare_fn, remap_table) in SOURCE_REGISTRY.items():
        missing = missing_classes_for(remap_table)
        if not missing:
            print(f"\n{source_name}: fully annotated, nothing to cross-reference.")
            continue
        dirs = prepare_fn(signature)
        images = gather_images(dirs, splits=tuple(args.splits))
        if args.limit and len(images) > args.limit:
            images = random.sample(images, args.limit)
        if not images:
            print(f"  {source_name}: no images found for splits {args.splits} -- skipping.")
            continue
        sources[source_name] = {"images": images, "missing": missing}

    if not sources:
        raise SystemExit("No sources have missing classes to cross-reference. Nothing to do.")

    # dets[model_tag][source_name] = {image_key: [dets]}
    dets: dict[str, dict[str, dict[str, list[dict]]]] = {m: {} for m in MODEL_TAGS}

    # ---- Phase 1: yolo_visdrone ----
    print(f"\n{'#' * 70}\nPHASE 1/4: {MODEL_DISPLAY['yolo_visdrone']}\n{'#' * 70}")
    print(f"Downloading/loading {args.yolo_repo}...")
    yolo_vd_weights = hf_hub_download(repo_id=args.yolo_repo, filename="best.pt")
    yolo_vd_model = YOLO(yolo_vd_weights)
    print(f"  Checkpoint classes: {sorted(yolo_vd_model.names.values())}")
    for source_name, s in sources.items():
        dets["yolo_visdrone"][source_name] = run_yolo(
            yolo_vd_model, "yolo_visdrone", source_name, s["images"], s["missing"],
            args.yolo_conf, args.device, args.yolo_batch, visdrone_reverse,
            VISDRONE_NAMES_FOR_UNIFIED, "yolovd")
    del yolo_vd_model
    empty_cuda_cache()

    # ---- Phase 2: rfdetr_visdrone ----
    print(f"\n{'#' * 70}\nPHASE 2/4: {MODEL_DISPLAY['rfdetr_visdrone']}\n{'#' * 70}")
    rfdetr_repo = f"dronefreak/visdrone-rfdetr-{args.rfdetr_size}"
    rfdetr_class_name = {"nano": "RFDETRNano", "small": "RFDETRSmall",
                          "medium": "RFDETRMedium"}[args.rfdetr_size]
    rfdetr_class = getattr(rfdetr_pkg, rfdetr_class_name)
    print(f"Downloading/loading {rfdetr_repo}...")
    rfdetr_weights = hf_hub_download(repo_id=rfdetr_repo,
                                       filename="checkpoint_best_total.pth")
    rfdetr_model = rfdetr_class(pretrain_weights=rfdetr_weights)
    print(f"  Checkpoint classes: {sorted(rfdetr_model.class_names)}")
    for source_name, s in sources.items():
        dets["rfdetr_visdrone"][source_name] = run_rfdetr(
            rfdetr_model, source_name, s["images"], s["missing"],
            args.rfdetr_threshold, args.rfdetr_batch, visdrone_reverse,
            args.rfdetr_nms_iou)
    del rfdetr_model
    empty_cuda_cache()

    # ---- Phase 3: yolo_coco ----
    if args.skip_coco:
        print(f"\n{'#' * 70}\nPHASE 3/4: {MODEL_DISPLAY['yolo_coco']} -- SKIPPED (--skip-coco)\n{'#' * 70}")
        for source_name in sources:
            dets["yolo_coco"][source_name] = {}
    else:
        print(f"\n{'#' * 70}\nPHASE 3/4: {MODEL_DISPLAY['yolo_coco']}\n{'#' * 70}")
        print(f"Loading {args.yolo_coco_weights} (auto-downloads if not local)...")
        yolo_coco_model = YOLO(args.yolo_coco_weights)
        print(f"  Checkpoint classes: {sorted(yolo_coco_model.names.values())}")
        for source_name, s in sources.items():
            dets["yolo_coco"][source_name] = run_yolo(
                yolo_coco_model, "yolo_coco", source_name, s["images"], s["missing"],
                yolo_coco_conf, args.device, args.yolo_batch, coco_reverse,
                COCO_NAMES_FOR_UNIFIED, "yolococo")
        del yolo_coco_model
        empty_cuda_cache()

    # ---- Phase 4: dino ----
    if args.skip_dino:
        print(f"\n{'#' * 70}\nPHASE 4/4: {MODEL_DISPLAY['dino']} -- SKIPPED (--skip-dino)\n{'#' * 70}")
        for source_name in sources:
            dets["dino"][source_name] = {}
    else:
        print(f"\n{'#' * 70}\nPHASE 4/4: {MODEL_DISPLAY['dino']}\n{'#' * 70}")
        print(f"Downloading/loading {args.dino_repo}...")
        dino_processor = AutoProcessor.from_pretrained(args.dino_repo)
        dino_model = AutoModelForZeroShotObjectDetection.from_pretrained(args.dino_repo)
        dino_model = dino_model.to(device_str)
        dino_model.eval()
        for source_name, s in sources.items():
            dets["dino"][source_name] = run_dino(
                dino_processor, dino_model, device_str, source_name,
                s["images"], s["missing"], args.dino_box_threshold,
                args.dino_text_threshold, args.dino_batch, args.dino_nms_iou)
        del dino_model, dino_processor
        empty_cuda_cache()

    # ---- Cluster + report, per source ----
    report = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "models": {
            "yolo_visdrone": args.yolo_repo,
            "rfdetr_visdrone": rfdetr_repo,
            "yolo_coco": (None if args.skip_coco else args.yolo_coco_weights),
            "dino": (None if args.skip_dino else args.dino_repo),
        },
        "iou_threshold": args.iou_threshold,
        "yolo_conf": args.yolo_conf,
        "yolo_coco_conf": (None if args.skip_coco else yolo_coco_conf),
        "rfdetr_threshold": args.rfdetr_threshold,
        "dino_box_threshold": (None if args.skip_dino else args.dino_box_threshold),
        "dino_text_threshold": (None if args.skip_dino else args.dino_text_threshold),
        "splits_scanned": args.splits,
        "visdrone_to_unified_mapping": VISDRONE_NAMES_FOR_UNIFIED,
        "coco_to_unified_mapping": COCO_NAMES_FOR_UNIFIED,
        "dino_phrases_for_unified": DINO_PHRASES_FOR_UNIFIED,
        "sources": {},
        "notes": (
            "v3: four independent voters (yolo_visdrone, rfdetr_visdrone, "
            "yolo_coco, dino) clustered per image/class by IoU into vote "
            "clusters (see cluster_detections()). vote_count == number of "
            "models that fired on that cluster. cross_reference_"
            "candidates.json holds vote_count>=2 clusters only (some "
            "independent corroboration); cross_reference_full.json holds "
            "every cluster including vote_count==1, keyed per image -- "
            "generate_pseudo_labels.py tiers on vote_count directly "
            "rather than the old agree/yolo_only/rfdetr_only buckets."
        ),
    }
    all_candidates = {}
    all_full_detections = {}

    for source_name, s in sources.items():
        print(f"\n{'=' * 70}\n{source_name} -- missing classes: {s['missing']}\n{'=' * 70}")
        dets_by_model_and_image = {m: dets[m].get(source_name, {}) for m in MODEL_TAGS}
        summary, candidates, full_detections = compare_and_review(
            source_name, s["images"], dets_by_model_and_image,
            args.iou_threshold, review_dir, args.max_review_images)

        print(f"\n  {source_name} vote-count totals:")
        for vc in (1, 2, 3, 4):
            print(f"    {vc}/4 votes: {summary['vote_count_totals'][vc]}")
        print(f"  Per-class:")
        for cls_name, counts in summary["per_class"].items():
            line = "  ".join(f"{vc}/4={counts[vc]}" for vc in (1, 2, 3, 4))
            print(f"    {cls_name:15s} {line}")

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
    print(f"Multi-vote candidates (vote_count>=2): {candidates_path}")
    print(f"Complete per-image record (all vote counts): {full_path}")
    print(f"Review images: {review_dir}/<source>_cross/")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()