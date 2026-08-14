"""
finetune_and_resolve_queue.py -- domain-adapts yolo_visdrone and
rfdetr_visdrone on your 170 hand-corrected images, then uses the
fine-tuned pair as two NEW independent voters to auto-decide as much of
the pending review-queue backlog (images not yet in
review_completed.json) as the calibration data actually supports.

WHY A SEPARATE SCRIPT: this never touches review_labels.py or its GUI --
it only reads pseudo_labels_review_queue.json + review_progress.json +
review_completed.json, and writes new "accept" decisions into
review_progress.json in the exact schema review_labels.py already
expects, so re-opening the normal review tool afterward just sees fewer
pending images -- nothing about the tool itself changes.

=======================================================================
CRITICAL DESIGN CHOICES -- READ BEFORE RUNNING
=======================================================================

1. HELD-OUT CALIBRATION SPLIT. The completed images are split into a
   FINE-TUNE set (used to train) and a CALIBRATION set (never trained
   on, held out purely to measure the fine-tuned models' real precision
   before trusting them on the ~8,700 pending images). Testing a
   threshold on images the model was fine-tuned on gives an
   optimistically biased reading -- ordinary train/val discipline,
   just applied to a 170-image dataset instead of a normal-sized one.
   Default 80/20 split, overridable via --val-frac.

2. NEVER TOUCHES: (a) any image already in review_completed.json --
   excluded from the backlog-scan phase entirely; (b) any queue item
   that already has a non-"pending" decision anywhere -- this script
   only ever WRITES a decision for an item that is currently pending or
   has no saved decision at all. Manual boxes and class overrides are
   read (to build fine-tuning labels / calibration ground truth) but
   never modified.

3. ACCEPT-ONLY BY DEFAULT, NOT ACCEPT/REJECT. Per review_labels.py's
   accepted_items(), "pending" and "reject" have IDENTICAL effect on
   the training-label output -- only "accept" is ever written to
   pseudo_labels_reviewed/. So there is no correctness upside to
   writing "reject" for what doesn't clear the bar here, only a
   downside: it permanently forecloses that item from being picked up
   by a future, better-calibrated pass (e.g. once you've hand-reviewed
   more images and re-run this script). Leaving it pending keeps it
   recoverable. Pass --also-reject-remainder if you specifically want
   those images administratively closed out anyway (e.g. to declutter
   the review tool's default view) -- off by default for the reason
   above.

4. PER-CLASS THRESHOLD, NOT ONE GLOBAL NUMBER. Calibration searches,
   independently per unified class, for the LOWEST confidence threshold
   that still clears --target-precision (default 0.90) with at least
   --min-calibration-n supporting examples on the held-out calibration
   split. A class without enough calibration examples to trust a
   threshold gets NO auto-accept rule this run -- everything in that
   class stays pending rather than guessing.

5. NOTHING IS FINAL WITHOUT YOU LOOKING AT IT FIRST. The calibration
   report (per-class threshold / precision / n) prints before anything
   touches review_progress.json, and the script stops for a y/n
   confirmation unless --yes is passed. Use --dry-run to fine-tune +
   calibrate + report and stop there, no writes at all.

6. RE-RUNNABLE. As your completed set grows (by hand, or via this
   script's own --qa-worthy spot checks -- see the printed report),
   re-run this from scratch -- more calibration data => tighter,
   more trustworthy thresholds => more of the backlog clears safely
   each time. There's no state carried between runs other than what's
   already in review_progress.json / review_completed.json.

=======================================================================
ASSUMED ENVIRONMENT -- check these against your actual setup
=======================================================================
  - prepare_datasets.DATASETS_DIR, class_map.UNIFIED_CLASSES,
    generate_pseudo_labels.{image_path_to_label_path, get_image_dims,
    box_to_yolo_line, mirror_under_pseudo_root}, review_labels.{
    item_key, load_json_dict, save_json, load_completed, save_completed,
    backup_progress_files, compute_completed_set, PROGRESS_FILENAME,
    COMPLETED_FILENAME, REVIEWED_DIRNAME} -- reused as-is, same as
    cross_reference_gaps.py / audit_sync.py already do.
  - cross_reference_gaps.{run_yolo, run_rfdetr, compute_iou,
    MODEL_DISPLAY} -- reused directly for the actual scanning passes,
    rather than reimplementing YOLO/RF-DETR inference here. This
    script monkeypatches two new entries into MODEL_DISPLAY at import
    time (see PATCH block below) since run_yolo/run_rfdetr look the
    display name up by tag.
  - ultralytics YOLO fine-tuning via model.train(data=<yaml>, ...) --
    standard API.
  - rfdetr fine-tuning via model.train(dataset_dir=<coco-dir>,
    epochs=..., batch_size=..., output_dir=...) -- THIS IS THE PART
    MOST LIKELY TO NEED A TWEAK. Check
        python -c "import rfdetr, inspect; print(inspect.signature(rfdetr.RFDETRSmall.train))"
    against finetune_rfdetr() below before trusting it blind -- rfdetr
    (and transformers, per cross_reference_gaps.py's own DINO note)
    has changed kwarg names across versions before.

USAGE:
    # 1. Fine-tune + calibrate + REPORT ONLY. Nothing written yet.
    python finetune_and_resolve_queue.py --dry-run

    # 2. Once the calibration report looks sane, the real pass
    #    (still asks for a y/n confirmation before writing):
    python finetune_and_resolve_queue.py

    # Skip the confirmation prompt (e.g. for a scheduled re-run):
    python finetune_and_resolve_queue.py --yes

    # Reuse checkpoints from a previous run instead of re-training
    # (useful for iterating on --target-precision / --min-calibration-n
    # without paying the fine-tune cost again):
    python finetune_and_resolve_queue.py \
        --yolo-weights datasets/specialist_finetune/yolo_data/runs/specialist_yolo/weights/best.pt \
        --rfdetr-weights datasets/specialist_finetune/rfdetr_data/runs/specialist_rfdetr/checkpoint_best_total.pth
"""

import argparse
import json
import random
import shutil
import time
from pathlib import Path

from huggingface_hub import hf_hub_download
from ultralytics import YOLO

import prepare_datasets
from class_map import UNIFIED_CLASSES
import generate_pseudo_labels as gpl
import review_labels as rl
import cross_reference_gaps as xrg

try:
    import rfdetr as rfdetr_pkg
except ImportError:
    rfdetr_pkg = None

YOLO_BASE_REPO = "dronefreak/visdrone-yolov26l"
RFDETR_BASE_REPO = "dronefreak/visdrone-rfdetr-small"
RFDETR_CLASS_NAME = "RFDETRSmall"

FINETUNE_ROOT_NAME = "specialist_finetune"  # under datasets_dir
RUN_REPORT_NAME = "specialist_finetune_report.json"

TAG_YOLO_FT = "yolo_finetuned"
TAG_RFDETR_FT = "rfdetr_finetuned"

# PATCH: run_yolo()/run_rfdetr() in cross_reference_gaps.py look up a
# display name for whichever model_tag they're given (used only for
# console printing) -- add entries for our two new fine-tuned models so
# reusing those functions as-is doesn't KeyError.
xrg.MODEL_DISPLAY[TAG_YOLO_FT] = "YOLO-VisDrone (fine-tuned)"
xrg.MODEL_DISPLAY[TAG_RFDETR_FT] = "RF-DETR-VisDrone (fine-tuned)"


# ---------------------------------------------------------------------
# Ground-truth extraction from already-decided images (same semantics
# as ReviewApp._accepted_items_for_image / _update_completed_list in
# review_labels.py, but standalone -- no ReviewApp instance needed).
# ---------------------------------------------------------------------

def accepted_items_for_image(image_key: str, items: list, progress: dict) -> list[dict]:
    """Final ground-truth boxes for one image: accepted queue items
    (honoring any class override) + accepted manual boxes. This is what
    training labels AND fine-tuning labels are built from."""
    overrides = progress.get(f"__override__{image_key}", {})
    manual = progress.get(f"__manual__{image_key}", [])
    out = []
    for it in items:
        k = rl.item_key(it)
        saved = progress.get(k, {})
        if saved.get("decision") == "accept":
            cls = overrides.get(k, it["cls"])
            box = saved.get("box", it["box"])
            out.append({"cls": cls, "box": box})
    out.extend({"cls": mb["cls"], "box": mb["box"]} for mb in manual
               if mb.get("decision", "accepted") == "accepted")
    return out


def decided_items_for_calibration(image_key: str, items: list, progress: dict) -> list[dict]:
    """EVERY queue item on this image that has a real decision (not
    pending) -- both accepts and rejects/deletes count as calibration
    ground truth (was this a real object or not)."""
    overrides = progress.get(f"__override__{image_key}", {})
    out = []
    for it in items:
        k = rl.item_key(it)
        saved = progress.get(k, {})
        decision = saved.get("decision", "pending")
        if decision == "pending":
            continue
        cls = overrides.get(k, it["cls"])
        box = saved.get("box", it["box"])
        out.append({"cls": cls, "box": box, "accept": decision == "accept"})
    return out


def split_completed_images(completed_images: set, val_frac: float, seed: int):
    imgs = sorted(completed_images)
    rnd = random.Random(seed)
    rnd.shuffle(imgs)
    n_val = max(1, int(round(len(imgs) * val_frac)))
    val = set(imgs[:n_val])
    train = set(imgs[n_val:])
    return train, val


# ---------------------------------------------------------------------
# Dataset export -- YOLO format (ultralytics) and COCO format (rfdetr)
# ---------------------------------------------------------------------

def export_yolo_dataset(image_keys, by_image, progress, out_dir: Path, split_name: str) -> int:
    img_dir = out_dir / "images" / split_name
    lbl_dir = out_dir / "labels" / split_name
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    cls_to_idx = {c: i for i, c in enumerate(UNIFIED_CLASSES)}
    n_boxes = 0
    for image_key in image_keys:
        items = by_image.get(image_key, [])
        accepted = accepted_items_for_image(image_key, items, progress)
        src = Path(image_key)
        dst_img = img_dir / src.name
        if not dst_img.exists():
            shutil.copy2(src, dst_img)
        w, h = gpl.get_image_dims(image_key)
        lines = []
        for a in accepted:
            if a["cls"] not in cls_to_idx:
                continue
            x1, y1, x2, y2 = a["box"]
            xc, yc = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
            bw, bh = (x2 - x1) / w, (y2 - y1) / h
            lines.append(f"{cls_to_idx[a['cls']]} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
        (lbl_dir / (src.stem + ".txt")).write_text("\n".join(lines) + ("\n" if lines else ""))
        n_boxes += len(lines)
    return n_boxes


def write_yolo_data_yaml(out_dir: Path) -> Path:
    yaml_path = out_dir / "data.yaml"
    yaml_path.write_text(
        f"path: {out_dir.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"nc: {len(UNIFIED_CLASSES)}\n"
        f"names: {list(UNIFIED_CLASSES)}\n"
    )
    return yaml_path


def export_coco_dataset(image_keys, by_image, progress, out_dir: Path, split_name: str) -> int:
    """rfdetr expects a COCO-format directory: <out_dir>/<split_name>/
    with images alongside a single _annotations.coco.json (Roboflow-
    style layout)."""
    split_dir = out_dir / split_name
    split_dir.mkdir(parents=True, exist_ok=True)
    categories = [{"id": i + 1, "name": c, "supercategory": "none"}
                  for i, c in enumerate(UNIFIED_CLASSES)]
    cls_to_id = {c["name"]: c["id"] for c in categories}
    images_json, annotations_json = [], []
    ann_id = 1
    for img_id, image_key in enumerate(image_keys, start=1):
        items = by_image.get(image_key, [])
        accepted = accepted_items_for_image(image_key, items, progress)
        src = Path(image_key)
        dst = split_dir / src.name
        if not dst.exists():
            shutil.copy2(src, dst)
        w, h = gpl.get_image_dims(image_key)
        images_json.append({"id": img_id, "file_name": src.name, "width": w, "height": h})
        for a in accepted:
            if a["cls"] not in cls_to_id:
                continue
            x1, y1, x2, y2 = a["box"]
            annotations_json.append({
                "id": ann_id, "image_id": img_id, "category_id": cls_to_id[a["cls"]],
                "bbox": [x1, y1, x2 - x1, y2 - y1],
                "area": max(0.0, x2 - x1) * max(0.0, y2 - y1), "iscrowd": 0,
            })
            ann_id += 1
    (split_dir / "_annotations.coco.json").write_text(
        json.dumps({"images": images_json, "annotations": annotations_json,
                    "categories": categories}))
    return len(annotations_json)


# ---------------------------------------------------------------------
# Fine-tuning
# ---------------------------------------------------------------------

def finetune_yolo(data_yaml: Path, epochs: int, batch: int, device, freeze: int,
                   run_name: str) -> Path:
    print(f"\n[yolo] downloading base checkpoint {YOLO_BASE_REPO} ...")
    base_weights = hf_hub_download(repo_id=YOLO_BASE_REPO, filename="best.pt")
    model = YOLO(base_weights)
    print(f"[yolo] base checkpoint classes: {sorted(model.names.values())} "
          f"-- detection head will be reinitialized for unified classes "
          f"{list(UNIFIED_CLASSES)} via transfer learning (standard "
          f"ultralytics behavior when data.yaml's nc differs from the "
          f"checkpoint's own).")
    results = model.train(data=str(data_yaml), epochs=epochs, batch=batch,
                           device=device, freeze=freeze,
                           patience=max(5, epochs // 3),
                           project=str(data_yaml.parent / "runs"), name=run_name,
                           verbose=True)
    best = Path(results.save_dir) / "weights" / "best.pt"
    print(f"[yolo] fine-tuned weights: {best}")
    return best


def finetune_rfdetr(coco_dir: Path, epochs: int, batch: int, run_name: str) -> Path:
    if rfdetr_pkg is None:
        raise SystemExit("rfdetr is not installed -- "
                          "pip install rfdetr --break-system-packages")
    print(f"\n[rfdetr] downloading base checkpoint {RFDETR_BASE_REPO} ...")
    base_weights = hf_hub_download(repo_id=RFDETR_BASE_REPO,
                                    filename="checkpoint_best_total.pth")
    rfdetr_class = getattr(rfdetr_pkg, RFDETR_CLASS_NAME)
    model = rfdetr_class(pretrain_weights=base_weights)
    print(f"[rfdetr] base checkpoint classes: {sorted(model.class_names)} -- "
          f"fine-tuning on unified classes {list(UNIFIED_CLASSES)}.")
    output_dir = coco_dir.parent / "runs" / run_name
    # NOTE: verify this signature against your installed rfdetr version
    # -- see the module docstring's "ASSUMED ENVIRONMENT" section.
    model.train(dataset_dir=str(coco_dir), epochs=epochs, batch_size=batch,
                output_dir=str(output_dir))
    ckpt = output_dir / "checkpoint_best_total.pth"
    if not ckpt.exists():
        candidates = sorted(output_dir.glob("checkpoint_best*.pth"))
        if not candidates:
            raise SystemExit(f"[rfdetr] no best checkpoint found under {output_dir} -- "
                              f"check the training run's actual output layout and "
                              f"adjust finetune_rfdetr() / --rfdetr-weights.")
        ckpt = candidates[0]
    print(f"[rfdetr] fine-tuned weights: {ckpt}")
    return ckpt


# ---------------------------------------------------------------------
# Scanning with the fine-tuned pair (reuses cross_reference_gaps.py's
# own run_yolo/run_rfdetr rather than reimplementing inference)
# ---------------------------------------------------------------------

def run_specialists(image_paths, all_classes, yolo_model, rfdetr_model, device, batch, tag):
    identity_reverse = {c.lower(): c for c in UNIFIED_CLASSES}
    identity_names = {c: [c] for c in UNIFIED_CLASSES}
    yolo_dets = xrg.run_yolo(yolo_model, TAG_YOLO_FT, tag, image_paths, all_classes,
                              0.05, device, batch, identity_reverse, identity_names,
                              f"ft_{tag}_yolo")
    rfdetr_dets = xrg.run_rfdetr(rfdetr_model, tag, image_paths, all_classes,
                                  0.05, batch, identity_reverse, None)
    # Normalize keys (Path separator quirks) so lookups by our own
    # image_key strings reliably hit, regardless of exactly how each
    # library reports back the path it scanned.
    yolo_dets = {str(Path(k)): v for k, v in yolo_dets.items()}
    rfdetr_dets = {str(Path(k)): v for k, v in rfdetr_dets.items()}
    return yolo_dets, rfdetr_dets


def _best_match_conf(box, cls, dets, iou_thr):
    best = None
    for d in dets:
        if d["cls"] != cls:
            continue
        iou = xrg.compute_iou(box, d["box"])
        if iou >= iou_thr and (best is None or d["conf"] > best):
            best = d["conf"]
    return best


def combined_score(row) -> float:
    """Both fine-tuned models agreeing (IoU-matched, same class) is
    treated conservatively -- min() of the two, not max() -- so one
    lucky high-confidence single model can't carry a decision on its
    own when only one specialist actually fired."""
    y, r = row.get("yolo_ft_conf"), row.get("rfdetr_ft_conf")
    if y is not None and r is not None:
        return min(y, r)
    if y is not None:
        return y
    if r is not None:
        return r
    return 0.0


def calibration_rows(image_keys, by_image, progress, yolo_dets, rfdetr_dets, iou_thr=0.5):
    rows = []
    for image_key in image_keys:
        items = by_image.get(image_key, [])
        gt = decided_items_for_calibration(image_key, items, progress)
        y_dets = yolo_dets.get(str(Path(image_key)), [])
        r_dets = rfdetr_dets.get(str(Path(image_key)), [])
        for g in gt:
            y_conf = _best_match_conf(g["box"], g["cls"], y_dets, iou_thr)
            r_conf = _best_match_conf(g["box"], g["cls"], r_dets, iou_thr)
            rows.append({"cls": g["cls"], "yolo_ft_conf": y_conf,
                         "rfdetr_ft_conf": r_conf, "accept": g["accept"]})
    return rows


def calibrate_thresholds(rows, target_precision: float, min_n: int):
    by_cls = {}
    for r in rows:
        by_cls.setdefault(r["cls"], []).append(r)
    thresholds = {}
    for cls, items in by_cls.items():
        scored = [(combined_score(r), r["accept"]) for r in items]
        best = None
        for T in [round(x * 0.05, 2) for x in range(1, 20)]:
            sub = [a for s, a in scored if s >= T]
            if len(sub) < min_n:
                continue
            prec = sum(sub) / len(sub)
            if prec >= target_precision:
                best = (T, prec, len(sub))
                break
        thresholds[cls] = best  # None = not enough safe signal for this class
    return thresholds


def collect_pending_rows(image_keys, by_image, progress, completed_set, yolo_dets, rfdetr_dets,
                          iou_thr=0.5):
    rows = []
    for image_key in image_keys:
        if image_key in completed_set:
            continue
        items = by_image.get(image_key, [])
        y_dets = yolo_dets.get(str(Path(image_key)), [])
        r_dets = rfdetr_dets.get(str(Path(image_key)), [])
        for it in items:
            k = rl.item_key(it)
            entry = progress.get(k, {})
            if entry.get("decision", "pending") != "pending":
                continue
            cls = it["cls"]
            box = entry.get("box", it["box"])
            y_conf = _best_match_conf(box, cls, y_dets, iou_thr)
            r_conf = _best_match_conf(box, cls, r_dets, iou_thr)
            rows.append({"key": k, "image": image_key, "cls": cls, "box": box,
                         "yolo_ft_conf": y_conf, "rfdetr_ft_conf": r_conf})
    return rows


def apply_to_backlog(pending_rows, thresholds):
    accepted, still_pending = [], []
    for row in pending_rows:
        rule = thresholds.get(row["cls"])
        if rule is None or combined_score(row) < rule[0]:
            still_pending.append(row)
        else:
            accepted.append(row)
    return accepted, still_pending


# ---------------------------------------------------------------------
# Writing decisions back into review_progress.json / output labels
# ---------------------------------------------------------------------

def write_accepted_decisions(accepted_rows, by_image, progress):
    touched = set()
    for row in accepted_rows:
        k, image_key = row["key"], row["image"]
        it = next(x for x in by_image[image_key] if rl.item_key(x) == k)
        existing = progress.get(k, {})
        entry = {
            "decision": "accept", "image": image_key, "cls": row["cls"],
            "box": row["box"], "orig_box": list(it["_orig_box"]),
            "auto_finetuned_specialist": True,
            "auto_yolo_ft_conf": row["yolo_ft_conf"],
            "auto_rfdetr_ft_conf": row["rfdetr_ft_conf"],
        }
        if "synced_children" in existing:
            entry["synced_children"] = existing["synced_children"]
        progress[k] = entry
        touched.add(image_key)
    return touched


def write_rejected_decisions(rows, by_image, progress):
    touched = set()
    for row in rows:
        k, image_key = row["key"], row["image"]
        it = next(x for x in by_image[image_key] if rl.item_key(x) == k)
        existing = progress.get(k, {})
        entry = {
            "decision": "reject", "image": image_key, "cls": row["cls"],
            "box": row["box"], "orig_box": list(it["_orig_box"]),
            "auto_finetuned_specialist_reject": True,
        }
        if "synced_children" in existing:
            entry["synced_children"] = existing["synced_children"]
        progress[k] = entry
        touched.add(image_key)
    return touched


def write_reviewed_labels_for_image(image_key, by_image, progress, reviewed_root):
    items = by_image.get(image_key, [])
    accepted = accepted_items_for_image(image_key, items, progress)
    real_label_path = gpl.image_path_to_label_path(Path(image_key))
    source_name = items[0]["source"] if items else "unknown"
    reviewed_label_path = gpl.mirror_under_pseudo_root(real_label_path, source_name, reviewed_root)
    if not accepted:
        if reviewed_label_path.exists():
            reviewed_label_path.unlink()
        return
    w, h = gpl.get_image_dims(image_key)
    lines = [gpl.box_to_yolo_line(a["cls"], a["box"], w, h) for a in accepted]
    reviewed_label_path.parent.mkdir(parents=True, exist_ok=True)
    reviewed_label_path.write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--val-frac", type=float, default=0.2,
                    help="Fraction of completed images held out for calibration "
                         "(never fine-tuned on).")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--min-completed", type=int, default=100,
                    help="Refuse to run below this many completed images -- "
                         "fine-tuning + calibration on fewer is unlikely to be "
                         "trustworthy. Lower this if you really want to proceed anyway.")
    p.add_argument("--yolo-epochs", type=int, default=40)
    p.add_argument("--yolo-batch", type=int, default=8)
    p.add_argument("--yolo-freeze", type=int, default=10,
                    help="Freeze this many leading layers during fine-tune -- "
                         "helps avoid catastrophic forgetting on a small dataset.")
    p.add_argument("--rfdetr-epochs", type=int, default=30)
    p.add_argument("--rfdetr-batch", type=int, default=4)
    p.add_argument("--scan-batch", type=int, default=8)
    p.add_argument("--device", default=0)
    p.add_argument("--target-precision", type=float, default=0.90)
    p.add_argument("--min-calibration-n", type=int, default=10)
    p.add_argument("--yolo-weights", default=None,
                    help="Skip fine-tuning, use this checkpoint directly.")
    p.add_argument("--rfdetr-weights", default=None,
                    help="Skip fine-tuning, use this checkpoint directly.")
    p.add_argument("--also-reject-remainder", action="store_true",
                    help="Also write decision='reject' for pending items that "
                         "didn't clear the threshold, closing those images out "
                         "administratively. Off by default -- see module "
                         "docstring point #3 (reject and pending have identical "
                         "effect on training output; reject just forecloses "
                         "reconsideration by a future pass).")
    p.add_argument("--dry-run", action="store_true",
                    help="Fine-tune + calibrate + report only -- never touches "
                         "review_progress.json.")
    p.add_argument("--yes", action="store_true",
                    help="Skip the confirmation prompt before writing decisions.")
    return p.parse_args()


def main():
    args = parse_args()
    datasets_dir = prepare_datasets.DATASETS_DIR
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    progress_path = datasets_dir / rl.PROGRESS_FILENAME
    completed_path = datasets_dir / rl.COMPLETED_FILENAME
    reviewed_root = datasets_dir / rl.REVIEWED_DIRNAME
    finetune_root = datasets_dir / FINETUNE_ROOT_NAME

    with open(queue_path) as f:
        all_items = json.load(f)
    for it in all_items:
        it["_orig_box"] = list(it["box"])
    by_image = {}
    for it in all_items:
        by_image.setdefault(it["image"], []).append(it)

    progress = rl.load_json_dict(progress_path)
    completed_set = rl.load_completed(completed_path)

    backup_dir = rl.backup_progress_files(datasets_dir, progress_path, completed_path)
    if backup_dir:
        print(f"[backup] snapshotted progress/completed -> {backup_dir}")

    if len(completed_set) < args.min_completed:
        raise SystemExit(
            f"Only {len(completed_set)} completed images -- need at least "
            f"{args.min_completed} to fine-tune + calibrate responsibly. "
            f"Lower --min-completed if you want to proceed anyway.")

    train_imgs, cal_imgs = split_completed_images(completed_set, args.val_frac, args.seed)
    print(f"[split] {len(train_imgs)} fine-tune images, {len(cal_imgs)} held-out "
          f"calibration images (never trained on).")

    # ---- Phase 1: export + fine-tune (or reuse provided weights) ----
    if args.yolo_weights:
        yolo_weights = Path(args.yolo_weights)
        print(f"[yolo] reusing provided weights: {yolo_weights}")
    else:
        yolo_dir = finetune_root / "yolo_data"
        n_tr = export_yolo_dataset(train_imgs, by_image, progress, yolo_dir, "train")
        n_va = export_yolo_dataset(cal_imgs, by_image, progress, yolo_dir, "val")
        print(f"[yolo] exported {n_tr} train boxes, {n_va} val boxes.")
        data_yaml = write_yolo_data_yaml(yolo_dir)
        yolo_weights = finetune_yolo(data_yaml, args.yolo_epochs, args.yolo_batch,
                                      args.device, args.yolo_freeze, "specialist_yolo")

    if args.rfdetr_weights:
        rfdetr_weights = Path(args.rfdetr_weights)
        print(f"[rfdetr] reusing provided weights: {rfdetr_weights}")
    else:
        rfdetr_dir = finetune_root / "rfdetr_data"
        n_tr = export_coco_dataset(train_imgs, by_image, progress, rfdetr_dir, "train")
        n_va = export_coco_dataset(cal_imgs, by_image, progress, rfdetr_dir, "valid")
        print(f"[rfdetr] exported {n_tr} train boxes, {n_va} val boxes.")
        rfdetr_weights = finetune_rfdetr(rfdetr_dir, args.rfdetr_epochs, args.rfdetr_batch,
                                          "specialist_rfdetr")

    # ---- Phase 2: load fine-tuned models ----
    yolo_model = YOLO(str(yolo_weights))
    rfdetr_class = getattr(rfdetr_pkg, RFDETR_CLASS_NAME)
    rfdetr_model = rfdetr_class(pretrain_weights=str(rfdetr_weights))
    all_classes = list(UNIFIED_CLASSES)

    # ---- Phase 3: calibrate on the held-out split ----
    cal_paths = [Path(k) for k in cal_imgs]
    print(f"\n[calibrate] scanning {len(cal_paths)} held-out images...")
    y_cal, r_cal = run_specialists(cal_paths, all_classes, yolo_model, rfdetr_model,
                                    args.device, args.scan_batch, "calibration")
    cal_rows = calibration_rows(cal_imgs, by_image, progress, y_cal, r_cal)
    thresholds = calibrate_thresholds(cal_rows, args.target_precision, args.min_calibration_n)

    print("\n[calibrate] per-class thresholds (None = not enough safe signal, stays manual):")
    print(f"  {'class':15s} {'threshold':>10s} {'precision':>10s} {'n':>5s}")
    for cls in UNIFIED_CLASSES:
        rule = thresholds.get(cls)
        if rule is None:
            print(f"  {cls:15s} {'--':>10s} {'--':>10s} {'--':>5s}")
        else:
            print(f"  {cls:15s} {rule[0]:>10.2f} {rule[1]:>10.3f} {rule[2]:>5d}")

    if args.dry_run:
        print("\n[dry-run] stopping before touching the pending backlog / review_progress.json.")
        return

    if not args.yes:
        resp = input("\nProceed to scan the pending backlog and write decisions "
                      "using these thresholds? [y/N] ")
        if resp.strip().lower() != "y":
            print("Aborted -- nothing written.")
            return

    # ---- Phase 4: scan the actual backlog ----
    pending_images = [k for k in by_image if k not in completed_set]
    pending_paths = [Path(k) for k in pending_images]
    print(f"\n[backlog] scanning {len(pending_paths)} pending images...")
    y_dets, r_dets = run_specialists(pending_paths, all_classes, yolo_model, rfdetr_model,
                                      args.device, args.scan_batch, "backlog")

    pending_rows = collect_pending_rows(pending_images, by_image, progress, completed_set,
                                         y_dets, r_dets)
    accepted_rows, still_pending_rows = apply_to_backlog(pending_rows, thresholds)
    print(f"\n[backlog] {len(accepted_rows)} item(s) auto-accepted, "
          f"{len(still_pending_rows)} remain pending for manual review.")

    touched = write_accepted_decisions(accepted_rows, by_image, progress)
    if args.also_reject_remainder:
        touched |= write_rejected_decisions(still_pending_rows, by_image, progress)
        print(f"[backlog] --also-reject-remainder: wrote 'reject' for "
              f"{len(still_pending_rows)} item(s) that didn't clear the bar.")

    for image_key in touched:
        write_reviewed_labels_for_image(image_key, by_image, progress, reviewed_root)
    rl.save_json(progress_path, progress)

    new_completed = rl.compute_completed_set(by_image, progress)
    newly_closed = new_completed - completed_set
    rl.save_completed(completed_path, new_completed)

    report = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "completed_images_used": len(completed_set),
        "finetune_images": len(train_imgs), "calibration_images": len(cal_imgs),
        "thresholds": {c: (None if thresholds.get(c) is None else
                            {"T": thresholds[c][0], "precision": thresholds[c][1],
                             "n": thresholds[c][2]}) for c in UNIFIED_CLASSES},
        "backlog_items_scanned": len(pending_rows),
        "auto_accepted": len(accepted_rows),
        "left_pending": len(still_pending_rows),
        "also_rejected_remainder": args.also_reject_remainder,
        "images_newly_completed": len(newly_closed),
    }
    (datasets_dir / RUN_REPORT_NAME).write_text(json.dumps(report, indent=2, default=str))

    print(f"\n[done] {len(newly_closed)} image(s) newly moved to review_completed.json "
          f"(now {len(new_completed)} total). review_progress.json updated. "
          f"Run report: {datasets_dir / RUN_REPORT_NAME}. "
          f"Pre-run backup preserved at {backup_dir}.")
    print("\nRecommend: spot-check a random sample of the newly auto-accepted items "
          "via review_labels.py --qa before trusting this at full scale.")


if __name__ == "__main__":
    main()