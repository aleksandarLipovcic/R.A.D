# `finetune_and_resolve_queue.py` — auto-resolving the review backlog

**File:** `finetune_and_resolve_queue.py`
**Run:** `python finetune_and_resolve_queue.py [options]` — dry-run
first, then for real
**Reads:** `pseudo_labels_review_queue.json`, `review_progress.json`,
`review_completed.json` (the same files `review_labels.py` maintains)
**Writes:** new "accept" decisions into `review_progress.json`, in the
exact schema `review_labels.py` already expects

## Responsibility

Domain-adapts the two VisDrone-tuned voters (`yolo_visdrone`,
`rfdetr_visdrone`) on the images already hand-corrected in
`review_labels.py`, then uses the fine-tuned pair as **two new,
independent voters** to auto-decide as much of the pending review-queue
backlog as the calibration data actually supports.

**Why a separate script**: this never touches `review_labels.py` or its
GUI — it only reads/writes the same JSON files, in the schema
`review_labels.py` already expects, so reopening the normal review tool
afterward just sees fewer pending images. Nothing about the tool itself
changes.

## Critical design choices

1. **Held-out calibration split.** Completed images are split into a
   fine-tune set (used to train) and a calibration set (never trained on,
   held out purely to measure the fine-tuned models' real precision before
   trusting them on the much larger pending backlog). Testing a threshold
   on images the model was fine-tuned on would give an optimistically
   biased reading — ordinary train/val discipline, applied to a
   170-image dataset instead of a normal-sized one. Default 80/20 split,
   overridable via `--val-frac`.
2. **Never touches** (a) any image already in `review_completed.json` —
   excluded from the backlog-scan phase entirely; (b) any queue item that
   already has a non-"pending" decision anywhere — this script only ever
   **writes** a decision for an item currently pending or with no saved
   decision at all. Manual boxes and class overrides are read (to build
   fine-tuning labels / calibration ground truth) but never modified.
3. **Accept-only by default, not accept/reject.** Per
   `review_labels.py`'s `accepted_items()`, "pending" and "reject" have
   **identical** effect on the training-label output — only "accept" is
   ever written to `pseudo_labels_reviewed/`. So there's no correctness
   upside to writing "reject" for what doesn't clear the calibration bar
   here, only a downside: it permanently forecloses that item from being
   picked up by a future, better-calibrated pass (once more images have
   been hand-reviewed and this script is re-run). Leaving it pending keeps
   it recoverable. `--also-reject-remainder` exists for administratively
   closing those images out anyway (e.g. to declutter the review tool's
   default view) — off by default for the reason above.
4. **Per-class threshold, not one global number.** Calibration searches,
   independently per unified class, for the **lowest** confidence
   threshold that still clears `--target-precision` (default 0.90) with at
   least `--min-calibration-n` supporting examples on the held-out
   calibration split. A class without enough calibration examples to trust
   a threshold gets **no** auto-accept rule this run — everything in that
   class stays pending rather than guessing.
5. **Nothing is final without a human looking at it first.** The
   calibration report (per-class threshold/precision/n) prints before
   anything touches `review_progress.json`, and the script stops for a y/n
   confirmation unless `--yes` is passed. `--dry-run` fine-tunes,
   calibrates, and reports, then stops there — no writes at all.
6. **Re-runnable.** As the hand-completed set grows (by hand, or via this
   script's own printed QA-worthy spot-check suggestions), re-run from
   scratch — more calibration data means tighter, more trustworthy
   thresholds, which clears more of the backlog safely each time. No state
   is carried between runs beyond what's already in
   `review_progress.json`/`review_completed.json`.

## Reuses existing pipeline code directly, not reimplementations

Explicitly documented as an environment assumption to verify if this
script ever breaks:

- Reuses `prepare_datasets.DATASETS_DIR`, `class_map.UNIFIED_CLASSES`,
  several `generate_pseudo_labels.py` helpers
  (`image_path_to_label_path`, `get_image_dims`, `box_to_yolo_line`,
  `mirror_under_pseudo_root`), and several `review_labels.py` helpers
  (`item_key`, `load_json_dict`, `save_json`, `load_completed`,
  `save_completed`, `backup_progress_files`, `compute_completed_set`, plus
  the progress/completed/reviewed filename constants) — as-is, the same
  way `cross_reference_gaps.py` and an internal `audit_sync.py` tool
  already do.
- Reuses `cross_reference_gaps.py`'s `run_yolo`/`run_rfdetr`/`compute_iou`/
  `MODEL_DISPLAY` directly for the actual scanning passes, rather than
  reimplementing YOLO/RF-DETR inference here. At import time, this script
  monkeypatches two new entries into `xrg.MODEL_DISPLAY` (`yolo_finetuned`,
  `rfdetr_finetuned`) so those reused functions don't `KeyError` when
  given the new fine-tuned model tags.
- YOLO fine-tuning uses the standard `model.train(data=<yaml>, ...)` API.
- RF-DETR fine-tuning uses `model.train(dataset_dir=<coco-dir>,
  epochs=..., batch_size=..., output_dir=...)` — flagged as **the part
  most likely to need a tweak** across `rfdetr` library versions. The
  source recommends checking
  `python -c "import rfdetr, inspect; print(inspect.signature(rfdetr.RFDETRSmall.train))"`
  against `finetune_rfdetr()` before trusting it, since `rfdetr` (and
  `transformers`, per `cross_reference_gaps.py`'s own DINO note) has
  changed kwarg names across versions before.

## CLI

```bash
# 1. Fine-tune + calibrate + REPORT ONLY. Nothing written yet.
python finetune_and_resolve_queue.py --dry-run

# 2. Once the calibration report looks sane, the real pass
#    (still asks for a y/n confirmation before writing):
python finetune_and_resolve_queue.py

# Skip the confirmation prompt (e.g. for a scheduled re-run):
python finetune_and_resolve_queue.py --yes

# Reuse checkpoints from a previous run instead of re-training:
python finetune_and_resolve_queue.py \
    --yolo-weights datasets/specialist_finetune/yolo_data/runs/specialist_yolo/weights/best.pt \
    --rfdetr-weights datasets/specialist_finetune/rfdetr_data/runs/specialist_rfdetr/checkpoint_best_total.pth
```

| Flag | Default | Notes |
|---|---|---|
| `--val-frac` | 0.2 | Held-out calibration split fraction |
| `--seed` | 42 | |
| `--min-completed` | 100 | Minimum hand-completed images required before this script will run at all |
| `--yolo-epochs` / `--yolo-batch` / `--yolo-freeze` | 40 / 8 / 10 | Fine-tune settings for the YOLO specialist; `--yolo-freeze` freezes early backbone layers, appropriate for a small fine-tune set |
| `--rfdetr-epochs` / `--rfdetr-batch` | 30 / 4 | Fine-tune settings for the RF-DETR specialist |
| `--scan-batch` | 8 | Batch size when the fine-tuned pair scans the backlog |
| `--device` | 0 | |
| `--target-precision` | 0.90 | Per-class calibration target |
| `--min-calibration-n` | 10 | Minimum supporting examples before a class gets an auto-accept rule at all |
| `--yolo-weights` / `--rfdetr-weights` | none | Reuse checkpoints from a previous run instead of re-training |
| `--also-reject-remainder` | off | Administratively close out everything that didn't clear calibration (see point 3 above) |
| `--dry-run` | off | Fine-tune + calibrate + report, no writes |
| `--yes` | off | Skip the confirmation prompt |
