# `check_label_gaps.py` — quantifying the missing-annotation risk

**File:** `check_label_gaps.py`
**Run:** `python check_label_gaps.py [options]`
**Reads:** `class_map.py`'s `VISDRONE_REMAP`/`EXTERNAL_REMAPS` (to derive
which classes each source structurally never labels)
**Writes:** `datasets/label_gap_report.json`,
`datasets/label_gap_review/<source>/` (annotated sample images)

## Responsibility

The first stage of the label-gap pipeline (see [ML_PIPELINE.md](../ML_PIPELINE.md)).
UAVDT only ever labels car/bus/truck (→ car/large_vehicle in the unified
taxonomy) — `person`, `motorcycle`, and `other_vehicle` are **never**
labeled there, even when visible in a frame. SARD only ever labels
`person` — every vehicle class is never labeled. Because YOLO's
classification loss treats every unlabeled region as confirmed background,
a real-but-unlabeled instance of a "missing" class in these sources
actively **punishes** the model for correctly detecting something it
learned from VisDrone.

This script does **not** fix anything or touch any label file. It runs a
YOLO checkpoint over each source's images, looking **only** for the
classes that source structurally never labels, and reports how often it
finds them — so the next decision (pseudo-label the gaps, mask the loss,
or do nothing because the risk is negligible) is backed by actual counts
and actual images, not a guess.

## Which classes are "missing" — derived, not hardcoded

`missing_classes_for(remap_table)` derives the missing-class set directly
from `VISDRONE_REMAP`/`EXTERNAL_REMAPS` in `class_map.py` — never
hardcoded in this script. This means the gap analysis stays correct
automatically if the taxonomy or remap tables change, and extends to any
future dataset added the same way (e.g. the sketched-out AU-AIR entry in
`class_map.py`).

## Scanning model — a fixed cross-domain-mismatch bug

Documented as a dated fix (2026-08-11): the default scanning model was
swapped from a generic COCO-pretrained checkpoint to
`dronefreak/visdrone-yolov26l` — a VisDrone-domain-finetuned checkpoint
from the Hugging Face Hub, the **same** model
`cross_reference_gaps.py`'s YOLO voter now uses. The reasoning: a full run
with the old COCO-pretrained + open-vocab-DINO pairing (see
[cross-reference-gaps.md](cross-reference-gaps.md)) showed the open-vocab
side's mismatch counts sitting right on the confidence floor — classic
domain-mismatch noise, not signal. But the COCO-pretrained YOLO side of
*this* script had never been cross-checked against a second model the way
the open-vocab side was, so it was never actually proven innocent of the
same failure mode: "COCO-pretrained on mostly ground-level photos"
scanning a UAV dataset of small, top-down, aerial objects is exactly the
same domain gap. Swapping to the VisDrone-finetuned checkpoint here removes
that risk, and means this script's counts and `cross_reference_gaps.py`'s
`yolo_only`/`agree` counts are now produced by the **literal same model on
the literal same images** — so if the two reports ever disagree, that's
informative rather than a self-inflicted false signal.

This also meant replacing a hand-maintained `COCO_NAMES_FOR_UNIFIED`
lookup table with `VISDRONE_NAMES_FOR_UNIFIED`
(`_build_visdrone_names_for_unified()`), derived by **inverting**
`class_map.py`'s own `VISDRONE_REMAP` — guaranteed correct against the
actual taxonomy this project trains on, rather than a hand-maintained COCO
analog table that could drift out of sync. `COCO_NAMES_FOR_UNIFIED` and
`resolve_coco_indices()` are kept in the file for reference/fallback, not
deleted.

## Output

- **Console summary** per source/class/confidence-threshold.
- **`datasets/label_gap_report.json`** — full stats, machine-readable, the
  input `cross_reference_gaps.py` doesn't directly consume but that a
  human (or `--min-...` threshold tuning) can inspect.
- **`datasets/label_gap_review/<source>/`** — annotated sample images (a
  mix of highest-confidence hits and random hits, via `save_review_images()`)
  for manual eyeballing before trusting any of this.

## CLI

```bash
python check_label_gaps.py
python check_label_gaps.py --model dronefreak/visdrone-yolov26l --conf-thresholds 0.25 0.5
```

| Flag | Default | Notes |
|---|---|---|
| `--model` | `dronefreak/visdrone-yolov26l` | The VisDrone-domain checkpoint (see above) |
| `--limit` | none | Cap the number of images scanned per source, for a quick test |
| `--conf-thresholds` | multiple | Report hit counts at several confidence thresholds at once, so a floor can be picked without a rerun |
| `--device` / `--batch` | 0 / 16 | |
| `--splits` | `train val` | Which dataset splits to scan |
| `--max-review-images` | 40 | Cap on annotated sample images saved per source |
| `--seed` | 42 | For reproducible random-sample selection in the review images |
