# `cross_reference_gaps.py` — four-model vote clustering

**File:** `cross_reference_gaps.py`
**Run:** `python cross_reference_gaps.py [options]`
**Consumes:** the images `check_label_gaps.py` flagged
**Produces:** `cross_reference_candidates.json`,
`cross_reference_full.json` — consumed downstream by
`generate_pseudo_labels.py` and `resolve_cross_class_conflicts.py`

## Responsibility

Re-scans the "missing-class" images `check_label_gaps.py` flagged with
**four independent detector architectures**, so pseudo-label candidates
for UAVDT/SARD's label gaps carry a **vote count** instead of a single
agree/disagree bit from one model.

## Version history (why four models, not two)

The module docstring documents a real design evolution:

- **V1** used a COCO-general detector + an open-vocabulary detector —
  domain-mismatch noise turned out to dominate.
- **V2** switched to a domain-matched pair (two VisDrone-tuned detectors)
  — fixed the domain-mismatch noise, but a domain-matched pair can still
  both miss the same thing for the same reason: e.g. a SARD frame where
  two VisDrone-tuned detectors both undercall a cluttered top-down car
  cluster because VisDrone itself doesn't have much of that exact scene
  composition in its own training distribution.
- **V3** (current) is back to **four** models, vote-counted rather than
  pairwise agree/disagree, specifically to break that shared-blind-spot
  problem:

| # | Model | Role |
|---|---|---|
| 1 | `yolo_visdrone` — `dronefreak/visdrone-yolov26l` | VisDrone-domain specialist (unchanged from V2) |
| 2 | `rfdetr_visdrone` — `dronefreak/visdrone-rfdetr-small` | Second, architecturally different VisDrone-domain specialist (unchanged from V2) |
| 3 | `yolo_coco` — stock `yolo26l.pt` | Broader, more generic training distribution — catches scenes the two VisDrone specialists both miss. Re-added from V1 |
| 4 | `dino` — `IDEA-Research/grounding-dino-tiny` | Open-vocabulary, text-prompted. Re-added from V1. **Slow** — V1's original log recorded ~73 min on UAVDT, ~67 min on SARD, the expensive part of a full run. Use `--skip-dino`/`--limit` while iterating, drop them for the real pass |

## Vote-based tiering

Detections from all four models are clustered per image/class by IoU
(`cluster_detections()`) into "vote clusters" — one cluster per physical
object, carrying which of the four models found it and at what
confidence. This script itself does **no** accept/reject decision — it
only measures, clusters, and outputs the full vote record.
`generate_pseudo_labels.py` (downstream) tiers on the resulting
`vote_count`:

- **1/4 votes** → always needs review — no second opinion at all.
- **2/4 votes** → review unless it clears a confidence floor (two
  independent detections is real signal, but not automatically enough on
  its own).
- **3–4/4 votes** → auto-accept unconditionally — that level of
  independent, architecturally-diverse consensus doesn't need a
  confidence floor on top of it.

See [generate-pseudo-labels.md](generate-pseudo-labels.md) for the actual
tiering logic that consumes this output.

## Phase-loaded model execution (VRAM management)

All four checkpoints loaded simultaneously would stack on top of each
other in VRAM on a 6GB laptop card — exactly the kind of WDDM
shared-memory spillover `train.py` guards against (see
[train.md](train.md#wddm--vram-spillover-guards)). So this script does
**not** load all four up front. Instead, for each model in turn: load it,
scan **every** source's images, unload it, move to the next model. No GPU
holds more than one model's weights at a time.

## CLI

```bash
python cross_reference_gaps.py
python cross_reference_gaps.py --skip-dino --limit 50   # fast iteration
python cross_reference_gaps.py --splits train val       # full real pass
```

| Flag | Default | Notes |
|---|---|---|
| `--yolo-repo` | `dronefreak/visdrone-yolov26l` | |
| `--rfdetr-size` | `small` | |
| `--yolo-coco-weights` | `yolo26l.pt` | |
| `--dino-repo` | `IDEA-Research/grounding-dino-tiny` | |
| `--skip-coco` / `--skip-dino` | off | Skip one of the four voters — `--skip-dino` is the main iteration-speed lever, since DINO is the slow one |
| `--limit` | none | Cap images per source, for fast iteration |
| `--yolo-conf` / `--yolo-coco-conf` / `--rfdetr-threshold` / `--dino-box-threshold` / `--dino-text-threshold` | 0.25 / (defaults to `--yolo-conf`) / 0.25 / 0.25 / 0.20 | Per-model confidence floors before a detection is even considered as a vote |
| `--rfdetr-nms-iou` / `--dino-nms-iou` | model defaults | Per-model NMS override |
| `--iou-threshold` | 0.5 | Same-class clustering threshold — the "is this plausibly the same physical box" bar shared across this script's clustering and `resolve_cross_class_conflicts.py`'s cross-class merge |
| `--device` | 0 | |
| `--yolo-batch` / `--rfdetr-batch` / `--dino-batch` | 16 / 4 / 4 | Per-model batch sizes — DINO and RF-DETR run smaller batches than YOLO |
| `--splits` | `train val` | |
| `--max-review-images` | 60 | Annotated sample images saved for eyeballing |
| `--seed` | 42 | |

## Cost expectations

Running all four models over the full train+val split of both UAVDT and
SARD is the expensive part of this pipeline — budget for it accordingly
(DINO alone was ~70 minutes per source in the original run that motivated
`--skip-dino`). Iterate with `--skip-dino --limit N` while tuning
thresholds, and only run the full four-model pass once thresholds look
right on the smaller sample.
