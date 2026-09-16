# `prepare_datasets.py` — multi-dataset prep & merge

**File:** `prepare_datasets.py`
**Run by:** `train.py` automatically before every run (cheap — each step
checks whether it's already done); can also run standalone —
`python prepare_datasets.py` — to inspect what's currently included.
**Reads:** `class_map.py`'s taxonomy and remap tables; `datasets/` on disk
**Writes:** `datasets/unified.yaml` (the merged training config),
`datasets/per_source_val.json` (per-dataset val manifest for
`train.py`'s `evaluate_per_source()`)

## What it does, per source

1. **VisDrone2019-DET** (`prepare_visdrone`) — ensured downloaded, remapped
   from its 10 native classes into the unified taxonomy.
2. **xView** (`prepare_xview`) — triggered only if `datasets/xView/`
   exists. **Currently inactive** — see
   [class-map.md](class-map.md#design-decisions-worth-knowing).
3. **UAVDT** and **SARD** (`prepare_uavdt`/`prepare_sard`, both routing
   through `prepare_roboflow_dataset`) — Roboflow-style folders
   (`data.yaml` + `{train,valid,test}/{images,labels}`), genuinely
   drone-native so Mosaic-mixing with VisDrone is fine.
   - UAVDT ships `train/` only → auto-split by video sequence
     (`_auto_split_by_sequence`/`_infer_sequence_key`), so a val split
     never leaks frames from the same continuous sequence as a train
     frame.
   - SARD ships a real train/valid/test split, used as-is.
4. **`datasets/external/`** (`prepare_external`) — scans for extra
   manually-added datasets in the same Roboflow-style format; each needs a
   `class_map.EXTERNAL_REMAPS` entry to be picked up.
5. **Sparse-class oversampling** (`_oversample_sparse_classes`) — see
   below.
6. **Per-class instance counting** (`_count_instances_per_class`) — see
   below.

### Manual Roboflow downloads

UAVDT and SARD are gated behind a free Roboflow account with no public
direct-download API, so `setup.py` can't fetch them automatically:

| Dataset | Source | Notes |
|---|---|---|
| UAVDT | https://universe.roboflow.com/kfupm-v0syf/uavdt-4g4uv | — |
| SARD | https://universe.roboflow.com/animesh-shastry/sard_yolo | Pick **"v1 Original"** — other versions are pre-augmented |

Export format **"YOLOv8"**, raw/1x, extract directly into
`datasets/UAVDT/` or `datasets/SARD/` (no extra wrapper folder).

## Sparse-class oversampling (`_oversample_sparse_classes`)

Duplicates **image paths**, not labels, for under-represented classes into
`datasets/oversample_train.txt`, added as an extra `train:` entry in
`unified.yaml`. `val` is never touched by this — oversampling only affects
what the model trains on, not how it's scored. Important caveat spelled
out in the docstring: Ultralytics' `CopyPaste` augmentation needs
segmentation polygons this bbox-only data doesn't have, so `--copy-paste`
is currently a no-op regardless of the value passed to `train.py` —
oversampling plus mixup are what's actually protecting sparse classes.
(`verify_pipeline_assumptions.py` check B empirically confirms this
inertness — see [verify-pipeline-assumptions.md](verify-pipeline-assumptions.md).)

## Per-class instance counting (`_count_instances_per_class`)

Total labeled *instances* per class across train, computed **before**
oversampling — described as the real sparsity signal, since a class can
appear in many images but still trail in total boxes if density-per-image
differs from other classes. The doc comment specifically warns not to
trust the oversampling default (`motorcycle`/`other_vehicle`) once
UAVDT/SARD are folded in without checking this count — `"person"`, the
project's priority class, isn't in that default and its real sparsity
should be checked against this count directly.

## Pending pseudo-label review exclusion

`generate_pseudo_labels.py` writes `datasets/pending_review_images.json`
every run (dry-run or `--apply`) — every image with at least one undecided
review item. Every `prepare_*()` function here drops matching images from
every train/val/test list it builds (Roboflow-style sources only —
VisDrone/xView never go through pseudo-labeling, so they're unaffected).

This matters most for **val**: an image with incomplete ground truth would
score a correct detection on the missing class as a false positive,
corrupting per-class mAP. See `_load_pending_review_images()`,
`_build_train_val_entry()`, and `_auto_split_by_sequence()` for where the
exclusion is actually applied. Path comparisons are normcased
(`os.path.normcase`, via `_norm_path()`) specifically so a Windows
separator/case mismatch can't let a pending image slip through the filter
silently. No `pending_review_images.json` on disk (pseudo-labeling never
run yet) means an empty exclusion set — i.e. unchanged pre-existing
behavior, not an error.

## Taxonomy-change re-remap safety net

`_needs_remap(root, signature)` compares the current
`class_map.taxonomy_signature()` against one written to disk
(`_write_signature`) the last time this dataset was remapped. A mismatch
(taxonomy or any remap table edited) triggers `_restore_from_backup()` —
restoring the dataset's **untouched original labels** from a one-time
backup (`_ensure_backup`/`labels_backup_original_<split>/`) and re-applying
the current remap from scratch, rather than trying to patch already-remapped
label files in place.

> **This safety net does not cover pseudo-labels.** The backup/restore
> cycle only knows about each dataset's original label files — anything
> merged in later by `generate_pseudo_labels.py --apply` is a different
> layer entirely. Check that script's own pre-restore warning (see
> [generate-pseudo-labels.md](generate-pseudo-labels.md)) before assuming
> a taxonomy edit is free with pseudo-labels already merged in.

## Standalone inspection

```bash
python prepare_datasets.py
```

Prints a summary of what's currently included (which sources were found,
per-class instance counts, oversampling applied) without needing to launch
a training run — useful for sanity-checking the merged dataset state on
its own.
