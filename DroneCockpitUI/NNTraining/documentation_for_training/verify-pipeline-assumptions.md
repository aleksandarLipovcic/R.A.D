# `verify_pipeline_assumptions.py` — empirical fact-checking

**File:** `verify_pipeline_assumptions.py`
**Run:** `python verify_pipeline_assumptions.py` — from the same directory
as `train.py`/`prepare_datasets.py`/`class_map.py`/`mosaic_guard.py`, since
it imports them directly, exactly as `train.py` does.

## Responsibility

Checks the claims made in `train.py`'s and `prepare_datasets.py`'s own
docstrings **against the actual prepared data and actually installed
library versions**, instead of trusting the comments. Every check prints
one of three verdicts: `CONFIRMED`, `CONTRADICTED`, or `INCONCLUSIVE`
(via the shared `verdict()` helper).

This exists because several of this pipeline's docstrings make specific,
checkable claims (e.g. "`--copy-paste` is a no-op on this data",
"8/9 degradation transforms build") that could silently become false after
a dataset addition or a library upgrade, with nothing else in the pipeline
positioned to notice.

## The six checks

| # | Check | What it does |
|---|---|---|
| **A** | `check_class_balance()` | Recomputes real per-class instance counts on the actual merged train set (via `prepare_datasets.py`'s own `_count_instances_per_class()`) and checks whether the *true* sparsest class is covered by `--oversample-classes`. This is the check that catches the exact failure mode `prepare_datasets.py`'s docstring warns about: `person` (the priority class) not being in the default oversample list even if it turns out to be the sparsest once UAVDT/SARD are folded in. |
| **B** | `check_copy_paste_is_really_inert()` | Scans actual label files in the merged train set for segmentation polygon data (more than 5 columns per line = polygon, not a plain bbox). Confirms or contradicts `prepare_datasets.py`'s claim that `--copy-paste` is currently a no-op because this bbox-only data has no polygons for it to work with. |
| **C** | `check_degradation_transform_builds()` | Actually calls `train.py`'s `build_degradation_transform()` against the installed albumentations version and counts how many of the 9 planned pieces built — rather than trusting a comment about what "should" build. |
| **D** | `check_coarse_dropout_is_literal_pixels()` | Builds a synthetic image, runs `CoarseDropout` on it, and measures the actual zeroed region in pixels — confirming whether `hole_height_range`/`hole_width_range` really are literal pixel counts (not a fraction of image size) on the installed version, since this determines whether the "at most 120 zeroed pixels" claim in `train.py`'s comment is still true. |
| **E** | `check_mosaic_filler_bias()` | Drives `InstanceCappedMosaic`'s **real** `get_indexes()` code path directly (not a reimplementation of its logic) against the actual prepared dataset, checking whether it over-selects sparse-instance sources (e.g. SARD) as "filler" when paired with a dense anchor image — the mechanism a code review flagged as a possible cause of degraded small-object/person detection. |
| **F** | `check_real_per_class_map(checkpoint)` | Optional, only runs with `--checkpoint`. Loads a real trained checkpoint and calls `train.py`'s own `report_per_class_map()` against it — the actual per-class mAP number the whole exercise exists to protect, computed from a real `model.val()` pass, not estimated. |

## CLI

```bash
python verify_pipeline_assumptions.py
python verify_pipeline_assumptions.py --checkpoint runs/detect/yolo26s_960/weights/best.pt
python verify_pipeline_assumptions.py --max-mosaic-instances 800
```

`--oversample-classes` (default `motorcycle,other_vehicle`) should be kept
in sync with whatever is actually passed to `train.py`, so check A
evaluates the real current configuration rather than a stale default.

## When to run this

Per [setup.md](setup.md#6-installed-version-fingerprint-print_installed_versions),
specifically after a `setup.py` re-run that upgraded any library version —
`train.py`'s degradation-augment and mosaic-guard monkeypatches both target
non-public Ultralytics/albumentations internals that can change shape
between releases without warning. Checks B and C are the ones most
directly relevant to confirming those patches still apply cleanly.
