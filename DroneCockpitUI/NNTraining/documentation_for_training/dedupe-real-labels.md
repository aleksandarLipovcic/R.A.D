# `dedupe_real_labels.py` — one-off duplicate-label cleanup

**File:** `dedupe_real_labels.py`
**Run:** `python dedupe_real_labels.py --datasets-dir datasets [--apply]`
**Reuses:** `generate_pseudo_labels.py`'s own `_parse_yolo_line()`,
`_is_duplicate_line()`, and `DUPLICATE_IOU_THRESHOLD` directly (`import
generate_pseudo_labels as gpl`)

## Why this exists

A historical cleanup script for real dataset label files that already
picked up **double-labeled objects** from a `--apply` run of
`generate_pseudo_labels.py` made *before* its merge-dedup logic was fixed
to compare boxes geometrically (IoU) instead of as exact strings (see
[generate-pseudo-labels.md](generate-pseudo-labels.md#duplicate-detection--geometric-not-string)).
Any real label file merged before that fix could contain the same
physical object labeled twice with slightly different box coordinates.

## What it does

For every real label file under `datasets/<SOURCE>/<split>/labels/` (the
same discovery glob `generate_pseudo_labels.py`'s
`sync_reviewed_backups()` uses — `find_real_label_dirs()` here reimplements
that same logic locally), walks its lines in order and drops any line
that's a **geometric duplicate** (same class, IoU ≥ `--iou-threshold`) of
one already kept earlier in that same file — keeping the **first**
occurrence, dropping the rest. Class mismatches or non-overlapping boxes
are always kept, no matter how close together.

## What it deliberately does not touch

- **`labels_backup_original_<split>/`** — `prepare_datasets.py`'s one-time
  pre-remap snapshot. Not read, not written.
- **`labels_backup_reviewed_<split>/`** — after a successful `--apply`,
  this is just a live mirror of the real files, so it would already
  contain the same duplicates. Rather than deduping it separately, this
  script re-syncs the mirror automatically once it's done, using the exact
  same `sync_reviewed_backups()` the main pipeline calls after its own
  `--apply` — so nothing else needs to be run afterward.

## Report before trusting

Every line considered a duplicate — dry run or `--apply` — is written to
`--report-file` (default `dedupe_report.json`), keyed by file path, so the
exact before/after can be eyeballed before trusting the cut, or the
threshold can be re-run against a different `--iou-threshold` without
re-scanning everything by hand.

## CLI

```bash
# Dry run -- report what WOULD change, write nothing:
python dedupe_real_labels.py --datasets-dir datasets

# Apply for real, then refresh labels_backup_reviewed_<split>/ to match:
python dedupe_real_labels.py --datasets-dir datasets --apply

# Different threshold, still just reporting:
python dedupe_real_labels.py --datasets-dir datasets --iou-threshold 0.5
```

| Flag | Default | Notes |
|---|---|---|
| `--datasets-dir` | `datasets` | |
| `--apply` | off | Actually rewrite the real label files; default is dry-run/report-only |
| `--iou-threshold` | `generate_pseudo_labels.DUPLICATE_IOU_THRESHOLD` | Same-class IoU at/above which two boxes count as one duplicated object — matches the main pipeline's own merge threshold by default |
| `--report-file` | `dedupe_report.json` | Full before/after detail, written on both dry run and `--apply` |

This is documented as a **one-off** cleanup for a specific historical
incident, not a script meant to be run routinely — routine duplicate
prevention now lives in `generate_pseudo_labels.py`'s fixed geometric merge
check itself.
