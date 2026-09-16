# `generate_pseudo_labels.py` — tiering votes into labels

**File:** `generate_pseudo_labels.py`
**Run:** `python generate_pseudo_labels.py [options]` (dry run by default;
`--apply` to actually write/merge)
**Reads:** `cross_reference_candidates.json` / `cross_reference_full.json`
(or the `_resolved` variants via `--candidates-file`/`--full-file`, see
[resolve-cross-class-conflicts.md](resolve-cross-class-conflicts.md))
**Writes:** `datasets/pseudo_labels/` (auto-accept tier),
`datasets/pseudo_labels_review/` (needs-review tier + rendered images),
`datasets/pseudo_labels_discarded.json`, `datasets/pseudo_labels_summary.json`,
`datasets/pending_review_images.json`; with `--apply`, also merges into the
real dataset label files

## Responsibility

Turns `cross_reference_gaps.py`'s vote-based output into actual YOLO-format
label files for UAVDT/SARD's missing classes, split into an **auto-accept**
tier (written straight to label files, no human involved) and a
**needs-review** tier (queued with rendered images for
`review_labels.py`).

## Three-tier split, by `vote_count`

| Tier | Condition | Behavior |
|---|---|---|
| **AUTO-ACCEPT** | `vote_count >= 3` | Written unconditionally, no confidence floor — three-plus independent models agreeing is strong enough on its own |
| **AUTO-ACCEPT (conditional)** | `vote_count == 2` | Written only if `min(both confidences) >= --min-conf` **and** pairwise IoU `>= --min-iou` |
| **NEEDS REVIEW** | `vote_count == 1` at/above `--min-single-vote-conf` ("single_model_only"); `vote_count == 2` failing the auto-accept gate above `--min-two-vote-review-conf` ("below_threshold"); or an `ALWAYS_REVIEW`-class cluster (default: `SARD:motorcycle` — classes with little/no multi-model corroboration) at/above `--min-always-review-conf` ("always_review_class") | Queued in `pseudo_labels_review_queue.json` with rendered images for `review_labels.py` |
| **DISCARDED** | Anything below the relevant floor above | Recorded (not silently dropped) in `pseudo_labels_discarded.json` with a reason, so floors can be retuned without rerunning the expensive model scan. Pass `0.0` to any floor to queue everything instead of discarding |

Per-class **sensitivity tables** (`two_vote_sensitivity`/
`single_vote_sensitivity` in `pseudo_labels_summary.json`) show pass-count
by threshold for the whole population, regardless of the current floor —
check these before picking `--min-*-conf(-per-class)` values, no rerun
needed to size a cut. Per-class overrides
(`--min-single-vote-conf-per-class`/`--min-two-vote-review-conf-per-class`,
repeatable `SOURCE:class:threshold`) exist because one class can dominate
the queue by orders of magnitude at a very different confidence
distribution than the rest.

## Clean-slate outputs

Every run (dry run or `--apply`) **wipes** `datasets/pseudo_labels/` and
`datasets/pseudo_labels_review/` before writing anything
(`clean_output_dir()`) — both trees are fully reproducible from the
candidate/full JSON plus the current flags, so there's never a good reason
to keep a stale previous run's files around. `--keep-existing-outputs`
skips the wipe for a manual A/B comparison only — note this does **not**
make the run fully additive: the queue/discarded/summary JSON files are
always regenerated from scratch regardless of this flag, so they can end
up describing a different candidate set than whatever old label/render
files remain on disk.

`clean_output_dir()` **refuses (raises)** if ever pointed at
`review_labels.py`'s manual-review tree (`pseudo_labels_reviewed/`,
`review_progress.json`, `review_completed.json`) — this script only ever
**reads** near those (read-only glob/read_text at merge time), never
writes or deletes there.

## `--apply` merge + hold-back

With `--apply`, merges `datasets/pseudo_labels/` (this script's fresh
auto-accept tier) and `datasets/pseudo_labels_reviewed/`
(`review_labels.py`'s output) into the real dataset label files, appending
and skipping duplicate lines (`merge_pseudo_trees_into_dataset()`).

**Any image that still has an undecided review item in EITHER tier has
ALL its boxes (from both trees) held back this run** — not partially
merged — because YOLO training treats an absent box as confirmed
background, and merging only the decided boxes on a partially-reviewed
frame would silently teach the model that the undecided region is
background. `compute_pending_review_images()` computes this set;
`write_pending_review_images()` writes it to
`datasets/pending_review_images.json`, which is exactly the file
`prepare_datasets.py` reads to exclude those images from train/val/test
lists entirely (see
[prepare-datasets.md](prepare-datasets.md#pending-pseudo-label-review-exclusion)).

## Duplicate detection — geometric, not string

`_is_duplicate_line()`/`_box_iou()` compare boxes **geometrically** (same
class, IoU above `DUPLICATE_IOU_THRESHOLD`) rather than as exact-string
matches, when merging into real label files. This is a documented fix — an
earlier version compared lines as exact strings, which missed duplicates
that were the same real object with slightly different box coordinates
from two different merge passes. `dedupe_real_labels.py` (see
[dedupe-real-labels.md](dedupe-real-labels.md)) exists specifically to
clean up real label files that already picked up double-labeled objects
from a run made before this fix.

## `sync_reviewed_backups()`

Keeps `labels_backup_reviewed_<split>/` — a live mirror of the real label
files, maintained for review-tooling purposes — in sync after an `--apply`
run. Uses the same real-label-file discovery glob
(`datasets/<SOURCE>/<split>/labels/`) that `dedupe_real_labels.py` reuses
directly (`import generate_pseudo_labels as gpl`) rather than
reimplementing it.

## CLI

```bash
# Dry run -- report tiering, write nothing:
python generate_pseudo_labels.py

# Apply: write auto-accept labels, merge into the real dataset,
# refresh the reviewed-backup mirror:
python generate_pseudo_labels.py --apply

# Point at resolved (cross-class-merged) input files:
python generate_pseudo_labels.py --apply \
    --candidates-file cross_reference_candidates_resolved.json \
    --full-file cross_reference_full_resolved.json
```

| Flag | Default | Notes |
|---|---|---|
| `--candidates-file` / `--full-file` | `cross_reference_candidates.json` / `cross_reference_full.json` | Swap for the `_resolved` variants after running `resolve_cross_class_conflicts.py` |
| `--min-conf` / `--min-iou` | 0.40 / 0.60 | Conditional 2-vote auto-accept gate |
| `--min-single-vote-conf` | 0.40 | Floor for a 1-vote item to be queued at all (vs. discarded) |
| `--min-always-review-conf` | 0.30 | Floor for `ALWAYS_REVIEW`-class clusters |
| `--min-two-vote-review-conf` | 0.0 | Floor for a failed-gate 2-vote item to be queued (vs. discarded) |
| `--min-single-vote-conf-per-class` / `--min-two-vote-review-conf-per-class` | none | Repeatable `SOURCE:class:threshold` overrides |
| `--always-review` | `SARD:motorcycle` | Repeatable `SOURCE:class` pairs treated as always-needs-review |
| `--sensitivity-thresholds` | multiple | Thresholds reported in the sensitivity tables |
| `--max-review-images` | 300 | Cap on rendered review images |
| `--apply` | off | Actually write/merge; otherwise dry run |
| `--keep-existing-outputs` | off | Skip wiping `pseudo_labels/`/`pseudo_labels_review/` — see caveat above |
| `--sync-backups-only` | off | Just run `sync_reviewed_backups()` without regenerating anything else |
