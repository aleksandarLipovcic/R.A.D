# `resolve_cross_class_conflicts.py` — merging cross-class duplicate clusters

**File:** `resolve_cross_class_conflicts.py`
**Run:** `python resolve_cross_class_conflicts.py [options]` — cheap, no
GPU, reads already-computed JSON, seconds to run
**Reads:** `cross_reference_full.json` (from `cross_reference_gaps.py`)
**Writes:** three **new** files — never modifies or deletes
`cross_reference_full.json`/`_candidates.json` or any `pseudo_labels*`
output

## Why this exists

`cross_reference_gaps.py`'s `cluster_detections()` clusters same-image
detections into vote clusters **per class first** — correct, because two
different real objects of different classes at the same location is a real
thing clustering shouldn't paper over. But that also means when two of the
four voters detect what is almost certainly the **same physical object**
but disagree on its exact unified-class label (taxonomy edge cases: car
vs. large_vehicle for a van or pickup, other_vehicle vs. car for a golf
cart, etc.), the two detections land in two **separate** class-specific
clusters — each carrying only one model's vote, i.e. two `vote_count==1`
"needs review" items instead of one better-corroborated item. On an
~8,000-image manual review queue, that's duplicate review work for what
is, physically, one box.

This script does **not** rerun any of the four detector models and does
**not** touch `cross_reference_gaps.py`'s output files in place. It reads
`cross_reference_full.json` (which already has every cluster's box + votes
for every image — the entire input this needs) and produces a second,
clearly separate merged view.

## What counts as "the same object"

Two clusters of *different* classes, same image, are merged if their
representative boxes' IoU ≥ `--iou-threshold` (default 0.5 — matching
`cross_reference_gaps.py`'s own clustering default: the same "is this
plausibly the same physical box" bar, just applied across the class
boundary that script deliberately doesn't cross).

## Transitivity cap: pairwise only, never chained

Merges are strictly **pairwise** — a merge group is always exactly 2
clusters, chosen greedily by highest IoU first, each cluster used in at
most one merge. This deliberately does **not** chain the way
`cluster_detections()` chains within a single class.

This is a fixed bug, not a design choice made from scratch: an earlier
version used union-find (which does chain), and a real failure mode showed
up in production — if motorcycle-A overlaps person-B, and person-B
separately overlaps motorcycle-C (an ordinary rider plus a second nearby
motorcycle), naive transitive grouping pulls A and C into the same group
even though they're the same class and never directly overlapped — a
same-class merge smuggled in through an unrelated third object, despite
same-class merges being an explicitly disallowed direct rule. This showed
up as motorcycle↔motorcycle / person↔person / other_vehicle↔other_vehicle
counts in `class_conflict_report.json` in a real run. Capping every merge
at exactly 2 clusters closes that path entirely.

## Which class wins

The winning class is whichever **original cluster** had the single highest
confidence among its own votes (its best-voting model, not an average) —
the same "trust the strongest individual signal" spirit
`cross_reference_gaps.py`'s own representative-model selection already
uses. The losing cluster's class + confidence is kept on the merged record
(`alt_classes`) purely for audit — never silently dropped.

## Why a merge can legitimately *increase* the vote count

The merged cluster's `votes` dict is the **union** of both original
clusters' per-model votes (per model, whichever contributing cluster had
the higher confidence for that model wins). `member_ious` is rebuilt the
same way real clusters carry it — per model, the IoU of that model's
contributing box against the winning box (1.0 for models that voted
directly on the winning cluster) — so `generate_pseudo_labels.py`'s
`two_vote_pairwise_iou()` gate still works correctly on merged clusters,
identically to native ones. This can lift a merged pair from `1+1=2`
separate "needs review" singles into one `vote_count==2` (or 3/4, in a
genuine 3+-way class disagreement) entry — two independently-arrived-at
boxes **is** real spatial corroboration, even when the two voters disagree
on the fine-grained label.

## Safety valves

- **`--conf-margin`** (default 0.0, off): if set > 0, a merge only happens
  when the *losing* cluster's confidence is within this margin of the
  winner's — a very low-confidence stray label shouldn't get to "donate" a
  vote to a real detection just by overlapping it.
- **`--skip-class-pair`** (repeatable, e.g. `person:motorcycle`): never
  merge across a specific class pair, regardless of IoU/margin.

## Caveat — read before trusting this on rider-type class pairs

A person riding a motorcycle is **two real, distinct objects** whose boxes
can legitimately overlap heavily. IoU alone can't tell "same object,
disagreed label" apart from "two different real objects that happen to
overlap." Check `class_conflict_report.json`'s per-class-pair counts and
the rendered review images before trusting merges on any class pair where
this is plausible. On UAVDT specifically, the three missing classes are
person/motorcycle/other_vehicle — exactly the "vulnerable road user" set
that physically co-occurs as riders, so person↔motorcycle and
person↔other_vehicle merges there are especially likely to be real riders,
not label disagreements.

## Output files

| File | Contents |
|---|---|
| `datasets/cross_reference_full_resolved.json` | Same shape as `cross_reference_full.json`, with cross-class overlapping clusters merged. Unmerged clusters pass through unchanged (no `resolved_by` key added) |
| `datasets/cross_reference_candidates_resolved.json` | Recomputed from the resolved full set: every merged-or-original cluster with `vote_count >= 2` |
| `datasets/class_conflict_report.json` | Per-source/per-class-pair merge counts, confidence deltas (winner conf − loser conf), and vote-count-before-vs-after — check this before trusting the merged set |
| `datasets/label_gap_review/<source>_conflict/` | Annotated samples — winning box in green ("WON: cls conf"), losing/absorbed class in red ("LOST: cls conf") on the same box, so merges can be eyeballed before trusting them. Wiped clean each run via `check_label_gaps.py`'s `clean_review_subdir()` (imported, not duplicated) |

## Using the merged output

`generate_pseudo_labels.py` reads `cross_reference_candidates.json`/
`cross_reference_full.json` by a fixed name — point it at the resolved
files instead via `--candidates-file`/`--full-file`:

```bash
python resolve_cross_class_conflicts.py
# inspect class_conflict_report.json and the review images first

python generate_pseudo_labels.py \
    --candidates-file cross_reference_candidates_resolved.json \
    --full-file cross_reference_full_resolved.json
```

Nothing else in `generate_pseudo_labels.py` needs to know a merge ever
happened — the resolved files are in the identical shape it already
expects.

## CLI

```bash
python resolve_cross_class_conflicts.py
python resolve_cross_class_conflicts.py --conf-margin 0.3 \
    --skip-class-pair person:motorcycle
```
