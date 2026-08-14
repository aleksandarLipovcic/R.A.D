"""
resolve_cross_class_conflicts.py -- Merges cross_reference_gaps.py's
per-image vote clusters ACROSS class boundaries when two clusters
overlap spatially but were assigned different unified classes, so a
single real object doesn't get queued twice under two different class
guesses.

WHY THIS EXISTS:
cluster_detections() in cross_reference_gaps.py clusters same-image
detections into vote clusters PER CLASS FIRST (see its by_cls split) --
correct, because two different real objects of different classes
sitting at the same location is a real thing that clustering shouldn't
silently paper over. But it also means that when two of the four
voters detect what is almost certainly the SAME physical object but
disagree on its exact unified-class label (VisDrone/COCO/DINO taxonomy
edge cases: car vs large_vehicle for a van or pickup, other_vehicle vs
car for a golf cart, etc.), the two detections land in two SEPARATE
class-specific clusters -- each carrying only that one model's vote,
i.e. two vote_count==1 "needs review" items in pseudo_labels_review_
queue.json instead of one better-corroborated item. On an ~8000-image
manual queue that's duplicate review work for what is, physically, one
box.

This script does NOT rerun any of the four detector models and does
NOT touch cross_reference_gaps.py's output files in place. It reads
cross_reference_full.json (which already has every cluster's box +
per-model votes for every image -- that's the ENTIRE input this needs)
and produces a second, clearly-separate set of files with a merged
view: no GPU, no re-scanning the ~8000 images, seconds not hours.

WHAT COUNTS AS THE "SAME OBJECT":
Two clusters (of different classes, same image) are merged if their
representative boxes' IoU >= --iou-threshold (default 0.5, matching
cross_reference_gaps.py's own clustering default -- same "is this
plausibly the same physical box" bar, just applied across the class
boundary that script deliberately doesn't cross). Merging uses a
union-find over ALL of an image's clusters (any class), so a three-way
chain (A-B overlap, B-C overlap, A-C don't) still ends up as one group
-- the same thing cluster_detections() already does within a single
class, just extended across the class boundary.

CAVEAT -- READ BEFORE TRUSTING THIS ON A CLASS PAIR LIKE person/
motorcycle: a person riding a motorcycle is TWO real, distinct objects
whose boxes can legitimately overlap heavily. IoU alone can't tell
"same object, disagreed label" apart from "two different real objects
that happen to overlap." Check class_conflict_report.json's per-
class-pair counts and the rendered review images (below) before
trusting merges on any class pair where that's plausible -- if you see
it happening, exclude that pair with --skip-class-pair. On UAVDT
specifically, the three missing classes are person/motorcycle/
other_vehicle -- exactly the "vulnerable road user" set that
physically co-occurs as riders, so person<->motorcycle and person<->
other_vehicle merges there are especially likely to be real riders,
not label disagreements. Check the numbers before trusting them.

TRANSITIVITY CAP -- merges are strictly PAIRWISE (a merge group is
always exactly 2 clusters), chosen greedily by highest IoU first, each
cluster used in at most one merge. This deliberately does NOT chain
the way cluster_detections() chains within a single class. Chaining
across the class boundary is unsafe here: if motorcycle-A overlaps
person-B, and person-B separately overlaps motorcycle-C (a very
ordinary rider + a second nearby motorcycle), naive transitive
grouping pulls A and C into the same group even though they're the
same class and never directly overlapped -- a same-class merge
smuggled in through an unrelated third object. An earlier version of
this script used union-find (which chains) and this exact failure mode
showed up in a real run as motorcycle<->motorcycle / person<->person /
other_vehicle<->other_vehicle counts in class_conflict_report.json,
despite same-class merges being explicitly disallowed as a direct
rule. Capping every merge at 2 clusters closes that path entirely.

WHICH CLASS WINS:
Within a merged group, the winning class is whichever ORIGINAL cluster
had the single highest confidence among its own votes (i.e. its best-
voting model, not an average) -- same "trust the strongest individual
signal" spirit cross_reference_gaps.py's own rep_model selection
already uses. The losing cluster's class + confidence is kept on the
merged record (alt_classes) purely for audit -- never silently
dropped.

WHY THIS CAN LEGITIMATELY *INCREASE* THE VOTE COUNT:
The merged cluster's "votes" dict is the union of both original
clusters' per-model votes (per model, whichever contributing cluster
had the higher confidence for that model wins; in the near-impossible
case the same model appears in both, its higher-confidence entry
wins). "member_ious" is rebuilt the same way real clusters carry it --
per model, the IoU of that model's contributing box against the
WINNING box (1.0 for models that voted directly on the winning
cluster) -- so generate_pseudo_labels.py's two_vote_pairwise_iou() gate
still works correctly on merged clusters, exactly as it does on
native ones. vote_count is the count of models with a non-null vote in
the union. This can lift a merged pair from 1+1=2 separate "needs
review" singles into one vote_count==2 (or 3/4, in a genuine 3+-way
class disagreement) entry -- two independently-arrived-at boxes IS
real spatial corroboration, even when the two voters disagree on the
fine-grained label.

--conf-margin (default 0.0, i.e. off): if set > 0, a merge is only
performed when the LOSING cluster's confidence is within --conf-margin
of the winner's. A very low-confidence stray label shouldn't get to
"donate" a vote to a real detection just by overlapping it -- if
that's a concern for your data, try --conf-margin 0.3 and compare
class_conflict_report.json's confidence-delta stats before/after.

--skip-class-pair (repeatable, e.g. --skip-class-pair person:motorcycle):
never merge across this specific class pair, regardless of IoU/margin
-- use this for any pair where "two real overlapping objects" is a
live possibility (see CAVEAT above).

OUTPUT (all NEW files -- cross_reference_full.json / _candidates.json /
pseudo_labels* are never modified or deleted by this script):
  datasets/cross_reference_full_resolved.json
      Same shape as cross_reference_full.json ({source: {image_key:
      [cluster, ...]}}), but with cross-class overlapping clusters
      merged per the rule above. Unmerged clusters pass through
      unchanged (no resolved_by key added).
  datasets/cross_reference_candidates_resolved.json
      Recomputed from the resolved full set: every merged-or-original
      cluster with vote_count >= 2, same shape as cross_reference_
      candidates.json ({source: [cluster_with_image, ...]}).
  datasets/class_conflict_report.json
      Per-source/per-class-pair merge counts, confidence deltas
      (winner conf - loser conf), and vote-count-before-vs-after --
      check this before trusting the merged set, same spirit as
      cross_reference_gaps.py's own confidence diagnostic.
  datasets/label_gap_review/<source>_conflict/
      Annotated samples: winning box in green (labeled "WON: cls
      conf"), losing/absorbed class printed below it in red (labeled
      "LOST: cls conf") on the same box -- so you can eyeball whether
      merges look like the same real object before trusting any of
      this. Wiped clean each run via check_label_gaps.py's
      clean_review_subdir() (imported, not duplicated).

TO ACTUALLY USE THE MERGED OUTPUT: generate_pseudo_labels.py reads
cross_reference_candidates.json / cross_reference_full.json by a fixed
name. Point it at the resolved files instead with the two flags added
in the accompanying patch (--candidates-file / --full-file) -- see the
patch note that came with this script for the exact change. Nothing
else in generate_pseudo_labels.py needs to know a merge ever happened,
since the resolved files are in the identical shape it already
expects.

REQUIREMENTS: same environment cross_reference_gaps.py already runs in
(this script imports directly from it and from check_label_gaps.py --
no new packages).

USAGE:
    # Cheap -- no GPU, reads already-computed JSON, seconds to run.
    python resolve_cross_class_conflicts.py

    # Check class_conflict_report.json and the review images first.
    # If a low-confidence stray shouldn't donate a vote to a real
    # detection, or a specific class pair looks like real co-located
    # objects rather than a label disagreement:
    python resolve_cross_class_conflicts.py --conf-margin 0.3 \
        --skip-class-pair person:motorcycle

    # Then point generate_pseudo_labels.py at the resolved files:
    python generate_pseudo_labels.py \
        --candidates-file cross_reference_candidates_resolved.json \
        --full-file cross_reference_full_resolved.json
"""

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

import cv2

import prepare_datasets
from cross_reference_gaps import compute_iou, MODEL_TAGS
from check_label_gaps import clean_review_subdir

WIN_COLOR = (0, 220, 0)     # green -- winning class
LOSE_COLOR = (0, 0, 255)    # red -- absorbed/losing class


def _best_conf(c: dict) -> float:
    confs = [v for v in c["votes"].values() if v is not None]
    return max(confs) if confs else 0.0


def merge_group(clusters: list[dict]) -> dict:
    """clusters: 2+ original clusters (from cross_reference_full.json)
    grouped as the same physical object. Returns one merged cluster in
    the same shape cross_reference_gaps.py's cluster_detections() would
    produce (cls/box/rep_model/votes/vote_count/member_ious), plus
    audit-only fields (resolved_by/alt_classes/original_vote_counts)."""
    ranked = sorted(clusters, key=_best_conf, reverse=True)
    winner = ranked[0]

    # Per model, keep whichever contributing cluster had the higher
    # confidence for that model, and remember which cluster it came
    # from so member_ious can be computed against the WINNING box --
    # generate_pseudo_labels.py's two_vote_pairwise_iou() reads
    # member_ious the same way for merged and native clusters alike.
    best_vote_source: dict[str, tuple] = {}  # model -> (conf, source_cluster)
    for c in clusters:
        for m, v in c["votes"].items():
            if v is None:
                continue
            if m not in best_vote_source or v > best_vote_source[m][0]:
                best_vote_source[m] = (v, c)

    merged_votes = {m: None for m in MODEL_TAGS}
    merged_ious = {m: None for m in MODEL_TAGS}
    for m, (conf, src_cluster) in best_vote_source.items():
        merged_votes[m] = conf
        merged_ious[m] = (1.0 if src_cluster is winner
                           else compute_iou(winner["box"], src_cluster["box"]))

    vote_count = sum(1 for v in merged_votes.values() if v is not None)

    alt_classes = [
        {"cls": c["cls"], "conf": _best_conf(c), "vote_count": c["vote_count"]}
        for c in ranked[1:]
    ]

    return {
        "cls": winner["cls"],
        "box": winner["box"],
        "rep_model": winner.get("rep_model"),
        "votes": merged_votes,
        "vote_count": vote_count,
        "member_ious": merged_ious,
        "resolved_by": "class_conflict_merge",
        "alt_classes": alt_classes,
        "original_vote_counts": [c["vote_count"] for c in ranked],
    }


def resolve_image_clusters(clusters: list[dict], iou_threshold: float,
                             conf_margin: float,
                             skip_pairs: set) -> tuple:
    """Returns (resolved_clusters, merge_records) for one image. Only
    ever merges clusters of DIFFERENT classes -- same-class clusters
    were already merged by cluster_detections() upstream, so two
    same-class entries here are a real "two distinct same-class
    objects near each other" situation, not a conflict to resolve.

    STRICTLY PAIRWISE: candidate (different-class, IoU-passing) pairs
    are collected, sorted by IoU descending, then matched greedily --
    each cluster can be claimed by at most one merge. This caps every
    merge group at exactly 2 clusters. See module docstring's
    TRANSITIVITY CAP note for why chaining (e.g. via union-find) is
    unsafe here: it can silently bridge two same-class clusters
    together through an unrelated third, different-class cluster."""
    if len(clusters) < 2:
        return list(clusters), []

    candidates = []  # (iou, i, j)
    for i in range(len(clusters)):
        for j in range(i + 1, len(clusters)):
            ci, cj = clusters[i], clusters[j]
            if ci["cls"] == cj["cls"]:
                continue
            pair = frozenset((ci["cls"], cj["cls"]))
            if pair in skip_pairs:
                continue
            iou = compute_iou(ci["box"], cj["box"])
            if iou < iou_threshold:
                continue
            if conf_margin > 0:
                lo, hi = sorted([_best_conf(ci), _best_conf(cj)])
                if hi - lo > conf_margin:
                    continue  # loser too far below winner -- don't merge
            candidates.append((iou, i, j))

    candidates.sort(key=lambda t: t[0], reverse=True)
    used = [False] * len(clusters)
    resolved, merge_records = [], []
    for iou, i, j in candidates:
        if used[i] or used[j]:
            continue
        used[i] = used[j] = True
        merged = merge_group([clusters[i], clusters[j]])
        resolved.append(merged)
        merge_records.append(merged)

    for idx, c in enumerate(clusters):
        if not used[idx]:
            resolved.append(c)

    return resolved, merge_records


def parse_skip_pairs(pairs: list) -> set:
    out = set()
    for pair in pairs:
        a, _, b = pair.partition(":")
        if not b:
            raise SystemExit(f"--skip-class-pair expects clsA:clsB, got '{pair}'")
        out.add(frozenset((a, b)))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--iou-threshold", type=float, default=0.5,
                    help="Box IoU bar for treating two different-class "
                         "clusters as the same physical object. Matches "
                         "cross_reference_gaps.py's own clustering "
                         "default.")
    p.add_argument("--conf-margin", type=float, default=0.0,
                    help="If > 0, only merge when the losing cluster's "
                         "confidence is within this margin of the "
                         "winner's. 0.0 (default) = no margin check.")
    p.add_argument("--skip-class-pair", nargs="*", default=[],
                    help="clsA:clsB pairs to NEVER merge across, "
                         "regardless of IoU/margin -- use for pairs "
                         "where two real co-located objects (e.g. "
                         "person:motorcycle) are plausible. Repeatable.")
    p.add_argument("--max-review-images", type=int, default=60)
    args = p.parse_args()

    skip_pairs = parse_skip_pairs(args.skip_class_pair)

    datasets_dir = prepare_datasets.DATASETS_DIR
    full_path = datasets_dir / "cross_reference_full.json"
    if not full_path.exists():
        raise SystemExit(f"{full_path} not found -- run cross_reference_gaps.py first.")

    with open(full_path) as f:
        full_by_source = json.load(f)

    resolved_full = {}
    resolved_candidates = {}
    report = {"iou_threshold": args.iou_threshold, "conf_margin": args.conf_margin,
              "skip_class_pairs": args.skip_class_pair, "sources": {}}
    review_dir = datasets_dir / "label_gap_review"

    for source_name, full_detections in full_by_source.items():
        source_resolved = {}
        all_merges = []
        class_pair_counts = defaultdict(int)
        conf_deltas = []
        vote_count_before_after = []

        for image_key, clusters in full_detections.items():
            resolved, merges = resolve_image_clusters(
                clusters, args.iou_threshold, args.conf_margin, skip_pairs)
            source_resolved[image_key] = resolved
            for m in merges:
                winner_conf = max(v for v in m["votes"].values() if v is not None)
                for alt in m["alt_classes"]:
                    pair_key = "<->".join(sorted([m["cls"], alt["cls"]]))
                    class_pair_counts[pair_key] += 1
                    conf_deltas.append(winner_conf - alt["conf"])
                vote_count_before_after.append(
                    (sum(m["original_vote_counts"]), m["vote_count"]))
                all_merges.append({**m, "image": image_key})

        resolved_full[source_name] = source_resolved
        resolved_candidates[source_name] = [
            {**c, "image": image_key}
            for image_key, clusters in source_resolved.items()
            for c in clusters if c["vote_count"] >= 2
        ]

        report["sources"][source_name] = {
            "total_merges": len(all_merges),
            "class_pair_counts": dict(class_pair_counts),
            "confidence_delta_stats": {
                "mean": statistics.mean(conf_deltas) if conf_deltas else None,
                "median": statistics.median(conf_deltas) if conf_deltas else None,
                "min": min(conf_deltas) if conf_deltas else None,
                "max": max(conf_deltas) if conf_deltas else None,
            },
            "vote_count_sum_before_vs_merged_after_sample": vote_count_before_after[:200],
        }

        print(f"{source_name}: {len(all_merges)} cross-class merges "
              f"{dict(class_pair_counts)}")

        if all_merges:
            out_dir = review_dir / f"{source_name.lower()}_conflict"
            clean_review_subdir(out_dir)
            ranked_merges = sorted(
                all_merges,
                key=lambda m: max(v for v in m["votes"].values() if v is not None),
                reverse=True,
            )[:args.max_review_images]
            print(f"  Saving {len(ranked_merges)} conflict review images to {out_dir}/ ...")
            for i, m in enumerate(ranked_merges):
                img = cv2.imread(m["image"])
                if img is None:
                    continue
                x1, y1, x2, y2 = [int(round(v)) for v in m["box"]]
                cv2.rectangle(img, (x1, y1), (x2, y2), WIN_COLOR, 2)
                winner_conf = max(v for v in m["votes"].values() if v is not None)
                cv2.putText(img, f"WON: {m['cls']} {winner_conf:.2f}", (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WIN_COLOR, 1, cv2.LINE_AA)
                for j, alt in enumerate(m["alt_classes"]):
                    y_txt = min(img.shape[0] - 5, y2 + 18 + 16 * j)
                    cv2.putText(img, f"LOST: {alt['cls']} {alt['conf']:.2f}", (x1, y_txt),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, LOSE_COLOR, 1, cv2.LINE_AA)
                out_path = out_dir / f"{i:03d}_{Path(m['image']).stem}.jpg"
                cv2.imwrite(str(out_path), img)

    full_out = datasets_dir / "cross_reference_full_resolved.json"
    candidates_out = datasets_dir / "cross_reference_candidates_resolved.json"
    report_out = datasets_dir / "class_conflict_report.json"

    with open(full_out, "w") as f:
        json.dump(resolved_full, f, indent=2, default=str)
    with open(candidates_out, "w") as f:
        json.dump(resolved_candidates, f, indent=2, default=str)
    with open(report_out, "w") as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\nResolved full set: {full_out}")
    print(f"Resolved candidates (vote_count>=2): {candidates_out}")
    print(f"Conflict report: {report_out}")
    print(f"Review images: {review_dir}/<source>_conflict/")


if __name__ == "__main__":
    main()