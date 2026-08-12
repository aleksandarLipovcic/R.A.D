"""
generate_pseudo_labels.py -- Turns cross_reference_gaps.py's VOTE-BASED
output into actual YOLO-format pseudo-label files for the missing
classes in UAVDT/SARD, split into an AUTO-ACCEPT tier (written straight
to label files) and a NEEDS-REVIEW tier (queued with rendered images).

=======================================================================
V3 REWRITE (2026-08-12): tiers on vote_count (how many of the four
cross_reference_gaps.py models agreed) instead of the old two-model
agree/yolo_only/rfdetr_only bucket shape.
=======================================================================

=======================================================================
V4 (2026-08-12): DISCARD gate for uncorroborated low-confidence hits.
=======================================================================
V3 unconditionally queued every vote_count==1 cluster for human review,
regardless of confidence. In practice this is the overwhelming majority
of the queue (a single UAVDT run put 165,909 of ~202,000 total clusters
at 1/4 votes) and cross_reference_gaps.py's own confidence diagnostic
showed most of them sitting right on the 0.25 scan floor (means mostly
0.30-0.45) -- exactly the domain-mismatch-noise signature the project's
other scripts already call out, not real missed detections. Reviewing
all of that by hand isn't tractable and isn't buying anything: a single
model firing near the floor with zero corroboration from three
architecturally different detectors is weak evidence, and this class
was never labeled in these sources to begin with, so discarding it is
a no-op on the dataset, not a loss.

Two new confidence floors split the old single-model-only / always-
review-class queue into an explicit DISCARD bucket vs. still-queued:

  --min-single-vote-conf (default 0.40): vote_count==1 clusters for a
      normal (non-always-review) class below this confidence are
      DISCARDED instead of queued. At/above it, still queued as before
      (reason: "single_model_only") -- one model at real confidence
      with no corroboration is still worth a human glance, just not
      everything down to 0.25.

  --min-always-review-conf (default 0.30, deliberately lower): ALWAYS_
      REVIEW classes (e.g. SARD motorcycle) have NO other evidence tier
      to fall back on -- their multi-vote counts are often zero, so
      single-model hits are the only signal that exists at all for that
      class/source pair. The floor here exists only to drop the
      literal-noise tail sitting at the scan floor, not to compete with
      --min-single-vote-conf's bar.

Both floors default > 0 so old callers get a smaller, more honest queue
by default; pass 0.0 for either to fully restore V3 behavior (queue
everything, discard nothing) if you'd rather eyeball the unfiltered
set once and tune from there.

DISCARDED items are counted per class in pseudo_labels_summary.json
(discarded_low_conf) and, for auditability, the full discarded list is
written to datasets/pseudo_labels_discarded.json (same shape as the
review queue) -- nothing is silently dropped without a record, it's
just not rendered as review images or put in front of a human by
default. No review images are generated for discards (would defeat the
point of cutting the manual-review volume).

=======================================================================
THREE-TIER SPLIT (auto-accept side, unchanged from V3):
  - AUTO-ACCEPT (vote_count >= 3): written straight to a YOLO-format
    label file, UNCONDITIONALLY -- no confidence floor. Three or four
    architecturally-independent models agreeing is strong enough
    evidence on its own that gating it further would just be adding
    noise to a decision that's already clear.
  - AUTO-ACCEPT (vote_count == 2), CONDITIONAL: written straight to a
    label file ONLY IF min(the two present confidences) >= --min-conf
    AND the pairwise IoU between those two members >= --min-iou. This
    is the same spirit as V2's whole agree-bucket gate, just scoped
    down to exactly the tier where two-model evidence needs a floor on
    top of it (three-plus votes don't need this gate at all anymore).
  - NEEDS REVIEW: vote_count == 1 at/above --min-single-vote-conf
    (reason: "single_model_only"); vote_count == 2 that fails the
    confidence/IoU gate (reason: "below_threshold"); and ALWAYS_REVIEW-
    class clusters at/above --min-always-review-conf regardless of
    vote_count (reason: "always_review_class").
  - DISCARDED (new in V4): vote_count == 1 below --min-single-vote-conf
    for normal classes, and ALWAYS_REVIEW-class clusters below
    --min-always-review-conf. Never written to a label file, never
    rendered as a review image; recorded in pseudo_labels_discarded.json
    and summarized per-class in pseudo_labels_summary.json.

WHY ALWAYS_REVIEW EXISTS (see class_map.py's own docstring for the
domain reasoning: SARD is wilderness/forest/quarry SAR footage): if a
class/source pair almost never gets multi-model corroboration (e.g.
SARD motorcycle), most or all of its evidence sits at vote_count==1 --
outside cross_reference_candidates.json entirely. ALWAYS_REVIEW pulls
every single-model hit that exists (from cross_reference_full.json) and
puts it in front of a human instead of silently dropping the class --
subject now to --min-always-review-conf's floor, see V4 note above.
Default:

    ALWAYS_REVIEW = {"SARD": ["motorcycle"]}

Override via --always-review SOURCE:class (repeatable).

OUTPUT: same shape as V3, plus one new file --
  datasets/pseudo_labels/<SOURCE>/<mirrors the real label path>.txt
      YOLO-format lines for AUTO-ACCEPT boxes only (both the
      unconditional 3/4-vote tier and the conditional 2-vote tier that
      passed its gate). Written to a SEPARATE tree -- see --apply.
  datasets/pseudo_labels_review_queue.json
      Every NEEDS REVIEW item (post-discard-gate), carrying a full
      "votes" dict (all four models' confidence-or-null) and
      "vote_count".
  datasets/pseudo_labels_discarded.json  [NEW in V4]
      Every DISCARDED item, same shape as the review queue plus a
      "reason" of "below_single_vote_floor" or
      "below_always_review_floor" -- an audit trail for what got cut
      and why, in case --min-single-vote-conf / --min-always-review-conf
      need retuning.
  datasets/pseudo_labels_review/<source>/
      Annotated review images for the (now smaller) queue.
  datasets/pseudo_labels_summary.json
      Per-source/per-class auto-accepted vs queued vs discarded counts
      BY TIER (3-4-vote unconditional, 2-vote conditional, plus a
      threshold-sensitivity table for the 2-vote tier only -- the 3/4-
      vote tier has no threshold to sweep, it's unconditional by
      design).

LABEL-PATH ASSUMPTION (verify before trusting --apply): unchanged --
derives each image's label file by replacing the "images" path
component with "labels" and swapping the extension to .txt. The script
prints the first derived (image -> label) pair per source at startup --
confirm that path is a real, existing file before trusting --apply.

--apply: unchanged -- merges datasets/pseudo_labels/ (this script's
auto-accept tier) and datasets/pseudo_labels_reviewed/
(review_labels.py's output, if present) into the real dataset,
appending and skipping byte-identical duplicate lines.

USAGE:
    # 1. Dry run with the new discard gate at its defaults (0.40 single-
    #    vote / 0.30 always-review). Cheap -- reads cross_reference_
    #    candidates.json / cross_reference_full.json, no GPU involved.
    python generate_pseudo_labels.py

    # 2. Check datasets/pseudo_labels_summary.json's per-class
    #    auto/queued/discarded counts and sensitivity table (2-vote
    #    tier only), and a handful of datasets/pseudo_labels_discarded
    #    .json entries to sanity-check nothing real is being thrown
    #    away. Adjust --min-conf / --min-iou / --min-single-vote-conf /
    #    --min-always-review-conf and rerun if the split looks wrong.
    #    Passing --min-single-vote-conf 0 --min-always-review-conf 0
    #    fully restores V3 behavior (nothing discarded) if you'd rather
    #    start from the unfiltered queue.

    # 3. Once satisfied:
    python generate_pseudo_labels.py --apply
"""

import argparse
import json
import statistics
from pathlib import Path

import cv2
from PIL import Image

import prepare_datasets
from class_map import UNIFIED_CLASSES

DEFAULT_ALWAYS_REVIEW = {"SARD": ["motorcycle"]}

MODEL_TAGS = ["yolo_visdrone", "rfdetr_visdrone", "yolo_coco", "dino"]
MODEL_ABBREV = {"yolo_visdrone": "yv", "rfdetr_visdrone": "rf",
                 "yolo_coco": "yc", "dino": "dn"}

COLOR_QUEUE = (0, 165, 255)   # orange


# ---------------------------------------------------------------------
# Path helpers -- unchanged
# ---------------------------------------------------------------------

def image_path_to_label_path(img_path: Path) -> Path:
    """
    root/<split>/images/xxx.jpg -> root/<split>/labels/xxx.txt
    Case-insensitive match on the "images" path component (Windows
    paths seen in this project mix case in places).
    """
    parts = list(img_path.parts)
    for i in range(len(parts) - 1, -1, -1):
        if parts[i].lower() == "images":
            parts[i] = "labels"
            return Path(*parts).with_suffix(".txt")
    raise ValueError(
        f"Could not find an 'images' path component in {img_path} -- "
        f"the images->labels convention this script assumes doesn't "
        f"match this path. Check prepare_datasets.py's actual layout "
        f"and adjust image_path_to_label_path() if needed.")


def mirror_under_pseudo_root(real_label_path: Path, source_name: str,
                              pseudo_root: Path) -> Path:
    """Mirrors a real label path under datasets/pseudo_labels/<SOURCE>/
    using the path segments from <SOURCE>/ onward."""
    parts = real_label_path.parts
    try:
        idx = [p.lower() for p in parts].index(source_name.lower())
    except ValueError:
        return pseudo_root / source_name / real_label_path.name
    return pseudo_root / source_name / Path(*parts[idx + 1:])


# ---------------------------------------------------------------------
# Loading + tiering
# ---------------------------------------------------------------------

def parse_always_review(pairs: list[str]) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for pair in pairs:
        source, _, cls = pair.partition(":")
        if not cls:
            raise SystemExit(f"--always-review expects SOURCE:class, got '{pair}'")
        out.setdefault(source, []).append(cls)
    return out


def load_json(path: Path):
    if not path.exists():
        raise SystemExit(
            f"{path} not found -- run cross_reference_gaps.py first, "
            f"this script only consumes its output.")
    with open(path) as f:
        return json.load(f)


def two_vote_min_conf(item: dict) -> float:
    """min of the two present confidences in a vote_count==2 cluster."""
    confs = [v for v in item["votes"].values() if v is not None]
    return min(confs) if confs else 0.0


def two_vote_pairwise_iou(item: dict) -> float:
    """The pairwise IoU between the two contributing models, read from
    member_ious. One entry is always 1.0 (the representative vs itself);
    the other present entry is the real cross-model IoU we want."""
    ious = [v for m, v in item["member_ious"].items()
            if v is not None and v < 1.0 - 1e-9]
    if ious:
        return min(ious)
    # both entries were 1.0 (degenerate/identical boxes) -- treat as
    # perfect agreement rather than as missing data.
    present = [v for v in item["member_ious"].values() if v is not None]
    return min(present) if present else 0.0


def single_vote_conf(cluster: dict) -> float:
    """The one present confidence in a vote_count==1 cluster. Used for
    both the single-model-only gate and the always-review gate -- both
    only ever have exactly one non-null vote by construction."""
    confs = [v for v in cluster["votes"].values() if v is not None]
    return confs[0] if confs else 0.0


def sensitivity_table(two_vote_items: list[dict], thresholds: list[float]) -> dict:
    """For the 2-vote tier only: how many would pass min-conf at each
    threshold (IoU gate held fixed at whatever --min-iou is)."""
    mins = [two_vote_min_conf(it) for it in two_vote_items]
    table = {}
    for t in thresholds:
        table[str(t)] = sum(1 for m in mins if m >= t)
    stats = {
        "n": len(mins),
        "mean": statistics.mean(mins) if mins else None,
        "median": statistics.median(mins) if mins else None,
    }
    return {"pass_count_by_threshold": table, "min_conf_stats": stats}


def tier_source(source_name: str, candidates: list[dict], full_detections: dict,
                 missing_classes: list[str], always_review_classes: list[str],
                 min_conf: float, min_iou: float,
                 min_single_vote_conf: float = 0.0,
                 min_always_review_conf: float = 0.0):
    """
    candidates: this source's vote_count>=2 clusters (from cross_
      reference_candidates.json), each already carrying "image".
    full_detections: {image_key: [cluster, ...]} for this source, ALL
      vote counts -- consulted for always_review_classes and for
      vote_count==1 clusters of normal classes.

    Returns (auto_accepted, queued, discarded). Each item is a flat
    dict with cls, image, box, votes, vote_count, and (queued/discarded
    only) reason.

    Gating:
      - vote_count >= 3: always auto-accepted, unconditional.
      - vote_count == 2: auto-accepted if it clears --min-conf /
        --min-iou, else queued (reason: "below_threshold"). Never
        discarded -- two independent models agreeing at all is treated
        as worth a human look even below the auto-accept bar.
      - vote_count == 1, normal class: queued if its lone confidence is
        >= min_single_vote_conf, else discarded (reason:
        "below_single_vote_floor").
      - always_review_classes, any vote_count: queued if its best
        present confidence is >= min_always_review_conf, else discarded
        (reason: "below_always_review_floor"). This OVERRIDES the
        normal vote_count>=2 auto-accept path too -- always_review
        classes are never silently auto-written, matching V3 behavior
        (SARD motorcycle etc. always goes through a human).
    """
    auto_accepted = []
    queued = []
    discarded = []

    by_class: dict[str, list[dict]] = {}
    for c in candidates:
        by_class.setdefault(c["cls"], []).append(c)

    for cls_name in missing_classes:
        if cls_name in always_review_classes:
            # Pull EVERY vote_count for this class from the full
            # per-image record, including vote_count==1 -- single-model
            # hits may be the only evidence that exists here at all.
            # Gated by min_always_review_conf instead of being queued
            # unconditionally (V3 behavior) -- see module docstring.
            for image_key, clusters in full_detections.items():
                for c in clusters:
                    if c["cls"] != cls_name:
                        continue
                    conf = max(v for v in c["votes"].values() if v is not None)
                    item = {
                        "source": source_name, "cls": cls_name, "image": image_key,
                        "box": c["box"], "votes": c["votes"],
                        "vote_count": c["vote_count"],
                    }
                    if conf < min_always_review_conf:
                        discarded.append({**item, "reason": "below_always_review_floor"})
                    else:
                        queued.append({**item, "reason": "always_review_class"})
            continue

        for c in by_class.get(cls_name, []):
            item = {
                "source": source_name, "cls": cls_name, "image": c["image"],
                "box": c["box"], "votes": c["votes"], "vote_count": c["vote_count"],
            }
            if c["vote_count"] >= 3:
                # Unconditional -- three or four independent models
                # agreeing doesn't need a confidence floor on top.
                auto_accepted.append(item)
            else:
                # vote_count == 2 -- gated, never discarded (see docstring).
                passes = (two_vote_min_conf(c) >= min_conf
                          and two_vote_pairwise_iou(c) >= min_iou)
                if passes:
                    auto_accepted.append(item)
                else:
                    queued.append({**item, "reason": "below_threshold"})

    # vote_count == 1 clusters for NON-always-review classes never reach
    # cross_reference_candidates.json in the first place (cross_
    # reference_gaps.py only writes vote_count>=2 there), so they'd
    # silently vanish unless pulled in here explicitly. Gated by
    # min_single_vote_conf -- below it, discarded rather than queued
    # (see V4 docstring note for why this is safe: no corroboration +
    # near-floor confidence is the project's own documented noise
    # signature, and these classes were never labeled here anyway).
    for cls_name in missing_classes:
        if cls_name in always_review_classes:
            continue
        for image_key, clusters in full_detections.items():
            for c in clusters:
                if c["cls"] != cls_name or c["vote_count"] != 1:
                    continue
                conf = single_vote_conf(c)
                item = {
                    "source": source_name, "cls": cls_name, "image": image_key,
                    "box": c["box"], "votes": c["votes"], "vote_count": 1,
                }
                if conf < min_single_vote_conf:
                    discarded.append({**item, "reason": "below_single_vote_floor"})
                else:
                    queued.append({**item, "reason": "single_model_only"})

    return auto_accepted, queued, discarded


# ---------------------------------------------------------------------
# Label file writing -- unchanged
# ---------------------------------------------------------------------

_dim_cache: dict[str, tuple[int, int]] = {}


def get_image_dims(image_path: str) -> tuple[int, int]:
    if image_path not in _dim_cache:
        with Image.open(image_path) as im:
            _dim_cache[image_path] = im.size  # (width, height)
    return _dim_cache[image_path]


def box_to_yolo_line(cls_name: str, box: list[float], img_w: int, img_h: int) -> str:
    cls_idx = UNIFIED_CLASSES.index(cls_name)
    x1, y1, x2, y2 = box
    xc = ((x1 + x2) / 2) / img_w
    yc = ((y1 + y2) / 2) / img_h
    w = (x2 - x1) / img_w
    h = (y2 - y1) / img_h
    return f"{cls_idx} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}"


def write_auto_accept_labels(auto_accepted: list[dict], source_name: str,
                              pseudo_root: Path) -> dict[Path, list[str]]:
    by_label_path: dict[Path, list[str]] = {}
    for item in auto_accepted:
        img_path = Path(item["image"])
        real_label_path = image_path_to_label_path(img_path)
        pseudo_label_path = mirror_under_pseudo_root(real_label_path, source_name, pseudo_root)
        w, h = get_image_dims(item["image"])
        line = box_to_yolo_line(item["cls"], item["box"], w, h)
        by_label_path.setdefault(pseudo_label_path, []).append(line)

    for label_path, lines in by_label_path.items():
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("\n".join(lines) + "\n")

    return by_label_path


def merge_pseudo_trees_into_dataset(roots: list[Path], datasets_dir: Path):
    """Unchanged -- see V2/V3 docstring for the two-tree merge reasoning
    (auto-accept tree gets wholesale-rewritten every run, reviewed tree
    doesn't, so they're merged at apply time instead of being the same
    tree)."""
    written, skipped_dupe = 0, 0
    per_root_counts = {}

    for root in roots:
        if not root.exists():
            continue
        root_written = 0
        for txt_path in root.rglob("*.txt"):
            rel = txt_path.relative_to(root)
            real_label_path = datasets_dir / rel
            new_lines = [l.strip() for l in txt_path.read_text().splitlines() if l.strip()]
            if not new_lines:
                continue

            existing = set()
            if real_label_path.exists():
                existing = {l.strip() for l in real_label_path.read_text().splitlines() if l.strip()}
            to_add = [l for l in new_lines if l not in existing]
            skipped_dupe += len(new_lines) - len(to_add)
            if not to_add:
                continue

            real_label_path.parent.mkdir(parents=True, exist_ok=True)
            with open(real_label_path, "a") as f:
                for l in to_add:
                    f.write(l + "\n")
            written += len(to_add)
            root_written += len(to_add)
        per_root_counts[str(root)] = root_written

    print(f"\n  --apply: wrote {written} new lines to real label files "
          f"(skipped {skipped_dupe} already-present duplicates):")
    for root_str, count in per_root_counts.items():
        print(f"    {count:>6} from {root_str}")


# ---------------------------------------------------------------------
# Review queue images
# ---------------------------------------------------------------------

def votes_str(item: dict) -> str:
    parts = [f"{MODEL_ABBREV[m]}{item['votes'][m]:.2f}"
             for m in MODEL_TAGS if item["votes"].get(m) is not None]
    return f"v{item['vote_count']}/4 " + "/".join(parts)


def save_review_images(queued: list[dict], source_name: str, review_dir: Path,
                        max_images: int):
    by_image: dict[str, list[dict]] = {}
    for item in queued:
        by_image.setdefault(item["image"], []).append(item)

    out_dir = review_dir / source_name.lower()
    out_dir.mkdir(parents=True, exist_ok=True)

    image_keys = list(by_image.keys())
    if len(image_keys) > max_images:
        print(f"  [warn] {len(image_keys)} images have queued items for "
              f"{source_name}, capping review renders at {max_images} "
              f"(--max-review-images). All items are still in "
              f"pseudo_labels_review_queue.json regardless.")
        image_keys = image_keys[:max_images]

    for i, image_key in enumerate(image_keys):
        img = cv2.imread(image_key)
        if img is None:
            continue
        for item in by_image[image_key]:
            x1, y1, x2, y2 = [int(round(v)) for v in item["box"]]
            cv2.rectangle(img, (x1, y1), (x2, y2), COLOR_QUEUE, 2)
            label = f"{item['cls']} {votes_str(item)} ({item['reason']})"
            cv2.putText(img, label, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, COLOR_QUEUE, 1, cv2.LINE_AA)
        out_path = out_dir / f"{i:03d}_{Path(image_key).stem}.jpg"
        cv2.imwrite(str(out_path), img)

    print(f"  [{source_name}] saved {len(image_keys)} review images to {out_dir}/")


# ---------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--min-conf", type=float, default=0.40,
                    help="2-vote tier ONLY: auto-accept requires "
                         "min(the two present confidences) >= this. The "
                         "3/4-vote tier is unconditional and ignores "
                         "this flag entirely. Check pseudo_labels_"
                         "summary.json's sensitivity table before "
                         "committing to a value.")
    p.add_argument("--min-iou", type=float, default=0.60,
                    help="2-vote tier ONLY: auto-accept requires the "
                         "pairwise IoU between the two contributing "
                         "models' boxes >= this -- stricter than the "
                         "clustering IoU cross_reference_gaps.py used "
                         "to form the cluster in the first place.")
    p.add_argument("--min-single-vote-conf", type=float, default=0.40,
                    help="[V4] vote_count==1 clusters for a normal "
                         "(non-always-review) class below this "
                         "confidence are DISCARDED instead of queued "
                         "for review -- see module docstring's V4 note. "
                         "Pass 0.0 to restore V3 behavior (queue every "
                         "single-vote hit regardless of confidence).")
    p.add_argument("--min-always-review-conf", type=float, default=0.30,
                    help="[V4] ALWAYS_REVIEW classes (e.g. SARD "
                         "motorcycle) below this confidence are "
                         "DISCARDED instead of queued. Deliberately "
                         "lower than --min-single-vote-conf since these "
                         "classes may have no other evidence tier at "
                         "all. Pass 0.0 to restore V3 behavior (queue "
                         "every hit for these classes regardless of "
                         "confidence).")
    p.add_argument("--always-review", nargs="*", default=["SARD:motorcycle"],
                    help="SOURCE:class pairs that are NEVER auto-accepted "
                         "regardless of vote_count, always queued for "
                         "manual review (subject to --min-always-review-"
                         "conf). Default matches the run that showed "
                         "SARD motorcycle at agree=0 in V2.")
    p.add_argument("--sensitivity-thresholds", type=float, nargs="+",
                    default=[0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60])
    p.add_argument("--max-review-images", type=int, default=300,
                    help="Cap on rendered review images per source (all "
                         "queued items are still in the JSON regardless).")
    p.add_argument("--apply", action="store_true",
                    help="Actually append auto-accepted labels to the "
                         "real dataset label files. Without this flag, "
                         "output only goes to datasets/pseudo_labels/ "
                         "(a dry run).")
    return p.parse_args()


def main():
    args = parse_args()
    always_review = parse_always_review(args.always_review)

    datasets_dir = prepare_datasets.DATASETS_DIR
    candidates_by_source = load_json(datasets_dir / "cross_reference_candidates.json")
    full_by_source = load_json(datasets_dir / "cross_reference_full.json")

    pseudo_root = datasets_dir / "pseudo_labels"
    review_dir = datasets_dir / "pseudo_labels_review"
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    discarded_path = datasets_dir / "pseudo_labels_discarded.json"
    summary_path = datasets_dir / "pseudo_labels_summary.json"

    all_queued = []
    all_discarded = []
    summary = {"min_conf": args.min_conf, "min_iou": args.min_iou,
               "min_single_vote_conf": args.min_single_vote_conf,
               "min_always_review_conf": args.min_always_review_conf,
               "always_review": always_review, "sources": {}}

    for source_name, candidates in candidates_by_source.items():
        full_detections = full_by_source.get(source_name, {})
        # missing_classes: every class that appears anywhere in either
        # file for this source, plus anything forced via --always-review.
        classes_in_candidates = {c["cls"] for c in candidates}
        classes_in_full = {c["cls"] for clusters in full_detections.values() for c in clusters}
        missing_classes = sorted(classes_in_candidates | classes_in_full
                                  | set(always_review.get(source_name, [])))
        if not missing_classes:
            continue

        print(f"\n{'=' * 70}\n{source_name}\n{'=' * 70}")

        auto_accepted, queued, discarded = tier_source(
            source_name, candidates, full_detections, missing_classes,
            always_review.get(source_name, []), args.min_conf, args.min_iou,
            args.min_single_vote_conf, args.min_always_review_conf)

        if auto_accepted:
            sample_img = Path(auto_accepted[0]["image"])
            print(f"  Label-path check: {sample_img}\n"
                  f"                 -> {image_path_to_label_path(sample_img)}\n"
                  f"    (confirm this points at a real file before trusting --apply)")

        by_label_path = write_auto_accept_labels(auto_accepted, source_name, pseudo_root)
        n_unconditional = sum(1 for a in auto_accepted if a["vote_count"] >= 3)
        n_conditional = len(auto_accepted) - n_unconditional
        print(f"  Auto-accepted: {len(auto_accepted)} boxes across "
              f"{len(by_label_path)} images -> {pseudo_root / source_name}/ "
              f"({n_unconditional} at 3-4/4 votes, {n_conditional} at "
              f"2/4 votes past threshold)")
        print(f"  Queued for review: {len(queued)}   "
              f"Discarded (below confidence floor, not written or queued): "
              f"{len(discarded)}")

        save_review_images(queued, source_name, review_dir, args.max_review_images)
        all_queued.extend(queued)
        all_discarded.extend(discarded)

        per_class_summary = {}
        for cls_name in missing_classes:
            cls_candidates_2vote = [c for c in candidates
                                     if c["cls"] == cls_name and c["vote_count"] == 2]
            per_class_summary[cls_name] = {
                "auto_accepted": sum(1 for a in auto_accepted if a["cls"] == cls_name),
                "auto_accepted_unconditional_3_4_vote": sum(
                    1 for a in auto_accepted if a["cls"] == cls_name and a["vote_count"] >= 3),
                "auto_accepted_conditional_2_vote": sum(
                    1 for a in auto_accepted if a["cls"] == cls_name and a["vote_count"] == 2),
                "queued": sum(1 for q in queued if q["cls"] == cls_name),
                "discarded_low_conf": sum(1 for d in discarded if d["cls"] == cls_name),
                "always_review": cls_name in always_review.get(source_name, []),
                "two_vote_sensitivity": sensitivity_table(
                    cls_candidates_2vote, args.sensitivity_thresholds) if cls_candidates_2vote else None,
            }
            s = per_class_summary[cls_name]
            print(f"    {cls_name:15s} auto={s['auto_accepted']:>5}  "
                  f"(3-4vote={s['auto_accepted_unconditional_3_4_vote']:>5} "
                  f"2vote={s['auto_accepted_conditional_2_vote']:>5})  "
                  f"queued={s['queued']:>5}  discarded={s['discarded_low_conf']:>6}  "
                  f"{'[ALWAYS REVIEW]' if s['always_review'] else ''}")

        summary["sources"][source_name] = per_class_summary

    if args.apply:
        reviewed_root = datasets_dir / "pseudo_labels_reviewed"
        merge_pseudo_trees_into_dataset([pseudo_root, reviewed_root], datasets_dir)

    with open(queue_path, "w") as f:
        json.dump(all_queued, f, indent=2, default=str)
    with open(discarded_path, "w") as f:
        json.dump(all_discarded, f, indent=2, default=str)
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)

    print(f"\n{'=' * 70}")
    print(f"Pseudo-labels (dry run tree): {pseudo_root}/")
    print(f"Review queue ({len(all_queued)} items): {queue_path}")
    print(f"Discarded, below confidence floor ({len(all_discarded)} items): {discarded_path}")
    print(f"Review images: {review_dir}/<source>/")
    print(f"Summary + sensitivity table: {summary_path}")
    if not args.apply:
        print(f"\nDRY RUN -- real dataset label files were NOT modified. "
              f"Rerun with --apply once you've checked the above.")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()