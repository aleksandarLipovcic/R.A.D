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
V5 (2026-08-12): guaranteed clean-slate outputs on EVERY run.
=======================================================================
Before V5, datasets/pseudo_labels/ (the auto-accept tree) and
datasets/pseudo_labels_review/ (the rendered review images) were only
ever ADDED to on each run -- write_auto_accept_labels() wrote a label
file for every image accepted THIS run, but never removed a label file
left over from a PREVIOUS run for an image that isn't accepted this
time (e.g. because you retuned --min-conf/--min-iou/--min-single-vote-
conf/--min-always-review-conf between runs, or reran after cross_
reference_gaps.py produced a different candidate set). Concretely: run
A auto-accepts image X at loose thresholds -> pseudo_labels/.../X.txt
exists. You tighten thresholds and rerun; X no longer qualifies. The
new run never touches X.txt because it has nothing to write for X --
so the stale, no-longer-justified label file from run A silently
survives into whatever --apply merges into the real dataset. Same
problem for pseudo_labels_review/: if run A rendered review images 000
through 250 and run B's (smaller) queue only fills 000-180, images
181-250 from run A are untouched leftovers that don't correspond to
anything in the CURRENT pseudo_labels_review_queue.json, and someone
skimming the review folder has no way to tell they're stale.

Both trees are fully reproducible from cross_reference_candidates.json
+ cross_reference_full.json plus the current CLI flags -- nothing
irreplaceable lives in either of them. So as of V5, EVERY run (dry run
or --apply) now wipes both trees completely via clean_output_dir()
before writing anything, unconditionally -- no flag needed to opt in,
because there's never a good reason to keep a previous run's partial
output lying around next to this run's. Pass --keep-existing-outputs
only if you explicitly want the old (pre-V5, additive) behavior for
some kind of manual A/B comparison between two runs' output folders;
this is NOT the normal/recommended way to use this script.

datasets/pseudo_labels_review_queue.json, pseudo_labels_discarded.json,
and pseudo_labels_summary.json were ALREADY safe before V5 -- each is a
single JSON file opened with mode "w" and written in one shot, which
inherently replaces its entire previous contents rather than appending
to them. No change needed there; noting it here so it's obvious this
wasn't also a silent-staleness risk.

WHAT V5 NEVER TOUCHES (hard safety boundary, not just convention):
clean_output_dir() is called ONLY on pseudo_root (datasets/
pseudo_labels/) and review_dir (datasets/pseudo_labels_review/) --
this script's OWN output trees. It is never called anywhere near
datasets/pseudo_labels_reviewed/ (review_labels.py's manual-review
output tree), datasets/review_progress.json, or datasets/
review_completed.json -- those hold actual human review work and
belong exclusively to review_labels.py; this script only ever READS
near them, at --apply time, via merge_pseudo_trees_into_dataset(),
which opens reviewed_root read-only (rglob + read_text, no write/
delete calls against that tree at all). clean_output_dir() also
refuses at runtime (raises, does not silently no-op) if ever called
with a path whose name is "pseudo_labels_reviewed" or that resolves
under a directory containing "review_progress.json"/"review_completed.
json" as siblings -- a belt-and-suspenders check purely to make an
accidental future call site (e.g. a copy-paste mistake) fail loudly
instead of deleting someone's manual review work.

PRE-CLEAN STALENESS WARNING: if datasets/review_progress.json already
has entries (i.e. manual review has started) when this script runs
again, regenerating the queue can change which items are queued vs.
auto-accepted vs. discarded this time -- an item a reviewer already
made a decision on may vanish from the new queue (harmless but stale
progress.json entry) or, less commonly, an item never seen before may
newly appear. This script now prints an explicit warning and a count
of existing progress entries in that situation, before doing anything
else, so this is a decision you make with the numbers in front of you
rather than something that silently happens. It does not block the
run -- review_labels.py's own progress/completed files are untouched
either way (see boundary note above) -- it's purely informational.

=======================================================================
[PATCH] --candidates-file / --full-file (added alongside
resolve_cross_class_conflicts.py): this script always used to read
cross_reference_candidates.json / cross_reference_full.json by a fixed
name. Both are now CLI-overridable so this script can consume
resolve_cross_class_conflicts.py's merged output (cross_reference_
candidates_resolved.json / cross_reference_full_resolved.json) --
which cross-class-merges vote clusters that cluster_detections() only
ever clustered within a single class -- without needing any other
change here. The resolved files are written in the identical shape
this script already expects, so nothing else in this file needed to
change. Defaults are unchanged (the original filenames), so omitting
these flags reproduces the exact pre-patch behavior.
=======================================================================

=======================================================================
V6: per-class single-vote floor + a sensitivity table for it.
=======================================================================
--min-single-vote-conf was always a single GLOBAL floor across every
missing class for a source. In practice one class can dominate the
queue by an order of magnitude (a real UAVDT run: person at 133,052
single-vote-tier items vs. motorcycle/other_vehicle in the low tens of
thousands combined) while sitting on a totally different confidence
distribution -- raising the global floor enough to meaningfully shrink
person's share also drags motorcycle/other_vehicle's floor up with it,
even if THEIR floor was already well-tuned. Two additions fix that:

  1. --min-single-vote-conf-per-class SOURCE:class:threshold
     (repeatable) overrides --min-single-vote-conf for one specific
     source+class. Anything not overridden keeps using the global
     value, unchanged. Always-review classes (e.g. SARD motorcycle)
     are untouched by this flag -- they only ever respond to
     --min-always-review-conf, same as before.

  2. Every per_class_summary entry now also carries
     "single_vote_sensitivity" -- the same shape as the pre-existing
     "two_vote_sensitivity" (pass_count_by_threshold + min_conf_stats),
     but built from that class's actual vote_count==1 confidences
     across the WHOLE dataset (not gated by the current floor). Check
     this in pseudo_labels_summary.json BEFORE picking a per-class
     threshold -- e.g. "at 0.55, person drops from 28,355 queued to
     19,200" -- the same way you'd already check two_vote_sensitivity
     before touching --min-conf. This is read-only reporting; it does
     not change what gets discarded on its own, --min-single-vote-
     conf-per-class does that.

Neither addition changes default behavior: omit both and you get the
exact same auto/queued/discarded split as before.
=======================================================================

=======================================================================
V7: discard floor for the 2-vote tier (was: never discarded).
=======================================================================
V3-V6 all treated any vote_count==2 cluster that failed the auto-accept
gate (min_conf + min_iou) as automatically worth a human look, no
matter how low its confidence -- "two independent models agreeing at
all is real corroboration" was the reasoning. In practice this let
weak two-model agreement (e.g. two models both firing ~0.35-0.40 on
the same spot, sometimes on nothing at all) flood the queue: on a real
UAVDT run, person's 2-vote tier alone was 18,662 queued items with zero
floor, and manual review confirmed a meaningful fraction of those were
visibly not real objects -- two weak, near-floor detections can agree
by coincidence in cluttered scenes just as easily as two independent
models can hallucinate the same near-floor single-vote noise V4's gate
was already built to catch. The corroboration argument holds up much
better once BOTH models clear a real confidence bar, not just "both
fired at all."

  --min-two-vote-review-conf (default 0.0, i.e. off): 2-vote clusters
      that fail the auto-accept gate are now DISCARDED instead of
      queued if two_vote_min_conf(cluster) is below this. At/above it,
      queued as before (reason: "below_threshold" -- unchanged). 0.0
      preserves exact pre-V7 behavior (nothing discarded here). Check
      each class's EXISTING two_vote_sensitivity table in pseudo_
      labels_summary.json before picking a value -- it already reports
      pass-count-by-threshold for this exact population, no rerun
      needed to size the cut.

  --min-two-vote-review-conf-per-class SOURCE:class:threshold
      (repeatable, same SOURCE:class:threshold shape as --min-single-
      vote-conf-per-class): overrides the global floor for one
      source+class. Never applies to always_review_classes -- those
      still only ever respond to --min-always-review-conf.

vote_count>=3 clusters are completely unaffected -- V7 only touches
the 2-vote "failed auto-accept" branch. Every discarded item here is
still recorded in pseudo_labels_discarded.json with reason
"below_two_vote_review_floor", same audit-trail guarantee as every
other discard path in this script.
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

OUTPUT (all fully rewritten from scratch every run -- see V5 note):
  datasets/pseudo_labels/<SOURCE>/<mirrors the real label path>.txt
      YOLO-format lines for AUTO-ACCEPT boxes only (both the
      unconditional 3/4-vote tier and the conditional 2-vote tier that
      passed its gate). Written to a SEPARATE tree -- see --apply.
      Directory is wiped clean at the start of every run before any
      file is written (V5).
  datasets/pseudo_labels_review_queue.json
      Every NEEDS REVIEW item (post-discard-gate), carrying a full
      "votes" dict (all four models' confidence-or-null) and
      "vote_count". Whole-file overwrite every run (always was).
  datasets/pseudo_labels_discarded.json  [NEW in V4]
      Every DISCARDED item, same shape as the review queue plus a
      "reason" of "below_single_vote_floor" or
      "below_always_review_floor" -- an audit trail for what got cut
      and why, in case --min-single-vote-conf / --min-always-review-conf
      need retuning. Whole-file overwrite every run (always was).
  datasets/pseudo_labels_review/<source>/
      Annotated review images for the (now smaller) queue. Directory
      is wiped clean at the start of every run before any file is
      written (V5).
  datasets/pseudo_labels_summary.json
      Per-source/per-class auto-accepted vs queued vs discarded counts
      BY TIER (3-4-vote unconditional, 2-vote conditional, plus a
      threshold-sensitivity table for the 2-vote tier only -- the 3/4-
      vote tier has no threshold to sweep, it's unconditional by
      design). Whole-file overwrite every run (always was).

LABEL-PATH ASSUMPTION (verify before trusting --apply): unchanged --
derives each image's label file by replacing the "images" path
component with "labels" and swapping the extension to .txt. The script
prints the first derived (image -> label) pair per source at startup --
confirm that path is a real, existing file before trusting --apply.

--apply: unchanged -- merges datasets/pseudo_labels/ (this script's
auto-accept tier, freshly regenerated this run) and datasets/
pseudo_labels_reviewed/ (review_labels.py's output, if present, NEVER
modified by this script -- read-only at merge time) into the real
dataset, appending and skipping byte-identical duplicate lines.

USAGE:
    # 1. Dry run with the new discard gate at its defaults (0.40 single-
    #    vote / 0.30 always-review). Cheap -- reads cross_reference_
    #    candidates.json / cross_reference_full.json, no GPU involved.
    #    Every run starts from a clean pseudo_labels/ + pseudo_labels_
    #    review/ -- see V5 note above.
    python generate_pseudo_labels.py

    # 1b. [PATCH] Same, but consuming resolve_cross_class_conflicts.py's
    #    cross-class-merged output instead of cross_reference_gaps.py's
    #    raw output:
    python generate_pseudo_labels.py \
        --candidates-file cross_reference_candidates_resolved.json \
        --full-file cross_reference_full_resolved.json

    # 2. Check datasets/pseudo_labels_summary.json's per-class
    #    auto/queued/discarded counts and sensitivity table (2-vote
    #    tier only), and a handful of datasets/pseudo_labels_discarded
    #    .json entries to sanity-check nothing real is being thrown
    #    away. Adjust --min-conf / --min-iou / --min-single-vote-conf /
    #    --min-always-review-conf and rerun if the split looks wrong --
    #    every rerun is a full clean regeneration, so there's never a
    #    mix of two different threshold settings' output sitting in the
    #    same folder.
    #    Passing --min-single-vote-conf 0 --min-always-review-conf 0
    #    fully restores V3 behavior (nothing discarded) if you'd rather
    #    start from the unfiltered queue.
    #
    #    IMPORTANT: do this threshold-tuning BEFORE starting manual
    #    review in review_labels.py. Once review_progress.json has real
    #    decisions in it, rerunning this script (even just to tweak a
    #    threshold, or to switch to the resolved files) regenerates the
    #    queue and can orphan some of that progress -- the script will
    #    warn you with a count if it detects this, but the clean way to
    #    work is: tune thresholds here first, confirm the summary/
    #    discarded files look right, THEN start review_labels.py. If
    #    you've already started reviewing, note that your decisions
    #    live in datasets/pseudo_labels_reviewed/ + review_progress.json
    #    -- this script never cleans or writes to either (see the V5
    #    "WHAT V5 NEVER TOUCHES" section above), so rerunning is safe;
    #    at worst some already-reviewed items won't reappear in the new
    #    queue, which is harmless since --apply picks their decisions up
    #    from pseudo_labels_reviewed/ regardless.

    # 3. Once satisfied and manual review (if any) is complete:
    python generate_pseudo_labels.py --apply
"""

import argparse
import json
import shutil
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

# Filenames that mark a directory as belonging to review_labels.py's
# manual-review workflow. clean_output_dir() refuses to touch any
# directory that is named this, or that directly contains either of
# these files as a sibling -- see the V5 docstring note's safety
# boundary section. This is deliberately redundant with "we simply
# never call clean_output_dir() on those paths" below -- the point is
# that a future copy-paste mistake at a NEW call site fails loudly
# instead of silently deleting manual review work.
PROTECTED_DIR_NAME = "pseudo_labels_reviewed"
PROTECTED_SIBLING_FILES = ("review_progress.json", "review_completed.json")


# ---------------------------------------------------------------------
# Clean-slate output helper -- V5
# ---------------------------------------------------------------------

def clean_output_dir(path: Path, label: str) -> None:
    """
    Wipes `path` completely (if it exists) and recreates it empty, so
    every run of this script starts its OWN output trees
    (pseudo_labels/, pseudo_labels_review/) from zero -- see this
    module's V5 docstring note for why that's necessary (previously,
    files from a prior run with different thresholds/candidates could
    silently survive alongside this run's files).

    SAFETY: refuses (raises RuntimeError, does not silently skip) if
    `path`'s name matches PROTECTED_DIR_NAME, or if `path` contains
    either of PROTECTED_SIBLING_FILES directly inside it. Those mark a
    directory as review_labels.py's manual-review output, which this
    script must NEVER delete. This function is only ever called (see
    main()) on pseudo_root and review_dir -- this check exists purely
    as a second, independent guard in case a future edit adds another
    call site by mistake.
    """
    if path.name == PROTECTED_DIR_NAME:
        raise RuntimeError(
            f"Refusing to clean {path} -- its name matches the protected "
            f"manual-review directory ({PROTECTED_DIR_NAME}). This "
            f"function must only ever be called on this script's OWN "
            f"output trees (pseudo_labels/, pseudo_labels_review/).")
    if path.exists():
        for sibling in PROTECTED_SIBLING_FILES:
            if (path / sibling).exists():
                raise RuntimeError(
                    f"Refusing to clean {path} -- it directly contains "
                    f"{sibling}, which marks this as a manual-review "
                    f"directory belonging to review_labels.py.")

    if path.exists():
        n_existing = sum(1 for _ in path.rglob("*") if _.is_file())
        shutil.rmtree(path)
        print(f"  [clean] removed {n_existing} existing file(s) from "
              f"{label} ({path})")
    else:
        print(f"  [clean] {label} ({path}) did not exist yet -- nothing to remove")
    path.mkdir(parents=True, exist_ok=True)


def warn_if_review_in_progress(datasets_dir: Path) -> None:
    """Purely informational (does not block, does not touch any file) --
    see V5 docstring note. Lets you rerun with your eyes open instead of
    being surprised later that some review_progress.json entries no
    longer correspond to anything in the freshly regenerated queue."""
    progress_path = datasets_dir / "review_progress.json"
    if not progress_path.exists():
        return
    try:
        with open(progress_path) as f:
            progress = json.load(f)
    except (json.JSONDecodeError, OSError):
        return
    # Only count real per-item decision entries, not the
    # "__manual__<image>" / "__override__<image>" bookkeeping keys
    # review_labels.py also stores in this same file.
    n_decisions = sum(1 for k in progress if not k.startswith("__"))
    if n_decisions == 0:
        return
    print(f"\n  {'!' * 66}")
    print(f"  WARNING: {progress_path} already has {n_decisions} manual "
          f"review decision(s) recorded.")
    print(f"  Regenerating the queue now may change which items are "
          f"queued/auto-accepted/discarded, which can leave some of "
          f"those decisions referring to items no longer in the new "
          f"queue (harmless, but stale). If you're just tuning "
          f"thresholds, prefer doing that BEFORE manual review starts.")
    print(f"  Your actual decisions live in datasets/pseudo_labels_"
          f"reviewed/ and this file -- neither is touched or cleaned "
          f"by this script, regardless of what you pass it.")
    print(f"  {'!' * 66}\n")


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


def parse_per_class_conf(triples: list[str]) -> dict[str, dict[str, float]]:
    """[V6] --min-single-vote-conf-per-class SOURCE:class:threshold ->
    {source: {cls: threshold}}."""
    out: dict[str, dict[str, float]] = {}
    for triple in triples:
        parts = triple.split(":")
        if len(parts) != 3:
            raise SystemExit(
                f"--min-single-vote-conf-per-class expects "
                f"SOURCE:class:threshold, got '{triple}'")
        source, cls, conf_str = parts
        try:
            conf = float(conf_str)
        except ValueError:
            raise SystemExit(
                f"--min-single-vote-conf-per-class: '{conf_str}' in "
                f"'{triple}' is not a number.")
        out.setdefault(source, {})[cls] = conf
    return out


def load_json(path: Path):
    if not path.exists():
        raise SystemExit(
            f"{path} not found -- run cross_reference_gaps.py first "
            f"(or resolve_cross_class_conflicts.py, if using "
            f"--candidates-file/--full-file to point at its resolved "
            f"output), this script only consumes their output.")
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


def confidence_sensitivity_table(confs: list[float], thresholds: list[float]) -> dict:
    """Generic 'how many would pass at each threshold' table -- confs is
    already the one relevant confidence value per item (min-of-two for
    the 2-vote tier, the lone vote for the 1-vote tier). 'Pass' means
    the item would stay queued/auto-accepted rather than be discarded/
    fail the gate at that threshold."""
    table = {}
    for t in thresholds:
        table[str(t)] = sum(1 for c in confs if c >= t)
    stats = {
        "n": len(confs),
        "mean": statistics.mean(confs) if confs else None,
        "median": statistics.median(confs) if confs else None,
    }
    return {"pass_count_by_threshold": table, "min_conf_stats": stats}


def sensitivity_table(two_vote_items: list[dict], thresholds: list[float]) -> dict:
    """For the 2-vote tier only: how many would pass min-conf at each
    threshold (IoU gate held fixed at whatever --min-iou is)."""
    return confidence_sensitivity_table(
        [two_vote_min_conf(it) for it in two_vote_items], thresholds)


def tier_source(source_name: str, candidates: list[dict], full_detections: dict,
                 missing_classes: list[str], always_review_classes: list[str],
                 min_conf: float, min_iou: float,
                 min_single_vote_conf: float = 0.0,
                 min_always_review_conf: float = 0.0,
                 min_single_vote_conf_overrides: dict[str, float] | None = None,
                 min_two_vote_review_conf: float = 0.0,
                 min_two_vote_review_conf_overrides: dict[str, float] | None = None):
    """
    min_single_vote_conf_overrides [V6]: {cls_name: threshold} for this
    source only. A class present here uses its own threshold instead of
    the global min_single_vote_conf for the vote_count==1 gate below --
    lets one dominant, differently-distributed class (e.g. UAVDT person)
    get its own floor without moving the floor for every other missing
    class on the same source. Never consulted for always_review_classes
    -- those only ever respond to min_always_review_conf.

    min_two_vote_review_conf / _overrides [V7]: same shape and same
    per-class-override mechanism, but for the vote_count==2 branch --
    see module docstring's V7 note for why a floor was added there too.
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
        --min-iou. Otherwise, queued (reason: "below_threshold") if
        two_vote_min_conf >= min_two_vote_review_conf, else discarded
        (reason: "below_two_vote_review_floor") [V7 -- previously
        always queued regardless of confidence].
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
    min_single_vote_conf_overrides = min_single_vote_conf_overrides or {}
    min_two_vote_review_conf_overrides = min_two_vote_review_conf_overrides or {}

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

        two_vote_floor = min_two_vote_review_conf_overrides.get(cls_name, min_two_vote_review_conf)
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
                # vote_count == 2 -- gated. [V7] items failing the
                # auto-accept gate now also check two_vote_floor before
                # queuing -- below it, discarded rather than queued
                # (previously always queued regardless of confidence).
                passes = (two_vote_min_conf(c) >= min_conf
                          and two_vote_pairwise_iou(c) >= min_iou)
                if passes:
                    auto_accepted.append(item)
                elif two_vote_min_conf(c) < two_vote_floor:
                    discarded.append({**item, "reason": "below_two_vote_review_floor"})
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
        floor = min_single_vote_conf_overrides.get(cls_name, min_single_vote_conf)
        for image_key, clusters in full_detections.items():
            for c in clusters:
                if c["cls"] != cls_name or c["vote_count"] != 1:
                    continue
                conf = single_vote_conf(c)
                item = {
                    "source": source_name, "cls": cls_name, "image": image_key,
                    "box": c["box"], "votes": c["votes"], "vote_count": 1,
                }
                if conf < floor:
                    discarded.append({**item, "reason": "below_single_vote_floor"})
                else:
                    queued.append({**item, "reason": "single_model_only"})

    return auto_accepted, queued, discarded


# ---------------------------------------------------------------------
# Label file writing
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
    """Writes this run's auto-accepted boxes into pseudo_root. As of V5,
    pseudo_root has already been fully wiped by clean_output_dir() in
    main() before this is ever called for the first source -- so every
    file this function creates is guaranteed to reflect ONLY this run's
    decisions, never a mix with a previous run's leftovers."""
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
    tree). Read-only against every root passed in -- only rglob() and
    read_text() are called on them, nothing is ever deleted or modified
    here. The only writes in this function target the REAL dataset
    label files under datasets_dir, never the pseudo_labels* trees
    themselves."""
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
    """Renders this run's queued items into review_dir. As of V5,
    review_dir has already been fully wiped by clean_output_dir() in
    main() before this is ever called for the first source -- so the
    only images present after this function runs are the ones that
    correspond to entries actually in THIS run's
    pseudo_labels_review_queue.json. out_dir itself is freshly created
    per source (it didn't exist a moment ago, since review_dir was just
    wiped), so no mkdir(exist_ok=True) risk of silently reusing a stale
    directory either."""
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
    # [PATCH] input file overrides -- point these at resolve_cross_class_
    # conflicts.py's output (cross_reference_*_resolved.json) to consume
    # its cross-class-merged clusters instead of cross_reference_gaps.
    # py's raw per-class output. Defaults are unchanged from before the
    # patch, so omitting these flags is identical to the old behavior.
    p.add_argument("--candidates-file", default="cross_reference_candidates.json",
                    help="[PATCH] Filename under datasets/ to load "
                         "vote_count>=2 candidates from. Point at "
                         "cross_reference_candidates_resolved.json to "
                         "use resolve_cross_class_conflicts.py's merged "
                         "set instead of cross_reference_gaps.py's raw "
                         "output.")
    p.add_argument("--full-file", default="cross_reference_full.json",
                    help="[PATCH] Filename under datasets/ for the full "
                         "per-image vote record. Pair with "
                         "--candidates-file when pointing at resolved "
                         "output.")
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
    p.add_argument("--min-single-vote-conf-per-class", nargs="*", default=[],
                    help="[V6] SOURCE:class:threshold (repeatable). "
                         "Overrides --min-single-vote-conf for one "
                         "specific source+class -- use when one class "
                         "dominates the queue (check each class's "
                         "single_vote_sensitivity in pseudo_labels_"
                         "summary.json first to pick a value). Classes "
                         "not listed keep using --min-single-vote-conf. "
                         "Never applies to --always-review classes.")
    p.add_argument("--min-two-vote-review-conf", type=float, default=0.0,
                    help="[V7] 2-vote clusters that fail the auto-accept "
                         "gate (--min-conf/--min-iou) are DISCARDED "
                         "instead of queued if two_vote_min_conf is "
                         "below this. 0.0 (default) preserves the old "
                         "behavior (queue everything regardless of "
                         "confidence). Check each class's EXISTING "
                         "two_vote_sensitivity table in pseudo_labels_"
                         "summary.json before picking a value.")
    p.add_argument("--min-two-vote-review-conf-per-class", nargs="*", default=[],
                    help="[V7] SOURCE:class:threshold (repeatable). "
                         "Overrides --min-two-vote-review-conf for one "
                         "specific source+class, same shape as "
                         "--min-single-vote-conf-per-class.")
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
    p.add_argument("--keep-existing-outputs", action="store_true",
                    help="[V5] Skip wiping datasets/pseudo_labels/ and "
                         "datasets/pseudo_labels_review/ before this "
                         "run -- restores the pre-V5 additive behavior. "
                         "NOT recommended for normal use (see V5 "
                         "docstring note for why the additive behavior "
                         "was a correctness risk); only pass this if "
                         "you deliberately want two runs' output sitting "
                         "side by side for a manual A/B comparison.")
    return p.parse_args()


def main():
    args = parse_args()
    always_review = parse_always_review(args.always_review)
    single_vote_overrides = parse_per_class_conf(args.min_single_vote_conf_per_class)
    two_vote_review_overrides = parse_per_class_conf(args.min_two_vote_review_conf_per_class)

    datasets_dir = prepare_datasets.DATASETS_DIR
    # [PATCH] was hardcoded to "cross_reference_candidates.json" /
    # "cross_reference_full.json" -- now reads whichever filenames
    # args.candidates_file / args.full_file resolve to (same defaults,
    # so behavior is unchanged unless you pass the new flags).
    print(f"Loading candidates from: {datasets_dir / args.candidates_file}")
    print(f"Loading full vote record from: {datasets_dir / args.full_file}")
    candidates_by_source = load_json(datasets_dir / args.candidates_file)
    full_by_source = load_json(datasets_dir / args.full_file)

    pseudo_root = datasets_dir / "pseudo_labels"
    review_dir = datasets_dir / "pseudo_labels_review"
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    discarded_path = datasets_dir / "pseudo_labels_discarded.json"
    summary_path = datasets_dir / "pseudo_labels_summary.json"

    warn_if_review_in_progress(datasets_dir)

    # V5: guaranteed clean slate for THIS script's own output trees,
    # every run, before a single file is written. Never touches
    # pseudo_labels_reviewed/ or review_progress.json/review_completed.
    # json -- those belong to review_labels.py. See clean_output_dir()'s
    # docstring for the safety guard backing that up.
    print("Cleaning previous run's output (see V5 docstring note)...")
    if args.keep_existing_outputs:
        print("  --keep-existing-outputs passed -- skipping clean, using "
              "additive (pre-V5) behavior. Not recommended for normal use.")
    else:
        clean_output_dir(pseudo_root, "auto-accept label tree")
        clean_output_dir(review_dir, "review images tree")

    all_queued = []
    all_discarded = []
    summary = {"min_conf": args.min_conf, "min_iou": args.min_iou,
               "min_single_vote_conf": args.min_single_vote_conf,
               "min_single_vote_conf_overrides": single_vote_overrides,
               "min_two_vote_review_conf": args.min_two_vote_review_conf,
               "min_two_vote_review_conf_overrides": two_vote_review_overrides,
               "min_always_review_conf": args.min_always_review_conf,
               "always_review": always_review,
               "candidates_file": args.candidates_file,
               "full_file": args.full_file, "sources": {}}

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
            args.min_single_vote_conf, args.min_always_review_conf,
            single_vote_overrides.get(source_name, {}),
            args.min_two_vote_review_conf,
            two_vote_review_overrides.get(source_name, {}))

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

        source_overrides = single_vote_overrides.get(source_name, {})
        source_two_vote_overrides = two_vote_review_overrides.get(source_name, {})
        per_class_summary = {}
        for cls_name in missing_classes:
            cls_candidates_2vote = [c for c in candidates
                                     if c["cls"] == cls_name and c["vote_count"] == 2]
            is_always_review = cls_name in always_review.get(source_name, [])
            # [V6] single_vote_sensitivity: built from EVERY vote_count==1
            # cluster for this class, regardless of the current floor --
            # this is what to check before choosing a --min-single-vote-
            # conf-per-class value. Skipped for always_review classes,
            # which use min_always_review_conf instead.
            cls_singlevote_confs = [] if is_always_review else [
                single_vote_conf(c)
                for clusters in full_detections.values()
                for c in clusters
                if c["cls"] == cls_name and c["vote_count"] == 1
            ]
            effective_floor = (None if is_always_review
                                else source_overrides.get(cls_name, args.min_single_vote_conf))
            effective_two_vote_floor = (None if is_always_review
                                         else source_two_vote_overrides.get(
                                             cls_name, args.min_two_vote_review_conf))
            per_class_summary[cls_name] = {
                "auto_accepted": sum(1 for a in auto_accepted if a["cls"] == cls_name),
                "auto_accepted_unconditional_3_4_vote": sum(
                    1 for a in auto_accepted if a["cls"] == cls_name and a["vote_count"] >= 3),
                "auto_accepted_conditional_2_vote": sum(
                    1 for a in auto_accepted if a["cls"] == cls_name and a["vote_count"] == 2),
                "queued": sum(1 for q in queued if q["cls"] == cls_name),
                "discarded_low_conf": sum(1 for d in discarded if d["cls"] == cls_name),
                "always_review": is_always_review,
                "effective_single_vote_floor": effective_floor,
                "effective_two_vote_review_floor": effective_two_vote_floor,
                "two_vote_sensitivity": sensitivity_table(
                    cls_candidates_2vote, args.sensitivity_thresholds) if cls_candidates_2vote else None,
                "single_vote_sensitivity": confidence_sensitivity_table(
                    cls_singlevote_confs, args.sensitivity_thresholds) if cls_singlevote_confs else None,
            }
            s = per_class_summary[cls_name]
            floor_tag = (f"[ALWAYS REVIEW]" if is_always_review
                         else f"[1v-floor={effective_floor}] [2v-floor={effective_two_vote_floor}]")
            print(f"    {cls_name:15s} auto={s['auto_accepted']:>5}  "
                  f"(3-4vote={s['auto_accepted_unconditional_3_4_vote']:>5} "
                  f"2vote={s['auto_accepted_conditional_2_vote']:>5})  "
                  f"queued={s['queued']:>5}  discarded={s['discarded_low_conf']:>6}  "
                  f"{floor_tag}")

        summary["sources"][source_name] = per_class_summary

    if args.apply:
        reviewed_root = datasets_dir / "pseudo_labels_reviewed"
        merge_pseudo_trees_into_dataset([pseudo_root, reviewed_root], datasets_dir)

    # Whole-file overwrites -- were already safe pre-V5 (mode "w" fully
    # replaces prior contents), noted here for completeness.
    with open(queue_path, "w") as f:
        json.dump(all_queued, f, indent=2, default=str)
    with open(discarded_path, "w") as f:
        json.dump(all_discarded, f, indent=2, default=str)
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)

    print(f"\n{'=' * 70}")
    print(f"Pseudo-labels (dry run tree, freshly regenerated this run): {pseudo_root}/")
    print(f"Review queue ({len(all_queued)} items): {queue_path}")
    print(f"Discarded, below confidence floor ({len(all_discarded)} items): {discarded_path}")
    print(f"Review images (freshly regenerated this run): {review_dir}/<source>/")
    print(f"Summary + sensitivity table: {summary_path}")
    if not args.apply:
        print(f"\nDRY RUN -- real dataset label files were NOT modified. "
              f"Rerun with --apply once you've checked the above.")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()