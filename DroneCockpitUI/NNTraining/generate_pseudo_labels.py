"""
generate_pseudo_labels.py -- Turns cross_reference_gaps.py's vote-based
output into YOLO-format pseudo-label files for the missing classes in
UAVDT/SARD, split into an AUTO-ACCEPT tier (written straight to label
files) and a NEEDS-REVIEW tier (queued with rendered images).

THREE-TIER SPLIT (by vote_count, 1-4 = how many of the four
cross_reference_gaps.py models agreed):
  - AUTO-ACCEPT, vote_count>=3: written unconditionally, no confidence
    floor -- three+ independent models agreeing is strong enough on
    its own.
  - AUTO-ACCEPT, vote_count==2 (conditional): written only if
    min(both confidences) >= --min-conf AND pairwise IoU >= --min-iou.
  - NEEDS REVIEW: vote_count==1 at/above --min-single-vote-conf
    ("single_model_only"); vote_count==2 failing the gate above
    --min-two-vote-review-conf ("below_threshold"); ALWAYS_REVIEW-class
    clusters (default: SARD motorcycle -- classes with little/no
    multi-model corroboration, so single-model hits are queued instead
    of silently dropped) at/above --min-always-review-conf
    ("always_review_class").
  - DISCARDED: anything below the relevant floor above. Recorded (not
    silently dropped) in pseudo_labels_discarded.json with a reason, so
    the floors can be retuned. Pass 0.0 to any floor to queue
    everything instead of discarding (restores the original,
    un-gated behavior).

Per-class sensitivity tables (two_vote_sensitivity /
single_vote_sensitivity in pseudo_labels_summary.json) show pass-count
by threshold for the WHOLE population regardless of the current floor
-- check these before picking --min-*-conf(-per-class) values, no
rerun needed to size a cut. Per-class overrides
(--min-single-vote-conf-per-class / --min-two-vote-review-conf-per-class,
repeatable SOURCE:class:threshold) exist because one class can
dominate the queue by orders of magnitude at a very different
confidence distribution than the rest.

CLEAN-SLATE OUTPUTS: every run (dry run or --apply) wipes
datasets/pseudo_labels/ and datasets/pseudo_labels_review/ before
writing anything (clean_output_dir()) -- both trees are fully
reproducible from the candidate/full JSON + current flags, so there's
never a good reason to keep a stale previous run's files lying around.
Pass --keep-existing-outputs to skip wiping the label/render trees for
a manual A/B comparison only -- note this does NOT make the run fully
additive: the queue/discarded/summary JSON files are always regenerated
from scratch regardless of this flag, so they can end up describing a
different candidate set than whatever old files remain in
pseudo_labels/. clean_output_dir() refuses (raises) if
ever pointed at review_labels.py's manual-review tree
(pseudo_labels_reviewed/, review_progress.json, review_completed.json)
-- this script only ever READS near those (read-only rglob/read_text
at merge time), never writes or deletes there.

--apply MERGE + HOLD-BACK: merges datasets/pseudo_labels/ (this
script's fresh auto-accept tier) and datasets/pseudo_labels_reviewed/
(review_labels.py's output) into the real dataset label files,
appending and skipping duplicate lines. Any image that still has an
undecided review item in EITHER tier has ALL its boxes (from both
trees) held back this run -- not partially merged -- because YOLO
training treats an absent box as confirmed background, and merging
only the decided boxes on a partially-reviewed frame would silently
teach the model "empty" for the undecided region. Rerunning --apply
later picks up newly-completed images automatically; this is the
normal, safe-to-repeat workflow for working through a large backlog.

BACKUP SYNC (--apply only, see sync_reviewed_backups() below): after
the merge above, every real dataset label file under
datasets/<SOURCE>/<split>/labels/ is mirrored byte-for-byte into
datasets/<SOURCE>/<split>/labels_backup_reviewed_<split>/, creating,
overwriting, or (for a backup file whose real counterpart no longer
exists) removing as needed -- a true mirror, not an append-only copy.
This is a SEPARATE tree from
prepare_datasets.py's own labels_backup_original_<split>/ -- that one
is a one-time, pre-remap, native-taxonomy snapshot prepare_datasets.py
depends on staying pristine forever for its idempotent remap logic
(see that module's docstring); this one is a live mirror of whatever
is CURRENTLY in the real label files, unified-taxonomy indices and
all, refreshed after every --apply. It's a full walk-and-compare
against the whole dataset every time, not just whatever this run
touched, so it also repairs any pre-existing drift (e.g. backups that
went stale before this syncing existed). Runnable on its own, without
a candidates/full JSON or a real pseudo-label run, via
--sync-backups-only -- see that flag below.

Every still-pending image (across dry-run or --apply) is also written
to datasets/pending_review_images.json every run, which
prepare_datasets.py reads to exclude those images from train/val/test
ENTIRELY (not just skip merging their boxes) -- see that module's
docstring for why (an incomplete-ground-truth image corrupts val mAP).

PRE-CLEAN STALENESS WARNING: if review_progress.json already has
decisions in it when this runs again, regenerating the queue can
orphan some of that progress (an item a reviewer already decided may
vanish from the new queue -- harmless but stale). This script warns
with a count before doing anything else; tune thresholds BEFORE
starting manual review to avoid it.

REVIEW QUEUE IDENTITY (see compute_pending_review_images() /
FINGERPRINT_PATH_NAME): review_completed.json tracks completion per
IMAGE, not per queue-item, so on its own it can't tell "this image's
queue is unchanged since it was reviewed" from "this image's queue
changed after a threshold tweak, but happens to still be marked
complete." This script closes that gap itself: it fingerprints each
image's current queue contents and persists the fingerprint that was
in force the last time that image was genuinely complete, in
datasets/review_fingerprints_at_completion.json. If a later run finds
review_completed.json says "done" but the current fingerprint doesn't
match the recorded one, it re-treats that image as pending (with a
warning) instead of trusting review_completed.json blindly -- so a
newly-surfaced box from a threshold change can never sneak into
--apply unreviewed.

USAGE:
    # Dry run at default floors (0.40 single-vote / 0.30 always-review).
    python generate_pseudo_labels.py

    # Or consuming resolve_cross_class_conflicts.py's merged output:
    python generate_pseudo_labels.py \\
        --candidates-file cross_reference_candidates_resolved.json \\
        --full-file cross_reference_full_resolved.json

    # Check pseudo_labels_summary.json's per-class counts + sensitivity
    # tables, tune --min-conf/--min-iou/--min-single-vote-conf/
    # --min-always-review-conf, rerun if needed (every rerun is a full
    # clean regen -- do this BEFORE manual review starts).

    # Build/update the trainable dataset from whatever's fully
    # reviewed so far -- safe to rerun anytime as more finishes review:
    python generate_pseudo_labels.py \\
        --candidates-file cross_reference_candidates_resolved.json \\
        --full-file cross_reference_full_resolved.json \\
        --min-single-vote-conf 0.45 --min-always-review-conf 0.0 --apply

    # Just repair/refresh labels_backup_reviewed_<split>/ against
    # whatever's currently in the real dataset label files -- no
    # candidates/full JSON needed, doesn't touch pseudo_labels/,
    # pseudo_labels_review/, or the review queue/discarded/summary
    # JSON files at all:
    python generate_pseudo_labels.py --sync-backups-only
"""

import argparse
import hashlib
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

# Kept byte-for-byte synced with the real dataset label files by
# sync_reviewed_backups() below, refreshed after every --apply run (and
# on demand via --sync-backups-only). Deliberately a DIFFERENT
# directory than prepare_datasets.py's own BACKUP_DIRNAME
# ("labels_backup_original_<split>") -- that one is a ONE-TIME,
# pre-remap, native-class-taxonomy snapshot prepare_datasets.py's
# idempotent remap logic depends on staying pristine forever (see that
# module's _restore_from_backup docstring); mixing already-unified-
# index merged pseudo-labels into it would corrupt a future taxonomy-
# triggered remap. This tree is a live, always-current mirror of
# what's actually in the real label files right now -- including
# everything --apply has ever merged in, from both the auto-accept and
# reviewed tiers -- so a disk problem or accidental hand-edit has a
# trustworthy, up-to-date fallback that isn't stuck at pre-pseudo-
# labeling state the way labels_backup_original_<split>/ is by design.
REVIEWED_BACKUP_DIRNAME = "labels_backup_reviewed"

# Filenames marking a directory as review_labels.py's manual-review
# tree -- clean_output_dir() refuses to touch anything matching these,
# as a second independent guard on top of "we simply never call it on
# those paths".
PROTECTED_DIR_NAME = "pseudo_labels_reviewed"
PROTECTED_SIBLING_FILES = ("review_progress.json", "review_completed.json")

# Written/read by this script only (review_labels.py never touches it):
# {image: fingerprint} for every image that was genuinely complete (per
# review_completed.json) as of the run that last confirmed it. Lets a
# later run detect "this image is in review_completed.json, but the
# review queue I'd generate for it today doesn't match what was
# actually reviewed" -- see compute_pending_review_images() and the
# module docstring's PRE-CLEAN STALENESS note.
FINGERPRINT_PATH_NAME = "review_fingerprints_at_completion.json"

# (fix) Two label lines for the SAME object rarely come out byte-
# identical -- review_labels.py's own coordinates, a hand-nudged/
# resized box, or just independent rounding to 6 decimal places can
# all put two annotations of one physical object a fraction of a pixel
# apart. The old dedup here compared lines as exact strings, so any of
# that made both copies look "different" and get merged in side by
# side -- the actual mechanism behind objects ending up double-labeled
# in the real dataset after --apply. Two boxes of the same class
# overlapping at least this much (IoU, in normalized YOLO space) are
# now treated as the same object instead.
DUPLICATE_IOU_THRESHOLD = 0.6


def _parse_yolo_line(line: str):
    """Parses one YOLO label line into (cls_idx, [x1,y1,x2,y2]) in
    normalized (0-1) coordinates, or None if the line isn't well-formed
    enough to compare geometrically (caller falls back to treating it
    as opaque text in that case)."""
    parts = line.split()
    if len(parts) < 5:
        return None
    try:
        cls_idx = int(parts[0])
        xc, yc, w, h = (float(v) for v in parts[1:5])
    except ValueError:
        return None
    return cls_idx, [xc - w / 2, yc - h / 2, xc + w / 2, yc + h / 2]


def _box_iou(a: list[float], b: list[float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return (inter / union) if union > 0 else 0.0


def _is_duplicate_line(line: str, parsed, against: list[tuple]) -> bool:
    """True if `line` (already parsed into `parsed`, possibly None)
    represents the same object as any entry already in `against` --
    a list of (line_text, parsed_or_None) pairs. Same class + IoU at or
    above DUPLICATE_IOU_THRESHOLD counts as the same object; a line
    that fails to parse falls back to exact-text comparison against
    other unparseable lines only, rather than being silently treated as
    always-unique or always-duplicate."""
    if parsed is not None:
        cls_idx, box = parsed
        for _, other_parsed in against:
            if other_parsed is None:
                continue
            other_cls, other_box = other_parsed
            if other_cls == cls_idx and _box_iou(box, other_box) >= DUPLICATE_IOU_THRESHOLD:
                return True
        return False
    return any(other_line == line for other_line, other_parsed in against if other_parsed is None)


# ---------------------------------------------------------------------
# Clean-slate output helper
# ---------------------------------------------------------------------

def clean_output_dir(path: Path, label: str) -> None:
    """Wipes `path` completely (if it exists) and recreates it empty,
    so this run's OWN output trees (pseudo_labels/, pseudo_labels_
    review/) always start from zero. Refuses (raises) if `path` looks
    like review_labels.py's manual-review directory -- see module
    docstring."""
    if path.name == PROTECTED_DIR_NAME:
        raise RuntimeError(
            f"Refusing to clean {path} -- its name matches the protected "
            f"manual-review directory ({PROTECTED_DIR_NAME}).")
    if path.exists():
        for sibling in PROTECTED_SIBLING_FILES:
            if (path / sibling).exists():
                raise RuntimeError(
                    f"Refusing to clean {path} -- it directly contains "
                    f"{sibling}, a manual-review marker file.")

    if path.exists():
        n_existing = sum(1 for _ in path.rglob("*") if _.is_file())
        shutil.rmtree(path)
        print(f"  [clean] removed {n_existing} existing file(s) from "
              f"{label} ({path})")
    else:
        print(f"  [clean] {label} ({path}) did not exist yet -- nothing to remove")
    path.mkdir(parents=True, exist_ok=True)


def warn_if_review_in_progress(datasets_dir: Path) -> None:
    """Informational only (never blocks or touches a file) -- see
    module docstring's PRE-CLEAN STALENESS note."""
    progress_path = datasets_dir / "review_progress.json"
    if not progress_path.exists():
        return
    try:
        with open(progress_path) as f:
            progress = json.load(f)
    except (json.JSONDecodeError, OSError):
        return
    n_decisions = sum(1 for k in progress if not k.startswith("__"))
    if n_decisions == 0:
        return
    print(f"\n  {'!' * 66}")
    print(f"  WARNING: {progress_path} already has {n_decisions} manual "
          f"review decision(s) recorded.")
    print(f"  Regenerating the queue now may leave some decisions "
          f"referring to items no longer in the new queue (harmless but "
          f"stale). Prefer tuning thresholds BEFORE manual review starts.")
    print(f"  Your decisions live in pseudo_labels_reviewed/ and this "
          f"file -- neither is touched by this script.")
    print(f"  {'!' * 66}\n")


# ---------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------

def image_path_to_label_path(img_path: Path) -> Path:
    """root/<split>/images/xxx.jpg -> root/<split>/labels/xxx.txt.
    Delegates to prepare_datasets._find_last_images_index() -- the same
    shared, case-insensitive lookup prepare_datasets.py's own
    _label_path_for_image() uses -- rather than a second independent
    copy of the same scan."""
    idx = prepare_datasets._find_last_images_index(img_path.parts)
    if idx is None:
        raise ValueError(
            f"Could not find an 'images' path component in {img_path}.")
    parts = img_path.parts
    label_parts = parts[:idx] + ("labels",) + parts[idx + 1:]
    return Path(*label_parts).with_suffix(".txt")


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
    """--min-*-conf-per-class SOURCE:class:threshold -> {source: {cls: threshold}}."""
    out: dict[str, dict[str, float]] = {}
    for triple in triples:
        parts = triple.split(":")
        if len(parts) != 3:
            raise SystemExit(
                f"expects SOURCE:class:threshold, got '{triple}'")
        source, cls, conf_str = parts
        try:
            conf = float(conf_str)
        except ValueError:
            raise SystemExit(f"'{conf_str}' in '{triple}' is not a number.")
        out.setdefault(source, {})[cls] = conf
    return out


def load_json(path: Path):
    if not path.exists():
        raise SystemExit(
            f"{path} not found -- run cross_reference_gaps.py first (or "
            f"resolve_cross_class_conflicts.py if using --candidates-file/"
            f"--full-file).")
    with open(path) as f:
        return json.load(f)


def two_vote_min_conf(item: dict) -> float:
    """min of the two present confidences in a vote_count==2 cluster."""
    confs = [v for v in item["votes"].values() if v is not None]
    return min(confs) if confs else 0.0


def two_vote_pairwise_iou(item: dict) -> float:
    """Pairwise IoU between the two contributing models, from
    member_ious (one entry is always 1.0, self-vs-self)."""
    ious = [v for m, v in item["member_ious"].items()
            if v is not None and v < 1.0 - 1e-9]
    if ious:
        return min(ious)
    present = [v for v in item["member_ious"].values() if v is not None]
    return min(present) if present else 0.0


def single_vote_conf(cluster: dict) -> float:
    """The one present confidence in a vote_count==1 cluster."""
    confs = [v for v in cluster["votes"].values() if v is not None]
    return confs[0] if confs else 0.0


def confidence_sensitivity_table(confs: list[float], thresholds: list[float]) -> dict:
    """How many of `confs` would pass at each threshold, plus basic
    stats -- used for both the 1-vote and 2-vote sensitivity tables."""
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
    """Splits one source's clusters into (auto_accepted, queued,
    discarded) per the module docstring's THREE-TIER SPLIT. Per-class
    override dicts take precedence over the matching global floor;
    always_review_classes only ever respond to min_always_review_conf."""
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
            # Pull every vote_count for this class from the full
            # per-image record (including vote_count==1 -- may be the
            # only evidence that exists for this class/source pair).
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
                auto_accepted.append(item)
            else:
                passes = (two_vote_min_conf(c) >= min_conf
                          and two_vote_pairwise_iou(c) >= min_iou)
                if passes:
                    auto_accepted.append(item)
                elif two_vote_min_conf(c) < two_vote_floor:
                    discarded.append({**item, "reason": "below_two_vote_review_floor"})
                else:
                    queued.append({**item, "reason": "below_threshold"})

    # vote_count==1 clusters for non-always-review classes never reach
    # cross_reference_candidates.json (only vote_count>=2 does), so pull
    # them from full_detections explicitly.
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
    """Raises ValueError (rather than silently writing a bad label) for
    an unknown unified class or a degenerate/out-of-bounds box -- this
    is the last step before a box can reach a real dataset label file,
    so upstream bugs should surface here instead of downstream in
    training. Callers should catch and record, not crash the run --
    see write_auto_accept_labels()."""
    if cls_name not in UNIFIED_CLASSES:
        raise ValueError(
            f"unknown unified class '{cls_name}' (known: {UNIFIED_CLASSES})")
    x1, y1, x2, y2 = box
    if x2 <= x1 or y2 <= y1:
        raise ValueError(
            f"degenerate box ({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}) "
            f"-- x2<=x1 or y2<=y1")
    if x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h:
        raise ValueError(
            f"box ({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}) falls outside "
            f"image bounds ({img_w}x{img_h})")
    cls_idx = UNIFIED_CLASSES.index(cls_name)
    xc = ((x1 + x2) / 2) / img_w
    yc = ((y1 + y2) / 2) / img_h
    w = (x2 - x1) / img_w
    h = (y2 - y1) / img_h
    return f"{cls_idx} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}"


def write_auto_accept_labels(auto_accepted: list[dict], source_name: str,
                              pseudo_root: Path) -> tuple[dict[Path, list[str]], list[dict]]:
    """Writes this run's auto-accepted boxes into pseudo_root, already
    wiped clean by clean_output_dir() before the first source runs.
    Lines are deduplicated per label file before writing (geometric
    dedup -- same class + IoU >= DUPLICATE_IOU_THRESHOLD, same rule
    --apply's merge uses, see merge_pseudo_trees_into_dataset) so a
    repeated cluster can't land in the dataset twice even when two
    near-identical detections for it round to slightly different
    coordinates. Returns (by_label_path, rejected) -- `rejected` holds
    any item whose box/class failed validation in box_to_yolo_line();
    these are skipped, not written, and should be surfaced to the
    caller rather than silently dropped."""
    by_label_path: dict[Path, list[str]] = {}
    rejected: list[dict] = []
    for item in auto_accepted:
        img_path = Path(item["image"])
        real_label_path = image_path_to_label_path(img_path)
        pseudo_label_path = mirror_under_pseudo_root(real_label_path, source_name, pseudo_root)
        w, h = get_image_dims(item["image"])
        try:
            line = box_to_yolo_line(item["cls"], item["box"], w, h)
        except ValueError as e:
            rejected.append({**item, "reason": f"invalid_box: {e}"})
            continue
        by_label_path.setdefault(pseudo_label_path, []).append(line)

    for label_path, lines in by_label_path.items():
        deduped: list[tuple] = []  # (line_text, parsed_or_None), order-preserving
        for l in lines:
            parsed = _parse_yolo_line(l)
            if _is_duplicate_line(l, parsed, deduped):
                continue
            deduped.append((l, parsed))
        deduped_lines = [l for l, _ in deduped]
        by_label_path[label_path] = deduped_lines
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("\n".join(deduped_lines) + "\n")

    if rejected:
        print(f"  [warn] {len(rejected)} auto-accepted box(es) for "
              f"{source_name} failed validation (degenerate/out-of-bounds "
              f"box or unknown class) -- skipped, NOT written. See "
              f"pseudo_labels_invalid_boxes.json.")

    return by_label_path, rejected


# ---------------------------------------------------------------------
# Pending-review gating for --apply
# ---------------------------------------------------------------------

def compute_image_fingerprint(items: list[dict]) -> str:
    """Stable fingerprint of one image's current set of queued items,
    so a later run can tell whether the queue for that image has
    changed since it was last confirmed complete (e.g. a threshold
    change surfaced a new box on an image a reviewer already finished).
    Box coords are rounded to absorb float-repr noise; order doesn't
    matter since the list is sorted before hashing."""
    normalized = sorted(
        (it["cls"], tuple(round(v, 2) for v in it["box"]), it["reason"])
        for it in items
    )
    payload = json.dumps(normalized, sort_keys=True).encode()
    return hashlib.sha256(payload).hexdigest()[:16]


def compute_pending_review_images(all_queued: list[dict],
                                   datasets_dir: Path) -> set[str]:
    """Images with at least one item in THIS run's needs-review queue
    that either (a) isn't yet in review_completed.json (review_labels.py's
    own "every box on this image is decided" signal), or (b) IS marked
    complete there but the current queue for that image no longer
    matches the queue that was actually reviewed -- detected via
    FINGERPRINT_PATH_NAME, which this script (not review_labels.py)
    maintains across runs.

    Why (b) matters: review_completed.json tracks completion per IMAGE,
    not per queue-item. If thresholds change between review sessions,
    an image that was fully reviewed under the old queue can pick up a
    brand-new item under the new queue (e.g. a previously-discarded
    detection now crosses the floor) while still reading as "complete"
    -- which would let --apply merge that new, never-reviewed box.
    Comparing fingerprints closes that gap without needing any change
    to review_labels.py.

    An image that was only ever auto-accepted (never queued) is never
    in the returned set."""
    completed_path = datasets_dir / "review_completed.json"
    completed = set()
    if completed_path.exists():
        with open(completed_path) as f:
            completed = set(json.load(f))

    by_image: dict[str, list[dict]] = {}
    for item in all_queued:
        by_image.setdefault(item["image"], []).append(item)
    queued_images = set(by_image.keys())
    current_fingerprints = {img: compute_image_fingerprint(items)
                             for img, items in by_image.items()}

    fp_path = datasets_dir / FINGERPRINT_PATH_NAME
    recorded_fingerprints: dict[str, str] = {}
    if fp_path.exists():
        try:
            with open(fp_path) as f:
                recorded_fingerprints = json.load(f)
        except (json.JSONDecodeError, OSError):
            recorded_fingerprints = {}

    naively_completed = queued_images & completed
    stale_completed = {
        img for img in naively_completed
        if recorded_fingerprints.get(img) != current_fingerprints[img]
    }

    if stale_completed:
        print(f"\n  {'!' * 66}")
        print(f"  WARNING: {len(stale_completed)} image(s) are marked "
              f"complete in review_completed.json, but this run's queue "
              f"for them doesn't match the fingerprint recorded when "
              f"they were last confirmed complete (thresholds likely "
              f"changed since). Treating as PENDING again -- these will "
              f"NOT be merged by --apply until re-reviewed:")
        for img in sorted(stale_completed)[:20]:
            print(f"    {img}")
        if len(stale_completed) > 20:
            print(f"    ... and {len(stale_completed) - 20} more")
        print(f"  {'!' * 66}\n")

    pending = (queued_images - completed) | stale_completed

    # Record the fingerprint currently "in force" for every genuinely
    # (non-stale) completed image, so a future threshold change can be
    # detected against it. Images no longer queued at all this run
    # (now auto-accepted, or discarded outright) are dropped from the
    # file -- nothing left to compare against.
    updated_fingerprints = {
        img: current_fingerprints[img]
        for img in queued_images
        if img in completed and img not in stale_completed
    }
    with open(fp_path, "w") as f:
        json.dump(updated_fingerprints, f, indent=2)

    return pending


def pending_images_to_label_paths(pending_images: set[str]) -> set[Path]:
    """Maps each still-pending image to the real dataset label path it
    would eventually land in, so merges can skip it entirely."""
    paths = set()
    for image_key in pending_images:
        try:
            paths.add(image_path_to_label_path(Path(image_key)))
        except ValueError:
            continue
    return paths


def merge_pseudo_trees_into_dataset(roots: list[Path], datasets_dir: Path,
                                     pending_label_paths: set[Path] | None = None):
    """Merges pseudo_labels/ (fresh this run) and pseudo_labels_reviewed/
    (review_labels.py's output, read-only) into the real dataset label
    files, appending and skipping GEOMETRIC duplicates -- same class +
    IoU >= DUPLICATE_IOU_THRESHOLD against a line already in the file,
    not just byte-identical text (see DUPLICATE_IOU_THRESHOLD's comment
    for why exact-string comparison let the same object get merged in
    twice). Any .txt from either root landing on a pending_label_paths
    destination is skipped entirely this run -- see module docstring's
    hold-back note. Only ever writes to the real dataset label files,
    never to either source tree."""
    pending_label_paths = pending_label_paths or set()
    written, skipped_dupe, skipped_pending = 0, 0, 0
    per_root_counts = {}
    pending_paths_seen = set()

    for root in roots:
        if not root.exists():
            continue
        root_written = 0
        for txt_path in root.rglob("*.txt"):
            rel = txt_path.relative_to(root)
            real_label_path = datasets_dir / rel

            if real_label_path in pending_label_paths:
                n_lines = sum(1 for l in txt_path.read_text().splitlines() if l.strip())
                skipped_pending += n_lines
                pending_paths_seen.add(real_label_path)
                continue

            new_lines = [l.strip() for l in txt_path.read_text().splitlines() if l.strip()]
            if not new_lines:
                continue

            existing = []
            if real_label_path.exists():
                for l in real_label_path.read_text().splitlines():
                    l = l.strip()
                    if l:
                        existing.append((l, _parse_yolo_line(l)))
            # `existing` is appended to as we go (not just checked against
            # its pre-loop snapshot) so a line repeated within
            # new_lines itself -- e.g. the same box present in both
            # pseudo_labels/ and pseudo_labels_reviewed/ for this image,
            # or two near-identical boxes for one object in the same
            # tree -- is only added once, not once per occurrence.
            to_add = []
            for l in new_lines:
                parsed = _parse_yolo_line(l)
                if _is_duplicate_line(l, parsed, existing):
                    continue
                to_add.append(l)
                existing.append((l, parsed))
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
    if pending_label_paths:
        print(f"  Held back {skipped_pending} box(es) across "
              f"{len(pending_paths_seen)} image(s) that still have an "
              f"undecided review item -- rerun --apply once they're "
              f"reviewed to pick them up.")


def write_pending_review_images(pending_images: set[str], datasets_dir: Path) -> Path:
    """Writes datasets/pending_review_images.json (resolved absolute
    path strings) every run so prepare_datasets.py can exclude these
    images from train/val/test entirely."""
    path = datasets_dir / "pending_review_images.json"
    resolved = sorted(str(Path(p).resolve()) for p in pending_images)
    with open(path, "w") as f:
        json.dump(resolved, f, indent=2)
    return path


# ---------------------------------------------------------------------
# Reviewed-backup sync (see module docstring's BACKUP SYNC section, and
# REVIEWED_BACKUP_DIRNAME's comment above, for why this is a separate
# tree from prepare_datasets.py's labels_backup_original_<split>/).
# ---------------------------------------------------------------------

def sync_reviewed_backups(datasets_dir: Path,
                           backup_dirname: str = REVIEWED_BACKUP_DIRNAME) -> dict:
    """Walks every REAL dataset label file under
    datasets_dir/<SOURCE>/<split>/labels/ (Roboflow-style layout only --
    SARD/UAVDT/external; VisDrone/xView never go through pseudo-labeling
    and use a different directory layout besides, so they're naturally
    excluded by the glob patterns below) and makes sure its counterpart
    under datasets_dir/<SOURCE>/<split>/<backup_dirname>_<split>/
    matches it byte-for-byte, creating or overwriting as needed.

    This is a full walk-and-compare against the CURRENT state of every
    real label file, not just whatever this run's --apply touched -- so
    it also retroactively repairs any backup that went stale before
    this syncing existed (exactly the gap that produced stale/missing
    labels_backup_reviewed_<split>/ content if this is being run for
    the first time against an already-merged dataset). Safe and
    idempotent to run any time; called automatically at the end of
    --apply (see main()), and also runnable standalone via
    --sync-backups-only without needing candidates/full JSON or a real
    pseudo-label run at all.

    Explicitly skips anything under a top-level "pseudo_labels*",
    "label_gap_review", or "review_backups" directory -- those are
    staging/output trees this script or review_labels.py already own,
    not the real dataset, and could otherwise get matched by the same
    "<split>/labels" directory-name pattern the glob below looks for.

    Also removes any backup .txt file whose real counterpart no longer
    exists (e.g. an image was excluded/removed from the dataset since
    the last sync) -- without this, the backup tree only ever grows and
    "mirror" would be inaccurate.

    Returns a stats dict: {"checked", "created", "updated",
    "already_synced", "deleted"}."""
    stats = {"checked": 0, "created": 0, "updated": 0,
              "already_synced": 0, "deleted": 0}
    excluded_tops = ("label_gap_review", "review_backups")

    # Roboflow-style layout is datasets/<SOURCE>/<split>/labels/ (one
    # level) for the top-level SARD/UAVDT, and datasets/external/<name>/
    # <split>/labels/ (two levels) for anything under datasets/external/
    # -- both patterns are globbed for and de-duplicated via the set.
    candidates = set(datasets_dir.glob("*/*/labels")) | set(datasets_dir.glob("*/*/*/labels"))

    for labels_dir in sorted(candidates):
        if not labels_dir.is_dir():
            continue
        top = labels_dir.relative_to(datasets_dir).parts[0]
        if top.startswith("pseudo_labels") or top in excluded_tops:
            continue

        split_name = labels_dir.parent.name
        backup_dir = labels_dir.parent / f"{backup_dirname}_{split_name}"

        real_names = set()
        for real_file in labels_dir.glob("*.txt"):
            real_names.add(real_file.name)
            stats["checked"] += 1
            backup_file = backup_dir / real_file.name
            real_content = real_file.read_text()
            if not backup_file.exists():
                backup_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(real_file, backup_file)
                stats["created"] += 1
            elif backup_file.read_text() != real_content:
                shutil.copy2(real_file, backup_file)
                stats["updated"] += 1
            else:
                stats["already_synced"] += 1

        if backup_dir.exists():
            for backup_file in backup_dir.glob("*.txt"):
                if backup_file.name not in real_names:
                    backup_file.unlink()
                    stats["deleted"] += 1

    return stats


# ---------------------------------------------------------------------
# Review queue images
# ---------------------------------------------------------------------

def votes_str(item: dict) -> str:
    parts = [f"{MODEL_ABBREV[m]}{item['votes'][m]:.2f}"
             for m in MODEL_TAGS if item["votes"].get(m) is not None]
    return f"v{item['vote_count']}/4 " + "/".join(parts)


def save_review_images(queued: list[dict], source_name: str, review_dir: Path,
                        max_images: int):
    """Renders this run's queued items into review_dir, already wiped
    clean for this source before this runs."""
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
    p.add_argument("--candidates-file", default="cross_reference_candidates.json",
                    help="Filename under datasets/ to load vote_count>=2 "
                         "candidates from. Point at "
                         "cross_reference_candidates_resolved.json to use "
                         "resolve_cross_class_conflicts.py's merged set.")
    p.add_argument("--full-file", default="cross_reference_full.json",
                    help="Filename under datasets/ for the full per-image "
                         "vote record. Pair with --candidates-file.")
    p.add_argument("--min-conf", type=float, default=0.40,
                    help="2-vote tier only: auto-accept requires "
                         "min(the two confidences) >= this. 3/4-vote "
                         "tier is unconditional.")
    p.add_argument("--min-iou", type=float, default=0.60,
                    help="2-vote tier only: auto-accept requires pairwise "
                         "IoU between the two models' boxes >= this.")
    p.add_argument("--min-single-vote-conf", type=float, default=0.40,
                    help="vote_count==1, normal classes: below this "
                         "confidence, discard instead of queue. 0.0 "
                         "queues everything.")
    p.add_argument("--min-always-review-conf", type=float, default=0.30,
                    help="ALWAYS_REVIEW classes: below this, discard "
                         "instead of queue. Deliberately lower than "
                         "--min-single-vote-conf.")
    p.add_argument("--min-single-vote-conf-per-class", nargs="*", default=[],
                    help="SOURCE:class:threshold (repeatable). Overrides "
                         "--min-single-vote-conf for one class -- check "
                         "single_vote_sensitivity in pseudo_labels_"
                         "summary.json first. Never applies to "
                         "--always-review classes.")
    p.add_argument("--min-two-vote-review-conf", type=float, default=0.0,
                    help="2-vote clusters failing the auto-accept gate: "
                         "below this, discard instead of queue. 0.0 "
                         "(default) queues everything.")
    p.add_argument("--min-two-vote-review-conf-per-class", nargs="*", default=[],
                    help="SOURCE:class:threshold (repeatable), same shape "
                         "as --min-single-vote-conf-per-class.")
    p.add_argument("--always-review", nargs="*", default=["SARD:motorcycle"],
                    help="SOURCE:class pairs never auto-accepted regardless "
                         "of vote_count, always queued (subject to "
                         "--min-always-review-conf).")
    p.add_argument("--sensitivity-thresholds", type=float, nargs="+",
                    default=[0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60])
    p.add_argument("--max-review-images", type=int, default=300,
                    help="Cap on rendered review images per source (all "
                         "queued items are still in the JSON regardless).")
    p.add_argument("--apply", action="store_true",
                    help="Actually append auto-accepted + fully-reviewed "
                         "labels to the real dataset label files. Without "
                         "this, output only goes to datasets/pseudo_labels/ "
                         "(dry run). See module docstring's hold-back note. "
                         "Also syncs labels_backup_reviewed_<split>/ "
                         "against the real label files afterward -- see "
                         "sync_reviewed_backups().")
    p.add_argument("--keep-existing-outputs", action="store_true",
                    help="Skip wiping pseudo_labels/ and pseudo_labels_"
                         "review/ before this run, so old label/render "
                         "files from a prior run may remain alongside "
                         "this run's. NOTE: this does NOT make the run "
                         "fully additive -- pseudo_labels_review_queue.json, "
                         "pseudo_labels_discarded.json, and pseudo_labels_"
                         "summary.json are still fully overwritten every "
                         "run regardless of this flag, so those can end up "
                         "describing a different set of candidates than "
                         "what's sitting in pseudo_labels/. Not recommended "
                         "for normal use -- only for a manual A/B "
                         "comparison of label trees.")
    p.add_argument("--sync-backups-only", action="store_true",
                    help="Skip everything else -- candidates/full JSON "
                         "aren't even loaded -- and just walk the real "
                         "dataset label files, bringing "
                         "labels_backup_reviewed_<split>/ up to date "
                         "against whatever's currently there, then exit. "
                         "Also repairs any pre-existing drift (backups "
                         "that went stale before this syncing existed). "
                         "See sync_reviewed_backups().")
    return p.parse_args()


def main():
    args = parse_args()

    # --sync-backups-only short-circuits everything else: no candidates/
    # full JSON needed, doesn't touch pseudo_labels/, pseudo_labels_
    # review/, or the review queue/discarded/summary JSON files at all
    # -- just brings labels_backup_reviewed_<split>/ in line with the
    # real dataset label files as they stand right now.
    if args.sync_backups_only:
        datasets_dir = prepare_datasets.DATASETS_DIR
        print(f"Syncing {REVIEWED_BACKUP_DIRNAME}_<split>/ backups against "
              f"the current real dataset label files under {datasets_dir}...")
        stats = sync_reviewed_backups(datasets_dir)
        print(f"  Checked {stats['checked']} label file(s): "
              f"{stats['created']} created, "
              f"{stats['updated']} brought up to date, "
              f"{stats['already_synced']} already in sync, "
              f"{stats['deleted']} stale backup(s) removed.")
        return

    always_review = parse_always_review(args.always_review)
    single_vote_overrides = parse_per_class_conf(args.min_single_vote_conf_per_class)
    two_vote_review_overrides = parse_per_class_conf(args.min_two_vote_review_conf_per_class)

    datasets_dir = prepare_datasets.DATASETS_DIR
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

    print("Cleaning previous run's output...")
    if args.keep_existing_outputs:
        print("  --keep-existing-outputs passed -- skipping clean, using "
              "additive behavior. Not recommended for normal use.")
    else:
        clean_output_dir(pseudo_root, "auto-accept label tree")
        clean_output_dir(review_dir, "review images tree")

    all_queued = []
    all_discarded = []
    all_invalid_boxes = []
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

        by_label_path, rejected_boxes = write_auto_accept_labels(
            auto_accepted, source_name, pseudo_root)
        n_unconditional = sum(1 for a in auto_accepted if a["vote_count"] >= 3)
        n_conditional = len(auto_accepted) - n_unconditional
        print(f"  Auto-accepted: {len(auto_accepted)} boxes across "
              f"{len(by_label_path)} images -> {pseudo_root / source_name}/ "
              f"({n_unconditional} at 3-4/4 votes, {n_conditional} at "
              f"2/4 votes past threshold"
              + (f", {len(rejected_boxes)} REJECTED on validation" if rejected_boxes else "")
              + ")")
        print(f"  Queued for review: {len(queued)}   "
              f"Discarded (below confidence floor): {len(discarded)}")

        save_review_images(queued, source_name, review_dir, args.max_review_images)
        all_queued.extend(queued)
        all_discarded.extend(discarded)
        all_invalid_boxes.extend(rejected_boxes)

        source_overrides = single_vote_overrides.get(source_name, {})
        source_two_vote_overrides = two_vote_review_overrides.get(source_name, {})
        per_class_summary = {}
        for cls_name in missing_classes:
            cls_candidates_2vote = [c for c in candidates
                                     if c["cls"] == cls_name and c["vote_count"] == 2]
            is_always_review = cls_name in always_review.get(source_name, [])
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

    pending_images = compute_pending_review_images(all_queued, datasets_dir)
    pending_path = write_pending_review_images(pending_images, datasets_dir)

    if args.apply:
        reviewed_root = datasets_dir / "pseudo_labels_reviewed"
        pending_label_paths = pending_images_to_label_paths(pending_images)
        if pending_images:
            by_source = {}
            for img in pending_images:
                src = next((it["source"] for it in all_queued if it["image"] == img), "?")
                by_source[src] = by_source.get(src, 0) + 1
            print(f"\n  {len(pending_images)} image(s) still have an "
                  f"undecided review item -- none of their boxes (from "
                  f"either tier) will be merged this run:")
            for src, n in sorted(by_source.items()):
                print(f"    {src:10s} {n}")
        else:
            print(f"\n  No pending review items outstanding -- every "
                  f"queued image has already been fully reviewed.")

        merge_pseudo_trees_into_dataset([pseudo_root, reviewed_root], datasets_dir,
                                          pending_label_paths=pending_label_paths)

        print(f"\n  Syncing {REVIEWED_BACKUP_DIRNAME}_<split>/ backups against "
              f"the real label files (also repairs any older drift from "
              f"before this syncing existed)...")
        backup_stats = sync_reviewed_backups(datasets_dir)
        print(f"    Checked {backup_stats['checked']} label file(s): "
              f"{backup_stats['created']} created, "
              f"{backup_stats['updated']} brought up to date, "
              f"{backup_stats['already_synced']} already in sync, "
              f"{backup_stats['deleted']} stale backup(s) removed.")

    invalid_boxes_path = datasets_dir / "pseudo_labels_invalid_boxes.json"
    with open(queue_path, "w") as f:
        json.dump(all_queued, f, indent=2, default=str)
    with open(discarded_path, "w") as f:
        json.dump(all_discarded, f, indent=2, default=str)
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)
    with open(invalid_boxes_path, "w") as f:
        json.dump(all_invalid_boxes, f, indent=2, default=str)

    print(f"\n{'=' * 70}")
    print(f"Pseudo-labels (dry run tree, freshly regenerated this run): {pseudo_root}/")
    print(f"Review queue ({len(all_queued)} items): {queue_path}")
    print(f"Discarded, below confidence floor ({len(all_discarded)} items): {discarded_path}")
    print(f"Review images (freshly regenerated this run): {review_dir}/<source>/")
    print(f"Summary + sensitivity table: {summary_path}")
    print(f"Pending-review images ({len(pending_images)}, excluded from "
          f"train/val/test entirely by prepare_datasets.py): {pending_path}")
    if all_invalid_boxes:
        print(f"Invalid boxes REJECTED, not written ({len(all_invalid_boxes)} "
              f"items -- degenerate/out-of-bounds box or unknown class): "
              f"{invalid_boxes_path}")
    if args.apply:
        print(f"Reviewed-backup mirror (kept in sync with the real label "
              f"files): datasets/<SOURCE>/<split>/{REVIEWED_BACKUP_DIRNAME}_<split>/")
    if not args.apply:
        print(f"\nDRY RUN -- real dataset label files were NOT modified. "
              f"Rerun with --apply once you've checked the above.")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()