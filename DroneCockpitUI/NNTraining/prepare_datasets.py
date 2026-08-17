"""
prepare_datasets.py — Multi-dataset prep for Project R.A.D detection
=======================================================================
Run automatically by train.py before every training run (safe/fast to
call every time -- each step checks whether it's already done). Can also
be run standalone to inspect what's currently included:

    python prepare_datasets.py

What it does:
  1. Ensures VisDrone2019-DET is downloaded (auto), then remaps its 10
     original classes into the shared taxonomy from class_map.py.
  2. If datasets/xView/ exists, triggers xView conversion+remap.
     CURRENTLY INACTIVE BY PROJECT DECISION -- see class_map.py's module
     docstring for why.
  3. UAVDT (datasets/UAVDT/) and SARD (datasets/SARD/) -- both first-class
     top-level dataset folders, same convention as VisDrone: a data.yaml
     (used only to read nc/names, NOT its train/val/test paths -- see
     the note in prepare_roboflow_dataset() below for why) plus
     {train,valid,test}/{images,labels} subfolders. Both are genuinely
     drone-native low-altitude footage (unlike xView's satellite
     imagery), so mosaic-compositing them together with VisDrone is
     intentional -- see class_map.py's module docstring.
       - UAVDT ships train/ only (no valid/test) -- auto-split by video
         sequence (see _auto_split_by_sequence()) rather than training
         with zero validation data.
       - SARD ships a real train/valid/test split already -- used as-is.
     MANUAL DOWNLOAD REQUIRED for both (Roboflow gates downloads behind
     a free account, no public direct-download API):
       UAVDT: https://universe.roboflow.com/kfupm-v0syf/uavdt-4g4uv
       SARD:  https://universe.roboflow.com/animesh-shastry/sard_yolo
              (pick version "v1 Original" specifically -- other versions
              on that project are grayscale/resized/augmented, which you
              don't want stacked under train.py's own augmentation, or
              mismatched against VisDrone/UAVDT's color imagery)
     Steps for either: sign in, Download Dataset -> export format
     "YOLOv8", skip/minimize Roboflow's own augmentation multiplier
     (raw/1x), extract so data.yaml sits directly at datasets/UAVDT/ or
     datasets/SARD/ (not nested inside an extra wrapper folder).
  4. Scans datasets/external/ for any additional manually-added datasets
     in the same data.yaml + split-folder format. Each one needs a
     matching entry in class_map.EXTERNAL_REMAPS -- datasets without one
     are listed but skipped, not guessed at.
  5. Writes datasets/unified.yaml: the merged dataset config train.py
     trains against by default, spanning every included dataset with a
     single consistent class list. Also writes datasets/per_source_val.json
     -- a manifest of each source's own val dir, used by train.py's
     evaluate_per_source() to report per-dataset accuracy after training
     (not just the blended number).
  6. SPARSE-CLASS OVERSAMPLING -- see _oversample_sparse_classes() below.
     Scans the assembled train image dirs for images containing any of a
     configurable set of under-represented classes (default: motorcycle,
     other_vehicle) and duplicates those image paths into a synthetic
     datasets/oversample_train.txt list, appended to unified.yaml's
     train: entries. This does NOT touch val -- only train dirs are ever
     passed in, so val stays a clean, untouched measurement of
     real-world distribution. Also does not modify any label files; it
     only affects how often existing (image, label) pairs are sampled
     per epoch. copy_paste/mixup augmentation still apply on top of
     oversampled images same as any other -- oversampling fixes "seen
     too rarely," copy_paste/mixup fix "not enough variety once seen."
       NOTE (verified against Ultralytics' actual CopyPaste
       implementation): CopyPaste only pastes objects using segmentation
       polygons (labels["instances"].segments). VisDrone/UAVDT/SARD are
       all bounding-box-only YOLO labels -- no polygons -- so
       --copy-paste is currently a no-op on this pipeline regardless of
       its value. Oversampling and mixup are the augmentation actually
       doing sparse-class protection right now; --copy-paste is left on
       in train.py only because it's harmless (costs nothing when
       inert), not because it's contributing anything.
  7. PER-CLASS INSTANCE COUNTING (new) -- see _count_instances_per_class()
     below. Unlike the per-SOURCE counts already printed at the end of
     main() (which tell you how many images each dataset contributed),
     this counts total labeled INSTANCES per class across the assembled
     train set, before oversampling. This is the number that actually
     tells you whether a class is sparse -- a class can appear in plenty
     of images but still trail badly in total boxes if instance density
     per image differs (SARD frames typically have a handful of people;
     VisDrone traffic scenes can have dozens of vehicles each). Use it to
     sanity-check --oversample-classes rather than assuming the default
     list (motorcycle, other_vehicle) is still the right one once
     UAVDT/SARD are folded in -- "person" is this project's stated
     priority class but isn't in that default list.

=======================================================================
V9 (2026-08-17): pending pseudo-label review images are EXCLUDED from
every train/val/test list entirely, not merely left with fewer boxes.
=======================================================================
generate_pseudo_labels.py's --apply already refuses to merge boxes for
an image that still has an undecided review item (V8) -- but before
V9, the IMAGE ITSELF was untouched by that: it stayed in whatever
train/val/test split it would normally fall into, carrying only
whatever labels it already had (e.g. UAVDT's native vehicle boxes,
missing person/motorcycle/other_vehicle until reviewed). That's a real
problem, not just an inconsistency -- most importantly for VAL: an
image with incomplete ground truth scores a model's correct detection
on the missing class as a false positive, which quietly corrupts
exactly the per-class mAP numbers train.py's report_per_class_map() /
evaluate_per_source() exist to give a clean read on. Training on it is
a softer cost (some frames imply "nothing here" where review just
hasn't happened yet) but still not what this project wants: only
images with NOTHING left pending should be usable anywhere in the
pipeline.

V9 fixes this at the source: _load_pending_review_images() reads
datasets/pending_review_images.json (written by generate_pseudo_
labels.py on EVERY run, dry-run or --apply -- see that module's V9
note) and every prepare_*() function below drops any matching image
from every train/val/test list it builds, for every Roboflow-style
source (UAVDT, SARD, datasets/external/*). VisDrone and xView are
untouched -- they never go through pseudo-labeling at all, so the
exclusion set never intersects them.

Mechanically: for a directory-based split (an existing valid/ split on
disk, e.g. SARD today), _build_train_val_entry() lists that directory's
images, and ONLY IF something needs excluding writes a filtered .txt
list of the survivors and returns THAT path instead of the plain
directory -- with nothing to exclude (the common case for a source once
it's fully reviewed), it returns the directory unchanged, identical to
pre-V9 behavior, no new file written. For UAVDT's auto-split-by-
sequence path (no valid/ on disk yet), _auto_split_by_sequence() now
filters the raw image list BEFORE grouping into sequences, so an
excluded image never even factors into which sequences get chosen for
val -- a sequence that loses all its images to exclusion simply
contributes nothing, rather than surviving as a mostly-empty group.

Backward compatible in every direction: no datasets/pending_review_
images.json (e.g. generate_pseudo_labels.py has never been run, or
nothing is currently pending) means an empty exclusion set, which means
every function below behaves EXACTLY as it did before V9 -- this is
purely additive gating, not a rewrite of the split logic itself.

EXPECT SMALLER UAVDT COUNTS RIGHT NOW: with most of UAVDT's pseudo-
label review still outstanding, a large share of its images are
currently excluded from train/val/test -- this is the fix working as
intended, not a bug. The usable pool grows automatically as more images
get reviewed and generate_pseudo_labels.py is rerun (with or without
--apply) to refresh pending_review_images.json.
=======================================================================

Remapping is idempotent and reversible: each dataset's ORIGINAL labels are
backed up once (labels_backup_original_<split>/), and re-applied fresh from
that backup any time class_map.py's taxonomy signature changes -- so
editing the taxonomy and rerunning this script is always safe, never
cumulative.
"""

import json
import random
import re
import shutil
from pathlib import Path

import yaml

from class_map import (
    UNIFIED_CLASSES,
    EXTERNAL_REMAPS,
    visdrone_index_remap,
    xview_index_remap,
    taxonomy_signature,
)

SCRIPT_DIR = Path(__file__).resolve().parent

# Use Ultralytics' own global datasets_dir setting rather than guessing a
# path relative to this script -- VisDrone and any manually-added datasets
# are downloaded/placed according to THAT setting, which may not be this
# script's own directory.
try:
    from ultralytics.utils import SETTINGS
    DATASETS_DIR = Path(SETTINGS.get("datasets_dir", SCRIPT_DIR / "datasets"))
except Exception:
    DATASETS_DIR = SCRIPT_DIR / "datasets"

EXTERNAL_DIR = DATASETS_DIR / "external"
UAVDT_ROOT = DATASETS_DIR / "UAVDT"
SARD_ROOT = DATASETS_DIR / "SARD"
UNIFIED_YAML_PATH = DATASETS_DIR / "unified.yaml"
PER_SOURCE_MANIFEST_PATH = DATASETS_DIR / "per_source_val.json"
OVERSAMPLE_LIST_PATH = DATASETS_DIR / "oversample_train.txt"
# [V9] Written by generate_pseudo_labels.py on every run (dry-run or
# --apply) -- see that module's V9 note. Every image listed here still
# has at least one undecided pseudo-label review item and is excluded
# from every train/val/test list this module builds.
PENDING_REVIEW_PATH = DATASETS_DIR / "pending_review_images.json"

SIGNATURE_FILENAME = ".taxonomy_signature"
BACKUP_DIRNAME = "labels_backup_original"

IMAGE_EXTS = (".jpg", ".jpeg", ".png")

# Default classes to oversample if train.py doesn't pass an explicit list
# (e.g. when this module is run standalone). Matches train.py's
# --oversample-classes default -- keep these in sync if you change one.
# See _count_instances_per_class() -- check real per-class counts before
# assuming this list still covers the actual sparse classes once
# UAVDT/SARD are folded in.
DEFAULT_OVERSAMPLE_CLASSES = ["motorcycle", "other_vehicle"]
DEFAULT_OVERSAMPLE_MULTIPLIER = 3


def _remap_label_file(path: Path, index_remap: list) -> None:
    """Rewrites one YOLO label .txt in place using index_remap (a list
    where position = original class index, value = new index or None to
    drop that line entirely)."""
    if not path.exists():
        return
    kept_lines = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split()
        old_idx = int(parts[0])
        if old_idx >= len(index_remap):
            continue  # unexpected class id, drop rather than crash
        new_idx = index_remap[old_idx]
        if new_idx is None:
            continue  # this class isn't in our taxonomy -- drop it
        kept_lines.append(" ".join([str(new_idx)] + parts[1:]))
    path.write_text("\n".join(kept_lines) + ("\n" if kept_lines else ""))


def _apply_remap_to_split(labels_dir: Path, index_remap: list) -> int:
    count = 0
    for txt_file in labels_dir.glob("*.txt"):
        _remap_label_file(txt_file, index_remap)
        count += 1
    return count


def _ensure_backup(labels_dir: Path, backup_dir: Path) -> None:
    if backup_dir.exists():
        return
    print(f"    Backing up original labels: {labels_dir} -> {backup_dir.name}/")
    shutil.copytree(labels_dir, backup_dir)


def _restore_from_backup(labels_dir: Path, backup_dir: Path) -> None:
    shutil.rmtree(labels_dir)
    shutil.copytree(backup_dir, labels_dir)


def _needs_remap(root: Path, signature: str) -> bool:
    sig_file = root / SIGNATURE_FILENAME
    return not sig_file.exists() or sig_file.read_text().strip() != signature


def _write_signature(root: Path, signature: str) -> None:
    (root / SIGNATURE_FILENAME).write_text(signature)


def _count_images_in(path_str: str) -> int:
    """Counts images referenced by a train/val entry, whether it's a
    directory or a txt list file (auto-split / oversample / [V9]
    pending-review-filtered output)."""
    p = Path(path_str)
    if p.is_dir():
        return sum(1 for f in p.iterdir() if f.suffix.lower() in IMAGE_EXTS)
    if p.is_file():
        return sum(1 for line in p.read_text().splitlines() if line.strip())
    return 0


def _count_all(paths: list[str]) -> int:
    return sum(_count_images_in(p) for p in paths)


def _iter_images_in_entry(entry: str):
    """Yields image Paths referenced by one train/val yaml entry, whether
    it's a directory of images or a txt list file (one path per line)."""
    p = Path(entry)
    if p.is_dir():
        for f in p.iterdir():
            if f.suffix.lower() in IMAGE_EXTS:
                yield f
    elif p.is_file():
        for line in p.read_text().splitlines():
            line = line.strip()
            if line:
                yield Path(line)


def _label_path_for_image(img_path: Path) -> Path | None:
    """
    Best-effort mapping from an image path to its YOLO label .txt,
    covering both dataset conventions used in this project:
      - VisDrone-style:   .../images/<split>/x.jpg -> .../labels/<split>/x.txt
      - Roboflow-style:   .../<split>/images/x.jpg  -> .../<split>/labels/x.txt
    Returns None if neither pattern matches, rather than guessing wrong.
    """
    parts = list(img_path.parts)
    if "images" not in parts:
        return None
    idx = len(parts) - 1 - parts[::-1].index("images")  # last "images" segment
    label_parts = parts[:idx] + ["labels"] + parts[idx + 1:]
    label_path = Path(*label_parts).with_suffix(".txt")
    return label_path if label_path.exists() else None


# ---------------------------------------------------------------------
# [V9] Pending-review image exclusion
# ---------------------------------------------------------------------

def _load_pending_review_images() -> set[str]:
    """[V9] Reads datasets/pending_review_images.json (written by
    generate_pseudo_labels.py every run -- see that module's V9 note)
    -- every image with at least one undecided pseudo-label review item
    right now. Returns resolved absolute path strings so filtering below
    can compare directly against Path.resolve() output regardless of how
    each image path was originally constructed elsewhere in the
    pipeline. Returns an empty set (not an error) if the file doesn't
    exist yet -- this project works fine without ever having run
    generate_pseudo_labels.py (e.g. before pseudo-labeling starts, or a
    source with no missing classes at all), and an empty exclusion set
    is a correct, safe default for that case: nothing is excluded,
    identical to pre-V9 behavior."""
    if not PENDING_REVIEW_PATH.exists():
        return set()
    with open(PENDING_REVIEW_PATH) as f:
        raw = json.load(f)
    return set(raw)


def _build_train_val_entry(images_dir: Path, exclude_images: set[str],
                             list_filename: str) -> str:
    """[V9] Returns the string to use as a train:/val:/test: yaml entry
    for one split's image directory -- the directory itself, UNLESS
    pending-review images need excluding, in which case this writes a
    filtered file list (one resolved absolute image path per line, same
    format _auto_split_by_sequence() already produces) alongside the
    dataset and returns THAT path instead.

    Backward compatible: with no exclusions at all (exclude_images is
    empty, or none of this split's images happen to be in it -- the
    ONLY case that existed before V9), this returns the plain directory
    string exactly as before -- no new file is written unless one is
    actually needed, so a fully-reviewed source (e.g. SARD today) sees
    zero behavior change."""
    if not exclude_images:
        return str(images_dir.resolve())
    all_images = [p for p in images_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS]
    kept = [p for p in all_images if str(p.resolve()) not in exclude_images]
    n_excluded = len(all_images) - len(kept)
    if n_excluded == 0:
        return str(images_dir.resolve())
    list_path = images_dir.parent / list_filename
    list_path.write_text("\n".join(str(p.resolve()) for p in kept) + "\n")
    print(f"    [pending-review exclude] {n_excluded}/{len(all_images)} images "
          f"in {images_dir} still have an undecided review item -- excluded; "
          f"{len(kept)} usable image(s) written to {list_path.name}")
    return str(list_path)


# ---------------------------------------------------------------------
# Per-class instance counting (new) -- see module docstring point 7.
# ---------------------------------------------------------------------

def _count_instances_per_class(dirs: list[str]) -> dict[str, int]:
    """
    Scans every label file referenced by `dirs` (dataset dirs or txt list
    files -- covers both VisDrone/Roboflow-style dirs and UAVDT's
    auto-split txt lists) and counts total labeled INSTANCES per class
    name -- not images.

    This is deliberately separate from the per-SOURCE image counts
    already printed at the end of main(): a source's image count doesn't
    tell you which CLASS is actually sparse once everything is merged.
    A class can appear in plenty of images but still trail badly in
    total boxes if instance density per image differs -- e.g. SARD
    frames typically contain a handful of people each, while VisDrone
    traffic scenes can have dozens of vehicles in one frame. Only a
    direct instance count across the merged set answers "is person
    actually underrepresented relative to the vehicle classes."

    Read-only -- does not touch any label file, purely a reporting pass.
    """
    counts = {name: 0 for name in UNIFIED_CLASSES}
    for entry in dirs:
        for img_path in _iter_images_in_entry(entry):
            label_path = _label_path_for_image(img_path)
            if label_path is None:
                continue
            for line in label_path.read_text().splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    idx = int(line.split()[0])
                except ValueError:
                    continue
                if 0 <= idx < len(UNIFIED_CLASSES):
                    counts[UNIFIED_CLASSES[idx]] += 1
    return counts


# ---------------------------------------------------------------------
# Sparse-class oversampling
# ---------------------------------------------------------------------

def _oversample_sparse_classes(train_dirs: list[str], target_classes: list[str],
                                 multiplier: int = DEFAULT_OVERSAMPLE_MULTIPLIER
                                 ) -> list[str]:
    """
    Scans every image referenced by train_dirs (directories OR txt list
    files -- covers both VisDrone/Roboflow-style dirs and UAVDT's
    auto-split txt lists) for labels containing any class in
    target_classes, and writes datasets/oversample_train.txt containing
    (multiplier - 1) EXTRA copies of each matching image's absolute path.
    That txt is added as an additional train: entry -- Ultralytics
    accepts repeated image paths across multiple train: list entries,
    each occurrence sampled independently per epoch, which is what
    actually increases how often the model sees these classes.

    Does NOT touch val_dirs (never passed in here) and does NOT modify
    any label file -- purely a sampling-frequency change over existing
    (image, label) pairs. Idempotent: reruns overwrite
    oversample_train.txt fresh rather than compounding.

    train_dirs is expected to already be pending-review-filtered (V9) --
    this function itself does no exclusion of its own, it just samples
    from whatever image paths it's handed.

    target_classes=[] or multiplier<=1 is a no-op (returns [] without
    writing anything), so this is safe to call unconditionally from
    main().
    """
    if not target_classes or multiplier <= 1:
        return []

    unknown = [c for c in target_classes if c not in UNIFIED_CLASSES]
    if unknown:
        print(f"\n[oversample] WARNING: {unknown} not in current "
              f"UNIFIED_CLASSES {UNIFIED_CLASSES} -- ignoring those.")
    target_names = [c for c in target_classes if c in UNIFIED_CLASSES]
    if not target_names:
        return []

    target_idxs = {UNIFIED_CLASSES.index(c) for c in target_names}

    print(f"\n[oversample] Scanning train images for classes {target_names} "
          f"(multiplier={multiplier}x)...")

    extra_paths: list[str] = []
    scanned = 0
    matched = 0
    for entry in train_dirs:
        for img_path in _iter_images_in_entry(entry):
            scanned += 1
            label_path = _label_path_for_image(img_path)
            if label_path is None:
                continue
            classes_present = set()
            for line in label_path.read_text().splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    classes_present.add(int(line.split()[0]))
                except ValueError:
                    continue
            if classes_present & target_idxs:
                matched += 1
                extra_paths.extend(
                    [str(img_path.resolve())] * (multiplier - 1))

    if not extra_paths:
        print(f"  [oversample] Scanned {scanned} images, found none "
              f"matching {target_names} -- nothing to oversample "
              f"(datasets not prepared yet, or these classes are truly "
              f"absent from the current train set).")
        if OVERSAMPLE_LIST_PATH.exists():
            OVERSAMPLE_LIST_PATH.unlink()
        return []

    DATASETS_DIR.mkdir(parents=True, exist_ok=True)
    OVERSAMPLE_LIST_PATH.write_text("\n".join(extra_paths) + "\n")
    print(f"  [oversample] {matched}/{scanned} train images contained a "
          f"target class -- wrote {len(extra_paths)} duplicate entries to "
          f"{OVERSAMPLE_LIST_PATH.name} ({multiplier}x total exposure for "
          f"those images).")
    return [str(OVERSAMPLE_LIST_PATH)]


# ---------------------------------------------------------------------
# Sequence-aware auto-split (used when a Roboflow dataset ships train/
# only, no valid/test -- currently UAVDT, but shared for anything else
# that ends up in this situation).
# ---------------------------------------------------------------------

# Roboflow appends "_<ext>.rf.<hash>" to the original filename on export
# (e.g. "M0101_000203_jpg.rf.ab12cd34ef1234567890.jpg"). Strip that before
# trying to find the original sequence/frame-number structure underneath.
_ROBOFLOW_SUFFIX_RE = re.compile(r"_(?:jpg|jpeg|png)\.rf\.[0-9a-fA-F]+$")


def _infer_sequence_key(image_name: str) -> str:
    """
    Best-effort grouping key for a video-frame filename, so a train/val
    split can keep whole sequences together instead of splitting
    consecutive near-duplicate frames across both sets.

    'M0101_000203_jpg.rf.ab12cd34ef.jpg' -> 'M0101'
    Falls back to the full (de-suffixed) stem if no trailing digit run
    is found. Verified against real UAVDT filenames from this project's
    actual export -- if a different mirror/export names things
    differently, check a handful of real filenames under train/images/
    and adjust the regex if grouping looks wrong.
    """
    stem = Path(image_name).stem
    stem = _ROBOFLOW_SUFFIX_RE.sub("", stem)
    m = re.match(r"^(.*?)[\-_]?(\d+)$", stem)
    return m.group(1) if m else stem


def _auto_split_by_sequence(images_dir: Path, out_train: Path, out_val: Path,
                              val_fraction: float = 0.1, seed: int = 42,
                              exclude_images: set[str] | None = None) -> None:
    """
    Writes out_train/out_val as txt files (one absolute image path per
    line -- Ultralytics accepts these directly as train:/val: entries,
    same mechanism xView's autosplit_*.txt already used), splitting
    whole video SEQUENCES (not individual frames) between them so no
    clip leaks near-duplicate frames across train and val.

    [V9] exclude_images: resolved absolute path strings (see
    _load_pending_review_images()) to drop BEFORE grouping into
    sequences -- an image still waiting on pseudo-label review never
    enters either split. Filtering happens at THIS level, not by post-
    processing the written txt files afterward, so val_fraction's
    sequence selection is computed over the already-clean pool rather
    than being skewed by sequences that lose most of their frames to
    exclusion after the split was already chosen. A sequence that has
    zero remaining images after filtering simply never appears in
    `sequences` below -- no empty-group artifacts.
    """
    exclude_images = exclude_images or set()
    images = sorted(p for p in images_dir.iterdir()
                     if p.suffix.lower() in IMAGE_EXTS)

    if exclude_images:
        before = len(images)
        images = [p for p in images if str(p.resolve()) not in exclude_images]
        n_excluded = before - len(images)
        if n_excluded:
            print(f"    [pending-review exclude] {n_excluded}/{before} images "
                  f"in {images_dir} still have an undecided review item -- "
                  f"excluded from this auto-split entirely ({len(images)} "
                  f"usable image(s) remain).")

    if not images:
        print(f"    [auto-split] No images found in {images_dir}, skipping.")
        return

    sequences: dict[str, list[Path]] = {}
    for img in images:
        sequences.setdefault(_infer_sequence_key(img.name), []).append(img)

    seq_keys = sorted(sequences.keys())
    rng = random.Random(seed)
    rng.shuffle(seq_keys)
    n_val_seqs = max(1, round(len(seq_keys) * val_fraction))
    val_keys = set(seq_keys[:n_val_seqs])

    train_list: list[Path] = []
    val_list: list[Path] = []
    for key, imgs in sequences.items():
        (val_list if key in val_keys else train_list).extend(imgs)

    out_train.write_text("\n".join(str(p.resolve()) for p in train_list) + "\n")
    out_val.write_text("\n".join(str(p.resolve()) for p in val_list) + "\n")

    print(f"    [auto-split] {len(val_list)} images ({len(val_keys)}/"
          f"{len(seq_keys)} sequences) -> val; {len(train_list)} images "
          f"({len(seq_keys) - len(val_keys)} sequences) -> train.")


# ---------------------------------------------------------------------
# VisDrone (Ultralytics auto-download convention: images/<split>,
# labels/<split> -- different shape from the Roboflow datasets below,
# handled separately for that reason).
# ---------------------------------------------------------------------

def prepare_visdrone(signature: str) -> dict:
    """Downloads VisDrone (no-op if already present) and remaps its
    labels into the unified taxonomy. Returns {"train": [...], "val": [...]}
    absolute image-dir paths for the merged yaml.

    [V9] Never touches pending-review exclusion -- VisDrone doesn't go
    through the pseudo-labeling/review pipeline at all (it's remapped
    directly from its own real labels), so no image of its could ever
    appear in datasets/pending_review_images.json in the first place."""
    from ultralytics.data.utils import check_det_dataset

    print("\n[VisDrone] Checking dataset (auto-downloads on first run)...")
    data = check_det_dataset("VisDrone.yaml")
    root = Path(data["path"])

    if _needs_remap(root, signature):
        print("[VisDrone] Taxonomy changed (or first run) -- remapping labels...")
        index_remap = visdrone_index_remap()
        for split in ("train", "val"):
            labels_dir = root / "labels" / split
            backup_dir = root / BACKUP_DIRNAME / split
            if not labels_dir.exists():
                continue
            _ensure_backup(labels_dir, backup_dir)
            _restore_from_backup(labels_dir, backup_dir)
            n = _apply_remap_to_split(labels_dir, index_remap)
            print(f"  [{split}] remapped {n} label files")
        _write_signature(root, signature)
    else:
        print("[VisDrone] Labels already remapped for current taxonomy, skipping.")

    return {
        "train": [str(root / "images" / "train")],
        "val": [str(root / "images" / "val")],
    }


# ---------------------------------------------------------------------
# xView -- inactive, see class_map.py's module docstring.
# ---------------------------------------------------------------------

def prepare_xview(signature: str) -> dict:
    """
    INACTIVE by project decision -- see class_map.py's module docstring.
    Still gracefully skips if datasets/xView/ doesn't exist (the normal
    case now). If someone DOES drop datasets/xView/ back in without
    first re-adding building/shed/parking_lot to UNIFIED_CLASSES, this
    fails with a clear message rather than a cryptic error three calls
    deep in xview_index_remap().

    [V9] Never touches pending-review exclusion -- same reasoning as
    prepare_visdrone(), xView never goes through pseudo-labeling.
    """
    xview_root = DATASETS_DIR / "xView"
    if not xview_root.exists():
        return {"train": [], "val": []}

    if "building" not in UNIFIED_CLASSES:
        raise RuntimeError(
            "datasets/xView/ is present, but building/shed/parking_lot "
            "were removed from UNIFIED_CLASSES (see class_map.py's module "
            "docstring for why). Either remove datasets/xView/, or "
            "re-add matching target classes to UNIFIED_CLASSES first if "
            "you're intentionally re-enabling satellite structure data."
        )

    from ultralytics.data.utils import check_det_dataset

    print("\n[xView] Found dataset -- checking/converting (one-time "
          "GeoJSON->YOLO conversion + autosplit on first run)...")
    try:
        data = check_det_dataset("xView.yaml")
    except Exception as e:
        print(f"[xView] Failed to prepare ({e}). Skipping xView for this run.")
        return {"train": [], "val": []}

    root = Path(data["path"])
    labels_dir = root / "labels" / "train"

    if _needs_remap(root, signature):
        print("[xView] Taxonomy changed (or first run) -- remapping labels...")
        backup_dir = root / BACKUP_DIRNAME / "train"
        if labels_dir.exists():
            _ensure_backup(labels_dir, backup_dir)
            _restore_from_backup(labels_dir, backup_dir)
            n = _apply_remap_to_split(labels_dir, xview_index_remap())
            print(f"  [train] remapped {n} label files")
        _write_signature(root, signature)
    else:
        print("[xView] Labels already remapped for current taxonomy, skipping.")

    train_list = root / "images" / "autosplit_train.txt"
    val_list = root / "images" / "autosplit_val.txt"
    return {
        "train": [str(train_list)] if train_list.exists() else [],
        "val": [str(val_list)] if val_list.exists() else [],
    }


# ---------------------------------------------------------------------
# Shared handler for Roboflow-style datasets (data.yaml + {train,valid,
# test}/{images,labels}) -- used by UAVDT, SARD, and anything under
# datasets/external/.
# ---------------------------------------------------------------------

def prepare_roboflow_dataset(display_name: str, root: Path,
                               remap_key: str, signature: str,
                               exclude_images: set[str] | None = None) -> dict:
    """
    Handles one Roboflow-exported dataset folder. Returns
    {"train": [...], "val": [...], "test": [...]} -- test is remapped
    (so it's ready if you ever want it) but never folded into train/val,
    kept as a genuinely untouched held-out set.

    Deliberately does NOT trust data.yaml's own train:/val:/test: path
    entries -- Roboflow writes these as "../train/images" etc, which
    assumes data.yaml sits one directory level deeper than it actually
    does when extracted directly into root/ (the convention this
    project uses). Resolving that literally would walk OUT of the
    dataset's own folder. Instead this uses the fixed convention
    directly: root/<split>/images, root/<split>/labels. data.yaml is
    only read for nc/names.

    [V9] exclude_images: resolved absolute path strings (see
    _load_pending_review_images()) -- any image still awaiting pseudo-
    label review is dropped from every split's output entirely, not
    just left with fewer boxes. See _build_train_val_entry() (for the
    existing-valid/-split branch) and _auto_split_by_sequence() (for
    the no-valid/-split branch) for the actual filtering. Empty/None
    exclude_images reproduces pre-V9 behavior exactly.
    """
    empty = {"train": [], "val": [], "test": []}
    exclude_images = exclude_images or set()

    if not root.exists():
        return empty

    data_yaml = root / "data.yaml"
    if not data_yaml.exists():
        print(f"\n[{display_name}] {root} exists but has no data.yaml -- skipping.")
        return empty

    remap_table = EXTERNAL_REMAPS.get(remap_key)
    if remap_table is None:
        print(f"\n[{display_name}] No class_map.EXTERNAL_REMAPS['{remap_key}'] "
              f"entry -- skipping. Add one to include it.")
        return empty

    with open(data_yaml) as f:
        meta = yaml.safe_load(f)
    names = meta.get("names")
    if isinstance(names, dict):
        names = [names[i] for i in sorted(names.keys())]
    if not names:
        print(f"\n[{display_name}] data.yaml has no usable 'names' list -- skipping.")
        return empty

    index_remap = [
        UNIFIED_CLASSES.index(remap_table[name])
        if remap_table.get(name) is not None else None
        for name in names
    ]

    print(f"\n[{display_name}] Found dataset at {root} (classes: {names})...")

    split_image_dirs: dict[str, Path] = {}
    for split in ("train", "valid", "val", "test"):
        candidate = root / split / "images"
        if candidate.exists() and any(candidate.iterdir()):
            split_image_dirs[split] = candidate

    if "train" not in split_image_dirs:
        print(f"  [{display_name}] No non-empty train/images found under "
              f"{root} -- skipping.")
        return empty

    if _needs_remap(root, signature):
        print(f"  [{display_name}] Taxonomy changed (or first run) -- "
              f"remapping labels...")
        for split, images_dir in split_image_dirs.items():
            labels_dir = images_dir.parent / "labels"
            if not labels_dir.exists():
                continue
            backup_dir = images_dir.parent / f"{BACKUP_DIRNAME}_{split}"
            _ensure_backup(labels_dir, backup_dir)
            _restore_from_backup(labels_dir, backup_dir)
            n = _apply_remap_to_split(labels_dir, index_remap)
            print(f"    [{split}] remapped {n} label files")
        _write_signature(root, signature)
    else:
        print(f"  [{display_name}] Labels already remapped for current "
              f"taxonomy, skipping.")

    val_key = "valid" if "valid" in split_image_dirs else (
        "val" if "val" in split_image_dirs else None)

    if val_key:
        # [V9] _build_train_val_entry() returns the plain directory
        # unchanged when nothing needs excluding (the common case once a
        # source is fully reviewed) -- only writes a filtered list file
        # when there's actually something to filter out.
        train_out = [_build_train_val_entry(
            split_image_dirs["train"], exclude_images, "train_filtered.txt")]
        val_out = [_build_train_val_entry(
            split_image_dirs[val_key], exclude_images, f"{val_key}_filtered.txt")]
        print(f"  [{display_name}] Using existing {val_key}/ split "
              f"({_count_images_in(val_out[0])} images"
              f"{', after excluding pending-review images' if exclude_images else ''}).")
    else:
        print(f"  [{display_name}] No valid/val split found on disk -- "
              f"auto-splitting train/ by sequence instead of training "
              f"with zero validation data.")
        out_train = root / "auto_split_train.txt"
        out_val = root / "auto_split_val.txt"
        _auto_split_by_sequence(split_image_dirs["train"], out_train, out_val,
                                  exclude_images=exclude_images)
        train_out = [str(out_train)] if out_train.exists() else []
        val_out = [str(out_val)] if out_val.exists() else []

    test_out: list[str] = []
    if "test" in split_image_dirs:
        test_out = [_build_train_val_entry(
            split_image_dirs["test"], exclude_images, "test_filtered.txt")]
        print(f"  [{display_name}] test/ split found "
              f"({_count_images_in(test_out[0])} images) -- labels "
              f"remapped but held out, not used for training/val.")

    return {"train": train_out, "val": val_out, "test": test_out}


def prepare_uavdt(signature: str, exclude_images: set[str] | None = None) -> dict:
    return prepare_roboflow_dataset("UAVDT", UAVDT_ROOT, "uavdt", signature,
                                      exclude_images)


def prepare_sard(signature: str, exclude_images: set[str] | None = None) -> dict:
    return prepare_roboflow_dataset("SARD", SARD_ROOT, "sard", signature,
                                      exclude_images)


def prepare_external(signature: str, exclude_images: set[str] | None = None) -> dict:
    """Scans datasets/external/<key>/ for any additional manually-added
    datasets in the same Roboflow-style format UAVDT/SARD use. Each
    folder needs a matching class_map.EXTERNAL_REMAPS[key] entry --
    folders without one are reported and skipped, not guessed at."""
    train_dirs: list[str] = []
    val_dirs: list[str] = []

    if not EXTERNAL_DIR.exists():
        return {"train": train_dirs, "val": val_dirs}

    for folder in sorted(EXTERNAL_DIR.iterdir()):
        if not folder.is_dir():
            continue
        result = prepare_roboflow_dataset(f"external/{folder.name}", folder,
                                            folder.name, signature,
                                            exclude_images)
        train_dirs.extend(result["train"])
        val_dirs.extend(result["val"])

    return {"train": train_dirs, "val": val_dirs}


def write_unified_yaml(train_dirs: list, val_dirs: list) -> Path:
    config = {
        "path": None,
        "train": train_dirs,
        "val": val_dirs,
        "nc": len(UNIFIED_CLASSES),
        "names": UNIFIED_CLASSES,
    }
    DATASETS_DIR.mkdir(parents=True, exist_ok=True)
    with open(UNIFIED_YAML_PATH, "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return UNIFIED_YAML_PATH


def write_per_source_manifest(sources: dict[str, dict]) -> None:
    """
    Writes datasets/per_source_val.json: {source_name: [val_dir_or_txt,
    ...]} for every source that has a val set. train.py's
    evaluate_per_source() reads this after training to report accuracy
    per DATASET (VisDrone/UAVDT/SARD individually), not just the single
    blended unified.yaml number -- the only way to actually tell whether
    adding UAVDT/SARD helped, since a gain on one source can hide a
    regression on another in a blended average.
    """
    manifest = {name: d["val"] for name, d in sources.items() if d["val"]}
    with open(PER_SOURCE_MANIFEST_PATH, "w") as f:
        json.dump(manifest, f, indent=2)


def main(oversample_classes: list[str] | None = None,
         oversample_multiplier: int = DEFAULT_OVERSAMPLE_MULTIPLIER) -> Path:
    """
    oversample_classes: UNIFIED_CLASSES names to duplicate train image
    paths for (see _oversample_sparse_classes()). Defaults to
    DEFAULT_OVERSAMPLE_CLASSES (motorcycle/other_vehicle) when this
    module is run standalone (`python prepare_datasets.py`); train.py
    passes its own --oversample-classes value explicitly. Pass [] to
    disable oversampling entirely.

    Check the "Per-class instance counts" block this prints (new) before
    trusting that default -- it's based on VisDrone alone and may not
    reflect the actual sparsest class once UAVDT/SARD are folded in.
    "person" is this project's stated priority class and is NOT in the
    default oversample list.

    [V9] Loads datasets/pending_review_images.json once (see
    _load_pending_review_images()) and threads it through UAVDT/SARD/
    external -- any image still awaiting pseudo-label review is excluded
    from every train/val/test list built below, not just left with
    fewer boxes. See this module's V9 docstring note.
    """
    if oversample_classes is None:
        oversample_classes = DEFAULT_OVERSAMPLE_CLASSES

    signature = taxonomy_signature()
    print(f"Taxonomy signature: {signature}")
    print(f"Unified classes ({len(UNIFIED_CLASSES)}): {UNIFIED_CLASSES}")

    # [V9] Loaded once, up front, so every source below excludes the
    # exact same current snapshot of pending images -- empty set (no-op,
    # identical to pre-V9 behavior) if generate_pseudo_labels.py has
    # never been run or nothing is currently pending.
    exclude_images = _load_pending_review_images()
    if exclude_images:
        print(f"\n[pending-review] {len(exclude_images)} image(s) listed in "
              f"{PENDING_REVIEW_PATH.name} still have an undecided pseudo-"
              f"label review item -- excluded from every train/val/test "
              f"list built below until fully reviewed. Rerun generate_"
              f"pseudo_labels.py (with or without --apply) after more "
              f"review happens to shrink this list.")
    else:
        print(f"\n[pending-review] No {PENDING_REVIEW_PATH.name} found (or "
              f"it's empty) -- nothing excluded on that basis.")

    visdrone_dirs = prepare_visdrone(signature)
    xview_dirs = prepare_xview(signature)
    uavdt_dirs = prepare_uavdt(signature, exclude_images)
    sard_dirs = prepare_sard(signature, exclude_images)
    external_dirs = prepare_external(signature, exclude_images)

    train_dirs = (visdrone_dirs["train"] + xview_dirs["train"]
                  + uavdt_dirs["train"] + sard_dirs["train"]
                  + external_dirs["train"])
    val_dirs = (visdrone_dirs["val"] + xview_dirs["val"]
                + uavdt_dirs["val"] + sard_dirs["val"]
                + external_dirs["val"])

    # Per-class instance counts (new) -- computed on train_dirs BEFORE
    # oversampling duplicates anything, so this reflects the real,
    # naturally-occurring class balance across the merged dataset. This
    # is what should actually drive --oversample-classes, not an assumed
    # default -- see this function's docstring. [V9] train_dirs here is
    # already pending-review-filtered, so these counts reflect only
    # fully-reviewed (or never-queued) images.
    print(f"\n{'=' * 70}")
    print("Per-class instance counts (train, before oversampling)")
    print(f"{'=' * 70}")
    class_counts = _count_instances_per_class(train_dirs)
    for name, n in class_counts.items():
        flag = "  <- in --oversample-classes" if name in oversample_classes else ""
        print(f"  {name:15s}  {n:>7}{flag}")
    print("Use this to sanity-check --oversample-classes -- a class can "
          "be the stated priority (e.g. 'person') and still end up the "
          "most underrepresented one if it isn't in that list.")
    print(f"{'=' * 70}")

    # Oversampling only ever ADDS extra train: entries (duplicate image
    # paths) -- val_dirs is never touched, so val stays a clean,
    # untouched measurement of real-world class distribution.
    oversample_dirs = _oversample_sparse_classes(
        train_dirs, target_classes=oversample_classes,
        multiplier=oversample_multiplier)
    train_dirs = train_dirs + oversample_dirs

    yaml_path = write_unified_yaml(train_dirs, val_dirs)

    named_sources = {
        "VisDrone": visdrone_dirs,
        "UAVDT": uavdt_dirs,
        "SARD": sard_dirs,
    }
    write_per_source_manifest(named_sources)

    print(f"\n{'=' * 70}")
    print(f"Unified dataset config written: {yaml_path}")
    print(f"  Train sources: {len(train_dirs)}")
    for d in train_dirs:
        print(f"    - {d}")
    print(f"  Val sources: {len(val_dirs)}")
    for d in val_dirs:
        print(f"    - {d}")
    print(f"{'=' * 70}")

    print(f"\n{'=' * 70}")
    print("Per-source contribution summary (image counts)")
    print(f"{'=' * 70}")
    for name, d in named_sources.items():
        n_train = _count_all(d["train"])
        n_val = _count_all(d["val"])
        n_test = _count_all(d.get("test", []))
        status = "" if (n_train or n_val) else "  (not found / not prepared)"
        print(f"  {name:10s}  train={n_train:>6}  val={n_val:>6}  "
              f"test={n_test:>6}{status}")
    if oversample_dirs:
        n_over = _count_all(oversample_dirs)
        print(f"  {'oversample':10s}  train=+{n_over:<5}  (extra duplicate "
              f"entries for {oversample_classes}, {oversample_multiplier}x)")
    print(f"{'=' * 70}")

    return yaml_path


if __name__ == "__main__":
    main()