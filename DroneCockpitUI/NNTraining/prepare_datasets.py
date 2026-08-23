"""
prepare_datasets.py — Multi-dataset prep for Project R.A.D detection
=======================================================================
Run automatically by train.py before every training run (cheap -- each
step checks whether it's already done). Can also run standalone to
inspect what's currently included: `python prepare_datasets.py`.

What it does:
  1. Ensures VisDrone2019-DET is downloaded, remaps its 10 classes into
     the shared taxonomy (class_map.py).
  2. If datasets/xView/ exists, triggers xView conversion+remap.
     CURRENTLY INACTIVE -- see class_map.py's docstring.
  3. UAVDT (datasets/UAVDT/) and SARD (datasets/SARD/) -- Roboflow-style
     folders (data.yaml + {train,valid,test}/{images,labels}), genuinely
     drone-native so mosaic-mixing with VisDrone is fine.
       - UAVDT ships train/ only -> auto-split by video sequence.
       - SARD ships a real train/valid/test split, used as-is.
     MANUAL DOWNLOAD REQUIRED (Roboflow gates downloads):
       UAVDT: https://universe.roboflow.com/kfupm-v0syf/uavdt-4g4uv
       SARD:  https://universe.roboflow.com/animesh-shastry/sard_yolo
              (pick "v1 Original" -- other versions are pre-augmented)
     Export format "YOLOv8", raw/1x, extract directly into
     datasets/UAVDT/ or datasets/SARD/ (no extra wrapper folder).
  4. Scans datasets/external/ for extra manually-added datasets in the
     same format -- each needs a class_map.EXTERNAL_REMAPS entry.
  5. Writes datasets/unified.yaml (the merged training config) and
     datasets/per_source_val.json (per-dataset val manifest, used by
     train.py's evaluate_per_source()).
  6. SPARSE-CLASS OVERSAMPLING -- see _oversample_sparse_classes().
     Duplicates image paths (not labels) for under-represented classes
     into datasets/oversample_train.txt, an extra train: entry. val is
     never touched. NOTE: Ultralytics' CopyPaste needs segmentation
     polygons this bbox-only data doesn't have, so --copy-paste is
     currently a no-op regardless of value -- oversampling + mixup are
     what's actually protecting sparse classes.
  7. PER-CLASS INSTANCE COUNTING -- see _count_instances_per_class().
     Total labeled instances per class across train, before
     oversampling -- the real sparsity signal (a class can appear in
     many images but still trail in boxes if density-per-image
     differs). Use this to sanity-check --oversample-classes rather
     than trusting the default (motorcycle/other_vehicle) once
     UAVDT/SARD are folded in -- "person" (the priority class) isn't
     in that default.

PENDING PSEUDO-LABEL REVIEW EXCLUSION: generate_pseudo_labels.py writes
datasets/pending_review_images.json every run (dry-run or --apply) --
every image with at least one undecided review item. Every prepare_*()
function below drops matching images from every train/val/test list it
builds (Roboflow-style sources only; VisDrone/xView never go through
pseudo-labeling). This matters most for val: an image with incomplete
ground truth would score a correct detection on the missing class as a
false positive, corrupting per-class mAP. See _load_pending_review_images()
/ _build_train_val_entry() / _auto_split_by_sequence(). Path comparisons
are normcased (os.path.normcase) so a Windows separator/case mismatch
can't let a pending image slip through silently -- see _norm_path().
No pending_review_images.json (pseudo-labeling never run) means an
empty exclusion set, i.e. unchanged pre-existing behavior.

  [FIX -- review item] prepare_roboflow_dataset()'s "Using existing
  <split>/ split (...)" summary line used to decide whether to print
  ", after excluding pending-review images" by checking whether the
  GLOBAL pending-review set was non-empty, not whether *this specific
  split* actually lost any images to it. With pending-review images
  concentrated in one source (e.g. all of them in UAVDT/train, as in
  the first real run), every OTHER split -- including ones with zero
  exclusions, like SARD's valid/ -- printed the "after excluding..."
  qualifier anyway, which is simply false for those splits.
  _build_train_val_entry() now returns the actual per-split excluded
  count alongside the path, and the summary line's wording is driven
  by that count instead of the global set. The per-split
  "[pending-review exclude] N/M images ... excluded" line (printed
  inside _build_train_val_entry() itself, unconditionally accurate)
  was never affected by this bug -- only the one-line dataset summary
  was misleading.

Remapping is idempotent and reversible: each dataset's ORIGINAL labels
are backed up once (labels_backup_original_<split>/) and re-applied
fresh from that backup whenever class_map.py's taxonomy signature
changes. IMPORTANT CAVEAT: that backup predates pseudo-labeling, so a
taxonomy-triggered restore reverts to pre-pseudo-label state -- see the
loud warning in _ensure_backup_safe_restore() below before that happens,
and rerun generate_pseudo_labels.py --apply afterward to re-merge.
"""

import json
import os
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

# Use Ultralytics' own global datasets_dir setting rather than guessing
# a path relative to this script.
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
PENDING_REVIEW_PATH = DATASETS_DIR / "pending_review_images.json"

SIGNATURE_FILENAME = ".taxonomy_signature"
BACKUP_DIRNAME = "labels_backup_original"

IMAGE_EXTS = (".jpg", ".jpeg", ".png")

# Matches train.py's --oversample-classes default -- keep in sync.
# Check _count_instances_per_class()'s printed output before assuming
# this is still the right list once UAVDT/SARD are folded in.
DEFAULT_OVERSAMPLE_CLASSES = ["motorcycle", "other_vehicle"]
DEFAULT_OVERSAMPLE_MULTIPLIER = 3


def _remap_label_file(path: Path, index_remap: list) -> None:
    """Rewrites one YOLO label .txt in place using index_remap (position
    = original class index, value = new index or None to drop the line)."""
    if not path.exists():
        return
    kept_lines = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split()
        try:
            old_idx = int(parts[0])
        except (ValueError, IndexError):
            continue  # malformed line -- drop rather than crash prep
        if old_idx >= len(index_remap):
            continue  # unexpected class id, drop rather than crash
        new_idx = index_remap[old_idx]
        if new_idx is None:
            continue
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


def _count_label_lines(labels_dir: Path) -> int:
    total = 0
    if not labels_dir.exists():
        return 0
    for txt in labels_dir.glob("*.txt"):
        total += sum(1 for l in txt.read_text().splitlines() if l.strip())
    return total


def _labels_differ_from_backup(labels_dir: Path, backup_dir: Path) -> bool:
    """True if labels_dir's content differs from backup_dir's at all --
    file added/removed, or any file's content changed.

    [FIX -- review item #4] The original detection here was `current
    total label lines > backup total label lines`, which is only a
    heuristic: it misses a merge that changes content without changing
    the total count (e.g. pseudo-labels added to some files while an
    unrelated edit removed the same number of lines from others), and
    it misses a merge that nets FEWER total lines than the backup
    (e.g. pseudo-labels only touched a handful of sparse-class files).
    Either way the loud pre-restore warning could silently fail to
    fire on a run that genuinely has merged pseudo-labels about to be
    discarded. Comparing actual file contents (which file names exist,
    and each shared file's exact text) catches any of these instead of
    only the "more total lines" case."""
    if not backup_dir.exists():
        return False
    current_files = ({f.name: f for f in labels_dir.glob("*.txt")}
                      if labels_dir.exists() else {})
    backup_files = {f.name: f for f in backup_dir.glob("*.txt")}
    if set(current_files) != set(backup_files):
        return True
    return any(cur.read_text() != backup_files[name].read_text()
               for name, cur in current_files.items())


def _restore_from_backup(labels_dir: Path, backup_dir: Path,
                           dataset_label: str = "") -> None:
    """Restores labels_dir from its pristine pre-pseudo-labeling backup.

    [FIX -- review item #2] The backup is taken once, BEFORE any
    generate_pseudo_labels.py --apply merge ever runs, but --apply
    writes merged pseudo-labels directly into labels_dir, never into
    the backup. So a later taxonomy change here would silently discard
    every merged pseudo-label and revert to the pristine snapshot,
    with nothing warning that happened. Recoverable (rerun
    generate_pseudo_labels.py --apply -- it's idempotent and reads from
    pseudo_labels_reviewed/, which this never touches), but only if you
    know to do it. This now prints a loud warning BEFORE restoring
    whenever labels_dir's content differs at all from the backup (see
    _labels_differ_from_backup()) -- the signature of merged content
    having been added/changed since the backup was taken."""
    before = _count_label_lines(labels_dir)
    backup_before = _count_label_lines(backup_dir)
    if _labels_differ_from_backup(labels_dir, backup_dir):
        print(f"\n  {'!' * 66}")
        print(f"  WARNING [{dataset_label}]: about to restore {labels_dir} "
              f"from its pre-pseudo-labeling backup ({backup_dir.name}/).")
        print(f"  Current label files have {before} lines vs {backup_before} "
              f"in the backup -- this looks like generate_pseudo_labels.py "
              f"--apply has merged pseudo-labels in since the backup was "
              f"taken. Those merged boxes are about to be DISCARDED by "
              f"this restore (the taxonomy remap always starts from the "
              f"original backup).")
        print(f"  This is recoverable: rerun `python generate_pseudo_labels.py "
              f"--apply` right after this finishes to re-merge everything "
              f"(that merge is idempotent and reads from "
              f"pseudo_labels_reviewed/, which is untouched by this).")
        print(f"  {'!' * 66}\n")
    shutil.rmtree(labels_dir)
    shutil.copytree(backup_dir, labels_dir)


def _needs_remap(root: Path, signature: str) -> bool:
    sig_file = root / SIGNATURE_FILENAME
    return not sig_file.exists() or sig_file.read_text().strip() != signature


def _write_signature(root: Path, signature: str) -> None:
    (root / SIGNATURE_FILENAME).write_text(signature)


def _count_images_in(path_str: str) -> int:
    """Counts images referenced by a train/val entry, whether it's a
    directory or a txt list file."""
    p = Path(path_str)
    if p.is_dir():
        return sum(1 for f in p.iterdir() if f.suffix.lower() in IMAGE_EXTS)
    if p.is_file():
        return sum(1 for line in p.read_text().splitlines() if line.strip())
    return 0


def _count_all(paths: list[str]) -> int:
    return sum(_count_images_in(p) for p in paths)


def _iter_images_in_entry(entry: str):
    """Yields image Paths referenced by one train/val yaml entry (a dir
    of images, or a txt list file, one path per line)."""
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


def _find_last_images_index(parts: tuple) -> int | None:
    """[FIX -- review item #3] Shared, case-insensitive scan for the
    LAST "images" path component in a tuple of path parts. This is the
    single implementation both this module and generate_pseudo_labels.py
    use for the images<->labels convention (generate_pseudo_labels.py
    imports and calls this directly). Previously this module had its
    own independent, exact-case copy of this scan (inside
    _label_path_for_image below) while generate_pseudo_labels.py's copy
    was already case-insensitive -- on Windows, where .resolve() can
    normalize a path to its actual on-disk casing, that mismatch made
    this module's copy silently skip real images from instance counts /
    oversampling. One shared implementation means that can't drift out
    of sync again. Returns None if no "images" component is found."""
    lowered = [p.lower() for p in parts]
    if "images" not in lowered:
        return None
    return len(parts) - 1 - lowered[::-1].index("images")


def _label_path_for_image(img_path: Path) -> Path | None:
    """Maps an image path to its YOLO label .txt, covering both dataset
    conventions used in this project (VisDrone: .../images/<split>/x.jpg
    -> .../labels/<split>/x.txt; Roboflow: .../<split>/images/x.jpg ->
    .../<split>/labels/x.txt). Returns None if no "images" component is
    found, or if the resulting label file doesn't exist, rather than
    guessing wrong. Case-insensitive -- see _find_last_images_index()."""
    parts = list(img_path.parts)
    idx = _find_last_images_index(tuple(parts))
    if idx is None:
        return None
    label_parts = parts[:idx] + ["labels"] + parts[idx + 1:]
    label_path = Path(*label_parts).with_suffix(".txt")
    return label_path if label_path.exists() else None


# ---------------------------------------------------------------------
# Pending-review image exclusion
# ---------------------------------------------------------------------

def _norm_path(path_str: str) -> str:
    """Single shared normalization point for pending-review path
    comparisons -- os.path.normcase() so a Windows path compares equal
    regardless of '/' vs '\\' or drive-letter casing. Mirrors
    mosaic_guard.py's InstanceCappedMosaic._dedupe_key(). No-op on
    POSIX."""
    return os.path.normcase(path_str)


def _load_pending_review_images() -> set[str]:
    """Reads datasets/pending_review_images.json (written by
    generate_pseudo_labels.py every run) -- every image with at least
    one undecided pseudo-label review item right now. Returns normcased
    resolved absolute path strings. Empty set (not an error) if the
    file doesn't exist yet -- a safe default meaning nothing is
    excluded."""
    if not PENDING_REVIEW_PATH.exists():
        return set()
    with open(PENDING_REVIEW_PATH) as f:
        raw = json.load(f)
    return {_norm_path(p) for p in raw}


def _build_train_val_entry(images_dir: Path, exclude_images: set[str],
                             list_filename: str) -> tuple[str, int]:
    """Returns (yaml_entry, n_excluded) for one split's image directory.
    yaml_entry is the directory itself, unless pending-review images
    need excluding for THIS split, in which case this writes a filtered
    file list (one resolved absolute path per line) and returns that
    path instead. With no exclusions for this split, returns the plain
    directory string (no new file written) -- identical to
    pre-exclusion behavior.

    [FIX -- review item] n_excluded is now returned to the caller
    (previously this only printed its own per-split exclusion line and
    returned the path). Callers were checking the GLOBAL exclude_images
    set's truthiness to decide whether to mention exclusion in their
    own summary messages, which is wrong whenever pending-review images
    are concentrated in some OTHER split/dataset -- e.g. every pending
    review image being in UAVDT/train doesn't mean SARD's valid/ split
    excluded anything, but the old code's summary line claimed it did.
    Returning the real per-split count lets callers get this right."""
    if not exclude_images:
        return str(images_dir.resolve()), 0
    all_images = [p for p in images_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS]
    kept = [p for p in all_images
            if _norm_path(str(p.resolve())) not in exclude_images]
    n_excluded = len(all_images) - len(kept)
    if n_excluded == 0:
        return str(images_dir.resolve()), 0
    list_path = images_dir.parent / list_filename
    list_path.write_text("\n".join(str(p.resolve()) for p in kept) + "\n")
    print(f"    [pending-review exclude] {n_excluded}/{len(all_images)} images "
          f"in {images_dir} still have an undecided review item -- excluded; "
          f"{len(kept)} usable image(s) written to {list_path.name}")
    return str(list_path), n_excluded


# ---------------------------------------------------------------------
# Per-class instance counting
# ---------------------------------------------------------------------

def _count_instances_per_class(dirs: list[str]) -> dict[str, int]:
    """Scans every label file referenced by `dirs` and counts total
    labeled INSTANCES per class name (not images) -- the actual signal
    for whether a class is sparse, since instance density per image
    varies by source. Read-only reporting pass."""
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
    """Scans every train image for labels containing any target class
    and writes datasets/oversample_train.txt with (multiplier-1) extra
    copies of each matching image's path, added as an extra train:
    entry -- Ultralytics samples repeated paths independently per
    epoch. Never touches val_dirs or any label file -- pure sampling-
    frequency change. Idempotent (overwrites fresh each run). train_dirs
    is expected to already be pending-review-filtered; this does no
    exclusion of its own. No-op (returns []) if target_classes is empty
    or multiplier<=1."""
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
              f"matching {target_names} -- nothing to oversample.")
        if OVERSAMPLE_LIST_PATH.exists():
            OVERSAMPLE_LIST_PATH.unlink()
        return []

    DATASETS_DIR.mkdir(parents=True, exist_ok=True)
    OVERSAMPLE_LIST_PATH.write_text("\n".join(extra_paths) + "\n")
    print(f"  [oversample] {matched}/{scanned} train images contained a "
          f"target class -- wrote {len(extra_paths)} duplicate entries to "
          f"{OVERSAMPLE_LIST_PATH.name} ({multiplier}x total exposure).")
    return [str(OVERSAMPLE_LIST_PATH)]


# ---------------------------------------------------------------------
# Sequence-aware auto-split (used when a Roboflow dataset ships train/
# only, no valid/test -- currently UAVDT).
# ---------------------------------------------------------------------

# Roboflow appends "_<ext>.rf.<hash>" on export -- strip before finding
# the original sequence/frame-number structure.
_ROBOFLOW_SUFFIX_RE = re.compile(r"_(?:jpg|jpeg|png)\.rf\.[0-9a-fA-F]+$")


def _infer_sequence_key(image_name: str) -> str:
    """Best-effort grouping key for a video-frame filename, so a
    train/val split keeps whole sequences together instead of splitting
    consecutive near-duplicate frames across both. Falls back to the
    de-suffixed stem if no trailing digit run is found. Verified against
    real UAVDT filenames from this project's export."""
    stem = Path(image_name).stem
    stem = _ROBOFLOW_SUFFIX_RE.sub("", stem)
    m = re.match(r"^(.*?)[\-_]?(\d+)$", stem)
    return m.group(1) if m else stem


def _auto_split_by_sequence(images_dir: Path, out_train: Path, out_val: Path,
                              val_fraction: float = 0.1, seed: int = 42,
                              exclude_images: set[str] | None = None) -> None:
    """Writes out_train/out_val as txt files (one absolute image path
    per line), splitting whole video SEQUENCES between them so no clip
    leaks near-duplicate frames across train and val.

    exclude_images (normcased resolved paths) is filtered out BEFORE
    grouping into sequences, so a still-pending image never enters
    either split and val_fraction's sequence selection isn't skewed by
    sequences that lose most frames to exclusion after the fact. A
    sequence with zero remaining images after filtering simply doesn't
    appear -- no empty-group artifacts."""
    exclude_images = exclude_images or set()
    images = sorted(p for p in images_dir.iterdir()
                     if p.suffix.lower() in IMAGE_EXTS)

    if exclude_images:
        before = len(images)
        images = [p for p in images
                  if _norm_path(str(p.resolve())) not in exclude_images]
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
# VisDrone (Ultralytics auto-download convention).
# ---------------------------------------------------------------------

def prepare_visdrone(signature: str) -> dict:
    """Downloads VisDrone (no-op if present) and remaps its labels.
    Returns {"train": [...], "val": [...]} absolute image-dir paths.
    Never touches pending-review exclusion -- VisDrone doesn't go
    through the pseudo-labeling pipeline at all."""
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
            _restore_from_backup(labels_dir, backup_dir, f"VisDrone/{split}")
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
    """INACTIVE by project decision. Skips cleanly if datasets/xView/
    doesn't exist. Fails with a clear message (rather than a cryptic
    error deep in xview_index_remap()) if xView IS present but
    building/shed/parking_lot aren't in the current taxonomy. Never
    touches pending-review exclusion -- xView never goes through
    pseudo-labeling."""
    xview_root = DATASETS_DIR / "xView"
    if not xview_root.exists():
        return {"train": [], "val": []}

    if "building" not in UNIFIED_CLASSES:
        raise RuntimeError(
            "datasets/xView/ is present, but building/shed/parking_lot "
            "were removed from UNIFIED_CLASSES (see class_map.py). Either "
            "remove datasets/xView/, or re-add matching classes first."
        )

    from ultralytics.data.utils import check_det_dataset

    print("\n[xView] Found dataset -- checking/converting...")
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
            _restore_from_backup(labels_dir, backup_dir, "xView/train")
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
# test}/{images,labels}) -- used by UAVDT, SARD, datasets/external/.
# ---------------------------------------------------------------------

def prepare_roboflow_dataset(display_name: str, root: Path,
                               remap_key: str, signature: str,
                               exclude_images: set[str] | None = None) -> dict:
    """Handles one Roboflow-exported dataset folder. Returns
    {"train": [...], "val": [...], "test": [...]} -- test is remapped
    but never folded into train/val.

    Deliberately ignores data.yaml's own train:/val:/test: entries
    (Roboflow writes "../train/images", which assumes an extra nesting
    level this project doesn't use) -- uses the fixed convention
    root/<split>/images, root/<split>/labels instead. data.yaml is only
    read for nc/names.

    exclude_images (normcased resolved paths): any image still awaiting
    pseudo-label review is dropped from every split entirely, not just
    left with fewer boxes -- see _build_train_val_entry() and
    _auto_split_by_sequence(). Empty/None reproduces unfiltered behavior."""
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
        # [FIX -- review item] sorted(names.keys()) sorts lexicographically
        # when Roboflow/YAML writes the keys as strings ("0","1","10","2",
        # ...), producing 0,1,10,2,... instead of 0,1,2,...,10 -- silently
        # scrambling the class-index<->name mapping used to build
        # index_remap below. Sort by the keys' integer value instead so
        # this is correct regardless of whether YAML parsed the keys as
        # int or str.
        names = [names[k] for k in sorted(names.keys(), key=lambda k: int(k))]
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
            _restore_from_backup(labels_dir, backup_dir,
                                   f"{display_name}/{split}")
            n = _apply_remap_to_split(labels_dir, index_remap)
            print(f"    [{split}] remapped {n} label files")
        _write_signature(root, signature)
    else:
        print(f"  [{display_name}] Labels already remapped for current "
              f"taxonomy, skipping.")

    val_key = "valid" if "valid" in split_image_dirs else (
        "val" if "val" in split_image_dirs else None)

    if val_key:
        train_path, _train_excluded = _build_train_val_entry(
            split_image_dirs["train"], exclude_images, "train_filtered.txt")
        train_out = [train_path]
        val_path, val_excluded = _build_train_val_entry(
            split_image_dirs[val_key], exclude_images, f"{val_key}_filtered.txt")
        val_out = [val_path]
        # [FIX -- review item] this used to check truthiness of the
        # GLOBAL exclude_images set here, which made every dataset's
        # summary line claim "after excluding pending-review images"
        # whenever ANY pending-review images existed anywhere in the
        # whole run -- even for a split (like SARD's valid/ when every
        # pending image was in UAVDT/train) that excluded zero images.
        # val_excluded is this split's actual excluded count, so the
        # qualifier now only appears when it's true for this split.
        print(f"  [{display_name}] Using existing {val_key}/ split "
              f"({_count_images_in(val_out[0])} images"
              f"{', after excluding pending-review images' if val_excluded else ''}).")
    else:
        print(f"  [{display_name}] No valid/val split found on disk -- "
              f"auto-splitting train/ by sequence instead.")
        out_train = root / "auto_split_train.txt"
        out_val = root / "auto_split_val.txt"
        _auto_split_by_sequence(split_image_dirs["train"], out_train, out_val,
                                  exclude_images=exclude_images)
        train_out = [str(out_train)] if out_train.exists() else []
        val_out = [str(out_val)] if out_val.exists() else []

    test_out: list[str] = []
    if "test" in split_image_dirs:
        test_path, test_excluded = _build_train_val_entry(
            split_image_dirs["test"], exclude_images, "test_filtered.txt")
        test_out = [test_path]
        print(f"  [{display_name}] test/ split found "
              f"({_count_images_in(test_out[0])} images"
              f"{', after excluding pending-review images' if test_excluded else ''}) "
              f"-- labels remapped but held out, not used for training/val.")

    return {"train": train_out, "val": val_out, "test": test_out}


def prepare_uavdt(signature: str, exclude_images: set[str] | None = None) -> dict:
    return prepare_roboflow_dataset("UAVDT", UAVDT_ROOT, "uavdt", signature,
                                      exclude_images)


def prepare_sard(signature: str, exclude_images: set[str] | None = None) -> dict:
    return prepare_roboflow_dataset("SARD", SARD_ROOT, "sard", signature,
                                      exclude_images)


def prepare_external(signature: str, exclude_images: set[str] | None = None) -> dict:
    """Scans datasets/external/<key>/ for extra manually-added datasets
    in the same Roboflow-style format. Folders without a matching
    class_map.EXTERNAL_REMAPS entry are reported and skipped.

    Returns {"train": [...], "val": [...], "sources": {...}} -- "train"/
    "val" are the flattened lists (unchanged shape, so existing callers
    that just fold external_dirs["train"]/["val"] into the overall
    train_dirs/val_dirs keep working). "sources" additionally keys each
    non-empty external dataset individually as "external/<folder name>"
    -> {"train":[...], "val":[...], "test":[...]}.

    [FIX -- review item] Previously this only returned the flattened
    "train"/"val" lists, so every external dataset's val split was
    merged into one anonymous blob before reaching
    write_per_source_manifest() -- external datasets participated in
    training/validation but never appeared individually in
    datasets/per_source_val.json, so train.py's evaluate_per_source()
    had no way to report accuracy for e.g. datasets/external/foo/ on
    its own. "sources" lets main() merge these into named_sources
    without losing each dataset's identity."""
    train_dirs: list[str] = []
    val_dirs: list[str] = []
    sources: dict[str, dict] = {}

    if not EXTERNAL_DIR.exists():
        return {"train": train_dirs, "val": val_dirs, "sources": sources}

    for folder in sorted(EXTERNAL_DIR.iterdir()):
        if not folder.is_dir():
            continue
        key = f"external/{folder.name}"
        result = prepare_roboflow_dataset(key, folder, folder.name, signature,
                                            exclude_images)
        train_dirs.extend(result["train"])
        val_dirs.extend(result["val"])
        if result["train"] or result["val"] or result.get("test"):
            sources[key] = result

    return {"train": train_dirs, "val": val_dirs, "sources": sources}


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
    """Writes datasets/per_source_val.json: {source: [val_dir_or_txt,
    ...]} -- train.py's evaluate_per_source() reads this to report
    accuracy per dataset, not just the blended unified.yaml number."""
    manifest = {name: d["val"] for name, d in sources.items() if d["val"]}
    with open(PER_SOURCE_MANIFEST_PATH, "w") as f:
        json.dump(manifest, f, indent=2)


def main(oversample_classes: list[str] | None = None,
         oversample_multiplier: int = DEFAULT_OVERSAMPLE_MULTIPLIER) -> Path:
    """oversample_classes: UNIFIED_CLASSES names to duplicate train
    image paths for. Defaults to DEFAULT_OVERSAMPLE_CLASSES when run
    standalone; train.py passes its own --oversample-classes. Pass []
    to disable.

    Check the "Per-class instance counts" block this prints before
    trusting the default -- "person" (the priority class) isn't in it.

    Loads datasets/pending_review_images.json once and threads it
    through UAVDT/SARD/external -- any image still awaiting pseudo-
    label review is excluded from every train/val/test list built
    below."""
    if oversample_classes is None:
        oversample_classes = DEFAULT_OVERSAMPLE_CLASSES

    signature = taxonomy_signature()
    print(f"Taxonomy signature: {signature}")
    print(f"Unified classes ({len(UNIFIED_CLASSES)}): {UNIFIED_CLASSES}")

    exclude_images = _load_pending_review_images()
    if exclude_images:
        print(f"\n[pending-review] {len(exclude_images)} image(s) listed in "
              f"{PENDING_REVIEW_PATH.name} still have an undecided pseudo-"
              f"label review item -- excluded from every train/val/test "
              f"list built below until fully reviewed.")
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

    print(f"\n{'=' * 70}")
    print("Per-class instance counts (train, before oversampling)")
    print(f"{'=' * 70}")
    class_counts = _count_instances_per_class(train_dirs)
    for name, n in class_counts.items():
        flag = "  <- in --oversample-classes" if name in oversample_classes else ""
        print(f"  {name:15s}  {n:>7}{flag}")
    print("Use this to sanity-check --oversample-classes -- a stated "
          "priority class can still be the most underrepresented one if "
          "it isn't in that list.")
    print(f"{'=' * 70}")

    oversample_dirs = _oversample_sparse_classes(
        train_dirs, target_classes=oversample_classes,
        multiplier=oversample_multiplier)
    train_dirs = train_dirs + oversample_dirs

    yaml_path = write_unified_yaml(train_dirs, val_dirs)

    # [FIX -- review item] external datasets used to be missing from
    # named_sources entirely -- write_per_source_manifest() (and the
    # per-source summary printed below) only ever saw VisDrone/UAVDT/
    # SARD, so anything under datasets/external/ participated in
    # training/val but never got its own line in per_source_val.json.
    # external_dirs["sources"] keys each one individually as
    # "external/<name>", so **-unpacking it here gives every external
    # dataset the same per-source visibility as the built-in ones.
    named_sources = {
        "VisDrone": visdrone_dirs,
        "UAVDT": uavdt_dirs,
        "SARD": sard_dirs,
        **external_dirs["sources"],
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