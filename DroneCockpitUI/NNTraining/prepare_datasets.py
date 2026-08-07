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

SIGNATURE_FILENAME = ".taxonomy_signature"
BACKUP_DIRNAME = "labels_backup_original"

IMAGE_EXTS = (".jpg", ".jpeg", ".png")


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
    directory or a txt list file (auto-split output)."""
    p = Path(path_str)
    if p.is_dir():
        return sum(1 for f in p.iterdir() if f.suffix.lower() in IMAGE_EXTS)
    if p.is_file():
        return sum(1 for line in p.read_text().splitlines() if line.strip())
    return 0


def _count_all(paths: list[str]) -> int:
    return sum(_count_images_in(p) for p in paths)


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
                              val_fraction: float = 0.1, seed: int = 42) -> None:
    """
    Writes out_train/out_val as txt files (one absolute image path per
    line -- Ultralytics accepts these directly as train:/val: entries,
    same mechanism xView's autosplit_*.txt already used), splitting
    whole video SEQUENCES (not individual frames) between them so no
    clip leaks near-duplicate frames across train and val.
    """
    images = sorted(p for p in images_dir.iterdir()
                     if p.suffix.lower() in IMAGE_EXTS)
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
    absolute image-dir paths for the merged yaml."""
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
                               remap_key: str, signature: str) -> dict:
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
    """
    empty = {"train": [], "val": [], "test": []}

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
        train_out = [str(split_image_dirs["train"].resolve())]
        val_out = [str(split_image_dirs[val_key].resolve())]
        print(f"  [{display_name}] Using existing {val_key}/ split "
              f"({_count_images_in(val_out[0])} images).")
    else:
        print(f"  [{display_name}] No valid/val split found on disk -- "
              f"auto-splitting train/ by sequence instead of training "
              f"with zero validation data.")
        out_train = root / "auto_split_train.txt"
        out_val = root / "auto_split_val.txt"
        _auto_split_by_sequence(split_image_dirs["train"], out_train, out_val)
        train_out = [str(out_train)] if out_train.exists() else []
        val_out = [str(out_val)] if out_val.exists() else []

    test_out: list[str] = []
    if "test" in split_image_dirs:
        test_out = [str(split_image_dirs["test"].resolve())]
        print(f"  [{display_name}] test/ split found "
              f"({_count_images_in(test_out[0])} images) -- labels "
              f"remapped but held out, not used for training/val.")

    return {"train": train_out, "val": val_out, "test": test_out}


def prepare_uavdt(signature: str) -> dict:
    return prepare_roboflow_dataset("UAVDT", UAVDT_ROOT, "uavdt", signature)


def prepare_sard(signature: str) -> dict:
    return prepare_roboflow_dataset("SARD", SARD_ROOT, "sard", signature)


def prepare_external(signature: str) -> dict:
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
                                            folder.name, signature)
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


def main() -> Path:
    signature = taxonomy_signature()
    print(f"Taxonomy signature: {signature}")
    print(f"Unified classes ({len(UNIFIED_CLASSES)}): {UNIFIED_CLASSES}")

    visdrone_dirs = prepare_visdrone(signature)
    xview_dirs = prepare_xview(signature)
    uavdt_dirs = prepare_uavdt(signature)
    sard_dirs = prepare_sard(signature)
    external_dirs = prepare_external(signature)

    train_dirs = (visdrone_dirs["train"] + xview_dirs["train"]
                  + uavdt_dirs["train"] + sard_dirs["train"]
                  + external_dirs["train"])
    val_dirs = (visdrone_dirs["val"] + xview_dirs["val"]
                + uavdt_dirs["val"] + sard_dirs["val"]
                + external_dirs["val"])

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
    print(f"{'=' * 70}")

    return yaml_path


if __name__ == "__main__":
    main()