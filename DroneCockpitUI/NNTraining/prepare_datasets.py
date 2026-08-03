"""
prepare_datasets.py — Multi-dataset prep for Project R.A.D detection
=======================================================================
Run automatically by train.py before every training run (safe/fast to
call every time -- each step checks whether it's already done). Can also
be run standalone to inspect what's currently included:

    python prepare_datasets.py

What it does:
  1. Ensures VisDrone2019-DET is downloaded (auto), then remaps its 10
     original classes into the shared taxonomy from class_map.py
     (see class_map.py's docstring for *why*: focuses model capacity on
     person/vehicle classes per project priorities).
  2. If datasets/xView/ exists (manually downloaded+extracted per the
     note below), triggers Ultralytics' own xView.yaml conversion hook
     (GeoJSON -> YOLO format + train/val autosplit, same mechanism
     VisDrone.yaml uses) and remaps its structure-related classes into
     the shared 'building' class. Skipped gracefully if not present yet.
  3. Scans datasets/external/ for any manually-added datasets in
     Roboflow/Ultralytics YOLO export format (a data.yaml + images/labels
     dirs) for datasets that AREN'T natively supported by Ultralytics.
     Each one needs a matching entry in class_map.EXTERNAL_REMAPS --
     datasets without one are listed but skipped, not guessed at.
  4. Writes datasets/unified.yaml: the merged dataset config train.py
     trains against by default, spanning every included dataset with a
     single consistent class list.

Remapping is idempotent and reversible: each dataset's ORIGINAL labels are
backed up once (labels_backup_original/), and re-applied fresh from that
backup any time class_map.py's taxonomy signature changes -- so editing
the taxonomy and rerunning this script is always safe, never cumulative.

On the xView (buildings) dataset: it has an actual "Building" class,
unlike DOTA, but is gated behind a manual NGA license registration at
https://challenge.xviewdataset.org -- there's no way to automate that
step, it's a legal wall, not a pipeline gap. Once registered, extract
train_images.zip, train_labels.zip, and val_images.zip so that
datasets/xView/ contains:
    datasets/xView/
    ├── train_images/
    ├── val_images/
    └── xView_train.geojson    <- must be directly here, not nested
                                  inside a train_labels/ folder
This script then handles conversion, remapping, and merging automatically
-- only that one manual download+extract step can't be automated.

Buildings are trained as part of THIS SAME model (not a separate one) via
the shared 'building' class -- but only using xView's structure classes,
deliberately NOT its vehicle classes, since satellite imagery is a
different visual domain (scale, blur, always-top-down angle) from drone
footage, and mixing xView's vehicles in risks diluting the vehicle
accuracy already built up from VisDrone rather than adding a clean new
capability.
"""

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
# (like xView) are downloaded/placed according to THAT setting, which may
# not be this script's own directory (e.g. it resolved to
# .../DroneCockpitUI/datasets/ here, one level up from NNTraining/).
try:
    from ultralytics.utils import SETTINGS
    DATASETS_DIR = Path(SETTINGS.get("datasets_dir", SCRIPT_DIR / "datasets"))
except Exception:
    DATASETS_DIR = SCRIPT_DIR / "datasets"

EXTERNAL_DIR = DATASETS_DIR / "external"
UNIFIED_YAML_PATH = DATASETS_DIR / "unified.yaml"

SIGNATURE_FILENAME = ".taxonomy_signature"
BACKUP_DIRNAME = "labels_backup_original"


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
    print(f"  Backing up original labels: {labels_dir.name} -> {backup_dir.name}/")
    shutil.copytree(labels_dir, backup_dir)


def _restore_from_backup(labels_dir: Path, backup_dir: Path) -> None:
    shutil.rmtree(labels_dir)
    shutil.copytree(backup_dir, labels_dir)


def _needs_remap(root: Path, signature: str) -> bool:
    sig_file = root / SIGNATURE_FILENAME
    return not sig_file.exists() or sig_file.read_text().strip() != signature


def _write_signature(root: Path, signature: str) -> None:
    (root / SIGNATURE_FILENAME).write_text(signature)


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


def prepare_xview(signature: str) -> dict:
    """
    xView is natively supported by Ultralytics (unlike a generic external
    dataset) -- its own xView.yaml handles GeoJSON->YOLO conversion and the
    train/val autosplit automatically, the same mechanism VisDrone.yaml
    uses. This just triggers that, then remaps the resulting labels into
    the 'building' class (see class_map.py for which xView classes map to
    building vs get dropped).

    Requires the manual download described in prepare_datasets.py's
    docstring to already be extracted under datasets/xView/. If it's not
    there, this is skipped gracefully -- training proceeds VisDrone-only.
    """
    xview_root = DATASETS_DIR / "xView"
    if not xview_root.exists():
        print("\n[xView] Not found under datasets/xView/ -- skipping "
              "(building class will have no training data yet). See this "
              "script's docstring for how to add it.")
        return {"train": [], "val": []}

    from ultralytics.data.utils import check_det_dataset

    print("\n[xView] Found dataset -- checking/converting (one-time "
          "GeoJSON->YOLO conversion + autosplit on first run)...")
    try:
        data = check_det_dataset("xView.yaml")
    except Exception as e:
        print(f"[xView] Failed to prepare ({e}). Check that "
              f"xView_train.geojson sits directly under datasets/xView/ "
              f"(not nested inside a train_labels/ folder) -- see this "
              f"script's docstring for the exact expected layout. "
              f"Skipping xView for this run.")
        return {"train": [], "val": []}

    root = Path(data["path"])
    labels_dir = root / "labels" / "train"  # xView's autosplit shares one
                                              # labels dir across train/val

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


def prepare_external(signature: str) -> dict:
    """Scans datasets/external/*/data.yaml for manually-added datasets.
    Only includes folders with a matching class_map.EXTERNAL_REMAPS entry;
    everything else is reported and skipped."""
    train_dirs: list[str] = []
    val_dirs: list[str] = []

    if not EXTERNAL_DIR.exists():
        return {"train": train_dirs, "val": val_dirs}

    for folder in sorted(EXTERNAL_DIR.iterdir()):
        if not folder.is_dir():
            continue
        data_yaml = folder / "data.yaml"
        if not data_yaml.exists():
            continue

        key = folder.name
        remap_table = EXTERNAL_REMAPS.get(key)
        if remap_table is None:
            print(f"\n[external/{key}] Found dataset but no class_map.py "
                  f"EXTERNAL_REMAPS['{key}'] entry -- skipping. Add one to "
                  f"include it.")
            continue

        print(f"\n[external/{key}] Preparing...")
        with open(data_yaml) as f:
            meta = yaml.safe_load(f)

        names = meta["names"]
        if isinstance(names, dict):
            names = [names[i] for i in sorted(names.keys())]
        index_remap = []
        for name in names:
            target = remap_table.get(name)
            index_remap.append(UNIFIED_CLASSES.index(target)
                                if target is not None else None)

        if _needs_remap(folder, signature):
            print(f"  Taxonomy changed (or first run) -- remapping labels...")
            for split in ("train", "valid", "val"):
                labels_dir = folder / split / "labels"
                if not labels_dir.exists():
                    labels_dir = folder / "labels" / split
                if not labels_dir.exists():
                    continue
                backup_dir = labels_dir.parent / f"{BACKUP_DIRNAME}_{labels_dir.name}"
                _ensure_backup(labels_dir, backup_dir)
                _restore_from_backup(labels_dir, backup_dir)
                n = _apply_remap_to_split(labels_dir, index_remap)
                print(f"  [{split}] remapped {n} label files")
            _write_signature(folder, signature)
        else:
            print(f"  Labels already remapped for current taxonomy, skipping.")

        # Resolve this dataset's own train/val image dirs from its data.yaml
        for split_key, bucket in (("train", train_dirs), ("val", val_dirs)):
            split_path = meta.get(split_key)
            if split_path is None:
                continue
            resolved = (folder / split_path).resolve()
            bucket.append(str(resolved))

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


def main() -> Path:
    signature = taxonomy_signature()
    print(f"Taxonomy signature: {signature}")
    print(f"Unified classes ({len(UNIFIED_CLASSES)}): {UNIFIED_CLASSES}")

    visdrone_dirs = prepare_visdrone(signature)
    xview_dirs = prepare_xview(signature)
    external_dirs = prepare_external(signature)

    train_dirs = visdrone_dirs["train"] + xview_dirs["train"] + external_dirs["train"]
    val_dirs = visdrone_dirs["val"] + xview_dirs["val"] + external_dirs["val"]

    yaml_path = write_unified_yaml(train_dirs, val_dirs)

    print(f"\n{'=' * 70}")
    print(f"Unified dataset config written: {yaml_path}")
    print(f"  Train sources: {len(train_dirs)}")
    for d in train_dirs:
        print(f"    - {d}")
    print(f"  Val sources: {len(val_dirs)}")
    for d in val_dirs:
        print(f"    - {d}")
    if not EXTERNAL_REMAPS:
        print("\n  No external datasets configured yet -- training on "
              "VisDrone only (person/car/large_vehicle/motorcycle/"
              "other_vehicle). See this script's docstring to add xView "
              "(buildings) or another vehicle/person dataset for "
              "robustness.")
    print(f"{'=' * 70}")

    return yaml_path


if __name__ == "__main__":
    main()