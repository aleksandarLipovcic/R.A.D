"""
class_map.py — Unified class taxonomy for Project R.A.D detection
=======================================================================
Every dataset gets its classes remapped into UNIFIED_CLASSES before
training, so multiple datasets can be merged into one coherent model.

Design notes:
  - Bicycle/tricycle fine-grained distinctions are intentionally NOT a
    priority -> folded into "other_vehicle" (project decision).
  - building/shed/parking_lot were REMOVED: xView (satellite, intact
    structures) was the only source, and mixing it with VisDrone's
    drone-native frames hurt vehicle/person accuracy (different scale/
    angle in the same mosaic batch), while building recall was already
    poor (0.199) on real disaster footage. xView's code/remap is kept
    below (inactive) rather than deleted, in case a drone-native
    structure/damage dataset shows up later.
  - UAVDT (traffic footage, car/bus/truck only, no person labels) and
    SARD (SAR-posed people, no vehicle labels) are both drone-native
    (like VisDrone) so mosaic-mixing them is fine, unlike xView.

CAVEAT: UAVDT frames were never annotated for people, SARD frames never
for vehicles. An unlabeled person in a UAVDT frame (or vehicle in SARD)
acts as a false "background" signal for that class/location. Low risk
given the domains barely overlap, but worth remembering if per-class
precision looks off.

To add a dataset: add a REMAP table (orig class -> UNIFIED_CLASSES name,
or None to drop), then see prepare_datasets.py for the folder layout.

Changing UNIFIED_CLASSES or any REMAP invalidates existing remapped
labels; prepare_datasets.py detects this via taxonomy_signature() and
re-applies the remap from each dataset's untouched backup, so editing
this file and rerunning is always safe -- for the ORIGINAL labels only.
See prepare_datasets.py's pre-restore warning for the pseudo-label caveat.
"""

import hashlib
import json

UNIFIED_CLASSES = [
    "person",         # 0 - VisDrone (pedestrian+people) + SARD (SAR poses)
    "car",            # 1 - VisDrone + UAVDT
    "large_vehicle",  # 2 - van/truck/bus (VisDrone + UAVDT)
    "motorcycle",     # 3 - VisDrone
    "other_vehicle",  # 4 - bicycle/tricycle catch-all (low priority)
    # "building"/"shed"/"parking_lot" removed -- see docstring.
    # Roads/land-cover were never included -- not representable as boxes.
]

VISDRONE_ORIGINAL_ORDER = [
    "pedestrian", "people", "bicycle", "car", "van",
    "truck", "tricycle", "awning-tricycle", "bus", "motor",
]

VISDRONE_REMAP = {
    "pedestrian":       "person",
    "people":           "person",
    "bicycle":          "other_vehicle",
    "car":              "car",
    "van":              "large_vehicle",
    "truck":            "large_vehicle",
    "tricycle":         "other_vehicle",
    "awning-tricycle":  "other_vehicle",
    "bus":              "large_vehicle",
    "motor":            "motorcycle",
}

# ---------------------------------------------------------------------
# xView -- INACTIVE, kept for reference. xview_index_remap() raises a
# clear error against the current taxonomy rather than silently
# producing wrong indices -- see class_map's REMOVED note above before
# re-enabling.
# ---------------------------------------------------------------------
XVIEW_ORIGINAL_ORDER = [
    "Fixed-wing Aircraft", "Small Aircraft", "Cargo Plane", "Helicopter",
    "Passenger Vehicle", "Small Car", "Bus", "Pickup Truck", "Utility Truck",
    "Truck", "Cargo Truck", "Truck w/Box", "Truck Tractor", "Trailer",
    "Truck w/Flatbed", "Truck w/Liquid", "Crane Truck", "Railway Vehicle",
    "Passenger Car", "Cargo Car", "Flat Car", "Tank car", "Locomotive",
    "Maritime Vessel", "Motorboat", "Sailboat", "Tugboat", "Barge",
    "Fishing Vessel", "Ferry", "Yacht", "Container Ship", "Oil Tanker",
    "Engineering Vehicle", "Tower crane", "Container Crane", "Reach Stacker",
    "Straddle Carrier", "Mobile Crane", "Dump Truck", "Haul Truck",
    "Scraper/Tractor", "Front loader/Bulldozer", "Excavator",
    "Cement Mixer", "Ground Grader", "Hut/Tent", "Shed", "Building",
    "Aircraft Hangar", "Damaged Building", "Facility", "Construction Site",
    "Vehicle Lot", "Helipad", "Storage Tank", "Shipping container lot",
    "Shipping Container", "Pylon", "Tower",
]

XVIEW_REMAP = {
    "Building":         "building",
    "Damaged Building": "building",
    "Facility":         "building",
    "Aircraft Hangar":  "building",
    "Shed":             "shed",
    "Hut/Tent":         "shed",
    "Vehicle Lot":      "parking_lot",
}

# ---------------------------------------------------------------------
# Roboflow-sourced datasets (UAVDT, SARD, datasets/external/<key>/).
# Matched by folder name; same remap-table shape for all of them.
# ---------------------------------------------------------------------
EXTERNAL_REMAPS: dict[str, dict[str, str | None]] = {
    # UAVDT: drone-captured traffic footage. nc:3, names:[bus,car,truck]
    # (roboflow.com/kfupm-v0syf/uavdt-4g4uv). train/ only -> auto-split
    # by video sequence, see prepare_datasets._auto_split_by_sequence().
    "uavdt": {
        "car":   "car",
        "bus":   "large_vehicle",
        "truck": "large_vehicle",
    },
    # SARD: Search And Rescue Dataset (Sambolek & Ivasic-Kos, IEEE
    # DataPort, doi:10.21227/ahxm-k331). nc:1, names:[person]. Ships
    # with a real train/valid/test split.
    "sard": {
        "person": "person",
    },
    # AU-AIR (not yet added): once converted via its official toolkit,
    # its 8 classes map almost 1:1:
    # "auair": {
    #     "human": "person", "car": "car", "van": "large_vehicle",
    #     "truck": "large_vehicle", "bus": "large_vehicle",
    #     "motorbike": "motorcycle", "bike": "other_vehicle",
    #     "trailer": "other_vehicle",
    # },
}


def unified_index(name: str) -> int:
    try:
        return UNIFIED_CLASSES.index(name)
    except ValueError:
        raise ValueError(
            f"'{name}' is not in the current UNIFIED_CLASSES taxonomy "
            f"({UNIFIED_CLASSES}). Expected if this came from "
            f"xview_index_remap() -- xView is inactive since building/"
            f"shed/parking_lot were removed. See module docstring.")


def visdrone_index_remap() -> list[int | None]:
    """position i = original VisDrone class index -> UNIFIED_CLASSES
    index (or None to drop those labels)."""
    out: list[int | None] = []
    for name in VISDRONE_ORIGINAL_ORDER:
        target = VISDRONE_REMAP.get(name)
        out.append(unified_index(target) if target is not None else None)
    return out


def xview_index_remap() -> list[int | None]:
    """INACTIVE -- raises via unified_index() since building/shed/
    parking_lot aren't in the current taxonomy. Left implemented so
    re-enabling xView is a taxonomy edit, not a rewrite."""
    out: list[int | None] = []
    for name in XVIEW_ORIGINAL_ORDER:
        target = XVIEW_REMAP.get(name)
        out.append(unified_index(target) if target is not None else None)
    return out


def taxonomy_signature() -> str:
    """Hash of UNIFIED_CLASSES + every REMAP table -- prepare_datasets.py
    uses this to detect a taxonomy change and re-remap from backups."""
    payload = json.dumps(
        {
            "unified": UNIFIED_CLASSES,
            "visdrone": VISDRONE_REMAP,
            "xview": XVIEW_REMAP,
            "external": EXTERNAL_REMAPS,
        },
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:16]