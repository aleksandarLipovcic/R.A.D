"""
class_map.py — Unified class taxonomy for Project R.A.D detection
=======================================================================
Every dataset used for training gets its classes remapped into this one
shared list before training starts. This is what lets multiple datasets
(different original class sets) be merged into a single coherent model
instead of each pulling the model in a different direction.

Goal driving this taxonomy: the drone pilot assistant cares about people
and vehicles first, with fine-grained bicycle/tricycle distinctions
explicitly NOT a priority (per project decision) -- so those get folded
into a single low-priority bucket rather than kept as separate classes
diluting training signal.

STRUCTURAL CLASSES REMOVED (building/shed/parking_lot) -- project
decision after the first trained model was tested against real disaster
footage: xView (satellite, ortho-rectified, intact structures) was the
only source for these, and testing showed it actively hurt vehicle/
person accuracy when mosaic-composited alongside VisDrone's drone-native
frames (different scale/angle/resolution in the same training batch),
while building recall itself was poor (0.199) and near-total on real
disaster footage where structures are damaged/rubble, not intact --
xView never saw that. Rather than keep three classes with zero current
training data (dead output capacity), they're removed here. Re-add them
once a genuinely drone-native, disaster-relevant structure/damage
dataset is wired in. xView's integration code and remap tables are left
in place below (commented as inactive) rather than deleted.

DATASET ADDITIONS (UAVDT, SARD) -- both are genuinely drone-native
low-altitude footage (same visual domain as VisDrone: oblique angle,
similar altitude range), unlike xView's satellite imagery -- so, unlike
xView, letting Mosaic freely composite across VisDrone+UAVDT+SARD is
intentional and desired, not a repeat of the xView domain-mixing
problem. See prepare_datasets.py for how each is prepared.
  - UAVDT: drone-captured low-altitude traffic footage. Adds vehicle
    diversity/volume (car/bus/truck), but contributes NO person labels
    at all -- worth remembering when reading per-class results, since
    UAVDT dilutes the person:vehicle label ratio rather than helping
    person detection.
  - SARD: purpose-built Search And Rescue dataset -- actors simulating
    standing/sitting/walking/running/lying/exhausted/injured poses
    across wilderness terrain, single class "person". This is the
    dataset actually targeting the project's stated SAR priority
    (existing VisDrone person labels are pedestrians/general crowds
    -- normal upright poses, not SAR-relevant ones like lying/downed).

CAVEAT worth knowing (not fixed by anything below -- a real limitation
of combining partially-labeled sources): UAVDT images were never
annotated for people, and SARD images were never annotated for
vehicles. If a UAVDT frame happens to contain an unlabeled person, or a
SARD frame an unlabeled vehicle, that acts as a false "background"
signal for that class at that location during training. Low risk given
UAVDT is traffic-camera-angle vehicle footage and SARD is wilderness/
forest/quarry footage (little natural co-occurrence), but not zero risk
-- if per-class precision looks oddly low after this run, this is one
place to look.

To add a new dataset:
  1. Add a REMAP table below (original class name -> one of UNIFIED_CLASSES,
     or None to drop that class from training entirely).
  2. See prepare_datasets.py for the folder convention datasets need to
     follow (root/data.yaml + root/{train,valid,test}/{images,labels}).

Changing UNIFIED_CLASSES or any REMAP table invalidates existing remapped
labels -- prepare_datasets.py detects this automatically (via
taxonomy_signature()) and re-applies the remap from each dataset's
untouched backup copy, so it's always safe to edit this file and rerun.

IMPORTANT: this taxonomy change means nc changed (8 -> 5). Any existing
checkpoint trained under the old 8-class taxonomy CANNOT be resumed
(train.py --resume will fail loudly, by design -- see its docstring).
Start a fresh run.
"""

import hashlib
import json

# The single shared class list every dataset gets remapped into.
# Order matters -- this is the class index order the model actually trains
# on. Append-only is safest once you have a trained checkpoint you want to
# keep resuming; inserting/removing/reordering entries invalidates it.
UNIFIED_CLASSES = [
    "person",         # 0 - VisDrone (pedestrian+people) + SARD (SAR-native
                       #     poses: standing/sitting/lying/exhausted/injured)
    "car",            # 1 - VisDrone + UAVDT
    "large_vehicle",  # 2 - van, truck, bus and similar (VisDrone + UAVDT)
    "motorcycle",     # 3 - VisDrone
    "other_vehicle",  # 4 - bicycle, tricycle, and similar (low priority
                       #     catch-all, kept only so these don't get
                       #     silently misdetected as something else)
    # "building", "shed", "parking_lot" REMOVED -- see module docstring.
    # Roads/forest/land-cover were never included -- those aren't
    # representable as bounding boxes at all (a road has no natural box;
    # it runs edge-to-edge in an irregular shape) -- they'd need semantic
    # segmentation, a genuinely different pipeline, not another class here.
]

# VisDrone2019-DET's original class order, as Ultralytics' own VisDrone.yaml
# assigns indices 0-9 (confirmed against the confusion-matrix axis labels
# from actual training output: pedestrian, people, bicycle, car, van,
# truck, tricycle, awning-tricycle, bus, motor).
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
# xView -- INACTIVE. Kept for reference only; see module docstring for
# why building/shed/parking_lot were removed from UNIFIED_CLASSES.
# xview_index_remap() below will raise a clear error if called against
# the current taxonomy rather than silently producing wrong indices --
# do NOT re-enable by just dropping datasets/xView/ back in without
# re-adding the target classes to UNIFIED_CLASSES first.
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
# Roboflow-sourced datasets (UAVDT, SARD, and anything dropped under
# datasets/external/<key>/). Each entry's key is matched against the
# dataset's folder name -- UAVDT/SARD are first-class top-level folders
# (datasets/UAVDT/, datasets/SARD/), matched by prepare_uavdt()/
# prepare_sard() directly; anything under datasets/external/<key>/ is
# matched by folder name via prepare_external(). Same remap-table shape
# either way, so this single dict covers all of them.
# ---------------------------------------------------------------------
EXTERNAL_REMAPS: dict[str, dict[str, str | None]] = {
    # UAVDT -- genuinely drone-captured (not satellite) low-altitude
    # traffic surveillance footage. Verified class list from data.yaml:
    # nc: 3, names: ['bus', 'car', 'truck']
    # (https://universe.roboflow.com/kfupm-v0syf/uavdt-4g4uv). Ships with
    # train/ only, no valid/test -- prepare_uavdt() auto-splits by video
    # sequence (see _auto_split_by_sequence()) rather than training with
    # zero validation data.
    "uavdt": {
        "car":   "car",
        "bus":   "large_vehicle",
        "truck": "large_vehicle",
    },
    # SARD -- Search And Rescue Dataset (Sambolek & Ivasic-Kos, IEEE
    # DataPort, doi:10.21227/ahxm-k331). Genuinely drone-captured, actors
    # simulating SAR scenarios (standing/sitting/walking/running/lying,
    # including exhausted/injured poses). Verified class list from the
    # animesh-shastry/sard_yolo mirror's data.yaml: nc: 1, names:
    # ['person']. Ships with a real train/valid/test split already.
    "sard": {
        "person": "person",
    },
    # Add AU-AIR here once converted to YOLO format via its official
    # toolkit (https://github.com/bozcani/auairdataset) -- its 8 classes
    # (human, car, van, truck, bike, motorbike, bus, trailer) map almost
    # 1:1 onto UNIFIED_CLASSES:
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
            f"({UNIFIED_CLASSES}). If this came from xview_index_remap(), "
            f"that's expected -- xView is currently inactive because "
            f"building/shed/parking_lot were removed. See class_map.py's "
            f"module docstring before re-enabling it.")


def visdrone_index_remap() -> list[int | None]:
    """
    Returns a list where position i = the original VisDrone class index,
    value = the corresponding UNIFIED_CLASSES index (or None if that
    VisDrone class isn't mapped to anything, meaning drop those labels).
    """
    out: list[int | None] = []
    for name in VISDRONE_ORIGINAL_ORDER:
        target = VISDRONE_REMAP.get(name)
        out.append(unified_index(target) if target is not None else None)
    return out


def xview_index_remap() -> list[int | None]:
    """
    INACTIVE -- see module docstring. Will raise a clear ValueError (via
    unified_index()) if actually called, since 'building'/'shed'/
    'parking_lot' aren't in the current UNIFIED_CLASSES. Left implemented
    (not deleted) so re-enabling xView later is a taxonomy edit, not a
    rewrite.
    """
    out: list[int | None] = []
    for name in XVIEW_ORIGINAL_ORDER:
        target = XVIEW_REMAP.get(name)
        out.append(unified_index(target) if target is not None else None)
    return out


def taxonomy_signature() -> str:
    """
    Short hash covering UNIFIED_CLASSES + every REMAP table. Used to detect
    "the taxonomy changed since labels were last remapped" so
    prepare_datasets.py knows to redo the remap from backups rather than
    silently training on stale class indices.
    """
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