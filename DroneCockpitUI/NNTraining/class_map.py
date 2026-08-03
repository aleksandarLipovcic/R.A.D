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
diluting training signal. Structural/terrain classes (building, shed,
parking_lot) come from xView -- see prepare_datasets.py's docstring for
how that integrates. Roads/forest/land-cover are deliberately NOT part of
this taxonomy at all (see the note above UNIFIED_CLASSES) -- those need
semantic segmentation, a separate future project, not bounding boxes.

To add a new dataset:
  1. Add a REMAP table below (original class name -> one of UNIFIED_CLASSES,
     or None to drop that class from training entirely).
  2. See prepare_datasets.py for the folder convention external datasets
     need to follow.

Changing UNIFIED_CLASSES or any REMAP table invalidates existing remapped
labels -- prepare_datasets.py detects this automatically (via
_taxonomy_signature()) and re-applies the remap from each dataset's
untouched backup copy, so it's always safe to edit this file and rerun.
"""

import hashlib
import json

# The single shared class list every dataset gets remapped into.
# Order matters -- this is the class index order the model actually trains
# on. Append-only is safest once you have a trained checkpoint you want to
# keep resuming; inserting/removing/reordering entries invalidates it.
UNIFIED_CLASSES = [
    "person",         # 0 - VisDrone: pedestrian + people
    "car",            # 1
    "large_vehicle",  # 2 - van, truck, bus and similar
    "motorcycle",     # 3
    "other_vehicle",  # 4 - bicycle, tricycle, and similar (low priority
                       #     catch-all, kept only so these don't get
                       #     silently misdetected as something else)
    "building",       # 5 - xView: Building, Damaged Building, Facility,
                       #     Aircraft Hangar
    "shed",           # 6 - xView: Shed, Hut/Tent
    "parking_lot",    # 7 - xView: Vehicle Lot
    # NOTE: roads, forest patches, and general land-cover are deliberately
    # NOT included here. Those aren't representable as bounding boxes at
    # all (a road has no natural box; it runs edge-to-edge in an irregular
    # shape) -- they need semantic segmentation, a genuinely different
    # model architecture/pipeline, not another class in this detector.
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

# xView's 60 classes, in the exact index order Ultralytics' own xView.yaml
# assigns (0-59, after its GeoJSON conversion). Only structure/building
# classes are mapped -- deliberately NOT pulling in xView's vehicle classes
# (Small Car, Bus, Truck, etc.) even though they overlap conceptually with
# VisDrone's, since satellite imagery is a different visual domain (scale,
# blur, always-top-down angle) from drone footage, and mixing them risks
# diluting vehicle accuracy rather than adding a clean new capability.
# "Construction Site" is excluded (see XVIEW_REMAP below) -- a large,
# vague, temporary area annotation, not a discrete structure.
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
    # Deliberately excluded:
    # - "Construction Site": a large, vague, temporary area annotation,
    #   not a discrete structure -- would skew box-size statistics.
    # - Storage Tank, Shipping Container(s), Pylon, Tower: genuine
    #   candidates for later if you want them (all discrete,
    #   box-representable "compound" objects) -- not added now since they
    #   weren't asked for; add a line above if you want one.
    # - every xView vehicle/vessel/rail class: deliberately kept out of
    #   this taxonomy (see prepare_datasets.py's docstring for why).
}

# Placeholder for future external datasets dropped under datasets/external/.
# Key = the folder name under datasets/external/, value = {original class
# name from that dataset's own data.yaml -> UNIFIED_CLASSES name or None}.
# prepare_datasets.py will refuse to include a folder it has no entry for
# here, rather than guessing -- add an entry when you add a dataset.
EXTERNAL_REMAPS: dict[str, dict[str, str | None]] = {
    # Example for when xView is manually downloaded and converted:
    # "xview": {
    #     "Building": "building",
    #     "Small Car": "car",
    #     "Bus": "large_vehicle",
    #     ... every other xView class not listed here is dropped ...
    # },
}


def unified_index(name: str) -> int:
    return UNIFIED_CLASSES.index(name)


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
    """Same idea as visdrone_index_remap(), for xView's 60 classes."""
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