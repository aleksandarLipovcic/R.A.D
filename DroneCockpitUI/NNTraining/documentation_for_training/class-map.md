# `class_map.py` — unified class taxonomy

**File:** `class_map.py`
**Read by:** nearly every other script in this pipeline —
`prepare_datasets.py`, `train.py`, `check_label_gaps.py`,
`cross_reference_gaps.py`, `generate_pseudo_labels.py`, and more.

## Responsibility

Defines the **one** class taxonomy every source dataset gets remapped into
before training, so multiple differently-labeled datasets can be merged
into one coherent model.

```python
UNIFIED_CLASSES = [
    "person",         # VisDrone (pedestrian+people) + SARD (SAR poses)
    "car",            # VisDrone + UAVDT
    "large_vehicle",  # van/truck/bus (VisDrone + UAVDT)
    "motorcycle",     # VisDrone
    "other_vehicle",  # bicycle/tricycle catch-all (low priority)
]
```

## Design decisions worth knowing

- **Bicycle/tricycle fine-grained distinctions are not a priority** — both
  fold into `other_vehicle` as a deliberate project decision, not an
  oversight.
- **`building`/`shed`/`parking_lot` were removed** from an earlier
  taxonomy version. xView (satellite imagery, intact structures) was the
  only source for them, and mixing xView's satellite-angle frames with
  VisDrone's drone-native frames in the same Mosaic batch hurt
  vehicle/person accuracy (different scale/angle composited together),
  while building recall was already poor (0.199) on real disaster
  footage. xView's remap tables (`XVIEW_ORIGINAL_ORDER`, `XVIEW_REMAP`)
  are kept in the file, **inactive**, rather than deleted — in case a
  drone-native structure/damage dataset shows up later to replace it.
- **UAVDT and SARD are both genuinely drone-native** (like VisDrone), so
  Mosaic-mixing them together is fine — the domain-mismatch problem was
  specific to xView's satellite framing, not to combining multiple
  datasets in general.

## The cross-dataset label-gap caveat

Stated directly in the module docstring, and the entire reason the
label-gap pipeline (`check_label_gaps.py` onward, see
[ML_PIPELINE.md](../ML_PIPELINE.md)) exists:

> UAVDT frames were never annotated for people, SARD frames never for
> vehicles. An unlabeled person in a UAVDT frame (or vehicle in SARD) acts
> as a false "background" signal for that class/location. Low risk given
> the domains barely overlap, but worth remembering if per-class precision
> looks off.

## Remap tables

| Table | Source | Status |
|---|---|---|
| `VISDRONE_REMAP` (+ `VISDRONE_ORIGINAL_ORDER`) | VisDrone's 10 native classes | Active |
| `XVIEW_REMAP` (+ `XVIEW_ORIGINAL_ORDER`) | xView's 60 native classes | **Inactive** — `xview_index_remap()` deliberately raises against the current taxonomy rather than silently producing wrong indices |
| `EXTERNAL_REMAPS` | Roboflow-style datasets, keyed by folder name (`uavdt`, `sard`; a commented-out `auair` entry sketches a future addition) | Active |

To add a new dataset: add a `REMAP` table here (original class name →
`UNIFIED_CLASSES` name, or `None` to drop that class entirely), then see
[prepare-datasets.md](prepare-datasets.md) for the folder-layout side of
onboarding it.

## `taxonomy_signature()`

Hashes `UNIFIED_CLASSES` plus every remap table (VisDrone, xView, external)
into a short SHA-256 prefix. `prepare_datasets.py` compares this against a
signature file it wrote last run to detect any taxonomy edit — a changed
signature triggers a full re-remap **from each dataset's untouched
backup**, so editing this file and rerunning `prepare_datasets.py` is
always safe for original labels. (See
[prepare-datasets.md](prepare-datasets.md) for the pseudo-label caveat this
does *not* cover.)

## `unified_index(name)`

The single point every remap function routes through to turn a class name
into its `UNIFIED_CLASSES` index. Raises a descriptive `ValueError` for any
name not in the current taxonomy — this is the mechanism that makes
`xview_index_remap()` fail loudly instead of producing silently-wrong
indices if xView is ever re-enabled without also restoring
`building`/`shed`/`parking_lot` to `UNIFIED_CLASSES`.
