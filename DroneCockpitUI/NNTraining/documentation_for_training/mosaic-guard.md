# `mosaic_guard.py` — Mosaic instance-count cap

**File:** `mosaic_guard.py`
**Class:** `InstanceCappedMosaic(Mosaic)` — subclasses Ultralytics' own
`Mosaic` augmentation
**Installed by:** `train.py` via `install_instance_cap()`, monkeypatching
Ultralytics' augmentation pipeline before training starts

## Why this exists

VisDrone has frames with **1000+ annotated instances** in a single image.
Ultralytics' Mosaic augmentation composites 4 or 9 source images into one
training sample *before* batching — if a composite happens to pull
together several dense frames, that one slot needs far more memory than a
typical batch, because loss/target-assignment cost scales with instance
count, not image size.

This isn't theoretical for this project — it's confirmed against real
training logs: batch 44 (378 → 3632 instances) and batch 61 (→ 4864
instances) lined up **exactly** with the GPU memory jumps that pushed past
this card's 6GB budget.

## What it does

Subclasses `Mosaic` and overrides `get_params()` — the method where
partner images for a composite are picked — to steer selection away from
combinations that would exceed `--max-mosaic-instances`, instead of the
stock behavior of picking uniformly at random.

It does **not** shrink the mosaic grid itself — `apply_image()`'s geometry
code hardcodes grid size, and that isn't touched.

### Selection algorithm

1. Candidates are sampled normally (same distribution as stock Mosaic).
2. Any candidate that would push the running instance-count total over
   budget is skipped and another is tried, up to a retry limit.
3. If the retry budget runs out, two fallback phases:
   - **Phase 1** — still budget-aware: sweep everything already seen
     during this call and take what fits.
   - **Phase 2** — genuine last resort: accept over-budget candidates,
     lightest (fewest-instance) first, so mosaic composition never stalls
     the training loop entirely.

Only `dataset.labels[i]["cls"]` (already-loaded label metadata) is read to
peek instance counts — a candidate image is **never decoded from disk**
just to maybe discard it. If that lookup fails for some reason, the
candidate is treated as maximally dense — fail conservative, never
permissive, so a lookup failure can't accidentally let an oversized
composite through.

## Proactive, not reactive

Explicitly framed as the **proactive** half of this project's VRAM
protection — `train.py`'s startup probe and ongoing VRAM watchdog (see
[train.md](train.md#wddm--vram-spillover-guards)) remain the **reactive**
safety net, since a single source image can occasionally be dense enough
alone to need real headroom even with zero mosaic partners at all.

## Call-order dependency (verified against Ultralytics internals)

```
BaseMixTransform.__call__()
  -> self.get_params(labels)
  -> Mosaic.get_params()
       -> super().get_params(labels)   [BaseMixTransform.get_params]
       -> self.get_indexes()
```

`InstanceCappedMosaic.get_params()` stashes its own running count
(`self._own_count`) **before** calling `super().get_params(labels)` — this
ordering is guaranteed to run before `get_indexes()` reads that count on
every call, since `get_indexes()` is never reached any other way in this
pipeline. This dependency on Ultralytics' internal call order is exactly
the kind of thing `verify_pipeline_assumptions.py` and `setup.py`'s
version-fingerprint step exist to catch if a future Ultralytics upgrade
changes it (see [setup.md](setup.md) and
[verify-pipeline-assumptions.md](verify-pipeline-assumptions.md)).

## Fixed bug: the last-resort fallback's missing dedup check

Documented in the source as a review-flagged fix. `get_indexes()`'s final
catch-all fallback — used only when Phase 1 and Phase 2 together still
can't fill the needed number of partner slots (e.g. a small dataset, or
heavy index-space duplication from `prepare_datasets.py`'s oversampling
collapsing many indices onto few unique physical files) — used to sample
with **zero dedup check**, unlike every other path in this method. That
silently violated this class's own stated guarantee (see
`_dedupe_key()`'s docstring: "a mosaic never wastes a partner slot on the
same image twice") in exactly the scenario most likely to reach this
fallback.

Reproduced directly in testing: with only 2 unique physical files behind a
6-index dataset and `max_tries=5`, the old code returned the same physical
image twice for a 3-slot mosaic. The fallback now respects the same
`dedupe_key` check as Phase 1/2, bounded by a `fallback_cap` try limit, and
only accepts a genuine duplicate as an absolute last resort (with a
one-time printed explanation) if the dataset truly doesn't have enough
unique images to fill the mosaic — which was always the implicit
assumption behind this code, just never actually enforced on this
particular path.

## Installation

```python
install_instance_cap(max_instances: int = _DEFAULT_MAX_INSTANCES, ...)
```

Called from `train.py` before the training loop starts, wiring
`InstanceCappedMosaic` in as Ultralytics' Mosaic implementation for that
run. `--max-mosaic-instances` on `train.py`'s own CLI controls the budget
this enforces (default tuned to the project's 6GB card — see
[train.md](train.md)).
