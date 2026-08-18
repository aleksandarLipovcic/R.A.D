"""
mosaic_guard.py -- caps the worst-case instance count of Ultralytics'
Mosaic augmentation for Project R.A.D.

Why this exists: VisDrone has some frames with 1000+ annotated instances
(dense crowds/traffic). Ultralytics' mosaic augmentation composites up
to 4 (or 9) source images into one training sample *before* batching --
if a composite happens to pull together several dense frames, that
single slot needs dramatically more memory than a typical batch (loss/
target-assignment cost scales with instance count, not image size),
regardless of --batch/--imgsz. Confirmed against the per-batch training
log: batch 44 (378 -> 3632 instances) and batch 61 (-> 4864 instances)
lined up exactly with the GPU_mem jumps that eventually pushed past this
card's 6GB.

What this does: subclasses Mosaic and overrides get_params() -- the step
where mosaic partner images are picked -- to steer selection away from
combinations that would exceed --max-mosaic-instances, instead of
picking partners uniformly at random. It does NOT shrink the mosaic
grid (4/9); Mosaic's canvas-building code (apply_image) hardcodes
geometry for a fixed grid size, so changing that per-call risks breaking
it. Instead it changes WHICH images fill that grid: candidates are
sampled the normal way, but any candidate that would push the running
instance total over budget is skipped and another tried, up to a retry
limit -- so a mosaic still happens (keeping the accuracy benefit), it's
just steered away from stacking several dense frames together. If the
retry budget is exhausted (e.g. most nearby images happen to be dense),
selection falls back to a two-phase strategy: first it still tries to
respect the budget using everything it's already seen, and only if
*that* still can't fill the remaining slots does it accept over-budget
candidates (lightest-first) as a genuine last resort -- so the cap is
violated only when it is truly unavoidable, not by default.

This is a PROACTIVE fix -- it makes the pathological batches far less
likely in the first place. train.py's startup probe and ongoing VRAM
watchdog stay in place as the REACTIVE safety net, since a single source
image can occasionally be dense enough on its own to need real headroom
even with zero mosaic partners -- no selection policy can fix that,
only imgsz/batch/model-size can.

Deliberately reads only dataset.labels[i]["cls"] (label metadata
Ultralytics already loads into memory from its dataset cache) to peek
instance counts -- NEVER calls get_image_and_label() speculatively for a
candidate we might discard, since that decodes the actual image from
disk and would defeat the point of a *cheap* guard. If that lookup ever
fails, the guard fails conservative (treats the candidate as maximally
dense) rather than permissive -- a bookkeeping failure should never make
this *less* restrictive.
"""

from typing import Any
import os
import random

from ultralytics.data.augment import Mosaic

# Class-level defaults, set once by install_instance_cap() before any
# dataset is built. Kept as class attributes (rather than always passing
# explicit kwargs) because Ultralytics' v8_transforms() instantiates
# Mosaic with a fixed positional/keyword signature we don't control --
# see install_instance_cap()'s docstring for why a straight subclass
# swap, not a wrapped factory function, is what gets patched in.
_DEFAULT_MAX_INSTANCES = 800
_DEFAULT_MAX_TRIES = 40


class InstanceCappedMosaic(Mosaic):
    """Mosaic that steers partner-image selection away from combinations
    that would exceed a total instance-count budget. See module
    docstring for the full rationale."""

    max_instances_default = _DEFAULT_MAX_INSTANCES
    max_tries_default = _DEFAULT_MAX_TRIES

    def __init__(self, dataset, imgsz: int = 640, p: float = 1.0, n: int = 4,
                 max_instances: int | None = None,
                 max_tries: int | None = None):
        super().__init__(dataset=dataset, imgsz=imgsz, p=p, n=n)
        cls = type(self)
        self.max_instances = (max_instances if max_instances is not None
                               else cls.max_instances_default)
        self.max_tries = (max_tries if max_tries is not None
                           else cls.max_tries_default)
        self.trip_count = 0  # how often the retry budget ran out; for logging

        # Oversampling (see prepare_datasets.py's
        # _oversample_sparse_classes()) writes duplicate absolute image
        # paths into oversample_train.txt as additional train: entries,
        # which means the SAME physical image can end up at multiple
        # distinct dataset indices. Deduping only by index (as an
        # earlier version of this file did) would let one mosaic draw
        # that same image twice under two different indices -- not a
        # budget/memory-safety problem (instance counts are still
        # summed correctly either way), but it wastes a partner slot on
        # redundant content instead of the variety mosaic is meant to
        # add, and oversampling makes that more likely than pure chance
        # would. im_files is a standard BaseDataset attribute for YOLO
        # detection datasets; fall back to index-only dedup (pre-
        # existing behavior) if it's ever missing rather than assuming.
        self._im_files = getattr(dataset, "im_files", None)
        if self._im_files is None:
            print("[mosaic_guard] note: dataset has no im_files attribute -- "
                  "falling back to index-only duplicate detection (won't "
                  "catch the same physical image at two different indices, "
                  "e.g. from oversampling).")

    def _dedupe_key(self, idx: int):
        """Identity used to detect 'already selected for this mosaic'.
        Prefers the physical image path (so two dataset indices that
        reference the same image, e.g. via oversample_train.txt-style
        duplicate entries, are correctly treated as one candidate);
        falls back to the raw index if im_files isn't available.

        Runs the path through os.path.normcase before comparing -- on
        Windows (this project's target platform), the same physical
        file can otherwise compare as "different" purely from '/' vs
        '\\' separators or drive-letter casing between however
        Ultralytics built dataset.im_files and however a given path was
        originally written (e.g. prepare_datasets.py's
        str(Path.resolve()) output for oversample_train.txt). Without
        this, the dedup silently stops catching real duplicates on
        exactly the platform this matters most for, instead of failing
        loudly. normcase is a no-op on POSIX."""
        if self._im_files is not None:
            try:
                return os.path.normcase(str(self._im_files[idx]))
            except (IndexError, TypeError):
                pass
        return idx

    def _sample_candidate_index(self) -> int:
        # Mirrors the base class's locality behavior: when buffer_enabled
        # (see Mosaic.__init__), prefer recently-loaded indices so we
        # don't regress disk-cache locality just for the sake of the cap.
        if self.buffer_enabled and len(self.dataset.buffer) > 0:
            return random.choice(list(self.dataset.buffer))
        dataset_len = len(self.dataset)
        if dataset_len <= 0:
            # Shouldn't happen in normal operation, but this is a
            # defensive memory guard -- fail with a clear message
            # instead of a cryptic `ValueError: empty range for
            # randrange()` out of random.randint().
            raise RuntimeError(
                "InstanceCappedMosaic: dataset is empty; cannot select "
                "mosaic partners."
            )
        return random.randint(0, dataset_len - 1)

    def _instance_count(self, index: int) -> int:
        try:
            return len(self.dataset.labels[index]["cls"])
        except (KeyError, TypeError, IndexError, AttributeError) as exc:
            # Conservative fallback: if we can't verify how dense a
            # candidate is, treat it as maximally dense rather than
            # empty. Treating a lookup failure as 0 instances would make
            # the guard *less* restrictive exactly when its bookkeeping
            # is unreliable -- the wrong direction for a memory-safety
            # mechanism. Only catch the failure modes we actually expect
            # (malformed/missing label entries); anything else is a real
            # bug and should still surface normally.
            print(f"[mosaic_guard] warning: instance-count lookup failed "
                  f"for dataset index {index} ({exc!r}); treating as "
                  f"max_instances ({self.max_instances}).")
            return self.max_instances

    @staticmethod
    def _own_instance_count(labels: dict[str, Any]) -> int:
        cls = labels.get("cls")
        return len(cls) if cls is not None else 0

    def get_params(self, labels: dict[str, Any]) -> dict[str, Any]:
        """
        Mosaic.get_params() does two things: pick partner indexes (via
        get_indexes(), which does NOT receive `labels`) and then, using
        those, compute the pixel-placement geometry for the composite.
        Overriding get_params() outright -- as an earlier draft of this
        guard did -- silently threw away that geometry step. Instead:
        stash the current image's own instance count where get_indexes()
        (overridden below) can read it, then fully delegate to Mosaic's
        real get_params() so layout computation stays intact.
        """
        self._own_count = self._own_instance_count(labels)
        return super().get_params(labels)

    def get_indexes(self):
        """
        Replaces Mosaic's uniform-random partner selection. Samples
        candidates the normal way, skipping any index already selected,
        and skips any that would push the running instance total over
        budget, trying another up to self.max_tries -- so a mosaic
        still happens (keeping the accuracy benefit), it's just steered
        away from stacking several dense frames together.

        If the retry budget runs out, falls back in two phases:
          1. Still budget-aware -- sweep everything already seen
             (lightest first) and take anything that still fits the
             remaining budget.
          2. Only if that still isn't enough, accept over-budget
             candidates (lightest first) as a genuine last resort, so
             selection never stalls or raises. This means the cap is
             only violated when literally no seen candidate combination
             fits it, not as the default fallback behavior.
        """
        needed = self.n - 1
        budget = self.max_instances - getattr(self, "_own_count", 0)

        if budget <= 0:
            # The base image alone is already at/over the cap -- no
            # partner choice fixes that, but we can still minimize
            # further damage instead of accepting the first random
            # candidate: sample a pool of *unique* candidates (by
            # physical image, not just index -- see _dedupe_key) and
            # take the least-dense `needed` of them, same policy as the
            # retry-exhausted fallback below. Bounded by max_tries so a
            # tiny dataset (fewer unique candidates than the pool
            # target) can't loop indefinitely.
            pool: dict[Any, tuple[int, int]] = {}
            pool_target = max(needed, self.max_tries // 2)
            attempts = 0
            while len(pool) < pool_target and attempts < self.max_tries:
                idx = self._sample_candidate_index()
                key = self._dedupe_key(idx)
                if key not in pool:
                    pool[key] = (idx, self._instance_count(idx))
                attempts += 1
            ordered = sorted(pool.values(), key=lambda t: t[1])
            selected = [idx for idx, _ in ordered[:needed]]
            while len(selected) < needed:  # pathologically tiny dataset
                selected.append(self._sample_candidate_index())
            return selected

        selected: list[int] = []
        selected_keys: set = set()
        running = 0
        seen: list[tuple[int, int]] = []
        tries = 0

        while len(selected) < needed and tries < self.max_tries:
            tries += 1
            idx = self._sample_candidate_index()
            key = self._dedupe_key(idx)
            if key in selected_keys:
                # Already chosen this image (by index or physical path)
                # for this mosaic -- skip without spending a lookup.
                continue
            count = self._instance_count(idx)
            seen.append((idx, count))
            if running + count <= budget:
                selected.append(idx)
                selected_keys.add(key)
                running += count

        if len(selected) < needed:
            self.trip_count += 1

            # Phase 1: still respect the budget. Sweep everything we've
            # seen so far (lightest first) and take anything that still
            # fits the remaining budget -- this is what the old fallback
            # skipped, and is what let it silently exceed max_instances.
            for idx, count in sorted(seen, key=lambda t: t[1]):
                if len(selected) >= needed:
                    break
                key = self._dedupe_key(idx)
                if key in selected_keys:
                    continue
                if running + count <= budget:
                    selected.append(idx)
                    selected_keys.add(key)
                    running += count

            # Phase 2: genuine last resort. Only now, if nothing seen
            # fits the remaining budget, accept over-budget candidates
            # (lightest first) so selection never stalls.
            if len(selected) < needed:
                for idx, count in sorted(seen, key=lambda t: t[1]):
                    if len(selected) >= needed:
                        break
                    key = self._dedupe_key(idx)
                    if key in selected_keys:
                        continue
                    selected.append(idx)
                    selected_keys.add(key)
                    running += count

            while len(selected) < needed:  # pathologically tiny dataset
                selected.append(self._sample_candidate_index())

        return selected


def install_instance_cap(max_instances: int = _DEFAULT_MAX_INSTANCES,
                          max_tries: int = _DEFAULT_MAX_TRIES) -> None:
    """
    Monkeypatches ultralytics.data.augment.Mosaic -> InstanceCappedMosaic.

    Must be called before any dataset/trainer is built. This works
    because v8_transforms() (ultralytics/data/augment.py) constructs
    Mosaic via a bare module-global name, resolved at CALL time (i.e.
    each time a dataset's build_transforms() runs) rather than at
    v8_transforms' definition time -- so patching the module attribute
    once, early, is sufficient. A plain subclass swap (rather than a
    wrapped factory function) is used specifically so isinstance checks
    elsewhere in Ultralytics (e.g. CopyPaste's internal handling) keep
    working unmodified.
    """
    import ultralytics.data.augment as augment_module

    InstanceCappedMosaic.max_instances_default = max_instances
    InstanceCappedMosaic.max_tries_default = max_tries
    augment_module.Mosaic = InstanceCappedMosaic

    # Sanity-check that the patch actually took effect. This is mostly
    # about catching future-you refactoring this into something that no
    # longer assigns the module attribute correctly (e.g. patching a
    # re-imported/aliased module object) rather than anything that can
    # fail today -- cheap insurance against a silent no-op patch.
    if augment_module.Mosaic is not InstanceCappedMosaic:
        raise RuntimeError(
            "[mosaic_guard] Failed to install InstanceCappedMosaic -- "
            "ultralytics.data.augment.Mosaic was not replaced. Check for "
            "an Ultralytics version change or an import-order issue."
        )

    print(f"[mosaic_guard] Instance-capped mosaic installed "
          f"(max_instances={max_instances}, max_tries={max_tries}) -- "
          f"mosaic partner selection will now actively avoid stacking "
          f"multiple dense VisDrone frames together instead of leaving "
          f"it to chance.")