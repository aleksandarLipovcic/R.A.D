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
it falls back to filling remaining slots with the lowest-instance-count
candidates seen so far, so selection never stalls or throws.

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
disk and would defeat the point of a *cheap* guard.
"""

from typing import Any
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

    def _sample_candidate_index(self) -> int:
        # Mirrors the base class's locality behavior: when buffer_enabled
        # (see Mosaic.__init__), prefer recently-loaded indices so we
        # don't regress disk-cache locality just for the sake of the cap.
        if self.buffer_enabled and len(self.dataset.buffer) > 0:
            return random.choice(list(self.dataset.buffer))
        return random.randint(0, len(self.dataset) - 1)

    def _instance_count(self, index: int) -> int:
        try:
            return len(self.dataset.labels[index]["cls"])
        except Exception:
            # Never let a bookkeeping lookup failure take down training
            # over what's meant to be a defensive guard.
            return 0

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
        candidates the normal way, but skips any that would push the
        running instance total over budget and tries another, up to
        self.max_tries -- so a mosaic still happens (keeping the
        accuracy benefit), it's just steered away from stacking several
        dense frames together. Falls back to the lowest-instance-count
        candidates seen if the retry budget runs out, so this never
        stalls or raises.
        """
        needed = self.n - 1
        budget = self.max_instances - getattr(self, "_own_count", 0)

        if budget <= 0:
            # The base image alone is already at/over the cap -- no
            # partner choice fixes that, but we can still minimize
            # further damage instead of accepting the first random
            # candidate: sample a pool and take the least-dense `needed`
            # of them, same policy as the retry-exhausted fallback below.
            pool = [self._sample_candidate_index()
                    for _ in range(max(needed, self.max_tries // 2))]
            seen = [(idx, self._instance_count(idx)) for idx in pool]
            seen.sort(key=lambda t: t[1])
            return [idx for idx, _ in seen[:needed]]

        selected: list[int] = []
        running = 0
        seen: list[tuple[int, int]] = []
        tries = 0

        while len(selected) < needed and tries < self.max_tries:
            tries += 1
            idx = self._sample_candidate_index()
            count = self._instance_count(idx)
            seen.append((idx, count))
            if running + count <= budget:
                selected.append(idx)
                running += count

        if len(selected) < needed:
            # Retry budget exhausted (e.g. most nearby images are dense).
            # Fall back to the lowest-instance-count candidates seen so
            # far so selection never stalls -- still a real mosaic, just
            # not a randomly-chosen one for the leftover slot(s).
            self.trip_count += 1
            for idx, count in sorted(seen, key=lambda t: t[1]):
                if len(selected) >= needed:
                    break
                if idx not in selected:
                    selected.append(idx)
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

    print(f"[mosaic_guard] Instance-capped mosaic installed "
          f"(max_instances={max_instances}, max_tries={max_tries}) -- "
          f"mosaic partner selection will now actively avoid stacking "
          f"multiple dense VisDrone frames together instead of leaving "
          f"it to chance.")