"""
mosaic_guard.py -- caps the worst-case instance count of Ultralytics'
Mosaic augmentation for Project R.A.D.

Why: VisDrone has frames with 1000+ annotated instances. Mosaic
composites up to 4/9 source images into one training sample *before*
batching -- if a composite pulls together several dense frames, that
one slot needs way more memory than a typical batch (loss/target-
assignment cost scales with instance count, not image size). Confirmed
against training logs: batch 44 (378->3632 instances) and batch 61
(->4864) lined up exactly with the GPU_mem jumps that pushed past this
card's 6GB.

What this does: subclasses Mosaic and overrides get_params() (where
partner images are picked) to steer selection away from combinations
that would exceed --max-mosaic-instances, instead of picking uniformly
at random. Does NOT shrink the mosaic grid (apply_image()'s geometry
code hardcodes grid size). Candidates are sampled normally; any that
would push the running total over budget are skipped and another is
tried, up to a retry limit. If the retry budget runs out, falls back
in two phases: (1) still budget-aware -- sweep everything already seen
and take what fits, (2) genuine last resort -- accept over-budget
candidates (lightest first) so selection never stalls.

This is PROACTIVE -- train.py's startup probe + VRAM watchdog remain
the REACTIVE safety net, since a single source image can occasionally
be dense enough alone to need real headroom even with zero partners.

Only reads dataset.labels[i]["cls"] (already-loaded label metadata) to
peek instance counts -- never decodes a candidate image from disk just
to maybe discard it. If that lookup fails, treats the candidate as
maximally dense (fail conservative, never permissive).

Call order this relies on (verified against ultralytics/data/augment.py):
BaseMixTransform.__call__() -> self.get_params(labels) -> Mosaic.get_params()
-> super().get_params(labels) [BaseMixTransform.get_params] -> self.get_indexes().
So InstanceCappedMosaic.get_params() stashing self._own_count *before*
calling super().get_params(labels) is guaranteed to run before
get_indexes() reads it, every call -- get_indexes() is never reached any
other way in this pipeline.

  [FIX -- review item] get_indexes()'s final catch-all fallback (used
  only when Phase 1 and Phase 2 together still can't fill `needed`
  slots from everything sampled so far -- e.g. a small dataset or
  heavy index-space duplication from prepare_datasets.py's oversampling
  collapsing many indices onto few unique physical files) used to
  sample with zero dedup check, unlike every other path in this
  method. That silently violated this class's own stated guarantee
  (see _dedupe_key()'s docstring: "a mosaic never wastes a partner
  slot on the same image twice") in exactly the scenario most likely
  to reach this fallback. Reproduced directly: with only 2 unique
  physical files behind a 6-index dataset and max_tries=5, the old
  code returned the same physical image twice for a 3-slot mosaic.
  The fallback now respects the same dedupe_key check as Phase 1/2,
  bounded by fallback_cap tries, and only accepts a genuine duplicate
  as an absolute last resort (with a one-time printed explanation) if
  the dataset truly doesn't have enough unique images to fill the
  mosaic -- which was always the implicit assumption, just never
  enforced on this path.

  [FIX -- review item] self.trip_count was incremented every time the
  retry budget ran out (comment: "for logging") but nothing anywhere
  -- this module or train.py -- ever printed, read, or otherwise
  surfaced it, so there was no way to tell from a training run how
  often the instance-cap guard had to fall back to Phase 1/2. It now
  prints a message the first few times it trips and periodically
  (log-scale) after that, so frequent trips are visible without
  spamming the log on a run where they're rare (the expected case).

  [POLISH -- review follow-up] get_indexes()'s main sampling loop kept
  `seen` as a plain list, so redrawing the same already-seen-but-not-
  yet-accepted candidate index (common once the budget gets tight)
  re-ran the instance-count lookup and appended a second entry. Not a
  correctness bug -- Phase 1/2 skip anything already in selected_keys
  -- just wasted label lookups. `seen` is now keyed by dedupe_key so
  each physical image's count is looked up at most once per call.

  [POLISH -- review follow-up] the duplicate-fallback warning in
  _fill_remaining_unique() only ever printed on the 1st/2nd/5th
  occurrence (_TRIP_REPORT_MULTIPLES), unlike trip_count's full
  log-scale schedule -- so a duplicate-fallback problem that turned
  frequent on a small dataset would go quiet after the 5th time.
  Both counters now go through the same _should_report() log-scale
  check.
"""

from typing import Any
import os
import random

from ultralytics.data.augment import Mosaic

_DEFAULT_MAX_INSTANCES = 800
_DEFAULT_MAX_TRIES = 40

# [FIX] trip_count log-scale reporting points -- print on the 1st,
# 2nd, 5th, 10th, 20th, 50th, ... trip, so a rare event is visible
# immediately but a frequent one doesn't spam every single mosaic call.
_TRIP_REPORT_MULTIPLES = (1, 2, 5)


def _should_report(n: int) -> bool:
    """Shared log-scale reporting schedule: fires on the 1st, 2nd, 5th,
    10th, 20th, 50th, 100th, ... occurrence. Used for both trip_count
    and the duplicate-fallback counter so a rare event is visible
    immediately and a frequent one doesn't spam the log.

    [FIX -- review item] The previous formula (`n % (10 ** (len(str(n))
    - 1)) == 0`) doesn't actually implement a 1-2-5 schedule -- it
    fires on EVERY multiple of 10 within a decade (10, 20, 30, ..., 90)
    and every multiple of 100 within a century (100, 200, ..., 900),
    not just 10/20/50/100/200/500. That's the opposite of what both
    call sites (trip_count, _duplicate_fallback_count) need this for:
    on a run where either counter climbs into the tens or hundreds --
    plausible on a small/heavily-oversampled dataset, exactly the case
    this guard exists for -- the old formula prints roughly 10x more
    often than intended, reintroducing the log-spam problem this
    function was written to avoid. Fixed to check membership against
    the actual {1,2,5} x 10^k sequence instead."""
    if n in _TRIP_REPORT_MULTIPLES:
        return True
    if n < 10:
        return False
    magnitude = 10 ** (len(str(n)) - 1)
    return n in (magnitude, 2 * magnitude, 5 * magnitude)


class InstanceCappedMosaic(Mosaic):
    """Mosaic that steers partner-image selection away from combinations
    that would exceed a total instance-count budget. See module
    docstring for the rationale."""

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
        self.trip_count = 0  # how often the retry budget ran out; now logged, see _report_trip()
        self._duplicate_fallback_count = 0  # how often a true duplicate had to be accepted

        # Oversampling (prepare_datasets._oversample_sparse_classes())
        # writes duplicate absolute image paths as extra train: entries,
        # so the same physical image can sit at multiple dataset indices.
        # Dedupe by physical path (via im_files) rather than index alone,
        # so a mosaic never wastes a partner slot on the same image
        # twice under two different indices.
        self._im_files = getattr(dataset, "im_files", None)
        if self._im_files is None:
            print("[mosaic_guard] note: dataset has no im_files attribute -- "
                  "falling back to index-only duplicate detection.")

    def _dedupe_key(self, idx: int):
        """Identity for 'already selected this mosaic'. Prefers the
        physical path (normcased -- on Windows the same file can compare
        'different' purely from separator/case differences between
        however im_files was built vs. how a path was written elsewhere,
        e.g. oversample_train.txt); falls back to raw index."""
        if self._im_files is not None:
            try:
                return os.path.normcase(str(self._im_files[idx]))
            except (IndexError, TypeError):
                pass
        return idx

    def _sample_candidate_index(self) -> int:
        # Mirrors base-class locality: prefer recently-loaded indices
        # when buffering, so we don't regress disk-cache locality.
        if self.buffer_enabled and len(self.dataset.buffer) > 0:
            return random.choice(list(self.dataset.buffer))
        dataset_len = len(self.dataset)
        if dataset_len <= 0:
            raise RuntimeError(
                "InstanceCappedMosaic: dataset is empty; cannot select "
                "mosaic partners.")
        return random.randint(0, dataset_len - 1)

    def _instance_count(self, index: int) -> int:
        try:
            return len(self.dataset.labels[index]["cls"])
        except (KeyError, TypeError, IndexError, AttributeError) as exc:
            # Fail conservative: treat an unreadable candidate as
            # maximally dense, not empty -- a lookup failure should
            # never make this guard less restrictive.
            print(f"[mosaic_guard] warning: instance-count lookup failed "
                  f"for dataset index {index} ({exc!r}); treating as "
                  f"max_instances ({self.max_instances}).")
            return self.max_instances

    @staticmethod
    def _own_instance_count(labels: dict[str, Any]) -> int:
        cls = labels.get("cls")
        return len(cls) if cls is not None else 0

    def _report_trip(self) -> None:
        """[FIX] trip_count was tracked but never surfaced anywhere.
        Prints on a log-scale schedule (1st, 2nd, 5th, 10th, 20th, ...
        trip) so occasional trips are visible without spamming a long
        training run if they become frequent -- frequency itself is a
        useful signal that --max-mosaic-instances or --max-tries may
        need adjusting."""
        if _should_report(self.trip_count):
            print(f"[mosaic_guard] note: retry budget exhausted for a "
                  f"mosaic partner selection ({self.trip_count} time(s) so "
                  f"far this run) -- fell back to a budget-aware sweep of "
                  f"already-seen candidates. Non-fatal; if this climbs "
                  f"steadily, consider raising --mosaic-max-tries or "
                  f"lowering --max-mosaic-instances.")

    def get_params(self, labels: dict[str, Any]) -> dict[str, Any]:
        """Mosaic.get_params() picks partner indexes (via get_indexes(),
        which doesn't see `labels`) then computes placement geometry.
        Stash this image's own instance count for get_indexes() to read,
        then delegate to the real get_params() so layout stays intact."""
        self._own_count = self._own_instance_count(labels)
        return super().get_params(labels)

    def get_indexes(self):
        """Replaces Mosaic's uniform-random partner selection with a
        budget-aware sample: skip candidates that would push the running
        instance total over budget, retry up to max_tries. If the retry
        budget runs out, fall back in two phases -- (1) still
        budget-aware, sweep everything already seen for what fits, then
        (2) accept over-budget candidates (lightest first) as a last
        resort so selection never stalls."""
        needed = self.n - 1
        budget = self.max_instances - getattr(self, "_own_count", 0)

        if budget <= 0:
            # Base image alone is already at/over cap -- minimize
            # further damage: sample a pool of unique candidates and
            # take the least-dense `needed` of them.
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
            selected_keys = {self._dedupe_key(idx) for idx in selected}
            self._fill_remaining_unique(selected, selected_keys, needed)
            return selected

        selected: list[int] = []
        selected_keys: set = set()
        running = 0
        # [POLISH] keyed by dedupe_key (not a plain list) so redrawing
        # the same not-yet-accepted candidate doesn't re-run
        # _instance_count() or add a redundant entry for Phase 1/2 to
        # sort through.
        seen: dict[Any, tuple[int, int]] = {}
        tries = 0

        while len(selected) < needed and tries < self.max_tries:
            tries += 1
            idx = self._sample_candidate_index()
            key = self._dedupe_key(idx)
            if key in selected_keys:
                continue
            if key not in seen:
                seen[key] = (idx, self._instance_count(idx))
            idx, count = seen[key]
            if running + count <= budget:
                selected.append(idx)
                selected_keys.add(key)
                running += count

        if len(selected) < needed:
            self.trip_count += 1
            self._report_trip()

            # Phase 1: still budget-aware -- sweep what we've seen.
            for key, (idx, count) in sorted(seen.items(), key=lambda kv: kv[1][1]):
                if len(selected) >= needed:
                    break
                if key in selected_keys:
                    continue
                if running + count <= budget:
                    selected.append(idx)
                    selected_keys.add(key)
                    running += count

            # Phase 2: last resort -- accept over-budget, lightest first.
            if len(selected) < needed:
                for key, (idx, count) in sorted(seen.items(), key=lambda kv: kv[1][1]):
                    if len(selected) >= needed:
                        break
                    if key in selected_keys:
                        continue
                    selected.append(idx)
                    selected_keys.add(key)
                    running += count

            # [FIX] used to be a bare `while len(selected) < needed:
            # selected.append(self._sample_candidate_index())` with NO
            # dedupe check -- the only place in this method that could
            # silently return the same physical image twice, breaking
            # the class's own stated guarantee. Now goes through the
            # same unique-candidate search as everywhere else, and only
            # accepts a genuine duplicate if the dataset truly can't
            # supply enough unique images (logged when that happens).
            self._fill_remaining_unique(selected, selected_keys, needed)

        return selected

    def _fill_remaining_unique(self, selected: list, selected_keys: set,
                                 needed: int) -> None:
        """Tops `selected` up to `needed` entries, preferring a physical
        image not already in `selected_keys`. Bounded by a generous try
        budget so a pathologically tiny dataset can't hang training;
        if that budget is exhausted, accepts duplicates as a true last
        resort and logs it (log-scale) so the degradation is visible
        rather than silent."""
        fallback_cap = max(self.max_tries * 4, needed * 20, 20)
        attempts = 0
        accepted_duplicate = False
        while len(selected) < needed:
            idx = self._sample_candidate_index()
            key = self._dedupe_key(idx)
            attempts += 1
            if key in selected_keys and attempts < fallback_cap:
                continue
            if key in selected_keys:
                accepted_duplicate = True
            selected.append(idx)
            selected_keys.add(key)
        if accepted_duplicate:
            self._duplicate_fallback_count += 1
            # [POLISH] shares the same log-scale schedule as
            # _report_trip() instead of only firing on 1st/2nd/5th, so
            # a duplicate-fallback problem that becomes frequent stays
            # visible instead of going quiet after the 5th occurrence.
            if _should_report(self._duplicate_fallback_count):
                print(f"[mosaic_guard] warning: dataset didn't have enough "
                      f"unique images to fill a mosaic without repeating one "
                      f"({self._duplicate_fallback_count} time(s) this run) "
                      f"-- only a concern if this is frequent, which would "
                      f"point to a very small dataset or --max-tries set "
                      f"too low relative to dataset size.")


def install_instance_cap(max_instances: int = _DEFAULT_MAX_INSTANCES,
                          max_tries: int = _DEFAULT_MAX_TRIES) -> None:
    """Monkeypatches ultralytics.data.augment.Mosaic -> InstanceCappedMosaic.
    Must run before any dataset/trainer is built -- v8_transforms()
    resolves the `Mosaic` name at call time, so patching the module
    attribute once, early, is enough. A subclass swap (not a wrapped
    factory) keeps isinstance checks elsewhere in Ultralytics working."""
    import ultralytics.data.augment as augment_module

    InstanceCappedMosaic.max_instances_default = max_instances
    InstanceCappedMosaic.max_tries_default = max_tries
    augment_module.Mosaic = InstanceCappedMosaic

    if augment_module.Mosaic is not InstanceCappedMosaic:
        raise RuntimeError(
            "[mosaic_guard] Failed to install InstanceCappedMosaic -- "
            "ultralytics.data.augment.Mosaic was not replaced. Check for "
            "an Ultralytics version change or an import-order issue.")

    print(f"[mosaic_guard] Instance-capped mosaic installed "
          f"(max_instances={max_instances}, max_tries={max_tries}).")