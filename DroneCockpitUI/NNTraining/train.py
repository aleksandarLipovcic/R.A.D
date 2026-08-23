"""
train.py — Fine-tune YOLO26 for Project R.A.D's aerial detection model
=======================================================================
Defaults are tuned for a 6GB laptop RTX 3060.

PIPELINE: trains a UNIFIED taxonomy (person/car/large_vehicle/motorcycle/
other_vehicle, see class_map.py) spanning VisDrone + UAVDT + SARD (see
prepare_datasets.py) -- all genuine drone-native footage, so free Mosaic
compositing across them is intentional (unlike the now-inactive xView
satellite addition, which hurt accuracy by mixing domains).
generate_pseudo_labels.py --apply cross-labels each source's originally-
missing classes (UAVDT: person; SARD: vehicles) once an image is fully
reviewed -- see evaluate_per_source()'s docstring before drawing
conclusions from a near-zero per-class row.

MODEL SIZE: --model defaults to "yolo26s.pt" (fast, known-fitting, good
for validating the pipeline itself). --model auto re-enables the
yolo26m -> yolo26s -> yolo26n fallback search, each candidate checked by
the VRAM probes below. yolo26m @ 960px is the real target once a yolo26s
smoke test looks right.

DEGRADATION AUGMENTATION: training sources are clean digital footage;
the deployment camera (E5-FPV) is analog CVBS (1500TVL, NTSC/PAL,
0.0001 lux min, 6mm/100deg lens, 5.8GHz link) -- a real train/deploy
domain gap. install_degradation_augment() monkeypatches Ultralytics'
Albumentations wrapper (train split only) with transforms calibrated
against that datasheet instead of generic noise -- see
build_degradation_transform()/install_degradation_augment() for the
per-transform reasoning and version handling across albumentations
1.x/2.x.

  [FIX -- review item] OpticalDistortion (the lens-barrel-distortion
  piece) was silently skipping on the installed albumentations 2.0.8:
  the two candidate parameter sets covered the pre-2.0 API
  (distort_limit + shift_limit) and the 2.2+ API (distort_range + mode),
  but not the actual 2.0.x-era API in between, which already dropped
  shift_limit in favor of `mode` but hadn't yet renamed distort_limit
  to distort_range. A third, version-correct candidate
  (distort_limit + mode, no shift_limit) has been added below so this
  transform now builds cleanly on 2.0.8 instead of being skipped.

WDDM / VRAM: on Windows, an over-budget CUDA allocation silently spills
into system RAM instead of raising OOM, causing catastrophic slowdowns
well after training looked healthy. Two callbacks guard this, both
raising VRAMBudgetExceeded like a real OOM: make_startup_probe_callback()
(first few batches) and make_ongoing_watchdog_callback() (every epoch,
whole run). mosaic_guard.py's --max-mosaic-instances is the proactive
half -- dense mosaic composites were the actual root cause of the VRAM
"staircase" this project hit twice. --vram-safety-margin controls the
threshold both callbacks trip at (default 0.90).

  [FIX -- review item] Both callbacks used to check only
  max_memory_allocated() (actual tensor bytes) against a 90% safety
  margin. Confirmed on a --batch 3 run: reserved VRAM (GPU_mem in the
  progress bar -- the allocator's pool, and the closer proxy for what
  actually drives WDDM's page-out decision) hit 94.6% while allocated
  apparently stayed under the trip line, and the run oscillated
  0.4-2.4 it/s between epochs (textbook staircase) for 10.7 hours
  without either probe firing. Both now also check
  max_memory_reserved() against the same margin.

DISK CACHE DRIVE: --cache disk caches decoded images between epochs as
.npy files, but Ultralytics writes those files next to the original
source images -- i.e. onto whatever drive the dataset itself lives on,
with no built-in way to point them elsewhere (an open, unimplemented
feature request: https://github.com/ultralytics/ultralytics/issues/18285).
On a laptop where the dataset drive is nearly full but another mounted
drive has room to spare, this silently disables caching entirely rather
than using the free space that's actually available. install_disk_cache_
redirect() retargets the .npy cache files onto whichever mounted drive
currently has the most free space, BEFORE Ultralytics' own disk-space
check runs -- so that check still makes the real pass/fail call, just
against a drive with room. See its docstring for the mechanism.

CRASH-SAFETY: last.pt/best.pt are overwritten every epoch, so --resume
loses at most one in-progress epoch. --save-period adds numbered
snapshots. See _is_resumable_checkpoint() for why a completed run's
last.pt can't be resumed, and _verify_resume_taxonomy() for why a
completed OR in-progress run's taxonomy is checked before resuming.

EVALUATION: --patience (30) is the main early-stop guard; best.pt is
always the best-val epoch. check_overfitting() flags aggregate box-loss
divergence; report_per_class_map()/evaluate_per_source() catch what an
aggregate can hide (e.g. person -- the SAR priority class -- lagging
while vehicles improve). --copy-paste is currently a no-op on this
pipeline (needs segmentation polygons this bbox-only data doesn't
have) -- mixup and --oversample-classes are what actually protects
sparse classes.

BASELINE HISTORY (update after each real run):
  yolo26n, 640px, VisDrone-10cls        -> mAP50 ~0.31 (plateaued)
  yolo26s, 960px, VisDrone-10cls        -> mAP50 ~0.465 (every class up)
  yolo26m, 960px, unified               -> never fit (WDDM spillover)
  yolo26s, 960px, unified+xView         -> WDDM spillover epoch 2
  yolo26m, 640px, unified, 3ep smoke    -> mAP50 0.380 (STALE, predates
                                            pseudo-label --apply merge)
  yolo26s, 640px, unified, post-mosaic_guard -> ~2GB VRAM, full-run
                                                 mAP: TBD
  yolo26m, 960px, unified, 5ep smoke (2026-08-19) -> mAP50 0.576,
                                            mAP50-95 0.336 (all 8/9
                                            degradation transforms now
                                            build cleanly, incl. the
                                            previously-skipped optical
                                            distortion -- see FIX note
                                            above)

USAGE:
    python train.py                             # yolo26s, unified taxonomy
    python train.py --model auto                # yolo26m->s->n fallback search
    python train.py --model yolo26s.pt --imgsz 640 --batch -1 --epochs 2
    python train.py --imgsz 960 --batch 1 --epochs 5   # smoke test at target imgsz
    python train.py --imgsz 640 --batch 4        # fallback if 960px trips WDDM probes
    python train.py --model yolo26m.pt --imgsz 960     # real target run
    python train.py --data VisDrone.yaml --model yolo26n.pt --imgsz 640
    python train.py --resume [--name RUN_NAME]
    python train.py --export-only [--weights PATH]
"""

import argparse
import csv
import hashlib
import json
import os
import platform
import shutil
import time
from pathlib import Path

# Must be set before torch is imported (read at CUDA init time).
# expandable_segments isn't supported on Windows (silent no-op there,
# but prints a warning) -- only included on non-Windows.
_alloc_conf_parts = ["garbage_collection_threshold:0.8"]
if platform.system() != "Windows":
    _alloc_conf_parts.insert(0, "expandable_segments:True")
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", ",".join(_alloc_conf_parts))

import yaml
from ultralytics import YOLO

import prepare_datasets
from class_map import UNIFIED_CLASSES
from mosaic_guard import install_instance_cap

SCRIPT_DIR = Path(__file__).resolve().parent
RUNS_PROJECT = SCRIPT_DIR / "runs" / "detect"

# per_source_val.json is written by prepare_datasets.py into ITS
# DATASETS_DIR, which follows Ultralytics' own global 'datasets_dir'
# setting, not necessarily a "datasets" folder next to this script --
# resolve the same way prepare_datasets.py does so both scripts agree.
try:
    from ultralytics.utils import SETTINGS
    DATASETS_DIR = Path(SETTINGS.get("datasets_dir", SCRIPT_DIR / "datasets"))
except Exception:
    DATASETS_DIR = SCRIPT_DIR / "datasets"

PER_SOURCE_MANIFEST_PATH = DATASETS_DIR / "per_source_val.json"

# Tried in this order for --model auto. Biggest first -- only given up
# on if the probes below confirm it genuinely doesn't fit.
MODEL_SIZE_CANDIDATES = ["yolo26m.pt", "yolo26s.pt", "yolo26n.pt"]


class VRAMBudgetExceeded(Exception):
    """Raised by either preflight callback when memory/timing signals
    indicate this model size doesn't fit -- see WDDM note above for why
    we can't just wait for a normal CUDA OOM on Windows.

    reason is "memory" (peak allocated/reserved over the safety margin,
    or the startup probe's per-batch time heuristic -- both genuine
    "this model size doesn't fit" signals) or "epoch_time" (the ongoing
    watchdog's --max-epoch-minutes ceiling on its own, with no memory
    signal alongside it).

    [FIX -- review item] Only "memory" should trigger --model auto's
    fallback to a smaller size. A slow epoch on its own isn't evidence
    a model doesn't fit -- it can just as easily be disk I/O
    contention, a competing process, or a one-off slow epoch, none of
    which "try a smaller model" actually fixes. Previously both reasons
    were treated identically, so --max-epoch-minutes could silently
    downgrade model size for a problem a smaller model wouldn't solve.
    See train_with_model_fallback()'s handling of this distinction."""

    def __init__(self, message, reason="memory"):
        super().__init__(message)
        self.reason = reason


def make_startup_probe_callback(device, warmup_batches=2, probe_batches=3,
                                  max_batch_seconds=5.0, safety_margin=0.90):
    """on_train_batch_end callback: watches the first few real training
    batches so an obviously-too-big model size gets caught in seconds,
    not hours. Complements make_ongoing_watchdog_callback(), which keeps
    watching for the rest of the run (confirmed: yolo26s passed this
    check cleanly and still spilled ~2800 batches later).

    [FIX -- review item] Both VRAM checks here used to look only at
    max_memory_allocated() (actual tensor bytes) against the 90%
    safety margin. Confirmed on a --batch 3 run: GPU_mem (the
    allocator's *reserved* pool, which is what the progress bar shows
    and what actually governs whether WDDM decides to page out) hit
    5.81/6.14GB (94.6%) while allocated apparently stayed under the
    90% trip line -- the run oscillated 0.4-2.4 it/s between epochs
    (the exact WDDM "staircase" this module's docstring already
    describes) for 10.7 hours without either probe firing. Both checks
    now also compare max_memory_reserved() against the same margin --
    reserved is the closer proxy for what WDDM actually reacts to.
    torch.cuda.reset_peak_memory_stats() (called below) resets both
    the allocated *and* reserved peak trackers, so no extra reset is
    needed for this."""
    state = {"count": 0, "checked": False, "last_t": None}

    def callback(trainer):
        import torch

        if not torch.cuda.is_available():
            # --device cpu is a supported sanity-check option; every
            # torch.cuda.* call below is meaningless there.
            if not state["checked"]:
                state["checked"] = True
                print("[startup probe] CUDA not available (--device cpu?) "
                      "-- VRAM probe skipped. No WDDM safety net this run.")
            return

        if state["checked"]:
            return

        # [FIX -- review item] CUDA execution is async: without a sync
        # here, time.time() can measure how fast the CPU issues queued
        # kernels rather than how long the GPU actually took on this
        # batch. Ultralytics' own progress-bar update (loss.item())
        # usually forces a sync anyway, but that's an implementation
        # detail this probe shouldn't quietly depend on for a
        # correctness-sensitive timing measurement. Only runs for the
        # first warmup_batches + probe_batches calls (5 by default), so
        # this sync is not a meaningful training-speed cost.
        torch.cuda.synchronize(device)
        now = time.time()
        state["count"] += 1

        if state["count"] == 1:
            # torch.cuda.max_memory_allocated() is a cumulative
            # process-wide high-water mark, not "this batch's usage".
            # --batch -1 (AutoBatch) intentionally profiles past the
            # card's physical ceiling before real training starts (a
            # real, self-handled probe -- see the "AutoBatch: Using
            # batch-size N" line). Without resetting here, that
            # exploratory peak got misread as real training and tripped
            # a false VRAMBudgetExceeded on a healthy run. Reset once,
            # right after AutoBatch finishes, so every later peak
            # reflects only real training. Runs with an explicit
            # --batch are unaffected either way.
            torch.cuda.reset_peak_memory_stats(device)

        prev_t = state["last_t"]
        state["last_t"] = now

        if state["count"] <= warmup_batches:
            return  # let first-batch overhead (cudnn autotune etc.) settle
        if state["count"] < warmup_batches + probe_batches:
            return

        state["checked"] = True
        total = torch.cuda.get_device_properties(device).total_memory
        peak_allocated = torch.cuda.max_memory_allocated(device)
        peak_reserved = torch.cuda.max_memory_reserved(device)
        batch_seconds = (now - prev_t) if prev_t else 0.0

        allocated_bad = peak_allocated > total * safety_margin
        reserved_bad = peak_reserved > total * safety_margin
        mem_bad = allocated_bad or reserved_bad
        time_bad = batch_seconds > max_batch_seconds

        if mem_bad or time_bad:
            reasons = []
            if mem_bad:
                reasons.append(
                    f"peak allocated {peak_allocated / 1e9:.2f}GB / peak "
                    f"reserved {peak_reserved / 1e9:.2f}GB vs "
                    f"{total / 1e9:.2f}GB physical (WDDM shared-memory "
                    f"spillover signature)")
            if time_bad:
                reasons.append(
                    f"{batch_seconds:.1f}s/batch after warmup (expected "
                    f"well under {max_batch_seconds}s)")
            raise VRAMBudgetExceeded(
                f"[startup probe] " + " and ".join(reasons))

    return callback


def make_ongoing_watchdog_callback(device, max_epoch_minutes=None,
                                     safety_margin=0.90):
    """on_train_epoch_end callback: re-checks cumulative peak allocated
    AND peak reserved memory against the card's physical total after
    every epoch, for the whole run (see the [FIX] note in
    make_startup_probe_callback()'s docstring for why reserved was
    added -- same fix applied here). Also flags any epoch over
    --max-epoch-minutes. Reads the same process-wide counters the
    startup probe resets on its first firing (which always happens
    before this callback's first on_train_epoch_end), so no separate
    reset is needed here.

    [FIX -- review item] The exception raised here now carries
    reason="epoch_time" when only the time ceiling tripped (no memory
    signal alongside it) vs reason="memory" otherwise -- see
    VRAMBudgetExceeded's docstring for why train_with_model_fallback()
    needs that distinction to avoid downgrading model size for a
    problem a smaller model wouldn't actually fix."""
    state = {"last_epoch_start": None}

    def callback(trainer):
        import torch

        now = time.time()
        epoch_seconds = (now - state["last_epoch_start"]
                          if state["last_epoch_start"] else None)
        state["last_epoch_start"] = now

        if not torch.cuda.is_available():
            if (max_epoch_minutes is not None and epoch_seconds is not None
                    and epoch_seconds > max_epoch_minutes * 60):
                raise VRAMBudgetExceeded(
                    f"[ongoing watchdog, epoch "
                    f"{getattr(trainer, 'epoch', '?')}] epoch took "
                    f"{epoch_seconds / 60:.1f} min, over the "
                    f"{max_epoch_minutes} min budget",
                    reason="epoch_time")
            return

        total = torch.cuda.get_device_properties(device).total_memory
        peak_allocated = torch.cuda.max_memory_allocated(device)
        peak_reserved = torch.cuda.max_memory_reserved(device)
        allocated_bad = peak_allocated > total * safety_margin
        reserved_bad = peak_reserved > total * safety_margin
        mem_bad = allocated_bad or reserved_bad

        time_bad = (max_epoch_minutes is not None
                    and epoch_seconds is not None
                    and epoch_seconds > max_epoch_minutes * 60)

        if mem_bad or time_bad:
            reasons = []
            if mem_bad:
                reasons.append(
                    f"cumulative peak allocated {peak_allocated / 1e9:.2f}GB "
                    f"/ peak reserved {peak_reserved / 1e9:.2f}GB vs "
                    f"{total / 1e9:.2f}GB physical (WDDM shared-memory "
                    f"spillover signature)")
            if time_bad:
                reasons.append(
                    f"epoch {getattr(trainer, 'epoch', '?')} took "
                    f"{epoch_seconds / 60:.1f} min, over the "
                    f"{max_epoch_minutes} min budget")
            raise VRAMBudgetExceeded(
                f"[ongoing watchdog, epoch "
                f"{getattr(trainer, 'epoch', '?')}] " + " and ".join(reasons),
                reason="memory" if mem_bad else "epoch_time")

    return callback


# ---------------------------------------------------------------------
# Disk cache drive redirect -- see module docstring's "DISK CACHE DRIVE"
# paragraph for the full "why".
# ---------------------------------------------------------------------

def _windows_drive_roots():
    """Every currently-mounted drive letter as a Path root ('C:\\',
    'D:\\', ...). Uses os.listdrives() (Python 3.12+, Windows-only,
    confirmed available on this project's Python 3.14 interpreter) so a
    new/changed drive letter is picked up automatically; falls back to
    manually probing C:-H: on an older interpreter."""
    try:
        return [Path(d) for d in os.listdrives()]
    except AttributeError:
        return [Path(f"{letter}:\\") for letter in "CDEFGH"
                if Path(f"{letter}:\\").exists()]


def _pick_best_cache_drive(preferred_drive=None):
    """Returns (Path root, free_bytes) for the cache drive to use, or
    None if no drive's free space could be read.

    If preferred_drive is given (--cache-drive), that drive is used
    as long as it's actually mounted and its free space is readable --
    no "most free space" comparison overrides an explicit choice.
    Falls back to "most free space" (with a printed note) if the
    preferred drive isn't found.

    [note -- reviewed, not changed] "Most free space" is a capacity
    heuristic, not a speed one -- a spinning HDD with more free space
    would still be picked over a fuller SSD. --cache-drive exists so
    that can be overridden explicitly once it's known which mounted
    drive is actually fastest on this machine; auto-detecting drive
    speed reliably (SSD vs HDD, USB vs internal) isn't attempted here.

    Deliberately doesn't try to precompute how many bytes the disk
    cache will actually need -- that math (image count x resized size x
    safety margin) is exactly what Ultralytics' check_cache_disk()
    already does, and install_disk_cache_redirect() lets that check run
    for real against the chosen drive. Picking the single largest-free-
    space drive (or the explicitly preferred one) is sufficient either
    way: if that drive doesn't have enough room, no smaller one would
    either, so the existing "not caching images to disk" fallback still
    applies cleanly."""
    roots = _windows_drive_roots()

    if preferred_drive is not None:
        preferred_root = Path(preferred_drive)
        matched = next(
            (r for r in roots if r.resolve() == preferred_root.resolve()
             or str(r).rstrip("\\").lower()
             == str(preferred_root).rstrip("\\").lower()),
            None)
        if matched is not None:
            try:
                free = shutil.disk_usage(matched).free
                return (matched, free)
            except OSError:
                print(f"[disk cache redirect] --cache-drive {preferred_drive} "
                      f"is mounted but its free space couldn't be read -- "
                      f"falling back to auto-selection by free space.")
        else:
            print(f"[disk cache redirect] --cache-drive {preferred_drive} "
                  f"isn't among the currently mounted drives "
                  f"({[str(r) for r in roots]}) -- falling back to "
                  f"auto-selection by free space.")

    best = None
    for root in roots:
        try:
            free = shutil.disk_usage(root).free
        except OSError:
            continue
        if best is None or free > best[1]:
            best = (root, free)
    return best


def install_disk_cache_redirect(preferred_drive=None):
    """Redirects Ultralytics' --cache disk .npy files onto a mounted
    drive with room to spare, instead of Ultralytics' default of
    writing them alongside each source image -- i.e. onto whatever
    drive the dataset itself happens to live on. There's no built-in
    Ultralytics option for this (open feature request, unimplemented as
    of this project's installed version:
    https://github.com/ultralytics/ultralytics/issues/18285).

    Picks --cache-drive if given and mounted, otherwise whichever
    mounted drive currently has the most free space -- see
    _pick_best_cache_drive()'s docstring for why "most free space" is a
    capacity heuristic, not a speed one, and why --cache-drive exists
    to let that be overridden.

    MECHANISM: BaseDataset.__init__ sets self.npy_files (one .npy path
    per source image) before calling self.check_cache_disk() to decide
    whether disk caching is viable at all.

    [FIX -- review item] The first version of this function patched
    check_cache_disk() to rewrite self.npy_files onto the chosen drive
    and then DELEGATE to the original check_cache_disk() for the actual
    pass/fail call. That looked right but wasn't: Ultralytics' real
    check_cache_disk() hardcodes its free-space probe against
    shutil.disk_usage(Path(self.im_files[0]).parent) -- the SOURCE
    image directory -- not self.npy_files at all (confirmed by reading
    the installed ultralytics package's ultralytics/data/base.py
    directly). So the delegated call kept measuring free space on the
    dataset's own drive regardless of where npy_files pointed, kept
    failing there, and the redirect was a silent no-op end to end
    (confirmed: a run with 173GB free on the redirected drive still
    printed the same "only 121GB free" warning, measured against the
    original 474GB drive). check_cache_disk() is therefore fully
    REPLACED below (same 30-image sample-and-extrapolate math and
    safety-margin formula as Ultralytics' version, so behavior still
    tracks a future safety_margin/dataset-size change identically) with
    its free-space probe pointed at cache_dir instead.

    Cache filenames are flat and content-hashed (md5 of the original
    image path) under a single '<drive>\\RAD_disk_cache\\' folder,
    rather than mirroring each source image's full directory structure
    -- avoids both Windows MAX_PATH issues from deeply nested mirrored
    paths and any filename collisions between sources that happen to
    share a base filename (VisDrone/UAVDT/SARD all use different naming
    conventions, but nothing enforces that going forward). MD5 here is
    purely a deterministic filename generator, not a security boundary
    -- collision resistance at this scale is not a concern worth
    upgrading to sha256 for.

    Windows-only (this project's only target platform) -- a no-op with
    a printed note on any other OS or if no drive's free space can be
    read. If the chosen drive still doesn't have enough room,
    Ultralytics' own check disables caching exactly as it did before,
    just having checked the best available drive instead of a fixed
    one.
    """
    if platform.system() != "Windows":
        print("[disk cache redirect] Not on Windows -- skipping, disk "
              "cache (if used) stays at Ultralytics' default location.")
        return

    picked = _pick_best_cache_drive(preferred_drive)
    if picked is None:
        print("[disk cache redirect] Could not read free space on any "
              "mounted drive -- skipping, disk cache (if used) stays at "
              "Ultralytics' default location.")
        return

    cache_root, free_bytes = picked
    cache_dir = cache_root / "RAD_disk_cache"
    try:
        cache_dir.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        print(f"[disk cache redirect] {cache_root} has the most free "
              f"space ({free_bytes / 1e9:.1f}GB) but its cache folder "
              f"couldn't be created ({e}) -- skipping, disk cache (if "
              f"used) stays at Ultralytics' default location.")
        return

    print(f"[disk cache redirect] {cache_root} "
          f"({free_bytes / 1e9:.1f}GB free) -- disk cache .npy files (if "
          f"--cache disk fits there) will be written to {cache_dir} "
          f"instead of alongside the source images.")

    import ultralytics.data.base as base_mod
    orig_check_cache_disk = base_mod.BaseDataset.check_cache_disk

    def patched_check_cache_disk(self, safety_margin: float = 0.5) -> bool:
        """Full replacement for BaseDataset.check_cache_disk() -- see
        install_disk_cache_redirect()'s [FIX] docstring note for why a
        redirect-then-delegate wrapper didn't work. Mirrors Ultralytics'
        own sample-30-images-and-extrapolate math exactly, but checks
        free space at cache_dir (the redirected drive) instead of
        Path(self.im_files[0]).parent, and points self.npy_files at
        cache_dir so cache_images_to_disk() (called right after this,
        unmodified) writes there for real."""
        if not hasattr(self, "im_files") or not hasattr(self, "ni"):
            # Unexpected BaseDataset shape (future Ultralytics version?)
            # -- fall back to stock behavior rather than guessing.
            return orig_check_cache_disk(self, safety_margin)

        self.npy_files = [
            cache_dir / (hashlib.md5(str(f).encode()).hexdigest() + ".npy")
            for f in self.im_files
        ]

        if not os.access(cache_dir, os.W_OK):
            self.cache = None
            print(f"[disk cache redirect] {cache_dir} is not writable -- "
                  f"not caching images to disk.")
            return False

        import random
        b, gb = 0, 1 << 30
        n = min(self.ni, 30)
        for _ in range(n):
            im = base_mod.imread(random.choice(self.im_files))
            if im is None:
                continue
            b += im.nbytes
        if b == 0:
            # Couldn't successfully sample a single image -- something
            # else is wrong; defer to the stock check rather than
            # dividing by a meaningless zero below.
            return orig_check_cache_disk(self, safety_margin)

        disk_required = b * self.ni / n * (1 + safety_margin)
        total, _used, free = shutil.disk_usage(cache_dir)
        prefix = getattr(self, "prefix", "")
        if disk_required > free:
            self.cache = None
            print(f"{prefix}{disk_required / gb:.1f}GB disk space "
                  f"required, with {int(safety_margin * 100)}% safety "
                  f"margin but only {free / gb:.1f}/{total / gb:.1f}GB "
                  f"free on {cache_dir} (redirected drive) -- not "
                  f"caching images to disk.")
            return False

        print(f"{prefix}{disk_required / gb:.1f}GB disk space required "
              f"-- {free / gb:.1f}/{total / gb:.1f}GB free on "
              f"{cache_dir}, proceeding with disk cache there.")
        return True

    base_mod.BaseDataset.check_cache_disk = patched_check_cache_disk
    print("[disk cache redirect] Installed -- the disk-space check now "
          "runs against the redirected drive for real.")


# ---------------------------------------------------------------------
# Degradation augmentation -- see module docstring for the full "why".
# ---------------------------------------------------------------------

def _interlace_combing(image, **kwargs):
    """Albumentations Lambda: mimics NTSC/PAL interlace 'combing' (a 1px
    row shift). Kept minimal -- this project's objects of interest can
    be only a handful of pixels wide, so anything beyond ~1px risks
    displacing an already-tight label box."""
    import numpy as np
    img = image.copy()
    img[1::2] = np.roll(img[1::2], 1, axis=1)
    return img


def _try_build_transform(name, *builders):
    """Tries each candidate builder in order (newest API shape first),
    treating ANY warning as a build failure (not just a hard exception)
    -- some albumentations parameter mismatches are silently accepted
    and ignored with only a UserWarning, not an error, which previously
    let a transform silently run at the library's default strength
    instead of this project's deliberately small one. Returns None (with
    a printed message) if every builder fails.

    [note -- reviewed, not changed] this does mean an UNRELATED
    UserWarning raised during a builder's construction (not just a
    "parameter ignored" one) would also count as a rejection for that
    candidate. In practice each builder is a single constructor call
    with no other work happening in that scope, so this is a low-risk
    trade-off -- but if a future albumentations version starts emitting
    an unrelated warning here, the printed "Skipping '...'" message will
    say so and is the thing to check first."""
    import warnings
    for i, builder in enumerate(builders):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings("error", category=UserWarning)
                result = builder()
            if i > 0:
                print(f"[degradation aug] '{name}': installed "
                      f"albumentations version needed fallback parameter "
                      f"set #{i + 1} of {len(builders)}.")
            return result
        except Exception:
            continue
    print(f"[degradation aug] Skipping '{name}' -- none of {len(builders)} "
          f"candidate parameter set(s) were accepted cleanly by the "
          f"installed albumentations version.")
    return None


def build_degradation_transform():
    """Builds an Albumentations pipeline calibrated against the E5-FPV
    camera's datasheet (1500TVL CVBS analog, NTSC/PAL, 0.0001 lux min,
    6mm/100deg FOV) rather than generic noise. Returns None (with a
    warning) if albumentations isn't installed or every transform fails
    to build -- non-fatal, training proceeds without degradation aug.

    Five transforms (Downscale, GaussNoise, CoarseDropout,
    ImageCompression, OpticalDistortion) have different parameter names
    between albumentations 1.x/2.x, and some 1.x names are silently
    accepted-and-ignored under 2.x rather than erroring -- worst case
    for CoarseDropout, whose 2.x default hole size is large enough to
    blank out a tiny labeled person. So version-correct kwargs are used
    directly per detected major version, with _try_build_transform's
    warning-as-error check as a second-line defense.

    NOTE on OpticalDistortion specifically: albumentations went through
    THREE distinct signatures for this transform, not two --
      pre-2.0:  distort_limit + shift_limit
      2.0.x:    distort_limit + mode (shift_limit removed, mode added)
      2.2+:     distort_range + mode (distort_limit renamed to
                distort_range in the "every range param ends in _range"
                cleanup)
    An installed 2.0.x/2.1.x version (like 2.0.8) matches NEITHER the
    pre-2.0 nor the 2.2+ shape, so all three candidates are tried below,
    newest-first.

    [note -- reviewed, not changed] CoarseDropout/OpticalDistortion/the
    interlace Lambda are image-only (no bbox_params -- see
    install_degradation_augment()'s docstring), so in principle they can
    displace a labeled object by a pixel or two without moving its box.
    This pipeline's parameters were deliberately kept weak specifically
    to bound that risk (CoarseDropout: <=3 holes of 4x10px each,
    p=0.08; OpticalDistortion: +/-0.03, roughly a third of
    albumentations' own 0.05 default, p=0.15) rather than removed
    outright -- a bbox-aware version would need segmentation-free box
    remapping logic this project doesn't have yet. Worth re-examining
    if per-class mAP for tiny/rare classes (report_per_class_map())
    looks worse with degradation aug on than off, but not changed here
    since that's an empirical question, not a code bug.
    """
    try:
        import albumentations as A
    except ImportError:
        print("[degradation aug] albumentations not installed -- skipping. "
              "Run: pip install albumentations   to enable it (recommended "
              "before the real run, not required).")
        return None

    try:
        alb_major = int(str(A.__version__).split(".")[0])
    except Exception:
        alb_major = 1
    print(f"[degradation aug] Detected albumentations {A.__version__} "
          f"(using {'2.x' if alb_major >= 2 else '1.x'}-style parameter "
          f"names).")

    pieces = []

    downscale = _try_build_transform(
        "Downscale (1500TVL bandwidth ceiling)",
        lambda: A.Downscale(scale_range=(0.35, 0.75), p=0.4),
        lambda: A.Downscale(scale_min=0.35, scale_max=0.75,
                             interpolation=1, p=0.4))
    if downscale is not None:
        pieces.append(downscale)

    blur = _try_build_transform(
        "motion/gaussian blur (low-light shutter + analog softening)",
        lambda: A.OneOf([
            A.MotionBlur(blur_limit=7, p=1.0),
            A.GaussianBlur(blur_limit=(3, 7), p=1.0),
        ], p=0.35))
    if blur is not None:
        pieces.append(blur)

    def build_gauss_noise():
        # var_limit (1.x) is raw pixel variance; std_range (2.x) is std
        # as a fraction of the 0-1 range. (10,60) var ~ (0.012,0.030)
        # std, widened slightly for a comparable visible effect.
        import warnings
        for candidate in (
            lambda: A.GaussNoise(std_range=(0.015, 0.05), p=1.0),
            lambda: A.GaussNoise(var_limit=(10.0, 60.0), p=1.0),
        ):
            try:
                with warnings.catch_warnings():
                    warnings.filterwarnings("error", category=UserWarning)
                    return candidate()
            except Exception:
                continue
        return None  # [FIX] see note below -- was `raise ValueError(...)`

    # [FIX -- robustness] build_gauss_noise() used to raise if neither of
    # its own candidates built cleanly, which propagated up through this
    # single lambda and made ISONoise's fate depend on GaussNoise's --
    # if GaussNoise ever stopped matching a future albumentations
    # version, the whole noise transform (including a perfectly good
    # ISONoise) would silently disappear too. ISONoise and GaussNoise
    # are now built independently and combined only if at least one
    # succeeds, the same "graceful partial degradation" the rest of
    # this file already uses.
    iso_noise = _try_build_transform(
        "low-light sensor noise: ISONoise (0.0001 lux gain grain)",
        lambda: A.ISONoise(color_shift=(0.01, 0.06), intensity=(0.15, 0.5),
                            p=1.0))
    gauss_noise = build_gauss_noise()
    if gauss_noise is None:
        print("[degradation aug] Skipping GaussNoise fallback within the "
              "low-light noise transform -- none of its candidate "
              "parameter sets were accepted cleanly by the installed "
              "albumentations version.")
    noise_options = [t for t in (iso_noise, gauss_noise) if t is not None]
    if noise_options:
        pieces.append(A.OneOf(noise_options, p=0.35))

    gamma = _try_build_transform(
        "exposure/gamma variation (dusk-to-daylight SAR range)",
        lambda: A.RandomGamma(gamma_limit=(60, 140), p=0.3))
    if gamma is not None:
        pieces.append(gamma)

    brightness = _try_build_transform(
        "brightness/contrast variation",
        lambda: A.RandomBrightnessContrast(
            brightness_limit=0.3, contrast_limit=0.3, p=0.25))
    if brightness is not None:
        pieces.append(brightness)

    # Deliberately weak -- image-only pipeline (no bbox_params), so a
    # large hole/distortion would corrupt a label rather than just add
    # noise. Confirmed under 2.x with num_holes_range=(1, 3): worst case
    # 3 holes of (4,4)/(10,10) => at most 120 zeroed pixels, i.e.
    # literal pixels, not a fraction of image size.
    # [FIX -- doc/code mismatch] this comment previously said "5 holes
    # ... 200 zeroed pixels", which doesn't match num_holes_range=(1, 3)
    # below (max 3 holes, max 120px). The code was already correct and
    # intentionally weak; only the stale comment has been corrected here
    # to describe what num_holes_range=(1, 3) actually produces.
    dropout = _try_build_transform(
        "RF dropout burst (deliberately small/rare)",
        lambda: A.CoarseDropout(num_holes_range=(1, 3),
                                  hole_height_range=(4, 4),
                                  hole_width_range=(10, 10),
                                  fill=0, p=0.08),
        lambda: A.CoarseDropout(max_holes=3, max_height=4, max_width=10,
                                  fill_value=0, p=0.08))
    if dropout is not None:
        pieces.append(dropout)

    # Barrel distortion for the E5-FPV's 6mm/100deg lens. +/-0.03 is
    # roughly a third of albumentations' own 0.05 default -- enough to
    # bow lines slightly, not enough to meaningfully displace a tight
    # box except at the extreme edge. Same image-only corruption risk
    # as CoarseDropout above -- kept weak on purpose.
    #
    # [FIX] Three candidates now, not two -- see the OpticalDistortion
    # note in this function's docstring. The installed 2.0.8 needs the
    # middle one (distort_limit + mode, no shift_limit); the previous
    # two candidates (2.2+'s distort_range+mode, and pre-2.0's
    # distort_limit+shift_limit) both failed to build on 2.0.8, which
    # is why this transform was being skipped every run.
    optical_distortion = _try_build_transform(
        "lens barrel distortion (6mm/100deg FOV, deliberately weak)",
        lambda: A.OpticalDistortion(distort_range=(-0.03, 0.03),
                                      mode="camera", p=0.15),
        lambda: A.OpticalDistortion(distort_limit=(-0.03, 0.03),
                                      mode="camera", p=0.15),
        lambda: A.OpticalDistortion(distort_limit=0.03, shift_limit=0.0,
                                      p=0.15))
    if optical_distortion is not None:
        pieces.append(optical_distortion)

    compression = _try_build_transform(
        "downstream compression artifacts (generic capture-chain proxy)",
        lambda: A.ImageCompression(quality_range=(30, 65), p=0.25),
        lambda: A.ImageCompression(quality_lower=30, quality_upper=65, p=0.25))
    if compression is not None:
        pieces.append(compression)

    interlace = _try_build_transform(
        "NTSC/PAL interlace combing (1px, low-probability by design)",
        lambda: A.Lambda(image=_interlace_combing, p=0.12))
    if interlace is not None:
        pieces.append(interlace)

    if not pieces:
        print("[degradation aug] No transforms could be built against the "
              "installed albumentations version -- degradation "
              "augmentation is disabled for this run.")
        return None

    print(f"[degradation aug] Built {len(pieces)}/9 planned transforms "
          f"with version-correct parameters.")
    return A.Compose(pieces)


def install_degradation_augment():
    """Monkeypatches ultralytics.data.augment.Albumentations.__init__ so
    the transform it builds is replaced with build_degradation_transform()'s
    analog-feed-biased pipeline instead of Ultralytics' generic defaults.

    self.contains_spatial is forced False -- this pipeline's A.Compose
    has no bbox_params, so it must be called image-only, even though
    Ultralytics would otherwise classify CoarseDropout/OpticalDistortion/
    Lambda as spatial transforms. Those three run here without bbox
    awareness, relying on small hole size / small distortion magnitude
    instead of Albumentations' own box remapping to avoid corrupting
    labels.

    Patches an internal (non-public) class -- if a future Ultralytics
    upgrade changes Albumentations' __init__ signature or call site,
    this may silently stop applying; sanity-check with a few dumped
    augmented images after any Ultralytics version bump. Safe with
    --workers 0 (this project's default) since it's applied once in the
    main process before any dataloader worker spins up.

    patched_init() forwards *args/**kwargs rather than a hardcoded
    signature, since an Ultralytics update once added a `transforms=`
    kwarg a hardcoded `def patched_init(self, p=1.0)` couldn't accept.
    """
    transform = build_degradation_transform()
    if transform is None:
        return

    import ultralytics.data.augment as aug_mod

    orig_init = aug_mod.Albumentations.__init__

    def patched_init(self, *args, **kwargs):
        orig_init(self, *args, **kwargs)
        self.transform = transform
        self.contains_spatial = False

    aug_mod.Albumentations.__init__ = patched_init
    print("[degradation aug] Installed analog-feed degradation transform "
          "(motion/gaussian blur, compression artifacts, ISO noise, gamma "
          "shift, lens barrel distortion).")


def _contains_albumentations(transform_obj, aug_mod, _depth=0, _max_depth=6) -> bool:
    """Best-effort recursive search through Ultralytics' Compose tree
    for an Albumentations instance, via .transforms / .pre_transform
    (non-public internal structure) -- degrades to False past
    _max_depth or on an unexpected shape rather than raising."""
    if _depth > _max_depth or transform_obj is None:
        return False
    if isinstance(transform_obj, aug_mod.Albumentations):
        return True
    for attr in ("transforms", "pre_transform"):
        child = getattr(transform_obj, attr, None)
        if child is None:
            continue
        children = child if isinstance(child, (list, tuple)) else [child]
        for c in children:
            if _contains_albumentations(c, aug_mod, _depth + 1, _max_depth):
                return True
    return False


def make_augmentation_scope_check_callback():
    """on_train_start callback: empirically confirms the degradation
    patch reaches the TRAIN loader and NOT the val loader, by walking
    each loader's actual transform tree (not just documenting the
    assumption). Prints 'unknown' rather than a false pass if a tree
    can't be walked, so a broken check never hides a real problem."""
    def callback(trainer):
        import ultralytics.data.augment as aug_mod

        def check_loader(loader, label):
            if loader is None or not hasattr(loader, "dataset"):
                print(f"[augmentation scope check] {label}: loader not "
                      f"found -- skipping (not fatal).")
                return None
            transforms = getattr(loader.dataset, "transforms", None)
            if transforms is None:
                print(f"[augmentation scope check] {label}: dataset has no "
                      f".transforms attribute -- unknown.")
                return None
            found = _contains_albumentations(transforms, aug_mod)
            print(f"[augmentation scope check] {label}: degradation "
                  f"augmentation {'PRESENT' if found else 'absent'}")
            return found

        print(f"\n{'=' * 70}\nAugmentation scope check (confirms "
              f"degradation aug is train-only)\n{'=' * 70}")

        train_found = check_loader(getattr(trainer, "train_loader", None),
                                    "train loader")

        val_loader = getattr(trainer, "test_loader", None)
        if val_loader is None and getattr(trainer, "validator", None) is not None:
            val_loader = getattr(trainer.validator, "dataloader", None)
        val_found = check_loader(val_loader, "val loader")

        if val_found is True:
            print("\n[WARNING] Degradation augmentation appears to be "
                  "reaching the VALIDATION set. Val metrics from this run "
                  "would no longer be clean -- investigate before trusting them.")
        elif train_found is False:
            print("\n[WARNING] Degradation augmentation does NOT appear to "
                  "reach the TRAIN set either -- the monkeypatch may not be "
                  "taking effect. Confirm install_degradation_augment() ran "
                  "before model.train() and albumentations imported OK.")
        elif train_found is None or val_found is None:
            print("\n[note] Scope check was inconclusive for at least one "
                  "loader -- Ultralytics internals may not match what this "
                  "check expects. Training proceeds normally either way.")
        else:
            print("\n[OK] Degradation augmentation confirmed train-only "
                  "for this run.")
        print(f"{'=' * 70}")

    return callback


def _device_arg_type(value: str):
    """--device's default is the int 0, but argparse hands back a bare
    STRING when the flag is typed on the CLI. torch.device() accepts an
    int, 'cpu', or 'cuda:0'-style strings, but NOT a bare numeric string
    like '0' -- so an explicit --device 0 crashed with an unrelated-
    looking torch device-parsing error until this converts it. 'cpu' and
    already-qualified strings pass through unchanged."""
    if value.lower() == "cpu":
        return value
    try:
        return int(value)
    except ValueError:
        return value  # e.g. "cuda:0", "0,1" -- already unambiguous to torch


def _cache_arg_type(value):
    """[FIX -- review item #4] --cache had no type=, so `--cache False`
    on the CLI arrived as the literal string "False", which is truthy
    -- Ultralytics would still cache to disk despite the docstring's
    "set to False if disk space is tight" instructions. Same class of
    bug _device_arg_type() already fixes for --device. Parses
    true/false case-insensitively to real bools; passes 'ram'/'disk'
    (or anything else) through unchanged."""
    if isinstance(value, bool):
        return value
    lowered = str(value).strip().lower()
    if lowered == "false":
        return False
    if lowered == "true":
        return True
    return value  # "ram" / "disk" -- passed through as-is


def _explicitly_provided_resume_overrides(argv=None):
    """[FIX -- review item] Returns ONLY the resume-overridable flags
    (imgsz/batch/device/close_mosaic/save_period/workers/cache/patience
    -- confirmed via the installed ultralytics package's
    BaseTrainer.check_resume() allowlist) that the user actually TYPED
    on the CLI this invocation, as a separate throwaway parse with
    every default set to argparse.SUPPRESS.

    The bug this fixes: parse_args()'s real defaults (imgsz=960,
    batch=1, device=0, workers=0, cache='disk', save_period=5,
    patience=30, close_mosaic=10) exist on `args` whether or not the
    user typed the flag. check_resume()'s override loop is
    `if k in overrides: setattr(...)` -- it has no way to tell "user
    typed --batch 1" from "the field defaults to 1" once both look
    identical in the dict handed to model.train(). Building
    resume_overrides straight from `args` (the old approach) therefore
    passed all 8 keys on EVERY --resume invocation, silently
    overwriting the checkpoint's own saved imgsz/batch/device/.../
    patience with this script's normal-run defaults even when the user
    typed bare `--resume` -- e.g. a checkpoint saved at batch=2 would
    silently resume at batch=1 instead of continuing at 2. Confirmed by
    reading ultralytics/engine/trainer.py's check_resume(): `overrides`
    there is exactly the kwargs dict this script hands to
    model.train(), so "key present" and "user typed it" were being
    treated as the same thing when they aren't.

    A second SUPPRESS-default parse of the same 8 flags is the fix --
    only a flag actually present in sys.argv ends up in the returned
    dict, so `python train.py --resume` (no other flags) now yields an
    EMPTY resume_overrides, and every one of these 8 settings genuinely
    resumes from the checkpoint's own args.yaml, exactly as the
    --resume help text has always claimed. `--resume --batch 2` still
    overrides just batch, as before. Kept as a separate parser (rather
    than changing parse_args()'s own defaults to SUPPRESS) so nothing
    else in this script -- the non-resume train path, the disk-cache-
    install check, etc., all of which need a real usable default
    whether or not the user typed the flag -- has to change."""
    sentinel = argparse.ArgumentParser(add_help=False)
    sentinel.add_argument("--imgsz", type=int, default=argparse.SUPPRESS)
    sentinel.add_argument("--batch", type=int, default=argparse.SUPPRESS)
    sentinel.add_argument("--device", type=_device_arg_type,
                            default=argparse.SUPPRESS)
    sentinel.add_argument("--close-mosaic", type=int,
                            default=argparse.SUPPRESS)
    sentinel.add_argument("--save-period", type=int,
                            default=argparse.SUPPRESS)
    sentinel.add_argument("--workers", type=int, default=argparse.SUPPRESS)
    sentinel.add_argument("--cache", type=_cache_arg_type,
                            default=argparse.SUPPRESS)
    sentinel.add_argument("--patience", type=int, default=argparse.SUPPRESS)
    known, _unused = sentinel.parse_known_args(argv)
    return vars(known)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="yolo26s.pt",
                    help="Base pretrained weights, or 'auto' to try "
                         "yolo26m first and fall back to yolo26s then "
                         "yolo26n if a size doesn't fit.")
    p.add_argument("--data", default=None,
                    help="Dataset yaml. Defaults to the auto-generated "
                         "datasets/unified.yaml -- unified taxonomy "
                         "spanning VisDrone+UAVDT+SARD plus datasets/"
                         "external/. Pass 'VisDrone.yaml' to reproduce "
                         "the original 10-class baseline instead.")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=960,
                    help="960 is RECOMMENDED -- 640px plateaus on small/"
                         "rare classes, 960px fixed that (see BASELINE "
                         "HISTORY). Doesn't need to match any source's "
                         "native resolution -- Ultralytics letterboxes "
                         "everything to one square size. mosaic_guard.py's "
                         "instance cap confirms real headroom (~2GB/6GB) "
                         "at 960px; fall back to 640 only if the startup "
                         "probe/watchdog trip.")
    p.add_argument("--batch", type=int, default=1,
                    help="Explicit batch size. batch=1 is a major speed "
                         "tax; with mosaic_guard.py installed, real runs "
                         "stay ~2GB/6GB VRAM, so there's likely headroom "
                         "to raise this -- test with a short --epochs 3-5 "
                         "run and watch nvidia-smi. Pass -1 for AutoBatch "
                         "(unreliable at 960px pre-mosaic_guard, may be "
                         "worth retrying now). AutoBatch's own profiling "
                         "pass deliberately probes past the physical "
                         "ceiling to find it -- a printed 'AutoBatch: "
                         "Using batch-size N' line means that already "
                         "resolved cleanly.")
    p.add_argument("--device", default=0, type=_device_arg_type,
                    help="GPU index. 0 = first (only, on a laptop) GPU. "
                         "'cpu' only for a sanity check, never real training.")
    p.add_argument("--workers", type=int, default=0,
                    help="Dataloader worker processes. Default 0 -- "
                         "spawning workers on Windows reloads the full "
                         "CUDA DLL stack per process, which has caused "
                         "paging-file exhaustion here. Raise to 2-4 only "
                         "after resizing Windows' paging file.")
    p.add_argument("--cache", default="disk", type=_cache_arg_type,
                    help="'disk' caches decoded images between epochs. "
                         "The .npy cache files are automatically "
                         "redirected onto a mounted drive with room to "
                         "spare (see install_disk_cache_redirect() / "
                         "--cache-drive) rather than always landing on "
                         "the dataset's own drive. Pass 'False' to "
                         "disable if disk space is tight everywhere.")
    p.add_argument("--cache-drive", default=None,
                    help="Explicit drive letter/root (e.g. 'D:\\') for "
                         "the --cache disk redirect, instead of "
                         "auto-picking whichever mounted drive has the "
                         "most free space. Auto-picking is a capacity "
                         "heuristic, not a speed one -- use this once "
                         "you know which mounted drive is actually "
                         "fastest (e.g. keep it off the drive that also "
                         "holds the dataset, or prefer an SSD over a "
                         "roomier HDD). Falls back to auto-selection "
                         "with a printed note if this drive isn't "
                         "mounted.")
    p.add_argument("--resume", action="store_true",
                    help="Continue from the most recently modified "
                         "runs/detect/*/weights/last.pt -- full optimizer "
                         "state preserved. --data/--name/--epochs and "
                         "every augmentation/oversampling/LR setting are "
                         "restored from that run's own saved args.yaml "
                         "and silently ignore whatever's passed alongside "
                         "--resume on the CLI. --imgsz/--batch/--device/"
                         "--workers/--cache/--save-period/--patience/"
                         "--close-mosaic ARE the exception -- Ultralytics "
                         "does honor those specific ones on resume (see "
                         "_explicitly_provided_resume_overrides() in "
                         "main()), so e.g. `--resume --batch 1` genuinely "
                         "lowers the batch size for the rest of that run, "
                         "and bare `--resume` with none of them typed "
                         "genuinely leaves all 8 at the checkpoint's own "
                         "saved values. Fails loudly if the taxonomy "
                         "(nc/class order) doesn't match the checkpoint -- "
                         "start fresh instead.")
    p.add_argument("--save-period", type=int, default=5,
                    help="Also write numbered checkpoints (epoch5.pt, "
                         "...) every N epochs. -1 to disable.")
    p.add_argument("--strict-resume-taxonomy", action="store_true",
                    help="Make --resume's taxonomy check "
                         "(_verify_resume_taxonomy()) fatal if it can't "
                         "be verified (missing args.yaml, missing data "
                         "yaml, unreadable names list) instead of "
                         "printing a warning and proceeding. Off by "
                         "default for backward compatibility with older "
                         "runs; recommended for the final/research run, "
                         "since an unverifiable check defeats the point "
                         "of guarding against a silent class reorder.")
    p.add_argument("--max-epoch-minutes", type=float, default=None,
                    help="Failsafe: abort this model size (ongoing "
                         "watchdog) if any epoch exceeds this many "
                         "minutes. On its own (no VRAM signal alongside "
                         "it) this does NOT trigger --model auto's "
                         "fallback to a smaller size -- see "
                         "VRAMBudgetExceeded's docstring for why.")
    p.add_argument("--vram-safety-margin", type=float, default=0.90,
                    help="Fraction of physical VRAM (peak allocated OR "
                         "peak reserved) the startup probe/ongoing "
                         "watchdog treat as the ceiling before raising "
                         "VRAMBudgetExceeded. 0.90 leaves real headroom "
                         "for WDDM safety on this 6GB/Windows setup; "
                         "raise toward 0.93-0.95 to be more permissive "
                         "once a model size's real margin is known, or "
                         "lower toward 0.85 to be more conservative, "
                         "without editing source.")
    p.add_argument("--copy-paste", type=float, default=0.3,
                    help="Copy-paste augmentation probability. Currently "
                         "a NO-OP on this pipeline -- CopyPaste needs "
                         "segmentation polygons this bbox-only data "
                         "doesn't have. Left on because it's harmless; "
                         "use mixup/oversampling for sparse-class "
                         "protection instead.")
    p.add_argument("--mixup", type=float, default=0.1,
                    help="Mixup augmentation probability. 0.0 to disable.")
    p.add_argument("--max-mosaic-instances", type=int, default=800,
                    help="Caps the instance count Mosaic partner "
                         "selection aims to stay under -- see "
                         "mosaic_guard.py. Set very large (e.g. 999999) "
                         "to effectively disable.")
    p.add_argument("--mosaic-max-tries", type=int, default=40,
                    help="How many candidate partners the instance cap "
                         "samples/rejects before falling back to the "
                         "least-dense candidates.")
    p.add_argument("--mosaic", type=float, default=1.0,
                    help="Mosaic augmentation probability. Lower this "
                         "(0.5-0.7) only if the VRAM probes keep tripping.")
    p.add_argument("--close-mosaic", type=int, default=10,
                    help="Disable mosaic for the last N epochs so the "
                         "model converges on clean images -- matters for "
                         "small-object recall. Raise if small objects "
                         "still look weak in final-epoch samples.")
    p.add_argument("--multi-scale", action="store_true",
                    help="Vary input size +/-50%% per batch. Costs more "
                         "VRAM headroom (prefer --imgsz 640 if enabling "
                         "this) but improves robustness to altitude-scale "
                         "variation.")
    p.add_argument("--no-degradation-aug", dest="degradation_aug",
                    action="store_false", default=True,
                    help="Disable the analog-feed degradation "
                         "augmentation pipeline. On by default -- "
                         "requires albumentations, auto-skips with a "
                         "warning if missing.")
    p.add_argument("--oversample-classes", default="motorcycle,other_vehicle",
                    help="Comma-separated UNIFIED_CLASSES names to "
                         "oversample in train. '' to disable. Check "
                         "prepare_datasets.py's 'Per-class instance "
                         "counts' block before trusting this default -- "
                         "'person' isn't in it.")
    p.add_argument("--oversample-multiplier", type=int, default=3,
                    help="How many times each oversampled-class image "
                         "path is duplicated in the train list per epoch.")
    p.add_argument("--patience", type=int, default=30,
                    help="Early stopping: stop if val mAP hasn't improved "
                         "in this many epochs. best.pt always comes from "
                         "the best-val epoch regardless. Raised from "
                         "Ultralytics' ~15-epoch norm since the "
                         "degradation augmentation makes val loss noisier "
                         "epoch-to-epoch. Lower toward 15 for a quick "
                         "comparison run.")
    p.add_argument("--cos-lr", dest="cos_lr", action="store_true",
                    default=True,
                    help="Cosine LR decay instead of linear (default on).")
    p.add_argument("--no-cos-lr", dest="cos_lr", action="store_false",
                    help="Use Ultralytics' linear LR decay instead.")
    p.add_argument("--label-smoothing", type=float, default=0.0,
                    help="Label smoothing. Reported deprecated/no-op on "
                         "the installed Ultralytics version -- kept for "
                         "forward-compat only.")
    p.add_argument("--name", default=None,
                    help="Run subfolder name under runs/detect/. Defaults "
                         "to '<model>_<imgsz>'. With --resume, targets "
                         "that run's last.pt directly instead of guessing "
                         "by mtime -- recommended if more than one run "
                         "folder exists.")
    p.add_argument("--export-only", action="store_true",
                    help="Skip training, just export an existing "
                         "runs/detect/<name>/weights/best.pt.")
    p.add_argument("--weights", default=None,
                    help="Checkpoint to export when --export-only is set. "
                         "Defaults to the most recently modified "
                         "runs/detect/*/weights/best.pt found.")
    p.add_argument("--skip-per-source-eval", action="store_true",
                    help="Skip the post-training per-source validation "
                         "pass and the blended per-class mAP report.")
    p.add_argument("--strict-per-source-eval", action="store_true",
                    help="Make a per-source evaluation failure inside "
                         "evaluate_per_source() fatal (raises after all "
                         "sources have been attempted) instead of only "
                         "printing a prominent warning and letting "
                         "training/export continue. Off by default -- "
                         "same backward-compatible/opt-in-strictness "
                         "pattern as --strict-resume-taxonomy. Recommend "
                         "turning this on for the actual Master's/final "
                         "run, since a partial per-source report "
                         "(e.g. 2/3 sources) can otherwise slip through "
                         "unnoticed.")
    return p.parse_args()


def default_run_name(model: str, imgsz: int) -> str:
    stem = Path(model).stem  # "yolo26s.pt" -> "yolo26s"
    return f"{stem}_{imgsz}"


def _is_resumable_checkpoint(path: Path) -> bool:
    """Checks whether a .pt checkpoint actually has resumable state
    (epoch counter + optimizer state) before handing it to Ultralytics'
    model.train(resume=True).

    A checkpoint from a run that completed normally has its optimizer
    stripped, so it's no longer resumable regardless of --epochs in its
    saved args.yaml. Ultralytics itself detects this and falls back to
    "new training" using ITS OWN internal defaults (coco8.yaml, 80
    classes, batch=16, workers=8...), not this project's real setup --
    confirmed this silently trained several real epochs against the
    wrong 4-image toy dataset before anyone noticed. Checking
    resumability ourselves first, and refusing loudly with the correct
    next step, avoids that repeat."""
    import torch

    try:
        ckpt = torch.load(path, map_location="cpu", weights_only=False)
    except Exception as e:
        print(f"[resume check] Could not load {path} to inspect it "
              f"({e}) -- treating as non-resumable to be safe.")
        return False

    has_epoch = ckpt.get("epoch") is not None
    has_optimizer = ckpt.get("optimizer") is not None
    return has_epoch and has_optimizer


def _resolve_saved_data_path(data_path_str: str | None, run_dir: Path) -> Path | None:
    """[FIX -- review item] A saved args.yaml 'data' path is whatever
    string Ultralytics happened to receive on that run's original
    command line -- which is very often relative (e.g.
    `--data datasets/unified.yaml`), and relative to whatever working
    directory THAT invocation was launched from. Both
    _verify_resume_taxonomy() and _recover_resume_data_path() used to
    resolve it only against the current process's cwd
    (`Path(data_path).exists()`), which fails if this --resume/export
    invocation happens to run from a different directory even though
    the dataset yaml never moved -- exactly the "resume from a
    different directory" case both functions' docstrings already flag
    as a known gap.

    Tries, in order: the path as given (absolute, or relative to the
    CURRENT working directory -- unchanged from before, so nothing that
    worked before stops working), then relative to SCRIPT_DIR (where
    this script -- and therefore, in every normal invocation of this
    project, the dataset yaml -- actually lives), then relative to
    run_dir (runs/detect/<name>/, in case a data yaml was ever placed
    alongside a specific run). Returns the first candidate that
    actually exists on disk, or None if none do -- callers keep their
    existing non-fatal "couldn't verify/recover" handling either way."""
    if not data_path_str:
        return None
    as_given = Path(data_path_str)
    candidates = [as_given]
    if not as_given.is_absolute():
        candidates.append(SCRIPT_DIR / as_given)
        candidates.append(run_dir / as_given)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _verify_resume_taxonomy(last_pt: Path, model: YOLO,
                              strict: bool = False) -> None:
    """[FIX -- review item] _is_resumable_checkpoint() only checks
    whether a checkpoint HAS resumable state, not whether the taxonomy
    it was trained against still matches what's currently on disk. The
    module docstring has long promised resume "fails loudly if the
    taxonomy (nc) changed", but the only thing that actually enforced
    that was an incidental PyTorch tensor-shape-mismatch crash on state
    dict load when nc itself changed -- a same-nc class REORDER (e.g.
    unified.yaml regenerated with 'motorcycle' and 'other_vehicle'
    swapped) would load without error and silently mean every
    prediction is mislabeled, since the detection head's tensor shape
    doesn't encode class order.

    Compares the checkpoint's own baked-in class list (model.names,
    read directly from the loaded .pt -- authoritative regardless of
    which data yaml was used originally) against whatever is CURRENTLY
    on disk at the data yaml path that same run's own args.yaml
    records it was trained against. This catches prepare_datasets.py
    having been re-run with a different taxonomy since this
    checkpoint's last epoch.

    Deliberately non-fatal by default on any read/parse problem
    (missing args.yaml on an older run, unexpected yaml shape, etc.) --
    prints a note and lets Ultralytics' own resume proceed rather than
    blocking on this check's own fragility. Only raises when a real,
    readable mismatch is found -- UNLESS strict=True.

    [FIX -- review item] strict (--strict-resume-taxonomy) turns every
    "couldn't verify" branch below into a hard SystemExit instead of a
    warning. Off by default (preserves the original permissive
    behavior for older/experimental runs where args.yaml or the data
    yaml may legitimately be gone), but worth turning on for a final
    run specifically because the failure mode this whole function
    guards against -- a same-nc class reorder silently mislabeling
    every prediction -- is exactly the kind of thing you don't want to
    be trusting an unverified assumption about.

    [FIX -- bug found during review] data_path is now initialized to
    None before the try: block. Previously it was only assigned inside
    the try (after the first `open(args_yaml)` succeeded), so an
    exception raised BEFORE that assignment -- e.g. args_yaml existing
    per .exists() but failing to open, or a YAML parse error on
    args_yaml itself -- hit the except block's own
    f"...{args_yaml} or {data_path}..." message and raised an unrelated
    NameError there instead of printing the intended non-fatal warning,
    crashing main() with a confusing traceback instead of the
    deliberately-graceful "skipping this check" path the docstring
    promises.

    [FIX -- review item] The saved 'data' path is now resolved via
    _resolve_saved_data_path() instead of a bare
    `Path(data_path).exists()` -- see that function's docstring for why
    a relative path saved from a different original working directory
    used to fail this check (and skip taxonomy verification entirely)
    even when the dataset yaml was never actually moved."""
    run_dir = last_pt.parent.parent
    args_yaml = run_dir / "args.yaml"
    data_path = None

    def _fail_or_warn(message: str) -> None:
        if strict:
            raise SystemExit(
                f"\n{message} --strict-resume-taxonomy is set -- "
                f"refusing to resume without a verified taxonomy match.")
        print(f"{message} Skipping taxonomy verification (pass "
              f"--strict-resume-taxonomy to make this fatal instead).")

    if not args_yaml.exists():
        _fail_or_warn(f"[resume] No {args_yaml} found.")
        return

    try:
        with open(args_yaml) as f:
            saved_args = yaml.safe_load(f) or {}
        raw_data_path = saved_args.get("data")
        resolved_data_path = _resolve_saved_data_path(raw_data_path, run_dir)
        if resolved_data_path is None:
            _fail_or_warn(
                f"[resume] This run's original data yaml ({raw_data_path}) "
                f"couldn't be found as given, under {SCRIPT_DIR}, or under "
                f"{run_dir}.")
            return
        data_path = resolved_data_path

        with open(data_path) as f:
            current_data = yaml.safe_load(f) or {}
        current_names = current_data.get("names")
        if isinstance(current_names, dict):
            current_names = [current_names[k] for k in sorted(current_names)]
        elif current_names is None:
            _fail_or_warn(f"[resume] {data_path} has no 'names' key.")
            return
    except Exception as e:
        _fail_or_warn(
            f"[resume] Couldn't read/parse {args_yaml} or {data_path} "
            f"to verify taxonomy ({e}).")
        return

    ckpt_names = [model.names[k] for k in sorted(model.names)]
    if current_names != ckpt_names:
        raise SystemExit(
            f"\n[resume] Taxonomy mismatch -- {last_pt} was trained "
            f"with classes {ckpt_names}, but {data_path} currently "
            f"defines {current_names}. This checkpoint's detection head "
            f"is baked in for the OLD class list; if nc is unchanged "
            f"but the order differs, resuming would NOT crash -- it "
            f"would silently mislabel every prediction instead. "
            f"Re-generate {data_path} back to the taxonomy this "
            f"checkpoint expects, or start a fresh (non-resume) run to "
            f"retrain against the current one.")

    print(f"[resume] Taxonomy verified -- {data_path} still matches "
          f"this checkpoint's {len(ckpt_names)} classes in the same "
          f"order.")


def _recover_resume_data_path(run_dir: Path) -> str | None:
    """[FIX -- review item] main()'s `data_path` starts as None and,
    before this fix, was only ever assigned in the fresh-run branch --
    a --resume run left it None straight through to the end-of-run
    report, silently SKIPPING report_per_class_map() (the per-class
    mAP breakdown) even though evaluate_per_source() still ran fine.
    So a resumed run's console output quietly had one fewer report
    than a fresh run's, with only a one-line note explaining why.

    Recovers the dataset yaml path from this run's own saved
    args.yaml -- the same file _verify_resume_taxonomy() already reads
    before training starts. 'data' isn't in check_resume()'s override
    allowlist (see _explicitly_provided_resume_overrides()'s
    docstring), so it's unaffected by anything resumed and safe to
    read fresh after training completes. Deliberately non-fatal, same
    as _verify_resume_taxonomy() -- a missing/unreadable args.yaml or
    data yaml just means the per-class report is skipped, same as
    before this fix, not that the run itself failed.

    [FIX -- review item] Now resolves the saved path via
    _resolve_saved_data_path() (same helper _verify_resume_taxonomy()
    uses) instead of a bare `Path(data_path).exists()` against the
    current working directory -- see that function's docstring for why
    a relative path saved from a different original working directory
    used to silently skip this report even when the dataset yaml was
    never actually moved."""
    args_yaml = run_dir / "args.yaml"
    if not args_yaml.exists():
        print(f"\n[resume] No {args_yaml} found -- can't recover the "
              f"dataset yaml for the post-training per-class report; "
              f"skipping that report.")
        return None
    try:
        with open(args_yaml) as f:
            saved_args = yaml.safe_load(f) or {}
        raw_data_path = saved_args.get("data")
    except Exception as e:
        print(f"\n[resume] Couldn't read {args_yaml} to recover the "
              f"dataset yaml ({e}) -- skipping the per-class report.")
        return None
    resolved = _resolve_saved_data_path(raw_data_path, run_dir)
    if resolved is None:
        print(f"\n[resume] {args_yaml}'s 'data' path ({raw_data_path}) "
              f"couldn't be found as given, under {SCRIPT_DIR}, or under "
              f"{run_dir} -- skipping the per-class report.")
        return None
    return str(resolved)


def find_latest_last_pt(name: str | None = None) -> Path:
    """Locates the last.pt to resume from. With `name`, resumes that
    specific run only (errors loudly if missing). Without it, picks the
    most-recently-modified last.pt across all runs/detect/*/, but
    prints every candidate first -- confirmed a stray unrelated run
    folder (e.g. a generic "train/" from an ad-hoc `yolo train`
    invocation) can otherwise silently win just by being newer. Pass
    --name explicitly to avoid relying on mtime at all."""
    if name:
        target = RUNS_PROJECT / name / "weights" / "last.pt"
        if not target.exists():
            raise FileNotFoundError(
                f"No last.pt found at {target} -- check --name matches "
                f"an existing run folder under {RUNS_PROJECT}.")
        return target

    candidates = sorted(RUNS_PROJECT.glob("*/weights/last.pt"),
                         key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(
            f"No {RUNS_PROJECT}/*/weights/last.pt found to resume from. "
            f"Run training without --resume first.")

    if len(candidates) > 1:
        print(f"\n[resume] Multiple runs found under {RUNS_PROJECT} -- "
              f"no --name given, picking by most recent mtime. If this "
              f"isn't the run you meant, rerun with --resume --name "
              f"<run_folder_name>:")
        for c in candidates:
            marker = " <- selected" if c == candidates[-1] else ""
            print(f"    {c}  (modified {c.stat().st_mtime}){marker}")

    return candidates[-1]


def find_latest_best_pt() -> Path:
    """[FIX -- review item] Used by --export-only when --weights isn't
    given. Previously picked the most-recently-modified best.pt with no
    visibility into what else was available -- a stray debug/test run
    folder finishing after the real one silently wins, exporting the
    wrong model for the C++ deployment pipeline with no crash and no
    warning. Now mirrors find_latest_last_pt()'s existing candidate
    list so a wrong pick is at least visible; --export-only has no
    --name equivalent to target a specific run the way --resume does,
    so pass --weights explicitly for production exports instead of
    relying on this."""
    candidates = sorted(RUNS_PROJECT.glob("*/weights/best.pt"),
                         key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(
            f"No {RUNS_PROJECT}/*/weights/best.pt found. Run training "
            f"first, or pass --weights explicitly.")

    if len(candidates) > 1:
        print(f"\n[export] Multiple runs found under {RUNS_PROJECT} -- "
              f"no --weights given, picking by most recent mtime. If "
              f"this isn't the run you meant, rerun with --export-only "
              f"--weights <path to that run's weights/best.pt>:")
        for c in candidates:
            marker = " <- selected" if c == candidates[-1] else ""
            print(f"    {c}  (modified {c.stat().st_mtime}){marker}")

    return candidates[-1]


def export_for_cpp(weights_path: Path):
    """Exports with end2end=False (one-to-many head) so ONNX output
    shape stays (1, nc+4, N) -- what DetectionLink::runInference()
    parses via manual class-argmax + cv::dnn::NMSBoxes."""
    print(f"\nExporting {weights_path} to ONNX (one-to-many head, NMS "
          f"still required on the C++ side)...")
    model = YOLO(str(weights_path))
    onnx_path = model.export(format="onnx", opset=17, simplify=True,
                              end2end=False, nms=False)
    onnx_path = Path(onnx_path)

    names_path = onnx_path.with_suffix(".names")
    class_names = model.names  # dict[int, str], index order matters
    with open(names_path, "w") as f:
        for idx in sorted(class_names.keys()):
            f.write(class_names[idx] + "\n")

    print(f"\nExported model:  {onnx_path}")
    print(f"Class names file: {names_path}")
    return onnx_path


def is_oom_error(exc: Exception) -> bool:
    return "out of memory" in str(exc).lower()


def check_overfitting(run_dir: Path, lookback: int = 10) -> None:
    """Reads results.csv and flags the textbook overfitting signature:
    val loss rising over the last `lookback` epochs while train loss
    keeps falling. Heuristic, aggregate-only -- see report_per_class_map()
    for the per-class check this can hide."""
    csv_path = run_dir / "results.csv"
    if not csv_path.exists():
        print(f"\n[overfitting check] No results.csv found at {csv_path}, "
              f"skipping.")
        return

    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < lookback + 1:
        print(f"\n[overfitting check] Only {len(rows)} epochs logged, too "
              f"few for a meaningful trend check (need > {lookback}).")
        return

    def col(name):
        for row in rows:
            for key in row:
                if key.strip() == name:
                    return [float(r[key]) for r in rows]
        return None

    train_box = col("train/box_loss")
    val_box = col("val/box_loss")
    if train_box is None or val_box is None:
        print(f"\n[overfitting check] Couldn't find train/box_loss and "
              f"val/box_loss columns in results.csv -- skipping.")
        return

    recent_train = train_box[-lookback:]
    recent_val = val_box[-lookback:]
    train_trend = recent_train[-1] - recent_train[0]
    val_trend = recent_val[-1] - recent_val[0]

    print(f"\n[overfitting check] Last {lookback} epochs: "
          f"train/box_loss {'fell' if train_trend < 0 else 'rose'} "
          f"{abs(train_trend):.4f}, val/box_loss "
          f"{'fell' if val_trend < 0 else 'rose'} {abs(val_trend):.4f}.")

    if train_trend < 0 and val_trend > 0:
        print("  WARNING: train loss still falling while val loss is "
              "rising over this window -- classic overfitting signature. "
              "best.pt was still saved from the best-val epoch, but "
              "consider more/varied training data, a lower --epochs "
              "ceiling, or stronger augmentation.")
    else:
        print("  No clear divergence in this window.")


def report_per_class_map(model: YOLO, data_path: str, batch: int,
                           workers: int = 0) -> None:
    """Runs model.val() on the full blended val set and prints mAP50-95
    PER CLASS -- the direct check for "one class detected well, another
    degraded" that a single averaged number hides (e.g. person, the SAR
    priority class, trailing vehicle classes). evaluate_per_source()
    breaks down by DATASET; this breaks down by CLASS on the combined
    val set.

    batch/workers are passed through explicitly (not left at
    Ultralytics' own larger defaults) and the CUDA cache is cleared
    first -- both fixes for real crashes hit on this 6GB card/Windows
    setup: an unset batch size OOM'd, and Windows spawning nonzero
    dataloader workers here (unlike every model.train() call, which
    always passes workers=0) reloaded the full CUDA DLL stack in a
    subprocess and exhausted the paging file."""
    import gc
    import torch
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    print(f"\n{'=' * 70}")
    print("Per-class mAP (blended val set, all sources combined)")
    print(f"{'=' * 70}")
    metrics = model.val(data=data_path, split="val", batch=batch,
                         workers=workers, verbose=False)
    per_class = metrics.box.maps  # array indexed by class, mAP50-95
    for idx, name in enumerate(UNIFIED_CLASSES):
        if idx < len(per_class):
            print(f"  {name:15s}  mAP50-95={per_class[idx]:.3f}")
    print(f"{'=' * 70}")
    print("If 'person' trails the vehicle classes here, that's your "
          "signal to increase person's share of training exposure "
          "(--oversample-classes) or add more SAR-relevant person data "
          "-- not a sign the pipeline is broken.")


def evaluate_per_source(model: YOLO, run_dir: Path, batch: int,
                          workers: int = 0, strict: bool = False) -> None:
    """Runs a separate model.val() against each source's own val set
    (VisDrone/UAVDT/SARD individually, from datasets/per_source_val.json)
    -- the only way to tell whether combining these datasets helped,
    since the blended val number can hide one source regressing while
    another improves. Skipped if the manifest doesn't exist.

    A 0.000 (or near-zero) per-class row is NOT proof that class is
    structurally absent from that source anymore: generate_pseudo_
    labels.py --apply cross-labels a source's originally-missing
    classes (UAVDT: person, SARD: vehicles) once an image clears
    review, so it may just mean little of that source has been merged
    for that class yet. Cross-check datasets/pending_review_images.json
    and pseudo_labels_summary.json before concluding otherwise -- and
    note that per-source mAP itself is measured on only the currently
    review-cleared portion of that source, which can be a small
    fraction of its total images; treat these numbers as "performance
    on the reviewed subset so far", not "performance on the full
    source dataset".

    batch/workers are passed through explicitly and the CUDA cache is
    cleared BEFORE each source's val() call -- same two Windows/6GB
    failure modes as report_per_class_map()'s docstring (leftover
    reserved memory from a prior val() call plus an unset workers=
    default spawning a paging-file-exhausting subprocess), confirmed
    from an actual crash on the UAVDT pass specifically.

    [FIX -- review item, the one remaining real bug from the last
    review] Previously ANY exception from a source's model.val() call
    was caught, printed as "[skipped: ...]", and training/export
    continued as if nothing had happened -- a genuine failure (CUDA
    OOM, a corrupted val image, a bad yaml) looked identical in the
    console output to an intentionally-empty/not-yet-reviewed source,
    and the final "Training complete." banner gave no indication the
    reported experiment might only cover e.g. 2/3 sources. Failures are
    now collected into failed_sources and, after every source has been
    attempted (so one bad source never prevents the others from being
    tried), the run either raises (strict=True, i.e.
    --strict-per-source-eval) or prints an impossible-to-miss warning
    block naming exactly which source(s) failed and why (default,
    preserves the old "don't block export" behavior but no longer
    silently)."""
    if not PER_SOURCE_MANIFEST_PATH.exists():
        print(f"\n[per-source eval] No {PER_SOURCE_MANIFEST_PATH} found "
              f"(prepare_datasets.py wasn't run this session?) -- skipping.")
        return

    with open(PER_SOURCE_MANIFEST_PATH) as f:
        per_source = json.load(f)
    if not per_source:
        print("\n[per-source eval] Manifest is empty -- skipping.")
        return

    print(f"\n{'=' * 70}")
    print("Per-source validation (isolating each dataset's contribution)")
    print(f"{'=' * 70}")

    import gc
    import torch

    tmp_dir = run_dir / "per_source_eval"
    tmp_dir.mkdir(exist_ok=True)

    failed_sources = []  # [(name, error_str), ...]

    for name, val_dirs in per_source.items():
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        tmp_yaml = tmp_dir / f"{name.lower()}.yaml"
        with open(tmp_yaml, "w") as f:
            yaml.safe_dump({
                "path": None,
                "train": val_dirs,  # unused by val(), key still required
                "val": val_dirs,
                "nc": len(UNIFIED_CLASSES),
                "names": UNIFIED_CLASSES,
            }, f, sort_keys=False)
        try:
            metrics = model.val(data=str(tmp_yaml), split="val",
                                 batch=batch, workers=workers, verbose=False)
            print(f"  {name:10s}  mAP50={metrics.box.map50:.3f}  "
                  f"mAP50-95={metrics.box.map:.3f}  "
                  f"precision={metrics.box.mp:.3f}  recall={metrics.box.mr:.3f}")
            per_class = metrics.box.maps
            for cls_idx, cls_name in enumerate(UNIFIED_CLASSES):
                if cls_idx < len(per_class):
                    print(f"      {cls_name:15s} mAP50-95={per_class[cls_idx]:.3f}")
        except Exception as e:
            print(f"  {name:10s}  [skipped: {e}]")
            failed_sources.append((name, str(e)))
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

    print(f"{'=' * 70}")
    print("Note: a 0.000 (or near-zero) row above may mean 'not enough of "
          "this source has been reviewed/merged for this class yet', not "
          "'this source can never have this class' -- check "
          "pending_review_images.json / pseudo_labels_summary.json first.")

    if failed_sources:
        names_and_errors = "; ".join(f"{n}: {e}" for n, e in failed_sources)
        if strict:
            raise RuntimeError(
                f"Per-source evaluation failed for: {names_and_errors}. "
                f"The reported experiment would otherwise be incomplete "
                f"(missing {len(failed_sources)}/{len(per_source)} "
                f"source(s)) -- refusing to continue with "
                f"--strict-per-source-eval set.")
        print(f"\n{'!' * 70}\n"
              f"WARNING: PER-SOURCE EVALUATION INCOMPLETE "
              f"({len(failed_sources)}/{len(per_source)} source(s) failed)\n"
              f"{'!' * 70}")
        for n, e in failed_sources:
            print(f"  {n} evaluation FAILED: {e}")
        print(f"{'!' * 70}\n"
              f"Training/export will still proceed, but any per-source "
              f"report from this run is INCOMPLETE -- do not treat a "
              f"missing source's numbers as '0' or 'not applicable'. "
              f"Pass --strict-per-source-eval to make this fatal instead.\n"
              f"{'!' * 70}")


def train_with_model_fallback(args, data_path: str):
    """Tries each candidate in MODEL_SIZE_CANDIDATES (or just args.model
    if not 'auto') until one survives both the startup probe and the
    ongoing watchdog without tripping VRAMBudgetExceeded or a real OOM.

    [FIX -- review item] A VRAMBudgetExceeded with reason="epoch_time"
    (the ongoing watchdog's --max-epoch-minutes ceiling tripping with
    no memory signal alongside it) is handled separately below and does
    NOT trigger a fallback to a smaller model -- see VRAMBudgetExceeded's
    docstring for why an epoch running long, on its own, isn't evidence
    a model doesn't fit.

    [FIX -- review item] When args.model == "auto" and the user also
    passed an explicit --name, every candidate used to be trained under
    THE SAME requested run name -- e.g. `--model auto --name
    final_run`. If yolo26m tripped the VRAM guard after Ultralytics had
    already created runs/detect/final_run/, the next candidate
    (yolo26s) got handed that identical name again, and Ultralytics
    itself silently disambiguates by appending a suffix
    (runs/detect/final_run2/), so the run that actually trained could
    end up living somewhere other than the requested/reported name --
    confusing for --resume/--export-only's own by-name lookups later.
    Each auto-search candidate's run directory now gets the model's own
    stem appended (e.g. final_run_yolo26m, final_run_yolo26s) so
    there's never a name collision between candidates in the first
    place. When --name isn't given, default_run_name() already bakes
    the stem in (e.g. yolo26m_960), so it was -- and remains --
    naturally unique per candidate without any change needed there."""
    import gc
    import torch

    candidates = MODEL_SIZE_CANDIDATES if args.model == "auto" else [args.model]

    train_kwargs = dict(
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        cache=args.cache,
        amp=True,
        patience=args.patience,
        cos_lr=args.cos_lr,
        copy_paste=args.copy_paste,
        mixup=args.mixup,
        mosaic=args.mosaic,
        close_mosaic=args.close_mosaic,
        multi_scale=args.multi_scale,
        save_period=args.save_period,
    )
    if args.label_smoothing:
        print("[note] --label-smoothing is set, but your installed "
              "Ultralytics reported this deprecated -- passing it "
              "through anyway.")
        train_kwargs["label_smoothing"] = args.label_smoothing

    for i, model_name in enumerate(candidates):
        if args.model == "auto" and args.name:
            # An explicit --name would otherwise collide across
            # candidates -- see this function's [FIX] docstring note.
            this_run_name = f"{args.name}_{Path(model_name).stem}"
        else:
            # No --name given: default_run_name() already includes the
            # model stem, so it's unique per candidate without change.
            this_run_name = args.name or default_run_name(model_name, args.imgsz)
        print(f"\n{'=' * 70}\nAttempting: {model_name} @ {args.imgsz}px, "
              f"batch={args.batch}\n{'=' * 70}")
        model = None
        try:
            model = YOLO(model_name)
            model.add_callback(
                "on_train_batch_end",
                make_startup_probe_callback(
                    args.device, safety_margin=args.vram_safety_margin))
            model.add_callback(
                "on_train_epoch_end",
                make_ongoing_watchdog_callback(
                    args.device, max_epoch_minutes=args.max_epoch_minutes,
                    safety_margin=args.vram_safety_margin))
            if args.degradation_aug:
                model.add_callback(
                    "on_train_start",
                    make_augmentation_scope_check_callback())
            model.train(
                data=data_path,
                project=str(RUNS_PROJECT),
                name=this_run_name,
                **train_kwargs,
            )
            return model, this_run_name, model_name
        except (RuntimeError, VRAMBudgetExceeded) as e:
            if isinstance(e, VRAMBudgetExceeded) and e.reason == "epoch_time":
                # Epoch-time budget alone isn't evidence this model
                # size doesn't fit -- could be disk I/O contention, a
                # competing process, a slow first epoch, etc. Don't
                # silently downgrade model size for this (that masks
                # the real cause behind a "fix" that likely wouldn't
                # help) -- fail loudly instead so it gets investigated.
                if model is not None:
                    del model
                gc.collect()
                torch.cuda.empty_cache()
                print(f"\n[epoch-time budget] {model_name} exceeded "
                      f"--max-epoch-minutes ({e}) with no VRAM signal "
                      f"alongside it -- NOT falling back to a smaller "
                      f"model, since a slow epoch alone doesn't mean "
                      f"{model_name} doesn't fit. Investigate the actual "
                      f"cause (disk I/O, background processes, --cache "
                      f"settings), or raise --max-epoch-minutes if this "
                      f"epoch was just legitimately slow.")
                raise
            is_vram_issue = isinstance(e, VRAMBudgetExceeded) or is_oom_error(e)
            if not is_vram_issue:
                raise
            # Drop the reference and gc.collect() BEFORE empty_cache() --
            # empty_cache() only releases blocks the allocator considers
            # idle, and this candidate's memory wasn't idle yet while
            # `model` still referenced it. Without this, the next
            # (smaller) candidate loaded while the failed one's memory
            # was still fully resident.
            if model is not None:
                del model
            gc.collect()
            torch.cuda.empty_cache()
            if i + 1 < len(candidates):
                print(f"\n[VRAM] {model_name} doesn't fit ({e}). Falling "
                      f"back to {candidates[i + 1]}...")
                continue
            print(f"\n[VRAM] {model_name} doesn't fit ({e}) and no smaller "
                  f"candidate is left. Try --imgsz 640 and/or check "
                  f"--batch is already at its floor (1).")
            raise

    raise RuntimeError("No model candidates configured.")  # unreachable


def main():
    args = parse_args()

    if args.export_only:
        # Runs before installing the training-only monkeypatches below --
        # export never touches Mosaic/Albumentations/disk-cache
        # placement, so there's nothing to gain from patching them here.
        weights = Path(args.weights) if args.weights else find_latest_best_pt()
        export_for_cpp(weights)
        return

    install_instance_cap(max_instances=args.max_mosaic_instances,
                          max_tries=args.mosaic_max_tries)

    if str(args.cache).lower() == "disk":
        install_disk_cache_redirect(preferred_drive=args.cache_drive)

    if args.degradation_aug:
        install_degradation_augment()

    data_path = None  # only set on a fresh (non-resume) run

    if args.resume:
        last_pt = find_latest_last_pt(args.name)

        if not _is_resumable_checkpoint(last_pt):
            raise SystemExit(
                f"\n[resume] {last_pt} has no epoch/optimizer state left "
                f"to resume -- that run already completed normally "
                f"(Ultralytics strips optimizer state from last.pt/best.pt "
                f"once a run finishes, regardless of --epochs in its "
                f"args.yaml). True resume only works for a run "
                f"INTERRUPTED before its target epoch count.\n\n"
                f"To train more epochs from these weights instead, run a "
                f"fresh (non-resume) pass using this checkpoint as the "
                f"starting model:\n\n"
                f"    python train.py --model \"{last_pt}\" --epochs 5\n\n"
                f"This is a warm start, not a true continuation -- LR "
                f"schedule/optimizer momentum restart from scratch, but "
                f"the learned weights carry over. Adjust --epochs/--imgsz/"
                f"--batch as you would for any normal run.")

        print(f"\nResuming training from {last_pt}...")
        model = YOLO(str(last_pt))

        # [FIX -- review item] Only checked resumability (epoch/optimizer
        # present) before, not whether the taxonomy this checkpoint was
        # trained against still matches what's currently on disk -- see
        # _verify_resume_taxonomy()'s docstring for the same-nc/reordered-
        # classes case that a raw PyTorch shape-mismatch crash wouldn't
        # have caught.
        _verify_resume_taxonomy(last_pt, model,
                                 strict=args.strict_resume_taxonomy)

        # train_with_model_fallback() (fresh-run path) adds the startup
        # probe, ongoing watchdog, and augmentation-scope check -- this
        # resume path needs the same per-Trainer callbacks explicitly,
        # since they don't carry over from the global monkeypatches alone.
        model.add_callback(
            "on_train_batch_end",
            make_startup_probe_callback(
                args.device, safety_margin=args.vram_safety_margin))
        model.add_callback(
            "on_train_epoch_end",
            make_ongoing_watchdog_callback(
                args.device, max_epoch_minutes=args.max_epoch_minutes,
                safety_margin=args.vram_safety_margin))
        if args.degradation_aug:
            model.add_callback(
                "on_train_start",
                make_augmentation_scope_check_callback())

        # [FIX -- review item] Verified against the installed
        # ultralytics package's BaseTrainer.check_resume(): it DOES
        # honor a specific allowlist of args re-passed to model.train()
        # alongside resume=True -- imgsz/batch/device/close_mosaic/
        # save_period/workers/cache/patience among them -- restoring
        # everything else (data/name/epochs/mosaic prob/mixup/cos_lr/
        # oversampling/...) from the checkpoint's own saved args.yaml
        # regardless of what's passed here.
        #
        # [FIX -- review item] Building this dict from `args` directly
        # (the previous version) put all 8 keys in unconditionally,
        # since argparse's own defaults for imgsz/batch/device/.../
        # patience are indistinguishable from a value the user actually
        # typed once they're sitting on `args`. check_resume() only
        # checks "is this key present in the dict I was handed", not
        # "did the user type this" -- so bare `--resume` was silently
        # overwriting the checkpoint's saved imgsz/batch/device/.../
        # patience with this script's normal-run defaults every time.
        # _explicitly_provided_resume_overrides() does a second,
        # SUPPRESS-defaulted parse of just these 8 flags so only ones
        # actually typed this invocation end up here -- see its
        # docstring for the full trace through ultralytics' source.
        resume_overrides = _explicitly_provided_resume_overrides()
        if resume_overrides:
            print(f"[resume] CLI overrides for this resume: "
                  f"{resume_overrides}")
        else:
            print("[resume] No --imgsz/--batch/--device/--close-mosaic/"
                  "--save-period/--workers/--cache/--patience typed this "
                  "invocation -- all 8 resume from this checkpoint's own "
                  "saved args.yaml unchanged.")

        try:
            model.train(resume=True, **resume_overrides)
        except VRAMBudgetExceeded as e:
            # Unlike a fresh run, there's no smaller MODEL to fall back
            # to here -- the checkpoint's architecture is fixed. But
            # --batch/--imgsz/--workers/--cache genuinely ARE honored on
            # resume (see resume_overrides above), so lowering those is
            # a real option here, not just "start over".
            raise SystemExit(
                f"\n[VRAM] Resumed run at {last_pt} hit the VRAM budget "
                f"check ({e}). Resuming can't fall back to a smaller "
                f"MODEL size, but --batch/--imgsz/--workers/--cache ARE "
                f"honored on resume -- try, e.g.:\n\n"
                f"    python train.py --resume --name "
                f"{last_pt.parent.parent.name} --batch 1 --imgsz 640\n\n"
                f"If that's already at the floor, free up VRAM, or start "
                f"a fresh (non-resume) run with a smaller --model, using "
                f"this checkpoint as a warm start (see the resumability "
                f"error message above).")

        run_name = last_pt.parent.parent.name
        used_model_name = None
        # [FIX -- review item] recovers the dataset yaml so the
        # end-of-run report block below (data_path is not None ->
        # report_per_class_map()) runs for a resumed run exactly like
        # it does for a fresh one, instead of always hitting the
        # "[per-class report] Skipped" branch.
        data_path = _recover_resume_data_path(last_pt.parent.parent)
    else:
        oversample_classes = [
            c.strip() for c in args.oversample_classes.split(",") if c.strip()
        ]
        data_path = args.data or str(prepare_datasets.main(
            oversample_classes=oversample_classes,
            oversample_multiplier=args.oversample_multiplier,
        ))
        model, run_name, used_model_name = train_with_model_fallback(args, data_path)

    best_pt = Path(model.trainer.best)
    run_dir = best_pt.parent.parent
    print(f"\nTraining complete. Run: {run_name}")
    print(f"Best checkpoint: {best_pt}")
    if used_model_name is not None and used_model_name != args.model:
        print(f"[note] Requested --model {args.model}, but training "
              f"actually completed on {used_model_name} -- see the "
              f"'[VRAM]' fallback messages above for why. If unintended, "
              f"check whether --imgsz 640 (more VRAM headroom) lets the "
              f"larger model fit instead.")

    # [FIX -- review item] evaluate_per_source()/report_per_class_map()
    # below used to be passed args.batch directly. That's fine for an
    # explicit --batch, but --batch -1 (AutoBatch) is only meaningful to
    # model.train() -- Ultralytics resolves it to a concrete int
    # (trainer.batch_size) during training, and passing the raw -1
    # through to model.val() here is undefined/unsupported. Captured
    # here (before `model` is freed below) so eval always uses the real
    # batch size training actually ran at, AutoBatch or not.
    eval_batch = getattr(model.trainer, "batch_size", None) or args.batch

    # `model` (trainer, optimizer state, EMA shadow, dataloader buffers)
    # stayed referenced through eval/export otherwise -- free it now so
    # eval_model below actually gets a clean VRAM footprint instead of
    # stacking on top of this.
    import gc
    import torch
    del model
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    check_overfitting(run_dir)

    if not args.skip_per_source_eval:
        eval_model = YOLO(str(best_pt))
        evaluate_per_source(eval_model, run_dir, eval_batch,
                             workers=args.workers,
                             strict=args.strict_per_source_eval)
        if data_path is not None:
            report_per_class_map(eval_model, data_path, eval_batch,
                                  workers=args.workers)
        else:
            print("\n[per-class report] Skipped -- no data_path available "
                  "for a --resume run in this process. Run `python "
                  "train.py --export-only` style follow-up, or call "
                  "report_per_class_map(model, 'datasets/unified.yaml', "
                  "a concrete --batch value) manually if needed after a "
                  "resumed run.")

        del eval_model
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    export_for_cpp(best_pt)


if __name__ == "__main__":
    main()