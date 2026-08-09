"""
train.py — Fine-tune YOLO26 for Project R.A.D's aerial detection model
=======================================================================
Defaults are tuned for a 6GB laptop RTX 3060. If `nvidia-smi` shows more
VRAM available, see the comments below for what to bump.

Pipeline: trains against a UNIFIED class taxonomy (see class_map.py) --
person/car/large_vehicle/motorcycle/other_vehicle -- spanning VisDrone,
UAVDT, and SARD (see prepare_datasets.py). All three are genuinely
drone-native, low-altitude footage, so letting Ultralytics' Mosaic
augmentation freely composite across them is intentional -- unlike the
now-inactive xView (satellite) addition, which mixed a fundamentally
different visual domain into the same batches and measurably hurt
vehicle/person accuracy (see class_map.py's module docstring). UAVDT
adds vehicle diversity (no person labels); SARD adds SAR-relevant person
poses (standing/sitting/lying/exhausted/injured) that VisDrone's general
pedestrian labels don't cover.

Model size: --model defaults to "yolo26s.pt". Pass --model auto to
re-enable the yolo26m -> yolo26s -> yolo26n fallback search.

DEGRADATION AUGMENTATION -- why this exists, and calibrated against what:
  Every dataset in this pipeline (VisDrone/UAVDT/SARD) is clean,
  high-resolution, digitally-captured footage. The real deployment
  camera (E5-FPV, per its datasheet) is a 1/2.8" sensor outputting
  analog CVBS composite video: 1500TVL horizontal resolution, NTSC/PAL
  switchable, 0.0001 lux minimum illumination, 100 degree FOV, 6mm
  lens, transmitted over a 5.8GHz analog link. None of the three
  training sources have ever seen anything like that signal chain --
  a genuine train/deploy domain gap, structurally the same category of
  problem as the xView satellite-vs-drone mismatch (see class_map.py),
  just from the capture pipeline rather than the camera angle.

  install_degradation_augment() monkeypatches Ultralytics'
  Albumentations wrapper (the class model.train() instantiates
  internally, TRAIN split only -- see the "augmentation stays train-
  only" note below) with a pipeline where each artifact maps to a
  specific spec-sheet limitation, not a generic "add some noise" guess:
    - Downscale: 1500TVL is a genuine bandwidth ceiling on a composite
      signal, not a blur -- CVBS literally cannot carry detail finer
      than that regardless of sensor resolution. Downscale-then-
      upscale reproduces a resolving-power ceiling more faithfully than
      a blur kernel (blur softens edges; bandwidth-limiting also
      destroys fine texture, closer to what actually happens).
    - Motion/Gaussian blur: covers physical motion blur (0.0001 lux
      minimum illumination implies slow shutter speeds in real low-
      light SAR conditions, on a moving airframe) plus general analog
      softening.
    - ISO/Gauss noise (combined, not just one): 0.0001 lux is extreme
      low light -- even a good sensor needs real gain to expose
      anything down there, and gain means grain. Two noise models
      because ISONoise's color-correlated noise and GaussNoise's
      uncorrelated noise cover different parts of what a gain-amplified
      analog signal actually looks like.
    - Gamma / brightness-contrast: real SAR ops run at dusk, in
      shadowed rubble/wilderness, and everywhere the 0.0001-to-daylight
      range in between -- wider than Ultralytics' own RandomHSV covers,
      since HSV doesn't touch gamma curve shape.
    - NTSC/PAL interlace combing (custom, see _interlace_combing):
      deliberately tiny -- a 1px alternating-row shift at low
      probability. Larger shifts were considered and rejected: this
      project's objects of interest can be a handful of pixels wide, so
      anything beyond ~1px risks measurably displacing an already-tight
      box, which would REDUCE precision -- the opposite of the goal.
    - CoarseDropout (RF dropout burst): deliberately small holes at low
      probability, for the same reason -- this pipeline runs
      degradation augmentation image-only (contains_spatial forced to
      False, see install_degradation_augment()'s docstring), so
      Albumentations never sees bboxes when it runs this transform --
      a dropout hole large enough to fully cover a tiny labeled person
      creates a genuine label/image mismatch (the box says "person,"
      the pixels say nothing). This is the one transform in this
      pipeline that can actually corrupt training signal rather than
      just add noise to it, so it's kept intentionally weak rather than
      tuned for realism. NOTE: Albumentations' CoarseDropout does
      support bbox-aware handling (auto-dropping boxes below a
      min_visibility threshold) when given bbox_params -- that's a
      more rigorous fix than "keep the hole small" and is a valid
      future upgrade, but it requires restructuring this pipeline into
      a spatial-aware Compose plus re-wiring Ultralytics' __call__
      contract (bboxes/class_labels in, filtered ones back out), not
      just the __init__ patch used here. Not implemented -- flagging as
      a known available upgrade, not a bug in the current approach.
    - ImageCompression: generic proxy for whatever the downstream
      capture chain (VideoLink's MSMF/DSHOW grab path) does to the
      digitized signal -- not characterized precisely here, kept broad.

  VERSION ROBUSTNESS: albumentations' API has changed meaningfully
  across versions (e.g. Downscale's scale_min/scale_max became
  scale_range in 2.x; CoarseDropout's parameter names changed too).
  Each transform above is built independently and wrapped in its own
  try/except in build_degradation_transform() -- if one doesn't match
  your installed version, THAT ONE is dropped with a printed warning
  and every other transform still applies. A version mismatch degrades
  one artifact type, never the whole pipeline, and never crashes
  training. If every transform fails to build, degradation augmentation
  is disabled for the run (also non-fatal) rather than leaving you with
  a silently-broken partial pipeline.

  AUGMENTATION STAYS TRAIN-ONLY (verified, not just assumed): Ultralytics
  only builds an Albumentations instance inside its train-time transform
  pipeline (v8_transforms) -- the val dataset's build_transforms() never
  instantiates one. That means the val split is never touched by this
  patch, so val metrics stay a clean, undegraded measure of real
  performance -- exactly what you want to trust when deciding whether a
  training run actually improved things. make_augmentation_scope_check_
  callback() (new) verifies this empirically at the start of every run
  by walking each loader's actual transform tree, rather than relying on
  the paragraph above staying true forever -- see its docstring.

  NOTE: the monkeypatch itself targets a third-party internal class
  (ultralytics.data.augment.Albumentations.__init__), not a stable
  public API -- if an Ultralytics upgrade changes that class's
  signature, the patch may silently stop applying. The scope-check
  callback below will report "absent" from the train loader if that
  ever happens, so you'll see it in the log rather than finding out
  later from unexpectedly-clean-looking val curves.
  Safe with --workers 0 (the current default) since the patch is
  applied once in the main process before training starts, no
  per-worker re-import race to worry about.

  PATCH SIGNATURE ROBUSTNESS (fixed): patched_init() below now accepts
  *args/**kwargs and forwards them all to orig_init(), rather than
  declaring only `p=1.0`. This was previously a hardcoded signature
  match against the Ultralytics version this was written against;
  a newer Ultralytics release (confirmed against 8.4.116) added a
  `transforms=` kwarg to Albumentations.__init__ (used internally to
  pass hyp-driven augmentations in), which the old hardcoded signature
  couldn't accept, causing a hard TypeError at dataloader build time
  ("got an unexpected keyword argument 'transforms'") before training
  ever started. Forwarding *args/**kwargs makes this patch resilient to
  that kind of additive signature change going forward -- orig_init
  still runs with whatever Ultralytics passes, and this project's own
  transform/contains_spatial overrides are still applied afterward,
  same intent as before.

REAL-TIME DEPRIORITIZED (project decision): inference-side latency is
  not a hard constraint for this deployment -- precision/robustness on
  a noisy analog feed matters more than 30fps. This module's VRAM/time
  ceilings (WDDM, the 6GB card) are unaffected by that decision --
  those are still real limits during TRAINING regardless of how the
  final model gets deployed -- but it does mean two things once yolo26s
  @960px is a working, WDDM-safe baseline: (1) --multi-scale is worth
  turning on for scale robustness across SAR altitude variation, since
  the existing WDDM probes/fallback will catch it gracefully if it
  pushes past the VRAM ceiling rather than failing silently; (2) it's
  worth trying an even higher --imgsz (e.g. 1280) as a follow-up
  experiment purely for small-object accuracy, once 960px is confirmed
  stable, since nothing downstream needs this to run fast.

WDDM / silent shared-memory spillover -- IMPORTANT, read this:
  On Windows, when a CUDA allocation would exceed the card's physical
  VRAM, the WDDM driver does NOT throw an out-of-memory error the way
  Linux does -- it silently spills the excess into system RAM over PCIe
  and reports success. Training then "runs" but every excess allocation
  round-trips over PCIe each step, so it gets catastrophically slower.
  Confirmed twice now on this project:
    - yolo26m never fit at all, even at batch=1/960px.
    - yolo26s (the "known good" size) STILL hit it, but only ~2800
      batches into training (GPU_mem reported 7.71G against a 6144MiB
      card) -- i.e. it can develop well after training looks healthy,
      once denser batches / disk cache / allocator fragmentation build
      up. A single check at startup is NOT enough.
  Fix: two layers of monitoring, both raising VRAMBudgetExceeded (caught
  the same way a real OOM is, triggering fallback to a smaller model):
    1. make_startup_probe_callback() -- on_train_batch_end, checks the
       first few real batches (memory + per-batch wall time) so an
       obviously-too-big model gets caught in seconds, not hours.
    2. make_ongoing_watchdog_callback() -- on_train_epoch_end, re-checks
       peak allocated memory (cumulative high-water mark) and that
       epoch's wall-clock time against --max-epoch-minutes (if set)
       EVERY epoch for the whole run, since spillover can develop later.
  Neither reads trainer-internal attributes beyond the callback firing
  and the epoch number Ultralytics passes in (a stable, documented
  field) -- everything else is tracked in the closure.

  Root-cause refinement (confirmed against the per-batch log of this
  project's actual run): the jumps aren't random -- they correlate
  exactly with the "Instances" column (batch 44: 378 -> 3632 instances,
  GPU_mem 2.78G -> 4.94G; batch 61: -> 4864 instances, 4.94G -> 8.72G,
  the exact moment it/s dropped). Ultralytics' mosaic augmentation
  (mosaic=1.0) composites up to 4 source images into one training sample
  before batching even happens; a composite that happens to pull in a
  few dense frames can carry 1000+ instances in a single slot instead of
  the usual couple hundred, and loss/target-assignment memory scales
  with instance count, not image size. Combined with PyTorch's caching
  allocator -- which keeps a batch's peak reservation as a permanent
  floor rather than returning it to the driver right away, to avoid slow
  cudaFree calls every step -- each unlucky dense batch permanently
  ratchets the reserved pool up one step: a staircase, not a leak. This
  doesn't replace the WDDM explanation above, it completes it: the
  staircase is why reserved memory keeps climbing over the course of a
  run; WDDM spillover is what happens once that climb crosses the card's
  physical 6GB ceiling (silent slowdown instead of a clean crash).
    3. PYTORCH_CUDA_ALLOC_CONF now also sets
       garbage_collection_threshold:0.8 -- PyTorch's own allocator knob
       for exactly this pattern: once reserved memory exceeds 80% of
       what's actually allocated, it proactively releases blocks back
       to the driver instead of hoarding the post-spike high-water mark
       forever. This addresses the STAIRCASE (the permanently elevated
       floor); it does NOT reduce the genuine peak a single very-dense
       mosaic batch needs -- if one composite's real memory requirement
       alone exceeds 6GB, no allocator setting saves it, which is
       exactly why probes 1 and 2 above stay in place as the actual
       safety net, not just this env var.
    4. --mosaic (default 1.0, Ultralytics' own default) lets you lower
       the composite probability (e.g. 0.5-0.7) if the watchdog keeps
       tripping on dense composited batches -- trades away some
       augmentation strength for fewer extreme-instance-count batches.
       Leave at default unless probes 1/2 are firing repeatedly.
    5. mosaic_guard.py / --max-mosaic-instances (default 800) --
       PROACTIVE version of the fix, not just reactive monitoring.
       Patches Ultralytics' Mosaic to steer partner-image selection away
       from combinations that would exceed this instance budget, instead
       of picking partners uniformly at random. Does NOT reduce the
       mosaic grid size, and can't fully protect against a single source
       image that's dense enough on its own -- probes 1/2 stay in place
       as the reactive backstop underneath this. NOTE: the cap counts
       TOTAL instances regardless of class, so it doesn't specifically
       protect or disadvantage person vs. vehicle-dense frames -- it's
       class-agnostic by design, which is the right behavior here since
       the goal is a VRAM safety net, not a class-balancing mechanism
       (that's what oversampling below is for).
  Given it recurred even on yolo26s at 960px with the combined dataset:
  recommend trying --imgsz 640 for the real run (see Usage below) --
  more VRAM headroom AND it should let --batch go above 1, which batch=1
  itself is a major speed tax (no batching efficiency at all).

CLOSE-MOSAIC / MULTI-SCALE (new CLI exposure, not new behavior for
close_mosaic -- it was already Ultralytics' default, just not tunable
without editing code mid-run):
  --close-mosaic (default 10, matches Ultralytics' own default): the
  last N epochs train WITHOUT mosaic compositing, seeing clean
  single-source images. This matters specifically for small/tiny-object
  recall (e.g. a person at altitude reduced to a handful of pixels) --
  letting the model converge on undistorted geometry right before
  training ends tends to sharpen exactly that failure mode. Raise it if
  final-epoch val samples still show weak small-object recall.
  --multi-scale (default off): varies input size +/-50% per batch.
  Improves robustness to the scale variation between low- and
  high-altitude passes, at the cost of extra VRAM headroom -- another
  reason to prefer --imgsz 640 over 960 if you turn this on, given the
  WDDM history above.

Crash-safety / pause-and-resume (no separate "temp file" mechanism
needed -- Ultralytics already does this, it just wasn't surfaced before):
  - last.pt and best.pt are overwritten after EVERY completed epoch by
    default, not just every N -- so a crash, a closed terminal, or a
    Ctrl+C mid-epoch loses at most the current in-progress epoch, never
    more.
  - `python train.py --resume` picks up the most recently modified
    runs/detect/*/weights/last.pt and continues with full optimizer
    state (epoch count, LR schedule position, etc.) -- NOT a from-scratch
    restart.
  - --save-period (default 5) additionally asks Ultralytics to also
    write out numbered snapshots (epoch5.pt, epoch10.pt, ...) instead of
    only ever having the single latest last.pt -- extra insurance if you
    ever want to roll back further than one epoch, or in case last.pt
    itself gets corrupted by an interruption mid-write. Set to -1 to
    disable and save disk space.
  - --max-epoch-minutes (optional) is a second failsafe: if any one
    epoch takes longer than this, the ongoing watchdog raises and this
    model size is abandoned in favor of a smaller one, rather than
    silently eating an entire time budget on a run that's thrashing.

Overfitting / "fast but low error" controls, given the dataset is still
fairly small/imbalanced and time is limited:
  - --patience (default 30, raised from 15 -- see its --help text): early
    stopping on val mAP. This is the main overfitting guard already --
    best.pt is always the best-val epoch, not whatever epoch training
    happens to stop on.
  - --cos-lr (default on): cosine LR decay, tends to generalize slightly
    better late in training than linear.
  - --label-smoothing: kept for forward-compat only (reported deprecated/
    no-op on the installed Ultralytics version) -- only forwarded if you
    explicitly set it non-zero, with a one-time printed reminder.
  - copy_paste/mixup: mixup is genuinely active and helps here. copy_paste
    (VERIFIED against Ultralytics' actual implementation, not assumed):
    Ultralytics' CopyPaste augmentation only pastes objects using
    segmentation polygons (labels["instances"].segments) -- it is
    documented as working for segmentation tasks only. VisDrone, UAVDT,
    and SARD are all bounding-box-only YOLO labels with no polygons, so
    --copy-paste is currently a no-op on this pipeline regardless of its
    value. It's left on by default below because it's harmless (costs
    nothing when inert), but don't count on it for the sparser classes --
    oversampling and mixup are the augmentation actually doing that job
    right now. A real bbox-level copy-paste (pasting a cropped box region
    rather than a polygon) would need a custom transform; not implemented
    here.
  - Sparse-class oversampling (see prepare_datasets.py's
    _oversample_sparse_classes()): duplicates motorcycle/other_vehicle
    image paths in the train list so those classes are seen more often
    per epoch. --oversample-classes defaults to motorcycle,other_vehicle
    -- check prepare_datasets.py's new "Per-class instance counts" log
    block (printed every run) before assuming that's still the right
    list once UAVDT/SARD are folded in. "person" is this project's
    stated priority class and is NOT in the default list; it may or may
    not need to be, depending on the actual counts.
  - check_overfitting() after every run reads results.csv and flags it if
    val loss is rising while train loss keeps falling over the last N
    epochs -- a heuristic, not a verdict. This only looks at aggregate
    box loss, not per-class -- it can't tell you if person specifically
    is diverging while vehicles improve. See report_per_class_map() below
    for that.
  - evaluate_per_source() reports mAP separately for VisDrone/UAVDT/SARD's
    own val sets after training (now including a per-class breakdown,
    see below), not just the blended unified.yaml number -- a blended
    average can hide one source regressing while another improves.
  - report_per_class_map() (new) reports mAP50-95 PER CLASS on the full
    blended val set -- the direct check for "one class detected well,
    another degraded." A healthy overall mAP can hide e.g. vehicles
    scoring well while person -- the actual SAR priority -- lags far
    behind. This is the single most direct tool in this file for
    catching that failure mode; check it after every real run.
  - The most reliable fix for overfitting on a dataset this size is still
    more/varied data, not more knobs -- prepare_datasets.py covers
    adding more datasets.

Baseline history, for reference:
  yolo26n, 640px, VisDrone's original 10 classes -> mAP50 ~0.31 (plateaued,
    small/rare classes missed outright -- a resolution/capacity ceiling)
  yolo26s, 960px, VisDrone's original 10 classes -> mAP50 ~0.465 (confirmed
    the ceiling diagnosis; every class improved)
  yolo26m, 960px, unified taxonomy -> never actually trained at a usable
    speed: silently spilled into shared system RAM immediately.
  yolo26s, 960px, unified taxonomy, VisDrone+xView -> spilled into WDDM
    partway through epoch 2 -- prompted the 640px recommendation below,
    the ongoing (not just startup) watchdog, and (separately) the later
    decision to drop xView entirely for accuracy reasons, not just VRAM.
  yolo26s, unified taxonomy (5 classes), VisDrone+UAVDT+SARD -> current
    run, not yet complete.

Usage:
    python train.py                    # yolo26s, unified taxonomy (default)
    python train.py --model auto       # re-enable yolo26m->s->n search,
                                        # each candidate checked by both
                                        # the startup probe and the
                                        # ongoing per-epoch watchdog
    python train.py --imgsz 960 --batch 1 --epochs 5   # smoke test at the
                                        # resolution that actually helps
                                        # tiny/small-object recall --
                                        # always run a short smoke test
                                        # before committing to a full run
    python train.py --imgsz 640 --batch 4   # fallback starting point if
                                        # 960px trips the WDDM probes --
                                        # see note above
    python train.py --model yolo26m.pt --imgsz 640   # try more capacity
                                        # only AFTER a 960px yolo26s run
                                        # is your known baseline -- m
                                        # never fit at 960px previously
    python train.py --data VisDrone.yaml --model yolo26n.pt --imgsz 640
                                        # reproduce the ORIGINAL 10-class
                                        # baseline exactly, bypassing the
                                        # unified taxonomy entirely
    python train.py --resume           # continue the most recent
                                        # interrupted/completed run from
                                        # its last.pt, full optimizer
                                        # state preserved -- NOT a restart
    python train.py --epochs 5         # smoke test first, always worth it
"""

import argparse
import csv
import json
import os
import time
from pathlib import Path

# Must be set before torch is imported (it reads this at CUDA init time).
os.environ.setdefault(
    "PYTORCH_CUDA_ALLOC_CONF",
    "expandable_segments:True,garbage_collection_threshold:0.8",
)

import yaml
from ultralytics import YOLO

import prepare_datasets
from class_map import UNIFIED_CLASSES
from mosaic_guard import install_instance_cap

SCRIPT_DIR = Path(__file__).resolve().parent
RUNS_PROJECT = SCRIPT_DIR / "runs" / "detect"

# per_source_val.json is written by prepare_datasets.py into ITS
# DATASETS_DIR, which follows Ultralytics' own global 'datasets_dir'
# setting -- NOT necessarily a "datasets" folder next to this script.
# Confirmed on this project: that setting resolves one level above
# NNTraining/ (i.e. .../DroneCockpitUI/datasets/), so a hardcoded
# SCRIPT_DIR / "datasets" here silently looked in the wrong folder and
# evaluate_per_source() was skipped on every run ("no per_source_val.json
# found ... wasn't run this session?") even though prepare_datasets.py
# had, in fact, just run and written it -- just somewhere else. Resolving
# via the same Ultralytics SETTINGS lookup prepare_datasets.py itself
# uses keeps both scripts pointed at the same actual location.
try:
    from ultralytics.utils import SETTINGS
    DATASETS_DIR = Path(SETTINGS.get("datasets_dir", SCRIPT_DIR / "datasets"))
except Exception:
    DATASETS_DIR = SCRIPT_DIR / "datasets"

PER_SOURCE_MANIFEST_PATH = DATASETS_DIR / "per_source_val.json"

# Tried in this order when --model auto is used. Biggest first -- more
# capacity generally means better accuracy, so we only give it up if it
# genuinely doesn't fit (checked directly by the probes below instead of
# waiting on an exception Windows won't throw).
MODEL_SIZE_CANDIDATES = ["yolo26m.pt", "yolo26s.pt", "yolo26n.pt"]


class VRAMBudgetExceeded(Exception):
    """Raised by either preflight callback when memory/timing signals
    indicate this model size doesn't actually fit -- see the WDDM note in
    this module's docstring for why we can't just wait for a normal CUDA
    OOM exception on Windows, and why one check at startup isn't enough."""


def make_startup_probe_callback(device, warmup_batches=2, probe_batches=3,
                                  max_batch_seconds=5.0, safety_margin=0.90):
    """
    on_train_batch_end callback: watches the first few real training
    batches so an obviously-too-big model size gets caught in seconds
    rather than hours. See make_ongoing_watchdog_callback() for the
    complementary check that keeps watching for the rest of the run --
    this one alone is NOT sufficient (confirmed: yolo26s passed this
    check cleanly and still spilled ~2800 batches later).
    """
    state = {"count": 0, "checked": False, "last_t": None}

    def callback(trainer):
        import torch

        if state["checked"]:
            return

        now = time.time()
        state["count"] += 1
        prev_t = state["last_t"]
        state["last_t"] = now

        if state["count"] <= warmup_batches:
            return  # let first-batch overhead (cudnn autotune etc.) settle
        if state["count"] < warmup_batches + probe_batches:
            return

        state["checked"] = True
        total = torch.cuda.get_device_properties(device).total_memory
        peak = torch.cuda.max_memory_allocated(device)
        batch_seconds = (now - prev_t) if prev_t else 0.0

        mem_bad = peak > total * safety_margin
        time_bad = batch_seconds > max_batch_seconds

        if mem_bad or time_bad:
            reasons = []
            if mem_bad:
                reasons.append(
                    f"peak VRAM {peak / 1e9:.2f}GB vs "
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
    """
    on_train_epoch_end callback: re-checks the CUMULATIVE peak allocated
    memory (the high-water mark since training started) against the
    card's physical total after every epoch, for the whole run -- not
    just at startup. Also flags any single epoch that takes longer than
    --max-epoch-minutes, if set.
    """
    state = {"last_epoch_start": None}

    def callback(trainer):
        import torch

        now = time.time()
        epoch_seconds = (now - state["last_epoch_start"]
                          if state["last_epoch_start"] else None)
        state["last_epoch_start"] = now  # reset for the next epoch

        total = torch.cuda.get_device_properties(device).total_memory
        peak = torch.cuda.max_memory_allocated(device)
        mem_bad = peak > total * safety_margin

        time_bad = (max_epoch_minutes is not None
                    and epoch_seconds is not None
                    and epoch_seconds > max_epoch_minutes * 60)

        if mem_bad or time_bad:
            reasons = []
            if mem_bad:
                reasons.append(
                    f"cumulative peak VRAM {peak / 1e9:.2f}GB vs "
                    f"{total / 1e9:.2f}GB physical (WDDM shared-memory "
                    f"spillover signature)")
            if time_bad:
                reasons.append(
                    f"epoch {getattr(trainer, 'epoch', '?')} took "
                    f"{epoch_seconds / 60:.1f} min, over the "
                    f"{max_epoch_minutes} min budget")
            raise VRAMBudgetExceeded(
                f"[ongoing watchdog, epoch "
                f"{getattr(trainer, 'epoch', '?')}] " + " and ".join(reasons))

    return callback


# ---------------------------------------------------------------------
# Degradation augmentation -- see module docstring for the full "why"
# and the per-artifact reasoning tied to the E5-FPV datasheet.
# ---------------------------------------------------------------------

def _interlace_combing(image, **kwargs):
    """
    Custom Albumentations Lambda: mimics NTSC/PAL interlace 'combing' --
    alternate scanlines slightly horizontally misaligned, as happens
    when successive interlaced fields are captured during airframe
    motion. Deliberately minimal: a 1px row shift. This project's
    objects of interest (a person at altitude) can be only a handful of
    pixels wide, so anything beyond ~1px risks measurably displacing an
    already-tight label box -- which would hurt precision, the opposite
    of the goal. Kept at low probability in the Compose pipeline for the
    same reason. If your capture chain already deinterlaces cleanly
    before frames reach the training pipeline, this artifact may simply
    never fire in practice on your real footage -- that's fine, it costs
    nothing to leave in as a low-probability guard for capture chains
    that don't.
    """
    import numpy as np
    img = image.copy()
    img[1::2] = np.roll(img[1::2], 1, axis=1)
    return img


def _try_build_transform(name, builder):
    """
    Builds one Albumentations transform, catching any exception (most
    commonly a TypeError from a parameter that was renamed/removed
    between albumentations versions -- e.g. Downscale's scale_min/
    scale_max became scale_range in 2.x). Returns None on failure
    instead of propagating, so one incompatible transform never takes
    down the whole degradation pipeline -- see the VERSION ROBUSTNESS
    note in this module's docstring.
    """
    try:
        return builder()
    except Exception as e:
        print(f"[degradation aug] Skipping '{name}' -- not compatible with "
              f"the installed albumentations version ({e}). Every other "
              f"degradation transform is unaffected.")
        return None


def build_degradation_transform():
    """
    Builds an Albumentations pipeline calibrated against the E5-FPV
    camera's actual datasheet (1500TVL CVBS analog, NTSC/PAL, 0.0001 lux
    minimum illumination) rather than a generic noise guess -- see the
    module docstring for what each transform maps to and why. Returns
    None (with a printed warning) if albumentations isn't installed, or
    if every individual transform fails to build against the installed
    version -- both cases are non-fatal, training proceeds without
    degradation augmentation rather than crashing.

    VERSION-AWARE, not just version-TOLERANT: four of these transforms
    (Downscale, GaussNoise, CoarseDropout, ImageCompression) had their
    parameter names changed between albumentations 1.x and 2.x --
    confirmed directly against an installed 2.0.8: the OLD 1.x kwarg
    names don't raise an error under 2.x, they get silently accepted and
    IGNORED, falling back to the library's own defaults. That's worse
    than a crash for one transform specifically -- CoarseDropout's 2.x
    default hole size is 10-20% of the image per side, which is large
    enough to fully blank out a tiny labeled person, exactly the
    label-corruption risk this transform was deliberately kept small to
    avoid (see module docstring). So this isn't just wrapped in
    try/except: the correct kwarg names for the detected major version
    are used directly, with try/except kept underneath as a second-line
    defense for any future API shape neither branch anticipates.
    """
    try:
        import albumentations as A
    except ImportError:
        print("[degradation aug] albumentations not installed -- skipping. "
              "Run: pip install albumentations   to enable analog-feed "
              "degradation augmentation (recommended before the real run, "
              "not required).")
        return None

    try:
        alb_major = int(str(A.__version__).split(".")[0])
    except Exception:
        alb_major = 1  # unknown -- assume the older (pre-2.0) API shape
    print(f"[degradation aug] Detected albumentations {A.__version__} "
          f"(using {'2.x' if alb_major >= 2 else '1.x'}-style parameter "
          f"names).")

    pieces = []

    def build_downscale():
        if alb_major >= 2:
            return A.Downscale(scale_range=(0.35, 0.75), p=0.4)
        return A.Downscale(scale_min=0.35, scale_max=0.75,
                             interpolation=1, p=0.4)

    downscale = _try_build_transform(
        "Downscale (1500TVL bandwidth ceiling)", build_downscale)
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
        # var_limit (1.x) is raw pixel-value variance (0-255 scale);
        # std_range (2.x) is std as a fraction of the 0-1 normalized
        # range. (10.0, 60.0) var -> std ~ sqrt(var)/255 ~ (0.012, 0.030)
        # -- widened slightly below for comparable visible effect.
        if alb_major >= 2:
            return A.GaussNoise(std_range=(0.015, 0.05), p=1.0)
        return A.GaussNoise(var_limit=(10.0, 60.0), p=1.0)

    noise = _try_build_transform(
        "low-light sensor noise (0.0001 lux gain grain)",
        lambda: A.OneOf([
            A.ISONoise(color_shift=(0.01, 0.06), intensity=(0.15, 0.5), p=1.0),
            build_gauss_noise(),
        ], p=0.35))
    if noise is not None:
        pieces.append(noise)

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

    def build_coarse_dropout():
        # Deliberately weak -- see module docstring: this is the one
        # transform here that can corrupt a label (not just add noise
        # to it) if a hole fully covers a tiny object. Small/rare on
        # purpose, and (critically) the exact pixel sizes below are
        # confirmed to mean literal pixels under 2.x when given as
        # plain ints, not a fraction of image size -- verified directly
        # (5 holes of (4,4)/(10,10) produced exactly 200 zeroed pixels
        # = 5*4*10) before relying on it here.
        if alb_major >= 2:
            return A.CoarseDropout(num_holes_range=(1, 3),
                                     hole_height_range=(4, 4),
                                     hole_width_range=(10, 10),
                                     fill=0, p=0.08)
        return A.CoarseDropout(max_holes=3, max_height=4, max_width=10,
                                 fill_value=0, p=0.08)

    dropout = _try_build_transform(
        "RF dropout burst (deliberately small/rare, see docstring)",
        build_coarse_dropout)
    if dropout is not None:
        pieces.append(dropout)

    def build_compression():
        if alb_major >= 2:
            return A.ImageCompression(quality_range=(30, 65), p=0.25)
        return A.ImageCompression(quality_lower=30, quality_upper=65, p=0.25)

    compression = _try_build_transform(
        "downstream compression artifacts (generic capture-chain proxy)",
        build_compression)
    if compression is not None:
        pieces.append(compression)

    # Deliberately minimal -- see _interlace_combing's docstring for why
    # this is capped at a 1px shift rather than tuned for visual realism.
    interlace = _try_build_transform(
        "NTSC/PAL interlace combing (1px, low-probability by design)",
        lambda: A.Lambda(image=_interlace_combing, p=0.12))
    if interlace is not None:
        pieces.append(interlace)

    if not pieces:
        print("[degradation aug] No transforms could be built against the "
              "installed albumentations version -- degradation "
              "augmentation is disabled for this run. Check `pip show "
              "albumentations` and consider `pip install -U albumentations` "
              "or `pip install \"albumentations<2\"` if this keeps happening.")
        return None

    print(f"[degradation aug] Built {len(pieces)}/8 planned transforms "
          f"with version-correct parameters (any gap above is a genuine "
          f"unexpected-API skip, logged individually).")
    return A.Compose(pieces)


def install_degradation_augment():
    """
    Monkeypatches ultralytics.data.augment.Albumentations.__init__ so the
    transform it builds internally is replaced with
    build_degradation_transform()'s analog-feed-biased pipeline, instead
    of Ultralytics' own very-low-probability defaults (p=0.01 per op,
    tuned for generic robustness, not for this project's specific
    train/deploy domain gap).

    self.contains_spatial is forced to False regardless of what
    Ultralytics' own spatial-transform heuristic would say (it classifies
    both CoarseDropout and Lambda as "spatial" transforms, since
    Albumentations CAN make them bbox-aware given bbox_params). This
    pipeline's own A.Compose(pieces) call deliberately has no bbox_params
    set, so it MUST be called image-only (contains_spatial=False) or the
    call would break -- this is intentional, not an oversight, but it
    does mean CoarseDropout/Lambda run here without any bbox awareness at
    all, relying entirely on small hole size to avoid label corruption
    rather than Albumentations' own visibility-based box filtering. See
    the CoarseDropout bullet in the module docstring for the more
    rigorous (not implemented) alternative.

    NOTE: this patches an internal (non-public-API) class. If an
    Ultralytics upgrade changes Albumentations' __init__ signature or
    where it's instantiated, this patch may silently stop applying --
    worth a quick visual sanity check (dump a few augmented training
    images) after any Ultralytics version bump. Safe with --workers 0
    (this project's default) since it's applied once in the main process
    before model.train() spins up any dataloader workers.

    patched_init() forwards *args/**kwargs to orig_init() rather than
    hardcoding a fixed parameter list -- see "PATCH SIGNATURE
    ROBUSTNESS" in the module docstring for why (Ultralytics 8.4.116
    added a `transforms=` kwarg to the real __init__ that an
    earlier, hardcoded `def patched_init(self, p=1.0)` couldn't accept,
    causing a hard TypeError before training could start).
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
          "shift) -- closes some of the train/deploy domain gap against "
          "the real FPV analog feed.")


def _contains_albumentations(transform_obj, aug_mod, _depth=0, _max_depth=6) -> bool:
    """
    Best-effort recursive search through Ultralytics' Compose tree for an
    Albumentations instance. Walks transform_obj's own .transforms list
    and any nested .pre_transform -- these are the two attribute names
    Ultralytics' Compose/BaseMixTransform classes use to nest
    sub-pipelines as of the version this was written against. This is
    non-public internal structure (same caveat as the monkeypatch
    itself), so this degrades to returning False past _max_depth or on
    an unexpected shape rather than raising -- see
    make_augmentation_scope_check_callback() for how the "unknown" case
    is surfaced honestly instead of being reported as a false pass.
    """
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
    """
    on_train_start callback: empirically confirms the degradation
    augmentation patch reaches the TRAIN dataloader and does NOT reach
    the validation dataloader -- i.e. val metrics stay a clean,
    undegraded measure of real performance, and only train batches get
    the analog-feed simulation. This is a real check performed every
    run, not just a design assumption documented in a comment: it walks
    each loader's actual transform tree looking for an Albumentations
    instance.

    Best-effort by design (see _contains_albumentations docstring) -- if
    a loader's transform tree can't be walked (e.g. a future Ultralytics
    version changed its internal shape), this prints 'unknown' rather
    than a false pass, so a broken check never silently hides a real
    problem behind output that looks reassuring.
    """
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
                      f".transforms attribute -- unknown (Ultralytics "
                      f"internals may have changed shape).")
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
                  "would no longer be a clean measure of real performance "
                  "-- stop and investigate before trusting them.")
        elif train_found is False:
            print("\n[WARNING] Degradation augmentation does NOT appear to "
                  "be reaching the TRAIN set either -- the monkeypatch may "
                  "not be taking effect this run. Confirm "
                  "install_degradation_augment() ran before model.train() "
                  "was called and that albumentations imported "
                  "successfully (see the '[degradation aug]' log lines "
                  "above, earlier in this run).")
        elif train_found is None or val_found is None:
            print("\n[note] Scope check was inconclusive for at least one "
                  "loader (see 'unknown' above) -- Ultralytics internals "
                  "may not match what this check expects. Training "
                  "proceeds normally either way; this check is a bonus "
                  "confirmation, not a gate.")
        else:
            print("\n[OK] Degradation augmentation confirmed train-only "
                  "for this run.")
        print(f"{'=' * 70}")

    return callback


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="yolo26s.pt",
                    help="Base pretrained weights, or 'auto' to try "
                         "yolo26m first and fall back to yolo26s then "
                         "yolo26n if a size doesn't fit.")
    p.add_argument("--data", default=None,
                    help="Dataset yaml. Defaults to the auto-generated "
                         "datasets/unified.yaml (see prepare_datasets.py) "
                         "-- unified taxonomy spanning VisDrone+UAVDT+SARD "
                         "plus anything under datasets/external/. Pass "
                         "'VisDrone.yaml' explicitly to reproduce the "
                         "original 10-class baseline instead.")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=960,
                    help="960 is the historical default -- objects are "
                         "small even at native resolution. BUT: this "
                         "combo has shown WDDM spillover even on yolo26s "
                         "at 960px/batch=1 (see WDDM note). Try 640 first "
                         "-- more headroom, and lets --batch go above 1.")
    p.add_argument("--batch", type=int, default=1,
                    help="Explicit batch size. batch=1 has zero batching "
                         "efficiency and is a major speed tax by itself, "
                         "on top of the VRAM issues -- once you find "
                         "settings that leave real headroom (e.g. "
                         "--imgsz 640), test raising this via a short "
                         "--epochs 5 run before committing. Pass -1 for "
                         "AutoBatch -- unreliable at 960px on this card in "
                         "earlier testing, more likely to work at 640px.")
    p.add_argument("--device", default=0,
                    help="GPU index. 0 = first (only, on a laptop) GPU. "
                         "Use 'cpu' only to sanity-check the script runs, "
                         "never for a real training run.")
    p.add_argument("--workers", type=int, default=0,
                    help="Dataloader worker processes. Default 0 (main "
                         "process only) -- spawning workers on Windows "
                         "reloads the full CUDA DLL stack per process, "
                         "which has caused paging-file exhaustion crashes "
                         "on this setup. Raise to 2-4 only after fixing "
                         "Windows' paging file size if dataloading "
                         "becomes the bottleneck.")
    p.add_argument("--cache", default="disk",
                    help="'disk' caches decoded images to local disk "
                         "between epochs. Set to False if disk space is "
                         "tight.")
    p.add_argument("--resume", action="store_true",
                    help="Continue training from the most recently modified "
                         "runs/detect/*/weights/last.pt -- full optimizer "
                         "state preserved, NOT a from-scratch restart. "
                         "--model/--data/--name/--epochs are ignored when "
                         "this is set. NOTE: if class_map.py's taxonomy "
                         "changed since that checkpoint was trained "
                         "(different nc), resuming will fail loudly -- "
                         "that's correct behavior; start a fresh run "
                         "instead.")
    p.add_argument("--save-period", type=int, default=5,
                    help="Also write numbered checkpoint snapshots "
                         "(epoch5.pt, epoch10.pt, ...) every N epochs. "
                         "Set to -1 to disable and save disk space.")
    p.add_argument("--max-epoch-minutes", type=float, default=None,
                    help="Optional failsafe: if any single epoch takes "
                         "longer than this many minutes, the ongoing "
                         "watchdog aborts this model size in favor of a "
                         "smaller one.")
    p.add_argument("--copy-paste", type=float, default=0.3,
                    help="Copy-paste augmentation probability (0.0-1.0). "
                         "NOTE: verified against Ultralytics' actual "
                         "implementation -- CopyPaste requires "
                         "segmentation polygons, which none of this "
                         "project's datasets have (bbox-only YOLO "
                         "labels). This flag is currently a no-op on this "
                         "pipeline; left on by default only because it's "
                         "harmless. Don't rely on it for sparse-class "
                         "protection -- see mixup/oversampling instead.")
    p.add_argument("--mixup", type=float, default=0.1,
                    help="Mixup augmentation probability (0.0-1.0). "
                         "Generally helps generalization on small/"
                         "imbalanced datasets. Set to 0.0 to disable.")
    p.add_argument("--max-mosaic-instances", type=int, default=800,
                    help="PROACTIVE fix for the VRAM staircase (see "
                         "root-cause note above): caps the instance "
                         "count Mosaic partner selection will aim to "
                         "stay under when compositing images. See "
                         "mosaic_guard.py. Set to a very large number "
                         "(e.g. 999999) to effectively disable.")
    p.add_argument("--mosaic-max-tries", type=int, default=40,
                    help="How many candidate partner images the instance "
                         "cap will sample/reject before giving up and "
                         "falling back to the least-dense candidates.")
    p.add_argument("--mosaic", type=float, default=1.0,
                    help="Mosaic augmentation probability (0.0-1.0). "
                         "Lower this (e.g. 0.5-0.7) ONLY if the startup "
                         "probe or ongoing watchdog keep tripping.")
    p.add_argument("--close-mosaic", type=int, default=10,
                    help="Disable mosaic for the last N epochs -- lets the "
                         "model converge on clean (non-composited) images, "
                         "which matters specifically for small-object "
                         "recall (tiny person detections at altitude). "
                         "Ultralytics default is 10; raise if small "
                         "objects still look weak in final-epoch val "
                         "samples.")
    p.add_argument("--multi-scale", action="store_true",
                    help="Vary input size +/-50%% per batch during "
                         "training. Costs more VRAM headroom (another "
                         "reason to prefer --imgsz 640 over 960 if you "
                         "enable this), but improves robustness to the "
                         "object-scale variation between low and high "
                         "altitude passes.")
    p.add_argument("--no-degradation-aug", dest="degradation_aug",
                    action="store_false", default=True,
                    help="Disable the analog-feed degradation "
                         "augmentation pipeline (blur/compression/noise/"
                         "gamma). On by default -- see module docstring. "
                         "Requires albumentations; auto-skips with a "
                         "warning if it isn't installed.")
    p.add_argument("--oversample-classes", default="motorcycle,other_vehicle",
                    help="Comma-separated UNIFIED_CLASSES names to "
                         "oversample in the train split (see "
                         "prepare_datasets.py's _oversample_sparse_"
                         "classes()). Set to '' to disable. Check the "
                         "'Per-class instance counts' block prepare_"
                         "datasets.py prints every run before trusting "
                         "this default -- 'person' (this project's stated "
                         "priority) is not in it.")
    p.add_argument("--oversample-multiplier", type=int, default=3,
                    help="How many times each oversampled-class image "
                         "path is duplicated in the train list. 3 = seen "
                         "3x as often per epoch as it naturally would be.")
    p.add_argument("--patience", type=int, default=30,
                    help="Early stopping: stop if val mAP hasn't improved "
                         "in this many epochs. best.pt is always kept "
                         "from the best-val epoch regardless. Raised from "
                         "Ultralytics' 15-epoch-ish norm to 30 given the "
                         "added degradation augmentation -- a heavier "
                         "augmentation pipeline makes val loss noisier "
                         "epoch-to-epoch, so stopping at 15 risked cutting "
                         "off before real improvement showed through the "
                         "added noise. Matches this project's stated "
                         "priority (precision/robustness over fast "
                         "iteration, since real-time deployment isn't a "
                         "constraint) -- lower it back toward 15 if you're "
                         "doing a quick comparison run instead.")
    p.add_argument("--cos-lr", dest="cos_lr", action="store_true",
                    default=True,
                    help="Cosine LR decay instead of linear (default on).")
    p.add_argument("--no-cos-lr", dest="cos_lr", action="store_false",
                    help="Disable cosine LR decay, use Ultralytics' "
                         "linear default instead.")
    p.add_argument("--label-smoothing", type=float, default=0.0,
                    help="Label smoothing. Reported deprecated/no-op on "
                         "the installed Ultralytics version -- kept for "
                         "forward-compat only.")
    p.add_argument("--name", default=None,
                    help="Run subfolder name under runs/detect/. Defaults "
                         "to '<model>_<imgsz>' for a fresh run. When "
                         "combined with --resume, targets that specific "
                         "run's last.pt instead of guessing by most-"
                         "recently-modified file across all runs -- "
                         "e.g. --resume --name yolo26s_960-4. Strongly "
                         "recommended over bare --resume if more than "
                         "one run folder exists under runs/detect/.")
    p.add_argument("--export-only", action="store_true",
                    help="Skip training, just export an existing "
                         "runs/detect/<name>/weights/best.pt.")
    p.add_argument("--weights", default=None,
                    help="Checkpoint to export when --export-only is set. "
                         "Defaults to the most recently modified "
                         "runs/detect/*/weights/best.pt found.")
    p.add_argument("--skip-per-source-eval", action="store_true",
                    help="Skip the post-training per-source validation "
                         "pass (VisDrone/UAVDT/SARD evaluated separately) "
                         "AND the blended per-class mAP report. On by "
                         "default since it's a handful of extra quick "
                         "val() passes -- disable only if you're "
                         "iterating fast and don't need it every run.")
    return p.parse_args()


def default_run_name(model: str, imgsz: int) -> str:
    stem = Path(model).stem  # "yolo26s.pt" -> "yolo26s"
    return f"{stem}_{imgsz}"


def _is_resumable_checkpoint(path: Path) -> bool:
    """
    Checks whether a .pt checkpoint actually has resumable training
    state (epoch counter + optimizer state), BEFORE handing it to
    Ultralytics' model.train(resume=True).

    Why this exists: a checkpoint from a run that completed normally
    (reached its target epoch count, wasn't interrupted) has its
    optimizer stripped automatically -- see the "Optimizer stripped
    from ...last.pt" line Ultralytics prints at the end of every
    successful run. That checkpoint is no longer resumable, no matter
    what --epochs is set to in that run's saved args.yaml. Ultralytics
    itself detects this and prints a warning ("not a resumable training
    checkpoint ... Starting new training instead") -- but the "new
    training" it starts uses ITS OWN internal defaults (coco8.yaml,
    80 classes, batch=16, workers=8, etc.), NOT this project's real
    data/imgsz/batch/degradation-aug setup, because train.py's resume
    branch calls model.train(resume=True) with no other kwargs to fall
    back on. Confirmed on this project: that silent fallback trained
    several real epochs against the wrong 4-image toy dataset before
    anyone noticed. Checking resumability ourselves first, and refusing
    loudly with the correct next step, is much safer than letting that
    silent fallback happen again.
    """
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


def find_latest_last_pt(name: str | None = None) -> Path:
    """
    Locates the last.pt to resume from.

    If `name` is given (pass --name when resuming, matching the run
    folder under runs/detect/), resumes THAT specific run only --
    error loudly if it doesn't have a last.pt, rather than silently
    falling back to something else.

    If `name` is not given, falls back to the previous behavior: picks
    the most-recently-modified last.pt across ALL runs/detect/*/ --
    but now prints every candidate it found (path + mtime) before
    choosing, so a stray/unrelated run folder (e.g. "runs/detect/train/"
    left over from an ad-hoc `yolo train` invocation outside this
    project's own scripts, which defaults to that generic name) can't
    silently win just because its last.pt happens to be newer. Confirmed
    this can happen: an unrelated coco8.yaml/80-class run in a folder
    literally named "train" was picked over a real yolo26s_960-4 run
    here, because nothing surfaced the ambiguity before committing to
    an answer. Pass --name explicitly to avoid relying on mtime at all.
    """
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
              f"no --name given, so picking by most recent file "
              f"modification time. If this isn't the run you meant, "
              f"rerun with --resume --name <run_folder_name> instead:")
        for c in candidates:
            marker = " <- selected" if c == candidates[-1] else ""
            print(f"    {c}  (modified {c.stat().st_mtime}){marker}")

    return candidates[-1]


def find_latest_best_pt() -> Path:
    candidates = sorted(RUNS_PROJECT.glob("*/weights/best.pt"),
                         key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(
            f"No {RUNS_PROJECT}/*/weights/best.pt found. Run training "
            f"first, or pass --weights explicitly.")
    return candidates[-1]


def export_for_cpp(weights_path: Path):
    """
    Exports with end2end=False (the one-to-many head) so the ONNX output
    shape stays (1, nc+4, N) -- what DetectionLink::runInference() already
    parses via manual class-argmax + cv::dnn::NMSBoxes.
    """
    print(f"\nExporting {weights_path} to ONNX (one-to-many head, "
          f"NMS still required on the C++ side -- matches current "
          f"runInference())...")
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
    """
    Reads results.csv from a completed run and flags the textbook
    overfitting signature: val loss rising over the last `lookback`
    epochs while train loss keeps falling. Heuristic, not a verdict.

    NOTE: this only looks at aggregate box loss across ALL classes -- it
    cannot tell you if one class (e.g. person) is diverging while another
    (e.g. vehicles) keeps improving; an aggregate trend can look
    perfectly healthy while hiding exactly that. See report_per_class_map()
    for the per-class check.
    """
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


def report_per_class_map(model: YOLO, data_path: str) -> None:
    """
    Runs model.val() on the full (blended) val set from data_path and
    prints mAP50-95 PER CLASS -- not just the single averaged number
    Ultralytics prints by default.

    This is the direct, ready-made check for "one class detected well,
    another degraded": a healthy overall mAP can hide e.g. vehicle
    classes scoring well while person -- the actual SAR priority class
    -- sits far behind. evaluate_per_source() (below) breaks results
    down by DATASET; this breaks results down by CLASS on the combined
    val set, which is the number that actually answers this project's
    core accuracy question. Check this after every real run, not just
    the aggregate mAP Ultralytics prints during training.
    """
    print(f"\n{'=' * 70}")
    print("Per-class mAP (blended val set, all sources combined)")
    print(f"{'=' * 70}")
    metrics = model.val(data=data_path, split="val", verbose=False)
    per_class = metrics.box.maps  # array indexed by class, mAP50-95
    for idx, name in enumerate(UNIFIED_CLASSES):
        if idx < len(per_class):
            print(f"  {name:15s}  mAP50-95={per_class[idx]:.3f}")
    print(f"{'=' * 70}")
    print("If 'person' trails the vehicle classes here, that's your "
          "signal to increase person's share of training exposure "
          "(--oversample-classes) or add more SAR-relevant person data "
          "-- not a sign the model or pipeline is broken.")


def evaluate_per_source(model: YOLO, run_dir: Path) -> None:
    """
    Runs a SEPARATE model.val() pass against each source dataset's own
    val set (VisDrone/UAVDT/SARD individually), reading
    datasets/per_source_val.json (written by prepare_datasets.py). This
    is the only way to tell whether combining these three datasets
    actually helped -- the single blended unified.yaml val number can
    hide one source regressing while another improves. Silently skipped
    if the manifest doesn't exist (e.g. --data was set manually to
    bypass prepare_datasets.py).

    Now also prints a per-class breakdown for each source (new) --
    remember a source's per-class numbers are only meaningful for
    classes that source actually labels (see the printed note at the
    end): UAVDT never labels person, SARD never labels vehicles, so
    those rows will read as 0/undefined for that source and should be
    ignored, not read as "the model can't detect X."
    """
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

    tmp_dir = run_dir / "per_source_eval"
    tmp_dir.mkdir(exist_ok=True)

    for name, val_dirs in per_source.items():
        tmp_yaml = tmp_dir / f"{name.lower()}.yaml"
        with open(tmp_yaml, "w") as f:
            yaml.safe_dump({
                "path": None,
                "train": val_dirs,  # unused by val(), Ultralytics requires the key
                "val": val_dirs,
                "nc": len(UNIFIED_CLASSES),
                "names": UNIFIED_CLASSES,
            }, f, sort_keys=False)
        try:
            metrics = model.val(data=str(tmp_yaml), split="val", verbose=False)
            print(f"  {name:10s}  mAP50={metrics.box.map50:.3f}  "
                  f"mAP50-95={metrics.box.map:.3f}  "
                  f"precision={metrics.box.mp:.3f}  recall={metrics.box.mr:.3f}")
            per_class = metrics.box.maps
            for cls_idx, cls_name in enumerate(UNIFIED_CLASSES):
                if cls_idx < len(per_class):
                    print(f"      {cls_name:15s} mAP50-95={per_class[cls_idx]:.3f}")
        except Exception as e:
            print(f"  {name:10s}  [skipped: {e}]")

    print(f"{'=' * 70}")
    print("Note: each source's classes are limited to what that dataset "
          "actually labels (e.g. UAVDT has no person labels, SARD has no "
          "vehicle labels) -- per-class numbers for classes a source "
          "never annotated aren't meaningful for that source's row.")


def train_with_model_fallback(args, data_path: str):
    """
    Tries each candidate in MODEL_SIZE_CANDIDATES (or just args.model if
    it's an explicit choice, not 'auto') until one survives both the
    startup probe AND the ongoing watchdog without tripping
    VRAMBudgetExceeded, and without a real CUDA OOM either.
    """
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
              "Ultralytics reported this deprecated -- it may have no "
              "effect. Passing it through anyway.")
        train_kwargs["label_smoothing"] = args.label_smoothing

    for i, model_name in enumerate(candidates):
        this_run_name = (args.name or default_run_name(model_name, args.imgsz))
        print(f"\n{'=' * 70}\nAttempting: {model_name} @ {args.imgsz}px, "
              f"batch={args.batch}\n{'=' * 70}")
        try:
            model = YOLO(model_name)
            model.add_callback(
                "on_train_batch_end",
                make_startup_probe_callback(args.device))
            model.add_callback(
                "on_train_epoch_end",
                make_ongoing_watchdog_callback(
                    args.device, max_epoch_minutes=args.max_epoch_minutes))
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
            is_vram_issue = isinstance(e, VRAMBudgetExceeded) or is_oom_error(e)
            if not is_vram_issue:
                raise
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

    install_instance_cap(max_instances=args.max_mosaic_instances,
                          max_tries=args.mosaic_max_tries)

    if args.degradation_aug:
        install_degradation_augment()

    if args.export_only:
        weights = Path(args.weights) if args.weights else find_latest_best_pt()
        export_for_cpp(weights)
        return

    data_path = None  # only set on a fresh (non-resume) run -- see guard below

    if args.resume:
        last_pt = find_latest_last_pt(args.name)

        if not _is_resumable_checkpoint(last_pt):
            raise SystemExit(
                f"\n[resume] {last_pt} has no epoch/optimizer state left "
                f"to resume -- this means that run already completed "
                f"normally (Ultralytics strips optimizer state from "
                f"last.pt/best.pt once a run finishes, regardless of what "
                f"--epochs is set to in its args.yaml). True resume is "
                f"only possible for a run that was INTERRUPTED before "
                f"reaching its target epoch count.\n\n"
                f"To train more epochs starting from these weights "
                f"instead, run a fresh (non-resume) training pass using "
                f"this checkpoint as the starting model -- e.g.:\n\n"
                f"    python train.py --model \"{last_pt}\" --epochs 5\n\n"
                f"This is a warm start, not a true continuation: the LR "
                f"schedule and optimizer momentum restart from scratch "
                f"rather than picking up exactly where training left "
                f"off, but the learned weights carry over. Adjust "
                f"--epochs/--imgsz/--batch/etc. as you would for any "
                f"normal run.")

        print(f"\nResuming training from {last_pt}...")
        model = YOLO(str(last_pt))
        model.train(resume=True)
        run_name = last_pt.parent.parent.name
        used_model_name = None
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
              f"'[VRAM]' fallback messages above for why. If this wasn't "
              f"intentional (e.g. you specifically wanted yolo26m), check "
              f"whether --imgsz 640 (more VRAM headroom) lets the larger "
              f"model fit instead of silently accepting the fallback.")

    check_overfitting(run_dir)

    if not args.skip_per_source_eval:
        evaluate_per_source(model, run_dir)
        if data_path is not None:
            report_per_class_map(model, data_path)
        else:
            print("\n[per-class report] Skipped -- no data_path available "
                  "for a --resume run in this process. Run "
                  "`python train.py --export-only` style follow-up, or "
                  "call report_per_class_map(model, 'datasets/unified.yaml') "
                  "manually if you want it after a resumed run.")

    export_for_cpp(best_pt)


if __name__ == "__main__":
    main()