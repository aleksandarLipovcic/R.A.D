"""
train.py — Fine-tune YOLO26 for Project R.A.D's aerial detection model
=======================================================================
Defaults are tuned for a 6GB laptop RTX 3060.

PIPELINE: trains a UNIFIED taxonomy (person/car/large_vehicle/motorcycle/
other_vehicle, see class_map.py) spanning VisDrone + UAVDT + SARD (see
prepare_datasets.py) -- all genuine drone-native low-altitude footage, so
free Mosaic compositing across them is intentional (unlike the now-
inactive xView satellite addition, which hurt accuracy by mixing in a
different visual domain). generate_pseudo_labels.py --apply cross-labels
each source's originally-missing classes (UAVDT: person; SARD: vehicles)
once an image is fully reviewed, so per-source/per-class numbers below
are no longer purely "native-only" -- see evaluate_per_source()'s own
docstring before drawing conclusions from a near-zero class row.

MODEL SIZE: --model defaults to "yolo26s.pt" (fast, known-fitting, good
for validating the dataset itself). --model auto re-enables the
yolo26m -> yolo26s -> yolo26n fallback search, each candidate checked by
the VRAM probes below. yolo26m @ 960px is the real target once a yolo26s
smoke test confirms the data pipeline looks right -- a yolo26s smoke-test
mAP is not a stand-in for yolo26m's eventual accuracy (capacity differs).

DEGRADATION AUGMENTATION: every training source is clean digital footage;
the real deployment camera (E5-FPV) is analog CVBS (1500TVL, NTSC/PAL,
0.0001 lux min, 6mm/100deg lens, 5.8GHz link) -- a genuine train/deploy
domain gap. install_degradation_augment() monkeypatches Ultralytics'
Albumentations wrapper (train split only) with transforms calibrated
against that specific datasheet (bandwidth-limited downscale, motion/ISO/
Gauss noise, gamma range, 1px interlace combing, small RF-dropout holes,
mild lens barrel distortion, compression artifacts) rather than generic
noise. See build_degradation_transform()'s and install_degradation_
augment()'s own docstrings for the per-transform reasoning, the
version-robustness handling across albumentations 1.x/2.x, and why
CoarseDropout/OpticalDistortion are kept deliberately weak (this pipeline
runs image-only, no bbox_params, so an aggressive spatial transform here
would corrupt labels rather than just add noise).

REAL-TIME is deprioritized (precision on a noisy feed matters more than
fps), which is why --multi-scale and a higher --imgsz are worth trying
once a baseline is stable, and why nothing here is tuned for inference
speed. IMAGE SIZE: 960px is the recommended default -- this project's own
baseline history (below) shows 640px plateaus on small/rare classes while
960px fixed that ceiling, and mosaic_guard.py's instance cap has since
confirmed real headroom (~2GB/6GB) at 960px. imgsz doesn't need to match
any source's native resolution; Ultralytics letterboxes everything to one
square size regardless.

WDDM / VRAM: on Windows, an over-budget CUDA allocation silently spills
into system RAM instead of raising OOM, causing catastrophic (not clean)
slowdowns that can develop well after training looks healthy. Two
callbacks guard against this, both raising VRAMBudgetExceeded (handled
like a real OOM): make_startup_probe_callback() (first few batches -- see
its docstring for the AutoBatch false-positive it now avoids) and
make_ongoing_watchdog_callback() (every epoch, for the whole run -- see
its docstring). mosaic_guard.py's --max-mosaic-instances is the proactive
half of the same fix: dense mosaic composites (1000+ instances in one
slot) were the actual root cause of the VRAM "staircase" this project hit
twice; capping composite density keeps the caching allocator from
ratcheting its reserved pool up permanently. See each function's own
docstring for version history and confirmed numbers -- not duplicated
here to avoid this docstring drifting out of sync with the code.

CRASH-SAFETY: last.pt/best.pt are overwritten every epoch (not just every
N), so --resume (via the most recent runs/detect/*/weights/last.pt, full
optimizer state) loses at most one in-progress epoch. --save-period adds
numbered snapshots as extra insurance. See _is_resumable_checkpoint()'s
docstring for why a completed run's last.pt can't be resumed, and what to
do instead.

OVERFITTING / EVALUATION: --patience (30) is the main guard -- best.pt is
always the best-val epoch. check_overfitting() flags aggregate box-loss
divergence; report_per_class_map() and evaluate_per_source() are the
per-class/per-source breakdowns that catch what an aggregate can hide
(e.g. person -- the SAR priority class -- lagging while vehicles improve).
--copy-paste is currently a no-op on this pipeline (Ultralytics' CopyPaste
needs segmentation polygons this bbox-only data doesn't have) -- left on
because it's harmless; mixup and --oversample-classes are what's actually
protecting the sparse classes. See each function's docstring for detail.

BASELINE HISTORY (for reference; update after each real run):
  yolo26n, 640px, VisDrone-10cls        -> mAP50 ~0.31 (plateaued)
  yolo26s, 960px, VisDrone-10cls        -> mAP50 ~0.465 (every class up)
  yolo26m, 960px, unified               -> never fit (WDDM spillover)
  yolo26s, 960px, unified+xView         -> WDDM spillover epoch 2
  yolo26m, 640px, unified, 3ep smoke    -> mAP50 0.380 (STALE, predates
                                            pseudo-label --apply merge)
  yolo26s, 640px, unified, post-mosaic_guard -> ~2GB VRAM, full-run
                                                 mAP: TBD

USAGE:
    python train.py                             # yolo26s, unified taxonomy
    python train.py --model auto                # yolo26m->s->n fallback search
    python train.py --model yolo26s.pt --imgsz 640 --batch -1 --epochs 2
                                                  # smoke test: validates the
                                                  # DATASET, not yolo26m's
                                                  # eventual accuracy
    python train.py --imgsz 960 --batch 1 --epochs 5   # smoke test at the
                                                         # real target imgsz
    python train.py --imgsz 640 --batch 4        # fallback if 960px trips
                                                  # the WDDM probes
    python train.py --model yolo26m.pt --imgsz 960     # real target run --
                                                         # after a yolo26s
                                                         # smoke test passes
    python train.py --data VisDrone.yaml --model yolo26n.pt --imgsz 640
                                                  # original 10-class baseline
    python train.py --resume [--name RUN_NAME]   # continue most recent (or
                                                  # named) run's last.pt
    python train.py --export-only [--weights PATH]     # ONNX export only
"""

import argparse
import csv
import json
import os
import platform
import time
from pathlib import Path

# Must be set before torch is imported (it reads this at CUDA init time).
# NOTE: expandable_segments is not supported on Windows (confirmed by the
# "expandable_segments not supported on this platform" UserWarning torch
# prints on this project's setup) -- it's silently a no-op there, so it's
# only included on non-Windows to avoid printing a warning for a flag
# that isn't doing anything. garbage_collection_threshold works on both.
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

        if not torch.cuda.is_available():
            # BUG FIX: this callback previously assumed CUDA is always
            # present and called torch.cuda.* unconditionally. --device
            # cpu is an explicitly documented, supported option (for
            # sanity-checking the script runs -- see --device's --help
            # text), and on a CPU-only run every torch.cuda.* call here
            # either raises or is meaningless. Skip cleanly instead of
            # crashing a CPU run with an unrelated CUDA error, and say
            # so once so it's clear this run has no VRAM safety net.
            if not state["checked"]:
                state["checked"] = True
                print("[startup probe] CUDA not available (--device cpu?) "
                      "-- VRAM probe skipped. This run has no WDDM "
                      "shared-memory safety net; fine for a sanity check, "
                      "not recommended for a real training run.")
            return

        if state["checked"]:
            return

        now = time.time()
        state["count"] += 1

        if state["count"] == 1:
            # FIX (see "KNOWN FALSE-POSITIVE, FIXED" in the module
            # docstring): torch.cuda.max_memory_allocated() is a
            # cumulative high-water mark for the WHOLE PROCESS, not
            # "what the current batch used." --batch -1 (AutoBatch)
            # deliberately profiles forward+backward at several
            # candidate batch sizes BEFORE real training starts,
            # intentionally probing past the card's physical ceiling to
            # find where it breaks (a real, self-handled OOM -- see the
            # "AutoBatch: Using batch-size N" line printed just before
            # training begins). Without this reset, that exploratory
            # peak was still sitting in the counter a few real batches
            # later and got misread as something the real training
            # loop did, tripping a false VRAMBudgetExceeded on a run
            # that was actually healthy (confirmed against Ultralytics'
            # own progress bar showing ~0.5GB GPU_mem on the exact
            # batch this previously aborted on). Resetting here -- the
            # first time this callback fires, i.e. right after
            # AutoBatch has already finished and real training has
            # begun -- makes every peak reading from this point on
            # reflect only real training, which is what both this
            # probe and the ongoing watchdog below are actually meant
            # to monitor. Runs with an explicit --batch (no AutoBatch
            # probing) were never affected by the bug, and are
            # unaffected by this reset too -- there's nothing stale to
            # clear for them.
            torch.cuda.reset_peak_memory_stats(device)

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

    Reads the same process-wide counter make_startup_probe_callback()
    resets on its first firing -- since that reset happens before this
    callback's first on_train_epoch_end (epoch end always comes after
    at least one batch end), this watchdog's "cumulative peak since
    training really started" tracking is correct without needing its
    own reset: it inherits the already-cleared baseline.
    """
    state = {"last_epoch_start": None}

    def callback(trainer):
        import torch

        now = time.time()
        epoch_seconds = (now - state["last_epoch_start"]
                          if state["last_epoch_start"] else None)
        state["last_epoch_start"] = now  # reset for the next epoch

        if not torch.cuda.is_available():
            # Same CPU-only guard as the startup probe above -- still
            # honor --max-epoch-minutes (that check has nothing to do
            # with CUDA), just skip the VRAM half of the check.
            if (max_epoch_minutes is not None and epoch_seconds is not None
                    and epoch_seconds > max_epoch_minutes * 60):
                raise VRAMBudgetExceeded(
                    f"[ongoing watchdog, epoch "
                    f"{getattr(trainer, 'epoch', '?')}] epoch took "
                    f"{epoch_seconds / 60:.1f} min, over the "
                    f"{max_epoch_minutes} min budget")
            return

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


def _try_build_transform(name, *builders):
    """
    Tries each candidate builder in order (most-current API shape
    first, falling back to older shapes) and returns the first one that
    builds with zero warnings.

    BUG FIX -- this used to take a single builder and only catch hard
    exceptions (a TypeError from a renamed/removed parameter). That's
    not the only failure mode: some albumentations parameter-name
    mismatches are silently ACCEPTED and ignored, reported only via a
    UserWarning, not an exception. CONFIRMED on this project's actual
    run: build_optical_distortion()'s alb_major>=2 branch assumed
    distort_range was the correct kwarg for "any albumentations 2.x",
    but the installed 2.0.8 rejected it with only a UserWarning
    ("Argument(s) 'distort_range' are not valid for transform
    OpticalDistortion") -- the old single-builder version of this
    function returned a real, successfully-constructed object anyway,
    silently running OpticalDistortion at the LIBRARY'S OWN DEFAULT
    distortion strength instead of this project's deliberately small
    +/-0.03 one. That's exactly the label-corruption risk the module
    docstring warns about for this transform (image-only, no
    bbox_params, so a stronger warp shifts pixels while label boxes
    stay put) -- and it was happening every run without ever being
    reported as a skip.

    Passing multiple builders and treating ANY warning as a build
    failure (via warnings.filterwarnings("error")) means whichever
    variant the installed version actually accepts cleanly wins, rather
    than depending on alb_major guessing right for every point release.
    Returns None (with a printed message) if every builder fails.
    """
    import warnings
    for i, builder in enumerate(builders):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings("error", category=UserWarning)
                result = builder()
            if i > 0:
                print(f"[degradation aug] '{name}': installed "
                      f"albumentations version needed fallback parameter "
                      f"set #{i + 1} of {len(builders)} (newer-API name(s) "
                      f"were rejected).")
            return result
        except Exception:
            continue
    print(f"[degradation aug] Skipping '{name}' -- none of {len(builders)} "
          f"candidate parameter set(s) were accepted cleanly by the "
          f"installed albumentations version. Every other degradation "
          f"transform is unaffected.")
    return None


def build_degradation_transform():
    """
    Builds an Albumentations pipeline calibrated against the E5-FPV
    camera's actual datasheet (1500TVL CVBS analog, NTSC/PAL, 0.0001 lux
    minimum illumination, 6mm lens / 100 degree FOV) rather than a
    generic noise guess -- see the module docstring for what each
    transform maps to and why. Returns None (with a printed warning) if
    albumentations isn't installed, or if every individual transform
    fails to build against the installed version -- both cases are
    non-fatal, training proceeds without degradation augmentation rather
    than crashing.

    VERSION-AWARE, not just version-TOLERANT: five of these transforms
    (Downscale, GaussNoise, CoarseDropout, ImageCompression,
    OpticalDistortion) had their parameter names changed between
    albumentations 1.x and 2.x -- confirmed directly against an
    installed 2.0.8 (and against albumentations' own 2.x API reference
    for OpticalDistortion specifically, whose shift_limit was dropped
    entirely and distort_limit renamed to distort_range): the OLD 1.x
    kwarg names don't raise an error under 2.x, they get silently
    accepted and IGNORED, falling back to the library's own defaults.
    That's worse than a crash for one transform specifically --
    CoarseDropout's 2.x default hole size is 10-20% of the image per
    side, which is large enough to fully blank out a tiny labeled
    person, exactly the label-corruption risk this transform was
    deliberately kept small to avoid (see module docstring). So this
    isn't just wrapped in try/except: the correct kwarg names for the
    detected major version are used directly, with try/except kept
    underneath as a second-line defense for any future API shape
    neither branch anticipates.
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
        # var_limit (1.x) is raw pixel-value variance (0-255 scale);
        # std_range (2.x) is std as a fraction of the 0-1 normalized
        # range. (10.0, 60.0) var -> std ~ sqrt(var)/255 ~ (0.012, 0.030)
        # -- widened slightly below for comparable visible effect.
        # Tries the 2.x name first, falls back to 1.x -- same
        # warning-as-error safety net as _try_build_transform (see its
        # docstring for why a silent-accept-and-ignore parameter name
        # is a real failure mode here, confirmed on OpticalDistortion),
        # applied locally since GaussNoise is nested inside another
        # transform's OneOf rather than registered as its own
        # top-level piece.
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
        raise ValueError("no GaussNoise parameter set was accepted cleanly")

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

    # Deliberately weak -- see module docstring: this is one of the two
    # transforms here (along with OpticalDistortion below) that can
    # corrupt a label (not just add noise to it) if it isn't kept
    # small, since this pipeline runs image-only with no bbox_params.
    # Small/rare on purpose, and (critically) the exact pixel sizes
    # below are confirmed to mean literal pixels under 2.x when given
    # as plain ints, not a fraction of image size -- verified directly
    # (5 holes of (4,4)/(10,10) produced exactly 200 zeroed pixels =
    # 5*4*10) before relying on it here.
    dropout = _try_build_transform(
        "RF dropout burst (deliberately small/rare, see docstring)",
        lambda: A.CoarseDropout(num_holes_range=(1, 3),
                                  hole_height_range=(4, 4),
                                  hole_width_range=(10, 10),
                                  fill=0, p=0.08),
        lambda: A.CoarseDropout(max_holes=3, max_height=4, max_width=10,
                                  fill_value=0, p=0.08))
    if dropout is not None:
        pieces.append(dropout)

    # Barrel distortion for the E5-FPV's 6mm/100-degree-FOV lens -- see
    # the "OpticalDistortion (lens barrel distortion, NEW)" bullet in
    # the module docstring for the full reasoning, including why this
    # is kept deliberately weak (same image-only/no-bbox_params
    # corruption risk as CoarseDropout above, but across the whole
    # frame instead of a small hole). distort_range/distort_limit
    # +/-0.03 is roughly a third of albumentations' own 0.05 default --
    # enough to bow lines slightly toward the frame edge, not enough to
    # meaningfully displace a tight box anywhere but the extreme edge.
    #
    # BUG FIX -- CONFIRMED on this project's actual run log: the
    # alb_major>=2 branch below (distort_range, mode="camera") was
    # rejected by the installed albumentations 2.0.8 with a UserWarning
    # ("Argument(s) 'distort_range' are not valid for transform
    # OpticalDistortion"), NOT an exception -- so the old code silently
    # got back a working OpticalDistortion object that had actually
    # fallen through to the library's own default distortion strength
    # instead of this project's deliberately small +/-0.03 one, on
    # every single run, with the log claiming "Built 9/9 planned
    # transforms with version-correct parameters" the whole time.
    # _try_build_transform now tries both parameter sets and only
    # accepts whichever one the installed version builds without any
    # warning -- see its docstring.
    optical_distortion = _try_build_transform(
        "lens barrel distortion (6mm/100deg FOV, deliberately weak, "
        "see docstring)",
        lambda: A.OpticalDistortion(distort_range=(-0.03, 0.03),
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

    print(f"[degradation aug] Built {len(pieces)}/9 planned transforms "
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
    CoarseDropout, OpticalDistortion, and Lambda as "spatial" transforms,
    since Albumentations CAN make them bbox-aware given bbox_params).
    This pipeline's own A.Compose(pieces) call deliberately has no
    bbox_params set, so it MUST be called image-only
    (contains_spatial=False) or the call would break -- this is
    intentional, not an oversight, but it does mean CoarseDropout/
    OpticalDistortion/Lambda run here without any bbox awareness at all,
    relying entirely on small hole size / small distortion magnitude to
    avoid label corruption rather than Albumentations' own
    visibility-based box filtering or geometric box remapping. See the
    CoarseDropout and OpticalDistortion bullets in the module docstring
    for the more rigorous (not implemented) alternative.

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
          "shift, lens barrel distortion) -- closes some of the "
          "train/deploy domain gap against the real FPV analog feed.")


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


def _device_arg_type(value: str):
    """
    BUG FIX: --device's default is the int 0, but argparse gives you a
    bare STRING when the flag is actually typed on the command line (no
    type= was previously set). torch.cuda.get_device_properties()/
    torch.device() accept an int (0), 'cpu', or a full 'cuda:0'-style
    string -- but NOT a bare numeric string like '0'
    (torch.device('0') raises "Invalid device string"). So the default
    (never touches this function) worked fine, but anyone who explicitly
    passed --device 0 on the CLI -- the single most common thing to
    type -- would crash the startup probe/watchdog with an unrelated-
    looking torch device-parsing error. 'cpu' and already-qualified
    strings ('cuda:0', '0,1') are passed through unchanged; anything
    that parses as a bare int is converted so it matches the untouched
    default's type.
    """
    if value.lower() == "cpu":
        return value
    try:
        return int(value)
    except ValueError:
        return value  # e.g. "cuda:0", "0,1" -- already unambiguous to torch


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
                    help="960 is the RECOMMENDED default for this project "
                         "-- this project's own baseline history shows "
                         "640px plateaus with small/rare classes missed "
                         "outright, while the same model at 960px fixed "
                         "that ceiling and every class improved (see the "
                         "IMAGE SIZE note in the module docstring). Note "
                         "imgsz does NOT need to match any source "
                         "dataset's native resolution -- Ultralytics "
                         "letterboxes every image (VisDrone/UAVDT/SARD "
                         "alike) to this single square size regardless. "
                         "960px HAS shown WDDM spillover in the past on "
                         "this card (see WDDM note) but mosaic_guard.py's "
                         "instance cap has since confirmed real VRAM "
                         "headroom (~2GB/6GB) at this size -- fall back "
                         "to 640 only if the startup probe/watchdog still "
                         "trip for you.")
    p.add_argument("--batch", type=int, default=1,
                    help="Explicit batch size. batch=1 has zero batching "
                         "efficiency and is a major speed tax by itself. "
                         "UPDATE: with mosaic_guard.py's instance cap "
                         "installed, a real run is now staying around "
                         "~2GB/6GB VRAM -- there is likely significant "
                         "headroom to raise this. Test via a short "
                         "--epochs 3-5 run before committing to a full "
                         "run: watch nvidia-smi / the printed VRAM probe "
                         "messages, and increase --batch until either "
                         "gets close to the 6GB ceiling. Pass -1 for "
                         "AutoBatch -- was unreliable at 960px in earlier "
                         "testing (pre-mosaic_guard), may be worth "
                         "retrying now given the lower actual usage. NOTE: "
                         "AutoBatch's own pre-training profiling pass "
                         "(the 'GPU_mem (GB)' table it prints, batch "
                         "sizes 1/2/4/8...) deliberately probes past the "
                         "card's physical VRAM to find the ceiling -- a "
                         "printed 'AutoBatch: Using batch-size N' line "
                         "means that already resolved cleanly on its own; "
                         "it is not a sign anything is wrong.")
    p.add_argument("--device", default=0, type=_device_arg_type,
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
                         "gamma/lens distortion). On by default -- see "
                         "module docstring. Requires albumentations; "
                         "auto-skips with a warning if it isn't "
                         "installed.")
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


def report_per_class_map(model: YOLO, data_path: str, batch: int,
                           workers: int = 0) -> None:
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

    FIX (OOM on 6GB card): model.val() defaults to Ultralytics' own
    validation batch size when none is given -- much larger than the
    --batch 1 this project actually needs on this card. batch is now
    passed through explicitly, and the CUDA cache is cleared right
    before the call so memory left reserved by training (or by a prior
    val() call, e.g. evaluate_per_source() running just before this)
    can't stack on top of it.

    FIX (real crash, CONFIRMED from an actual run log -- "paging file
    too small" OSError loading cublas64_13.dll, right after training
    finished and its own automatic post-train validation had already
    succeeded cleanly): unlike model.train() -- which this project
    always calls with workers=args.workers (0 by default, specifically
    because Windows reloads the ENTIRE CUDA DLL stack per spawned
    worker process, see --workers's --help text) -- model.val() was
    being called here with no workers= argument at all, so it fell back
    to Ultralytics' own (nonzero) validation default. On Windows that
    spawns a real subprocess via multiprocessing.spawn, which
    re-imports this whole script from scratch (import torch -> reload
    the full CUDA DLL stack a second time, while the parent process
    still has its own copy resident) -- exactly the failure mode
    --workers=0 exists to avoid for training, just not previously
    applied to eval too. workers now defaults to 0 to match, and is
    threaded through from args.workers by the caller in main() so a
    deliberately-raised --workers value (once the paging file is
    resized, per that flag's own help text) applies consistently to
    both training and eval instead of only half of the pipeline.
    """
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
          "-- not a sign the model or pipeline is broken.")


def evaluate_per_source(model: YOLO, run_dir: Path, batch: int,
                          workers: int = 0) -> None:
    """
    Runs a SEPARATE model.val() pass against each source dataset's own
    val set (VisDrone/UAVDT/SARD individually), reading
    datasets/per_source_val.json (written by prepare_datasets.py). This
    is the only way to tell whether combining these three datasets
    actually helped -- the single blended unified.yaml val number can
    hide one source regressing while another improves. Silently skipped
    if the manifest doesn't exist (e.g. --data was set manually to
    bypass prepare_datasets.py).

    Now also prints a per-class breakdown for each source. Historically
    (pre generate_pseudo_labels.py), a source's per-class numbers were
    only meaningful for classes that source natively annotated -- UAVDT
    had no person labels, SARD had no vehicle labels -- with the other
    rows reading as 0/undefined and safe to ignore.

    UPDATE -- this is no longer unconditionally true. generate_pseudo_
    labels.py cross-references the other three detection models against
    exactly the classes a source didn't originally label, and --apply
    merges those pseudo-labels into the real dataset for any image
    that's been FULLY reviewed (see that script's V8/V9 notes on
    pending_review_images.json -- still-pending images are excluded from
    every train/val/test list entirely, not partially merged). So a
    source's own val set drawn here CAN now genuinely contain ground
    truth for a class it didn't originally annotate: a UAVDT val image
    may have a real person box, a SARD val image may have a real vehicle
    box. A row reading 0.000 (or very low) is therefore no longer proof
    that class is structurally absent from that source -- it may simply
    mean few/none of that source's images carrying that class have
    cleared review and been merged yet. Cross-check against
    datasets/pending_review_images.json and pseudo_labels_summary.json's
    per-class auto/queued/discarded counts before drawing a conclusion
    from a near-zero row here.

    FIX (real crash, CONFIRMED from an actual run log -- "paging file
    too small" OSError loading cublas64_13.dll, striking on the FIRST
    source (VisDrone) right after training finished and its own
    automatic post-train validation had already succeeded cleanly):
    model.val() here was being called with no workers= argument, unlike
    every model.train() call in this project, which always passes
    workers=args.workers (0 by default) specifically because Windows
    reloads the entire CUDA DLL stack per spawned dataloader worker
    process (see --workers's --help text). Without an explicit
    workers=0 here, model.val() fell back to Ultralytics' own nonzero
    validation default, which spawns a real subprocess via
    multiprocessing.spawn on Windows -- that subprocess re-imports this
    whole script from scratch (import torch -> reload the full CUDA DLL
    stack a SECOND time, while the parent process still has its own
    copy resident), and that second load is what actually exhausted the
    paging file. This is a different resource than the GPU-VRAM OOM the
    gc.collect()/empty_cache() calls below guard against -- those two
    fixes address separate failure modes that happened to surface in
    the same function, not the same bug twice. workers now defaults to
    0 to match training, and is threaded through from args.workers by
    the caller in main().
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

    import gc
    import torch

    tmp_dir = run_dir / "per_source_eval"
    tmp_dir.mkdir(exist_ok=True)

    for name, val_dirs in per_source.items():
        # FIX (OOM on 6GB card, confirmed root cause of the UAVDT
        # crash): clear the caching allocator BEFORE each source's
        # val() call. Without this, memory reserved by the previous
        # source's val() (VisDrone ran fine here) -- or by training
        # itself -- can still be held when the next call starts, even
        # though it's logically done being used. By the time UAVDT's
        # turn came up the card had "free: 0, total: 6441926656" left;
        # an allocation failure that severe can crash the CUDA context
        # hard enough that the `except Exception` below never gets a
        # clean shot at it, which is why it died silently instead of
        # printing "[skipped: ...]".
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

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
            # FIX: batch=batch -- without this, model.val() fell back to
            # Ultralytics' own (much larger) default validation batch
            # size instead of the --batch 1 this project actually needs.
            # FIX: workers=workers -- see the module-level fix note in
            # this function's docstring; this is what stops Windows from
            # spawning a paging-file-exhausting subprocess here.
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
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

    print(f"{'=' * 70}")
    print("Note: each source's ORIGINAL native classes are limited (UAVDT "
          "had no person labels, SARD had no vehicle labels) -- but "
          "generate_pseudo_labels.py --apply merges cross-source pseudo-"
          "labels into any image that has been FULLY reviewed, so a "
          "source can now genuinely carry boxes for a class it didn't "
          "originally annotate. A 0.000 (or near-zero) row above may mean "
          "'not enough of this source has been reviewed/merged for this "
          "class yet', not 'this source can never have this class' -- "
          "check datasets/pending_review_images.json and pseudo_labels_"
          "summary.json's per-class counts before assuming the latter.")


def train_with_model_fallback(args, data_path: str):
    """
    Tries each candidate in MODEL_SIZE_CANDIDATES (or just args.model if
    it's an explicit choice, not 'auto') until one survives both the
    startup probe AND the ongoing watchdog without tripping
    VRAMBudgetExceeded, and without a real CUDA OOM either.
    """
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
              "Ultralytics reported this deprecated -- it may have no "
              "effect. Passing it through anyway.")
        train_kwargs["label_smoothing"] = args.label_smoothing

    for i, model_name in enumerate(candidates):
        this_run_name = (args.name or default_run_name(model_name, args.imgsz))
        print(f"\n{'=' * 70}\nAttempting: {model_name} @ {args.imgsz}px, "
              f"batch={args.batch}\n{'=' * 70}")
        model = None
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
            # BUG FIX: empty_cache() used to run here while `model` (this
            # candidate's weights, optimizer state, EMA shadow copy, and
            # dataloader) was STILL referenced by the local variable --
            # empty_cache() only releases blocks the allocator considers
            # idle, and none of this candidate's memory was idle yet,
            # it was just about to become unreachable on the next loop
            # iteration's reassignment. That meant the next (smaller)
            # candidate was loading while the failed one's memory was
            # still fully resident -- exactly the kind of accumulation
            # this fallback path exists to avoid. Explicitly drop the
            # reference and run a real gc.collect() first so PyTorch's
            # allocator has something to actually give back before the
            # next candidate asks for VRAM. `model = None` above (before
            # the try) guards the case where YOLO(model_name) itself is
            # what raised, so there's nothing yet to delete.
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
        # MINOR FIX: this check used to run AFTER installing the
        # training-only monkeypatches below, so a pure --export-only
        # invocation (which never calls model.train() at all) still
        # patched Ultralytics' Mosaic/Albumentations internals and
        # printed their banners for no reason. Export doesn't touch
        # either, so there's nothing to gain from installing them here.
        weights = Path(args.weights) if args.weights else find_latest_best_pt()
        export_for_cpp(weights)
        return

    install_instance_cap(max_instances=args.max_mosaic_instances,
                          max_tries=args.mosaic_max_tries)

    if args.degradation_aug:
        install_degradation_augment()

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

        # BUG FIX: this branch used to call model.train(resume=True)
        # with no callbacks attached at all -- train_with_model_fallback()
        # (the fresh-run path, below) adds the startup probe, the
        # ongoing watchdog, and the augmentation-scope check, but this
        # resume path skipped all three. That meant every resumed run
        # had NO WDDM safety net whatsoever -- exactly the failure mode
        # this file's whole WDDM section exists to catch, and resumed
        # runs are not less likely to hit it (they pick up mid-training,
        # right where the staircase/mosaic-density issue was already
        # underway). install_degradation_augment()/install_instance_cap()
        # in main() are global monkeypatches so they already applied
        # here regardless -- it was specifically the per-Trainer
        # add_callback() calls that were missing.
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

        try:
            model.train(resume=True)
        except VRAMBudgetExceeded as e:
            # Unlike the fresh-run path, there's no smaller model size to
            # fall back to here -- the checkpoint's architecture is
            # fixed. Fail loudly with a concrete next step instead of
            # silently falling back to Ultralytics' own defaults (see
            # _is_resumable_checkpoint's docstring for why a silent
            # fallback is specifically dangerous on this project).
            raise SystemExit(
                f"\n[VRAM] Resumed run at {last_pt} hit the VRAM budget "
                f"check ({e}). Resuming can't fall back to a smaller "
                f"model size -- the checkpoint's architecture is fixed. "
                f"Free up VRAM, or start a fresh (non-resume) run with a "
                f"smaller --imgsz/--batch or --model, using this "
                f"checkpoint as a warm start (see the resumability error "
                f"message above for that pattern).")

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

    # BUG FIX: this is the piece the earlier "reload fresh for eval" fix
    # was missing. `model` (with its full .trainer -- optimizer state,
    # EMA shadow copy, dataloader buffers, often larger than the model
    # weights themselves) was never released here -- it stayed
    # referenced by this variable for the REST of main(), straight
    # through eval and export. So `eval_model = YOLO(str(best_pt))`
    # below wasn't actually giving eval a clean VRAM footprint; it was
    # adding a second model's memory on top of the first one, which was
    # still fully resident the whole time. Only `best_pt`'s path is
    # needed from here on, so free everything else now.
    import gc
    import torch
    del model
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    check_overfitting(run_dir)

    if not args.skip_per_source_eval:
        eval_model = YOLO(str(best_pt))
        evaluate_per_source(eval_model, run_dir, args.batch,
                             workers=args.workers)
        if data_path is not None:
            report_per_class_map(eval_model, data_path, args.batch,
                                  workers=args.workers)
        else:
            print("\n[per-class report] Skipped -- no data_path available "
                  "for a --resume run in this process. Run "
                  "`python train.py --export-only` style follow-up, or "
                  "call report_per_class_map(model, 'datasets/unified.yaml', "
                  "args.batch) manually if you want it after a resumed run.")

        # Same reasoning as above: eval_model isn't needed once eval is
        # done, and export_for_cpp() loads its own fresh model from
        # best_pt -- free eval_model first so export doesn't have to
        # compete with it.
        del eval_model
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    export_for_cpp(best_pt)


if __name__ == "__main__":
    main()