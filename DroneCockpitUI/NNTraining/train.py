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
    4. --mosaic (new, default 1.0, Ultralytics' own default) lets you
       lower the composite probability (e.g. 0.5-0.7) if the watchdog
       keeps tripping on dense composited batches -- trades away some
       augmentation strength for fewer extreme-instance-count batches.
       Leave at default unless probes 1/2 are firing repeatedly.
    5. mosaic_guard.py / --max-mosaic-instances (new, default 800) --
       PROACTIVE version of the fix, not just reactive monitoring.
       Patches Ultralytics' Mosaic to steer partner-image selection away
       from combinations that would exceed this instance budget, instead
       of picking partners uniformly at random. Does NOT reduce the
       mosaic grid size, and can't fully protect against a single source
       image that's dense enough on its own -- probes 1/2 stay in place
       as the reactive backstop underneath this.
  Given it recurred even on yolo26s at 960px with the combined dataset:
  recommend trying --imgsz 640 for the real run (see Usage below) --
  more VRAM headroom AND it should let --batch go above 1, which batch=1
  itself is a major speed tax (no batching efficiency at all).

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
  - --save-period (default 5, new) additionally asks Ultralytics to also
    write out numbered snapshots (epoch5.pt, epoch10.pt, ...) instead of
    only ever having the single latest last.pt -- extra insurance if you
    ever want to roll back further than one epoch, or in case last.pt
    itself gets corrupted by an interruption mid-write. Set to -1 to
    disable and save disk space.
  - --max-epoch-minutes (new, optional) is a second failsafe: if any one
    epoch takes longer than this, the ongoing watchdog raises and this
    model size is abandoned in favor of a smaller one, rather than
    silently eating an entire time budget on a run that's thrashing.

Overfitting / "fast but low error" controls, given the dataset is still
fairly small/imbalanced and time is limited:
  - --patience (default 15): early stopping on val mAP. This is the main
    overfitting guard already -- best.pt is always the best-val epoch,
    not whatever epoch training happens to stop on.
  - --cos-lr (default on): cosine LR decay, tends to generalize slightly
    better late in training than linear.
  - --label-smoothing: kept for forward-compat only (reported deprecated/
    no-op on the installed Ultralytics version) -- only forwarded if you
    explicitly set it non-zero, with a one-time printed reminder.
  - copy_paste/mixup (already present): keep these on -- they matter more
    than usual here since the sparser classes (motorcycle/other_vehicle)
    and SARD's SAR-specific poses have the least real data to learn from.
  - check_overfitting() after every run reads results.csv and flags it if
    val loss is rising while train loss keeps falling over the last N
    epochs -- a heuristic, not a verdict.
  - evaluate_per_source() (new) additionally reports mAP separately for
    VisDrone/UAVDT/SARD's own val sets after training, not just the
    blended unified.yaml number -- see its docstring for why this
    matters: a blended average can hide one source regressing while
    another improves.
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
    python train.py --imgsz 640 --batch 4   # recommended starting point
                                        # given past WDDM spillover -- see
                                        # note above
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
PER_SOURCE_MANIFEST_PATH = SCRIPT_DIR / "datasets" / "per_source_val.json"

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
                         "Helps the sparser classes stay detectable "
                         "without needing new source data. Set to 0.0 to "
                         "disable.")
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
    p.add_argument("--patience", type=int, default=15,
                    help="Early stopping: stop if val mAP hasn't improved "
                         "in this many epochs. best.pt is always kept "
                         "from the best-val epoch regardless.")
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
                         "to '<model>_<imgsz>'.")
    p.add_argument("--export-only", action="store_true",
                    help="Skip training, just export an existing "
                         "runs/detect/<name>/weights/best.pt.")
    p.add_argument("--weights", default=None,
                    help="Checkpoint to export when --export-only is set. "
                         "Defaults to the most recently modified "
                         "runs/detect/*/weights/best.pt found.")
    p.add_argument("--skip-per-source-eval", action="store_true",
                    help="Skip the post-training per-source validation "
                         "pass (VisDrone/UAVDT/SARD evaluated separately). "
                         "On by default since it's a handful of extra "
                         "quick val() passes -- disable only if you're "
                         "iterating fast and don't need it every run.")
    return p.parse_args()


def default_run_name(model: str, imgsz: int) -> str:
    stem = Path(model).stem  # "yolo26s.pt" -> "yolo26s"
    return f"{stem}_{imgsz}"


def find_latest_last_pt() -> Path:
    candidates = sorted(RUNS_PROJECT.glob("*/weights/last.pt"),
                         key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(
            f"No {RUNS_PROJECT}/*/weights/last.pt found to resume from. "
            f"Run training without --resume first.")
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
            model.train(
                data=data_path,
                project=str(RUNS_PROJECT),
                name=this_run_name,
                **train_kwargs,
            )
            return model, this_run_name
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

    if args.export_only:
        weights = Path(args.weights) if args.weights else find_latest_best_pt()
        export_for_cpp(weights)
        return

    if args.resume:
        last_pt = find_latest_last_pt()
        print(f"\nResuming training from {last_pt}...")
        model = YOLO(str(last_pt))
        model.train(resume=True)
        run_name = last_pt.parent.parent.name
    else:
        data_path = args.data or str(prepare_datasets.main())
        model, run_name = train_with_model_fallback(args, data_path)

    best_pt = Path(model.trainer.best)
    run_dir = best_pt.parent.parent
    print(f"\nTraining complete. Run: {run_name}")
    print(f"Best checkpoint: {best_pt}")

    check_overfitting(run_dir)

    if not args.skip_per_source_eval:
        evaluate_per_source(model, run_dir)

    export_for_cpp(best_pt)


if __name__ == "__main__":
    main()