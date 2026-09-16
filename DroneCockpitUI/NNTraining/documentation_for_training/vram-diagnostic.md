# `vram_diagnostic.py` — root-causing a suspected VRAM leak

**File:** `vram_diagnostic.py`
**Companion to:** `train.py`
**Run:** `python vram_diagnostic.py --model yolo26s.pt --imgsz 640 --batch 4 --num-batches 3000`

## Responsibility

Isolates whether a suspected WDDM VRAM spillover is a genuine **memory
leak** (allocated memory climbs without bound) versus a **fixed-size
workload** simply needing more VRAM than available — without running a
full, hours-long training job to find out.

## Why this script exists (the evidence that pointed at a leak)

Two observations pointed away from a simple sizing problem:

1. A Task Manager screenshot showed **Dedicated GPU memory** climbing in a
   continuous staircase over several minutes before **Shared memory**
   started climbing on top of it — a fixed sizing problem plateaus quickly
   at steady-state; it doesn't keep climbing for minutes.
2. An earlier version of this script's own 8-batch probe (using
   Ultralytics' bare `copy_paste=0.0`/`mixup=0.0` defaults, since those
   weren't passed) sat flat at ~1.8–2.0GB the whole time — consistent with
   8 batches simply being too few to see a slow leak accumulate, and/or
   not exercising the same augmentation path as the real training run
   (which uses `copy_paste=0.3`, `mixup=0.1`).

## What changed to actually catch it

- `--copy-paste`/`--mixup` now **default to `train.py`'s real values**
  (0.3/0.1) instead of Ultralytics' bare defaults, so this genuinely
  exercises the same code path as the real training run.
- `--num-batches` default raised to **3000** — the project's own full run
  showed spillover developing around batch ~2800, so a short probe would
  miss it entirely.
- Per-batch memory readings are written to a CSV (`--csv-out`) every
  batch, with only a one-line console update every `--log-every` batches
  — per-batch console printing would be unreadable at this scale.
- `check_for_leak()` at the end compares average allocated memory in the
  **first 10%** of logged batches against the **last 10%**. A fixed-size
  workload should show these roughly equal (some noise from
  variable-content batches); a genuine leak shows a clear, one-directional
  upward difference. Framed explicitly as a heuristic pointing at the
  CSV/snapshot for confirmation, not a final verdict — the threshold
  (`delta > 0.15` GB) is deliberately loose.

## CLI

```bash
# Long run matching real training settings -- the one that should
# actually reproduce the climb:
python vram_diagnostic.py --model yolo26s.pt --imgsz 640 --batch 4 \
    --num-batches 3000

# Isolation test: does the climb disappear with augmentation off?
# If yes, that points the finger at copy_paste/mixup specifically.
python vram_diagnostic.py --model yolo26s.pt --imgsz 640 --batch 4 \
    --num-batches 3000 --copy-paste 0.0 --mixup 0.0
```

| Flag | Default | Notes |
|---|---|---|
| `--model` | `yolo26s.pt` | |
| `--data` | `datasets/unified.yaml` via `prepare_datasets.py`, same as `train.py` | |
| `--imgsz` / `--batch` / `--device` | 640 / 4 / 0 | |
| `--num-batches` | 3000 | |
| `--log-every` | 50 | Console-only throttle; every batch still hits the CSV |
| `--cache` | `disk` | Match whatever `--cache` the real training run uses |
| `--copy-paste` / `--mixup` | 0.3 / 0.1 | Match `train.py`'s real defaults, not Ultralytics' bare ones |
| `--record-history` | off | Enables torch's memory-history recorder, dumping a `.pickle` snapshot readable at https://pytorch.org/memory_viz for exact call stacks. **Off by default for long runs** — the recorder's own bookkeeping grows with `--num-batches` and can itself skew the numbers being measured; turn it on only for short, targeted runs |
| `--out` | `vram_snapshot.pickle` | Only used with `--record-history` |
| `--csv-out` | `vram_trace.csv` | Per-batch trace |

## Reading the output

- **Allocated** = memory actually holding live tensors right now.
- **Reserved** = memory the CUDA caching allocator is holding onto (can be
  more than Allocated — that gap is fragmentation, not genuine usage).
- If Allocated alone is already close to the physical VRAM figure printed
  at startup **and stays flat over time**, the model+batch genuinely needs
  that much — the fix is a smaller `--batch`/`--imgsz`, not a bug hunt.
- If Allocated keeps climbing per `check_for_leak()`'s verdict, that's a
  leak, not a sizing issue — try the `--copy-paste 0 --mixup 0` isolation
  run, and consider `pip install -U ultralytics` since memory-growth bugs
  do get fixed in point releases.
- If `check_for_leak()` finds no clear signature but Task Manager still
  showed climbing Dedicated/Shared memory during the same run, the leak
  may be in reserved-but-unallocated (fragmentation) memory instead —
  check the `reserved_gb` column in the CSV for the same trend.

Implementation note: the probe runs via an Ultralytics `on_train_batch_end`
callback that raises an internal `_StopProbe` exception once
`--num-batches` is reached, cutting off a `model.train(epochs=1000, ...)`
call early — effectively unbounded epochs, cut short deterministically at
an exact batch count rather than an epoch boundary.
