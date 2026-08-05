"""
vram_diagnostic.py — Root-cause a suspected WDDM VRAM spillover / leak
=======================================================================
Companion to train.py. Two things pointed away from a simple "this batch
doesn't fit" sizing problem and toward a genuine memory LEAK:
  1. The Task Manager screenshot showed Dedicated GPU memory climbing in
     a continuous staircase over several minutes before Shared memory
     started climbing on top of it -- a fixed sizing problem plateaus
     quickly at steady-state, it doesn't keep climbing for minutes.
  2. The first version of this script's own 8-batch probe (default
     copy_paste=0.0, mixup=0.0, since those weren't passed) sat flat at
     ~1.8-2.0GB the whole time -- consistent with 8 batches simply being
     too few to see a slow leak accumulate, and/or not exercising the
     same augmentation path as the real run (copy_paste=0.3, mixup=0.1).

What changed in this version:
  - --copy-paste/--mixup now default to train.py's real values (0.3/0.1)
    instead of Ultralytics' bare defaults (0.0/0.0), so this actually
    exercises the same code path as your real training run.
  - --num-batches default raised to 3000 (your earlier full run showed
    spillover developing around batch ~2800) -- long enough to actually
    watch a slow leak happen instead of missing it in an 8-batch window.
  - Per-batch console printing would be unreadable at this scale, so
    memory readings are now written to a CSV (--csv-out) every batch,
    with only a one-line console update every --log-every batches.
  - check_for_leak() at the end compares average allocated memory in the
    first 10% of batches vs the last 10%. A fixed-size workload should
    show these roughly equal (some noise from variable-content batches);
    a leak shows a clear upward difference. This is a heuristic pointing
    you at the CSV/snapshot for confirmation, not a final verdict.

Usage:
    # Long run matching your real training settings -- this is the one
    # that should actually reproduce the climb:
    python vram_diagnostic.py --model yolo26s.pt --imgsz 640 --batch 4 \\
        --num-batches 3000

    # Isolation test: does the climb disappear with augmentation off?
    # If yes, that points the finger at copy_paste/mixup specifically.
    python vram_diagnostic.py --model yolo26s.pt --imgsz 640 --batch 4 \\
        --num-batches 3000 --copy-paste 0.0 --mixup 0.0

Then:
  - Check the printed leak verdict and the shape of --csv-out (plot
    'allocated_gb' vs 'batch' in Excel/anything -- a leak shows a rising
    line, a sizing issue shows a flat line with noise).
  - Open the .pickle snapshot at https://pytorch.org/memory_viz for the
    exact call stack behind the largest allocations, if you want to
    pinpoint the specific line of code.
  - NOTE: the memory history recorder (--record-history) keeps its own
    metadata for every allocation across all --num-batches, which is
    itself memory-hungry over thousands of batches -- disabled by
    default for long runs (use it only for short, targeted runs like the
    original --num-batches 8 default) to avoid the profiler itself
    skewing the numbers you're trying to read.
"""

import argparse
import csv
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SNAPSHOT_PATH = SCRIPT_DIR / "vram_snapshot.pickle"
CSV_PATH = SCRIPT_DIR / "vram_trace.csv"


class _StopProbe(Exception):
    """Internal signal to end the run after enough batches are captured."""


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="yolo26s.pt")
    p.add_argument("--data", default=None,
                    help="Defaults to datasets/unified.yaml via "
                         "prepare_datasets.py, same as train.py.")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=4)
    p.add_argument("--device", default=0)
    p.add_argument("--num-batches", type=int, default=3000,
                    help="How many real batches to run. Your full run "
                         "showed spillover developing around batch ~2800 "
                         "-- default raised to 3000 so a leak actually "
                         "has room to show itself. Use a small number "
                         "(e.g. 8) only for a quick sanity check.")
    p.add_argument("--log-every", type=int, default=50,
                    help="Print a console line every N batches (every "
                         "batch is still written to --csv-out regardless "
                         "-- this only controls console spam).")
    p.add_argument("--cache", default="disk",
                    help="Match whatever --cache you actually train with.")
    p.add_argument("--copy-paste", type=float, default=0.3,
                    help="Defaults to train.py's real value (0.3), not "
                         "Ultralytics' bare default (0.0) -- so this "
                         "actually exercises the same augmentation path "
                         "as your real training run. Set to 0.0 as an "
                         "isolation test.")
    p.add_argument("--mixup", type=float, default=0.1,
                    help="Same reasoning as --copy-paste -- defaults to "
                         "train.py's real value, not Ultralytics' bare "
                         "default.")
    p.add_argument("--record-history", action="store_true",
                    help="Enable torch's memory history recorder and dump "
                         "a .pickle snapshot for pytorch.org/memory_viz. "
                         "Off by default for long runs (the recorder's own "
                         "bookkeeping grows with --num-batches and can "
                         "skew the numbers you're trying to read) -- turn "
                         "it on for short, targeted runs instead.")
    p.add_argument("--out", default=str(SNAPSHOT_PATH),
                    help="Where to write the .pickle snapshot (only used "
                         "with --record-history).")
    p.add_argument("--csv-out", default=str(CSV_PATH),
                    help="Where to write the per-batch memory trace CSV.")
    return p.parse_args()


def check_for_leak(csv_path: Path, window_frac: float = 0.10) -> None:
    """
    Heuristic leak check: compares mean 'allocated_gb' in the first and
    last window_frac of logged batches. A meaningfully higher mean at the
    end, with no corresponding drop back down, points at a leak rather
    than batch-to-batch content variance (which would show as noise
    around a flat mean, not a one-directional climb).
    """
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    if n < 20:
        print(f"\n[leak check] Only {n} batches logged, too few for a "
              f"meaningful trend check.")
        return

    window = max(5, int(n * window_frac))
    first = [float(r["allocated_gb"]) for r in rows[:window]]
    last = [float(r["allocated_gb"]) for r in rows[-window:]]
    first_mean = sum(first) / len(first)
    last_mean = sum(last) / len(last)
    delta = last_mean - first_mean

    print(f"\n[leak check] Mean allocated over first {window} batches: "
          f"{first_mean:.3f}GB. Mean over last {window} batches: "
          f"{last_mean:.3f}GB. Delta: {delta:+.3f}GB.")

    # Threshold is deliberately loose -- this is a pointer to investigate
    # further (via the CSV/snapshot), not a certainty.
    if delta > 0.15:
        print("  LIKELY LEAK: allocated memory grew meaningfully over the "
              "run rather than settling into a steady band. A fixed-size "
              "workload (same model/batch/imgsz every step) should not do "
              "this. Plot 'allocated_gb' vs 'batch' from "
              f"{csv_path} to see the actual shape of the climb -- a "
              "straight-ish rising line supports a leak; a step that "
              "rises once then flattens supports something like a larger "
              "batch/image being encountered partway through instead.")
    else:
        print("  No clear leak signature in this window -- allocated "
              "memory stayed roughly flat. If Task Manager still showed "
              "climbing Dedicated/Shared memory during this same run, the "
              "leak may be in reserved-but-unallocated (fragmentation) "
              "memory instead -- check the 'reserved_gb' column in the "
              "CSV for the same trend.")


def main():
    args = parse_args()
    import torch
    from ultralytics import YOLO

    if not torch.cuda.is_available():
        print("No CUDA device visible -- nothing to profile.")
        sys.exit(1)

    device = args.device
    total = torch.cuda.get_device_properties(device).total_memory
    print(f"Physical VRAM on device {device}: {total / 1e9:.2f}GB")

    import prepare_datasets
    data_path = args.data or str(prepare_datasets.main())

    print(f"\nTracing memory over up to {args.num_batches} real batches at "
          f"{args.model} @ {args.imgsz}px, batch={args.batch}, "
          f"copy_paste={args.copy_paste}, mixup={args.mixup}...")
    print(f"Per-batch trace -> {args.csv_out} (console update every "
          f"{args.log_every} batches)\n")

    if args.record_history:
        torch.cuda.memory._record_memory_history(max_entries=200_000)

    csv_file = open(args.csv_out, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["batch", "allocated_gb", "reserved_gb", "peak_gb"])

    batch_count = {"n": 0}

    def probe_callback(trainer):
        batch_count["n"] += 1
        allocated = torch.cuda.memory_allocated(device)
        reserved = torch.cuda.memory_reserved(device)
        peak = torch.cuda.max_memory_allocated(device)

        csv_writer.writerow([
            batch_count["n"],
            f"{allocated / 1e9:.4f}",
            f"{reserved / 1e9:.4f}",
            f"{peak / 1e9:.4f}",
        ])
        if batch_count["n"] % args.log_every == 0:
            csv_file.flush()
            print(f"  batch {batch_count['n']:>5}/{args.num_batches}: "
                  f"allocated={allocated / 1e9:.2f}GB  "
                  f"reserved={reserved / 1e9:.2f}GB  "
                  f"peak={peak / 1e9:.2f}GB  "
                  f"(physical={total / 1e9:.2f}GB)")

        if batch_count["n"] >= args.num_batches:
            raise _StopProbe()

    model = YOLO(args.model)
    model.add_callback("on_train_batch_end", probe_callback)

    try:
        model.train(
            data=data_path,
            epochs=1000,  # effectively unbounded -- _StopProbe cuts it off
            imgsz=args.imgsz,
            batch=args.batch,
            device=device,
            workers=0,
            cache=args.cache,
            amp=True,
            copy_paste=args.copy_paste,
            mixup=args.mixup,
            plots=False,
            val=False,
            project=str(SCRIPT_DIR / "runs" / "diagnostic"),
            name="probe",
            exist_ok=True,
        )
    except _StopProbe:
        pass
    except RuntimeError as e:
        if "out of memory" in str(e).lower():
            print(f"\nHit a REAL CUDA OOM (not silent spillover) after "
                  f"{batch_count['n']} batches: {e}")
        else:
            csv_file.close()
            raise

    csv_file.close()

    if args.record_history:
        out_path = Path(args.out)
        torch.cuda.memory._dump_snapshot(str(out_path))
        torch.cuda.memory._record_memory_history(enabled=None)
        print(f"\nSnapshot written: {out_path}")
        print(f"Open https://pytorch.org/memory_viz and drag this file in "
              f"for exact call stacks behind the largest allocations.")

    print(f"\n{'=' * 70}")
    print(f"Per-batch trace written: {args.csv_out} "
          f"({batch_count['n']} rows)")
    print(f"{'=' * 70}")
    print(torch.cuda.memory_summary(device=device, abbreviated=False))

    check_for_leak(Path(args.csv_out))

    print(
        "\nHow to read the summary above:\n"
        "  - 'Allocated' = memory actually holding live tensors right now.\n"
        "  - 'Reserved' = memory the CUDA caching allocator is holding\n"
        "    onto (may be more than Allocated -- that gap is\n"
        "    fragmentation, not genuine usage).\n"
        "  - If Allocated alone is already close to the physical VRAM\n"
        "    figure above AND flat over time, the model+batch genuinely\n"
        "    needs that much -- the fix is a smaller --batch/--imgsz.\n"
        "  - If Allocated keeps climbing per the leak check above, that's\n"
        "    a leak, not a sizing issue -- try the --copy-paste 0 --mixup 0\n"
        "    isolation run, and try 'pip install -U ultralytics' since a\n"
        "    newer release than yours is available and memory-growth bugs\n"
        "    do get fixed in point releases.\n"
    )


if __name__ == "__main__":
    main()