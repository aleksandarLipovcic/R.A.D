"""
vram_diagnostic.py — Root-cause a suspected WDDM VRAM spillover
=======================================================================
Companion to train.py, for when Task Manager's GPU tab (Performance ->
GPU 0 -> "Dedicated GPU memory" vs "Shared GPU memory") has already
confirmed Shared memory is climbing during training -- i.e. you know
WDDM is spilling, and now want to know WHAT is consuming the memory
instead of guessing (mosaic buffers? a particular xView batch? optimizer
state? disk-cache overhead?).

What this does:
  Runs a handful of REAL training batches (actual model, actual dataset,
  actual augmentation pipeline -- not a synthetic stand-in) via
  Ultralytics, exactly like a real run would, but:
    1. Turns on torch's built-in memory history recorder before the
       first batch.
    2. Stops itself after --num-batches (default 8) via a callback --
       long enough to get past warmup and into steady state, short
       enough to run in seconds not hours.
    3. Dumps a .pickle snapshot you can drag-and-drop at
       https://pytorch.org/memory_viz -- an interactive timeline showing
       every allocation, its size, and the Python call stack that made
       it. This is the actual answer to "what's consuming the memory",
       not an inference from GPU_mem numbers.
    4. Also prints torch.cuda.memory_summary(), which breaks down
       allocated vs reserved (reserved-but-not-allocated is
       fragmentation, not real usage -- an important distinction if the
       spillover turns out to be a fragmentation issue rather than a
       genuine "doesn't fit" issue) and shows cudaMalloc retry counts.

Usage:
    python vram_diagnostic.py --model yolo26s.pt --imgsz 640 --batch 4
    python vram_diagnostic.py --imgsz 640 --batch 2   # compare against
                                                        # a smaller batch
                                                        # to see how much
                                                        # peak memory
                                                        # scales with it

Then open the printed .pickle path at https://pytorch.org/memory_viz
(everything happens in your browser, nothing uploaded) and look at:
  - The tallest spike -- hover it to see the exact call stack. If it's
    inside augmentation/mosaic code, that's your answer. If it's inside
    the loss/backward pass, the model+batch genuinely needs that much.
  - Whether the baseline (between spikes) keeps climbing batch over
    batch -- that would indicate a leak (something not being freed),
    which is a different, more serious problem than "this batch is just
    big".
"""

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SNAPSHOT_PATH = SCRIPT_DIR / "vram_snapshot.pickle"


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
    p.add_argument("--num-batches", type=int, default=8,
                    help="How many real batches to run before stopping. "
                         "Enough to get past warmup (first 1-2 batches "
                         "include cudnn autotuning overhead that isn't "
                         "representative) and into steady state.")
    p.add_argument("--cache", default="disk",
                    help="Match whatever --cache you actually train with "
                         "-- disk caching itself has its own memory/IO "
                         "behavior worth including in the profile.")
    p.add_argument("--out", default=str(SNAPSHOT_PATH),
                    help="Where to write the .pickle snapshot.")
    return p.parse_args()


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

    print(f"\nRecording memory history for up to {args.num_batches} real "
          f"batches at {args.model} @ {args.imgsz}px, batch={args.batch}...")
    torch.cuda.memory._record_memory_history(max_entries=200_000)

    batch_count = {"n": 0}

    def probe_callback(trainer):
        batch_count["n"] += 1
        allocated = torch.cuda.memory_allocated(device)
        reserved = torch.cuda.memory_reserved(device)
        peak = torch.cuda.max_memory_allocated(device)
        print(f"  batch {batch_count['n']:>3}: "
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
            epochs=1,
            imgsz=args.imgsz,
            batch=args.batch,
            device=device,
            workers=0,
            cache=args.cache,
            amp=True,
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
            raise

    out_path = Path(args.out)
    torch.cuda.memory._dump_snapshot(str(out_path))
    torch.cuda.memory._record_memory_history(enabled=None)  # stop recording

    print(f"\n{'=' * 70}")
    print(f"Snapshot written: {out_path}")
    print(f"Open https://pytorch.org/memory_viz and drag this file in to "
          f"see exactly what allocated memory at each point, with call "
          f"stacks.")
    print(f"{'=' * 70}\n")
    print(torch.cuda.memory_summary(device=device, abbreviated=False))

    print(
        "\nHow to read this:\n"
        "  - 'Allocated' = memory actually holding live tensors right now.\n"
        "  - 'Reserved' = memory the CUDA caching allocator is holding\n"
        "    onto (may be more than Allocated -- that gap is\n"
        "    fragmentation, not genuine usage).\n"
        "  - If Allocated alone is already close to the physical VRAM\n"
        "    figure above, the model+batch genuinely needs that much --\n"
        "    the fix is a smaller --batch/--imgsz, not a leak/fragmentation\n"
        "    fix.\n"
        "  - If Reserved is much higher than Allocated, fragmentation is a\n"
        "    real contributor -- PYTORCH_CUDA_ALLOC_CONF=expandable_segments\n"
        "    would normally help here, but your last run showed that's a\n"
        "    no-op on your current torch/CUDA/Windows combo, so the more\n"
        "    reliable fix is still reducing --batch/--imgsz.\n"
        "  - If per-batch 'allocated' keeps climbing batch over batch\n"
        "    instead of settling into a steady band, that's a leak, not a\n"
        "    sizing issue -- worth flagging separately if you see it.\n"
    )


if __name__ == "__main__":
    main()