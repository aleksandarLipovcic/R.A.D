"""
benchmark_speed.py -- Inference speed of the Project R.A.D. detector
=======================================================================
Answers one question for the paper: "how many milliseconds per frame does
the model need on this hardware, and is that fast enough for the video?"

It takes real frames from your test video (evenly spread over the clip),
warms the model up, then times N forward passes and reports mean / median /
95th percentile per stage and the resulting frames per second.

THREE BACKENDS (run each separately, one line per result is appended to the
JSON so you can compare them afterwards):

  torch   -- best.pt through Ultralytics/PyTorch.
             Your GPU currently fails with CUDNN_STATUS_SUBLIBRARY_VERSION_
             MISMATCH. --no-cudnn turns cuDNN off inside PyTorch, which
             sidesteps that error and lets the GPU run with PyTorch's own
             convolution kernels. This is somewhat SLOWER than a working
             cuDNN, so treat the GPU number as conservative (say so in the
             paper).
  onnx    -- the exported best.onnx through Ultralytics + ONNX Runtime.
             Exported automatically if the .onnx file does not exist.
             CPU works with plain `pip install onnxruntime`. For GPU you need
             `onnxruntime-gpu` with a matching CUDA/cuDNN; if it is missing
             the run silently falls back to CPU -- the script prints which
             providers are available so you can tell.
  opencv  -- cv2.dnn on the ONNX file, the closest match to what a C++
             DetectionLink built on OpenCV does. GPU needs your CUDA-enabled
             OpenCV build. Some end-to-end (NMS-free) YOLO26 exports contain
             operators OpenCV DNN cannot load; the script then says so
             instead of crashing.

USAGE (from the folder with best.pt; use your own video path):

  # CPU baseline (PyTorch)
  python benchmark_speed.py --weights runs\\detect\\yolom_main_run\\weights\\best.pt ^
      --video "datasets\\test\\real_video.mp4" --backend torch --device cpu

  # GPU (PyTorch, cuDNN off to avoid the version-mismatch error)
  python benchmark_speed.py --weights runs\\detect\\yolom_main_run\\weights\\best.pt ^
      --video "datasets\\test\\real_video.mp4" --backend torch --device 0 --no-cudnn

  # GPU half precision (usually the fastest PyTorch option)
  python benchmark_speed.py ... --backend torch --device 0 --no-cudnn --half

  # ONNX on CPU / GPU
  python benchmark_speed.py ... --backend onnx --device cpu
  python benchmark_speed.py ... --backend onnx --device 0

  # OpenCV DNN on the exported ONNX (mimics the C++ backend)
  python benchmark_speed.py ... --backend opencv --device cpu
  python benchmark_speed.py ... --backend opencv --device 0 --half

Add --crop-width 1341 to drop the black bar on the right of real_video.mp4
(same effect as cropping before detection, see the paper's section 13.6).
Results: speed_results.json (one entry per run) and a printed table.
"""

import argparse
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--weights", required=True, help="Path to best.pt")
    p.add_argument("--video", required=True, help="Test video (frames are sampled from it)")
    p.add_argument("--backend", choices=["torch", "onnx", "opencv"], default="torch")
    p.add_argument("--device", default="cpu", help="'cpu' or a GPU index such as 0")
    p.add_argument("--half", action="store_true", help="FP16 (GPU only)")
    p.add_argument("--no-cudnn", action="store_true",
                   help="Disable cuDNN in PyTorch (workaround for the version-mismatch error)")
    p.add_argument("--imgsz", type=int, default=960)
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--frames", type=int, default=30, help="How many distinct frames to sample")
    p.add_argument("--warmup", type=int, default=10)
    p.add_argument("--runs", type=int, default=100, help="Timed forward passes")
    p.add_argument("--crop-width", type=int, default=None,
                   help="Keep only the left N pixels of each frame (drops the black bar)")
    p.add_argument("--onnx", default=None, help="Path to .onnx (default: next to the weights)")
    p.add_argument("--out", default="speed_results.json")
    return p.parse_args()


def load_frames(video, n, crop_width):
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open video: {video}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = []
    for i in np.linspace(0, max(total - 1, 0), n).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, f = cap.read()
        if ok:
            frames.append(f[:, :crop_width] if crop_width else f)
    cap.release()
    if not frames:
        raise SystemExit("No frames could be read from the video.")
    return frames


def is_cuda(device):
    return str(device).lower() not in ("cpu", "-1", "")


def sync_cuda(device):
    if is_cuda(device):
        try:
            import torch
            if torch.cuda.is_available():
                torch.cuda.synchronize()
        except Exception:
            pass


def bench_ultralytics(model_path, args, frames, half):
    """Shared by 'torch' (best.pt) and 'onnx' (best.onnx)."""
    from ultralytics import YOLO
    try:  # the deprecated-'half' warning would otherwise print once per frame
        import logging
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
    except Exception:
        pass
    model = YOLO(model_path, task="detect")
    kw = dict(imgsz=args.imgsz, conf=args.conf, verbose=False,
              device=args.device if is_cuda(args.device) else "cpu",
              half=bool(half and is_cuda(args.device)))
    for i in range(args.warmup):
        model.predict(frames[i % len(frames)], **kw)
    sync_cuda(args.device)
    rows = []
    for i in range(args.runs):
        t0 = time.perf_counter()
        r = model.predict(frames[i % len(frames)], **kw)
        sync_cuda(args.device)
        total = (time.perf_counter() - t0) * 1000
        sp = r[0].speed
        rows.append((sp["preprocess"], sp["inference"], sp["postprocess"], total))
    return rows


def bench_torch(args, frames):
    if args.no_cudnn:
        import torch
        torch.backends.cudnn.enabled = False
        print("cuDNN disabled inside PyTorch (--no-cudnn).")
    return bench_ultralytics(args.weights, args, frames, args.half)


def ensure_onnx(args):
    onnx = Path(args.onnx) if args.onnx else Path(args.weights).with_suffix(".onnx")
    if onnx.exists():
        return onnx
    print(f"{onnx} not found -- exporting with imgsz={args.imgsz} ...")
    from ultralytics import YOLO
    out = YOLO(args.weights).export(format="onnx", imgsz=args.imgsz, simplify=True,
                                    half=bool(args.half and is_cuda(args.device)),
                                    device=args.device if is_cuda(args.device) else "cpu")
    return Path(out)


def bench_onnx(args, frames):
    try:
        import onnxruntime as ort
        print("ONNX Runtime providers available:", ort.get_available_providers())
        if is_cuda(args.device) and "CUDAExecutionProvider" not in ort.get_available_providers():
            print("WARNING: CUDAExecutionProvider is not available -- this run will use the CPU "
                  "(install onnxruntime-gpu with matching CUDA/cuDNN for a GPU number).")
    except ImportError:
        raise SystemExit("onnxruntime is not installed:  pip install onnxruntime")
    onnx = ensure_onnx(args)
    return bench_ultralytics(str(onnx), args, frames, False)


def bench_opencv(args, frames):
    onnx = ensure_onnx(args)
    try:
        net = cv2.dnn.readNetFromONNX(str(onnx))
    except cv2.error as e:
        raise SystemExit("OpenCV DNN could not load this ONNX model (an operator is probably "
                         "unsupported, common with end-to-end YOLO26 exports). Details:\n"
                         f"{e}\nTry exporting with end2end disabled, or use --backend onnx.")
    if is_cuda(args.device):
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16 if args.half else cv2.dnn.DNN_TARGET_CUDA)
    else:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    size = (args.imgsz, args.imgsz)

    def one(frame):
        t0 = time.perf_counter()
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, size, swapRB=True, crop=False)
        t1 = time.perf_counter()
        net.setInput(blob)
        net.forward()
        t2 = time.perf_counter()
        return (t1 - t0) * 1000, (t2 - t1) * 1000, (t2 - t0) * 1000

    for i in range(args.warmup):
        one(frames[i % len(frames)])
    rows = []
    for i in range(args.runs):
        pre, inf, tot = one(frames[i % len(frames)])
        rows.append((pre, inf, 0.0, tot))  # postprocess (decoding) is not timed here
    return rows


def stats(vals):
    v = sorted(vals)
    return {"mean": statistics.fmean(v), "median": statistics.median(v),
            "p95": v[min(len(v) - 1, int(round(0.95 * (len(v) - 1))))]}


def hardware_label(args):
    cpu = platform.processor() or platform.machine()
    gpu = None
    try:
        import torch
        if is_cuda(args.device) and torch.cuda.is_available():
            gpu = torch.cuda.get_device_name(int(args.device))
    except Exception:
        pass
    return cpu, gpu


def main():
    args = parse_args()
    frames = load_frames(args.video, args.frames, args.crop_width)
    h, w = frames[0].shape[:2]
    print(f"{len(frames)} frames sampled, frame size {w}x{h}, imgsz={args.imgsz}, "
          f"backend={args.backend}, device={args.device}, half={args.half}")

    rows = {"torch": bench_torch, "onnx": bench_onnx, "opencv": bench_opencv}[args.backend](args, frames)

    cols = list(zip(*rows))
    res = {"preprocess_ms": stats(cols[0]), "inference_ms": stats(cols[1]),
           "postprocess_ms": stats(cols[2]), "total_ms": stats(cols[3])}
    mean_total = res["total_ms"]["mean"]
    fps = 1000.0 / mean_total if mean_total > 0 else float("inf")
    cpu, gpu = hardware_label(args)
    entry = {"backend": args.backend, "device": str(args.device), "fp16": bool(args.half),
             "cudnn_disabled": bool(args.no_cudnn), "imgsz": args.imgsz,
             "frame_size": f"{w}x{h}", "runs": args.runs, "warmup": args.warmup,
             "cpu": cpu, "gpu": gpu, "fps_from_mean_total": fps, **res}

    print(f"\n{'stage':14s} {'mean ms':>9s} {'median ms':>10s} {'p95 ms':>9s}")
    for k, lab in [("preprocess_ms", "preprocess"), ("inference_ms", "inference"),
                   ("postprocess_ms", "postprocess"), ("total_ms", "TOTAL")]:
        s = res[k]
        print(f"{lab:14s} {s['mean']:9.1f} {s['median']:10.1f} {s['p95']:9.1f}")
    print(f"\n-> about {fps:.1f} frames per second ({mean_total:.1f} ms per frame)")
    for target in (60, 30, 15):
        ok = mean_total <= 1000.0 / target
        print(f"   {target:>2d} fps needs <= {1000.0 / target:5.1f} ms per frame: {'YES' if ok else 'no'}")

    out = Path(args.out)
    data = json.loads(out.read_text(encoding="utf-8")) if out.exists() else []
    data.append(entry)
    out.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"\nAppended to {out}")


if __name__ == "__main__":
    sys.exit(main())