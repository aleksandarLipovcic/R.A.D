"""
fpv_eval.py -- measure the deployed detector on OUR OWN FPV footage
====================================================================
eval_report.py scores the model on the public datasets (VisDrone, UAVDT,
SARD). The drone flies an analog 5.8 GHz FPV camera, which none of those
datasets look like, so their numbers do not say how well the model does in
the air. This script measures that, and uses it to tune the cockpit.

It runs the SAME .onnx file DetectionLink loads, with the SAME letterbox
and output parsing as DetectionLink::runInference() (both ONNX layouts),
so a number here is a number the drone gets -- not what model.predict()
in PyTorch would get.

THREE COMMANDS

  1. extract -- frames from a flight video to label:
       python fpv_eval.py extract --video flight1.mp4 --out datasets/fpv_eval --every 15
     Writes datasets/fpv_eval/images/<video>_<frame>.jpg. With --prelabel
     <model.onnx>, also writes the model's own boxes as YOLO .txt labels
     (conf >= 0.25) into labels/, so labelling is correcting, not drawing
     from zero. Correct them with any YOLO-format labeller. An EMPTY .txt
     means "checked, nothing here" (a hard negative -- counts for false
     alarms); an image WITHOUT a .txt is treated as unlabelled and skipped.

  2. eval -- score the model on the labelled frames, per class:
       python fpv_eval.py eval --model ../models/yolo26m_main.onnx --data datasets/fpv_eval
     Options --variants base,deinterlace,flip,tiles compares inference
     variants on accuracy AND time per frame (see VARIANTS below), so a
     change is adopted in the C++ side only if it is measured to help.

  3. eval ... --write-thresholds -- also writes <model>.thresholds.json
     next to the model: per-class confidence thresholds at the best
     F-score on this footage (F2 for person: missing a person costs more
     than a false alarm in search and rescue; F1 for vehicles). The
     cockpit loads it at startup (DetectionLink.set_class_confidence_threshold).

VARIANTS (what each measures)
  base         exactly what the drone does today.
  deinterlace  analog video is interlaced; motion leaves comb artefacts.
               Keeps the even field lines and interpolates the odd ones.
  flip         test-time augmentation: frame + mirrored frame, merged
               (2x inference).
  tiles        full frame + 2x2 overlapping tiles, merged (5x inference):
               small, distant objects are enlarged -- the SAR case.

OUTPUT (under --out, default runs/fpv_eval/<timestamp>)
  fpv_eval.json   everything: per variant, per class AP50 / precision /
                  recall / best threshold, ms per frame.
  printed table   the same, readable.

Needs: opencv-python (or -headless), numpy. No PyTorch, no Ultralytics.
"""
import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
IOU_MATCH = 0.5            # AP50 -- same as the mAP50 train.py reports
NMS_IOU = 0.7              # Ultralytics predict default, same as the C++ side
MAX_DET = 300
SCORE_FLOOR = 0.01         # candidate floor for AP curves
PRELABEL_CONF = 0.25
F_BETA = {"person": 2.0}   # recall-weighted classes; others use F1


# =============================================================================
# Inference -- mirrors DetectionLink::runInference()
# =============================================================================
class OnnxDetector:
    def __init__(self, model_path: Path, imgsz: int | None = None, cuda: bool = False):
        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        if cuda:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.imgsz = imgsz or self._input_size_from_model(model_path) or 960
        names = model_path.with_suffix(".names")
        self.names = ([l.strip() for l in names.read_text().splitlines() if l.strip()]
                      if names.exists() else [])

    @staticmethod
    def _input_size_from_model(model_path: Path):
        try:
            import onnx
            m = onnx.load(str(model_path), load_external_data=False)
            dims = m.graph.input[0].type.tensor_type.shape.dim
            v = dims[2].dim_value
            return int(v) if v > 0 else None
        except Exception:
            return None

    def detect(self, bgr: np.ndarray) -> np.ndarray:
        """Returns Nx6 [x1, y1, x2, y2, conf, cls] in frame pixels."""
        h0, w0 = bgr.shape[:2]
        s = self.imgsz
        scale = min(s / w0, s / h0)
        sw, sh = max(1, int(w0 * scale)), max(1, int(h0 * scale))
        px, py = (s - sw) // 2, (s - sh) // 2
        lb = np.full((s, s, 3), 114, np.uint8)
        lb[py:py + sh, px:px + sw] = cv2.resize(bgr, (sw, sh))
        blob = cv2.dnn.blobFromImage(lb, 1.0 / 255.0, (s, s), swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        boxes = parse_output(out, s)
        if len(boxes):
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] - px) / scale
            boxes[:, [1, 3]] = (boxes[:, [1, 3]] - py) / scale
            boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w0)
            boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h0)
            keep = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
            boxes = boxes[keep]
        return boxes


def parse_output(out: np.ndarray, input_size: int) -> np.ndarray:
    """Both layouts DetectionLink accepts; returns Nx6 in input space."""
    if out.ndim == 3:
        d1, d2 = out.shape[1], out.shape[2]
        a = out[0]
    elif out.ndim == 2:
        d1, d2 = out.shape
        a = out
    else:
        raise ValueError(f"unsupported output shape {out.shape}")
    if d2 == 6 and d1 >= 6:                                 # end-to-end
        rows = a[a[:, 4] >= 0.001]
        return np.column_stack([rows[:, :5], np.round(rows[:, 5])]).astype(np.float32)
    if d1 > 4 and d2 > d1:                                  # raw [4+nc, anchors]
        a = a.T
        scores = a[:, 4:]
        cls = scores.argmax(1)
        conf = scores[np.arange(len(a)), cls]
        m = conf >= SCORE_FLOOR
        a, cls, conf = a[m], cls[m], conf[m]
        xyxy = np.column_stack([a[:, 0] - a[:, 2] / 2, a[:, 1] - a[:, 3] / 2,
                                a[:, 0] + a[:, 2] / 2, a[:, 1] + a[:, 3] / 2])
        dets = np.column_stack([xyxy, conf, cls]).astype(np.float32)
        return nms(dets, NMS_IOU, MAX_DET)
    raise ValueError(f"unsupported output shape {out.shape}")


def nms(dets: np.ndarray, iou: float, max_det: int = MAX_DET) -> np.ndarray:
    """Class-aware NMS (same class-offset trick as the C++ side)."""
    if len(dets) == 0:
        return dets
    off = dets[:, 5:6] * 1e5
    rects = np.column_stack([dets[:, 0:1] + off, dets[:, 1:2] + off,
                             dets[:, 2] - dets[:, 0], dets[:, 3] - dets[:, 1]])
    keep = cv2.dnn.NMSBoxes(rects.tolist(), dets[:, 4].tolist(), 0.0, iou, top_k=max_det)
    keep = np.array(keep).reshape(-1)
    return dets[keep] if len(keep) else dets[:0]


# =============================================================================
# Variants
# =============================================================================
def deinterlace(bgr):
    out = bgr.copy()
    even = bgr[0::2].astype(np.float32)
    nxt = np.vstack([even[1:], even[-1:]])
    out[1::2] = ((even + nxt) * 0.5)[: out[1::2].shape[0]].astype(np.uint8)
    return out


def run_variant(det: OnnxDetector, bgr, variant: str) -> np.ndarray:
    if variant == "base":
        return det.detect(bgr)
    if variant == "deinterlace":
        return det.detect(deinterlace(bgr))
    if variant == "flip":
        a = det.detect(bgr)
        b = det.detect(cv2.flip(bgr, 1))
        if len(b):
            w = bgr.shape[1]
            b[:, [0, 2]] = w - b[:, [2, 0]]
        return nms(np.vstack([a, b]), NMS_IOU)
    if variant == "tiles":
        h, w = bgr.shape[:2]
        th, tw = int(h * 0.6), int(w * 0.6)          # 2x2 tiles, 20 % overlap
        parts = [det.detect(bgr)]
        for y0 in (0, h - th):
            for x0 in (0, w - tw):
                d = det.detect(bgr[y0:y0 + th, x0:x0 + tw])
                if len(d):
                    # drop boxes cut by an inner tile edge (the full frame has them whole)
                    cut = (((d[:, 0] < 2) & (x0 > 0)) | ((d[:, 2] > tw - 2) & (x0 + tw < w)) |
                           ((d[:, 1] < 2) & (y0 > 0)) | ((d[:, 3] > th - 2) & (y0 + th < h)))
                    d = d[~cut]
                    d[:, [0, 2]] += x0
                    d[:, [1, 3]] += y0
                    parts.append(d)
        return nms(np.vstack(parts), 0.5)
    raise ValueError(f"unknown variant {variant}")


# =============================================================================
# Metrics
# =============================================================================
def box_iou(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ix1 = np.maximum(a[:, None, 0], b[None, :, 0])
    iy1 = np.maximum(a[:, None, 1], b[None, :, 1])
    ix2 = np.minimum(a[:, None, 2], b[None, :, 2])
    iy2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(ix2 - ix1, 0, None) * np.clip(iy2 - iy1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / np.maximum(area_a[:, None] + area_b[None, :] - inter, 1e-9)


def match_image(preds: np.ndarray, gts: np.ndarray, cls: int):
    """Greedy by confidence (COCO style). Returns (conf, is_tp) per prediction of class cls."""
    p = preds[preds[:, 5] == cls]
    g = gts[gts[:, 4] == cls] if len(gts) else gts
    if len(p) == 0:
        return np.zeros(0), np.zeros(0, bool)
    p = p[np.argsort(-p[:, 4])]
    tp = np.zeros(len(p), bool)
    if len(g):
        ious = box_iou(p[:, :4], g[:, :4])
        used = np.zeros(len(g), bool)
        for i in range(len(p)):
            j = int(np.argmax(np.where(used, -1, ious[i])))
            if not used[j] and ious[i, j] >= IOU_MATCH:
                used[j] = True
                tp[i] = True
    return p[:, 4], tp


def ap_101(recall: np.ndarray, precision: np.ndarray) -> float:
    """COCO 101-point interpolated AP (what Ultralytics reports)."""
    mrec = np.concatenate([[0.0], recall, [1.0]])
    mpre = np.concatenate([[1.0], precision, [0.0]])
    mpre = np.flip(np.maximum.accumulate(np.flip(mpre)))
    x = np.linspace(0, 1, 101)
    trapz = getattr(np, "trapezoid", None) or np.trapz
    return float(trapz(np.interp(x, mrec, mpre), x))


def class_metrics(confs: np.ndarray, tps: np.ndarray, n_gt: int, beta: float) -> dict:
    res = {"n_gt": int(n_gt), "n_pred": int(len(confs))}
    if n_gt == 0 or len(confs) == 0:
        res.update(ap50=0.0 if n_gt else None, best_threshold=None)
        return res
    order = np.argsort(-confs)
    confs, tps = confs[order], tps[order]
    ctp = np.cumsum(tps)
    cfp = np.cumsum(~tps)
    recall = ctp / n_gt
    precision = ctp / np.maximum(ctp + cfp, 1)
    b2 = beta * beta
    f = (1 + b2) * precision * recall / np.maximum(b2 * precision + recall, 1e-9)
    k = int(np.argmax(f))
    res.update(ap50=round(ap_101(recall, precision), 4),
               best_threshold=round(float(confs[k]), 3),
               precision_at_best=round(float(precision[k]), 3),
               recall_at_best=round(float(recall[k]), 3),
               f_beta=beta, f_at_best=round(float(f[k]), 3))
    return res


# =============================================================================
# Data
# =============================================================================
def load_labels(txt: Path, w: int, h: int) -> np.ndarray:
    rows = []
    for line in txt.read_text().splitlines():
        v = line.split()
        if len(v) < 5:
            continue
        c, cx, cy, bw, bh = int(v[0]), *map(float, v[1:5])
        rows.append([(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h, c])
    return np.array(rows, np.float32).reshape(-1, 5)


def labelled_images(data: Path):
    img_dir, lbl_dir = data / "images", data / "labels"
    imgs = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    out, skipped = [], 0
    for p in imgs:
        t = lbl_dir / (p.stem + ".txt")
        if t.exists():
            out.append((p, t))
        else:
            skipped += 1
    return out, skipped


# =============================================================================
# Commands
# =============================================================================
def cmd_extract(args):
    out = Path(args.out)
    (out / "images").mkdir(parents=True, exist_ok=True)
    det = None
    if args.prelabel:
        (out / "labels").mkdir(parents=True, exist_ok=True)
        det = OnnxDetector(Path(args.prelabel), args.imgsz, args.cuda)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"cannot open {args.video}")
    stem = Path(args.video).stem
    idx = n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            name = f"{stem}_{idx:06d}"
            cv2.imwrite(str(out / "images" / f"{name}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if det is not None:
                h, w = frame.shape[:2]
                d = det.detect(frame)
                d = d[d[:, 4] >= PRELABEL_CONF]
                lines = [f"{int(r[5])} {(r[0] + r[2]) / 2 / w:.6f} {(r[1] + r[3]) / 2 / h:.6f} "
                         f"{(r[2] - r[0]) / w:.6f} {(r[3] - r[1]) / h:.6f}" for r in d]
                (out / "labels" / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            n += 1
        idx += 1
    print(f"{n} frames written to {out / 'images'}"
          + (f" with model pre-labels in {out / 'labels'} -- CORRECT THEM before evaluating" if det else ""))


def evaluate(det: OnnxDetector, items, variant: str, names: list[str]) -> dict:
    per_class = {c: ([], [], 0) for c in range(len(names))}
    t_total = 0.0
    fp_on_empty = 0
    for img_path, lbl_path in items:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        gts = load_labels(lbl_path, w, h)
        t0 = time.perf_counter()
        preds = run_variant(det, img, variant)
        t_total += time.perf_counter() - t0
        if len(gts) == 0:
            fp_on_empty += int((preds[:, 4] >= 0.25).sum()) if len(preds) else 0
        for c in per_class:
            confs, tps = match_image(preds, gts, c)
            cl, tl, ng = per_class[c]
            cl.append(confs)
            tl.append(tps)
            per_class[c] = (cl, tl, ng + int((gts[:, 4] == c).sum()) if len(gts) else ng)
    classes = {}
    for c, (cl, tl, ng) in per_class.items():
        name = names[c]
        confs = np.concatenate(cl) if cl else np.zeros(0)
        tps = np.concatenate(tl) if tl else np.zeros(0, bool)
        classes[name] = class_metrics(confs, tps, ng, F_BETA.get(name, 1.0))
    aps = [m["ap50"] for m in classes.values() if m.get("ap50") is not None]
    return {"variant": variant,
            "images": len(items),
            "ms_per_frame": round(1000 * t_total / max(1, len(items)), 1),
            "map50": round(float(np.mean(aps)), 4) if aps else None,
            "false_alarms_on_empty_frames_at_0.25": fp_on_empty,
            "classes": classes}


def cmd_eval(args):
    model = Path(args.model)
    det = OnnxDetector(model, args.imgsz, args.cuda)
    names = det.names or [f"class_{i}" for i in range(100)]
    items, skipped = labelled_images(Path(args.data))
    if not items:
        sys.exit(f"no labelled images under {args.data} (images/ + labels/)")
    print(f"{len(items)} labelled frames ({skipped} without a label file skipped), "
          f"model {model.name}, input {det.imgsz}")
    results = []
    for v in args.variants.split(","):
        r = evaluate(det, items, v.strip(), names)
        results.append(r)
        print(f"\n=== {r['variant']}: mAP50 {r['map50']}  |  {r['ms_per_frame']} ms/frame (CPU/cv2 "
              f"unless --cuda)  |  false alarms on empty frames: {r['false_alarms_on_empty_frames_at_0.25']}")
        print(f"  {'class':15s} {'GT':>5s} {'AP50':>6s} {'thr':>6s} {'P':>6s} {'R':>6s}")
        for name, m in r["classes"].items():
            if m["n_gt"] == 0:
                continue   # no labelled objects: no recall/AP to show
            fmt = lambda k: f"{m[k]:6.3f}" if m.get(k) is not None else "     -"
            print(f"  {name:15s} {m['n_gt']:5d} {fmt('ap50')} {fmt('best_threshold')} "
                  f"{fmt('precision_at_best')} {fmt('recall_at_best')}")

    out = Path(args.out or f"runs/fpv_eval/{datetime.now():%Y%m%d_%H%M%S}")
    out.mkdir(parents=True, exist_ok=True)
    (out / "fpv_eval.json").write_text(json.dumps(
        {"model": str(model), "data": str(args.data), "input_size": det.imgsz,
         "results": results}, indent=2))
    print(f"\nwritten {out / 'fpv_eval.json'}")

    if args.write_thresholds:
        base = results[0]
        thr = {n: m["best_threshold"] for n, m in base["classes"].items()
               if m.get("best_threshold") is not None and m["n_gt"] >= args.min_gt}
        if not thr:
            print(f"no class has >= {args.min_gt} labelled objects -- thresholds NOT written")
            return
        path = model.with_suffix(".thresholds.json")
        path.write_text(json.dumps({
            "generated": datetime.now().isoformat(timespec="seconds"),
            "source": f"fpv_eval.py on {len(items)} frames of {args.data}, variant '{base['variant']}'",
            "thresholds": thr}, indent=2))
        print(f"written {path}: {thr}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    e = sub.add_parser("extract", help="frames from a flight video (optionally pre-labelled)")
    e.add_argument("--video", required=True)
    e.add_argument("--out", default="datasets/fpv_eval")
    e.add_argument("--every", type=int, default=15, help="keep every Nth frame (15 = 2 per s at 30 fps)")
    e.add_argument("--prelabel", default=None, help="ONNX model whose boxes become draft labels")
    e.add_argument("--imgsz", type=int, default=None)
    e.add_argument("--cuda", action="store_true")
    e.set_defaults(fn=cmd_extract)

    v = sub.add_parser("eval", help="score the model on labelled frames")
    v.add_argument("--model", required=True, help="the .onnx the cockpit uses (with its .names)")
    v.add_argument("--data", default="datasets/fpv_eval")
    v.add_argument("--variants", default="base", help="comma list of base,deinterlace,flip,tiles")
    v.add_argument("--imgsz", type=int, default=None, help="default: read from the ONNX input")
    v.add_argument("--cuda", action="store_true", help="cv2 CUDA backend (needs a CUDA OpenCV build)")
    v.add_argument("--out", default=None)
    v.add_argument("--write-thresholds", action="store_true",
                   help="write <model>.thresholds.json from the FIRST variant")
    v.add_argument("--min-gt", type=int, default=20,
                   help="classes with fewer labelled objects keep the global threshold")
    v.set_defaults(fn=cmd_eval)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
