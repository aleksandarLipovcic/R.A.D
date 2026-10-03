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

COMMANDS  (extract / eval / ref)

  0. ref -- if a flight ALREADY has reference boxes (CSV: frame, class,
     box), skip labelling entirely and score the raw video against it:
       python fpv_eval.py ref --model ../models/yolo26m_main.onnx \\
           --video rad_fpv_cropped_part0.mp4 --reference labels/part0
     --reference is either a folder of per-frame YOLO txt files
     (<name>_<frame>.txt, 0-based; empty file = checked empty frame) or a
     CSV with one row per box (columns found by header name).

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


def file_samples(items):
    """(image, gts) per labelled image file (images/ + labels/ layout)."""
    for img_path, lbl_path in items:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        yield img, load_labels(lbl_path, w, h)


MERGE_VEHICLES = False     # --merge-vehicles: score every non-person class as one "vehicle"


def merged_names(names: list[str]) -> tuple[list[str], np.ndarray]:
    """(names, class-index remap) for --merge-vehicles: person stays, all other
    classes become 'vehicle' -- the same grouping the cockpit's tracker uses."""
    if "person" not in names:
        return names, np.arange(len(names))
    p = names.index("person")
    remap = np.array([0 if i == p else 1 for i in range(len(names))])
    return ["person", "vehicle"], remap


def evaluate(det: OnnxDetector, samples, variant: str, names: list[str]) -> dict:
    """samples: iterable of (BGR image, Nx5 gts [x1,y1,x2,y2,cls] in pixels)."""
    remap = None
    if MERGE_VEHICLES:
        names, remap = merged_names(names)
    per_class = {c: ([], [], 0) for c in range(len(names))}
    t_total = 0.0
    fp_on_empty = 0
    n_images = 0
    for img, gts in samples:
        n_images += 1
        t0 = time.perf_counter()
        preds = run_variant(det, img, variant)
        t_total += time.perf_counter() - t0
        if remap is not None:
            if len(preds):
                preds = preds.copy()
                preds[:, 5] = remap[preds[:, 5].astype(int)]
                preds = nms(preds, NMS_IOU)        # car + van boxes on one object -> one vehicle
            if len(gts):
                gts = gts.copy()
                gts[:, 4] = remap[gts[:, 4].astype(int)]
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
    return {"variant": variant + (" (vehicles merged)" if remap is not None else "")
                       + (f" @IoU{IOU_MATCH:g}" if IOU_MATCH != 0.5 else ""),
            "images": n_images,
            "ms_per_frame": round(1000 * t_total / max(1, n_images), 1),
            "map50": round(float(np.mean(aps)), 4) if aps else None,
            "false_alarms_on_empty_frames_at_0.25": fp_on_empty,
            "classes": classes}


def report(args, model: Path, det: OnnxDetector, results: list, data_desc: str, n_frames: int):
    """Prints the tables, writes fpv_eval.json and (optionally) the thresholds file."""
    for r in results:
        print(f"\n=== {r['variant']}: mAP50 {r['map50']}  |  {r['ms_per_frame']} ms/frame (CPU/cv2 "
              f"unless --cuda)  |  false alarms on empty frames: {r['false_alarms_on_empty_frames_at_0.25']}")
        ap_col = f"AP{round(IOU_MATCH * 100)}"
        print(f"  {'class':15s} {'GT':>5s} {ap_col:>6s} {'thr':>6s} {'P':>6s} {'R':>6s}")
        for name, m in r["classes"].items():
            if m["n_gt"] == 0:
                continue   # no labelled objects: no recall/AP to show
            fmt = lambda k: f"{m[k]:6.3f}" if m.get(k) is not None else "     -"
            print(f"  {name:15s} {m['n_gt']:5d} {fmt('ap50')} {fmt('best_threshold')} "
                  f"{fmt('precision_at_best')} {fmt('recall_at_best')}")

    out = Path(args.out or f"runs/fpv_eval/{datetime.now():%Y%m%d_%H%M%S}")
    out.mkdir(parents=True, exist_ok=True)
    (out / "fpv_eval.json").write_text(json.dumps(
        {"model": str(model), "data": data_desc, "input_size": det.imgsz,
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
            "source": f"fpv_eval.py on {n_frames} frames of {data_desc}, variant '{base['variant']}'",
            "thresholds": thr}, indent=2))
        print(f"written {path}: {thr}")


def cmd_eval(args):
    model = Path(args.model)
    det = OnnxDetector(model, args.imgsz, args.cuda)
    names = det.names or [f"class_{i}" for i in range(100)]
    items, skipped = labelled_images(Path(args.data))
    if not items:
        sys.exit(f"no labelled images under {args.data} (images/ + labels/)")
    print(f"{len(items)} labelled frames ({skipped} without a label file skipped), "
          f"model {model.name}, input {det.imgsz}")
    results = [evaluate(det, file_samples(items), v.strip(), names) for v in args.variants.split(",")]
    report(args, model, det, results, str(args.data), len(items))


# =============================================================================
# Reference CSV (one row per box per frame) + the raw video
# =============================================================================
_FRAME_COLS = ("frame", "frame_idx", "frame_index", "frame_id", "frame_no")
_CLASS_NAME_COLS = ("class", "class_name", "label", "name", "cls_name")
_CLASS_ID_COLS = ("class_id", "cls", "cls_id", "category_id")


def load_reference_csv(path: Path, names: list[str], class_map: dict, ref_names: list[str] | None):
    """Returns ({frame: Nx5 [x1,y1,x2,y2,cls]}, report). Columns are found by
    header name; boxes as x1,y1,x2,y2 or x,y,w,h (pixels). Class names go
    through --class-map, then must match the model's .names."""
    import csv
    with open(path, newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh)
        cols = {c.strip().lower(): c for c in (reader.fieldnames or [])}

        def pick(options):
            return next((cols[o] for o in options if o in cols), None)

        c_frame = pick(_FRAME_COLS)
        c_name, c_id = pick(_CLASS_NAME_COLS), pick(_CLASS_ID_COLS)
        corner = all(k in cols for k in ("x1", "y1", "x2", "y2"))
        xywh = all(k in cols for k in ("x", "y", "w", "h"))
        if c_frame is None or (c_name is None and c_id is None) or not (corner or xywh):
            sys.exit(f"{path}: cannot recognise the columns {reader.fieldnames}.\n"
                     f"  needed: a frame column {_FRAME_COLS}, a class column "
                     f"{_CLASS_NAME_COLS + _CLASS_ID_COLS}, and x1,y1,x2,y2 or x,y,w,h. "
                     f"Send the header line so the reader can be extended.")
        index = {n: i for i, n in enumerate(names)}
        frames, unknown, kept = {}, {}, 0
        for row in reader:
            if c_name is not None and row[c_name].strip():
                cname = row[c_name].strip()
            else:
                cid = int(float(row[c_id]))
                cname = ref_names[cid] if ref_names and cid < len(ref_names) else (
                    names[cid] if cid < len(names) else str(cid))
            cname = class_map.get(cname, cname)
            if cname not in index:
                unknown[cname] = unknown.get(cname, 0) + 1
                continue
            if corner:
                x1, y1, x2, y2 = (float(row[cols[k]]) for k in ("x1", "y1", "x2", "y2"))
            else:
                x, y, w, h = (float(row[cols[k]]) for k in ("x", "y", "w", "h"))
                x1, y1, x2, y2 = x, y, x + w, y + h
            f = int(float(row[c_frame]))
            frames.setdefault(f, []).append([x1, y1, x2, y2, index[cname]])
            kept += 1
    return ({f: np.array(v, np.float32) for f, v in frames.items()},
            {"boxes": kept, "frames_with_boxes": len(frames), "unknown_classes": unknown,
             "box_format": "x1,y1,x2,y2" if corner else "x,y,w,h"})


def load_reference_dir(path: Path, frame_w: int, frame_h: int, n_classes: int, base: int = 0):
    """A folder of per-frame YOLO txt files (<anything>_<frame>.txt, normalized
    `cls xc yc w h`). Every file counts as a checked frame -- an EMPTY file is
    a frame with nothing in it (hard negative). Returns ({frame: Nx5}, report)."""
    import re
    frames, bad_cls, n_boxes = {}, {}, 0
    for t in sorted(path.glob("*.txt")):
        m = re.search(r"(\d+)$", t.stem)
        if not m:
            continue
        f = int(m.group(1)) - base
        rows = []
        for line in t.read_text().splitlines():
            v = line.split()
            if len(v) < 5:
                continue
            c = int(float(v[0]))
            if not 0 <= c < n_classes:
                bad_cls[c] = bad_cls.get(c, 0) + 1
                continue
            xc, yc, bw, bh = (float(x) for x in v[1:5])
            rows.append([(xc - bw / 2) * frame_w, (yc - bh / 2) * frame_h,
                         (xc + bw / 2) * frame_w, (yc + bh / 2) * frame_h, c])
        frames[f] = np.array(rows, np.float32).reshape(-1, 5)
        n_boxes += len(rows)
    return frames, {"boxes": n_boxes, "frames_with_boxes": sum(1 for a in frames.values() if len(a)),
                    "label_files": len(frames), "unknown_classes": bad_cls,
                    "box_format": "YOLO txt per frame"}


def video_samples(video: Path, gt: dict, frames: list[int]):
    """(image, gts) for the selected frame numbers (0-based), reading the video once."""
    want = set(frames)
    last = max(frames) if frames else -1
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        sys.exit(f"cannot open {video}")
    idx = 0
    while idx <= last:
        ok, img = cap.read()
        if not ok:
            break
        if idx in want:
            yield img, gt.get(idx, np.zeros((0, 5), np.float32))
        idx += 1
    cap.release()


def cmd_ref(args):
    model = Path(args.model)
    det = OnnxDetector(model, args.imgsz, args.cuda)
    names = det.names or [f"class_{i}" for i in range(100)]
    class_map = dict(kv.split("=", 1) for kv in args.class_map.split(",") if "=" in kv)
    ref_names = args.ref_names.split(",") if args.ref_names else None
    cap = cv2.VideoCapture(args.video)
    n_video = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.isOpened() else 0
    vw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if cap.isOpened() else 0
    vh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if cap.isOpened() else 0
    cap.release()
    if n_video <= 0:
        sys.exit(f"cannot open {args.video}")

    fps = 0.0
    cap = cv2.VideoCapture(args.video)
    if cap.isOpened():
        fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    print(f"video: {Path(args.video).name}  {vw}x{vh}  {n_video} frames  {fps:.0f} fps")

    ref = Path(args.reference)
    if ref.is_dir():
        gt, info = load_reference_dir(ref, vw, vh, len(names), args.ref_frame_base)
        if info["label_files"] != n_video:
            print(f"  WARNING: {info['label_files']} label files but {n_video} video frames -- "
                  f"is this the exact video the labels were made for (same cut, same "
                  f"crop)? Normalized boxes are placed using THIS video's size.")
        print(f"reference: {info['label_files']} label files, {info['boxes']} boxes on "
              f"{info['frames_with_boxes']} frames ({info['box_format']}, classes by index = "
              f"{', '.join(names[:8])}{', ...' if len(names) > 8 else ''}); "
              f"model {model.name}, input {det.imgsz}")
    else:
        gt, info = load_reference_csv(ref, names, class_map, ref_names)
        print(f"reference: {info['boxes']} boxes on {info['frames_with_boxes']} frames "
              f"({info['box_format']}); model {model.name}, input {det.imgsz}")
    if info["unknown_classes"]:
        print(f"  NOT counted (class not in the model's .names -- map with --class-map "
              f"ref=model): {info['unknown_classes']}")
    if not gt:
        sys.exit("no usable reference labels")
    if args.frames == "labeled":
        frames = sorted(f for f in gt if f % args.every == 0)
    else:   # every Nth frame of the whole video; frames without boxes = hard negatives
        frames = list(range(0, n_video, args.every))
    if args.max_frame is not None:
        frames = [f for f in frames if f <= args.max_frame]
    beyond = [f for f in gt if f >= n_video]
    if beyond:
        print(f"  WARNING: {len(beyond)} reference frames are beyond the video's "
              f"{n_video} frames -- wrong video, or 1-based frame numbers?")
    print(f"evaluating {len(frames)} frames ({args.frames}, every {args.every}) of "
          f"{Path(args.video).name} ({n_video} frames)")
    results = [evaluate(det, video_samples(Path(args.video), gt, frames), v.strip(), names)
               for v in args.variants.split(",")]
    report(args, model, det, results, f"{Path(args.video).name} + {Path(args.reference).name}",
           len(frames))


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
    v.add_argument("--iou", type=float, default=0.5,
                   help="box overlap needed for a match (default 0.5 = AP50). A much higher score at "
                        "0.3 means the objects ARE found but the boxes (model or reference) are imprecise")
    v.add_argument("--merge-vehicles", action="store_true",
                   help="score person vs. one 'vehicle' class (car/van/... confusions don't count), "
                        "like the cockpit tracker's grouping")
    v.add_argument("--imgsz", type=int, default=None, help="default: read from the ONNX input")
    v.add_argument("--cuda", action="store_true", help="cv2 CUDA backend (needs a CUDA OpenCV build)")
    v.add_argument("--out", default=None)
    v.add_argument("--write-thresholds", action="store_true",
                   help="write <model>.thresholds.json from the FIRST variant")
    v.add_argument("--min-gt", type=int, default=20,
                   help="classes with fewer labelled objects keep the global threshold")
    v.set_defaults(fn=cmd_eval)

    r = sub.add_parser("ref", help="score the model on a raw video against a reference-box CSV")
    r.add_argument("--model", required=True, help="the .onnx the cockpit uses (with its .names)")
    r.add_argument("--video", required=True, help="the RAW flight video (no boxes drawn in)")
    r.add_argument("--reference", required=True,
                   help="a folder of per-frame YOLO txt labels (<name>_<frame>.txt) -- preferred, "
                        "empty files count as checked empty frames -- or a CSV, one row per box")
    r.add_argument("--ref-frame-base", type=int, default=0,
                   help="frame number of the FIRST video frame in the label file names (0 or 1)")
    r.add_argument("--frames", choices=("labeled", "all"), default="labeled",
                   help="labeled = only frames that have reference boxes (default); all = every "
                        "Nth frame, frames without boxes count as hard negatives -- use only if "
                        "the reference covers the WHOLE video")
    r.add_argument("--every", type=int, default=5, help="use every Nth frame (default 5)")
    r.add_argument("--max-frame", type=int, default=None)
    r.add_argument("--class-map", default="", help="rename reference classes, e.g. truck=large_vehicle,bus=large_vehicle")
    r.add_argument("--ref-names", default=None,
                   help="comma list of class names for a numeric-only reference class column")
    r.add_argument("--variants", default="base", help="comma list of base,deinterlace,flip,tiles")
    r.add_argument("--iou", type=float, default=0.5,
                   help="box overlap needed for a match (default 0.5 = AP50). A much higher score at "
                        "0.3 means the objects ARE found but the boxes (model or reference) are imprecise")
    r.add_argument("--merge-vehicles", action="store_true",
                   help="score person vs. one 'vehicle' class (car/van/... confusions don't count), "
                        "like the cockpit tracker's grouping")
    r.add_argument("--imgsz", type=int, default=None)
    r.add_argument("--cuda", action="store_true")
    r.add_argument("--out", default=None)
    r.add_argument("--write-thresholds", action="store_true")
    r.add_argument("--min-gt", type=int, default=20)
    r.set_defaults(fn=cmd_ref)

    args = p.parse_args()
    global IOU_MATCH, MERGE_VEHICLES
    IOU_MATCH = getattr(args, "iou", 0.5)
    MERGE_VEHICLES = getattr(args, "merge_vehicles", False)
    if MERGE_VEHICLES and getattr(args, "write_thresholds", False):
        sys.exit("--write-thresholds needs the real classes -- run it without --merge-vehicles")
    args.fn(args)


if __name__ == "__main__":
    main()
