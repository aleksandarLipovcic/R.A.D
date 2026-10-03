"""
compare_detections.py -- Compare your NN's detections on the validation
video against the reference detections from detect_video.py.

INPUT: two directories of per-frame YOLO txt files, one line per box:
    cls xc yc w h [conf]        (unified class indices, normalized coords)
The frame number is taken from the LAST number in each filename, e.g.
part0_000123.txt -> 123, rad_fpv_cropped_part0_124.txt -> 124.

FRAME NUMBERING -- READ THIS: the reference labels are 0-based (first
frame = 0). Ultralytics' own `predict(save_txt=True, save_conf=True)` on a
video names its files <video>_<N>.txt with N 1-BASED. Set --pred-frame-base
to match however your files were written (default 1 = Ultralytics
convention). As a guard, the script also checks frame shifts of -3..+3
and warns if a shift other than the one you picked fits much better.

MATCHING: per frame, greedy one-to-one by IoU (highest first), boxes
matched only at IoU >= --iou. Reported per class:
  TP  = NN box matched to a reference box of the same class
  FP  = NN box with no matching reference box
  FN  = reference box the NN did not find
  class_confusion = boxes matched at IoU but with different classes
                    (counted as FP for the NN class and FN for the ref class)
The reference is itself a model, not hand-labeled ground truth -- treat
"precision/recall" here as AGREEMENT with the reference, and inspect the
frames listed in mismatches.csv before drawing conclusions.

USAGE:
    python compare_detections.py --ref labels/part0 --pred my_nn_labels/part0 \
        --pred-frame-base 1 --out compare_part0
"""
import argparse, csv, re
from collections import defaultdict
from pathlib import Path

UNIFIED_CLASSES = ["person", "car", "large_vehicle", "motorcycle", "other_vehicle"]


def load_dir(d: Path, base: int, conf_thr: float) -> dict:
    """-> {frame_index_0_based: [(cls, x1, y1, x2, y2, conf)]}"""
    out = {}
    for f in d.glob("*.txt"):
        m = re.findall(r"(\d+)", f.stem)
        if not m:
            continue
        idx = int(m[-1]) - base
        boxes = []
        for line in f.read_text().splitlines():
            p = line.split()
            if len(p) < 5:
                continue
            c = int(float(p[0])); xc, yc, w, h = map(float, p[1:5])
            cf = float(p[5]) if len(p) > 5 else 1.0
            if cf < conf_thr:
                continue
            boxes.append((c, xc - w / 2, yc - h / 2, xc + w / 2, yc + h / 2, cf))
        out[idx] = boxes
    return out


def iou(a, b):
    ix = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    iy = max(0.0, min(a[4], b[4]) - max(a[2], b[2]))
    inter = ix * iy
    u = (a[3] - a[1]) * (a[4] - a[2]) + (b[3] - b[1]) * (b[4] - b[2]) - inter
    return inter / u if u > 0 else 0.0


def match_frame(ref, pred, thr):
    pairs = sorted(((iou(r, p), i, j) for i, r in enumerate(ref) for j, p in enumerate(pred)),
                   reverse=True)
    ur, up, matches = set(), set(), []
    for v, i, j in pairs:
        if v < thr:
            break
        if i in ur or j in up:
            continue
        ur.add(i); up.add(j); matches.append((i, j, v))
    return matches, [i for i in range(len(ref)) if i not in ur], [j for j in range(len(pred)) if j not in up]


def match_frame_same_class(ref, pred, thr):
    """Same-class pairs first (as in the report's section 14), then the
    leftovers are paired regardless of class -- those are the class
    confusions. A duplicate car+large_vehicle box on one car then counts
    as one TP (car) and one FP (large_vehicle), not as a confusion."""
    pairs = sorted(((iou(r, p), i, j) for i, r in enumerate(ref) for j, p in enumerate(pred)
                    if r[0] == p[0]), reverse=True)
    ur, up, matches = set(), set(), []
    for v, i, j in pairs:
        if v < thr:
            break
        if i in ur or j in up:
            continue
        ur.add(i); up.add(j); matches.append((i, j, v))
    ri = [i for i in range(len(ref)) if i not in ur]
    pj = [j for j in range(len(pred)) if j not in up]
    m2, fn2, fp2 = match_frame([ref[i] for i in ri], [pred[j] for j in pj], thr)
    matches += [(ri[i], pj[j], v) for i, j, v in m2]
    return matches, [ri[i] for i in fn2], [pj[j] for j in fp2]


def total_matches(ref, pred, shift, thr):
    """Sum of matched-box IoUs -- more sensitive to a 1-frame misalignment
    than a plain match count on slow-moving footage."""
    return sum(v for f in ref for _, _, v in match_frame(ref[f], pred.get(f + shift, []), thr)[0])


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--ref", required=True); a.add_argument("--pred", required=True)
    a.add_argument("--ref-frame-base", type=int, default=0)
    a.add_argument("--pred-frame-base", type=int, default=1)
    a.add_argument("--ref-conf", type=float, default=0.25)
    a.add_argument("--pred-conf", type=float, default=0.25)
    a.add_argument("--iou", type=float, default=0.5)
    a.add_argument("--out", default="compare_out")
    a.add_argument("--class-agnostic", action="store_true",
                   help="Ignore classes -- measures only whether the same objects were "
                        "found and how well the boxes line up. Useful because the COCO "
                        "reference tends to call vans 'car' while a VisDrone-trained NN "
                        "calls them large_vehicle (van->large_vehicle in class_map).")
    a.add_argument("--match-same-class", action="store_true",
                   help="Pair same-class boxes first, then the rest regardless of class "
                        "(the report's section-14 rule). Without it, boxes are paired by IoU "
                        "only and the class is checked afterwards.")
    a = a.parse_args()

    ref = load_dir(Path(a.ref), a.ref_frame_base, a.ref_conf)
    pred = load_dir(Path(a.pred), a.pred_frame_base, a.pred_conf)
    if a.class_agnostic:
        global UNIFIED_CLASSES
        UNIFIED_CLASSES = ["any_object"]
        ref = {f: [(0, *b[1:]) for b in bs] for f, bs in ref.items()}
        pred = {f: [(0, *b[1:]) for b in bs] for f, bs in pred.items()}
    if not ref or not pred:
        raise SystemExit(f"no label files found (ref={len(ref)}, pred={len(pred)} frames)")
    print(f"ref frames {min(ref)}..{max(ref)} ({len(ref)}), pred frames {min(pred)}..{max(pred)} ({len(pred)})")

    shift_scores = {s: total_matches(ref, pred, s, a.iou) for s in range(-3, 4)}
    best = max(shift_scores, key=shift_scores.get)
    print("frame-alignment check (summed IoU per shift): "
          + ", ".join(f"{k:+d}: {v:.1f}" for k, v in shift_scores.items()))
    if best != 0:
        print(f"  WARNING: shifting predictions by {best} frame(s) fits better -- check "
              f"--pred-frame-base (currently {a.pred_frame_base}). Results below use NO shift.")

    stats = defaultdict(lambda: {"TP": 0, "FP": 0, "FN": 0, "iou_sum": 0.0})
    confusion = defaultdict(int)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    mm = csv.writer(open(out / "mismatches.csv", "w", newline=""))
    mm.writerow(["frame", "type", "ref_class", "pred_class", "iou", "ref_conf", "pred_conf",
                 "x1n", "y1n", "x2n", "y2n"])
    for f in sorted(set(ref) | set(pred)):
        R, P = ref.get(f, []), pred.get(f, [])
        matches, fn, fp = (match_frame_same_class if a.match_same_class else match_frame)(R, P, a.iou)
        for i, j, v in matches:
            rc, pc = R[i][0], P[j][0]
            if rc == pc:
                stats[rc]["TP"] += 1; stats[rc]["iou_sum"] += v
            else:
                stats[pc]["FP"] += 1; stats[rc]["FN"] += 1
                confusion[(UNIFIED_CLASSES[rc], UNIFIED_CLASSES[pc])] += 1
                mm.writerow([f, "class_confusion", UNIFIED_CLASSES[rc], UNIFIED_CLASSES[pc],
                             f"{v:.3f}", R[i][5], P[j][5], *[f"{x:.4f}" for x in P[j][1:5]]])
        for i in fn:
            stats[R[i][0]]["FN"] += 1
            mm.writerow([f, "missed_by_nn", UNIFIED_CLASSES[R[i][0]], "", "", R[i][5], "",
                         *[f"{x:.4f}" for x in R[i][1:5]]])
        for j in fp:
            stats[P[j][0]]["FP"] += 1
            mm.writerow([f, "extra_in_nn", "", UNIFIED_CLASSES[P[j][0]], "", "", P[j][5],
                         *[f"{x:.4f}" for x in P[j][1:5]]])

    rows = []
    print(f"\n{'class':15s} {'TP':>6} {'FP':>6} {'FN':>6} {'precision':>10} {'recall':>8} {'meanIoU':>8}")
    tot = {"TP": 0, "FP": 0, "FN": 0}
    for c in sorted(stats):
        s = stats[c]
        pr = s["TP"] / (s["TP"] + s["FP"]) if s["TP"] + s["FP"] else float("nan")
        rc = s["TP"] / (s["TP"] + s["FN"]) if s["TP"] + s["FN"] else float("nan")
        mi = s["iou_sum"] / s["TP"] if s["TP"] else float("nan")
        for k in tot: tot[k] += s[k]
        rows.append([UNIFIED_CLASSES[c], s["TP"], s["FP"], s["FN"], f"{pr:.3f}", f"{rc:.3f}", f"{mi:.3f}"])
        print(f"{UNIFIED_CLASSES[c]:15s} {s['TP']:>6} {s['FP']:>6} {s['FN']:>6} {pr:>10.3f} {rc:>8.3f} {mi:>8.3f}")
    P_ = tot["TP"] / max(1, tot["TP"] + tot["FP"]); R_ = tot["TP"] / max(1, tot["TP"] + tot["FN"])
    print(f"{'ALL':15s} {tot['TP']:>6} {tot['FP']:>6} {tot['FN']:>6} {P_:>10.3f} {R_:>8.3f}")
    if confusion:
        print("\nclass confusions (reference -> NN): " +
              ", ".join(f"{r}->{p}: {n}" for (r, p), n in sorted(confusion.items(), key=lambda x: -x[1])))
    with open(out / "summary.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["class", "TP", "FP", "FN", "precision", "recall", "mean_iou"])
        w.writerows(rows); w.writerow(["ALL", tot["TP"], tot["FP"], tot["FN"], f"{P_:.3f}", f"{R_:.3f}", ""])
    print(f"\nwrote {out/'summary.csv'} and {out/'mismatches.csv'}")


if __name__ == "__main__":
    main()
