"""
export_nn_boxes.py -- write every NN detection of a video into ONE csv file
(frame, class, confidence, box) so it can be compared with the reference labels.

Two ways to use it (run from the NNTraining folder):

A) Run the model on the video (CPU, same speed as predict_test.py):
   python export_nn_boxes.py --weights runs\\detect\\yolom_main_run\\weights\\best.pt ^
       --source ..\\datasets\\test\\rad_fpv_part0.mp4 --out nn_boxes_part0.csv

B) If you already ran `yolo predict ... save_txt=True save_conf=True`, just
   convert that labels folder (takes a second, no model needed):
   python export_nn_boxes.py --labels-dir runs\\predict_test\\part0_labels\\labels ^
       --source ..\\datasets\\test\\rad_fpv_part0.mp4 --out nn_boxes_part0.csv

Frame numbers in the csv are 0-based (first frame = 0), same as the reference.
"""
import torch_dll_fix  # noqa: F401 -- before torch: use torch's own cuDNN (see module)
import argparse, csv, re
from pathlib import Path

import cv2


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True, help="the video file")
    p.add_argument("--out", required=True, help="output csv")
    p.add_argument("--weights", help="best.pt (mode A)")
    p.add_argument("--labels-dir", help="folder of yolo txt labels (mode B)")
    p.add_argument("--imgsz", type=int, default=960)
    p.add_argument("--conf", type=float, default=0.10,
                   help="keep boxes down to this confidence; filter later")
    p.add_argument("--device", default="cpu")
    a = p.parse_args()

    cap = cv2.VideoCapture(a.source)
    W, H = int(cap.get(3)), int(cap.get(4))
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()

    rows = []
    if a.labels_dir:
        names = ["person", "car", "large_vehicle", "motorcycle", "other_vehicle"]
        for f in Path(a.labels_dir).glob("*.txt"):
            frame = int(re.findall(r"\d+", f.stem)[-1]) - 1  # ultralytics video labels are 1-based
            for line in f.read_text().split("\n"):
                t = line.split()
                if len(t) < 6:
                    continue
                c = int(float(t[0])); xc, yc, w, h, cf = map(float, t[1:6])
                rows.append([frame, c, names[c], cf, (xc - w / 2) * W, (yc - h / 2) * H,
                             (xc + w / 2) * W, (yc + h / 2) * H])
    else:
        from ultralytics import YOLO
        model = YOLO(a.weights)
        for i, r in enumerate(model.predict(source=a.source, imgsz=a.imgsz, conf=a.conf,
                                             device=a.device, stream=True, verbose=False)):
            for c, cf, (x1, y1, x2, y2) in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(),
                                               r.boxes.xyxy.tolist()):
                rows.append([i, int(c), model.names[int(c)], cf, x1, y1, x2, y2])
            if i % 250 == 0:
                print(f"frame {i}/{n_frames}", flush=True)

    rows.sort(key=lambda r: (r[0], -r[3]))
    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "class_id", "class", "conf", "x1", "y1", "x2", "y2"])
        for r in rows:
            w.writerow([r[0], r[1], r[2], f"{r[3]:.4f}"] + [f"{v:.1f}" for v in r[4:]])
    print(f"wrote {len(rows)} boxes from {len({r[0] for r in rows})} frames -> {a.out}")


if __name__ == "__main__":
    main()