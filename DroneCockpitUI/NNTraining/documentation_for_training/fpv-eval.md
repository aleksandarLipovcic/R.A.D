# `fpv_eval.py` — accuracy on our own FPV footage

**File:** `fpv_eval.py`
**Run:** `python fpv_eval.py {extract | eval} ...`
**Needs:** `opencv-python` (or `-headless`) and `numpy`. It doesn't need PyTorch or Ultralytics.

## Responsibility

`eval_report.py` scores the model on the public datasets. The drone flies
an analog 5.8 GHz FPV camera, which none of those datasets look like, so
their mAP doesn't say how well the model does in the air. `predict_test.py`
lets a human *look* at a flight video. This script *measures* it, and turns
the measurement into settings the cockpit uses.

It runs the **same `.onnx`** that `DetectionLink` loads, with the **same
letterbox and output parsing** as `DetectionLink::runInference()` (both
ONNX layouts, see below). A number here is a number the drone gets.

## Workflow

```
flight video ──extract──▶ frames + draft labels ──(correct in a labeller)──▶ labelled set
                                                                               │
                  <model>.thresholds.json ◀──eval --write-thresholds───────────┘
                           │
                           ▼
        cockpit start-up: DetectionLink.set_class_confidence_threshold(...)
```

1. **Extract frames** from a flight recording, 2 per second at 30 fps:

   ```
   python fpv_eval.py extract --video flight1.mp4 --out datasets/fpv_eval --every 15 --prelabel ../models/yolo26m_main.onnx
   ```

   `--prelabel` writes the model's own boxes (confidence ≥ 0.25) as YOLO
   `.txt` labels, so labelling means correcting boxes, not drawing every
   one from scratch. **Correct them** in any YOLO-format labeller before
   evaluating. Otherwise the model is graded against its own answers.

   Label rules:
   - An **empty** `.txt` means "checked, nothing here". That frame is a
     hard negative and counts towards false alarms. Include frames of
     trees, rocks and shadows the model has mistaken for people.
   - An image **without** a `.txt` is treated as unlabelled and skipped,
     so a half-finished set can already be evaluated.
   - Class indices follow the `.names` file next to the model.

2. **Evaluate:**

   ```
   python fpv_eval.py eval --model ../models/yolo26m_main.onnx --data datasets/fpv_eval --variants base,deinterlace,flip,tiles
   ```

   The output is one table per variant: per class, the number of labelled
   objects, AP50 (COCO 101-point, the same as the mAP50 `train.py`
   prints), and the confidence threshold with the best F-score plus its
   precision and recall. Below each table: ms per frame, and false alarms
   (confidence ≥ 0.25) on the empty frames. Everything is saved to
   `runs/fpv_eval/<timestamp>/fpv_eval.json`.

3. **Tune the cockpit:** add `--write-thresholds`. It writes
   `<model>.thresholds.json` next to the model, from the first variant.
   The threshold for each class is the one with the best F-score on this
   footage:
   - **F2 for `person`**: recall counts double, because a missed person
     costs more than a false alarm in search and rescue.
   - **F1 for vehicles.**

   A class with fewer than `--min-gt` (default 20) labelled objects keeps
   the global threshold, since a tiny sample would give a noisy
   threshold. The cockpit loads the file at start-up and prints
   `per-class thresholds from yolo26m_main.thresholds.json: {...}`.

## Variants

| Variant | What it tests | Cost |
|---|---|---|
| `base` | Exactly what the drone does today | 1× |
| `deinterlace` | Analog video is interlaced, and motion leaves comb artefacts. This variant keeps the even field lines and interpolates the odd ones | ~1× |
| `flip` | Test-time augmentation: frame + mirrored frame, merged with NMS | 2× |
| `tiles` | Full frame + 2×2 overlapping tiles (60 % of the frame each), merged. Small, distant objects are enlarged, which is the SAR case. Boxes cut by an inner tile edge are dropped, because the full frame has them whole | 5× |

A variant is only worth moving into `DetectionLink` (C++) if it's measured
to help on our footage **and** its cost fits the detection interval. The
ms/frame here is CPU with pip's OpenCV unless `--cuda` is given, so
compare the variants relative to each other, not as absolute flight
numbers.

## ONNX layouts

`fpv_eval.py` and `DetectionLink::runInference()` both accept:

- **End-to-end** `[1, 300, 6]`: YOLO26's default export
  (`export_model.py`, `train.py --export-only`). NMS runs inside the
  model.
- **Raw** `[1, 4+classes, anchors]`: an `end2end=False` export. Class
  argmax, then class-aware NMS at IoU 0.7 with a maximum of 300 boxes
  (Ultralytics' `predict()` defaults).

Tested on `yolo26n` exported both ways: both layouts gave identical boxes
and identical AP.
