# `fpv_eval.py` — accuracy on our own FPV footage

**File:** `fpv_eval.py`
**Run:** `python fpv_eval.py {extract | eval | ref} ...`
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

## Scoring against existing reference labels (`ref`)

If a flight already has reference boxes, for example the keyframe +
tracker labels in `part0_reference_boxes.csv` / `part1_reference_boxes.csv`,
there is nothing to label. `ref` runs the model on the **raw** video and
compares it frame by frame with the CSV:

```
python fpv_eval.py ref --model ../models/yolo26m_main.onnx --video rad_fpv_cropped_part0.mp4 --reference labels/part0 --every 5
```

`--reference` can be either of two things:

- **A folder of per-frame YOLO txt files** (preferred), e.g. the
  `labels/part0/part0_NNNNNN.txt` files from the reference zips.
  - Lines are normalized `cls xc yc w h`, with the class index taken from
    the model's `.names`.
  - The frame number is taken from the digits at the end of the file name:
    0-based by default, or set `--ref-frame-base 1`.
  - Every file counts as a checked frame. An empty file is a frame with
    nothing in it, so detections there count as false alarms.
- **A CSV with one row per box**, as described below.

- **Use the raw video, not the `*_reference_labeled.mp4` copy.** That copy
  has the boxes drawn into the picture.
- **Columns are recognised by their header names.** A frame column
  (`frame`, `frame_idx`, …), a class column (`class` by name, or
  `class_id`), and the box as `x1,y1,x2,y2` or `x,y,w,h` in pixels. Extra
  columns such as `conf` are ignored. Frame numbers are 0-based. If the
  header isn't recognised, the error message lists what was found.
- **Class names must match the model's `.names`.** Rename them with
  `--class-map truck=large_vehicle,bus=large_vehicle`. Classes that still
  don't match are listed as "NOT counted" rather than silently dropped.
- **`--frames labeled`** (the default) uses only frames that have
  reference boxes. Use **`--frames all`** only if the reference covers the
  whole video; frames without boxes then count as hard negatives, i.e.
  any detection there is a false alarm.
- **`--every 5`** uses every 5th frame. Neighbouring frames are almost
  identical, so using all of them only costs time.
- `--variants` and `--write-thresholds` work as in `eval`.

## Diagnosing a weak class

- **`--iou 0.3`** loosens the box-overlap rule from 0.5 to 0.3. If a class
  scores much higher this way, the model *does* find the objects and only
  the boxes are imprecise, in the model or the reference. That's typical
  for 10–20 px pedestrians, where a 3 px shift already fails IoU 0.5.
- **`--merge-vehicles`** scores person vs. a single `vehicle` class, the
  same grouping the cockpit tracker uses. If vehicles jump up this way, the
  remaining error is car ↔ van naming, not missed vehicles. It can't be
  combined with `--write-thresholds`.

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
