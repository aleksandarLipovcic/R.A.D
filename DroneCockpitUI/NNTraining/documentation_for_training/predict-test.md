# `predict_test.py` — visual sanity check before flight

**File:** `predict_test.py`
**Run:** `python predict_test.py --weights <best.pt> --source <video_or_folder> [options]`

## Responsibility

Metrics on VisDrone/UAVDT/SARD's own val splits don't say how the model
behaves on the project's **actual** camera/altitude/lighting — none of
those three datasets is the real E5-FPV analog feed. This script runs a
checkpoint against a real test-flight video (or folder of images) and
produces output meant to be reviewed by a human before trusting the model
in the air.

## Output

- **An annotated `.mp4`** under `--out`, written **directly by this
  script**, frame-by-frame, via `cv2.VideoWriter` — deliberately **not**
  via Ultralytics' internal `save=True` path, which was silently failing
  to produce a usable video file in testing. `open_video_writer()` tries a
  fallback codec chain (`mp4v` → `XVID`/.avi → `MJPG`/.avi) since `mp4v`
  silently fails to open a working writer on some Windows OpenCV builds,
  and raises loudly (`RuntimeError`) if none of the fallbacks work,
  pointing at the likely cause (`opencv-python` and
  `opencv-python-headless` both installed and conflicting).
- **`detections.csv`** — one row per frame: total detections plus a
  per-class count and mean confidence. Described as the thing to actually
  open and scroll/plot afterward, since eyeballing a printed line per
  frame doesn't scale past a handful of images and a real test clip can be
  hundreds to thousands of frames. Frame numbers match the video's own
  frame order (starting at 0), not wall-clock time — divide by the source
  video's FPS to get time offsets.
- **A per-class aggregate summary**, printed at the end (total detections,
  how many frames that class appeared in, mean confidence) — the number to
  look at first for "is this model behaving," rather than scrolling
  per-frame output.
- **A zero-detection frame count.** On a real flight clip with people or
  vehicles continuously in view, a high zero-detection count is itself a
  signal worth investigating (missed detections, or `--conf` set too high
  for this footage) — flagged automatically in the printed output if it
  exceeds 20%.

## `--hide-classes` — historical note

Earlier versions of this project trained against a taxonomy that included
xView-derived classes (`shed`, `parking_lot`) which validated as
essentially unlearned (mAP50 ~0.001–0.016) — hidden by default at the time
so a pilot never saw a "detection" that was really just noise. xView is
now inactive and those classes aren't part of the current unified taxonomy
(see [class-map.md](class-map.md)), so **nothing is hidden by default
anymore** (`DEFAULT_HIDDEN = []`). Per the current unified-taxonomy 5-epoch
smoke-test baseline (blended val mAP50-95): person 0.215, car 0.517,
large_vehicle 0.373, motorcycle 0.154, other_vehicle 0.090 — all weak-but-
functional at this early stage, not non-functional like the old
`shed`/`parking_lot` classes were, so there's no longer a clear default
candidate to hide. If a class ends up looking genuinely broken on real
footage (visibly wrong boxes, not just low recall), pass `--hide-classes`
explicitly for that test run rather than trusting a stale default — e.g.
`--hide-classes other_vehicle`, currently the weakest class.

## CLI

```bash
python predict_test.py --weights runs/detect/yolo26s_960-4/weights/best.pt \
    --source path/to/test_flight.mp4 --conf 0.35

# Hide a specific class's output entirely:
python predict_test.py --weights best.pt --source test_flight.mp4 \
    --hide-classes other_vehicle
```

| Flag | Default | Notes |
|---|---|---|
| `--weights` | required | `best.pt` or `best.onnx` |
| `--source` | required | Folder of images, a single image, or a video — anything Ultralytics' `predict()` accepts |
| `--imgsz` | 960 | |
| `--conf` | 0.35 | Deliberately higher than training-time defaults (0.25), for an easier first read of real precision without a flood of low-confidence boxes |
| `--hide-classes` | none | See above |
| `--out` | `runs/predict_test` | Where the annotated video + `detections.csv` are written |
| `--quiet` | off | Suppresses the per-frame console line — useful for a long clip where hundreds/thousands of printed lines aren't reviewable anyway; the aggregate summary and CSV are written regardless |

## What to actually look for

Per the script's own closing guidance: false positives on background
clutter, missed obvious cars/people, and whether box tightness looks
usable for a pilot overlay (not just "technically overlapping the
object"). The CSV is the better tool for spotting patterns across a whole
clip — a stretch of frames with a confidence dip, or a class that only
fires at certain altitudes/ranges — than scrolling annotated frames one by
one.
