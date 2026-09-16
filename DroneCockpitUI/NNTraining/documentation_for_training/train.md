# `train.py` — YOLO26 fine-tuning

**File:** `train.py`
**Run:** `python train.py [options]`
**Depends on:** `prepare_datasets.py` (called automatically at startup),
`class_map.py` (`UNIFIED_CLASSES`), `mosaic_guard.py`
(`install_instance_cap`)

## Responsibility

The actual training run: fine-tunes a YOLO26 checkpoint on the unified
taxonomy (person/car/large_vehicle/motorcycle/other_vehicle) merged from
VisDrone + UAVDT + SARD, with a set of defenses layered on top that exist
specifically because this project targets **a 6GB laptop RTX 3060**, not a
datacenter GPU.

## Model size selection

`--model` defaults to `yolo26s.pt` — fast, known-fitting, good for
validating the pipeline itself. `--model auto` re-enables a
`yolo26m → yolo26s → yolo26n` fallback search
(`MODEL_SIZE_CANDIDATES`), each candidate checked against the VRAM probes
below before being accepted. `yolo26m @ 960px` is described as the real
target once a `yolo26s` smoke test looks right.

## WDDM / VRAM-spillover guards

On Windows, an over-budget CUDA allocation **silently spills into system
RAM** instead of raising an OOM error — causing catastrophic slowdowns
well after training already looked healthy. Two callbacks guard this, each
raising `VRAMBudgetExceeded` (behaving like a real OOM the rest of the
script already knows how to handle):

- **`make_startup_probe_callback()`** — checked over the first few
  batches, before committing to a full run.
- **`make_ongoing_watchdog_callback()`** — checked every epoch, for the
  whole run, since spillover can develop gradually well into training
  (this project's own logs showed spillover developing around batch
  ~2800 in one run — see [vram-diagnostic.md](vram-diagnostic.md)).

Both check `max_memory_allocated()` **and** `max_memory_reserved()`
against `--vram-safety-margin` (default 0.90). The reserved-memory check
was added after a real incident, documented in the source: on a `--batch 3`
run, reserved VRAM (`GPU_mem` in the Ultralytics progress bar — the
allocator's pool, and the closer proxy for what actually drives WDDM's
page-out decision) hit 94.6% while *allocated* memory stayed under the
trip line, and the run oscillated between 0.4 and 2.4 it/s across epochs
(a textbook spillover "staircase") for **10.7 hours** without either probe
firing under the allocated-only check. `mosaic_guard.py`'s instance cap
(see [mosaic-guard.md](mosaic-guard.md)) is the *proactive* half of this
same defense — dense Mosaic composites were the confirmed root cause of
the staircase this project hit twice.

## Disk cache drive redirect

`--cache disk` caches decoded images between epochs as `.npy` files, but
Ultralytics writes those files **next to the original source images** —
i.e. onto whatever drive the dataset lives on, with no built-in way to
redirect them (an open, unimplemented Ultralytics feature request,
[#18285](https://github.com/ultralytics/ultralytics/issues/18285)). On a
laptop where the dataset drive is nearly full but another mounted drive has
room, this silently **disables caching entirely** rather than using
available space elsewhere.

`install_disk_cache_redirect()` retargets the `.npy` cache files onto
whichever mounted drive currently has the most free space
(`_windows_drive_roots()`/`_pick_best_cache_drive()`) — applied **before**
Ultralytics' own disk-space check runs, so that check still makes the real
pass/fail decision, just against a drive that actually has room.

## Degradation augmentation

Training sources (VisDrone/UAVDT/SARD) are all clean digital footage; the
deployment camera (E5-FPV) is analog CVBS — 1500TVL, NTSC/PAL, 0.0001 lux
minimum, a 6mm/100° lens, over a 5.8GHz analog link. That's a real
train/deploy domain gap. `install_degradation_augment()` monkeypatches
Ultralytics' `Albumentations` wrapper (**train split only**) with
`build_degradation_transform()`'s pipeline, calibrated against that exact
datasheet instead of generic noise:

| Transform | Calibrated for |
|---|---|
| `Downscale` | 1500TVL analog bandwidth ceiling |
| `MotionBlur` / `GaussianBlur` (one-of) | Low-light shutter speed + analog softening |
| `ISONoise` / `GaussNoise` (built independently, combined if either succeeds) | 0.0001 lux sensor gain grain |
| `RandomGamma` | Dusk-to-daylight SAR operating range |
| `RandomBrightnessContrast` | General exposure variation |
| `CoarseDropout` | RF dropout burst — deliberately small (≤3 holes, 4×10px each) and rare (p=0.08) |
| `OpticalDistortion` | 6mm/100° lens barrel distortion — deliberately weak (±0.03, about a third of albumentations' own default) |
| `ImageCompression` | Generic downstream compression-artifact proxy |
| `Lambda(_interlace_combing)` | NTSC/PAL interlace combing, 1px, low-probability by design |

`CoarseDropout`, `OpticalDistortion`, and the interlace `Lambda` are
**image-only** (no `bbox_params`), so in principle they could displace a
labeled object by a pixel or two without moving its box — the doc comment
notes this was a deliberate, bounded risk (parameters kept weak
specifically to limit it) rather than an oversight, since a bbox-aware
version would need segmentation-free box-remapping logic this project
doesn't have. Worth re-examining if per-class mAP for tiny/rare classes
looks *worse* with degradation aug on than off — an empirical question,
not a known bug.

### Cross-version albumentations handling

Five of the nine transforms have different parameter names between
albumentations 1.x and 2.x, and some 1.x names are **silently
accepted-and-ignored** under 2.x rather than erroring — worst case for
`CoarseDropout`, whose 2.x default hole size is large enough to blank out
a tiny labeled person entirely. `_try_build_transform()` tries
version-correct kwargs directly, with a warning-as-error check as a second
line of defense against a silent parameter mismatch. `OpticalDistortion`
specifically went through **three** distinct signatures across
albumentations releases (pre-2.0, 2.0.x, 2.2+) — documented as a fixed bug:
an installed 2.0.8 matched neither of the two candidates originally coded
(pre-2.0 and 2.2+), so it was silently skipped every run until a third,
2.0.x-correct candidate was added.

Any transform that fails to build for the installed library version is
skipped individually rather than failing the whole pipeline — training
still proceeds with fewer than 9 pieces, with a console message reporting
how many actually built (`verify_pipeline_assumptions.py` check C
re-verifies this build count independently — see
[verify-pipeline-assumptions.md](verify-pipeline-assumptions.md)).

## Resume & taxonomy safety

- `_is_resumable_checkpoint()` — a **completed** run's `last.pt` cannot be
  resumed (Ultralytics itself would error confusingly); this check catches
  that case with a clear message instead.
- `_verify_resume_taxonomy()` — checks a completed **or** in-progress
  run's class list against the current `UNIFIED_CLASSES` before resuming,
  since resuming into a taxonomy that's since changed (a `class_map.py`
  edit) would silently corrupt training.
- `last.pt`/`best.pt` are overwritten every epoch, so `--resume` loses at
  most one in-progress epoch; `--save-period` adds numbered snapshots for
  more granular rollback.

## Export for the C++ backend

`export_for_cpp(weights_path)` is the actual bridge between this training
subproject and the cockpit's `DetectionLink` (see
[modules/detectionlink.md](../modules/detectionlink.md)):

```python
model.export(format="onnx", opset=17, simplify=True, end2end=False, nms=False)
```

`end2end=False` is deliberate — it keeps the ONNX output shape as
`(1, nc+4, N)`, the raw one-to-many head output `DetectionLink::runInference()`
parses via manual class-argmax + `cv::dnn::NMSBoxes` on the C++ side (NMS
is **not** baked into the graph). A sibling `.names` file is written next
to the `.onnx`, one class name per line in index order — this is exactly
the file `DetectionLink::setModelPath()` auto-loads for class labels (see
[modules/detectionlink.md](../modules/detectionlink.md#configuration-surface-call-before-start-most-are-safe-at-runtime)),
and the file `DroneCockpitUI.py`'s `set_known_object_width()` class names
must match exactly (see
[frontend/app-shell.md](../frontend/app-shell.md)).

## Evaluation

- **`check_overfitting()`** — reads `results.csv`, flags the textbook
  aggregate signature: val box-loss rising over the last N epochs while
  train box-loss keeps falling. Heuristic and aggregate-only.
- **`report_per_class_map()`** — runs `model.val()` on the full blended val
  set and prints mAP50-95 **per class** — the direct check for "one class
  detected well, another degraded" that a single averaged number hides
  (e.g. `person`, the SAR-priority class, quietly trailing the vehicle
  classes). Explicitly passes `batch`/`workers` rather than Ultralytics'
  own larger defaults and clears the CUDA cache first — both fixes for
  real crashes hit on this 6GB/Windows setup (an unset batch size OOM'd;
  nonzero dataloader workers here, unlike every `model.train()` call,
  reloaded the full CUDA DLL stack in a subprocess and exhausted the
  paging file).
- **`evaluate_per_source()`** — runs a **separate** `model.val()` against
  each source's own val set individually (from
  `datasets/per_source_val.json`, written by `prepare_datasets.py`) — the
  only way to tell whether combining the datasets actually helped, since a
  blended val number can hide one source regressing while another
  improves. Its own docstring warns against over-reading a near-zero
  per-class row for a source that structurally never labels that class
  (UAVDT/person, SARD/vehicles) — that's expected, not a regression.

## CLI usage

```bash
python train.py                             # yolo26s, unified taxonomy
python train.py --model auto                # yolo26m->s->n fallback search
python train.py --model yolo26s.pt --imgsz 640 --batch -1 --epochs 2
python train.py --imgsz 960 --batch 1 --epochs 5   # smoke test at target imgsz
python train.py --imgsz 640 --batch 4        # fallback if 960px trips WDDM probes
python train.py --model yolo26m.pt --imgsz 960     # real target run
python train.py --data VisDrone.yaml --model yolo26n.pt --imgsz 640
python train.py --resume [--name RUN_NAME]
python train.py --export-only [--weights PATH]
```

## Baseline history (maintained in the source, updated after each real run)

| Config | Result |
|---|---|
| yolo26n, 640px, VisDrone-10cls | mAP50 ~0.31 (plateaued) |
| yolo26s, 960px, VisDrone-10cls | mAP50 ~0.465 (every class up) |
| yolo26m, 960px, unified | never fit (WDDM spillover) |
| yolo26s, 960px, unified+xView | WDDM spillover at epoch 2 |
| yolo26m, 640px, unified, 3-epoch smoke | mAP50 0.380 (**stale** — predates the pseudo-label `--apply` merge) |
| yolo26s, 640px, unified, post-mosaic_guard | ~2GB VRAM, full-run mAP TBD |
| yolo26m, 960px, unified, 5-epoch smoke (2026-08-19) | mAP50 0.576, mAP50-95 0.336 — all 8/9 degradation transforms now build cleanly, including the previously-skipped OpticalDistortion |
