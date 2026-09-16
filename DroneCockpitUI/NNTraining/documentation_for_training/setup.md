# `setup.py` — environment & dataset prep

**File:** `setup.py`
**Run:** once, before `train.py` — `python setup.py` (inside a fresh venv)

## Responsibility

Six sequential checks, each idempotent — safe to re-run any time, since
every step first checks whether its work is already done:

1. **PyTorch + torchvision**, verifying CUDA actually works (`verify_cuda()`)
   rather than trusting a successful `pip install` — a CPU-only torch
   install produces no error message, which is called out as the most
   common way this silently goes wrong. `check_gpu_driver()` +
   `pick_wheel_tag()` pick the right CUDA-enabled wheel index for the
   detected driver. Once CUDA is confirmed, `print_vram_summary()` prints
   physical VRAM — `train.py`'s defaults (`imgsz 640`, `batch 4`,
   `max-mosaic-instances 800`) are all calibrated to a 6GB card, so this
   surfaces that assumption immediately rather than discovering a mismatch
   hours into a WDDM spillover.
2. **Ultralytics** (`install_ultralytics()`) — pulls in YOLO26 and the
   VisDrone downloader.
3. **Albumentations** (`install_albumentations()`) — powers `train.py`'s
   degradation-augmentation pipeline (motion/gaussian blur, compression
   artifacts, ISO noise, gamma shift) that closes some of the gap between
   clean training footage and the real analog FPV feed. `train.py` still
   runs without it (auto-skips with a warning), but this step verifies it
   up front rather than discovering the gap mid-run.
4. **VisDrone pre-download** (`download_visdrone()`) — so `train.py` starts
   training immediately instead of spending its first several minutes
   downloading a 2GB archive.
5. **UAVDT/SARD presence check** (`check_manual_datasets()`) — these two
   *cannot* be automated the way VisDrone is: Roboflow gates both behind a
   free account with no public direct-download API. This step only ever
   reports what it finds and prints the manual download/extraction steps
   if either is missing (see
   [prepare-datasets.md](prepare-datasets.md#manual-roboflow-downloads)
   for the exact links and extraction convention).
6. **Installed-version fingerprint** (`print_installed_versions()`) —
   prints the exact installed torch/ultralytics/albumentations versions.
   `train.py`'s degradation-augment and mosaic-guard monkeypatches both
   target **non-public** Ultralytics internals, which can silently stop
   applying if a future `pip install --upgrade` lands on a version where
   those internals changed shape. This is the one place in the whole
   pipeline that records what was actually verified working — if training
   results drift unexpectedly after a future re-run of this script,
   compare against what got printed here, and re-run
   [verify-pipeline-assumptions.md](verify-pipeline-assumptions.md)'s
   checks B/C (copy-paste inertness, degradation-transform build count) to
   confirm the patches still apply cleanly.

## Usage

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
source .venv/bin/activate       # Linux/WSL
python setup.py
```

## Python 3.14 CUDA wheel caveat

Documented directly in the source: CUDA-enabled PyTorch wheels for the
`cp314` tag have been inconsistent across CUDA versions at time of writing
— some package indexes publish them, some don't yet. If this script
exhausts every CUDA wheel bucket it tries and still lands on CPU-only
torch, the recommended fallback isn't to keep guessing — create a
**separate venv on Python 3.12** for this training subproject specifically,
where CUDA wheel coverage is mature:

```bash
py -3.12 -m venv .venv312
.venv312\Scripts\activate
python setup.py
```
