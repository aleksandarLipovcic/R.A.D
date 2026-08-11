"""
setup.py — Environment + dataset prep for Project R.A.D detection training
=============================================================================
Run this once before train.py. It:
  1. Installs PyTorch + torchvision (checking CUDA actually works, not just
     that the install succeeded -- a CPU-only torch install with no error
     message is the most common way this silently goes wrong). Also prints
     physical VRAM once CUDA is confirmed -- train.py's defaults (imgsz 640,
     batch 4, max-mosaic-instances 800) are all calibrated to a 6GB card, so
     this surfaces that assumption up front instead of only finding out via
     a WDDM spillover hours into a real run.
  2. Installs ultralytics (pulls in YOLO26 + the VisDrone downloader).
  3. Installs albumentations (new) -- powers train.py's degradation
     augmentation pipeline (motion/gaussian blur, compression artifacts,
     ISO noise, gamma shift) that closes some of the gap between the
     clean training datasets and the real analog FPV feed. train.py
     still runs fine without it (auto-skips with a warning), but this
     step means you don't find that out mid-run -- it's verified here,
     up front, same as CUDA is.
  4. Pre-downloads and validates the VisDrone dataset, so train.py starts
     training immediately instead of spending its first several minutes
     downloading a 2GB archive.
  5. Checks whether UAVDT/SARD are present under datasets/ (new). These
     two CANNOT be automated the way VisDrone is -- Roboflow gates both
     behind a free account with no public direct-download API, so this
     step only ever reports what it finds and prints the manual steps
     if either is missing. See prepare_datasets.py's module docstring
     for the exact download links and extraction convention.
  6. Prints the exact installed versions of torch/ultralytics/
     albumentations at the end (new). train.py's degradation-augment and
     mosaic-guard monkeypatches both target non-public Ultralytics
     internals (see train.py's docstring) -- they can silently stop
     applying if a future `pip install --upgrade` (from re-running this
     script, or anyone else on this project running it fresh) lands on a
     version where those internals changed shape. Nothing else in this
     pipeline records what was actually verified working, so this is
     the one place that does -- if training results drift unexpectedly
     after a future setup.py re-run, compare against what got printed
     here, and re-run verify_pipeline_assumptions.py's checks B/C
     (copy-paste inertness, degradation transform build count) to
     confirm the patches still apply cleanly.

Usage:
    python -m venv .venv
    .venv\\Scripts\\activate          (Windows)
    source .venv/bin/activate         (Linux/WSL)
    python setup.py

Re-running is safe -- each step checks whether it's already done before
doing it again.

Note on Python 3.14: CUDA-enabled PyTorch wheels for the cp314 tag have
been inconsistent across CUDA versions (some indexes publish them, some
don't yet). If this script exhausts every CUDA bucket below and is still
CPU-only, the fallback isn't to keep guessing -- create a separate venv
on Python 3.12 for this training subproject specifically, where CUDA
wheel coverage is mature and this stops being a moving target:
    py -3.12 -m venv .venv312
    .venv312\\Scripts\\activate
    python setup.py
"""

import subprocess
import sys
import shutil
from pathlib import Path


def run(cmd: list[str], description: str) -> bool:
    print(f"\n{'=' * 70}\n{description}\n{'=' * 70}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"\n[FAILED] {description} (exit code {result.returncode})")
        return False
    return True


def check_gpu_driver() -> str | None:
    """
    Returns the driver's max-supported CUDA version string (e.g. '13.3')
    by parsing `nvidia-smi`, or None if nvidia-smi isn't found/parseable.
    This is the DRIVER's ceiling, not necessarily what gets installed --
    it just tells us it's safe to install a CUDA build up to this version.
    """
    if shutil.which("nvidia-smi") is None:
        print("[WARN] nvidia-smi not found on PATH. Either no NVIDIA GPU, "
              "or drivers aren't installed. Training will fall back to CPU "
              "and be dramatically slower -- fix this before training a "
              "full run, not after.")
        return None

    try:
        out = subprocess.run(["nvidia-smi"], capture_output=True, text=True,
                              check=True).stdout
    except subprocess.CalledProcessError:
        print("[WARN] nvidia-smi ran but returned an error.")
        return None

    for line in out.splitlines():
        if "CUDA Version" in line:
            # Example line: "| NVIDIA-SMI 610.74  ... CUDA Version: 13.3 |"
            # Also handles the newer nvidia-smi layout that prints
            # "CUDA UMD Version: 13.3" on its own header line.
            try:
                cuda_ver = line.split("CUDA Version:")[1].strip().split()[0].rstrip("|").strip()
                print(f"[OK] GPU detected. Driver supports up to CUDA {cuda_ver}.")
                return cuda_ver
            except (IndexError, ValueError):
                pass
        if "CUDA UMD Version" in line:
            try:
                cuda_ver = line.split("CUDA UMD Version:")[1].strip().split()[0].rstrip("|").strip()
                print(f"[OK] GPU detected. Driver supports up to CUDA {cuda_ver}.")
                return cuda_ver
            except (IndexError, ValueError):
                pass

    print("[WARN] Could not parse CUDA version from nvidia-smi output.")
    return None


# PyTorch wheel index tags, newest first. Kept as an ordered list (not just
# a dict) so the "pick highest eligible" fallback logic below is a single
# linear scan instead of float-parsing version strings, which breaks on
# CUDA's inconsistent point-release numbering (12.6 vs 13.0 vs 13.3, etc).
CUDA_WHEEL_BUCKETS = [
    ("13.0", "cu130"),
    ("12.8", "cu128"),
    ("12.6", "cu126"),
    ("12.4", "cu124"),
    ("12.1", "cu121"),
    ("11.8", "cu118"),
]


def pick_wheel_tag(cuda_ver: str) -> str:
    """
    Maps a driver's reported CUDA ceiling to the closest PyTorch wheel
    index tag at or below it. E.g. a 13.3 driver -> cu130 (PyTorch doesn't
    publish a cu133 wheel, but cu130 wheels run fine on newer drivers --
    CUDA is backward compatible that direction).
    """
    try:
        driver_version = float(".".join(cuda_ver.split(".")[:2]))
    except ValueError:
        print(f"[WARN] Couldn't parse '{cuda_ver}' as a version number, "
              f"defaulting to cu126 (safe, widely-supported baseline).")
        return "cu126"

    for bucket_ver, tag in CUDA_WHEEL_BUCKETS:
        if driver_version >= float(bucket_ver):
            return tag

    # Driver is older than every known bucket ceiling -- go with the oldest.
    return CUDA_WHEEL_BUCKETS[-1][1]


def install_pytorch(cuda_ver: str | None):
    """
    Installs torch/torchvision. Recent PyPI PyTorch builds are CUDA-enabled
    by default on Linux, but Windows plain installs commonly land on a
    CPU-only build -- so a plain install is tried first, but we VERIFY cuda
    works afterward rather than trusting that. If it doesn't, fall back to
    an explicit CUDA-tagged index matching the driver's supported version.
    """
    print(f"\n{'=' * 70}\nInstalling PyTorch\n{'=' * 70}")
    run([sys.executable, "-m", "pip", "install", "--upgrade",
         "torch", "torchvision"], "pip install torch torchvision")

    if verify_cuda():
        print_vram_summary()
        return

    if cuda_ver is None:
        print("\n[ERROR] torch.cuda.is_available() is False, and no GPU "
              "driver was detected. If you do have an RTX 3060, install "
              "NVIDIA drivers first, then re-run this script.")
        return

    tag = pick_wheel_tag(cuda_ver)
    print(f"\n[RETRY] Default install wasn't CUDA-enabled. Reinstalling "
          f"with explicit index for {tag}...")
    run([sys.executable, "-m", "pip", "install", "--upgrade",
         "--force-reinstall", "torch", "torchvision",
         "--index-url", f"https://download.pytorch.org/whl/{tag}"],
        f"pip install torch torchvision ({tag})")

    if verify_cuda():
        print_vram_summary()
        return

    # Still CPU-only after an explicit CUDA index install. On Python 3.9-3.13
    # this would almost always be a wheel/driver mismatch worth troubleshooting
    # further. On Python 3.14 it's very likely that cp314 CUDA wheels simply
    # aren't published yet for this CUDA bucket -- so say that plainly instead
    # of sending the person in circles re-trying pip installs.
    py_ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    print(f"\n[ERROR] Still CPU-only after explicit CUDA install ({tag}, "
          f"Python {py_ver}).")
    if sys.version_info[:2] >= (3, 14):
        print(
            "You're on Python 3.14 -- CUDA wheel coverage for this Python "
            "version has been inconsistent across CUDA releases. Rather "
            "than keep guessing wheel tags, create a separate venv on "
            "Python 3.12 just for this training subproject:\n"
            "    py -3.12 -m venv .venv312\n"
            "    .venv312\\Scripts\\activate\n"
            "    python setup.py\n"
            "Python 3.12 has mature, stable CUDA wheel coverage and removes "
            "this as a variable."
        )
    else:
        print("Check https://pytorch.org/get-started/locally/ and install "
              "manually with the exact command it gives you for your "
              "driver version.")


def verify_cuda() -> bool:
    check = subprocess.run(
        [sys.executable, "-c",
         "import torch; print(torch.cuda.is_available())"],
        capture_output=True, text=True)
    available = check.stdout.strip() == "True"
    if available:
        name_check = subprocess.run(
            [sys.executable, "-c",
             "import torch; print(torch.cuda.get_device_name(0))"],
            capture_output=True, text=True)
        print(f"[OK] CUDA available. GPU: {name_check.stdout.strip()}")
    else:
        print("[WARN] torch.cuda.is_available() returned False.")
    return available


def print_vram_summary() -> None:
    """
    Prints physical VRAM once CUDA is confirmed working (new). Every
    tuned default in train.py -- --imgsz 640, --batch 4,
    --max-mosaic-instances 800, the whole WDDM startup-probe/watchdog
    setup -- is calibrated specifically against a 6GB card. This doesn't
    change any of those defaults, it just surfaces the assumption at
    setup time instead of letting it stay implicit until a WDDM spillover
    shows up hours into a real run on a different card.
    """
    check = subprocess.run(
        [sys.executable, "-c",
         "import torch; print(torch.cuda.get_device_properties(0).total_memory)"],
        capture_output=True, text=True)
    raw = check.stdout.strip()
    if not raw.isdigit():
        print("[WARN] Could not read VRAM size from torch -- skipping "
              "VRAM sanity check.")
        return
    gb = int(raw) / 1e9
    print(f"[OK] Physical VRAM: {gb:.1f}GB")
    if gb < 5.5:
        print("[WARN] Under 5.5GB -- train.py's defaults assume a 6GB "
              "card (imgsz=640, batch=4, max-mosaic-instances=800). "
              "Consider lowering --max-mosaic-instances and/or --batch "
              "for your first smoke test.")
    elif gb > 8.5:
        print("[note] Comfortably above the 6GB card train.py's defaults "
              "target -- you likely have headroom to raise --batch, "
              "--imgsz, or try --model yolo26m.pt / --model auto, once a "
              "640px yolo26s baseline is confirmed working. See train.py's "
              "module docstring for the recommended order to try these in.")


def install_ultralytics() -> bool:
    return run([sys.executable, "-m", "pip", "install", "--upgrade",
                "ultralytics"], "Installing ultralytics (YOLO26 + dataset tooling)")


def install_albumentations() -> bool:
    """
    Powers train.py's install_degradation_augment() -- the analog-FPV-feed
    degradation pipeline (motion/gaussian blur, compression artifacts,
    ISO noise, gamma shift). NON-FATAL if this fails: train.py checks for
    albumentations itself and skips the degradation augmentation with a
    printed warning rather than crashing, so a failure here doesn't block
    setup.py from finishing -- but verifying it now means you find out
    here, not three hours into a training run's log output.
    """
    ok = run([sys.executable, "-m", "pip", "install", "--upgrade",
              "albumentations"], "Installing albumentations (degradation "
             "augmentation for analog-feed robustness)")
    if not ok:
        print("[WARN] albumentations install failed -- train.py's "
              "degradation augmentation will auto-skip (non-fatal, "
              "training still runs). Try `pip install albumentations` "
              "manually, or pass --no-degradation-aug to train.py to "
              "silence the warning if you're skipping it intentionally.")
        return False

    check = subprocess.run(
        [sys.executable, "-c", "import albumentations; print('OK')"],
        capture_output=True, text=True)
    if check.stdout.strip() == "OK":
        print("[OK] albumentations importable -- degradation augmentation "
              "will be active by default in train.py.")
        return True
    print(f"[WARN] albumentations installed but failed to import: "
          f"{check.stderr.strip()}. train.py will auto-skip degradation "
          f"augmentation with a warning -- not fatal, but worth "
          f"investigating before a long run if you want that feature.")
    return False


def download_visdrone():
    print(f"\n{'=' * 70}\nDownloading and validating VisDrone dataset\n{'=' * 70}")
    script = (
        "try:\n"
        "    from ultralytics.data.utils import check_det_dataset\n"
        "    data = check_det_dataset('VisDrone.yaml')\n"
        "    print('[OK] VisDrone ready at:', data.get('path', data))\n"
        "except Exception as e:\n"
        "    print('[WARN] check_det_dataset failed:', e)\n"
        "    print('Falling back to a 1-epoch dry run to trigger the '\n"
        "          'standard download path instead...')\n"
        "    from ultralytics import YOLO\n"
        "    m = YOLO('yolo26n.pt')\n"
        "    m.train(data='VisDrone.yaml', epochs=1, imgsz=640, device='cpu')\n"
        "    print('[OK] VisDrone downloaded via dry-run training pass.')\n"
    )
    result = subprocess.run([sys.executable, "-c", script])
    if result.returncode != 0:
        print("[FAILED] Dataset download did not complete cleanly. "
              "train.py will retry the download automatically on first "
              "run, but check your internet connection / disk space if "
              "it fails there too.")


def _resolve_datasets_dir() -> Path:
    """
    Mirrors prepare_datasets.py's own datasets_dir resolution exactly --
    Ultralytics' SETTINGS first, falling back to a script-relative
    datasets/ folder (NOT the current working directory) if SETTINGS
    isn't importable. Previously this fell back to Path.cwd() / "datasets"
    instead, which only agrees with prepare_datasets.py's fallback when
    setup.py happens to be invoked from the project root -- anchoring to
    __file__ instead removes that dependency on invocation location.
    """
    try:
        from ultralytics.utils import SETTINGS
        return Path(SETTINGS.get("datasets_dir", Path(__file__).resolve().parent / "datasets"))
    except Exception:
        return Path(__file__).resolve().parent / "datasets"


def check_manual_datasets():
    """
    UAVDT and SARD (new check) can't be pre-downloaded here the way
    VisDrone is -- both are gated behind a free Roboflow account with no
    public direct-download API, so there's no URL this script can safely
    curl. What this CAN do is report whether they're already present in
    the expected location and, for anything missing, print the exact
    manual steps rather than leaving you to dig through
    prepare_datasets.py's docstring to find them again.

    Uses _resolve_datasets_dir(), the same datasets_dir resolution
    prepare_datasets.py uses (falls back to a local datasets/ folder
    anchored to this script's own location, not the current working
    directory, if ultralytics' SETTINGS isn't importable yet -- e.g. if
    ultralytics install failed earlier in this script).
    """
    datasets_dir = _resolve_datasets_dir()

    print(f"\n{'=' * 70}\nChecking UAVDT / SARD (manual download required)\n{'=' * 70}")

    targets = {
        "UAVDT": (datasets_dir / "UAVDT",
                  "https://universe.roboflow.com/kfupm-v0syf/uavdt-4g4uv"),
        "SARD": (datasets_dir / "SARD",
                 "https://universe.roboflow.com/animesh-shastry/sard_yolo"
                 "  (pick version \"v1 Original\" specifically -- other "
                 "versions on that project are grayscale/resized/"
                 "augmented)"),
    }

    missing = []
    for name, (root, url) in targets.items():
        if (root / "data.yaml").exists():
            print(f"  [OK] {name} found at {root}")
        else:
            print(f"  [MISSING] {name} not found at {root}")
            missing.append((name, root, url))

    if missing:
        print(f"\nManual step required for: "
              f"{', '.join(n for n, _, _ in missing)}")
        for name, root, url in missing:
            print(f"\n  {name}:")
            print(f"    1. Sign in and download from: {url}")
            print(f"    2. Export format: YOLOv8, raw/1x (skip Roboflow's "
                  f"own augmentation multiplier)")
            print(f"    3. Extract so data.yaml sits directly at {root}\\ "
                  f"(not nested in an extra wrapper folder)")
        print(f"\ntrain.py will still run without these -- VisDrone alone "
              f"is enough to train -- but the unified taxonomy is built "
              f"for VisDrone+UAVDT+SARD together, so accuracy on vehicle "
              f"diversity (UAVDT) and SAR-specific person poses (SARD) "
              f"will be limited until both are in place.")
    else:
        print("\n[OK] Both UAVDT and SARD are present -- prepare_datasets.py "
              "will pick them up automatically on the next train.py run.")


def print_installed_versions():
    """
    Records what actually got installed (new). train.py's degradation-
    augment and mosaic-guard patches both target non-public Ultralytics
    internals -- if a future `pip install --upgrade` (from re-running
    this script) lands on a version where those internals changed shape,
    the patch can silently stop applying with no error, and nothing else
    in this pipeline records what was verified working at the time a
    training run's results were produced. This is the one place that
    does. If results drift unexpectedly after a future setup.py re-run,
    compare against what's printed here, and re-run
    verify_pipeline_assumptions.py's checks B/C to confirm the patches
    still apply cleanly on whatever versions are now installed.
    """
    print(f"\n{'=' * 70}\nInstalled versions (record these -- compare against\n"
          f"this if training results drift after a future setup.py re-run)\n"
          f"{'=' * 70}")
    for pkg in ("torch", "ultralytics", "albumentations"):
        check = subprocess.run(
            [sys.executable, "-c", f"import {pkg}; print({pkg}.__version__)"],
            capture_output=True, text=True)
        ver = check.stdout.strip() or "not installed"
        print(f"  {pkg:15s} {ver}")
    print(f"\nAfter any future re-run of this script, it's worth re-running:\n"
          f"    python verify_pipeline_assumptions.py\n"
          f"specifically checks B (copy-paste inertness) and C (degradation "
          f"transform build count) -- both depend on non-public Ultralytics/"
          f"albumentations internals that a version bump could silently "
          f"change.")


def main():
    print("Project R.A.D — training environment setup\n")

    cuda_ver = check_gpu_driver()
    install_pytorch(cuda_ver)

    if not install_ultralytics():
        print("\n[ERROR] ultralytics install failed. Check the pip output "
              "above -- common causes are no internet access or a Python "
              "version below 3.9 (check with `python --version`).")
        sys.exit(1)

    install_albumentations()

    download_visdrone()
    check_manual_datasets()
    print_installed_versions()

    print(f"\n{'=' * 70}\nSetup complete\n{'=' * 70}")
    print("Next step: python train.py --epochs 5   (smoke test first)")


if __name__ == "__main__":
    main()