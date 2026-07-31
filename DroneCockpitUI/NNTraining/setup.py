"""
setup.py — Environment + dataset prep for Project R.A.D detection training
=============================================================================
Run this once before train.py. It:
  1. Installs PyTorch + torchvision (checking CUDA actually works, not just
     that the install succeeded -- a CPU-only torch install with no error
     message is the most common way this silently goes wrong).
  2. Installs ultralytics (pulls in YOLO26 + the VisDrone downloader).
  3. Pre-downloads and validates the VisDrone dataset, so train.py starts
     training immediately instead of spending its first several minutes
     downloading a 2GB archive.

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


def install_ultralytics() -> bool:
    return run([sys.executable, "-m", "pip", "install", "--upgrade",
                "ultralytics"], "Installing ultralytics (YOLO26 + dataset tooling)")


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


def main():
    print("Project R.A.D — training environment setup\n")

    cuda_ver = check_gpu_driver()
    install_pytorch(cuda_ver)

    if not install_ultralytics():
        print("\n[ERROR] ultralytics install failed. Check the pip output "
              "above -- common causes are no internet access or a Python "
              "version below 3.9 (check with `python --version`).")
        sys.exit(1)

    download_visdrone()

    print(f"\n{'=' * 70}\nSetup complete\n{'=' * 70}")
    print("Next step: python train.py --epochs 5   (smoke test first)")


if __name__ == "__main__":
    main()