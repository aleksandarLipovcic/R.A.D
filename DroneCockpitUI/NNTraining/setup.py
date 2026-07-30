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
    Returns the driver's max-supported CUDA version string (e.g. '12.4')
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
            # Example line: "| NVIDIA-SMI 551.23   Driver Version: 551.23   CUDA Version: 12.4 |"
            try:
                cuda_ver = line.split("CUDA Version:")[1].strip().split()[0]
                print(f"[OK] GPU detected. Driver supports up to CUDA {cuda_ver}.")
                return cuda_ver
            except (IndexError, ValueError):
                pass

    print("[WARN] Could not parse CUDA version from nvidia-smi output.")
    return None


def install_pytorch(cuda_ver: str | None):
    """
    Installs torch/torchvision. Recent PyPI PyTorch builds are CUDA-enabled
    by default on Windows/Linux, so a plain install is tried first -- but
    we VERIFY cuda works afterward rather than trusting that. If it
    doesn't, fall back to an explicit CUDA-tagged index matching the
    driver's supported version.
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

    # Map driver CUDA version to a PyTorch wheel index. PyTorch doesn't
    # publish a wheel for every CUDA point release -- pick the closest
    # supported bucket at or below the driver's ceiling.
    major_minor = ".".join(cuda_ver.split(".")[:2])
    buckets = {
        "12.6": "cu126", "12.4": "cu124", "12.1": "cu121", "11.8": "cu118",
    }
    tag = buckets.get(major_minor)
    if tag is None:
        # Pick the highest bucket that doesn't exceed the driver's version
        numeric = [(float(k), v) for k, v in buckets.items()]
        eligible = [v for k, v in numeric if k <= float(major_minor)]
        tag = eligible[-1] if eligible else "cu118"

    print(f"\n[RETRY] Default install wasn't CUDA-enabled. Reinstalling "
          f"with explicit index for {tag}...")
    run([sys.executable, "-m", "pip", "install", "--upgrade",
         "--force-reinstall", "torch", "torchvision",
         "--index-url", f"https://download.pytorch.org/whl/{tag}"],
        f"pip install torch torchvision ({tag})")

    if not verify_cuda():
        print("\n[ERROR] Still CPU-only after explicit CUDA install. "
              "Check https://pytorch.org/get-started/locally/ and install "
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