"""
cudnn_check.py -- does PyTorch's cuDNN work in this Python, and which DLLs
does it really load?  (see torch_dll_fix.py for the background)

    python cudnn_check.py              # the way the training scripts start
    python cudnn_check.py --raw        # WITHOUT torch_dll_fix (shows the problem)
    python cudnn_check.py --raw --cv2-first   # cv2 imported before torch

Each run is a fresh process, so the three tell apart "PATH problem",
"cv2 loads the toolkit's cuDNN first" and "fixed".
"""
import sys

RAW = "--raw" in sys.argv
CV2_FIRST = "--cv2-first" in sys.argv

if CV2_FIRST:
    import cv2
    print(f"cv2 {cv2.__version__} from {cv2.__file__}")
if not RAW:
    import torch_dll_fix  # noqa: F401
    print(f"torch_dll_fix: removed from PATH: {torch_dll_fix.REMOVED_FROM_PATH or 'nothing'}; "
          f"cuDNN warm-up before cv2: {torch_dll_fix.WARMUP}")

import torch  # noqa: E402

print(f"torch {torch.__version__}  CUDA {torch.version.cuda}  cuDNN {torch.backends.cudnn.version()}  "
      f"GPU available: {torch.cuda.is_available()}")
if not CV2_FIRST:
    import cv2  # the training scripts import it too (via ultralytics / albumentations)
    print(f"cv2 {cv2.__version__} from {cv2.__file__}")

ok = False
if torch.cuda.is_available():
    try:
        conv = torch.nn.Conv2d(3, 16, 3).cuda()
        with torch.autocast("cuda", dtype=torch.float16):
            y = conv(torch.randn(2, 3, 128, 128, device="cuda"))
        torch.cuda.synchronize()
        ok = True
        print(f"cuDNN convolution (FP16 autocast): OK {tuple(y.shape)}")
    except RuntimeError as e:
        print(f"cuDNN convolution FAILED: {str(e).splitlines()[0]}")

from torch_dll_fix import loaded_modules  # noqa: E402  (only reads, applies nothing new)

print("\nLoaded cuDNN / CUDA runtime / NVRTC DLLs in this process:")
for p in loaded_modules("cudnn", "cudart", "nvrtc", "cublas"):
    print("  " + p)
if not torch.cuda.is_available():
    print("\nRESULT: no CUDA GPU visible to torch -- check the NVIDIA driver / torch CUDA build.")
else:
    print("\nRESULT:", "OK -- training can use the GPU." if ok else "FAILED -- send this whole output.")
