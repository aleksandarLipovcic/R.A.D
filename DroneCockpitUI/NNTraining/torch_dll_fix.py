"""
torch_dll_fix.py -- make PyTorch use ITS OWN cuDNN on Windows
=============================================================
Import this BEFORE torch / ultralytics (it imports neither):

    import torch_dll_fix  # noqa: F401

THE PROBLEM IT FIXES
  PyTorch's Windows wheels ship their own cuDNN in site-packages\\torch\\lib
  (torch 2.13+cu130 -> cuDNN 9.20). cuDNN 9 loads its sub-libraries
  (cudnn_engines_*, cudnn_cnn64_9, ...) by name at run time, and Windows
  then also searches PATH. The CUDA toolkit installed for the OpenCV CUDA
  build puts a DIFFERENT cuDNN on PATH
  (C:\\Program Files\\NVIDIA GPU Computing Toolkit\\CUDA\\v13.3\\bin\\x64),
  so torch ends up with a mix of two versions and the first convolution
  fails with

      CUDNN_STATUS_SUBLIBRARY_VERSION_MISMATCH

  (seen in train.py's AMP check and in eval_report.py on GPU).

  Putting torch\\lib first on PATH for THIS process only makes every
  cuDNN lookup find torch's matching set. Nothing on the system changes,
  so the cockpit / OpenCV keep using the toolkit's cuDNN as before.
  No effect on Linux/macOS or when torch isn't installed.
"""
import importlib.util
import os
import sys


def _apply() -> None:
    if sys.platform != "win32" or "torch" in sys.modules:
        return
    spec = importlib.util.find_spec("torch")
    if spec is None or not spec.origin:
        return
    lib = os.path.join(os.path.dirname(spec.origin), "lib")
    if not os.path.isdir(lib):
        return
    path = os.environ.get("PATH", "")
    if not path.lower().startswith(lib.lower() + os.pathsep):
        os.environ["PATH"] = lib + os.pathsep + path
    try:
        os.add_dll_directory(lib)
    except (AttributeError, OSError):
        pass


_apply()
