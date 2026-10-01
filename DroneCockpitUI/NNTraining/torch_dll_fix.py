"""
torch_dll_fix.py -- make PyTorch use ITS OWN cuDNN on Windows
=============================================================
Import this FIRST, before torch / ultralytics / cv2 / albumentations:

    import torch_dll_fix  # noqa: F401

THE PROBLEM IT FIXES
  PyTorch's Windows wheels ship their own cuDNN in site-packages\\torch\\lib
  (torch 2.13+cu130 -> cuDNN 9.20). This PC also has a DIFFERENT cuDNN in
  the CUDA 13.3 toolkit (C:\\Program Files\\NVIDIA GPU Computing Toolkit\\
  CUDA\\v13.3\\bin\\x64, installed for the OpenCV CUDA build). If any part of
  that one gets into the Python process, torch ends up with a mix of two
  versions and the first convolution fails with

      CUDNN_STATUS_SUBLIBRARY_VERSION_MISMATCH

  Two ways it gets in, both handled here:
    1. PATH: cuDNN 9 loads its sub-libraries by name, and Windows also
       searches PATH. -> torch\\lib goes first on PATH, and every other PATH
       folder holding a cuDNN DLL is dropped (this process only).
    2. Load order: a DLL that is already loaded is reused BY NAME. If cv2
       (a CUDA OpenCV build) is imported before torch, the toolkit's
       cudnn64_9.dll is already in the process when torch asks for its own.
       -> torch is imported HERE, first, so its cuDNN is the one loaded.

  Nothing on the system changes; the cockpit / OpenCV keep using the
  toolkit's cuDNN as before. No effect off Windows or without torch.

ESCAPE HATCH
  set RAD_DISABLE_CUDNN=1   -> torch.backends.cudnn.enabled = False
  (works with any DLL mix, but convolutions run noticeably slower).

DIAGNOSIS
  python cudnn_check.py   -- shows which cuDNN DLL files are really loaded.
"""
import glob
import importlib.util
import os
import sys


def _torch_lib_dir():
    spec = importlib.util.find_spec("torch")
    if spec is None or not spec.origin:
        return None
    lib = os.path.join(os.path.dirname(spec.origin), "lib")
    return lib if os.path.isdir(lib) else None


def _clean_path(lib: str) -> list:
    """torch\\lib first; folders with a foreign cuDNN removed. Returns removed."""
    keep, removed = [lib], []
    for p in os.environ.get("PATH", "").split(os.pathsep):
        if not p or os.path.normcase(os.path.normpath(p)) == os.path.normcase(os.path.normpath(lib)):
            continue
        if glob.glob(os.path.join(p, "cudnn*64_9.dll")):
            removed.append(p)
            continue
        keep.append(p)
    os.environ["PATH"] = os.pathsep.join(keep)
    return removed


def loaded_modules(*needles):
    """Full paths of DLLs loaded in this process whose name contains a needle."""
    if sys.platform != "win32":
        return []
    import ctypes
    from ctypes import wintypes
    psapi = ctypes.WinDLL("psapi")
    k32 = ctypes.WinDLL("kernel32")
    k32.GetCurrentProcess.restype = wintypes.HANDLE
    psapi.EnumProcessModules.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.HMODULE),
                                         wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    psapi.EnumProcessModules.restype = wintypes.BOOL
    k32.GetModuleFileNameW.argtypes = [wintypes.HMODULE, wintypes.LPWSTR, wintypes.DWORD]
    k32.GetModuleFileNameW.restype = wintypes.DWORD
    mods = (wintypes.HMODULE * 4096)()
    needed = wintypes.DWORD()
    if not psapi.EnumProcessModules(k32.GetCurrentProcess(), mods, ctypes.sizeof(mods),
                                    ctypes.byref(needed)):
        return []
    n = min(needed.value // ctypes.sizeof(wintypes.HMODULE), len(mods))
    buf = ctypes.create_unicode_buffer(1024)
    out = []
    for i in range(n):
        if k32.GetModuleFileNameW(mods[i], buf, 1024):
            name = os.path.basename(buf.value).lower()
            if any(s in name for s in needles):
                out.append(buf.value)
    return sorted(out)


REMOVED_FROM_PATH = []
FOREIGN_CUDNN_BEFORE_TORCH = []


def _apply() -> None:
    if sys.platform != "win32":
        return
    lib = _torch_lib_dir()
    if lib is None:
        return
    if "torch" not in sys.modules:
        REMOVED_FROM_PATH.extend(_clean_path(lib))
        try:
            os.add_dll_directory(lib)
        except (AttributeError, OSError):
            pass
        FOREIGN_CUDNN_BEFORE_TORCH.extend(
            p for p in loaded_modules("cudnn")
            if os.path.normcase(os.path.dirname(p)) != os.path.normcase(lib))
        import torch  # noqa: F401  -- load torch's own cuDNN first (see module doc)
    if os.environ.get("RAD_DISABLE_CUDNN") == "1":
        import torch
        torch.backends.cudnn.enabled = False
        print("[torch_dll_fix] RAD_DISABLE_CUDNN=1 -- cuDNN disabled (slower convolutions).")
    if FOREIGN_CUDNN_BEFORE_TORCH:
        print("[torch_dll_fix] WARNING: a foreign cuDNN was already loaded before torch:\n  "
              + "\n  ".join(FOREIGN_CUDNN_BEFORE_TORCH)
              + "\n  Something imported before torch_dll_fix loaded it -- make "
                "`import torch_dll_fix` the very first import.")


_apply()
