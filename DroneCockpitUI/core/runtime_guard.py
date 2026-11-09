"""
runtime_guard.py -- process-level protections for a real-time ground station

Called once from DroneCockpitUI.py at start-up. Everything is best-effort and
a no-op outside Windows, so tests and other platforms are unaffected.

  single_instance()   a second cockpit would fight the first one for the
                      Pocket's COM port and the camera; it is refused.
  keep_awake()        Windows must not sleep, turn the display off or
                      suspend USB while the cockpit runs (USB selective
                      suspend would drop the Pocket's port mid-flight).
  timer_resolution()  1 ms system timer: Sleep()/waits in the C++ threads
                      (MSP poll 10 ms, serial timeouts) otherwise round up
                      to 15.6 ms.
  raise_priority()    ABOVE_NORMAL process priority: video capture and the
                      radio link keep their share of the CPU even when other
                      programs (browser, updates, antivirus) get busy.
  install_crash_logging()
                      every unhandled exception (main thread, worker
                      threads, Tk callbacks) and hard crashes in native code
                      (faulthandler) go to logs/crash_YYYYMMDD.log with a
                      timestamp; a counter lets the safety monitor raise an
                      alarm.
  alarm_sound(level)  non-blocking system sound for the safety monitor.
"""
from __future__ import annotations

import faulthandler
import os
import sys
import threading
import time
import traceback

_IS_WIN = sys.platform == "win32"
_mutex_handle = None
_crash_file = None


class ErrorCounter:
    """Thread-safe count of logged exceptions (UI + worker threads)."""

    def __init__(self) -> None:
        self._n = 0
        self._lock = threading.Lock()

    def bump(self) -> None:
        with self._lock:
            self._n += 1

    @property
    def count(self) -> int:
        with self._lock:
            return self._n


errors = ErrorCounter()


def _log_exception(where: str, exc_type, exc, tb) -> None:
    errors.bump()
    text = "".join(traceback.format_exception(exc_type, exc, tb))
    line = f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {where} ===\n{text}"
    try:
        sys.__stderr__.write(line)
    except Exception:
        pass
    if _crash_file is not None:
        try:
            _crash_file.write(line)
            _crash_file.flush()
        except Exception:
            pass


def install_crash_logging(log_dir: str, tk_root=None) -> None:
    """Log unhandled exceptions from all threads (and Tk callbacks if a root
    is given); enable faulthandler for native crashes."""
    global _crash_file
    if _crash_file is None:                 # second call only adds the Tk hook
        try:
            os.makedirs(log_dir, exist_ok=True)
            _crash_file = open(os.path.join(log_dir, time.strftime("crash_%Y%m%d.log")),
                               "a", encoding="utf-8")
            faulthandler.enable(file=_crash_file, all_threads=True)
        except OSError:
            _crash_file = None
            try:
                faulthandler.enable()       # to stderr (fails without a console)
            except Exception:
                pass

    sys.excepthook = lambda t, e, tb: _log_exception("main thread", t, e, tb)
    threading.excepthook = lambda a: _log_exception(
        f"thread {getattr(a.thread, 'name', '?')}", a.exc_type, a.exc_value, a.exc_traceback)
    if tk_root is not None:
        # Tk swallows callback exceptions (prints them); log + count instead.
        tk_root.report_callback_exception = lambda t, e, tb: _log_exception(
            "Tk callback", t, e, tb)


def single_instance(name: str = "Local\\RAD_DroneCockpitUI") -> bool:
    """True if this is the only running cockpit (Windows named mutex)."""
    global _mutex_handle
    if not _IS_WIN:
        return True
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        _mutex_handle = k32.CreateMutexW(None, False, name)
        return k32.GetLastError() != 183            # ERROR_ALREADY_EXISTS
    except Exception:
        return True


def keep_awake(on: bool = True) -> None:
    if not _IS_WIN:
        return
    try:
        import ctypes
        ES_CONTINUOUS, ES_SYSTEM_REQUIRED, ES_DISPLAY_REQUIRED = 0x80000000, 0x1, 0x2
        flags = ES_CONTINUOUS | (ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED if on else 0)
        ctypes.windll.kernel32.SetThreadExecutionState(flags)
    except Exception:
        pass


def timer_resolution(on: bool = True) -> None:
    if not _IS_WIN:
        return
    try:
        import ctypes
        (ctypes.windll.winmm.timeBeginPeriod if on else ctypes.windll.winmm.timeEndPeriod)(1)
    except Exception:
        pass


def raise_priority() -> None:
    if not _IS_WIN:
        return
    try:
        import ctypes
        ABOVE_NORMAL_PRIORITY_CLASS = 0x00008000
        k32 = ctypes.windll.kernel32
        k32.SetPriorityClass(k32.GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS)
    except Exception:
        pass


def alarm_sound(level: int, tk_root=None) -> None:
    """Non-blocking alarm: CRITICAL (3) = error sound, else warning sound."""
    if _IS_WIN:
        try:
            import winsound
            winsound.MessageBeep(winsound.MB_ICONHAND if level >= 3 else winsound.MB_ICONEXCLAMATION)
            return
        except Exception:
            pass
    if tk_root is not None:
        try:
            tk_root.bell()
        except Exception:
            pass
