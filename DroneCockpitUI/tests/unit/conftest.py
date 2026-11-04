# conftest.py
import gc
import time
import pytest

collect_ignore = ["DroneCockpitUI.py", "main.py"]

@pytest.fixture(scope="session")
def tk_root():
    import tkinter as tk
    root = tk.Tk()
    root.withdraw()
    yield root
    try:
        root.update()
    except Exception:
        pass
    root.destroy()

@pytest.fixture(autouse=True)
def flush_tkinter(tk_root):
    """Flush pending after() callbacks between every test."""
    yield
    try:
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            tk_root.update()
            time.sleep(0.01)
    except Exception:
        pass
    # Free this test's Tk objects HERE, on the main thread. Left to the
    # garbage collector they can be freed later on a worker thread, and
    # tkinter then warns "main thread is not in main loop".
    gc.collect()