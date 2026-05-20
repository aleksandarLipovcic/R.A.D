# =============================================================================
# conftest.py  —  System Acceptance Test Session Fixtures
#
# V-Model reference: IMU Subsystem V-Model — System Acceptance Testing (SAT)
# Tests:  SAT-IMU-001 .. SAT-IMU-005
# Covers: SYS-001 .. SYS-005
#
# Requires:
#   - XFlight Hobby F405 V3 connected via USB, Betaflight 4.5.3 running
#   - Props OFF for all tests
#   - DroneBackend.pyd in Release\ or Debug\ subfolder
#   - IMUWidget.py accessible from the project root
#
# Run from project root:
#   pytest sat_tests/ -v --tb=short -s
#
# The -s flag is important: SAT-IMU-003 prints operator prompts to stdout
# that require a human to physically tilt the FC at the right moment.
# =============================================================================

import sys
import os
import time
import pytest

# ---------------------------------------------------------------------------
# Path bootstrap — DroneBackend.pyd in Release\ or Debug\
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

for _sub in ("Release", "Debug"):
    _candidate = os.path.join(_ROOT, _sub)
    if os.path.isdir(_candidate) and _candidate not in sys.path:
        sys.path.insert(0, _candidate)

# IMUWidget lives in the project root
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import DroneBackend as _db


# ---------------------------------------------------------------------------
# Session-scoped DroneLink — connect once, share across all SAT tests.
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def drone_link():
    port = _db.auto_detect_f405()
    if port == "NOT_FOUND":
        pytest.skip("F405 not found — connect board via USB and retry")

    dl = _db.DroneLink()
    ok = dl.connect(port)
    if not ok:
        pytest.skip(f"DroneLink.connect('{port}') returned False")

    print(f"\n[SAT conftest] Connected on {port} — waiting 2 s for telemetry to stabilise")
    time.sleep(2.0)
    yield dl

    dl.disconnect()
    print("\n[SAT conftest] Disconnected")


# ---------------------------------------------------------------------------
# Session-scoped IMUSensor
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def imu_sensor(drone_link):
    return _db.IMUSensor(drone_link)


# ---------------------------------------------------------------------------
# Session-scoped IMUWidget in a hidden Tkinter root.
# Shared so the widget state persists across tests — mirrors real GCS usage.
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def imu_widget(drone_link):
    import tkinter as tk
    from IMUWidget import IMUWidget

    root = tk.Tk()
    root.withdraw()
    widget = IMUWidget(root)

    # Prime the widget with one live update before tests begin
    s = drone_link.get_latest_state()
    widget.update_ui(s.to_dict())

    yield root, widget

    try:
        root.destroy()
    except Exception:
        pass