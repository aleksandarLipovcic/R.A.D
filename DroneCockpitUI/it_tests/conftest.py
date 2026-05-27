# =============================================================================
# conftest.py  —  Integration Test Session Fixtures
#
# V-Model reference: IMU Subsystem V-Model, Section 7
# Tests:  IT-IMU-001 .. IT-IMU-004
#
# Requires:
#   - XFlight Hobby F405 V3 connected via USB
#   - Betaflight 4.5.3 running (default IMU ranges: ±16g / ±2000°/s)
#   - DroneBackend.pyd present in Release\ or Debug\ subfolder
#
# Run:
#   pytest it_tests/ -v --tb=short
# =============================================================================

import sys
import os
import time
import pytest

# ---------------------------------------------------------------------------
# Path bootstrap — add Release\ then Debug\ so DroneBackend.pyd is importable
# regardless of which build configuration was last compiled.
# Call from the project root:  pytest it_tests/ -v
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)   # one level up from it_tests/

for _sub in ("Release", "Debug"):
    _candidate = os.path.join(_ROOT, _sub)
    if os.path.isdir(_candidate) and _candidate not in sys.path:
        sys.path.insert(0, _candidate)

import DroneBackend as _db   # fails fast with a clear error if .pyd not found


# ---------------------------------------------------------------------------
# Session-scoped DroneLink — connect once, share across all IT tests.
#
# Using session scope avoids the ~1 s serial open/close overhead on every
# test, and keeps the worker thread alive so packet_count grows naturally
# throughout the session (used by IT-IMU-001 and IT-IMU-004).
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def drone_link():
    port = _db.auto_detect_f405()
    if port == "NOT_FOUND":
        pytest.skip("F405 not found — connect board via USB and retry")

    dl = _db.DroneLink()
    ok = dl.connect(port)
    if not ok:
        pytest.skip(f"DroneLink.connect('{port}') returned False — "
                    "check drivers (CP210x / CH340) and COM port")

    print(f"\n[conftest] Connected on {port} — waiting 1 s for telemetry to stabilise")
    time.sleep(1.0)   # let the worker thread fill currentState before tests begin

    yield dl

    dl.disconnect()
    print("\n[conftest] Disconnected")


# ---------------------------------------------------------------------------
# Session-scoped IMUSensor — wraps the shared DroneLink.
# Exposed as a separate fixture so test files can import just what they need.
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def imu_sensor(drone_link):
    return _db.IMUSensor(drone_link)