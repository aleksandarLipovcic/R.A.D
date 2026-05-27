# it_tests/ — Integration Test Suite
# IMU Subsystem V-Model, Section 7 (IT-IMU-001 .. IT-IMU-004)

## Folder layout

    it_tests/
    ├── conftest.py           ← shared fixtures (DroneLink, IMUSensor)
    ├── test_IT_IMU_001.py    ← Live Data Continuity       (SRS-IMU-001/003)
    ├── test_IT_IMU_002.py    ← Scale vs Gravity           (SRS-IMU-004a)
    ├── test_IT_IMU_003.py    ← End-to-End Latency         (SRS-IMU-007)
    ├── test_IT_IMU_004.py    ← Corrupt Frame Recovery     (SRS-IMU-008/008b)
    └── README.md

## Prerequisites

| Item | Requirement |
|---|---|
| Board | XFlight Hobby F405 V3, USB connected |
| Firmware | Betaflight 4.5.3 (default IMU ranges: ±4g / ±2000°/s) |
| Build output | `DroneBackend.pyd` in `..\Release\` or `..\Debug\` |
| Python | 3.10+, pytest ≥ 8.0 |
| For IT-002 | FC flat and level on anti-vibration foam, props OFF |

## How to run

From the **project root** (one level above `it_tests/`):

```
# All integration tests
pytest it_tests/ -v --tb=short

# Single test file
pytest it_tests/test_IT_IMU_001.py -v

# Single test
pytest it_tests/test_IT_IMU_002.py::TestScaleValidation::test_IT_IMU_002_accel_z_mean -v
```

## IT-IMU-002 precondition

Place the FC **flat and level on foam** before running the scale test.
The MPU-6500 Z-axis should read +1 g (gravity pointing down into the board).
Running on a vibrating surface or at an angle will fail the stdev assertion.

## IT-IMU-004 injection mechanism

`set_poll_interval_ms(0)` spins the worker loop at maximum rate so the
80 ms `sendMSP()` read timeout fires frequently, producing empty frame
returns that `parseIMU()` rejects. This drives `consecutiveFails` above
`FAIL_THRESHOLD` (5) and flips `link_healthy` to False — matching the
corrupt-frame scenario described in the ITS without needing to intercept
the serial bus.