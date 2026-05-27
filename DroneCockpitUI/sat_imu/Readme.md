# sat_tests/ — System Acceptance Test Suite
# IMU Subsystem V-Model — Top-Right of V
# SAT-IMU-001 .. SAT-IMU-005  ←→  SYS-001 .. SYS-005

## Position in the V-Model

```
SYS-001..005          ←────────────────────→   SAT-IMU-001..005
(Section 2)                                     (this folder)
     ↓                                               ↑
SRS-IMU-001..008b ←──→ UT + IT (Sections 6–7) ──────┘
```

SAT tests sit at the TOP-RIGHT of the V. They verify system-level
requirements (SYS) using the complete, integrated GCS — no mocking,
no component isolation. The board must be connected and Betaflight running.

## Test–Requirement Map

| Test file            | SAT ID       | SYS-REQ | SRS coverage                        |
|----------------------|--------------|---------|-------------------------------------|
| test_SAT_IMU_001.py  | SAT-IMU-001  | SYS-001 | SRS-IMU-001, 002, 003               |
| test_SAT_IMU_002.py  | SAT-IMU-002  | SYS-002 | SRS-IMU-004a, 004b, 004c            |
| test_SAT_IMU_003.py  | SAT-IMU-003  | SYS-003 | SRS-IMU-005, 006, 006a              |
| test_SAT_IMU_004.py  | SAT-IMU-004  | SYS-004 | SRS-IMU-007                         |
| test_SAT_IMU_005.py  | SAT-IMU-005  | SYS-005 | SRS-IMU-008, 008b                   |

## Prerequisites

| Item              | Requirement                                          |
|-------------------|------------------------------------------------------|
| Board             | XFlight Hobby F405 V3, USB connected                 |
| Firmware          | Betaflight 4.5.3 (default IMU ranges)                |
| Props             | **OFF** for all tests                                |
| Build output      | `DroneBackend.pyd` in `..\Release\` or `..\Debug\`  |
| Python            | 3.10+, pytest ≥ 8.0                                 |
| SAT-IMU-002       | FC flat and level on anti-vibration foam             |
| SAT-IMU-003       | Operator present to physically tilt FC on prompt     |

## How to run

From the **project root** (one level above `sat_tests/`):

```bash
# All SAT tests (use -s to see operator prompts for SAT-003)
pytest sat_tests/ -v --tb=short -s

# Automated tests only (skip SAT-003 operator interaction)
pytest sat_tests/ -v --tb=short -s --ignore=sat_tests/test_SAT_IMU_003.py

# Single test
pytest sat_tests/test_SAT_IMU_001.py -v -s
```

## SAT-IMU-003 operator procedure

SAT-003 is semi-automated. The software validates the alert state machine
by inspecting widget internals, but requires a human to physically tilt the FC:

1. **WARN test** — when prompted, rotate FC briskly (~45 °/s on any axis).
   You have 5 s. The amber WARN state must appear.
2. **CRIT test** — when prompted, rotate FC very rapidly (~120 °/s).
   You have 5 s. The red flashing CRIT state must appear.
3. **Recovery** — set FC back flat. All axes must return to SAFE (green)
   within 8 s (covers the 4 s CRIT hold-down + margin).

Always run with `-s` so the prompts print to the terminal.

## SAT-IMU-002 precondition

Place the FC **flat and level on foam** before running.
The MPU-6500 Z-axis must read +1 g (gravity pointing down into the board).

## Fault injection mechanism (SAT-IMU-005)

`set_poll_interval_ms(0)` spins the worker loop at maximum rate so the
80 ms `sendMSP()` read timeout fires continuously, producing empty frame
returns that `parseIMU()` rejects (buf.size() < 18). This drives
`consecutiveFails` above `FAIL_THRESHOLD` (5) and flips `link_healthy`
to False — identical to the behaviour specified in SRS-IMU-008b and
verified by IT-IMU-004 at the integration level.