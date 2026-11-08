# Command uplink (laptop → drone)

> **Status: implemented (design A, Lua bridge), NOT yet tested on hardware.**
> Verified off-hardware only: C++ unit tests (`DroneBackend/tests/test_uplink.cpp`),
> `radcmd.lua` under Lua 5.2 with a mocked EdgeTX API cross-checked against
> the C++ decoders (`DroneCockpitUI/tests/unit/test_lua_bridge.py`), and the Python controller
> and altitude hold against a vertical model (`DroneCockpitUI/tests/unit/test_uplink_python.py`).
> Setup, bench tests and flight procedure: [command-uplink-uputstvo.md](command-uplink-uputstvo.md) (Serbian).

## Requirements

| # | Requirement | Status |
|---|---|---|
| R1 | Gimbal pan + tilt from the laptop, angles for georeferencing | implemented (CH8 tilt → servo 0 on M5, CH9 pan → servo 1 on M6; `control/gimbal.py`) |
| R2 | Emergency actions (RTH / land) from the laptop | **not implemented** — stays on the pilot's switches |
| R3 | Movement: forward/back/left/right/yaw, altitude hold | implemented (stick offsets in ANGLE mode, laptop-side baro altitude hold) |
| R4 | Pilot keeps final authority | radio-side gate switch, ADD mixing, bounded throttle delta |
| R5 | Losing the laptop never affects flight | radio watchdog 500 ms, laptop lease 300 ms |
| R6 | Telemetry keeps working | the Lua script re-encodes the same CRSF frames Telem Mirror produced |

## Data path

```
UplinkController (Python, 25 Hz) ─set_command()─► CommandLink (C++, no port)
CrsfLink worker ─nextFrame()/WriteFile─► Pocket USB-VCP "LUA" ─► radcmd.lua
radcmd.lua ─mixer outputs─► CH1–4 (ADD, gated) + CH8/CH9 gimbal ══ELRS══► Betaflight
radcmd.lua ─CRSF telemetry + RC_CHANNELS 0x16 + RAD_STATUS 0x7E─► CrsfLink
```

- **One port owner.** `CrsfLink` keeps owning the Pocket's COM port (auto-detect,
  auto-reconnect unchanged). A `CommandLink` attached with `set_uplink()` is
  serviced from the same worker thread.
- **Write only to a proven bridge.** `CommandLink::frameDue()` stays false until
  a valid `RAD_STATUS` frame arrived within `bridgeTimeoutMs` (1.5 s). A port in
  Telem Mirror mode, a CLI or a flight controller is never written to.
- **Telemetry unchanged.** `CrsfStateMapper` ignores the two new frame types
  (default branch); `TelemetryWorker` and all widgets work as before.

## Command frame (laptop → Pocket)

Standard CRSF `RC_CHANNELS_PACKED` (0x16, address 0xC8, CRC-8/DVB-S2), so the
same bytes also fit EdgeTX's proposed "CRSF Trainer over USB-VCP" mode.

| Channel | Meaning |
|---|---|
| 1 / 2 / 4 | roll / pitch / yaw **offset** (−100…100 %) |
| 3 | throttle target, absolute µs |
| 5 | gimbal tilt (−100…100 %) |
| 6 / 7 | sticks enable / throttle enable (1811 on, 172 off) |
| 8 | alive counter, changes every frame (a frozen sender is not "fresh") |
| 9 | gimbal pan (−100…100 %) |

## RAD_STATUS (0x7E, Pocket → laptop, USB only, never over RF)

24 bytes (version 2), big-endian: version, flags (fresh, gate, sticks
active, throttle active, telemetry live), echoed alive value (round-trip
time), command frames received, CRC errors, command age, pilot throttle
stick, applied throttle delta, script period, applied roll/pitch/yaw/tilt
outputs, applied pan output. Version 1 bridges (22 bytes, no pan) are still
accepted (`BridgeStatus.hasPan = false`).

## Safety model

| Layer | Where | Rule |
|---|---|---|
| Gate switch | radio (Lua + switch condition SD on the CH1–4 mixer lines) | Pilot switch off → roll/pitch/yaw/throttle outputs 0 |
| Watchdog | radio | No new alive counter for 500 ms → neutral, gimbal holds |
| Authority | radio mixer | Laptop offsets are ADDed to the pilot's sticks |
| Throttle | radio | Delta to the pilot's stick, ±THR_BAND (≈100 µs), only with stick > ≈1200 µs |
| Limits | laptop | roll/pitch ≤ 25 %, yaw ≤ 30 %, throttle 1150–1750 µs, slew-limited |
| UI heartbeat | laptop | Command panel (Tk thread) silent for 500 ms → movement 0, altitude hold continues |
| Lease | laptop | Command older than 300 ms → sticks/throttle disabled |
| Flight state | laptop | Commands only when ELRS telemetry is live, armed, ANGLE (bench override: `COCKPIT_UPLINK_BENCH=1`) |
| Log | laptop | Every command change → `logs/uplink_*.csv` |

**Change to the original rule "never command throttle":** replaced by the
bounded delta above. The laptop can only move throttle within ±100 µs of
where the pilot holds the stick; releasing the gate returns throttle to the
stick with a jump of at most that band. Altitude hold engages bumplessly
from the throttle the radio is transmitting (RC_CHANNELS echo).

Arming, flight modes, failsafe and GPS Rescue are never touched by the
laptop.

## Gimbal

Two MG90S servos on flight controller outputs M5/M6, 5 V from a separate
step-down. Betaflight `feature SERVO_TILT` with "forward from channel":
servo 0 (GIMBAL_PITCH, M5) ← CH8 = tilt, servo 1 (GIMBAL_ROLL, M6) ← CH9 =
pan. Betaflight copies the forwarded channel straight to the servo pulse
(then clamps to the servo's min/max), so 1 % of channel = 5.12 µs and the
servo **Rate does not reverse a forwarded channel** (it only scales CAMSTAB
stabilisation); direction is set in the cockpit calibration.

The R.A.D. mount has its base plate vertical at the nose, so the pan servo
turns about the drone's forward axis (`GIMBAL_MOUNT_AXIS = "roll"`):
`TelemetrySnapshot.gimbalRollDeg`, outermost in the gimbal chain
(roll → pan → tilt). Positive = a downward-looking camera swings right.

`control/gimbal.py` (`GimbalModel`):
- commands in degrees → pulse via per-axis calibration (`center_us`,
  `us_per_deg`, `direction`, soft limits) → channel %;
- angles for georeferencing from the best feedback: MSP_SERVO (USB,
  `DroneState.servoValues`, polled every 50 ms) > radio channel echo (CH8/CH9
  from the bridge) > command, passed through a servo model (300 °/s, 80 ms
  dead time);
- `gimbalMoving` while the model is still moving: DetectionLink then keeps
  each object's last fused position instead of fusing rays from a moving
  camera (record `range_method = "held (gimbal moving)"`).

With normal ±100 % radio limits Betaflight sees ~988–2012 µs, about 100° of
MG90S travel per axis (not the bracket's 180°). The horns are placed so this
window covers tilt 0–90° and pan ±45°; calibration in
[command-uplink-uputstvo.md](command-uplink-uputstvo.md) §4–5.

## Threads and real time

Video, telemetry and control run at the same time and must never wait for
each other. Every part runs in its own thread; they only exchange
snapshots under short locks.

| Thread | Rate | Does | Can block on |
|---|---|---|---|
| Tk (main) | events + 10 Hz panel refresh | keys/buttons → intents; draws status | nothing in the control path |
| `UplinkController` (Python) | 25 Hz | gates, `AltitudeHold`, `CommandLink.setCommand()` | GIL only (buffered log, flushed ≤ 1 Hz) |
| `CrsfLink` worker (C++) | ReadFile ≤ 20 ms | COM port: telemetry in, command frames out (~33 Hz) | the port only; no GIL |
| `VideoLink`, `DetectionLink` (C++) | stream rate | video decode, detections | their own sockets/devices |
| `TelemetryWorker` (Python) | 60 Hz | reads snapshots for the widgets | GIL only |

- **No deadlock.** `CrsfLink::stateMutex_`, `CrsfLink::cfgMutex_` and
  `CommandLink::m_` are never held together, and none is held during
  port I/O. The C++ worker never takes the GIL.
- **One thread per port.** The COM handle is synchronous, so Windows
  serialises reads and writes on it anyway; only the `CrsfLink` worker
  touches it. A command frame waits at most one ReadFile timeout (20 ms).
- **The radio stream never depends on Python timing.** The C++ worker
  re-sends the last command at ~33 Hz; a late Python tick only makes that
  value up to one tick older. The lease (300 ms) only trips if the
  controller thread really stops.
- **GIL.** `DroneCockpitUI.py` sets `sys.setswitchinterval(0.002)`. Measured
  on 2 cores with two CPU-bound Python threads: worst controller period
  130–240 ms at the default 5 ms, 65–85 ms at 2 ms (target 40 ms; lease 300 ms). The panel shows
  the worst period of the last 2 s ("laptop loop max").
- **Every stall ends in neutral:**

  | What stalls | Detected by | Result |
  |---|---|---|
  | Tk thread (heavy redraw, dialog) while a key is held | UI heartbeat, 500 ms | movement 0, altitude hold continues |
  | `UplinkController` thread | `CommandLink` lease, 300 ms | sticks + throttle neutral |
  | `CrsfLink` worker, USB cable, whole app | `radcmd.lua` watchdog, 500 ms | radio back to pilot sticks |
  | ELRS link | Betaflight failsafe | as configured on the FC |

Tests: `test_uplink.cpp::testConcurrency` (three real threads hammering
`CommandLink`; also run with `-fsanitize=thread`, no races),
`test_uplink_python.py::test_ui_freeze_stops_movement` and
`::test_controller_thread_never_blocks_on_ui`.

## Bench tests

| Test | Question | Pass criterion |
|---|---|---|
| UT-UPL-A1 | Bridge runs: serial in/out from a mix script, binary with zero bytes | `RAD_STATUS` arrives, telemetry CRC errors 0 |
| UT-UPL-A2 | Offsets reach Betaflight only with the gate on | Receiver tab follows `--sweep` only with the switch on |
| UT-UPL-A3 | Latency laptop → Betaflight channel | median < 100 ms (`uplink_bench.py --latency`) |
| UT-UPL-A4 | Telemetry from re-encoded sensors | attitude ≥ 5 Hz, GPS/battery ≥ 1 Hz, units match USB |
| UT-UPL-A5 | Gimbal | follows `--tilt` / `--pan`, also with the gate off |
| UT-GMB-1..4 | Gimbal calibration, direction, limits, georeferencing angles | see command-uplink-uputstvo.md §5 |
| UT-UPL-S1..S3 | Gate off / USB unplugged / app killed mid-sweep | channels back to the sticks within 500 ms |
| UT-UPL-S4 | Script CPU budget | no "Script killed" in 10 min |
| ALT-1..3, MOV-1 | Rod rig: bumpless engage, step response, battery sag, movement | see command-uplink-uputstvo.md §6 |

UT-UPL-B1 (MSP override over CRSF) is no longer needed for R3.

## Not done / next

- R2 (RTH/land request from the laptop): would be one more Lua output on a
  free AUX channel mapped in the Modes tab, behind a two-step confirm.
- Position hold: needs a laptop GPS position loop on top of the movement
  offsets (drift with wind is not corrected today).
- If EdgeTX ships CRSF Trainer over USB-VCP with telemetry back on the same
  port, the laptop side stays the same and only the radio mixing changes.
