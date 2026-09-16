# `DroneLink` — MSP flight-controller link

**Files:** `DroneLink.h`, `DroneLink.cpp`
**Depends on:** `GPSNeoM10` (parsers), `MagQMC5883L` (used inside
`parseDebug`), `BaroBMP280` (used inside `parseBaro`)
**Exposed to Python as:** `DroneBackend.DroneLink` (see
[bindings.md](bindings.md#dronelink))

## Responsibility

`DroneLink` owns the USB-serial MSP connection to the Betaflight flight
controller, polls every telemetry message the cockpit needs, and publishes a
single thread-safe `DroneState` snapshot that Python (and internally,
`IMUSensor`) reads via `getLatestState()`.

## Connection lifecycle

```
connect(portName) → openSerialPort() → spawn communicationLoop() thread
disconnect()      → keepRunning = false → join thread → closeSerialPort()
```

`AutoDetectF405()` (free function, not a method) scans `COM1`–`COM29` and
returns the first port that opens successfully, for cockpit auto-connect UX.

## The polling loop (`communicationLoop()`)

Runs at `POLL_INTERVAL_MS` (10 ms → 100 Hz) by default, adjustable at
runtime via `setPollIntervalMs()`. Each tick issues a fixed sequence of MSP
requests (`sendMSP(id)` + a matching `parseX()` call) and, at the end of the
tick, calls `commitState()` to atomically publish the accumulated
`DroneState` under `dataMutex`.

Not every message is polled every tick — two counters throttle the
expensive/slow-changing ones:

| Counter | Interval | Message | Why |
|---|---|---|---|
| `svPollTickCounter_` | `SV_POLL_TICKS` = 100 ticks (~1 s) | `MSP_GPS_SV_INFO` (164) | Satellite list — no port-cycle cost, but no need for 100 Hz |
| `slowPollTickCounter_` | `SLOW_POLL_TICKS` = 50 ticks (~500 ms) | `MSP_BATTERY_STATE` (242) | Battery state changes slowly; 2 Hz is plenty |

A comment in the header flags the real constraint: with ~13 MSP polls per
tick at 1–2 ms each, the 10 ms/100 Hz budget is tight on slow hardware. If
`communicationLoop()` overruns, call `setPollIntervalMs(20)` to drop to a
50 Hz rate with headroom.

### Parsers, one per MSP message

| MSP ID | Constant | Parser | Populates |
|---|---|---|---|
| 101 | `MSP::STATUS` | `parseStatus` | `armed`, `flightModeFlags`, `flightModeName`, `sensorStatus`, `i2cErrorCount`, `cpuLoadPercent`, `pidProfile` |
| 102 | `MSP::RAW_IMU` | `parseIMU` (protected, for test access) | `ax/ay/az`, `gx/gy/gz` (raw ADC counts) |
| 104 | `MSP::MOTOR` | `parseMotors` | `motorValues[]`, `motorCount` |
| 105 | `MSP::RC` | `parseRCChannels` | `rcChannels[]`, `rcChannelCount` |
| 106 | `MSP::RAW_GPS` | `parseGPSRaw` → `GPSNeoM10::parseRaw` | `gps.fixType/numSat/latitude/longitude/altitudeM/...` |
| 107 | `MSP::COMP_GPS` | `parseGPSComp` → `GPSNeoM10::parseComp` | `gps.distToHomM`, `gps.bearingToHome`, `gps.gpsHeartbeat` |
| 108 | `MSP::ATTITUDE` | `parseAttitude` | `roll`, `pitch`, `yaw` |
| 109 | `MSP::ALTITUDE` | `parseBaro` → `BaroBMP280::parse` | `baroAltitudeCm`, `baroVarioCmPerSec`, `baroValid` |
| 110 | `MSP::ANALOG` | `parseAnalog` | `batteryVoltage`, `batteryCurrent`, `batteryMahDrawn`, `rssi` |
| 121 | `MSP::NAV_STATUS` | `parseNavStatus` → `GPSNeoM10::parseNavStatus` | `navStatus.*` |
| 150 | `MSP::STATUS_EX` | `parseStatusEx` | `armingDisableFlags`, `armingDisableStr` |
| 164 | `MSP::GPS_SV_INFO` | `pollSatellitesMSP` → `parseMspSvInfo` → `GPSNeoM10::parseMspSvInfo` | `svList`, `svInfoValid`, `svSource = "MSP"` |
| 242 | `MSP::BATTERY_STATE` | `parseBatteryState` | `batteryCellCount`, `batteryCapacityMah`, `batteryPercentage`, `batteryState`, overwrites `batteryVoltage` with higher-resolution data |
| 254 | `MSP::DEBUG` | `parseDebug` (uses `MagQMC5883L`) | `magX/magY/magZ`, `magHeadingDeg`, `magValid` — **requires** `debug_mode = MAG_CALIB` set in the Betaflight CLI |

`decodeFlightMode(flags)` and `decodeArmingDisable(flags)` are static helpers
that turn the raw bitmasks (`FlightMode::*`, `ArmingDisable::*` namespaces in
the header) into human-readable strings for the UI
(`"ANGLE"`, `"ACRO+MAG"`, `"GPS RESCUE"`, `"ANGLE [DISARMED]"`, etc.).

## `DroneState` — the shared snapshot

Single struct holding everything the cockpit and other C++ classes need:
IMU raw counts, attitude, battery (both the fast `MSP_ANALOG` fields and the
slower, more precise `MSP_BATTERY_STATE` fields), barometer, magnetometer,
calibration progress, FC status/arming diagnostics, motor outputs, RC
channels, GPS (`GPSReading`), satellite list (`vector<SVInfoEntry>`), nav
status, and link diagnostics (`lastRttMs`, `fcCycleMs`, `linkHealthy`,
`packetCount`).

It is written exclusively by the worker thread and read only through
`getLatestState()`, which takes `dataMutex` and returns a copy — never a
reference — so callers can't observe a half-written state and don't need to
hold any lock themselves.

## Calibration

`startMagCalibration()` / `startAccCalibration()` just raise an atomic
request flag (`magCalRequested` / `accCalRequested`) from whatever thread
calls them (normally the Python/UI thread); the worker thread consumes the
flag, runs the countdown (`MAG_CAL_DURATION_S` = 30 s, `ACC_CAL_DURATION_S`
= 5 s) using `magCalActive_`/`magCalStartTime_` etc. (worker-thread-only
state), and mirrors progress into `DroneState.magCalActive` /
`magCalSecondsRemaining` (and the accelerometer equivalents) via
`commitState()` so Python can show a countdown.

## GPS passthrough configuration

`applyGPSConfig(const GPSConfig&)` is the **only** path that talks to the
NEO-M10 GPS module directly rather than through the FC's own MSP telemetry.
It uses the two-step `MSP_SET_PASSTHROUGH` (245 / `0xF5`) protocol
documented at length in `DroneLink.h`:

1. **Activate** — send `MSP_SET_PASSTHROUGH` with a one-byte payload equal
   to the GPS UART index (`setGpsUartIndex()`, default 0 = UART1, confirmed
   for this project's wiring). The FC ACKs, then suspends MSP and starts
   forwarding raw bytes.
2. **Configure** — talk raw UBX frames directly to the GPS
   (`buildCfgGNSS`, `buildCfgRate`, `buildCfgPrt`, `buildCfgNav5`,
   `buildCfgCfg` in `GPSNeoM10`), reading UBX ACKs (`parseAck`) via
   `readUbxResponse()`.

Betaflight 4.x **never exits passthrough on its own** — the only way back to
normal MSP mode is closing and reopening the serial port, which resets the
USB-serial chip (CP210x/CH340) and takes ~150–250 ms. `applyGPSConfig()`
reopens the saved `portName_` after the exchange for exactly this reason.
Result fields (`GPSConfigResult`) report which stage acked successfully
(`gnssAck`, `rateAck`, `protocolAck`, `saveAck`, `overallOk`) plus a free-text
`errorDetail`.

This path is unrelated to satellite *polling* — `pollSatellitesMSP()` reads
`MSP_GPS_SV_INFO` (164) over ordinary MSP and needs no UART index or
passthrough at all; the legacy UBX-passthrough satellite poll
(`pollSatellites()` / `parseNavSvInfo()`) is kept only in case a future board
needs it, but is not on the normal polling path.

## Test / fault-injection hooks

- `setFailInjection(bool)` — when active, `sendMSP()` returns an empty
  buffer immediately on every call, deterministically simulating a dead
  link without depending on real serial timing. Documented as test-only;
  must be restored to `false` in test teardown and must never be called
  from production flight code.
- `parseIMU()` is `protected` rather than `private` specifically so tests
  can exercise it without widening the public API.

## Constants worth knowing when tuning

| Constant | Value | Meaning |
|---|---|---|
| `MSP_BAUD` | 57600 | DroneLink ↔ FC USB baud rate (confirmed via `sv_baud_probe.py`) |
| `POLL_INTERVAL_MS` | 10 | Default 100 Hz poll rate |
| `FAIL_THRESHOLD` | 5 | Consecutive failed polls before the link is considered unhealthy |
| `MAX_MOTORS` | 8 | Array size for `motorValues[]` |
| `MAX_RC_CH` | 18 | Array size for `rcChannels[]` (CRSF/ELRS channel budget) |

Note the GPS UART baud (115200, set in Betaflight Configurator → Ports) is
entirely separate from `MSP_BAUD` and is never touched by `DroneLink` — the
FC negotiates it with the NEO-M10 internally via `gps_auto_baud=ON`.
