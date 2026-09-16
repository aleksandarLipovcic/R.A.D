# Sensor helper classes

These four classes turn raw MSP payload bytes into physical, UI-ready
values. None of them own a thread or a connection — they're called
synchronously from inside `DroneLink::communicationLoop()` (see
[dronelink.md](dronelink.md)), except `IMUSensor`, which is a small
standalone unit-conversion wrapper Python can instantiate directly.

## `GPSNeoM10` — GPS parsing & UBX configuration

**Files:** `GPSneoM10.h`, `GPSneoM10.cpp`

A **static-only** class (no instance state) providing two families of
functionality:

### MSP parsers (primary data path)

| Method | MSP source | Fills |
|---|---|---|
| `parseRaw()` | `MSP_RAW_GPS` (106) | `GPSReading.fixType/numSat/latitude/longitude/altitudeM/groundSpeedMs/groundCourse/hdop` |
| `parseComp()` | `MSP_COMP_GPS` (107) | `GPSReading.distToHomM/bearingToHome/gpsHeartbeat` |
| `parseNavStatus()` | `MSP_NAV_STATUS` (121) | `NavStatus` (fix type, GPS/DGPS flags, nav engine health) |
| `parseMspSvInfo()` | `MSP_GPS_SV_INFO` (164) | `vector<SVInfoEntry>` — the satellite list, **primary** path |

### `MSP_GPS_SV_INFO` wire format — the part worth reading twice

The header contains an extensive, load-bearing comment on how Betaflight
4.5.x actually packs this message, because the naive reading of the byte
layout is wrong:

```
rec[0] = gnssId   (0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 5=QZSS, 6=GLONASS)
rec[1] = svid     (satellite vehicle ID / PRN)
rec[2] = UBX flags byte: bits[2:0] = qualityInd (0-7), bit[3] = svUsed
rec[3] = cno      (carrier-to-noise, dBHz)
```

**Do not** try to extract `gnssId` from the upper nibble of `rec[2]` — that
byte holds UBX correction-status flags, not a packed `gnssId`+`quality`
field, despite what an older/legacy layout might suggest. The NEO-M10
always reports `numCh = 32`, which is always above
`GPS_SV_MAXSATS_LEGACY` (16), so it is always on this "enhanced" path.
`elev`, `azim`, and `prRes` are always 0 in this mode (not present in the
4-byte record) — the Python UI should hide those columns when
`sv_source == "MSP"`.

### UBX parsers & frame builders (passthrough path only)

`parseNavSvInfo()` and `parseAck()` decode raw UBX frames received during a
`MSP_SET_PASSTHROUGH` session; `buildNavSvInfoPoll()`, `buildCfgGNSS()`,
`buildCfgRate()`, `buildCfgPrt()`, `buildCfgNav5()`, and `buildCfgCfg()`
build the corresponding UBX request frames (checksummed via the private
`ubxChecksum()`). These are used exclusively by
`DroneLink::applyGPSConfig()` — see
[dronelink.md](dronelink.md#gps-passthrough-configuration) — never by the
normal telemetry poll.

### Key types

- `GPSConfig` / `GPSConfigResult` — input/output of `applyGPSConfig()`:
  constellation mask (`GPSConstellationFlags`, e.g. `GNSS_DEFAULT` =
  GPS+Galileo+GLONASS), update rate (`GPSUpdateRate`), protocol
  (`GPSProtocol`), SBAS, elevation/signal masks.
- `GPSReading` — the live fix (see table above).
- `NavStatus` — FC nav-engine health from `MSP_NAV_STATUS`.
- `SVInfoEntry` — one satellite channel: `gnssId`, `svid`, `quality`
  (0–7), `cno` (dBHz), `used`, plus human-readable `gnssName` and
  `statusStr` ("used", "fully locked", "locked", "searching", "idle").

## `IMUSensor` — raw → physical unit conversion

**Files:** `IMUSensor.h`, `IMUSensor.cpp`

Thin wrapper around a `DroneLink*`: every call pulls the latest
`DroneState` via `getLatestState()` and re-derives its output — it holds no
cached state of its own, so it's safe to call from any thread at any
frequency.

- `getRawData()` → `IMUData` — the raw ADC counts (`accX/Y/Z`, `gyroX/Y/Z`)
  exactly as `DroneState` stores them, no conversion.
- `getScaledData()` → `IMUScaled` — physical units:
  - Accelerometer: `ACC_SCALE = 1/2048` (g per count), because Betaflight
    4.5.x initializes the MPU-6500 at ±16g (`INV_FSR_16G`), giving 2048
    LSB/g in hardware, and `MSP_RAW_IMU` transmits `accADC` with **no
    additional prescaling**.
  - Gyroscope: `GYRO_SCALE = 1/16.4` (°/s per count), for the MPU-6500's
    ±2000 °/s range, also transmitted unprescaled.

> **Historical note captured in the code:** `ACC_SCALE` was previously
> `1/8192` under an incorrect assumption of a ±4g hardware range. That
> produced a 4× under-read, tracked as defect **DEF-002** / test
> **IT-IMU-002**, and the header comment explicitly documents the fix and
> why 2048 is correct — worth knowing if this constant is ever "corrected"
> back by someone unfamiliar with the history.

## `MagQMC5883L` — heading computation

**Files:** `MagQMC5883L.h`, `MagQMC5883L.cpp`

**There is no `MSP_RAW_MAG` in Betaflight 4.x.** The old MultiWii ID 130 was
repurposed by Betaflight as `MSP_BATTERY_STATE`. The header documents this
prominently because it's an easy trap: raw mag X/Y/Z is only obtainable via
the **debug subsystem**, requiring `set debug_mode = MAG_CALIB` (+ `save`)
in the Betaflight CLI, then polling `MSP_DEBUG` (254), where
`debug[0..2]` = raw mag X/Y/Z and `debug[3]` = heading error / calibration
quality. That polling and parsing happens in
`DroneLink::parseDebug()` (see [dronelink.md](dronelink.md)), **not** in
this class.

`MagQMC5883L` itself is kept only for two static, hardware-independent
helpers so they're unit-testable in isolation:

- `computeHeading(x, y)` — tilt-uncorrected 2-D heading via
  `atan2(y, x)`, normalized to 0–360° (0° = magnetic North). **Only
  accurate when the drone is level** — no roll/pitch compensation is
  applied. A tilt-compensated version would need roll/pitch from
  `DroneState` folded in before the `atan2` call.
- `verifyChecksum(buf)` — the standard MSP v1 XOR checksum (XOR of bytes
  `[3 .. size-2]` must equal the last byte).

**Betaflight CLI prerequisites** for this whole data path to exist at all:

```
set mag_hardware = QMC5883   (or AUTO if it's the only mag present)
set align_mag    = CW90      (adjust to your GPS module's mounting orientation)
set debug_mode   = MAG_CALIB
save
```

## `BaroBMP280` — altitude & vertical speed

**Files:** `BaroBMP280.h` (all logic, header-only static methods),
`BaroBMP280.cpp` (empty translation unit + integration notes only)

The BMP280 is never talked to directly — Betaflight reads it internally and
R.A.D. requests the FC-fused output via `MSP_ALTITUDE` (command 109, 6-byte
payload: `int32 estimatedAltitudeCm` + `int16 varioCmPerSec`).

- `BaroBMP280::parse(buf, out)` → `BaroReading` — validates frame length
  (≥12 bytes total) and command byte (`buf[4] == 109`), then decodes the
  little-endian altitude/vario fields.
- `BaroBMP280::scale(raw, qnhOffsetCm)` → `BaroData` — converts to
  metric/imperial (`altitudeM/Ft`, `varioMps/Fpm`) and additionally computes
  a QNH-corrected altitude (`altitudeMqnh/Ftqnh`) by subtracting a
  user-settable sea-level reference offset before converting.

`BaroBMP280.cpp`'s only real content is a **step-by-step integration
runbook** for wiring a new MSP-sourced field end to end through this
codebase — useful as a template for adding another sensor later:

1. Add the MSP constant + `DroneState` fields in `DroneLink.h`.
2. Add the poll + `parseBaro()` call inside `DroneLink::communicationLoop()`
   / the `parseBaro()` implementation in `DroneLink.cpp`.
3. Expose the new `DroneState` fields in `Bindings.cpp`
   (`.def_readonly(...)`).
4. Read them on the Python side and hand them to the UI widget.

Barometer fields already exist in `DroneState`
(`baroAltitudeCm`, `baroVarioCmPerSec`, `baroValid`) and are already bound
in `Bindings.cpp` — this runbook documents how they got there rather than
work still to do.
