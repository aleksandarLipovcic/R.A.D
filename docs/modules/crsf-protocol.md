# `CrsfProtocol` & `CrsfStateMapper` — CRSF decoding and mapping

**Files:** `CrsfProtocol.h`, `CrsfProtocol.cpp`, `CrsfStateMapper.h`,
`CrsfStateMapper.cpp`
**Used by:** [`CrsfLink`](crsflink.md) only
**Exposed to Python:** no, only through `CrsfLink` and the `DroneState` it
fills

Both units are **pure logic**: no Win32, no threads, no I/O. `CrsfLink`
feeds them bytes and timestamps from its reader thread while holding its
own lock. Because they have no hardware dependencies, every framing and
mapping rule can be unit-tested without a radio.

---

## `CrsfProtocol` — framing and decoders (`namespace Crsf`)

### Frame layout

All multi-byte fields are **big-endian**. MSP is little-endian, so do not
reuse MSP helpers here.

```
[addr/sync][len][type][payload ...][crc8]
  len  = bytes after itself = 1 (type) + payload + 1 (crc)   → 2 ≤ len ≤ 62
  crc8 = CRC-8/DVB-S2 (poly 0xD5, init 0x00, no reflection) over type + payload
  max total frame size = 64 bytes
```

Accepted start-of-frame bytes are `0xC8` (flight controller, also the
generic sync byte), `0xEA` (handset), `0xEC` (CRSF receiver) and `0xEE`
(TX module).

### `StreamParser`

`feed(data, len, out)` accepts chunks of any size, exactly as `ReadFile`
returns them, and appends every complete, CRC-valid `Frame`
(`addr`, `type`, `payload`) to `out`. It handles:

- **frames split across reads:** incomplete tails stay in the internal
  buffer until the rest arrives;
- **garbage between frames:** bytes are skipped until a sync byte is
  followed by a plausible length;
- **bad CRC:** it moves forward **one byte** and resyncs, so a real frame
  that starts inside a corrupted one is not lost;
- **pure noise:** the buffer is cleared once it grows past 4 × 64 bytes.

Counters: `framesOk()`, `crcErrors()`, `bytesDiscarded()`. `CrsfLink`
copies the first two into `RadioLinkStats::framesTotal` / `crcErrors`.

### Decoded frame types

| Type | ID | Decoder | Output struct / units |
|---|---|---|---|
| GPS | `0x02` | `decodeGps` | `Gps`: lat/lon in degrees (wire value ×1e7), ground speed km/h (wire ×10), heading ° (wire ×100), altitude m MSL (wire value has a **+1000 m offset**), satellites |
| VARIO | `0x07` | `decodeVario` | `Vario`: vertical speed cm/s |
| BATTERY | `0x08` | `decodeBattery` | `Battery`: V and A (0.1 resolution), mAh drawn (uint24), remaining % |
| BARO_ALTITUDE | `0x09` | `decodeBaroAltitude` | `BaroAltitude`: cm. If bit 15 is set: coarse mode, whole metres. Otherwise: decimetres with a +10000 offset |
| LINK_STATISTICS | `0x14` | `decodeLinkStats` | `LinkStats`: uplink RSSI ant. 1/2 in dBm (ExpressLRS sends a **signed** byte, 0xFB = −5 dBm; the CRSF spec's unsigned magnitude, 70 = −70 dBm, is also accepted: bytes ≥ 128 are signed, < 128 a magnitude), LQ %, SNR, active antenna, RF mode index, TX power (enum → mW via `txPowerEnumToMw`), downlink RSSI/LQ/SNR |
| ATTITUDE | `0x1E` | `decodeAttitude` | `Attitude`: pitch/roll/yaw in degrees (wire: int16 radians × 10000, order **pitch, roll, yaw**). Yaw is normalised to [0, 360); `rawYaw` is kept for bench checks |
| FLIGHT_MODE | `0x21` | `decodeFlightMode` | null-terminated ASCII string, e.g. `"STAB"`, `"ACRO*"` |

`HEARTBEAT` (`0x0B`), `RC_CHANNELS` (`0x16`) and `DEVICE_INFO` (`0x29`) are
recognised by `typeName()` for logs but are not decoded.

Every decoder returns `false` if the frame type is wrong or the payload is
too short. They never read past the buffer.

**Yaw encoding caveat.** The CRSF spec defines yaw as a signed int16 in
[-π, π]. Some older FC firmwares send an unsigned value in [0, 2π).
`decodeAttitude(f, out, yawSigned)` supports both. Bench test
**IT-ELRS-004** (compare with the USB yaw between 180° and 360°) decides
the right setting, which is then applied with
`CrsfLink::setYawSigned()`.

### Helpers

- `crc8(data, len)`: CRC-8/DVB-S2.
- `txPowerEnumToMw(e)`: `{0, 10, 25, 100, 500, 1000, 2000, 250, 50}` mW,
  or 0 for an unknown index.
- `typeName(type)`: short name for sniffers and logs.
- `buildFrame(addr, type, payload)`: builds a complete frame with length
  and CRC. It exists **only for tests and simulators** and is never used
  on the live read path.

---

## `CrsfStateMapper` — frames → `DroneState`

`CrsfLink` owns one `CrsfStateMapper` and calls two methods on it:

- `apply(frame, state, nowMs)`: once per decoded frame. It writes the
  frame into `DroneState` and records the timestamp for that data group.
- `refresh(state, portOpen, nowMs, thresholds)`: about 10 Hz from the
  worker, and again right before every snapshot handed to Python. It
  recomputes data ages, update rates, the altitude fallback and the link
  status.

### Field mapping

The mapper converts everything into the **same units `DroneLink` uses for
MSP**, so the widgets and `TelemetryWorker._build_ui_data()` work
unchanged:

| `DroneState` field(s) | Source | Conversion / note |
|---|---|---|
| `roll`, `pitch` | ATTITUDE | degrees → **decidegrees** (MSP convention) |
| `yaw` | ATTITUDE | whole degrees, 0–359 |
| `magHeadingDeg`, `magValid` | ATTITUDE yaw | Fused heading, not the raw magnetometer. Keeps the compass alive; `magX/Y/Z` stay 0 |
| `batteryVoltage/Current/MahDrawn/Percentage` | BATTERY | mAh clamped to 16 bit, % clamped to 100 |
| `batteryCellCount`, `batteryState` | derived | See *Battery* below |
| `gps.latitude/longitude/altitudeM/numSat` | GPS | — |
| `gps.groundSpeedMs` | GPS | km/h → **cm/s** (the field name is historical) |
| `gps.groundCourse` | GPS | degrees → decidegrees |
| `gps.fixType` | derived | 2 (2D) if a non-zero position has ≥ 4 satellites, else 0. CRSF has no fix type |
| `gps.hdop` | — | Always **9999** ("invalid"). CRSF has no HDOP |
| `gps.positionUsable`, `navStatus.fixOk` | derived | `fixType ≥ 2` and `numSat ≥ minSatsForFix` |
| `gps.distToHomM`, `bearingToHome`, `compValid` | derived | See *Home point* below |
| `baroAltitudeCm`, `baroValid` | BARO_ALTITUDE, or GPS fallback | `radio.altitudeSource` = `"BARO"` / `"GPS"` / `"NONE"` |
| `baroVarioCmPerSec` | VARIO | — |
| `armed`, `flightModeFlags`, `flightModeName` | FLIGHT_MODE | See *Flight mode* below |
| `armingDisableFlags`, `armingDisableStr` | FLIGHT_MODE `!ERR` | Flags always 0 (unknown). The text says "ARMING BLOCKED (reasons only via USB)" |
| `rssi` | LINK_STATISTICS | Uplink LQ % scaled to 0–255, so the existing `rc_link_quality` shows the real ELRS LQ |
| `sensorStatus` | derived | ACC\|GYRO bits on attitude, GPS bit on GPS, BARO bit on baro/vario |
| `radio.*` | LINK_STATISTICS + mapper | All ELRS extras (see [bindings.md](bindings.md#radiolinkstatus--radiolinkstats)) |
| `linkSource` | — | Always `"ELRS"` |
| `linkHealthy` | status | `true` for `TELEMETRY_OK` and `DEGRADED` |

### Flight mode

Betaflight 4.5's `crsfFrameFlightMode()` sends a short string. A trailing
`*` means **disarmed**. `mapFlightMode()` turns it into the same flags and
names that `DroneLink::decodeFlightMode()` produces:

| Raw | `flightModeName` | Flags | Extra |
|---|---|---|---|
| `!FS!` | `FAILSAFE` | `FAILSAFE` | — |
| `RTH` | `GPS RESCUE` | `GPS_RESCUE` | — |
| `STAB` | `ANGLE` | `ANGLE` | — |
| `HOR` | `HORIZON` | `HORIZON` | — |
| `ACRO`, `AIR` | `ACRO` | — | — |
| `MANU` | `PASSTHRU` | — | — |
| `!ERR` | `ARMING BLOCKED` | — | `radio.armingBlocked = true` |
| `WAIT` | `WAIT GPS` | — | `radio.gpsWaiting = true` |
| anything else | the raw text | — | — |

`ARM` is set unless the text ends in `*`. When disarmed, ` [DISARMED]` is
appended to the name. The unmodified string is kept in
`radio.rawFlightMode`.

### Home point

CRSF has no home distance, so the mapper computes it on the laptop the same
way Betaflight does. On every **arming edge** the old home is discarded,
and the first usable GPS fix while armed becomes the new home (this also
covers a GPS fix that arrives after arming). From then on every GPS or
flight-mode frame updates `distToHomM` (haversine, metres) and
`bearingToHome` (initial great-circle bearing, stored as −180..+180 like
`MSP_COMP_GPS`). `distanceM()` and `bearingDeg()` are public static
helpers.

### Battery

If `cellCount` is set, it is used. Otherwise the cell count is detected
**once**, from the first reading, as `ceil(V / 4.35)`. The per-cell voltage
is then compared with `criticalCellV` (3.3 V) and `warningCellV` (3.5 V) to
set `CRITICAL` / `WARNING` / `OK`. A pack voltage below 1 V means
`NOT_PRESENT`.

### Ages, rates and the altitude fallback (`refresh`)

- **Ages:** `msSinceLastFrame`, `attitudeAgeMs`, `gpsAgeMs`,
  `batteryAgeMs`, `flightModeAgeMs`, `baroAgeMs`, `linkStatsAgeMs`, each
  in ms, with **−1 = never received**.
- **Rates:** per-group frame counts over a rolling window of about 2 s
  (`attitudeHz`, `gpsHz`, …, `totalFrameHz`).
- **Altitude fallback:** if baro data is older than `baroStaleMs` (2 s),
  `gpsAltitudeFallback` is on, a home is set and the position is usable,
  then `baroAltitudeCm` = (GPS altitude − home altitude) × 100 and
  `altitudeSource = "GPS"`.
- **Status:** `evaluateStatus()` (see
  [crsflink.md](crsflink.md#link-status--radiolinkstatus)), plus the
  `linkLostCount` transition counter.

`resetTiming()` clears only timestamps and rate counters. The last known
position and attitude in `DroneState` are **deliberately kept**.

### Configuration structs

`CrsfStateMapper::Config` (`yawSigned`, `minSatsForFix`, `cellCount`,
`warningCellV`, `criticalCellV`, `capacityMah`, `gpsAltitudeFallback`,
`baroStaleMs`) and `StatusThresholds` (`lostTimeoutMs`, `degradedLq`,
`instrumentStaleMs`) are set through the `CrsfLink` setters listed in
[crsflink.md](crsflink.md#configuration).
