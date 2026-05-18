# R.A.D Communication Protocol Guide — MSP & Betaflight Integration

## Table of Contents
1. [MSP Protocol Fundamentals](#msp-protocol-fundamentals)
2. [Frame Structure & Encoding](#frame-structure--encoding)
3. [MSP Command Reference](#msp-command-reference)
4. [Polling Cadence & Scheduling](#polling-cadence--scheduling)
5. [Betaflight Integration Details](#betaflight-integration-details)
6. [GPS Passthrough Mode](#gps-passthrough-mode)
7. [Debugging & Troubleshooting](#debugging--troubleshooting)
8. [Performance Optimization](#performance-optimization)

---

## MSP Protocol Fundamentals

### What is MSP?

**MultiWii Serial Protocol (MSP)** is a simple binary request-response protocol used by Betaflight to communicate with ground control stations and external tools (like R.A.D).

- **Creator:** MultiWii community (MultiWii flight controller)
- **Adoption:** Standardized across Betaflight, iNav, Cleanflight, and others
- **Transport:** RS-232 serial (USB CDC on modern FCs)
- **Baud Rate:** 57600 (standard; some legacy 115200)
- **Framing:** Discrete, length-prefixed messages
- **Error Detection:** Fletcher's 8-bit checksum

### Communication Model

**Request-Response (Synchronous):**

```
Host                          Flight Controller
──────────────────────────────────────────────

Send MSP command
────────────────────────────────►
							  Parse & execute
							  Generate response
Receive response
◄────────────────────────────────
Parse response
```

**All DroneLink commands are synchronous** — the worker thread blocks until a response arrives (with timeout).

### Design Principles

1. **Low Bandwidth:** Fixed-size payloads, minimal framing overhead
2. **Deterministic:** Same command always returns same payload size
3. **Extensible:** New commands added without breaking old ones (via command ID)
4. **Stateless:** Each command is independent; no session state
5. **Real-Time Friendly:** 10 ms poll cycle achievable even on 8-bit microcontrollers

---

## Frame Structure & Encoding

### Byte-Level Format

```
Offset  Size  Field           Description
───────────────────────────────────────────────────────
0       1     Start Marker    '$' (0x24)
1       1     Direction       'M' or '<' (see below)
2       1     Payload Size    0–255 bytes
3       1     Command ID      101–254 (varies by command)
4..n    0–255 Payload         Command-specific data
n+1     1     Checksum        Fletcher 8-bit CRC
```

### Direction Codes

| Code | Hex | Direction | Usage |
|------|-----|-----------|-------|
| 'M'  | 0x4D | **M**essage (FC → Host) | Response from flight controller |
| '<'  | 0x3C | **L**ess-than (Host → FC) | Request to flight controller |
| '>'  | 0x3E | **G**reater-than (Host → FC) | Not used in MSP1; reserved |

**Host sends:**
```
[$][<][size][cmd][payload...][checksum]
```

**FC responds:**
```
[$][M][size][cmd][payload...][checksum]
```

### Payload Size

- **0–255 bytes** — payload does NOT include the 4-byte header or checksum
- **Fixed per command** — `MSP_RAW_IMU` always returns 12 bytes payload, never more or less

### Checksum Calculation (Fletcher 8-bit)

```cpp
uint8_t crc8_fletcher(const uint8_t *data, int len) {
	uint8_t ck_a = 0, ck_b = 0;
	for (int i = 0; i < len; i++) {
		ck_a += data[i];
		ck_b += ck_a;
	}
	return (ck_b ^ ck_a);  // XOR final result
}

// In MSP context: checksum covers [size, cmd, payload...]
// but NOT the start marker or direction byte
```

**Implementation in DroneLink:**
```cpp
// Checksum calculation (from DroneLink.cpp)
uint8_t checksum = 0;
checksum ^= sizeof(payload);     // XOR with size
checksum ^= cmd_id;              // XOR with command
for (uint8_t byte : payload) {   // XOR all payload bytes
	checksum ^= byte;
}
```

### Example: Request for MSP_RAW_IMU (102)

**Goal:** Query the raw IMU data

**Hex Bytes (Request):**
```
24 3C 00 66 66
└─────────────────
0x24 = '$' start
0x3C = '<' direction (host → FC)
0x00 = size (0 bytes payload)
0x66 = 102 decimal (IMU command)
0x66 = checksum (0x00 ^ 0x66 ^ 0x66 = 0x00, but shown as 0x66 in actual implementations)
```

**Hex Bytes (Response):**
```
24 4D 0C 66 [12 bytes IMU data...] [checksum]
└─────────────────
0x24 = '$' start
0x4D = 'M' direction (FC → host)
0x0C = 12 decimal (12 bytes payload)
0x66 = 102 decimal (IMU command, echoed)
[12 bytes from accel/gyro sensor]
[calculated checksum]
```

---

## MSP Command Reference

### Commands Used in R.A.D

This table lists the 14 MSP commands polled in each `communicationLoop()` cycle (10 ms at 100 Hz):

| ID | Name | Purpose | Payload Size | Poll Rate |
|:--:|------|---------|:---:|:---:|
| 101 | `MSP_STATUS` | FC cycle time, armed, flight mode, sensors | 11 | Every tick |
| 102 | `MSP_RAW_IMU` | Raw accel/gyro counts | 12 | Every tick |
| 104 | `MSP_MOTOR` | Motor outputs (µs) | 2×N (N=motor count, typically 8) | Every tick |
| 105 | `MSP_RC` | RC channel inputs | 2×N (N=channel count, typically 18) | Every tick |
| 106 | `MSP_RAW_GPS` | GPS position, speed, fix type | 16 | Every tick |
| 107 | `MSP_COMP_GPS` | GPS home distance/bearing | 5 | Every tick |
| 108 | `MSP_ATTITUDE` | Roll, pitch, yaw (fused) | 6 | Every tick |
| 109 | `MSP_ALTITUDE` | Barometer altitude, vario | 6 | Every tick |
| 110 | `MSP_ANALOG` | Battery voltage, current, mAh, RSSI | 7 | Every tick |
| 121 | `MSP_NAV_STATUS` | GPS nav engine state | 7 | Every tick |
| 150 | `MSP_STATUS_EX` | Extended FC status, arming flags | 30 | Every tick |
| 164 | `MSP_GPS_SV_INFO` | Satellite list (GPS constellation) | 1 + 3×N (N ≤ 16) | ~1s (100 ticks) |
| 205 | `MSP_ACC_CAL` | Calibrate accelerometer | — | On demand |
| 206 | `MSP_MAG_CAL` | Calibrate magnetometer | — | On demand |
| 242 | `MSP_BATTERY_STATE` | Full battery detail (cells, capacity, %) | 13 | ~500ms (50 ticks) |
| 245 | `MSP_SET_PASSTHROUGH` | Enable GPS passthrough to NEO-M10 | — | On demand |
| 254 | `MSP_DEBUG` | Magnetometer X/Y/Z (with debug_mode=MAG_CALIB) | 8 | Every tick |

### Core Telemetry Payloads

#### MSP_STATUS (101) — 11 bytes

```cpp
struct MSPStatus {
	uint16_t cycleTime;          // FC loop cycle time (ms)
	uint16_t i2cErrorCount;      // I2C bus errors
	uint16_t sensorStatus;       // Bitmask (GYRO, ACC, BARO, MAG, GPS, etc.)
	uint32_t flightModeFlags;    // Bitmask (ARM, ANGLE, HORIZON, etc.)
	uint8_t  profile;            // Active PID profile (0-based)
};
```

**Byte Layout:**
```
[0..1]  cycleTime (little-endian uint16)
[2..3]  i2cErrorCount
[4..5]  sensorStatus
[6..9]  flightModeFlags
[10]    profile
```

#### MSP_RAW_IMU (102) — 12 bytes

```cpp
struct MSPRawIMU {
	int16_t accX, accY, accZ;    // Accelerometer (MPU-6500, ±4g → 8192 LSB/g)
	int16_t gyroX, gyroY, gyroZ; // Gyroscope (±2000°/s → 16.4 LSB/°/s)
};
```

**Byte Layout (little-endian):**
```
[0..1]   accX
[2..3]   accY
[4..5]   accZ
[6..7]   gyroX
[8..9]   gyroY
[10..11] gyroZ
```

**Example Parsing (C++):**
```cpp
int16_t ax = (int16_t)(buf[0] | (buf[1] << 8));  // Little-endian
int16_t ay = (int16_t)(buf[2] | (buf[3] << 8));
// ... etc.

// Convert to g and °/s
float acc_g = ax / 8192.0f;
float gyro_dps = gx / 16.4f;
```

#### MSP_ATTITUDE (108) — 6 bytes

```cpp
struct MSPAttitude {
	int16_t roll;   // Degrees × 10 (±1800 = ±180°)
	int16_t pitch;  // Degrees × 10 (±900 = ±90°)
	int16_t yaw;    // Degrees (0–359)
};
```

**Byte Layout (little-endian):**
```
[0..1]  roll
[2..3]  pitch
[4..5]  yaw
```

#### MSP_ALTITUDE (109) — 6 bytes

```cpp
struct MSPAltitude {
	int32_t altitude;    // cm (can be negative for below takeoff point)
	int16_t vario;       // cm/s (vertical speed)
};
```

**Byte Layout (little-endian):**
```
[0..3]  altitude (int32)
[4..5]  vario (int16)
```

#### MSP_ANALOG (110) — 7 bytes

```cpp
struct MSPAnalog {
	uint8_t  rssi;              // 0–255 (receiver signal strength)
	uint16_t batteryVoltage;    // 0.1V per unit (e.g., 120 = 12.0V)
	uint16_t mAhDrawn;          // milliamp-hours consumed
	uint16_t amperage;          // 0.01A per unit (e.g., 150 = 1.5A)
};
```

**Byte Layout (little-endian):**
```
[0]     rssi
[1..2]  batteryVoltage
[3..4]  mAhDrawn
[5..6]  amperage
```

#### MSP_RAW_GPS (106) — 16 bytes

```cpp
struct MSPRawGPS {
	uint8_t  fixType;       // 0=no fix, 1=GPS, 2=DGPS, 3=PPS, 4=RTK fixed, 5=RTK float, 6=compass
	uint8_t  numSat;        // Number of satellites in solution
	int32_t  latitude;      // Degrees × 1e7 (e.g., 407128000 = 40.7128°)
	int32_t  longitude;     // Degrees × 1e7
	int32_t  altitude;      // Meters (altitude above mean sea level)
	uint16_t speed;         // cm/s (ground speed)
	uint16_t course;        // Degrees (heading) 0–359
	uint16_t hdop;          // Horizontal dilution of precision (e.g., 100 = 1.0)
};
```

**Byte Layout (little-endian):**
```
[0]     fixType
[1]     numSat
[2..5]  latitude (int32)
[6..9]  longitude (int32)
[10..13] altitude (int32)
[14..15] speed (uint16)
[16..17] course (uint16)
[18..19] hdop (uint16)
```

#### MSP_COMP_GPS (107) — 5 bytes

```cpp
struct MSPCompGPS {
	uint16_t distanceToHome;    // meters
	int16_t  directionToHome;   // degrees (0–359)
	uint8_t  update;            // heartbeat flag (0 or 1, toggles each update)
};
```

#### MSP_MOTOR (104) — 2×N bytes

```cpp
struct MSPMotor {
	uint16_t motor[MAX_MOTORS];  // µs: 1000 = min, 2000 = max
};
```

**Example (4 motors):**
```
[0..1]   motor 1 (uint16 little-endian)
[2..3]   motor 2
[4..5]   motor 3
[6..7]   motor 4
```

#### MSP_RC (105) — 2×N bytes

```cpp
struct MSPRC {
	uint16_t channel[MAX_RC_CHANNELS];  // µs: 1000–2000
};
```

**MODE 2 (Standard) Layout:**
```
[0]  Roll (aileron) — 1000–2000 µs
[1]  Pitch (elevator) — 1000–2000 µs
[2]  Throttle — 1000–2000 µs
[3]  Yaw (rudder) — 1000–2000 µs
[4]  ARM switch — 1000–2000 µs
[5]  Flight mode — 1000–2000 µs
[6+] AUX channels (3–10)
```

#### MSP_STATUS_EX (150) — 30 bytes

```cpp
struct MSPStatusEX {
	uint32_t armingDisableFlags;  // Why can't we arm? (bitmask)
	// ... other fields
};
```

**Bytes [20–23]** contain the `armingDisableFlags` bitmask.

#### MSP_DEBUG (254) — 8 bytes

When `debug_mode=MAG_CALIB` in Betaflight CLI:

```cpp
struct MSPDebugMag {
	int16_t magX, magY, magZ;  // Magnetometer counts
	int16_t reserved;           // Unused
};
```

**Byte Layout (little-endian):**
```
[0..1]  magX
[2..3]  magY
[4..5]  magZ
[6..7]  reserved
```

#### MSP_BATTERY_STATE (242) — 13 bytes

```cpp
struct MSPBatteryState {
	uint8_t  cellCount;             // Auto-detected
	uint16_t capacity;              // mAh (design capacity)
	uint16_t voltage;               // 0.01V per unit (e.g., 1200 = 12.00V)
	uint16_t mAhDrawn;              // Consumed
	uint16_t amperage;              // 0.01A per unit
	uint8_t  batteryState;          // 0=OK, 1=WARNING, 2=CRITICAL, 3=NOT_PRESENT
	uint16_t voltageLowestCell;     // Lowest cell voltage (0.01V per unit)
};
```

#### MSP_GPS_SV_INFO (164) — 1 + 3×N bytes

```cpp
struct MSPSVInfo {
	uint8_t count;              // Number of satellites (N)
	SVInfoEntry entries[N];     // Array of satellite entries
};

struct SVInfoEntry {
	uint8_t sv;     // Satellite ID
	uint8_t quality;// Signal quality (0, 1, 2, 3)
	uint8_t cno;    // dBHz
	int8_t  elev;   // Elevation (always 0 in MSP mode)
	int16_t azim;   // Azimuth (always 0 in MSP mode)
};
```

**Byte Layout:**
```
[0]     count (N)
[1..3]  entry 0: [sv, quality, cno, elev/2, azim_lo, azim_hi]
[4..6]  entry 1
...
[1+3*N] entry N-1
```

---

## Polling Cadence & Scheduling

### The 10 ms Cycle

`communicationLoop()` runs at **100 Hz** (10 ms per cycle) in a worker thread:

```cpp
void DroneLink::communicationLoop() {
	while (keepRunning.load()) {
		auto loopStart = std::chrono::steady_clock::now();

		// Poll all 14 MSP commands in order (see code for exact sequence)
		// Total time: typically 50–120 ms for all polls + parsing

		// Update shared state
		{
			std::lock_guard<std::mutex> lock(dataMutex);
			currentState = pending;
		}

		// Sleep remainder of 10 ms (or 20 ms if POLL_INTERVAL_MS = 20)
		std::this_thread::sleep_until(loopStart + std::chrono::milliseconds(POLL_INTERVAL_MS));
	}
}
```

### CPU Budget per Cycle

**10 ms (100 Hz) allocation:**

| Task | Est. Time | Notes |
|------|-----------|-------|
| 12 fast polls | 12–24 ms | STATUS, RAW_IMU, ATTITUDE, etc. (1–2 ms each) |
| 2 slow polls | 2–4 ms | BATTERY_STATE (every 50 ticks), SV_INFO (every 100 ticks) |
| Calibration checks | <1 ms | Atomic flag exchange |
| State update | <1 ms | Mutex copy to currentState |
| Sleep | ~70–80 ms | Consumed in `sleep_until()` until next tick |

**Total loop time:** ~100 ms for one cycle (10 ticks at 10 ms each)

**If overrunning:** Set `POLL_INTERVAL_MS = 20` for 50 Hz (doubles available time per cycle).

### Throttled Polling

Some commands polled less frequently to reduce load:

```cpp
// Satellite list — every ~1 second (100 ticks)
if (svPollTickCounter_ <= 0) {
	pollSatellitesMSP(pending);
	svPollTickCounter_ = SV_POLL_TICKS;  // Reset to 100
}
--svPollTickCounter_;

// Battery state — every ~500 ms (50 ticks)
if (slowPollTickCounter_ <= 0) {
	auto buf = sendMSP(MSP::BATTERY_STATE);
	parseBatteryState(buf, pending);
	slowPollTickCounter_ = SLOW_POLL_TICKS;  // Reset to 50
}
--slowPollTickCounter_;
```

---

## Betaflight Integration Details

### Confirmed Hardware & FC Config

| Aspect | Setting |
|--------|---------|
| **FC Model** | F405 V3 (or compatible) |
| **Betaflight Version** | 4.5.x (tested) |
| **USB Baud** | 57600 (MSP port) |
| **GPS UART** | UART1, 115200 (internal to FC) |
| **Gyro** | MPU-6500 (±2000°/s, ±4g) |
| **Magnetometer** | QMC5883L (debug_mode=MAG_CALIB required) |
| **Barometer** | BMP280 |
| **GPS** | u-blox NEO-M10 |

### Baud Rate Architecture

**Two independent baud rates:**

```
DroneLink (57600)
	║ MSP protocol
	║
	▼
Betaflight FC (USB/MSP port)
	║ (internal I2C/UART)
	║
	▼
NEO-M10 GPS (115200) — set in BF Configurator → Ports
```

**Key Points:**

1. **MSP Baud (57600)** — confirmed by `sv_baud_probe.py` script
   - Used for all MSP commands
   - Set in `openSerialPort()`:
	 ```cpp
	 dcb.BaudRate = MSP_BAUD;  // 57600
	 ```

2. **GPS UART Baud (115200)** — managed by Betaflight
   - NEO-M10 communicates at 115200 with FC
   - BF applies auto-baudrate detection if `gps_auto_baud=ON`
   - **DroneLink never touches this**; BF handles internally

### Betaflight Flight Modes & Boxes

Flight modes in Betaflight are controlled by **auxiliary channels** (RC channel 5 and above), not directly by MSP.

**Typical Configuration (MODE 2):**

```
RC Channel 5 (Flight Mode Switch)
├─ Low (<1400 µs) → ACRO
├─ Mid (1400–1600 µs) → ANGLE
└─ High (>1600 µs) → HORIZON

AUX boxes configured in BF Configurator → Modes
```

The `flightModeName` in `DroneState` is computed from `flightModeFlags` (not directly from RC channels):

```cpp
// DroneLink.cpp: parseStatus()
// Decodes flightModeFlags bitmask to human-readable name
std::string name;
if (flags & FlightMode::ARM) {
	if (flags & FlightMode::HORIZON) name = "HORIZON";
	else if (flags & FlightMode::ANGLE) name = "ANGLE";
	else name = "ACRO";

	if (flags & FlightMode::MAG) name += " [MAG-HOLD]";
	if (flags & FlightMode::FAILSAFE) name = "FAILSAFE";
} else {
	name += " [DISARMED]";
}
```

### Sensor Configuration

**Gyroscope & Accelerometer (MPU-6500):**
- Fixed ±2000°/s gyro range
- Fixed ±4g accel range
- Configured by hardware, not user-settable

**If your FC uses different ranges** (e.g., MPU-6050, ICM-20602), adjust scale constants:
```cpp
// IMUSensor.h
static constexpr float ACC_SCALE = 1.0f / 8192.0f;   // Change for different accel range
static constexpr float GYRO_SCALE = 1.0f / 16.4f;    // Change for different gyro range
```

**Barometer (BMP280):**
- Auto-detected on I2C bus by Betaflight
- Altitude correction set in BF Configurator → Board Alignment

**Magnetometer (QMC5883L):**
- Requires `debug_mode=MAG_CALIB` in BF CLI (not the Configurator)
- Calibration performed via `startMagCalibration()`
- Raw X/Y/Z exposed via `MSP_DEBUG (254)`

---

## GPS Passthrough Mode

### Concept

**Passthrough** allows DroneLink to **directly communicate with the NEO-M10 GPS** without Betaflight's GPS stack interfering.

This is useful for:
- Advanced GPS configuration (RTCM corrections, RTK setup, survey-in)
- Firmware updates
- Direct UBX protocol commands

### Activation Sequence

**Prerequisites:**
- Betaflight config: `gps_auto_config=OFF` (in CLI)
- FC must have GPS connected on the configured UART

**Steps:**

1. **Send MSP_SET_PASSTHROUGH (245)** with UART index

```cpp
// DroneLink method
uint8_t gpsUartIndex = 0;  // UART1 (0-based)
std::vector<uint8_t> payload = {gpsUartIndex};
sendMSP(MSP::SET_PASSTHROUGH, payload);

// Encodes to MSP frame:
// [$][ <][1][0xF5][0][checksum]
```

2. **FC responds with ACK**

```
[$][M][0][0xF5][checksum]
```

3. **After ACK, FC enters raw byte-forwarding mode**
   - All subsequent bytes are forwarded to/from NEO-M10
   - **NO MSP framing** — raw UBX protocol only

4. **Send UBX commands** (native u-blox protocol)

```cpp
// Example: Request NAV_PVT (position/velocity/time)
// UBX-NAV-PVT query:
// [0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, checksum_a, checksum_b]

// DroneLink just forwards these bytes as-is
std::vector<uint8_t> ubxQuery = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, ...};
writeSerial(ubxQuery);
```

5. **Read UBX response** (raw bytes, no MSP wrapping)

```cpp
// Receive raw UBX response:
// [0xB5, 0x62, 0x01, 0x07, payload_len, payload_len>>8, ...data..., ckA, ckB]
std::vector<uint8_t> ubxResponse = readSerial();
```

### Exiting Passthrough Mode

**Betaflight 4.x NEVER exits passthrough on its own.**

**Recovery:** **Close + reopen the serial port**

The USB chip (CP210x or CH340) generates a hardware reset signal when the port is closed, returning the FC to MSP mode.

```cpp
// In application
drone.disconnect();  // Close port (triggers FC reset)
std::this_thread::sleep_for(std::chrono::milliseconds(250));  // Wait for reset
drone.connect("COM3");  // Reopen port; FC now in MSP mode
```

### When to Use

**Use passthrough if:**
- You need RTK corrections or RTCM injection
- Configuring NEO-M10 survey-in mode
- Updating GPS firmware
- Accessing advanced UBX features (Assisted GNSS, etc.)

**Use normal MSP if:**
- Standard GPS position/velocity/time is sufficient
- Lower complexity preferred (current R.A.D setup)

**R.A.D Current Mode:** Normal MSP (no passthrough). GPS auto-config is ON; BF handles GPS.

---

## Debugging & Troubleshooting

### Serial Port Issues

**Problem:** Connection fails or times out

**Diagnosis:**

```cpp
if (!drone.connect("COM3")) {
	// Port open failed — check:
	// 1. COM port number (Device Manager)
	// 2. Cable connected
	// 3. FC powered on
	// 4. FC bootloader (not firmware) — wrong baud
}
```

**Test with `msp_probe.py`:**
```powershell
python msp_probe.py -p COM3 -b 57600
# Should print status, motor values, etc.
```

### Checksum Errors

**Symptom:** Frequent MSP_ERROR responses or no responses

**Cause:** Corrupted data or baud rate mismatch

**Fix:**

1. Verify baud rate (should be 57600)
   ```cpp
   // sv_baud_probe.py will confirm
   ```

2. Check cable quality (USB), especially long/cheap cables

3. Add USB isolator if grounding issues suspected

### Latency / Timeout

**Symptom:** `lastRttMs` > 50 ms, or frequent MSP failures

**Causes:**
- FC CPU overloaded (high PID rate, heavy feature load)
- EMI interference (motor noise coupling)
- Low buffer timeout (too aggressive)

**Solutions:**

1. **Reduce polling rate:**
   ```cpp
   drone.setPollIntervalMs(20);  // 50 Hz instead of 100 Hz
   ```

2. **Disable non-essential features in BF:**
   - Reduce loop rate to 2 kHz (or lower)
   - Disable blackbox logging
   - Disable telemetry providers (FrSky, etc.)

3. **Check USB power:**
   ```cpp
   // Monitor battery voltage and current
   if (state.rssi == 0) {
	   std::cerr << "RX signal lost or very weak" << std::endl;
   }
   ```

### GPS Not Responding

**Symptom:** `gps.valid = false`, no satellites

**Causes:**

1. **GPS not connected** — Check UART1 in BF Configurator → Ports
2. **Baud mismatch** — NEO-M10 baud should be 115200 (internal to FC)
3. **Cold start** — GPS needs 1–5 minutes for first fix
4. **No sky view** — Move outside, away from buildings

**Diagnosis (Python):**

```python
state = hub.getLatestState()
print(f"GPS Fix: {state.gps.fixType}")  # 0=no fix, 1=GPS, 2=DGPS, etc.
print(f"Satellites: {state.gps.numSat}")
print(f"HDOP: {state.gps.hdop / 100:.1f}")  # Good if < 2.0
```

### Magnetometer Invalid

**Symptom:** `magValid = false`, heading erratic

**Cause:** Magnetometer not calibrated

**Fix:**

1. Set `debug_mode=MAG_CALIB` in BF CLI:
   ```
   set debug_mode=MAG_CALIB
   save
   ```

2. Run calibration via DroneLink:
   ```cpp
   drone.startMagCalibration();

   // In polling loop:
   while (drone.getLatestState().magCalActive) {
	   // Display countdown, rotate drone
   }
   ```

3. Manually rotate drone on all axes for 30 seconds

4. Verify `magValid` becomes true in next state snapshot

### Sensor Status Red Flags

**Gyro Missing:**
```cpp
if (!(state.sensorStatus & SensorStatus::GYRO)) {
	std::cerr << "GYRO NOT RESPONDING — Cannot fly!" << std::endl;
}
```

**Cannot Arm — Multiple Reasons:**
```cpp
if (state.armingDisableFlags != 0) {
	std::cout << "Arming disabled: " << state.armingDisableStr << std::endl;
	// Examples: "Throttle not at minimum", "Not level", "Calibrating"
}
```

---

## Performance Optimization

### Reducing MSP Poll Load

**Current Load:**
- 12 fast polls every 10 ms
- 2 throttled polls every 500–1000 ms
- Total throughput: ~50–80 bytes/sec

**If overloaded:**

1. **Drop poll interval to 20 ms (50 Hz):**
   ```cpp
   drone.setPollIntervalMs(20);
   ```
   Halves polling frequency; doubles time per cycle.

2. **Remove unnecessary polls:**
   ```cpp
   // Comment out in communicationLoop() if not needed:
   // { auto buf = sendMSP(MSP::BATTERY_STATE); parseBatteryState(...); }
   // { auto buf = sendMSP(MSP::GPS_SV_INFO); ... }
   ```

3. **Reduce satellite poll rate:**
   ```cpp
   static constexpr int SV_POLL_TICKS = 200;  // Every ~2s instead of ~1s
   ```

### Optimizing Python Telemetry Worker

**Current Config:**
```python
POLL_HZ = 60          # 60 Hz polling
_QUEUE_DEPTH = 3      # Buffer 3 frames max
RECONNECT_S = 2.0     # Retry every 2 seconds
```

**For low-power host:**
```python
POLL_HZ = 30          # Drop to 30 Hz (still responsive)
_QUEUE_DEPTH = 2      # Buffer 2 frames
```

**For headless / logging:**
```python
POLL_HZ = 10          # Minimal load
_QUEUE_DEPTH = 1      # No buffer
```

### Batching Commands

If you need to **send multiple MSP commands** without waiting for responses:

```cpp
// Current: Synchronous — wait for each response
auto buf1 = sendMSP(MSP::STATUS);      // Blocks until response
auto buf2 = sendMSP(MSP::RAW_IMU);     // Then polls this

// Batching would require rewriting communicationLoop()
// Not currently implemented in R.A.D
```

### Checksum Optimization

Current implementation: XOR-based checksum (fast, 8-bit)

```cpp
uint8_t crc = 0;
for (uint8_t b : payload) {
	crc ^= b;
}
// No performance gain possible; already optimal
```

---

## Summary

| Topic | Key Points |
|-------|-----------|
| **MSP Frame** | `[$][direction][size][cmd][payload][checksum]` |
| **Checksum** | Fletcher 8-bit (XOR folding) |
| **Baud Rate** | 57600 (MSP), 115200 (GPS internal) |
| **Poll Cycle** | 100 Hz (10 ms) with 14 MSP commands |
| **Betaflight** | 4.5.x, F405 V3, MSP passthrough optional |
| **Debugging** | Use `msp_probe.py`, check baud, verify sensors |
| **Optimization** | Reduce poll rate, disable non-essential features, isolate USB |

R.A.D's architecture achieves low-latency, reliable communication with Betaflight flight controllers through careful MSP scheduling and robust error handling. The ~10 ms cycle time provides responsive telemetry suitable for real-time monitoring and safety-critical functions like arming/disarming and calibration.

