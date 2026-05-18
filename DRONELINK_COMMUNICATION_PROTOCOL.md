# DroneLink Communication Protocol & Message Decoding

## Overview

**DroneLink** is the C++ telemetry backend that handles all communication between the host PC and a Betaflight flight controller (FC) connected via USB serial. It:

1. Opens a serial port connection (57600 baud, 8N1)
2. Sends MSP (Multiwii Serial Protocol) requests in a worker thread
3. Receives and parses MSP responses
4. Updates a shared `DroneState` structure with telemetry data
5. Exposes the state to Python via pybind11 bindings

This document covers the complete communication flow from serial I/O through message parsing to Python exposure.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Betaflight FC (XFlight F405 V3)              │
│                   NEO-M10 GNSS module connected to UART1             │
└────────────────────────────┬────────────────────────────────────────┘
							 │
							 │ USB Serial @ 57600 baud (MSP protocol)
							 │
┌────────────────────────────▼────────────────────────────────────────┐
│                        DroneLink (C++ Backend)                      │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ Worker Thread: communicationLoop() [100 Hz = 10 ms/iteration]│  │
│  │                                                               │  │
│  │ MSP Request/Response Cycle (per iteration):                 │  │
│  │  1. sendMSP(cmd_id) → construct frame & write to serial     │  │
│  │  2. readMSP() → receive & validate response                 │  │
│  │  3. parseXXX(buf, pending) → extract fields                 │  │
│  │  4. Repeat for 13 different MSP commands                    │  │
│  │  5. commitState(pending) → update shared state (mutex lock) │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ Main Thread / Python: getLatestState()                      │  │
│  │   ├─ lock_guard<mutex>                                      │  │
│  │   └─ return copy of DroneState (all fields accessible)      │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────────────────┬────────────────────────────────────────┘
							 │
							 │ pybind11 bindings
							 │
┌────────────────────────────▼────────────────────────────────────────┐
│                    Python Tkinter GUI (DroneCockpitUI)              │
│                                                                      │
│  state = drone_link.get_latest_state()                              │
│  state.ax, state.ay, state.az              # Raw IMU accel          │
│  state.roll_deg, state.pitch_deg, state.yaw_deg  # Attitude         │
│  state.gps.latitude, state.gps.longitude        # Position          │
│  state.sv_list                                 # Satellite list     │
│  ... and 30+ other telemetry fields available                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## MSP Protocol Basics

### Frame Format

All MSP communication uses a simple binary framing format:

**Request (Host → FC):**
```
Byte 0    Byte 1    Byte 2    Byte 3      Byte 4      Byte 5          Bytes 6..N      Byte N+1
──────────────────────────────────────────────────────────────────────────────────────────────
'$'       'M'       '<'       payload_len command_id  command_id      [payload bytes] checksum
0x24      0x4D      0x3C      1 byte      1 byte      1 byte          optional        1 byte
					(request                           (XOR csum)
					 direction)
```

**Response (FC → Host):**
```
Byte 0    Byte 1    Byte 2    Byte 3      Byte 4      Bytes 5..N      Byte N+1
─────────────────────────────────────────────────────────────────────────────
'$'       'M'       '>'       payload_len command_id  [payload bytes] checksum
0x24      0x4D      0x3E      1 byte      1 byte      N bytes         1 byte
					(response                         (FC-generated    (XOR csum)
					 direction)                        data)
```

**Checksum Calculation:**
```cpp
uint8_t csum = payload_len ^ command_id;
for (uint8_t byte : payload)
	csum ^= byte;
```

### MSP Command IDs

DroneLink polls the following commands (from `DroneLink.h`):

| Cmd ID | Name | Interval | Payload Size | Purpose |
|--------|------|----------|--------------|---------|
| 101 | MSP_STATUS | Every tick | ~10-15 | FC cycle time, armed state, sensor status |
| 102 | MSP_RAW_IMU | Every tick | 12 | Raw accel (3×i16) + gyro (3×i16) from MPU-6500 |
| 104 | MSP_MOTOR | Every tick | 8 | Motor throttle outputs (4×u16 PWM) |
| 105 | MSP_RC | Every tick | ~32 | RC channel inputs (up to 16×u16) |
| 106 | MSP_RAW_GPS | Every tick | 18 | GPS fix, satellites, position, speed, HDOP |
| 107 | MSP_COMP_GPS | Every tick | 5 | Distance & bearing to home, GPS heartbeat |
| 108 | MSP_ATTITUDE | Every tick | 6 | Roll, pitch, yaw (3×i16 = decidegrees) |
| 109 | MSP_ALTITUDE | Every tick | 4 | Barometer altitude + vario (2×i16) |
| 110 | MSP_ANALOG | Every tick | 7 | Battery voltage, current, mAh, RSSI |
| 121 | MSP_NAV_STATUS | Every tick | 7 | GPS fix validity, DGPS used, etc. |
| 150 | MSP_STATUS_EX | Every tick | 11 | Arming disable flags, flight modes |
| 164 | MSP_GPS_SV_INFO | Every ~1s | 129 | **Satellite list** (32 channels × 4 bytes) |
| 242 | MSP_BATTERY_STATE | Every ~500ms | 12 | Full battery detail (voltage, current, capacity) |
| 254 | MSP_DEBUG | Every tick | 8 | Debug values (used for mag calibration) |

---

## Communication Loop in Detail

### 1. Worker Thread Initialization (DroneLink.cpp:89-99)

```cpp
bool DroneLink::connect(const std::string& portName) {
	if (connected.load()) disconnect();

	portName_ = portName;

	if (!openSerialPort(portName)) return false;

	connected.store(true);
	keepRunning.store(true);
	workerThread = std::thread(&DroneLink::communicationLoop, this);
	return true;
}
```

**What happens:**
- Opens the serial port at 57600 baud (8 data bits, 1 stop bit, no parity)
- Spawns a worker thread running `communicationLoop()`
- Returns immediately; thread runs in background

### 2. Serial Port Configuration (DroneLink.cpp:36-76)

```cpp
bool DroneLink::openSerialPort(const std::string& portName) {
	// Construct full path: COM3 → \\.\COM3
	std::string fullPath = "\\\\.\\" + portName;

	// Open handle
	hSerial = CreateFileA(
		fullPath.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0, NULL, OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL, NULL);

	if (hSerial == INVALID_HANDLE_VALUE) return false;

	// Get current port settings
	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(hSerial, &dcb)) return false;

	// Configure for Betaflight MSP
	dcb.BaudRate = 57600;           // Confirmed by sv_baud_probe.py
	dcb.ByteSize = 8;               // 8 data bits
	dcb.StopBits = ONESTOPBIT;      // 1 stop bit
	dcb.Parity = NOPARITY;          // No parity
	dcb.fOutxCtsFlow = FALSE;       // No CTS flow control
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutxDsrFlow = FALSE;       // No DSR flow control
	dcb.fDtrControl = DTR_CONTROL_DISABLE;

	if (!SetCommState(hSerial, &dcb)) return false;

	// Configure timeouts
	COMMTIMEOUTS to = { 0 };
	to.ReadIntervalTimeout = MAXDWORD;           // 0 means block until data
	to.ReadTotalTimeoutMultiplier = MAXDWORD;    // Don't use multiplier
	to.ReadTotalTimeoutConstant = 1;             // 1 ms timeout
	to.WriteTotalTimeoutConstant = 50;           // 50 ms write timeout
	to.WriteTotalTimeoutMultiplier = 2;          // +2 ms per byte

	SetCommTimeouts(hSerial, &to);
	return true;
}
```

**Key points:**
- **Baud rate:** 57600 — This is the MSP communication speed between the host and FC
- **Flow control disabled:** Modern USB serial chips (CP210x, CH340) don't need it
- **Timeouts:** Short timeouts prevent the program hanging if FC is disconnected

### 3. Main Communication Loop (DroneLink.cpp:143-260)

```cpp
void DroneLink::communicationLoop() {
	int consecutiveFails = 0;

	while (keepRunning.load()) {
		auto loopStart = std::chrono::steady_clock::now();

		DroneState pending;  // Local working copy
		{
			std::lock_guard<std::mutex> lock(dataMutex);
			pending = currentState;  // Start with last known state
		}

		// ── Calibration state tracking ────────────────────────────────────
		if (magCalRequested.exchange(false)) {
			sendMSP(MSP::MAG_CAL);
			magCalActive_ = true;
			magCalStartTime_ = std::chrono::steady_clock::now();
		}

		// ... mag cal countdown logic ...

		// ── Regular MSP polls (every iteration) ────────────────────────────
		{ auto buf = sendMSP(MSP::STATUS);     parseStatus(buf, pending); }
		{
			auto t0 = std::chrono::high_resolution_clock::now();
			auto buf = sendMSP(MSP::RAW_IMU);
			auto t1 = std::chrono::high_resolution_clock::now();
			if (parseIMU(buf, pending)) {
				std::chrono::duration<double, std::milli> rtt = t1 - t0;
				pending.lastRttMs = rtt.count();
				anySuccess = true;
			}
		}

		{ auto buf = sendMSP(MSP::ATTITUDE);   parseAttitude(buf, pending); }
		{ auto buf = sendMSP(MSP::ANALOG);     parseAnalog(buf, pending); }
		{ auto buf = sendMSP(MSP::DEBUG);      parseDebug(buf, pending); }
		{ auto buf = sendMSP(MSP::ALTITUDE);   parseBaro(buf, pending); }
		{ auto buf = sendMSP(MSP::RAW_GPS);    parseGPSRaw(buf, pending); }
		{ auto buf = sendMSP(MSP::COMP_GPS);   parseGPSComp(buf, pending); }
		{ auto buf = sendMSP(MSP::NAV_STATUS); parseNavStatus(buf, pending); }

		// ── Satellite list — polled every ~1 second ────────────────────────
		if (svPollTickCounter_ <= 0) {
			pollSatellitesMSP(pending);
			svPollTickCounter_ = SV_POLL_TICKS;  // Reset to 100
		}
		--svPollTickCounter_;

		// ── Battery state — polled every ~500 ms ───────────────────────────
		if (slowPollTickCounter_ <= 0) {
			auto buf = sendMSP(MSP::BATTERY_STATE);
			parseBatteryState(buf, pending);
			slowPollTickCounter_ = SLOW_POLL_TICKS;  // Reset to 50
		}
		--slowPollTickCounter_;

		// ── Health tracking ────────────────────────────────────────────────
		if (anySuccess) {
			consecutiveFails = 0;
			pending.linkHealthy = true;
			pending.packetCount++;
		} else {
			if (++consecutiveFails >= FAIL_THRESHOLD)
				pending.linkHealthy = false;
		}

		// ── Commit to shared state ─────────────────────────────────────────
		commitState(pending);

		// ── Maintain 10 ms (100 Hz) loop rate ──────────────────────────────
		auto elapsed = std::chrono::steady_clock::now() - loopStart;
		auto budget = std::chrono::milliseconds(pollIntervalMs.load());
		if (elapsed < budget)
			std::this_thread::sleep_for(budget - elapsed);
	}
}
```

**Loop timing:**
- Runs at 100 Hz (10 ms per iteration)
- 13 MSP requests per iteration × 1-2 ms each = 13-26 ms of actual communication
- Compensates by sleeping if done early, or warns about overrun if not

**Key logic:**
- `pending` is a local copy that accumulates all parsed data
- `svPollTickCounter_` decrements each iteration; when it reaches 0, satellite data is polled (then reset to 100 = ~1 second)
- `slowPollTickCounter_` similarly throttles battery state (every ~500 ms)
- After all updates, `commitState()` atomically swaps pending into the shared state

### 4. MSP Request/Response Cycle (DroneLink.cpp:442-510)

```cpp
std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
	if (hSerial == INVALID_HANDLE_VALUE) return {};

	// ── Write request ──────────────────────────────────────────────────────
	uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
	//               preamble    payload_len  cmd    checksum (cmd itself for
	//                           (0 = no payload)          zero-payload requests)

	DWORD written = 0;
	if (!WriteFile(hSerial, req, sizeof(req), &written, NULL)
		|| written != sizeof(req))
		return {};

	// ── Read response ──────────────────────────────────────────────────────
	constexpr size_t MAX_FRAME = 512;
	uint8_t raw[MAX_FRAME];
	size_t  total = 0;
	int     payloadLen = -1;

	auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(80);

	// Read bytes until we have a complete frame or timeout
	while (total < MAX_FRAME) {
		if (std::chrono::steady_clock::now() > deadline) break;

		DWORD rd = 0;
		uint8_t b = 0;
		if (!ReadFile(hSerial, &b, 1, &rd, NULL) || rd == 0) {
			std::this_thread::sleep_for(std::chrono::microseconds(150));
			continue;
		}

		raw[total++] = b;

		// Once we have 4 bytes, we know payload_len (at byte 3)
		if (total == 4) payloadLen = raw[3];

		// Complete frame is: 6 bytes (header/checksum) + payloadLen
		if (payloadLen >= 0 && total == static_cast<size_t>(6 + payloadLen))
			break;
	}

	// ── Validate response ──────────────────────────────────────────────────
	if (total < 6) return {};                           // Too short
	if (raw[0] != '$' || raw[1] != 'M' || raw[2] != '>') {
		PurgeComm(hSerial, PURGE_RXCLEAR);             // Wedged — clear buffer
		return {};
	}
	if (raw[4] != mspID) return {};                     // Wrong command ID

	// ── Return payload (bytes 5 to 5+payloadLen-1) ──────────────────────────
	std::vector<uint8_t> result(raw, raw + total);
	return result;
}
```

**What happens:**
1. Constructs a request frame: `$ M < [0 payload bytes] cmd_id checksum`
2. Writes the 6-byte request to the serial port
3. Reads bytes one at a time until a complete response is received (or timeout)
4. Validates the response (correct preamble `$M>`, correct command ID echo)
5. Returns the complete frame (including headers) so the parser can extract the payload

**Error handling:**
- If read times out or header is corrupted → return empty vector
- Parser checks for empty response and skips the update (retains last known value)

---

## Message Parsing

Each MSP response is parsed by a dedicated function. Here are key examples:

### Example 1: MSP_RAW_GPS (cmd 106) — GPS Position

**FC Response Payload Layout:**
```
Byte 0      fix_type    (0=no fix, 1=dead reckon, 2=2D, 3=3D)
Byte 1      num_sat     (number of satellites used in solution)
Bytes 2-5   latitude    (i32 little-endian, degrees × 1e7)
Bytes 6-9   longitude   (i32 little-endian, degrees × 1e7)
Bytes 10-11 altitude    (u16 little-endian, centimetres)
Bytes 12-13 ground_speed(u16 little-endian, cm/s)
Bytes 14-15 ground_course(u16 little-endian, decidegrees 0-3599)
Bytes 16-17 HDOP        (u16 little-endian, × 100, 9999 = unknown)
```

**DroneLink parser (in DroneLink.cpp):**
```cpp
bool DroneLink::parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s) {
	// buf is the complete frame: [6-byte header] [payload] [checksum]
	// We need the payload starting at buf[5]

	if (buf.size() < 24) return false;  // 6 header + 18 payload

	const uint8_t* p = buf.data() + 5;  // Point to first payload byte

	s.gps.fixType      = p[0];
	s.gps.numSat       = p[1];
	s.gps.latitude     = (int32_t)(p[2] | (p[3]<<8) | (p[4]<<16) | (p[5]<<24)) / 1e7;
	s.gps.longitude    = (int32_t)(p[6] | (p[7]<<8) | (p[8]<<16) | (p[9]<<24)) / 1e7;
	s.gps.altitudeM    = (float)(p[10] | (p[11]<<8)) / 100.0f;
	s.gps.groundSpeedMs= p[12] | (p[13]<<8);
	s.gps.groundCourse = p[14] | (p[15]<<8);
	s.gps.hdop         = p[16] | (p[17]<<8);

	s.gps.rawValid = true;
	s.gps.positionUsable = (s.gps.fixType >= 2)
						&& (s.gps.numSat >= 4)
						&& (s.gps.hdop < 500);
	return true;
}
```

**Data extracted:**
- `s.gps.latitude`, `s.gps.longitude` — decimal degrees (e.g., 44.77)
- `s.gps.altitudeM` — metres above MSL
- `s.gps.groundSpeedMs` — cm/s (divide by 100 for m/s)
- `s.gps.groundCourse` — 0-3599 decidegrees (divide by 10 for degrees)
- `s.gps.positionUsable` — True only if fix is good AND has ≥4 satellites AND HDOP < 5.0

### Example 2: MSP_ATTITUDE (cmd 108) — Drone Orientation

**FC Response Payload Layout:**
```
Bytes 0-1   roll        (i16 little-endian, decidegrees -1800 to +1800 = -180° to +180°)
Bytes 2-3   pitch       (i16 little-endian, decidegrees -900 to +900 = -90° to +90°)
Bytes 4-5   yaw         (i16 little-endian, degrees 0-359, wrapping)
```

**Parser:**
```cpp
bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 11) return false;  // 6 header + 5 minimum... actually 6 header + 6 payload
	const uint8_t* p = buf.data() + 5;

	s.roll  = (int16_t)(p[0] | (p[1]<<8));      // stored as decidegrees
	s.pitch = (int16_t)(p[2] | (p[3]<<8));
	s.yaw   = (int16_t)(p[4] | (p[5]<<8));

	return true;
}
```

**Data extracted:**
- `s.roll`, `s.pitch` — in decidegrees; GUI divides by 10 for degrees
- `s.yaw` — in degrees

### Example 3: MSP_GPS_SV_INFO (cmd 164) — Satellite List

This is the critical command for satellite display. Payload layout:

```
Byte 0      numCh   (number of channels, typically 32 for NEO-M10)
Per channel (4 bytes):
  [0]       chn     (channel byte: upper nibble = GNSS ID, lower = channel index)
  [1]       svid    (satellite vehicle ID / PRN)
  [2]       quality (0-7: 0=no signal, 7=fully locked)
  [3]       cno     (carrier-to-noise ratio, dBHz, 0=not tracked)
```

**Parser (in GPSneoM10.cpp, called from DroneLink):**
```cpp
bool GPSNeoM10::parseMspSvInfo(const std::vector<uint8_t>& buf,
								std::vector<SVInfoEntry>& svList) {
	// Validate MSP frame
	if (buf.size() < 8) return false;
	if (buf[0] != '$' || buf[1] != 'M' || buf[2] != '>') return false;
	if (buf[4] != 164) return false;

	uint8_t payLen = buf[3];
	if (buf.size() < static_cast<size_t>(6 + payLen)) return false;

	uint8_t numCh = buf[5];  // First payload byte
	if (numCh == 0) {
		svList.clear();
		return true;
	}

	if (payLen < static_cast<uint8_t>(1 + numCh * 4)) return false;

	// For M10 (>16 channels), GNSS ID is in upper nibble of chn byte
	const bool gnssInHighNibble = (numCh > 16);

	std::vector<SVInfoEntry> result;
	result.reserve(numCh);

	const uint8_t* base = buf.data() + 6;  // First channel record

	for (uint8_t i = 0; i < numCh; ++i) {
		const uint8_t* rec = base + i * 4;

		uint8_t chnByte = rec[0];
		uint8_t svid    = rec[1];
		uint8_t quality = rec[2];
		uint8_t cno     = rec[3];

		uint8_t gnssId, chnIdx;
		if (gnssInHighNibble) {
			gnssId = (chnByte >> 4) & 0x0F;
			chnIdx = chnByte & 0x0F;
		} else {
			gnssId = 0;
			chnIdx = chnByte;
		}

		SVInfoEntry sv;
		sv.chn      = chnIdx;
		sv.svid     = svid;
		sv.quality  = quality;
		sv.cno      = cno;
		sv.gnssId   = gnssId;
		sv.gnssName = gnssName(gnssId);  // Maps 0→"GPS", 6→"GLONASS", etc.
		sv.used     = (quality >= 4);    // Contributes to fix
		sv.elev     = 0;                 // Not available in MSP cmd 164
		sv.azim     = 0;
		sv.prRes    = 0;

		// Status string for display
		if (sv.used) {
			sv.statusStr = "used";
		} else if (cno > 0) {
			sv.statusStr = "tracked";
		} else {
			sv.statusStr = (quality == 1 || quality == 2)
						 ? qualityStatus(quality) : "idle";
		}

		result.push_back(sv);
	}

	svList = std::move(result);
	return true;
}
```

**Data extracted:**
- A vector of `SVInfoEntry` objects, each with:
  - `gnssName` — "GPS", "GLONASS", "Galileo", "BeiDou", "SBAS", "QZSS"
  - `svid` — satellite PRN/slot number
  - `cno` — signal strength in dBHz (0-63)
  - `quality` — 0-7 (u-blox quality index)
  - `used` — boolean indicating if this satellite is used in the fix
  - `statusStr` — human-readable status ("used", "tracked", "idle", etc.)

---

## Data Sharing with Python

### Step 1: DroneState Structure (DroneLink.h)

```cpp
struct DroneState {
	// ── IMU data ────────────────────
	int16_t ax, ay, az;  // accelerometer raw ADC counts
	int16_t gx, gy, gz;  // gyroscope raw ADC counts

	// ── Attitude ────────────────────
	int16_t roll;        // decidegrees
	int16_t pitch;
	int16_t yaw;

	// ── GPS position ────────────────
	GPSReading gps;      // Contains latitude, longitude, altitude, speed, etc.

	// ── GPS satellites ──────────────
	std::vector<SVInfoEntry> svList;       // ← THE SATELLITE LIST
	bool                     svInfoValid;   // ← Data ready flag
	std::string              svSource;      // ← "MSP" or "UBX"

	// ... 20+ other fields ...
};
```

### Step 2: Thread-Safe State Export (DroneLink.cpp:112-115)

```cpp
DroneState DroneLink::getLatestState() {
	std::lock_guard<std::mutex> lock(dataMutex);
	return currentState;  // Returns a copy; safe to use from Python
}
```

**Key:** `lock_guard` ensures the worker thread doesn't modify `currentState` while Python is reading it.

### Step 3: pybind11 Bindings (Bindings.cpp:214-223)

```cpp
// Expose DroneState fields to Python
.def_readonly("sv_list", &DroneState::svList,
	"List of SVInfoEntry. Updated every ~1 s via MSP cmd 164. "
	"Fields gnss_name/svid/cno/quality/status_str/used/gnss_id are "
	"always populated. elev/azim are 0 (not available via MSP).")
.def_readonly("sv_info_valid", &DroneState::svInfoValid,
	"True when sv_list has been populated at least once (~1 s after connect).")
.def_readonly("sv_source", &DroneState::svSource,
	"'MSP' when populated via cmd 164 (normal). "
	"'UBX' when populated via passthrough (only if gps_auto_config=OFF). "
	"Check this before showing elev/azim columns in the satellite table.")
```

### Step 4: Python GUI Usage (DroneCockpitUI/GPSWidget.py)

```python
# Get latest telemetry from C++ backend
state = drone_link.get_latest_state()

# Access satellite list
if state.sv_info_valid:
	for sv in state.sv_list:
		print(f"{sv.gnss_name} {sv.svid}: CNO={sv.cno} dBHz, Status={sv.status_str}")
		if sv.used:
			print("  ↳ Contributing to fix")
```

---

## Data Flow Diagram: Request to GUI

```
Step 1: Python requests state
		┌─────────────────────┐
		│ drone.get_latest_   │
		│ state() called       │
		└────────────┬────────┘
					 │
Step 2: C++ returns copy
		┌────────────▼────────┐
		│ lock_guard acquires  │
		│ mutex, returns copy  │
		│ of currentState      │
		└────────────┬────────┘
					 │
Step 3: Python receives DroneState
		┌────────────▼────────────────────┐
		│ state.sv_list = [SVInfoEntry,   │
		│                 SVInfoEntry,    │
		│                 ...]            │
		│ state.sv_info_valid = True      │
		│ state.sv_source = "MSP"         │
		└────────────┬────────────────────┘
					 │
Step 4: Python UI renders
		┌────────────▼──────────────────────┐
		│ for sv in state.sv_list:          │
		│   if sv.used:                      │
		│     display_as_green(sv)           │
		│   elif sv.cno > 0:                 │
		│     display_as_gray(sv)            │
		│   else:                            │
		│     display_as_off(sv)             │
		└───────────────────────────────────┘
```

---

## Complete Message Sequence: Real-Time Example

### Every 100 ms (one loop iteration):

```
T=0 ms:   communicationLoop() iteration starts
		  └─ pending = copy of currentState

T=1 ms:   sendMSP(101) → MSP_STATUS request
		  ┌─ Write: $ M < [0] 101 101
		  └─ Read: $ M > [payload_len] 101 [status_bytes] [csum]
		  └─ parseStatus() extracts fc_cycle_ms, armed, sensor status
		  └─ pending.fcCycleMs = 2500  (2.5 ms per FC loop)

T=2 ms:   sendMSP(102) → MSP_RAW_IMU request
		  ├─ Parse: ax=-15, ay=42, az=4095 (raw ADC counts)
		  ├─ pending.ax = -15, pending.ay = 42, pending.az = 4095

T=3 ms:   sendMSP(108) → MSP_ATTITUDE request
		  ├─ Parse: roll=150, pitch=-45, yaw=270 (in decidegrees)
		  ├─ pending.roll = 150 (15.0° to GUI), pending.pitch = -45 (-4.5°)

T=4 ms:   sendMSP(106) → MSP_RAW_GPS request
		  ├─ Parse: lat=44.77, lon=17.21, alt=250.5m, speed=350cm/s
		  ├─ pending.gps.latitude = 44.77
		  ├─ pending.gps.groundSpeedMs = 350 (3.5 m/s)

... (continue for remaining 9 commands) ...

T=13 ms:  if (svPollTickCounter_ == 0):  [happens every 100 iterations]
		  ├─ sendMSP(164) → MSP_GPS_SV_INFO request
		  ├─ Receives 129-byte response with 32 satellites
		  ├─ parseMspSvInfo() populates:
		  │  pending.svList = [
		  │    SVInfoEntry(gnss="GPS", svid=1, cno=45, used=true, ...),
		  │    SVInfoEntry(gnss="GPS", svid=4, cno=38, used=true, ...),
		  │    ...
		  │  ]
		  ├─ pending.svInfoValid = true
		  └─ pending.svSource = "MSP"

T=25 ms:  commitState(pending)
		  ├─ lock_guard<mutex> lock(dataMutex)
		  ├─ currentState = pending  ← Atomic swap!
		  └─ lock released

T=25 ms:  [Meanwhile, in Python main thread:]
		  ├─ state = drone.get_latest_state()  [thread-safe copy]
		  ├─ Returns: state.sv_list = [array of 10 SVInfoEntry objects]
		  ├─ GUI receives state and renders satellite table
		  │  Row 1: GPS 1   | CNO: 45 dBHz | █████████░ | USED
		  │  Row 2: GPS 4   | CNO: 38 dBHz | ████████░░ | USED
		  │  Row 3: GPS 8   | CNO: 15 dBHz | ███░░░░░░░ | TRACKED
		  │  ...

T=90 ms:  communicationLoop() sleeps to maintain 100 Hz
		  └─ sleep for 10 - 25 = negative → no sleep, next iteration can start
```

---

## Troubleshooting: "No Satellite Data" Problem

If `state.sv_list` is empty or `sv_info_valid` is False, check:

### 1. Is polling happening?
**DroneLink.cpp:235-239** — The satellite poll only fires every 100 iterations (≈1 second).

```cpp
if (svPollTickCounter_ <= 0) {
	pollSatellitesMSP(pending);
	svPollTickCounter_ = SV_POLL_TICKS;  // Reset to 100
}
--svPollTickCounter_;
```

**Check:** Wait 2-3 seconds after connect before checking state.

### 2. Is the response being parsed?
**DroneLink.cpp:274-285** — The wrapper method updates pending state:

```cpp
bool DroneLink::pollSatellitesMSP(DroneState& pending) {
	auto buf = sendMSP(MSP::GPS_SV_INFO);
	std::vector<SVInfoEntry> sv;
	if (!GPSNeoM10::parseMspSvInfo(buf, sv))
		return false;

	pending.svList = std::move(sv);
	pending.svInfoValid = true;
	pending.svSource = "MSP";
	return true;
}
```

**Check:** Add debug logging to confirm `buf` has data and parse returns true.

### 3. Is the state being committed?
**DroneLink.cpp:633-636** — The atomic state update:

```cpp
void DroneLink::commitState(const DroneState& s) {
	std::lock_guard<std::mutex> lock(dataMutex);
	currentState = s;  // ← pending.svList is copied here
}
```

**Check:** Call `drone.get_latest_state()` and inspect `sv_list` directly.

### 4. Is the Python binding correct?
**Bindings.cpp:214** — The Python property definition:

```cpp
.def_readonly("sv_list", &DroneState::svList,
	"List of SVInfoEntry...")
```

**Check:** Inspect `state.sv_list` in Python; should be a pybind11 list-like object.

### 5. Is the parser being called at all?
**GPSneoM10.cpp:174-273** — Add logging to the parser:

```cpp
bool GPSNeoM10::parseMspSvInfo(...) {
	std::cerr << "parseMspSvInfo() called with buf size=" << buf.size() << std::endl;
	if (buf.size() < 8) {
		std::cerr << "  Frame too short!" << std::endl;
		return false;
	}
	// ...
}
```

**Check:** Rebuild and run; should see output every ~1 second.

---

## Summary

DroneLink's communication architecture is:

1. **Serial I/O (Windows API):**
   - Opens COM port at 57600 baud
   - Reads/writes binary frames

2. **MSP Protocol:**
   - Constructs 6-byte request frames
   - Receives variable-length response frames
   - Validates headers and checksums

3. **Parsing:**
   - Dedicated parser function per MSP command
   - Extracts binary fields (i16, u16, i32, etc.)
   - Applies unit conversions (e.g., ÷100, ÷1e7)
   - Computes derived fields (e.g., positionUsable)

4. **Satellite Pipeline:**
   - Polls MSP cmd 164 every ~1 second
   - Parses 32-channel response
   - Decodes GNSS ID from nibbles
   - Creates SVInfoEntry vector

5. **Thread Safety:**
   - Worker thread updates local `pending` state
   - `commitState()` atomically swaps pending → shared `currentState`
   - `getLatestState()` returns thread-safe copy to Python

6. **Python Exposure:**
   - pybind11 bindings expose DroneState and SVInfoEntry
   - GUI accesses `state.sv_list` directly
   - No manual serialization needed

---

This document covers the complete data flow from serial bytes to Python GUI.
