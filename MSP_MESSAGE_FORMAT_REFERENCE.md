# MSP Message Format Reference & Examples

## Quick Reference: All MSP Commands Polled by DroneLink

This document is a detailed reference for every MSP message that DroneLink sends and parses.

---

## Format Template

For each command:

```
MSP_COMMAND_NAME (ID)
├── Request
│   ├── Frame format
│   └── Payload size
├── Response
│   ├── Payload layout (bytes)
│   ├── Field descriptions
│   └── Units & ranges
└── DroneLink handler
	├── Parser function
	└── DroneState fields updated
```

---

## 1. MSP_STATUS (ID 101)

**Purpose:** Polling interval, armed state, sensor health, flight mode

### Request Format
```
$ M < [0] [101] [101]
  1 2 3  4  5    6
  │ │ │  │  │    └─ Checksum (payload_len ^ cmd_id)
  │ │ │  │  └────── Command ID
  │ │ │  └────────── Payload length (zero for status query)
  │ └─ └─────────── MSP header (always "$M<" for request)
  └────────────── Preamble
```

### Response Format
```
Bytes 0-1:   cycle_time        (u16 LE)  µs per FC loop cycle
Bytes 2-3:   i2c_errors        (u16 LE)  cumulative I2C errors
Bytes 4-5:   sensor_flags      (u16 LE)  bitmask of sensor status
Byte 6-7:    flight_mode_flags (u16 LE)  armed, stabilize, etc.
Byte 8:      reserved / profile
Bytes 9-10:  load_percent      (u16 LE)  CPU load 0-100
Bytes 11-12: gyro_cycles_sec   (u16 LE)  IMU update rate
... (extended fields in BF 4.x)
```

### DroneLink Handler
```cpp
// DroneLink.cpp
bool DroneLink::parseStatus(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 18) return false;
	const uint8_t* p = buf.data() + 5;

	s.fcCycleMs = (u16le(p) / 1000.0);  // Convert µs to ms
	s.i2cErrors = u16le(p+2);
	s.sensorFlags = u16le(p+4);
	s.flightModeFlags = u16le(p+6);
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [11] [101] [E8 03] [00 00] [FF FF] [00 01] [00] [64 00] [E8 03] ...
Parsed:
  cycle_time = 0x03E8 (1000 µs = 1 ms)
  i2c_errors = 0x0000 (0 errors)
  sensor_flags = 0xFFFF (all sensors OK)
  flight_mode_flags = 0x0100 (armed!)
```

---

## 2. MSP_RAW_IMU (ID 102)

**Purpose:** Raw accelerometer and gyroscope counts from MPU-6500

### Request Format
```
$ M < [0] [102] [102]
```

### Response Format
```
Bytes 0-1:   accel_x  (i16 LE)  raw ADC counts  (range: -32768 to +32767)
Bytes 2-3:   accel_y  (i16 LE)
Bytes 4-5:   accel_z  (i16 LE)
Bytes 6-7:   gyro_x   (i16 LE)  raw ADC counts  (range: -32768 to +32767)
Bytes 8-9:   gyro_y   (i16 LE)
Bytes 10-11: gyro_z   (i16 LE)
```

### DroneLink Handler
```cpp
bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 17) return false;
	const uint8_t* p = buf.data() + 5;

	s.ax = i16le(p+0);
	s.ay = i16le(p+2);
	s.az = i16le(p+4);
	s.gx = i16le(p+6);
	s.gy = i16le(p+8);
	s.gz = i16le(p+10);
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [12] [102] [10 FF] [25 00] [FF 0F] [E8 03] [12 04] [00 00] ...
Parsed:
  accel_x = -240  (slightly backward)
  accel_y = 37    (slightly right)
  accel_z = 4095  (near 1g = 4096 counts)
  gyro_x = 1000   (rolling right)
  gyro_y = 1042   (pitching backward)
  gyro_z = 0      (no yaw rotation)
```

### Notes
- For a level drone at rest on flat ground: ax ≈ 0, ay ≈ 0, az ≈ 4096 (1g)
- Gyro at rest should be near 0 (deadband ±20-50 counts typical)

---

## 3. MSP_ATTITUDE (ID 108)

**Purpose:** Fused roll, pitch, yaw from IMU

### Request Format
```
$ M < [0] [108] [108]
```

### Response Format
```
Bytes 0-1:   roll   (i16 LE)  decidegrees (-1800 to +1800 = -180° to +180°)
Bytes 2-3:   pitch  (i16 LE)  decidegrees (-900 to +900 = -90° to +90°)
Bytes 4-5:   yaw    (i16 LE)  degrees (0-359, wrapping)
```

### DroneLink Handler
```cpp
bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 11) return false;
	const uint8_t* p = buf.data() + 5;

	s.roll  = i16le(p+0);   // Stored as decidegrees
	s.pitch = i16le(p+2);
	s.yaw   = i16le(p+4);   // Stored as degrees
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [6] [108] [2C 00] [E0 FF] [B4 00] ...
Parsed:
  roll  = 44 decidegrees = 4.4° (tilted right)
  pitch = -32 decidegrees = -3.2° (tilted backward)
  yaw   = 180 degrees (pointing opposite initial heading)
```

### Notes
- GUI divides roll and pitch by 10 for display (44 → 4.4°)
- Yaw ranges 0-359; wraps at 360°

---

## 4. MSP_ALTITUDE (ID 109)

**Purpose:** Barometric altitude and vertical speed

### Request Format
```
$ M < [0] [109] [109]
```

### Response Format
```
Bytes 0-1:   estAlt  (i16 LE)  centimetres above home
Bytes 2-3:   vario   (i16 LE)  cm/s vertical velocity (positive = up)
```

### DroneLink Handler
```cpp
bool DroneLink::parseBaro(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 10) return false;
	const uint8_t* p = buf.data() + 5;

	s.baroAltitudeCm = i16le(p+0);
	s.baroVarioCmPerSec = i16le(p+2);
	s.baroValid = true;
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [4] [109] [E8 03] [00 00] ...
Parsed:
  estAlt = 1000 cm = 10 metres
  vario = 0 cm/s (not climbing or sinking)
```

---

## 5. MSP_ANALOG (ID 110)

**Purpose:** Battery voltage, current, consumed mAh, RSSI

### Request Format
```
$ M < [0] [110] [110]
```

### Response Format
```
Bytes 0-1:   vbat        (u16 LE)  volts × 100  (e.g., 1200 = 12.00V)
Bytes 2-3:   mah_drawn   (u16 LE)  mAh consumed
Bytes 4-5:   map         (i16 LE)  MAP value (not used for brushless)
Bytes 6:     rssi        (u8)      0-255 (may be PWM or percent)
```

### DroneLink Handler
```cpp
bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 12) return false;
	const uint8_t* p = buf.data() + 5;

	uint16_t vbat_raw = u16le(p+0);
	s.batteryVoltage = vbat_raw / 100.0f;  // Convert to volts
	s.mahDrawn = u16le(p+2);
	s.rssi = p[6];  // 0-255
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [7] [110] [B0 04] [00 10] [00 00] [FF] ...
Parsed:
  vbat = 0x04B0 = 1200 → 12.00V
  mah_drawn = 0x1000 = 4096 mAh
  rssi = 255 (full signal)
```

---

## 6. MSP_RAW_GPS (ID 106)

**Purpose:** GPS position, speed, satellite count, HDOP

### Request Format
```
$ M < [0] [106] [106]
```

### Response Format
```
Byte 0:      fix_type  (u8)      0=no fix, 1=dead reckon, 2=2D, 3=3D
Byte 1:      num_sat   (u8)      satellites used in solution
Bytes 2-5:   latitude  (i32 LE)  degrees × 1e7 (e.g., 447700000 = 44.7700°)
Bytes 6-9:   longitude (i32 LE)  degrees × 1e7
Bytes 10-11: altitude  (u16 LE)  centimetres MSL
Bytes 12-13: speed     (u16 LE)  cm/s ground speed
Bytes 14-15: course    (u16 LE)  decidegrees (0-3599)
Bytes 16-17: hdop      (u16 LE)  × 100 (500 = 5.00, good fix)
```

### DroneLink Handler
```cpp
bool DroneLink::parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 24) return false;  // 6 + 18
	const uint8_t* p = buf.data() + 5;

	s.gps.fixType = p[0];
	s.gps.numSat = p[1];
	s.gps.latitude = i32le(p+2) / 1e7;
	s.gps.longitude = i32le(p+6) / 1e7;
	s.gps.altitudeM = u16le(p+10) / 100.0f;
	s.gps.groundSpeedMs = u16le(p+12);
	s.gps.groundCourse = u16le(p+14);
	s.gps.hdop = u16le(p+16);
	s.gps.rawValid = true;
	s.gps.positionUsable = (s.gps.fixType >= 2) && (s.gps.numSat >= 4) && (s.gps.hdop < 500);
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [18] [106] [03] [0A] [00 77 A5 02] [00 53 46 04] [A8 61] [60 01] [D0 07] [E8 03] ...
Parsed:
  fix_type = 3 (3D fix!)
  num_sat = 10 (10 satellites in solution)
  latitude = 0x02A57700 = 44770000 → 44.770000°
  longitude = 0x04465300 = 71758099 → 71.758099°
  altitude = 0x61A8 = 25000 cm = 250 m
  speed = 0x0160 = 352 cm/s ≈ 3.5 m/s
  course = 0x07D0 = 2000 decidegrees = 200°
  hdop = 0x03E8 = 1000 → 10.00 (mediocre fix)
```

### Notes
- `positionUsable` is true only if: fixType ≥ 2 AND numSat ≥ 4 AND hdop < 500
- HDOP < 200 is excellent, 200-500 is good, > 500 is poor

---

## 7. MSP_COMP_GPS (ID 107)

**Purpose:** Distance and bearing to home, GPS heartbeat

### Request Format
```
$ M < [0] [107] [107]
```

### Response Format
```
Bytes 0-1:   dist_home  (u16 LE)  metres to home
Bytes 2-3:   bearing    (i16 LE)  degrees -180 to +180
Byte 4:      heartbeat  (u8)      toggles 0↔1 on each new GPS frame
```

### DroneLink Handler
```cpp
bool DroneLink::parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 10) return false;
	const uint8_t* p = buf.data() + 5;

	s.gps.distToHomM = u16le(p+0);
	s.gps.bearingToHome = i16le(p+2);
	s.gps.gpsHeartbeat = p[4];
	s.gps.compValid = true;
	return true;
}
```

### Example Response
```
Raw bytes: $ M > [5] [107] [E8 03] [B4 00] [01] ...
Parsed:
  dist_home = 0x03E8 = 1000 metres
  bearing = 0x00B4 = 180° (drone is south of home)
  heartbeat = 1 (toggled since last poll)
```

---

## 8. MSP_NAV_STATUS (ID 121)

**Purpose:** GPS fix validity and DGPS status

### Request Format
```
$ M < [0] [121] [121]
```

### Response Format
```
Byte 0:   fix_type      (u8)   0=no fix, 1=2D, 2=3D
Byte 1:   gps_flags     (u8)   bit0=fixOk, bit1=dgpsUsed
Byte 2:   mode_flags    (u8)   navigation mode flags
Byte 3:   hw_status     (u8)   hardware status
... (extended fields)
```

### DroneLink Handler
```cpp
bool DroneLink::parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s) {
	if (buf.size() < 9) return false;
	const uint8_t* p = buf.data() + 5;

	s.navStatus.fixType = p[0];
	s.navStatus.gpsFlags = p[1];
	s.navStatus.fixOk = (p[1] & 0x01) != 0;
	s.navStatus.dgpsUsed = (p[1] & 0x02) != 0;
	s.navStatus.valid = true;
	return true;
}
```

---

## 9. MSP_MOTOR (ID 104)

**Purpose:** ESC throttle outputs (PWM 1000-2000 µs)

### Request Format
```
$ M < [0] [104] [104]
```

### Response Format
```
Bytes 0-1:   motor0  (u16 LE)  PWM 1000-2000 µs
Bytes 2-3:   motor1  (u16 LE)
Bytes 4-5:   motor2  (u16 LE)
Bytes 6-7:   motor3  (u16 LE)
```

### Example Response
```
Raw bytes: $ M > [8] [104] [E8 03] [E8 03] [E8 03] [E8 03] ...
Parsed:
  motor0 = 0x03E8 = 1000 µs (idle)
  motor1 = 0x03E8 = 1000 µs (idle)
  motor2 = 0x03E8 = 1000 µs (idle)
  motor3 = 0x03E8 = 1000 µs (idle)
```

---

## 10. MSP_RC (ID 105)

**Purpose:** RC channel inputs (up to 16 channels)

### Request Format
```
$ M < [0] [105] [105]
```

### Response Format
```
Variable length; typically 32 bytes (16 channels × 2 bytes)
Bytes 0-1:   channel_0  (u16 LE)  PWM 1000-2000 µs
Bytes 2-3:   channel_1  (u16 LE)
... up to 16 channels ...
```

### Example Response (4 channels)
```
Raw bytes: $ M > [8] [105] [E8 03] [E8 03] [E8 03] [E8 03] ...
Parsed:
  rc[0] = 1000 µs (throttle - idle)
  rc[1] = 1000 µs (yaw - centered)
  rc[2] = 1000 µs (pitch - centered)
  rc[3] = 1000 µs (roll - centered)
```

---

## 11. MSP_GPS_SV_INFO (ID 164) — **SATELLITE LIST**

**Purpose:** Per-satellite signal strength and quality (polled every ~1 second)

### Request Format
```
$ M < [0] [164] [164]
```

### Response Format
```
Byte 0:              numCh  (number of channels, 32 for NEO-M10)

Per channel (4 bytes):
  Byte [1 + i*4 + 0]: chn     (channel byte)
					  Upper nibble = GNSS ID (when numCh > 16)
					  Lower nibble = channel index 0-31
  Byte [1 + i*4 + 1]: svid    (satellite vehicle ID / PRN)
  Byte [1 + i*4 + 2]: quality (0-7: 0=idle, 7=locked)
  Byte [1 + i*4 + 3]: cno     (carrier-to-noise, dBHz, 0-63)
```

### DroneLink Handler
```cpp
bool GPSNeoM10::parseMspSvInfo(const std::vector<uint8_t>& buf,
								std::vector<SVInfoEntry>& svList) {
	// (See DRONELINK_COMMUNICATION_PROTOCOL.md for full implementation)
	// Summary: parses 32 satellites, extracts GNSS ID, quality, CNO
	// Returns vector of SVInfoEntry objects
}
```

### Example Response (10 satellites out of 32)
```
Raw bytes: $ M > [0x80] [164] [0x20]  // numCh=32
		   [0x01 0x01 0x05 0x2D]      // GPS PRN 1: quality 5, CNO 45
		   [0x04 0x04 0x05 0x26]      // GPS PRN 4: quality 5, CNO 38
		   [0x08 0x08 0x02 0x0F]      // GPS PRN 8: quality 2, CNO 15
		   [0x00 0x00 0x00 0x00]      // Empty channel
		   ... (28 more channels, mostly empty) ...

Parsed:
  sv[0]: gnss="GPS", svid=1, quality=5, cno=45 (LOCKED)
  sv[1]: gnss="GPS", svid=4, quality=5, cno=38 (LOCKED)
  sv[2]: gnss="GPS", svid=8, quality=2, cno=15 (ACQUIRED)
  sv[3-31]: empty or other constellations
```

### GNSS ID Mapping
```
0 = GPS      (PRN 1-32)
1 = SBAS     (PRN 120-158)
2 = Galileo  (PRN 1-36)
3 = BeiDou   (PRN 1-37)
5 = QZSS     (PRN 1-10)
6 = GLONASS  (PRN 1-24)
```

### Quality Level Meanings
```
0 = idle         (channel allocated, no signal)
1 = searching    (acquisition in progress)
2 = acquired     (signal found, no lock)
3 = unusable     (signal present but unusable)
4 = code locked  (code lock, can use for positioning)
5 = carrier lock (code + carrier locked)
6 = carrier lock
7 = fully locked (code + carrier + Doppler locked)
```

---

## 12. MSP_STATUS_EX (ID 150)

**Purpose:** Extended status including arming disable flags

### Request Format
```
$ M < [0] [150] [150]
```

### Response Format
```
Bytes 0-11: (mostly same as MSP_STATUS)
Bytes 12-13: arming_disable_flags (u16 LE)
```

### Arming Disable Flags
```
0x0001 = Gyro not calibrated
0x0002 = Accelerometer not calibrated
0x0004 = Compass not calibrated
0x0008 = RC not calibrated
0x0010 = Barometer problem
0x0020 = Compass problem
0x0040 = Accelerometer problem
0x0080 = GPS problem
0x0100 = Illegal config
0x0200 = System overloaded
... etc
```

---

## 13. MSP_BATTERY_STATE (ID 242)

**Purpose:** Full battery telemetry (voltage, current, capacity, cell count)

### Request Format
```
$ M < [0] [242] [242]
```

### Response Format
```
Bytes 0-1:   cell_count     (u16 LE)  number of cells
Bytes 2-3:   voltage_mv     (u16 LE)  millivolts
Bytes 4-5:   draw_amp_raw   (u16 LE)  raw current × 10
Bytes 6-7:   mah_drawn      (u16 LE)  consumed
Bytes 8-11:  mah_capacity   (i32 LE)  nominal capacity
```

---

## Byte Order Notes

All multi-byte values are **little-endian** (LSB first):

```cpp
// Convert u16 from raw bytes
uint16_t value = (bytes[1] << 8) | bytes[0];
// Or use helper:
uint16_t value = u16le(bytes);

// Convert i16
int16_t value = (int16_t)((bytes[1] << 8) | bytes[0]);
// Or:
int16_t value = i16le(bytes);

// Convert i32
int32_t value = (int32_t)((bytes[3]<<24) | (bytes[2]<<16) | (bytes[1]<<8) | bytes[0]);
// Or:
int32_t value = i32le(bytes);
```

---

## Checksum Calculation

All MSP frames include an XOR checksum:

```cpp
uint8_t calc_checksum(const std::vector<uint8_t>& frame) {
	// frame = [$ M < paylen cmd [payload...] checksum]
	uint8_t csum = frame[3] ^ frame[4];  // paylen ^ cmd_id
	for (size_t i = 5; i < frame.size() - 1; ++i) {
		csum ^= frame[i];  // XOR with each payload byte
	}
	return csum;
}
```

**Verification:**
```cpp
uint8_t received_csum = frame.back();
uint8_t calculated_csum = calc_checksum(frame);
if (received_csum != calculated_csum) {
	// Checksum failed!
}
```

---

## Quick Debugging: Reading Raw MSP Frames

To capture and decode a raw MSP response:

```cpp
std::vector<uint8_t> buf = sendMSP(106);  // MSP_RAW_GPS

// Print hex dump
std::cout << "Raw bytes: ";
for (uint8_t b : buf)
	std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
std::cout << std::endl;

// Print parsed fields
if (buf.size() >= 24) {
	const uint8_t* p = buf.data() + 5;
	std::cout << "fixType=" << (int)p[0] << std::endl;
	std::cout << "numSat=" << (int)p[1] << std::endl;
	std::cout << "lat=" << (i32le(p+2) / 1e7) << std::endl;
	std::cout << "lon=" << (i32le(p+6) / 1e7) << std::endl;
}
```

---

## Summary Table

| Command | ID | Request | Response | Interval | Purpose |
|---------|----|---------|-----------|-----------| ---------|
| STATUS | 101 | 6 | 12+ | Every tick | Cycle time, armed |
| RAW_IMU | 102 | 6 | 12 | Every tick | Accel + gyro |
| ATTITUDE | 108 | 6 | 6 | Every tick | Roll/pitch/yaw |
| ALTITUDE | 109 | 6 | 4 | Every tick | Baro alt |
| ANALOG | 110 | 6 | 7 | Every tick | Battery, RSSI |
| RAW_GPS | 106 | 6 | 18 | Every tick | Position, speed |
| COMP_GPS | 107 | 6 | 5 | Every tick | Home dist/bearing |
| NAV_STATUS | 121 | 6 | 7 | Every tick | Fix validity |
| MOTOR | 104 | 6 | 8 | Every tick | ESC throttle |
| RC | 105 | 6 | 32 | Every tick | RC inputs |
| STATUS_EX | 150 | 6 | 13 | Every tick | Arming flags |
| **GPS_SV_INFO** | **164** | **6** | **129** | **~1 sec** | **Satellite list** |
| BATTERY_STATE | 242 | 6 | 12 | ~500ms | Battery detail |
| DEBUG | 254 | 6 | 8 | Every tick | Debug values |

---

This reference covers all MSP messages used by DroneLink. Use it to:
- Understand what each message contains
- Debug raw frame captures
- Implement custom parsers
- Verify expected response sizes
