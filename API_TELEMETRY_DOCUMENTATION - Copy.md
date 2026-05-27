# R.A.D — API & Telemetry Reference Documentation

## Table of Contents
1. [DroneLink C++ API](#dronelink-c-api)
2. [Data Structures & Telemetry Fields](#data-structures--telemetry-fields)
3. [Sensor Classes API](#sensor-classes-api)
4. [Python Telemetry Integration](#python-telemetry-integration)
5. [Battery State & Arming Diagnostics](#battery-state--arming-diagnostics)
6. [Flight Mode & Sensor Status Flags](#flight-mode--sensor-status-flags)
7. [Common Parsing Patterns](#common-parsing-patterns)
8. [Error Handling & Recovery](#error-handling--recovery)

---

## DroneLink C++ API

### Connection Management

#### `bool connect(const std::string& portName)`

Establishes serial connection to flight controller.

**Parameters:**
- `portName` (string) — COM port name (e.g., "COM3", "COM4")

**Returns:**
- `true` on successful connection start
- `false` if serial port open fails

**Behavior:**
- Opens Windows serial port at 57600 baud
- Spawns `communicationLoop()` worker thread
- Worker polls all MSP commands immediately

**Example:**
```cpp
DroneLink drone;
if (drone.connect("COM3")) {
	std::cout << "Connected!" << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for first polls
} else {
	std::cerr << "Connection failed" << std::endl;
}
```

#### `void disconnect()`

Cleanly shuts down serial connection.

**Behavior:**
- Stops `communicationLoop()` worker thread
- Closes serial port handle
- Safe to call multiple times

**Example:**
```cpp
// ... use drone ...
drone.disconnect();
```

#### `bool isConnected() const`

Quick check of connection status.

**Returns:**
- `true` if connected and worker thread alive
- `false` if disconnected

**Note:** Non-blocking; does not detect stale connections. Use `linkHealthy` in `DroneState` for health verification.

---

### State Retrieval

#### `DroneState getLatestState()`

Returns thread-safe snapshot of all telemetry.

**Returns:** `DroneState` struct (see [Data Structures](#data-structures--telemetry-fields))

**Thread Safety:** Atomic copy under mutex; safe from any thread

**Latency:** Typically <1 ms from worker write to snapshot copy

**Example:**
```cpp
DroneState state = drone.getLatestState();
std::cout << "Roll: " << state.roll << " degrees" << std::endl;
std::cout << "Battery: " << state.batteryVoltage << " V" << std::endl;
```

---

### Calibration Control

#### `void startMagCalibration()`

Initiates magnetometer calibration sequence.

**Behavior:**
- Sets `magCalRequested` atomic flag
- Worker thread sends `MSP_MAG_CAL` (205) and tracks 30 s countdown
- Drone must be **armed** during calibration

**Example:**
```cpp
drone.startMagCalibration();
// Monitor DroneState.magCalSecondsRemaining in polling loop
// When magCalSecondsRemaining reaches 0, calibration complete
```

#### `void startAccCalibration()`

Initiates accelerometer calibration sequence.

**Behavior:**
- Sets `accCalRequested` atomic flag
- Worker thread sends `MSP_ACC_CAL` (206) and tracks 5 s countdown
- Drone must be **level on stable surface** during calibration

**Example:**
```cpp
drone.startAccCalibration();
// Monitor DroneState.accCalSecondsRemaining
```

---

### Configuration

#### `void setPollIntervalMs(int ms)`

Adjusts worker loop cycle time.

**Parameters:**
- `ms` — interval in milliseconds (default: 10 ms for 100 Hz)

**Effect:**
- Reduces CPU load if set to 20 ms (50 Hz) or higher
- Increases latency proportionally
- Useful for low-power hosts or overload scenarios

**Example:**
```cpp
drone.setPollIntervalMs(20);  // Drop to 50 Hz for headroom
```

---

## Data Structures & Telemetry Fields

### DroneState Structure

**Purpose:** Complete snapshot of all drone telemetry at one instant

**Thread Safety:** Written exclusively by `communicationLoop()`, read via `getLatestState()` snapshot copy

### IMU & Attitude Fields

```cpp
// Raw accelerometer counts (±4g range, 8192 counts/g)
int16_t ax = 0, ay = 0, az = 0;

// Raw gyroscope counts (±2000°/s range, 16.4 counts/°/s)
int16_t gx = 0, gy = 0, gz = 0;

// Fused attitude (Kalman filter output from FC)
int16_t roll = 0;    // ±180 degrees (roll = bank angle)
int16_t pitch = 0;   // ±90 degrees (pitch = nose-up/down)
int16_t yaw = 0;     // 0–359 degrees (yaw = heading)
```

**Interpretation:**
- **Roll:** Aircraft wing tilt; positive = right wing down
- **Pitch:** Aircraft nose angle; positive = climbing
- **Yaw:** Compass heading; 0° = North, 90° = East, etc.

**Units:**
- Roll/Pitch: degrees × 10 for precision (e.g., 450 = 45.0°)
- Yaw: full degrees

**Example (C++):**
```cpp
DroneState state = drone.getLatestState();
float roll_deg = state.roll / 10.0f;
float pitch_deg = state.pitch / 10.0f;
std::cout << "Attitude: roll=" << roll_deg << " pitch=" << pitch_deg << std::endl;
```

### Battery & Power Fields

```cpp
// From MSP_ANALOG (110) — updated every 10 ms
float    batteryVoltage = 0.0f;      // Volts (0.0–6.0 typical LiPo)
float    batteryCurrent = 0.0f;      // Amps
uint16_t batteryMahDrawn = 0;        // Milliamp-hours consumed since boot

// From MSP_BATTERY_STATE (242) — updated every ~500 ms
uint8_t      batteryCellCount = 0;   // Auto-detected (2–6 cells typical)
uint16_t     batteryCapacityMah = 0; // Rated capacity (mAh)
uint8_t      batteryPercentage = 0;  // Computed: 100 × (mAh remaining) / capacity
BatteryState batteryState;           // OK / WARNING / CRITICAL / NOT_PRESENT
```

**BatteryState Enum:**
```cpp
enum class BatteryState : uint8_t {
	OK = 0,           // Voltage healthy
	WARNING = 1,      // Low voltage warning (e.g., 3.0 V/cell)
	CRITICAL = 2,     // Critical voltage (e.g., 2.8 V/cell) — land immediately
	NOT_PRESENT = 3,  // Battery not detected
	INIT = 4          // Initializing
};
```

**UI Mapping:**
| State | Color | Action |
|-------|-------|--------|
| OK | Green | Normal flight |
| WARNING | Yellow | Land soon |
| CRITICAL | Red | LAND NOW |
| NOT_PRESENT | Gray | No battery detected (emergency) |

**Example (Python):**
```python
state = drone.getLatestState()
voltage = state.batteryVoltage
capacity = state.batteryCapacityMah
percent = state.batteryPercentage
print(f"Battery: {voltage:.2f}V ({percent}%) of {capacity} mAh")
```

### Barometer Fields

```cpp
int32_t baroAltitudeCm = 0;      // Altitude above launch point (cm)
int16_t baroVarioCmPerSec = 0;   // Vertical speed (cm/s, positive = climbing)
bool    baroValid = false;        // BMP280 sensor healthy?
```

**Usage:**
- **baroAltitudeCm / 100 = meters** above reference
- **baroVarioCmPerSec / 100 = m/s** climb rate
- Check `baroValid` before displaying; invalid if BMP280 failed

**Example:**
```cpp
float alt_m = state.baroAltitudeCm / 100.0f;
float climb_mps = state.baroVarioCmPerSec / 100.0f;
std::cout << "Alt: " << alt_m << " m, Climb: " << climb_mps << " m/s" << std::endl;
```

### Magnetometer Fields

```cpp
int16_t magX = 0, magY = 0, magZ = 0;  // Raw magnetometer counts (Tesla × 10⁵)
float   magHeadingDeg = 0.0f;          // Computed 2D heading (0–360°)
bool    magValid = false;              // QMC5883L sensor healthy?
```

**Prerequisites:**
- Requires `debug_mode=MAG_CALIB` in Betaflight CLI
- Sensor must be calibrated (soft iron / hard iron offsets)

**Interpretation:**
- **magHeadingDeg:** Compass heading, tilt-uncorrected
  - 0° = Magnetic North
  - 90° = Magnetic East
  - Can jitter ±5–10° without full tilt compensation

**Example:**
```cpp
if (state.magValid) {
	std::cout << "Compass heading: " << state.magHeadingDeg << "°" << std::endl;
} else {
	std::cout << "Magnetometer not ready (calibrate first)" << std::endl;
}
```

### GPS Fields

```cpp
GPSReading gps;  // Struct containing:
struct GPSReading {
	// Position
	int32_t latitude = 0;      // Degrees × 10⁷ (e.g., 40.7128 → 407128000)
	int32_t longitude = 0;     // Degrees × 10⁷
	int32_t altitudeCm = 0;    // Altitude in centimeters

	// Motion
	uint16_t speed = 0;        // Speed (cm/s)
	uint16_t course = 0;       // Course over ground (0–359°)

	// Fix Quality
	uint8_t fixType = 0;       // 0=none, 1=GPS, 2=DGPS, 3=PPS, 4=RTK Fixed, 5=RTK Float, 6=compass
	uint8_t numSat = 0;        // Satellite count in solution
	uint16_t hdop = 0;         // Horizontal dilution of precision (0.5–99.9)

	// Status
	bool gpsHeartbeat = false; // Pulses every GPS update (for diagnostics)
	bool valid = false;        // Fix obtained?
};

std::vector<SVInfoEntry> svList;  // Satellite list (from MSP_GPS_SV_INFO)
bool                     svInfoValid = false;
std::string              svSource;  // "MSP" or "UBX" (passthrough)

NavStatus navStatus;  // GPS nav engine state (from MSP_NAV_STATUS)
```

**GPSReading Parsing Examples:**

```cpp
// Convert latitude to decimal degrees
float lat = state.gps.latitude / 1.0e7f;

// Convert speed from cm/s to km/h
float speed_kmh = state.gps.speed * 0.036f;

// Check fix quality
if (state.gps.fixType >= 1) {
	std::cout << "GPS fix, " << (int)state.gps.numSat << " satellites" << std::endl;
} else {
	std::cout << "No GPS fix" << std::endl;
}

// HDOP interpretation
if (state.gps.hdop < 200) {
	std::cout << "Excellent GPS geometry (HDOP < 2.0)" << std::endl;
}
```

**SVInfoEntry Structure:**
```cpp
struct SVInfoEntry {
	uint8_t sv;        // Satellite ID (1–32 GPS, 65+ GLONASS, etc.)
	uint8_t quality;   // Signal quality (0=none, 1=searching, 2=signal, 3=locked)
	uint8_t cno;       // Signal strength (dBHz, 0–50)
	int8_t  elev;      // Elevation (degrees, -90–90); 0 in MSP mode
	int16_t azim;      // Azimuth (degrees, 0–359); 0 in MSP mode
};
```

### FC Status Fields

```cpp
// Core status (from MSP_STATUS, 101)
bool        armed = false;           // Drone armed?
uint32_t    flightModeFlags = 0;     // Bitmask — use FlightMode:: constants
std::string flightModeName;          // Human-readable: "ACRO [DISARMED]"
uint16_t    sensorStatus = 0;        // Bitmask — use SensorStatus:: constants
uint16_t    i2cErrorCount = 0;       // Cumulative I2C bus errors
uint16_t    cpuLoadPercent = 0;      // FC CPU usage 0–100%
uint8_t     pidProfile = 0;          // Active PID profile (0–5)

// Extended status (from MSP_STATUS_EX, 150)
uint32_t    armingDisableFlags = 0;  // Bitmask — why can't we arm?
std::string armingDisableStr;        // Comma-separated reasons

// Diagnostics
double   lastRttMs = 0.0;            // Last MSP round-trip time (ms)
double   fcCycleMs = 0.0;            // FC's own loop cycle (ms, from STATUS)
bool     linkHealthy = false;        // >5 consecutive MSP failures?
uint32_t packetCount = 0;            // Total MSP responses received
```

**Flight Mode Flags:**
```cpp
namespace FlightMode {
	constexpr uint32_t ARM = (1u << 0);           // Armed
	constexpr uint32_t ANGLE = (1u << 1);         // Angle mode (self-leveling)
	constexpr uint32_t HORIZON = (1u << 2);       // Horizon mode
	constexpr uint32_t MAG = (1u << 3);           // Mag-hold enabled
	constexpr uint32_t HEADFREE = (1u << 4);      // Head-free mode
	constexpr uint32_t FAILSAFE = (1u << 8);      // Failsafe triggered
	constexpr uint32_t GPS_RESCUE = (1u << 16);   // GPS Rescue mode
	constexpr uint32_t ANTI_GRAV = (1u << 24);    // Anti-gravity throttle
}

// Check if armed
if (state.flightModeFlags & FlightMode::ARM) {
	std::cout << "ARMED" << std::endl;
}

// Check angle mode
if (state.flightModeFlags & FlightMode::ANGLE) {
	std::cout << "Angle mode (self-leveling)" << std::endl;
}
```

**Sensor Status Flags:**
```cpp
namespace SensorStatus {
	constexpr uint16_t ACC = (1u << 0);        // Accelerometer OK
	constexpr uint16_t BARO = (1u << 1);       // Barometer OK
	constexpr uint16_t MAG = (1u << 2);        // Magnetometer OK
	constexpr uint16_t GPS = (1u << 3);        // GPS OK (has fix)
	constexpr uint16_t RANGEFINDER = (1u << 4);  // Rangefinder OK
	constexpr uint16_t GYRO = (1u << 5);       // Gyroscope OK
}

// Check which sensors are healthy
if (!(state.sensorStatus & SensorStatus::GYRO)) {
	std::cout << "ERROR: Gyro not responding!" << std::endl;
}
```

**Arming Disable Flags (Why can't we arm?):**
```cpp
namespace ArmingDisable {
	constexpr uint32_t NO_GYRO = (1u << 0);           // Gyro missing
	constexpr uint32_t FAILSAFE = (1u << 1);          // Failsafe active
	constexpr uint32_t RX_FAILSAFE = (1u << 2);       // Receiver failsafe
	constexpr uint32_t THROTTLE = (1u << 7);          // Throttle not at min
	constexpr uint32_t ANGLE = (1u << 8);             // Not level enough
	constexpr uint32_t BOOT_GRACE_TIME = (1u << 9);   // Still in boot grace
	constexpr uint32_t CALIBRATING = (1u << 12);      // Sensors calibrating
	constexpr uint32_t ACC_CALIBRATION = (1u << 24);  // Accel calibration running
	// ... many more ...
}

// Check why we can't arm
if (state.armingDisableFlags & ArmingDisable::THROTTLE) {
	std::cout << "Cannot arm: throttle not at minimum" << std::endl;
}
if (state.armingDisableFlags & ArmingDisable::ANGLE) {
	std::cout << "Cannot arm: drone not level" << std::endl;
}
```

### Motor & RC Fields

```cpp
// Motor outputs (from MSP_MOTOR, 104) — throttle sent to ESCs
uint16_t motorValues[MAX_MOTORS] = {};  // µs: 1000 = min, 2000 = max
uint8_t  motorCount = 0;                // Number of motors active

// RC channel inputs (from MSP_RC, 105) — signals from transmitter
uint16_t rcChannels[MAX_RC_CH] = {};    // µs: 1000–2000
uint8_t  rcChannelCount = 0;            // Number of channels

// Typical channel layout (MODE 2):
// [0] Aileron (Roll)
// [1] Elevator (Pitch)
// [2] Throttle
// [3] Rudder (Yaw)
// [4] ARM switch
// [5] Flight mode switch
// [6–17] AUX3 through AUX10
```

**Example:**
```cpp
// Check if ARM switch is high
if (state.rcChannels[4] > 1500) {
	std::cout << "ARM switch UP" << std::endl;
}

// Get throttle value (0–1)
float throttle = (state.rcChannels[2] - 1000.0f) / 1000.0f;
std::cout << "Throttle: " << (throttle * 100.0f) << "%" << std::endl;
```

### Calibration Status Fields

```cpp
bool magCalActive = false;            // Magnetometer calibration in progress?
int  magCalSecondsRemaining = 0;      // Countdown timer (0–30)
bool accCalActive = false;            // Accelerometer calibration in progress?
int  accCalSecondsRemaining = 0;      // Countdown timer (0–5)
```

**Usage (Python UI):**
```python
state = hub.getLatestState()
if state.magCalActive:
	progress = 30 - state.magCalSecondsRemaining
	print(f"Mag calibration: {progress}/30 seconds")
	draw_progress_bar(progress, 30)
```

---

## Sensor Classes API

### IMUSensor Class

**File:** `DroneBackend/IMUSensor.h` and `.cpp`

```cpp
class IMUSensor {
public:
	struct IMUData {
		int16_t accX, accY, accZ;  // Raw counts from MSP_RAW_IMU
		int16_t gyroX, gyroY, gyroZ;
	};

	struct IMUScaled {
		float accX, accY, accZ;    // Acceleration in g
		float gyroX, gyroY, gyroZ; // Angular velocity in °/s
	};

	IMUSensor(DroneLink* hub);

	// Non-blocking — returns latest snapshot
	IMUData getRawData();
	IMUScaled getScaledData();
};
```

#### `IMUData getRawData()`

Returns raw ADC counts from MPU-6500.

**Returns:** IMUData struct with int16_t fields

**Scale Factors (do not apply):**
- Accelerometer: 8192 counts per 1g
- Gyroscope: 16.4 counts per 1°/s

**Example:**
```cpp
IMUSensor imu(&drone);
auto raw = imu.getRawData();
std::cout << "Raw accel X: " << raw.accX << " counts" << std::endl;
```

#### `IMUScaled getScaledData()`

Returns physical units (g and °/s).

**Returns:** IMUScaled struct with float fields

**Scale Factors (pre-applied):**
- Accelerometer: divide by 8192 → g
- Gyroscope: divide by 16.4 → °/s

**Example:**
```cpp
auto scaled = imu.getScaledData();
std::cout << "Accel: " << scaled.accX << " g, " 
		  << scaled.accY << " g, "
		  << scaled.accZ << " g" << std::endl;
std::cout << "Gyro: " << scaled.gyroX << " °/s" << std::endl;
```

**Note:** Scale constants are hardcoded for MPU-6500 with ±4g / ±2000°/s ranges. If your FC uses different ranges, edit `IMUSensor.h`:
```cpp
static constexpr float ACC_SCALE = 1.0f / 8192.0f;  // Edit if ±8g or ±2g
static constexpr float GYRO_SCALE = 1.0f / 16.4f;   // Edit if ±250°/s or ±4000°/s
```

### GPSNeoM10 Class

**File:** `DroneBackend/GPSNeoM10.h`

Provides static helper functions for GPS decoding (used internally by DroneLink parser).

```cpp
class GPSNeoM10 {
public:
	// Parse GPSReading and NavStatus from raw buffers
	static void parseRaw(const std::vector<uint8_t>& buf, GPSReading& out);
	static void parseComp(const std::vector<uint8_t>& buf, GPSReading& out);
	static void parseNavStatus(const std::vector<uint8_t>& buf, NavStatus& out);

	// Parse satellite list (MSP_GPS_SV_INFO)
	static void parseSVInfo(const std::vector<uint8_t>& buf,
						   std::vector<SVInfoEntry>& out);
};
```

**Normal Usage:** Called internally by `DroneLink::parseGPSRaw()`, etc. Not typically called directly by application code.

### BaroBMP280 Class

**File:** `DroneBackend/BaroBMP280.h`

Provides static helper for barometer decoding (used internally by DroneLink).

```cpp
class BaroBMP280 {
public:
	static void parseAltitude(const std::vector<uint8_t>& buf,
							 int32_t& altitudeCm,
							 int16_t& varioCmPerSec);
};
```

**Usage:** Called by `DroneLink::parseBaro()`. Not typically called directly.

### MagQMC5883L Class

**File:** `DroneBackend/MagQMC5883L.h`

Provides static helper for magnetometer decoding and heading calculation.

```cpp
class MagQMC5883L {
public:
	// Parse raw X/Y/Z counts from MSP_DEBUG
	static void parseDebug(const std::vector<uint8_t>& buf,
						  int16_t& magX, int16_t& magY, int16_t& magZ);

	// Compute 2D heading from X/Y component (tilt-uncorrected)
	static float computeHeading(int16_t magX, int16_t magY);
};
```

**Usage Example:**
```cpp
int16_t x, y, z;
MagQMC5883L::parseDebug(buf, x, y, z);
float heading = MagQMC5883L::computeHeading(x, y);
```

---

## Python Telemetry Integration

### TelemetryWorker Class

**File:** `DroneCockpitUI/telemetry_worker.py`

**Purpose:** Bridge between C++ backend and Python UI; runs in daemon thread

```python
class TelemetryWorker:
	def __init__(self, hub):
		"""
		hub: DroneLink instance (from C++ bindings)
		"""
		self._hub = hub
		self._queue = queue.Queue(maxsize=3)
		self._running = threading.Event()
		self._connected = threading.Event()
		self._thread = None
		...
```

#### `start()`

Spawns background polling thread.

**Example:**
```python
worker = TelemetryWorker(drone_backend)
worker.start()
```

#### `stop()`

Signals worker to stop, waits for thread exit.

**Example:**
```python
worker.stop()  # Will block up to 2 seconds waiting for join
```

#### `get_frame() -> Optional[dict]`

Non-blocking frame retrieval.

**Returns:**
- Latest `ui_data` dict (Python dict, not DroneState struct)
- `None` if queue empty

**Behavior:**
- Drains stale frames automatically
- Always returns freshest data available
- Non-blocking even if queue empty

**Example:**
```python
frame = worker.get_frame()
if frame:
	print(f"Battery: {frame['battery_voltage']:.2f} V")
	print(f"Armed: {frame['armed']}")
else:
	print("No telemetry yet")
```

#### `is_connected` (property)

Readable boolean indicating link status.

**Example:**
```python
if worker.is_connected:
	print("Drone link active")
else:
	print("Drone not connected")
```

### UI Data Dictionary Format

The `ui_data` dict returned by `get_frame()` contains:

```python
ui_data = {
	# IMU & Attitude
	"roll": float,           # degrees
	"pitch": float,          # degrees
	"yaw": float,            # degrees (0–359)
	"ax": float,             # acceleration g
	"ay": float,
	"az": float,
	"gx": float,             # gyro °/s
	"gy": float,
	"gz": float,

	# Battery & Power
	"battery_voltage": float,      # Volts
	"battery_current": float,      # Amps
	"battery_mah_drawn": int,      # mAh
	"battery_capacity": int,       # mAh
	"battery_percentage": int,     # 0–100%
	"battery_state": str,          # "OK", "WARNING", "CRITICAL"
	"battery_cell_count": int,     # detected cells

	# Altitude & Climb
	"altitude_m": float,           # meters
	"vario_mps": float,            # m/s (positive = climbing)
	"baro_valid": bool,

	# Magnetometer
	"mag_heading": float,          # degrees 0–360
	"mag_valid": bool,
	"mag_x": int, "mag_y": int, "mag_z": int,  # raw counts

	# GPS
	"gps_lat": float,              # decimal degrees
	"gps_lon": float,
	"gps_alt_m": float,
	"gps_speed_kmh": float,
	"gps_course": float,           # degrees 0–359
	"gps_fix": str,                # "No Fix", "GPS", "DGPS", "RTK Fixed", etc.
	"gps_sat": int,                # satellite count
	"gps_hdop": float,             # DOP value (lower = better)
	"gps_valid": bool,

	# FC Status
	"armed": bool,
	"flight_mode_name": str,       # "ACRO", "ANGLE [DISARMED]", etc.
	"fc_cpu_load": int,            # 0–100%
	"link_healthy": bool,
	"last_rtt_ms": float,          # latency indicator
	"fc_cycle_ms": float,          # FC loop cycle time
	"sensor_status_flags": int,    # bitmask
	"arming_disable_str": str,     # "Throttle not at minimum" etc.

	# RC Channels (subset shown)
	"rc_throttle": float,          # 0.0–1.0
	"rc_roll": float,              # -1.0–1.0
	"rc_pitch": float,
	"rc_yaw": float,
	"rc_arm_switch": int,          # 1000–2000 µs

	# Motors
	"motor_values": list,          # [1000–2000, ...] µs for each motor
	"motor_count": int,

	# Calibration
	"mag_cal_active": bool,
	"mag_cal_remaining_s": int,
	"acc_cal_active": bool,
	"acc_cal_remaining_s": int,
}
```

### Integration with Tkinter Widgets

**Example: Updating IMUWidget from frame**

```python
def update_display(self, frame):
	if not frame:
		return

	# Extract relevant fields
	roll = frame["roll"]
	pitch = frame["pitch"]
	yaw = frame["yaw"]

	# Update widget displays
	self.roll_label.config(text=f"Roll: {roll:.1f}°")
	self.pitch_label.config(text=f"Pitch: {pitch:.1f}°")
	self.yaw_label.config(text=f"Yaw: {yaw:.1f}°")

	# Update graphs, 3D views, etc.
	self.plot_attitude(roll, pitch, yaw)
```

---

## Battery State & Arming Diagnostics

### Battery State Machine

**Threshold-Based State Transitions:**

```
State         Voltage Range (per cell)  UI Color  Action
─────────────────────────────────────────────────────────
INIT          Starting state            Gray      Initializing
OK            > 3.2 V                   Green     Normal flight
WARNING       3.0–3.2 V                 Yellow    Land soon (next 1–3 minutes)
CRITICAL      < 3.0 V                   Red       Land immediately (seconds remaining)
NOT_PRESENT   No battery detected       Gray      Critical error
```

**Example (C++):**
```cpp
DroneState state = drone.getLatestState();

std::string status;
switch (state.batteryState) {
	case BatteryState::OK:
		status = "✓ Battery OK";
		break;
	case BatteryState::WARNING:
		status = "⚠ Low Battery — Land Soon";
		break;
	case BatteryState::CRITICAL:
		status = "🔴 CRITICAL — Land NOW!";
		break;
	case BatteryState::NOT_PRESENT:
		status = "❌ No Battery Detected";
		break;
	default:
		status = "? Unknown state";
}

std::cout << status << " (" << state.batteryVoltage << "V)" << std::endl;
```

### Battery Percentage Calculation

Betaflight computes percentage as:

```
percentage = (remaining_mah / capacity_mah) × 100
```

Where `remaining_mah = capacity_mah - drawn_mah`.

**Interpretation:**
- **100%:** Fresh battery
- **50%:** Halfway through flight
- **10%:** Only a few minutes left (WARNING threshold)
- **0%:** Battery depleted (CRITICAL threshold)

**Note:** If `batteryCapacityMah` is not set in BF (Configurator → Battery → Capacity), the percentage will always read 0. Set it first.

### Arming Disable Reasons

When drone **cannot arm**, `armingDisableStr` contains human-readable reasons:

**Common Reasons:**

| Flag | Reason | Fix |
|------|--------|-----|
| `THROTTLE` | Throttle not at minimum | Lower stick to idle |
| `ANGLE` | Not level | Place drone on flat surface |
| `NO_GYRO` | Gyro sensor not responding | Check gyro, reboot FC |
| `FAILSAFE` | Failsafe active (RX loss) | Restore RX signal |
| `CALIBRATING` | Sensors calibrating | Wait (usually <30s on boot) |
| `BOOT_GRACE_TIME` | Still in boot grace | Wait ~5s after power-on |
| `ACC_CALIBRATION` | Accel calibration running | Wait for `accCalSecondsRemaining` to reach 0 |

**Full arming disable flag reference:**

```cpp
namespace ArmingDisable {
	NO_GYRO             = (1u << 0);   // Gyro missing / not ready
	FAILSAFE            = (1u << 1);   // Failsafe active
	RX_FAILSAFE         = (1u << 2);   // Receiver failsafe
	BAD_RX_RECOVERY     = (1u << 3);   // Bad RX recovery
	BOXFAILSAFE         = (1u << 4);   // Failsafe mode box active
	RUNAWAY_TAKEOFF     = (1u << 5);   // Runaway takeoff detected
	CRASH_DETECTED      = (1u << 6);   // Crash detected
	THROTTLE            = (1u << 7);   // Throttle not at minimum
	ANGLE               = (1u << 8);   // Not level (acro mode)
	BOOT_GRACE_TIME     = (1u << 9);   // Boot grace period active
	NOPREARM            = (1u << 10);  // Pre-arm checks failed
	LOAD                = (1u << 11);  // CPU overload
	CALIBRATING         = (1u << 12);  // Sensor calibration active
	CLI                 = (1u << 13);  // CLI mode active
	CMS_MENU            = (1u << 14);  // CMS menu open
	BST                 = (1u << 15);  // BST mode active
	MSP                 = (1u << 16);  // MSP mode active
	PARALYZE            = (1u << 17);  // Paralyze mode active
	GPS                 = (1u << 18);  // GPS not ready (GPS-rescue mode)
	RESC_SW             = (1u << 21);  // Rescue switch active
	DSHOT_BITBANG       = (1u << 23);  // DShot bitbang active
	ACC_CALIBRATION     = (1u << 24);  // Accel calibration active
	MOTOR_PROTOCOL      = (1u << 25);  // Motor protocol not set
	ARM_SWITCH          = (1u << 26);  // ARM switch off
}
```

---

## Flight Mode & Sensor Status Flags

### Flight Mode Interpretation

**Typical Mode Combinations:**

```
flightModeName          Flags                      Meaning
───────────────────────────────────────────────────────────────
"ACRO"                  ARM only                   Manual stick control, no self-level
"ACRO [DISARMED]"       Nothing                    ACRO mode armed status Off
"ANGLE"                 ARM | ANGLE                Self-leveling enabled
"HORIZON"               ARM | HORIZON              Hybrid (ANGLE + ACRO rates)
"ANGLE [MAG-HOLD]"      ARM | ANGLE | MAG         ANGLE + compass heading hold
"FAILSAFE"              FAILSAFE                   RX loss, returning home
"GPS-RESCUE"            ARM | GPS_RESCUE           GPS emergency return-to-home
```

**Code to parse:**

```cpp
std::string modeName = "ACRO";  // Default

if (state.flightModeFlags & FlightMode::ARM) {
	if (state.flightModeFlags & FlightMode::HORIZON) {
		modeName = "HORIZON";
	} else if (state.flightModeFlags & FlightMode::ANGLE) {
		modeName = "ANGLE";
	}
	// Add modifiers
	if (state.flightModeFlags & FlightMode::MAG) {
		modeName += " [MAG-HOLD]";
	}
	if (state.flightModeFlags & FlightMode::GPS_RESCUE) {
		modeName = "GPS-RESCUE";
	}
	if (state.flightModeFlags & FlightMode::FAILSAFE) {
		modeName = "FAILSAFE";
	}
} else {
	modeName += " [DISARMED]";
}
```

### Sensor Health Status

**Flag Interpretation:**

```cpp
if (state.sensorStatus & SensorStatus::GYRO) {
	gyroOK = true;  // Gyroscope responding
}
if (state.sensorStatus & SensorStatus::ACC) {
	accOK = true;   // Accelerometer responding
}
if (state.sensorStatus & SensorStatus::BARO) {
	baroOK = true;  // Barometer responding
}
if (state.sensorStatus & SensorStatus::MAG) {
	magOK = true;   // Magnetometer responding
}
if (state.sensorStatus & SensorStatus::GPS) {
	gpsOK = true;   // GPS has fix
}
```

**UI Display Example:**

```python
sensors = [
	("Gyro", frame["sensor_status_flags"] & (1 << 5)),
	("Accel", frame["sensor_status_flags"] & (1 << 0)),
	("Baro", frame["sensor_status_flags"] & (1 << 1)),
	("Mag", frame["sensor_status_flags"] & (1 << 2)),
	("GPS", frame["sensor_status_flags"] & (1 << 3)),
]

for name, healthy in sensors:
	color = "green" if healthy else "red"
	print(f"[{color.upper()}] {name}")
```

---

## Common Parsing Patterns

### Reading a Sensor Value with Validation

```cpp
DroneState state = drone.getLatestState();

// Pattern: Check valid flag before using
if (state.gps.valid && state.gps.fixType >= 1) {
	float lat = state.gps.latitude / 1.0e7f;
	float lon = state.gps.longitude / 1.0e7f;
	std::cout << "Position: " << lat << ", " << lon << std::endl;
} else {
	std::cout << "No GPS fix" << std::endl;
}
```

### Checking Multiple Conditions

```cpp
// Can we arm?
if (!state.armed && 
	state.sensorStatus & SensorStatus::GYRO &&
	state.sensorStatus & SensorStatus::ACC &&
	state.armingDisableFlags == 0) {
	std::cout << "Ready to arm" << std::endl;
} else {
	std::cout << "Cannot arm: " << state.armingDisableStr << std::endl;
}
```

### Converting Fixed-Point to Float

```cpp
// Roll/Pitch are degrees × 10
float roll_deg = state.roll / 10.0f;
float pitch_deg = state.pitch / 10.0f;

// GPS coordinates are degrees × 1e7
float lat = state.gps.latitude / 1.0e7f;

// Altitude is centimeters
float alt_m = state.baroAltitudeCm / 100.0f;

// Speed is cm/s
float speed_kmh = state.gps.speed * 0.036f;  // cm/s to km/h
```

### Iterating Over Motor Values

```cpp
std::cout << "Motor outputs (µs):" << std::endl;
for (int i = 0; i < state.motorCount; ++i) {
	std::cout << "  Motor " << (i + 1) << ": " << state.motorValues[i] << std::endl;
}
```

### Checking RC Stick Position

```cpp
// Standard MODE 2 layout
float aileron = (state.rcChannels[0] - 1500.0f) / 500.0f;    // -1 to +1
float elevator = (state.rcChannels[1] - 1500.0f) / 500.0f;
float throttle = (state.rcChannels[2] - 1000.0f) / 1000.0f;  // 0 to 1
float rudder = (state.rcChannels[3] - 1500.0f) / 500.0f;     // -1 to +1

std::cout << "Sticks: A=" << aileron << " E=" << elevator 
		  << " T=" << throttle << " R=" << rudder << std::endl;
```

---

## Error Handling & Recovery

### Connection Loss Detection

**Automatic Health Tracking:**

```cpp
// In communicationLoop():
int consecutiveFails = 0;

if (!anySuccess) {
	consecutiveFails++;
	if (consecutiveFails >= FAIL_THRESHOLD) {
		pending.linkHealthy = false;
	}
} else {
	consecutiveFails = 0;
	pending.linkHealthy = true;
}
```

**Monitor in Application:**

```cpp
DroneState state = drone.getLatestState();
if (!state.linkHealthy) {
	std::cerr << "Link degraded — " << state.lastRttMs << " ms latency" << std::endl;
	// Possibly reconnect
}
```

### Serial Port Recovery

If the serial port becomes stale (USB cable disconnect, FC reboot):

```cpp
if (!drone.isConnected() || !state.linkHealthy) {
	drone.disconnect();
	std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for port cleanup
	if (!drone.connect("COM3")) {
		std::cerr << "Reconnection failed" << std::endl;
		// Retry or fail gracefully
	}
}
```

### Handling Empty or Truncated MSP Responses

DroneLink parsers check buffer size before reading:

```cpp
// In parseIMU():
void DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& pending) {
	if (buf.size() < 12) return;  // Not enough data, skip this update

	// Safe to read 12 bytes
	pending.ax = (int16_t)(buf[0] | (buf[1] << 8));
	// ... etc.
}
```

Applications should assume that `getLatestState()` may return slightly stale data if a particular MSP poll failed, but the overall state remains consistent.

### Sensor Not Ready

Some sensors require time to initialize:

```cpp
if (!state.magValid) {
	std::cout << "Waiting for magnetometer..." << std::endl;
	// Skip mag-dependent features
}

if (state.sensorStatus & SensorStatus::GPS == 0) {
	std::cout << "GPS not locked" << std::endl;
	// No position data available
}
```

### Watchdog Pattern (Python)

```python
def update_ui(self):
	"""Called from Tkinter timer every 50 ms."""
	frame = self.worker.get_frame()

	if frame is None:
		# No new data; UI will show stale values
		# But we don't crash — just wait for next frame
		self.root.after(50, self.update_ui)
		return

	# Check for stale data (no update for >1 second)
	time_since_update = time.time() - self.last_frame_time
	if time_since_update > 1.0:
		self.status_label.config(
			text="⚠ Telemetry stale (>1s)",
			foreground="orange"
		)
	else:
		self.status_label.config(
			text="✓ Connected",
			foreground="green"
		)

	self.render_frame(frame)
	self.root.after(50, self.update_ui)
```

---

## Summary

This reference covers:

| Topic | Key Points |
|-------|-----------|
| **C++ API** | `connect()`, `getLatestState()`, `startMagCalibration()`, etc. |
| **DroneState** | 50+ telemetry fields covering IMU, GPS, battery, FC status |
| **Sensor Classes** | `IMUSensor`, `GPSNeoM10`, `BaroBMP280`, `MagQMC5883L` helpers |
| **Python Bridge** | `TelemetryWorker` queues `ui_data` dicts for Tkinter |
| **Battery** | State machine (OK → WARNING → CRITICAL), percentage calc |
| **Arming** | Diagnostics via `armingDisableStr` and flags |
| **Flags** | Flight mode, sensor status, arming disable bitmasks |
| **Parsing** | Fixed-point conversion, validation patterns, error recovery |

Use these patterns and structures to build robust, responsive drone control software on top of R.A.D.

