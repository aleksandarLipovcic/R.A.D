# R.A.D Ground Control Station — Architecture Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Component Architecture](#component-architecture)
3. [Data Flow Architecture](#data-flow-architecture)
4. [Communication Protocol](#communication-protocol)
5. [Thread Safety & State Management](#thread-safety--state-management)
6. [Sensor Systems](#sensor-systems)
7. [UI & Telemetry Pipeline](#ui--telemetry-pipeline)
8. [Calibration System](#calibration-system)

---

## System Overview

**R.A.D (Recon Area Drone)** is a ground control and monitoring application for autonomous drone operations. It interfaces with a drone via:

- **RadioMaster Pocket Transmitter** — Radio link
- **ExpressLRS (ELRS)** — Low-latency telemetry protocol
- **Betaflight Flight Controller** — FC with MSP (MultiWii Serial Protocol) interface
- **GPS/IMU/Barometer/Magnetometer Suite** — Onboard sensors

### Key Characteristics

| Aspect | Details |
|--------|---------|
| **Host Platform** | Windows (Microsoft Visual Studio Community 2026) |
| **FC Protocol** | MSP (Betaflight 4.5.x compatible) |
| **Connection** | USB serial to FC (57600 baud) |
| **Polling Rate** | 100 Hz (10 ms intervals) |
| **Telemetry Display** | Python Tkinter GUI (DroneCockpitUI) |

---

## Component Architecture

The system is organized into two primary layers:

### **Layer 1: DroneBackend (C++)**

Runs on Windows; manages all low-level drone communication via Betaflight MSP protocol.

```
DroneBackend (C++)
├── DroneLink (core comms engine)
│   ├── Serial port management (Windows APIs)
│   ├── MSP frame encoding/decoding
│   ├── 100 Hz communication loop
│   └── Thread-safe state snapshot
├── Sensor Handlers
│   ├── IMUSensor (MPU-6500)
│   ├── GPSNeoM10 (NEO-M10 GPS module)
│   ├── BaroBMP280 (BMP280 barometer)
│   └── MagQMC5883L (QMC5883L magnetometer)
└── Bindings (Python ↔ C++ interface)
```

**DroneLink** is the central hub:
- Maintains persistent serial connection to FC
- Executes 14 different MSP commands per 10 ms cycle
- Parses inbound telemetry and maintains `DroneState` snapshot
- Exposes thread-safe API for state queries

### **Layer 2: DroneCockpitUI (Python)**

Runs as separate process; visualizes drone state via Tkinter widgets.

```
DroneCockpitUI (Python)
├── TelemetryWorker (background thread)
│   ├── Polls DroneBackend (via C++ bindings)
│   ├── Frames telemetry into UI-ready dicts
│   └── Queues frames for Tkinter thread
├── DroneCockpitUI (main Tkinter window)
│   ├── Creates draggable instrument panels
│   ├── Updates displays from queue
│   └── Handles user input (calibration, arming)
├── Instrument Widgets
│   ├── IMUWidget (accelerometer/gyroscope display)
│   ├── ADI (Attitude Indicator, 3D visualization)
│   ├── MagWidget (magnetometer heading)
│   ├── BaroWidget (altitude/climb rate)
│   ├── GPSWidget (position, fix status, satellites)
│   ├── FCStatusWidget (armed state, modes, sensors)
│   └── ArmingWidget (arming flags, diagnostics)
└── Drone3DView (OpenGL 3D model visualization)
```

**TelemetryWorker** bridges the gap:
- Runs in daemon thread (non-blocking to UI)
- Queries DroneBackend at 60 Hz (configurable)
- Transforms raw telemetry into display-ready format
- Uses thread-safe queue to deliver frames to Tkinter

---

## Data Flow Architecture

### High-Level Flow

```
			  ┌─ Windows Serial Port (57600 baud)
			  │
		 ┌────▼────────────────────────────────────┐
		 │     BETAFLIGHT FLIGHT CONTROLLER         │
		 │  (MSP Protocol @ 100 Hz polls)           │
		 │                                          │
		 │  Sensors:                                │
		 │  • MPU-6500 (IMU) → MSP_RAW_IMU (102)   │
		 │  • NEO-M10 (GPS) → MSP_RAW_GPS (106)    │
		 │  • BMP280 (Baro) → MSP_ALTITUDE (109)   │
		 │  • QMC5883L (Mag) → MSP_DEBUG (254)     │
		 │  • Motor outputs → MSP_MOTOR (104)      │
		 │  • RC inputs → MSP_RC (105)             │
		 └────┬────────────────────────────────────┘
			  │ (MSP responses)
		 ┌────▼──────────────────┐
		 │   DroneLink (C++)      │
		 │  (communication loop)  │
		 │                        │
		 │ Decoding/Parsing       │
		 │ Thread-safe state      │
		 └────┬──────────────────┘
			  │ (getLatestState())
		 ┌────▼────────────────────────────────┐
		 │   TelemetryWorker (Python)           │
		 │   (60 Hz polling background thread)  │
		 │                                      │
		 │   Transforms → ui_data dict         │
		 │   Queues frame (FIFO, max 3)        │
		 └────┬────────────────────────────────┘
			  │ (get_frame())
		 ┌────▼────────────────────┐
		 │   Tkinter Main Loop      │
		 │   (responsive UI thread) │
		 │                          │
		 │   Update widgets         │
		 │   Handle user input      │
		 └──────────────────────────┘
```

### Cycle Breakdown (per 10 ms tick in communicationLoop)

1. **Calibration checks** — Mag/accel calibration state tracking
2. **MSP polls 1-12** — Standard telemetry (STATUS, RAW_IMU, ATTITUDE, ANALOG, DEBUG, ALTITUDE, RAW_GPS, COMP_GPS, NAV_STATUS, STATUS_EX, MOTOR, RC)
3. **Throttled polls** — SV_INFO every ~1s, BATTERY_STATE every ~500ms
4. **State update** — Atomic snapshot written to `currentState` under mutex
5. **Health tracking** — Consecutive failure counter for link diagnostics

---

## Communication Protocol

### MSP (MultiWii Serial Protocol)

**Frame Format:**
```
[0]   Start marker       '$'
[1]   Direction          'M' (FC → host)  or  '<' (host → FC)
[2]   Size               Payload length (0–255)
[3]   Command            MSP command ID (101–254, etc.)
[4..] Payload            Raw data bytes
[n]   CRC                Fletcher's checksum
```

**Example: MSP_RAW_IMU (102)** — 12 bytes payload

```cpp
struct IMURaw {
	int16_t ax, ay, az;   // accelerometer (3 axes, 2 bytes each)
	int16_t gx, gy, gz;   // gyroscope (3 axes, 2 bytes each)
};
```

Parsed from little-endian byte stream:
```
buf[0..1]   = accX   (int16_t)
buf[2..3]   = accY   (int16_t)
buf[4..5]   = accZ   (int16_t)
buf[6..7]   = gyroX  (int16_t)
buf[8..9]   = gyroY  (int16_t)
buf[10..11] = gyroZ  (int16_t)
```

### Betaflight Integration

**Confirmed Hardware/FC Config:**
- **FC Model:** F405 V3 or similar (Betaflight 4.5.x)
- **GPS Module:** NEO-M10 on UART1 (115200 baud internally, set by BF)
- **USB MSP Port:** 57600 baud (fixed)
- **Arming:** RadioMaster Pocket transmitter channel 4 (ARM switch)

**Passthrough Mode (GPS Control)**

When BF is configured with `gps_auto_config=OFF`, DroneLink can activate GPS passthrough:

1. **Send MSP_SET_PASSTHROUGH (245)** with GPS UART index
   ```
   Host → FC:  $M< 0x01 0xF5 [UART_INDEX] [CRC]
   ```
2. **FC exits MSP, enters raw byte forwarding**
   - All subsequent bytes are forwarded to/from NEO-M10 verbatim
   - No MSP framing
3. **UBX protocol** (u-blox native) can be used directly
4. **Recovery:** Close + reopen serial port (USB chip reset returns FC to MSP mode)

**Note:** In normal mode (`gps_auto_config=ON`), DroneLink never enters passthrough; BF handles GPS internally.

---

## Thread Safety & State Management

### Architecture Pattern

**Golden Rule:** `DroneState` is written **exclusively** by the `communicationLoop()` worker thread, and read by external threads via a **mutex-protected snapshot**.

```cpp
class DroneLink {
private:
	std::mutex dataMutex;
	DroneState currentState;  // WRITER: communicationLoop() only
							  // READERS: external threads (via getLatestState)

	std::thread workerThread;
	std::atomic<bool> connected;
	std::atomic<bool> keepRunning;
	std::atomic<bool> magCalRequested;
	std::atomic<bool> accCalRequested;

public:
	DroneState getLatestState() {
		std::lock_guard<std::mutex> lock(dataMutex);
		return currentState;  // Atomic copy returned to caller
	}
};
```

### Guarantees

1. **Zero Races:** Worker writes; others copy-on-read.
2. **Consistent Snapshots:** Each `getLatestState()` call returns a coherent state (all fields from same polling cycle).
3. **Non-Blocking Reads:** Snapshot is fast; no waits on worker.
4. **Simple Calibration Control:** Atomic flags (`magCalRequested`, `accCalRequested`) signal intent without synchronous response.

### Atomic Flags

```cpp
magCalRequested.store(true);  // UI thread signals request
// In communicationLoop():
if (magCalRequested.exchange(false)) {  // Worker consumes & resets
	sendMSP(MSP::MAG_CAL);
	magCalActive_ = true;
	magCalStartTime_ = std::chrono::steady_clock::now();
}
```

---

## Sensor Systems

### IMU (Inertial Measurement Unit) — MPU-6500

**Sensor:** MPU-6500 (6-axis: accel + gyro)

**Raw Data Range (MSP_RAW_IMU):**
- **Accelerometer:** ±4g → LSB/g = 8192 counts/g
- **Gyroscope:** ±2000°/s → LSB/°/s = 16.4 counts/°/s

**Data Retrieval:**
1. DroneLink queries `MSP_RAW_IMU` (cmd 102) every 10 ms
2. Parses 12-byte payload into `DroneState.ax/ay/az/gx/gy/gz`
3. Python `IMUSensor` class provides scaling utilities

**Scaling Example:**
```cpp
float accelG = rawCount / 8192.0f;       // Convert to g
float gyroDegreesPerSec = rawCount / 16.4f;  // Convert to °/s
```

### GPS — NEO-M10

**Sensor:** u-blox NEO-M10 (multi-GNSS: GPS, GLONASS, Galileo, BeiDou)

**Data Sources:**
- **MSP_RAW_GPS (106):** Lat/lon, altitude, speed, course, fix type, sat count
- **MSP_COMP_GPS (107):** Distance & bearing to home, GPS heartbeat
- **MSP_GPS_SV_INFO (164):** Satellite list (updated ~1s, 100 ticks)
- **MSP_NAV_STATUS (121):** GPS nav engine status & fix flags

**Fix Types:**
```
0 = No fix
1 = GPS fix (2D)
2 = DGPS fix
3 = PPS fix
4 = Real Time Kinematic (RTK) fixed
5 = Real Time Kinematic (RTK) float
6 = Compass heading (requires magnetometer)
```

**Data Flow:**
```
NEO-M10 (UART1, 115200 baud)
	↓ (Betaflight internal)
GPS state in FC flash
	↓ (MSP polling)
DroneLink parses → DroneState.gps
	↓ (Python binding)
TelemetryWorker → GPSWidget display
```

### Barometer — BMP280

**Sensor:** BMP280 (pressure/temperature → altitude)

**Data Source:** MSP_ALTITUDE (109)

**Payload:**
```cpp
int32_t baroAltitudeCm;   // Altitude in cm
int16_t baroVarioCmPerSec;  // Vertical speed in cm/s
```

**Usage:** Display altitude & climb rate (vario) to pilot

### Magnetometer — QMC5883L

**Sensor:** QMC5883L (3-axis compass, calibrable)

**Data Source:** MSP_DEBUG (254) when `debug_mode=MAG_CALIB` in BF CLI

**Payload (6 bytes):**
```cpp
int16_t magX, magY, magZ;
```

**Processing:**
1. Raw magnetometer vector → 2D heading (tilt-uncorrected)
2. Heading range: 0–360°
3. Can be combined with gyro yaw for better orientation estimate

**Calibration:** Required before reliable heading; launched via UI button → sends MSP_MAG_CAL (205), active for 30s

---

## UI & Telemetry Pipeline

### TelemetryWorker (Python)

**Purpose:** Bridge between C++ backend and Tkinter UI thread

**Design:**
```python
class TelemetryWorker:
	def __init__(self, hub):
		self._hub = hub  # C++ DroneLink instance
		self._queue = queue.Queue(maxsize=3)  # Bounded buffer
		self._running = threading.Event()
		self._thread = None

	def _run(self):
		"""Background thread target."""
		while self._running.is_set():
			state = self._hub.getLatestState()
			ui_data = self._transform_state(state)
			try:
				self._queue.put_nowait(ui_data)
			except queue.Full:
				# Drop oldest frame, keep buffer fresh
				try:
					self._queue.get_nowait()
				except queue.Empty:
					pass
				self._queue.put_nowait(ui_data)
			time.sleep(1.0 / POLL_HZ)  # 60 Hz
```

**Frame Rate:** 60 Hz (configurable)
**Buffer Depth:** 3 frames (oldest discarded if new arrives before consumption)
**No Blocking:** Tkinter never waits; just reads latest available

### Instrument Panels

**Draggable Panel System:**
```
DroneCockpitUI (Tkinter root)
  ├─ Canvas (workspace)
  │   ├─ DraggablePanel "IMU"
  │   ├─ DraggablePanel "ADI" (Attitude Indicator)
  │   ├─ DraggablePanel "GPS"
  │   ├─ DraggablePanel "Baro" (altitude)
  │   ├─ DraggablePanel "Mag" (magnetometer)
  │   ├─ DraggablePanel "FC Status"
  │   └─ DraggablePanel "Arming Diagnostics"
  │
  └─ Menu (export, save layout, calibrate, etc.)
```

Each panel:
- Is **draggable** (left-click title bar)
- Is **resizable** (drag edges/corners)
- Can be **locked** (right-click context menu)
- Layout is **persisted** to `cockpit_layout.json`

### Update Flow

```
1. TelemetryWorker._run() polls every 16.7 ms (60 Hz)
   ↓
2. Transforms DroneState → ui_data dict
   ↓
3. Enqueues to _queue (non-blocking, bounded)
   ↓
4. Tkinter main loop (every ~50 ms) calls worker.get_frame()
   ↓
5. Consumes latest frame from queue
   ↓
6. Distribute fields to panels:
   - IMUWidget.update(imu_data)
   - GPSWidget.update(gps_data)
   - FCStatusWidget.update(armed, modes, sensors)
   - ... etc.
   ↓
7. Tkinter renders updated widgets
```

---

## Calibration System

### Magnetometer Calibration

**Trigger:** User clicks "Calibrate Mag" in UI

**Sequence:**

1. UI calls `hub.startMagCalibration()`
2. Sets atomic flag: `magCalRequested = true`
3. Worker thread loop detects flag and **sends MSP_MAG_CAL (205)**
4. Sets `magCalActive_ = true`, records start time
5. Continues normal polling for 30 seconds
6. Exposes remaining time via `DroneState.magCalSecondsRemaining`
7. UI displays countdown
8. After 30s, timeout triggers completion

**Requirements:** Drone must be armed; pilot manually rotates aircraft on all axes during the calibration window.

### Accelerometer Calibration

**Trigger:** User clicks "Calibrate Accel" in UI

**Sequence:** Same pattern as mag calibration, but:
- **Command:** MSP_ACC_CAL (206)
- **Duration:** 5 seconds
- **Requirement:** Aircraft must be level on a flat, stable surface

**Typical Use:** After physical damage or FC replacement

### Status Tracking

```cpp
struct DroneState {
	bool magCalActive = false;
	int  magCalSecondsRemaining = 0;
	bool accCalActive = false;
	int  accCalSecondsRemaining = 0;
};
```

UI consumes these fields to display progress bar and countdown timer.

---

## Health & Diagnostics

### Link Health Metric

**Tracked in DroneLink:**

```cpp
int consecutiveFails = 0;  // Incremented on poll failure
// ... on success:
if (anySuccess) {
	consecutiveFails = 0;
	pending.linkHealthy = (consecutiveFails < FAIL_THRESHOLD);
}
```

**Threshold:** 5 consecutive failures → `linkHealthy = false`

### Diagnostics Exposed

| Field | Purpose |
|-------|---------|
| `lastRttMs` | Round-trip time of last IMU poll (latency indicator) |
| `fcCycleMs` | FC's own loop cycle time (from MSP_STATUS) |
| `linkHealthy` | Boolean: >5 consecutive failures? |
| `packetCount` | Total packets received (rollover counter) |

**UI Display:** FCStatusWidget shows red/yellow/green link indicator based on `linkHealthy` and `rssi` (signal strength).

---

## Development Notes

### Adding a New MSP Command

1. **Define command ID** in `DroneLink.h` `namespace MSP { ... }`
2. **Create parser** function in `DroneLink.cpp`:
   ```cpp
   void DroneLink::parseMyCommand(const std::vector<uint8_t>& buf, DroneState& pending) {
	   if (buf.size() < expected_size) return;
	   // Parse buf[] into pending.* fields
   }
   ```
3. **Add to poll loop** in `communicationLoop()`:
   ```cpp
   { auto buf = sendMSP(MSP::MY_COMMAND); parseMyCommand(buf, pending); }
   ```
4. **Expose in DroneState** struct as new fields

### Adjusting Sensor Scales

Edit constants in header files:
- **IMUSensor.h:** `ACC_SCALE`, `GYRO_SCALE`
- **BaroBMP280.h:** Altitude conversion factors
- **MagQMC5883L.h:** Magnetometer sensitivity

Rebuild C++ backend after changes.

### Performance Tuning

**Communication Loop Budget:** 10 ms per cycle at 100 Hz

If overrunning:
```cpp
// In DroneLink.h
static constexpr int POLL_INTERVAL_MS = 20;  // Drop to 50 Hz
```

This halves the poll rate but doubles available CPU per cycle.

**Telemetry Display:** 60 Hz configured in `telemetry_worker.py`

```python
POLL_HZ = 60
_POLL_INTERVAL = 1.0 / POLL_HZ
```

Reduce if GPU-constrained; increase if UI is sluggish.

---

## Summary

R.A.D is a tightly integrated two-process system:

| Layer | Tech | Role |
|-------|------|------|
| **Backend** | C++ (Windows) | Low-level MSP polling, sensor parsing, state management |
| **Frontend** | Python/Tkinter | Display, user input, panel management |
| **IPC** | Python ctypes bindings | C++ methods callable from Python |
| **Sync** | Queue + Events | Non-blocking telemetry delivery |

The architecture prioritizes:
- **Responsiveness** (100 Hz polling, non-blocking UI)
- **Thread Safety** (mutex-protected snapshots, atomic flags)
- **Modularity** (sensor classes, plugin widgets)
- **Reliability** (health tracking, link diagnostics)

