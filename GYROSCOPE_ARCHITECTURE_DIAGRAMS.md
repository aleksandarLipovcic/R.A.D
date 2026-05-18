# Gyroscope (RAW_IMU) Architecture & Data Flow Diagrams

## Overview

This document provides visual diagrams showing how gyroscope data flows from the physical MPU-6500 sensor through the Betaflight FC, DroneLink, Python bindings, and finally to the GUI display.

---

## 1. Complete Data Flow: From Sensor to GUI

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        PHYSICAL HARDWARE LAYER                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  MPU-6500 IMU Sensor (on Betaflight FC)                                    │
│  ├─ Accelerometer: ±4g range (8192 LSB/g)                                  │
│  ├─ Gyroscope:     ±2000°/s range (16.4 LSB/°/s)                           │
│  └─ Magnetometer:  QMC5883L (separate, not in RAW_IMU)                    │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
									▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    BETAFLIGHT FIRMWARE LAYER                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ReadIMUSensor() [called by gyroUpdateSensor() every ~1 ms]                │
│  └─ Reads raw ADC counts from MPU-6500 I2C                                 │
│     ├─ accX, accY, accZ (int16_t, LSB counts)                              │
│     └─ gyroX, gyroY, gyroZ (int16_t, LSB counts)                           │
│                                                                               │
│  Gyro Update Loop [runs at ~8 kHz, critically timed]                       │
│  └─ Applies:                                                                │
│     ├─ Gyro offsets (calibration)                                          │
│     ├─ Low-pass filtering                                                   │
│     ├─ Temperature compensation                                             │
│     └─ Stores in thread-safe struct for MSP export                         │
│                                                                               │
│  MSP_RAW_IMU (cmd 102) Handler [runs every ~10 ms]                         │
│  └─ Exports current raw counts to MSP frame buffer                         │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
									▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MSP SERIAL PROTOCOL                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  USB Serial: 57600 baud (DroneLink ↔ Betaflight FC)                        │
│                                                                               │
│  MSP_RAW_IMU Frame Format:                                                  │
│  ┌──────┬──────┬─────────┬─────┬──────────────────────┬──────────┐        │
│  │ "$M<"│ 12   │   102   │ ... │   12 bytes payload   │ checksum │        │
│  │      │      │ (cmd)   │     │  (6 × int16_t)       │          │        │
│  └──────┴──────┴─────────┴─────┴──────────────────────┴──────────┘        │
│                                                                               │
│  Payload (12 bytes, little-endian):                                         │
│    Bytes 0-1:   accel_x (i16 LE)  -32768 to +32767 counts                 │
│    Bytes 2-3:   accel_y (i16 LE)                                           │
│    Bytes 4-5:   accel_z (i16 LE)                                           │
│    Bytes 6-7:   gyro_x  (i16 LE)  ← GYROSCOPE X-AXIS                      │
│    Bytes 8-9:   gyro_y  (i16 LE)  ← GYROSCOPE Y-AXIS                      │
│    Bytes 10-11: gyro_z  (i16 LE)  ← GYROSCOPE Z-AXIS                      │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
									▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      DRONELINK C++ BACKEND LAYER                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  communicationLoop() [100 Hz worker thread]                                │
│  └─ Every 10 ms iteration:                                                  │
│     ├─ sendMSP(MSP::RAW_IMU)  ← Request raw IMU data (cmd 102)             │
│     │  ├─ Sends: $ M < 0 102 102                                           │
│     │  └─ Receives: 129-byte response with acc + gyro counts               │
│     │                                                                       │
│     └─ parseIMU(buf, pending)  ← Parse response into pending state         │
│        ├─ Reads 6 × int16_t from buffer                                   │
│        ├─ pending.ax/ay/az = accel counts                                  │
│        ├─ pending.gx/gy/gz = GYRO counts  ← Stored here                   │
│        └─ Returns bool success                                             │
│                                                                               │
│  commitState(pending)  [~10 ms, atomic mutex lock]                         │
│  └─ Atomically copies:                                                      │
│     ├─ pending.gx/gy/gz → currentState.gx/gy/gz                           │
│     └─ Updates shared state for Python threads                             │
│                                                                               │
│  DroneState struct (shared memory)                                          │
│  ├─ int16_t gx, gy, gz;  ← Raw gyro counts (-32768 to +32767)             │
│  └─ Protected by dataMutex                                                 │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
									▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     PYTHON BINDING LAYER (pybind11)                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  DroneLink.getLatestState()  [Python → C++]                                │
│  └─ Returns thread-safe copy of DroneState                                 │
│     ├─ state.ax, state.ay, state.az (accel counts)                         │
│     ├─ state.gx, state.gy, state.gz (GYRO COUNTS)  ← Available here       │
│     └─ ~10 ms update rate (synchronized with commitState)                  │
│                                                                               │
│  IMUSensor class  [Python convenience wrapper]                             │
│  ├─ getRawData()  → returns IMUData { gx, gy, gz } as-is                   │
│  └─ getScaledData() → returns IMUScaled { gyroX, gyroY, gyroZ }            │
│     └─ Conversion: counts ÷ 16.4 = degrees/second                         │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
									▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PYTHON GUI LAYER                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  MainWindow.py                                                              │
│  └─ Main update loop (every ~100 ms, rate-limited)                         │
│     ├─ state = drone_link.getLatestState()                                │
│     ├─ scaled_data = imu_sensor.getScaledData()                           │
│     │  └─ { gyroX: °/s, gyroY: °/s, gyroZ: °/s }                          │
│     └─ imu_widget.update_imu(scaled_data)                                 │
│                                                                               │
│  IMUWidget.py                                                               │
│  └─ update_imu(data)  ← Receives scaled gyro data in °/s                  │
│     ├─ Checks against thresholds:                                          │
│     │  ├─ GYRO_WARN_DPS = 30°/s  (yellow alert)                           │
│     │  └─ GYRO_CRIT_DPS = 100°/s (red alert, flashing)                    │
│     │                                                                       │
│     ├─ Updates cell background colors:                                     │
│     │  ├─ Dark (#0d1520) if within limits                                 │
│     │  ├─ Amber (#C87000) if in warning range                            │
│     │  └─ Red (#CC0000) flashing if critical                             │
│     │                                                                       │
│     ├─ Formats numeric display (X.XX °/s)                                 │
│     └─ Renders on Tkinter canvas                                          │
│                                                                               │
│  Visual Display:                                                             │
│  ┌─────────────────────────────────────────┐                              │
│  │  Roll Rate  │  45.2 °/s   │  WARN      │  (amber, 30 < 45.2 < 100)    │
│  ├─────────────────────────────────────────┤                              │
│  │  Pitch Rate │  12.5 °/s   │  OK        │  (green, < 30)               │
│  ├─────────────────────────────────────────┤                              │
│  │  Yaw Rate   │ 150.0 °/s   │ CRIT       │  (red flashing, > 100)       │
│  └─────────────────────────────────────────┘                              │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Timing Diagram: Gyro Update Cadence

```
TIME ───────────────────────────────────────────────────────────────────────────►

FC Internal Sensors (Betaflight):
  gyroUpdateSensor()  [~8 kHz, critical real-time]
  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │
  └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘
	 1ms period: very frequent, filtered updates

DroneLink Poll Loop (C++, 100 Hz):
  tick 0        tick 1        tick 2        tick 3
  ├─────────────┼─────────────┼─────────────┼──────────
  │ sendMSP(102)│ sendMSP(102)│ sendMSP(102)│ sendMSP
  │ parseIMU()  │ parseIMU()  │ parseIMU()  │ parseIM
  │ commitState │ commitState │ commitState │ commitS
  └─────────────┴─────────────┴─────────────┴──────────
	   10 ms          10 ms        10 ms       10 ms

Python GUI Update (Tkinter, ~100 ms):
  ┌─ getLatestState()
  │  ├─ [reads current gx/gy/gz]
  │  ├─ scale by 1/16.4
  │  └─ returns IMUScaled
  │
  ├─ Display in IMUWidget
  │  ├─ check thresholds
  │  ├─ update colors (green/amber/red)
  │  └─ render text
  │
  └─ Next update in ~100ms

Summary:
  FC Gyro reads:     Every 1 ms   (very responsive)
  DroneLink polls:   Every 10 ms  (MSP cadence)
  Python displays:   Every 100 ms (GUI refresh rate)

Latency path: Gyro sensor → FC filtering (~10 ms) → MSP poll (~10 ms)
			  → DroneLink parse (~1 ms) → Python read (~1 ms) → GUI (~10 ms)
			  ≈ 30-50 ms total end-to-end latency
```

---

## 3. Data Transformation Pipeline

```
Physical Sensor Value → ADC Count → MSP Frame → DroneState → Python → GUI

┌──────────────────┐
│ Gyroscope sensor │
│   (MPU-6500)     │
│ actual motion    │
│ in physical space│
└────────┬─────────┘
		 │ 1 ms feedback loop to FC
		 ▼
┌──────────────────────────────────┐
│  ADC Conversion                  │
│  Motion → Electrical voltage     │
│  Voltage → Digital count         │
│  Range: ±2000°/s = 16.4 LSB/°/s │
│  Stored: int16_t (-32768..+32767)│
│  Typical: ±500 counts for moderate│
│          rotation rate            │
└────────┬─────────────────────────┘
		 │ 100 Hz MSP poll
		 ▼
┌──────────────────────────────────┐
│  MSP_RAW_IMU Frame (cmd 102)    │
│  Format: 6 × int16_t (12 bytes) │
│  Little-endian encoding          │
│  Bytes 6-7: gyro_x counts        │
│  Bytes 8-9: gyro_y counts        │
│  Bytes 10-11: gyro_z counts      │
│                                  │
│  Example raw frame:              │
│  [... 10 34 05 12 FF ...]        │
│       ↑gyX ↑gyY ↑gyZ             │
│       0x340A = 13322 counts      │
│       0x1205 = 4626 counts       │
│       0xFF05 = -251 counts       │
└────────┬──────────────────────────┘
		 │ Serial @ 57600 baud (1.7 ms/frame)
		 ▼
┌──────────────────────────────────┐
│  DroneState Structure (C++)      │
│  int16_t gx, gy, gz;             │
│  Thread-safe (mutex protected)   │
│  Updated every 10 ms             │
│  Holds raw counts (-32768..+327 67)
│                                  │
│  Example state:                  │
│  { gx: 13322,                    │
│    gy: 4626,                     │
│    gz: -251 }                    │
└────────┬──────────────────────────┘
		 │ getLatestState() via pybind11
		 ▼
┌──────────────────────────────────┐
│  Python IMUSensor.getScaledData()│
│  Conversion: counts / 16.4 → °/s │
│  Returns IMUScaled {             │
│    float gyroX, gyroY, gyroZ     │
│  }                               │
│                                  │
│  Example scaled:                 │
│  { gyroX: 811.71°/s,  ← dividing │
│    gyroY: 282.07°/s,     by      │
│    gyroZ: -15.30°/s  }    16.4   │
└────────┬──────────────────────────┘
		 │ ~100 ms GUI update
		 ▼
┌──────────────────────────────────┐
│  IMUWidget.update_imu()          │
│  Check thresholds:               │
│  - WARN at 30°/s                 │
│  - CRIT at 100°/s                │
│                                  │
│  Color assignment:               │
│  IF |rate| < 30:     GREEN       │
│  IF 30 ≤ |rate| < 100: AMBER    │
│  IF |rate| ≥ 100:    RED+FLASH   │
│                                  │
│  Render on canvas with colors    │
└────────┬──────────────────────────┘
		 │
		 ▼
┌──────────────────────────────────┐
│   DISPLAY IN GUI                 │
│  ┌──────────────────────────────┐│
│  │ Roll Rate  │  45.2 °/s │WARN ││
│  │ Pitch Rate │  12.5 °/s │ OK  ││
│  │ Yaw Rate   │ 150.0 °/s │CRIT ││
│  └──────────────────────────────┘│
└──────────────────────────────────┘
```

---

## 4. Thread Safety & Synchronization

```
Betaflight FC (Real-time constraints):
  [Gyro Update ~8 kHz] ──────────────────────────────────► Local IMU buffer
															  │
									┌─────────────────────────┘
									│
DroneLink Background Thread (100 Hz):
  ┌─ Iteration N
  │  ├─ sendMSP(RAW_IMU) ──► receive MSP response with latest gx/gy/gz
  │  ├─ parseIMU(buf) ──► Extract counts into pending.gx/gy/gz
  │  ├─ [Other parsers...]
  │  ├─ Acquire dataMutex
  │  └─ commitState(pending) ──► currentState.gx/gy/gz = pending.gx/gy/gz
  │     │                         Release dataMutex
  │     │                         ↓
  │     └────────────────────► Shared Memory (currentState)
  │                            ├─ gx/gy/gz (int16_t)
  │                            └─ Protected by dataMutex
  │
  └─ Iteration N+1 (10 ms later)
	 ├─ [Process other data]
	 └─ Commit again

Python Main Thread (GUI thread, ~100 ms cadence):
  ┌─ Call getLatestState()
  │  ├─ Acquire dataMutex (blocks if commitState() in progress)
  │  ├─ Copy currentState to local variable
  │  └─ Release dataMutex
  │     │
  │     └─ Read state.gx/gy/gz (no longer blocked)
  │        ├─ Scale: gx / 16.4 = °/s
  │        └─ Display in IMUWidget
  │
  └─ Next update in ~100 ms
```

**Key Properties:**
- ✅ Writes (DroneLink) happen every 10 ms
- ✅ Reads (Python) happen every 100 ms
- ✅ Mutex ensures no torn reads
- ✅ No spin-locks; safe to call from any thread
- ✅ Thread-safe: DroneLink guarantees most recent state visible to Python

---

## 5. State Machine: From Sensor to Display

```
START
  │
  ├─► FC Powers on
  │    ├─ MPU-6500 initializes
  │    ├─ Gyro offset calibration runs
  │    └─ Gyro updates available every ~1 ms
  │
  ├─► DroneLink connects (serial port open)
  │    ├─ Start communicationLoop() worker thread
  │    └─ Thread starts polling at 100 Hz
  │
  ├─► First MSP_RAW_IMU poll (cmd 102)
  │    ├─ Send request
  │    ├─ Receive response (12 bytes payload)
  │    ├─ parseIMU() extracts gx/gy/gz counts
  │    ├─ Store in pending state
  │    ├─ commitState() → currentState.gx/gy/gz
  │    └─ gx/gy/gz available to Python [READY]
  │
  ├─► Python reads state (~100 ms cadence)
  │    ├─ getLatestState() returns DroneState
  │    ├─ Read state.gx/gy/gz (raw counts)
  │    ├─ Create IMUSensor and scale data
  │    ├─ Pass to IMUWidget.update_imu()
  │    └─ Render on GUI canvas
  │
  ├─► Continuous operation
  │    ├─ Every 10 ms: New MSP_RAW_IMU response parsed and committed
  │    ├─ Every 100 ms: Python reads latest state and updates display
  │    └─ Loop continues until disconnect
  │
  └─► Disconnect or error
	   ├─ communicationLoop() stops
	   ├─ Serial port closes
	   └─ gx/gy/gz become stale (not updated)
```

---

## 6. Error Handling & Failure Points

```
Where Gyro Data Can Be Lost:

┌────────────────────────────────────────────────────────────────────────────┐
│ POINT 1: FC Hardware Failure                                              │
│ ═════════════════════════════════════════════════════════════════════════ │
│ If MPU-6500 is not responding or has failed I2C:                          │
│   FC MSP_STATUS.sensorStatus & SensorStatus::GYRO == false                │
│ Check DroneState.sensorStatus in Python:                                  │
│   if not (state.sensorStatus & 0x0020):  # GYRO bit                       │
│       print("GYRO SENSOR FAILED!")                                        │
│ Action: No MSP_RAW_IMU data will be valid; check FC hardware             │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│ POINT 2: Serial Communication Timeout                                     │
│ ═════════════════════════════════════════════════════════════════════════ │
│ If DroneLink.sendMSP() times out waiting for response:                    │
│   parseIMU() returns false                                                │
│   commitState() skips that iteration (old values persist)                 │
│ Check: Verify serial port is open and baud rate is 57600                 │
│ Action: Reconnect DroneLink or check USB cable                           │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│ POINT 3: Parsing Error                                                    │
│ ═════════════════════════════════════════════════════════════════════════ │
│ If MSP frame is malformed or checksum fails:                              │
│   sendMSP() detects error and returns empty buffer                       │
│   parseIMU(empty_buf) returns false                                       │
│ Action: Add logging to parseIMU() to detect this                         │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│ POINT 4: Python Binding Issue                                             │
│ ═════════════════════════════════════════════════════════════════════════ │
│ If pybind11 binding for gx/gy/gz is missing or wrong type:                │
│   state.gx raises AttributeError                                          │
│ Action: Verify Bindings.cpp exposes DroneState.gx/gy/gz                  │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│ POINT 5: Scaling/Unit Conversion Error                                    │
│ ═════════════════════════════════════════════════════════════════════════ │
│ If GYRO_SCALE constant is wrong (should be 16.4, not 2048):               │
│   getScaledData() returns wrong units (e.g., 0.001°/s instead of 100°/s) │
│ Action: Verify IMUSensor.h has correct scale factor (1.0f / 16.4f)       │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│ POINT 6: GUI Display Bug                                                  │
│ ═════════════════════════════════════════════════════════════════════════ │
│ If IMUWidget.update_imu() is not called or data is None:                  │
│   Gyro data in backend is correct, but not visible in GUI                 │
│ Action: Follow debugging guide in GYROSCOPE_DEBUGGING_GUIDE.md           │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 7. Comparison: Gyro Data vs GPS Satellite Data

```
					GYROSCOPE (RAW_IMU)        GPS SATELLITES (GPS_SV_INFO)
═════════════════════════════════════════════════════════════════════════════
MSP Command         102 (RAW_IMU)               164 (GPS_SV_INFO)

Response Size       12 bytes                    129 bytes (32 sats × 4 + header)

Poll Rate           100 Hz (every 10 ms)        ~1 Hz (every 1000 ms)

Data Fields         6 int16_t                   32 × 4 bytes (satellite array)
					(ax, ay, az, gx, gy, gz)    

Update Frequency    Very frequent               Slow (1 second)

Data Type           Raw ADC counts              Parsed satellite objects

Scaling             Yes (÷16.4 → °/s)           Yes (GNSS ID, quality mapping)

Python Type         int16_t → float             SVInfoEntry vector

GUI Display         Numeric + alerts            Bar chart + list

Typical Values      -500 to +500 counts         10-32 objects with SVID, CNO, etc.

Importance          Real-time control          Navigation/position only
```

---

## Summary

The gyroscope data follows a clean, well-defined path:

1. **FC Hardware** (8 kHz) → Raw ADC counts
2. **Betaflight Filter** (10 ms) → Clean, offset-corrected counts
3. **MSP Serial** (57600 baud) → 12-byte frame with 6 × int16_t
4. **DroneLink C++** (100 Hz) → Parse and commit to shared state
5. **Python Binding** (pybind11) → Access via state.gx/gy/gz
6. **IMUSensor Class** → Scale and convert to physical units
7. **IMUWidget GUI** → Display with color alerts and thresholds

Each layer is well-documented and testable. See the detailed documents for implementation specifics.
