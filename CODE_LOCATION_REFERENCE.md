# Code Location Reference: Satellite Pipeline

## Quick File Navigation

### 📄 GPSneoM10.h (Data Structures & Parser Interface)
```
Lines 87-117:     SVInfoEntry struct — one satellite's data
  - Line 105-109: Fields always available (chn, svid, quality, cno, gnssId)
  - Line 110-112: Fields only in UBX mode (elev, azim, prRes)
  - Line 113-116: Computed fields (gnssName, statusStr, flags, used)

Lines 119-160:    GPSNeoM10 class — static parser functions
  - Line 135-161: parseMspSvInfo() declaration
```

### 📄 GPSneoM10.cpp (Parser Implementation)
```
Lines 60-87:      gnssName() function — GNSS ID → string mapping
Lines 88-99:      qualityStatus() function — quality → string mapping

Lines 174-273:    parseMspSvInfo() implementation ⭐
  - Line 196-210: Frame validation (MSP header, cmd 164, payload length)
  - Line 215:     GNSS ID encoding detection (numCh > 16)
  - Line 223-269: Channel iteration loop (4 bytes per satellite)
  - Line 233-239: GNSS ID decoding from nibbles
  - Line 241-268: SVInfoEntry population
  - Line 257-266: Status string computation
  - Line 271:     Return populated vector
```

### 📄 DroneLink.h (State & Methods)
```
Lines 145-162:    DroneState struct ⭐
  - Line 160:     std::vector<SVInfoEntry> svList
  - Line 161:     bool svInfoValid
  - Line 162:     std::string svSource

Lines 229-232:    svPollTickCounter_ member variable
  - SV poll scheduling tick counter

Lines 254-256:    pollSatellitesMSP() method declaration
Lines 277:        parseMspSvInfo() wrapper method declaration
```

### 📄 DroneLink.cpp (Communication Loop & Polling)
```
Lines 88-99:      connect() method
  - Starts worker thread running communicationLoop()

Lines 111-114:    getLatestState() method ⭐
  - Thread-safe snapshot export
  - EXPOSED TO PYTHON

Lines 235-239:    Communication loop — satellite polling ⭐
  - SV_POLL_TICKS = 100 iterations = ~1 second
  - Calls pollSatellitesMSP(pending)

Lines 274-285:    pollSatellitesMSP() method ⭐
  - Sends MSP cmd 164
  - Calls GPSNeoM10::parseMspSvInfo()
  - Updates pending.svList, pending.svInfoValid, pending.svSource

Lines 609-611:    parseNavStatus() wrapper (for completeness)

Lines 620-627:    parseMspSvInfo() wrapper ⭐
  - Bridge between low-level MSP buffer and DroneState
  - Same logic as pollSatellitesMSP() but independent

Lines 633-636:    commitState() method
  - Locks mutex and copies pending → currentState
  - Called at end of each loop iteration
```

### 📄 Bindings.cpp (Python API)
```
Lines 46-49:      SVInfoEntry Python binding start
Lines 62-89:      SVInfoEntry .def_readonly() declarations ⭐
  - All 11 satellite fields exposed to Python
  - Includes docstrings for each field

Lines 204-208:    gps field (GPSReading struct)
  - Provides access to GPS position data

Lines 210-223:    Satellite list fields ⭐
  - Line 214: .def_readonly("sv_list", &DroneState::svList)
  - Line 218: .def_readonly("sv_info_valid", &DroneState::svInfoValid)
  - Line 220: .def_readonly("sv_source", &DroneState::svSource)
  - Each with docstring guidance

Lines 242-325:    to_dict() method ⭐
  - Converts DroneState to Python dict
  - Lines 244-246: Converts SVInfoEntry vector to Python list
  - Lines 311-313: Satellite list in dict output
    - "gps_sv_list" — list of satellites
    - "gps_sv_info_valid" — validity flag
    - "gps_sv_source" — data source identifier

Lines 330-340:    DroneLink Python binding
  - Line 338: .def("get_latest_state", ...) — PYTHON ENTRY POINT
```

---

## Key Constants

### In DroneLink.h
```cpp
Line 105:   static constexpr int SV_POLL_TICKS = 100;
            // 100 iterations @ 10ms/iteration = ~1 second

Line 179:   static constexpr int POLL_INTERVAL_MS = 10;
            // 10 ms loop rate = 100 Hz
```

### In DroneLink.cpp
```cpp
Lines 37-70:   MSP namespace (includes GPS_SV_INFO = 164)
```

---

## Data Flow: Critical Line Numbers

### 1️⃣ Poll Scheduling
```cpp
DroneLink.cpp:235-239        ← When to poll
  ↓
DroneLink.cpp:274-285        ← How to poll (pollSatellitesMSP)
  ↓
GPSneoM10.cpp:174-273        ← Parse response (parseMspSvInfo)
  ↓
DroneLink.cpp:633-636        ← Commit state (commitState)
```

### 2️⃣ State Snapshot Export
```cpp
Python calls: drone.get_latest_state()
  ↓
Bindings.cpp:338             ← Python entry point
  ↓
DroneLink.cpp:111-114        ← C++ implementation
  ↓
DroneState struct            ← Contains svList, svInfoValid, svSource
```

### 3️⃣ Python Access Pattern
```python
state = drone.get_latest_state()  ← From Bindings.cpp:338

state.sv_list                      ← Vector<SVInfoEntry>
                                    From Bindings.cpp:214
                                    Comes from DroneLink.h:160

state.sv_info_valid                ← Bool
                                    From Bindings.cpp:218
                                    Comes from DroneLink.h:161

state.sv_source                    ← String ("MSP" or "UBX")
                                    From Bindings.cpp:220
                                    Comes from DroneLink.h:162

for sv in state.sv_list:
    sv.gnss_name                   ← From Bindings.cpp:84
    sv.svid                        ← From Bindings.cpp:66
    sv.cno                         ← From Bindings.cpp:74
    sv.quality                     ← From Bindings.cpp:71
    sv.status_str                  ← From Bindings.cpp:86
    sv.used                        ← From Bindings.cpp:88
```

---

## Thread Safety Barriers

### Mutex Protection Points
```cpp
DroneLink.cpp:633      commitState()
                       ↑
                       std::lock_guard<std::mutex> lock(dataMutex);
                       ↑
                       Protects: DroneLink.h:241 currentState

DroneLink.cpp:111-114  getLatestState()
                       ↑
                       std::lock_guard<std::mutex> lock(dataMutex);
                       ↑
                       Protects: Reading currentState
```

### Worker Thread
```cpp
DroneLink.cpp:97       std::thread workerThread = communicationLoop()
                       ↑
                       Isolated DroneState pending
                       Committed via mutex in commitState()
```

---

## Optional: Unit Test Injection Points

```cpp
DroneLink.h:277        parseMspSvInfo(buf, s)
                       ↑
                       This method accepts raw MSP buffer
                       Useful for unit tests that don't need serial I/O

Example usage:
    std::vector<uint8_t> mock_msp_response = {
        '$', 'M', '>', 0x80, 164,  // MSP header
        0x20,                        // numCh = 32
        // ... 128 channel bytes ...
    };

    DroneState test_state;
    bool result = droneLink.parseMspSvInfo(mock_msp_response, test_state);
```

---

## Backward Compatibility Notes

### Legacy UBX Passthrough Path
```cpp
DroneLink.cpp:288-290     pollSatellites() — DEPRECATED, kept for applyGPSConfig()
DroneLink.cpp:300-372     pollSatellites() implementation — NOT called from loop
GPSneoM10.cpp:295-340     parseNavSvInfo() — Parses UBX-NAV-SAT (alternate format)

Status: Intentionally inactive
Reason: MSP cmd 164 is faster, simpler, doesn't require passthrough
Use: Only if applyGPSConfig() needs it for configuration writes
```

---

## Summary: Where to Look for What

| Task | File | Lines |
|------|------|-------|
| Understand SVInfoEntry | GPSneoM10.h | 104-117 |
| See MSP parser logic | GPSneoM10.cpp | 174-273 |
| Trace state flow | DroneLink.cpp | 235-239, 274-285 |
| Add feature to DroneState | DroneLink.h | 160-162 |
| Modify poll interval | DroneLink.h | 105 |
| Change poll frequency | DroneLink.h | 105 |
| Add Python bindings | Bindings.cpp | 62-89, 214-223 |
| Test state export | DroneLink.cpp | 111-114 |
| Understand timing | DroneLink.cpp | 254-258 |

---

**Note:** All line numbers are approximate. Search by function name or comment for exact locations.
