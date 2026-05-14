# Quick Reference: Satellite Data Pipeline

## What's Already Implemented ✅

### DroneLink.h (Header)
```cpp
struct DroneState {
    std::vector<SVInfoEntry> svList;       // ← Satellite vector
    bool                     svInfoValid;   // ← Data ready flag
    std::string              svSource;      // ← "MSP" or "UBX"
};

// Methods:
bool pollSatellitesMSP(DroneState& pending);  // ← Polls every 1 sec
bool parseMspSvInfo(const std::vector<uint8_t>& buf, DroneState& s);
```

### DroneLink.cpp (Implementation)
```cpp
// Lines 235-239: Poll scheduling (every 100 iterations = ~1 sec)
if (svPollTickCounter_ <= 0) {
    pollSatellitesMSP(pending);
    svPollTickCounter_ = SV_POLL_TICKS;
}

// Lines 274-285: Primary poll method
bool DroneLink::pollSatellitesMSP(DroneState& pending) {
    auto buf = sendMSP(MSP::GPS_SV_INFO);  // cmd 164
    std::vector<SVInfoEntry> sv;
    if (!GPSNeoM10::parseMspSvInfo(buf, sv)) return false;
    pending.svList = std::move(sv);
    pending.svInfoValid = true;
    pending.svSource = "MSP";
    return true;
}

// Lines 111-114: State export to Python
DroneState DroneLink::getLatestState() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentState;
}
```

### Bindings.cpp (Python Bridge)
```cpp
// Lines 62-89: SVInfoEntry bindings
py::class_<SVInfoEntry>(m, "SVInfoEntry")
    .def_readonly("gnss_name", &SVInfoEntry::gnssName)
    .def_readonly("svid", &SVInfoEntry::svid)
    .def_readonly("cno", &SVInfoEntry::cno)
    .def_readonly("quality", &SVInfoEntry::quality)
    .def_readonly("status_str", &SVInfoEntry::statusStr)
    .def_readonly("used", &SVInfoEntry::used)
    // ... more fields ...

// Lines 214-223: DroneState satellite fields
.def_readonly("sv_list", &DroneState::svList)
.def_readonly("sv_info_valid", &DroneState::svInfoValid)
.def_readonly("sv_source", &DroneState::svSource)

// Lines 244-246: Dictionary conversion
py::list sv_list;
for (const SVInfoEntry& sv : s.svList)
    sv_list.append(py::cast(sv));
```

---

## How Python Gets the Data

### Access Pattern 1: Direct Access
```python
drone = DroneBackend.DroneLink()
drone.connect("COM3")
time.sleep(1.5)  # Wait for first poll

state = drone.get_latest_state()

if state.sv_info_valid:
    for sv in state.sv_list:
        print(f"{sv.gnss_name} {sv.svid}: CNO={sv.cno} ({sv.status_str})")
```

### Access Pattern 2: Dictionary
```python
state_dict = state.to_dict()

sv_list = state_dict["gps_sv_list"]           # List of satellites
valid = state_dict["gps_sv_info_valid"]       # Boolean
source = state_dict["gps_sv_source"]          # "MSP" or "UBX"
```

---

## Data Flow Timeline

| When | What Happens |
|------|-------------|
| **T=0 ms** | `drone.connect()` called, worker thread starts |
| **T=10 ms** | First loop iteration, svPollTickCounter_ = 0 |
| **T~10-20 ms** | MSP cmd 164 sent, response parsed, svList populated |
| **T=20 ms** | `svInfoValid = true`, `svSource = "MSP"` |
| **T=1010 ms** | Second poll fires, svList updated |
| **Anytime** | Python calls `drone.get_latest_state()` → gets current svList |

---

## What's Guaranteed ✅

| Aspect | Guarantee |
|--------|-----------|
| **Data Freshness** | Updated every ~1 second (SV_POLL_TICKS = 100 @ 10ms loop) |
| **Thread Safety** | Mutex-protected state sharing (no race conditions) |
| **Error Handling** | Failed parse leaves state unchanged (no partial updates) |
| **API Availability** | All 11 SVInfoEntry fields exposed to Python |
| **Source Tracking** | `sv_source = "MSP"` identifies data path |
| **Readiness Flag** | `sv_info_valid = true` only after first successful parse |

---

## What's NOT Available in MSP Mode ⚠️

These fields are **always 0** when `sv_source == "MSP"` (they're in UBX but not MSP cmd 164):
- `elev` — Elevation above horizon (degrees)
- `azim` — Azimuth (degrees)
- `prRes` — Pseudorange residual

**Python UI should hide these columns when `sv_source == "MSP"`** (see Bindings.cpp lines 58-60).

---

## What's Available ✅

These fields are **always populated** via MSP:
- `gnss_name` — "GPS", "GLONASS", "Galileo", "BeiDou", "SBAS", "QZSS"
- `svid` — Satellite vehicle ID (1-32 for GPS, etc.)
- `cno` — Carrier-to-noise ratio in dBHz (0-63, 0 = not tracked)
- `quality` — Signal quality 0-7 (0=none, 4+=usable, 5-7=fully locked)
- `gnss_id` — GNSS system ID (0-6)
- `flags` — Packed flags (bits 0-2=quality, bit 3=used)
- `used` — Boolean: true when quality >= 4 (contributes to fix)
- `status_str` — "used" / "tracked" / "idle" / "searching" / "acquired" / "unusable"
- `chn` — Channel index (0-31 for M10)

---

## No Changes Needed ✅

The entire pipeline is working:
- ✅ C++ parser (GPSneoM10.cpp) — verified against Python test
- ✅ Polling (DroneLink.cpp) — every 1 second via tick counter
- ✅ State management (DroneLink.h) — thread-safe mutex
- ✅ Python bindings (Bindings.cpp) — complete field coverage

**Ready for UI integration immediately.**

---

## Integration Checklist for UI

- [ ] Import `DroneBackend` module
- [ ] Create `DroneLink()` instance
- [ ] Call `connect("COM3")` or `AutoDetectF405()`
- [ ] Wait 1-2 seconds for first satellite poll
- [ ] Check `state.sv_info_valid` before accessing `state.sv_list`
- [ ] Check `state.sv_source == "MSP"` before hiding elev/azim columns
- [ ] Iterate `state.sv_list` to display satellites
- [ ] Highlight satellites where `sv.used == True`
- [ ] Use `sv.cno` for signal strength bar/graph
- [ ] Display `sv.status_str` and `sv.gnss_name` for each satellite

---

**Status:** Ready to integrate into DroneCockpitUI ✅
