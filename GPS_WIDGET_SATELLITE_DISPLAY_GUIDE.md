# GPS Widget: Satellite Display Implementation & Troubleshooting

## Overview

The **GPSWidget** is the Tkinter GUI component responsible for displaying GPS data from the DroneLink backend. It has three tabs:

1. **Navigation** — numeric readout (position, altitude, speed, bearing)
2. **Satellites** — per-satellite signal strength bars (the problematic one)
3. **Map** — live OpenStreetMap with drone marker

This document explains how satellite data flows from DroneLink through the binding layer to GPSWidget, with detailed troubleshooting for the "no data" problem.

---

## Data Flow: DroneLink → Python → GUI

### Step 1: DroneLink C++ backend (every ~1 second)

```cpp
// DroneLink.cpp:274-285
bool DroneLink::pollSatellitesMSP(DroneState& pending) {
	auto buf = sendMSP(MSP::GPS_SV_INFO);  // Request cmd 164

	std::vector<SVInfoEntry> sv;
	if (!GPSNeoM10::parseMspSvInfo(buf, sv))  // Parse response
		return false;

	pending.svList = std::move(sv);           // ← 32 SVInfoEntry objects
	pending.svInfoValid = true;               // ← Set validity flag
	pending.svSource = "MSP";                 // ← Identify data path
	return true;
}
```

**Output:** `pending.svList` contains a vector of `SVInfoEntry` objects.

### Step 2: State committed to shared memory

```cpp
// DroneLink.cpp:633-636
void DroneLink::commitState(const DroneState& s) {
	std::lock_guard<std::mutex> lock(dataMutex);
	currentState = s;  // ← svList is now in shared state
}
```

**Output:** `currentState.svList` is thread-safe for Python to read.

### Step 3: Python calls getLatestState()

```python
# DroneCockpitUI/MainWindow.py (or wherever you call update_gps)
state = drone_link.get_latest_state()

# state is a pybind11-wrapped DroneState object with these attributes:
#   state.sv_list → list of SVInfoEntry objects (wrapped from C++)
#   state.sv_info_valid → bool
#   state.sv_source → str ("MSP" or "UBX")
```

**Output:** Python gets a DroneState object with `sv_list` containing wrapped C++ objects.

### Step 4: Pass to GPSWidget.update_gps()

```python
# MainWindow.py
gps_widget.update_gps({
	"gps_sv_list": list(state.sv_list),      # ← Convert to Python list
	"gps_sv_info_valid": state.sv_info_valid,
	"gps_sv_source": state.sv_source,
	# ... other GPS fields ...
})
```

**Important:** Must convert C++ vector to Python list explicitly!

### Step 5: GPSWidget normalizes and displays

```python
# GPSWidget.py:_normalize_sv_list()
def _normalize_sv_list(raw_list) -> list:
	"""
	Accept SVInfoEntry C++ objects or dicts.
	Return list of dicts with keys: gnss_id, sv_id, cno, used, quality, elev, azim, status
	"""
	out = []
	for item in raw_list:
		try:
			if isinstance(item, dict):
				# Already a dict
				out.append({
					"gnss_id": item.get("gnss_name", item.get("gnss_id", "?")),
					"sv_id": item.get("svid", item.get("sv_id", 0)),
					"cno": int(item.get("cno", 0)),
					"used": bool(item.get("used", False)),
					"quality": item.get("quality", 0),
					"elev": int(item.get("elev", 0)),
					"azim": int(item.get("azim", 0)),
					"status": item.get("status_str", item.get("status", "idle")),
				})
			else:
				# Assume pybind11-wrapped SVInfoEntry object
				out.append({
					"gnss_id": item.gnss_name,     # ← Read C++ attribute
					"sv_id": item.svid,
					"cno": int(item.cno),
					"used": bool(item.used),
					"quality": item.quality,
					"elev": int(item.elev),
					"azim": int(item.azim),
					"status": item.status_str,
				})
		except Exception:
			continue  # Skip any malformed entry
	return out
```

**Output:** A list of plain Python dicts with normalized keys.

### Step 6: _SatCanvas renders the satellites

```python
# GPSWidget.py (inside _SatCanvas class)
def _refresh(self, sv_list):
	"""Draw satellite signal bars on canvas."""
	self._cv.delete("all")  # Clear canvas

	# Sort: used first, then tracked, then idle
	used = [sv for sv in sv_list if sv["used"]]
	tracked = [sv for sv in sv_list if sv["cno"] > 0 and not sv["used"]]
	idle = [sv for sv in sv_list if sv["cno"] == 0]

	ordered = sorted(used, key=lambda x: -x["cno"]) + \
			  sorted(tracked, key=lambda x: -x["cno"]) + \
			  sorted(idle, key=lambda x: (x["gnss_id"], x["sv_id"]))

	# Draw table header and rows
	y = 20
	for i, sv in enumerate(ordered):
		color = _C["sv_used"] if sv["used"] else (_C["sv_locked"] if sv["cno"] > 0 else _C["sv_nodata"])
		bar_width = int(sv["cno"] / 63.0 * 200)  # Scale 0-63 dBHz to bar width

		# Draw row
		self._cv.create_text(10, y, text=f'{sv["gnss_id"]} {sv["sv_id"]}', fill=color, ...)
		self._cv.create_rectangle(150, y-5, 150+bar_width, y+5, fill=color, ...)

		y += 25  # Next row
```

---

## GPSWidget Structure

### Class Hierarchy

```
GPSWidget (tk.Frame)
├── _NavigationFrame (tab)
│   ├── Position display (lat/lon)
│   ├── Altitude display
│   ├── Speed/course display
│   └── Home bearing/distance
│
├── _SatelliteFrame (tab)  ← The problematic tab
│   └── _SatCanvas (tk.Canvas with satellite bars)
│
└── _MapFrame (tab)
	├── Map canvas (OSM tiles)
	├── Zoom controls
	└── Pan/center buttons
```

### Key Methods

| Method | Purpose | Called When |
|--------|---------|-------------|
| `__init__()` | Initialize all three tabs | Widget created |
| `update_gps(ui_data)` | Update with new telemetry | Every 100 ms (from main loop) |
| `_normalize_sv_list(raw_list)` | Convert C++ objects to dicts | Before rendering satellites |
| `_SatCanvas._refresh(sv_list)` | Render satellite bars on canvas | On update or user interaction |

---

## Detailed Troubleshooting: "No Data to Show"

### Issue Description

- GPS position, altitude, satellite count are displayed ✓
- Satellite **count** shows "10 satellites" ✓
- But satellite **list** (bars) shows "No data to show" ✗

This indicates:
- `state.sv_list` has data (count is accurate)
- But `_normalize_sv_list()` is not receiving it, or failing to parse it

### Debug Step 1: Verify DroneLink is polling

**In DroneLink.cpp:274-285, add logging:**

```cpp
bool DroneLink::pollSatellitesMSP(DroneState& pending) {
	auto buf = sendMSP(MSP::GPS_SV_INFO);

	std::cerr << "[pollSatellitesMSP] Response buffer size: " << buf.size() << std::endl;

	std::vector<SVInfoEntry> sv;
	if (!GPSNeoM10::parseMspSvInfo(buf, sv)) {
		std::cerr << "[pollSatellitesMSP] Parse FAILED" << std::endl;
		return false;
	}

	std::cerr << "[pollSatellitesMSP] Parsed " << sv.size() << " satellites" << std::endl;

	pending.svList = std::move(sv);
	pending.svInfoValid = true;
	pending.svSource = "MSP";
	return true;
}
```

**Rebuild and run.** You should see output every ~1 second:
```
[pollSatellitesMSP] Response buffer size: 129
[pollSatellitesMSP] Parsed 32 satellites
[pollSatellitesMSP] Parsed 32 satellites
```

If you don't see output → satellite polling is never firing. Check:
- Is `svPollTickCounter_` being decremented? (DroneLink.cpp:239)
- Is 1 second passing? (100 iterations × 10ms)
- Is the connection still active?

### Debug Step 2: Verify Python is receiving state

**In your main loop (MainWindow.py or wherever you call update_gps):**

```python
state = drone_link.get_latest_state()

print(f"[DEBUG] state.sv_info_valid: {state.sv_info_valid}")
print(f"[DEBUG] state.sv_source: {state.sv_source}")
print(f"[DEBUG] len(state.sv_list): {len(state.sv_list)}")

if len(state.sv_list) > 0:
	sv0 = state.sv_list[0]
	print(f"[DEBUG] First satellite: {sv0.gnss_name} {sv0.svid}, CNO={sv0.cno}")
```

**Expected output (after ~2 seconds):**
```
[DEBUG] state.sv_info_valid: True
[DEBUG] state.sv_source: MSP
[DEBUG] len(state.sv_list): 32
[DEBUG] First satellite: GPS 1, CNO=45
```

If `sv_list` is empty → data is not reaching Python. Check:
- Did you rebuild the C++ project after the polling code?
- Is the DroneBackend.pyd file up-to-date?
- Is Python importing the rebuilt .pyd?

### Debug Step 3: Verify data is passed to GPSWidget

**In GPSWidget.update_gps(), add logging:**

```python
def update_gps(self, ui_data):
	print(f"[GPSWidget] update_gps called")
	print(f"[GPSWidget] gps_sv_list type: {type(ui_data.get('gps_sv_list'))}")
	print(f"[GPSWidget] gps_sv_list length: {len(ui_data.get('gps_sv_list', []))}")

	sv_list_raw = ui_data.get("gps_sv_list", [])
	sv_list = self._normalize_sv_list(sv_list_raw)

	print(f"[GPSWidget] After normalize: {len(sv_list)} satellites")

	# ... rest of the method ...
```

**Expected output:**
```
[GPSWidget] update_gps called
[GPSWidget] gps_sv_list type: <class 'list'>
[GPSWidget] gps_sv_list length: 10
[GPSWidget] After normalize: 10 satellites
```

If you see:
- `gps_sv_list type: <class 'NoneType'>` → You didn't pass `gps_sv_list` to update_gps()
- `gps_sv_list length: 0` → C++ list is empty
- `After normalize: 0` → The normalization function is failing

### Debug Step 4: Verify normalization is working

**In _normalize_sv_list(), add error tracking:**

```python
def _normalize_sv_list(raw_list) -> list:
	out = []
	for idx, item in enumerate(raw_list):
		try:
			if isinstance(item, dict):
				out.append({
					"gnss_id": item.get("gnss_name", item.get("gnss_id", "?")),
					"sv_id": item.get("svid", item.get("sv_id", 0)),
					"cno": int(item.get("cno", 0)),
					"used": bool(item.get("used", False)),
					"quality": item.get("quality", 0),
					"elev": int(item.get("elev", 0)),
					"azim": int(item.get("azim", 0)),
					"status": item.get("status_str", item.get("status", "idle")),
				})
			else:
				# Assume pybind11-wrapped SVInfoEntry object
				print(f"[normalize] Item {idx}: type={type(item)}, "
					  f"gnss_name={getattr(item, 'gnss_name', 'N/A')}, "
					  f"svid={getattr(item, 'svid', 'N/A')}")

				out.append({
					"gnss_id": item.gnss_name,
					"sv_id": item.svid,
					"cno": int(item.cno),
					"used": bool(item.used),
					"quality": item.quality,
					"elev": int(item.elev),
					"azim": int(item.azim),
					"status": item.status_str,
				})
		except Exception as e:
			print(f"[normalize] Item {idx} FAILED: {e}")
			import traceback
			traceback.print_exc()
			continue
	return out
```

**Expected output:**
```
[normalize] Item 0: type=<... SVInfoEntry ...>, gnss_name=GPS, svid=1
[normalize] Item 1: type=<... SVInfoEntry ...>, gnss_name=GPS, svid=4
...
```

If you see exceptions → The C++ object attributes are not accessible from Python. This means:
- The pybind11 binding is incomplete
- Check Bindings.cpp:62-89 (SVInfoEntry bindings)

### Debug Step 5: Verify canvas rendering

**In _SatCanvas._refresh(), add logging:**

```python
def _refresh(self, sv_list):
	print(f"[_SatCanvas._refresh] Rendering {len(sv_list)} satellites")

	self._cv.delete("all")

	used = [sv for sv in sv_list if sv["used"]]
	tracked = [sv for sv in sv_list if sv["cno"] > 0 and not sv["used"]]
	idle = [sv for sv in sv_list if sv["cno"] == 0]

	print(f"[_SatCanvas._refresh] Sorted: {len(used)} used, {len(tracked)} tracked, {len(idle)} idle")

	ordered = sorted(used, key=lambda x: -x["cno"]) + \
			  sorted(tracked, key=lambda x: -x["cno"]) + \
			  sorted(idle, key=lambda x: (x["gnss_id"], x["sv_id"]))

	# ... draw rows ...
```

**Expected output:**
```
[_SatCanvas._refresh] Rendering 10 satellites
[_SatCanvas._refresh] Sorted: 7 used, 3 tracked, 0 idle
```

If you see "Rendering 0" → Data is being lost before the canvas is called.

---

## Complete Debug Flow: From C++ to GUI

Run this diagnostic sequence:

### 1. C++ level (add these to DroneLink.cpp and rebuild)

```cpp
// In pollSatellitesMSP() — Line 274
std::cerr << "\n=== pollSatellitesMSP ===" << std::endl;
std::cerr << "buf.size()=" << buf.size() << std::endl;

// After parse
std::cerr << "sv.size()=" << sv.size() << std::endl;
if (sv.size() > 0) {
	std::cerr << "  sv[0]: gnssName=" << sv[0].gnssName 
			  << " svid=" << (int)sv[0].svid 
			  << " cno=" << (int)sv[0].cno << std::endl;
}

// After state update
std::cerr << "pending.svList.size()=" << pending.svList.size() << std::endl;
```

### 2. Python level (add these to your main loop)

```python
import time
time.sleep(2)  # Wait for first poll

state = drone_link.get_latest_state()
print("\n=== Python State ===")
print(f"sv_info_valid={state.sv_info_valid}")
print(f"sv_source={state.sv_source}")
print(f"len(sv_list)={len(state.sv_list)}")
for i, sv in enumerate(state.sv_list[:3]):
	print(f"  sv[{i}]: {sv.gnss_name} {sv.svid} CNO={sv.cno}")
```

### 3. Widget level (add to GPSWidget.update_gps)

```python
print("\n=== GPSWidget.update_gps ===")
sv_raw = ui_data.get("gps_sv_list", [])
print(f"Raw sv_list length: {len(sv_raw)}")
print(f"Raw sv_list type: {type(sv_raw)}")

sv_normalized = self._normalize_sv_list(sv_raw)
print(f"Normalized length: {len(sv_normalized)}")

self._sat_frame.update_satellites(sv_normalized)
```

### 4. Canvas level (add to _SatCanvas._refresh)

```python
print("\n=== _SatCanvas._refresh ===")
print(f"Input sv_list length: {len(sv_list)}")
# ... rest of rendering logic ...
print(f"Rendered {len(ordered)} satellites on canvas")
```

---

## Common Causes of "No Data"

| Symptom | Cause | Fix |
|---------|-------|-----|
| "No data" + empty sv_list | C++ not polling | Wait 2 seconds, check svPollTickCounter_ |
| "No data" + sv_count=10 shows | Data in C++, lost in Python | Check pybind11 bindings (Bindings.cpp) |
| Exception in _normalize_sv_list | C++ object attributes missing | Rebuild .pyd with updated Bindings.cpp |
| Renders empty rows | Canvas not called | Check update_gps() → _sat_frame.update() chain |
| Shows "Waiting for GPS fix" | No 3D fix yet | Get more satellites or clear sky view |

---

## Solution Checklist

- [ ] Rebuild DroneBackend C++ project (Visual Studio)
- [ ] Copy new DroneBackend.pyd to DroneCockpitUI folder
- [ ] Verify state.sv_list has data (print in Python)
- [ ] Verify state.sv_list is passed to update_gps()
- [ ] Verify _normalize_sv_list receives the list
- [ ] Verify _SatCanvas._refresh is called with data
- [ ] Check that GPS has 3D fix (position shows valid)
- [ ] Wait 2-3 seconds after connect for first poll
- [ ] Check that MSP_GPS_SV_INFO (cmd 164) responses are ~129 bytes

---

## Summary

The satellite display requires:

1. **DroneLink polls MSP cmd 164** every ~1 second → populates `pending.svList`
2. **DroneLink commits state** → `currentState.svList` is updated atomically
3. **Python calls getLatestState()** → receives `state.sv_list` (wrapped C++ vector)
4. **Main loop passes to GPSWidget** → `update_gps({"gps_sv_list": state.sv_list, ...})`
5. **GPSWidget normalizes** → converts C++ objects to Python dicts
6. **_SatCanvas renders** → draws signal bars

If any step fails, "no data" appears. Use the debug logging above to identify which step is broken.
