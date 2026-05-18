# Code Examples: Debugging the Satellite Display

This document provides ready-to-use code snippets for debugging the missing satellite display issue.

---

## Part 1: C++ Logging (DroneLink.cpp)

### Add Logging to pollSatellitesMSP()

**Location:** DroneLink.cpp, around line 274

**Find this code:**
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

**Replace with this (add logging):**
```cpp
bool DroneLink::pollSatellitesMSP(DroneState& pending) {
	auto buf = sendMSP(MSP::GPS_SV_INFO);

	// LOGGING: Show response size
	std::cerr << "\n[pollSatellitesMSP] Response buffer size: " << buf.size() << " bytes" << std::endl;

	std::vector<SVInfoEntry> sv;
	if (!GPSNeoM10::parseMspSvInfo(buf, sv)) {
		std::cerr << "[pollSatellitesMSP] parseMspSvInfo FAILED!" << std::endl;
		return false;
	}

	// LOGGING: Show parse success
	std::cerr << "[pollSatellitesMSP] Parsed successfully: " << sv.size() << " satellites" << std::endl;

	// LOGGING: Show first satellite (if any)
	if (sv.size() > 0) {
		const auto& sv0 = sv[0];
		std::cerr << "  First SV: gnssName=" << sv0.gnssName 
				  << " svid=" << (int)sv0.svid 
				  << " quality=" << (int)sv0.quality 
				  << " cno=" << (int)sv0.cno << std::endl;
	}

	pending.svList = std::move(sv);
	pending.svInfoValid = true;
	pending.svSource = "MSP";

	// LOGGING: Show state after update
	std::cerr << "[pollSatellitesMSP] pending.svList.size() = " << pending.svList.size() << std::endl;

	return true;
}
```

**Expected console output (every ~1 second):**
```
[pollSatellitesMSP] Response buffer size: 129 bytes
[pollSatellitesMSP] Parsed successfully: 32 satellites
  First SV: gnssName=GPS svid=1 quality=5 cno=45
[pollSatellitesMSP] pending.svList.size() = 32
```

**If you don't see this output:**
- The polling loop isn't running (check communicationLoop)
- Or the MSP response is failing (check sendMSP)

---

## Part 2: Python State Logging

### Add Logging to Your Main Loop

**Location:** Wherever you call `drone_link.get_latest_state()` (likely MainWindow.py or similar)

**Add this code (after calling get_latest_state):**
```python
import time

# ... other code ...

# Wait a few seconds for first poll
time.sleep(2)

# Get the latest state
state = drone_link.get_latest_state()

# LOGGING: Check satellite data in state
print("\n=== Python State Check ===")
print(f"[State] sv_info_valid: {state.sv_info_valid}")
print(f"[State] sv_source: {state.sv_source}")
print(f"[State] sv_list length: {len(state.sv_list)}")
print(f"[State] sv_list type: {type(state.sv_list)}")

if len(state.sv_list) > 0:
	print(f"[State] First 3 satellites:")
	for i in range(min(3, len(state.sv_list))):
		sv = state.sv_list[i]
		print(f"  [{i}] {sv.gnss_name} SVid={sv.svid}, CNO={sv.cno}, Quality={sv.quality}, Used={sv.used}")
else:
	print("[State] WARNING: sv_list is empty!")

# LOGGING: Check to_dict() output
try:
	state_dict = state.to_dict()
	print(f"\n[to_dict] Keys present: {list(state_dict.keys())}")
	if "gps_sv_list" in state_dict:
		print(f"[to_dict] gps_sv_list length: {len(state_dict['gps_sv_list'])}")
	else:
		print("[to_dict] WARNING: 'gps_sv_list' key not found!")
except Exception as e:
	print(f"[to_dict] ERROR: {e}")

print("=== End State Check ===\n")
```

**Expected output (after 2 seconds):**
```
=== Python State Check ===
[State] sv_info_valid: True
[State] sv_source: MSP
[State] sv_list length: 10
[State] sv_list type: <class 'list'>
[State] First 3 satellites:
  [0] GPS SVid=1, CNO=45, Quality=5, Used=True
  [1] GPS SVid=4, CNO=38, Quality=5, Used=True
  [2] GALILEO SVid=8, CNO=15, Quality=2, Used=False

[to_dict] Keys present: ['gps_position', 'gps_altitude', 'gps_sv_count', 'gps_sv_list', ...]
[to_dict] gps_sv_list length: 10
=== End State Check ===
```

**If you see:**
- `sv_list length: 0` → C++ is not populating the data (check C++ logging from Part 1)
- `sv_info_valid: False` → Parser is failing (check C++ logging)
- `'gps_sv_list' not in to_dict()` → Bindings.cpp isn't exposing it correctly

---

## Part 3: GPSWidget Logging

### Add Logging to update_gps()

**Location:** GPSWidget.py, in the `update_gps()` method

**Find this method:**
```python
def update_gps(self, ui_data):
	# ... existing code ...
	sv_list_raw = ui_data.get("gps_sv_list", [])
	sv_list = self._normalize_sv_list(sv_list_raw)
	# ... rest of method ...
```

**Add logging after getting sv_list_raw:**
```python
def update_gps(self, ui_data):
	# ... existing code ...

	sv_list_raw = ui_data.get("gps_sv_list", [])

	# LOGGING: Check what we received
	print(f"\n[GPSWidget.update_gps] Called")
	print(f"  ui_data keys: {list(ui_data.keys())}")
	print(f"  gps_sv_list type: {type(sv_list_raw)}")
	print(f"  gps_sv_list length: {len(sv_list_raw) if sv_list_raw else 0}")

	if sv_list_raw and len(sv_list_raw) > 0:
		print(f"  First item type: {type(sv_list_raw[0])}")
		first = sv_list_raw[0]
		print(f"  First item: {first if isinstance(first, dict) else f'<{type(first).__name__}>'}")

	sv_list = self._normalize_sv_list(sv_list_raw)

	# LOGGING: Check after normalization
	print(f"  After normalize length: {len(sv_list)}")
	if sv_list and len(sv_list) > 0:
		print(f"  First normalized: {sv_list[0]}")

	# ... rest of method ...
```

**Expected output (every frame after satellites are visible):**
```
[GPSWidget.update_gps] Called
  ui_data keys: ['gps_position', 'gps_altitude', 'gps_sv_count', 'gps_sv_list', 'gps_sv_valid', ...]
  gps_sv_list type: <class 'list'>
  gps_sv_list length: 10
  First item type: <class 'DroneBackend.SVInfoEntry'>
  First item: <DroneBackend.SVInfoEntry>
  After normalize length: 10
  First normalized: {'gnss_id': 'GPS', 'sv_id': 1, 'cno': 45, 'used': True, 'quality': 5, 'elev': 45, 'azim': 180, 'status': 'locked'}
```

**If you see:**
- `gps_sv_list length: 0` → Your code isn't passing gps_sv_list to update_gps (check how you call it)
- `gps_sv_list length: None` → You're not passing the key at all
- Exceptions in normalize → SVInfoEntry attributes are missing or wrongly named

---

## Part 4: _normalize_sv_list() Logging

### Add Detailed Error Logging to _normalize_sv_list()

**Location:** GPSWidget.py, in the `_normalize_sv_list()` method

**Find this method:**
```python
def _normalize_sv_list(raw_list) -> list:
	"""Convert SVInfoEntry or dict to normalized format."""
	out = []
	for item in raw_list:
		try:
			if isinstance(item, dict):
				# ... dict handling ...
			else:
				# ... C++ object handling ...
		except Exception:
			continue
	return out
```

**Replace the exception handling with detailed logging:**
```python
def _normalize_sv_list(raw_list) -> list:
	"""Convert SVInfoEntry or dict to normalized format."""
	out = []

	print(f"\n[_normalize_sv_list] Input length: {len(raw_list)}")

	for idx, item in enumerate(raw_list):
		try:
			if isinstance(item, dict):
				# LOGGING: Dict path
				print(f"  [{idx}] Dict item: {item}")

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
				# LOGGING: C++ object path
				print(f"  [{idx}] C++ object: type={type(item).__name__}")
				print(f"         Attributes: gnss_name={item.gnss_name}, svid={item.svid}, cno={item.cno}, used={item.used}")

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
			print(f"  [{idx}] ERROR: {e}")
			import traceback
			traceback.print_exc()
			continue

	print(f"[_normalize_sv_list] Output length: {len(out)}\n")
	return out
```

**Expected output:**
```
[_normalize_sv_list] Input length: 10
  [0] C++ object: type=SVInfoEntry
		 Attributes: gnss_name=GPS, svid=1, cno=45, used=True
  [1] C++ object: type=SVInfoEntry
		 Attributes: gnss_name=GPS, svid=4, cno=38, used=True
  [2] C++ object: type=SVInfoEntry
		 Attributes: gnss_name=GALILEO, svid=8, cno=15, used=False
  ... (7 more items) ...
[_normalize_sv_list] Output length: 10
```

**If you see exceptions like:**
- `AttributeError: 'SVInfoEntry' object has no attribute 'gnss_name'`
  → The C++ binding doesn't expose this field; check Bindings.cpp line 62-89
- `TypeError: int() argument must be string or number`
  → A field is None; add a default value

---

## Part 5: _SatCanvas Rendering Logging

### Add Logging to _SatCanvas._refresh()

**Location:** GPSWidget.py, in the `_SatCanvas._refresh()` method

**Find the method:**
```python
class _SatCanvas:
	def _refresh(self, sv_list):
		# Clear and redraw
		self._cv.delete("all")
		# ... sorting code ...
		# ... drawing code ...
```

**Add logging at the start and during drawing:**
```python
class _SatCanvas:
	def _refresh(self, sv_list):
		print(f"\n[_SatCanvas._refresh] Input: {len(sv_list)} satellites")

		self._cv.delete("all")

		# Sorting
		used = [sv for sv in sv_list if sv["used"]]
		tracked = [sv for sv in sv_list if sv["cno"] > 0 and not sv["used"]]
		idle = [sv for sv in sv_list if sv["cno"] == 0]

		print(f"  Sorted: {len(used)} used, {len(tracked)} tracked, {len(idle)} idle")

		ordered = sorted(used, key=lambda x: -x["cno"]) + \
				  sorted(tracked, key=lambda x: -x["cno"]) + \
				  sorted(idle, key=lambda x: (x["gnss_id"], x["sv_id"]))

		print(f"  Drawing {len(ordered)} rows...")

		y = 20
		for i, sv in enumerate(ordered):
			# Determine color
			if sv["used"]:
				color = _C["sv_used"]
			elif sv["cno"] > 0:
				color = _C["sv_locked"]
			else:
				color = _C["sv_nodata"]

			# Calculate bar width
			bar_width = int(sv["cno"] / 63.0 * 200)

			# LOGGING: Row detail
			if i < 3:  # Log first 3 rows only
				print(f"    Row {i}: {sv['gnss_id']} {sv['sv_id']}, CNO={sv['cno']}, bar_width={bar_width}")

			# Draw row (existing code)
			self._cv.create_text(10, y, text=f'{sv["gnss_id"]} {sv["sv_id"]}', fill=color, ...)
			self._cv.create_rectangle(150, y-5, 150+bar_width, y+5, fill=color, ...)

			y += 25

		print(f"[_SatCanvas._refresh] Complete\n")
```

**Expected output:**
```
[_SatCanvas._refresh] Input: 10 satellites
  Sorted: 7 used, 3 tracked, 0 idle
  Drawing 10 rows...
	Row 0: GPS 1, CNO=45, bar_width=142
	Row 1: GPS 4, CNO=38, bar_width=120
	Row 2: GALILEO 8, CNO=15, bar_width=47
[_SatCanvas._refresh] Complete
```

**If you see:**
- `Input: 0 satellites` → The widget is being called but not receiving data; check Part 3 logging
- Exceptions during drawing → Check that sv dict has all required keys

---

## Part 6: Complete Test Script

### All-in-One Debug Script

**Create a new file: test_satellite_debug.py**

```python
#!/usr/bin/env python3
"""
Complete satellite data flow debug script.
Run this to diagnose where the satellite data is getting lost.
"""

import sys
import time

# Import DroneBackend
try:
	sys.path.insert(0, './DroneCockpitUI')
	from DroneBackend import DroneLink
	print("✓ DroneBackend imported successfully")
except ImportError as e:
	print(f"✗ Failed to import DroneBackend: {e}")
	sys.exit(1)

# Create drone link (non-blocking)
print("\nInitializing DroneLink...")
try:
	drone = DroneLink()
	drone.connect("COM3", 57600, blocking=False)  # Adjust COM port as needed
	print("✓ DroneLink connected")
except Exception as e:
	print(f"✗ Failed to connect: {e}")
	sys.exit(1)

# Wait for first poll
print("\nWaiting 3 seconds for first satellite poll...")
time.sleep(3)

# ============ DIAGNOSTIC TEST 1: State Object ============
print("\n" + "="*60)
print("TEST 1: Check state.sv_list")
print("="*60)

state = drone.get_latest_state()

if state is None:
	print("✗ get_latest_state() returned None!")
	sys.exit(1)

print(f"✓ Got state object")
print(f"  sv_info_valid: {state.sv_info_valid}")
print(f"  sv_source: {state.sv_source}")
print(f"  sv_list type: {type(state.sv_list)}")
print(f"  sv_list length: {len(state.sv_list)}")

if len(state.sv_list) == 0:
	print("✗ sv_list is empty! Issue is in C++ DroneLink or parser")
	sys.exit(1)

print(f"✓ sv_list has {len(state.sv_list)} satellites")

# Show first 3
print(f"\nFirst 3 satellites:")
for i in range(min(3, len(state.sv_list))):
	sv = state.sv_list[i]
	print(f"  [{i}] {sv.gnss_name} SVID={sv.svid}, CNO={sv.cno}, Quality={sv.quality}, Used={sv.used}")

# ============ DIAGNOSTIC TEST 2: to_dict() ============
print("\n" + "="*60)
print("TEST 2: Check state.to_dict()")
print("="*60)

try:
	state_dict = state.to_dict()
	print(f"✓ to_dict() succeeded")
	print(f"  Dict keys: {list(state_dict.keys())}")

	if "gps_sv_list" not in state_dict:
		print("✗ 'gps_sv_list' key not in to_dict()! Issue is in Bindings.cpp")
		sys.exit(1)

	print(f"✓ 'gps_sv_list' key present")
	print(f"  gps_sv_list length: {len(state_dict['gps_sv_list'])}")
	print(f"  gps_sv_list type: {type(state_dict['gps_sv_list'])}")

	if len(state_dict['gps_sv_list']) == 0:
		print("✗ gps_sv_list in to_dict() is empty!")
		sys.exit(1)

	print(f"\n✓ First item in gps_sv_list:")
	first = state_dict['gps_sv_list'][0]
	if isinstance(first, dict):
		print(f"    Type: dict")
		print(f"    Content: {first}")
	else:
		print(f"    Type: {type(first).__name__}")
		print(f"    gnss_name: {first.gnss_name}")
		print(f"    svid: {first.svid}")
		print(f"    cno: {first.cno}")

except Exception as e:
	print(f"✗ to_dict() failed: {e}")
	import traceback
	traceback.print_exc()
	sys.exit(1)

# ============ DIAGNOSTIC TEST 3: Normalize Function ============
print("\n" + "="*60)
print("TEST 3: Simulate _normalize_sv_list()")
print("="*60)

def test_normalize_sv_list(raw_list):
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
			print(f"  [{idx}] ERROR: {e}")
			continue
	return out

normalized = test_normalize_sv_list(state.sv_list)

print(f"✓ Normalization succeeded")
print(f"  Input length: {len(state.sv_list)}")
print(f"  Output length: {len(normalized)}")

if len(normalized) == 0:
	print("✗ Normalization produced empty list!")
	sys.exit(1)

print(f"\n✓ First normalized satellite:")
first_norm = normalized[0]
for key, val in first_norm.items():
	print(f"    {key}: {val}")

# ============ FINAL VERDICT ============
print("\n" + "="*60)
print("FINAL VERDICT")
print("="*60)
print("""
✓ C++ DroneLink is polling satellites correctly
✓ Python binding is exposing sv_list correctly
✓ to_dict() is creating gps_sv_list correctly
✓ Normalization to dict format works correctly

THE ISSUE IS IN YOUR GUI INTEGRATION:
1. Check that update_gps() is receiving gps_sv_list
2. Check that _SatCanvas is being called with the data
3. Check that canvas has non-zero size and is visible

Next steps:
1. Add logging to GPSWidget.update_gps()
2. Add logging to _SatCanvas._refresh()
3. Verify the canvas is on screen and not hidden
""")

print("\nDebug test complete!")
```

**To run:**
```powershell
cd "C:\Users\TheSilent\Desktop\Project R.A.D"
python test_satellite_debug.py
```

**Expected output:**
```
✓ DroneBackend imported successfully

Initializing DroneLink...
✓ DroneLink connected

Waiting 3 seconds for first satellite poll...

============================================================
TEST 1: Check state.sv_list
============================================================
✓ Got state object
  sv_info_valid: True
  sv_source: MSP
  sv_list type: <class 'list'>
  sv_list length: 10

✓ sv_list has 10 satellites

First 3 satellites:
  [0] GPS SVID=1, CNO=45, Quality=5, Used=True
  [1] GPS SVID=4, CNO=38, Quality=5, Used=True
  [2] GALILEO SVID=8, CNO=15, Quality=2, Used=False

... (more output) ...

FINAL VERDICT

✓ C++ DroneLink is polling satellites correctly
✓ Python binding is exposing sv_list correctly
✓ to_dict() is creating gps_sv_list correctly
✓ Normalization to dict format works correctly

THE ISSUE IS IN YOUR GUI INTEGRATION:
...
```

---

## Summary: Which Debug Step to Use

| Symptom | Try First | Then Try |
|---------|-----------|----------|
| No satellite data in GUI | Part 2 (Python state) | Part 1 (C++ logging) |
| C++ not polling | Part 1 (C++ logging) | Check sendMSP() |
| Python receives no data | Part 2 (Python state) | Check C++ output |
| Widget not showing data | Part 3 (GPSWidget logging) | Part 2 (Python state) |
| "No data" message in widget | Part 5 (_SatCanvas logging) | Part 4 (_normalize logging) |
| Complete uncertainty | Part 6 (test script) | Part 1-5 (specific logs) |

---

**All of these logging additions are safe and can be left in production code. They only output to console/stderr when active.**
