"""
safety.py -- pure safety rules for laptop control (no Tk, no backend)

Used by UplinkController (gates) and SafetyMonitor (alarms). Everything
here is a plain function of telemetry values so it is unit-tested without
hardware (tests/unit/test_safety.py).

Limits (change here, documented in command-uplink-uputstvo.md, chapter 7).
They are set for the R.A.D. search-and-rescue envelope (radio range about
5 km, flights up to 5 km high) so that they only act at the edge of it:
  ALT_CEILING_M     altitude hold / laptop climb never targets more than this
                    above the arming point.
  GEOFENCE_M        beyond this distance from home the laptop may only move
                    the drone TOWARD home (yaw always allowed): the ELRS
                    receiver's range.
  RANGE_WARN_M      warning when the drone gets this far (80 % of the range).
  MIN_LQ_PCT        ELRS uplink quality below this for LQ_LOW_S stops laptop
                    movement (long range runs at low LQ normally, so only a
                    sustained drop counts).
  VIDEO_STOP_S      video lost this long -> laptop movement stops.
  VIDEO_LAND_S      video lost this long while the laptop flies -> auto-land.
  FS_MIN_LAND_US    without a learnt hover throttle: an AUTO-LAND failsafe
                    below this certainly falls.
  LAND_*            laptop auto-land descent profile.
"""
from __future__ import annotations

import math
from typing import Optional

ALT_CEILING_M = 5000.0
GEOFENCE_M = 5000.0
RANGE_WARN_M = 4000.0
GEOFENCE_TOWARD_COS = 0.5          # motion within +-60 deg of the home bearing
MIN_LQ_PCT = 30
LQ_LOW_S = 1.0
VIDEO_LOSS_S = 1.0                 # frame age that counts as "no video" (VideoLink)
VIDEO_STOP_S = 0.5
VIDEO_LAND_S = 5.0
FS_MIN_LAND_US = 1100
FS_MAX_DELAY_DS = 20               # stage-1 guard time > 2.0 s: warning
FS_CLIMB_VZ = -0.2                 # model: failsafe throttle descends slower -> unsafe
FS_FALL_VZ = -5.0                  # model: descends faster than this -> unsafe

LAND_FAST_MPS = 1.5                # descent speed high up
LAND_SLOW_MPS = 0.7                # ... in the last LAND_SLOW_ALT_M above the arming point
LAND_SLOW_ALT_M = 20.0
LAND_TOUCH_VZ = 0.25               # |vz| below this ...
LAND_TOUCH_S = 2.0                 # ... for this long with throttle at minimum = landed

PROCEDURES = {0: "AUTO-LAND", 1: "DROP", 2: "GPS-RESCUE"}


def assess_failsafe(fs: Optional[dict], model=None, volt: float = 0.0,
                    gps_fix: Optional[bool] = None, armed: bool = False) -> tuple:
    """Flight controller failsafe settings (MSP_FAILSAFE_CONFIG, read over
    USB) -> (level, text). level: "ok" / "info" / "warn" / "unsafe" / "unknown".

    fs keys: procedure (0/1/2), throttle (us), delay (0.1 s), landing_time.
    model: control.landing_model.LandingModel (learnt hover) or None,
    volt: pack voltage now, gps_fix: GPS 3D fix now (None = unknown).
    """
    if not fs:
        return "unknown", "FC failsafe not read yet (connect USB once before flight)"
    proc = int(fs.get("procedure", -1))
    name = PROCEDURES.get(proc, f"procedure {proc}")
    thr = int(fs.get("throttle", 0))
    delay = int(fs.get("delay", 0))
    if proc == 1:
        return "unsafe", "failsafe DROP: motors stop on link loss, the drone falls"
    if proc not in PROCEDURES:
        return "warn", f"unknown failsafe {name}"
    if proc == 2 and gps_fix is False:
        return ("unsafe" if armed else "warn",
                "failsafe GPS-RESCUE but no GPS fix: the FC cannot return -- "
                "use AUTO-LAND while GPS is off")
    if proc == 0:
        vz = model.vz_at(thr, volt) if (model is not None and volt) else None
        if vz is None:
            if thr < FS_MIN_LAND_US:
                return "unsafe", (f"failsafe AUTO-LAND with failsafe_throttle {thr} us: "
                                  "too low, the drone falls instead of descending")
            return "info", (f"AUTO-LAND at {thr} us, not yet checked against the "
                            "hover throttle (hover steadily to learn it)")
        if vz > FS_CLIMB_VZ:
            return "unsafe", (f"failsafe_throttle {thr} us does not descend ({vz:+.1f} m/s "
                              f"at {volt:.1f} V): the drone would not come down")
        if vz < FS_FALL_VZ:
            return "unsafe", (f"failsafe_throttle {thr} us descends {-vz:.1f} m/s "
                              f"at {volt:.1f} V: the drone falls")
        text = f"AUTO-LAND {thr} us: {-vz:.1f} m/s down now"
        if delay > FS_MAX_DELAY_DS:
            return "warn", text + f", guard time {delay / 10:.1f} s (long)"
        return "ok", text
    if delay > FS_MAX_DELAY_DS:
        return "warn", f"{name}, guard time {delay / 10:.1f} s (long)"
    return "ok", name


def failsafe_cover_m(fs: Optional[dict], model=None, volt: float = 0.0) -> Optional[float]:
    """Height an AUTO-LAND failsafe can descend before Betaflight disarms
    (descent speed * failsafe_landing_time). None if not AUTO-LAND or unknown."""
    if not fs or int(fs.get("procedure", -1)) != 0 or model is None or not volt:
        return None
    vz = model.vz_at(int(fs.get("throttle", 0)), volt)
    if vz is None or vz >= 0:
        return None
    return -vz * max(0, int(fs.get("landing_time", 0)))


def land_rate(alt_above_arming_m: Optional[float]) -> float:
    """Laptop auto-land descent speed (m/s, positive). Terrain is unknown,
    so the slow phase starts LAND_SLOW_ALT_M above the arming point."""
    if alt_above_arming_m is None or alt_above_arming_m <= LAND_SLOW_ALT_M:
        return LAND_SLOW_MPS
    return LAND_FAST_MPS


def geofence_filter(fwd: float, right: float, heading_deg: float,
                    dist_m: Optional[float], bearing_home_deg: Optional[float],
                    fence_m: float = GEOFENCE_M) -> tuple:
    """Movement intents (body frame, -1..1) -> (fwd, right, note).

    Inside the fence nothing changes. Outside it, a movement survives only
    if its direction over the ground is within +-60 deg of the bearing to
    home; anything else (outward, sideways) is cancelled. Yaw is not
    touched, so the operator can always turn the drone toward home.
    """
    if dist_m is None or bearing_home_deg is None or dist_m <= fence_m:
        return fwd, right, ""
    note = f"outside geofence ({dist_m:.0f} m > {fence_m:.0f} m): only toward home"
    if fwd == 0.0 and right == 0.0:
        return fwd, right, note
    move_deg = heading_deg + math.degrees(math.atan2(right, fwd))
    c = math.cos(math.radians(move_deg - bearing_home_deg))
    if c >= GEOFENCE_TOWARD_COS:
        return fwd, right, note
    return 0.0, 0.0, note + " (movement away blocked)"


def clamp_altitude_target(target_m: float, ceiling_m: float = ALT_CEILING_M) -> float:
    return min(float(target_m), ceiling_m)
