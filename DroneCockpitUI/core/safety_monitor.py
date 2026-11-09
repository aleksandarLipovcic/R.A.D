"""
safety_monitor.py -- one place that decides what the operator must be warned about

The cockpit calls SafetyMonitor.evaluate() 10x per second from the Tk thread
with a plain dict of facts (video, radio, USB, battery, laptop control,
flight controller failsafe, app health). It returns the active alerts,
most severe first, and on every change:

  * writes a line to logs/safety_YYYYMMDD.log (flight record of what was
    wrong and when -- also after a crash, the file is flushed per line);
  * plays a sound: CRITICAL alerts repeat every REPEAT_S while active.

No Tk, no backend: unit-tested in tests/unit/test_safety.py. The sound
callback is injected (winsound on Windows, Tk bell elsewhere).

Severity
  CRITICAL  something the operator must act on now (video or link lost
            while flying, battery critical, laptop control impossible,
            controller error, FC failsafe would drop the drone)
  WARNING   degraded but flying is still controlled
  INFO      state the operator should know (pilot override, geofence)
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

CRITICAL, WARNING, INFO = 3, 2, 1
LEVEL_NAME = {CRITICAL: "CRITICAL", WARNING: "WARNING", INFO: "INFO"}
REPEAT_S = 10.0
try:                                  # the same limits the controller uses
    from control.safety import VIDEO_LAND_S as LAND_AFTER_S, RANGE_WARN_M, VIDEO_STOP_S
except ImportError:                   # pragma: no cover
    LAND_AFTER_S, RANGE_WARN_M, VIDEO_STOP_S = 5.0, 4000.0, 0.5


def armed_or(up: dict) -> bool:
    return bool(up.get("armed")) or bool(up.get("bench"))


@dataclass(frozen=True)
class Alert:
    level: int
    key: str
    text: str


def evaluate_facts(f: dict) -> List[Alert]:
    """Facts -> alerts. Pure function (see module doc for the keys)."""
    a: List[Alert] = []
    flying = bool(f.get("armed"))
    laptop = bool(f.get("laptop_enabled"))

    # ── video ──────────────────────────────────────────────────────────────
    vs = f.get("video_state", "")
    if f.get("video_ever_live"):
        if vs in ("SignalLost", "Disconnected", "Searching"):
            a.append(Alert(CRITICAL if (flying or laptop) else WARNING, "video",
                           "FPV VIDEO LOST" + (" — laptop movement stopped" if laptop else "")))
        elif vs == "NoVideoInput":
            a.append(Alert(CRITICAL if (flying or laptop) else WARNING, "video",
                           "NO VIDEO SIGNAL from the receiver (blank picture)"))
        elif f.get("video_age_ms", 0) > 1000:
            a.append(Alert(WARNING, "video", f"video frozen {f['video_age_ms'] / 1000:.1f} s"))

    # ── drone telemetry / radio link ─────────────────────────────────────────
    rs = f.get("radio_status", "NO_RADIO")
    if f.get("radio_ever_live"):
        if rs == "TELEMETRY_LOST":
            a.append(Alert(CRITICAL, "radio", "DRONE LINK LOST — Betaflight failsafe takes over"))
        elif rs == "NO_RADIO":
            a.append(Alert(CRITICAL if laptop else WARNING, "radio",
                           "Pocket disconnected from the laptop"))
        elif rs == "DEGRADED":
            a.append(Alert(WARNING, "radio", f"radio link degraded (LQ {f.get('lq', '?')} %)"))

    # ── battery ──────────────────────────────────────────────────────────────
    batt = str(f.get("battery_state", ""))
    if "CRITICAL" in batt:
        a.append(Alert(CRITICAL, "battery", "BATTERY CRITICAL — land now"))
    elif "WARNING" in batt:
        a.append(Alert(WARNING, "battery", "battery low — return and land"))

    # ── laptop control ───────────────────────────────────────────────────────
    up = f.get("uplink") or {}
    if laptop:
        if up.get("state") == "NO_BRIDGE":
            a.append(Alert(CRITICAL, "bridge", "Pocket Lua bridge not responding"))
        if up.get("controller_errors_new"):
            a.append(Alert(CRITICAL, "controller", "laptop controller error (see console/log)"))
        for b in up.get("move_blockers", []):
            if b.startswith("pilot override"):
                continue
            a.append(Alert(WARNING, "move:" + b.split(" ")[0], f"movement stopped: {b}"))
        if up.get("pilot_override"):
            a.append(Alert(INFO, "override", "pilot override — laptop offsets off"))
        if up.get("geofence"):
            a.append(Alert(WARNING, "geofence", up["geofence"]))
        if (up.get("hold_engaged") and up.get("altitude_m") is not None
                and up["altitude_m"] > up.get("alt_ceiling_m", 1e9) - 50):
            a.append(Alert(WARNING, "ceiling", f"near altitude ceiling {up['alt_ceiling_m']:.0f} m"))
        # landing
        if up.get("landed"):
            a.append(Alert(CRITICAL, "landing", "LANDED — pilot: throttle down and disarm"))
        elif up.get("landing"):
            a.append(Alert(CRITICAL, "landing",
                           f"AUTO-LANDING ({up.get('land_reason', '')}) "
                           f"{up.get('land_rate_mps', 0):.1f} m/s — L or pilot switch to abort"))
        elif up.get("land_fail"):
            a.append(Alert(CRITICAL, "landing",
                           f"CANNOT AUTO-LAND: {up['land_fail']} — pilot must land"))
        vl = float(up.get("video_lost_s", 0) or 0)
        if vl >= VIDEO_STOP_S and not up.get("landing") and up.get("gate_on") and armed_or(up):
            left = max(0.0, LAND_AFTER_S - vl)
            a.append(Alert(CRITICAL, "video_land", f"no video: auto-land in {left:.0f} s"))
    rng = up.get("range_m") if up else None
    if rng is not None and rng >= RANGE_WARN_M:
        a.append(Alert(WARNING, "range", f"{rng / 1000:.1f} km from home — radio range limit"))
    cover = up.get("fc_failsafe_cover_m") if up else None
    alt = up.get("altitude_m") if up else None
    if flying and cover is not None and alt is not None and alt > cover:
        a.append(Alert(WARNING, "fs_cover",
                       f"{alt:.0f} m up: FC AUTO-LAND failsafe covers only {cover:.0f} m "
                       "(disarms in the air on link loss)"))

    # ── flight controller failsafe configuration ─────────────────────────────
    fs_level, fs_text = f.get("fc_failsafe", ("unknown", ""))
    if fs_level == "unsafe":
        a.append(Alert(CRITICAL, "failsafe", f"FC FAILSAFE UNSAFE: {fs_text}"))
    elif fs_level == "warn":
        a.append(Alert(WARNING, "failsafe", f"FC failsafe: {fs_text}"))
    elif fs_level in ("unknown", "info") and laptop:
        a.append(Alert(INFO, "failsafe", fs_text))

    # ── application health ───────────────────────────────────────────────────
    if f.get("ui_errors_new"):
        a.append(Alert(WARNING, "ui", "UI error (logged in logs/crash_*.log)"))

    a.sort(key=lambda x: -x.level)
    return a


class SafetyMonitor:
    def __init__(self, log_dir: str, sound: Optional[Callable[[int], None]] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._log_dir = log_dir
        self._sound = sound
        self._clock = clock
        self._active: Dict[str, Alert] = {}
        self._acked: Dict[str, int] = {}   # key -> level acknowledged
        self._last_sound = -1e9
        self._log = None

    # ── master warning / caution acknowledge ────────────────────────────────
    def acknowledge(self) -> None:
        """Operator pressed MASTER WARN/CAUT: the current alerts stop
        flashing and the CRITICAL sound stops repeating. A new alert (or an
        alert that gets more severe) is unacknowledged again."""
        self._acked = {k: a.level for k, a in self._active.items()}
        self._write("* acknowledged: " + ", ".join(a.text for a in self._active.values()))

    def has_unacknowledged(self, min_level: int = WARNING) -> bool:
        return any(a.level >= min_level and self._acked.get(k, 0) < a.level
                   for k, a in self._active.items())

    def evaluate(self, facts: dict) -> List[Alert]:
        alerts = evaluate_facts(facts)
        now = self._clock()
        current = {x.key: x for x in alerts}
        new = [x for k, x in current.items()
               if k not in self._active or self._active[k].level < x.level]
        gone = [x for k, x in self._active.items() if k not in current]
        for x in new:
            self._write(f"+ {LEVEL_NAME[x.level]:8s} {x.text}")
        for x in gone:
            self._write(f"- cleared  {x.text}")
        self._active = current
        self._acked = {k: lv for k, lv in self._acked.items() if k in current}

        top = alerts[0].level if alerts else 0
        if self._sound is not None:
            if any(x.level >= WARNING for x in new):
                self._play(max(x.level for x in new), now)
            elif (top == CRITICAL and now - self._last_sound >= REPEAT_S
                  and self.has_unacknowledged(CRITICAL)):
                self._play(CRITICAL, now)
        return alerts

    def _play(self, level: int, now: float) -> None:
        self._last_sound = now
        try:
            self._sound(level)
        except Exception:
            pass

    def _write(self, line: str) -> None:
        try:
            if self._log is None:
                os.makedirs(self._log_dir, exist_ok=True)
                name = time.strftime("safety_%Y%m%d.log")
                self._log = open(os.path.join(self._log_dir, name), "a", encoding="utf-8")
            self._log.write(time.strftime("%H:%M:%S ") + line + "\n")
            self._log.flush()                # rare events: flush each one
        except OSError:
            pass

    def close(self) -> None:
        if self._log is not None:
            try:
                self._log.close()
            except OSError:
                pass
            self._log = None
