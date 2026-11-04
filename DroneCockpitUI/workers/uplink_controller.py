"""
uplink_controller.py  --  laptop control of the drone through the Pocket
=======================================================================
Background thread (25 Hz) between the cockpit UI and DroneBackend.CommandLink.

    CommandPanel (Tk) --intents--> UplinkController --set_command()--> CommandLink
                                         ^                                 |
          CrsfLink.get_latest_state() ---+  (armed, mode, baro altitude)   v
                                                     CrsfLink worker -> Pocket

What the laptop can do
----------------------
  * move: forward/back, left/right, yaw left/right -- small stick OFFSETS
    (percent of stick travel) on top of the pilot's sticks, only in ANGLE
    mode so an offset means a lean angle, not a rotation rate;
  * altitude hold: throttle from AltitudeHold around the hover throttle the
    radio was sending when it was engaged;
  * gimbal tilt (always, also without the pilot switch).
What it can never do: arm, disarm, change flight mode, trigger failsafe.

Gates, checked every tick (any failing gate -> neutral, hold dropped):
  enabled by the user, Lua bridge detected, ELRS telemetry live, drone armed,
  flight mode ANGLE. The pilot's switch is checked on the radio itself.
Set COCKPIT_UPLINK_BENCH=1 to drop the armed/ANGLE gates for bench tests
with the propellers OFF -- the panel shows a red BENCH label.

Every command change is written to logs/uplink_<date>.csv.

Threads and real time (nothing here can stall video or telemetry)
-----------------------------------------------------------------
  Tk thread           CommandPanel: key/button events, 10 Hz status display.
                      Only sets intents under a short lock; never waits.
  UplinkController    this thread, 25 Hz: reads telemetry snapshots, runs
                      AltitudeHold, calls CommandLink.set_command(). Never
                      touches Tk, never blocks on I/O except a buffered log.
  CrsfLink worker     C++, owns the Pocket's COM port: reads telemetry,
                      writes command frames. Needs no GIL.
  VideoLink, DetectionLink, TelemetryWorker run on their own threads and
  share nothing with this path except mutex-protected snapshots.

Each thread is guarded against the one before it stalling:
  * Tk thread frozen (heavy redraw, dialog): the panel's heartbeat stops,
    movement intents expire after UI_TIMEOUT_S -> sticks to neutral, even
    if a key was held when it froze. Altitude hold keeps regulating.
  * this thread stalled: CommandLink's lease (300 ms) -> neutral.
  * CrsfLink worker or the USB link stalled: radcmd.lua watchdog (500 ms).
"""

import csv
import os
import collections
import threading
import time
from typing import Optional

from control.altitude_hold import AltitudeHold, AltitudeHoldConfig

UPLINK_HZ = 25
RADIO_LIVE = ("TELEMETRY_OK", "DEGRADED")
BENCH_MODE = os.environ.get("COCKPIT_UPLINK_BENCH", "0") == "1"
ALT_HOLD_DROP_STALE_S = 1.5        # baro missing this long -> hold dropped
UI_TIMEOUT_S = 0.5                 # no panel heartbeat this long -> movement neutral
LOG_FLUSH_S = 1.0                  # command log is flushed at most this often
THROTTLE_CH = 2                     # CH3 (0-based) in the radio channel echo

# DroneCockpitUI/logs (this file lives in DroneCockpitUI/workers/)
_LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


class UplinkController:
    def __init__(self, command_link, radio, hold_cfg: Optional[AltitudeHoldConfig] = None,
                 backend=None, log_dir: str = _LOG_DIR, bench: bool = BENCH_MODE) -> None:
        self._cmd = command_link
        self._radio = radio
        self._backend = backend         # DroneBackend module (for CommandValues)
        self.bench = bench
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Intents (written by the Tk thread)
        self._enabled = False
        self._fwd = self._right = self._yaw = 0.0      # -1..1
        self._speed_pct = 10.0                         # stick % at full intent
        self._yaw_pct = 15.0
        self._tilt_pct = 0.0
        self._hold_requested = False
        self._ui_beat = float("-inf")                  # last CommandPanel heartbeat
        self._last_flush = 0.0
        self._gaps = collections.deque()             # (t, dt) of the last 2 s

        self.hold = AltitudeHold(hold_cfg)
        self._stale_since: Optional[float] = None

        # Published state (read by the Tk thread; replaced as a whole dict)
        self._status: dict = {"blockers": ["not started"], "hold_engaged": False}
        self._log_dir = log_dir
        self._log_file = None
        self._log = None
        self._last_logged = None
        self._last_log_t = 0.0

    # ══ Tk-thread API ═══════════════════════════════════════════════════════
    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="UplinkController", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.set_enabled(False)
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._cmd is not None:
            self._cmd.neutral()
            self._cmd.set_enabled(False)
        if self._log_file:
            self._log_file.close()
            self._log_file = None

    def set_enabled(self, on: bool) -> None:
        with self._lock:
            self._enabled = bool(on)
            if not on:
                self._fwd = self._right = self._yaw = 0.0
                self._hold_requested = False
        if self._cmd is not None:
            self._cmd.set_enabled(bool(on))
            if not on:
                self._cmd.neutral()

    def set_move(self, forward: float, right: float, yaw: float) -> None:
        """Intents in -1..1 (key held = 1). Scaled by the speed setting."""
        clamp = lambda v: max(-1.0, min(1.0, float(v)))
        with self._lock:
            self._fwd, self._right, self._yaw = clamp(forward), clamp(right), clamp(yaw)

    def ui_heartbeat(self, t: Optional[float] = None) -> None:
        """Called by the Tk thread (panel refresh + every key event). Movement
        intents are only honoured while these keep coming, so a frozen UI
        can never leave a held key driving the drone."""
        self._ui_beat = time.monotonic() if t is None else t

    def stop_move(self) -> None:
        self.set_move(0, 0, 0)

    def set_speed(self, stick_pct: float, yaw_pct: Optional[float] = None) -> None:
        with self._lock:
            self._speed_pct = max(0.0, min(100.0, float(stick_pct)))
            if yaw_pct is not None:
                self._yaw_pct = max(0.0, min(100.0, float(yaw_pct)))

    def set_gimbal_tilt(self, pct: float) -> None:
        with self._lock:
            self._tilt_pct = max(-100.0, min(100.0, float(pct)))

    def request_altitude_hold(self, on: bool) -> None:
        with self._lock:
            self._hold_requested = bool(on)

    def nudge_altitude(self, delta_m: float) -> None:
        with self._lock:
            if self.hold.engaged:
                self.hold.nudge_target(delta_m)

    def get_status(self) -> dict:
        return self._status

    # ══ Worker ══════════════════════════════════════════════════════════════
    def _run(self) -> None:
        period = 1.0 / UPLINK_HZ
        last = time.monotonic()
        while self._running.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            self._gaps.append((now, dt))
            while self._gaps and now - self._gaps[0][0] > 2.0:
                self._gaps.popleft()
            try:
                self._tick(dt, now)
            except Exception as exc:            # never let the loop die silently
                self._status = {"blockers": [f"error: {exc}"], "hold_engaged": False}
                if self._cmd is not None:
                    self._cmd.neutral()
            time.sleep(max(0.0, period - (time.monotonic() - now)))

    def _values(self):
        if self._backend is not None:
            return self._backend.CommandValues()
        import DroneBackend                      # cockpit runtime
        return DroneBackend.CommandValues()

    def _tick(self, dt: float, now: float) -> None:
        with self._lock:
            enabled = self._enabled
            fwd, right, yaw = self._fwd, self._right, self._yaw
            speed, yaw_speed, tilt = self._speed_pct, self._yaw_pct, self._tilt_pct
            hold_req = self._hold_requested
        # UI heartbeat: a frozen Tk thread must not leave a held key driving
        # the drone. Only movement is cancelled; altitude hold and gimbal keep
        # running, they don't depend on a key being held.
        ui_alive = (now - self._ui_beat) <= UI_TIMEOUT_S
        if not ui_alive:
            fwd = right = yaw = 0.0

        st = self._cmd.get_status()
        rs = self._radio.get_latest_state() if self._radio is not None else None

        # ── gates ──────────────────────────────────────────────────────────
        blockers = []
        if not enabled:
            blockers.append("laptop control off")
        if not st.bridge_detected:
            blockers.append("no Lua bridge on the Pocket")
        radio_status = rs.radio.status_str if rs is not None else "NO_RADIO"
        if radio_status not in RADIO_LIVE:
            blockers.append(f"ELRS telemetry {radio_status}")
        armed = bool(getattr(rs, "armed", False)) if rs is not None else False
        mode = str(getattr(rs, "flight_mode_name", "")) if rs is not None else ""
        if not self.bench:
            if not armed:
                blockers.append("drone not armed")
            if not mode.startswith("ANGLE"):
                blockers.append(f"flight mode {mode or '?'} (needs ANGLE)")
        sticks_ok = not blockers
        gate_on = bool(st.bridge.gate_on)

        # ── altitude ───────────────────────────────────────────────────────
        alt_m = vz = None
        alt_age = -1
        if rs is not None and getattr(rs, "baro_valid", False):
            alt_m = rs.baro_altitude_cm / 100.0
            alt_age = int(getattr(rs.radio, "baro_age_ms", -1))
            vz = getattr(rs, "baro_vario_cm_per_sec", 0) / 100.0

        hold_reason = ""
        if self.hold.engaged:
            if not hold_req or not sticks_ok or not gate_on:
                hold_reason = "released" if not hold_req else (
                    "pilot switch off" if sticks_ok else blockers[0])
                self.hold.disengage()
                with self._lock:
                    self._hold_requested = False
        elif hold_req:
            hover = self._radio_throttle_us(st)
            if not sticks_ok:
                hold_reason = blockers[0]
            elif not gate_on:
                hold_reason = "pilot switch off"
            elif alt_m is None or alt_age < 0 or alt_age > self.hold.cfg.stale_ms:
                hold_reason = "no baro altitude"
            elif hover is None:
                hold_reason = "no throttle echo from the radio"
            else:
                self.hold.engage(hover, alt_m)
            if not self.hold.engaged:
                with self._lock:
                    self._hold_requested = False

        thr_us = 1000.0
        if self.hold.engaged:
            thr_us = self.hold.update(alt_m if alt_m is not None else self.hold.target_m,
                                      dt, alt_age, vz)
            if self.hold.stale:
                self._stale_since = self._stale_since or now
                if now - self._stale_since > ALT_HOLD_DROP_STALE_S:
                    self.hold.disengage()
                    hold_reason = "baro altitude lost"
                    with self._lock:
                        self._hold_requested = False
            else:
                self._stale_since = None

        # ── command ────────────────────────────────────────────────────────
        cv = self._values()
        cv.gimbal_tilt_pct = tilt
        cv.sticks_enabled = sticks_ok
        cv.roll_pct = right * speed if sticks_ok else 0.0
        cv.pitch_pct = fwd * speed if sticks_ok else 0.0
        cv.yaw_pct = yaw * yaw_speed if sticks_ok else 0.0
        cv.throttle_enabled = self.hold.engaged
        cv.throttle_us = thr_us
        if enabled:
            self._cmd.set_command(cv)        # renews the lease
        else:
            self._cmd.set_gimbal_tilt_pct(tilt)

        self._status = {
            "enabled": enabled,
            "ui_alive": ui_alive,
            # loop timing over the last 2 s (25 Hz -> 40 ms); the lease is
            # 300 ms, so anything well below that is harmless
            "loop_max_ms": max((g for _, g in list(self._gaps)), default=0.0) * 1000.0,
            "bench": self.bench,
            "state": st.state,
            "blockers": blockers,
            "gate_on": gate_on,
            "sticks_ok": sticks_ok,
            "armed": armed,
            "flight_mode": mode,
            "radio_status": radio_status,
            "hold_engaged": self.hold.engaged,
            "hold_reason": hold_reason,
            "hold_target_m": self.hold.target_m,
            "hold_request_m": self.hold.request_m,
            "hold_output_us": self.hold.output_us,
            "hold_hover_us": self.hold.hover_us,
            "hold_stale": self.hold.stale,
            "altitude_m": alt_m,
            "altitude_age_ms": alt_age,
            "cmd_roll_pct": cv.roll_pct, "cmd_pitch_pct": cv.pitch_pct,
            "cmd_yaw_pct": cv.yaw_pct, "cmd_tilt_pct": tilt,
            "rtt_ms": float(st.rtt_avg_ms),
            "script_period_ms": int(st.bridge.script_period_ms),
            "lua_sticks_active": bool(st.bridge.sticks_active),
            "lua_throttle_active": bool(st.bridge.throttle_active),
            "lua_throttle_delta_us": int(st.bridge.throttle_delta) / 2.0,
            "radio_channels_us": list(st.radio_channels_us) if st.radio_channels_valid else [],
            "frames_sent": int(st.frames_sent),
            "write_errors": int(st.write_errors),
        }
        self._write_log(now, cv, st.state)

    @staticmethod
    def _radio_throttle_us(st) -> Optional[float]:
        if not st.radio_channels_valid:
            return None
        ch = list(st.radio_channels_us)
        return float(ch[THROTTLE_CH]) if len(ch) > THROTTLE_CH else None

    # ── command log (safety rule: every command is logged) ─────────────────
    def _write_log(self, now: float, cv, state: str) -> None:
        row = (state, round(cv.roll_pct, 1), round(cv.pitch_pct, 1), round(cv.yaw_pct, 1),
               bool(cv.sticks_enabled), bool(cv.throttle_enabled), round(cv.throttle_us),
               round(cv.gimbal_tilt_pct, 1))
        if row == self._last_logged and now - self._last_log_t < 1.0:
            return
        try:
            if self._log is None:
                os.makedirs(self._log_dir, exist_ok=True)
                name = time.strftime("uplink_%Y%m%d_%H%M%S.csv")
                self._log_file = open(os.path.join(self._log_dir, name), "w", newline="")
                self._log = csv.writer(self._log_file)
                self._log.writerow(["time", "state", "roll_pct", "pitch_pct", "yaw_pct",
                                    "sticks_en", "thr_en", "thr_us", "tilt_pct",
                                    "alt_m", "target_m"])
            s = self._status
            self._log.writerow([time.strftime("%H:%M:%S") + f".{int(now * 1000) % 1000:03d}",
                                *row, s.get("altitude_m"), round(self.hold.target_m, 2)])
            if now - self._last_flush >= LOG_FLUSH_S:   # disk I/O at most 1 Hz
                self._log_file.flush()
                self._last_flush = now
        except OSError:
            pass                               # logging must never stop control
        self._last_logged = row
        self._last_log_t = now
