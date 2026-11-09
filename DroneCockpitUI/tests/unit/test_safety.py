"""
test_safety.py -- control/safety.py (pure rules), core/safety_monitor.py
(alarms + log) and the safety gates in UplinkController, without hardware.

    pytest tests/unit/test_safety.py        (from DroneCockpitUI/)
    python tests/unit/test_safety.py        (also works stand-alone)
"""
import os
import sys
import tempfile
import types

_APP = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _APP not in sys.path:
    sys.path.insert(0, _APP)
from control import safety                                                 # noqa: E402
from core.safety_monitor import (SafetyMonitor, evaluate_facts,            # noqa: E402
                                 CRITICAL, WARNING, INFO, REPEAT_S)
from core import runtime_guard                                             # noqa: E402
from control.landing_model import LandingModel                             # noqa: E402
from control.altitude_hold import AltitudeHold                             # noqa: E402
from workers.uplink_controller import UplinkController                     # noqa: E402


# ── control/safety.py ────────────────────────────────────────────────────────
def test_assess_failsafe():
    lvl, txt = safety.assess_failsafe(None)
    assert lvl == "unknown" and "USB" in txt
    # the R.A.D. configuration as found: AUTO-LAND with throttle 1020 -> falls
    lvl, txt = safety.assess_failsafe({"procedure": 0, "throttle": 1020, "delay": 15})
    assert lvl == "unsafe" and "1020" in txt
    assert safety.assess_failsafe({"procedure": 1, "throttle": 1400, "delay": 15})[0] == "unsafe"
    # without a learnt hover: plausible value is only "info" (not blocking)
    assert safety.assess_failsafe({"procedure": 0, "throttle": 1350, "delay": 15})[0] == "info"
    assert safety.assess_failsafe({"procedure": 2, "throttle": 1000, "delay": 15}) == \
        ("ok", "GPS-RESCUE")
    assert safety.assess_failsafe({"procedure": 2, "throttle": 1000, "delay": 45})[0] == "warn"
    assert safety.assess_failsafe({"procedure": 7, "throttle": 1300, "delay": 15})[0] == "warn"
    # GPS-RESCUE with GPS off (indoor rig): warning on the ground, unsafe armed
    gr = {"procedure": 2, "throttle": 1000, "delay": 15}
    assert safety.assess_failsafe(gr, gps_fix=False)[0] == "warn"
    assert safety.assess_failsafe(gr, gps_fix=False, armed=True)[0] == "unsafe"
    # with a learnt hover (1450 us at 15 V): checked by predicted descent
    m = _learnt_model()
    fs = lambda thr: {"procedure": 0, "throttle": thr, "delay": 15, "landing_time": 60}
    assert safety.assess_failsafe(fs(1460), m, 15.0)[0] == "unsafe"      # would climb
    assert "not descend" in safety.assess_failsafe(fs(1445), m, 15.0)[1]  # -0.14 m/s: no
    assert safety.assess_failsafe(fs(1420), m, 15.0)[0] == "ok"
    assert safety.assess_failsafe(fs(1200), m, 15.0)[0] == "unsafe"      # falls
    cover = safety.failsafe_cover_m(fs(1415), m, 15.0)
    assert cover is not None and 50 < cover < 100                       # ~1 m/s * 60 s


def _learnt_model(hover=1450.0, volt=15.0, slope_us=35.0, seconds=40.0, spread=True):
    """Model fed with synthetic steady flight: u = a + b*vz."""
    m = LandingModel()
    a, b = (hover - 1000.0) * volt, slope_us * volt
    t = 0.0
    n = int(seconds / 0.04)
    for i in range(n):
        vz = (((i // 250) % 3) - 1) * 0.8 if spread else 0.0     # -0.8 / 0 / +0.8 in 10 s blocks
        thr = 1000.0 + (a + b * vz) / volt
        m.add(t, thr, vz, volt, 1.0, -1.0, 10.0)
        t += 0.04
    return m


def test_landing_model():
    m = LandingModel()
    assert not m.known and not m.recommend()["ok"]
    m = _learnt_model()
    assert m.known
    a, b, measured = m.fit()
    assert measured and abs(b - 35.0 * 15.0) < 5
    assert abs(m.hover_us(15.0) - 1450.0) < 1.0
    assert abs(m.hover_us(16.8) - (1000 + 450 * 15 / 16.8)) < 1.0      # less throttle when full
    assert abs(m.vz_at(1450.0 - 35.0, 15.0) + 1.0) < 0.05
    r = m.recommend(cell_count=4)
    assert r["ok"] and r["vz_full"] <= -0.5 + 1e-6, r                    # descends on a full pack
    assert r["vz_empty"] >= -3.0 - 0.05, r                               # not too fast when empty
    assert 10 <= r["landing_time"] <= 250 and "failsafe_throttle" in r["cli"]
    # tilted / accelerating flight is ignored
    m2 = LandingModel()
    for i in range(500):
        m2.add(i * 0.04, 1450, 0.0, 15.0, roll_deg=20.0)
    assert m2.data_s == 0
    # high flight: AUTO-LAND cannot cover it -> note
    m.max_alt_m = 1500.0
    r = m.recommend(cell_count=4)
    assert r["landing_time"] == 250 and any("GPS-RESCUE" in n for n in r["notes"])
    # no spread: slope stays the prior and says so
    m3 = _learnt_model(spread=False)
    assert not m3.fit()[2] and any("default" in n for n in m3.recommend(4)["notes"])
    # persistence
    path = os.path.join(tempfile.mkdtemp(), "lm.json")
    m.save(path)
    m4 = LandingModel()
    assert m4.load(path) and abs(m4.hover_us(15.0) - 1450.0) < 1.0
    assert m4.data_s <= m4.cfg.saved_weight_s + 1e-6


def _vertical_sim(hold_or_none, thr_of, steps, alt=10.0, vz=0.0, hover=1450.0, dt=0.04):
    """Tiny vertical model: accel = k*(thr-hover) - drag*vz, ground at 0."""
    out = []
    for _ in range(steps):
        thr = thr_of(alt, vz)
        acc = 0.06 * (thr - hover) - 1.2 * vz
        vz += acc * dt
        alt += vz * dt
        if alt <= 0.0:
            alt, vz = 0.0, 0.0
        out.append((alt, vz, thr))
    return out


def test_altitude_hold_landing():
    h = AltitudeHold()
    h.engage(1450.0, 10.0)
    h.start_landing()
    h.land_rate_mps = 0.7
    t = {"n": 0}

    def thr_of(alt, vz):
        t["n"] += 1
        return h.update(alt, 0.04, 100, vz)
    trace = _vertical_sim(h, thr_of, int(40 / 0.04))
    assert trace[-1][0] == 0.0 and h.landed, "lands from 10 m in 40 s"
    peak_down = min(v for _, v, _ in trace)
    assert peak_down > -1.5, f"controlled descent ({peak_down:.2f} m/s)"
    assert h.output_us == h.hover_us - h.cfg.band_us
    # abort: hold where it is
    h2 = AltitudeHold()
    h2.engage(1450.0, 10.0)
    h2.start_landing()
    for _ in range(50):
        h2.update(9.0, 0.04, 100, -0.5)
    h2.stop_landing()
    assert not h2.landing and abs(h2.request_m - 9.0) < 1e-6


def test_geofence_filter():
    F = 300.0
    # inside the fence / no home: untouched
    assert safety.geofence_filter(1, 0, 0, 100, 180, F) == (1, 0, "")
    assert safety.geofence_filter(1, 0, 0, None, None, F) == (1, 0, "")
    # outside, heading north, home south: forward (away) blocked
    f, r, note = safety.geofence_filter(1, 0, 0, 400, 180, F)
    assert (f, r) == (0.0, 0.0) and "blocked" in note
    # backward (toward home) allowed
    f, r, note = safety.geofence_filter(-1, 0, 0, 400, 180, F)
    assert (f, r) == (-1, 0) and "only toward home" in note
    # heading east, home west -> backward allowed, right (south) blocked
    assert safety.geofence_filter(-1, 0, 90, 400, 270, F)[:2] == (-1, 0)
    assert safety.geofence_filter(0, 1, 90, 400, 270, F)[:2] == (0.0, 0.0)
    # 45 deg off the home bearing is still "toward home" (+-60)
    assert safety.geofence_filter(1, 0, 0, 400, 45, F)[:2] == (1, 0)
    assert safety.geofence_filter(1, 0, 0, 400, 75, F)[:2] == (0.0, 0.0)
    # no movement: note only
    f, r, note = safety.geofence_filter(0, 0, 0, 400, 180, F)
    assert (f, r) == (0, 0) and note
    # default fence = radio range: 4.9 km is still free
    assert safety.geofence_filter(1, 0, 0, 4900, 180) == (1, 0, "")


def test_altitude_ceiling():
    assert safety.clamp_altitude_target(6000) == safety.ALT_CEILING_M == 5000.0
    assert safety.clamp_altitude_target(150) == 150.0
    assert safety.land_rate(100) == safety.LAND_FAST_MPS
    assert safety.land_rate(5) == safety.LAND_SLOW_MPS == safety.land_rate(None)


# ── core/safety_monitor.py ───────────────────────────────────────────────────
def _keys(alerts):
    return {a.key: a.level for a in alerts}


def test_evaluate_facts():
    assert evaluate_facts({}) == []
    # video lost while flying: CRITICAL; before it was ever live: nothing
    k = _keys(evaluate_facts({"armed": True, "video_ever_live": True, "video_state": "SignalLost"}))
    assert k["video"] == CRITICAL
    assert "video" not in _keys(evaluate_facts({"armed": True, "video_state": "SignalLost"}))
    k = _keys(evaluate_facts({"video_ever_live": True, "video_state": "SignalLost"}))
    assert k["video"] == WARNING                      # on the ground
    k = _keys(evaluate_facts({"video_ever_live": True, "video_state": "Connected",
                              "video_age_ms": 1500}))
    assert k["video"] == WARNING                      # frozen picture
    # drone link
    k = _keys(evaluate_facts({"radio_ever_live": True, "radio_status": "TELEMETRY_LOST"}))
    assert k["radio"] == CRITICAL
    k = _keys(evaluate_facts({"radio_ever_live": True, "radio_status": "DEGRADED", "lq": 40}))
    assert k["radio"] == WARNING
    # battery
    assert _keys(evaluate_facts({"battery_state": "BatteryState.CRITICAL"}))["battery"] == CRITICAL
    assert _keys(evaluate_facts({"battery_state": "WARNING"}))["battery"] == WARNING
    # laptop control
    up = {"state": "NO_BRIDGE", "controller_errors_new": True,
          "move_blockers": ["FPV video lost", "pilot override (sticks moved)"],
          "pilot_override": True, "geofence": "outside geofence"}
    k = _keys(evaluate_facts({"laptop_enabled": True, "uplink": up}))
    assert k["bridge"] == CRITICAL and k["controller"] == CRITICAL
    assert k["move:FPV"] == WARNING and k["override"] == INFO and k["geofence"] == WARNING
    assert not any(x.startswith("move:pilot") for x in k)
    # laptop off: controller status ignored
    assert evaluate_facts({"laptop_enabled": False, "uplink": up}) == []
    # FC failsafe
    fs = safety.assess_failsafe({"procedure": 0, "throttle": 1020, "delay": 15})
    assert _keys(evaluate_facts({"fc_failsafe": fs}))["failsafe"] == CRITICAL
    # most severe first
    a = evaluate_facts({"battery_state": "WARNING", "fc_failsafe": fs})
    assert a[0].level == CRITICAL


def test_monitor_landing_alerts():
    base = {"laptop_enabled": True, "armed": True}
    k = _keys(evaluate_facts(dict(base, uplink={"landing": True, "land_reason": "video lost",
                                               "land_rate_mps": 1.5})))
    assert k["landing"] == CRITICAL
    k = _keys(evaluate_facts(dict(base, uplink={"landed": True})))
    assert k["landing"] == CRITICAL
    k = _keys(evaluate_facts(dict(base, uplink={"land_fail": "no baro altitude"})))
    assert k["landing"] == CRITICAL
    up = {"video_lost_s": 0.2, "gate_on": True, "armed": True}
    assert "video_land" not in _keys(evaluate_facts(dict(base, uplink=up))), "flicker: no alarm"
    up["video_lost_s"] = 2.0
    a = [x for x in evaluate_facts(dict(base, uplink=up)) if x.key == "video_land"]
    assert a and "3 s" in a[0].text
    k = _keys(evaluate_facts(dict(base, uplink={"range_m": 4500.0})))
    assert k["range"] == WARNING
    k = _keys(evaluate_facts(dict(base, uplink={"fc_failsafe_cover_m": 300.0, "altitude_m": 800.0})))
    assert k["fs_cover"] == WARNING
    k = _keys(evaluate_facts(dict(base, uplink={"fc_failsafe_cover_m": 300.0, "altitude_m": 100.0})))
    assert "fs_cover" not in k


def test_monitor_log_and_sound():
    t = {"now": 0.0}
    sounds = []
    d = tempfile.mkdtemp()
    m = SafetyMonitor(d, sound=sounds.append, clock=lambda: t["now"])
    lost = {"armed": True, "video_ever_live": True, "video_state": "SignalLost"}
    m.evaluate(lost)
    assert sounds == [CRITICAL]
    t["now"] = 1.0
    m.evaluate(lost)
    assert sounds == [CRITICAL]                      # no repeat inside REPEAT_S
    t["now"] = 1.0 + REPEAT_S
    m.evaluate(lost)
    assert sounds == [CRITICAL, CRITICAL]            # CRITICAL repeats
    t["now"] += 1
    assert m.evaluate({"video_ever_live": True, "video_state": "Connected"}) == []
    m.evaluate({"battery_state": "WARNING"})
    assert sounds[-1] == WARNING
    n = len(sounds)
    t["now"] += 2 * REPEAT_S
    m.evaluate({"battery_state": "WARNING"})
    assert len(sounds) == n                          # warnings do not repeat
    m.close()
    logs = [f for f in os.listdir(d) if f.startswith("safety_")]
    assert len(logs) == 1
    text = open(os.path.join(d, logs[0]), encoding="utf-8").read()
    assert "+ CRITICAL FPV VIDEO LOST" in text and "- cleared  FPV VIDEO LOST" in text
    assert "battery low" in text


def test_monitor_acknowledge():
    t = {"now": 0.0}
    sounds = []
    m = SafetyMonitor(tempfile.mkdtemp(), sound=sounds.append, clock=lambda: t["now"])
    lost = {"armed": True, "video_ever_live": True, "video_state": "SignalLost"}
    m.evaluate(lost)
    assert m.has_unacknowledged() and sounds == [CRITICAL]
    m.acknowledge()
    assert not m.has_unacknowledged()
    t["now"] = 2 * REPEAT_S
    m.evaluate(lost)
    assert sounds == [CRITICAL], "acknowledged CRITICAL does not repeat"
    m.evaluate(dict(lost, battery_state="CRITICAL"))
    assert m.has_unacknowledged() and len(sounds) == 2, "a new alert is unacknowledged again"
    m.evaluate({})                                    # all cleared
    m.evaluate(lost)
    assert m.has_unacknowledged(), "a returning alert must be acknowledged again"
    m.close()


def test_monitor_sound_failure_is_harmless():
    def boom(_lvl):
        raise RuntimeError("no audio device")
    m = SafetyMonitor(tempfile.mkdtemp(), sound=boom)
    assert m.evaluate({"battery_state": "CRITICAL"})[0].key == "battery"
    m.close()


def test_runtime_guard_portable():
    # off Windows all of these are no-ops; on Windows they must not raise
    assert runtime_guard.single_instance("Local\\RAD_test_" + str(os.getpid())) in (True, False)
    runtime_guard.keep_awake(True)
    runtime_guard.keep_awake(False)
    runtime_guard.timer_resolution(True)
    runtime_guard.timer_resolution(False)
    n = runtime_guard.errors.count
    runtime_guard.errors.bump()
    assert runtime_guard.errors.count == n + 1


# ── UplinkController safety gates ────────────────────────────────────────────
class _Values:
    def __init__(self):
        self.roll_pct = self.pitch_pct = self.yaw_pct = 0.0
        self.sticks_enabled = self.throttle_enabled = False
        self.throttle_us = 1000.0
        self.gimbal_tilt_pct = 0.0
        self.gimbal_pan_pct = 0.0


class _Cmd:
    def __init__(self):
        self.sent = []
        self.override = False
        self.angle = True
        self.fail = False

    def set_enabled(self, on): pass
    def neutral(self): self.sent.append("neutral")
    def set_gimbal_pct(self, p, t): pass

    def set_command(self, cv):
        self.sent.append(cv)

    def get_status(self):
        if self.fail:
            raise RuntimeError("boom")
        b = types.SimpleNamespace(gate_on=True, script_period_ms=30, sticks_active=True,
                                  throttle_active=True, throttle_delta=0, version=2,
                                  pilot_override=self.override, angle_on=self.angle)
        ch = [1500.0] * 16
        ch[2] = 1400.0                                # throttle echo = hover
        return types.SimpleNamespace(state="ACTIVE", bridge_detected=True, bridge=b,
                                     rtt_avg_ms=60.0, radio_channels_valid=True,
                                     radio_channels_us=ch, frames_sent=1, write_errors=0)


class _Radio:
    def __init__(self):
        self.lq = 100
        self.batt = "OK"
        self.dist = 50.0
        self.bearing = 180.0
        self.yaw = 0.0
        self.sats = 12

    def get_latest_state(self):
        return types.SimpleNamespace(
            armed=True, flight_mode_name="ANGLE", baro_valid=True, baro_altitude_cm=1000,
            baro_vario_cm_per_sec=0, battery_state=self.batt, yaw=self.yaw,
            gps=types.SimpleNamespace(dist_to_home_m=self.dist, bearing_to_home=self.bearing,
                                      num_sat=self.sats),
            radio=types.SimpleNamespace(status_str="TELEMETRY_OK", baro_age_ms=100,
                                        uplink_lq=self.lq, home_set=True))


def _controller(video=None, fs=None):
    cmd, radio = _Cmd(), _Radio()
    flags = {"video": True if video is None else video, "fs": fs}
    c = UplinkController(cmd, radio, backend=types.SimpleNamespace(CommandValues=_Values),
                         log_dir=tempfile.mkdtemp(), bench=False,
                         video_ok=lambda: flags["video"], fc_failsafe=lambda: flags["fs"])
    c.set_enabled(True)
    return c, cmd, radio, flags


def _step(c, t):
    c.ui_heartbeat(t)
    c._tick(0.04, t)
    return c.get_status()


def test_gate_video_lost_then_land():
    c, cmd, radio, flags = _controller()
    c.set_move(1.0, 0.0, 0.0)
    c.request_altitude_hold(True)
    s = _step(c, 0.0)
    assert s["sticks_ok"] and s["hold_engaged"] and cmd.sent[-1].pitch_pct > 0
    flags["video"] = False
    s = _step(c, 0.04)
    assert s["sticks_ok"], "a 40 ms video drop-out does not stop anything"
    s = _step(c, 0.6)
    cv = cmd.sent[-1]
    assert not cv.sticks_enabled and cv.pitch_pct == 0.0
    assert s["hold_engaged"] and cv.throttle_enabled and not s["landing"]
    assert "FPV video lost" in s["move_blockers"] and not s["video_live"]
    s = _step(c, 0.04 + safety.VIDEO_LAND_S + 0.01)
    assert s["landing"] and s["land_reason"] == "video lost", "video lost 5 s -> auto-land"
    flags["video"] = True
    s = _step(c, 6.0)
    assert not s["landing"] and s["hold_engaged"], "picture back: landing aborted, height held"
    assert s["sticks_ok"] and cmd.sent[-1].pitch_pct > 0, "movement resumes with video"


def test_short_video_dropouts_never_land():
    c, cmd, radio, flags = _controller()
    c.request_altitude_hold(True)
    t = 0.0
    for k in range(400):                       # 16 s of a flickering analog picture
        flags["video"] = (k % 25) != 0         # 40 ms out every second
        s = _step(c, t)
        t += 0.04
        assert not s["landing"]


def test_gate_link_quality_override_angle():
    c, cmd, radio, flags = _controller()
    c.set_move(0.0, 1.0, 0.0)
    radio.lq = 40                                  # long range: still flying normally
    assert _step(c, 0.0)["sticks_ok"]
    radio.lq = 20
    assert _step(c, 0.04)["sticks_ok"], "short LQ dip is ignored"
    s = _step(c, 1.1)
    assert not s["sticks_ok"] and any("link quality" in b for b in s["move_blockers"])
    radio.lq = 90
    cmd.override = True
    s = _step(c, 0.04)
    assert not s["sticks_ok"] and s["pilot_override"] and cmd.sent[-1].roll_pct == 0.0
    cmd.override = False
    cmd.angle = False
    s = _step(c, 0.08)
    assert any("ANGLE switch" in b for b in s["move_blockers"])
    cmd.angle = True
    assert _step(c, 0.12)["sticks_ok"]


def test_gate_battery_lands_and_failsafe_stops():
    c, cmd, radio, flags = _controller()
    c.set_move(1.0, 0.0, 0.0)
    assert _step(c, 0.0)["sticks_ok"]
    radio.batt = "CRITICAL"
    s = _step(c, 0.04)
    assert s["landing"] and s["land_reason"] == "battery critical", "battery critical -> land"
    assert cmd.sent[-1].throttle_enabled and cmd.sent[-1].pitch_pct == 0.0

    c, cmd, radio, flags = _controller(fs={"procedure": 0, "throttle": 1020, "delay": 15})
    c.set_move(1.0, 0.0, 0.0)
    s = _step(c, 0.0)
    assert not s["sticks_ok"] and any("FC failsafe unsafe" in b for b in s["blockers"])
    assert s["fc_failsafe"][0] == "unsafe"
    flags["fs"] = {"procedure": 2, "throttle": 1000, "delay": 15}
    assert _step(c, 0.04)["sticks_ok"]
    radio.sats = 0                              # GPS off (indoor rig) + GPS-RESCUE, armed
    s = _step(c, 0.08)
    assert not s["sticks_ok"] and "GPS" in s["fc_failsafe"][1]


def test_gate_geofence_and_ceiling():
    c, cmd, radio, flags = _controller()
    radio.dist = 4000.0
    c.set_move(1.0, 0.0, 0.0)
    assert _step(c, 0.0)["sticks_ok"] and cmd.sent[-1].pitch_pct > 0, "4 km: free"
    radio.dist, radio.bearing, radio.yaw = 5200.0, 180.0, 0.0
    c.set_move(1.0, 0.0, 0.0)                         # away from home
    s = _step(c, 0.0)
    assert cmd.sent[-1].pitch_pct == 0.0 and "blocked" in s["geofence"]
    c.set_move(-1.0, 0.0, 0.5)                        # toward home + yaw
    _step(c, 0.04)
    assert cmd.sent[-1].pitch_pct < 0 and cmd.sent[-1].yaw_pct > 0

    c.request_altitude_hold(True)
    _step(c, 0.08)
    c.nudge_altitude(9000.0)
    s = _step(c, 0.12)
    assert c.hold.request_m <= safety.ALT_CEILING_M
    assert s["alt_ceiling_m"] == safety.ALT_CEILING_M


class _SimRadio(_Radio):
    """_Radio whose baro follows a vertical model driven by the throttle sent."""
    def __init__(self, cmd):
        super().__init__()
        self.cmd, self.alt, self.vz = cmd, 8.0, 0.0

    def step(self, dt=0.04, hover=1450.0):
        cv = self.cmd.sent[-1] if self.cmd.sent and not isinstance(self.cmd.sent[-1], str) else None
        thr = hover
        if cv is not None and cv.throttle_enabled:
            thr = max(hover - 100, min(hover + 100, cv.throttle_us))   # radio band
        acc = 0.06 * (thr - hover) - 1.2 * self.vz
        self.vz += acc * dt
        self.alt += self.vz * dt
        if self.alt <= 0:
            self.alt, self.vz = 0.0, 0.0
        self.echo = thr

    def get_latest_state(self):
        s = super().get_latest_state()
        s.baro_altitude_cm = self.alt * 100.0
        s.baro_vario_cm_per_sec = self.vz * 100.0
        s.battery_voltage, s.roll, s.pitch, s.battery_cell_count = 15.0, 0, 0, 4
        return s


def test_operator_land_touchdown_and_abort():
    cmd = _Cmd()
    radio = _SimRadio(cmd)
    c = UplinkController(cmd, radio, backend=types.SimpleNamespace(CommandValues=_Values),
                         log_dir=tempfile.mkdtemp(), bench=False,
                         video_ok=lambda: True, fc_failsafe=lambda: None)
    c.set_enabled(True)
    cmd.angle = True
    c.request_land(True)                        # hold not engaged yet: engages + lands
    t = 0.0
    s = _step(c, t)
    assert s["landing"] and s["land_reason"] == "operator"
    for _ in range(int(60 / 0.04)):
        radio.step()
        t += 0.04
        s = _step(c, t)
        if s["landed"]:
            break
    assert s["landed"] and radio.alt == 0.0, f"touched down (alt {radio.alt:.2f})"
    assert cmd.sent[-1].throttle_us == 1000.0, "landed: throttle target 1000 (band below the stick)"
    assert c.model.data_s > 0, "the descent fed the landing model"
    # abort mid-air
    cmd2 = _Cmd()
    radio2 = _SimRadio(cmd2)
    c2 = UplinkController(cmd2, radio2, backend=types.SimpleNamespace(CommandValues=_Values),
                          log_dir=tempfile.mkdtemp(), bench=False, video_ok=lambda: True)
    c2.set_enabled(True)
    c2.request_land(True)
    t = 0.0
    for _ in range(50):
        radio2.step()
        s = _step(c2, t)
        t += 0.04
    c2.request_land(False)
    s = _step(c2, t)
    assert not s["landing"] and s["hold_engaged"], "abort -> hold where it is"
    assert abs(s["hold_request_m"] - radio2.alt) < 0.3


def test_land_refused_without_baro_does_not_start_later():
    c, cmd, radio, flags = _controller()
    orig = radio.get_latest_state

    def no_baro():
        s = orig()
        s.baro_valid = False
        return s
    radio.get_latest_state = no_baro
    c.request_land(True)
    s = _step(c, 0.0)
    assert not s["landing"] and "baro" in s["land_fail"]
    radio.get_latest_state = orig
    s = _step(c, 0.5)
    assert not s["landing"], "a refused Land request is dropped, not started later"


def test_controller_exception_goes_neutral():
    c, cmd, radio, flags = _controller()
    cmd.fail = True
    c._running.set()
    import threading
    import time
    th = threading.Thread(target=c._run, daemon=True)
    th.start()
    time.sleep(0.15)
    c._running.clear()
    th.join(1.0)
    s = c.get_status()
    assert s["state"] == "ERROR" and s["controller_errors"] >= 1 and s["enabled"]
    assert "neutral" in cmd.sent


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except AssertionError as e:
                fails += 1
                print("FAIL", name, e)
    sys.exit(1 if fails else 0)
