"""
test_uplink_python.py -- AltitudeHold and UplinkController without hardware.

    pytest tests/unit/test_uplink_python.py        (from DroneCockpitUI/)
    python tests/unit/test_uplink_python.py        (also works stand-alone)

AltitudeHold is checked against a simple vertical model of the drone
(thrust proportional to throttle above idle, battery sag as a slow thrust
loss, baro noise and a 100 ms telemetry delay at 10 Hz). It is a sanity
check of the logic, not a tuning: gains are tuned on the rod rig.
"""
import os
import random
import sys
import tempfile
import types

_APP = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _APP not in sys.path:
    sys.path.insert(0, _APP)
from control.altitude_hold import AltitudeHold, AltitudeHoldConfig      # noqa: E402
from workers.uplink_controller import UplinkController                  # noqa: E402

PASS = FAIL = 0
FAILURES = []


def check(cond, what):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    if not cond:
        FAILURES.append(what)
        print("FAIL:", what)


def _assert_clean():
    """pytest: fail the test if any check() in it failed."""
    failed = list(FAILURES)
    FAILURES.clear()
    assert not failed, "; ".join(failed)


# ── vertical model ────────────────────────────────────────────────────────────
def simulate(seconds=30.0, sag_per_s=0.0, step_at=None, step_m=0.0, seed=1):
    rnd = random.Random(seed)
    g = 9.81
    hover_us = 1480.0
    k = g / (hover_us - 1050.0)          # accel per us above idle at hover
    z, vz = 10.0, 0.0
    hold = AltitudeHold()
    hold.engage(hover_us, z)
    dt_sim, ctrl_dt = 0.005, 0.04        # 200 Hz physics, 25 Hz controller
    baro_period, delay = 0.1, 0.1
    samples, t, next_baro, next_ctrl = [], 0.0, 0.0, 0.0
    last_baro = (z, 0.0)                 # (value, time measured)
    thr, max_err, settled_err = hover_us, 0.0, []
    while t < seconds:
        if t >= next_baro:
            samples.append((t, z + rnd.gauss(0, 0.08)))
            next_baro += baro_period
        while samples and samples[0][0] <= t - delay:
            last_baro = samples.pop(0)
        if t >= next_ctrl:
            if step_at is not None and abs(t - step_at) < ctrl_dt / 2:
                hold.nudge_target(step_m)
            age_ms = int((t - last_baro[0]) * 1000)
            thr = hold.update(last_baro[1], ctrl_dt, age_ms)
            next_ctrl += ctrl_dt
        eff = 1.0 - sag_per_s * t
        acc = k * eff * (thr - 1050.0) - g - 0.4 * vz
        vz += acc * dt_sim
        z += vz * dt_sim
        err = abs(hold.target_m - z)
        max_err = max(max_err, err)
        if t > seconds - 5:
            settled_err.append(err)
        t += dt_sim
    return hold, max_err, sum(settled_err) / len(settled_err), z


def test_altitude_hold():
    hold, max_err, final_err, z = simulate(30)
    check(max_err < 0.5, f"steady hover error {max_err:.2f} m")
    hold, max_err, final_err, z = simulate(60, sag_per_s=0.0015)   # ~9 % thrust loss
    check(final_err < 0.35, f"battery sag compensated by I term ({final_err:.2f} m)")
    check(abs(hold.integral_us) > 5, "integral picked up the sag")
    hold, _, final_err, z = simulate(30, step_at=5.0, step_m=2.0)
    check(abs(hold.target_m - 12.0) < 1e-6, "target ramped to the request")
    check(final_err < 0.35, f"+2 m step settled ({final_err:.2f} m, z={z:.2f})")

    # output never leaves the band
    h = AltitudeHold(AltitudeHoldConfig(band_us=100))
    h.engage(1500, 0.0)
    for _ in range(200):
        u = h.update(-50.0, 0.04, 100)            # huge error
        check(1400 <= u <= 1600, "band respected")
    check(abs(h.integral_us) <= h.cfg.i_limit_us + 1e-9, "integral clamped")

    # stale altitude freezes output
    h.engage(1500, 0.0)
    u1 = h.update(-1.0, 0.04, 100)
    u2 = h.update(-5.0, 0.04, 5000)
    check(u1 == u2 and h.stale, "stale altitude freezes the output")
    _assert_clean()


# ── controller with fakes ─────────────────────────────────────────────────────
class FakeValues:
    def __init__(self):
        self.roll_pct = self.pitch_pct = self.yaw_pct = 0.0
        self.sticks_enabled = self.throttle_enabled = False
        self.throttle_us = 1000.0
        self.gimbal_tilt_pct = 0.0


class FakeCmd:
    def __init__(self):
        self.enabled = False
        self.sent = []
        self.tilt = None
        self.neutral_calls = 0
        self.gate = True
        self.bridge = True
        self.radio_ch = [1500.0] * 16
        self.radio_ch[2] = 1470.0

    def set_enabled(self, on): self.enabled = on
    def neutral(self): self.neutral_calls += 1
    def set_command(self, cv): self.sent.append(cv)
    def set_gimbal_tilt_pct(self, p): self.tilt = p

    def get_status(self):
        b = types.SimpleNamespace(gate_on=self.gate, script_period_ms=30, sticks_active=self.gate,
                                  throttle_active=False, throttle_delta=0)
        return types.SimpleNamespace(state="ACTIVE" if self.gate else "STANDBY",
                                     bridge_detected=self.bridge, bridge=b, rtt_avg_ms=60.0,
                                     radio_channels_valid=True, radio_channels_us=self.radio_ch,
                                     frames_sent=1, write_errors=0)


class FakeRadio:
    def __init__(self):
        self.s = types.SimpleNamespace(
            armed=True, flight_mode_name="ANGLE", baro_valid=True, baro_altitude_cm=1000,
            baro_vario_cm_per_sec=0, radio=types.SimpleNamespace(status_str="TELEMETRY_OK",
                                                                 baro_age_ms=100))

    def get_latest_state(self): return self.s


def make():
    cmd, radio = FakeCmd(), FakeRadio()
    backend = types.SimpleNamespace(CommandValues=FakeValues)
    c = UplinkController(cmd, radio, backend=backend, log_dir=tempfile.mkdtemp(), bench=False)
    return c, cmd, radio


def test_controller():
    c, cmd, radio = make()
    c.ui_heartbeat(0.0)
    c._tick(0.04, 0.0)
    check(not cmd.sent and cmd.tilt == 0.0, "disabled: only gimbal, no command")
    check("laptop control off" in c.get_status()["blockers"], "blocker listed")

    c.set_enabled(True)
    c.set_speed(10)
    c.set_move(1, -1, 0)
    c.ui_heartbeat(0.04)
    c._tick(0.04, 0.04)
    cv = cmd.sent[-1]
    check(cv.sticks_enabled and cv.pitch_pct == 10 and cv.roll_pct == -10, "forward-left 10 %")

    radio.s.flight_mode_name = "HORIZON"
    c.ui_heartbeat(0.08)
    c._tick(0.04, 0.08)
    cv = cmd.sent[-1]
    check(not cv.sticks_enabled and cv.pitch_pct == 0, "not ANGLE -> neutral")
    radio.s.flight_mode_name = "ANGLE"

    radio.s.armed = False
    c.ui_heartbeat(0.12)
    c._tick(0.04, 0.12)
    check(not cmd.sent[-1].sticks_enabled, "disarmed -> neutral")
    radio.s.armed = True

    radio.s.radio.status_str = "TELEMETRY_LOST"
    c.ui_heartbeat(0.16)
    c._tick(0.04, 0.16)
    check(not cmd.sent[-1].sticks_enabled, "telemetry lost -> neutral")
    radio.s.radio.status_str = "TELEMETRY_OK"

    # altitude hold: bumpless engage from the radio's CH3
    c.stop_move()
    c.request_altitude_hold(True)
    c.ui_heartbeat(0.20)
    c._tick(0.04, 0.20)
    cv = cmd.sent[-1]
    check(c.hold.engaged and cv.throttle_enabled, "hold engaged")
    check(abs(cv.throttle_us - 1470.0) < 1.0, f"bumpless: starts at radio CH3 ({cv.throttle_us})")
    c.nudge_altitude(1.0)
    for i in range(10):
        c.ui_heartbeat(0.24 + i * 0.04)
        c._tick(0.04, 0.24 + i * 0.04)
    check(cmd.sent[-1].throttle_us > 1470.0, "climb request raises throttle")

    # pilot switch off -> hold dropped
    cmd.gate = False
    c.ui_heartbeat(1.0)
    c._tick(0.04, 1.0)
    check(not c.hold.engaged and not cmd.sent[-1].throttle_enabled, "gate off drops the hold")
    check("pilot switch" in c.get_status()["hold_reason"], "reason shown")
    cmd.gate = True

    # hold refused without baro
    radio.s.baro_valid = False
    c.request_altitude_hold(True)
    c.ui_heartbeat(1.04)
    c._tick(0.04, 1.04)
    check(not c.hold.engaged and c.get_status()["hold_reason"] == "no baro altitude",
          "no baro -> no hold")
    radio.s.baro_valid = True

    # baro lost while holding -> dropped after 1.5 s
    c.request_altitude_hold(True)
    c.ui_heartbeat(2.0)
    c._tick(0.04, 2.0)
    check(c.hold.engaged, "hold engaged again")
    radio.s.radio.baro_age_ms = 5000
    c.ui_heartbeat(2.04)
    c._tick(0.04, 2.04)
    check(c.hold.engaged, "still holding right after baro loss")
    c.ui_heartbeat(3.7)
    c._tick(0.04, 3.7)
    check(not c.hold.engaged, "hold dropped after baro loss")
    radio.s.radio.baro_age_ms = 100

    # disable -> neutral and CommandLink disabled
    c.set_enabled(False)
    check(not cmd.enabled and cmd.neutral_calls > 0, "disable reaches CommandLink")

    # bench mode skips armed / ANGLE gates
    cb, cmdb, radiob = make()
    cb.bench = True
    radiob.s.armed = False
    radiob.s.flight_mode_name = "ACRO"
    cb.set_enabled(True)
    cb.set_move(1, 0, 0)
    cb.ui_heartbeat(0.0)
    cb._tick(0.04, 0.0)
    check(cmdb.sent[-1].sticks_enabled, "bench mode")
    _assert_clean()


def test_ui_freeze_stops_movement():
    """Tk thread freezes while W is held: movement must stop within
    UI_TIMEOUT_S, altitude hold must keep regulating."""
    from workers.uplink_controller import UI_TIMEOUT_S
    c, cmd, radio = make()
    c.set_enabled(True)
    c.set_speed(20)
    c.request_altitude_hold(True)
    c.set_move(1, 0, 0)                      # W held
    t = 0.0
    c.ui_heartbeat(t)
    c._tick(0.04, t)
    check(cmd.sent[-1].pitch_pct == 20 and c.hold.engaged, "moving forward, hold on")
    check(c.get_status()["ui_alive"], "ui alive")
    # UI frozen from here on: no more heartbeats, the key is still "held"
    while t < UI_TIMEOUT_S - 0.05:
        t += 0.04
        c._tick(0.04, t)
    check(cmd.sent[-1].pitch_pct == 20, "short hiccup (< timeout) tolerated")
    t = UI_TIMEOUT_S + 0.01
    c._tick(0.04, t)
    cv = cmd.sent[-1]
    check(cv.pitch_pct == 0 and cv.roll_pct == 0 and cv.yaw_pct == 0, "frozen UI -> neutral sticks")
    check(c.hold.engaged and cv.throttle_enabled, "altitude hold keeps running")
    check(not c.get_status()["ui_alive"], "status shows ui stalled")
    # UI recovers -> held key works again (no stale latch)
    c.ui_heartbeat(t + 0.04)
    c._tick(0.04, t + 0.04)
    check(cmd.sent[-1].pitch_pct == 20, "UI back -> movement resumes")
    _assert_clean()


def test_controller_thread_never_blocks_on_ui():
    """Real 25 Hz thread against a 'UI' that hammers set_move/get_status:
    every tick must finish quickly and the loop period must hold."""
    import threading, time as _t
    c, cmd, radio = make()
    c.set_enabled(True)
    stop = threading.Event()

    def ui():
        i = 0
        while not stop.is_set():
            c.ui_heartbeat()
            c.set_move((i % 3) - 1, 0, 0)
            c.get_status()
            i += 1
    th = threading.Thread(target=ui, daemon=True)
    th.start()
    c.start()
    _t.sleep(1.0)
    stop.set()
    th.join(1.0)
    c.stop()
    n = len(cmd.sent)
    check(18 <= n <= 30, f"~25 commands in 1 s under UI load ({n})")
    _assert_clean()


if __name__ == "__main__":
    for fn in (test_altitude_hold, test_controller, test_ui_freeze_stops_movement,
               test_controller_thread_never_blocks_on_ui):
        try:
            fn()
        except AssertionError:
            pass
    print(f"{PASS} passed, {FAIL} failed")
    sys.exit(1 if FAIL else 0)
