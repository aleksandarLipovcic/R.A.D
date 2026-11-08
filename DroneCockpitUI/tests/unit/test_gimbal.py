"""
test_gimbal.py -- control/gimbal.py (servo calibration, servo model, feedback
sources) and its use in UplinkController, without hardware.

    pytest tests/unit/test_gimbal.py        (from DroneCockpitUI/)
    python tests/unit/test_gimbal.py        (also works stand-alone)
"""
import os
import sys
import tempfile
import types

_APP = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _APP not in sys.path:
    sys.path.insert(0, _APP)
from control.gimbal import (GimbalAxisConfig, GimbalModel, ServoModelConfig,   # noqa: E402
                            default_pan_config, default_tilt_config, US_PER_PCT)
from workers.uplink_controller import UplinkController                           # noqa: E402

PASS = FAIL = 0
FAILURES = []


def check(cond, what):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    if not cond:
        FAILURES.append(what)
        print("FAIL:", what)


def near(a, b, tol, what):
    check(abs(a - b) <= tol, f"{what}: {a} vs {b}")


def _assert_clean():
    failed = list(FAILURES)
    FAILURES.clear()
    assert not failed, "; ".join(failed)


# ── calibration / conversions ────────────────────────────────────────────────
def test_axis_conversions():
    tilt = default_tilt_config()
    near(tilt.deg_to_us(45.0), 1500.0, 1e-9, "tilt 45 deg = channel centre (horn placement)")
    near(tilt.us_to_deg(tilt.deg_to_us(73.0)), 73.0, 1e-9, "tilt round trip")
    near(tilt.deg_to_pct(45.0), 0.0, 1e-9, "tilt 45 deg = 0 %")
    near(tilt.deg_to_pct(90.0), 450.0 / US_PER_PCT, 1e-9, "tilt 90 deg in %")
    lo, hi = tilt.reachable()
    check((lo, hi) == (0.0, 90.0), f"tilt window 0..90 inside the channel range ({lo}, {hi})")
    near(tilt.clamp(120.0), 90.0, 1e-9, "tilt clamped to soft limit")

    pan = default_pan_config()
    lo, hi = pan.reachable()
    check((lo, hi) == (-45.0, 45.0), f"pan soft limits +-45 ({lo}, {hi})")
    # widen the soft limits: now the channel range (988..2012 us) limits
    wide = GimbalAxisConfig("pan", min_deg=-90, max_deg=90)
    lo, hi = wide.reachable()
    near(lo, -51.2, 1e-9, "channel range limits at -51.2 deg (10 us/deg)")
    near(hi, 51.2, 1e-9, "channel range limits at +51.2 deg")
    check(-100.0 <= wide.deg_to_pct(90) <= 100.0, "percent never leaves -100..100")

    rev = GimbalAxisConfig("x", direction=-1, min_deg=-40, max_deg=40)
    check(rev.deg_to_us(20) < 1500, "reversed axis: + deg = shorter pulse")
    near(rev.us_to_deg(rev.deg_to_us(-17)), -17.0, 1e-9, "reversed round trip")
    _assert_clean()


# ── servo model ──────────────────────────────────────────────────────────────
def test_servo_model():
    g = GimbalModel(servo=ServoModelConfig(rate_dps=300, dead_time_s=0.08, settle_deg=1.5))
    g.update(0.0)                                   # no feedback: commanded = pulse
    a = g.angles(0.0)
    check(not a.moving and a.source == "cmd", "starts settled on the command")
    near(a.tilt_deg, 0.0, 1e-9, "starts at tilt 0")

    g.set_target(tilt_deg=90.0)
    g.update(1.0)
    a = g.angles(1.0)
    check(a.moving, "new target -> moving")
    near(a.tilt_deg, 0.0, 1e-9, "dead time: not moved yet")
    a = g.angles(1.08 + 0.1)                        # 0.1 s of motion after dead time
    near(a.tilt_deg, 30.0, 1e-6, "300 deg/s -> 30 deg after 0.1 s (extrapolated)")
    check(a.moving, "still moving at 30 deg")
    for k in range(1, 30):
        g.update(1.0 + k * 0.04)
    a = g.angles(1.0 + 29 * 0.04)
    near(a.tilt_deg, 90.0, 1e-9, "arrives at 90")
    check(not a.moving, "settled after 90/300 s + dead time")

    g.set_target(tilt_deg=91.0)                     # beyond limit -> clamped, no move
    g.update(3.0)
    check(not g.angles(3.0).moving, "clamped target equal to current -> no motion")
    _assert_clean()


def test_feedback_sources():
    g = GimbalModel(servo=ServoModelConfig(dead_time_s=0.0, rate_dps=1e6))
    servo = [0] * 8
    servo[0] = 1950                                 # tilt servo (index 0) -> 90 deg
    servo[1] = 1700                                 # pan servo (index 1) -> +20 deg
    radio = [1500.0] * 16
    radio[7] = 1500.0                               # CH8 -> 45 deg tilt
    radio[8] = 1300.0                               # CH9 -> -20 deg pan
    g.update(0.0, servo_us=servo, radio_us=radio)
    g.update(0.01, servo_us=servo, radio_us=radio)
    a = g.angles(0.01)
    check(a.source == "usb", "MSP_SERVO wins over the radio echo")
    near(a.tilt_deg, 90.0, 1e-6, "tilt from servo[0]")
    near(a.pan_deg, 20.0, 1e-6, "pan from servo[1]")

    g.update(0.02, servo_us=None, radio_us=radio)
    g.update(0.03, servo_us=None, radio_us=radio)
    a = g.angles(0.03)
    check(a.source == "radio", "radio echo when USB is gone")
    near(a.tilt_deg, 45.0, 1e-6, "tilt from CH8")
    near(a.pan_deg, -20.0, 1e-6, "pan from CH9")

    g.update(0.04, servo_us=[0] * 8, radio_us=None)  # servo outputs not driven
    g.set_target(pan_deg=10, tilt_deg=30)
    g.update(0.05)
    a = g.angles(0.06)
    check(a.source == "cmd", "commanded angles as last resort")
    near(a.tilt_deg, 30.0, 1e-6, "tilt = command")
    _assert_clean()


def test_snapshot_fields():
    a = GimbalModel(axis="roll")
    a.set_target(pan_deg=25, tilt_deg=80)
    a.update(0.0)
    f = a.angles(1.0).snapshot_fields()
    check(f["gimbal_roll_deg"] == 25 and f["gimbal_pan_deg"] == 0.0,
          "R.A.D. mount: pan axis goes to gimbal_roll_deg")
    near(f["gimbal_tilt_deg"], 80, 1e-9, "tilt passed through")
    b = GimbalModel(axis="yaw")
    b.set_target(pan_deg=25)
    b.update(0.0)
    f = b.angles(1.0).snapshot_fields()
    check(f["gimbal_pan_deg"] == 25 and f["gimbal_roll_deg"] == 0.0,
          "classic mount: pan axis goes to gimbal_pan_deg")
    try:
        GimbalModel(axis="sideways")
        check(False, "bad axis rejected")
    except ValueError:
        check(True, "bad axis rejected")
    _assert_clean()


# ── UplinkController integration ─────────────────────────────────────────────
class _Values:
    def __init__(self):
        self.roll_pct = self.pitch_pct = self.yaw_pct = 0.0
        self.sticks_enabled = self.throttle_enabled = False
        self.throttle_us = 1000.0
        self.gimbal_tilt_pct = 0.0
        self.gimbal_pan_pct = 0.0


class _OldValues:                                   # .pyd without the pan channel
    __slots__ = ("roll_pct", "pitch_pct", "yaw_pct", "sticks_enabled",
                 "throttle_enabled", "throttle_us", "gimbal_tilt_pct")

    def __init__(self):
        self.roll_pct = self.pitch_pct = self.yaw_pct = 0.0
        self.sticks_enabled = self.throttle_enabled = False
        self.throttle_us = 1000.0
        self.gimbal_tilt_pct = 0.0


class _Cmd:
    def __init__(self, with_pan=True):
        self.sent, self.gimbal, self.tilt_only = [], None, None
        self.radio_ch = [1500.0] * 16
        self.radio_valid = True
        if with_pan:
            self.set_gimbal_pct = lambda p, t: setattr(self, "gimbal", (p, t))

    def set_enabled(self, on): pass
    def neutral(self): pass
    def set_command(self, cv): self.sent.append(cv)
    def set_gimbal_tilt_pct(self, t): self.tilt_only = t

    def get_status(self):
        b = types.SimpleNamespace(gate_on=True, script_period_ms=30, sticks_active=True,
                                  throttle_active=False, throttle_delta=0)
        return types.SimpleNamespace(state="ACTIVE", bridge_detected=True, bridge=b,
                                     rtt_avg_ms=60.0, radio_channels_valid=self.radio_valid,
                                     radio_channels_us=self.radio_ch, frames_sent=1,
                                     write_errors=0)


class _Radio:
    def get_latest_state(self):
        return types.SimpleNamespace(
            armed=True, flight_mode_name="ANGLE", baro_valid=True, baro_altitude_cm=1000,
            baro_vario_cm_per_sec=0,
            radio=types.SimpleNamespace(status_str="TELEMETRY_OK", baro_age_ms=100))


def test_controller_gimbal():
    servo = {"v": None}
    cmd = _Cmd()
    c = UplinkController(cmd, _Radio(), backend=types.SimpleNamespace(CommandValues=_Values),
                         log_dir=tempfile.mkdtemp(), bench=False,
                         servo_feedback=lambda: servo["v"])
    c.set_gimbal(pan_deg=20, tilt_deg=90)
    c._tick(0.04, 0.0)
    pan_pct, tilt_pct = c.gimbal.command_pct()
    check(cmd.gimbal == (pan_pct, tilt_pct), "disabled: gimbal still sent (set_gimbal_pct)")
    near(pan_pct, 200 / US_PER_PCT, 1e-6, "pan 20 deg = +200 us = 39 %")
    near(tilt_pct, 450 / US_PER_PCT, 1e-6, "tilt 90 deg = +450 us")
    g = c.get_status()["gimbal"]
    check(g["source"] == "radio", f"radio echo used when no USB ({g['source']})")

    c.set_enabled(True)
    c.ui_heartbeat(0.04)
    c._tick(0.04, 0.04)
    cv = cmd.sent[-1]
    check(cv.gimbal_pan_pct == pan_pct and cv.gimbal_tilt_pct == tilt_pct,
          "enabled: gimbal travels in the full command")

    servo["v"] = [1950, 1700, 0, 0, 0, 0, 0, 0]
    c.ui_heartbeat(0.08)
    c._tick(0.04, 0.08)
    check(c.get_status()["gimbal"]["source"] == "usb", "MSP_SERVO feedback preferred")

    c.nudge_gimbal(d_pan=100)                       # clamped to +45
    check(c.gimbal.target()[0] == 45.0, "nudge clamped to the soft limit")

    old_cmd = _Cmd(with_pan=False)
    o = UplinkController(old_cmd, _Radio(), backend=types.SimpleNamespace(CommandValues=_OldValues),
                         log_dir=tempfile.mkdtemp(), bench=False)
    o._tick(0.04, 0.0)
    check(old_cmd.tilt_only is not None, "old backend: tilt only, no crash")
    o.set_enabled(True)
    o.ui_heartbeat(0.04)
    o._tick(0.04, 0.04)
    check(old_cmd.sent and not o.get_status()["gimbal"]["pan_supported"],
          "old CommandValues without gimbal_pan_pct: pan disabled, command still sent")
    _assert_clean()


if __name__ == "__main__":
    for fn in (test_axis_conversions, test_servo_model, test_feedback_sources,
               test_snapshot_fields, test_controller_gimbal):
        try:
            fn()
        except AssertionError:
            pass
    print(f"{PASS} passed, {FAIL} failed")
    sys.exit(1 if FAIL else 0)
