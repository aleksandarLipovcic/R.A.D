"""
gimbal.py -- 2-axis camera gimbal: commands in degrees, angles for georeferencing

Hardware (Project R.A.D.)
-------------------------
Two MG90S servos on a pan/tilt bracket, powered from a separate 5 V
step-down, signals from the flight controller outputs:

    M5 (resource SERVO 1 = A01) -> Betaflight servo 0 = GIMBAL_PITCH -> TILT
    M6 (resource SERVO 2 = A08) -> Betaflight servo 1 = GIMBAL_ROLL  -> PAN

The bracket's base plate is mounted VERTICALLY at the nose, so the base
servo turns about the drone's forward axis: looking at the drone from the
front it swings 90 deg one way and 90 deg the other. That is a ROLL axis,
not a classic yaw-pan axis -- with the camera tilted down it sweeps the view
from the left side to the right side. The tilt servo rides on it and turns
the camera from "looking forward" (0 deg) to "looking straight down"
(+90 deg). Set GimbalModel(axis="yaw") for a classic flat-mounted pan/tilt.

Command path
------------
    degrees --(calibration)--> servo pulse us --> channel % --> CommandLink
    CH9 (pan) / CH8 (tilt) --> radcmd.lua (Pan/Tlt) --> Pocket mixer
    --> ELRS --> Betaflight "forward from channel" --> servo pulse on M6/M5

Betaflight copies the forwarded channel value straight to the servo
(rcData, then the servo's min/max), so 1 % of channel = 5.12 us of pulse
and the reachable pulse range is whatever the radio sends: ~988..2012 us
with normal +-100 % limits. With ~10 us per degree (MG90S) that is about
100 deg of travel per axis, NOT the full 180 deg of the bracket -- see the
manual (docs/modules/command-uplink-uputstvo.md, gimbal chapter) for how to
place that window (tilt 0..90, pan +-45) and how to calibrate.

Angles used for georeferencing
------------------------------
The servo itself reports nothing, so the angle is reconstructed from the
best available pulse width, in this order:

    "usb"   MSP_SERVO from Betaflight (USB cable): the pulse really output
    "radio" the channel value the Pocket transmits (RAD bridge echo)
    "cmd"   what the cockpit commands (no feedback at all)

and then passed through a simple servo model (rate limit + dead time),
because an MG90S needs ~0.15-0.3 s for a large move. While the model says
the gimbal is still moving, `moving` is True and DetectionLink holds each
object's last position instead of fusing rays from a moving camera.

Pure logic, no Tk, no backend import: fully unit-tested
(tests/unit/test_gimbal.py). Thread-safe: the Tk thread commands, the
UplinkController thread updates, the DetectionLink thread reads.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional, Sequence

US_PER_PCT = 5.12            # 1 % of a channel = 5.12 us (mixer 100 % = 1024 = 512 us)
CHANNEL_CENTER_US = 1500.0


@dataclass
class GimbalAxisConfig:
    """Calibration of one servo axis.

    Measure it on the bench (manual, gimbal chapter): command two pulse
    widths, measure the two angles with a protractor/phone, then
        us_per_deg = (us2 - us1) / (deg2 - deg1)      (sign -> direction)
        center_us  = us1 - deg1 * us_per_deg
    """
    name: str
    center_us: float = 1500.0        # pulse at which this axis is at 0 deg
    us_per_deg: float = 10.0         # MG90S: ~10 us/deg (500..2400 us ~ 180 deg)
    direction: int = 1               # +1: more us = more positive angle
    min_deg: float = -45.0           # soft limits (mechanical stops!)
    max_deg: float = 45.0
    channel_min_us: float = 988.0    # what Betaflight receives at -100 % / +100 %
    channel_max_us: float = 2012.0
    servo_index: int = 0             # index in Betaflight's servo[] (MSP_SERVO)
    channel_index: int = 7           # 0-based radio channel (CH8 = 7)

    # ---- conversions --------------------------------------------------------
    def deg_to_us(self, deg: float) -> float:
        return self.center_us + self.direction * deg * self.us_per_deg

    def us_to_deg(self, us: float) -> float:
        return (us - self.center_us) / (self.direction * self.us_per_deg)

    def reachable(self) -> tuple:
        """(lo_deg, hi_deg): soft limits intersected with what the channel
        range can reach."""
        a = self.us_to_deg(self.channel_min_us)
        b = self.us_to_deg(self.channel_max_us)
        lo, hi = min(a, b), max(a, b)
        return max(lo, self.min_deg), min(hi, self.max_deg)

    def clamp(self, deg: float) -> float:
        lo, hi = self.reachable()
        return max(lo, min(hi, float(deg)))

    def deg_to_pct(self, deg: float) -> float:
        pct = (self.deg_to_us(self.clamp(deg)) - CHANNEL_CENTER_US) / US_PER_PCT
        return max(-100.0, min(100.0, pct))


def default_pan_config() -> GimbalAxisConfig:
    # Base servo, M6 -> Betaflight servo 1 (GIMBAL_ROLL), radio CH9.
    # Horn placed so that 1500 us = camera centred.
    return GimbalAxisConfig("pan", center_us=1500.0, us_per_deg=10.0, direction=1,
                            min_deg=-45.0, max_deg=45.0, servo_index=1, channel_index=8)


def default_tilt_config() -> GimbalAxisConfig:
    # Tilt servo, M5 -> Betaflight servo 0 (GIMBAL_PITCH), radio CH8.
    # Horn placed so that 1500 us = 45 deg (half way down): the ~100 deg
    # window then covers 0 deg (forward) .. 90 deg (straight down).
    return GimbalAxisConfig("tilt", center_us=1050.0, us_per_deg=10.0, direction=1,
                            min_deg=0.0, max_deg=90.0, servo_index=0, channel_index=7)


@dataclass
class ServoModelConfig:
    rate_dps: float = 300.0          # MG90S under load, 5 V (no load ~600 deg/s)
    dead_time_s: float = 0.08        # radio path + Betaflight + servo start
    settle_deg: float = 1.5          # |target - estimate| below this = settled
    feedback_timeout_s: float = 0.5  # a feedback source older than this is dropped


@dataclass
class GimbalAngles:
    pan_deg: float = 0.0             # estimated actual angles
    tilt_deg: float = 0.0
    target_pan_deg: float = 0.0      # commanded
    target_tilt_deg: float = 0.0
    moving: bool = False
    source: str = "cmd"              # "usb" / "radio" / "cmd"
    axis: str = "roll"               # mount type of the pan axis

    def snapshot_fields(self) -> dict:
        """Fields for DroneBackend.TelemetrySnapshot."""
        roll = self.pan_deg if self.axis == "roll" else 0.0
        pan = self.pan_deg if self.axis == "yaw" else 0.0
        return {"gimbal_roll_deg": roll, "gimbal_pan_deg": pan,
                "gimbal_tilt_deg": self.tilt_deg, "gimbal_moving": self.moving}


class _AxisState:
    def __init__(self, start_deg: float) -> None:
        self.est = start_deg                 # estimated actual angle
        self.pulse_deg = start_deg           # angle of the pulse the servo gets
        self.pulse_t = 0.0                   # when that pulse value last changed


class GimbalModel:
    def __init__(self, pan: Optional[GimbalAxisConfig] = None,
                 tilt: Optional[GimbalAxisConfig] = None,
                 servo: Optional[ServoModelConfig] = None,
                 axis: str = "roll") -> None:
        if axis not in ("roll", "yaw"):
            raise ValueError("axis must be 'roll' or 'yaw'")
        self.pan_cfg = pan or default_pan_config()
        self.tilt_cfg = tilt or default_tilt_config()
        self.servo = servo or ServoModelConfig()
        self.axis = axis
        self._lock = threading.Lock()
        self._target_pan = self.pan_cfg.clamp(0.0)
        self._target_tilt = self.tilt_cfg.clamp(0.0)
        self._pan = _AxisState(self._target_pan)
        self._tilt = _AxisState(self._target_tilt)
        self._t = None                       # time of the last update()
        self._source = "cmd"

    # ---- commands (Tk thread) ---------------------------------------------
    def set_target(self, pan_deg: Optional[float] = None,
                   tilt_deg: Optional[float] = None) -> None:
        with self._lock:
            if pan_deg is not None:
                self._target_pan = self.pan_cfg.clamp(pan_deg)
            if tilt_deg is not None:
                self._target_tilt = self.tilt_cfg.clamp(tilt_deg)

    def nudge(self, d_pan: float = 0.0, d_tilt: float = 0.0) -> None:
        with self._lock:
            self._target_pan = self.pan_cfg.clamp(self._target_pan + d_pan)
            self._target_tilt = self.tilt_cfg.clamp(self._target_tilt + d_tilt)

    def target(self) -> tuple:
        with self._lock:
            return self._target_pan, self._target_tilt

    def command_pct(self) -> tuple:
        """(pan_pct, tilt_pct) for CommandLink.set_gimbal_pct()."""
        with self._lock:
            return (self.pan_cfg.deg_to_pct(self._target_pan),
                    self.tilt_cfg.deg_to_pct(self._target_tilt))

    def limits(self) -> dict:
        return {"pan": self.pan_cfg.reachable(), "tilt": self.tilt_cfg.reachable()}

    # ---- feedback + servo model (UplinkController thread) -----------------
    def update(self, now: float, servo_us: Optional[Sequence[float]] = None,
               radio_us: Optional[Sequence[float]] = None) -> None:
        """servo_us: Betaflight servo outputs (MSP_SERVO, USB) or None.
        radio_us: channels the radio transmits (bridge echo) or None.
        The caller passes None for a source that is missing or stale."""
        with self._lock:
            pan_deg, tilt_deg, src = self._pulse_angles(servo_us, radio_us)
            self._source = src
            for st, deg in ((self._pan, pan_deg), (self._tilt, tilt_deg)):
                if abs(deg - st.pulse_deg) > 0.05:
                    st.pulse_deg = deg
                    st.pulse_t = now
            if self._t is not None:
                dt = max(0.0, min(0.5, now - self._t))
                for st in (self._pan, self._tilt):
                    st.est = self._advance(st, now, dt)
            self._t = now

    def angles(self, now: Optional[float] = None) -> GimbalAngles:
        """Estimated angles, extrapolated to `now` (any thread)."""
        with self._lock:
            pan, tilt = self._pan.est, self._tilt.est
            if now is not None and self._t is not None:
                dt = max(0.0, min(0.5, now - self._t))
                pan = self._advance(self._pan, now, dt)
                tilt = self._advance(self._tilt, now, dt)
            moving = any(self._moving(st, est, now)
                         for st, est in ((self._pan, pan), (self._tilt, tilt)))
            return GimbalAngles(pan_deg=pan, tilt_deg=tilt,
                                target_pan_deg=self._target_pan,
                                target_tilt_deg=self._target_tilt,
                                moving=moving, source=self._source, axis=self.axis)

    # ---- internals (lock held) ---------------------------------------------
    def _pulse_angles(self, servo_us, radio_us):
        p, t = self.pan_cfg, self.tilt_cfg
        if servo_us is not None and len(servo_us) > max(p.servo_index, t.servo_index):
            sp, stl = servo_us[p.servo_index], servo_us[t.servo_index]
            if sp > 0 and stl > 0:                 # 0 = servo output not driven
                return p.us_to_deg(sp), t.us_to_deg(stl), "usb"
        if radio_us is not None and len(radio_us) > max(p.channel_index, t.channel_index):
            return (p.us_to_deg(radio_us[p.channel_index]),
                    t.us_to_deg(radio_us[t.channel_index]), "radio")
        return self._target_pan, self._target_tilt, "cmd"

    def _advance(self, st: _AxisState, now: float, dt: float) -> float:
        # Dead time: the servo starts moving dead_time_s after the pulse
        # changed. Only the part of dt after that counts.
        active = min(dt, max(0.0, now - (st.pulse_t + self.servo.dead_time_s)))
        step = self.servo.rate_dps * active
        d = st.pulse_deg - st.est
        if abs(d) <= step:
            return st.pulse_deg
        return st.est + (step if d > 0 else -step)

    def _moving(self, st: _AxisState, est: float, now: Optional[float]) -> bool:
        if abs(st.pulse_deg - est) > self.servo.settle_deg:
            return True
        # a pulse change that is still inside the dead time
        return now is not None and now - st.pulse_t < self.servo.dead_time_s \
            and abs(st.pulse_deg - est) > 0.05
