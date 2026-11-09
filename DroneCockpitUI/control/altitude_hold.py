"""
altitude_hold.py  --  laptop-side altitude hold (baro altitude -> throttle)
===========================================================================
Pure logic: no Tk, no threads, no DroneBackend. UplinkController calls
update() at a fixed rate with the ELRS telemetry it already has.

Why a regulator and not a fixed throttle
----------------------------------------
A fixed throttle does not hold height: as the pack voltage sags the same
throttle gives less thrust and the drone slowly sinks. This controller keeps
the baro altitude at a target by adjusting throttle around the hover value
that was being flown when it was engaged (bumpless hand-over):

    u = hover_us + Kp * e + I - Kd * vz          e = target - altitude
    I += Ki * e * dt   (clamped, frozen while the output is saturated)
    u  clamped to hover_us +- band_us

Limits that keep it safe
------------------------
  * band_us (default 100 us) bounds the throttle around the engage value.
    The Lua bridge on the radio bounds it again, +-THR_BAND around the
    pilot's stick, so the laptop can never go far from what the pilot holds.
  * The target moves at most target_rate_mps, so "climb 2 m" is a ramp.
  * Stale altitude (no baro for stale_ms) freezes the output and reports
    stale; UplinkController drops the hold if it stays stale.

Landing
-------
start_landing() turns the same loop into a controlled descent: the target
moves down at land_rate_mps (set by the caller every step, see
safety.land_rate) and never more than LAND_LEAD_M below the measured
altitude, so nothing winds up while the throttle band limits the descent.
Touchdown = throttle at the bottom of the band while |vz| stays below
touch_vz_mps for touch_s: then `landed` is set and the output stays at the
minimum (the pilot lowers the stick and disarms; the laptop cannot disarm).
stop_landing() holds the altitude where the drone is.

Gains are STARTING values for the rod test rig and must be tuned there (see
UPUTSTVO.md, ALT-1..ALT-3). Units: metres, m/s, microseconds.
"""

from dataclasses import dataclass


@dataclass
class AltitudeHoldConfig:
    kp_us_per_m: float = 40.0        # 1 m error -> 40 us
    ki_us_per_m_s: float = 8.0       # integral gain (us per metre-second)
    kd_us_per_mps: float = 25.0      # damping on vertical speed
    band_us: float = 100.0           # max |output - hover_us|
    i_limit_us: float = 60.0         # integral clamp
    target_rate_mps: float = 0.5     # max target change speed
    stale_ms: int = 800              # altitude older than this = stale
    vz_filter_tau_s: float = 0.3     # low-pass when vz is derived from altitude
    land_lead_m: float = 2.0         # landing target at most this below the altitude
    touch_vz_mps: float = 0.25       # touchdown: |vz| below this ...
    touch_s: float = 2.0             # ... for this long, throttle at the minimum


class AltitudeHold:
    def __init__(self, cfg: AltitudeHoldConfig | None = None) -> None:
        self.cfg = cfg or AltitudeHoldConfig()
        self.engaged = False
        self.hover_us = 1500.0
        self.target_m = 0.0          # ramped target actually used
        self.request_m = 0.0         # where the user wants to be
        self.integral_us = 0.0
        self.output_us = 1500.0
        self.stale = False
        self._last_alt = None
        self._vz_est = 0.0
        self.landing = False
        self.landed = False
        self.land_rate_mps = 0.7
        self._still_s = 0.0

    # ── engage / disengage ─────────────────────────────────────────────────
    def engage(self, hover_us: float, altitude_m: float) -> None:
        """Start holding the current altitude from the current throttle."""
        self.engaged = True
        self.hover_us = float(hover_us)
        self.target_m = self.request_m = float(altitude_m)
        self.integral_us = 0.0
        self.output_us = self.hover_us
        self.stale = False
        self._last_alt = float(altitude_m)
        self._vz_est = 0.0
        self.landing = self.landed = False
        self._still_s = 0.0

    def disengage(self) -> None:
        self.engaged = False
        self.integral_us = 0.0
        self.landing = self.landed = False

    # ── landing ────────────────────────────────────────────────────────────
    def start_landing(self) -> None:
        if self.engaged and not self.landing:
            self.landing, self.landed, self._still_s = True, False, 0.0

    def stop_landing(self) -> None:
        """Abort: hold the altitude the drone is at now."""
        if self.landing:
            self.landing = self.landed = False
            here = self._last_alt if self._last_alt is not None else self.target_m
            self.target_m = self.request_m = here

    # ── target ─────────────────────────────────────────────────────────────
    def set_target(self, altitude_m: float) -> None:
        self.request_m = float(altitude_m)

    def nudge_target(self, delta_m: float) -> None:
        self.request_m += float(delta_m)

    # ── control step ───────────────────────────────────────────────────────
    def update(self, altitude_m: float, dt_s: float, altitude_age_ms: int,
               vz_mps: float | None = None) -> float:
        """One control step. Returns the throttle in microseconds."""
        c = self.cfg
        if not self.engaged:
            return self.output_us
        dt = max(0.0, min(dt_s, 0.2))

        self.stale = altitude_age_ms < 0 or altitude_age_ms > c.stale_ms
        if self.stale or dt == 0.0:
            return self.output_us        # freeze: no integration on old data

        if self.landed:
            self.output_us = self.hover_us - c.band_us
            return self.output_us
        if self.landing:
            # descend: target moves down, never far below where the drone is
            self.target_m = max(self.target_m - self.land_rate_mps * dt,
                                altitude_m - c.land_lead_m)
            self.request_m = self.target_m
        else:
            # Ramp the target toward the request.
            step = c.target_rate_mps * dt
            diff = self.request_m - self.target_m
            self.target_m += max(-step, min(step, diff))

        # Vertical speed: telemetry vario if given, else filtered derivative.
        if self._last_alt is not None:
            raw = (altitude_m - self._last_alt) / dt
            a = dt / (c.vz_filter_tau_s + dt)
            self._vz_est += a * (raw - self._vz_est)
        self._last_alt = altitude_m
        vz = self._vz_est if vz_mps is None else vz_mps

        err = self.target_m - altitude_m
        p = c.kp_us_per_m * err
        d = -c.kd_us_per_mps * vz
        unsat = self.hover_us + p + self.integral_us + d
        lo, hi = self.hover_us - c.band_us, self.hover_us + c.band_us

        # Anti-windup: integrate only if it doesn't push further into a limit.
        pushing_up = unsat >= hi and err > 0
        pushing_down = unsat <= lo and err < 0
        if not (pushing_up or pushing_down):
            self.integral_us += c.ki_us_per_m_s * err * dt
            self.integral_us = max(-c.i_limit_us, min(c.i_limit_us, self.integral_us))

        u = self.hover_us + p + self.integral_us + d
        self.output_us = max(lo, min(hi, u))

        if self.landing:
            at_min = self.output_us <= lo + 0.2 * c.band_us
            if at_min and abs(vz) < c.touch_vz_mps:
                self._still_s += dt
                if self._still_s >= c.touch_s:
                    self.landed = True
                    self.output_us = lo
            else:
                self._still_s = 0.0
        return self.output_us
