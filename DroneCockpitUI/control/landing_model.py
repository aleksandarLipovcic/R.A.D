"""
landing_model.py -- learn the drone's hover throttle from flight data and
compute safe landing values (laptop auto-land and Betaflight AUTO-LAND).

Pure logic (no Tk, no backend), unit-tested in tests/unit/test_safety.py.

Why learn it
------------
Betaflight's AUTO-LAND failsafe holds ONE fixed throttle (failsafe_throttle)
for failsafe_landing_time and then disarms. Too low and the drone falls, at
or above hover and it never comes down (fly-away). The right value depends
on the drone's weight (payload!) and on the battery voltage, so instead of
guessing it, the cockpit measures it whenever the drone flies steadily.

Model
-----
Thrust is roughly proportional to (duty * voltage)^2, so for the same thrust
the product (throttle_us - 1000) * V stays about constant while the pack
sags. With that normalised throttle

    u = (thr_us - 1000) * V          [us * V]

steady vertical speed near hover is close to linear:

    u = a + b * vz                    a = hover, b = slope (b > 0)

Samples are taken only when the flight is steady (level within MAX_TILT,
vertical acceleration small, throttle above idle). a and b come from an
exponentially weighted least-squares fit (older data fades with tau_s, so a
new payload is learnt within a few minutes). Without enough vertical-speed
spread the slope is the prior (prior_slope_us_per_mps); every laptop
auto-land produces descent data, so after the first one the slope is
measured too.

Failsafe throttle
-----------------
One fixed throttle has to work from a full to an empty pack. With the model:
  * at FULL voltage it must still descend at least descent_min (otherwise a
    fresh battery could make the drone climb away),
  * at EMPTY voltage (under load) it should not descend faster than
    descent_max,
  * between the two it aims at descent_target at full voltage.
failsafe_landing_time must cover the height: Betaflight disarms when it runs
out (max 250 s), so cover_m = slowest descent * landing time is reported and
the cockpit warns when the drone is higher than that above the arming point.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class LandingModelConfig:
    max_tilt_deg: float = 8.0          # only near-level flight
    max_accel_mps2: float = 0.4        # only steady vertical speed
    min_throttle_us: float = 1150.0    # airborne, not idling on the ground
    tau_s: float = 180.0               # forgetting time constant (seconds of data)
    prior_slope_us_per_mps: float = 35.0   # at the pack's voltage, until measured
    min_hover_s: float = 10.0          # steady data needed for the hover value
    min_slope_s: float = 15.0          # ... and for the slope
    min_vz_std_mps: float = 0.35       # vertical-speed spread needed for the slope
    descent_target_mps: float = 1.0
    descent_min_mps: float = 0.5
    descent_max_mps: float = 3.0
    cell_full_v: float = 4.2
    cell_empty_v: float = 3.5          # empty under hover load
    land_time_max_s: int = 250         # Betaflight failsafe_landing_time limit
    land_time_min_s: int = 10
    saved_weight_s: float = 60.0       # a loaded model counts as this much data


class LandingModel:
    def __init__(self, cfg: Optional[LandingModelConfig] = None) -> None:
        self.cfg = cfg or LandingModelConfig()
        self._sw = self._sx = self._sy = self._sxx = self._sxy = self._sv = 0.0
        self._last_t: Optional[float] = None
        self._last_vz: Optional[float] = None
        self._acc = 0.0
        self.max_alt_m = 0.0               # highest altitude above arming seen

    # ── data ────────────────────────────────────────────────────────────────
    def add(self, t: float, thr_us: Optional[float], vz_mps: Optional[float],
            volt: Optional[float], roll_deg: float = 0.0, pitch_deg: float = 0.0,
            alt_m: Optional[float] = None) -> bool:
        """One telemetry sample (25 Hz). Returns True if it was used."""
        c = self.cfg
        if alt_m is not None:
            self.max_alt_m = max(self.max_alt_m, float(alt_m))
        if thr_us is None or vz_mps is None or not volt or volt <= 3.0:
            self._last_t = None
            return False
        dt = 0.04 if self._last_t is None else max(0.0, min(0.2, t - self._last_t))
        if self._last_vz is not None and dt > 0:
            raw = (vz_mps - self._last_vz) / dt
            self._acc += (dt / (0.3 + dt)) * (raw - self._acc)    # filtered accel
        self._last_t, self._last_vz = t, vz_mps
        if (thr_us < c.min_throttle_us or abs(roll_deg) > c.max_tilt_deg
                or abs(pitch_deg) > c.max_tilt_deg or abs(self._acc) > c.max_accel_mps2
                or dt == 0.0):
            return False
        u = (thr_us - 1000.0) * volt
        k = math.exp(-dt / c.tau_s)
        self._sw = self._sw * k + dt
        self._sx = self._sx * k + dt * vz_mps
        self._sy = self._sy * k + dt * u
        self._sxx = self._sxx * k + dt * vz_mps * vz_mps
        self._sxy = self._sxy * k + dt * vz_mps * u
        self._sv = self._sv * k + dt * volt
        return True

    # ── fit ─────────────────────────────────────────────────────────────────
    @property
    def data_s(self) -> float:
        return self._sw

    @property
    def known(self) -> bool:
        return self._sw >= self.cfg.min_hover_s

    @property
    def mean_volt(self) -> float:
        return self._sv / self._sw if self._sw > 0 else 0.0

    def _prior_b(self) -> float:
        return self.cfg.prior_slope_us_per_mps * (self.mean_volt or 14.8)

    def fit(self) -> tuple:
        """(a, b, slope_measured)."""
        if self._sw <= 0:
            return 0.0, self._prior_b(), False
        mx, my = self._sx / self._sw, self._sy / self._sw
        var = self._sxx / self._sw - mx * mx
        cov = self._sxy / self._sw - mx * my
        prior = self._prior_b()
        b, measured = prior, False
        if self._sw >= self.cfg.min_slope_s and var >= self.cfg.min_vz_std_mps ** 2:
            cand = cov / var
            if 0.3 * prior <= cand <= 3.0 * prior:      # physically plausible
                b, measured = cand, True
        return my - b * mx, b, measured

    def hover_us(self, volt: float) -> Optional[float]:
        if not self.known or not volt:
            return None
        a, _, _ = self.fit()
        return 1000.0 + a / volt

    def vz_at(self, thr_us: float, volt: float) -> Optional[float]:
        """Steady vertical speed (m/s, + = up) the model predicts."""
        if not self.known or not volt:
            return None
        a, b, _ = self.fit()
        return ((thr_us - 1000.0) * volt - a) / b

    def thr_for(self, vz_mps: float, volt: float) -> float:
        a, b, _ = self.fit()
        return 1000.0 + (a + b * vz_mps) / volt

    # ── Betaflight AUTO-LAND values ─────────────────────────────────────────
    def cells(self, cell_count: int = 0) -> int:
        if cell_count and cell_count > 0:
            return int(cell_count)
        return max(1, int(round(self.mean_volt / 3.85))) if self.mean_volt else 4

    def recommend(self, cell_count: int = 0, planned_alt_m: Optional[float] = None) -> dict:
        c = self.cfg
        if not self.known:
            need = max(0.0, c.min_hover_s - self._sw)
            return {"ok": False, "data_s": round(self._sw, 1),
                    "text": f"fly {need:.0f} s more of steady hover to learn the landing throttle"}
        n = self.cells(cell_count)
        vf, ve = n * c.cell_full_v, n * c.cell_empty_v
        _, _, measured = self.fit()
        notes = []
        thr = self.thr_for(-c.descent_target_mps, vf)
        if self.vz_at(thr, ve) < -c.descent_max_mps:          # too fast on an empty pack
            thr = self.thr_for(-c.descent_max_mps, ve)
            notes.append(f"> {c.descent_max_mps:.1f} m/s would be reached on an empty pack")
        never_climb = self.thr_for(-c.descent_min_mps, vf)    # must descend on a full pack
        if thr > never_climb:
            thr = never_climb
            notes.append("full-pack descent limited to the minimum; faster when empty")
        thr = int(math.floor(thr))
        vz_f, vz_e = self.vz_at(thr, vf), self.vz_at(thr, ve)
        slowest = max(0.05, -max(vz_f, vz_e))
        alt = max(planned_alt_m or 0.0, self.max_alt_m)
        t = int(math.ceil(alt / slowest * 1.15 + 5)) if alt > 0 else c.land_time_max_s
        t = max(c.land_time_min_s, min(c.land_time_max_s, t))
        cover = slowest * (t - 5)
        if alt > cover:
            notes.append(f"AUTO-LAND covers only {cover:.0f} m (Betaflight max {c.land_time_max_s} s): "
                         "higher up it disarms in the air -- use GPS-RESCUE when GPS is on")
        if not measured:
            notes.append("descent slope is a default; one laptop auto-land measures it")
        cli = ("set failsafe_procedure = AUTO-LAND\n"
               f"set failsafe_throttle = {thr}\n"
               f"set failsafe_landing_time = {t}\n"
               "save")
        return {"ok": True, "throttle": thr, "landing_time": t, "cells": n,
                "vz_full": round(vz_f, 2), "vz_empty": round(vz_e, 2),
                "cover_m": round(cover, 0), "slope_measured": measured,
                "data_s": round(self._sw, 1), "notes": notes, "cli": cli,
                "text": (f"failsafe_throttle {thr} us, landing_time {t} s: "
                         f"{-vz_f:.1f} m/s full pack, {-vz_e:.1f} m/s empty")}

    # ── persistence (the model carries over to the next flight) ────────────
    def to_dict(self) -> dict:
        return {"sums": [self._sw, self._sx, self._sy, self._sxx, self._sxy, self._sv],
                "cfg": asdict(self.cfg)}

    def load_dict(self, d: dict) -> None:
        sums = [float(x) for x in d.get("sums", [])]
        if len(sums) != 6 or sums[0] <= 0:
            return
        scale = min(1.0, self.cfg.saved_weight_s / sums[0])
        (self._sw, self._sx, self._sy, self._sxx, self._sxy, self._sv) = [x * scale for x in sums]

    def save(self, path: str) -> None:
        try:
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.to_dict(), f)
            os.replace(tmp, path)
        except OSError:
            pass

    def load(self, path: str) -> bool:
        try:
            with open(path, encoding="utf-8") as f:
                self.load_dict(json.load(f))
            return self.known
        except (OSError, ValueError):
            return False
