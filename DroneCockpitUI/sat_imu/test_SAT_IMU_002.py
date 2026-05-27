# =============================================================================
# test_SAT_IMU_002.py  —  Physical Unit Correctness — Gravity Reference
#
# V-Model reference: System Acceptance Testing, SAT-IMU-002
# SYS requirement:   SYS-002
# SRS coverage:      SRS-IMU-004a, SRS-IMU-004b, SRS-IMU-004c
#
# Objective:
#   With the FC stationary and level, verify that the widget az cell reads
#   1.0 g ± 0.05 g and that all gyroscope cells read close to 0 °/s
#   (|gx|, |gy|, |gz| ≤ 2 °/s). Confirms the full unit-conversion chain:
#
#     Raw az  (MSP_RAW_IMU)
#       → IMUSensor.getScaledData().accZ  via ACC_SCALE = 1/2048  [SRS-IMU-004a]
#       → IMUWidget display               via ACCEL_SCALE = 2048  [SRS-IMU-004c]
#
#     Raw gx/gy/gz (MSP_RAW_IMU)
#       → IMUSensor.getScaledData().gyroX/Y/Z  via GYRO_SCALE = 1/16.4  [SRS-IMU-004b]
#       → IMUWidget display                    via GYRO_SCALE = 16.4    [SRS-IMU-004c]
#
# Scale rationale (DEF-002, corrected):
#   BF MSP_RAW_IMU pre-divides accel ADC by 4 before transmission.
#   Effective MSP-layer sensitivity = 2048 LSB/g  (not raw 8192 LSB/g).
#   ACC_SCALE = 1/2048. Gyroscope is transmitted at full hardware sensitivity:
#   16.4 LSB/(°/s) at ±2000 °/s.
#
# Preconditions:
#   - F405 V3 flat and LEVEL on anti-vibration foam
#   - Props OFF, no vibration sources nearby
#   - Allow 5 s for values to stabilise before reading
#
# Pass Criteria (SYS-002):
#   - mean(accZ)     in [0.95, 1.05] g over 10 s (100+ samples)
#   - std(accZ)      < 0.01 g  (sensor noise floor; stationary FC)
#   - mean(|gyroX|)  ≤ 2.0 °/s
#   - mean(|gyroY|)  ≤ 2.0 °/s
#   - mean(|gyroZ|)  ≤ 2.0 °/s
#   - Widget-computed az (raw_az / ACCEL_SCALE) matches imu_sensor.accZ ± 0.01 g
#
# Run:
#   pytest sat_tests/test_SAT_IMU_002.py -v -s
# =============================================================================

import ctypes
import time
import winsound

import pytest


# ── Operator notification helpers (same pattern as SAT-IMU-003) ───────────────

_MB_OK              = 0x00
_MB_ICONINFORMATION = 0x40
_MB_ICONWARNING     = 0x30
_MB_TOPMOST         = 0x40000
_MB_SETFOREGROUND   = 0x10000


def _msgbox(title: str, message: str, icon: int = _MB_ICONINFORMATION) -> None:
    try:
        ctypes.windll.user32.MessageBoxW(
            0, message, title,
            _MB_OK | icon | _MB_TOPMOST | _MB_SETFOREGROUND
        )
    except AttributeError:
        print(f"\n[OPERATOR PROMPT] {title}\n{message}")


def _beep() -> None:
    try:
        winsound.Beep(880, 200)
        time.sleep(0.05)
        winsound.Beep(1100, 300)
    except Exception:
        pass


# ── Constants ─────────────────────────────────────────────────────────────────

_STABILISE_S    = 5.0    # wait before collecting samples
_COLLECT_S      = 10.0   # collection window
_POLL_INTERVAL  = 0.1    # 10 Hz — well above 3–6 Hz actual MSP rate
_ACC_MEAN_LO    = 0.95   # g — lower acceptance bound
_ACC_MEAN_HI    = 1.05   # g — upper acceptance bound
_ACC_STD_MAX    = 0.01   # g — maximum standard deviation (stationary FC)
_GYRO_MAX_DPS   = 2.0    # °/s — maximum gyro magnitude at rest


class TestGravityReference:
    """SAT-IMU-002 — SYS-002: Physical unit correctness against gravity vector."""

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _collect_samples(self, drone_link, imu_widget, imu_sensor,
                         duration_s: float) -> dict[str, list]:
        root, widget = imu_widget
        results: dict[str, list] = {
            "acc_z": [], "gyro_x": [], "gyro_y": [], "gyro_z": [],
            "widget_az": [],
        }
        t0 = time.monotonic()

        while time.monotonic() - t0 < duration_s:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()

            # FIX 1: snake_case method name matches pybind11 binding
            # FIX 2: binding returns a dict, not an attribute object
            #        keys are "az_g", "gx_dps", "gy_dps", "gz_dps"
            scaled = imu_sensor.get_scaled_data()
            results["acc_z"].append(scaled["az_g"])
            results["gyro_x"].append(abs(scaled["gx_dps"]))
            results["gyro_y"].append(abs(scaled["gy_dps"]))
            results["gyro_z"].append(abs(scaled["gz_dps"]))

            # Widget-computed az: what the widget displays (SRS-IMU-004c)
            raw_az   = data.get("az", 0)
            widget_g = raw_az / widget.ACCEL_SCALE
            results["widget_az"].append(widget_g)

            time.sleep(_POLL_INTERVAL)

        return results

    @staticmethod
    def _mean(values: list[float]) -> float:
        return sum(values) / len(values) if values else 0.0

    @staticmethod
    def _std(values: list[float]) -> float:
        if len(values) < 2:
            return 0.0
        m = sum(values) / len(values)
        variance = sum((v - m) ** 2 for v in values) / (len(values) - 1)
        return variance ** 0.5

    # ── Tests ─────────────────────────────────────────────────────────────────

    def test_SAT_IMU_002_setup_prompt(self, drone_link, imu_widget, imu_sensor):
        """Operator placement prompt — must pass before gravity tests run.

        Not a true assertion test; ensures operator has placed the FC correctly
        before the measurement tests collect samples.
        """
        _beep()
        _msgbox(
            "SAT-IMU-002 — Setup Required",
            "Place the F405 V3 FLAT and LEVEL on anti-vibration foam.\n\n"
            "Orientation: Z axis pointing UP (USB connector horizontal).\n\n"
            f"Allow {_STABILISE_S:.0f} s for values to stabilise after clicking OK.",
            _MB_ICONWARNING,
        )

        root, widget = imu_widget
        print(f"\n  [SAT-002] Stabilising for {_STABILISE_S:.0f} s...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < _STABILISE_S:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()
            time.sleep(_POLL_INTERVAL)

        # This test always passes — it is purely a setup gate.
        assert True

    def test_SAT_IMU_002_accZ_mean_in_range(self, drone_link, imu_widget, imu_sensor):
        """Mean accZ must be in [0.95, 1.05] g with FC stationary and level.

        Validates ACC_SCALE = 1/2048 (SRS-IMU-004a, DEF-002):
          raw_az ≈ 2048 counts → 2048 × (1/2048) = 1.000 g.
        Cross-validated against IT-IMU-002: mean = 1.001 g (475 samples).
        """
        samples = self._collect_samples(
            drone_link, imu_widget, imu_sensor, _COLLECT_S
        )
        mean_g = self._mean(samples["acc_z"])
        n      = len(samples["acc_z"])

        print(f"\n  [SAT-002] accZ  mean={mean_g:.4f} g  "
              f"n={n}  range=[{_ACC_MEAN_LO}, {_ACC_MEAN_HI}] g")

        assert _ACC_MEAN_LO <= mean_g <= _ACC_MEAN_HI, (
            f"accZ mean {mean_g:.4f} g is outside acceptance band "
            f"[{_ACC_MEAN_LO}, {_ACC_MEAN_HI}] g.\n"
            f"  Samples: {n}\n"
            "  Common causes:\n"
            "    • ACC_SCALE still set to 1/8192 (raw ADC) instead of 1/2048 "
            "(MSP-layer) — see DEF-002\n"
            "    • FC not level — check bubble level\n"
            "    • Accelerometer calibration not performed in Betaflight"
        )

    def test_SAT_IMU_002_accZ_std_within_noise_floor(
            self, drone_link, imu_widget, imu_sensor):
        """AccZ standard deviation must be < 0.01 g (stationary FC).

        Validates that the sensor is not saturating, oscillating, or producing
        noise beyond the MPU-6500 datasheet noise floor.
        """
        samples = self._collect_samples(
            drone_link, imu_widget, imu_sensor, _COLLECT_S
        )
        std_g = self._std(samples["acc_z"])

        print(f"\n  [SAT-002] accZ  std={std_g:.5f} g  (limit < {_ACC_STD_MAX} g)")

        assert std_g < _ACC_STD_MAX, (
            f"accZ std {std_g:.5f} g exceeds {_ACC_STD_MAX} g noise floor.\n"
            "  → Check for vibration sources. Place FC on foam, not rigid surface."
        )

    def test_SAT_IMU_002_gyro_at_rest(self, drone_link, imu_widget, imu_sensor):
        """Mean |gyroX|, |gyroY|, |gyroZ| must each be ≤ 2 °/s at rest.

        Validates GYRO_SCALE = 1/16.4 (SRS-IMU-004b): stationary MPU-6500
        gyroscope bias drift is typically < 1 °/s. The 2 °/s limit is a
        generous acceptance band that covers sensor-to-sensor variation.
        """
        samples = self._collect_samples(
            drone_link, imu_widget, imu_sensor, _COLLECT_S
        )
        means = {
            "gyroX": self._mean(samples["gyro_x"]),
            "gyroY": self._mean(samples["gyro_y"]),
            "gyroZ": self._mean(samples["gyro_z"]),
        }

        for axis, mean_dps in means.items():
            print(f"\n  [SAT-002] {axis}  mean |rate| = {mean_dps:.3f} °/s  "
                  f"(limit ≤ {_GYRO_MAX_DPS} °/s)")

        violations = {
            axis: v for axis, v in means.items() if v > _GYRO_MAX_DPS
        }
        assert not violations, (
            f"Gyro axes exceed {_GYRO_MAX_DPS} °/s at rest: {violations}\n"
            "  → Confirm FC is stationary. Run gyro calibration in Betaflight\n"
            "    (set gyro_calib_dur = 3 ; gyro_calib_temperature = 0).\n"
            "  → Check GYRO_SCALE constant in IMUSensor.h: must be 1/16.4."
        )

    def test_SAT_IMU_002_widget_az_matches_sensor(
            self, drone_link, imu_widget, imu_sensor):
        """Widget-displayed az must agree with imu_sensor.accZ within ±0.01 g.

        Validates SRS-IMU-004c: the widget applies ACCEL_SCALE = 2048 to the
        raw ADC value from to_dict(), which must produce the same physical
        output as IMUSensor.getScaledData().accZ (ACC_SCALE = 1/2048).
        Both paths read from the same DroneState.az field.
        """
        samples = self._collect_samples(
            drone_link, imu_widget, imu_sensor, _COLLECT_S
        )

        mean_sensor = self._mean(samples["acc_z"])
        mean_widget = self._mean(samples["widget_az"])
        delta       = abs(mean_sensor - mean_widget)

        print(f"\n  [SAT-002] sensor accZ = {mean_sensor:.4f} g  "
              f"widget az = {mean_widget:.4f} g  "
              f"delta = {delta:.5f} g")

        assert delta < 0.01, (
            f"Widget az ({mean_widget:.4f} g) diverges from "
            f"sensor accZ ({mean_sensor:.4f} g) by {delta:.5f} g (limit < 0.01 g).\n"
            "  → IMUSensor.ACC_SCALE and IMUWidget.ACCEL_SCALE must both\n"
            "    resolve to the same 1/2048 factor. Check both constants."
        )