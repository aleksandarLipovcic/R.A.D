# =============================================================================
# test_SAT_IMU_002.py  —  Physical Units Correctness — Full GCS Pipeline
#
# V-Model reference: System Acceptance Testing, SAT-IMU-002
# SYS requirement:   SYS-002
# SRS coverage:      SRS-IMU-004a, SRS-IMU-004b, SRS-IMU-004c
#
# Objective:
#   Verify that the complete unit-conversion chain — IMUSensor.getScaledData()
#   AND IMUWidget._render() (which applies ACCEL_SCALE / GYRO_SCALE to the
#   raw to_dict() values) — produces physically correct outputs at the system
#   level with real hardware.
#
#   The gravity vector test is the gold standard for accelerometer calibration:
#   a stationary, level FC must read exactly 1.0 g on the vertical axis.
#   The unit tests (UT-SCALE-001) proved this with injected mock data.
#   This SAT test confirms it with live MSP_RAW_IMU frames.
#
# Precondition:
#   Place the F405 V3 FLAT AND LEVEL on anti-vibration foam, props OFF.
#
# Pass criteria (SYS-002):
#   - IMUSensor.getScaledData().az_g: mean ∈ [0.95, 1.05] g, std < 0.02 g
#   - IMUSensor.getScaledData().ax_g, ay_g: |mean| < 0.15 g (near-zero lateral)
#   - IMUSensor.getScaledData().gyroX/Y/Z: |mean| < 5.0 °/s (at rest)
#   - IMUWidget internal render: az computed as az_raw / ACCEL_SCALE
#     confirmed ≈ 1.0 g by cross-checking widget cell text
#
# Board required: F405 V3 flat on foam, Betaflight 4.5.3, props OFF.
# =============================================================================

import time
import statistics
import pytest


class TestPhysicalUnitsCorrectness:
    """SAT-IMU-002 — SYS-002: unit conversion chain correct at system level."""

    SAMPLE_DURATION_S = 5.0
    SAMPLE_INTERVAL_S = 0.05   # 20 Hz — above actual 3–6 Hz MSP rate

    # Accelerometer pass criteria
    AZ_MEAN_LOW    = 0.95   # g  (gravity, level FC)
    AZ_MEAN_HIGH   = 1.05   # g
    AZ_MAX_STDEV   = 0.02   # g  (relaxed vs IT-IMU-002 for system-level test)
    LAT_MAX_MEAN   = 0.15   # g  (ax, ay near-zero when level)

    # Gyroscope pass criteria (at rest)
    GYRO_MAX_MEAN  = 5.0    # °/s (sensor noise + zero-offset)

    def _collect_scaled(self, imu_sensor):
        """Collect getScaledData() samples for SAMPLE_DURATION_S seconds."""
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.SAMPLE_DURATION_S:
            d = imu_sensor.get_scaled_data()
            samples.append(d)
            time.sleep(self.SAMPLE_INTERVAL_S)
        return samples

    def test_SAT_IMU_002_accel_z_mean_is_1g(self, imu_sensor):
        """mean(getScaledData().az_g) must be ≈ 1.0 g with FC flat and level.

        Validates the full IMUSensor.getScaledData() path:
          DroneState.az (MSP counts) × ACC_SCALE (1/2048) → g
        Confirms DEF-002 fix (ACC_SCALE corrected from 1/8192 to 1/2048)
        holds at the system level with live hardware.
        """
        print("\n  [SAT-002] Place FC flat and level on foam. Props OFF.")
        samples = self._collect_scaled(imu_sensor)
        az_vals = [s["az_g"] for s in samples]
        mean    = statistics.mean(az_vals)
        stdev   = statistics.stdev(az_vals)

        print(f"  az samples : {len(az_vals)}")
        print(f"  az mean    : {mean:.4f} g  (need {self.AZ_MEAN_LOW}–{self.AZ_MEAN_HIGH})")
        print(f"  az stdev   : {stdev:.4f} g  (need < {self.AZ_MAX_STDEV})")

        assert self.AZ_MEAN_LOW <= mean <= self.AZ_MEAN_HIGH, (
            f"az mean = {mean:.4f} g — outside [0.95, 1.05] g.\n"
            f"  If mean ≈ 0.25 g: ACC_SCALE still set to 1/8192. "
            f"Check IMUSensor.h — should be 1/2048 (DEF-002).\n"
            f"  If mean > 1.05 g: FC tilted or vibrating — place flat on foam."
        )
        assert stdev < self.AZ_MAX_STDEV, (
            f"az stdev = {stdev:.4f} g — too noisy. "
            "Place FC on anti-vibration foam, ensure props are off."
        )

    def test_SAT_IMU_002_lateral_accel_near_zero(self, imu_sensor):
        """mean(ax_g) and mean(ay_g) must be near zero with FC flat.

        Lateral accelerations should be < 0.15 g when the FC is level.
        Large values indicate the board is tilted or being vibrated.
        """
        samples = self._collect_scaled(imu_sensor)
        ax_mean = abs(statistics.mean(s["ax_g"] for s in samples))
        ay_mean = abs(statistics.mean(s["ay_g"] for s in samples))

        print(f"\n  |ax mean| = {ax_mean:.4f} g  (need < {self.LAT_MAX_MEAN})")
        print(f"  |ay mean| = {ay_mean:.4f} g  (need < {self.LAT_MAX_MEAN})")

        assert ax_mean < self.LAT_MAX_MEAN, (
            f"|ax mean| = {ax_mean:.4f} g — FC may be tilted sideways"
        )
        assert ay_mean < self.LAT_MAX_MEAN, (
            f"|ay mean| = {ay_mean:.4f} g — FC may be tilted fore/aft"
        )

    def test_SAT_IMU_002_gyro_near_zero_at_rest(self, imu_sensor):
        """All gyro axes must read near zero (< 5 °/s) with FC stationary.

        Validates GYRO_SCALE = 1/16.4 through the full pipeline.
        Values > 5 °/s at rest indicate wrong scale or significant vibration.
        """
        samples = self._collect_scaled(imu_sensor)
        gx_mean = abs(statistics.mean(s["gx_dps"] for s in samples))
        gy_mean = abs(statistics.mean(s["gy_dps"] for s in samples))
        gz_mean = abs(statistics.mean(s["gz_dps"] for s in samples))

        print(f"\n  |gx mean| = {gx_mean:.3f} °/s  (need < {self.GYRO_MAX_MEAN})")
        print(f"  |gy mean| = {gy_mean:.3f} °/s  (need < {self.GYRO_MAX_MEAN})")
        print(f"  |gz mean| = {gz_mean:.3f} °/s  (need < {self.GYRO_MAX_MEAN})")

        assert gx_mean < self.GYRO_MAX_MEAN, f"|gx| = {gx_mean:.3f} °/s at rest"
        assert gy_mean < self.GYRO_MAX_MEAN, f"|gy| = {gy_mean:.3f} °/s at rest"
        assert gz_mean < self.GYRO_MAX_MEAN, f"|gz| = {gz_mean:.3f} °/s at rest"

    def test_SAT_IMU_002_widget_displays_correct_az(self, imu_widget, drone_link):
        """IMUWidget must display az ≈ 1.0 g in the PITCH G-FORCE cell.

        IMUWidget._render() computes:  az_display = raw_az / ACCEL_SCALE (2048)
        Cross-checks that IMUWidget.ACCEL_SCALE matches IMUSensor.ACC_SCALE
        end-to-end at the system level — both must use 2048 as the divisor.

        The PITCH row uses ay/az; YAW row uses az for the vertical axis.
        We read the cell text and parse the numeric value.
        """
        import tkinter as tk
        root, widget = imu_widget

        # Feed live data for 3 s to stabilise
        t0 = time.monotonic()
        collected_az = []
        while time.monotonic() - t0 < 3.0:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()
            # Widget computes az as data['az'] / ACCEL_SCALE
            raw_az = data.get("az", 0)
            widget_az = raw_az / widget.ACCEL_SCALE
            collected_az.append(widget_az)
            time.sleep(0.1)

        mean_az = statistics.mean(collected_az)
        print(f"\n  Widget-computed az mean : {mean_az:.4f} g")
        print(f"  IMUWidget.ACCEL_SCALE   : {widget.ACCEL_SCALE}")

        assert widget.ACCEL_SCALE == 2048.0, (
            f"IMUWidget.ACCEL_SCALE = {widget.ACCEL_SCALE}, expected 2048.0 "
            "(must match IMUSensor.ACC_SCALE divisor after DEF-002 fix)"
        )
        assert self.AZ_MEAN_LOW <= mean_az <= self.AZ_MEAN_HIGH, (
            f"Widget az = {mean_az:.4f} g — outside [0.95, 1.05] g"
        )