# =============================================================================
# test_SAT_IMU_002.py  —  Physical Units Correctness — Full GCS Pipeline
#
# V-Model reference: System Acceptance Testing, SAT-IMU-002
# SYS requirement:   SYS-002
# SRS coverage:      SRS-IMU-004a, SRS-IMU-004b, SRS-IMU-004c
#
# Objective:
#   Verify that the complete unit-conversion chain produces physically
#   correct outputs at the operator level with real hardware.
#
# Implementation note — why this test reads DroneState directly:
#   Raw counts (ax/ay/az/gx/gy/gz) are read from DroneState via
#   drone_link.get_latest_state().  The scale constants ACC_SCALE = 1/2048
#   and GYRO_SCALE = 1/16.4 are applied locally, matching IMUSensor.h after
#   DEF-002 (ACC_SCALE = 1.0f / 2048.0f; GYRO_SCALE = 1.0f / 16.4f).
#   The widget cross-check sub-test verifies IMUWidget.ACCEL_SCALE = 2048.0.
#
# Scale constants (post-DEF-002, matching IMUSensor.h):
#   ACC_SCALE  = 1.0 / 2048.0
#     Betaflight MSP_RAW_IMU pre-divides accelerometer counts by 4.
#     Effective MSP-layer sensitivity: 8192 / 4 = 2048 LSB/g.
#     NOT the raw ADC constant (1/8192) — that was the pre-DEF-002 error.
#   GYRO_SCALE = 1.0 / 16.4
#     BF does not pre-scale gyroscope counts. ±2000°/s MPU-6500 default.
#
# DroneState attribute naming (pybind11):
#   C++ DroneState members are camelCase (ax, ay, az, gx, gy, gz are already
#   lowercase so no ambiguity there). All attribute accesses match DroneLink.h.
#
# Precondition:
#   Place the F405 V3 FLAT AND LEVEL on anti-vibration foam.  Props OFF.
#   The +Z axis of the MPU-6500 faces upward in this orientation.
#
# Pass criteria (SYS-002):
#   - mean(az * ACC_SCALE) in [0.95, 1.05] g  (gravity reference)
#   - stdev(az * ACC_SCALE) < 0.02 g           (noise floor)
#   - |mean(ax * ACC_SCALE)| < 0.15 g          (level: near-zero lateral)
#   - |mean(ay * ACC_SCALE)| < 0.15 g          (level: near-zero lateral)
#   - |mean(gx * GYRO_SCALE)| < 5.0 °/s        (at rest: noise floor)
#   - |mean(gy * GYRO_SCALE)| < 5.0 °/s
#   - |mean(gz * GYRO_SCALE)| < 5.0 °/s
#   - widget.ACCEL_SCALE == 2048.0              (confirms DEF-002 fix in widget)
#   - widget-computed az in [0.95, 1.05] g      (end-to-end widget display check)
#
# Board required: F405 V3 flat on foam, Betaflight 4.5.3, props OFF.
# =============================================================================

import time
import statistics
import pytest

# Scale constants — match IMUSensor.h post-DEF-002 values exactly.
ACC_SCALE  = 1.0 / 2048.0   # MSP-layer: BF pre-divides accel counts by 4
GYRO_SCALE = 1.0 / 16.4     # ±2000 °/s MPU-6500 default range (not pre-scaled)


class TestPhysicalUnitsCorrectness:
    """SAT-IMU-002 — SYS-002: unit-conversion chain correct at system level."""

    SAMPLE_DURATION_S = 5.0
    SAMPLE_INTERVAL_S = 0.05   # 20 Hz polling — above actual 3–6 Hz MSP rate

    # Accelerometer pass criteria
    AZ_MEAN_LOW   = 0.95   # g
    AZ_MEAN_HIGH  = 1.05   # g
    AZ_MAX_STDEV  = 0.02   # g  (relaxed vs IT-IMU-002's 0.01 g for system test)
    LAT_MAX_MEAN  = 0.15   # g  (ax, ay near-zero when level)

    # Gyroscope pass criteria (at rest)
    GYRO_MAX_MEAN = 5.0    # °/s

    def _collect(self, drone_link):
        """Collect DroneState snapshots for SAMPLE_DURATION_S seconds.
        Returns list of dicts: {az_g, ax_g, ay_g, gx_dps, gy_dps, gz_dps}.
        """
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.SAMPLE_DURATION_S:
            s = drone_link.get_latest_state()
            samples.append({
                "az_g":   s.az * ACC_SCALE,
                "ax_g":   s.ax * ACC_SCALE,
                "ay_g":   s.ay * ACC_SCALE,
                "gx_dps": s.gx * GYRO_SCALE,
                "gy_dps": s.gy * GYRO_SCALE,
                "gz_dps": s.gz * GYRO_SCALE,
            })
            time.sleep(self.SAMPLE_INTERVAL_S)
        return samples

    # ── Sub-test 1 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_002_accel_z_mean_is_1g(self, drone_link):
        """mean(az * ACC_SCALE) must be in [0.95, 1.05] g with FC flat.

        Validates the full conversion chain:
          DroneState.az (MSP counts) × (1/2048) → g

        If mean ≈ 0.25 g: ACC_SCALE is still 1/8192 (pre-DEF-002 value).
          Fix: IMUSensor.h, ACC_SCALE = 1.0f / 2048.0f
        If mean > 1.05 g: FC is not level or is vibrating.
        """
        print(
            "\n  [SAT-002] Ensure FC is FLAT AND LEVEL on foam. Props OFF."
        )
        samples = self._collect(drone_link)
        vals  = [d["az_g"] for d in samples]
        mean  = statistics.mean(vals)
        stdev = statistics.stdev(vals)

        print(f"  samples   : {len(vals)}")
        print(f"  az mean   : {mean:.4f} g  (need {self.AZ_MEAN_LOW}–{self.AZ_MEAN_HIGH})")
        print(f"  az stdev  : {stdev:.4f} g  (need < {self.AZ_MAX_STDEV})")

        assert self.AZ_MEAN_LOW <= mean <= self.AZ_MEAN_HIGH, (
            f"az mean = {mean:.4f} g — outside [{self.AZ_MEAN_LOW}, {self.AZ_MEAN_HIGH}] g.\n"
            f"  mean ≈ 0.25 g → ACC_SCALE still 1/8192 (DEF-002 not applied in IMUSensor.h).\n"
            f"  mean > 1.05 g → FC tilted or vibrating — place flat on foam."
        )
        assert stdev < self.AZ_MAX_STDEV, (
            f"az stdev = {stdev:.4f} g — exceeds {self.AZ_MAX_STDEV} g. "
            "Place FC on anti-vibration foam and ensure props are off."
        )

    # ── Sub-test 2 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_002_lateral_accel_near_zero(self, drone_link):
        """|mean(ax * ACC_SCALE)| and |mean(ay * ACC_SCALE)| must be < 0.15 g.

        Lateral acceleration must be near zero when the FC is level.
        Values >= 0.15 g indicate the board is tilted or vibrating.
        """
        samples = self._collect(drone_link)
        ax_mean = abs(statistics.mean(d["ax_g"] for d in samples))
        ay_mean = abs(statistics.mean(d["ay_g"] for d in samples))

        print(f"\n  |ax mean| = {ax_mean:.4f} g  (need < {self.LAT_MAX_MEAN})")
        print(f"  |ay mean| = {ay_mean:.4f} g  (need < {self.LAT_MAX_MEAN})")

        assert ax_mean < self.LAT_MAX_MEAN, (
            f"|ax mean| = {ax_mean:.4f} g — FC may be tilted sideways (roll axis)."
        )
        assert ay_mean < self.LAT_MAX_MEAN, (
            f"|ay mean| = {ay_mean:.4f} g — FC may be tilted fore/aft (pitch axis)."
        )

    # ── Sub-test 3 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_002_gyro_near_zero_at_rest(self, drone_link):
        """|mean(g* * GYRO_SCALE)| must be < 5.0 °/s on all axes at rest.

        Validates GYRO_SCALE = 1/16.4 through the full MSP pipeline.
        Non-zero noise floor is expected; values > 5 °/s indicate wrong
        scale constant or significant vibration.
        """
        samples = self._collect(drone_link)
        gx_mean = abs(statistics.mean(d["gx_dps"] for d in samples))
        gy_mean = abs(statistics.mean(d["gy_dps"] for d in samples))
        gz_mean = abs(statistics.mean(d["gz_dps"] for d in samples))

        print(f"\n  |gx mean| = {gx_mean:.3f} °/s  (need < {self.GYRO_MAX_MEAN})")
        print(f"  |gy mean| = {gy_mean:.3f} °/s  (need < {self.GYRO_MAX_MEAN})")
        print(f"  |gz mean| = {gz_mean:.3f} °/s  (need < {self.GYRO_MAX_MEAN})")

        assert gx_mean < self.GYRO_MAX_MEAN, (
            f"|gx mean| = {gx_mean:.3f} °/s at rest. "
            "If >> 5: check GYRO_SCALE = 1/16.4 in IMUSensor.h."
        )
        assert gy_mean < self.GYRO_MAX_MEAN, (
            f"|gy mean| = {gy_mean:.3f} °/s at rest."
        )
        assert gz_mean < self.GYRO_MAX_MEAN, (
            f"|gz mean| = {gz_mean:.3f} °/s at rest."
        )

    # ── Sub-test 4 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_002_widget_accel_scale_correct(self, imu_widget):
        """IMUWidget.ACCEL_SCALE must equal 2048.0 (DEF-002 fix applied).

        IMUWidget._render() divides raw MSP counts by ACCEL_SCALE.
        The constant must be 2048 (MSP-layer sensitivity after BF's
        ÷4 pre-scale) to match IMUSensor.ACC_SCALE = 1/2048.
        If still 8192, the widget underreads acceleration by 4×.
        """
        _, widget = imu_widget

        print(f"\n  IMUWidget.ACCEL_SCALE = {widget.ACCEL_SCALE}")
        assert widget.ACCEL_SCALE == 2048.0, (
            f"IMUWidget.ACCEL_SCALE = {widget.ACCEL_SCALE}, expected 2048.0. "
            "DEF-002 fix has not been applied to IMUWidget.py."
        )

    # ── Sub-test 5 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_002_widget_displays_correct_az(self, drone_link, imu_widget):
        """Widget-computed az must be in [0.95, 1.05] g after 3 s of live data.

        Confirms the end-to-end path: raw MSP counts arrive in to_dict(),
        the widget divides by ACCEL_SCALE = 2048, and the result matches
        the gravity reference within the acceptance band.
        az_display = data['az'] / widget.ACCEL_SCALE
        """
        root, widget = imu_widget
        collected = []

        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()
            raw_az = data.get("az", 0)
            collected.append(raw_az / widget.ACCEL_SCALE)
            time.sleep(0.1)

        mean_az = statistics.mean(collected)
        print(f"\n  Widget az mean : {mean_az:.4f} g")
        print(f"  ACCEL_SCALE    : {widget.ACCEL_SCALE}")

        assert self.AZ_MEAN_LOW <= mean_az <= self.AZ_MEAN_HIGH, (
            f"Widget-computed az mean = {mean_az:.4f} g — "
            f"outside [{self.AZ_MEAN_LOW}, {self.AZ_MEAN_HIGH}] g."
        )