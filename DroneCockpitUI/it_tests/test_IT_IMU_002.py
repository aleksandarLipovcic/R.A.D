# =============================================================================
# test_IT_IMU_002.py  —  Scale Validation Against Known Gravity Vector
#
# V-Model reference: IMU Subsystem V-Model, Section 7, IT-IMU-002
# SRS coverage:      SRS-IMU-004a (ACC_SCALE = 1/2048)
#
# Pass criteria (from ITS):
#   - mean(accZ) in [0.95, 1.05] g
#   - std(accZ)  < 0.01 g
#   - sample count >= 400  (confirms sensor is actively streaming)
#
# All three criteria are evaluated over the SAME sample set, consistent with
# the ITS requirement: "all sub-tests PASSED" over a single 5 s collection
# window (475 samples observed during execution).
#
# Precondition:
#   Place the F405 V3 FLAT and LEVEL on anti-vibration foam before running.
#   The Z-axis accelerometer should read +1g (gravity pointing down into FC).
#   Do NOT run on a vibrating surface — motor vibration will inflate stdev.
#
# Board required: F405 V3 flat on foam, Betaflight 4.5.3, props OFF.
# =============================================================================

import time
import statistics
import pytest


class TestScaleValidation:
    """IT-IMU-002 — getScaledData().az_g ≈ 1.0 g when FC is flat and level."""

    SAMPLE_DURATION_S = 5.0
    SAMPLE_INTERVAL_S = 0.01     # 100 Hz
    MIN_SAMPLES       = 400      # sanity: must collect at least this many
    MEAN_LOW          = 0.95     # g
    MEAN_HIGH         = 1.05     # g
    MAX_STDEV         = 0.01     # g

    # -------------------------------------------------------------------------
    # Shared fixture — collect once, evaluate all sub-tests on the same set.
    # Using autouse=False + explicit parameter keeps it scoped to this class.
    # -------------------------------------------------------------------------
    @pytest.fixture(scope="class")
    def az_samples(self, imu_sensor):
        """
        Collect accZ samples once for the entire test class.

        All three sub-tests (mean, stdev, sample_count) operate on the
        identical set, satisfying the ITS requirement that criteria are
        evaluated over the same collection window.
        """
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.SAMPLE_DURATION_S:
            d = imu_sensor.get_scaled_data()
            # get_scaled_data() returns dict: az_g is the Z-axis in g
            samples.append(d["az_g"])
            time.sleep(self.SAMPLE_INTERVAL_S)

        print(f"\n  [IT-IMU-002] collection complete: {len(samples)} samples "
              f"in {self.SAMPLE_DURATION_S} s")
        return samples

    # -------------------------------------------------------------------------
    # Sub-tests — all receive the same az_samples fixture instance
    # -------------------------------------------------------------------------
    def test_IT_IMU_002_sample_count(self, az_samples):
        """Must collect ≥ 400 samples in 5 s (confirms sensor is streaming)."""
        print(f"\n  samples collected : {len(az_samples)}  (need ≥ {self.MIN_SAMPLES})")
        assert len(az_samples) >= self.MIN_SAMPLES, (
            f"Only {len(az_samples)} samples collected in {self.SAMPLE_DURATION_S} s"
        )

    def test_IT_IMU_002_accel_z_mean(self, az_samples):
        """mean(accZ) must be in [0.95, 1.05] g with FC flat and level."""
        mean = statistics.mean(az_samples)

        print(f"\n  samples     : {len(az_samples)}")
        print(f"  accZ mean   : {mean:.4f} g  (need {self.MEAN_LOW} – {self.MEAN_HIGH})")
        assert self.MEAN_LOW <= mean <= self.MEAN_HIGH, (
            f"accZ mean = {mean:.4f} g — outside [0.95, 1.05] g. "
            "Is the FC flat and level?"
        )

    def test_IT_IMU_002_accel_z_stdev(self, az_samples):
        """std(accZ) must be < 0.01 g (low vibration / stable surface)."""
        stdev = statistics.stdev(az_samples)

        print(f"\n  samples     : {len(az_samples)}")
        print(f"  accZ stdev  : {stdev:.4f} g  (need < {self.MAX_STDEV})")
        assert stdev < self.MAX_STDEV, (
            f"accZ stdev = {stdev:.4f} g — too noisy. "
            "Place FC on anti-vibration foam and ensure props are off."
        )