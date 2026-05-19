# =============================================================================
# test_IT_IMU_001.py  —  Live Data Continuity
#
# V-Model reference: IMU Subsystem V-Model, Section 7, IT-IMU-001
# SRS coverage:      SRS-IMU-001 (≥50 Hz), SRS-IMU-003 (non-zero IMU data)
#
# Pass criteria (from ITS):
#   - packet_count increases by ≥ 250 over 5 s          (≥50 Hz poll rate)
#   - At least one IMU field non-zero in every snapshot  (live data flowing)
#   - last_rtt_ms < 50 ms for ≥ 95% of samples          (timing OK)
#
# Board required: F405 V3 connected via USB, Betaflight 4.5.3 running.
# =============================================================================

import time
import pytest


class TestLiveDataContinuity:
    """IT-IMU-001 — MSP_RAW_IMU delivers fresh packets at ≥ 50 Hz for 5 s."""

    SAMPLE_DURATION_S  = 5.0
    SAMPLE_INTERVAL_S  = 0.005    # 200 Hz sampling rate
    MIN_PACKET_DELTA   = 250      # ≥ 250 new packets in 5 s = ≥ 50 Hz
    MAX_RTT_MS         = 50.0
    MIN_RTT_PASS_PCT   = 95.0

    def test_IT_IMU_001_packet_rate(self, drone_link):
        """packet_count must increase by ≥ 250 over 5 s (≥ 50 Hz)."""
        s0 = drone_link.get_latest_state()
        start_count = s0.packet_count

        time.sleep(self.SAMPLE_DURATION_S)

        s1 = drone_link.get_latest_state()
        delta = s1.packet_count - start_count

        print(f"\n  packet delta = {delta}  (need ≥ {self.MIN_PACKET_DELTA})")
        assert delta >= self.MIN_PACKET_DELTA, (
            f"Packet rate too low: {delta} packets in {self.SAMPLE_DURATION_S} s "
            f"(need ≥ {self.MIN_PACKET_DELTA})"
        )

    def test_IT_IMU_001_imu_fields_nonzero(self, drone_link):
        """Every snapshot must have at least one non-zero IMU field."""
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.SAMPLE_DURATION_S:
            s = drone_link.get_latest_state()
            samples.append(s)
            time.sleep(self.SAMPLE_INTERVAL_S)

        zero_snapshots = [
            i for i, s in enumerate(samples)
            if not any([s.ax, s.ay, s.az, s.gx, s.gy, s.gz])
        ]

        print(f"\n  snapshots collected : {len(samples)}")
        print(f"  all-zero snapshots  : {len(zero_snapshots)}")
        assert len(zero_snapshots) == 0, (
            f"{len(zero_snapshots)} snapshot(s) had all-zero IMU fields — "
            "MSP_RAW_IMU may not be returning data"
        )

    def test_IT_IMU_001_rtt_under_50ms(self, drone_link):
        """last_rtt_ms must be < 50 ms for ≥ 95% of samples."""
        rtts = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.SAMPLE_DURATION_S:
            s = drone_link.get_latest_state()
            if s.last_rtt_ms > 0:
                rtts.append(s.last_rtt_ms)
            time.sleep(self.SAMPLE_INTERVAL_S)

        if not rtts:
            pytest.fail("No RTT samples collected — last_rtt_ms was always 0")

        under_50 = sum(1 for r in rtts if r < self.MAX_RTT_MS)
        pct = under_50 / len(rtts) * 100.0

        print(f"\n  RTT samples : {len(rtts)}")
        print(f"  RTT < 50 ms : {pct:.1f}%  (need ≥ {self.MIN_RTT_PASS_PCT}%)")
        print(f"  RTT min/max : {min(rtts):.2f} / {max(rtts):.2f} ms")

        assert pct >= self.MIN_RTT_PASS_PCT, (
            f"Only {pct:.1f}% of RTT samples < 50 ms "
            f"(need ≥ {self.MIN_RTT_PASS_PCT}%)"
        )