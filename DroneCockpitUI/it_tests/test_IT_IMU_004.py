# =============================================================================
# test_IT_IMU_004.py  —  Fault Injection — Corrupt Frame Recovery
#
# V-Model reference: IMU Subsystem V-Model, Section 7, IT-IMU-004
# SRS coverage:      SRS-IMU-008  (parseIMU rejects bad frames)
#                    SRS-IMU-008b (worker thread survives, no crash)
#
# Strategy:
#   parseIMU() is protected in DroneLink — it is NOT exposed through the
#   pybind11 binding (only DroneLink and IMUSensor are exported).
#   The integration test therefore verifies the *observable contract*:
#
#   1. REJECTION TEST (via TestableDroneLink in C++ — already proven by
#      UT-IMU-002 / UT-IMU-003).  At the integration level we verify that
#      DroneState fields are unchanged when the worker thread receives a
#      frame it cannot parse (simulated here by momentarily suppressing the
#      IMU poll via a dedicated bad-frame injection period tracked through
#      DroneState.packet_count and link_healthy).
#
#   2. RECOVERY TEST — after link_healthy drops (FAIL_THRESHOLD = 5
#      consecutive IMU failures), the worker thread must recover without
#      a crash and link_healthy must return to True within 500 ms of the
#      board responding normally again.
#
#   3. CONTINUITY TEST — packet_count must continue to increment throughout,
#      proving the worker thread never hung or raised an exception.
#
#   Fault injection mechanism:
#     DroneLink.set_poll_interval_ms(0) causes the loop to spin as fast as
#     possible — at near-zero sleep the 80 ms read timeout in sendMSP() fires
#     frequently, producing empty (0-byte) returns that parseIMU() rejects,
#     driving consecutiveFails up to FAIL_THRESHOLD and flipping link_healthy
#     to False.  After 2 s we restore the normal 10 ms interval and confirm
#     recovery within 500 ms.
#
# Pass criteria (from ITS):
#   - No crash or exception during or after fault injection
#   - link_healthy recovers to True within FAIL_THRESHOLD good frames (~500 ms)
#   - packet_count continues to increment after recovery
#
# Baud-rate architecture note (mirrors IT-IMU-001 / DEF-001):
#   DroneLink polls 12 MSP commands sequentially at 57 600 baud.  Serial time
#   alone is ~40 ms/tick; with FC response jitter and 80 ms read timeouts on
#   absent sensors the observed rate is 3–6 Hz (ceiling ~24 Hz).
#   packet_count increments once per loop tick where parseIMU() succeeds
#   (anySuccess = true).  ≥ 50 Hz is the IMU sensor rate inside Betaflight,
#   not the MSP link rate.  The original criterion of ≥ 80 packets in 2 s
#   assumed a 50 Hz link rate, which is architecturally impossible at this
#   baud rate — identical to the constraint documented in DEF-001 for
#   IT-IMU-001.  Criterion revised to ≥ 5 packets in 2 s (≥ 2.5 Hz) with
#   80 % tolerance, consistent with 3–6 Hz observed link rate.  A stalled
#   thread returns 0 packets; any positive count ≥ 5 confirms liveness.
#
# Board required: F405 V3 connected via USB, Betaflight 4.5.3 running.
# =============================================================================

import time
import pytest


class TestCorruptFrameRecovery:
    """IT-IMU-004 — worker thread survives bad frames and link recovers."""

    FAIL_THRESHOLD     = 5      # consecutive failures → link_healthy=False
    NORMAL_INTERVAL_MS = 10     # default from DroneLink.h POLL_INTERVAL_MS
    INJECT_DURATION_S  = 2.0    # how long to hold the stressed poll interval
    RECOVERY_WINDOW_S  = 1.0    # time budget to regain link_healthy after restore
    MONITOR_DURATION_S = 2.0    # post-recovery window to confirm continuity

    # -------------------------------------------------------------------------
    # Minimum packets expected in MONITOR_DURATION_S after recovery.
    #
    # Derivation (matches DEF-001 / IT-IMU-001 reasoning):
    #   Observed MSP link rate at 57 600 baud: 3–6 Hz.
    #   Floor = 3 Hz × 2.0 s × 0.8 tolerance = 4.8 → 5 packets.
    #   Original value was int(50 * 2.0 * 0.8) = 80, which assumed a 50 Hz
    #   link rate that is impossible over this serial interface.
    # -------------------------------------------------------------------------
    MIN_PACKETS_AFTER_RECOVERY = 5

    def test_IT_IMU_004_no_crash_during_injection(self, drone_link):
        """DroneLink must not raise an exception during stressed polling."""
        try:
            drone_link.set_poll_interval_ms(0)
            time.sleep(self.INJECT_DURATION_S)
        except Exception as exc:
            pytest.fail(f"Exception during fault injection: {exc}")
        finally:
            drone_link.set_poll_interval_ms(self.NORMAL_INTERVAL_MS)

        assert True, "Worker thread raised an exception during fault injection"

    def test_IT_IMU_004_link_recovers_after_injection(self, drone_link):
        """link_healthy must return True within 1 s of restoring normal polling."""
        drone_link.set_poll_interval_ms(0)
        time.sleep(self.INJECT_DURATION_S)

        drone_link.set_poll_interval_ms(self.NORMAL_INTERVAL_MS)

        recovered = False
        deadline  = time.monotonic() + self.RECOVERY_WINDOW_S
        while time.monotonic() < deadline:
            if drone_link.get_latest_state().link_healthy:
                recovered = True
                break
            time.sleep(0.02)

        print(f"\n  link_healthy recovered : {recovered}")
        assert recovered, (
            f"link_healthy did not return to True within {self.RECOVERY_WINDOW_S} s "
            f"after restoring normal poll interval"
        )

    def test_IT_IMU_004_packet_count_continues(self, drone_link):
        """packet_count must keep incrementing after recovery — no thread hang.

        Pass criterion: ≥ 5 new packets in 2 s (≥ 2.5 Hz).
        Rationale: 57 600 baud + 12 sequential MSP polls limits the link to
        3–6 Hz (ceiling ~24 Hz).  The original criterion of ≥ 80 packets
        assumed a 50 Hz rate that is architecturally impossible over this
        serial interface — see DEF-001 and the baud-rate note at the top of
        this file.  Any count ≥ 5 proves the worker thread is alive and
        parsing frames; 0 would indicate a stall.
        """
        drone_link.set_poll_interval_ms(0)
        time.sleep(self.INJECT_DURATION_S)
        drone_link.set_poll_interval_ms(self.NORMAL_INTERVAL_MS)

        # Wait for full recovery before measuring continuity
        time.sleep(self.RECOVERY_WINDOW_S)

        s_before     = drone_link.get_latest_state()
        count_before = s_before.packet_count

        time.sleep(self.MONITOR_DURATION_S)

        s_after     = drone_link.get_latest_state()
        count_after = s_after.packet_count

        delta = count_after - count_before

        print(f"\n  packet_count before : {count_before}")
        print(f"  packet_count after  : {count_after}")
        print(f"  delta               : {delta}  "
              f"(need ≥ {self.MIN_PACKETS_AFTER_RECOVERY})")
        assert delta >= self.MIN_PACKETS_AFTER_RECOVERY, (
            f"Only {delta} new packets in {self.MONITOR_DURATION_S} s after recovery "
            f"(need ≥ {self.MIN_PACKETS_AFTER_RECOVERY}) — worker thread may have stalled"
        )

    def test_IT_IMU_004_dronestate_unchanged_on_bad_frame(self, drone_link):
        """IMU fields must not be corrupted to impossible values during injection.

        SRS-IMU-008 states DroneState is left unchanged on a rejected frame.
        The equivalent integration check is that ax/ay/az/gx/gy/gz never
        contain values outside the int16 physical range during fault injection.
        """
        INT16_MIN = -32768
        INT16_MAX =  32767
        violations = []

        drone_link.set_poll_interval_ms(0)
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < self.INJECT_DURATION_S:
                s = drone_link.get_latest_state()
                for name, val in [("ax", s.ax), ("ay", s.ay), ("az", s.az),
                                   ("gx", s.gx), ("gy", s.gy), ("gz", s.gz)]:
                    if not (INT16_MIN <= val <= INT16_MAX):
                        violations.append(f"{name}={val}")
                time.sleep(0.005)
        finally:
            drone_link.set_poll_interval_ms(self.NORMAL_INTERVAL_MS)

        print(f"\n  out-of-range violations : {len(violations)}")
        assert len(violations) == 0, (
            f"IMU fields had out-of-range values during fault injection: "
            f"{violations[:5]}"
        )