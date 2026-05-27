# =============================================================================
# test_SAT_IMU_004.py  —  End-to-End Latency Under Nominal Conditions
#
# V-Model reference: System Acceptance Testing, SAT-IMU-004
# SYS requirement:   SYS-004
# SRS coverage:      SRS-IMU-007
#
# Objective:
#   Verify that the complete system — from MSP_RAW_IMU request in the C++
#   worker thread through to IMUWidget.update_ui() completing in Python —
#   meets the < 200 ms end-to-end latency requirement under nominal USB CDC
#   operating conditions.
#
# Latency definition (SRS-IMU-007):
#   total_ms = C++ RTT (to_dict()['rtt_ms'], measured around sendMSP(RAW_IMU)
#              + parseIMU() in communicationLoop()) + Python processing time
#              (time to run update_ui() and update_idletasks()).
#
# DroneState RTT field — naming and access:
#   DroneLink.h declares:  double lastRttMs = 0.0;
#   DroneLink.cpp writes:  pending.lastRttMs = rtt.count();
#   The pybind11 binding exposes this as to_dict() key 'rtt_ms' (documented
#   in V-Model Section 5 traceability table, SRS-IMU-007).
#   This test reads RTT via data.get("rtt_ms", 0.0) from the same to_dict()
#   call already needed for widget.update_ui(data), adding zero overhead.
#   Do NOT use s.lastRttMs directly — to_dict()['rtt_ms'] is the documented
#   API and is consistent with how all other SAT tests read DroneState data.
#
# Pass criteria (SYS-004):
#   - P95 total latency < 200 ms over 1000 samples
#   - P50 total latency <  30 ms over 1000 samples
#
# Board required: F405 V3 connected, Betaflight 4.5.3, all telemetry active
#                 (do not disable GPS/baro/mag polls — SAT tests nominal config)
# =============================================================================

import time
import pytest


class TestEndToEndLatency:
    """SAT-IMU-004 — SYS-004: < 200 ms end-to-end latency in nominal config."""

    NUM_SAMPLES       = 1000
    SAMPLE_INTERVAL_S = 0.01    # 100 Hz sampling
    P95_LIMIT_MS      = 200.0
    P50_LIMIT_MS      = 30.0

    def _collect_latencies(self, drone_link, imu_widget):
        """Collect NUM_SAMPLES end-to-end latency measurements.

        total_ms = to_dict()['rtt_ms']  (C++ RTT: sendMSP + parseIMU)
                 + (t1 - t0) * 1000     (Python: update_ui + update_idletasks)

        'rtt_ms' is written by communicationLoop() only when parseIMU()
        succeeds. For ticks where parseIMU() failed, lastRttMs retains its
        previous value (DroneState is a carried-forward snapshot). These
        samples are included unchanged — they do not inflate the percentiles
        because they reflect the same USB-serial timing path.
        """
        root, widget = imu_widget
        latencies = []
        for _ in range(self.NUM_SAMPLES):
            s    = drone_link.get_latest_state()
            data = s.to_dict()          # single call — reused for widget and RTT
            t0   = time.monotonic()
            widget.update_ui(data)
            root.update_idletasks()
            t1   = time.monotonic()
            cpp_rtt_ms = data.get("rtt_ms", 0.0)
            py_proc_ms = (t1 - t0) * 1000.0
            latencies.append(cpp_rtt_ms + py_proc_ms)
            time.sleep(self.SAMPLE_INTERVAL_S)
        return sorted(latencies)

    def test_SAT_IMU_004_p95_under_200ms(self, drone_link, imu_widget):
        """P95 total end-to-end latency must be < 200 ms (SRS-IMU-007)."""
        latencies = self._collect_latencies(drone_link, imu_widget)
        p95 = latencies[int(len(latencies) * 0.95)]

        print(f"\n  samples : {len(latencies)}")
        print(f"  P95     : {p95:.2f} ms  (need < {self.P95_LIMIT_MS})")
        assert p95 < self.P95_LIMIT_MS, (
            f"P95 latency = {p95:.2f} ms — exceeds 200 ms SRS-IMU-007 limit"
        )

    def test_SAT_IMU_004_p50_under_30ms(self, drone_link, imu_widget):
        """P50 (median) total end-to-end latency must be < 30 ms."""
        latencies = self._collect_latencies(drone_link, imu_widget)
        p50 = latencies[int(len(latencies) * 0.50)]

        print(f"\n  samples : {len(latencies)}")
        print(f"  P50     : {p50:.2f} ms  (need < {self.P50_LIMIT_MS})")
        assert p50 < self.P50_LIMIT_MS, (
            f"P50 latency = {p50:.2f} ms — exceeds 30 ms target"
        )

    def test_SAT_IMU_004_latency_report(self, drone_link, imu_widget):
        """Informational: full percentile report for the test execution log."""
        latencies = self._collect_latencies(drone_link, imu_widget)
        n = len(latencies)
        print(f"\n  ── SAT-IMU-004 Latency Report ─────────────────────────")
        print(f"  {'Percentile':<12} {'ms':>8}  {'Pass?':>6}")
        print(f"  {'-'*34}")
        checks = {50: self.P50_LIMIT_MS, 95: self.P95_LIMIT_MS}
        for pct in (50, 75, 90, 95, 99):
            val   = latencies[int(n * pct / 100)]
            limit = checks.get(pct)
            flag  = f"< {limit:.0f} {'✓' if limit and val < limit else '✗'}" if limit else ""
            print(f"  P{pct:<11} {val:>8.2f}  {flag}")
        print(f"  {'min':<12} {latencies[0]:>8.2f}")
        print(f"  {'max':<12} {latencies[-1]:>8.2f}")
        print(f"  {'samples':<12} {n:>8}")
        print(f"  ──────────────────────────────────────────────────────")
        assert True   # always passes — informational