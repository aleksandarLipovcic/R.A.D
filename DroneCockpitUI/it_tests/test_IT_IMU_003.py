# =============================================================================
# test_IT_IMU_003.py  —  End-to-End Latency Measurement
#
# V-Model reference: IMU Subsystem V-Model, Section 7, IT-IMU-003
# SRS coverage:      SRS-IMU-007 (total latency < 200 ms)
#
# Latency definition (matches ITS procedure):
#   total_ms = C++ RTT (last_rtt_ms, measured inside communicationLoop()
#              around sendMSP + parseIMU) + Python processing time
#              (time to call to_dict() + widget update_ui()).
#
# Pass criteria (from ITS):
#   - P95 total latency < 200 ms
#   - P50 total latency <  30 ms
#
# Note: IMUWidget is imported directly from the source tree.
#       Adjust the sys.path insert below if your layout differs.
#
# Board required: F405 V3 connected via USB, Betaflight 4.5.3 running.
# =============================================================================

import sys
import os
import time
import pytest

# ---------------------------------------------------------------------------
# Make the IMUWidget importable — it lives alongside the test scripts in the
# project root (one level above this it_tests/ folder).
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from IMUWidget import IMUWidget   # noqa: E402  (path must be set first)
import tkinter as tk              # noqa: E402


class TestEndToEndLatency:
    """IT-IMU-003 — total pipeline latency (C++ RTT + Python) meets SRS-IMU-007."""

    NUM_SAMPLES  = 1000
    INTERVAL_S   = 0.01    # 100 Hz
    P95_LIMIT_MS = 200.0
    P50_LIMIT_MS = 30.0

    @pytest.fixture(autouse=True, scope="class")
    def _setup_widget(self, request):
        """Create a single hidden Tkinter root + IMUWidget for the class."""
        root = tk.Tk()
        root.withdraw()
        widget = IMUWidget(root)
        request.cls._root   = root
        request.cls._widget = widget
        yield
        try:
            root.destroy()
        except Exception:
            pass

    def _collect_latencies(self, drone_link) -> list:
        latencies = []
        for _ in range(self.NUM_SAMPLES):
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            t0   = time.monotonic()
            self._widget.update_ui(data)
            t1   = time.monotonic()
            # last_rtt_ms is the C++ portion (sendMSP round-trip + parseIMU)
            total_ms = s.last_rtt_ms + (t1 - t0) * 1000.0
            latencies.append(total_ms)
            time.sleep(self.INTERVAL_S)
        return sorted(latencies)

    def test_IT_IMU_003_p95_under_200ms(self, drone_link):
        """P95 total latency must be < 200 ms."""
        latencies = self._collect_latencies(drone_link)
        p95 = latencies[int(len(latencies) * 0.95)]

        print(f"\n  samples : {len(latencies)}")
        print(f"  P95     : {p95:.2f} ms  (need < {self.P95_LIMIT_MS})")
        assert p95 < self.P95_LIMIT_MS, (
            f"P95 latency = {p95:.2f} ms — exceeds 200 ms limit"
        )

    def test_IT_IMU_003_p50_under_30ms(self, drone_link):
        """P50 (median) total latency must be < 30 ms."""
        latencies = self._collect_latencies(drone_link)
        p50 = latencies[int(len(latencies) * 0.50)]

        print(f"\n  samples : {len(latencies)}")
        print(f"  P50     : {p50:.2f} ms  (need < {self.P50_LIMIT_MS})")
        assert p50 < self.P50_LIMIT_MS, (
            f"P50 latency = {p50:.2f} ms — exceeds 30 ms limit"
        )

    def test_IT_IMU_003_latency_summary(self, drone_link):
        """Print full latency percentile table (always passes — informational)."""
        latencies = self._collect_latencies(drone_link)
        n = len(latencies)
        print(f"\n  {'Percentile':<12} {'ms':>8}")
        print(f"  {'-'*22}")
        for pct in (50, 75, 90, 95, 99):
            val = latencies[int(n * pct / 100)]
            flag = " ✓" if (pct < 95 and val < self.P50_LIMIT_MS) or \
                           (pct == 95 and val < self.P95_LIMIT_MS) else ""
            print(f"  P{pct:<11} {val:>8.2f}{flag}")
        print(f"  {'min':<12} {latencies[0]:>8.2f}")
        print(f"  {'max':<12} {latencies[-1]:>8.2f}")
        # Always passes — summary is for the test report only
        assert True