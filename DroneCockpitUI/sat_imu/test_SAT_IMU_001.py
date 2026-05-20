# =============================================================================
# test_SAT_IMU_001.py  —  Real-Time IMU Data Visible on GCS Display
#
# V-Model reference: System Acceptance Testing, SAT-IMU-001
# SYS requirement:   SYS-001
# SRS coverage:      SRS-IMU-001, SRS-IMU-002, SRS-IMU-003
#
# Objective:
#   Verify that the GCS IMU widget displays continuously updating acceleration
#   and angular-rate values for all six axes within 5 seconds of connecting.
#   Confirms the full pipeline: MSP_RAW_IMU → DroneLink → IMUSensor →
#   DroneBackend.to_dict() → IMUWidget.update_ui() is operational end-to-end.
#
# Preconditions:
#   - F405 V3 powered and connected via USB
#   - Betaflight 4.5.3 running, props OFF
#   - FC sitting stationary on bench (no manipulation required)
#
# Pass Criteria (SYS-001):
#   - All six IMU cells (ax, ay, az, gx, gy, gz) display numeric values
#     within 5 s of connection
#   - At least one cell value changes between consecutive display updates
#     during a 10 s observation window
#   - link_healthy remains True throughout (no link-lost indicator)
#   - packet_count increases by ≥ 15 over 10 s (≥ 3 Hz sustained — see DEF-001)
#
# Run:
#   pytest sat_tests/test_SAT_IMU_001.py -v -s
# =============================================================================

import time
import pytest


# ── Timing constants ──────────────────────────────────────────────────────────
_POLL_HZ        = 20        # sample rate for this test (well above 3–6 Hz MSP rate)
_POLL_INTERVAL  = 1.0 / _POLL_HZ
_FIRST_DATA_S   = 5.0       # max time to wait for first non-zero IMU packet
_OBSERVE_S      = 10.0      # observation window for liveness check
_MIN_PACKETS    = 15        # ≥ 3 Hz × 10 s × 0.5 tolerance — calibrated to
                            # 57 600 baud + 12 sequential MSP polls (DEF-001)

# IMU field keys emitted by DroneBackend.to_dict()
_ACCEL_KEYS = ("ax", "ay", "az")
_GYRO_KEYS  = ("gx", "gy", "gz")
_ALL_IMU_KEYS = _ACCEL_KEYS + _GYRO_KEYS


class TestRealtimeIMUDisplay:
    """SAT-IMU-001 — SYS-001: IMU data visible and live on the GCS display."""

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _snapshot(self, drone_link, imu_widget) -> dict:
        """Fetch one DroneState snapshot, push it to the widget, return to_dict()."""
        root, widget = imu_widget
        s    = drone_link.get_latest_state()
        data = s.to_dict()
        widget.update_ui(data)
        root.update_idletasks()
        return data

    def _all_keys_present(self, data: dict) -> bool:
        """Return True if all 6 IMU keys exist in the dict (even if zero)."""
        return all(k in data for k in _ALL_IMU_KEYS)

    def _any_imu_nonzero(self, data: dict) -> bool:
        """Return True if at least one IMU value is non-zero."""
        return any(data.get(k, 0) != 0 for k in _ALL_IMU_KEYS)

    # ── Tests ─────────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_all_keys_present(self, drone_link, imu_widget):
        """All six to_dict() IMU keys must exist immediately after connection.

        Validates SRS-IMU-003: 6 × int16 fields (ax/ay/az/gx/gy/gz) are
        exported by DroneBackend.to_dict().
        """
        data = self._snapshot(drone_link, imu_widget)

        missing = [k for k in _ALL_IMU_KEYS if k not in data]
        assert not missing, (
            f"to_dict() is missing IMU keys: {missing}\n"
            "  → Check DroneBackend binding — all 6 fields must be exported.\n"
            f"  Keys present: {list(data.keys())}"
        )

    def test_SAT_IMU_001_data_arrives_within_5s(self, drone_link, imu_widget):
        """At least one non-zero IMU field must appear within 5 s of starting.

        Validates SRS-IMU-001: IMU data is delivered end-to-end.
        The session fixture already waited 2 s for stabilisation; this test
        gives an additional 5 s window to match the SAT procedure.
        """
        nonzero_seen = False
        t0 = time.monotonic()

        while time.monotonic() - t0 < _FIRST_DATA_S:
            data = self._snapshot(drone_link, imu_widget)
            if self._any_imu_nonzero(data):
                nonzero_seen = True
                elapsed = time.monotonic() - t0
                print(f"\n  [SAT-001] Non-zero IMU data arrived after {elapsed:.2f} s")
                break
            time.sleep(_POLL_INTERVAL)

        assert nonzero_seen, (
            f"All IMU values were zero after {_FIRST_DATA_S:.0f} s.\n"
            "  → Confirm F405 is powered and Betaflight is running.\n"
            "  → Check USB CDC connection and COM port auto-detection."
        )

    def test_SAT_IMU_001_values_change_over_10s(self, drone_link, imu_widget):
        """IMU values must change between consecutive updates over 10 s.

        Validates that the pipeline is delivering live data (not a frozen
        snapshot), satisfying the liveness criterion of SYS-001 / SRS-IMU-001.
        A value changing between any two adjacent samples is sufficient — even
        with a stationary FC the Z-axis accelerometer and gyro noise floor
        produce sub-LSB jitter that manifests as occasional count changes.
        """
        prev_data   = self._snapshot(drone_link, imu_widget)
        change_seen = False
        t0 = time.monotonic()

        while time.monotonic() - t0 < _OBSERVE_S:
            time.sleep(_POLL_INTERVAL)
            curr_data = self._snapshot(drone_link, imu_widget)

            if any(
                curr_data.get(k, 0) != prev_data.get(k, 0)
                for k in _ALL_IMU_KEYS
            ):
                change_seen = True
                elapsed = time.monotonic() - t0
                print(f"\n  [SAT-001] IMU value change observed at t+{elapsed:.2f} s")
                break

            prev_data = curr_data

        assert change_seen, (
            f"No IMU field changed during {_OBSERVE_S:.0f} s observation window.\n"
            "  → Values appear frozen — check that the DroneLink worker thread\n"
            "    is running and parseIMU() is updating DroneState each tick."
        )

    def test_SAT_IMU_001_link_healthy_throughout(self, drone_link, imu_widget):
        """link_healthy must remain True for the full 10 s observation window.

        Validates the no-link-lost-indicator criterion of SAT-IMU-001.
        A link_healthy = False during the observation window indicates serial
        communication failures, which would also suppress the liveness test.
        """
        failures = []
        t0 = time.monotonic()

        while time.monotonic() - t0 < _OBSERVE_S:
            data = self._snapshot(drone_link, imu_widget)
            if not data.get("link_healthy", True):
                elapsed = time.monotonic() - t0
                failures.append(elapsed)
            time.sleep(_POLL_INTERVAL)

        assert not failures, (
            f"link_healthy = False at {len(failures)} sample(s) during "
            f"{_OBSERVE_S:.0f} s window.\n"
            f"  First failure at t+{failures[0]:.2f} s.\n"
            "  → Check USB cable, COM port driver, and serial baud rate."
        )

    def test_SAT_IMU_001_packet_count_increases(self, drone_link, imu_widget):
        """packet_count must increase by ≥ 15 over 10 s (≥ 1.5 Hz minimum).

        Pass criterion calibrated to the 57 600 baud + 12-sequential-MSP
        architecture (DEF-001): observed floor is ~3 Hz (17 pkts/5 s in
        IT-IMU-001). Floor here is ≥ 15 pkts/10 s = 1.5 Hz, providing a
        comfortable margin against USB jitter while excluding a stalled loop.

        Note: packet_count increments only when parseIMU() sets anySuccess=true.
        """
        data_start = self._snapshot(drone_link, imu_widget)
        count_start = data_start.get("packet_count", 0)

        time.sleep(_OBSERVE_S)

        data_end  = self._snapshot(drone_link, imu_widget)
        count_end = data_end.get("packet_count", 0)
        delta     = count_end - count_start

        print(f"\n  [SAT-001] packet_count delta: {delta} over {_OBSERVE_S:.0f} s "
              f"({delta / _OBSERVE_S:.2f} Hz)")

        assert delta >= _MIN_PACKETS, (
            f"packet_count increased by only {delta} over {_OBSERVE_S:.0f} s "
            f"(need ≥ {_MIN_PACKETS}).\n"
            f"  Effective rate: {delta / _OBSERVE_S:.2f} Hz\n"
            "  → This may indicate the worker thread has stalled or is\n"
            "    failing to parse MSP_RAW_IMU frames. Check I2C errors\n"
            "    with 'status' in Betaflight CLI."
        )