# =============================================================================
# test_SAT_IMU_001.py  —  Real-Time Inertial Data — Full GCS Pipeline
#
# V-Model reference: System Acceptance Testing, SAT-IMU-001
# SYS requirement:   SYS-001
# SRS coverage:      SRS-IMU-001, SRS-IMU-002, SRS-IMU-003
#
# Objective:
#   Verify that the complete system pipeline — FC → DroneLink worker thread →
#   pybind11 DroneBackend → IMUWidget.update_ui() — delivers fresh, non-zero
#   inertial data to the GCS display continuously, and that the operator
#   can see live values on screen within 3 s of connection.
#
# Pass criteria (SYS-001):
#   - linkHealthy is True after 2 s warmup
#   - At least one accelerometer field (ax/ay/az) is non-zero after warmup
#   - At least one gyroscope field (gx/gy/gz) is non-zero after warmup
#   - packetCount increases by >= 15 over 5 s (>= 3 Hz loop throughput
#     at 57 600 baud with 12 sequential MSP polls per tick)
#   - IMUWidget.update_ui() accepts 20 consecutive live to_dict() snapshots
#     without raising any exception
#   - Widget rotation cells do not show '---' placeholder after 3 s of data
#
# Timing note:
#   packetCount increments on every communicationLoop() tick regardless of
#   IMU parse success. At 57 600 baud with 12 sequential MSP polls and an
#   80 ms per-call timeout, each tick takes 100–300 ms in practice, giving
#   3–10 loop iterations per second. The >= 15 floor over 5 s (= 3 Hz)
#   confirms the loop is alive and progressing — it does not directly count
#   successful IMU parses. linkHealthy = True is the separate check that
#   at least one parseIMU() per FAIL_THRESHOLD ticks succeeded.
#
# DroneState attribute naming (pybind11):
#   The C++ DroneState members use camelCase (linkHealthy, packetCount).
#   The pybind11 binding exposes them under those exact names — no automatic
#   snake_case aliasing occurs. All attribute accesses in this file use the
#   camelCase names that match DroneLink.h.
#
# Board required: F405 V3 powered and connected, Betaflight 4.5.3 running.
# =============================================================================

import time
import pytest


class TestRealTimeDataPipeline:
    """SAT-IMU-001 — SYS-001: live IMU data reaches the GCS display."""

    WARMUP_S          = 2.0
    SAMPLE_DURATION_S = 5.0
    MIN_PACKET_DELTA  = 15      # >= 3 Hz loop rate at 57 600 baud
    WIDGET_FEED_S     = 3.0     # seconds to feed live data before cell check

    # ── Sub-test 1 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_link_healthy_after_warmup(self, drone_link):
        """linkHealthy must be True after 2 s warmup.

        linkHealthy goes True as soon as one parseIMU() succeeds in the
        worker thread.  False after warmup means every IMU parse is failing:
        check serial port, FC power, no Betaflight Configurator open on the
        same COM port.
        """
        time.sleep(self.WARMUP_S)
        s = drone_link.get_latest_state()

        print(f"\n  linkHealthy  = {s.linkHealthy}")
        print(f"  packetCount  = {s.packetCount}")

        assert s.linkHealthy, (
            "linkHealthy is False after warmup — "
            ">= FAIL_THRESHOLD (5) consecutive IMU parse failures. "
            "Check: board powered, BF 4.5.3 running, no other tool on same port."
        )

    # ── Sub-test 2 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_accel_fields_nonzero(self, drone_link):
        """At least one accelerometer field must be non-zero after warmup.

        A stationary FC always reads non-zero gravity on at least one accel
        axis.  All-zero means MSP_RAW_IMU is not reaching parseIMU() or
        DroneState is not being committed to shared memory.
        """
        s = drone_link.get_latest_state()

        print(f"\n  ax={s.ax}  ay={s.ay}  az={s.az}")

        assert any([s.ax, s.ay, s.az]), (
            "All accelerometer fields are zero. "
            "Expected non-zero gravity reading on at least one axis. "
            "Check: sensor_status bit 0 (ACC) should be set in MSP_STATUS."
        )

    # ── Sub-test 3 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_gyro_fields_nonzero(self, drone_link):
        """At least one gyroscope field must be non-zero after warmup.

        The MPU-6500 gyroscope always has a measurable noise floor even
        completely at rest.  All-zero means the gyro is unpowered or
        parseIMU() is not writing to DroneState.
        """
        s = drone_link.get_latest_state()

        print(f"\n  gx={s.gx}  gy={s.gy}  gz={s.gz}")

        assert any([s.gx, s.gy, s.gz]), (
            "All gyroscope fields are zero. "
            "Expected non-zero noise floor on at least one axis. "
            "Check: sensor_status bit 5 (GYRO) should be set in MSP_STATUS."
        )

    # ── Sub-test 4 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_packet_rate_sustained(self, drone_link):
        """packetCount must increase by >= 15 over 5 s (>= 3 Hz loop rate).

        Confirms the communicationLoop() worker thread is alive and cycling
        continuously.  packetCount increments on every tick irrespective of
        IMU parse success, so this measures loop throughput, not IMU rate.
        At 57 600 baud the loop runs at 3–10 Hz in practice; the >= 15
        floor (3 Hz × 5 s) is a conservative liveness check.
        """
        s0 = drone_link.get_latest_state()
        c0 = s0.packetCount
        time.sleep(self.SAMPLE_DURATION_S)
        s1 = drone_link.get_latest_state()
        delta = s1.packetCount - c0
        hz    = delta / self.SAMPLE_DURATION_S

        print(f"\n  packetCount delta = {delta}  ({hz:.2f} Hz over {self.SAMPLE_DURATION_S} s)")
        assert delta >= self.MIN_PACKET_DELTA, (
            f"Only {delta} loop ticks in {self.SAMPLE_DURATION_S} s ({hz:.2f} Hz). "
            f"Need >= {self.MIN_PACKET_DELTA} (>= 3 Hz). "
            "Worker thread may have stalled — check for exception in C++ layer."
        )

    # ── Sub-test 5 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_widget_accepts_live_data(self, drone_link, imu_widget):
        """IMUWidget.update_ui() must handle 20 consecutive live snapshots
        without raising any exception.

        This is the key SAT addition over IT-IMU-001: it drives the full
        C++ DroneState → pybind11 to_dict() → Python widget pipeline and
        confirms there is no key mismatch, type error, or widget crash.
        """
        root, widget = imu_widget
        errors = []

        for i in range(20):
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            try:
                widget.update_ui(data)
                root.update_idletasks()
            except Exception as exc:
                errors.append(f"call {i}: {type(exc).__name__}: {exc}")
            time.sleep(0.1)

        print(f"\n  update_ui() calls: 20   errors: {len(errors)}")
        if errors:
            print(f"  First error: {errors[0]}")

        assert len(errors) == 0, (
            f"{len(errors)} exception(s) during update_ui(): {errors[:3]}"
        )

    # ── Sub-test 6 ────────────────────────────────────────────────────────────

    def test_SAT_IMU_001_widget_cells_show_live_values(self, drone_link, imu_widget):
        """Widget rotation cells must not show '---' after 3 s of live data.

        '---' is the initial placeholder set in _make_cell() before the first
        update_ui() call.  After live data arrives all axis cells must display
        numeric values, confirming the render path is reached end-to-end.
        Tests skip gracefully if the widget is in a tier that does not expose
        axis cells (e.g. window too narrow for FULL/MEDIUM tier).
        """
        root, widget = imu_widget

        t0 = time.monotonic()
        while time.monotonic() - t0 < self.WIDGET_FEED_S:
            widget.update_ui(drone_link.get_latest_state().to_dict())
            root.update_idletasks()
            time.sleep(0.1)

        axes = getattr(widget, "_tier_widgets", {}).get("axes", {})
        if not axes:
            pytest.skip(
                "Widget is not in FULL/MEDIUM tier — no axis cell dict exposed. "
                "Resize the window wider to activate the full display tier."
            )

        stale = [
            f"{key}.rot"
            for key, cells in axes.items()
            if cells.get("rot") and cells["rot"].cget("text").strip() == "---"
        ]

        print(f"\n  Stale cells still showing '---': {stale}")
        assert len(stale) == 0, (
            f"These cells still show '---' after {self.WIDGET_FEED_S} s of live data: "
            f"{stale}. The render path is not being reached for these axes."
        )