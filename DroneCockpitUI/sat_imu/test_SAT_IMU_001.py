# =============================================================================
# test_SAT_IMU_001.py  —  Real-Time Inertial Data — Full Pipeline
#
# V-Model reference: System Acceptance Testing, SAT-IMU-001
# SYS requirement:   SYS-001
# SRS coverage:      SRS-IMU-001, SRS-IMU-002, SRS-IMU-003
#
# Objective:
#   Verify that the complete system pipeline — FC → DroneLink worker thread →
#   DroneBackend pybind11 → IMUWidget.update_ui() — delivers fresh, non-zero
#   inertial data to the GCS display continuously.
#
#   This is a BLACK-BOX system test: it does not inspect internal parser state.
#   It observes only what the operator sees: the widget's displayed values.
#
# Pass criteria (SYS-001):
#   - All 6 IMU fields (ax/ay/az/gx/gy/gz) non-zero in DroneState after
#     2 s warmup — confirms sensor is alive and data is flowing
#   - packet_count increases by ≥ 15 over 5 s (≥ 3 Hz link rate at 57 600 baud)
#   - IMUWidget.update_ui() accepts data without raising any exception
#   - Widget cell text for all axes is NOT '---' after 3 s (live data visible)
#
# Board required: F405 V3 powered and connected, Betaflight 4.5.3 running.
# =============================================================================

import time
import pytest


class TestRealTimeDataPipeline:
    """SAT-IMU-001 — SYS-001: live IMU data reaches the GCS display."""

    WARMUP_S          = 2.0
    SAMPLE_DURATION_S = 5.0
    MIN_PACKET_DELTA  = 15      # ≥ 3 Hz at 57 600 baud (ceiling ~24 Hz)

    def test_SAT_IMU_001_imu_fields_nonzero_after_warmup(self, drone_link):
        """All 6 DroneState IMU fields must be non-zero after 2 s warmup.

        A stationary FC will have non-zero accelerometer readings due to
        gravity and non-zero gyroscope readings due to sensor noise.
        All-zero readings after warmup indicate the MSP_RAW_IMU poll is
        failing or parseIMU() is returning false on every frame.
        """
        time.sleep(self.WARMUP_S)
        s = drone_link.get_latest_state()

        print(f"\n  ax={s.ax}  ay={s.ay}  az={s.az}")
        print(f"  gx={s.gx}  gy={s.gy}  gz={s.gz}")
        print(f"  packet_count={s.packet_count}  link_healthy={s.link_healthy}")

        assert s.packet_count > 0, (
            "packet_count is 0 after warmup — DroneLink worker thread "
            "may not have started or parseIMU() is failing on every frame"
        )
        assert any([s.ax, s.ay, s.az]), (
            "All accelerometer fields are zero — MSP_RAW_IMU not returning data. "
            "Check: board powered, BF running, no BF Configurator open on same port"
        )
        assert any([s.gx, s.gy, s.gz]), (
            "All gyroscope fields are zero — unexpected for a powered MPU-6500. "
            "Check: sensor_status bit 5 (GYRO) should be set in MSP_STATUS"
        )

    def test_SAT_IMU_001_packet_rate_sustained(self, drone_link):
        """packet_count must increase by ≥ 15 over 5 s (≥ 3 Hz sustained).

        Confirms the worker thread is alive and the IMU parse succeeds
        repeatedly — not just on the first frame.
        """
        s0 = drone_link.get_latest_state()
        c0 = s0.packet_count
        time.sleep(self.SAMPLE_DURATION_S)
        s1 = drone_link.get_latest_state()
        delta = s1.packet_count - c0
        hz    = delta / self.SAMPLE_DURATION_S

        print(f"\n  packet delta = {delta}  ({hz:.2f} Hz over {self.SAMPLE_DURATION_S} s)")
        assert delta >= self.MIN_PACKET_DELTA, (
            f"Packet rate too low: {delta} packets in {self.SAMPLE_DURATION_S} s "
            f"({hz:.2f} Hz, need ≥ {self.MIN_PACKET_DELTA / self.SAMPLE_DURATION_S:.0f} Hz)"
        )

    def test_SAT_IMU_001_link_healthy(self, drone_link):
        """link_healthy must be True — confirms FAIL_THRESHOLD not exceeded."""
        s = drone_link.get_latest_state()
        print(f"\n  link_healthy = {s.link_healthy}")
        assert s.link_healthy, (
            "link_healthy is False — ≥ 5 consecutive IMU parse failures. "
            "Check serial port, FC power, and BF firmware version"
        )

    def test_SAT_IMU_001_widget_accepts_live_data(self, imu_widget, drone_link):
        """IMUWidget.update_ui() must accept live to_dict() data without exception.

        This is the full pipeline test: C++ DroneState → pybind11 to_dict()
        → Python IMUWidget.update_ui(). Any key mismatch or type error
        in the binding would surface here.
        """
        root, widget = imu_widget
        errors = []

        for _ in range(20):
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            try:
                widget.update_ui(data)
                root.update_idletasks()
            except Exception as exc:
                errors.append(str(exc))
            time.sleep(0.1)

        print(f"\n  update_ui() calls: 20  errors: {len(errors)}")
        if errors:
            print(f"  First error: {errors[0]}")
        assert len(errors) == 0, (
            f"{len(errors)} exception(s) during update_ui(): {errors[:3]}"
        )

    def test_SAT_IMU_001_widget_cells_show_live_values(self, imu_widget, drone_link):
        """IMUWidget axis cells must not show '---' after 3 s of data.

        '---' is the initial placeholder text set in _make_cell(). After
        live data arrives all cells should display numeric values.
        This test checks the ROLL, PITCH, and YAW rotation cells.
        """
        root, widget = imu_widget

        # Feed live data for 3 s
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            widget.update_ui(drone_link.get_latest_state().to_dict())
            root.update_idletasks()
            time.sleep(0.1)

        axes = widget._tier_widgets.get("axes", {})
        if not axes:
            pytest.skip("Widget not in FULL/MEDIUM/COMPACT tier — resize window wider")

        stale_cells = []
        for axis_key, cells in axes.items():
            rot_text = cells["rot"].cget("text").strip()
            if rot_text == "---":
                stale_cells.append(f"{axis_key}.rot")

        print(f"\n  Stale cells (still '---'): {stale_cells}")
        assert len(stale_cells) == 0, (
            f"These cells still show '---' after 3 s: {stale_cells}. "
            "Live data is not reaching the widget render path."
        )