# =============================================================================
# test_SAT_IMU_003.py  —  Operator-Visible Alert System
#
# V-Model reference: System Acceptance Testing, SAT-IMU-003
# SYS requirement:   SYS-003
# SRS coverage:      SRS-IMU-005, SRS-IMU-006, SRS-IMU-006a
#
# Objective:
#   Verify that the WARN (amber) and CRIT (red flashing) visual alerts
#   are correctly triggered and visible to the operator in the live GCS
#   when the FC is physically rotated past the gyroscope thresholds.
#
#   This test is SEMI-AUTOMATED: the software validates the alert state
#   machine by inspecting widget internals, but the operator must physically
#   tilt the FC to generate the required angular rates.
#
# Operator procedure:
#   WARN test: Hold FC steady, then rotate it briskly (~45 °/s on any axis)
#              when prompted. The amber WARN state should appear.
#   CRIT test: Rotate the FC very rapidly (~120 °/s) when prompted.
#              The red flashing CRIT state should appear.
#   Recovery:  Set FC back flat. The widget should return to SAFE (green)
#              after the hold-down timers expire (2 s for WARN, 4 s for CRIT).
#
# Automated validation:
#   The test feeds live data to the widget and inspects the _gyro_state
#   dict and _crit_cells registry — the same internal state the unit tests
#   exercise. This confirms the full pipeline (hardware → parser → to_dict()
#   → widget state machine) with no mocking.
#
# Pass criteria (SYS-003):
#   - WARN state enters after brisk rotation (≥ 30 °/s gyro reading)
#   - CRIT state enters after rapid rotation (≥ 100 °/s gyro reading)
#   - CRIT flash ticker active (_cell_flash_job not None) during CRIT
#   - CRIT hold persists for ≥ 4 s after motion stops
#   - All axes return to SAFE after hold-down expires
#
# Board required: F405 V3 connected, Betaflight 4.5.3 running, props OFF.
# =============================================================================

import time
import pytest


class TestOperatorAlertSystem:
    """SAT-IMU-003 — SYS-003: WARN/CRIT alerts visible in the live GCS."""

    POLL_INTERVAL_S   = 0.05    # 20 Hz widget update rate
    MOTION_WINDOW_S   = 5.0     # time given to operator to tilt FC
    POST_MOTION_S     = 0.5     # settle time after motion window closes
    WARN_THRESHOLD    = 30.0    # °/s
    CRIT_THRESHOLD    = 100.0   # °/s
    HOLD_WARN_S       = 2.0     # must remain WARN for 2 s after drop
    HOLD_CRIT_S       = 4.0     # must remain CRIT for 4 s after drop
    RECOVERY_TIMEOUT  = 8.0     # max time to wait for SAFE after motion

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _feed_widget(self, drone_link, imu_widget, duration_s):
        """Feed live to_dict() data to the widget for duration_s seconds.
        Returns list of (timestamp, max_abs_dps_seen) tuples.
        """
        root, widget = imu_widget
        log = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()
            gs = widget.GYRO_SCALE
            max_dps = max(
                abs(data.get("gx", 0)) / gs,
                abs(data.get("gy", 0)) / gs,
                abs(data.get("gz", 0)) / gs,
            )
            log.append((time.monotonic() - t0, max_dps))
            time.sleep(self.POLL_INTERVAL_S)
        return log

    def _any_axis_state(self, widget, state):
        """Return True if any gyro axis is in the given state."""
        return any(
            v["state"] == state
            for v in widget._gyro_state.values()
        )

    def _all_axes_state(self, widget, state):
        """Return True if ALL gyro axes are in the given state."""
        return all(
            v["state"] == state
            for v in widget._gyro_state.values()
        )

    # ── Tests ─────────────────────────────────────────────────────────────────

    def test_SAT_IMU_003_warn_alert_triggers(self, drone_link, imu_widget):
        """WARN state must be entered when gyro reads ≥ 30 °/s.

        Operator: rotate FC briskly (~45 °/s) on any axis when prompted.
        The test waits up to 5 s for the WARN state to appear.
        """
        root, widget = imu_widget

        print(f"\n  [SAT-003 WARN] ► Rotate FC briskly (~45 °/s) on any axis. "
              f"You have {self.MOTION_WINDOW_S:.0f} s ◄")

        warn_seen  = False
        max_dps    = 0.0
        t0 = time.monotonic()

        while time.monotonic() - t0 < self.MOTION_WINDOW_S:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()

            gs = widget.GYRO_SCALE
            dps = max(abs(data.get("gx",0))/gs, abs(data.get("gy",0))/gs,
                      abs(data.get("gz",0))/gs)
            max_dps = max(max_dps, dps)

            if self._any_axis_state(widget, "warn") or \
               self._any_axis_state(widget, "crit"):
                warn_seen = True
                print(f"  Alert state entered at t+{time.monotonic()-t0:.1f}s "
                      f"(max {max_dps:.1f} °/s)")
                break
            time.sleep(self.POLL_INTERVAL_S)

        print(f"  Max gyro rate observed: {max_dps:.1f} °/s")
        assert warn_seen, (
            f"WARN state never entered after {self.MOTION_WINDOW_S} s. "
            f"Max rate seen: {max_dps:.1f} °/s (need ≥ {self.WARN_THRESHOLD} °/s). "
            "Rotate the FC more briskly."
        )

    def test_SAT_IMU_003_warn_hold_persists(self, drone_link, imu_widget):
        """WARN hold must persist for ≥ 2 s after gyro drops below threshold.

        After triggering WARN by tilting, set FC flat and confirm the
        amber state remains for the 2 s hold-down period (SRS-IMU-005).
        """
        root, widget = imu_widget

        # First trigger WARN
        print(f"\n  [SAT-003 WARN HOLD] ► Tilt FC briefly (~45 °/s), then set FLAT. "
              f"You have {self.MOTION_WINDOW_S:.0f} s ◄")

        triggered = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.MOTION_WINDOW_S:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()
            if self._any_axis_state(widget, "warn") or \
               self._any_axis_state(widget, "crit"):
                triggered = True
                break
            time.sleep(self.POLL_INTERVAL_S)

        if not triggered:
            pytest.skip("Could not trigger WARN — rotate faster")

        # Now confirm hold persists for 2 s while feeding safe-rate data
        hold_start = time.monotonic()
        still_held = True
        while time.monotonic() - hold_start < self.HOLD_WARN_S:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()
            if self._all_axes_state(widget, "safe"):
                still_held = False
                break
            time.sleep(self.POLL_INTERVAL_S)

        elapsed = time.monotonic() - hold_start
        print(f"  WARN hold persisted for: {elapsed:.2f} s  (need ≥ {self.HOLD_WARN_S})")
        assert still_held, (
            f"WARN state dropped to SAFE after only {elapsed:.2f} s "
            f"(need ≥ {self.HOLD_WARN_S} s hold-down)"
        )

    def test_SAT_IMU_003_crit_alert_triggers(self, drone_link, imu_widget):
        """CRIT state must be entered when gyro reads ≥ 100 °/s.

        Operator: rotate FC very rapidly (~120 °/s) when prompted.
        The flash ticker must be active (_cell_flash_job not None).
        """
        root, widget = imu_widget

        print(f"\n  [SAT-003 CRIT] ► Rotate FC RAPIDLY (~120 °/s). "
              f"You have {self.MOTION_WINDOW_S:.0f} s ◄")

        crit_seen = False
        max_dps   = 0.0
        t0 = time.monotonic()

        while time.monotonic() - t0 < self.MOTION_WINDOW_S:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()

            gs  = widget.GYRO_SCALE
            dps = max(abs(data.get("gx",0))/gs, abs(data.get("gy",0))/gs,
                      abs(data.get("gz",0))/gs)
            max_dps = max(max_dps, dps)

            if self._any_axis_state(widget, "crit"):
                crit_seen = True
                print(f"  CRIT entered at t+{time.monotonic()-t0:.1f}s "
                      f"(max {max_dps:.1f} °/s)")
                break
            time.sleep(self.POLL_INTERVAL_S)

        print(f"  Max gyro rate observed: {max_dps:.1f} °/s")
        assert crit_seen, (
            f"CRIT state never entered after {self.MOTION_WINDOW_S} s. "
            f"Max rate seen: {max_dps:.1f} °/s (need ≥ {self.CRIT_THRESHOLD} °/s). "
            "Rotate the FC more forcefully."
        )

    def test_SAT_IMU_003_crit_flash_ticker_active(self, drone_link, imu_widget):
        """Flash ticker must be active (_cell_flash_job not None) during CRIT.

        Validates SRS-IMU-006a at system level — the shared flash ticker
        that drives 500 ms cadence must start when a CRIT cell is registered.
        """
        root, widget = imu_widget

        print(f"\n  [SAT-003 FLASH] ► Rotate FC rapidly to trigger CRIT. "
              f"You have {self.MOTION_WINDOW_S:.0f} s ◄")

        ticker_confirmed = False
        t0 = time.monotonic()

        while time.monotonic() - t0 < self.MOTION_WINDOW_S:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()

            if self._any_axis_state(widget, "crit"):
                # Give ticker one cycle to start (up to 600 ms)
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    root.update()
                    if widget._cell_flash_job is not None:
                        ticker_confirmed = True
                        break
                    time.sleep(0.05)
                break
            time.sleep(self.POLL_INTERVAL_S)

        print(f"  Flash ticker active: {ticker_confirmed}  "
              f"job={widget._cell_flash_job}")
        assert ticker_confirmed, (
            "Flash ticker (_cell_flash_job) was None during CRIT state. "
            "The _register_crit() → _start_cell_flash() chain has a defect."
        )

    def test_SAT_IMU_003_recovery_to_safe(self, drone_link, imu_widget):
        """All axes must return to SAFE after hold-down expires.

        Set FC flat after any prior motion. Wait up to 8 s for all
        gyro axes to return to 'safe' state (CRIT hold = 4 s + margin).
        Confirms the hold-down timer expires correctly in the live system.
        """
        root, widget = imu_widget

        print(f"\n  [SAT-003 RECOVERY] Set FC FLAT and STILL. "
              f"Waiting up to {self.RECOVERY_TIMEOUT:.0f} s for SAFE state...")

        recovered = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.RECOVERY_TIMEOUT:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()
            if self._all_axes_state(widget, "safe"):
                recovered = True
                elapsed = time.monotonic() - t0
                print(f"  All axes returned to SAFE after {elapsed:.1f} s")
                break
            time.sleep(self.POLL_INTERVAL_S)

        assert recovered, (
            f"Gyro axes did not return to SAFE within {self.RECOVERY_TIMEOUT} s. "
            f"Current states: { {k: v['state'] for k, v in widget._gyro_state.items()} }. "
            "Place FC flat and still. If still failing, check hold timer logic."
        )
        # Flash ticker must have stopped
        assert widget._cell_flash_job is None, (
            "Flash ticker still running after all axes returned to SAFE. "
            "_unregister_crit() or the ticker self-cancel logic has a defect."
        )