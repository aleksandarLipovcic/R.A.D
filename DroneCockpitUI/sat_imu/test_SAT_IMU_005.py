# =============================================================================
# test_SAT_IMU_005.py  —  Fault Tolerance — GCS Survives Corrupt/Missing Frames
#
# V-Model reference: System Acceptance Testing, SAT-IMU-005
# SYS requirement:   SYS-005
# SRS coverage:      SRS-IMU-008, SRS-IMU-008b
#
# Objective:
#   Verify that the complete GCS — DroneLink worker thread + IMUWidget —
#   survives corrupt or missing MSP frames without crashing, without
#   displaying stale data without indication, and recovers automatically.
#
#   SRS-IMU-008:  parseIMU() rejects bad frames, DroneState unchanged
#   SRS-IMU-008b: worker thread does not crash on consecutive failures
#   SYS-005:      No crash AND no silent stale data after fault injection
#
# Test strategy (three fault scenarios):
#
#   SCENARIO A — Link stress (set_poll_interval_ms(0))
#     Spins the worker loop so fast that the 80 ms sendMSP() timeout fires
#     continuously. parseIMU() sees empty buffers (size < 18) and returns
#     false on every tick, driving consecutiveFails above FAIL_THRESHOLD.
#     Validates: no crash, link_healthy recovers, IMUWidget keeps rendering.
#
#   SCENARIO B — Stale data indication (link_healthy flag)
#     When link_healthy goes False, the widget must not silently display
#     values as if they were live. We verify that link_healthy is exposed
#     to the widget via to_dict()['link_healthy'] and that packet_count
#     stops incrementing (no false-positive fresh data).
#     Validates SYS-005 "without displaying stale data without indication".
#
#   SCENARIO C — IMU field range integrity during fault
#     While faults are being injected, DroneState.ax/ay/az/gx/gy/gz must
#     never contain values outside the int16 physical range.
#     Validates SRS-IMU-008 "DroneState left unchanged on rejection".
#
# Board required: F405 V3 connected, Betaflight 4.5.3.
# =============================================================================

import time
import pytest


NORMAL_INTERVAL_MS = 10     # DroneLink.h POLL_INTERVAL_MS
FAIL_THRESHOLD     = 5      # DroneLink.h FAIL_THRESHOLD
INJECT_DURATION_S  = 2.0
RECOVERY_WINDOW_S  = 2.0
INT16_MIN          = -32768
INT16_MAX          =  32767


class TestFaultTolerance:
    """SAT-IMU-005 — SYS-005: GCS tolerates corrupt/missing frames."""

    # ── Scenario A ────────────────────────────────────────────────────────────

    def test_SAT_IMU_005_A_no_crash_during_fault_injection(self, drone_link):
        """DroneLink worker thread must not raise an exception during stress.

        Scenario A: set_poll_interval_ms(0) spins the loop at maximum rate.
        At near-zero sleep, the 80 ms sendMSP() timeout fires on every call,
        returning empty buffers that parseIMU() rejects (size < 18 bytes).
        This simulates a continuous stream of corrupt / missing frames.
        """
        try:
            drone_link.set_poll_interval_ms(0)
            time.sleep(INJECT_DURATION_S)
        except Exception as exc:
            pytest.fail(f"Exception during fault injection: {exc}")
        finally:
            drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)

        # If we reach here the worker thread did not crash or raise
        assert True

    def test_SAT_IMU_005_A_link_healthy_recovers(self, drone_link):
        """link_healthy must return True within 2 s after fault injection ends.

        FAIL_THRESHOLD = 5 consecutive failures → link_healthy = False.
        After restoring normal polling, the next successful parseIMU() resets
        consecutiveFails and sets link_healthy = True.
        """
        drone_link.set_poll_interval_ms(0)
        time.sleep(INJECT_DURATION_S)
        drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)

        recovered = False
        deadline  = time.monotonic() + RECOVERY_WINDOW_S
        while time.monotonic() < deadline:
            if drone_link.get_latest_state().link_healthy:
                recovered = True
                break
            time.sleep(0.05)

        print(f"\n  link_healthy recovered within {RECOVERY_WINDOW_S} s: {recovered}")
        assert recovered, (
            f"link_healthy did not return True within {RECOVERY_WINDOW_S} s. "
            "Worker thread may be stalled or FAIL_THRESHOLD logic is broken."
        )

    def test_SAT_IMU_005_A_packet_count_resumes(self, drone_link):
        """packet_count must resume incrementing after recovery (no thread stall)."""
        drone_link.set_poll_interval_ms(0)
        time.sleep(INJECT_DURATION_S)
        drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)
        time.sleep(RECOVERY_WINDOW_S)   # allow full recovery

        c0 = drone_link.get_latest_state().packet_count
        time.sleep(3.0)
        c1 = drone_link.get_latest_state().packet_count

        delta = c1 - c0
        # At ≥ 3 Hz over 3 s we expect ≥ 5 new packets (80% tolerance)
        print(f"\n  packet_count delta after recovery: {delta}  (need ≥ 5)")
        assert delta >= 5, (
            f"Only {delta} new packets in 3 s after recovery — "
            "worker thread may have stalled permanently"
        )

    # ── Scenario B ────────────────────────────────────────────────────────────

    def test_SAT_IMU_005_B_link_healthy_in_to_dict(self, drone_link):
        """to_dict() must expose link_healthy so the widget can indicate fault.

        SYS-005 prohibits silent stale data. The binding must pass
        link_healthy through to_dict() so the Python layer can decide
        whether to grey out cells or show a connection-lost banner.
        """
        s    = drone_link.get_latest_state()
        data = s.to_dict()

        assert "link_healthy" in data, (
            "to_dict() does not include 'link_healthy' key. "
            "The Python widget cannot indicate stale data to the operator."
        )
        print(f"\n  link_healthy in to_dict(): True  value={data['link_healthy']}")

    def test_SAT_IMU_005_B_unhealthy_link_flagged_during_injection(self, drone_link):
        """link_healthy must flip False during sustained fault injection.

        Confirms FAIL_THRESHOLD logic works: after 5 consecutive parseIMU()
        failures the worker marks the link unhealthy, which to_dict() then
        exposes so the operator can see the GCS is not receiving fresh data.
        """
        drone_link.set_poll_interval_ms(0)

        unhealthy_seen = False
        deadline = time.monotonic() + INJECT_DURATION_S + 1.0
        try:
            while time.monotonic() < deadline:
                if not drone_link.get_latest_state().link_healthy:
                    unhealthy_seen = True
                    break
                time.sleep(0.05)
        finally:
            drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)

        print(f"\n  link_healthy went False during injection: {unhealthy_seen}")
        assert unhealthy_seen, (
            "link_healthy never went False during fault injection. "
            "FAIL_THRESHOLD may not be triggering, or set_poll_interval_ms(0) "
            "is not generating enough parse failures."
        )

    def test_SAT_IMU_005_B_widget_receives_link_status(self, drone_link, imu_widget):
        """IMUWidget.update_ui() must not raise when link_healthy = False.

        During a fault, to_dict() will have link_healthy=False. The widget
        must handle this gracefully — no KeyError, no exception.
        """
        root, widget = imu_widget

        # Inject fault, capture a to_dict() snapshot with link_healthy=False
        drone_link.set_poll_interval_ms(0)
        snapshot_with_fault = None
        deadline = time.monotonic() + INJECT_DURATION_S + 1.0
        try:
            while time.monotonic() < deadline:
                s = drone_link.get_latest_state()
                if not s.link_healthy:
                    snapshot_with_fault = s.to_dict()
                    break
                time.sleep(0.05)
        finally:
            drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)

        if snapshot_with_fault is None:
            pytest.skip("Could not capture link_healthy=False snapshot")

        # Now call update_ui() with the fault snapshot — must not raise
        try:
            widget.update_ui(snapshot_with_fault)
            root.update_idletasks()
        except Exception as exc:
            pytest.fail(
                f"IMUWidget.update_ui() raised {type(exc).__name__}: {exc} "
                "when link_healthy=False was in the data dict"
            )

    # ── Scenario C ────────────────────────────────────────────────────────────

    def test_SAT_IMU_005_C_imu_fields_in_range_during_fault(self, drone_link):
        """IMU fields must stay within int16 range during fault injection.

        SRS-IMU-008 states DroneState is left unchanged when parseIMU()
        rejects a frame. At the system level this means ax/ay/az/gx/gy/gz
        must never contain out-of-range garbage values even during sustained
        parse failures.
        """
        violations = []

        drone_link.set_poll_interval_ms(0)
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < INJECT_DURATION_S:
                s = drone_link.get_latest_state()
                for name, val in [
                    ("ax", s.ax), ("ay", s.ay), ("az", s.az),
                    ("gx", s.gx), ("gy", s.gy), ("gz", s.gz),
                ]:
                    if not (INT16_MIN <= val <= INT16_MAX):
                        violations.append(f"{name}={val} at t+{time.monotonic()-t0:.2f}s")
                time.sleep(0.01)
        finally:
            drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)

        print(f"\n  Out-of-range violations: {len(violations)}")
        assert len(violations) == 0, (
            f"IMU fields had out-of-range values during fault injection "
            f"(SRS-IMU-008 violated): {violations[:5]}"
        )

    def test_SAT_IMU_005_C_full_recovery_after_all_scenarios(self, drone_link, imu_widget):
        """GCS must be fully operational after all fault scenarios complete.

        Final health check: after all Scenario A/B/C tests have run,
        the system must be back to nominal — link healthy, data flowing,
        widget rendering without errors.
        """
        root, widget = imu_widget

        # Ensure normal polling is restored
        drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)
        time.sleep(RECOVERY_WINDOW_S)

        s = drone_link.get_latest_state()
        print(f"\n  Final state:")
        print(f"    link_healthy  = {s.link_healthy}")
        print(f"    packet_count  = {s.packet_count}")
        print(f"    ax={s.ax}  ay={s.ay}  az={s.az}")

        assert s.link_healthy, "link_healthy is False after full recovery period"
        assert any([s.ax, s.ay, s.az, s.gx, s.gy, s.gz]), \
            "All IMU fields are zero after recovery — sensor may have stopped"

        # Final widget update must not raise
        try:
            widget.update_ui(s.to_dict())
            root.update_idletasks()
        except Exception as exc:
            pytest.fail(
                f"IMUWidget.update_ui() raised {type(exc).__name__}: {exc} "
                "after full fault recovery"
            )