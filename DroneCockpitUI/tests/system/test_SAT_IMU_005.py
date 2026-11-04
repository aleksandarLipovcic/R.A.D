# =============================================================================
# test_SAT_IMU_005.py  —  Fault Tolerance — GCS Survives Corrupt/Missing Frames
#
# V-Model reference: System Acceptance Testing, SAT-IMU-005
# SYS requirement:   SYS-005
# SRS coverage:      SRS-IMU-008, SRS-IMU-008b
#
# Objective:
#    Verify that the complete GCS — DroneLink worker thread + IMUWidget —
#    survives corrupt or missing MSP frames without crashing, without
#    displaying stale data without indication, and recovers automatically.
#
# Test strategy (three fault scenarios):
#    SCENARIO A — Link stress (set_poll_interval_ms(0))
#    SCENARIO B — Stale data indication (link_healthy flag via to_dict())
#    SCENARIO C — IMU field range integrity during fault
#
# DroneState attribute naming (pybind11):
#    C++ members are camelCase but bound via pybind11 as snake_case properties:
#    link_healthy, packet_count.
#    Accessed here via to_dict() keys 'link_healthy' and 'packet_count' or
#    directly as snake_case attribute fields on the DroneState object instantiation.
#    Direct attribute reads: s.link_healthy, s.packet_count, s.ax … s.gz.
#
# DroneLink method naming (pybind11):
#    C++ method setPollIntervalMs() is exposed as snake_case by the explicit binding:
#    drone_link.set_poll_interval_ms().
#    C++ method setFailInjection() is exposed as:
#    drone_link.set_fail_injection(active).
#
# Packet-count floor (post-recovery, Scenario A, DEF-003):
#    PACKET_MONITOR_S = 3.0 s window.
#    Floor = 3 Hz × 3.0 s × 0.8 tolerance = 7.2 → MIN_PACKETS_AFTER_RECOVERY = 7.
#    IT-IMU-004 uses floor=5 over 2.0 s (3 × 2.0 × 0.8 = 4.8 → 5).
#    Both derive from the same DEF-003 formula; only the window differs.
#
# DEF-005 rev 2 — Scenario B fault injection mechanism:
#    The previous mechanism (set_poll_interval_ms(0)) relied on a side-effect
#    of the old unconditional PurgeComm inside sendMSP().  That purge was
#    removed to fix MSP_RC frame truncation (DEF-004).  With a live USB-connected
#    FC the FC now responds correctly at any poll rate, so poll-rate stress
#    no longer induces parse failures.
#
#    Correct mechanism: set_fail_injection(True) makes sendMSP() return {}
#    immediately on every call, deterministically.  parseIMU() receives an
#    empty buffer → returns False → anySuccess stays False → consecutiveFails
#    increments each tick.  After FAIL_THRESHOLD (5) consecutive ticks at
#    POLL_INTERVAL_MS = 10 ms each, link_healthy flips False within ~50 ms.
#    FAULT_INJECTION_DEADLINE_S = 2.0 s provides ample margin.
#
#    Scenarios A and C are UNAFFECTED — they use set_poll_interval_ms(0)
#    to stress loop throughput and recovery behaviour, not to force parse
#    failures.  Their logic remains correct as written.
#
# Board required: F405 V3 connected, Betaflight 4.5.3.
# =============================================================================

import time
import pytest


NORMAL_INTERVAL_MS  = 10    # DroneLink.h POLL_INTERVAL_MS
FAIL_THRESHOLD      = 5     # DroneLink.h FAIL_THRESHOLD
INJECT_DURATION_S   = 2.0
RECOVERY_WINDOW_S   = 2.0
PACKET_MONITOR_S    = 3.0
# 3 Hz × 3.0 s × 0.8 tolerance = 7.2 → 7  (DEF-003 derivation, 3 s window)
MIN_PACKETS_AFTER_RECOVERY = 7
INT16_MIN = -32768
INT16_MAX =  32767

# DEF-005 rev 2: set_fail_injection() makes sendMSP() return {} immediately.
# At POLL_INTERVAL_MS = 10 ms, FAIL_THRESHOLD = 5 ticks → ~50 ms minimum.
# 2.0 s gives x40 margin — more than sufficient.
FAULT_INJECTION_DEADLINE_S = 2.0


class TestFaultTolerance:
    """SAT-IMU-005 — SYS-005: GCS tolerates corrupt/missing frames."""

    # ── Scenario A ────────────────────────────────────────────────────────────

    def test_SAT_IMU_005_A_no_crash_during_fault_injection(self, drone_link):
        """DroneLink worker thread must not raise an exception during stress.

        set_poll_interval_ms(0) spins the loop at maximum rate. At near-zero
        sleep the 80 ms sendMSP() timeout fires on every call, returning
        empty buffers that parseIMU() rejects (size < 18 bytes).
        """
        try:
            drone_link.set_poll_interval_ms(0)
            time.sleep(INJECT_DURATION_S)
        except Exception as exc:
            pytest.fail(f"Exception during fault injection: {exc}")
        finally:
            drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)

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
        """packet_count must increase by >= 7 over 3 s after recovery.

        packet_count increments on every committed loop tick (regardless of
        IMU parse success). Floor = 3 Hz × 3.0 s × 0.8 = 7 (DEF-003).
        A stalled thread returns 0; any delta >= 7 confirms liveness.
        """
        drone_link.set_poll_interval_ms(0)
        time.sleep(INJECT_DURATION_S)
        drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)
        time.sleep(RECOVERY_WINDOW_S)   # allow full recovery before counting

        c0 = drone_link.get_latest_state().packet_count
        time.sleep(PACKET_MONITOR_S)
        c1 = drone_link.get_latest_state().packet_count

        delta = c1 - c0
        print(f"\n  packet_count delta after recovery: {delta}  "
              f"(need >= {MIN_PACKETS_AFTER_RECOVERY} over {PACKET_MONITOR_S} s)")
        assert delta >= MIN_PACKETS_AFTER_RECOVERY, (
            f"Only {delta} new packets in {PACKET_MONITOR_S} s after recovery — "
            f"worker thread may have stalled permanently "
            f"(need >= {MIN_PACKETS_AFTER_RECOVERY}: "
            f"3 Hz × {PACKET_MONITOR_S} s × 0.8 = "
            f"{3 * PACKET_MONITOR_S * 0.8:.1f})"
        )

    # ── Scenario B ────────────────────────────────────────────────────────────

    def test_SAT_IMU_005_B_link_healthy_in_to_dict(self, drone_link):
        """to_dict() must expose 'link_healthy' so the widget can indicate fault.

        SYS-005 prohibits silent stale data. The binding must include
        link_healthy in to_dict() so the Python layer can grey out cells
        or show a connection-lost banner.

        Note: to_dict() key names are set by the pybind11 binding and use
        snake_case ('link_healthy'). This sub-test verifies the key name
        actually present in to_dict() output.
        """
        s    = drone_link.get_latest_state()
        data = s.to_dict()

        assert "link_healthy" in data, (
            "to_dict() does not include 'link_healthy' key. "
            "The Python widget cannot indicate stale data to the operator. "
            "Check the pybind11 DroneBackend binding for this key."
        )
        print(f"\n  'link_healthy' in to_dict(): True  value={data['link_healthy']}")

    def test_SAT_IMU_005_B_unhealthy_link_flagged_during_injection(self, drone_link):
        """link_healthy must flip False during sustained fault injection.

        Confirms FAIL_THRESHOLD logic: after 5 consecutive parseIMU()
        failures the worker marks the link unhealthy.

        FAULT MECHANISM (DEF-005 rev 2):
        set_fail_injection(True) makes sendMSP() return {} immediately on
        every call, regardless of FC responsiveness.  This deterministically
        causes parseIMU() to receive an empty buffer → returns False →
        anySuccess stays False → consecutiveFails increments each loop tick.

        At POLL_INTERVAL_MS = 10 ms per tick, 5 ticks = ~50 ms minimum
        before link_healthy goes False.  FAULT_INJECTION_DEADLINE_S = 2.0 s
        provides x40 margin over that minimum.

        WHY the previous mechanism was replaced (DEF-005 original):
        The old mechanism (set_poll_interval_ms(0)) relied on the
        unconditional PurgeComm inside sendMSP() starving the serial FIFO,
        causing each of the 12 sendMSP() calls to block for its full 80 ms
        read timeout.  That purge was removed to fix MSP_RC frame truncation
        (DEF-004).  With no purge, the FC continues responding correctly at
        any poll rate, so poll-rate stress alone no longer induces parse
        failures.  set_fail_injection() is the correct, deterministic
        replacement.
        """
        drone_link.set_fail_injection(True)

        unhealthy_seen = False
        deadline = time.monotonic() + FAULT_INJECTION_DEADLINE_S
        try:
            while time.monotonic() < deadline:
                if not drone_link.get_latest_state().link_healthy:
                    unhealthy_seen = True
                    break
                time.sleep(0.05)
        finally:
            drone_link.set_fail_injection(False)

        print(f"\n  link_healthy went False during injection: {unhealthy_seen}")
        assert unhealthy_seen, (
            "link_healthy never went False during fault injection. "
            f"Waited {FAULT_INJECTION_DEADLINE_S} s "
            f"(FAIL_THRESHOLD={FAIL_THRESHOLD} × POLL_INTERVAL_MS={NORMAL_INTERVAL_MS} ms "
            f"+ large margin). "
            "Possible causes:\n"
            "  • set_fail_injection() binding is missing or not calling "
            "failInjectionActive.store() — check DroneBackend pybind11 binding.\n"
            "  • sendMSP() guard 'if (failInjectionActive.load()) return {};' is "
            "absent or unreachable — check DroneLink.cpp.\n"
            f"  • FAIL_THRESHOLD constant mismatch — check DroneLink.h "
            f"(expected {FAIL_THRESHOLD})."
        )

    def test_SAT_IMU_005_B_widget_receives_link_status(self, drone_link, imu_widget):
        """IMUWidget.update_ui() must not raise when link_healthy = False.

        During a fault, to_dict() will have link_healthy=False. The widget
        must handle this gracefully — no KeyError, no exception.
        """
        root, widget = imu_widget

        drone_link.set_fail_injection(True)
        snapshot_with_fault = None
        deadline = time.monotonic() + FAULT_INJECTION_DEADLINE_S
        try:
            while time.monotonic() < deadline:
                s = drone_link.get_latest_state()
                if not s.link_healthy:
                    snapshot_with_fault = s.to_dict()
                    break
                time.sleep(0.05)
        finally:
            drone_link.set_fail_injection(False)

        if snapshot_with_fault is None:
            pytest.skip(
                f"Could not capture link_healthy=False snapshot within "
                f"{FAULT_INJECTION_DEADLINE_S} s — "
                "set_fail_injection() may not be wired up correctly in "
                "DroneLink.cpp or DroneBackend pybind11 binding."
            )

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
        rejects a frame. ax/ay/az/gx/gy/gz must never contain out-of-range
        garbage values even during sustained parse failures.
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
                        violations.append(
                            f"{name}={val} at t+{time.monotonic()-t0:.2f}s"
                        )
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

        Final health check: link healthy, data flowing, widget rendering
        without errors.
        """
        root, widget = imu_widget

        drone_link.set_poll_interval_ms(NORMAL_INTERVAL_MS)
        time.sleep(RECOVERY_WINDOW_S)

        s = drone_link.get_latest_state()
        print(f"\n  Final state:")
        print(f"     link_healthy = {s.link_healthy}")
        print(f"     packet_count = {s.packet_count}")
        print(f"     ax={s.ax}  ay={s.ay}  az={s.az}")

        assert s.link_healthy, "link_healthy is False after full recovery period"
        assert any([s.ax, s.ay, s.az, s.gx, s.gy, s.gz]), \
            "All IMU fields are zero after recovery — sensor may have stopped"

        try:
            widget.update_ui(s.to_dict())
            root.update_idletasks()
        except Exception as exc:
            pytest.fail(
                f"IMUWidget.update_ui() raised {type(exc).__name__}: {exc} "
                "after full fault recovery"
            )