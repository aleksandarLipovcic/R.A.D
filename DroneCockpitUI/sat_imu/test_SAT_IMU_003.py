# =============================================================================
# test_SAT_IMU_003.py  —  Operator Alert: WARN and CRIT Visual Indicators
#
# V-Model reference: System Acceptance Testing, SAT-IMU-003
# SYS requirement:   SYS-003
# SRS coverage:      SRS-IMU-005, SRS-IMU-006, SRS-IMU-006a
#
# Objective:
#   Verify that the WARN (amber) and CRIT (red flashing) visual alerts are
#   correctly triggered and visible to the operator in the live GCS when the
#   FC is physically rotated past the gyroscope thresholds.
#
#   This test is SEMI-AUTOMATED: the software validates the alert state
#   machine by inspecting widget internals, but the operator must physically
#   tilt the FC to generate the required angular rates.
#
# ── SCALE CONSTANT (SRS-IMU-004c) ─────────────────────────────────────────────
#
#   IMUWidget.GYRO_SCALE = 16.4  (MPU-6500 ±2000 °/s: 16.4 LSB per °/s)
#
#   The widget receives raw ADC counts from DroneBackend.to_dict() and divides
#   by GYRO_SCALE to produce °/s for display and threshold comparison:
#
#       displayed_dps = raw_gx / 16.4
#
#   This test reads GYRO_SCALE directly from the widget instance, so it
#   remains correct even if the constant is later changed.
#
#   WARN at 30 °/s  → raw_gx ≈ 492 counts   (30 × 16.4)
#   CRIT at 100 °/s → raw_gx ≈ 1640 counts  (100 × 16.4)
#
# ── HOW TO RUN FROM VISUAL STUDIO ─────────────────────────────────────────────
#
#   Tools ▸ Options ▸ Python ▸ Testing ▸ Additional pytest arguments:  -s -v
#   A MessageBox dialog appears before each rotation — click OK then tilt.
#   A double-beep signals each prompt; a rising tone confirms threshold capture.
#
# Pass Criteria (SYS-003):
#   - WARN entered after brisk rotation (≥ 30 °/s)
#   - WARN hold persists ≥ 2 s after rate drops (SRS-IMU-005)
#   - CRIT entered after rapid rotation (≥ 100 °/s)
#   - CRIT flash ticker active (_cell_flash_job not None)
#   - CRIT hold persists ≥ 4 s after rate drops (SRS-IMU-006)
#   - All axes return to SAFE after hold-down expires
#   - Flash ticker self-cancels when all axes return to SAFE (SRS-IMU-006a)
#
# Board required: F405 V3 connected, Betaflight 4.5.3 running, props OFF.
# =============================================================================

import ctypes
import time
import tkinter as tk
from typing import Optional, Tuple
import winsound

import pytest


# =============================================================================
# Operator notification helpers
# =============================================================================

_MB_OK              = 0x00
_MB_ICONINFORMATION = 0x40
_MB_ICONWARNING     = 0x30
_MB_TOPMOST         = 0x40000
_MB_SETFOREGROUND   = 0x10000


def _msgbox(title: str, message: str, icon: int = _MB_ICONINFORMATION) -> None:
    """Blocking Win32 MessageBox — appears on top of all windows.

    Execution halts until operator clicks OK, giving time to read and prepare.
    Falls back to print() on non-Windows (CI runners).
    """
    try:
        ctypes.windll.user32.MessageBoxW(
            0, message, title,
            _MB_OK | icon | _MB_TOPMOST | _MB_SETFOREGROUND
        )
    except AttributeError:
        print(f"\n[OPERATOR PROMPT] {title}\n{message}")


def _beep_attention() -> None:
    try:
        winsound.Beep(880, 200)
        time.sleep(0.05)
        winsound.Beep(1100, 300)
    except Exception:
        pass


def _beep_success() -> None:
    try:
        winsound.Beep(660, 150)
        winsound.Beep(880, 200)
    except Exception:
        pass


def _beep_fail() -> None:
    try:
        winsound.Beep(330, 500)
    except Exception:
        pass


# =============================================================================
# TestOperatorAlertSystem
# =============================================================================

class TestOperatorAlertSystem:
    """SAT-IMU-003 — SYS-003: WARN/CRIT alerts visible in the live GCS."""

    POLL_INTERVAL_S  = 0.05    # 20 Hz widget update rate
    MOTION_WINDOW_S  = 5.0     # seconds for operator to perform tilt
    WARN_THRESHOLD   = 30.0    # °/s — SRS-IMU-005
    CRIT_THRESHOLD   = 100.0   # °/s — SRS-IMU-006
    HOLD_WARN_S      = 2.0     # hold-down for WARN (SRS-IMU-005)
    HOLD_CRIT_S      = 4.0     # hold-down for CRIT (SRS-IMU-006)
    RECOVERY_TIMEOUT = 10.0    # max wait for SAFE after motion stops

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _dps_from_data(self, widget, data: dict) -> float:
        """Convert raw gyro counts to max °/s using the widget's own GYRO_SCALE.

        GYRO_SCALE = 16.4 (MPU-6500 ±2000 °/s, per SRS-IMU-004c).
        The widget receives raw ADC counts and divides by 16.4 — this helper
        replicates that calculation so the test observes exactly the same rate
        the widget's state machine sees.
        """
        gs = widget.GYRO_SCALE   # 16.4
        return max(
            abs(data.get("gx", 0)) / gs,
            abs(data.get("gy", 0)) / gs,
            abs(data.get("gz", 0)) / gs,
        )

    def _any_axis_state(self, widget, state: str) -> bool:
        return any(v["state"] == state for v in widget._gyro_state.values())

    def _all_axes_state(self, widget, state: str) -> bool:
        return all(v["state"] == state for v in widget._gyro_state.values())

    def _poll_loop(self, drone_link, imu_widget, duration_s: float,
                   stop_on_state: Optional[str] = None) -> Tuple[bool, float]:
        """Feed live data into the widget for up to duration_s seconds.

        Returns (state_seen, max_dps_observed).
        Stops early if stop_on_state is reached on any axis.
        CRIT always satisfies a stop_on_state='warn' request too.
        """
        root, widget = imu_widget
        state_seen = False
        max_dps    = 0.0
        t0 = time.monotonic()

        while time.monotonic() - t0 < duration_s:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()

            dps = self._dps_from_data(widget, data)
            max_dps = max(max_dps, dps)

            if stop_on_state:
                if self._any_axis_state(widget, stop_on_state) or (
                    stop_on_state == "warn"
                    and self._any_axis_state(widget, "crit")
                ):
                    state_seen = True
                    break

            time.sleep(self.POLL_INTERVAL_S)

        return state_seen, max_dps

    def _live_rate_window(
        self,
        drone_link,
        imu_widget,
        duration_s: float,
        warn_dps: float,
        crit_dps: float,
    ) -> Tuple[bool, float]:
        """Show a live °/s bar while polling for CRIT state.

        Returns (crit_seen, max_dps_observed).

        An always-on-top Tkinter overlay displays:
          • A horizontal bar filling left-to-right as °/s rises.
          • Colour zones: grey (0–warn) → amber (warn–crit) → red (≥crit).
          • A white needle tracking the current reading.
          • Numeric readout and countdown timer.
          • Threshold tick-marks labelled "WARN 30" and "CRIT 100".
          • A green "✓ CRIT CAPTURED" banner on capture.

        The window closes itself as soon as CRIT is confirmed or time expires.
        """
        root_widget, widget = imu_widget

        # ── layout constants ──────────────────────────────────────────
        W, H    = 500, 175
        BAR_X0  = 30
        BAR_X1  = 470
        BAR_Y0  = 60
        BAR_Y1  = 100
        BAR_W   = BAR_X1 - BAR_X0      # 440 px
        MAX_DPS = crit_dps * 2.0        # bar clips at 2× CRIT threshold

        C_BG      = "#0a0a10"
        C_SAFE_Z  = "#2a2a44"
        C_WARN_Z  = "#7a4400"
        C_CRIT_Z  = "#660000"
        C_NEEDLE  = "#ffffff"
        C_WARN_TK = "#C87000"
        C_CRIT_TK = "#CC0000"
        C_TEXT    = "#c0d0e0"
        C_CAPTURE = "#00ff88"

        def _dps_to_x(dps):
            return BAR_X0 + int(min(dps / MAX_DPS, 1.0) * BAR_W)

        warn_x = _dps_to_x(warn_dps)
        crit_x = _dps_to_x(crit_dps)

        def _fill_colour(dps):
            if dps >= crit_dps:  return C_CRIT_TK
            if dps >= warn_dps:  return C_WARN_TK
            return "#4444aa"

        # ── build overlay ─────────────────────────────────────────────
        ov = tk.Toplevel(root_widget)
        ov.title("SAT-IMU-003 — Live Rate Monitor")
        ov.configure(bg=C_BG)
        ov.geometry(f"{W}x{H}+80+80")
        ov.resizable(False, False)
        try:
            ov.attributes("-topmost", True)
        except tk.TclError:
            pass

        cv = tk.Canvas(ov, width=W, height=H, bg=C_BG, highlightthickness=0)
        cv.pack()

        # static background zones
        cv.create_rectangle(BAR_X0, BAR_Y0, warn_x, BAR_Y1,
                            fill=C_SAFE_Z, outline="")
        cv.create_rectangle(warn_x, BAR_Y0, crit_x, BAR_Y1,
                            fill=C_WARN_Z, outline="")
        cv.create_rectangle(crit_x, BAR_Y0, BAR_X1, BAR_Y1,
                            fill=C_CRIT_Z, outline="")

        # threshold tick-marks
        cv.create_line(warn_x, BAR_Y0 - 6, warn_x, BAR_Y1 + 6,
                       fill=C_WARN_TK, width=2)
        cv.create_text(warn_x, BAR_Y1 + 14,
                       text=f"WARN {warn_dps:.0f}", fill=C_WARN_TK,
                       font=("Consolas", 9, "bold"), anchor="n")
        cv.create_line(crit_x, BAR_Y0 - 6, crit_x, BAR_Y1 + 6,
                       fill=C_CRIT_TK, width=2)
        cv.create_text(crit_x, BAR_Y1 + 14,
                       text=f"CRIT {crit_dps:.0f}", fill=C_CRIT_TK,
                       font=("Consolas", 9, "bold"), anchor="n")

        # bar border
        cv.create_rectangle(BAR_X0, BAR_Y0, BAR_X1, BAR_Y1,
                            outline="#334455", width=1)

        # header label
        cv.create_text(W // 2, 12,
                       text="GYRO RATE — CRIT TEST",
                       fill="#4a7a9a",
                       font=("Consolas", 10, "bold"),
                       anchor="center")

        # dynamic items (updated each poll tick)
        fill_rect   = cv.create_rectangle(BAR_X0, BAR_Y0 + 2,
                                          BAR_X0, BAR_Y1 - 2,
                                          fill=C_SAFE_Z, outline="")
        needle      = cv.create_line(BAR_X0, BAR_Y0 - 4,
                                     BAR_X0, BAR_Y1 + 4,
                                     fill=C_NEEDLE, width=3)
        dps_text    = cv.create_text(W // 2, BAR_Y0 - 22,
                                     text="0.0 °/s",
                                     fill=C_TEXT,
                                     font=("Consolas", 15, "bold"),
                                     anchor="center")
        timer_txt   = cv.create_text(W - 10, 10,
                                     text=f"{duration_s:.0f} s",
                                     fill="#445566",
                                     font=("Consolas", 10),
                                     anchor="ne")
        capture_txt = cv.create_text(W // 2, BAR_Y1 + 42,
                                     text="",
                                     fill=C_CAPTURE,
                                     font=("Consolas", 14, "bold"),
                                     anchor="center")

        # ── poll loop ─────────────────────────────────────────────────
        crit_seen = False
        max_dps   = 0.0
        t0        = time.monotonic()

        while True:
            elapsed   = time.monotonic() - t0
            remaining = max(0.0, duration_s - elapsed)

            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)

            dps     = self._dps_from_data(widget, data)
            max_dps = max(max_dps, dps)

            # update bar, needle, readout
            nx = _dps_to_x(dps)
            cv.coords(fill_rect, BAR_X0, BAR_Y0 + 2, nx, BAR_Y1 - 2)
            cv.itemconfig(fill_rect, fill=_fill_colour(dps))
            cv.coords(needle, nx, BAR_Y0 - 4, nx, BAR_Y1 + 4)
            cv.itemconfig(dps_text,  text=f"{dps:>6.1f} °/s")
            cv.itemconfig(timer_txt, text=f"{remaining:.1f} s")

            if self._any_axis_state(widget, "crit"):
                crit_seen = True
                cv.itemconfig(capture_txt, text="✓  CRIT CAPTURED")
                ov.update()
                time.sleep(0.4)   # hold banner visible briefly before close
                break

            ov.update()

            if elapsed >= duration_s:
                break

            time.sleep(self.POLL_INTERVAL_S)

        try:
            ov.destroy()
        except tk.TclError:
            pass

        return crit_seen, max_dps

    # ── Tests ─────────────────────────────────────────────────────────────────

    def test_SAT_IMU_003_warn_alert_triggers(self, drone_link, imu_widget):
        """WARN state must be entered when gyro reads ≥ 30 °/s (SRS-IMU-005).

        Operator: rotate FC briskly (~45 °/s on any axis) within the window.
        Target raw count: ≥ 492 counts on any gyro axis (492 / 16.4 = 30 °/s).
        """
        root, widget = imu_widget

        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — WARN Test",
            f"Rotate the FC BRISKLY (~45 °/s) on ANY axis.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.\n\n"
            f"Target: ≥ {self.WARN_THRESHOLD:.0f} °/s on any axis.\n\n"
            f"(Raw gyro target: ≥ {int(self.WARN_THRESHOLD * widget.GYRO_SCALE)} counts)",
            _MB_ICONWARNING,
        )

        warn_seen, max_dps = self._poll_loop(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            stop_on_state="warn",
        )

        if warn_seen:
            _beep_success()
        else:
            _beep_fail()

        print(f"\n  [SAT-003 WARN] seen={warn_seen}  max={max_dps:.1f} °/s  "
              f"GYRO_SCALE={widget.GYRO_SCALE}")

        assert warn_seen, (
            f"WARN state never entered after {self.MOTION_WINDOW_S:.0f} s.\n"
            f"  Max rate seen     : {max_dps:.1f} °/s\n"
            f"  WARN threshold    : {self.WARN_THRESHOLD:.0f} °/s\n"
            f"  widget.GYRO_SCALE : {widget.GYRO_SCALE}\n"
            "  → Rotate more briskly. Target a visible snap of ~45 °/s."
        )

    def test_SAT_IMU_003_warn_hold_persists(self, drone_link, imu_widget):
        """WARN hold must persist ≥ 2 s after the rate drops (SRS-IMU-005).

        Procedure: trigger WARN, then set FC flat. The amber state must remain
        for the full 2 s hold-down period even with sub-threshold live data.
        """
        root, widget = imu_widget

        # Step 1 — trigger WARN
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — WARN Hold Test (1/2)",
            f"Tilt FC briefly (~45 °/s), then SET IT FLAT immediately.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.",
            _MB_ICONWARNING,
        )

        triggered, _ = self._poll_loop(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            stop_on_state="warn",
        )

        if not triggered:
            pytest.skip(
                "Could not trigger WARN — rotate faster. "
                "(Test skipped so remainder of suite can continue.)"
            )

        # Step 2 — prompt to set flat, then drain until rate drops
        _msgbox(
            "SAT-IMU-003 — WARN Hold Test (2/2)",
            "WARN triggered!\n\n"
            "Now SET the FC FLAT AND STILL and click OK.\n\n"
            f"The amber indicator must stay on for ≥ {self.HOLD_WARN_S:.0f} s.",
            _MB_ICONINFORMATION,
        )

        # Wait for live rate to drop below threshold (hold-down clock starts here)
        drain_deadline = time.monotonic() + 3.0
        while time.monotonic() < drain_deadline:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()
            if self._dps_from_data(widget, data) < self.WARN_THRESHOLD:
                break
            time.sleep(self.POLL_INTERVAL_S)

        # Measure: widget must remain in WARN for the full hold period
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
        print(f"\n  [SAT-003 WARN HOLD] held={still_held}  "
              f"elapsed={elapsed:.2f} s  (need ≥ {self.HOLD_WARN_S:.1f} s)")

        if still_held:
            _beep_success()
        else:
            _beep_fail()

        assert still_held, (
            f"WARN state dropped to SAFE after only {elapsed:.2f} s.\n"
            f"  Required hold : ≥ {self.HOLD_WARN_S:.1f} s (SRS-IMU-005)\n"
            "  → Check hold_until timer in widget._gyro_state_label()."
        )

    def test_SAT_IMU_003_crit_alert_triggers(self, drone_link, imu_widget):
        """CRIT state must be entered when gyro reads ≥ 100 °/s (SRS-IMU-006).

        Operator: rotate FC very rapidly (~120 °/s) within the window.
        Target raw count: ≥ 1640 counts on any gyro axis (1640 / 16.4 = 100 °/s).

        A live bar overlay shows the current rate and threshold markers so the
        operator can see exactly how hard to snap the FC.
        """
        root, widget = imu_widget

        _beep_attention()
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — CRIT Test",
            f"TARGET: ≥ {self.CRIT_THRESHOLD:.0f} °/s on any axis "
            f"(raw ≥ {int(self.CRIT_THRESHOLD * widget.GYRO_SCALE)} counts).\n\n"
            "── HOW TO ACHIEVE THIS ─────────────────────────────────\n"
            "Use a SHORT, SHARP SNAP — NOT a slow sweep.\n\n"
            "  ✓ CORRECT: Snap the FC through ~30–45° in under 0.25 s,\n"
            "             then stop hard. Yaw (horizontal spin) is easiest.\n"
            "             Think of flicking a wrist to crack a whip.\n\n"
            "  ✗ WRONG:   Sweeping slowly through 90° or 180°.\n"
            "             That peaks at ~80–90 °/s — not enough.\n\n"
            "A LIVE RATE BAR will appear after you click OK.\n"
            "Watch the bar cross the red CRIT line.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} s after clicking OK.\n\n"
            "⚠ Props must be OFF. Hold FC securely at the edges.",
            _MB_ICONWARNING,
        )

        crit_seen, max_dps = self._live_rate_window(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            warn_dps=self.WARN_THRESHOLD,
            crit_dps=self.CRIT_THRESHOLD,
        )

        if crit_seen:
            _beep_success()
        else:
            _beep_fail()

        print(f"\n  [SAT-003 CRIT] seen={crit_seen}  max={max_dps:.1f} °/s")

        assert crit_seen, (
            f"CRIT state never entered after {self.MOTION_WINDOW_S:.0f} s.\n"
            f"  Max rate seen     : {max_dps:.1f} °/s\n"
            f"  CRIT threshold    : {self.CRIT_THRESHOLD:.0f} °/s\n"
            f"  widget.GYRO_SCALE : {widget.GYRO_SCALE}\n"
            f"  Shortfall         : {self.CRIT_THRESHOLD - max_dps:.1f} °/s "
            f"({(self.CRIT_THRESHOLD - max_dps) / self.CRIT_THRESHOLD * 100:.0f}% below threshold)\n\n"
            "  ── Technique reminder ──────────────────────────────────────\n"
            "  SNAP through 30–45° in < 0.25 s, then stop hard (yaw axis).\n"
            "  Do NOT sweep slowly through a wide arc.\n"
            "  A 40° snap in 0.25 s = ~160 °/s average — easily above 100.\n"
            "  A 90° sweep in 1 s   = ~90 °/s average — not enough.\n"
            "  ────────────────────────────────────────────────────────────"
        )

    def test_SAT_IMU_003_crit_flash_ticker_active(self, drone_link, imu_widget):
        """Flash ticker must be active during CRIT state (SRS-IMU-006a).

        Validates that _register_crit() → _start_cell_flash() starts the
        shared 500 ms ticker. The test waits up to 600 ms for root.after()
        to fire. root.update() (not update_idletasks()) is required to
        drain the after() queue when the Tk mainloop is not running.
        """
        root, widget = imu_widget

        _beep_attention()
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — Flash Ticker Test",
            f"Rotate the FC VERY RAPIDLY to trigger CRIT.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.",
            _MB_ICONWARNING,
        )

        crit_triggered   = False
        ticker_confirmed = False
        t0 = time.monotonic()

        while time.monotonic() - t0 < self.MOTION_WINDOW_S:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()

            if self._any_axis_state(widget, "crit"):
                crit_triggered = True
                # Give the after() scheduler up to 600 ms (> one 500 ms period).
                # root.update() processes the after() queue; update_idletasks()
                # does NOT — this is required for the ticker to start.
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    root.update()
                    if widget._cell_flash_job is not None:
                        ticker_confirmed = True
                        break
                    time.sleep(0.05)
                break

            time.sleep(self.POLL_INTERVAL_S)

        if ticker_confirmed:
            _beep_success()
        elif not crit_triggered:
            _beep_fail()

        print(f"\n  [SAT-003 FLASH] crit={crit_triggered}  "
              f"ticker={ticker_confirmed}  job={widget._cell_flash_job}")

        if not crit_triggered:
            pytest.fail(
                f"CRIT not reached — cannot verify flash ticker.\n"
                f"  Rotate more forcefully (≥ {self.CRIT_THRESHOLD:.0f} °/s)."
            )

        assert ticker_confirmed, (
            "CRIT entered but _cell_flash_job is None after 600 ms.\n"
            "  → Check _register_crit() → _start_cell_flash() call chain.\n"
            "  → Confirm root.update() (not update_idletasks()) is called "
            "after CRIT is set."
        )

    def test_SAT_IMU_003_crit_hold_persists(self, drone_link, imu_widget):
        """CRIT hold must persist ≥ 4 s after the rate drops (SRS-IMU-006)."""
        root, widget = imu_widget

        _beep_attention()
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — CRIT Hold Test (1/2)",
            f"Rotate FC VERY RAPIDLY, then SET IT FLAT.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.",
            _MB_ICONWARNING,
        )

        triggered, _ = self._poll_loop(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            stop_on_state="crit",
        )

        if not triggered:
            pytest.skip("Could not trigger CRIT — rotate more forcefully.")

        _msgbox(
            "SAT-IMU-003 — CRIT Hold Test (2/2)",
            "CRIT triggered!\n\n"
            "SET the FC FLAT AND STILL and click OK.\n\n"
            f"The red flashing indicator must stay on for ≥ {self.HOLD_CRIT_S:.0f} s.",
            _MB_ICONINFORMATION,
        )

        drain_deadline = time.monotonic() + 3.0
        while time.monotonic() < drain_deadline:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root.update_idletasks()
            if self._dps_from_data(widget, data) < self.CRIT_THRESHOLD:
                break
            time.sleep(self.POLL_INTERVAL_S)

        hold_start = time.monotonic()
        still_held = True

        while time.monotonic() - hold_start < self.HOLD_CRIT_S:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()
            if self._all_axes_state(widget, "safe"):
                still_held = False
                break
            time.sleep(self.POLL_INTERVAL_S)

        elapsed = time.monotonic() - hold_start
        print(f"\n  [SAT-003 CRIT HOLD] held={still_held}  "
              f"elapsed={elapsed:.2f} s  (need ≥ {self.HOLD_CRIT_S:.1f} s)")

        if still_held:
            _beep_success()
        else:
            _beep_fail()

        assert still_held, (
            f"CRIT dropped to SAFE after only {elapsed:.2f} s.\n"
            f"  Required hold : ≥ {self.HOLD_CRIT_S:.1f} s (SRS-IMU-006)\n"
            "  → Check hold_until timer in widget._gyro_state_label()."
        )

    def test_SAT_IMU_003_recovery_to_safe(self, drone_link, imu_widget):
        """All axes must return to SAFE and flash ticker must stop.

        Set FC flat. All axes must reach 'safe' within RECOVERY_TIMEOUT.
        Flash ticker (_cell_flash_job) must be None once SAFE is reached.
        """
        root, widget = imu_widget

        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — Recovery Test",
            f"Set FC FLAT AND STILL.\n\n"
            f"Keep it still for up to {self.RECOVERY_TIMEOUT:.0f} s.\n\n"
            "All gyro cells must return to the green SAFE state.",
            _MB_ICONINFORMATION,
        )

        recovered = False
        t0 = time.monotonic()

        while time.monotonic() - t0 < self.RECOVERY_TIMEOUT:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root.update_idletasks()
            if self._all_axes_state(widget, "safe"):
                recovered = True
                elapsed = time.monotonic() - t0
                print(f"\n  [SAT-003 RECOVERY] All axes SAFE after {elapsed:.1f} s")
                break
            time.sleep(self.POLL_INTERVAL_S)

        if recovered:
            _beep_success()
        else:
            _beep_fail()

        current = {k: v["state"] for k, v in widget._gyro_state.items()}

        assert recovered, (
            f"Axes did not reach SAFE within {self.RECOVERY_TIMEOUT:.0f} s.\n"
            f"  Current states : {current}\n"
            "  → Place FC flat and still. Check hold-down timer self-expiry."
        )

        assert widget._cell_flash_job is None, (
            "Flash ticker still active after all axes returned to SAFE.\n"
            "  → Check _unregister_crit() cancels the ticker when the\n"
            "    _crit_cells registry becomes empty (SRS-IMU-006a)."
        )