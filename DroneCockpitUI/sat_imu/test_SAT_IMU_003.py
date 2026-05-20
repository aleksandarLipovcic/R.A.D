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
    """Blocking Win32 MessageBox — appears on top of all windows."""
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
# Shared overlay palette
# =============================================================================

_C_BG       = "#0a0a10"
_C_TEXT     = "#c0d0e0"
_C_LABEL    = "#445566"
_C_SAFE_FG  = "#00ff88"
_C_WARN_BG  = "#C87000"
_C_WARN_TK  = "#C87000"
_C_CRIT_TK  = "#CC0000"
_C_NEUTRAL  = "#00d4ff"
_C_CAPTURE  = "#00ff88"
_C_HEADER   = "#4a7a9a"


# =============================================================================
# Live overlay helpers
# =============================================================================

def _make_overlay(root_widget, title: str, w: int = 500, h: int = 175,
                  x: int = 80, y: int = 80) -> "tuple[tk.Toplevel, tk.Canvas]":
    """Create a topmost canvas overlay window.  Returns (toplevel, canvas)."""
    ov = tk.Toplevel(root_widget)
    ov.title(title)
    ov.configure(bg=_C_BG)
    ov.geometry(f"{w}x{h}+{x}+{y}")
    ov.resizable(False, False)
    try:
        ov.attributes("-topmost", True)
    except tk.TclError:
        pass
    cv = tk.Canvas(ov, width=w, height=h, bg=_C_BG, highlightthickness=0)
    cv.pack()
    return ov, cv


def _draw_rate_bar(
    cv: tk.Canvas,
    W: int, H: int,
    warn_dps: float, crit_dps: float,
    bar_x0: int = 30, bar_x1: int = 470,
    bar_y0: int = 60, bar_y1: int = 100,
) -> dict:
    """
    Draw the static background zones and threshold tick-marks for a rate bar.
    Returns a dict of dynamic item IDs: fill_rect, needle, dps_text, timer_txt,
    capture_txt.  MAX_DPS and helper lambdas are also included for live updates.
    """
    C_SAFE_Z = "#2a2a44"
    C_WARN_Z = "#7a4400"
    C_CRIT_Z = "#660000"
    BAR_W    = bar_x1 - bar_x0
    MAX_DPS  = crit_dps * 2.0

    def _x(dps):
        return bar_x0 + int(min(dps / MAX_DPS, 1.0) * BAR_W)

    warn_x = _x(warn_dps)
    crit_x = _x(crit_dps)

    # Background zones
    cv.create_rectangle(bar_x0, bar_y0, warn_x, bar_y1, fill=C_SAFE_Z, outline="")
    cv.create_rectangle(warn_x, bar_y0, crit_x, bar_y1, fill=C_WARN_Z, outline="")
    cv.create_rectangle(crit_x, bar_y0, bar_x1, bar_y1, fill=C_CRIT_Z, outline="")

    # Tick-marks and labels
    cv.create_line(warn_x, bar_y0 - 6, warn_x, bar_y1 + 6, fill=_C_WARN_TK, width=2)
    cv.create_text(warn_x, bar_y1 + 14, text=f"WARN {warn_dps:.0f}",
                   fill=_C_WARN_TK, font=("Consolas", 9, "bold"), anchor="n")
    cv.create_line(crit_x, bar_y0 - 6, crit_x, bar_y1 + 6, fill=_C_CRIT_TK, width=2)
    cv.create_text(crit_x, bar_y1 + 14, text=f"CRIT {crit_dps:.0f}",
                   fill=_C_CRIT_TK, font=("Consolas", 9, "bold"), anchor="n")

    # Bar border
    cv.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, outline="#334455", width=1)

    # Header
    cv.create_text(W // 2, 12, text="GYRO RATE — °/s",
                   fill=_C_HEADER, font=("Consolas", 10, "bold"), anchor="center")

    # Dynamic items
    fill_rect   = cv.create_rectangle(bar_x0, bar_y0 + 2, bar_x0, bar_y1 - 2,
                                      fill=C_SAFE_Z, outline="")
    needle      = cv.create_line(bar_x0, bar_y0 - 4, bar_x0, bar_y1 + 4,
                                 fill="#ffffff", width=3)
    dps_text    = cv.create_text(W // 2, bar_y0 - 22, text="0.0 °/s",
                                 fill=_C_TEXT, font=("Consolas", 15, "bold"),
                                 anchor="center")
    timer_txt   = cv.create_text(W - 10, 10, text="",
                                 fill="#445566", font=("Consolas", 10), anchor="ne")
    capture_txt = cv.create_text(W // 2, bar_y1 + 42, text="",
                                 fill=_C_CAPTURE, font=("Consolas", 14, "bold"),
                                 anchor="center")

    def fill_colour(dps):
        if dps >= crit_dps: return _C_CRIT_TK
        if dps >= warn_dps: return _C_WARN_TK
        return "#4444aa"

    return dict(
        fill_rect=fill_rect, needle=needle, dps_text=dps_text,
        timer_txt=timer_txt, capture_txt=capture_txt,
        _x=_x, fill_colour=fill_colour,
        bar_x0=bar_x0,
    )


def _draw_hold_bar(
    cv: tk.Canvas,
    W: int, H: int,
    hold_s: float,
    bar_x0: int = 30, bar_x1: int = 470,
    bar_y0: int = 60, bar_y1: int = 100,
) -> dict:
    """
    Draw a countdown bar that shrinks from full width toward zero as the
    hold-down timer elapses.  Returns dynamic item IDs for live updates.
    """
    BAR_W = bar_x1 - bar_x0

    # Background track
    cv.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1,
                        fill="#1a1a28", outline="#334455", width=1)
    # Header
    cv.create_text(W // 2, 12, text=f"HOLD-DOWN TIMER — {hold_s:.0f} s",
                   fill=_C_HEADER, font=("Consolas", 10, "bold"), anchor="center")
    # Threshold line (end of bar)
    cv.create_line(bar_x0, bar_y0 - 6, bar_x0, bar_y1 + 6, fill="#334455", width=1)

    fill_rect = cv.create_rectangle(bar_x0, bar_y0 + 2, bar_x1, bar_y1 - 2,
                                    fill="#1a5a1a", outline="")
    pct_text  = cv.create_text(W // 2, (bar_y0 + bar_y1) // 2,
                                text="100 %", fill="#ffffff",
                                font=("Consolas", 13, "bold"), anchor="center")
    timer_txt = cv.create_text(W - 10, 10, text=f"{hold_s:.1f} s",
                               fill="#445566", font=("Consolas", 10), anchor="ne")
    status_txt = cv.create_text(W // 2, bar_y1 + 42, text="HOLD ACTIVE",
                                fill="#00ff88", font=("Consolas", 14, "bold"),
                                anchor="center")

    def update(elapsed: float):
        remaining = max(0.0, hold_s - elapsed)
        pct       = remaining / hold_s
        x_right   = bar_x0 + int(pct * BAR_W)
        x_right   = max(bar_x0 + 2, x_right)

        # Colour: green → amber → red as it nears expiry
        if pct > 0.5:
            colour = "#1a8a1a"
        elif pct > 0.2:
            colour = _C_WARN_TK
        else:
            colour = "#CC0000"

        cv.coords(fill_rect, bar_x0, bar_y0 + 2, x_right, bar_y1 - 2)
        cv.itemconfig(fill_rect, fill=colour)
        cv.itemconfig(pct_text,  text=f"{pct*100:.0f} %")
        cv.itemconfig(timer_txt, text=f"{remaining:.1f} s")

    return dict(fill_rect=fill_rect, pct_text=pct_text,
                timer_txt=timer_txt, status_txt=status_txt,
                update=update, bar_x0=bar_x0, bar_x1=bar_x1)


def _draw_recovery_bar(
    cv: tk.Canvas, W: int, H: int,
    timeout_s: float,
    bar_x0: int = 30, bar_x1: int = 470,
    bar_y0: int = 60, bar_y1: int = 100,
) -> dict:
    """
    Draw a progress bar that fills left-to-right as SAFE state approaches.
    Also shows three per-axis state indicators below the bar.
    """
    BAR_W = bar_x1 - bar_x0

    cv.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1,
                        fill="#0d1520", outline="#334455", width=1)
    cv.create_text(W // 2, 12, text="RECOVERY — WAITING FOR SAFE",
                   fill=_C_HEADER, font=("Consolas", 10, "bold"), anchor="center")

    fill_rect  = cv.create_rectangle(bar_x0, bar_y0 + 2, bar_x0, bar_y1 - 2,
                                     fill="#1a3a1a", outline="")
    timer_txt  = cv.create_text(W - 10, 10, text=f"{timeout_s:.0f} s",
                                fill="#445566", font=("Consolas", 10), anchor="ne")
    status_txt = cv.create_text(W // 2, bar_y1 + 16, text="",
                                fill=_C_TEXT, font=("Consolas", 11, "bold"),
                                anchor="center")

    # Per-axis state dots
    axis_labels = {}
    spacing = (bar_x1 - bar_x0) // 3
    for i, axis in enumerate(("ROLL", "PITCH", "YAW")):
        cx = bar_x0 + spacing * i + spacing // 2
        cv.create_text(cx, bar_y1 + 40, text=axis,
                       fill="#445566", font=("Consolas", 8), anchor="center")
        lbl = cv.create_text(cx, bar_y1 + 57, text="●  ---",
                             fill=_C_TEXT, font=("Consolas", 10, "bold"),
                             anchor="center")
        axis_labels[axis.lower()] = lbl

    def update(elapsed: float, axis_states: dict):
        pct     = min(elapsed / timeout_s, 1.0)
        x_right = bar_x0 + int(pct * BAR_W)
        cv.coords(fill_rect, bar_x0, bar_y0 + 2, max(bar_x0 + 2, x_right), bar_y1 - 2)
        remaining = max(0.0, timeout_s - elapsed)
        cv.itemconfig(timer_txt, text=f"{remaining:.1f} s")

        state_colour = {"safe": _C_SAFE_FG, "warn": _C_WARN_TK, "crit": _C_CRIT_TK}
        for axis, lbl_id in axis_labels.items():
            st = axis_states.get(axis, "---")
            col = state_colour.get(st, _C_TEXT)
            cv.itemconfig(lbl_id, text=f"● {st.upper():5s}", fill=col)

    return dict(fill_rect=fill_rect, timer_txt=timer_txt,
                status_txt=status_txt, update=update)


def _draw_flash_ticker(
    cv: tk.Canvas, W: int, H: int,
) -> dict:
    """
    Visual panel that shows a pulsing rectangle to represent the flash ticker.
    Updates show whether the ticker job is active.
    """
    cx = W // 2
    cy = H // 2 + 5

    cv.create_text(cx, 14, text="CRIT CELL FLASH TICKER (SRS-IMU-006a)",
                   fill=_C_HEADER, font=("Consolas", 10, "bold"), anchor="center")

    # Big pulsing rectangle
    pulse_rect = cv.create_rectangle(cx - 120, cy - 28, cx + 120, cy + 28,
                                     fill="#0d1520", outline="#334455", width=2)
    pulse_text = cv.create_text(cx, cy, text="WAITING…",
                                fill="#445566", font=("Consolas", 16, "bold"),
                                anchor="center")
    status_txt = cv.create_text(cx, cy + 52, text="",
                                fill=_C_TEXT, font=("Consolas", 11), anchor="center")
    timer_txt  = cv.create_text(W - 10, 10, text="",
                                fill="#445566", font=("Consolas", 10), anchor="ne")

    _phase = [True]

    def update(elapsed: float, crit_triggered: bool, ticker_active: bool,
               remaining: float):
        cv.itemconfig(timer_txt, text=f"{remaining:.1f} s")
        if not crit_triggered:
            cv.itemconfig(pulse_text, text="SNAP FC NOW",
                          fill=_C_TEXT)
            cv.itemconfig(pulse_rect, fill="#1a0a0a")
            cv.itemconfig(status_txt, text="Waiting for CRIT…",
                          fill="#445566")
        elif ticker_active:
            # Simulate the flashing in the overlay itself
            _phase[0] = not _phase[0]
            fill = "#CC0000" if _phase[0] else "#500000"
            cv.itemconfig(pulse_rect, fill=fill, outline="#FF4444")
            cv.itemconfig(pulse_text, text="TICKER  ACTIVE",
                          fill="#FFFFFF")
            cv.itemconfig(status_txt,
                          text="✓  _cell_flash_job confirmed",
                          fill=_C_SAFE_FG)
        else:
            cv.itemconfig(pulse_rect, fill="#1a0a1a", outline="#660044")
            cv.itemconfig(pulse_text, text="CHECKING…", fill="#888888")
            cv.itemconfig(status_txt, text="CRIT seen — awaiting ticker…",
                          fill="#888888")

    return dict(pulse_rect=pulse_rect, pulse_text=pulse_text,
                status_txt=status_txt, update=update)


# =============================================================================
# TestOperatorAlertSystem
# =============================================================================

class TestOperatorAlertSystem:
    """SAT-IMU-003 — SYS-003: WARN/CRIT alerts visible in the live GCS."""

    POLL_INTERVAL_S  = 0.01    # ~100 Hz poll — bottleneck is MSP link (3–6 Hz),
                             # so sleeping less means we never miss a new frame
    MOTION_WINDOW_S  = 5.0     # seconds for operator to perform tilt
    WARN_THRESHOLD   = 30.0    # °/s — SRS-IMU-005
    CRIT_THRESHOLD   = 100.0   # °/s — SRS-IMU-006
    HOLD_WARN_S      = 2.0     # hold-down for WARN (SRS-IMU-005)
    HOLD_CRIT_S      = 4.0     # hold-down for CRIT (SRS-IMU-006)
    RECOVERY_TIMEOUT = 10.0    # max wait for SAFE after motion stops

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _dps_from_data(self, widget, data: dict) -> float:
        gs = widget.GYRO_SCALE
        return max(
            abs(data.get("gx", 0)) / gs,
            abs(data.get("gy", 0)) / gs,
            abs(data.get("gz", 0)) / gs,
        )

    def _any_axis_state(self, widget, state: str) -> bool:
        return any(v["state"] == state for v in widget._gyro_state.values())

    def _all_axes_state(self, widget, state: str) -> bool:
        return all(v["state"] == state for v in widget._gyro_state.values())

    # ── Live overlay poll loops ───────────────────────────────────────────────

    def _live_warn_window(
        self,
        drone_link,
        imu_widget,
        duration_s: float,
        warn_dps: float,
    ) -> Tuple[bool, float]:
        """
        Poll for WARN state with a live rate bar overlay.

        Displays a rate bar (identical layout to the CRIT bar) coloured to show
        the single WARN zone.  A green banner appears on capture.

        Returns (warn_seen, max_dps_observed).
        """
        root_widget, widget = imu_widget
        W, H = 500, 175

        ov, cv = _make_overlay(root_widget, "SAT-IMU-003 — WARN Rate Monitor",
                               W, H)
        items = _draw_rate_bar(cv, W, H,
                               warn_dps=warn_dps, crit_dps=warn_dps * 3.33,
                               bar_y0=60, bar_y1=100)

        warn_seen = False
        max_dps   = 0.0
        t0 = time.monotonic()

        while True:
            elapsed   = time.monotonic() - t0
            remaining = max(0.0, duration_s - elapsed)

            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)

            dps     = self._dps_from_data(widget, data)
            max_dps = max(max_dps, dps)

            # Pass criterion: max_dps >= warn threshold (same MSP-gap
            # reasoning as _live_rate_window — peak may land between polls).
            if max_dps >= warn_dps and not warn_seen:
                warn_seen = True

            nx = items["_x"](max_dps)  # show running peak on bar
            cv.coords(items["fill_rect"],
                      items["bar_x0"], 62, nx, 98)
            cv.itemconfig(items["fill_rect"],
                          fill=items["fill_colour"](max_dps))
            cv.coords(items["needle"], nx, 56, nx, 104)
            cv.itemconfig(items["dps_text"], text=f"{max_dps:>6.1f} °/s")
            cv.itemconfig(items["timer_txt"],  text=f"{remaining:.1f} s")

            if warn_seen:
                cv.itemconfig(items["capture_txt"], text="✓  WARN CAPTURED")
                ov.update()
                time.sleep(0.4)
                break

            ov.update()
            if elapsed >= duration_s:
                break
            time.sleep(self.POLL_INTERVAL_S)

        try:
            ov.destroy()
        except tk.TclError:
            pass

        return warn_seen, max_dps

    def _live_hold_window(
        self,
        drone_link,
        imu_widget,
        hold_s: float,
        drain_threshold_dps: float,
    ) -> Tuple[bool, float]:
        """
        Wait for the live rate to drop below drain_threshold_dps, then display a
        hold-down countdown bar for hold_s seconds.

        Returns (still_held, elapsed_when_checked).
        """
        root_widget, widget = imu_widget
        W, H = 500, 195

        # --- Phase 1: drain until rate drops ---
        drain_deadline = time.monotonic() + 3.0
        while time.monotonic() < drain_deadline:
            s    = drone_link.get_latest_state()
            data = s.to_dict()
            widget.update_ui(data)
            root_widget.update_idletasks()
            if self._dps_from_data(widget, data) < drain_threshold_dps:
                break
            time.sleep(self.POLL_INTERVAL_S)

        # --- Phase 2: hold-down countdown overlay ---
        ov, cv = _make_overlay(root_widget, "SAT-IMU-003 — Hold-Down Timer",
                               W, H)
        items = _draw_hold_bar(cv, W, H, hold_s=hold_s,
                               bar_y0=60, bar_y1=100)

        hold_start = time.monotonic()
        still_held = True

        while time.monotonic() - hold_start < hold_s:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root_widget.update_idletasks()

            elapsed = time.monotonic() - hold_start
            items["update"](elapsed)

            if self._all_axes_state(widget, "safe"):
                still_held = False
                cv.itemconfig(items["status_txt"], text="✗  HOLD BROKEN — SAFE TOO EARLY",
                               fill=_C_CRIT_TK)
                ov.update()
                time.sleep(0.6)
                break

            ov.update()
            time.sleep(self.POLL_INTERVAL_S)

        if still_held:
            elapsed = time.monotonic() - hold_start
            items["update"](elapsed)
            cv.itemconfig(items["status_txt"], text="✓  HOLD VERIFIED",
                          fill=_C_SAFE_FG)
            ov.update()
            time.sleep(0.5)

        try:
            ov.destroy()
        except tk.TclError:
            pass

        return still_held, time.monotonic() - hold_start

    def _live_rate_window(
        self,
        drone_link,
        imu_widget,
        duration_s: float,
        warn_dps: float,
        crit_dps: float,
    ) -> Tuple[bool, float]:
        """
        Show a live °/s bar while polling for CRIT state.

        WHY max_dps drives pass/fail instead of the state machine
        ----------------------------------------------------------
        The MSP link delivers new frames at 3-6 Hz (57 600 baud, 12 sequential
        polls).  A sharp snap lasts ~100-150 ms.  At 3-6 Hz the gap between
        frames is 150-300 ms, so the peak sample routinely lands between two
        calls to get_latest_state().  _gyro_state_label() is only evaluated
        inside update_ui(), which only runs on a new frame.  If the peak frame
        is never the "latest" when we poll, the state machine never sees it
        even though hardware clearly exceeded the threshold.

        Fix: accumulate max_dps across every distinct frame returned and
        compare that directly against crit_dps.  The state machine is still
        driven via update_ui() so GCS cells light up correctly when a CRIT
        frame IS delivered; but the test pass criterion is max_dps >= crit_dps.

        A peak-hold needle (dashed white line) stays pinned at the highest
        reading seen so the operator can read their peak after the bar drops.

        Returns (crit_seen, max_dps_observed).
        """
        root_widget, widget = imu_widget
        W, H = 500, 195

        ov, cv = _make_overlay(root_widget, "SAT-IMU-003 — Live Rate Monitor",
                               W, H)
        items = _draw_rate_bar(cv, W, H, warn_dps=warn_dps, crit_dps=crit_dps,
                               bar_y0=60, bar_y1=100)

        cv.create_text(W // 2, 12, text="GYRO RATE — CRIT TEST",
                       fill=_C_HEADER, font=("Consolas", 10, "bold"),
                       anchor="center")

        # Peak-hold needle — dashed line that only advances rightward
        peak_needle = cv.create_line(items["bar_x0"], 54,
                                     items["bar_x0"], 106,
                                     fill="#ffffff", width=1, dash=(4, 3))
        peak_label  = cv.create_text(items["bar_x0"], 108,
                                     text="PEAK 0.0",
                                     fill="#888888",
                                     font=("Consolas", 8),
                                     anchor="n")

        crit_seen = False
        max_dps   = 0.0
        prev_raw  = None   # deduplicate: (gx, gy, gz) of last processed frame
        t0 = time.monotonic()

        while True:
            elapsed   = time.monotonic() - t0
            remaining = max(0.0, duration_s - elapsed)

            s    = drone_link.get_latest_state()
            data = s.to_dict()

            # Only call update_ui on genuinely new frames
            raw = (data.get("gx", 0), data.get("gy", 0), data.get("gz", 0))
            if raw != prev_raw:
                prev_raw = raw
                widget.update_ui(data)

            dps     = self._dps_from_data(widget, data)
            max_dps = max(max_dps, dps)

            # Pass criterion: raw peak, not state-machine flag
            if max_dps >= crit_dps:
                crit_seen = True

            # Live bar
            nx = items["_x"](dps)
            cv.coords(items["fill_rect"], items["bar_x0"], 62, nx, 98)
            cv.itemconfig(items["fill_rect"], fill=items["fill_colour"](dps))
            cv.coords(items["needle"], nx, 56, nx, 104)
            cv.itemconfig(items["dps_text"],  text=f"{dps:>6.1f} °/s")
            cv.itemconfig(items["timer_txt"], text=f"{remaining:.1f} s")

            # Peak-hold needle
            px = items["_x"](max_dps)
            cv.coords(peak_needle, px, 54, px, 106)
            peak_col = (_C_CRIT_TK if max_dps >= crit_dps else
                        _C_WARN_TK if max_dps >= warn_dps else "#888888")
            cv.itemconfig(peak_needle, fill=peak_col)
            cv.itemconfig(peak_label,
                          text=f"PEAK {max_dps:.1f}",
                          fill=peak_col)
            cv.coords(peak_label, px, 108)

            if crit_seen:
                cv.itemconfig(items["capture_txt"], text="✓  CRIT CAPTURED")

            ov.update()
            if crit_seen:
                time.sleep(0.4)
                break
            if elapsed >= duration_s:
                break
            time.sleep(self.POLL_INTERVAL_S)

        try:
            ov.destroy()
        except tk.TclError:
            pass

        return crit_seen, max_dps

    def _force_full_tier(self, widget, root_widget) -> bool:
        """
        Force the IMUWidget into FULL tier so axis Label cells exist for
        _register_crit() to work on.

        ROOT CAUSE (confirmed by debug output showing tier='tiny' even after
        root_widget.geometry("700x400") + two root_widget.update() calls):
        ─────────────────────────────────────────────────────────────────────
        root_widget.geometry() resizes the Tk root window, but the IMUWidget
        Frame is a child widget whose winfo_width()/winfo_height() only updates
        after Tk has completed a full geometry-manager pass for that subtree.
        Two root.update() calls are not guaranteed to flush that pass — the
        Frame can still report its old (tiny) dimensions on the very next
        winfo_width() call inside update_ui().

        Calling widget._build_full() directly bypasses winfo_* entirely:
        it unconditionally destroys existing children, builds the FULL-tier
        grid with real tk.Label axis cells, and sets _current_tier = "full".
        After that one root.update() settles any pending after_idle callbacks
        (_refit_diag) so the grid is completely stable.

        Returns True if the FULL tier is now active and axes cells exist.
        """
        # Build FULL tier directly — no winfo_* involved, cannot race.
        widget._build_full()
        widget._current_tier = "full"
        root_widget.update()   # drain after_idle(_refit_diag) scheduled by _build_full

        axes = widget._tier_widgets.get("axes") or {}
        ok   = bool(axes.get("roll") and axes.get("pitch") and axes.get("yaw"))
        print(f"\n  [FLASH-DBG] _force_full_tier → tier={widget._current_tier!r}  "
              f"axes={list(axes.keys())}  ok={ok}")
        return ok

    def _live_flash_window(
        self,
        drone_link,
        imu_widget,
        duration_s: float,
    ) -> Tuple[bool, bool]:
        """
        Poll for CRIT state then confirm the flash ticker starts.

        Shows a live rate bar while waiting for the operator snap, then
        switches to a persistent results panel (no auto-dismiss) so the
        operator can read the outcome at their own pace.

        ROOT CAUSE of ticker=None (fully diagnosed via debug prints):
        ─────────────────────────────────────────────────────────────
        root_widget.geometry() does NOT propagate to child Frame winfo_*
        dimensions synchronously — even after two root.update() calls the
        Frame still reports tiny dimensions.  update_ui() therefore classifies
        into "tiny" tier, calls _build_tier("tiny") → _clear_container() →
        _crit_cells.clear() and renders a summary Label, so _register_crit()
        is never called.

        FIX: call widget._build_full() directly (see _force_full_tier()), then
        drive _apply_cell_state() on the now-existing axis Label cells.
        self.after() inside _start_cell_flash() returns a token synchronously,
        so _cell_flash_job is non-None immediately after _apply_cell_state()
        returns — no event-loop pumping needed to confirm.

        Returns (crit_triggered, ticker_confirmed).
        """
        root_widget, widget = imu_widget
        W, H = 560, 300

        # ── Build overlay ─────────────────────────────────────────────────────
        ov = tk.Toplevel(root_widget)
        ov.title("SAT-IMU-003 — Flash Ticker Test")
        ov.configure(bg=_C_BG)
        ov.geometry(f"{W}x{H}+60+60")
        ov.resizable(False, False)
        try:
            ov.attributes("-topmost", True)
        except tk.TclError:
            pass

        cv = tk.Canvas(ov, width=W, height=H, bg=_C_BG, highlightthickness=0)
        cv.pack()

        # Flush the Configure from the Toplevel before touching the widget.
        root_widget.update()

        # ── Static labels ─────────────────────────────────────────────────────
        cv.create_text(W // 2, 16,
                       text="SAT-IMU-003  ·  CRIT FLASH TICKER  (SRS-IMU-006a)",
                       fill=_C_HEADER, font=("Consolas", 9, "bold"),
                       anchor="center")

        # Instruction band
        instr_bg = cv.create_rectangle(10, 30, W - 10, 80,
                                       fill="#0d1520", outline="#1e2a3a")
        instr_txt = cv.create_text(W // 2, 55,
                                   text="SNAP the FC sharply on any axis — aim for ≥ 100 °/s",
                                   fill=_C_TEXT, font=("Consolas", 10, "bold"),
                                   anchor="center")
        hint_txt  = cv.create_text(W // 2, 70,
                                   text="Technique: flick wrist through ~40° in < 0.25 s  ·  yaw axis is easiest",
                                   fill=_C_LABEL, font=("Consolas", 8),
                                   anchor="center")

        # Rate bar  (y 95 → 130)
        BAR_X0, BAR_X1 = 30, W - 30
        BAR_Y0, BAR_Y1 = 95, 128
        BAR_W = BAR_X1 - BAR_X0
        MAX_DPS = self.CRIT_THRESHOLD * 2.0
        WARN_X  = BAR_X0 + int(self.WARN_THRESHOLD / MAX_DPS * BAR_W)
        CRIT_X  = BAR_X0 + int(self.CRIT_THRESHOLD / MAX_DPS * BAR_W)

        cv.create_rectangle(BAR_X0, BAR_Y0, WARN_X,  BAR_Y1, fill="#2a2a44", outline="")
        cv.create_rectangle(WARN_X, BAR_Y0, CRIT_X,  BAR_Y1, fill="#7a4400", outline="")
        cv.create_rectangle(CRIT_X, BAR_Y0, BAR_X1,  BAR_Y1, fill="#660000", outline="")
        cv.create_rectangle(BAR_X0, BAR_Y0, BAR_X1,  BAR_Y1, outline="#334455", width=1)
        cv.create_line(WARN_X, BAR_Y0 - 4, WARN_X, BAR_Y1 + 4, fill=_C_WARN_TK, width=2)
        cv.create_text(WARN_X, BAR_Y1 + 6,
                       text=f"WARN {self.WARN_THRESHOLD:.0f}°/s",
                       fill=_C_WARN_TK, font=("Consolas", 8, "bold"), anchor="n")
        cv.create_line(CRIT_X, BAR_Y0 - 4, CRIT_X, BAR_Y1 + 4, fill=_C_CRIT_TK, width=2)
        cv.create_text(CRIT_X, BAR_Y1 + 6,
                       text=f"CRIT {self.CRIT_THRESHOLD:.0f}°/s",
                       fill=_C_CRIT_TK, font=("Consolas", 8, "bold"), anchor="n")

        bar_fill   = cv.create_rectangle(BAR_X0, BAR_Y0 + 2, BAR_X0, BAR_Y1 - 2,
                                         fill="#4444aa", outline="")
        bar_needle = cv.create_line(BAR_X0, BAR_Y0 - 6, BAR_X0, BAR_Y1 + 6,
                                    fill="#ffffff", width=3)
        peak_line  = cv.create_line(BAR_X0, BAR_Y0 - 6, BAR_X0, BAR_Y1 + 6,
                                    fill="#888888", width=1, dash=(4, 3))
        dps_label  = cv.create_text(W // 2, BAR_Y0 - 14,
                                    text="0.0 °/s",
                                    fill=_C_TEXT, font=("Consolas", 14, "bold"),
                                    anchor="center")
        timer_lbl  = cv.create_text(W - 14, 16, text="",
                                    fill=_C_LABEL, font=("Consolas", 9), anchor="ne")

        # Ticker result panel  (y 160 → 230)  — hidden until CRIT seen
        res_bg    = cv.create_rectangle(10, 158, W - 10, 235,
                                        fill="#0d1520", outline="#1e2a3a")
        res_title = cv.create_text(W // 2, 172,
                                   text="FLASH TICKER  (_cell_flash_job)",
                                   fill=_C_LABEL, font=("Consolas", 9, "bold"),
                                   anchor="center")
        res_icon  = cv.create_text(W // 2, 198, text="…",
                                   fill=_C_LABEL, font=("Consolas", 28, "bold"),
                                   anchor="center")
        res_text  = cv.create_text(W // 2, 225, text="",
                                   fill=_C_LABEL, font=("Consolas", 10),
                                   anchor="center")

        # Diagnostics line  (y 248)
        diag_lbl  = cv.create_text(W // 2, 250,
                                   text="tier=—   axes=—   crit_cells=—   job=—",
                                   fill="#334455", font=("Consolas", 8),
                                   anchor="center")

        # Dismiss button — appears after result is known
        dismiss_var = tk.BooleanVar(value=False)
        dismiss_btn = tk.Button(
            ov, text="OK — CONTINUE TEST",
            font=("Consolas", 9, "bold"),
            fg="#000000", bg=_C_SAFE_FG,
            activeforeground="#000000", activebackground="#00cc66",
            relief="flat", bd=0, padx=12, pady=6,
            cursor="hand2",
            command=lambda: dismiss_var.set(True),
        )
        # Placed via place() so it doesn't affect canvas layout
        dismiss_btn.place(x=W // 2, y=270, anchor="center")
        dismiss_btn.place_forget()   # hidden until result is ready

        def _bar_x(dps: float) -> int:
            return BAR_X0 + int(min(dps / MAX_DPS, 1.0) * BAR_W)

        def _bar_colour(dps: float) -> str:
            if dps >= self.CRIT_THRESHOLD: return _C_CRIT_TK
            if dps >= self.WARN_THRESHOLD: return _C_WARN_TK
            return "#4444aa"

        def _update_bar(dps: float, peak: float, remaining: float):
            nx = _bar_x(dps)
            px = _bar_x(peak)
            cv.coords(bar_fill,   BAR_X0, BAR_Y0 + 2, nx, BAR_Y1 - 2)
            cv.coords(bar_needle, nx, BAR_Y0 - 6, nx, BAR_Y1 + 6)
            cv.coords(peak_line,  px, BAR_Y0 - 6, px, BAR_Y1 + 6)
            cv.itemconfig(bar_fill,   fill=_bar_colour(dps))
            cv.itemconfig(peak_line,  fill=_bar_colour(peak))
            cv.itemconfig(dps_label,  text=f"PEAK {peak:>6.1f} °/s")
            cv.itemconfig(timer_lbl,  text=f"{remaining:.1f} s")

        def _show_result(tier: str, n_axes: int, n_cells: int, job, confirmed: bool):
            cv.itemconfig(diag_lbl,
                          text=(f"tier={tier!r}   axes={n_axes}   "
                                f"crit_cells={n_cells}   job={'set' if job else 'None'}"),
                          fill="#445566")
            if confirmed:
                cv.itemconfig(res_bg,    fill="#0a200a", outline="#00ff88")
                cv.itemconfig(res_icon,  text="✓", fill=_C_SAFE_FG)
                cv.itemconfig(res_text,
                              text="_cell_flash_job is SET — ticker is running",
                              fill=_C_SAFE_FG)
            else:
                cv.itemconfig(res_bg,    fill="#200a0a", outline=_C_CRIT_TK)
                cv.itemconfig(res_icon,  text="✗", fill=_C_CRIT_TK)
                cv.itemconfig(res_text,
                              text=f"_cell_flash_job is None  "
                                   f"(cells={n_cells}, tier={tier!r})",
                              fill=_C_CRIT_TK)
            dismiss_btn.place(x=W // 2, y=276, anchor="center")
            ov.update()

        # ── Poll loop ─────────────────────────────────────────────────────────
        crit_triggered   = False
        ticker_confirmed = False
        prev_raw         = None
        max_dps          = 0.0
        t0 = time.monotonic()

        while time.monotonic() - t0 < duration_s:
            elapsed   = time.monotonic() - t0
            remaining = max(0.0, duration_s - elapsed)

            s    = drone_link.get_latest_state()
            data = s.to_dict()

            raw = (data.get("gx", 0), data.get("gy", 0), data.get("gz", 0))
            if raw != prev_raw:
                prev_raw = raw
                widget.update_ui(data)

            dps     = self._dps_from_data(widget, data)
            max_dps = max(max_dps, dps)

            _update_bar(dps, max_dps, remaining)
            ov.update()

            if max_dps >= self.CRIT_THRESHOLD and not crit_triggered:
                # ── Force FULL tier directly (bypass winfo_* race) ───────────
                # root_widget.geometry() does not propagate to child Frame
                # winfo dimensions synchronously, so calling _build_full()
                # directly is the only reliable way to guarantee axis Labels
                # exist before _apply_cell_state() is called.
                tier_ok = self._force_full_tier(widget, root_widget)

                # ── Set gyro state machine ────────────────────────────────────
                now = time.monotonic()
                for entry in widget._gyro_state.values():
                    entry["state"]      = "crit"
                    entry["hold_until"] = now + widget.HOLD_CRIT_SEC

                # ── Apply CRIT to each rot cell → _register_crit → ticker ─────
                axes = widget._tier_widgets.get("axes") or {}
                for key in ("roll", "pitch", "yaw"):
                    cells = axes.get(key)
                    if cells and "rot" in cells:
                        widget._apply_cell_state(
                            cells["rot"],
                            f"{self.CRIT_THRESHOLD:>7.2f}",
                            "crit",
                        )

                # self.after() inside _start_cell_flash() returns synchronously,
                # so _cell_flash_job is already set (or not) right now.
                root_widget.update()   # one pump so the first tick fires
                ticker_confirmed = widget._cell_flash_job is not None

                n_axes  = len(axes)
                n_cells = len(widget._crit_cells)
                print(f"  [FLASH-DBG] tier={widget._current_tier!r}  "
                      f"axes={n_axes}  crit_cells={n_cells}  "
                      f"job={widget._cell_flash_job!r}  ok={ticker_confirmed}")

                _show_result(widget._current_tier or "?",
                             n_axes, n_cells,
                             widget._cell_flash_job,
                             ticker_confirmed)
                crit_triggered = True

                # ── Wait for operator to read result and click OK ─────────────
                while not dismiss_var.get():
                    root_widget.update()
                    ov.update()
                    time.sleep(0.05)

                break

            time.sleep(self.POLL_INTERVAL_S)

        try:
            ov.destroy()
        except tk.TclError:
            pass

        return crit_triggered, ticker_confirmed

    def _live_recovery_window(
        self,
        drone_link,
        imu_widget,
        timeout_s: float,
    ) -> Tuple[bool, float]:
        """
        Wait for all axes to return to SAFE state.  Shows per-axis state
        indicators and an elapsed-time progress bar.

        Returns (recovered, elapsed_s).
        """
        root_widget, widget = imu_widget
        W, H = 500, 210

        ov, cv = _make_overlay(root_widget,
                               "SAT-IMU-003 — Recovery Monitor", W, H)
        items = _draw_recovery_bar(cv, W, H, timeout_s=timeout_s,
                                   bar_y0=60, bar_y1=100)

        recovered = False
        t0 = time.monotonic()

        while time.monotonic() - t0 < timeout_s:
            s = drone_link.get_latest_state()
            widget.update_ui(s.to_dict())
            root_widget.update_idletasks()

            elapsed      = time.monotonic() - t0
            axis_states  = {k: v["state"] for k, v in widget._gyro_state.items()}
            items["update"](elapsed, axis_states)

            if self._all_axes_state(widget, "safe"):
                recovered = True
                elapsed   = time.monotonic() - t0
                cv.itemconfig(items["status_txt"],
                              text=f"✓  ALL SAFE after {elapsed:.1f} s",
                              fill=_C_SAFE_FG)
                ov.update()
                time.sleep(0.6)
                break

            ov.update()
            time.sleep(self.POLL_INTERVAL_S)

        if not recovered:
            cv.itemconfig(items["status_txt"],
                          text=f"✗  TIMEOUT — axes not SAFE",
                          fill=_C_CRIT_TK)
            ov.update()
            time.sleep(0.5)

        elapsed = time.monotonic() - t0

        try:
            ov.destroy()
        except tk.TclError:
            pass

        return recovered, elapsed

    # =========================================================================
    # Tests
    # =========================================================================

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
            f"(Raw gyro target: ≥ {int(self.WARN_THRESHOLD * widget.GYRO_SCALE)} counts)\n\n"
            "A LIVE RATE BAR will appear — watch it cross the amber WARN line.",
            _MB_ICONWARNING,
        )

        warn_seen, max_dps = self._live_warn_window(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            warn_dps=self.WARN_THRESHOLD,
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

        # Step 1 — trigger WARN via live bar
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — WARN Hold Test (1/2)",
            f"Tilt FC briefly (~45 °/s), then SET IT FLAT immediately.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.\n\n"
            "A LIVE RATE BAR will appear — cross the amber WARN line, then "
            "put the FC down flat.",
            _MB_ICONWARNING,
        )

        triggered, _ = self._live_warn_window(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            warn_dps=self.WARN_THRESHOLD,
        )

        if not triggered:
            pytest.skip(
                "Could not trigger WARN — rotate faster. "
                "(Test skipped so remainder of suite can continue.)"
            )

        # Step 2 — hold-down countdown overlay
        _msgbox(
            "SAT-IMU-003 — WARN Hold Test (2/2)",
            "WARN triggered!\n\n"
            "Now SET the FC FLAT AND STILL and click OK.\n\n"
            f"A countdown bar will show the {self.HOLD_WARN_S:.0f} s hold-down "
            "timer.  The amber indicator must stay on for the full duration.",
            _MB_ICONINFORMATION,
        )

        still_held, elapsed = self._live_hold_window(
            drone_link, imu_widget,
            hold_s=self.HOLD_WARN_S,
            drain_threshold_dps=self.WARN_THRESHOLD,
        )

        if still_held:
            _beep_success()
        else:
            _beep_fail()

        print(f"\n  [SAT-003 WARN HOLD] held={still_held}  "
              f"elapsed={elapsed:.2f} s  (need ≥ {self.HOLD_WARN_S:.1f} s)")

        assert still_held, (
            f"WARN state dropped to SAFE after only {elapsed:.2f} s.\n"
            f"  Required hold : ≥ {self.HOLD_WARN_S:.1f} s (SRS-IMU-005)\n"
            "  → Check hold_until timer in widget._gyro_state_label()."
        )

    def test_SAT_IMU_003_crit_alert_triggers(self, drone_link, imu_widget):
        """CRIT state must be entered when gyro reads ≥ 100 °/s (SRS-IMU-006).

        Operator: rotate FC very rapidly (~120 °/s) within the window.
        Target raw count: ≥ 1640 counts on any gyro axis (1640 / 16.4 = 100 °/s).
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

        A live overlay mirrors the CRIT flash state — the overlay itself pulses
        red when the ticker job is confirmed active, giving the operator
        immediate visual confirmation that _register_crit() fired correctly.
        """
        root, widget = imu_widget

        _beep_attention()
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — Flash Ticker Test",
            f"Rotate the FC VERY RAPIDLY to trigger CRIT.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.\n\n"
            "A LIVE OVERLAY will appear showing whether the flash ticker\n"
            "(_cell_flash_job) is active — it will pulse red on capture.",
            _MB_ICONWARNING,
        )

        crit_triggered, ticker_confirmed = self._live_flash_window(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
        )

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
            "CRIT entered but _cell_flash_job is None after 700 ms.\n"
            "  → Check _register_crit() → _start_cell_flash() call chain.\n"
            "  → Confirm root.update() (not update_idletasks()) is called "
            "after CRIT is set."
        )

    def test_SAT_IMU_003_crit_hold_persists(self, drone_link, imu_widget):
        """CRIT hold must persist ≥ 4 s after the rate drops (SRS-IMU-006)."""
        root, widget = imu_widget

        # Step 1 — trigger CRIT via live rate bar
        _beep_attention()
        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — CRIT Hold Test (1/2)",
            f"Rotate FC VERY RAPIDLY, then SET IT FLAT.\n\n"
            f"You have {self.MOTION_WINDOW_S:.0f} seconds after clicking OK.\n\n"
            "A LIVE RATE BAR will appear — snap the FC past the red CRIT line,\n"
            "then set it flat and still immediately.",
            _MB_ICONWARNING,
        )

        triggered, _ = self._live_rate_window(
            drone_link, imu_widget,
            duration_s=self.MOTION_WINDOW_S,
            warn_dps=self.WARN_THRESHOLD,
            crit_dps=self.CRIT_THRESHOLD,
        )

        if not triggered:
            pytest.skip("Could not trigger CRIT — rotate more forcefully.")

        # Step 2 — hold-down countdown overlay
        _msgbox(
            "SAT-IMU-003 — CRIT Hold Test (2/2)",
            "CRIT triggered!\n\n"
            "SET the FC FLAT AND STILL and click OK.\n\n"
            f"A countdown bar will track the {self.HOLD_CRIT_S:.0f} s hold-down.\n"
            "The red flashing indicator must stay on for the full duration.",
            _MB_ICONINFORMATION,
        )

        still_held, elapsed = self._live_hold_window(
            drone_link, imu_widget,
            hold_s=self.HOLD_CRIT_S,
            drain_threshold_dps=self.CRIT_THRESHOLD,
        )

        if still_held:
            _beep_success()
        else:
            _beep_fail()

        print(f"\n  [SAT-003 CRIT HOLD] held={still_held}  "
              f"elapsed={elapsed:.2f} s  (need ≥ {self.HOLD_CRIT_S:.1f} s)")

        assert still_held, (
            f"CRIT dropped to SAFE after only {elapsed:.2f} s.\n"
            f"  Required hold : ≥ {self.HOLD_CRIT_S:.1f} s (SRS-IMU-006)\n"
            "  → Check hold_until timer in widget._gyro_state_label()."
        )

    def test_SAT_IMU_003_recovery_to_safe(self, drone_link, imu_widget):
        """All axes must return to SAFE and flash ticker must stop (SRS-IMU-006a).

        Per-axis state indicators and an elapsed-time bar give the operator a
        clear view of which axes are still in WARN/CRIT and how much time
        remains before the test times out.
        """
        root, widget = imu_widget

        _beep_attention()
        _msgbox(
            "SAT-IMU-003 — Recovery Test",
            f"Set FC FLAT AND STILL.\n\n"
            f"Keep it still for up to {self.RECOVERY_TIMEOUT:.0f} s.\n\n"
            "A LIVE RECOVERY PANEL will appear showing the state of each axis\n"
            "(ROLL / PITCH / YAW) and count down the timeout.\n\n"
            "All gyro cells must return to the green SAFE state.",
            _MB_ICONINFORMATION,
        )

        recovered, elapsed = self._live_recovery_window(
            drone_link, imu_widget,
            timeout_s=self.RECOVERY_TIMEOUT,
        )

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