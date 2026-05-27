# =============================================================================
# test_imu_widget.py
#
# Unit tests for IMUWidget alert state machine logic:
#   UT-ALERT-001  Safe state below WARN threshold
#   UT-ALERT-002  WARN threshold entry and 2 s hold-down
#   UT-ALERT-003  CRIT threshold entry and 4 s hold-down
#   UT-ALERT-004  Flash ticker — start on register, stop on last unregister
#
# V-Model reference: IMU Subsystem V-Model, Section 6.3
# SRS coverage:      SRS-IMU-005, SRS-IMU-006, SRS-IMU-006a
#
# API contract with IMUWidget (verified against IMUWidget.py):
#   _gyro_state_label(axis, abs_dps, now)  — takes ABSOLUTE °/s already
#   _crit_cells                            — dict registry (NOT _crit_labels)
#   _register_crit(lbl) / _unregister_crit(lbl)
#   _cell_flash_job                        — None when ticker stopped
#   _FLASH_MS = 500
#
# Run:
#   pytest test_imu_widget.py -v
# =============================================================================

import time
import tkinter as tk
import pytest

pytestmark = pytest.mark.filterwarnings(
    "ignore::pytest.PytestUnraisableExceptionWarning"
)

from IMUWidget import IMUWidget


# ---------------------------------------------------------------------------
# Shared Tkinter root — one per session
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def tk_root():
    root = tk.Tk()
    root.withdraw()
    yield root
    try:
        root.update_idletasks()
    except Exception:
        pass
    try:
        root.destroy()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Fresh IMUWidget per test — full state isolation
#
# FIX (was):  teardown called tk_root.after_cancel() then tk_root.update().
#
# The tk_root.update() call processes ALL pending Tkinter after() callbacks
# across every widget alive in the session.  If any of those callbacks raises
# a TclError (e.g. _do_flash firing on a stale widget), the error goes through
# Tkinter's report_callback_exception() handler — completely bypassing the
# surrounding `except Exception: pass` — and pytest catches it via
# sys.unraisablehook, marking the current test as failed even though its own
# assertion passed.
#
# FIX (now):
#   1. Cancel pending jobs via w.after_cancel() (widget-local, not tk_root).
#   2. Call w.destroy() so the widget's Tcl path is deleted and any remaining
#      after() callbacks on it raise TclError silently inside their own
#      try/except guards rather than propagating through the event loop.
#   3. Use update_idletasks() (idle queue only) instead of update() (full event
#      + timer dispatch) to flush geometry events without firing timer callbacks.
# ---------------------------------------------------------------------------
@pytest.fixture
def widget(tk_root):
    w = IMUWidget(tk_root)
    for axis in ("roll", "pitch", "yaw"):
        w._gyro_state[axis] = {"state": "safe", "hold_until": 0.0}

    yield w

    # ── Cancel every known pending after() job on this widget ────────────────
    # Must use w.after_cancel(), not tk_root.after_cancel().  Both reach the
    # same Tcl interpreter, but using the widget keeps intent clear and avoids
    # accidentally cancelling unrelated root-level jobs.
    for attr in ("_cell_flash_job", "_flash_job"):
        job = getattr(w, attr, None)
        if job is not None:
            try:
                w.after_cancel(job)
            except Exception:
                pass
        setattr(w, attr, None)

    w._crit_cells.clear()

    # ── Destroy the widget to release its Tcl path ───────────────────────────
    # IMUWidget.destroy() cancels flash jobs a second time (safe, idempotent)
    # then calls super().destroy() which deletes the Tcl widget command.
    # Any after() callbacks that fire after this point will get a TclError
    # inside their own try/except guards and stop themselves cleanly.
    try:
        w.destroy()
    except Exception:
        pass

    # ── Flush idle events only — no timer callbacks ───────────────────────────
    # update_idletasks() processes pending geometry/layout events (e.g. any
    # Configure callbacks queued during widget creation) without dispatching
    # pending after() timer callbacks.  This avoids firing stale timers from
    # other tests' widgets that happen to be resident in the event queue.
    try:
        tk_root.update_idletasks()
    except Exception:
        pass


# =============================================================================
# UT-ALERT-001 : Safe state below WARN threshold
# SRS: SRS-IMU-005
# =============================================================================

class TestSafeState:

    def test_UT_ALERT_001_below_warn_threshold(self, widget):
        """29.9 °/s must stay 'safe' — one count below the 30 °/s boundary."""
        t = time.monotonic()
        assert widget._gyro_state_label("roll", 29.9, t) == "safe"

    def test_UT_ALERT_001_zero_rate_is_safe(self, widget):
        """0 °/s must be 'safe'."""
        assert widget._gyro_state_label("pitch", 0.0, time.monotonic()) == "safe"

    def test_UT_ALERT_001_small_positive_rate(self, widget):
        """5.0 °/s is well below WARN — must be 'safe'."""
        assert widget._gyro_state_label("yaw", 5.0, time.monotonic()) == "safe"

    def test_UT_ALERT_001_boundary_just_below(self, widget):
        """29.99 °/s must be safe (strict less-than at boundary)."""
        assert widget._gyro_state_label("roll", 29.99, time.monotonic()) == "safe"

    def test_UT_ALERT_001_hold_until_zero_after_safe(self, widget):
        """hold_until must remain 0.0 when state stays safe."""
        t = time.monotonic()
        widget._gyro_state_label("roll", 5.0, t)
        assert widget._gyro_state["roll"]["hold_until"] == 0.0

    def test_UT_ALERT_001_all_axes_independent(self, widget):
        """All three axes independently report safe at low rates."""
        t = time.monotonic()
        for axis, rate in [("roll", 10.0), ("pitch", 0.1), ("yaw", 5.0)]:
            result = widget._gyro_state_label(axis, rate, t)
            assert result == "safe", f"Axis '{axis}' expected safe, got '{result}'"


# =============================================================================
# UT-ALERT-002 : WARN threshold entry and 2 s hold-down
# SRS: SRS-IMU-005
# =============================================================================

class TestWarnState:

    def test_UT_ALERT_002_entry_at_30dps(self, widget):
        """Exactly 30.0 °/s must enter WARN."""
        t = time.monotonic()
        assert widget._gyro_state_label("roll", 30.0, t) == "warn"

    def test_UT_ALERT_002_entry_at_50dps(self, widget):
        """50 °/s (inside WARN band) must be 'warn'."""
        assert widget._gyro_state_label("roll", 50.0, time.monotonic()) == "warn"

    def test_UT_ALERT_002_entry_at_99p9dps(self, widget):
        """99.9 °/s — just below CRIT boundary — must be 'warn'."""
        assert widget._gyro_state_label("pitch", 99.9, time.monotonic()) == "warn"

    def test_UT_ALERT_002_hold_active_at_0p5s(self, widget):
        """After WARN entry: dropping to 5 °/s at t+0.5 s must still be 'warn'."""
        t0 = time.monotonic()
        assert widget._gyro_state_label("roll", 30.0, t0) == "warn"
        assert widget._gyro_state_label("roll", 5.0, t0 + 0.5) == "warn"

    def test_UT_ALERT_002_hold_active_at_1p9s(self, widget):
        """Hold still active at t+1.9 s."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 30.0, t0)
        assert widget._gyro_state_label("roll", 5.0, t0 + 1.9) == "warn"

    def test_UT_ALERT_002_hold_expires_at_2p1s(self, widget):
        """WARN hold must expire: at t+2.1 s with rate=5 °/s → 'safe'."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 30.0, t0)
        assert widget._gyro_state_label("roll", 5.0, t0 + 2.1) == "safe"

    def test_UT_ALERT_002_hold_timer_value(self, widget):
        """hold_until must be approximately t0 + 2.0 s on WARN entry."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 30.0, t0)
        hold_until = widget._gyro_state["roll"]["hold_until"]
        assert abs(hold_until - (t0 + 2.0)) < 0.05, (
            f"hold_until offset = {hold_until - t0:.4f}s, expected ~2.0s"
        )

    def test_UT_ALERT_002_axes_independent(self, widget):
        """WARN on 'roll' must not affect 'pitch'."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 30.0, t0)
        assert widget._gyro_state_label("pitch", 5.0, t0) == "safe"


# =============================================================================
# UT-ALERT-003 : CRIT threshold entry and 4 s hold-down
# SRS: SRS-IMU-006
# =============================================================================

class TestCritState:

    def test_UT_ALERT_003_entry_at_100dps(self, widget):
        """Exactly 100 °/s must enter CRIT."""
        assert widget._gyro_state_label("roll", 100.0, time.monotonic()) == "crit"

    def test_UT_ALERT_003_entry_above_100dps(self, widget):
        """500 °/s must be CRIT."""
        assert widget._gyro_state_label("yaw", 500.0, time.monotonic()) == "crit"

    def test_UT_ALERT_003_four_sample_sequence(self, widget):
        """Full 4-sample sequence: s1=crit, s2=crit, s3=crit, s4=safe."""
        t0 = time.monotonic()
        s1 = widget._gyro_state_label("roll", 100.0, t0)
        s2 = widget._gyro_state_label("roll", 5.0,   t0 + 1.0)
        s3 = widget._gyro_state_label("roll", 5.0,   t0 + 3.9)
        s4 = widget._gyro_state_label("roll", 5.0,   t0 + 4.1)
        assert s1 == "crit", f"s1={s1}"
        assert s2 == "crit", f"s2={s2}"
        assert s3 == "crit", f"s3={s3}"
        assert s4 == "safe", f"s4={s4}"

    def test_UT_ALERT_003_hold_timer_set_to_4s(self, widget):
        """CRIT hold_until must be approximately t0 + 4.0 s."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 100.0, t0)
        hold_until = widget._gyro_state["roll"]["hold_until"]
        assert abs(hold_until - (t0 + 4.0)) < 0.05

    def test_UT_ALERT_003_crit_overrides_warn(self, widget):
        """WARN state must escalate to CRIT when rate reaches 100 °/s."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 30.0, t0)
        assert widget._gyro_state["roll"]["state"] == "warn"
        result = widget._gyro_state_label("roll", 100.0, t0 + 0.1)
        assert result == "crit", f"WARN → CRIT escalation failed, got '{result}'"

    def test_UT_ALERT_003_hold_refreshed_during_crit(self, widget):
        """hold_until must refresh on each CRIT sample."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 100.0, t0)
        hold_1 = widget._gyro_state["roll"]["hold_until"]
        widget._gyro_state_label("roll", 150.0, t0 + 1.0)
        hold_2 = widget._gyro_state["roll"]["hold_until"]
        assert hold_2 > hold_1

    def test_UT_ALERT_003_after_expiry_warn_band_is_warn(self, widget):
        """After CRIT hold expires, 50 °/s (in WARN band) → 'warn'."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 100.0, t0)
        result = widget._gyro_state_label("roll", 50.0, t0 + 4.1)
        assert result == "warn", f"Expected 'warn' after CRIT expiry, got '{result}'"

    def test_UT_ALERT_003_after_expiry_safe_band_is_safe(self, widget):
        """After CRIT hold expires, 5 °/s → 'safe'."""
        t0 = time.monotonic()
        widget._gyro_state_label("roll", 100.0, t0)
        result = widget._gyro_state_label("roll", 5.0, t0 + 4.1)
        assert result == "safe", f"Expected 'safe' after CRIT expiry, got '{result}'"


# =============================================================================
# UT-ALERT-004 : Flash ticker
# SRS: SRS-IMU-006a
# =============================================================================

class TestFlashTicker:

    def test_UT_ALERT_004_flash_ms_constant(self, widget):
        """_FLASH_MS must equal 500."""
        assert widget._FLASH_MS == 500

    def test_UT_ALERT_004_ticker_starts_on_register(self, widget, tk_root):
        """Registering a CRIT cell must start the flash ticker job."""
        lbl = tk.Label(tk_root, text="GX")
        assert widget._cell_flash_job is None
        widget._register_crit(lbl)
        assert widget._cell_flash_job is not None
        widget._unregister_crit(lbl)

    def test_UT_ALERT_004_ticker_not_duplicated(self, widget, tk_root):
        """Registering a second CRIT cell must not create a second ticker job."""
        la = tk.Label(tk_root, text="GX")
        lb = tk.Label(tk_root, text="GY")
        widget._register_crit(la)
        job1 = widget._cell_flash_job
        widget._register_crit(lb)
        job2 = widget._cell_flash_job
        assert job1 == job2, "Second register must not spawn a duplicate ticker"
        widget._unregister_crit(la)
        widget._unregister_crit(lb)

    def test_UT_ALERT_004_registry_tracks_labels(self, widget, tk_root):
        """_crit_cells must contain the label after register, not after unregister."""
        lbl = tk.Label(tk_root, text="GZ")
        widget._register_crit(lbl)
        assert lbl in widget._crit_cells
        widget._unregister_crit(lbl)
        assert lbl not in widget._crit_cells

    def test_UT_ALERT_004_ticker_stops_after_last_unregister(self, widget, tk_root):
        """Ticker must self-cancel once _crit_cells is empty."""
        lbl = tk.Label(tk_root, text="FLASH")
        widget._register_crit(lbl)
        assert widget._cell_flash_job is not None
        widget._unregister_crit(lbl)
        assert len(widget._crit_cells) == 0
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            tk_root.update()
            time.sleep(0.01)
        assert widget._cell_flash_job is None, (
            "Flash ticker must stop after all CRIT cells unregistered"
        )

    def test_UT_ALERT_004_partial_unregister_keeps_ticker(self, widget, tk_root):
        """Removing some but not all CRIT cells must keep the ticker running."""
        la = tk.Label(tk_root, text="A")
        lb = tk.Label(tk_root, text="B")
        lc = tk.Label(tk_root, text="C")
        widget._register_crit(la)
        widget._register_crit(lb)
        widget._register_crit(lc)
        widget._unregister_crit(la)
        widget._unregister_crit(lb)
        assert widget._cell_flash_job is not None, (
            "Ticker must keep running while ≥1 CRIT label remains"
        )
        widget._unregister_crit(lc)
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            tk_root.update()
            time.sleep(0.01)
        assert widget._cell_flash_job is None


# =============================================================================
# Constant / threshold inspection (SRS-IMU-004c)
# =============================================================================

def test_gyro_scale():
    """GYRO_SCALE must be 16.4 per SRS-IMU-004c."""
    assert IMUWidget.GYRO_SCALE == 16.4

def test_accel_scale():
    """ACCEL_SCALE must be 2048.0 per SRS-IMU-004c."""
    assert IMUWidget.ACCEL_SCALE == 2048.0

def test_gyro_warn_dps():
    """GYRO_WARN_DPS must be 30.0 per SRS-IMU-005."""
    assert IMUWidget.GYRO_WARN_DPS == 30.0

def test_gyro_crit_dps():
    """GYRO_CRIT_DPS must be 100.0 per SRS-IMU-006."""
    assert IMUWidget.GYRO_CRIT_DPS == 100.0

def test_hold_warn_sec():
    """HOLD_WARN_SEC must be 2.0 per SRS-IMU-005."""
    assert IMUWidget.HOLD_WARN_SEC == 2.0

def test_hold_crit_sec():
    """HOLD_CRIT_SEC must be 4.0 per SRS-IMU-006."""
    assert IMUWidget.HOLD_CRIT_SEC == 4.0

def test_flash_ms():
    """_FLASH_MS must be 500 per SRS-IMU-006a."""
    assert IMUWidget._FLASH_MS == 500