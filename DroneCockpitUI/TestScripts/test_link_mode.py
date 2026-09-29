# =============================================================================
# test_link_mode.py
#
# Unit tests for the USB / radio (ELRS) link-aware UI:
#   UT-LINK-001  link_mode helpers (source, availability, staleness)
#   UT-LINK-002  TelemetryWorker adds link keys to an ELRS frame
#   UT-LINK-003  IMUWidget: no fake gyro/acc values or red G alarm on radio
#   UT-LINK-004  MagWidget: radio heading label, raw bars + calibration off
#   UT-LINK-005  ArmingWidget: unverifiable checks never give READY TO ARM
#   UT-LINK-006  GPSWidget: radio fix text, never "3D FIX"
#   UT-LINK-007  FCStatusWidget: USB-only metrics show a placeholder
#   UT-LINK-008  Drone3DView: stale attitude is hatched and labelled
#   UT-LINK-009  BaroWidget: altitude source tag
#
# All widget tests run headless (Xvfb on CI / Linux):
#   xvfb-run -a python -m pytest TestScripts/test_link_mode.py -v
# =============================================================================

import tkinter as tk
import pytest

pytestmark = pytest.mark.filterwarnings(
    "ignore::pytest.PytestUnraisableExceptionWarning"
)

import link_mode
from telemetry_worker import TelemetryWorker


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------
def _frame(source="USB", **overrides):
    """A ui_data frame shaped like TelemetryWorker._build_ui_data() output."""
    radio = source == "ELRS"
    d = {
        "ax": 0, "ay": 0, "az": 0 if radio else 2048,
        "gx": 0, "gy": 0, "gz": 0,
        "roll": 2.0, "pitch": -1.0, "yaw": 90.0,
        "mag_heading_deg": 90.0, "mag_valid": True,
        "mag_x": 0 if radio else 120, "mag_y": 0 if radio else -40,
        "mag_z": 0 if radio else 300,
        "battery_voltage": 16.4, "battery_current": 3.2,
        "battery_mah_drawn": 120, "battery_cell_count": 4,
        "battery_percentage": 90, "battery_state": "OK",
        "baro_valid": True, "baro_altitude_cm": 1234,
        "baro_vario_cm_per_sec": 0,
        "sensor_acc_present": True, "sensor_gyro_present": True,
        "sensor_baro_present": True, "sensor_gps_present": True,
        "sensor_mag_present": not radio, "sensor_rangefinder_present": False,
        "armed": False, "flight_mode_name": "ANGLE [DISARMED]",
        "cpu_load_percent": 0 if radio else 22, "fc_cycle_ms": 0 if radio else 0.25,
        "i2c_error_count": 0, "pid_profile": 0,
        "motor_1_us": 0, "motor_2_us": 0, "motor_3_us": 0, "motor_4_us": 0,
        "rc_channel_count": 0 if radio else 16,
        "rc_roll": 0 if radio else 1500, "rc_pitch": 0 if radio else 1500,
        "rc_throttle": 0 if radio else 1000, "rc_yaw": 0 if radio else 1500,
        "rc_arm": 0 if radio else 1000,
        "rc_link_quality": 90,
        "gps_fix_type": 2, "gps_num_sat": 9, "gps_num_sats": 9,
        "gps_hdop": 99.0 if radio else 0.9,
        "gps_latitude": 44.77, "gps_longitude": 17.19, "gps_altitude_m": 160.0,
        "gps_ground_speed_cms": 0, "gps_ground_course": 0,
        "gps_dist_to_home_m": 0.0, "gps_bearing_to_home": 0,
        "gps_heartbeat": 1, "gps_raw_valid": True, "gps_comp_valid": False,
        "gps_position_usable": True, "gps_sv_list": [],
        "rtt_ms": 2.0,
    }
    if radio:
        d.update({
            "link_source": "ELRS",
            "available_raw_imu": False, "available_raw_mag": False,
            "available_motors": False, "available_rc_channels": False,
            "available_sat_list": False, "available_hdop": False,
            "available_arming_flags": False, "available_cpu_load": False,
            "arming_blocked": False, "gps_waiting": False,
            "altitude_source": "BARO",
            "age_attitude_ms": 50, "age_gps_ms": 400, "age_battery_ms": 400,
            "age_flight_mode_ms": 400, "age_baro_ms": 400,
            "rate_attitude_hz": 12.5, "rate_gps_hz": 1.0,
        })
    else:
        d["link_source"] = "USB"
    d.update(overrides)
    return d


@pytest.fixture(scope="module")
def tk_root():
    root = tk.Tk()
    root.withdraw()
    yield root
    try:
        root.destroy()
    except Exception:
        pass


@pytest.fixture
def host(tk_root):
    """A visible 900x520 Toplevel so widgets pick their full layout tier."""
    top = tk.Toplevel(tk_root)
    top.geometry("900x520")
    yield top
    try:
        top.destroy()
    except Exception:
        pass


def _mount(host, widget):
    widget.pack(fill="both", expand=True)
    host.update()
    return widget


# ---------------------------------------------------------------------------
# UT-LINK-001  link_mode helpers
# ---------------------------------------------------------------------------
class TestLinkModeHelpers:
    def test_frame_without_source_is_usb(self):
        assert link_mode.source({}) == "USB"
        assert not link_mode.is_radio({})

    def test_unknown_capability_counts_as_available(self):
        assert link_mode.available({}, "motors")
        assert not link_mode.available({"available_motors": False}, "motors")

    def test_usb_never_stale(self):
        assert link_mode.stale_ms({"age_attitude_ms": 99999}, "attitude") == 0

    def test_radio_fresh_and_stale(self):
        d = {"link_source": "ELRS", "age_attitude_ms": 200}
        assert link_mode.stale_ms(d, "attitude") == 0
        d["age_attitude_ms"] = 2000
        assert link_mode.stale_ms(d, "attitude") == 2000
        d["age_attitude_ms"] = -1
        assert link_mode.stale_ms(d, "attitude") == -1

    def test_stale_text(self):
        assert link_mode.stale_text(3200) == "STALE 3.2s"
        assert link_mode.stale_text(-1) == "NO DATA"


# ---------------------------------------------------------------------------
# UT-LINK-002  TelemetryWorker frame keys
# ---------------------------------------------------------------------------
class _Obj:
    """Backend-struct stand-in: explicit attributes, 0 for anything else."""
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        return 0


def test_worker_adds_radio_keys():
    radio = _Obj(link_stats_valid=True, uplink_lq=87, arming_blocked=False,
                 gps_waiting=False, altitude_source="GPS",
                 attitude_age_ms=40, gps_age_ms=900, battery_age_ms=700,
                 flight_mode_age_ms=650, baro_age_ms=-1,
                 attitude_hz=15.0, gps_hz=1.0)
    state = _Obj(yaw=90, roll=10, pitch=-20, sv_list=[], rc_channels=[],
                 motor_values=[], gps=_Obj(hdop=9999), radio=radio,
                 battery_state=_Obj(name="OK"))
    w = TelemetryWorker(hub=None)
    d = w._build_ui_data(state, "ELRS")
    assert d["link_source"] == "ELRS"
    assert d["rc_link_quality"] == 87
    assert d["available_motors"] is False
    assert d["age_flight_mode_ms"] == 650
    assert d["age_baro_ms"] == -1
    assert d["rate_attitude_hz"] == 15.0
    assert d["altitude_source"] == "GPS"


# ---------------------------------------------------------------------------
# UT-LINK-003  IMUWidget
# ---------------------------------------------------------------------------
def _imu(host):
    from IMUWidget import IMUWidget
    return _mount(host, IMUWidget(host))


def test_imu_radio_shows_placeholders_not_alarms(host):
    w = _imu(host)
    w.update_ui(_frame("ELRS"))
    host.update()
    assert w._current_tier == "full"
    axes = w._tier_widgets["axes"]
    for key in ("roll", "pitch", "yaw"):
        assert axes[key]["rot"].cget("text") == link_mode.NA_SHORT
        assert axes[key]["acc"].cget("text") == link_mode.NA_SHORT
    # az = 0 on the radio link must NOT flash the vertical-G cell red
    assert not w._crit_cells
    assert w._tier_widgets["adj_btn"].cget("state") == "disabled"
    assert w._tier_widgets["rtt_lbl_name"].cget("text") == "ATT AGE:"
    assert w._tier_widgets["src_lbl"].cget("text") == "RADIO"


def test_imu_radio_stale_attitude(host):
    w = _imu(host)
    w.update_ui(_frame("ELRS", age_attitude_ms=4000))
    host.update()
    ang = w._tier_widgets["axes"]["roll"]["ang"]
    assert ang.cget("fg") == link_mode.C_STALE_FG
    assert w._tier_widgets["src_lbl"].cget("text") == "STALE 4.0s"


def test_imu_back_to_usb_restores(host):
    w = _imu(host)
    w.update_ui(_frame("ELRS"))
    w.update_ui(_frame("USB"))
    host.update()
    assert w._tier_widgets["adj_btn"].cget("state") == "normal"
    assert w._tier_widgets["rtt_lbl_name"].cget("text") == "TOTAL RTT:"
    acc = w._tier_widgets["axes"]["yaw"]["acc"].cget("text")
    assert acc.strip() == "1.000"


# ---------------------------------------------------------------------------
# UT-LINK-004  MagWidget
# ---------------------------------------------------------------------------
def test_mag_radio(host):
    from MagWidget import MagWidget
    w = _mount(host, MagWidget(host, on_mag_calibrate=lambda: None,
                               on_acc_calibrate=lambda: None))
    w.update_mag(_frame("ELRS"))
    host.update()
    tw = w._tier_widgets
    assert tw["mag_btn"].cget("state") == "disabled"
    assert link_mode.NA_LONG in tw["mag_btn"].cget("text")
    assert tw["acc_btn"].cget("state") == "disabled"
    assert "RADIO" in tw["status"].cget("text")
    if "home_frame" in tw:              # full layout: home row replaces raw bars
        assert tw["home_frame"].winfo_ismapped()
        assert not tw["bars_frame"].winfo_ismapped()

    w.update_mag(_frame("USB"))
    host.update()
    assert w._tier_widgets["mag_btn"].cget("state") == "normal"
    assert "LOCK" in w._tier_widgets["status"].cget("text")


# ---------------------------------------------------------------------------
# UT-LINK-005  ArmingWidget
# ---------------------------------------------------------------------------
def _arming(host):
    from ArmingWidget import ArmingWidget
    w = _mount(host, ArmingWidget(host))
    for var in w._manual_vars.values():
        var.set(True)
    return w


def test_arming_radio_motors_unverified(host):
    w = _arming(host)
    w.update_arming(_frame("ELRS"))
    host.update()
    assert w._auto_state["motors"] is None
    banner = w._banner_lbl.cget("text")
    assert "NEED USB" in banner
    assert "READY TO ARM" not in banner


def test_arming_usb_ready(host):
    w = _arming(host)
    w.update_arming(_frame("USB"))
    host.update()
    assert w._auto_state["motors"] is True
    assert "READY TO ARM" in w._banner_lbl.cget("text")


def test_arming_radio_blocked_fails_rc_row(host):
    w = _arming(host)
    w.update_arming(_frame("ELRS", arming_blocked=True))
    host.update()
    assert w._auto_state["rc_link"] is False
    assert "BLOCKED" in w._banner_lbl.cget("text")


# ---------------------------------------------------------------------------
# UT-LINK-006  GPSWidget fix text
# ---------------------------------------------------------------------------
def test_gps_fix_status_text():
    from GPSWidget import _fix_status
    assert _fix_status(True, 2)[0] == "3D FIX"
    assert _fix_status(True, 2, radio=True)[0] == "RF FIX"
    assert _fix_status(True, 0, radio=True)[0] == "NO FIX"
    assert _fix_status(True, 2, radio=True, stale=True)[0].strip() == "STALE"


def test_gps_widget_radio_frame(host):
    from GPSWidget import GPSWidget
    w = _mount(host, GPSWidget(host))
    w.update_gps(_frame("ELRS"))
    host.update()
    assert w._sat_panel._note and "USB only" in w._sat_panel._note


# ---------------------------------------------------------------------------
# UT-LINK-007  FCStatusWidget
# ---------------------------------------------------------------------------
def test_fc_status_radio(host):
    from FCStatusWidget import FCStatusWidget
    w = _mount(host, FCStatusWidget(host))
    w.update_fc_status(_frame("ELRS"))
    host.update()
    assert w._src_lbl.cget("text") == "RADIO"
    assert w._lbl_cycle.cget("text") == link_mode.NA_SHORT
    assert w._cpu_lbl.cget("text") == link_mode.NA_SHORT
    assert w._lq_lbl.cget("text") == "ELRS 90%"
    if w._motor_mode == "bars":
        assert w._motor_lbls[0].cget("text") == link_mode.NA_SHORT
    # radio cannot confirm the magnetometer → "MAG?", not a dead pill
    assert w._sensor_refs["MAG"][2].cget("text") == "MAG?"


def test_fc_status_radio_battery_stale(host):
    from FCStatusWidget import FCStatusWidget
    w = _mount(host, FCStatusWidget(host))
    w.update_fc_status(_frame("ELRS", age_battery_ms=8000))
    host.update()
    assert w._bat_state_lbl.cget("text") == "STALE"


# ---------------------------------------------------------------------------
# UT-LINK-008  Drone3DView
# ---------------------------------------------------------------------------
def _canvas_texts(cv):
    return [cv.itemcget(i, "text") for i in cv.find_all()
            if cv.type(i) == "text"]


def test_adi_stale_overlay(host):
    from Drone3DView import Drone3DView
    v = _mount(host, Drone3DView(host))
    v.update_orientation(2.0, -1.0, 90.0, 90.0, True,
                         source="ELRS", stale_ms=2500)
    texts = _canvas_texts(v)
    assert any("STALE 2.5s" in t for t in texts)
    assert "FC·RF" in texts
    assert "MAG◆" not in texts


def test_adi_usb_unchanged(host):
    from Drone3DView import Drone3DView
    v = _mount(host, Drone3DView(host))
    v.update_orientation(2.0, -1.0, 90.0, 90.0, True)
    texts = _canvas_texts(v)
    assert "GYRO" in texts and "MAG◆" in texts
    assert not any("STALE" in t for t in texts)


# ---------------------------------------------------------------------------
# UT-LINK-009  BaroWidget
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("overrides, expected", [
    ({"link_source": "USB"}, "BARO · USB"),
    ({}, "BARO · RADIO"),
    ({"altitude_source": "GPS"}, "GPS Δ HOME · RADIO"),
    ({"altitude_source": "NONE"}, "NO ALT · RADIO"),
    ({"age_baro_ms": 9000}, "STALE 9.0s"),
])
def test_baro_source_tag(host, overrides, expected):
    from BaroWidget import BaroWidget
    w = _mount(host, BaroWidget(host))
    frame = _frame("USB") if overrides.get("link_source") == "USB" \
        else _frame("ELRS", **overrides)
    w.update_baro(frame)
    assert w._src_lbl.cget("text") == expected


# ---------------------------------------------------------------------------
# UT-LINK-010  Calibration refused while armed (USB magnetometer)
# ---------------------------------------------------------------------------
def test_mag_calibration_disabled_while_armed(host):
    from MagWidget import MagWidget
    calls = []
    w = _mount(host, MagWidget(host, on_mag_calibrate=lambda: calls.append("mag"),
                               on_acc_calibrate=lambda: calls.append("acc")))
    w.update_mag(_frame("USB", armed=True))
    host.update()
    assert w._tier_widgets["mag_btn"].cget("state") == "disabled"
    assert "DISARM" in w._tier_widgets["mag_btn"].cget("text")
    w._on_mag_cal_pressed()
    assert calls == []
    w.update_mag(_frame("USB", armed=False))
    host.update()
    assert w._tier_widgets["mag_btn"].cget("state") == "normal"


# ---------------------------------------------------------------------------
# UT-LINK-011  IMU latency footer: hidden by default, toggle works
# ---------------------------------------------------------------------------
def test_imu_latency_footer_toggle(host):
    from IMUWidget import IMUWidget
    w = _mount(host, IMUWidget(host))
    w.update_ui(_frame("USB"))
    host.update()
    assert not w._diag_visible
    w._show_latency.set(True)
    w._on_latency_toggle()
    host.update()
    assert w._diag_visible
    w._show_latency.set(False)
    w._on_latency_toggle()
    assert not w._diag_visible
