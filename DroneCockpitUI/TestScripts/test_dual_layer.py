# =============================================================================
# test_dual_layer.py
#
# Unit tests for the two-face (USB / radio) instrument panels:
#   UT-LAYER-001  LayerSwitch: debounced, all-or-nothing, NONE keeps layer
#   UT-LAYER-002  Switch time from a source change is < 2 s at the UI rate
#   UT-LAYER-003  DualLayerPanel: one face visible, feed routing, hidden hook
#   UT-LAYER-004  RadioAttitudePanel: tilt alarms, stale, link footer
#   UT-LAYER-005  RadioNavPanel: home arrow, turn cue, home-not-set
#   UT-LAYER-006  RadioFlightPanel: link bars, stale stats, shared timer
#   UT-LAYER-007  FCStatusWidget keeps the flight timer while hidden
#   UT-LAYER-008  TelemetryWorker link-statistics fields
#
#   xvfb-run -a python -m pytest TestScripts/test_dual_layer.py -v
# =============================================================================

import tkinter as tk
import pytest

pytestmark = pytest.mark.filterwarnings(
    "ignore::pytest.PytestUnraisableExceptionWarning"
)

import link_mode
from dual_layer import DualLayerPanel, LayerSwitch, LAYER_HOLD_S
from test_link_mode import _frame


def _radio(**kw):
    """Radio frame with fresh ELRS link statistics."""
    base = dict(elrs_link_stats_valid=True, elrs_uplink_lq=96,
                elrs_downlink_lq=88, elrs_uplink_rssi1_dbm=-67,
                elrs_uplink_rssi2_dbm=-71, elrs_uplink_snr=9,
                elrs_tx_power_mw=250, elrs_active_antenna=0,
                elrs_rf_mode_index=5, age_link_stats_ms=120,
                rate_battery_hz=1.0, rate_flight_mode_hz=1.0,
                radio_link_lost_count=0, radio_reconnect_count=1,
                radio_crc_errors=3, home_set=True, gps_comp_valid=True,
                gps_dist_to_home_m=420.0, gps_bearing_to_home=90)
    base.update(kw)
    return _frame("ELRS", **base)


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
    top = tk.Toplevel(tk_root)
    top.geometry("520x420")
    yield top
    try:
        top.destroy()
    except Exception:
        pass


def _mount(host, w):
    w.pack(fill="both", expand=True)
    host.update()
    return w


# ---------------------------------------------------------------------------
# UT-LAYER-001 / 002  LayerSwitch
# ---------------------------------------------------------------------------
class TestLayerSwitch:
    def test_starts_on_usb(self):
        assert LayerSwitch().radio is False

    def test_switch_needs_hold_time(self):
        sw = LayerSwitch(hold_s=1.0)
        assert not sw.observe("ELRS", 0.0)
        assert sw.pending
        assert not sw.observe("ELRS", 0.5)
        assert sw.observe("ELRS", 1.0)
        assert sw.radio is True and not sw.pending

    def test_none_keeps_current_layer(self):
        sw = LayerSwitch(hold_s=0.0)
        sw.observe("ELRS", 0.0)
        sw.observe("ELRS", 0.0)
        assert sw.radio
        for t in range(10):
            assert not sw.observe("NONE", float(t))
        assert sw.radio

    def test_flicker_does_not_switch(self):
        sw = LayerSwitch(hold_s=1.0)
        t = 0.0
        for _ in range(50):              # USB/ELRS alternating every 0.4 s
            sw.observe("ELRS", t); t += 0.4
            sw.observe("USB", t); t += 0.4
        assert sw.radio is False

    def test_switch_back_to_usb(self):
        sw = LayerSwitch(hold_s=1.0)
        sw.observe("ELRS", 0.0); sw.observe("ELRS", 1.0)
        assert sw.radio
        sw.observe("USB", 5.0)
        assert sw.observe("USB", 6.0)
        assert sw.radio is False

    @pytest.mark.parametrize("target", ["ELRS", "USB"])
    def test_switch_time_under_two_seconds(self, target):
        sw = LayerSwitch()
        sw.radio = target != "ELRS"      # start on the other layer
        t, tick = 0.0, 0.02              # 50 Hz UI pump
        while not sw.observe(target, t):
            t += tick
            assert t < 2.0
        assert LAYER_HOLD_S <= t < LAYER_HOLD_S + 2 * tick


# ---------------------------------------------------------------------------
# UT-LAYER-003  DualLayerPanel
# ---------------------------------------------------------------------------
class _Face(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, width=50, height=50)
        self.fed, self.hidden = [], []

    def upd(self, d):
        self.fed.append(d)

    def on_hidden_data(self, d):
        self.hidden.append(d)


def test_dual_layer_routing(host):
    dual = _mount(host, DualLayerPanel(host))
    usb, radio = _Face(dual), _Face(dual)
    dual.set_faces(usb, "upd", radio, "upd")
    host.update()
    assert usb.winfo_ismapped() and not radio.winfo_ismapped()

    dual.feed({"n": 1})
    assert usb.fed == [{"n": 1}] and radio.hidden == [{"n": 1}] and not radio.fed

    dual.show_radio(True)
    host.update()
    assert radio.winfo_ismapped() and not usb.winfo_ismapped()
    dual.feed({"n": 2})
    assert radio.fed == [{"n": 2}] and usb.hidden == [{"n": 2}]
    assert dual.active is radio and dual.radio_active


# ---------------------------------------------------------------------------
# UT-LAYER-004  RadioAttitudePanel
# ---------------------------------------------------------------------------
def test_attitude_panel_alarms_and_footer(host):
    from radio_panels import RadioAttitudePanel, CRIT_A, CRIT_B, AMBER_BG
    p = _mount(host, RadioAttitudePanel(host))
    p.update_radio(_radio(roll=35.0, pitch=18.0))
    assert p._values["roll"].cget("bg") in (CRIT_A, CRIT_B)
    assert p._values["pitch"].cget("bg") == AMBER_BG
    assert p._foot["lq"].cget("text") == "90%"
    assert p._foot["rssi"].cget("text") == "-67 dBm"
    assert p._foot["rate"].cget("text") == "12.5 Hz"
    assert p._tag.cget("text") == "RADIO"


def test_attitude_panel_stale(host):
    from radio_panels import RadioAttitudePanel
    p = _mount(host, RadioAttitudePanel(host))
    p.update_radio(_radio(roll=35.0, age_attitude_ms=3000))
    # stale overrides the alarm: an old attitude must not look live
    assert p._values["roll"].cget("fg") == link_mode.C_STALE_FG
    assert p._tag.cget("text") == "STALE 3.0s"
    assert not p._crit


# ---------------------------------------------------------------------------
# UT-LAYER-005  RadioNavPanel
# ---------------------------------------------------------------------------
def test_nav_panel_home(host):
    from radio_panels import RadioNavPanel
    p = _mount(host, RadioNavPanel(host))
    p.update_radio(_radio(yaw=0.0))            # home due east, nose north
    assert p._rows["dist"].cget("text") == "420 m"
    assert p._rows["brg"].cget("text") == "090°"
    assert p._rows["turn"].cget("text") == "R 90°"
    assert p._home_rel == pytest.approx(90.0)

    p.update_radio(_radio(yaw=85.0))
    assert p._rows["turn"].cget("text") == "ON COURSE"

    p.update_radio(_radio(gps_dist_to_home_m=2500.0))
    assert p._rows["dist"].cget("text") == "2.50 km"


def test_nav_panel_home_not_set(host):
    from radio_panels import RadioNavPanel
    p = _mount(host, RadioNavPanel(host))
    p.update_radio(_radio(home_set=False))
    assert p._home_rel is None
    assert p._rows["dist"].cget("text") == "HOME NOT SET"
    assert p._rows["brg"].cget("text") == "---"


def test_nav_panel_heading_readout_and_cal_buttons(host):
    from radio_panels import RadioNavPanel
    p = _mount(host, RadioNavPanel(host))
    p.update_radio(_radio(yaw=214.0))
    assert p._hdg_lbl.cget("text") == "214.0° SW"
    assert "RADIO" in p._status.cget("text")
    assert all(b.cget("state") == "disabled" for b in p._cal_btns)
    assert "USB ONLY" in p._cal_btns[0].cget("text")
    p.update_radio(_radio(armed=True))
    assert "DISARM FIRST" in p._cal_btns[0].cget("text")


# ---------------------------------------------------------------------------
# UT-LAYER-006  RadioFlightPanel
# ---------------------------------------------------------------------------
def test_flight_panel_link_and_timer(host):
    from radio_panels import RadioFlightPanel
    p = _mount(host, RadioFlightPanel(host, timer_provider=lambda: (125, 475, True)))
    p.update_radio(_radio(armed=True, flight_mode_name="ANGLE"))
    assert p._arm_lbl.cget("text") == "ARMED"
    assert p._bars["up"][1].cget("text") == "96 %"
    assert p._bars["rssi2"][1].cget("text") == "-71 dBm"
    assert "TX 250 mW" in p._link_line.cget("text")
    assert p._timer_lbl.cget("text") == "FLT 02:05    REM 07:55"
    assert "RECONNECT 1" in p._count_line.cget("text")


def test_flight_panel_stale(host):
    from radio_panels import RadioFlightPanel
    p = _mount(host, RadioFlightPanel(host))
    p.update_radio(_radio(age_link_stats_ms=6000, age_battery_ms=6000))
    assert p._bars["up"][1].cget("text") == "---"
    assert "NO LINK STATISTICS" in p._link_line.cget("text")
    assert p._bat_state.cget("text") == "STALE"


# ---------------------------------------------------------------------------
# UT-LAYER-007  FCStatusWidget shared timer
# ---------------------------------------------------------------------------
def test_fc_status_timer_runs_while_hidden(host):
    from FCStatusWidget import FCStatusWidget
    w = _mount(host, FCStatusWidget(host))
    w.on_hidden_data(_frame("ELRS", armed=True))
    w._tick()                             # timer tick without a visible update
    elapsed, remaining, armed = w.flight_timer()
    assert armed is True
    assert remaining == 600 - elapsed


# ---------------------------------------------------------------------------
# UT-LAYER-008  TelemetryWorker link-statistics fields
# ---------------------------------------------------------------------------
def test_worker_radio_link_fields_defaults():
    from telemetry_worker import _radio_link_fields
    d = _radio_link_fields(None)
    assert d["elrs_link_stats_valid"] is False
    assert d["age_link_stats_ms"] == -1
    assert d["home_set"] is False
