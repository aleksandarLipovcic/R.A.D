# =============================================================================
# test_detection_map.py
#
# Unit tests for the copilot's detection window (DetectionMapWidget):
#   UT-DETMAP-001  Many records of one object (parked car) -> ONE row,
#                  updated in place: sightings, best confidence, last
#                  position, uncertainty radius
#   UT-DETMAP-002  Different objects get separate rows; untracked records
#                  (track_id 0, older backend) still get one row each
#   UT-DETMAP-003  Selecting a row shows the most confident sighting's
#                  screenshot and the fused position's radius
#   UT-DETMAP-004  The map draws an uncertainty circle per object
#
#   xvfb-run -a python -m pytest TestScripts/test_detection_map.py -v
# =============================================================================

from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.filterwarnings(
    "ignore::pytest.PytestUnraisableExceptionWarning"
)

from DetectionMapWidget import DetectionMapWidget

_next_id = [0]


def _rec(track_id, conf, lat=45.0, lon=15.0, radius=40.0, sightings=0, geo=True, t_ms=1_700_000_000_000):
    _next_id[0] += 1
    return SimpleNamespace(
        id=_next_id[0], track_id=track_id, class_name="car", confidence=conf,
        best_confidence=0.0, sightings=sightings, timestamp_ms=t_ms,
        georeferenced=geo, latitude=lat, longitude=lon, uncertainty_m=radius,
        distance_m=60.0, bearing_deg=90.0, range_method="coarse",
        screenshot_path=f"shot_{_next_id[0]}.jpg",
        telemetry=SimpleNamespace(heading_deg=90.0, gimbal_tilt_deg=30.0))


@pytest.fixture
def widget(tk_root):
    w = DetectionMapWidget(tk_root)
    w._load_preview = lambda path: setattr(w, "_shown_preview", path)   # no image files in tests
    yield w
    w.destroy()


def test_one_row_per_object(widget):
    """UT-DETMAP-001"""
    confs = [0.41, 0.55, 0.38, 0.81, 0.60] * 4            # 20 records of the same parked car
    for i, c in enumerate(confs):
        widget.add_records([_rec(7, c, radius=60.0 - i, sightings=(i + 1) * 13,
                                 t_ms=1_700_000_000_000 + i * 10_000)])
    rows = widget._tree.get_children()
    assert len(rows) == 1
    track, cls, best, seen, _t, geo, acc = widget._tree.item(rows[0], "values")
    assert (str(track), cls, best) == ("7", "car", "0.81")
    assert str(seen) == str(20 * 13)                       # backend's sighting count, not record count
    assert acc == "±41"                                    # newest (fused) radius
    assert geo.startswith("45.00000")


def test_separate_objects_and_untracked(widget):
    """UT-DETMAP-002"""
    widget.add_records([_rec(1, 0.5), _rec(2, 0.6), _rec(1, 0.7)])
    widget.add_records([_rec(0, 0.4), _rec(0, 0.45)])      # no tracker id: one row each
    assert len(widget._tree.get_children()) == 4


def test_select_shows_best_sighting(widget):
    """UT-DETMAP-003"""
    low, best, last = _rec(3, 0.40), _rec(3, 0.90), _rec(3, 0.50, radius=25.0)
    widget.add_records([low, best, last])
    row = widget._tree.get_children()[0]
    widget._tree.selection_set(row)
    widget._tree.focus(row)
    widget._on_select(None)
    assert widget._shown_preview == best.screenshot_path
    assert "±25 m" in widget._detail_lbl.cget("text")
    assert "best 0.90" in widget._detail_lbl.cget("text")


def test_map_draws_uncertainty_circle(widget):
    """UT-DETMAP-004"""
    widget.geometry("900x560")
    widget.update()
    widget.add_records([_rec(5, 0.7, radius=80.0), _rec(6, 0.7, lat=45.001, radius=30.0)])
    widget.update()
    widget._redraw_map()
    dashed = [i for i in widget._map_canvas.find_all()
              if widget._map_canvas.type(i) == "oval" and widget._map_canvas.itemcget(i, "dash")]
    assert len(dashed) == 2
