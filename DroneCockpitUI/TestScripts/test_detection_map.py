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
#   UT-DETMAP-005  Object filter (class / vehicle group / confidence /
#                  sightings / text) hides rows AND their map circles
#   UT-DETMAP-006  find_scenes: ">= N" / "= 0" / vehicle group, merging
#                  passes less than the gap apart into one scene
#   UT-DETMAP-007  Scene search in the window: selecting a scene shows only
#                  its objects, "Clear scene" restores the list
#   UT-DETMAP-008  Frame index keeps "last seen" current; "last N min"
#                  filter drops objects not seen recently
#   UT-DETMAP-009  DetectionWorker pulls the frame index incrementally and
#                  degrades cleanly on a DroneBackend without it
#
#   xvfb-run -a python -m pytest TestScripts/test_detection_map.py -v
# =============================================================================

from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.filterwarnings(
    "ignore::pytest.PytestUnraisableExceptionWarning"
)

from DetectionMapWidget import DetectionMapWidget, find_scenes
from detection_worker import DetectionWorker

_next_id = [0]


def _rec(track_id, conf, lat=45.0, lon=15.0, radius=40.0, sightings=0, geo=True, t_ms=1_700_000_000_000,
         cls="car"):
    _next_id[0] += 1
    return SimpleNamespace(
        id=_next_id[0], track_id=track_id, class_name=cls, confidence=conf,
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


def _rows(widget):
    return [widget._tree.item(i, "values") for i in widget._tree.get_children()]


def _dashed_circles(widget):
    widget.update()
    widget._redraw_map()
    return [i for i in widget._map_canvas.find_all()
            if widget._map_canvas.type(i) == "oval" and widget._map_canvas.itemcget(i, "dash")]


def test_object_filter(widget):
    """UT-DETMAP-005"""
    widget.geometry("900x700")
    widget.add_records([_rec(1, 0.80, cls="person", sightings=12),
                        _rec(2, 0.40, cls="car", sightings=3, lat=45.001),
                        _rec(3, 0.65, cls="large_vehicle", sightings=30, lat=45.002)])
    assert len(_rows(widget)) == 3 and len(_dashed_circles(widget)) == 3

    widget._flt_class.set("person")
    assert [r[0] for r in _rows(widget)] == ["1"] and len(_dashed_circles(widget)) == 1
    widget._flt_class.set("vehicle")                       # car + large_vehicle
    assert sorted(r[0] for r in _rows(widget)) == ["2", "3"]
    widget._flt_conf.set("0.50")
    assert [r[0] for r in _rows(widget)] == ["3"]
    widget.reset_search()
    widget._flt_seen.set("10")
    assert sorted(r[0] for r in _rows(widget)) == ["1", "3"]
    widget.reset_search()
    widget._flt_text.set("#2")
    assert [r[0] for r in _rows(widget)] == ["2"]
    assert "Showing 1 of 3" in widget._search_status.get()
    widget.reset_search()
    assert [r[0] for r in _rows(widget)] == ["1", "2", "3"]   # arrival order kept


def test_find_scenes():
    """UT-DETMAP-006"""
    t = 1_700_000_000_000
    P, C, L = "person", "car", "large_vehicle"
    frames = [(t + 0,    ((1, P),)),
              (t + 250,  ((1, P), (2, P))),
              (t + 500,  ((1, P), (2, P), (3, C))),
              (t + 750,  ((3, C),)),
              (t + 5000, ((4, P), (5, P), (6, L))),
              (t + 9000, ())]
    s = find_scenes(frames, "person", ">=", 2)
    assert [(x["start_ms"] - t, x["end_ms"] - t, x["max_count"], x["track_ids"]) for x in s] == \
        [(250, 500, 2, {1, 2}), (5000, 5000, 2, {4, 5})]
    s = find_scenes(frames, "person", ">=", 1)
    assert len(s) == 2 and s[0]["start_ms"] == t and s[0]["end_ms"] == t + 500   # 0..500 merged
    s = find_scenes(frames, "vehicle", ">=", 1)
    assert [x["track_ids"] for x in s] == [{3}, {6}]
    s = find_scenes(frames, "any", "=", 0)
    assert [x["start_ms"] - t for x in s] == [9000]
    assert find_scenes([], "person", ">=", 1) == []


def test_scene_search_filters_list(widget):
    """UT-DETMAP-007"""
    t = 1_700_000_000_000
    widget.add_records([_rec(1, 0.8, cls="person", t_ms=t), _rec(2, 0.7, cls="person", t_ms=t),
                        _rec(3, 0.9, cls="car", t_ms=t), _rec(4, 0.6, cls="person", t_ms=t + 60_000)])
    widget.add_frame_index([(t, ((1, "person"), (2, "person"), (3, "car"))),
                            (t + 250, ((1, "person"), (2, "person"), (3, "car"))),
                            (t + 60_000, ((4, "person"),))])
    scenes = widget.run_scene_search("person", ">=", 2)
    assert len(scenes) == 1 and len(widget._scene_list.get_children()) == 1
    assert "1 scene(s)" in widget._search_status.get()
    widget._scene_list.selection_set("s0")
    widget._on_scene_select(None)
    assert sorted(r[0] for r in _rows(widget)) == ["1", "2"]
    assert "Scene" in widget._search_status.get()
    widget.clear_scene()
    assert len(_rows(widget)) == 4


def test_frame_index_last_seen_and_time_window(widget):
    """UT-DETMAP-008"""
    now = int(__import__("time").time() * 1000)
    widget.add_records([_rec(1, 0.8, cls="person", t_ms=now - 20 * 60_000),
                        _rec(2, 0.8, cls="person", t_ms=now - 20 * 60_000)])
    widget.add_frame_index([(now - 30_000, ((1, "person"),))])     # object 1 still in view 30 s ago
    widget._flt_window.set("last 5 min")
    assert [r[0] for r in _rows(widget)] == ["1"]
    widget._flt_window.set("all time")
    assert len(_rows(widget)) == 2


def test_worker_pulls_frame_index():
    """UT-DETMAP-009"""
    class Entry:
        def __init__(self, ts, ids, classes):
            self.timestamp_ms, self.track_ids, self.class_names = ts, ids, classes

    class Link:
        def __init__(self):
            self.frames = [Entry(10, [1], ["person"]), Entry(20, [1, 2], ["person", "car"])]
            self.asked = []
        def get_detection_count(self): return 0
        def get_last_pass_duration_ms(self): return 0.0
        def get_last_pass_timestamp_ms(self): return 0
        def get_records_since(self, _id): return []
        def get_frame_index_since(self, ms):
            self.asked.append(ms)
            return [e for e in self.frames if e.timestamp_ms > ms]

    link = Link()
    w = DetectionWorker(link, poll_hz=50)
    w.start()
    __import__("time").sleep(0.15)
    w.stop()
    assert w.frame_index_supported
    assert w.get_new_frame_index() == [(10, ((1, "person"),)), (20, ((1, "person"), (2, "car")))]
    assert w.get_new_frame_index() == []
    assert max(link.asked) == 20                     # incremental after the first pull

    class OldLink:                                   # DroneBackend built before the frame index
        def get_detection_count(self): return 0
        def get_last_pass_duration_ms(self): return 0.0
        def get_last_pass_timestamp_ms(self): return 0
        def get_records_since(self, _id): return []
    w = DetectionWorker(OldLink(), poll_hz=50)
    w.start(); __import__("time").sleep(0.05); w.stop()
    assert not w.frame_index_supported and w.get_new_frame_index() == []
