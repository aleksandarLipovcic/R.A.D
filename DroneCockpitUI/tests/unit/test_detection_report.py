# =============================================================================
# test_detection_report.py
#
# Unit tests for the self-contained detection reports
# (core/detection_report.py) and their hook-up:
#   UT-DETREP-001  Map zoom fits the object, its ± radius and the dron;
#                  closer together -> higher zoom
#   UT-DETREP-002  Coordinate formats: DMS and MGRS against known values
#   UT-DETREP-003  One record -> collage JPEG (with EXIF GPS of the object),
#                  JSON sidecar and an index.csv row, under <dir>/<date>/
#   UT-DETREP-004  No position / invalid telemetry / missing frame still
#                  gives a report, with nulls instead of fake zeros
#   UT-DETREP-005  Offline: a failed tile download does not stall the queue
#                  (cache-only back-off), the map says tiles are missing
#   UT-DETREP-006  DetectionWorker hands every new record to the writer
#   UT-DETREP-007  Detections window: "Open report" finds the collage of
#                  the selected object's best sighting
#
#   xvfb-run -a python -m pytest tests/unit/test_detection_report.py -v
# =============================================================================

import csv
import json
import os
import time
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.filterwarnings(
    "ignore::pytest.PytestUnraisableExceptionWarning"
)

from PIL import Image

from core import MapTiles
from core import detection_report as dr

T_MS = 1_793_885_805_797          # 2026-11-05 13:36:45.797 UTC


def _tel(valid=True, lat=44.7671, lon=17.1881):
    return SimpleNamespace(valid=valid, latitude=lat, longitude=lon, altitude_m=42.3,
                           heading_deg=235.0, roll_deg=1.2, pitch_deg=-3.4,
                           gimbal_pan_deg=0.0, gimbal_tilt_deg=55.0, ground_speed_ms=4.2,
                           gps_fix_type=2, gps_num_sat=14, battery_voltage=15.6, armed=True)


def _rec(frame_path, rec_id=3, geo=True, tel=None, lat=44.7665, lon=17.1870, radius=18.0):
    return SimpleNamespace(
        id=rec_id, track_id=12, class_name="car", confidence=0.873, best_confidence=0.91,
        sightings=4, timestamp_ms=T_MS, georeferenced=geo,
        latitude=lat if geo else 0.0, longitude=lon if geo else 0.0,
        uncertainty_m=radius if geo else 0.0, distance_m=95.0 if geo else 0.0,
        bearing_deg=232.0 if geo else 0.0, range_method="ground_plane" if geo else "",
        bbox_x=300, bbox_y=200, bbox_w=70, bbox_h=45,
        screenshot_path=frame_path, telemetry=tel if tel is not None else _tel())


@pytest.fixture
def frame(tmp_path):
    p = tmp_path / "det_20261105_133645_3_0.jpg"
    Image.new("RGB", (720, 480), (90, 110, 90)).save(p)
    return str(p)


@pytest.fixture
def tiles(tmp_path, monkeypatch):
    """Tile cache in tmp, every tile present as a flat image, no network."""
    root = tmp_path / "tiles"
    monkeypatch.setattr(MapTiles, "CACHE_ROOT", str(root))

    def fake_download(provider, z, x, y, timeout=6.0):
        p = MapTiles.TileCache.tile_path(provider, z, x, y)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        Image.new("RGB", (256, 256), (40, 80, 40)).save(p)
        return True

    monkeypatch.setattr(MapTiles, "download_tile", fake_download)
    return root


# ── UT-DETREP-001 ────────────────────────────────────────────────────────────

def test_zoom_fits_object_radius_and_dron():
    obj, drone = (44.7665, 17.1870), (44.7671, 17.1881)       # ~110 m apart
    z = dr.choose_zoom([obj, drone], 18.0, dr.MAP_W, dr.MAP_H)
    assert dr.MIN_ZOOM <= z <= dr.MAX_ZOOM
    for la, lo in (obj, drone):                               # both inside the panel
        x, y = dr.mercator_pixel(la, lo, z)
        cx, cy = dr.mercator_pixel((obj[0] + drone[0]) / 2, (obj[1] + drone[1]) / 2, z)
        assert abs(x - cx) < dr.MAP_W / 2 and abs(y - cy) < dr.MAP_H / 2
    far = dr.choose_zoom([obj, (44.80, 17.25)], 18.0, dr.MAP_W, dr.MAP_H)
    big_radius = dr.choose_zoom([obj, drone], 400.0, dr.MAP_W, dr.MAP_H)
    assert far < z and big_radius < z


# ── UT-DETREP-002 ────────────────────────────────────────────────────────────

def test_coordinate_formats():
    assert dr.format_dms(44.76652938, True) == "44°45'59.51\" N"
    assert dr.format_dms(17.18701172, False) == "17°11'13.24\" E"
    assert dr.format_dms(-0.5, True).endswith(" S")
    # Banja Luka; checked against the 'mgrs' reference library.
    assert dr.to_mgrs(44.7722, 17.1910) == "33T XK 73367 59980"
    assert dr.to_mgrs(85.0, 0.0) == "—"                       # polar: not supported


# ── UT-DETREP-003 ────────────────────────────────────────────────────────────

def test_report_files_and_contents(tmp_path, frame, tiles):
    out = tmp_path / "reports"
    w = dr.DetectionReportWriter(str(out))
    jpg = w.write_report(_rec(frame))

    jpg_expected, json_path = dr.report_paths(_rec(frame), str(out))
    assert jpg == jpg_expected and os.path.exists(jpg) and os.path.exists(json_path)
    assert os.path.basename(os.path.dirname(jpg)) == time.strftime("%Y-%m-%d", time.localtime(T_MS / 1000))
    assert os.path.basename(jpg).endswith("_d3_T12_car.jpg")

    img = Image.open(jpg)
    assert img.size == (dr.COLLAGE_W, dr.COLLAGE_H)
    gps = img.getexif().get_ifd(0x8825)
    lat = sum(float(v) / 60 ** i for i, v in enumerate(gps[2]))
    lon = sum(float(v) / 60 ** i for i, v in enumerate(gps[4]))
    assert gps[1] == "N" and gps[3] == "E"
    assert abs(lat - 44.7665) < 1e-5 and abs(lon - 17.1870) < 1e-5

    d = json.load(open(json_path, encoding="utf-8"))
    assert d["class_name"] == "car" and d["class_name_sr"] == "automobil"
    assert d["confidence"] == pytest.approx(0.873)
    assert d["object"]["latitude"] == pytest.approx(44.7665)
    assert d["object"]["uncertainty_m"] == 18.0
    assert d["object"]["mgrs"].startswith("33T XK")
    assert d["drone"]["altitude_agl_m"] == pytest.approx(42.3)
    assert d["time"]["utc"] == "2026-11-05T13:36:45.797Z"
    assert d["source_frame"] == frame and d["report_image"] == os.path.basename(jpg)
    assert d["map"]["missing_tiles"] == 0

    rows = list(csv.reader(open(os.path.join(os.path.dirname(jpg), "index.csv"),
                                encoding="utf-8-sig"), delimiter=";"))
    assert rows[0] == dr.CSV_COLUMNS and len(rows) == 2
    assert rows[1][dr.CSV_COLUMNS.index("klasa")] == "car"
    assert rows[1][-1] == os.path.basename(jpg)

    w.write_report(_rec(frame, rec_id=4))                     # second row, one header
    rows = list(csv.reader(open(os.path.join(os.path.dirname(jpg), "index.csv"),
                                encoding="utf-8-sig"), delimiter=";"))
    assert len(rows) == 3


# ── UT-DETREP-004 ────────────────────────────────────────────────────────────

def test_report_without_position_or_telemetry(tmp_path, tiles):
    w = dr.DetectionReportWriter(str(tmp_path))
    rec = _rec("does_not_exist.jpg", geo=False, tel=SimpleNamespace(valid=False, latitude=0.0, longitude=0.0))
    jpg = w.write_report(rec)
    d = json.load(open(jpg[:-4] + ".json", encoding="utf-8"))
    assert d["object"]["georeferenced"] is False
    assert d["object"]["latitude"] is None and d["object"]["mgrs"] is None
    assert d["drone"]["telemetry_valid"] is False and d["drone"]["latitude"] is None
    assert d["map"]["zoom"] is None
    assert 0x8825 not in Image.open(jpg).getexif()            # no fake GPS position

    # Object without position but dron fix: map centred on the dron, EXIF = dron.
    jpg2 = w.write_report(_rec("does_not_exist.jpg", rec_id=5, geo=False))
    d2 = json.load(open(jpg2[:-4] + ".json", encoding="utf-8"))
    assert d2["map"]["zoom"] == dr.DEFAULT_ZOOM_NO_OBJECT
    assert Image.open(jpg2).getexif().get_ifd(0x8825)


# ── UT-DETREP-005 ────────────────────────────────────────────────────────────

def test_offline_does_not_stall(tmp_path, frame, monkeypatch):
    monkeypatch.setattr(MapTiles, "CACHE_ROOT", str(tmp_path / "empty"))
    calls = []

    def failing_download(provider, z, x, y, timeout=6.0):
        calls.append((z, x, y))
        time.sleep(0.05)
        return False

    monkeypatch.setattr(MapTiles, "download_tile", failing_download)
    w = dr.DetectionReportWriter(str(tmp_path / "out"))
    w.start()
    w.submit([_rec(frame, rec_id=i) for i in range(1, 6)])
    t0 = time.monotonic()
    w.stop(flush_timeout_s=10)
    assert w.written == 5 and w.failed == 0
    assert len(calls) == 1                                    # back-off after the first failure
    assert time.monotonic() - t0 < 5
    d = json.load(open(dr.report_paths(_rec(frame, rec_id=1), str(tmp_path / "out"))[1], encoding="utf-8"))
    assert d["map"]["missing_tiles"] > 0


# ── UT-DETREP-006 ────────────────────────────────────────────────────────────

def test_worker_forwards_records_to_writer():
    from workers.detection_worker import DetectionWorker

    recs = [SimpleNamespace(id=1), SimpleNamespace(id=2)]

    class Link:
        served = False

        def get_detection_count(self):
            return 2

        def get_last_pass_duration_ms(self):
            return 10.0

        def get_last_pass_timestamp_ms(self):
            return 0

        def get_records_since(self, last_id):
            if Link.served:
                return []
            Link.served = True
            return recs

    class Writer:
        got = []

        def submit(self, records):
            Writer.got.extend(records)

    w = DetectionWorker(Link(), poll_hz=50, report_writer=Writer())
    w.start()
    time.sleep(0.15)
    w.stop()
    assert [r.id for r in Writer.got] == [1, 2]
    assert [r.id for r in w.get_new_records()] == [1, 2]     # window still gets them too


# ── UT-DETREP-007 ────────────────────────────────────────────────────────────

def test_window_open_report(tk_root, tmp_path, frame, tiles, monkeypatch):
    from widgets.DetectionMapWidget import DetectionMapWidget

    win = DetectionMapWidget(tk_root)
    try:
        win._load_preview = lambda path: None
        rec = _rec(frame)
        win.add_records([rec])
        assert win.report_path_for(rec) is None               # no report dir yet

        win.set_report_dir(str(tmp_path / "reports"))
        assert win.report_path_for(rec) is None               # not written yet
        jpg = dr.DetectionReportWriter(str(tmp_path / "reports")).write_report(rec)
        assert win.report_path_for(rec) == jpg

        win._tree.selection_set(win._tree.get_children()[0])
        win._on_select(None)
        assert win._open_report_btn_visible and win._report_rec is rec
        import widgets.DetectionMapWidget as mod
        opened = []
        monkeypatch.setattr(mod.sys, "platform", "win32")
        monkeypatch.setattr(mod.os, "startfile", opened.append, raising=False)
        win._open_report()
        assert opened == [jpg]

        win._return_to_live()
        assert not win._open_report_btn_visible and win._report_rec is None
    finally:
        win.destroy()
