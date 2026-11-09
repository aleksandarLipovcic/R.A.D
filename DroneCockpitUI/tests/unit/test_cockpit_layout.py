"""
test_cockpit_layout.py -- responsive cockpit formats (core/cockpit_layout.py)
and the color theme (core/theme.py), without Tk.

    pytest tests/unit/test_cockpit_layout.py        (from DroneCockpitUI/)
"""
import colorsys
import itertools
import os
import sys

_APP = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _APP not in sys.path:
    sys.path.insert(0, _APP)
from core import cockpit_layout as L        # noqa: E402
from core import theme                      # noqa: E402

ALL = list(L.MIN_SIZE)
ESSENTIAL = {"fpv", "adi", "baro", "fc_status"}


def _overlap(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return ax < bx + bw and bx < ax + aw and ay < by + bh and by < ay + ah


def _check_rects(rects, ws_w, ws_h):
    for (na, a), (nb, b) in itertools.combinations(rects.items(), 2):
        assert not _overlap(a, b), f"{na} {a} overlaps {nb} {b}"
    for n, (x, y, w, h) in rects.items():
        assert x >= 0 and y >= 0 and x + w <= ws_w + 1 and y + h <= ws_h + 1, (n, ws_w, ws_h)


def test_formats_grow_with_the_window():
    order = []
    for w, h in [(800, 380), (1000, 560), (1366, 690), (1500, 820), (1900, 1000), (2500, 1350)]:
        fmt, rects, hidden = L.layout(w, h, ALL)
        order.append(L.FORMATS.index(fmt))
        _check_rects(rects, w, h)
        assert ESSENTIAL <= set(rects), f"essential instruments always shown ({fmt})"
    assert order == sorted(order, reverse=True), "bigger window never gets a smaller format"
    assert L.layout(800, 380, ALL)[0] == "S"
    assert L.layout(1900, 1000, ALL)[0] == "XL"


def test_every_shown_panel_gets_its_minimum():
    for w, h in [(L.minimum_workspace()[0], L.minimum_workspace()[1]), (1100, 640), (1600, 900)]:
        fmt, rects, _ = L.layout(w, h, ALL)
        for n, (_, _, pw, ph) in rects.items():
            mw, mh = L.MIN_SIZE[n]
            assert pw >= mw - 1 and ph >= mh - 1, f"{fmt} {n} {pw}x{ph} < {mw}x{mh}"


def test_priority_order_of_dropping():
    _, _, hidden = L.layout(1000, 560, ALL)
    assert set(hidden) == {"mag", "gps", "arming", "imu"}
    mw, mh = L.min_size(L.format_tree("M"), L.PREF_SIZE)
    fmt, _, hidden = L.layout(mw + 10, mh + 10, ALL)
    assert fmt == "M" and "mag" not in hidden and "gps" in hidden


def test_hidden_panels_leave_no_hole():
    wanted = [p for p in ALL if p != "baro"]
    fmt, rects, _ = L.layout(1400, 760, wanted)
    assert "baro" not in rects
    adi = rects["adi"]
    right = max(x + w for x, y, w, h in rects.values())
    assert adi[0] + adi[2] >= right - 2, "the ADI takes the altitude tape's room"


def test_minimum_window_and_hysteresis():
    mw, mh = L.minimum_workspace()
    assert 700 <= mw <= 900 and 350 <= mh <= 450
    pw, ph = L.min_size(L.format_tree("M"), L.PREF_SIZE)
    assert L.choose_format(pw, ph, ALL) == "M"
    assert L.choose_format(pw - 10, ph - 10, ALL) == "S"
    assert L.choose_format(pw - 10, ph - 10, ALL, current="M") == "M", "hysteresis holds the format"
    assert L.choose_format(pw - 40, ph - 40, ALL, current="M") == "S"


def _hue(c):
    r, g, b = (int(c[i:i + 2], 16) / 255 for i in (1, 3, 5))
    return colorsys.rgb_to_hsv(r, g, b)


def test_theme_keeps_color_meaning():
    if not theme.MILITARY:
        assert theme.C("#00ff88") == "#00ff88"
        return
    red, amber, green, cyan = (_hue(theme.C(c)) for c in ("#ff3355", "#ffaa00", "#00ff88", "#00d4ff"))
    assert red[0] < 0.05 or red[0] > 0.95
    assert 0.08 < amber[0] < 0.16
    assert 0.3 < green[0] < 0.45
    assert 0.5 < cyan[0] < 0.6
    # backgrounds stay dark and keep their order
    a, b = theme.C("#0f0f1a"), theme.C("#1e3060")
    assert _hue(a)[2] < 0.15 and _hue(a)[2] < _hue(b)[2]
    # white stays (near) white, transparency key untouched, non-colors pass
    assert _hue(theme.C("#ffffff"))[2] > 0.9
    assert theme.C("#010203") == "#010203"
    assert theme.C("white") == "white" and theme.C("") == ""
    # ADI keeps blue sky / brown ground
    assert 0.55 < _hue(theme.C("#1565C0"))[0] < 0.65
    assert 0.05 < _hue(theme.C("#7B3F00"))[0] < 0.1


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
