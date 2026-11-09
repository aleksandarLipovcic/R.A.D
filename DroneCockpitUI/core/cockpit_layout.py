"""
cockpit_layout.py -- responsive cockpit layout (pure geometry, no Tk)

In COCKPIT mode the pilot window is not a free workspace: the panels are
placed by the size of the window, like the formats of a glass cockpit.
Bigger window = more instruments; smaller window = only what the pilot
needs to fly, and below the smallest format the window cannot shrink.

Priority (what stays when space runs out), from the essential down:

  1  fpv         live video -- the pilot's eyes
  1  adi         attitude
  1  baro        altitude / vertical speed tape (right of the ADI, as the
                 altimeter sits right of the ADI in the "basic T")
  1  fc_status   armed / flight mode / battery / link
  2  mag         heading (below the attitude block, as the HSI sits below
                 the ADI in the basic T)
  3  gps         navigation / map
  4  arming      pre-flight checklist
  4  imu         raw sensor detail

Formats (workspace = window minus the toolbar):

  S   essential   fpv | adi alt        fc_status strip along the bottom
  M   + heading   fpv | adi alt / hdg  fc_status strip
  L   + nav       fpv | adi alt / hdg  fc_status | gps strip
  XL  + systems   arming/imu column | L

The geometry is a split tree: ("h" | "v", [(child, min_px, weight), ...]).
A child is a panel name or another split. Panels the user hid in the
Panels menu are dropped from the tree and their neighbours take the space,
so the screen never has holes.
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Tuple

Rect = Tuple[int, int, int, int]          # x, y, w, h

# Smallest size (incl. the 20 px title bar) at which each instrument still
# shows its values without clipping. Measured on the widgets at 100 % DPI.
MIN_SIZE: Dict[str, Tuple[int, int]] = {
    "fpv": (360, 220),
    "adi": (250, 240),
    "baro": (215, 240),
    "fc_status": (380, 168),
    "mag": (300, 170),
    "gps": (420, 200),
    "arming": (330, 260),
    "imu": (330, 220),
}

# Comfortable size: a format is only chosen when every instrument in it gets
# at least this much. So a growing window first makes the essential
# instruments (above all the video) comfortable, and only then adds more.
PREF_SIZE: Dict[str, Tuple[int, int]] = {
    "fpv": (640, 400),
    "adi": (300, 280),
    "baro": (210, 300),
    "fc_status": (440, 190),
    "mag": (380, 190),
    "gps": (520, 230),
    "arming": (380, 300),
    "imu": (380, 260),
}

GAP = 4                                     # space between panels (px)


def _min(node, axis: str, table=None) -> int:
    """Minimum extent of a node along axis ("w" or "h")."""
    table = table or MIN_SIZE
    if isinstance(node, str):
        return table.get(node, (120, 80))[0 if axis == "w" else 1]
    kind, kids = node
    sizes = [max(m, _min(c, axis, table)) if ((kind == "h") == (axis == "w"))
             else _min(c, axis, table)
             for c, m, _ in kids]
    if not kids:
        return 0
    if (kind == "h") == (axis == "w"):        # along the split: sum
        return sum(sizes) + GAP * (len(kids) - 1)
    return max(sizes)                         # across the split: largest


def min_size(tree, table=None) -> Tuple[int, int]:
    return _min(tree, "w", table), _min(tree, "h", table)


def prune(node, visible: Iterable[str]):
    """Remove hidden panels; collapse empty / single-child splits."""
    vis = set(visible)
    if isinstance(node, str):
        return node if node in vis else None
    kind, kids = node
    out = []
    for c, m, w in kids:
        pc = prune(c, vis)
        if pc is not None:
            out.append((pc, m, w))
    if not out:
        return None
    if len(out) == 1:
        return out[0][0]
    return (kind, out)


def place(node, rect: Rect, out: Optional[Dict[str, Rect]] = None) -> Dict[str, Rect]:
    """Split tree -> {panel: (x, y, w, h)} inside rect."""
    out = {} if out is None else out
    if node is None:
        return out
    x, y, w, h = rect
    if isinstance(node, str):
        out[node] = (int(x), int(y), int(max(1, w)), int(max(1, h)))
        return out
    kind, kids = node
    along = w if kind == "h" else h
    axis = "w" if kind == "h" else "h"
    free = along - GAP * (len(kids) - 1)
    mins = [max(m, _min(c, axis)) for c, m, _ in kids]
    weights = [max(0.0, wt) for _, _, wt in kids]
    extra = free - sum(mins)
    tw = sum(weights) or 1.0
    sizes = [mn + (extra * wt / tw if extra > 0 else 0) for mn, wt in zip(mins, weights)]
    if extra < 0:                               # below the minimum: shrink proportionally
        k = free / max(1, sum(mins))
        sizes = [mn * k for mn in mins]
    pos = x if kind == "h" else y
    for (child, _, _), size in zip(kids, sizes):
        r = (pos, y, size, h) if kind == "h" else (x, pos, w, size)
        place(child, r, out)
        pos += size + GAP
    return out


# ── formats ─────────────────────────────────────────────────────────────────
def _attitude_block(with_heading: bool):
    """ADI with the altitude tape on its right; heading below (basic T)."""
    adi_alt = ("h", [("adi", 0, 3.0), ("baro", 0, 1.0)])
    if not with_heading:
        return adi_alt
    return ("v", [(adi_alt, 0, 2.2), ("mag", 0, 1.0)])


def format_tree(fmt: str):
    if fmt == "S":
        top = ("h", [("fpv", 0, 1.6), (_attitude_block(False), 0, 1.0)])
        return ("v", [(top, 0, 4.0), ("fc_status", 0, 0.0)])
    if fmt == "M":
        top = ("h", [("fpv", 0, 1.7), (_attitude_block(True), 0, 1.0)])
        return ("v", [(top, 0, 4.0), ("fc_status", 0, 0.0)])
    if fmt == "L":
        top = ("h", [("fpv", 0, 1.8), (_attitude_block(True), 0, 1.0)])
        bottom = ("h", [("fc_status", 0, 1.0), ("gps", 0, 1.6)])
        return ("v", [(top, 0, 2.6), (bottom, 0, 1.0)])
    # XL: systems column on the left
    left = ("v", [("arming", 0, 1.2), ("imu", 0, 1.0)])
    return ("h", [(left, 0, 1.0), (format_tree("L"), 0, 3.6)])


FORMATS = ("XL", "L", "M", "S")             # largest first
FORMAT_NAMES = {"S": "ESSENTIAL", "M": "STANDARD", "L": "EXTENDED", "XL": "FULL"}

# Hysteresis keeps the format from flipping back and forth around a
# boundary while the window is being resized.
HYSTERESIS_PX = 24


def choose_format(ws_w: int, ws_h: int, visible: Iterable[str],
                  current: Optional[str] = None) -> str:
    vis = list(visible)
    for fmt in FORMATS:
        tree = prune(format_tree(fmt), vis)
        if tree is None:
            continue
        mw, mh = min_size(tree, PREF_SIZE)
        slack = HYSTERESIS_PX if fmt == current else 0
        if ws_w + slack >= mw and ws_h + slack >= mh:
            return fmt
    return "S"


def layout(ws_w: int, ws_h: int, visible: Iterable[str],
           current: Optional[str] = None, margin: int = 4) -> Tuple[str, Dict[str, Rect], List[str]]:
    """-> (format, {panel: rect}, panels hidden by the format)."""
    vis = list(visible)
    fmt = choose_format(ws_w, ws_h, vis, current)
    tree = prune(format_tree(fmt), vis)
    rects = place(tree, (margin, margin, ws_w - 2 * margin, ws_h - 2 * margin)) if tree else {}
    hidden = [p for p in vis if p not in rects]
    return fmt, rects, hidden


def minimum_workspace(visible: Iterable[str] = ("fpv", "adi", "baro", "fc_status"),
                      margin: int = 4) -> Tuple[int, int]:
    """Smallest workspace the essential format needs (window minsize)."""
    tree = prune(format_tree("S"), visible)
    if tree is None:
        return 400, 300
    w, h = min_size(tree)
    return w + 2 * margin, h + 2 * margin
