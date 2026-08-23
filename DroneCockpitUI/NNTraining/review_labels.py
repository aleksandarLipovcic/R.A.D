"""
review_labels.py -- Interactive, resumable review tool for the queue
generate_pseudo_labels.py produces (datasets/pseudo_labels_review_queue.
json). Shows one image at a time with its queued boxes overlaid; click
a box to accept/reject/reclassify it, or drag on empty space to draw a
new box the queue missed entirely. Progress autosaves as you go --
close it anytime, rerun later, it picks up where you left off. Tkinter
GUI (+ Pillow for image display).

FEATURES
  Queue (model) boxes -- click opens a menu: Accept / Reject / Reset to
  Pending / Change Class / "Delete (hide from view, recoverable)".
  Click-only by default; a menu option "Enable Reposition" arms exactly
  one drag (move or resize) on that box, then re-locks. Deleted boxes
  are hidden unless "Show deleted" is checked, and can be reset back to
  pending from there.

  Manual boxes (cyan, hand-drawn) -- drag on empty canvas to draw one;
  picking its class auto-accepts it. Freely draggable and resizable (8
  handles: 4 corners + 4 edge midpoints, min size MIN_BOX_PX). Ctrl+C /
  Ctrl+V copies the selected box as a new manual box, offset by
  PASTE_OFFSET_PX; repeated Ctrl+V without a new Ctrl+C cascades
  further each time.

  Original dataset labels (magenta) -- reference only, shown by default
  (--hide-original to hide), never selectable/draggable.

  Per-model panel -- visibility checkboxes + a detection-count readout
  per model for the current image (models with zero detections shown
  in red), independent of what's currently displayed.

  Autosave -- every mutation writes to disk (debounced ~400ms; an
  immediate un-debounced flush happens on navigate, close, or window
  focus-loss). Manual "Save Now" also available.

  Undo/redo -- Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z, scoped per image (stack
  resets on navigation).

  Fast Mode -- left-click a queue box = Accept, right-click = Reject,
  no menu (toggle in View menu/toolbar).

  Bulk actions -- "Accept all pending" / "Reject all pending" for the
  current image (confirms first; one undo step reverts the whole
  batch).

  Number keys 1-9 -- apply the Nth class (CLASS_NAMES order) to the
  selected box.

  Zoom/pan -- mouse wheel or +/-/0 to zoom (0 = fit to window),
  middle-click-drag or scrollbars to pan.

  Completed list -- once every box on an image has a decision, its
  path moves to datasets/review_completed.json and it's excluded from
  normal review. --qa opens a session sourced only from that list (a
  second pass/reviewer); "Switch to QA / Completed" in the toolbar
  swaps between the two live, without restarting. Resetting any box
  back to pending drops the image out of the completed list again.

  Sidebar -- image list split into Remaining/Reviewed, grouped by
  source dataset then by frame set/sequence (color-coded), each set
  annotated with a "reviewed/total" count and a warning marker if it's
  below MIN_REVIEWED_FRAMES_PER_SEQUENCE. Click an image to jump to it.
  "Check sequence coverage" gives the same numbers as a full text
  report. Resizable pane; expand/scroll state persists across rebuilds.
  Purely a navigation aid -- never affects decisions or output.

  Cross-frame sync -- deciding a queue or manual box (accept/reject/
  delete) looks at nearby frames in the same sequence (parsed from the
  filename, e.g. "M1201_img000322_..." -> sequence M1201, frame 322)
  and applies the same decision to a same-class box at a matched
  position, within the configured window (--sync-window / --no-sync,
  toolbar checkbox). Same class only, one nearest match per neighbor
  frame, never overwrites an existing decision or reopens a completed
  image; only the decision is synced, never geometry. Uses a simple
  constant-velocity (dead-reckoning) position predictor as it walks
  outward frame-by-frame, so matches track a moving/panning object
  rather than assuming a fixed pixel spot. Synced boxes are tagged
  "[synced]" on canvas. Runs on save (not on every click), so a burst
  of edits collapses into one sync pass. Changing your mind on a
  source box's decision retracts exactly what it had synced (tracked
  per-box) before applying the new decision forward. "Undo Last Sync"
  reverts the most recent sync batch.

  Startup self-healing -- every launch backs up review_progress.json
  and review_completed.json to datasets/review_backups/<timestamp>/
  (--no-backup to skip), then reconciles saved decisions against the
  current queue: exact key match first, falling back to same-image/
  same-class nearest-box matching within a tolerance if a box's stored
  key no longer matches (e.g. the queue was regenerated with slightly
  shifted coordinates). Unresolved entries are kept under their old
  key (never deleted) and printed for manual review. Skip with
  --no-key-reconcile.

WHY A SEPARATE OUTPUT TREE (see generate_pseudo_labels.py's module
docstring for the full reasoning): this writes to datasets/
pseudo_labels_reviewed/<SOURCE>/..., NOT datasets/pseudo_labels/ (the
auto-accept tier). generate_pseudo_labels.py overwrites pseudo_labels/
wholesale on every rerun -- if this tool wrote there too, a later rerun
would silently erase your manual review work. --apply merges BOTH
trees into the real dataset, so this separation costs nothing at apply
time.

SCOPE: only images that have at least one queue entry are shown -- the
much larger auto-accepted tier is never touched here. Use --source /
--cls to focus a session -- this is built to be resumable across many
sessions, not a one-sitting task.

USAGE:
    python review_labels.py                # everything pending
    python review_labels.py --source UAVDT --cls person
    python review_labels.py --show-all      # revisit already-decided
                                             # images too (normally
                                             # hidden once complete)
    python review_labels.py --qa            # QA pass: only images a
                                             # first reviewer already
                                             # fully decided (also
                                             # reachable live from the
                                             # toolbar)
    python review_labels.py --hide-original # don't overlay existing
                                             # real-dataset labels
    python review_labels.py --no-sync       # start with cross-frame
                                             # sync OFF
    python review_labels.py --sync-window 5 # sync up to 5 frames out
                                             # in each direction
    python review_labels.py --no-backup         # skip the automatic
                                                 # per-launch backup
                                                 # (not recommended)
    python review_labels.py --no-key-reconcile  # skip the automatic
                                                 # startup key-repair
                                                 # pass (debugging only)

HISTORY (one line per release -- see git log for full detail):
  V5  -- rewrite: OpenCV/keyboard UI -> Tkinter GUI.
  V6  -- fixed item_key() using the live (not original) box; added
         zoom/pan and box-position persistence.
  V7  -- locked down dragging to manual boxes only (queue boxes need
         explicit "Enable Reposition"); added per-model detection
         counts, real autosave, separate completed list, original
         labels shown by default.
  V8  -- added recoverable delete, full per-image undo/redo, Fast
         Mode, bulk accept/reject, number-key class shortcuts,
         auto-advance, streak counter.
  V9  -- added resize handles for manual/armed boxes, resize-aware
         cursors, Escape cancels drag, faster autosave + focus-loss
         flush.
  V10 -- added cross-frame decision sync (same class, position
         tolerance, never overwrites) with per-session on/off + window
         controls.
  V11 -- fixed a real incident (decisions "disappearing" after a queue
         regen): manual boxes now go through pending->decided like
         queue boxes; added startup reconciliation, automatic
         per-launch backups, and sync coverage for manual boxes.
  V12 -- added live "Switch to QA / Completed"; fixed toolbar/status
         bar layout getting pushed off-window; full model names in UI;
         manual boxes auto-accept on draw again.
  V13 -- correctness pass: completion checks now use the full
         unfiltered queue, reversing a synced decision now retracts
         what it synced, reposition-arm no longer disarmed by a
         non-drag click, sync now respects class overrides, sync
         writes force-flush immediately.
  V14 -- added Ctrl+C/Ctrl+V box copy/paste with cascading offset.
  V15 -- fixed Delete/BackSpace not working while a box's context menu
         was open.
  V16 -- fixed sequence/frame parsing for filenames without a leading
         underscore (was effectively randomizing sync neighbors);
         added motion-predicted (constant-velocity) cross-frame
         matching. NOTE: sync entries written before this fix may be
         mismatched -- verify synced_from against the corrected regex
         before trusting old data.
  V17 -- fixed a Spinbox crash that could silently break sync for the
         rest of a session; sync now runs on save instead of on every
         click.
  V18 -- added the Remaining/Reviewed sidebar and "Check sequence
         coverage" report (navigation only, no effect on decisions).
  V19 -- sidebar now nests by frame set/sequence with per-set color
         and inline reviewed/total counts + low-coverage warning.
  V20 -- fixed the sidebar auto-jumping to Reviewed the moment an
         image was finished during normal review.
  V21 -- sidebar is now a resizable pane instead of a fixed width.
  V22 -- fixed sidebar expand/scroll state resetting on every rebuild.
"""

import argparse
import colorsys
import copy
import hashlib
import json
import re
import shutil
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
from PIL import Image, ImageTk

import prepare_datasets
from class_map import UNIFIED_CLASSES
import generate_pseudo_labels as gpl

REVIEWED_DIRNAME = "pseudo_labels_reviewed"
PROGRESS_FILENAME = "review_progress.json"
COMPLETED_FILENAME = "review_completed.json"
BACKUP_DIRNAME = "review_backups"

# Must match the keys cross_reference_gaps.py writes into each queue
# item's "votes" dict.
MODEL_KEYS = ["yolo_visdrone", "rfdetr_visdrone", "yolo_coco", "dino"]

# (V12) Full, readable names shown everywhere in the UI (toolbar,
# per-image detection counts, sync console log) instead of the old
# two-letter codes (yv/rf/yc/dn). MODEL_KEYS itself is unchanged since
# it has to keep matching the "votes" keys cross_reference_gaps.py
# writes -- only the display strings changed.
MODEL_SHORT = {"yolo_visdrone": "YOLO-VisDrone", "rfdetr_visdrone": "RF-DETR",
               "yolo_coco": "YOLO-COCO", "dino": "DINO"}

COLOR_PENDING = "#ffa500"    # orange
COLOR_ACCEPT = "#00c800"     # green
COLOR_REJECT = "#dc0000"     # red
COLOR_DELETED = "#777777"    # grey, dashed -- hidden by default, recoverable
COLOR_MANUAL = "#00c8ff"     # cyan-ish -- accepted manual boxes
COLOR_ORIGINAL = "#ff00ff"   # magenta, existing real-dataset labels (reference only)
COLOR_ARMED = "#ffff00"      # yellow, queue box temporarily armed for one reposition
COLOR_HANDLE = "#ffffff"     # resize-handle squares
COLOR_HANDLE_OUTLINE = "#000000"

# (V18) Light background stripe per source dataset, used ONLY by the
# sidebar image list (see _build_sidebar_tree) to make the mix of sets
# in a filtered session visually obvious at a glance. Purely cosmetic --
# has no effect on review logic, output, or which images are shown.
# Extend this if/when another source dataset gets folded into the
# combined set; anything not listed here falls back to SOURCE_COLOR_OTHER.
SOURCE_COLORS = {
    "SARD": "#dce8ff",     # light blue
    "UAVDT": "#dff5df",    # light green
    "VisDrone": "#fff3d6", # light amber
}
SOURCE_COLOR_OTHER = "#ececec"  # light grey, any source not listed above

# (V18) Threshold used by the "Check sequence coverage" report AND
# (V19) the inline per-set sidebar annotation -- flags a source/
# sequence combination as needing more manual attention if fewer than
# this many of its frames are in the Reviewed/completed list yet.
# Purely informational, changes nothing on disk.
MIN_REVIEWED_FRAMES_PER_SEQUENCE = 3

DISPLAY_MAX_W = 1280
DISPLAY_MAX_H = 860

# (V12) Rough vertical/horizontal space reserved for everything that
# ISN'T the image canvas -- menubar, 3 toolbar rows, the detection-
# count row, the status bar, and the nav row -- used to size the
# canvas so the bottom nav bar can never be pushed off-screen. This is
# intentionally generous; _load_image() also re-measures the real
# chrome height once the window exists and uses whichever is bigger.
CHROME_RESERVED_H = 300
CHROME_RESERVED_W = 60
MIN_CANVAS_W = 480
MIN_CANVAS_H = 320

# (V18) Initial width of the left-hand image-list sidebar, in pixels.
# (V21) No longer a hard fixed width -- this is now just the STARTING
# width of a draggable pane; see SIDEBAR_MIN_WIDTH_PX / MIN_CANVAS_W
# and _build_ui's PanedWindow setup below.
SIDEBAR_WIDTH_PX = 270
# (V21) Floor on how narrow the sidebar pane can be dragged -- keeps
# the resize handle from collapsing it to nothing/unreadable. The
# image canvas has its own floor, MIN_CANVAS_W below, so the sash can
# never be dragged far enough to make one pane cover or squeeze out
# the other -- tk.PanedWindow physically cannot overlap panes; each
# pane's minsize is what stops the drag before it gets uncomfortably
# tight, not any overlap risk.
SIDEBAR_MIN_WIDTH_PX = 160

MIN_ZOOM = 1.0     # can't zoom out past "fit to window"
MAX_ZOOM = 8.0
ZOOM_STEP = 1.15

AUTOSAVE_DEBOUNCE_MS = 400
AUTO_ADVANCE_DELAY_MS = 400
UNDO_LIMIT = 50

# Resize handles: side length of each square handle in screen (canvas)
# pixels, and how many extra pixels around a handle still count as a
# hit (so you don't have to pixel-hunt to grab one).
HANDLE_SIZE = 7
HANDLE_HIT_PAD = 5

# How far (in screen/canvas pixels) a box's popup menu is offset from
# the box's own edge, so the menu opens BESIDE the box instead of
# directly on top of it. Previously the menu popped up exactly at the
# click point, which for a small box could cover the box (and its
# resize handles) entirely -- making it impossible to then grab a
# handle to resize/move it without first closing the menu and hunting
# for the box underneath. See _menu_pos_clear_of_box().
MENU_OFFSET_PX = 18
MENU_EST_WIDTH_PX = 190  # rough width to check we're not offsetting off-screen

# How far (in ORIGINAL IMAGE pixels) each successive Ctrl+V paste is
# offset, diagonally, from the copied box's original position. Without
# this every paste would land in the exact same spot and stack
# invisibly on top of the previous one/the original -- offsetting means
# repeated Ctrl+V visibly cascades new copies across the image so you
# can see each one to drag into place. Resets to 0 on every fresh
# Ctrl+C (see _copy_selected).
PASTE_OFFSET_PX = 15

# Smallest a box (manual, or a queue box mid-resize) is allowed to
# shrink to, in ORIGINAL IMAGE pixels (not screen/zoom pixels) -- keeps
# a resize drag from collapsing a box to zero/negative width or height.
MIN_BOX_PX = 4.0

CLASS_NAMES = list(UNIFIED_CLASSES)

# Decision states a queue box can be in. "deleted" behaves like "reject"
# for output purposes (see accepted_items()) but is hidden from the
# canvas unless the "Show deleted" checkbox is on.
DECISION_STATES = ("pending", "accept", "reject", "deleted")

# Decision states a MANUAL (hand-drawn) box can be in (V11). Mirrors
# DECISION_STATES in spirit: a manual box is accepted immediately when
# drawn (V12), but can still be moved to "rejected" or back to
# "pending" from its menu if you drew it by mistake.
MANUAL_DECISION_STATES = ("pending", "accepted", "rejected")

# Handle names, and which x/y edges of the box each one controls.
# "x": -1 means it moves x1 (left edge), +1 means it moves x2 (right
# edge), 0 means it doesn't touch x at all. Same for "y".
HANDLES = [
    ("nw", -1, -1), ("n", 0, -1), ("ne", 1, -1),
    ("w", -1, 0),                 ("e", 1, 0),
    ("sw", -1, 1),  ("s", 0, 1),  ("se", 1, 1),
]
# Cursor to show for each handle (standard Tk cursor names).
HANDLE_CURSOR = {
    "nw": "size_nw_se", "se": "size_nw_se",
    "ne": "size_ne_sw", "sw": "size_ne_sw",
    "n": "sb_v_double_arrow", "s": "sb_v_double_arrow",
    "e": "sb_h_double_arrow", "w": "sb_h_double_arrow",
}

# --- Cross-frame sync (V10 / V11 / V16) --------------------------------
# Filenames like "011_M1201_img000322_jpg_rf_<hash>.jpg" (Roboflow-style)
# OR, as of V16, "M1201_img000322_jpg.rf.<hash>.jpg" (this dataset's real
# layout -- no leading underscore before the sequence id) encode a
# sequence id ("M1201") and a frame number ("000322"). This is a
# heuristic over the filename ONLY -- images that don't match this
# pattern simply have no sync neighbors, which is a safe no-op.
#
# V16 FIX: the sequence id is now matched at either the START of the
# filename OR after an underscore -- (?:^|_) -- instead of requiring a
# literal leading underscore unconditionally. See the V16 module note
# above for why the old, underscore-only version silently never matched
# this dataset's actual filenames and what that broke.
SEQ_FRAME_RE = re.compile(r"(?:^|_)(?P<seq>[A-Za-z0-9]+)_img(?P<frame>\d+)_")

SYNC_DEFAULT_WINDOW = 3          # frames each direction, default
SYNC_MAX_WINDOW = 10
# How close (as a fraction of the image diagonal) a same-class box in a
# neighbor frame has to be to count as "the same object" for syncing,
# and (V11) for deciding a position in a neighbor is "already covered"
# so manual-box sync doesn't stack a duplicate on top of it.
SYNC_POS_TOLERANCE_FRAC = 0.035

# (V11) Tolerance used ONLY by the startup key-reconciliation pass, as
# a fraction of a saved box's own diagonal -- deliberately tighter than
# the cross-frame sync tolerance above, since this is meant to catch
# "same box, coordinates drifted by float noise", not "probably the
# same object a few frames later".
KEY_RECONCILE_TOL_FRAC = 0.02

FRAME_DIGITS_RE = re.compile(r"\d+")


def parse_seq_frame(image_key: str):
    """Returns (sequence_id, frame_number) for grouping/sorting frames
    within a sequence. Tries two layouts, in order:

      1. Sequence embedded in the FILENAME, e.g. "M1201_img000322_..."
         (this dataset's real layout, V16) or a Roboflow-style export
         "..._M1201_img000322_..." (leading underscore also matches).
      2. Sequence as the PARENT DIRECTORY name with the frame number
         being the last run of digits in the filename, e.g.
         ".../M1201/img000322.jpg" -> ("M1201", 322). This is the
         common UAVDT/VisDrone/SARD-style layout (one folder per
         sequence) and is tried whenever (1) doesn't match.

    Returns None if neither applies -- that image simply gets no sync
    neighbors (safe no-op), rather than silently guessing wrong.
    """
    p = Path(image_key)
    m = SEQ_FRAME_RE.search(p.name)
    if m:
        return m.group("seq"), int(m.group("frame"))
    if p.parent.name:
        digits = FRAME_DIGITS_RE.findall(p.stem)
        if digits:
            return p.parent.name, int(digits[-1])
    return None


def _color_for_sequence(seq_key: str) -> str:
    """(V19) Deterministically generates a light, distinguishable
    background color for a given frame-SET (sequence) key, so scanning
    down the sidebar makes it visually obvious where one drone pass/
    burst ends and the next begins -- separate from (and in addition
    to) the per-source-dataset stripe already used for SOURCE_COLORS
    at the section-header level. The same seq_key always produces the
    same color, across rebuilds and sessions, since it's derived from a
    stable hash rather than insertion order (insertion order isn't
    stable once images move between the Remaining and Reviewed
    sections). Kept light (high lightness, moderate saturation) so
    black text stays readable on top of it, matching the existing
    SOURCE_COLORS palette in spirit."""
    digest = hashlib.md5(seq_key.encode("utf-8")).hexdigest()
    hue = (int(digest[:8], 16) % 360) / 360.0
    r, g, b = colorsys.hls_to_rgb(hue, 0.85, 0.55)
    return "#{:02x}{:02x}{:02x}".format(int(r * 255), int(g * 255), int(b * 255))


def item_key(item: dict) -> str:
    """Stable id for a queue item -- (image, cls, rounded ORIGINAL box)
    is deterministic across runs since it's derived from item["_orig_box"],
    which is captured once right after the queue JSON loads and never
    touched again. Deliberately NOT derived from item["box"] (mutable --
    see V6 note in the module docstring for why that used to crash).

    NOTE (V11): this can still drift if generate_pseudo_labels.py is
    rerun and produces slightly different float coordinates -- that's
    exactly the bug reconcile_progress() below exists to repair on
    every launch, so a drifted key no longer means "your decision is
    gone", just "it gets re-matched at startup"."""
    box_str = ",".join(f"{v:.1f}" for v in item["_orig_box"])
    return f"{item['image']}|{item['cls']}|{box_str}"


def load_json_dict(path: Path) -> dict:
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return {}


def save_json(path: Path, data) -> None:
    with open(path, "w") as f:
        json.dump(data, f, indent=2, default=str)


def _clamp_box_to_image(box, w, h):
    """(fix) Last-resort safety net before handing a box to
    gpl.box_to_yolo_line(), which raises ValueError for anything even a
    fraction of a pixel outside (0,0)-(w,h). Boxes SHOULD already be
    clamped wherever they're created/edited (drawing, resizing, moving,
    pasting, cross-frame sync) -- see on_mouse_up's "draw" branch,
    _apply_resize, and _paste_clipboard -- but this catches anything
    that slips through anyway (e.g. a box saved by an older version of
    this script before that clamping existed, sitting in progress.json).
    Without this, one bad box makes _write_reviewed_labels() raise on
    every autosave forever: _flush() never reaches "self.dirty = False"
    when it throws, so the image stays dirty, and every later action
    that flushes (navigating, closing, losing focus...) re-raises the
    same exception -- which is what looks like the whole app freezing."""
    x1, y1, x2, y2 = box
    x1 = max(0.0, min(x1, w))
    x2 = max(0.0, min(x2, w))
    y1 = max(0.0, min(y1, h))
    y2 = max(0.0, min(y2, h))
    if x2 <= x1:
        x2 = min(w, x1 + 1.0)
        x1 = max(0.0, x2 - 1.0)
    if y2 <= y1:
        y2 = min(h, y1 + 1.0)
        y1 = max(0.0, y2 - 1.0)
    return [x1, y1, x2, y2]


def load_completed(path: Path) -> set:
    if path.exists():
        with open(path) as f:
            return set(json.load(f))
    return set()


def save_completed(path: Path, completed: set) -> None:
    save_json(path, sorted(completed))


def load_original_labels(label_path: Path, img_w: int, img_h: int) -> list[dict]:
    """Reads whatever YOLO-format label file already exists at the real
    dataset path for this image (if any) and returns boxes in the same
    {"cls": str, "box": [x1,y1,x2,y2]} pixel-coordinate shape used
    everywhere else here. Purely for reference display -- this tool
    never writes to this path, never edits these boxes, and reloads
    them fresh every time an image is opened."""
    if not label_path.exists():
        return []
    out = []
    for line in label_path.read_text().strip().splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            cls_idx = int(parts[0])
            xc, yc, bw, bh = (float(v) for v in parts[1:5])
        except ValueError:
            continue
        cls_name = (UNIFIED_CLASSES[cls_idx] if 0 <= cls_idx < len(UNIFIED_CLASSES)
                    else f"cls{cls_idx}")
        x1 = (xc - bw / 2) * img_w
        y1 = (yc - bh / 2) * img_h
        x2 = (xc + bw / 2) * img_w
        y2 = (yc + bh / 2) * img_h
        out.append({"cls": cls_name, "box": [x1, y1, x2, y2]})
    return out


# ------------------------------------------------------------------
# V11: startup safety net -- automatic backups + self-healing key
# reconciliation. These run in main(), before the Tk window opens, on
# EVERY launch (not a one-off script) so the tool can't drift back into
# the "looks empty on reload" bug after enough reruns of
# generate_pseudo_labels.py.
# ------------------------------------------------------------------

def backup_progress_files(datasets_dir: Path, progress_path: Path,
                           completed_path: Path):
    """Copies review_progress.json and review_completed.json into
    datasets/review_backups/<timestamp>/ before anything this run
    touches them. This is the actual safety net: even if key
    reconciliation below (or anything else) ever gets a match wrong,
    the exact on-disk state from the moment this session started is
    sitting right there to restore from. Returns the backup dir, or
    None if there was nothing to back up yet (first-ever run)."""
    if not progress_path.exists() and not completed_path.exists():
        return None
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = datasets_dir / BACKUP_DIRNAME / ts
    backup_dir.mkdir(parents=True, exist_ok=True)
    for p in (progress_path, completed_path):
        if p.exists():
            shutil.copy2(p, backup_dir / p.name)
    return backup_dir


def reconcile_progress(by_image_full: dict, progress: dict,
                        tol_frac: float = KEY_RECONCILE_TOL_FRAC):
    """Runs on every startup. Some decisions in `progress` may be keyed
    against box coordinates from a PREVIOUS run of
    generate_pseudo_labels.py that no longer match item_key() computed
    from the CURRENTLY loaded queue (regenerating the queue can shift
    float box coordinates slightly, changing the rounded key -- see the
    V11 module note for the full story). Rather than silently treating
    those as "pending" from then on, this looks up every saved
    box-decision entry: exact key match first, then same-image/
    same-class nearest-box-center fallback within `tol_frac` of the
    box's own diagonal. Returns (repaired_progress, stats, unresolved).

    Manual-box and class-override entries are keyed by image path only
    and are always immune to this drift -- copied over untouched.
    Nothing is ever deleted here: an entry that can't be matched is
    kept under its OLD key (so it behaves as it did before this
    feature existed) instead of being dropped.

    IMPORTANT: the fuzzy fallback matches on each entry's "orig_box"
    (the box's stable DETECTION-time position, saved alongside -- but
    separate from -- its current, possibly hand-edited "box"), against
    the CURRENT queue's "_orig_box" for each candidate. Both sides of
    that comparison are always detector output, so a rerun of
    generate_pseudo_labels.py should only ever shift them by float
    noise -- which is exactly what `tol_frac` is sized for. Matching on
    the edited "box" instead (as an earlier version of this function
    did) breaks precisely for boxes you deliberately repositioned or
    resized to correct them: that edit can legitimately move a box far
    from where it was originally detected, so comparing it to a fresh
    detection's position looks like "no match" even though it's the
    same box -- silently orphaning exactly the edits you did the most
    work on. Entries saved before this fix has no "orig_box" field; for
    those we fall back to "box" (the old behavior) since there's
    nothing better to match on."""
    box_entries = {k: v for k, v in progress.items() if not k.startswith("__")}
    meta_entries = {k: v for k, v in progress.items() if k.startswith("__")}

    repaired = dict(meta_entries)
    exact, fuzzy = 0, 0
    unresolved = []

    for old_key, entry in box_entries.items():
        image = entry.get("image")
        cls = entry.get("cls")
        # Prefer the stable detection-identity box; only legacy entries
        # (saved before "orig_box" existed) fall back to the edited box.
        ref_box = entry.get("orig_box") or entry.get("box")
        items = by_image_full.get(image, [])

        if not items:
            unresolved.append((old_key, "image no longer in current queue"))
            continue

        hit = None
        for it in items:
            if item_key(it) == old_key:
                hit = it
                break
        if hit is not None:
            repaired[item_key(hit)] = entry
            exact += 1
            continue

        if not ref_box or len(ref_box) < 4:
            unresolved.append((old_key, "no box coordinates to fuzzy-match with"))
            continue
        candidates = [it for it in items if it["cls"] == cls]
        if not candidates:
            unresolved.append((old_key, f"no '{cls}' items for this image anymore"))
            continue

        cx, cy = (ref_box[0] + ref_box[2]) / 2, (ref_box[1] + ref_box[3]) / 2
        diag = ((ref_box[2] - ref_box[0]) ** 2 + (ref_box[3] - ref_box[1]) ** 2) ** 0.5 or 1.0
        tol = max(diag * tol_frac, 3.0)
        best, best_d = None, None
        for it in candidates:
            bb = it["_orig_box"]
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            d = ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5
            if d <= tol and (best_d is None or d < best_d):
                best, best_d = it, d

        if best is not None:
            repaired[item_key(best)] = entry
            fuzzy += 1
        else:
            legacy_note = "" if entry.get("orig_box") else " (legacy entry, no orig_box saved)"
            unresolved.append((old_key, f"nearest '{cls}' box was outside tolerance{legacy_note}"))

    for old_key, _ in unresolved:
        repaired[old_key] = box_entries[old_key]

    stats = {"exact": exact, "fuzzy": fuzzy, "unresolved": len(unresolved)}
    return repaired, stats, unresolved


def compute_completed_set(by_image_full: dict, progress: dict) -> set:
    """Self-healing recomputation of the completed-images set, done
    from the (already-reconciled) progress dict + the FULL (unfiltered
    by --source/--cls) queue, instead of trusting whatever
    review_completed.json happened to say on disk. An image counts as
    done when every one of its queue items is non-pending AND (V11)
    every manual box on it is non-pending. Manual boxes saved before
    V11 have no "decision" field -- treated as "accepted" here, since
    they were already being included in output unconditionally at the
    time they were drawn, so this preserves previously-completed status
    instead of retroactively un-completing old sessions."""
    completed = set()
    for image, items in by_image_full.items():
        queue_done = all(
            progress.get(item_key(it), {}).get("decision", "pending") != "pending"
            for it in items)
        manual = progress.get(f"__manual__{image}", [])
        manual_done = all(mb.get("decision", "accepted") != "pending" for mb in manual)
        if queue_done and manual_done:
            completed.add(image)
    return completed


class ReviewApp:
    """Owns the Tkinter window and all mutable state for the image
    currently on screen. One instance runs the whole review session;
    _load_image() swaps in a new image's state as you navigate."""

    def __init__(self, root, image_keys, by_image, progress, reviewed_root,
                 progress_path, completed_path, completed_set,
                 show_original_default: bool, qa_mode: bool,
                 sync_enabled_default: bool = True,
                 sync_window_default: int = SYNC_DEFAULT_WINDOW,
                 all_image_keys_filtered=None,
                 by_image_full=None):
        self.root = root
        self.image_keys = image_keys
        self.by_image = by_image
        # (V13 fix A) The FULL, unfiltered-by---source/--cls image ->
        # queue-items map. Completion checks MUST use this, not the
        # (possibly filtered) self.by_image above -- otherwise a
        # session run with --cls person can mark an image "complete"
        # while it still has pending "car" boxes just because those
        # boxes aren't part of this session's filter. Falls back to
        # self.by_image if the caller doesn't pass one (keeps this
        # class usable standalone/in tests without behavior change
        # when there is no filtering to begin with).
        self.by_image_full = by_image_full if by_image_full is not None else by_image
        self.progress = progress
        self.reviewed_root = reviewed_root
        self.progress_path = progress_path
        self.completed_path = completed_path
        self.completed_set = completed_set
        self.qa_mode = qa_mode

        # (V12) The full set of images matching --source/--cls, BEFORE
        # any completed/pending split -- kept around so "Switch to QA /
        # Completed" can recompute pending vs. completed live from the
        # CURRENT completed_set at any point in the session, instead of
        # only being reachable by relaunching with --qa.
        self.all_image_keys_filtered = (all_image_keys_filtered
                                         if all_image_keys_filtered is not None
                                         else list(image_keys))

        # (V18) image_key -> source dataset name (e.g. "SARD", "UAVDT",
        # "VisDrone"), derived once from the FULL unfiltered queue so the
        # sidebar can color/group every image regardless of the current
        # --source/--cls filter. Falls back to "unknown" for an image
        # with no queue items at all (shouldn't normally happen, since
        # by_image_full is built FROM the queue, but kept defensive).
        self.image_source = {
            image: items[0].get("source", "unknown")
            for image, items in self.by_image_full.items()
        }
        # (V18) Populated by _build_sidebar_tree(); maps a Treeview leaf
        # item id -> the image_key it represents. Group/section nodes are
        # simply absent from this dict.
        self._sidebar_item_to_key = {}
        # (V22) Populated by _build_sidebar_tree(); maps a Treeview GROUP
        # node's item id (section/source/set -- never a leaf) to a
        # STABLE key that survives a rebuild (unlike the item id itself,
        # which is a fresh Tk-generated id every time the tree is
        # rebuilt). Used by _on_sidebar_node_open/_close to record which
        # groups the person has expanded into self._sidebar_open_state,
        # so a rebuild can restore the same expand state instead of
        # collapsing everything back to defaults.
        self._sidebar_node_to_state_key = {}
        # (V22) Set of stable group keys currently expanded, persisted
        # ACROSS rebuilds (a full tree rebuild happens after every save
        # -- see _refresh_sidebar -- since an image moving from
        # Remaining to Reviewed changes which groups exist at all).
        # Seeded with the "Remaining" section open by default, matching
        # the original one-time default; everything else starts closed
        # until the person expands it, exactly as before -- the only
        # change is that closing/reopening now STICKS across rebuilds
        # instead of resetting every time. Source/set keys are
        # deliberately NOT scoped to Remaining vs Reviewed (see
        # _build_sidebar_tree) -- expanding "UAVDT > M0209" while
        # working through it keeps that same set expanded when its last
        # image finishes and it reappears one level over, under
        # Reviewed, instead of collapsing right when you're mid-set.
        self._sidebar_open_state = {"section:Remaining"}

        self.idx = 0
        self.dirty = False
        self._autosave_after_id = None
        self._autoadvance_after_id = None
        self._next_manual_id = 0   # stable per-box id, see manual-box note below
        self.base_scale = 1.0
        self.zoom = 1.0
        self.scale = 1.0
        self.pil_img_full = None
        self.tk_img = None
        self.item_map = {}        # canvas item id -> (kind, key)
        self.handle_map = {}      # canvas item id -> (kind, key, handle_name)
        self.drag = None
        self.selected_key = None
        self._rubber_id = None
        self.reposition_armed = None   # (kind, key) currently allowed one drag/resize

        # (V14) Clipboard for copy/paste: {"cls": str, "box": [x1,y1,x2,y2]}
        # (box in ORIGINAL IMAGE pixel coordinates, same space every other
        # box is stored in) or None if nothing has been copied yet this
        # session. _paste_count tracks how many times Ctrl+V has been
        # pressed since the last Ctrl+C, so successive pastes cascade by
        # PASTE_OFFSET_PX instead of stacking on top of each other -- see
        # _copy_selected() / _paste_clipboard().
        self.clipboard = None
        self._paste_count = 0

        # (V17) Deferred sync: each entry is a zero-arg callable doing
        # exactly what used to fire immediately on click. Drained in
        # order at the start of every _flush() -- autosave debounce,
        # Save Now, Next/Previous, close, or focus-loss -- so a burst
        # of edits on one image becomes one sync pass, and every save
        # re-runs from current state (each queued call's own
        # retract+reapply already supersedes whatever was synced
        # before).
        self._pending_sync_actions = []

        self.image_key = None
        self.source_name = None
        self.queue_items = []
        self.decisions = {}
        self.class_overrides = {}
        self.manual_boxes = []
        self.original_boxes = []
        self.model_counts = {m: 0 for m in MODEL_KEYS}

        # Undo/redo -- per-image stacks of full-state snapshots. Reset
        # every time _load_image() runs (see V8 note #2 for why this is
        # deliberately not cross-image).
        self.undo_stack = []
        self.redo_stack = []

        # Session-wide streak counter: consecutive accept/reject clicks.
        self.streak = 0

        self.show_original = tk.BooleanVar(value=show_original_default)
        self.show_all_queue = tk.BooleanVar(value=True)
        self.show_deleted = tk.BooleanVar(value=False)
        self.fast_mode = tk.BooleanVar(value=False)
        self.auto_advance = tk.BooleanVar(value=False)
        self.model_vars = {m: tk.BooleanVar(value=False) for m in MODEL_KEYS}

        self.model_count_labels = {}

        # --- Cross-frame sync (V10 / V11) state ---
        self.sync_enabled = tk.BooleanVar(value=sync_enabled_default)
        self.seq_neighbors_window = tk.IntVar(value=sync_window_default)
        # (V17) Last known-good sync-window value -- used by
        # _sync_window_value() to recover if the Spinbox's IntVar ever
        # ends up holding a non-numeric Tcl value.
        self._last_valid_sync_window = sync_window_default
        self.sync_debug = tk.BooleanVar(value=False)
        self.sync_status_var = tk.StringVar(value="")
        self._last_sync_batch = []     # [{"kind":..., "image_key":..., ...}, ...]
        self._img_dims_cache = {}
        self._orig_boxes_cache = {}
        self._orig_hidden_count = 0
        self.synced_from_map = {}      # key -> source image_key, for the CURRENT image

        # Precompute (sequence_key -> sorted [(frame_num, image_key), ...])
        # and image_key -> (sequence_key, frame_num), once, from every
        # image in this session's filtered by_image (so sync never
        # reaches into images outside --source/--cls scope).
        self.image_seq_key = {}
        self.image_frame_num = {}
        seq_map: dict = {}
        for k in self.by_image.keys():
            sf = parse_seq_frame(k)
            if sf is None:
                continue
            seq_id, frame_num = sf
            seq_key = f"{Path(k).parent}::{seq_id}"
            self.image_seq_key[k] = seq_key
            self.image_frame_num[k] = frame_num
            seq_map.setdefault(seq_key, []).append((frame_num, k))
        self.sequence_frames = {sk: sorted(v) for sk, v in seq_map.items()}

        n_parsed = len(self.image_seq_key)
        n_total = len(self.by_image)
        print(f"[sync] parsed sequence/frame for {n_parsed}/{n_total} images "
              f"across {len(self.sequence_frames)} sequence(s).")
        if n_parsed == 0 and n_total > 0:
            sample = next(iter(self.by_image.keys()))
            print(f"[sync] WARNING: could not parse a sequence/frame from ANY "
                  f"image path -- sync will find zero neighbors for everything. "
                  f"Example path: {sample!r}. Expected either a filename like "
                  f"'..._<SEQ>_img<FRAME>_...' or a folder-per-sequence layout "
                  f"like '.../<SEQ>/img<FRAME>.ext'. If your paths look "
                  f"different, tell Claude the actual 'image' path format from "
                  f"pseudo_labels_review_queue.json so parse_seq_frame() can be "
                  f"adjusted.")

        self._build_ui()
        self._build_sidebar_tree()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._load_image(0)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        menubar = tk.Menu(self.root)

        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Save Now", command=self.save_current, accelerator="S")
        file_menu.add_command(label="Save && Quit", command=self.quit_saving)
        menubar.add_cascade(label="File", menu=file_menu)

        edit_menu = tk.Menu(menubar, tearoff=0)
        edit_menu.add_command(label="Undo", command=self.undo, accelerator="Ctrl+Z")
        edit_menu.add_command(label="Redo", command=self.redo, accelerator="Ctrl+Y")
        edit_menu.add_separator()
        edit_menu.add_command(label="Copy Selected Box", command=self._copy_selected, accelerator="Ctrl+C")
        edit_menu.add_command(label="Paste Box", command=self._paste_clipboard, accelerator="Ctrl+V")
        edit_menu.add_command(label="Delete Selected Box", command=self._delete_selected, accelerator="Del")
        edit_menu.add_separator()
        edit_menu.add_command(label="Accept All Pending", command=self.bulk_accept_pending)
        edit_menu.add_command(label="Reject All Pending", command=self.bulk_reject_pending)
        edit_menu.add_separator()
        edit_menu.add_command(label="Undo Last Sync", command=self.undo_last_sync)
        menubar.add_cascade(label="Edit", menu=edit_menu)

        view_menu = tk.Menu(menubar, tearoff=0)
        view_menu.add_checkbutton(label="Show Original Labels",
                                   variable=self.show_original, command=self.redraw)
        view_menu.add_checkbutton(label="Show All Queue Boxes",
                                   variable=self.show_all_queue, command=self.redraw)
        view_menu.add_checkbutton(label="Show Deleted/Rejected (recoverable)",
                                   variable=self.show_deleted, command=self.redraw)
        view_menu.add_separator()
        for m in MODEL_KEYS:
            view_menu.add_checkbutton(label=f"Show {MODEL_SHORT[m]} detections",
                                       variable=self.model_vars[m], command=self.redraw)
        view_menu.add_separator()
        view_menu.add_checkbutton(label="Fast Mode (click=accept, right-click=reject)",
                                   variable=self.fast_mode)
        view_menu.add_checkbutton(label="Auto-advance when image is fully decided",
                                   variable=self.auto_advance)
        view_menu.add_separator()
        view_menu.add_checkbutton(label="Sync nearby frames (cross-frame propagation)",
                                   variable=self.sync_enabled)
        view_menu.add_checkbutton(label="Verbose sync log (console)",
                                   variable=self.sync_debug)
        view_menu.add_separator()
        view_menu.add_command(label="Switch to QA / Completed view",
                               command=self.toggle_qa_mode)
        view_menu.add_separator()
        view_menu.add_command(label="Check Sequence Coverage",
                               command=self._show_sequence_coverage)
        view_menu.add_separator()
        view_menu.add_command(label="Zoom In\t+", command=lambda: self._zoom_centered(ZOOM_STEP))
        view_menu.add_command(label="Zoom Out\t-", command=lambda: self._zoom_centered(1 / ZOOM_STEP))
        view_menu.add_command(label="Fit to Window\t0", command=self.reset_zoom)
        menubar.add_cascade(label="View", menu=view_menu)

        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="Controls", command=self.show_help_dialog)
        menubar.add_cascade(label="Help", menu=help_menu)

        self.root.config(menu=menubar)

        # (V12) Bottom-of-window chrome (status bar + Prev/Next/Save nav)
        # is built and packed to the BOTTOM *before* the scrollable image
        # canvas is packed. In Tk's pack geometry manager, space is
        # claimed in the order pack() is called, not by which "side" a
        # widget uses -- so packing the canvas (which expands to fill
        # everything left over) before the nav bar used to let the
        # canvas swallow the space the nav bar needed, pushing it off
        # the bottom of the window with no way to get it back short of
        # a bigger monitor. Reserving the nav/status space first fixes
        # that permanently; the canvas (which already has zoom + pan +
        # scrollbars for exactly this situation) just gets whatever
        # room remains.
        self.status_var = tk.StringVar()
        ttk.Label(self.root, textvariable=self.status_var, anchor="w").pack(
            side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 2))

        nav = ttk.Frame(self.root)
        nav.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)
        ttk.Button(nav, text="\u25c0 Previous", command=self.go_prev).pack(side=tk.LEFT)
        ttk.Button(nav, text="Next \u25b6", command=self.go_next).pack(side=tk.LEFT, padx=4)
        ttk.Button(nav, text="Save Now", command=self.save_current).pack(side=tk.LEFT, padx=12)
        self.qa_toggle_btn = ttk.Button(nav, text="Switch to QA / Completed",
                                         command=self.toggle_qa_mode)
        self.qa_toggle_btn.pack(side=tk.LEFT, padx=12)
        self.progress_var = tk.StringVar()
        ttk.Label(nav, textvariable=self.progress_var).pack(side=tk.RIGHT)

        toolbar = ttk.Frame(self.root)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)
        ttk.Label(toolbar, text="Show:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Checkbutton(toolbar, text="Original labels (real dataset, read-only)",
                         variable=self.show_original, command=self.redraw).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(toolbar, text="All queue boxes", variable=self.show_all_queue,
                         command=self.redraw).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(toolbar, text="Deleted/Rejected", variable=self.show_deleted,
                         command=self.redraw).pack(side=tk.LEFT, padx=4)
        ttk.Separator(toolbar, orient="vertical").pack(side=tk.LEFT, fill="y", padx=6)
        ttk.Label(toolbar, text="Models:").pack(side=tk.LEFT, padx=(0, 4))
        for m in MODEL_KEYS:
            ttk.Checkbutton(toolbar, text=MODEL_SHORT[m], variable=self.model_vars[m],
                             command=self.redraw).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="All", width=4,
                   command=self._select_all_models).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(toolbar, text="None", width=4,
                   command=self._select_no_models).pack(side=tk.LEFT, padx=2)
        ttk.Separator(toolbar, orient="vertical").pack(side=tk.LEFT, fill="y", padx=6)
        ttk.Button(toolbar, text="\u2212", width=2,
                   command=lambda: self._zoom_centered(1 / ZOOM_STEP)).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="+", width=2,
                   command=lambda: self._zoom_centered(ZOOM_STEP)).pack(side=tk.LEFT, padx=(2, 4))
        ttk.Button(toolbar, text="Fit", command=self.reset_zoom).pack(side=tk.LEFT)

        toolbar2 = ttk.Frame(self.root)
        toolbar2.pack(side=tk.TOP, fill=tk.X, padx=6)
        ttk.Checkbutton(toolbar2, text="Fast Mode (click=accept / right-click=reject)",
                         variable=self.fast_mode).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Checkbutton(toolbar2, text="Auto-advance",
                         variable=self.auto_advance).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Separator(toolbar2, orient="vertical").pack(side=tk.LEFT, fill="y", padx=6)
        ttk.Button(toolbar2, text="Accept All Pending",
                   command=self.bulk_accept_pending).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar2, text="Reject All Pending",
                   command=self.bulk_reject_pending).pack(side=tk.LEFT, padx=2)
        ttk.Separator(toolbar2, orient="vertical").pack(side=tk.LEFT, fill="y", padx=6)
        ttk.Button(toolbar2, text="\u21b6 Undo", command=self.undo).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar2, text="\u21b7 Redo", command=self.redo).pack(side=tk.LEFT, padx=2)
        ttk.Separator(toolbar2, orient="vertical").pack(side=tk.LEFT, fill="y", padx=6)
        ttk.Button(toolbar2, text="Copy", command=self._copy_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar2, text="Paste", command=self._paste_clipboard).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar2, text="Delete", command=self._delete_selected).pack(side=tk.LEFT, padx=2)

        # Cross-frame sync controls (V10/V11): on/off + how many frames
        # out in each direction to look for a matching pending box (or
        # to copy an accepted manual box into), plus a dedicated undo
        # for just the auto-applied batch.
        toolbar3 = ttk.Frame(self.root)
        toolbar3.pack(side=tk.TOP, fill=tk.X, padx=6)
        ttk.Checkbutton(toolbar3, text="Sync nearby frames",
                         variable=self.sync_enabled).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(toolbar3, text="\u00b1").pack(side=tk.LEFT)
        # (V17) validatecommand restricts the field to digits (or empty,
        # mid-edit) so a stray keystroke can't leave the underlying Tcl
        # value non-numeric and permanently break every subsequent
        # .get() for the rest of the session -- see _sync_window_value()
        # and the V17 module note for the crash this used to cause.
        vcmd = (self.root.register(self._validate_sync_window_input), "%P")
        ttk.Spinbox(toolbar3, from_=0, to=SYNC_MAX_WINDOW, width=3,
                    textvariable=self.seq_neighbors_window,
                    validate="key", validatecommand=vcmd).pack(side=tk.LEFT, padx=(2, 2))
        ttk.Label(toolbar3, text="frames").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(toolbar3, text="Undo Last Sync",
                   command=self.undo_last_sync).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(toolbar3, text="Verbose sync log (console)",
                         variable=self.sync_debug).pack(side=tk.LEFT, padx=(8, 2))
        ttk.Label(toolbar3, textvariable=self.sync_status_var,
                  foreground="#1a6f1a").pack(side=tk.LEFT, padx=10)

        # Per-model detection-count readout for the CURRENT image, so a
        # model contributing zero boxes here is obvious rather than just
        # silently absent. Independent of the visibility checkboxes above.
        counts_bar = ttk.Frame(self.root)
        counts_bar.pack(side=tk.TOP, fill=tk.X, padx=6)
        ttk.Label(counts_bar, text="Detections in this image \u2014").pack(side=tk.LEFT, padx=(0, 6))
        for m in MODEL_KEYS:
            lbl = tk.Label(counts_bar, text=f"{MODEL_SHORT[m]}: 0")
            lbl.pack(side=tk.LEFT, padx=8)
            self.model_count_labels[m] = lbl

        # (V18) main_area holds the left-hand image-list sidebar and the
        # scrollable image canvas side by side. This is the ONLY
        # structural change to the chrome layout -- nav/status are still
        # packed to self.root directly, BEFORE main_area, so V12's "nav
        # bar can never be pushed off-screen" guarantee is untouched.
        #
        # (V21) Upgraded from a plain Frame to a classic tk.PanedWindow
        # so the sidebar/canvas split gets a draggable sash -- the sidebar
        # started as a fixed SIDEBAR_WIDTH_PX with no way to see a long
        # filename that got elided, and a plain side-by-side pack has no
        # concept of a resize handle at all. tk.PanedWindow (not ttk's --
        # ttk::panedwindow panes only support a "weight" option, no
        # minsize) is used specifically because each pane can be given
        # its own minsize: the two panes can NEVER overlap or cover one
        # another (that's a hard property of the widget, not something
        # this minsize enforces), and minsize on each just stops the
        # sash from being dragged so far that one pane gets squeezed
        # into something unusably tiny. stretch="never" on the sidebar
        # means resizing the whole WINDOW doesn't stretch the sidebar --
        # extra/lost space goes to the canvas pane instead, so the
        # sidebar's width stays exactly where you last dragged it until
        # you drag it again.
        main_area = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                                    sashwidth=6, sashrelief=tk.RAISED,
                                    opaqueresize=True, bd=0)
        main_area.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(4, 0))

        sidebar = self._build_sidebar(main_area)
        main_area.add(sidebar, width=SIDEBAR_WIDTH_PX,
                       minsize=SIDEBAR_MIN_WIDTH_PX, stretch="never")

        canvas_frame = ttk.Frame(main_area)
        main_area.add(canvas_frame, minsize=MIN_CANVAS_W, stretch="always")

        self.canvas = tk.Canvas(canvas_frame, bg="#111111", cursor="tcross")
        hbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        vbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)

        self.canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)

        # Hover feedback: show the right resize/move/draw cursor before
        # you've even clicked, so the handles are discoverable.
        self.canvas.bind("<Motion>", self.on_mouse_move_hover)

        # Fast mode: right-click a queue box = instant reject.
        self.canvas.bind("<ButtonPress-3>", self.on_right_click)

        # Panning: middle-click-drag.
        self.canvas.bind("<ButtonPress-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan_drag)

        # Zoom: mouse wheel (Windows/Mac) and Button-4/5 (Linux/X11).
        self.canvas.bind("<MouseWheel>", self.on_mousewheel)
        self.canvas.bind("<Button-4>", self.on_mousewheel)
        self.canvas.bind("<Button-5>", self.on_mousewheel)

        self.root.bind("<Key>", self.on_key)
        self.root.bind("<Control-z>", lambda e: self.undo())
        self.root.bind("<Control-y>", lambda e: self.redo())
        self.root.bind("<Control-Z>", lambda e: self.redo())  # Ctrl+Shift+Z on many platforms

        # (V14) Copy/paste: Ctrl+C copies whichever box is currently
        # selected (see self.selected_key), Ctrl+V pastes a new manual
        # box from the clipboard, cascading by PASTE_OFFSET_PX on each
        # repeated paste. Bound on both lowercase (the common case) and
        # uppercase keysyms so it still fires if Caps Lock happens to be
        # on -- Tk reports a different keysym in that case, same reason
        # <Control-Z> is bound above alongside <Control-y>.
        self.root.bind("<Control-c>", lambda e: self._copy_selected())
        self.root.bind("<Control-C>", lambda e: self._copy_selected())
        self.root.bind("<Control-v>", lambda e: self._paste_clipboard())
        self.root.bind("<Control-V>", lambda e: self._paste_clipboard())

        # Delete/Backspace: delete whichever box is currently selected
        # (set by clicking a box -- see _open_box_context_menu /
        # fast-mode / drag-select). No-op if nothing is selected. Bound
        # directly rather than routed through on_key() since Delete/
        # BackSpace keysyms aren't in the small set on_key() already
        # switches on.
        #
        # (V15) NOTE: this root-level binding only fires while NO menu
        # currently holds Tk's keyboard grab. tk.Menu.tk_popup() (used
        # by _open_box_context_menu) takes that grab for the duration
        # the menu is posted, which is what makes arrow-key navigation
        # through the menu work -- but it also means Delete/BackSpace
        # pressed while a box's context menu is open never reaches this
        # binding at all. _open_box_context_menu() ALSO binds Delete/
        # BackSpace directly on the popup menu widget itself (right
        # before tk_popup()) so the shortcut works identically whether
        # the menu is open or closed -- see the comment there.
        self.root.bind("<Delete>", lambda e: self._delete_selected())
        self.root.bind("<BackSpace>", lambda e: self._delete_selected())

        # Crash-safety: flush the current image to disk immediately the
        # moment this window loses OS focus (Alt-Tab, another app,
        # screen lock, sleep) -- don't wait out the autosave debounce.
        # Covers the vast majority of "app got killed mid-edit" cases
        # that the old fixed 1s debounce could theoretically lose.
        self.root.bind("<FocusOut>", lambda e: self._flush_now_if_dirty())

        # (V12) Reasonable minimum window size so the nav/status bars
        # (now packed first) and a usable slice of canvas can never
        # both be squeezed to nothing.
        self.root.minsize(900, 480)

    def _build_sidebar(self, parent):
        """(V18) Left-hand image-list sidebar. Purely a navigation aid --
        does not read or write review_progress.json/review_completed.json
        itself, only the in-memory self.all_image_keys_filtered /
        self.completed_set / self.image_source ReviewApp already
        maintains. See _build_sidebar_tree() for how it's populated and
        _on_sidebar_select()/_jump_to_image() for what a click does.

        (V19) Now nests a third level per source -- one row per frame
        SET (sequence), each with its own generated color (see
        _color_for_sequence) and an inline reviewed/total count -- so
        the tree itself doubles as the "have I gotten to every set yet"
        view, not just the separate "Check sequence coverage" report.

        (V21) Returns the sidebar Frame instead of packing itself --
        the caller (_build_ui) now adds it as a pane of a PanedWindow,
        which owns its placement/sizing, so this method only builds its
        CONTENTS (which still use plain pack() internally, same as
        before -- only how the sidebar-as-a-whole is placed by its
        parent changed). self._sidebar_frame is kept so
        _display_size_budget() can read back whatever width the sash is
        CURRENTLY at, rather than assuming the original SIDEBAR_WIDTH_PX,
        when it sizes the image canvas."""
        sidebar = ttk.Frame(parent)
        self._sidebar_frame = sidebar

        ttk.Label(sidebar, text="Images by Set (Remaining / Reviewed)",
                  font=("TkDefaultFont", 9, "bold")).pack(side=tk.TOP, anchor="w", pady=(0, 2))

        tree_frame = ttk.Frame(sidebar)
        tree_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.sidebar_tree = ttk.Treeview(tree_frame, show="tree", selectmode="browse")
        sidebar_vbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.sidebar_tree.yview)
        self.sidebar_tree.configure(yscrollcommand=sidebar_vbar.set)
        self.sidebar_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sidebar_vbar.pack(side=tk.LEFT, fill=tk.Y)

        # Color tags: one per known source (light stripe, see
        # SOURCE_COLORS), plus a grey fallback for anything unlisted --
        # used on the SOURCE-level group rows. Per-SET (sequence) tags
        # are configured on the fly in _build_sidebar_tree() as sets are
        # discovered, since there can be many more of them than there
        # are known sources.
        for src, color in SOURCE_COLORS.items():
            self.sidebar_tree.tag_configure(f"src_{src}", background=color)
        self.sidebar_tree.tag_configure("src_other", background=SOURCE_COLOR_OTHER)

        self.sidebar_tree.bind("<<TreeviewSelect>>", self._on_sidebar_select)
        # (V22) Track expand/collapse so a rebuild (which happens after
        # every save) can restore it instead of resetting everything to
        # closed -- see self._sidebar_open_state and
        # _on_sidebar_node_open/_close below.
        self.sidebar_tree.bind("<<TreeviewOpen>>", self._on_sidebar_node_open)
        self.sidebar_tree.bind("<<TreeviewClose>>", self._on_sidebar_node_close)

        legend = ttk.Frame(sidebar)
        legend.pack(side=tk.TOP, fill=tk.X, pady=(4, 2))
        ttk.Label(legend, text="Source colors:", font=("TkDefaultFont", 8)).pack(side=tk.TOP, anchor="w")
        for src, color in list(SOURCE_COLORS.items()):
            row = tk.Frame(legend)
            row.pack(side=tk.TOP, anchor="w")
            tk.Label(row, text="  ", bg=color, relief="solid", borderwidth=1).pack(side=tk.LEFT)
            tk.Label(row, text=f" {src}", font=("TkDefaultFont", 8)).pack(side=tk.LEFT)
        ttk.Label(legend, text="Each frame set below also gets its own\ncolor -- expand a source to see them.",
                  font=("TkDefaultFont", 7), foreground="#555555",
                  justify="left").pack(side=tk.TOP, anchor="w", pady=(4, 0))

        ttk.Button(sidebar, text="Refresh list",
                   command=self._refresh_sidebar).pack(side=tk.TOP, fill=tk.X, pady=(4, 2))
        ttk.Button(sidebar, text="Check sequence coverage",
                   command=self._show_sequence_coverage).pack(side=tk.TOP, fill=tk.X)

        return sidebar

    def _validate_sync_window_input(self, proposed: str) -> bool:
        """(V17) Spinbox validatecommand -- only digits or empty
        (mid-edit) are allowed to land in the field. This is the
        belt-and-suspenders half of the fix; _sync_window_value() below
        is what actually guarantees .get() can never raise even if a
        bad value somehow gets in (e.g. via a Tk spin-button quirk this
        validation doesn't cover)."""
        return proposed == "" or proposed.isdigit()

    def _sync_window_value(self) -> int:
        """(V17) Safe read of the sync-window Spinbox's IntVar. ttk.
        Spinbox has no built-in validation of its own, so a stray
        keystroke (or a Tk spin-button quirk) can leave the underlying
        Tcl value non-numeric -- after that, every .get() on the IntVar
        raises TclError, silently aborting every sync for the rest of
        the session (see the V17 module note for the full story). Never
        raises: on a bad value it resets the field to the last
        known-good window and returns that instead."""
        try:
            val = int(self.seq_neighbors_window.get())
        except (tk.TclError, ValueError):
            val = getattr(self, "_last_valid_sync_window", SYNC_DEFAULT_WINDOW)
            print(f"[sync] WARNING: sync-window field had a bad value, reset to {val}.")
            self.seq_neighbors_window.set(val)
            return val
        val = max(0, min(SYNC_MAX_WINDOW, val))
        self._last_valid_sync_window = val
        return val

    def _select_all_models(self):
        for v in self.model_vars.values():
            v.set(True)
        self.redraw()

    def _select_no_models(self):
        for v in self.model_vars.values():
            v.set(False)
        self.redraw()

    # ------------------------------------------------------------------
    # (V18) Sidebar image list -- build, select, jump, coverage report.
    # (V19) now also groups/colors by frame SET, not just by source.
    # None of this reads/writes progress/completed files directly; it
    # only reflects the in-memory state ReviewApp already maintains, and
    # is rebuilt (see call sites of _refresh_sidebar) whenever that state
    # could have changed the Remaining/Reviewed split.
    # ------------------------------------------------------------------

    def _build_sidebar_tree(self):
        """(re)populates the sidebar Treeview from scratch: two top-level
        sections, "Remaining" and "Reviewed", each split into per-source
        groups (color-tagged, see SOURCE_COLORS), each of THOSE split
        further into per-frame-SET groups (V19 -- same sequence grouping
        "Check Sequence Coverage" uses, see parse_seq_frame()), each with
        its own distinguishing color (see _color_for_sequence) and an
        inline "n / total" frame-count annotation, and finally one leaf
        per image (tagged with its set's color) sorted by filename.
        Cheap enough to call after every save -- typical session sizes
        here are in the hundreds to low thousands of images, not enough
        for a full Treeview rebuild to be noticeable on a debounced
        ~400ms cadence.

        (V22) A full rebuild is unavoidable -- an image finishing review
        moves it from Remaining to Reviewed, which can change which
        source/set GROUPS exist at all (a set that just lost its last
        remaining image disappears from under Remaining and appears
        under Reviewed), so incrementally patching the tree in place
        isn't really simpler than just rebuilding it. What used to make
        that rebuild disruptive is that every group defaulted back to
        closed each time -- so finishing one image in a set you had
        expanded collapsed it, and you had to re-expand your way back
        down to it, on every single save. Expand state now persists
        across the rebuild instead (self._sidebar_open_state, kept in
        sync by _on_sidebar_node_open/_close), and the vertical scroll
        position is preserved too, so completing an image no longer
        visibly disturbs whatever part of the tree you were looking
        at -- the row for that specific image just quietly moves from
        the Remaining branch of its set to the Reviewed branch of the
        SAME set, which stays expanded and in view exactly as before."""
        tree = self.sidebar_tree
        selected_key = self.image_key  # preserve highlight across a rebuild
        scroll_frac = tree.yview()[0] if tree.get_children() else 0.0
        tree.delete(*tree.get_children())
        self._sidebar_item_to_key = {}
        self._sidebar_node_to_state_key = {}

        remaining = [k for k in self.all_image_keys_filtered if k not in self.completed_set]
        reviewed = [k for k in self.all_image_keys_filtered if k in self.completed_set]

        # (V19) Per (source, set) totals and reviewed-so-far counts,
        # computed once across the WHOLE filtered session (not just
        # whichever section is being built) -- so a set's row can always
        # show "x of N total" regardless of whether that set currently
        # has more remaining or more reviewed frames, and so the
        # "needs more" marker reflects the set's real overall progress,
        # not just what happens to be in front of you in this section.
        set_totals: dict = {}
        set_reviewed: dict = {}
        for k in self.all_image_keys_filtered:
            src = self.image_source.get(k, "unknown")
            sf = parse_seq_frame(k)
            seq = sf[0] if sf else "(unparsed set)"
            group = (src, seq)
            set_totals[group] = set_totals.get(group, 0) + 1
            if k in self.completed_set:
                set_reviewed[group] = set_reviewed.get(group, 0) + 1

        for section_label, keys in (("Remaining", remaining), ("Reviewed", reviewed)):
            # (V22) Section open/closed state persists via a stable key
            # ("section:Remaining" / "section:Reviewed") rather than
            # always defaulting Remaining-open/Reviewed-closed -- so if
            # you manually open "Reviewed" to spot-check something, it
            # stays open across the next few saves instead of snapping
            # shut again.
            sec_state_key = f"section:{section_label}"
            section_node = tree.insert(
                "", "end", text=f"{section_label} ({len(keys)})",
                open=(sec_state_key in self._sidebar_open_state))
            self._sidebar_node_to_state_key[section_node] = sec_state_key

            by_src: dict = {}
            for k in keys:
                by_src.setdefault(self.image_source.get(k, "unknown"), []).append(k)
            for src in sorted(by_src):
                src_keys_all = by_src[src]
                tag = f"src_{src}" if src in SOURCE_COLORS else "src_other"
                # (V22) Source state key is deliberately NOT scoped to
                # section -- expanding "UAVDT" under Remaining keeps
                # "UAVDT" expanded under Reviewed too, since they're the
                # same logical group as far as the person is concerned,
                # just currently showing a different subset of images.
                src_state_key = f"source:{src}"
                src_node = tree.insert(section_node, "end",
                                        text=f"{src} ({len(src_keys_all)})",
                                        open=(src_state_key in self._sidebar_open_state),
                                        tags=(tag,))
                self._sidebar_node_to_state_key[src_node] = src_state_key

                # (V19) Group this source's images (within this section)
                # by frame set, and give each set its own row + color.
                by_seq: dict = {}
                for k in src_keys_all:
                    sf = parse_seq_frame(k)
                    seq = sf[0] if sf else "(unparsed set)"
                    by_seq.setdefault(seq, []).append(k)

                for seq in sorted(by_seq):
                    seq_keys = sorted(by_seq[seq], key=lambda kk: Path(kk).name)
                    group = (src, seq)
                    total = set_totals.get(group, len(seq_keys))
                    reviewed_overall = set_reviewed.get(group, 0)
                    needs_more = reviewed_overall < MIN_REVIEWED_FRAMES_PER_SEQUENCE
                    flag = "  \u26a0" if needs_more else ""
                    seq_tag = f"seq_{src}_{seq}"
                    self.sidebar_tree.tag_configure(
                        seq_tag, background=_color_for_sequence(f"{src}::{seq}"))
                    # (V22) Same reasoning as the source key above -- NOT
                    # scoped to section, so a set you're actively working
                    # through stays expanded as its images individually
                    # cross over from Remaining to Reviewed one at a
                    # time, instead of collapsing on every single one.
                    set_state_key = f"set:{src}:{seq}"
                    seq_node = tree.insert(
                        src_node, "end",
                        text=f"{seq}  ({len(seq_keys)}/{total} total){flag}",
                        open=(set_state_key in self._sidebar_open_state), tags=(seq_tag,))
                    self._sidebar_node_to_state_key[seq_node] = set_state_key
                    for k in seq_keys:
                        leaf = tree.insert(seq_node, "end", text=Path(k).name, tags=(seq_tag,))
                        self._sidebar_item_to_key[leaf] = k

        if selected_key is not None:
            self._highlight_sidebar_selection()
        # (V22) Restore roughly the same scroll position the tree was at
        # before this rebuild -- without this, every rebuild snapped the
        # view back to the very top (the "Remaining" header), which read
        # as the list jumping around even once expand state itself
        # stopped resetting.
        tree.yview_moveto(scroll_frac)

    def _refresh_sidebar(self):
        self._build_sidebar_tree()

    def _on_sidebar_node_open(self, event=None):
        """(V22) Records that a group node (section/source/set) was
        expanded, keyed by its STABLE key (see _sidebar_node_to_state_key)
        rather than its Tk item id, so the state survives the next full
        tree rebuild instead of being lost the moment the old item id
        stops existing. Tk sets the tree's focus to the node that
        triggered <<TreeviewOpen>> just before generating the event,
        which is the standard way to identify it."""
        item_id = self.sidebar_tree.focus()
        state_key = self._sidebar_node_to_state_key.get(item_id)
        if state_key:
            self._sidebar_open_state.add(state_key)

    def _on_sidebar_node_close(self, event=None):
        """(V22) Mirror of _on_sidebar_node_open for <<TreeviewClose>>."""
        item_id = self.sidebar_tree.focus()
        state_key = self._sidebar_node_to_state_key.get(item_id)
        if state_key:
            self._sidebar_open_state.discard(state_key)

    def _highlight_sidebar_selection(self):
        """Selects (and scrolls to) whichever sidebar leaf corresponds to
        self.image_key, without triggering another jump -- _on_sidebar_
        select() below no-ops when the clicked/selected leaf is already
        the current image.

        (V20 FIX) Only does this within the section matching the CURRENT
        mode (Remaining while reviewing normally, Reviewed while in QA).
        Treeview.see() force-opens every ancestor of whatever item it
        scrolls to -- so previously, the instant the image on screen
        became fully decided (its last pending box got a decision) and
        moved from Remaining into Reviewed, this would call see() on its
        NEW leaf under the "Reviewed" branch, which yanked that section
        open and scrolled the sidebar down into it -- even though you
        were still in ordinary review (qa_mode still False) and never
        asked to look at Reviewed. That's the "list auto-switches to the
        completed view when I finish an image" bug. Now, if the image's
        Remaining/Reviewed status doesn't match the session's current
        mode, this just leaves the sidebar's scroll/expand state alone --
        the tree still contains and updates both sections underneath
        (the counts and per-set annotations are unaffected), it just
        won't be dragged over to show one you didn't ask for. The only
        ways into the Reviewed section remain explicit: the "Switch to
        QA / Completed" button, or clicking a Reviewed image directly."""
        is_reviewed = self.image_key in self.completed_set
        if is_reviewed != self.qa_mode:
            return
        for item_id, key in self._sidebar_item_to_key.items():
            if key == self.image_key:
                self.sidebar_tree.see(item_id)
                self.sidebar_tree.selection_set(item_id)
                return

    def _on_sidebar_select(self, event=None):
        sel = self.sidebar_tree.selection()
        if not sel:
            return
        key = self._sidebar_item_to_key.get(sel[0])
        if key is None:
            return  # a section/source/set group node, not a leaf image
        if key == self.image_key:
            return
        self._jump_to_image(key)

    def _jump_to_image(self, image_key):
        """Navigates straight to `image_key`, wherever it currently sits
        (still-pending or already-completed), flushing the image
        currently on screen first exactly like Next/Previous/Switch-to-
        QA already do. Silently does nothing if `image_key` somehow
        isn't part of this session's --source/--cls filter."""
        if image_key not in self.all_image_keys_filtered:
            return
        self._flush_now_if_dirty()
        target_qa = image_key in self.completed_set
        keys = [k for k in self.all_image_keys_filtered
                if (k in self.completed_set) == target_qa]
        if image_key not in keys:
            return
        self.qa_mode = target_qa
        self.image_keys = keys
        self.qa_toggle_btn.config(
            text="Back to Pending Review" if self.qa_mode else "Switch to QA / Completed")
        self._load_image(keys.index(image_key))

    def _show_sequence_coverage(self):
        """(V18) Read-only report: for every (source, sequence) pair in
        the CURRENT --source/--cls filter, how many of its frames are in
        the Reviewed/completed list right now, flagging anything under
        MIN_REVIEWED_FRAMES_PER_SEQUENCE. Uses the same parse_seq_frame()
        cross-frame sync relies on, so "sequence" here means the same
        thing it means everywhere else in this tool (and the same thing
        the V19 sidebar rows mean). Changes nothing on disk -- purely
        informational, meant to answer "do we have enough manually
        reviewed keyframes per set/sequence to start training yet?"."""
        totals: dict = {}
        reviewed_counts: dict = {}
        for k in self.all_image_keys_filtered:
            src = self.image_source.get(k, "unknown")
            sf = parse_seq_frame(k)
            seq = sf[0] if sf else "(unparsed sequence)"
            group = (src, seq)
            totals[group] = totals.get(group, 0) + 1
            if k in self.completed_set:
                reviewed_counts[group] = reviewed_counts.get(group, 0) + 1

        if not totals:
            messagebox.showinfo("Sequence coverage", "No images in the current filter.")
            return

        low = []
        lines = []
        for group in sorted(totals):
            src, seq = group
            c = reviewed_counts.get(group, 0)
            t = totals[group]
            flag = "" if c >= MIN_REVIEWED_FRAMES_PER_SEQUENCE else "   <-- needs more"
            lines.append(f"{src:>10s} | {seq:<20s} reviewed {c:>3d} / {t:<4d}{flag}")
            if c < MIN_REVIEWED_FRAMES_PER_SEQUENCE:
                low.append(group)

        header = (f"{len(low)} of {len(totals)} sequence(s) have fewer than "
                   f"{MIN_REVIEWED_FRAMES_PER_SEQUENCE} reviewed frame(s).\n\n")
        body = "\n".join(lines)
        # messagebox has no scrollbar -- keep it readable by truncating
        # very long reports rather than showing an unusably tall dialog.
        if len(body) > 4000:
            body = body[:4000] + "\n... (truncated -- narrow the --source/--cls filter to see the rest)"
        messagebox.showinfo("Sequence coverage (reviewed frames per set/sequence)",
                             header + body)

    # ------------------------------------------------------------------
    # QA / completed toggle (V12)
    # ------------------------------------------------------------------

    def toggle_qa_mode(self):
        """Swaps the current session between reviewing PENDING images
        and browsing/correcting already-COMPLETED ones, live, without
        restarting the app. Recomputed fresh from self.completed_set
        every time it's called, so it always reflects whatever's
        actually been decided so far this session -- not just what was
        true at launch. Flushes the current image first so nothing is
        lost on the switch."""
        self._flush_now_if_dirty()
        target_qa = not self.qa_mode
        if target_qa:
            keys = [k for k in self.all_image_keys_filtered if k in self.completed_set]
            empty_msg = ("No images are fully reviewed yet for these filters -- "
                         "nothing to open in QA/Completed view.")
        else:
            keys = [k for k in self.all_image_keys_filtered if k not in self.completed_set]
            empty_msg = ("Everything matching these filters is fully reviewed. "
                         "Staying in QA/Completed view so you can still browse "
                         "and correct it.")
        if not keys:
            messagebox.showinfo("Nothing to show", empty_msg)
            return
        self.qa_mode = target_qa
        self.image_keys = keys
        self.qa_toggle_btn.config(
            text="Back to Pending Review" if self.qa_mode else "Switch to QA / Completed")
        self._load_image(0)

    # ------------------------------------------------------------------
    # Image loading / navigation
    # ------------------------------------------------------------------

    def _display_size_budget(self):
        """(V12) Computes the max width/height the image canvas can use
        without pushing the nav/status bars (or any toolbar row) off
        the visible screen. Uses the real screen size, minus a
        reserved chrome allowance, and re-measures the window's actual
        non-canvas chrome once it's been drawn at least once (more
        accurate than the static estimate for unusual font sizes/DPI)."""
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()

        reserved_h = CHROME_RESERVED_H
        try:
            self.root.update_idletasks()
            win_h = self.root.winfo_height()
            canvas_h = self.canvas.winfo_height() if hasattr(self, "canvas") else 0
            if win_h > 1 and canvas_h > 1:
                measured_chrome = win_h - canvas_h
                if measured_chrome > 0:
                    reserved_h = max(reserved_h, measured_chrome)
        except tk.TclError:
            pass

        # (V18) The sidebar eats into available width -- reserve enough
        # that the canvas doesn't try to claim space the sidebar is
        # using. (V21) Since the sidebar is now a resizable pane, this
        # reads back its CURRENT width (wherever the sash has been
        # dragged to) rather than assuming it's still at the original
        # SIDEBAR_WIDTH_PX -- falls back to that constant before the
        # window has ever been drawn (winfo_width() returns 1).
        sidebar_w = SIDEBAR_WIDTH_PX
        if hasattr(self, "_sidebar_frame"):
            try:
                measured = self._sidebar_frame.winfo_width()
                if measured > 1:
                    sidebar_w = measured
            except tk.TclError:
                pass
        reserved_w = CHROME_RESERVED_W + sidebar_w

        max_w = min(DISPLAY_MAX_W, max(MIN_CANVAS_W, screen_w - reserved_w))
        max_h = min(DISPLAY_MAX_H, max(MIN_CANVAS_H, screen_h - reserved_h))
        return max_w, max_h

    def _load_image(self, idx):
        while 0 <= idx < len(self.image_keys):
            image_key = self.image_keys[idx]
            img = cv2.imread(image_key)
            if img is not None:
                break
            print(f"  [warn] could not read {image_key}, skipping.")
            idx += 1
        else:
            messagebox.showinfo("Done", "No more readable images.")
            self.root.destroy()
            return

        self.idx = idx
        self.image_key = image_key
        items = self.by_image[image_key]
        self.source_name = items[0]["source"]
        self.queue_items = items

        h, w = img.shape[:2]
        self.img_w, self.img_h = w, h
        self._img_dims_cache[image_key] = (w, h)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.pil_img_full = Image.fromarray(rgb)

        # (V12) Fit the image within whatever room is actually left
        # after the always-visible nav/status/toolbar chrome, instead
        # of a fixed 1280x860 that could already be taller than the
        # screen on its own.
        max_w, max_h = self._display_size_budget()
        self.base_scale = min(max_w / w, max_h / h, 1.0)
        self.zoom = 1.0

        base_w = max(1, int(w * self.base_scale))
        base_h = max(1, int(h * self.base_scale))
        self.canvas.config(width=base_w, height=base_h)

        self.decisions = {}
        self.synced_from_map = {}
        self.model_counts = {m: 0 for m in MODEL_KEYS}
        for it in self.queue_items:
            it["_orig_box"] = list(it.get("_orig_box", it["box"]))
            k = item_key(it)
            saved = self.progress.get(k, {})
            self.decisions[k] = saved.get("decision", "pending")
            if "box" in saved:
                it["box"] = list(saved["box"])
            if saved.get("synced_from"):
                self.synced_from_map[k] = saved["synced_from"]
            votes = it.get("votes", {})
            for m in MODEL_KEYS:
                if votes.get(m) is not None:
                    self.model_counts[m] += 1

        self.class_overrides = dict(self.progress.get(f"__override__{image_key}", {}))
        self.manual_boxes = [dict(b) for b in self.progress.get(f"__manual__{image_key}", [])]
        # Manual boxes are identified by a stable "_id", not their position
        # in the list -- deleting/editing box #2 of 5 must not silently
        # shift what boxes #3-5 refer to for the rest of the session.
        # Backfill ids (and, V11, "decision") for saves written before
        # those existed. Pre-V11 manual boxes were included in output
        # unconditionally, so they backfill to "accepted" -- not
        # "pending" -- to avoid retroactively un-completing old work.
        next_id = 0
        for mb in self.manual_boxes:
            if "_id" not in mb:
                mb["_id"] = next_id
            if "decision" not in mb:
                mb["decision"] = "accepted"
            next_id = max(next_id, mb["_id"] + 1)
        self._next_manual_id = next_id

        real_label_path = gpl.image_path_to_label_path(Path(image_key))
        self.original_boxes = load_original_labels(real_label_path, w, h)

        self.dirty = False
        self.selected_key = None
        self.drag = None
        self.reposition_armed = None

        # (V14) Copy/paste clipboard is deliberately NOT cleared here --
        # unlike undo/redo (which is scoped per-image on purpose), a
        # copied box is exactly the kind of thing you'd want to carry
        # across a Next/Previous navigation (copy a box on frame N,
        # paste equivalents into frame N+1, N+2, ...), so it persists
        # for the whole session until a fresh Ctrl+C replaces it.

        # (V17) Deferred sync queue reset. Harmless -- by the time you
        # navigate away, it's already been drained by the flush that
        # precedes navigation (see go_next/go_prev/_flush_now_if_dirty).
        self._pending_sync_actions = []

        # Undo/redo is scoped per-image -- a fresh image starts with a
        # clean slate rather than carrying over unrelated history.
        self.undo_stack = []
        self.redo_stack = []
        self.streak = 0

        if self._autoadvance_after_id is not None:
            self.root.after_cancel(self._autoadvance_after_id)
            self._autoadvance_after_id = None

        mode_tag = " [QA MODE]" if self.qa_mode else ""
        self.root.title(f"Label Review{mode_tag} \u2014 {Path(image_key).name} "
                         f"({idx + 1}/{len(self.image_keys)})")
        self._render_canvas_image()
        self._update_model_count_labels()
        self.redraw()
        # (V18) Keep the sidebar's highlighted row in sync with whatever
        # image is now on screen, however we got here (Next/Previous,
        # QA toggle, or a sidebar click itself).
        if hasattr(self, "sidebar_tree"):
            self._highlight_sidebar_selection()

    def _update_model_count_labels(self):
        for m in MODEL_KEYS:
            n = self.model_counts[m]
            lbl = self.model_count_labels[m]
            lbl.config(text=f"{MODEL_SHORT[m]}: {n}",
                       fg=("#cc0000" if n == 0 else "#1a1a1a"),
                       font=("TkDefaultFont", 9, "bold" if n == 0 else "normal"))

    def go_next(self):
        self._flush_now_if_dirty()
        if self.idx + 1 < len(self.image_keys):
            self._load_image(self.idx + 1)
        else:
            messagebox.showinfo("End of queue", "This is the last image.")

    def go_prev(self):
        self._flush_now_if_dirty()
        if self.idx - 1 >= 0:
            self._load_image(self.idx - 1)
        else:
            messagebox.showinfo("Start of queue", "This is the first image.")

    def quit_saving(self):
        self._flush_now_if_dirty()
        self.root.destroy()

    def on_close(self):
        self._flush_now_if_dirty()
        self.root.destroy()

    def on_key(self, event):
        if event.keysym == "s":
            self.save_current()
        elif event.keysym == "Right":
            self.go_next()
        elif event.keysym == "Left":
            self.go_prev()
        elif event.keysym in ("plus", "equal", "KP_Add"):
            self._zoom_centered(ZOOM_STEP)
        elif event.keysym in ("minus", "KP_Subtract"):
            self._zoom_centered(1 / ZOOM_STEP)
        elif event.keysym == "0":
            self.reset_zoom()
        elif event.keysym == "Escape" and self.drag:
            self._cancel_drag()
        elif event.keysym in "123456789" and self.selected_key is not None:
            # Quick class-assign: Nth class in CLASS_NAMES order, applied
            # to whichever box is currently selected (last one clicked).
            idx = int(event.keysym) - 1
            if idx < len(CLASS_NAMES):
                kind, key = self.selected_key
                cls = CLASS_NAMES[idx]
                self._push_undo()
                if kind == "queue":
                    self.class_overrides[key] = cls
                else:
                    self._find_manual(key)["cls"] = cls
                self._mark_dirty()
                self.redraw()

    def _cancel_drag(self):
        """Escape while dragging: abandon the in-progress draw, move, or
        resize. For move/resize we already _push_undo()'d once the drag
        crossed the move threshold, so the box's pre-drag geometry is
        sitting right there on top of the undo stack -- just pop it back
        via undo() rather than duplicating restore logic."""
        mode = self.drag.get("mode") if self.drag else None
        if mode == "draw":
            self.canvas.delete(self._rubber_id)
        elif mode in ("move", "resize") and self.drag.get("snapshotted"):
            self.undo()  # restores pre-drag geometry, also clears redo of it? no -- fine either way
            self.redo_stack.clear()  # a cancelled drag isn't something you'd want to "redo" back in
        self.drag = None
        self._update_status()

    # ------------------------------------------------------------------
    # Zoom / pan
    # ------------------------------------------------------------------

    def _render_canvas_image(self):
        self.scale = self.base_scale * self.zoom
        disp_w = max(1, int(self.img_w * self.scale))
        disp_h = max(1, int(self.img_h * self.scale))
        resized = self.pil_img_full.resize((disp_w, disp_h), Image.BILINEAR)
        self.tk_img = ImageTk.PhotoImage(resized)
        self.canvas.config(scrollregion=(0, 0, disp_w, disp_h))

    def _zoom_at(self, view_x, view_y, factor):
        old_scale = self.scale
        new_zoom = min(MAX_ZOOM, max(MIN_ZOOM, self.zoom * factor))
        if abs(new_zoom - self.zoom) < 1e-9:
            return

        cx = self.canvas.canvasx(view_x)
        cy = self.canvas.canvasy(view_y)
        img_x = cx / old_scale if old_scale else 0
        img_y = cy / old_scale if old_scale else 0

        self.zoom = new_zoom
        self._render_canvas_image()
        self.redraw()

        new_scale = self.scale
        total_w = max(1, self.img_w * new_scale)
        total_h = max(1, self.img_h * new_scale)
        new_cx = img_x * new_scale
        new_cy = img_y * new_scale
        frac_x = (new_cx - view_x) / total_w
        frac_y = (new_cy - view_y) / total_h
        self.canvas.xview_moveto(min(1.0, max(0.0, frac_x)))
        self.canvas.yview_moveto(min(1.0, max(0.0, frac_y)))
        self._update_status()

    def _zoom_centered(self, factor):
        self._zoom_at(self.canvas.winfo_width() / 2, self.canvas.winfo_height() / 2, factor)

    def reset_zoom(self):
        self.zoom = 1.0
        self._render_canvas_image()
        self.canvas.xview_moveto(0)
        self.canvas.yview_moveto(0)
        self.redraw()

    def on_mousewheel(self, event):
        if getattr(event, "num", None) == 4:
            factor = ZOOM_STEP
        elif getattr(event, "num", None) == 5:
            factor = 1 / ZOOM_STEP
        else:
            delta = getattr(event, "delta", 0)
            if delta == 0:
                return
            factor = ZOOM_STEP if delta > 0 else 1 / ZOOM_STEP
        self._zoom_at(event.x, event.y, factor)

    def on_pan_start(self, event):
        self.canvas.scan_mark(event.x, event.y)

    def on_pan_drag(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    # ------------------------------------------------------------------
    # Undo / redo
    # ------------------------------------------------------------------

    def _snapshot(self) -> dict:
        """Full per-image mutable state, deep-copied so later in-place
        edits (e.g. dragging a box) can never leak back into a snapshot
        already sitting on the stack."""
        return {
            "decisions": dict(self.decisions),
            "class_overrides": dict(self.class_overrides),
            "manual_boxes": copy.deepcopy(self.manual_boxes),
            "queue_boxes": {item_key(it): list(it["box"]) for it in self.queue_items},
        }

    def _restore(self, snap: dict):
        self.decisions = dict(snap["decisions"])
        self.class_overrides = dict(snap["class_overrides"])
        self.manual_boxes = copy.deepcopy(snap["manual_boxes"])
        for it in self.queue_items:
            k = item_key(it)
            if k in snap["queue_boxes"]:
                it["box"] = list(snap["queue_boxes"][k])
        self.selected_key = None
        self.reposition_armed = None
        self._mark_dirty()
        self.redraw()

    def _push_undo(self):
        """Call BEFORE a mutation. Snapshots current state onto the undo
        stack and clears redo (a fresh action invalidates any redo
        history, same as every other undo/redo implementation)."""
        self.undo_stack.append(self._snapshot())
        if len(self.undo_stack) > UNDO_LIMIT:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def undo(self):
        if not self.undo_stack:
            return
        self.redo_stack.append(self._snapshot())
        self._restore(self.undo_stack.pop())
        self.streak = 0

    def redo(self):
        if not self.redo_stack:
            return
        self.undo_stack.append(self._snapshot())
        self._restore(self.redo_stack.pop())
        self.streak = 0

    # ------------------------------------------------------------------
    # Copy / paste (V14)
    # ------------------------------------------------------------------

    def _copy_selected(self):
        """Copies whichever box is currently selected (self.selected_key
        -- set by clicking/right-clicking/fast-mode-clicking a box, or by
        drawing/pasting a new one) onto the in-memory clipboard: its
        class and its box, in ORIGINAL IMAGE pixel coordinates. No-op if
        nothing is selected. A fresh copy always resets the paste
        cascade back to a single PASTE_OFFSET_PX step -- see
        _paste_clipboard()."""
        if self.selected_key is None:
            return
        kind, key = self.selected_key
        try:
            box = self._get_box(kind, key)
        except KeyError:
            return
        if kind == "queue":
            it = self._queue_item_by_key(key)
            if it is None:
                return
            cls = self.class_overrides.get(key, it["cls"])
        else:
            try:
                cls = self._find_manual(key)["cls"]
            except KeyError:
                return
        self.clipboard = {"cls": cls, "box": list(box)}
        self._paste_count = 0
        self._update_status()

    def _paste_clipboard(self):
        """Pastes the copied box as a brand-new MANUAL box (same class,
        same size), offset diagonally by PASTE_OFFSET_PX * (how many
        times Ctrl+V has been pressed since the last Ctrl+C) so repeated
        pastes cascade visibly across the image instead of stacking
        invisibly on the same spot. Goes through the exact same path a
        hand-drawn box does (_add_manual_box): auto-accepted, undoable,
        autosaved, and cross-frame synced if sync is on. No-op if
        nothing has been copied yet."""
        if self.clipboard is None:
            return
        self._paste_count += 1
        offset = PASTE_OFFSET_PX * self._paste_count
        x1, y1, x2, y2 = self.clipboard["box"]
        w = x2 - x1
        h = y2 - y1
        new_x1 = x1 + offset
        new_y1 = y1 + offset
        # Clamp so a cascade of pastes near an edge can't push the box
        # off the image entirely -- same clamping logic resize already
        # uses, just applied to a translated copy instead of a dragged
        # edge.
        new_x1 = max(0.0, min(new_x1, self.img_w - w))
        new_y1 = max(0.0, min(new_y1, self.img_h - h))
        new_box = [new_x1, new_y1, new_x1 + w, new_y1 + h]
        self._add_manual_box(self.clipboard["cls"], new_box)
        # The newly pasted box becomes the new selection so an immediate
        # Ctrl+C would copy IT, not the original -- consistent with
        # "selection follows what you just did" everywhere else (drawing,
        # fast-mode accept, etc.).
        if self.manual_boxes:
            self.selected_key = ("manual", self.manual_boxes[-1]["_id"])
            self.redraw()

    # ------------------------------------------------------------------
    # Cross-frame sync (V10 queue decisions / V11 manual boxes / V13
    # retraction / V16 motion-predicted chained matching / V17 deferred
    # to save-time)
    # ------------------------------------------------------------------

    def _get_image_dims_cached(self, image_key):
        dims = self._img_dims_cache.get(image_key)
        if dims is None:
            dims = gpl.get_image_dims(image_key)
            self._img_dims_cache[image_key] = dims
        return dims

    def _get_original_boxes_cached(self, image_key):
        """(fix) Same idea as _get_image_dims_cached, but for the
        real-dataset "orig, read-only" reference labels of an arbitrary
        (not-currently-loaded) neighbor image. _position_already_covered
        needs this: a neighbor frame can already carry a correct
        original ground-truth label for an object even when it has no
        queue item and no manual box for it (e.g. the original dataset
        labeled that frame directly instead of relying on the
        detector). Without checking this too, manual-box sync would
        stamp a fresh duplicate copy right on top of an already-correct
        original label the instant it propagated into that frame --
        which is exactly the kind of doubled labeling on one object
        that this cache exists to prevent."""
        boxes = self._orig_boxes_cache.get(image_key)
        if boxes is None:
            img_w, img_h = self._get_image_dims_cached(image_key)
            real_label_path = gpl.image_path_to_label_path(Path(image_key))
            boxes = load_original_labels(real_label_path, img_w, img_h)
            self._orig_boxes_cache[image_key] = boxes
        return boxes

    def _neighbor_image_keys(self, image_key):
        """Flat (unordered-by-distance) list of every neighbor frame
        within the sync window, in ascending-frame order. Still used by
        manual-box sync (_sync_propagate_manual_add) and
        _position_already_covered, which have no detector-observed
        position to derive a velocity from and so have no use for the
        directional/distance-ordered split below -- see V16 module
        note. Queue-decision sync (_apply_sync_batch) uses
        _ordered_neighbor_frames() instead."""
        seq_key = self.image_seq_key.get(image_key)
        if seq_key is None:
            return []
        frame = self.image_frame_num[image_key]
        window = self._sync_window_value()
        order = self.sequence_frames.get(seq_key, [])
        return [k for (fnum, k) in order if k != image_key and abs(fnum - frame) <= window]

    def _ordered_neighbor_frames(self, image_key):
        """(V16) Returns (backward, forward): two lists of
        (frame_num, image_key) tuples within the sync window, each
        ordered NEAREST-FIRST moving away from image_key's own frame --
        backward = descending frame number (toward earlier frames),
        forward = ascending frame number (toward later frames). Used by
        _predict_and_match_chain() to walk outward frame-by-frame in
        each direction independently, so a running position prediction
        can be built up and refined with each real observation instead
        of always testing every neighbor against the SOURCE frame's
        static box position."""
        seq_key = self.image_seq_key.get(image_key)
        if seq_key is None:
            return [], []
        frame = self.image_frame_num[image_key]
        window = self._sync_window_value()
        order = self.sequence_frames.get(seq_key, [])
        backward = [(fnum, k) for (fnum, k) in order
                    if k != image_key and 0 < frame - fnum <= window]
        forward = [(fnum, k) for (fnum, k) in order
                   if k != image_key and 0 < fnum - frame <= window]
        backward.sort(key=lambda t: t[0], reverse=True)   # nearest first
        forward.sort(key=lambda t: t[0])                  # nearest first
        return backward, forward

    def _queue_item_by_key(self, key):
        for it in self.queue_items:
            if item_key(it) == key:
                return it
        return None

    def _resolve_neighbor_class(self, image_key, it):
        """(V13 fix D) A neighbor candidate's EFFECTIVE class, honoring
        any class override already saved for it on that image -- not
        just its raw queue-detected class. Without this, a neighbor box
        you'd already relabelled via "Change Class" could be missed as
        a sync target, or wrongly matched under the wrong class."""
        overrides = self.progress.get(f"__override__{image_key}", {})
        return overrides.get(item_key(it), it["cls"])

    def _find_matching_queue_item(self, image_key, cls, box, debug_log=None):
        """Same-class queue box in `image_key` whose center is within
        SYNC_POS_TOLERANCE_FRAC of the image diagonal from `box`'s
        center. `box` is normally the source item's own position for a
        1-frame-out neighbor, but as of V16 the caller
        (_predict_and_match_chain) may instead pass a MOTION-PREDICTED
        position for frames further out -- this function itself doesn't
        care which, it just searches near whatever box it's given. Uses
        each candidate's CURRENT (possibly already-synced or
        hand-repositioned) box from progress.json, not its original
        detector box, and its CURRENT effective class (honoring any
        class override -- V13 fix D). Returns the nearest match, or
        None. If `debug_log` (a list) is passed, appends a one-line
        explanation of what was found/rejected and why."""
        items = self.by_image_full.get(image_key, [])
        if not items:
            if debug_log is not None:
                debug_log.append(f"    {Path(image_key).name}: no queue items at all")
            return None
        try:
            img_w, img_h = self._get_image_dims_cached(image_key)
        except Exception as e:
            if debug_log is not None:
                debug_log.append(f"    {Path(image_key).name}: couldn't read image dims ({e})")
            return None
        diag = (img_w ** 2 + img_h ** 2) ** 0.5
        tol = diag * SYNC_POS_TOLERANCE_FRAC
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        best, best_d = None, None
        same_class_count = 0
        nearest_wrong_dist = None
        for it in items:
            eff_cls = self._resolve_neighbor_class(image_key, it)
            if eff_cls != cls:
                continue
            same_class_count += 1
            k = item_key(it)
            saved = self.progress.get(k, {})
            bb = saved.get("box", it["box"])
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            d = ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5
            if d <= tol and (best_d is None or d < best_d):
                best_d, best = d, it
            elif nearest_wrong_dist is None or d < nearest_wrong_dist:
                nearest_wrong_dist = d
        if debug_log is not None:
            if best is not None:
                debug_log.append(
                    f"    {Path(image_key).name}: MATCH '{cls}' at dist={best_d:.1f}px (tol={tol:.1f}px)")
            elif same_class_count == 0:
                debug_log.append(
                    f"    {Path(image_key).name}: no '{cls}' boxes in queue here at all")
            else:
                debug_log.append(
                    f"    {Path(image_key).name}: {same_class_count} '{cls}' box(es) here, "
                    f"nearest is {nearest_wrong_dist:.1f}px away (tol={tol:.1f}px) -- too far, no match")
        return best

    def _predict_and_match_chain(self, source_frame_num, source_box, cls,
                                  frame_list, debug_log=None):
        """(V16) Walks `frame_list` -- a nearest-first ordered
        [(frame_num, image_key), ...] in ONE direction from the source
        frame (see _ordered_neighbor_frames) -- and, at each step,
        searches for a same-class pending queue box near a PREDICTED
        position rather than always the source box's raw position.

        The prediction starts as the source box itself. The first real
        observation (a matched box, whether or not its decision is
        still pending -- see below) just confirms/refines the current
        position. Once there have been TWO real observations, a
        per-frame velocity is derived from them (position delta /
        frame-number delta) and used to extrapolate the predicted
        position for the next frame out; that velocity is refreshed on
        every subsequent real observation. If a frame in between has no
        matching candidate at all (e.g. it failed the review queue's
        own confidence gate), the predictor simply coasts on the last
        known velocity rather than resetting -- so a short gap doesn't
        throw off the whole chain.

        An observation used to update the trajectory does NOT have to
        be "pending" -- an already-decided neighbor is still real
        evidence of where the object actually is, so it's used to keep
        the prediction accurate for frames further out even though its
        own decision is left untouched (never overwritten). Only
        PENDING matches are returned for the caller to actually decide.

        Returns a list of (image_key, matched_item) for every frame
        where a still-pending match was found within tolerance -- the
        caller applies the actual decision."""
        results = []
        last_frame_num, last_box = source_frame_num, source_box
        velocity = None  # [dx1, dy1, dx2, dy2] per frame-number step

        for fnum, nk in frame_list:
            gap = fnum - last_frame_num
            if velocity is not None and gap != 0:
                predicted = [last_box[i] + velocity[i] * gap for i in range(4)]
            else:
                predicted = last_box

            if nk in self.completed_set:
                if debug_log is not None:
                    debug_log.append(f"    {Path(nk).name}: skipped (already in completed list)")
                continue

            match = self._find_matching_queue_item(nk, cls, predicted, debug_log=debug_log)
            if match is None:
                continue  # no observation here -- keep predicting from the last real one

            mk = item_key(match)
            matched_box = self.progress.get(mk, {}).get("box", match["box"])

            # Update the trajectory from this real observation regardless
            # of whether we're allowed to act on its decision -- it's
            # still genuine evidence of where the object is, and using it
            # keeps predictions further down this chain accurate.
            if gap != 0:
                velocity = [(matched_box[i] - last_box[i]) / gap for i in range(4)]
            last_frame_num, last_box = fnum, matched_box

            cur = self.progress.get(mk, {}).get("decision", "pending")
            if cur != "pending":
                if debug_log is not None:
                    debug_log.append(
                        f"    {Path(nk).name}: match found but already decided ({cur}) -- "
                        f"used as a trajectory point only, decision left alone")
                continue

            results.append((nk, match))

        return results

    def _position_already_covered(self, image_key, cls, box) -> bool:
        """(V11) True if a queue box OR manual box of the same
        EFFECTIVE class (honoring class overrides -- V13 fix D) already
        sits within the sync position tolerance of `box` in
        `image_key`. Used to stop manual-box sync from stacking a
        duplicate on top of something already there -- a queue box you
        already accepted, or a manual box drawn independently in that
        frame.

        (fix) ALSO checks that neighbor's real-dataset "orig, read-only"
        labels (see _get_original_boxes_cached). A manual box exists to
        cover an object the detector missed on ONE frame -- it doesn't
        mean every neighbor frame is missing it too. Plenty of neighbor
        frames may already carry a correct original ground-truth label
        for that same object even though they have no queue item or
        manual box for it at all (nothing for the earlier two checks to
        find). Without this, syncing stamped a fresh duplicate copy
        right on top of an already-correctly-labeled object every time
        that happened -- the doubled-labeling-on-one-object symptom."""
        try:
            img_w, img_h = self._get_image_dims_cached(image_key)
        except Exception:
            return False
        diag = (img_w ** 2 + img_h ** 2) ** 0.5
        tol = diag * SYNC_POS_TOLERANCE_FRAC
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

        for it in self.by_image_full.get(image_key, []):
            eff_cls = self._resolve_neighbor_class(image_key, it)
            if eff_cls != cls:
                continue
            saved = self.progress.get(item_key(it), {})
            bb = saved.get("box", it["box"])
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            if ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5 <= tol:
                return True

        for mb in self.progress.get(f"__manual__{image_key}", []):
            if mb["cls"] != cls:
                continue
            bb = mb["box"]
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            if ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5 <= tol:
                return True

        try:
            orig_boxes = self._get_original_boxes_cached(image_key)
        except Exception:
            orig_boxes = []
        for ob in orig_boxes:
            if ob["cls"] != cls:
                continue
            bb = ob["box"]
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            if ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5 <= tol:
                return True
        return False

    def _sync_after_decision_change(self, source_key, prev_state, new_state):
        """(V13 fix B) Called after a single queue-box decision change
        on the CURRENT image (Accept/Reject/Delete/Reset). Two halves:

          1. RETRACT: if the box was previously in a state that had
             been synced forward (accept/reject/deleted), undo exactly
             those synced copies -- but only the ones still sitting in
             the state they were synced to (never a copy the user has
             since manually re-decided by hand).
          2. FORWARD: if the box's new state is accept/reject/deleted,
             sync it forward the same way V10 always did.

        This is what makes "I changed my mind on this box" propagate
        the same way the original decision did, instead of leaving
        stale synced copies in neighbor frames forever.

        (V17) This now runs from inside _drain_pending_sync() -- called
        from _flush(), not directly from the click handler -- so it no
        longer needs to force its own flush at the end (the caller,
        _flush(), is already in the middle of writing everything to
        disk)."""
        if not self.sync_enabled.get():
            return
        it = self._queue_item_by_key(source_key)
        if it is None:
            return

        retracted = set()
        if prev_state in ("accept", "reject", "deleted"):
            retracted = self._retract_sync_children(source_key, prev_state)

        forward_touched = set()
        if new_state in ("accept", "reject", "deleted"):
            cls = self.class_overrides.get(source_key, it["cls"])
            box = it["box"]
            forward_touched = self._apply_sync_batch([(source_key, cls, box, new_state)])

        extra = retracted - forward_touched
        if extra:
            for nk in extra:
                self._write_reviewed_labels_for_image(nk)
                self._recompute_completed_for_image(nk)
            save_json(self.progress_path, self.progress)
            save_completed(self.completed_path, self.completed_set)
            if not forward_touched:
                self.sync_status_var.set(f"Retracted sync on {len(extra)} frame(s)")

    def _retract_sync_children(self, source_key, prev_state):
        """(V13 fix B) Reverts every neighbor queue box this source item
        previously auto-synced (while it was in `prev_state`) back to
        pending. Only retracts a neighbor if it (a) was synced FROM
        this exact source item and (b) still holds the state it was
        synced to -- if the user has since manually re-decided that
        specific neighbor box, it's left alone. Returns the set of
        touched neighbor image_keys (caller is responsible for writing
        their reviewed-labels output and recomputing their completed
        status -- this only mutates self.progress in memory)."""
        entry = self.progress.get(source_key)
        if not entry:
            return set()
        children = entry.get("synced_children", [])
        if not children:
            return set()
        touched = set()
        for mk in children:
            child = self.progress.get(mk)
            if (child and child.get("synced_from_key") == source_key
                    and child.get("decision") == prev_state):
                child["decision"] = "pending"
                child.pop("synced_from", None)
                child.pop("synced_from_key", None)
                if child.get("image"):
                    touched.add(child["image"])
        # Cleared unconditionally: any child NOT retracted above has
        # diverged (manually re-decided since it was synced), so we
        # stop tracking it as "ours" rather than risk clobbering a
        # manual edit on a future retraction.
        entry["synced_children"] = []
        return touched

    def _apply_sync_batch(self, sources):
        """(V16: motion-predicted chain, see _predict_and_match_chain)
        sources: list of (source_key, cls, box, state) tuples
        describing decisions just made on QUEUE boxes on the image
        currently on screen. For each, walks outward from the source
        frame in both directions (backward, forward), nearest-frame-
        first, maintaining a running position prediction, and for each
        neighbor finds a same-EFFECTIVE-class queue box near that
        prediction that is STILL PENDING there, applying the same
        decision. Never touches a neighbor box that already has a
        decision (yours or a previous sync's -- though an already-
        decided match still refines the trajectory, see
        _predict_and_match_chain), and never reopens an image already
        in the completed list. Geometry is never touched, only the
        decision. Records which neighbor entries each source created
        (source's "synced_children") so a later change to that SAME
        source can cleanly retract them (V13 fix B). Returns the set of
        touched neighbor image_keys."""
        touched_images = set()
        if not self.sync_enabled.get():
            return touched_images
        verbose = self.sync_debug.get()
        applied = []

        seq_key = self.image_seq_key.get(self.image_key)
        if seq_key is None:
            self.sync_status_var.set(
                "Sync: couldn't parse a sequence/frame from this image's path -- see console")
            if verbose:
                print(f"[sync] '{self.image_key}': not parseable, no neighbors possible.")
            return touched_images

        backward, forward = self._ordered_neighbor_frames(self.image_key)
        if not backward and not forward:
            self.sync_status_var.set(
                f"Sync: 0 neighbor frame(s) found for this image (seq={seq_key!r})")
            if verbose:
                print(f"[sync] '{self.image_key}' seq={seq_key!r} frame="
                      f"{self.image_frame_num.get(self.image_key)}: 0 neighbors within "
                      f"\u00b1{self._sync_window_value()} frames.")
            return touched_images

        if verbose:
            print(f"[sync] '{self.image_key}' seq={seq_key!r} frame="
                  f"{self.image_frame_num.get(self.image_key)}: "
                  f"{len(backward)} backward + {len(forward)} forward neighbor(s) "
                  f"(motion-predicted chain)")

        source_frame_num = self.image_frame_num.get(self.image_key)

        for source_key, cls, box, state in sources:
            debug_log = [] if verbose else None
            source_entry = self.progress.setdefault(source_key, {})
            children = list(source_entry.get("synced_children", []))

            chain_results = []
            if source_frame_num is not None:
                chain_results.extend(self._predict_and_match_chain(
                    source_frame_num, box, cls, backward, debug_log=debug_log))
                chain_results.extend(self._predict_and_match_chain(
                    source_frame_num, box, cls, forward, debug_log=debug_log))

            for nk, match in chain_results:
                mk = item_key(match)
                self.progress[mk] = {
                    "decision": state,
                    "image": nk,
                    "cls": match["cls"],
                    "box": self.progress.get(mk, {}).get("box", match["box"]),
                    "synced_from": self.image_key,
                    "synced_from_key": source_key,
                }
                applied.append({"kind": "queue", "image_key": nk, "match_key": mk,
                                 "source_key": source_key})
                touched_images.add(nk)
                if mk not in children:
                    children.append(mk)
            source_entry["synced_children"] = children
            if debug_log:
                print(f"[sync] decision={state!r} cls={cls!r} box={[round(v) for v in box]} "
                      f"(motion-predicted chain):")
                for line in debug_log:
                    print(line)

        if not applied:
            self.sync_status_var.set(
                "Sync: 0 applied (no matching class/position found along the chain)"
                if not verbose else
                "Sync: 0 applied -- see console for details")
            return touched_images

        for nk in touched_images:
            self._write_reviewed_labels_for_image(nk)
            self._recompute_completed_for_image(nk)
        save_json(self.progress_path, self.progress)
        save_completed(self.completed_path, self.completed_set)
        self._last_sync_batch = applied
        self.sync_status_var.set(
            f"Synced {len(applied)} box(es) across {len(touched_images)} nearby frame(s)")
        return touched_images

    def _sync_propagate_manual_add(self, mb):
        """(V11) Mirrors the queue-decision sync, but for a manual box
        that was just Accepted (V12: this now happens automatically the
        moment the box is drawn -- or pasted, V14). There's no detector
        candidate to match against in a neighbor frame -- so instead of
        matching, this DIRECTLY CREATES a copy of the box (same class,
        same pixel position) in each in-window neighbor frame, unless
        that position is already covered by something (see
        _position_already_covered) or the neighbor is already
        completed. Every copy created is tagged "synced_from" and
        recorded onto `mb["_sync_children"]` so a later Reject/Reset/
        Delete of THIS box can cleanly remove exactly the copies it
        made -- never a box drawn independently elsewhere.

        (V16) NOT motion-predicted, unlike queue-decision sync -- a
        manual box has no detector candidate anywhere to observe a real
        position from in a neighbor frame, so there's no second
        observation to derive a velocity from. It still stamps a
        same-position copy in every neighbor, same as before V16.

        (V17) Now runs from inside _drain_pending_sync() (called from
        _flush()), so it no longer needs to force its own flush at the
        end -- the caller is already mid-flush."""
        if not self.sync_enabled.get():
            return
        neighbors = self._neighbor_image_keys(self.image_key)
        if not neighbors:
            return

        applied = []
        for nk in neighbors:
            if nk in self.completed_set:
                continue
            if self._position_already_covered(nk, mb["cls"], mb["box"]):
                continue
            existing = list(self.progress.get(f"__manual__{nk}", []))
            next_id = max((m.get("_id", 0) for m in existing), default=-1) + 1
            new_mb = {"cls": mb["cls"], "box": list(mb["box"]), "_id": next_id,
                      "decision": "accepted", "synced_from": self.image_key}
            existing.append(new_mb)
            self.progress[f"__manual__{nk}"] = existing
            mb.setdefault("_sync_children", []).append((nk, next_id))
            applied.append({"kind": "manual_add", "image_key": nk, "manual_id": next_id})
            self._write_reviewed_labels_for_image(nk)
            self._recompute_completed_for_image(nk)

        if applied:
            save_json(self.progress_path, self.progress)
            save_completed(self.completed_path, self.completed_set)
            self._last_sync_batch = applied
            self.sync_status_var.set(f"Synced manual box to {len(applied)} nearby frame(s)")

    def _sync_propagate_manual_remove(self, mb):
        """(V11) Reverses _sync_propagate_manual_add: removes every
        synced copy this specific box previously created (tracked in
        its own "_sync_children"), wherever they ended up. Only ever
        touches boxes that trace back to THIS box.

        (V17) Now runs from inside _drain_pending_sync() (called from
        _flush()), so it no longer needs to force its own flush at the
        end -- the caller is already mid-flush."""
        children = mb.pop("_sync_children", [])
        if not children:
            return
        touched = set()
        for nk, manual_id in children:
            existing = self.progress.get(f"__manual__{nk}", [])
            new_list = [m for m in existing if m.get("_id") != manual_id]
            if len(new_list) != len(existing):
                self.progress[f"__manual__{nk}"] = new_list
                touched.add(nk)
        for nk in touched:
            self._write_reviewed_labels_for_image(nk)
            self._recompute_completed_for_image(nk)
        if touched:
            save_json(self.progress_path, self.progress)
            save_completed(self.completed_path, self.completed_set)
            self.sync_status_var.set(f"Removed synced manual box from {len(touched)} frame(s)")

    def undo_last_sync(self):
        """Reverts exactly the last auto-applied sync batch (a queue-
        decision sync, a manual-box sync, or a mix from one bulk
        action) back out, across every frame it touched -- not the
        general per-image undo stack. Also cleans up the source item's
        "synced_children" bookkeeping (V13) so it can't keep pointing
        at a match_key that no longer reflects a sync this source is
        responsible for."""
        batch = self._last_sync_batch
        if not batch:
            messagebox.showinfo("Undo Last Sync", "No sync batch to undo.")
            return
        touched_images = set()
        for entry in batch:
            kind = entry.get("kind", "queue")
            if kind == "queue":
                mk = entry["match_key"]
                if mk in self.progress:
                    self.progress[mk]["decision"] = "pending"
                    self.progress[mk].pop("synced_from", None)
                    self.progress[mk].pop("synced_from_key", None)
                src_key = entry.get("source_key")
                if src_key and src_key in self.progress:
                    children = self.progress[src_key].get("synced_children", [])
                    if mk in children:
                        children.remove(mk)
                touched_images.add(entry["image_key"])
            elif kind == "manual_add":
                nk = entry["image_key"]
                manual_id = entry["manual_id"]
                existing = self.progress.get(f"__manual__{nk}", [])
                self.progress[f"__manual__{nk}"] = [m for m in existing if m.get("_id") != manual_id]
                touched_images.add(nk)
        for nk in touched_images:
            self._write_reviewed_labels_for_image(nk)
            self._recompute_completed_for_image(nk)
        save_json(self.progress_path, self.progress)
        save_completed(self.completed_path, self.completed_set)
        self._last_sync_batch = []
        self.sync_status_var.set(f"Reverted sync on {len(touched_images)} frame(s)")
        if self.image_key in touched_images:
            self._load_image(self.idx)
        else:
            self._update_status()

    # ------------------------------------------------------------------
    # Saving / autosave / completed-list bookkeeping
    # ------------------------------------------------------------------

    def accepted_items(self) -> list[dict]:
        out = []
        for it in self.queue_items:
            k = item_key(it)
            if self.decisions[k] == "accept":
                cls = self.class_overrides.get(k, it["cls"])
                out.append({"cls": cls, "box": it["box"]})
        # V11: a manual box only reaches output once explicitly Accepted
        # (V12: this happens automatically the moment it's drawn or
        # pasted, V14).
        out.extend({"cls": mb["cls"], "box": mb["box"]} for mb in self.manual_boxes
                   if mb.get("decision", "pending") == "accepted")
        return out

    def _write_reviewed_labels(self):
        accepted = self.accepted_items()
        img_path = Path(self.image_key)
        real_label_path = gpl.image_path_to_label_path(img_path)
        reviewed_label_path = gpl.mirror_under_pseudo_root(
            real_label_path, self.source_name, self.reviewed_root)

        if not accepted:
            if reviewed_label_path.exists():
                reviewed_label_path.unlink()
            return

        w, h = gpl.get_image_dims(self.image_key)
        lines = [gpl.box_to_yolo_line(a["cls"], _clamp_box_to_image(a["box"], w, h), w, h)
                 for a in accepted]
        reviewed_label_path.parent.mkdir(parents=True, exist_ok=True)
        reviewed_label_path.write_text("\n".join(lines) + "\n")

    def _accepted_items_for_image(self, image_key):
        """Same as accepted_items(), but computed from review_progress.json
        for an arbitrary (not-currently-loaded) image -- used by cross-
        frame sync to update a neighbor frame's output without loading
        it into the UI."""
        items = self.by_image_full.get(image_key, [])
        overrides = self.progress.get(f"__override__{image_key}", {})
        manual = self.progress.get(f"__manual__{image_key}", [])
        out = []
        for it in items:
            k = item_key(it)
            saved = self.progress.get(k, {})
            if saved.get("decision") == "accept":
                cls = overrides.get(k, it["cls"])
                box = saved.get("box", it["box"])
                out.append({"cls": cls, "box": box})
        # Legacy (pre-V11) manual boxes have no "decision" -- default to
        # "accepted" so old output doesn't silently disappear.
        out.extend({"cls": mb["cls"], "box": mb["box"]} for mb in manual
                   if mb.get("decision", "accepted") == "accepted")
        return out

    def _write_reviewed_labels_for_image(self, image_key):
        """_write_reviewed_labels(), generalized to any image -- used
        when a sync auto-applies a decision to a neighbor frame that
        isn't the one currently on screen."""
        accepted = self._accepted_items_for_image(image_key)
        img_path = Path(image_key)
        real_label_path = gpl.image_path_to_label_path(img_path)
        items = self.by_image_full.get(image_key, [])
        source_name = items[0]["source"] if items else self.source_name
        reviewed_label_path = gpl.mirror_under_pseudo_root(
            real_label_path, source_name, self.reviewed_root)

        if not accepted:
            if reviewed_label_path.exists():
                reviewed_label_path.unlink()
            return

        w, h = self._get_image_dims_cached(image_key)
        lines = [gpl.box_to_yolo_line(a["cls"], _clamp_box_to_image(a["box"], w, h), w, h)
                 for a in accepted]
        reviewed_label_path.parent.mkdir(parents=True, exist_ok=True)
        reviewed_label_path.write_text("\n".join(lines) + "\n")

    def _update_completed_list(self):
        """An image counts as 'done' once every queue item on it --
        across the FULL unfiltered queue, not just whatever this
        session's --source/--cls filter shows (V13 fix A) -- has a
        real decision (nothing pending -- "deleted" counts as decided,
        same as "accept"/"reject") AND (V11) every manual box on it is
        non-pending too. Moves the image into (or out of, if something
        got reset to pending) review_completed.json -- a separate,
        explicit list a second reviewer can open with --qa, or (V12)
        with the in-app "Switch to QA / Completed" toggle, instead of
        everything living undifferentiated in progress.json."""
        full_items = self.by_image_full.get(self.image_key, [])
        queue_done = all(
            self.progress.get(item_key(it), {}).get("decision", "pending") != "pending"
            for it in full_items)
        manual_done = all(mb.get("decision", "pending") != "pending"
                           for mb in self.manual_boxes)
        fully_decided = queue_done and manual_done
        changed = False
        if fully_decided and self.image_key not in self.completed_set:
            self.completed_set.add(self.image_key)
            changed = True
        elif not fully_decided and self.image_key in self.completed_set:
            self.completed_set.discard(self.image_key)
            changed = True
        if changed:
            save_completed(self.completed_path, self.completed_set)
        return fully_decided

    def _recompute_completed_for_image(self, image_key):
        """_update_completed_list(), generalized to any image -- used
        for neighbor frames touched by a sync. Uses the FULL unfiltered
        queue (V13 fix A), same reasoning as _update_completed_list.
        Does NOT write the completed-list file itself (caller batches
        that after touching possibly several images)."""
        items = self.by_image_full.get(image_key, [])
        manual = self.progress.get(f"__manual__{image_key}", [])
        fully_decided = (
            all(self.progress.get(item_key(it), {}).get("decision", "pending") != "pending"
                for it in items)
            and all(mb.get("decision", "accepted") != "pending" for mb in manual))
        if fully_decided and image_key not in self.completed_set:
            self.completed_set.add(image_key)
        elif not fully_decided and image_key in self.completed_set:
            self.completed_set.discard(image_key)

    def _drain_pending_sync(self):
        """(V17) Runs every deferred sync action queued since the last
        flush, in the order they were queued, then clears the queue.
        Each queued callable is exactly the call that used to fire
        immediately on click (_sync_after_decision_change /
        _sync_propagate_manual_add / _sync_propagate_manual_remove /
        _apply_sync_batch) -- their own retract-then-reapply logic
        already makes each one safe to run from whatever the current
        state is, regardless of how many other edits happened first.
        Called first thing in _flush() so a whole burst of edits on one
        image collapses into a single sync pass per save instead of one
        sync call per click."""
        if not self._pending_sync_actions:
            return
        actions, self._pending_sync_actions = self._pending_sync_actions, []
        for action in actions:
            action()

    def _flush(self):
        """Writes everything for the current image to disk: per-item
        decisions + current box position into progress.json, the
        accepted-boxes label file into pseudo_labels_reviewed/, and
        updates the completed-images list.

        (V17) Drains any deferred cross-frame sync actions first, so
        sync always runs as part of -- and immediately before -- the
        same save that commits the decisions which triggered it.

        (V18) Also refreshes the sidebar tree, since this is the one
        place that always runs right after the completed-images list
        could have changed (this image's own status, or -- via
        cross-frame sync -- one or more neighbor images')."""
        self._drain_pending_sync()
        for it in self.queue_items:
            k = item_key(it)
            existing = self.progress.get(k, {})
            entry = {
                "decision": self.decisions[k],
                "image": self.image_key,
                "cls": it["cls"],
                "box": it["box"],
                # Stable detection-identity box, saved SEPARATELY from the
                # (possibly hand-edited) "box" above. reconcile_progress()
                # matches on this, not on "box" -- see the note there for
                # why matching on the edited box silently orphaned exactly
                # the boxes you'd manually repositioned/resized.
                "orig_box": list(it["_orig_box"]),
            }
            if k in self.synced_from_map:
                entry["synced_from"] = self.synced_from_map[k]
            # (V13 fix B) Preserve this item's sync bookkeeping across a
            # flush -- _flush() rebuilds the entry from scratch every
            # time, so without this a routine autosave would silently
            # wipe out "synced_children", breaking retraction the next
            # time this box's decision changes.
            if "synced_children" in existing:
                entry["synced_children"] = existing["synced_children"]
            self.progress[k] = entry
        self.progress[f"__manual__{self.image_key}"] = self.manual_boxes
        self.progress[f"__override__{self.image_key}"] = self.class_overrides
        self._write_reviewed_labels()
        save_json(self.progress_path, self.progress)
        fully_decided = self._update_completed_list()
        if hasattr(self, "sidebar_tree"):
            self._refresh_sidebar()
        return fully_decided

    def _mark_dirty(self):
        """Call after any edit. Schedules an autosave a short moment
        from now -- debounced so a burst of clicks (accepting several
        boxes in a row) doesn't write the file after every single one,
        while still guaranteeing nothing survives more than
        AUTOSAVE_DEBOUNCE_MS unsaved if the app were to crash. Focus-
        loss (Alt-Tab, sleep, etc.) also force-flushes immediately --
        see the <FocusOut> binding in _build_ui -- so in practice the
        debounce window is the only real exposure left."""
        self.dirty = True
        if self._autosave_after_id is not None:
            self.root.after_cancel(self._autosave_after_id)
        self._autosave_after_id = self.root.after(AUTOSAVE_DEBOUNCE_MS, self._do_autosave)

    def _do_autosave(self):
        self._autosave_after_id = None
        self._flush()
        self.dirty = False
        self._update_status()

    def _flush_now_if_dirty(self):
        """Used when navigating away, closing, losing window focus, or
        (V13) right after a decision change that had a cross-frame sync
        side-effect -- don't wait out the debounce timer, write
        immediately."""
        if self._autosave_after_id is not None:
            self.root.after_cancel(self._autosave_after_id)
            self._autosave_after_id = None
        if self.dirty:
            self._flush()
            self.dirty = False

    def save_current(self):
        self._flush_now_if_dirty()
        self._update_status()

    def _maybe_auto_advance(self):
        """Called after any decision changes. If auto-advance is on and
        the image just became fully decided, queue a move to the next
        image after a short delay (so the last change is visibly on
        screen for a beat before it jumps). Uses the FULL unfiltered
        queue (V13 fix A) so auto-advance doesn't fire early just
        because everything IN THE CURRENT FILTER happens to be
        decided."""
        if not self.auto_advance.get():
            return
        full_items = self.by_image_full.get(self.image_key, [])
        queue_done = all(
            self.progress.get(item_key(it), {}).get("decision", self.decisions.get(item_key(it), "pending")) != "pending"
            if item_key(it) not in self.decisions
            else self.decisions[item_key(it)] != "pending"
            for it in full_items)
        manual_done = all(mb.get("decision", "pending") != "pending"
                           for mb in self.manual_boxes)
        fully_decided = queue_done and manual_done
        if fully_decided and self._autoadvance_after_id is None:
            self._autoadvance_after_id = self.root.after(
                AUTO_ADVANCE_DELAY_MS, self._do_auto_advance)

    def _do_auto_advance(self):
        self._autoadvance_after_id = None
        if self.idx + 1 < len(self.image_keys):
            self.go_next()

    # ------------------------------------------------------------------
    # Bulk actions
    # ------------------------------------------------------------------

    def bulk_accept_pending(self):
        pending_keys = [k for k, v in self.decisions.items() if v == "pending"]
        if not pending_keys:
            return
        if not messagebox.askyesno(
                "Accept all pending",
                f"Accept all {len(pending_keys)} pending box(es) in this image?"):
            return
        self._push_undo()
        sources = []
        for k in pending_keys:
            self.decisions[k] = "accept"
            it = self._queue_item_by_key(k)
            if it is not None:
                cls = self.class_overrides.get(k, it["cls"])
                sources.append((k, cls, it["box"], "accept"))
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()
        # (V17) Deferred: queued for the next _flush() rather than
        # applied immediately.
        self._pending_sync_actions.append(lambda s=sources: self._apply_sync_batch(s))

    def bulk_reject_pending(self):
        pending_keys = [k for k, v in self.decisions.items() if v == "pending"]
        if not pending_keys:
            return
        if not messagebox.askyesno(
                "Reject all pending",
                f"Reject all {len(pending_keys)} pending box(es) in this image?"):
            return
        self._push_undo()
        sources = []
        for k in pending_keys:
            self.decisions[k] = "reject"
            it = self._queue_item_by_key(k)
            if it is not None:
                cls = self.class_overrides.get(k, it["cls"])
                sources.append((k, cls, it["box"], "reject"))
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()
        # (V17) Deferred: queued for the next _flush() rather than
        # applied immediately.
        self._pending_sync_actions.append(lambda s=sources: self._apply_sync_batch(s))

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _queue_item_visible(self, it) -> bool:
        k = item_key(it)
        if self.decisions[k] == "deleted" and not self.show_deleted.get():
            return False
        if self.show_all_queue.get():
            return True
        votes = it.get("votes", {})
        return any(self.model_vars[m].get() and votes.get(m) is not None for m in MODEL_KEYS)

    def _manual_box_visible(self, mb) -> bool:
        """(V11) Rejected manual boxes hide by default, same as
        "deleted" queue boxes -- both reuse the "Show Deleted/Rejected"
        toggle so a rejected box is never permanently gone from view,
        just tucked away."""
        if mb.get("decision", "pending") == "rejected" and not self.show_deleted.get():
            return False
        return True

    def _box_is_resizable(self, kind, key) -> bool:
        """Manual boxes are always resizable. A queue box is resizable
        only while explicitly armed for this one drag (same gate as
        moving it -- see V7 note in the module docstring)."""
        if kind == "manual":
            return True
        return self.reposition_armed == (kind, key)

    def _draw_handles(self, box, kind, key):
        """Draws the 8 resize handles for a selected, resizable box and
        registers them in self.handle_map so on_mouse_down can hit-test
        them. Only called for the currently-selected box, and only if
        it's actually resizable right now (see _box_is_resizable)."""
        x1, y1, x2, y2 = [v * self.scale for v in box]
        xs = {-1: x1, 0: (x1 + x2) / 2, 1: x2}
        ys = {-1: y1, 0: (y1 + y2) / 2, 1: y2}
        half = HANDLE_SIZE / 2
        for name, hx, hy in HANDLES:
            cx, cy = xs[hx], ys[hy]
            hid = self.canvas.create_rectangle(
                cx - half, cy - half, cx + half, cy + half,
                fill=COLOR_HANDLE, outline=COLOR_HANDLE_OUTLINE, width=1)
            self.handle_map[hid] = (kind, key, name)

    def _draw_box(self, box, color, label, key=None, kind=None, dash=None):
        x1, y1, x2, y2 = [v * self.scale for v in box]
        selected = kind is not None and self.selected_key == (kind, key)
        armed = self.reposition_armed == (kind, key)
        outline = COLOR_ARMED if armed else color
        rect_id = self.canvas.create_rectangle(
            x1, y1, x2, y2, outline=outline, width=3 if (selected or armed) else 2, dash=dash)
        suffix = "  [drag to move now]" if armed else ""
        text_id = self.canvas.create_text(
            x1 + 2, max(10, y1 - 8), text=label + suffix, fill=outline, anchor="sw",
            font=("Consolas", 8))
        if kind is not None:
            self.item_map[rect_id] = (kind, key)
            self.item_map[text_id] = (kind, key)
            if selected and self._box_is_resizable(kind, key):
                self._draw_handles(box, kind, key)

    def _current_covered_by_review(self, cls, box) -> bool:
        """(fix) True if the CURRENT image already has an accepted queue
        item or accepted manual box of the same class sitting within the
        sync position tolerance of `box`. Used to hide an "orig,
        read-only" reference box once it's redundant with a decision
        the review tool itself already made for that same object.

        This is exactly what happens once a manual/accepted box has been
        through --apply: that run merges the box straight into the real
        dataset label file, and the NEXT time this image is opened,
        load_original_labels() reads that same merged file back in as
        "original" reference content -- so the same object then gets
        drawn twice: once as the still-live accepted decision (colored
        by its state) and once as a magenta "orig, read-only" box
        sitting almost exactly on top of it. Nothing is actually
        double-counted in the output (accepted_items() never reads
        original_boxes at all), but it LOOKS like a duplicate label on
        screen, and it's what prompted the original double-labeling bug
        report. Checked against the LIVE in-memory decisions/manual
        boxes for the image currently on screen, not the saved
        progress.json snapshot, so it reflects edits made this instant
        even before the next autosave."""
        try:
            img_w, img_h = self.img_w, self.img_h
        except Exception:
            return False
        diag = (img_w ** 2 + img_h ** 2) ** 0.5
        tol = diag * SYNC_POS_TOLERANCE_FRAC
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

        for it in self.queue_items:
            k = item_key(it)
            if self.decisions.get(k) != "accept":
                continue
            eff_cls = self.class_overrides.get(k, it["cls"])
            if eff_cls != cls:
                continue
            bb = it["box"]
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            if ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5 <= tol:
                return True

        for mb in self.manual_boxes:
            if mb.get("decision", "pending") != "accepted":
                continue
            if mb["cls"] != cls:
                continue
            bb = mb["box"]
            bcx, bcy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            if ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5 <= tol:
                return True
        return False

    def redraw(self):
        self.canvas.delete("all")
        self.item_map = {}
        self.handle_map = {}
        self.canvas.create_image(0, 0, anchor="nw", image=self.tk_img, tags="bg")

        # Original real-dataset labels: reference-only. Note these are
        # NEVER passed kind=/key= below, so they never enter self.item_map
        # -- there is no code path that lets a mouse click or drag hit
        # one. That's what guarantees they can't be moved OR deleted by
        # mistake -- the V8 delete feature only touches queue items,
        # which are the only boxes that ever get registered here.
        #
        # (fix) Skip drawing one that's already covered by a decision
        # the review tool itself has accepted for this image -- see
        # _current_covered_by_review. Tracked in self._orig_hidden_count
        # purely for the status bar, so hiding these doesn't look like
        # original labels silently vanished.
        self._orig_hidden_count = 0
        if self.show_original.get():
            for ob in self.original_boxes:
                if self._current_covered_by_review(ob["cls"], ob["box"]):
                    self._orig_hidden_count += 1
                    continue
                self._draw_box(ob["box"], COLOR_ORIGINAL, f"{ob['cls']} [orig, read-only]", dash=(3, 2))

        for it in self.queue_items:
            if not self._queue_item_visible(it):
                continue
            k = item_key(it)
            state = self.decisions[k]
            color = {"pending": COLOR_PENDING, "accept": COLOR_ACCEPT,
                     "reject": COLOR_REJECT, "deleted": COLOR_DELETED}[state]
            cls_name = self.class_overrides.get(k, it["cls"])
            synced_tag = "  [synced]" if k in self.synced_from_map else ""
            label = f"{cls_name} {gpl.votes_str(it)} [{state}]{synced_tag}"
            self._draw_box(it["box"], color, label, key=k, kind="queue",
                           dash=(2, 2) if state == "deleted" else None)

        # V11: manual boxes now carry their own pending/accepted/rejected
        # state, drawn with the same color language as queue boxes
        # (orange=pending, cyan=accepted, red/hidden=rejected) so it's
        # immediately visible which ones still need a decision. (V12:
        # a freshly drawn box is auto-accepted, so in practice you'll
        # mostly see cyan here unless you've since rejected/reset one.)
        for mb in self.manual_boxes:
            if not self._manual_box_visible(mb):
                continue
            state = mb.get("decision", "pending")
            color = {"pending": COLOR_PENDING, "accepted": COLOR_MANUAL,
                      "rejected": COLOR_REJECT}[state]
            synced_tag = "  [synced]" if mb.get("synced_from") else ""
            label = f"{mb['cls']} [manual, {state}]{synced_tag}"
            self._draw_box(mb["box"], color, label, key=mb["_id"], kind="manual",
                           dash=None if state == "accepted" else (2, 2))

        self._update_status()

    def _update_status(self):
        n_pending = sum(1 for v in self.decisions.values() if v == "pending")
        n_accept = sum(1 for v in self.decisions.values() if v == "accept")
        n_reject = sum(1 for v in self.decisions.values() if v == "reject")
        n_deleted = sum(1 for v in self.decisions.values() if v == "deleted")
        n_manual_pending = sum(1 for mb in self.manual_boxes
                                if mb.get("decision", "pending") == "pending")
        unsaved = "  [saving\u2026]" if self.dirty else "  [saved]"
        streak_txt = f"  |  streak={self.streak}" if self.streak > 1 else ""
        blocked_txt = ("  |  \u26a0 manual pending, can't complete"
                        if n_manual_pending else "")
        clipboard_txt = ("  |  \U0001f4cb copied" if self.clipboard is not None else "")
        self.status_var.set(
            f"{Path(self.image_key).name}  |  pending={n_pending} accept={n_accept} "
            f"reject={n_reject} deleted={n_deleted} "
            f"manual={len(self.manual_boxes)}(pending={n_manual_pending}) "
            f"orig={len(self.original_boxes)}"
            + (f"(hidden={self._orig_hidden_count})" if self._orig_hidden_count else "")
            + f"  |  zoom={int(self.zoom * 100)}%"
            f"{streak_txt}{blocked_txt}{clipboard_txt}{unsaved}")
        done = len(self.completed_set)
        total = len(self.image_keys) if self.qa_mode else (len(self.image_keys) + done)
        self.progress_var.set(
            f"Image {self.idx + 1}/{len(self.image_keys)}    "
            f"Fully reviewed overall: {done}"
            + ("" if self.qa_mode else f"/{total}"))

    # ------------------------------------------------------------------
    # Box get/set (shared by drag-move/resize and the context menus)
    # ------------------------------------------------------------------

    def _find_manual(self, manual_id):
        for mb in self.manual_boxes:
            if mb["_id"] == manual_id:
                return mb
        raise KeyError(f"manual box not found for id {manual_id!r}")

    def _get_box(self, kind, key):
        if kind == "queue":
            for it in self.queue_items:
                if item_key(it) == key:
                    return it["box"]
            raise KeyError(f"queue item not found for key {key!r}")
        return self._find_manual(key)["box"]

    def _set_box(self, kind, key, new_box):
        if kind == "queue":
            for it in self.queue_items:
                if item_key(it) == key:
                    it["box"] = new_box
                    return
            raise KeyError(f"queue item not found for key {key!r}")
        else:
            self._find_manual(key)["box"] = new_box

    def _set_decision(self, key, state):
        prev_state = self.decisions.get(key, "pending")
        self._push_undo()
        self.decisions[key] = state
        if state in ("accept", "reject"):
            self.streak += 1
        else:
            self.streak = 0
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()
        # (V17) Deferred: queued for the next _flush() rather than
        # applied immediately. (V13 fix B) Handles BOTH forward-syncing
        # the new state AND retracting whatever the box's PREVIOUS
        # state had synced -- covers Accept->Reject, Accept->Delete,
        # ->Reset to Pending, etc., not just the original
        # "pending->decided" case.
        self._pending_sync_actions.append(
            lambda k=key, p=prev_state, s=state: self._sync_after_decision_change(k, p, s))

    def _override_class(self, key, cls):
        self._push_undo()
        self.class_overrides[key] = cls
        self._mark_dirty()
        self.redraw()

    def _set_manual_class(self, manual_id, cls):
        self._push_undo()
        self._find_manual(manual_id)["cls"] = cls
        self._mark_dirty()
        self.redraw()

    def _set_manual_decision(self, manual_id, state):
        """(V11) Accept / Reject / Reset to Pending for a manual box --
        a freshly drawn box is now auto-accepted (V12), so this is
        mainly how you correct a box you drew by mistake. An image
        can't complete while any manual box on it is still "pending" (see
        _update_completed_list). Accepting also syncs the box (as a
        fresh copy) into nearby frames; un-accepting (reject or reset
        to pending) removes exactly the copies that specific accept
        created.

        (V17) Both sync calls are now deferred, queued for the next
        _flush() rather than applied immediately."""
        self._push_undo()
        mb = self._find_manual(manual_id)
        prev = mb.get("decision", "pending")
        mb["decision"] = state
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()
        if state == "accepted" and prev != "accepted":
            self._pending_sync_actions.append(lambda m=mb: self._sync_propagate_manual_add(m))
        elif state != "accepted" and prev == "accepted":
            self._pending_sync_actions.append(lambda m=mb: self._sync_propagate_manual_remove(m))

    def _delete_manual(self, manual_id):
        self._push_undo()
        mb = self._find_manual(manual_id)
        # (V17) Deferred: queued for the next _flush() rather than
        # applied immediately.
        self._pending_sync_actions.append(lambda m=mb: self._sync_propagate_manual_remove(m))
        self.manual_boxes = [m for m in self.manual_boxes if m["_id"] != manual_id]
        self._mark_dirty()
        self.redraw()

    def _add_manual_box(self, cls, box):
        """Appends a new manually-drawn (or, V14, pasted) box. No cap on
        how many you can add -- call this as many times as you like (one
        per drag-draw on empty canvas, or one per Ctrl+V). Each gets its
        own stable id so adding/deleting others around it never
        disturbs it.

        (V12) Auto-accepted immediately: it's included in output and
        counted toward completing the image the instant it's created,
        and (if sync is on) is propagated to nearby frames right away --
        no separate "open its menu and click Accept" step for the
        common case of a box you meant to add. If you drew/pasted one
        by mistake, open its menu and use Reject / Reset to Pending /
        Delete to correct it.

        (V17) The sync propagation is now deferred, queued for the
        next _flush() rather than applied immediately."""
        self._push_undo()
        mb = {"cls": cls, "box": box, "_id": self._next_manual_id, "decision": "accepted"}
        self.manual_boxes.append(mb)
        self._next_manual_id += 1
        self.streak += 1
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()
        self._pending_sync_actions.append(lambda m=mb: self._sync_propagate_manual_add(m))

    def _arm_reposition(self, kind, key):
        self.reposition_armed = (kind, key)
        self.selected_key = (kind, key)
        self.redraw()

    def _delete_selected(self):
        """Delete/Backspace handler: deletes whichever box is currently
        selected (self.selected_key, set whenever a box's menu was
        opened or it was clicked -- it stays set after the menu closes,
        including via "Cancel", so selecting a box then pressing Delete
        works without the menu needing to stay open). Mirrors exactly
        what the menu's own Delete option does for that box kind, so
        there's no separate deletion behavior to keep in sync:
          - queue box  -> same as "Delete (hide from view)": moves to
            the "deleted" decision state (hidden, fully recoverable via
            the "Show Deleted/Rejected" checkbox + Reset to Pending).
          - manual box -> same as its menu's "Delete": removed outright
            (via _delete_manual, which also retracts any cross-frame
            sync copies it created).
        No-op if nothing is selected.

        (V15) Called both from the root-level <Delete>/<BackSpace>
        bindings AND from the popup menu's own <Delete>/<BackSpace>
        bindings (see _open_box_context_menu) -- the menu case unposts
        the menu first, then calls this exact same method, so behavior
        is identical either way."""
        if self.selected_key is None:
            return
        kind, key = self.selected_key
        if kind == "queue":
            if self._queue_item_by_key(key) is None:
                # Stale selection -- item no longer in this image's queue.
                # Nothing to delete; just drop the selection.
                self.selected_key = None
                return
            if self.decisions.get(key) != "deleted":
                self._set_decision(key, "deleted")
        else:
            if not any(mb["_id"] == key for mb in self.manual_boxes):
                # Stale selection -- most likely two Delete/BackSpace
                # events fired for the same keypress (seen on Windows when
                # a box's context menu closes at nearly the same moment
                # the root-level binding also sees the key), so the box
                # was already removed by the first call before this one
                # ran. Not an error -- just drop the stale selection
                # instead of crashing.
                self.selected_key = None
                return
            self._delete_manual(key)
        self.selected_key = None
        self.redraw()

    # ------------------------------------------------------------------
    # Menus
    # ------------------------------------------------------------------

    def _canvas_to_root(self, canvas_x, canvas_y):
        """Converts a point in canvas item-coordinate space (the same
        space box outlines are drawn in -- i.e. already multiplied by
        self.scale) into root/screen coordinates suitable for
        tk_popup(), accounting for however far the canvas is currently
        scrolled."""
        view_x = canvas_x - self.canvas.canvasx(0)
        view_y = canvas_y - self.canvas.canvasy(0)
        return (self.canvas.winfo_rootx() + int(view_x),
                self.canvas.winfo_rooty() + int(view_y))

    def _menu_pos_clear_of_box(self, box, fallback_root):
        """Picks a screen position for a box's popup menu that sits
        just outside the box's edge (offset by MENU_OFFSET_PX) instead
        of directly on top of it -- so the box itself, and its resize
        handles once selected, stay fully visible and clickable
        underneath instead of being covered by the menu. Prefers
        opening to the right of the box's top-right corner; falls back
        to the left of the box if there isn't roughly enough room to
        the right (rough estimate, not exact -- Tk will still nudge an
        edge-of-screen menu back on screen on its own if this guess is
        a little off). Falls back to `fallback_root` (the raw click
        point) if the box can't be resolved for some reason."""
        if not box:
            return fallback_root
        try:
            x1, y1, x2, y2 = [v * self.scale for v in box]
        except Exception:
            return fallback_root
        canvas_visible_w = self.canvas.winfo_width()
        right_edge_view_x = x2 - self.canvas.canvasx(0)
        if right_edge_view_x + MENU_OFFSET_PX + MENU_EST_WIDTH_PX < canvas_visible_w:
            return self._canvas_to_root(x2 + MENU_OFFSET_PX, y1)
        return self._canvas_to_root(x1 - MENU_OFFSET_PX - MENU_EST_WIDTH_PX, y1)

    def _open_class_menu(self, x_root, y_root, on_pick):
        menu = tk.Menu(self.root, tearoff=0)
        for c in CLASS_NAMES:
            menu.add_command(label=c, command=lambda c=c: on_pick(c))
        menu.add_separator()
        menu.add_command(label="Cancel")
        menu.tk_popup(x_root, y_root)

    def _open_box_context_menu(self, x_root, y_root, kind, key):
        self.selected_key = (kind, key)
        self.redraw()
        try:
            box = self._get_box(kind, key)
        except KeyError:
            box = None
        x_root, y_root = self._menu_pos_clear_of_box(box, (x_root, y_root))
        menu = tk.Menu(self.root, tearoff=0)
        if kind == "queue":
            state = self.decisions[key]
            menu.add_command(label="Accept", command=lambda: self._set_decision(key, "accept"))
            menu.add_command(label="Reject (= exclude from output)",
                              command=lambda: self._set_decision(key, "reject"))
            menu.add_command(label="Reset to Pending",
                              command=lambda: self._set_decision(key, "pending"))
            menu.add_separator()
            class_menu = tk.Menu(menu, tearoff=0)
            for c in CLASS_NAMES:
                class_menu.add_command(label=c, command=lambda c=c: self._override_class(key, c))
            menu.add_cascade(label="Change Class", menu=class_menu)
            menu.add_separator()
            menu.add_command(label="Enable Reposition/Resize (then drag once)",
                              command=lambda: self._arm_reposition(kind, key))
            menu.add_separator()
            menu.add_command(label="Copy (Ctrl+C)", command=self._copy_selected)
            menu.add_command(label="Paste (Ctrl+V)", command=self._paste_clipboard)
            menu.add_separator()
            if state == "deleted":
                menu.add_command(label="Undelete (reset to Pending)",
                                  command=lambda: self._set_decision(key, "pending"))
            else:
                # (Windows fix) underline=0 registers "D" as this native
                # popup menu's own accelerator letter for this item. On
                # Windows, tk_popup() renders a NATIVE Win32 popup menu
                # that runs its own message loop and owns keyboard input
                # for as long as it's posted -- Tk-level key bindings
                # (menu.bind("<Delete>", ...), added below right before
                # tk_popup()) never receive events while that native loop
                # has control, so on Windows they silently never fire even
                # though the exact same binding works on X11/macOS where
                # Tk draws (and owns input for) the menu itself. A native
                # single-letter accelerator is handled by Windows' own
                # menu-accelerator table instead of Tk, so it works
                # regardless of which platform is drawing the menu -- this
                # is what actually makes "press D while the menu is open"
                # delete the box on Windows. Fix 2 (Escape then Delete)
                # remains a no-code-change workaround: Escape returns
                # keyboard control to self.root, where the ordinary
                # <Delete> binding fires normally, exactly as the earlier
                # crash traceback showed.
                menu.add_command(label="Delete (hide from view)", underline=0,
                                  command=lambda: self._set_decision(key, "deleted"))
        else:
            # V11: manual boxes get the same Accept/Reject/Reset trio as
            # queue boxes -- this is the safety gate the box has to pass
            # through before it can reach output or let the image complete.
            # (V12: Accept already happened automatically when drawn, so
            # this menu is mainly for correcting a mistaken box.)
            mb_state = self._find_manual(key).get("decision", "pending")
            menu.add_command(label="Accept", command=lambda: self._set_manual_decision(key, "accepted"))
            menu.add_command(label="Reject (= exclude from output)",
                              command=lambda: self._set_manual_decision(key, "rejected"))
            menu.add_command(label="Reset to Pending",
                              command=lambda: self._set_manual_decision(key, "pending"))
            menu.add_separator()
            class_menu = tk.Menu(menu, tearoff=0)
            for c in CLASS_NAMES:
                class_menu.add_command(label=c, command=lambda c=c: self._set_manual_class(key, c))
            menu.add_cascade(label="Change Class", menu=class_menu)
            menu.add_separator()
            menu.add_command(label="Copy (Ctrl+C)", command=self._copy_selected)
            menu.add_command(label="Paste (Ctrl+V)", command=self._paste_clipboard)
            menu.add_separator()
            # (Windows fix, see the queue-box branch above for the full
            # explanation) underline=0 gives this item a native "D"
            # accelerator so Delete-while-menu-open works on Windows,
            # where tk_popup()'s native message loop otherwise swallows
            # Tk-level <Delete>/<BackSpace> bindings entirely.
            menu.add_command(label="Delete", underline=0, command=lambda: self._delete_manual(key))
        menu.add_separator()
        menu.add_command(label="Cancel")

        # (V15) Delete/BackSpace bugfix: tk.Menu.tk_popup() takes Tk's
        # keyboard grab for as long as the menu is posted (that's what
        # makes arrow-key navigation through the menu itself work), so
        # while this menu is open, key events -- including Delete and
        # BackSpace -- go to the MENU widget, never to self.root. The
        # <Delete>/<BackSpace> bindings in _build_ui are on self.root,
        # so they simply never fired while a box's menu was open; the
        # only way to delete was to explicitly click the menu's own
        # Delete item. Binding the same keys directly on this menu
        # widget fixes it: unpost the menu first (so it doesn't linger
        # on screen after the box it referred to is gone/changed), then
        # call the exact same _delete_selected() the root-level binding
        # already uses, so the two paths behave identically.
        #
        # (Windows note) These two bindings are exactly right for
        # Linux/macOS, where Tk itself draws and owns input for the
        # posted menu. On Windows, tk_popup() hands the menu off to a
        # NATIVE Win32 popup, which runs its own modal message loop and
        # owns all keyboard input while posted -- these Tk-level
        # bindings simply never see the keystroke there, regardless of
        # order or of unposting first. That's a Tk/Windows platform
        # limitation, not a mistake in how these are wired. The
        # underline=0 accelerators added above are the actual Windows
        # fix; these bindings are kept as-is since they're still what
        # makes Delete/BackSpace work correctly while a menu is open on
        # Linux/macOS.
        menu.bind("<Delete>", lambda e: (menu.unpost(), self._delete_selected()))
        menu.bind("<BackSpace>", lambda e: (menu.unpost(), self._delete_selected()))

        menu.tk_popup(x_root, y_root)

    def show_help_dialog(self):
        messagebox.showinfo("Controls", (
            "Click a box:                 open its menu -- Accept / Reject / "
            "Reset to Pending / Change Class, plus Delete for boxes you drew and "
            "for model (queue) boxes, plus Copy/Paste\n\n"
            "Drag ON a box:                only works for boxes YOU drew (cyan). "
            "The model's own proposed boxes can't be bumped by accident -- click "
            "one and use \"Enable Reposition/Resize\" in its menu if it genuinely "
            "needs adjusting, then drag it (or one of its handles) once -- it "
            "locks again automatically the moment you actually drag it (just "
            "opening the menu again does NOT disarm it). Original dataset labels "
            "(magenta) can never be clicked, dragged, resized, or deleted at "
            "all.\n\n"
            "Resize a box:                 click a box once to select it (its "
            "menu opens -- just click Cancel to dismiss and keep it selected). "
            "8 small square handles appear on its corners and edges -- drag any "
            "handle to stretch that side. Works for manual (cyan) boxes any "
            "time, and for a queue box only while it's armed via \"Enable "
            "Reposition/Resize\". A box can't be shrunk past a few pixels.\n\n"
            "Drag on empty area:           draw a new box, then pick its class "
            "from the popup. The new box is auto-accepted (V12) as soon as you "
            "pick a class -- it's included in output right away. Open its menu "
            "afterward if you need to Reject / Reset to Pending / Delete it, or "
            "just add as many as you like, one drag at a time.\n\n"
            "Copy / Paste (Ctrl+C / Ctrl+V):  select any box (manual or model), "
            "press Ctrl+C to copy its class and size, then Ctrl+V to paste a new "
            "manual box -- same class, same size -- offset a little from the "
            "original so you can see and drag it into place, instead of drawing "
            "a same-sized box from scratch every time. Keep pressing Ctrl+V to "
            "stamp out more copies, each one offset a bit further; a fresh "
            "Ctrl+C on any box resets that cascade. Also available from a box's "
            "right-click menu, and from the toolbar's Copy/Paste buttons. Pasted "
            "boxes are auto-accepted and undoable just like a hand-drawn one.\n\n"
            "Click on empty area (no drag): deselects whatever box was "
            "selected -- clears its resize handles and disarms an in-progress "
            "\"Enable Reposition/Resize\" arm if one was active.\n\n"
            "Manual box states:            accepted (cyan, in output -- the "
            "default the instant you draw or paste one) / pending (orange, only "
            "seen if you Reset one) / rejected (red, hidden unless \"Show "
            "Deleted/Rejected\" is on). An image CANNOT move to the completed "
            "list while any manual box on it is still pending -- same rule queue "
            "boxes have always had.\n\n"
            "Delete (queue boxes):         hides the box from the canvas and "
            "excludes it from output, but it's fully recoverable -- check "
            "\"Show Deleted/Rejected\" to see grey/dashed deleted boxes again and "
            "reset one back to Pending if you deleted it by mistake.\n\n"
            "Delete (manual boxes):        removed outright (Delete key, "
            "Backspace, or its menu's Delete) -- also cleans up any cross-frame "
            "sync copies that box created.\n\n"
            "Fast Mode:                    left-click a queue box = Accept, "
            "right-click = Reject, no menu popup. Toggle it in the toolbar or "
            "View menu. Manual boxes and Enable-Reposition boxes are unaffected.\n\n"
            "Accept All / Reject All Pending:  bulk-decide every still-pending "
            "queue box on the current image at once (confirms first). Counts as "
            "one undo step. (Manual boxes are auto-accepted on draw/paste, so "
            "there's normally nothing pending among them to bulk-decide.)\n\n"
            "Undo / Redo:                  Ctrl+Z / Ctrl+Y (or Ctrl+Shift+Z), "
            "also in the Edit menu and toolbar. Covers every edit on the current "
            "image -- decisions, class changes, manual add/delete/paste, drags "
            "and resizes. Resets when you move to another image.\n\n"
            "Esc while dragging:           cancels the drag/resize in progress "
            "and snaps the box back to where it was.\n\n"
            "1-9 keys:                     apply the Nth class (in class-list "
            "order) to whichever box you most recently clicked.\n\n"
            "Delete / Backspace key:       deletes whichever box is currently "
            "selected -- a queue box moves to \"deleted\" (hidden, recoverable "
            "via \"Show Deleted/Rejected\"), a manual box is removed outright "
            "(and any frames it synced to are cleaned up too). No-op if nothing "
            "is selected. Works whether or not the box's own menu is currently "
            "open (on Windows, while a menu is open, press D instead -- see the "
            "Delete item's underlined accelerator -- since Windows' native popup "
            "menu doesn't forward Delete/Backspace to this shortcut the way "
            "Linux/macOS do; Escape-then-Delete also always works).\n\n"
            "Mouse wheel:                  zoom in/out, centered on the cursor\n"
            "+ / - keys:                   zoom in/out\n"
            "0 key / \"Fit\" button:         reset to fit-to-window\n"
            "Middle-click + drag / scrollbars: pan around when zoomed in\n\n"
            "\"Original labels\" checkbox:  the real dataset's existing labels, "
            "shown ON by default so you don't accidentally re-draw something "
            "that's already labeled\n"
            "Model checkboxes + \"All\"/\"None\": choose which models' proposed "
            "boxes are visible (labelled with each model's full name -- "
            "YOLO-VisDrone, RF-DETR, YOLO-COCO, DINO). The counts next to each "
            "model's name (above the image) show how many boxes THAT model "
            "actually proposed for this image -- shown in red/bold if it's zero, "
            "so a model silently missing an image is obvious.\n\n"
            "Sync nearby frames:           when ON (default), Accept/Reject/"
            "Delete on a queue box, OR Accept/Reject/Reset/Delete on a manual "
            "box, walks outward frame-by-frame in each direction and applies the "
            "same action there too -- for queue boxes, to an already-queued box "
            "of the SAME EFFECTIVE CLASS (honoring any class override) near a "
            "MOTION-PREDICTED position that starts at the source box and is "
            "refined using real observations further out (only if the matched "
            "box is still pending); for manual boxes, by copying the box itself "
            "into the neighbor frame at the same position (only if nothing of "
            "that class already sits near that spot there). It never overrides "
            "a decision you or a previous sync already made, and never touches "
            "an already-completed image. Synced boxes show a \"[synced]\" tag. "
            "Changing your mind on a source box later (Accept -> Reject, -> "
            "Delete, or -> Reset to Pending) automatically retracts exactly the "
            "copies THAT box created, as long as they haven't since been "
            "manually re-decided by hand. As of V17, the actual sync pass runs "
            "at SAVE time (autosave, Save Now, navigation, close, or focus-"
            "loss) rather than on every single click, so a burst of edits on "
            "one image before its next save collapses into one sync pass. The "
            "\u00b1N spinner controls how many frames out to look; \"Undo Last "
            "Sync\" reverts exactly the last auto-applied batch, everywhere it "
            "touched.\n\n"
            "Switch to QA / Completed:     swaps the current session, live, "
            "between images still pending review and images that are already "
            "fully decided (same list --qa opens from the command line) -- no "
            "need to close and relaunch. Everything is still fully editable "
            "there: use it to spot-check finished work, or to find and fix a "
            "box you decided by mistake. The button/menu item flips back to "
            "\"Back to Pending Review\" while you're in that view.\n\n"
            "Image list sidebar (left side):  every image in the current "
            "--source/--cls filter, split into \"Remaining\" and \"Reviewed\", "
            "each grouped by source dataset (SARD/UAVDT/VisDrone, light color "
            "stripe -- see the legend under the list), and (V19) each source "
            "further split into per-frame-SET rows -- one per ~100-frame burst "
            "(same grouping \"Check Sequence Coverage\" uses) -- each with its "
            "OWN distinguishing color and an inline \"n/total\" frame count, so "
            "you can tell at a glance how many frames a set has and how many "
            "you've gotten to, without opening the coverage report. A small "
            "warning marker appears on a set's row (in either section) if its "
            "overall reviewed count is still below a few frames, so an "
            "easy-to-miss set doesn't quietly fall out of sight. Click any "
            "image to jump straight to it, in either state, without "
            "Next/Previous-ing your way there. Rebuilds automatically after "
            "every save. \"Check sequence coverage\" gives the same numbers as "
            "a single full-text report across every set at once.\n\n"
            "Auto-advance:                 optional -- once every box on the "
            "image (queue AND manual, across the FULL queue, not just what a "
            "--source/--cls filter shows) has a decision, automatically jump to "
            "the next image.\n\n"
            "Everything autosaves as you go (within under half a second of any "
            "change, or instantly if the window loses focus) -- Next/Previous/"
            "closing/Alt-Tabbing never lose work, and there's no save prompt "
            "because there's nothing left unsaved to ask about. \"Save Now\" / "
            "'s' just forces it immediately. Cross-frame sync (see above) runs "
            "as part of that same save.\n\n"
            "Once every box (queue and manual) on an image has a decision -- "
            "again, across the FULL queue for that image, not just a --source/"
            "--cls filtered view -- that image moves into a separate completed "
            "list (review_completed.json) and won't show up again in a normal "
            "review session. Reach it again any time with the \"Switch to QA / "
            "Completed\" button, or by running with --qa.\n\n"
            "Startup safety net (V11):     every launch backs up "
            "review_progress.json and review_completed.json into "
            "datasets/review_backups/<timestamp>/ before touching anything, then "
            "re-matches any saved decision whose box coordinates drifted from a "
            "rerun of generate_pseudo_labels.py (rather than treating it as lost/"
            "pending), and recomputes the completed list from the result. Watch "
            "the console at startup for a summary. Disable either step with "
            "--no-backup / --no-key-reconcile if you ever need to.\n"
        ))

    # ------------------------------------------------------------------
    # Mouse handling
    # ------------------------------------------------------------------

    def on_right_click(self, event):
        """Fast Mode only: right-click a queue box = instant Reject.
        No effect on manual boxes, original boxes, or when Fast Mode
        is off (menu-based reject still works everywhere via left-click
        -> context menu)."""
        if not self.fast_mode.get():
            return
        current = self.canvas.find_withtag("current")
        hit = self.item_map.get(current[0]) if current else None
        if not hit:
            return
        kind, key = hit
        if kind == "queue" and self.reposition_armed != (kind, key):
            self.selected_key = (kind, key)
            self._set_decision(key, "reject")

    def _handle_hit_at(self, view_x, view_y):
        """Hit-tests the resize handles with a small pixel pad, since
        the handle squares themselves are tiny and precision-clicking
        them would be its own usability problem. Returns
        (kind, key, handle_name) or None."""
        cx = self.canvas.canvasx(view_x)
        cy = self.canvas.canvasy(view_y)
        pad = HANDLE_HIT_PAD
        best = None
        best_dist = None
        for hid, (kind, key, name) in self.handle_map.items():
            coords = self.canvas.coords(hid)
            if not coords:
                continue
            x1, y1, x2, y2 = coords
            if (x1 - pad) <= cx <= (x2 + pad) and (y1 - pad) <= cy <= (y2 + pad):
                hcx, hcy = (x1 + x2) / 2, (y1 + y2) / 2
                dist = (hcx - cx) ** 2 + (hcy - cy) ** 2
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best = (kind, key, name)
        return best

    def on_mouse_move_hover(self, event):
        """Pure visual feedback -- no state changes. Shows a resize
        cursor over a handle, a move cursor over a draggable box, or
        the default crosshair (for drawing) elsewhere."""
        if self.drag is not None:
            return  # mid-drag, don't fight the cursor set for that drag
        handle_hit = self._handle_hit_at(event.x, event.y)
        if handle_hit:
            self.canvas.config(cursor=HANDLE_CURSOR[handle_hit[2]])
            return
        current = self.canvas.find_withtag("current")
        hit = self.item_map.get(current[0]) if current else None
        if hit:
            kind, key = hit
            if kind == "manual" or self.reposition_armed == (kind, key):
                self.canvas.config(cursor="fleur")
            else:
                self.canvas.config(cursor="hand2")
        else:
            self.canvas.config(cursor="tcross")

    def on_mouse_down(self, event):
        cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)

        # Resize handles take priority over everything else under the
        # cursor -- they're only drawn on the already-selected box, so
        # there's no ambiguity about which box a handle belongs to.
        handle_hit = self._handle_hit_at(event.x, event.y)
        if handle_hit:
            kind, key, name = handle_hit
            self.drag = {"mode": "resize", "kind": kind, "key": key, "handle": name,
                         "start": (cx, cy), "moved": False, "snapshotted": False,
                         "orig_box": list(self._get_box(kind, key))}
            return

        current = self.canvas.find_withtag("current")
        hit = self.item_map.get(current[0]) if current else None
        if hit:
            kind, key = hit
            if kind == "manual" or self.reposition_armed == (kind, key):
                self.drag = {"mode": "move", "kind": kind, "key": key,
                             "start": (cx, cy), "moved": False, "snapshotted": False,
                             "orig_box": list(self._get_box(kind, key))}
            elif kind == "queue" and self.fast_mode.get():
                # Fast Mode: left-click a queue box accepts it immediately,
                # no menu, no drag possible.
                self.drag = {"mode": "fast_accept", "kind": kind, "key": key,
                             "start": (cx, cy)}
            else:
                # Queue box, not armed, not fast mode -- click-only, no drag.
                self.drag = {"mode": "queue_click", "kind": kind, "key": key,
                             "start": (cx, cy)}
        else:
            self.drag = {"mode": "draw", "start": (cx, cy), "moved": False}
            self._rubber_id = self.canvas.create_rectangle(
                cx, cy, cx, cy,
                outline=COLOR_MANUAL, width=2, dash=(4, 2))

    def on_mouse_drag(self, event):
        if not self.drag:
            return
        cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        sx, sy = self.drag["start"]

        if self.drag["mode"] == "draw":
            if abs(cx - sx) > 3 or abs(cy - sy) > 3:
                self.drag["moved"] = True
            self.canvas.coords(self._rubber_id, sx, sy, cx, cy)
        elif self.drag["mode"] == "move":
            if abs(cx - sx) > 3 or abs(cy - sy) > 3:
                self.drag["moved"] = True
            if self.drag["moved"]:
                if not self.drag["snapshotted"]:
                    # Snapshot once at the start of the drag, not on every
                    # tick -- one undo step should revert the whole move,
                    # not one pixel of it.
                    self._push_undo()
                    self.drag["snapshotted"] = True
                box = self._get_box(self.drag["kind"], self.drag["key"])
                dx = (cx - sx) / self.scale
                dy = (cy - sy) / self.scale
                self._set_box(self.drag["kind"], self.drag["key"],
                              [box[0] + dx, box[1] + dy, box[2] + dx, box[3] + dy])
                self.drag["start"] = (cx, cy)
                self._mark_dirty()
                self.redraw()
        elif self.drag["mode"] == "resize":
            if abs(cx - sx) > 1 or abs(cy - sy) > 1:
                self.drag["moved"] = True
            if self.drag["moved"]:
                if not self.drag["snapshotted"]:
                    self._push_undo()
                    self.drag["snapshotted"] = True
                self._apply_resize(cx, cy)
                self._mark_dirty()
                self.redraw()
        # "queue_click" / "fast_accept" modes: intentionally ignore drag motion.

    def _apply_resize(self, cx, cy):
        """Moves whichever edge(s) the active handle controls to follow
        the cursor, in ORIGINAL IMAGE coordinates, clamped so the box
        can never invert or collapse below MIN_BOX_PX. Corner handles
        (hx != 0 and hy != 0) move both edges; edge handles move one."""
        d = self.drag
        kind, key, handle = d["kind"], d["key"], d["handle"]
        hx, hy = next((hx_, hy_) for name, hx_, hy_ in HANDLES if name == handle)
        img_x = cx / self.scale
        img_y = cy / self.scale

        x1, y1, x2, y2 = self._get_box(kind, key)
        if hx == -1:
            x1 = min(img_x, x2 - MIN_BOX_PX)
        elif hx == 1:
            x2 = max(img_x, x1 + MIN_BOX_PX)
        if hy == -1:
            y1 = min(img_y, y2 - MIN_BOX_PX)
        elif hy == 1:
            y2 = max(img_y, y1 + MIN_BOX_PX)

        # Clamp to image bounds too -- a handle dragged off-canvas
        # shouldn't be able to push the box coordinates negative or
        # past the image size (scrollregion lets you drag past the
        # visible area while zoomed/panned).
        x1 = max(0.0, min(x1, self.img_w))
        y1 = max(0.0, min(y1, self.img_h))
        x2 = max(0.0, min(x2, self.img_w))
        y2 = max(0.0, min(y2, self.img_h))

        self._set_box(kind, key, [x1, y1, x2, y2])

    def on_mouse_up(self, event):
        if not self.drag:
            return
        cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        mode = self.drag["mode"]

        if mode == "draw":
            self.canvas.delete(self._rubber_id)
            sx, sy = self.drag["start"]
            if abs(cx - sx) > 6 and abs(cy - sy) > 6:
                x1, y1 = min(sx, cx) / self.scale, min(sy, cy) / self.scale
                x2, y2 = max(sx, cx) / self.scale, max(sy, cy) / self.scale
                # (fix) Clamp to image bounds -- same clamping _apply_resize
                # and _paste_clipboard already do -- so a box drawn with its
                # start/end near the canvas edge can't end up a fraction of
                # a pixel outside the image (e.g. x1 = -0.3). An out-of-
                # bounds manual box auto-accepts immediately (V12) and then
                # blows up box_to_yolo_line() on every single autosave from
                # then on, which -- since _flush() never reaches
                # "self.dirty = False" when it raises -- makes EVERY later
                # navigation/flush attempt (they all call
                # _flush_now_if_dirty first) throw the same exception again.
                # That's what looks like the app "freezing": it's actually
                # failing to save and re-failing on every subsequent action.
                x1 = max(0.0, min(x1, self.img_w))
                x2 = max(0.0, min(x2, self.img_w))
                y1 = max(0.0, min(y1, self.img_h))
                y2 = max(0.0, min(y2, self.img_h))
                if x2 - x1 < MIN_BOX_PX:
                    x2 = min(self.img_w, x1 + MIN_BOX_PX)
                    x1 = max(0.0, x2 - MIN_BOX_PX)
                if y2 - y1 < MIN_BOX_PX:
                    y2 = min(self.img_h, y1 + MIN_BOX_PX)
                    y1 = max(0.0, y2 - MIN_BOX_PX)
                box = [x1, y1, x2, y2]
                # Offset away from the box just drawn (same reasoning as
                # _menu_pos_clear_of_box for the box-edit context menu) --
                # otherwise the class-picker opens right on top of the box
                # you just drew, and picking a class then immediately
                # wanting to nudge/resize it means fighting the menu for
                # the same screen space it's still covering.
                menu_x, menu_y = self._menu_pos_clear_of_box(
                    box, (event.x_root, event.y_root))
                self._open_class_menu(menu_x, menu_y,
                                       on_pick=lambda cls: self._add_manual_box(cls, box))
            elif self.selected_key is not None or self.reposition_armed is not None:
                # (fix) A plain click on empty canvas -- started as "draw"
                # but never actually dragged past the threshold -- used to
                # be a complete no-op, which meant the ONLY way to drop a
                # selection (and its resize handles) was to click another
                # box. That's backwards: empty space should be the "give
                # me nothing selected" gesture. Clear the selection and
                # disarm any one-shot reposition/resize arm that was still
                # waiting on a drag, then redraw so the handles disappear
                # immediately.
                self.selected_key = None
                self.reposition_armed = None
                self.redraw()
        elif mode in ("move", "resize"):
            # (V13 fix C) Only disarm a one-shot "Enable Reposition/Resize"
            # once an actual move/resize drag happened. Previously this
            # disarmed on ANY mouse-up on the armed box -- including a
            # plain click that just reopens its context menu -- so the
            # box could silently lose its arming before you ever got to
            # drag it.
            if self.drag["moved"] and self.reposition_armed == (self.drag["kind"], self.drag["key"]):
                self.reposition_armed = None   # one-shot: disarm after use
                self.redraw()
            if not self.drag["moved"] and mode == "move":
                self._open_box_context_menu(event.x_root, event.y_root,
                                             self.drag["kind"], self.drag["key"])
        elif mode == "queue_click":
            self._open_box_context_menu(event.x_root, event.y_root,
                                         self.drag["kind"], self.drag["key"])
        elif mode == "fast_accept":
            self.selected_key = (self.drag["kind"], self.drag["key"])
            self._set_decision(self.drag["key"], "accept")

        self.drag = None
        self.on_mouse_move_hover(event)  # refresh cursor for wherever we ended up


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default=None, help="Only review this source (e.g. UAVDT).")
    p.add_argument("--cls", default=None, help="Only review this class (e.g. person).")
    p.add_argument("--show-all", action="store_true",
                    help="Include images that are already fully decided "
                         "(default: only show images with pending items).")
    p.add_argument("--hide-original", action="store_true",
                    help="Start with the existing/original-label overlay OFF "
                         "(default: shown, so you don't double-label something "
                         "already in the real dataset).")
    p.add_argument("--qa", action="store_true",
                    help="Quality-assurance mode: open a session sourced ONLY "
                         "from review_completed.json (images a previous pass "
                         "already fully decided), for a second reviewer to "
                         "check over. Ignores --show-all. You can also reach "
                         "this live from inside the app via the 'Switch to QA "
                         "/ Completed' button -- no need to relaunch.")
    p.add_argument("--no-sync", action="store_true",
                    help="Start with cross-frame sync OFF (default: on). "
                         "Toggleable at any time in the toolbar/View menu.")
    p.add_argument("--sync-window", type=int, default=SYNC_DEFAULT_WINDOW,
                    help=f"How many frames out (each direction) in the same "
                         f"sequence to look for a matching pending box when "
                         f"syncing decisions (default: {SYNC_DEFAULT_WINDOW}). "
                         f"Adjustable at any time via the toolbar spinner.")
    p.add_argument("--no-backup", action="store_true",
                    help="(V11) Skip the automatic snapshot of "
                         "review_progress.json/review_completed.json taken on "
                         "every launch into datasets/review_backups/. Not "
                         "recommended -- this is the actual safety net.")
    p.add_argument("--no-key-reconcile", action="store_true",
                    help="(V11) Skip the automatic startup pass that re-matches "
                         "saved decisions whose box coordinates drifted from a "
                         "rerun of generate_pseudo_labels.py, and the completed-"
                         "list recomputation that follows it. Only useful for "
                         "debugging that step itself.")
    return p.parse_args()


def main():
    args = parse_args()
    datasets_dir = prepare_datasets.DATASETS_DIR
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    progress_path = datasets_dir / PROGRESS_FILENAME
    completed_path = datasets_dir / COMPLETED_FILENAME
    reviewed_root = datasets_dir / REVIEWED_DIRNAME

    if not queue_path.exists():
        raise SystemExit(f"{queue_path} not found -- run generate_pseudo_labels.py first.")

    with open(queue_path) as f:
        all_items_full = json.load(f)

    # Per-person packages (from review_batch_assigner.py) store "image"
    # relative to DATASETS_DIR so the queue file is portable across
    # machines with different absolute project paths -- resolve those
    # against THIS machine's DATASETS_DIR here, once, up front. The
    # master queue (straight from generate_pseudo_labels.py) already
    # uses absolute paths and is left untouched.
    for it in all_items_full:
        p = Path(it["image"])
        if not p.is_absolute():
            it["image"] = str((datasets_dir / p).resolve())

    # Capture each item's original, immutable box up front -- item_key()
    # depends on this, not on the (editable) "box" field. Must happen
    # before anything below calls item_key(), and before --source/--cls
    # filtering so reconciliation always sees the FULL queue.
    for it in all_items_full:
        it["_orig_box"] = list(it["box"])

    by_image_full: dict = {}
    for it in all_items_full:
        by_image_full.setdefault(it["image"], []).append(it)

    progress = load_json_dict(progress_path)
    completed_set = load_completed(completed_path)

    # --- V11 startup safety net: backup, then self-heal ------------------
    if not args.no_backup:
        backup_dir = backup_progress_files(datasets_dir, progress_path, completed_path)
        if backup_dir:
            print(f"[backup] snapshotted progress/completed files -> {backup_dir}")

    if not args.no_key_reconcile:
        progress, stats, unresolved = reconcile_progress(by_image_full, progress)
        print(f"[reconcile] {stats['exact']} exact-key match(es), "
              f"{stats['fuzzy']} recovered by fuzzy box match, "
              f"{stats['unresolved']} unresolved.")
        if unresolved:
            print("[reconcile] unresolved entries (kept under their old key -- "
                  "NOT deleted, just not matched to a current queue item):")
            for k, reason in unresolved[:20]:
                print(f"    {reason}: {k}")
            if len(unresolved) > 20:
                print(f"    ... and {len(unresolved) - 20} more")
        save_json(progress_path, progress)

        recomputed_completed = compute_completed_set(by_image_full, progress)
        if recomputed_completed != completed_set:
            added = recomputed_completed - completed_set
            removed = completed_set - recomputed_completed
            if added:
                print(f"[reconcile] {len(added)} image(s) now recognized as fully "
                      f"reviewed that weren't marked complete before.")
            if removed:
                print(f"[reconcile] {len(removed)} image(s) dropped from the "
                      f"completed list (they have pending item(s) again).")
            completed_set = recomputed_completed
            save_completed(completed_path, completed_set)

    n_decided_images = sum(
        1 for image, items in by_image_full.items()
        if any(progress.get(item_key(it), {}).get("decision", "pending") != "pending"
               for it in items))
    n_manual_pending_total = sum(
        1 for image in by_image_full
        for mb in progress.get(f"__manual__{image}", [])
        if mb.get("decision", "pending") == "pending")
    print(f"[startup] {len(completed_set)} image(s) fully reviewed, "
          f"{n_decided_images} image(s) with at least one saved decision, "
          f"{n_manual_pending_total} manual box(es) still pending review.")
    # --- end V11 startup safety net ---------------------------------------

    all_items = all_items_full
    if args.source:
        all_items = [it for it in all_items if it["source"] == args.source]
    if args.cls:
        all_items = [it for it in all_items if it["cls"] == args.cls]

    by_image: dict = {}
    for it in all_items:
        by_image.setdefault(it["image"], []).append(it)

    all_image_keys = list(by_image.keys())

    if args.qa:
        image_keys = [k for k in all_image_keys if k in completed_set]
        if not image_keys:
            print("Nothing in the completed list yet for these filters "
                  "-- nothing for --qa to show.")
            return
    else:
        image_keys = [k for k in all_image_keys if k not in completed_set]
        if not args.show_all:
            def has_pending(image_key):
                queue_pending = any(
                    progress.get(item_key(it), {}).get("decision", "pending") == "pending"
                    for it in by_image[image_key])
                manual_pending = any(
                    mb.get("decision", "pending") == "pending"
                    for mb in progress.get(f"__manual__{image_key}", []))
                return queue_pending or manual_pending
            image_keys = [k for k in image_keys if has_pending(k)]
        if not image_keys:
            print("Nothing to review (try --show-all, or check --source/--cls "
                  "filters -- everything matching may already be in the "
                  "completed list; see --qa).")
            return

    print(f"{len(image_keys)} images to review. Opening review window...")

    root = tk.Tk()
    root.title("Label Review")
    # (V12) all_image_keys (matching --source/--cls, NOT pre-filtered by
    # completed status) is passed through so the in-app "Switch to QA /
    # Completed" toggle can recompute pending-vs-completed live from the
    # session's current completed_set, instead of being locked to
    # whichever mode the app happened to be launched in.
    # (V13 fix A) by_image_full (the TRUE unfiltered queue) is also
    # passed through so every completeness check inside ReviewApp -- not
    # just the ones at startup -- reflects the whole image, not just
    # whatever this session's --source/--cls filter shows.
    ReviewApp(root, image_keys, by_image, progress, reviewed_root, progress_path,
              completed_path, completed_set,
              show_original_default=not args.hide_original, qa_mode=args.qa,
              sync_enabled_default=not args.no_sync,
              sync_window_default=args.sync_window,
              all_image_keys_filtered=all_image_keys,
              by_image_full=by_image_full)
    root.mainloop()


if __name__ == "__main__":
    main()