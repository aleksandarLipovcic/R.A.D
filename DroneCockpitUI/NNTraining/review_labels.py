"""
review_labels.py -- Interactive, resumable review tool for the queue
generate_pseudo_labels.py produces (datasets/pseudo_labels_review_queue.
json). Shows one image at a time with its queued boxes overlaid; click
a box to accept/reject/reclassify it, or drag on empty space to draw a
new box the queue missed entirely. Progress autosaves as you go --
close it anytime, rerun later, it picks up where you left off.

V5 NOTE: full rewrite of the interaction layer, OpenCV's highgui window
+ keyboard commands (V1-V4) replaced with a Tkinter GUI. Same
review_progress.json shape, same pseudo_labels_reviewed/ output tree,
same --source/--cls/--show-all/--show-original flags. New runtime
dependency: Pillow (PIL).

V6 NOTE -- bugfix + zoom/pan release.
  1. FIXED: item_key() used to hash a queue item's CURRENT "box" field.
     Dragging a box changed its own key mid-session, so self.decisions
     (and the progress file) still had the OLD key -> KeyError on the
     next redraw(), which cascaded into _get_box() returning garbage
     for manual boxes too (TypeError: list indices ... not str). Fixed
     by deriving item_key() from item["_orig_box"] -- captured once,
     read-only, right after the queue JSON loads -- instead of the
     live/editable "box" field.
  2. FIXED: _get_box()/_set_box() silently fell through to the wrong
     branch on a lookup miss instead of failing loudly.
  3. ADDED: queue-item box positions now actually persist across
     relaunches (previously only the decision did).
  4. ADDED: zoom (mouse wheel / +/- / 0-to-fit) + pan (scrollbars,
     middle-click-drag).

V7 NOTE -- workflow/safety release.
  1. DRAGGING IS NOW LOCKED DOWN. Only boxes YOU drew (cyan, "manual")
     can be dragged by default. The model's own proposed boxes (queue
     items) can no longer be bumped out of place by an accidental
     click-drag -- clicking one always just opens its menu. If a queue
     box genuinely needs repositioning before you accept it, its menu
     has an explicit "Enable Reposition (then drag once)" -- one
     deliberate click arms exactly one drag on that one box, then it
     locks again. Original dataset labels (magenta) were never
     clickable/draggable in the first place (no code path registers
     them as interactive) -- that's unchanged, and is exactly the
     guarantee you asked for: nothing you didn't explicitly draw can be
     moved by mistake.
  2. ADDED: a small per-model detection-count panel under the toolbar
     ("this image: yv=3 rf=2 yc=0 dn=4") so a model contributing
     nothing to an image is visible at a glance (shown in red) instead
     of just... not being there. Counts are independent of the
     show/hide checkboxes -- they reflect what's actually in the queue
     for this image, not just what's currently displayed.
  3. ADDED: real autosave. Every accept/reject/class-change/manual add/
     delete/reposition writes to disk automatically (debounced ~1s so
     rapid clicking doesn't thrash the file; navigating to another
     image or closing the window always flushes immediately,
     un-debounced). The old "save before continuing?" prompt is gone --
     there's nothing left to prompt about, your work is already on
     disk. A manual Save button/File>Save/'s' key still exist if you
     just want the reassurance.
  4. ADDED: a genuinely separate "done" list. Once every box in an
     image has a decision (nothing pending), that image's path is
     written to datasets/review_completed.json. The normal review
     session never shows images from that list (so you can't
     accidentally re-review something someone else should QA), and a
     new --qa flag opens a session sourced ONLY from that list, for a
     second person to check over already-decided images. An image
     drops back out of the completed list automatically if you (or the
     QA reviewer) reset something back to pending.
  5. CHANGED DEFAULT: original dataset labels are now shown by default
     (they used to require --show-original). Since the whole point of
     showing them is "don't manually re-label something that's already
     in the real dataset", defaulting to hidden worked against that.
     Pass --hide-original if you want the old default back.

V8 NOTE -- deletion, undo/redo, and speed release.
  1. ADDED: queue (model) boxes can now be genuinely DELETED, not just
     rejected. Reject already excluded a box from the accepted output,
     but kept it on screen forever as a red box -- there was no way to
     get it out of your visual field the way manual boxes could be
     removed with their menu's Delete option. "Delete" is now a new
     decision state that behaves exactly like reject for output
     purposes (never in accepted_items()) but is hidden from the
     canvas by default. A "Show deleted (recoverable)" checkbox in the
     toolbar reveals them again (drawn grey/dashed) so you can hit
     "Reset to Pending" on one if you deleted it by mistake -- nothing
     is ever unrecoverable short of you never re-showing the checkbox.
     Original dataset labels (magenta) are UNCHANGED by this -- they
     still have no decision state at all and still cannot be selected,
     dragged, or deleted; this whole feature only applies to the 4
     models' queue boxes, exactly as asked.
  2. ADDED: full undo/redo. Every mutation (accept/reject/delete/reset,
     class override, manual add/delete/class-change, and box drags)
     snapshots the per-image state onto an undo stack first via
     _push_undo(). Ctrl+Z / Ctrl+Y (and Ctrl+Shift+Z) walk backward and
     forward through it; also in File menu. Deliberately scoped PER
     IMAGE -- the stack resets on navigation, since undoing across
     images you've already left would be more confusing than useful
     for a "fix my last few clicks" tool. Undoing/redoing counts as an
     edit and autosaves like anything else.
  3. ADDED: fast mode. Left-click a queue box = Accept, right-click =
     Reject, no menu popup -- for images where you're mostly rubber-
     stamping. Off by default (View menu / toolbar checkbox); when off,
     both clicks open the normal menu as before. Middle-click and drag
     behavior are unaffected either way.
  4. ADDED: bulk actions. "Accept all pending" / "Reject all pending"
     buttons act on every currently-pending queue box in the image at
     once (asks for confirmation first), each pushing a single undo
     step so one Ctrl+Z reverts the whole bulk action.
  5. ADDED: number keys 1-9 apply the corresponding class (in
     CLASS_NAMES order) to the currently selected box, if any --
     avoids the cascading Change Class menu for common classes.
  6. ADDED: optional auto-advance -- when the last pending queue box on
     an image gets a decision, automatically move to the next image
     after a short delay. Toggle in the toolbar, off by default.
  7. ADDED: a small session streak counter in the status bar ("5 in a
     row") that increments on consecutive accept/reject decisions and
     resets on undo, mostly so long sessions have some small feedback
     loop.

WHAT CHANGED, keyboard command -> new equivalent:
  click box, cycle       -> click box: pops up a menu with Accept /
  pending/accept/reject     Reject / Reset to Pending / Change Class /
                             (queue boxes only) Delete
  drag-draw + digit key   -> drag-draw on empty canvas -> release opens
                              a class-picker menu at the cursor (works
                              for any number of boxes, one at a time)
  (nothing before)        -> click-and-drag ON a manual (cyan) box
                              moves it. Queue boxes are click-only
                              unless explicitly armed for one reposition
                              via their menu (see V7 note above).
  (nothing before)        -> per-box menu also offers "Change Class"
                              and, for manual boxes, "Delete"; queue
                              boxes now also get "Delete (hide from
                              view)" (see V8 note #1)
  a / r (accept/reject     -> still available: the per-box menu's
    all pending)              Accept/Reject apply to one box at a time,
                              or use Fast Mode (V8) / bulk buttons (V8)
  u (undo last manual box) -> real per-image undo/redo now exists
                              (Ctrl+Z / Ctrl+Y, see V8 note #2); manual
                              boxes still also have a direct Delete
                              option in their menu
  o (toggle original)      -> "Original labels" checkbox, ON by default
                              (magenta, non-clickable, reference-only,
                              reloaded fresh from disk each time an
                              image opens -- see load_original_labels())
  (nothing before)         -> per-model visibility checkboxes + a
                              detection-count readout per model for the
                              current image, plus "All"/"None" buttons
  n/p (next/prev)          -> Next / Previous buttons / Left / Right
                              arrow keys. Autosaves before moving, no
                              prompt needed. Can also happen
                              automatically -- see Auto-advance (V8).
  q/Esc (save+quit)        -> File > Save & Quit, or the window's close
                              button (autosaves either way)
  h (toggle help)          -> Help > Controls
  (nothing before)         -> mouse wheel = zoom (centered on cursor),
                              '+'/'-' = zoom, '0' = fit to window,
                              middle-click-drag or scrollbars = pan
  (nothing before)         -> Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z = undo/redo
  (nothing before)         -> 1-9 = apply Nth class to selected box
  (nothing before)         -> Fast Mode: left-click queue box = accept,
                              right-click = reject (no menu)

WHY A SEPARATE OUTPUT TREE (see generate_pseudo_labels.py's module
docstring for the full reasoning): this writes to datasets/
pseudo_labels_reviewed/<SOURCE>/..., NOT datasets/pseudo_labels/ (the
auto-accept tree). generate_pseudo_labels.py overwrites pseudo_labels/
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
                                             # fully decided
    python review_labels.py --hide-original # don't overlay existing
                                             # real-dataset labels
"""

import argparse
import copy
import json
import tkinter as tk
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

# Must match the keys cross_reference_gaps.py writes into each queue
# item's "votes" dict.
MODEL_KEYS = ["yolo_visdrone", "rfdetr_visdrone", "yolo_coco", "dino"]
MODEL_SHORT = {"yolo_visdrone": "yv", "rfdetr_visdrone": "rf",
               "yolo_coco": "yc", "dino": "dn"}

COLOR_PENDING = "#ffa500"    # orange
COLOR_ACCEPT = "#00c800"     # green
COLOR_REJECT = "#dc0000"     # red
COLOR_DELETED = "#777777"    # grey, dashed -- hidden by default, recoverable
COLOR_MANUAL = "#00c8ff"     # cyan-ish
COLOR_ORIGINAL = "#ff00ff"   # magenta, existing real-dataset labels (reference only)
COLOR_ARMED = "#ffff00"      # yellow, queue box temporarily armed for one reposition

DISPLAY_MAX_W = 1280
DISPLAY_MAX_H = 860

MIN_ZOOM = 1.0     # can't zoom out past "fit to window"
MAX_ZOOM = 8.0
ZOOM_STEP = 1.15

AUTOSAVE_DEBOUNCE_MS = 1000
AUTO_ADVANCE_DELAY_MS = 400
UNDO_LIMIT = 50

CLASS_NAMES = list(UNIFIED_CLASSES)

# Decision states a queue box can be in. "deleted" behaves like "reject"
# for output purposes (see accepted_items()) but is hidden from the
# canvas unless the "Show deleted" checkbox is on.
DECISION_STATES = ("pending", "accept", "reject", "deleted")


def item_key(item: dict) -> str:
    """Stable id for a queue item -- (image, cls, rounded ORIGINAL box)
    is deterministic across runs since it's derived from item["_orig_box"],
    which is captured once right after the queue JSON loads and never
    touched again. Deliberately NOT derived from item["box"] (mutable --
    see V6 note in the module docstring for why that used to crash)."""
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


class ReviewApp:
    """Owns the Tkinter window and all mutable state for the image
    currently on screen. One instance runs the whole review session;
    _load_image() swaps in a new image's state as you navigate."""

    def __init__(self, root, image_keys, by_image, progress, reviewed_root,
                 progress_path, completed_path, completed_set,
                 show_original_default: bool, qa_mode: bool):
        self.root = root
        self.image_keys = image_keys
        self.by_image = by_image
        self.progress = progress
        self.reviewed_root = reviewed_root
        self.progress_path = progress_path
        self.completed_path = completed_path
        self.completed_set = completed_set
        self.qa_mode = qa_mode

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
        self.drag = None
        self.selected_key = None
        self._rubber_id = None
        self.reposition_armed = None   # (kind, key) currently allowed one drag

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

        self._build_ui()
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
        edit_menu.add_command(label="Accept All Pending", command=self.bulk_accept_pending)
        edit_menu.add_command(label="Reject All Pending", command=self.bulk_reject_pending)
        menubar.add_cascade(label="Edit", menu=edit_menu)

        view_menu = tk.Menu(menubar, tearoff=0)
        view_menu.add_checkbutton(label="Show Original Labels",
                                   variable=self.show_original, command=self.redraw)
        view_menu.add_checkbutton(label="Show All Queue Boxes",
                                   variable=self.show_all_queue, command=self.redraw)
        view_menu.add_checkbutton(label="Show Deleted (recoverable)",
                                   variable=self.show_deleted, command=self.redraw)
        view_menu.add_separator()
        for m in MODEL_KEYS:
            view_menu.add_checkbutton(label=f"Show {m} detections",
                                       variable=self.model_vars[m], command=self.redraw)
        view_menu.add_separator()
        view_menu.add_checkbutton(label="Fast Mode (click=accept, right-click=reject)",
                                   variable=self.fast_mode)
        view_menu.add_checkbutton(label="Auto-advance when image is fully decided",
                                   variable=self.auto_advance)
        view_menu.add_separator()
        view_menu.add_command(label="Zoom In\t+", command=lambda: self._zoom_centered(ZOOM_STEP))
        view_menu.add_command(label="Zoom Out\t-", command=lambda: self._zoom_centered(1 / ZOOM_STEP))
        view_menu.add_command(label="Fit to Window\t0", command=self.reset_zoom)
        menubar.add_cascade(label="View", menu=view_menu)

        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="Controls", command=self.show_help_dialog)
        menubar.add_cascade(label="Help", menu=help_menu)

        self.root.config(menu=menubar)

        toolbar = ttk.Frame(self.root)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)
        ttk.Label(toolbar, text="Show:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Checkbutton(toolbar, text="Original labels (real dataset, read-only)",
                         variable=self.show_original, command=self.redraw).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(toolbar, text="All queue boxes", variable=self.show_all_queue,
                         command=self.redraw).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(toolbar, text="Deleted", variable=self.show_deleted,
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

        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(4, 0))

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

        # Fast mode: right-click a queue box = instant reject.
        self.canvas.bind("<ButtonPress-3>", self.on_right_click)

        # Panning: middle-click-drag.
        self.canvas.bind("<ButtonPress-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan_drag)

        # Zoom: mouse wheel (Windows/Mac) and Button-4/5 (Linux/X11).
        self.canvas.bind("<MouseWheel>", self.on_mousewheel)
        self.canvas.bind("<Button-4>", self.on_mousewheel)
        self.canvas.bind("<Button-5>", self.on_mousewheel)

        self.status_var = tk.StringVar()
        ttk.Label(self.root, textvariable=self.status_var, anchor="w").pack(
            side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 2))

        nav = ttk.Frame(self.root)
        nav.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)
        ttk.Button(nav, text="\u25c0 Previous", command=self.go_prev).pack(side=tk.LEFT)
        ttk.Button(nav, text="Next \u25b6", command=self.go_next).pack(side=tk.LEFT, padx=4)
        ttk.Button(nav, text="Save Now", command=self.save_current).pack(side=tk.LEFT, padx=12)
        self.progress_var = tk.StringVar()
        ttk.Label(nav, textvariable=self.progress_var).pack(side=tk.RIGHT)

        self.root.bind("<Key>", self.on_key)
        self.root.bind("<Control-z>", lambda e: self.undo())
        self.root.bind("<Control-y>", lambda e: self.redo())
        self.root.bind("<Control-Z>", lambda e: self.redo())  # Ctrl+Shift+Z on many platforms

    def _select_all_models(self):
        for v in self.model_vars.values():
            v.set(True)
        self.redraw()

    def _select_no_models(self):
        for v in self.model_vars.values():
            v.set(False)
        self.redraw()

    # ------------------------------------------------------------------
    # Image loading / navigation
    # ------------------------------------------------------------------

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
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.pil_img_full = Image.fromarray(rgb)

        self.base_scale = min(DISPLAY_MAX_W / w, DISPLAY_MAX_H / h, 1.0)
        self.zoom = 1.0

        base_w = max(1, int(w * self.base_scale))
        base_h = max(1, int(h * self.base_scale))
        self.canvas.config(width=base_w, height=base_h)

        self.decisions = {}
        self.model_counts = {m: 0 for m in MODEL_KEYS}
        for it in self.queue_items:
            it["_orig_box"] = list(it.get("_orig_box", it["box"]))
            k = item_key(it)
            saved = self.progress.get(k, {})
            self.decisions[k] = saved.get("decision", "pending")
            if "box" in saved:
                it["box"] = list(saved["box"])
            votes = it.get("votes", {})
            for m in MODEL_KEYS:
                if votes.get(m) is not None:
                    self.model_counts[m] += 1

        self.class_overrides = dict(self.progress.get(f"__override__{image_key}", {}))
        self.manual_boxes = [dict(b) for b in self.progress.get(f"__manual__{image_key}", [])]
        # Manual boxes are identified by a stable "_id", not their position
        # in the list -- deleting/editing box #2 of 5 must not silently
        # shift what boxes #3-5 refer to for the rest of the session.
        # Backfill ids for saves written before this existed.
        next_id = 0
        for mb in self.manual_boxes:
            if "_id" not in mb:
                mb["_id"] = next_id
            next_id = max(next_id, mb["_id"] + 1)
        self._next_manual_id = next_id

        real_label_path = gpl.image_path_to_label_path(Path(image_key))
        self.original_boxes = load_original_labels(real_label_path, w, h)

        self.dirty = False
        self.selected_key = None
        self.drag = None
        self.reposition_armed = None

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
        elif event.keysym == "Escape" and self.drag and self.drag.get("mode") == "draw":
            self.canvas.delete(self._rubber_id)
            self.drag = None
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
    # Saving / autosave / completed-list bookkeeping
    # ------------------------------------------------------------------

    def accepted_items(self) -> list[dict]:
        out = []
        for it in self.queue_items:
            k = item_key(it)
            if self.decisions[k] == "accept":
                cls = self.class_overrides.get(k, it["cls"])
                out.append({"cls": cls, "box": it["box"]})
        out.extend({"cls": mb["cls"], "box": mb["box"]} for mb in self.manual_boxes)
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
        lines = [gpl.box_to_yolo_line(a["cls"], a["box"], w, h) for a in accepted]
        reviewed_label_path.parent.mkdir(parents=True, exist_ok=True)
        reviewed_label_path.write_text("\n".join(lines) + "\n")

    def _update_completed_list(self):
        """An image counts as 'done' once every queue item on it has a
        real decision (nothing pending -- "deleted" counts as decided,
        same as "accept"/"reject"). Moves it into (or out of, if
        something got reset to pending) review_completed.json -- a
        separate, explicit list a second reviewer can open with --qa,
        instead of everything living undifferentiated in progress.json."""
        fully_decided = all(v != "pending" for v in self.decisions.values())
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

    def _flush(self):
        """Writes everything for the current image to disk: per-item
        decisions + current box position into progress.json, the
        accepted-boxes label file into pseudo_labels_reviewed/, and
        updates the completed-images list."""
        for it in self.queue_items:
            k = item_key(it)
            self.progress[k] = {
                "decision": self.decisions[k],
                "image": self.image_key,
                "cls": it["cls"],
                "box": it["box"],
            }
        self.progress[f"__manual__{self.image_key}"] = self.manual_boxes
        self.progress[f"__override__{self.image_key}"] = self.class_overrides
        self._write_reviewed_labels()
        save_json(self.progress_path, self.progress)
        return self._update_completed_list()

    def _mark_dirty(self):
        """Call after any edit. Schedules an autosave a short moment
        from now -- debounced so a burst of clicks (accepting several
        boxes in a row) doesn't write the file after every single one,
        while still guaranteeing nothing survives more than ~1s
        unsaved if the app were to crash."""
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
        """Used when navigating away or closing -- don't wait out the
        debounce timer, write immediately."""
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
        screen for a beat before it jumps)."""
        if not self.auto_advance.get():
            return
        fully_decided = all(v != "pending" for v in self.decisions.values())
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
        for k in pending_keys:
            self.decisions[k] = "accept"
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()

    def bulk_reject_pending(self):
        pending_keys = [k for k, v in self.decisions.items() if v == "pending"]
        if not pending_keys:
            return
        if not messagebox.askyesno(
                "Reject all pending",
                f"Reject all {len(pending_keys)} pending box(es) in this image?"):
            return
        self._push_undo()
        for k in pending_keys:
            self.decisions[k] = "reject"
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()

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

    def redraw(self):
        self.canvas.delete("all")
        self.item_map = {}
        self.canvas.create_image(0, 0, anchor="nw", image=self.tk_img, tags="bg")

        # Original real-dataset labels: reference-only. Note these are
        # NEVER passed kind=/key= below, so they never enter self.item_map
        # -- there is no code path that lets a mouse click or drag hit
        # one. That's what guarantees they can't be moved OR deleted by
        # mistake -- the V8 delete feature only touches queue items,
        # which are the only boxes that ever get registered here.
        if self.show_original.get():
            for ob in self.original_boxes:
                self._draw_box(ob["box"], COLOR_ORIGINAL, f"{ob['cls']} [orig, read-only]", dash=(3, 2))

        for it in self.queue_items:
            if not self._queue_item_visible(it):
                continue
            k = item_key(it)
            state = self.decisions[k]
            color = {"pending": COLOR_PENDING, "accept": COLOR_ACCEPT,
                     "reject": COLOR_REJECT, "deleted": COLOR_DELETED}[state]
            cls_name = self.class_overrides.get(k, it["cls"])
            label = f"{cls_name} {gpl.votes_str(it)} [{state}]"
            self._draw_box(it["box"], color, label, key=k, kind="queue",
                           dash=(2, 2) if state == "deleted" else None)

        for mb in self.manual_boxes:
            self._draw_box(mb["box"], COLOR_MANUAL, f"{mb['cls']} [manual, draggable]",
                           key=mb["_id"], kind="manual")

        self._update_status()

    def _update_status(self):
        n_pending = sum(1 for v in self.decisions.values() if v == "pending")
        n_accept = sum(1 for v in self.decisions.values() if v == "accept")
        n_reject = sum(1 for v in self.decisions.values() if v == "reject")
        n_deleted = sum(1 for v in self.decisions.values() if v == "deleted")
        unsaved = "  [saving\u2026]" if self.dirty else "  [saved]"
        streak_txt = f"  |  streak={self.streak}" if self.streak > 1 else ""
        self.status_var.set(
            f"{Path(self.image_key).name}  |  pending={n_pending} accept={n_accept} "
            f"reject={n_reject} deleted={n_deleted} manual={len(self.manual_boxes)} "
            f"orig={len(self.original_boxes)}  |  zoom={int(self.zoom * 100)}%"
            f"{streak_txt}{unsaved}")
        done = len(self.completed_set)
        total = len(self.image_keys) if self.qa_mode else (len(self.image_keys) + done)
        self.progress_var.set(
            f"Image {self.idx + 1}/{len(self.image_keys)}    "
            f"Fully reviewed overall: {done}"
            + ("" if self.qa_mode else f"/{total}"))

    # ------------------------------------------------------------------
    # Box get/set (shared by drag-move and the context menus)
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
        self._push_undo()
        self.decisions[key] = state
        if state in ("accept", "reject"):
            self.streak += 1
        else:
            self.streak = 0
        self._mark_dirty()
        self.redraw()
        self._maybe_auto_advance()

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

    def _delete_manual(self, manual_id):
        self._push_undo()
        self.manual_boxes = [mb for mb in self.manual_boxes if mb["_id"] != manual_id]
        self._mark_dirty()
        self.redraw()

    def _add_manual_box(self, cls, box):
        """Appends a new manually-drawn box. No cap on how many you can
        add -- call this as many times as you like (one per drag-draw on
        empty canvas). Each gets its own stable id so adding/deleting
        others around it never disturbs it."""
        self._push_undo()
        self.manual_boxes.append({"cls": cls, "box": box, "_id": self._next_manual_id})
        self._next_manual_id += 1
        self._mark_dirty()
        self.redraw()

    def _arm_reposition(self, kind, key):
        self.reposition_armed = (kind, key)
        self.redraw()

    # ------------------------------------------------------------------
    # Menus
    # ------------------------------------------------------------------

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
            menu.add_command(label="Enable Reposition (then drag once)",
                              command=lambda: self._arm_reposition(kind, key))
            menu.add_separator()
            if state == "deleted":
                menu.add_command(label="Undelete (reset to Pending)",
                                  command=lambda: self._set_decision(key, "pending"))
            else:
                menu.add_command(label="Delete (hide from view)",
                                  command=lambda: self._set_decision(key, "deleted"))
        else:
            class_menu = tk.Menu(menu, tearoff=0)
            for c in CLASS_NAMES:
                class_menu.add_command(label=c, command=lambda c=c: self._set_manual_class(key, c))
            menu.add_cascade(label="Change Class", menu=class_menu)
            menu.add_command(label="Delete", command=lambda: self._delete_manual(key))
        menu.add_separator()
        menu.add_command(label="Cancel")
        menu.tk_popup(x_root, y_root)

    def show_help_dialog(self):
        messagebox.showinfo("Controls", (
            "Click a box:                 open its menu -- Accept / Reject / "
            "Reset to Pending / Change Class, plus Delete for boxes you drew and "
            "for model (queue) boxes\n\n"
            "Drag ON a box:                only works for boxes YOU drew (cyan). "
            "The model's own proposed boxes can't be bumped by accident -- click "
            "one and use \"Enable Reposition\" in its menu if it genuinely needs "
            "moving, then drag it once (it locks again automatically). Original "
            "dataset labels (magenta) can never be clicked, dragged, or deleted "
            "at all.\n\n"
            "Drag on empty area:           draw a new box, then pick its class "
            "from the popup. Add as many as you like.\n\n"
            "Delete (queue boxes):         hides the box from the canvas and "
            "excludes it from output, but it's fully recoverable -- check "
            "\"Show Deleted\" to see grey/dashed deleted boxes again and reset "
            "one back to Pending if you deleted it by mistake.\n\n"
            "Fast Mode:                    left-click a queue box = Accept, "
            "right-click = Reject, no menu popup. Toggle it in the toolbar or "
            "View menu. Manual boxes and Enable-Reposition boxes are unaffected.\n\n"
            "Accept All / Reject All Pending:  bulk-decide every still-pending "
            "queue box on the current image at once (confirms first). Counts as "
            "one undo step.\n\n"
            "Undo / Redo:                  Ctrl+Z / Ctrl+Y (or Ctrl+Shift+Z), "
            "also in the Edit menu and toolbar. Covers every edit on the current "
            "image -- decisions, class changes, manual add/delete, drags. Resets "
            "when you move to another image.\n\n"
            "1-9 keys:                     apply the Nth class (in class-list "
            "order) to whichever box you most recently clicked.\n\n"
            "Mouse wheel:                  zoom in/out, centered on the cursor\n"
            "+ / - keys:                   zoom in/out\n"
            "0 key / \"Fit\" button:         reset to fit-to-window\n"
            "Middle-click + drag / scrollbars: pan around when zoomed in\n\n"
            "\"Original labels\" checkbox:  the real dataset's existing labels, "
            "shown ON by default so you don't accidentally re-draw something "
            "that's already labeled\n"
            "Model checkboxes + \"All\"/\"None\": choose which models' proposed "
            "boxes are visible. The counts next to each model's name (above the "
            "image) show how many boxes THAT model actually proposed for this "
            "image -- shown in red/bold if it's zero, so a model silently missing "
            "an image is obvious.\n\n"
            "Auto-advance:                 optional -- once every box on the "
            "image has a decision, automatically jump to the next image.\n\n"
            "Everything autosaves as you go (within about a second of any "
            "change) -- Next/Previous/closing the window never lose work, and "
            "there's no save prompt because there's nothing left unsaved to ask "
            "about. \"Save Now\" / 's' just forces it immediately.\n\n"
            "Once every box on an image has a decision, that image moves into "
            "a separate completed list (review_completed.json) and won't show "
            "up again in a normal review session. Run with --qa to open a "
            "session sourced only from that list, e.g. for a second reviewer.\n"
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

    def on_mouse_down(self, event):
        cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        current = self.canvas.find_withtag("current")
        hit = self.item_map.get(current[0]) if current else None
        if hit:
            kind, key = hit
            if kind == "manual" or self.reposition_armed == (kind, key):
                self.drag = {"mode": "move", "kind": kind, "key": key,
                             "start": (cx, cy), "moved": False, "snapshotted": False}
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
        # "queue_click" / "fast_accept" modes: intentionally ignore drag motion.

    def on_mouse_up(self, event):
        if not self.drag:
            return
        cx, cy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        mode = self.drag["mode"]

        if mode == "draw":
            self.canvas.delete(self._rubber_id)
            sx, sy = self.drag["start"]
            if abs(cx - sx) > 6 and abs(cy - sy) > 6:
                box = [min(sx, cx) / self.scale, min(sy, cy) / self.scale,
                       max(sx, cx) / self.scale, max(sy, cy) / self.scale]
                self._open_class_menu(event.x_root, event.y_root,
                                       on_pick=lambda cls: self._add_manual_box(cls, box))
        elif mode == "move":
            if self.reposition_armed == (self.drag["kind"], self.drag["key"]):
                self.reposition_armed = None   # one-shot: disarm after use
                self.redraw()
            if not self.drag["moved"]:
                self._open_box_context_menu(event.x_root, event.y_root,
                                             self.drag["kind"], self.drag["key"])
        elif mode == "queue_click":
            self._open_box_context_menu(event.x_root, event.y_root,
                                         self.drag["kind"], self.drag["key"])
        elif mode == "fast_accept":
            self.selected_key = (self.drag["kind"], self.drag["key"])
            self._set_decision(self.drag["key"], "accept")

        self.drag = None


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
                         "check over. Ignores --show-all.")
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
        all_items = json.load(f)
    progress = load_json_dict(progress_path)
    completed_set = load_completed(completed_path)

    if args.source:
        all_items = [it for it in all_items if it["source"] == args.source]
    if args.cls:
        all_items = [it for it in all_items if it["cls"] == args.cls]

    # Capture each item's original, immutable box up front -- item_key()
    # depends on this, not on the (editable) "box" field. Must happen
    # before anything below calls item_key().
    for it in all_items:
        it["_orig_box"] = list(it["box"])

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
                return any(progress.get(item_key(it), {}).get("decision", "pending") == "pending"
                           for it in by_image[image_key])
            image_keys = [k for k in image_keys if has_pending(k)]
        if not image_keys:
            print("Nothing to review (try --show-all, or check --source/--cls "
                  "filters -- everything matching may already be in the "
                  "completed list; see --qa).")
            return

    print(f"{len(image_keys)} images to review. Opening review window...")

    root = tk.Tk()
    root.title("Label Review")
    ReviewApp(root, image_keys, by_image, progress, reviewed_root, progress_path,
              completed_path, completed_set,
              show_original_default=not args.hide_original, qa_mode=args.qa)
    root.mainloop()


if __name__ == "__main__":
    main()