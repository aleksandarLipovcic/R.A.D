"""
review_batch_assigner.py -- One tool covering the WHOLE multi-reviewer
loop: handing out review work in small, safe chunks, AND recombining
finished work back into the master dataset.

Two subcommands:

  assign  -- Tkinter checkbox GUI over datasets/
             pseudo_labels_review_queue.json (generate_pseudo_labels.py's
             output). Pick which remaining SEQUENCES go to which
             person, in ~200-300 image batches ("one or two datasets'
             worth" -- not the whole multi-thousand-image backlog at
             once), while permanently guaranteeing no image is ever
             handed to two people.

  merge   -- After people run review_labels.py in their own folders and
             you've collected their folders back into --handoff-dir, a
             Tkinter GUI lists every returned folder found there, lets
             you check which people to merge this round, and recombines
             their datasets/review_progress.json,
             datasets/review_completed.json, and
             datasets/pseudo_labels_reviewed/ into your master
             datasets/ tree -- validating every entry against who was
             actually assigned what, and showing a conflict-resolution
             dialog (keep OLD or NEW) for anything claimed twice.

This replaces four separate scripts (split_review_assignments.py,
assign_review_batches.py, assign_review_batches_gui.py,
merge_review_results.py) with one file -- same behavior, just
consolidated so the project doesn't accumulate near-duplicate modules.

PARTITION UNIT: whole SEQUENCES (source + sequence id), never
individual frames -- review_labels.py's cross-frame sync only looks at
neighbor frames within the same sequence, so splitting one across two
people would silently break sync at the boundary; reviewing a whole
clip together also gives visual continuity a scattered set of frames
wouldn't. parse_seq_key() below is kept in sync with review_labels.py's
own SEQ_FRAME_RE / parse_seq_frame() by design.

PENDING-ONLY GUARANTEE: images already fully reviewed (present in
datasets/review_completed.json) are excluded from sequence
construction entirely, before assignment_state is even consulted --
they can never appear in the checklist and can never be bundled into
anyone's package. This covers work completed before this tool existed
or merged in some other way, not just batches this tool itself handed
out. The GUI's overview line shows the full breakdown: total images in
the set, how many are already reviewed, how many are pending, and how
many pending images are still unassigned.

STATE FILE (--state-file, default <out-dir>/assignment_state.json):
  The permanent ledger of every batch ever handed out: which
  sequences/images, to whom, when. "Remaining" in the GUI always means
  "queue items minus everything ever recorded here" -- not just minus
  what's staged this session -- so re-running this tool next week, or
  for a different person, can never re-assign an image someone already
  has. Back this file up; losing it doesn't corrupt anyone's review
  progress, but this tool would no longer know what's already out.

  assignments_master.json ({"people": {person: [image, ...]}}) is kept
  in sync on every finalize -- this is what the `merge` subcommand
  validates every returned entry against.

INCREMENTAL / TOP-UP BEHAVIOR: assigning a second batch to a person who
already has a package does NOT create a second folder -- it merges the
new items into their EXISTING --out-dir/<person>/datasets/
pseudo_labels_review_queue.json (list extend + dedupe) and copies only
the new images/labels alongside what's already there. Their
review_progress.json is keyed per-item/per-image, so this is safe:
existing decisions are untouched, new items just show up as pending
next time they launch review_labels.py. The log panel lists exactly
which images are new in a top-up, in case you'd rather send just those
instead of the whole folder again.

PORTABILITY -- CHECK BEFORE HANDING PACKAGES OUT: every person's copy
of prepare_datasets.py must resolve DATASETS_DIR to wherever THEY put
the datasets/ folder on THEIR machine. If prepare_datasets.DATASETS_DIR
is currently a fixed absolute path (e.g. baked in as your own Windows
folder), change it to something relative to the script's own location
first, e.g.:
    DATASETS_DIR = Path(__file__).resolve().parent / "datasets"
so each person's package just works the moment they unzip it.

HOW TO USE THE GUI (assign):
  1. The left panel lists every sequence still unassigned, as
     checkboxes, biggest (by item count) first: "SOURCE::seq_id   N
     images   M items". Filter by typing in the search box.
  2. Type/pick a person, set a target chunk size, click "Propose ~N"
     to auto-check a batch near that size -- or just check boxes by
     hand. The running total updates live.
  3. Click "Add to pending batch" -- stages it (no disk writes yet)
     and removes those sequences from the checklist.
  4. Repeat for other people/batches. The "Pending batches" panel on
     the right shows everything staged this session, with an Undo
     button.
  5. Click "Finalize" to actually write/top-up each person's package
     folder and save the state file. You can finalize multiple times
     in one session -- pending clears after each finalize, and you can
     keep assigning more afterward.

WHAT `merge` MERGES, per person, and HOW:

  1. datasets/review_progress.json
     Every key is validated against assignments_master.json before
     being merged in: box-decision keys (item_key() shape, "image|cls|
     box") have their image recovered by splitting on the FIRST "|"
     (image paths can't contain "|", so this is unambiguous); "__manual__
     <image>" / "__override__<image>" keys have the image recovered by
     stripping the known prefix. Any key whose image ISN'T in that
     person's assignment is reported and skipped instead of merged --
     that's the "someone reviewed the wrong folder" safety net.

  2. datasets/review_completed.json
     Same per-image validation, then unioned. (This is deliberately a
     raw union, not a re-derivation of completeness -- run
     review_labels.py once normally against the merged master files
     afterwards and its own startup self-heal, compute_completed_set(),
     will recompute this from progress + the live queue anyway. Don't
     hand-edit around that; let it do its job.)

  3. datasets/pseudo_labels_reviewed/<SOURCE>/...
     For every image assigned to a person, this computes BOTH that
     person's copy of the reviewed-label path and the master copy,
     using the exact same gpl.image_path_to_label_path() /
     gpl.mirror_under_pseudo_root() calls review_labels.py itself uses
     -- nothing here guesses at directory-structure conventions
     independently. If the image is in that person's OWN returned
     review_completed.json (i.e. they finished it) and no reviewed
     label file exists for it, that means every box was rejected/
     deleted -- treated as authoritative, and any stale master label
     file for that image is removed. If the image ISN'T in their
     completed list yet, a missing label file just means "not done",
     and the master is left alone.

  CONFLICTS: since assignment is disjoint by construction, two
  different people should never legitimately touch the same key or
  image. If they do (hand-edited assignments, a re-used folder, etc.),
  `merge` opens a dialog listing every conflicting key/image and lets
  you pick, one at a time, whether to keep the OLD (first-seen) or NEW
  (later) person's value -- nothing is auto-resolved silently.

HOW TO USE THE GUI (merge):
  1. Collect each person's returned folder back into --handoff-dir
     (people sometimes rename this folder "merge/" once it's being used
     for hand-back instead of hand-out -- same folder either way).
  2. Run `merge`. The GUI scans --handoff-dir and lists every person
     folder found, with how many images they were assigned, how many
     they've marked complete, and how many are still pending.
  3. Check the people whose returned work you want to merge this round
     (Select all / Clear all help for large rosters).
  4. Click "Update". If everything is clean it merges straight away. If
     any review_progress.json key or image is claimed by more than one
     selected person, a dialog appears first -- choose OLD or NEW for
     each one, then continue.
  5. The log panel reports exactly what was merged, what was skipped
     (e.g. an entry that referenced an image outside that person's
     assignment), and confirms the master files were backed up first.

USAGE:
    python review_batch_assigner.py assign --out-dir handoff
    python review_batch_assigner.py assign --out-dir handoff --target-chunk-images 250
    python review_batch_assigner.py merge --handoff-dir handoff
"""

import argparse
import json
import re
import shutil
import sys
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from pathlib import Path

import prepare_datasets
import generate_pseudo_labels as gpl

SCRIPT_FILES = ["prepare_datasets.py", "class_map.py",
                 "generate_pseudo_labels.py", "review_labels.py"]

DEFAULT_TARGET_CHUNK_IMAGES = 250

# Kept in sync with review_labels.py's SEQ_FRAME_RE / parse_seq_frame()
# by design -- same heuristic, so the sequence boundaries used here to
# split work always match the ones review_labels.py's cross-frame sync
# uses to find neighbor frames.
SEQ_FRAME_RE = re.compile(r"(?:^|_)(?P<seq>[A-Za-z0-9]+)_img(?P<frame>\d+)_")
FRAME_DIGITS_RE = re.compile(r"\d+")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    a = sub.add_parser("assign", help="Launch the checkbox GUI to hand out batches.")
    a.add_argument("--queue-file", default="pseudo_labels_review_queue.json")
    a.add_argument("--out-dir", default="handoff")
    a.add_argument("--state-file", default=None,
                   help="Default: <out-dir>/assignment_state.json")
    a.add_argument("--scripts-dir", default=".",
                   help="Where to find prepare_datasets.py/class_map.py/"
                        "generate_pseudo_labels.py/review_labels.py to "
                        "copy into each package.")
    a.add_argument("--target-chunk-images", type=int, default=DEFAULT_TARGET_CHUNK_IMAGES,
                   help="Default proposed chunk size in images (~200-300 "
                        "recommended -- roughly one or two source datasets' "
                        "worth). Overridable per-assignment in the GUI.")

    m = sub.add_parser("merge", help="GUI: recombine returned folders into the master datasets/.")
    m.add_argument("--people", nargs="*", default=None,
                   help="Optional: pre-check only these people in the GUI "
                        "checklist instead of every returned folder found "
                        "under --handoff-dir. Names must match --out-dir/"
                        "<person> from assign and the keys in "
                        "assignments_master.json.")
    m.add_argument("--handoff-dir", default="handoff",
                   help="The --out-dir used by `assign` (people sometimes call "
                        "this folder 'merge' once they start dropping returned "
                        "work back into it -- same folder, just rename it if "
                        "you like). Must contain assignments_master.json, and "
                        "each returned <person>/ folder with their filled-in "
                        "datasets/review_progress.json, "
                        "datasets/review_completed.json, and "
                        "datasets/pseudo_labels_reviewed/.")
    m.add_argument("--queue-file", default="pseudo_labels_review_queue.json",
                   help="Must match the --queue-file used at assign time -- "
                        "used here only to look up each image's source "
                        "dataset for the reviewed-label path.")
    m.add_argument("--no-backup", action="store_true",
                   help="Skip snapshotting the master review_progress.json/"
                        "review_completed.json before overwriting them. Not "
                        "recommended. (Also togglable in the GUI.)")

    return p.parse_args()


# ------------------------------------------------------------------
# Sequence grouping -- same heuristic review_labels.py's cross-frame
# sync uses, so batch boundaries always line up with sync boundaries.
# ------------------------------------------------------------------

def parse_seq_key(image_key: str) -> str:
    """Returns just the sequence id for grouping. Images that don't
    match either layout each become their own singleton "sequence" so
    they still get assigned somewhere -- they just carry no sync-
    neighbor grouping guarantee, same as in review_labels.py itself."""
    p = Path(image_key)
    m = SEQ_FRAME_RE.search(p.name)
    if m:
        return m.group("seq")
    if p.parent.name and FRAME_DIGITS_RE.findall(p.stem):
        return p.parent.name
    return f"__singleton__{image_key}"


def load_queue(datasets_dir: Path, queue_filename: str) -> list:
    path = datasets_dir / queue_filename
    if not path.exists():
        raise SystemExit(f"{path} not found -- run generate_pseudo_labels.py first.")
    with open(path) as f:
        return json.load(f)


def build_sequences(by_image: dict) -> dict:
    """{sequence_key: [image_key, ...]} -- images grouped by sequence,
    sequence_key prefixed with source so the same literal id showing up
    in two different sources never collides into one bucket."""
    sequences: dict = {}
    for image_key, items in by_image.items():
        source = items[0]["source"]
        seq = f"{source}::{parse_seq_key(image_key)}"
        sequences.setdefault(seq, []).append(image_key)
    return sequences


def portable_image_key(image_key: str) -> str:
    """Rewrites an image_key to be relative to DATASETS_DIR, so a
    person's queue file is portable to a machine where the project
    folder lives at a completely different absolute path -- only the
    person's own datasets/ subtree needs to line up, which it does
    since copy_image_and_label() below copies files to that exact
    same relative layout. Falls back to just the filename if the
    image somehow isn't under DATASETS_DIR (mirrors
    copy_image_and_label()'s own fallback), and leaves the key
    untouched if it's already relative (e.g. re-running this on an
    already-fixed queue file)."""
    p = Path(image_key)
    if not p.is_absolute():
        return image_key
    try:
        rel = p.resolve().relative_to(prepare_datasets.DATASETS_DIR.resolve())
    except ValueError:
        rel = Path(p.name)
    return str(rel)


def copy_image_and_label(image_key: str, out_datasets_dir: Path) -> tuple:
    """Copies the image file, and its real label file if one exists,
    into out_datasets_dir at the same relative-to-DATASETS_DIR path.
    Returns (image_copied, label_copied)."""
    src_img = Path(image_key)
    try:
        rel = src_img.resolve().relative_to(prepare_datasets.DATASETS_DIR.resolve())
    except ValueError:
        rel = Path(src_img.name)

    dst_img = out_datasets_dir / rel
    dst_img.parent.mkdir(parents=True, exist_ok=True)
    image_copied = False
    if src_img.exists():
        shutil.copy2(src_img, dst_img)
        image_copied = True

    label_copied = False
    try:
        src_label = gpl.image_path_to_label_path(src_img)
    except ValueError:
        src_label = None
    if src_label is not None and src_label.exists():
        try:
            rel_label = src_label.resolve().relative_to(prepare_datasets.DATASETS_DIR.resolve())
            dst_label = out_datasets_dir / rel_label
        except ValueError:
            dst_label = dst_img.with_suffix(".txt")
        dst_label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_label, dst_label)
        label_copied = True

    return image_copied, label_copied


def load_master_completed_images(datasets_dir: Path) -> set:
    """Reads datasets/review_completed.json directly -- a flat JSON list
    of image paths -- WITHOUT importing review_labels.py, so `assign`
    doesn't need review_labels.py's cv2/Pillow dependencies just to know
    what's already fully reviewed. Filename is hardcoded to match
    review_labels.COMPLETED_FILENAME (kept as a plain string here
    deliberately, to avoid the heavy import). Returns an empty set if
    the file doesn't exist yet (nothing reviewed so far)."""
    path = datasets_dir / "review_completed.json"
    if not path.exists():
        return set()
    with open(path) as f:
        return set(json.load(f))


# ------------------------------------------------------------------
# State (persistent ledger of every batch ever handed out)
# ------------------------------------------------------------------

def load_state(state_path: Path) -> dict:
    if state_path.exists():
        with open(state_path) as f:
            return json.load(f)
    return {"batches": [], "people": {}}  # people: {name: {"images": [...], "n_batches": int}}


def save_state(state_path: Path, state: dict) -> None:
    state_path.parent.mkdir(parents=True, exist_ok=True)
    with open(state_path, "w") as f:
        json.dump(state, f, indent=2, default=str)


def all_assigned_images(state: dict) -> set:
    out = set()
    for info in state["people"].values():
        out.update(info["images"])
    return out


def write_assignments_master(out_dir: Path, queue_file: str, state: dict) -> None:
    """Keeps assignments_master.json (the flat file merge_review_results.py
    reads) in sync with the state file's cumulative per-person images."""
    master = {"queue_file": queue_file,
              "people": {p: sorted(info["images"]) for p, info in state["people"].items()}}
    with open(out_dir / "assignments_master.json", "w") as f:
        json.dump(master, f, indent=2, default=str)


# ------------------------------------------------------------------
# Sequence bookkeeping
# ------------------------------------------------------------------

def sequence_stats(sequences: dict, by_image: dict) -> dict:
    """{seq_key: {"n_images":, "n_items":, "source":}}"""
    stats = {}
    for seq, imgs in sequences.items():
        source = seq.split("::", 1)[0]
        stats[seq] = {
            "n_images": len(imgs),
            "n_items": sum(len(by_image[i]) for i in imgs),
            "source": source,
        }
    return stats


def remaining_sequences(sequences: dict, assigned: set) -> dict:
    """Excludes any sequence with even one image already assigned --
    partition unit is whole sequences, so in normal operation a
    sequence is either fully in `assigned` or not in it at all; this
    guards against ever silently re-splitting one across two people if
    the assignment history was hand-edited."""
    return {seq: imgs for seq, imgs in sequences.items()
            if not any(img in assigned for img in imgs)}


def greedy_propose(seq_keys: list, stats: dict, target_images: int) -> list:
    """Largest-item-count-first, stopping once the running image total
    reaches target (allowed to overshoot on the sequence that crosses
    it -- sequences are never split)."""
    ordered = sorted(seq_keys, key=lambda s: stats[s]["n_items"], reverse=True)
    chosen, total_images = [], 0
    for seq in ordered:
        if total_images >= target_images:
            break
        chosen.append(seq)
        total_images += stats[seq]["n_images"]
    return chosen


# ------------------------------------------------------------------
# Package writing (create new person, or top-up an existing one)
# ------------------------------------------------------------------

def write_or_topup_package(person: str, new_images: list, by_image: dict,
                            out_dir: Path, queue_filename: str, scripts_dir: Path) -> tuple:
    """Creates person_dir if new, else merges new_images into their
    existing queue file/images/labels. Returns (n_img_copied,
    n_label_copied, n_img_missing)."""
    person_dir = out_dir / person
    person_datasets_dir = person_dir / "datasets"
    person_datasets_dir.mkdir(parents=True, exist_ok=True)
    queue_path = person_datasets_dir / queue_filename

    existing_items = []
    if queue_path.exists():
        with open(queue_path) as f:
            existing_items = json.load(f)
    existing_keys = {(it["image"], it["cls"], tuple(it.get("_orig_box", it["box"])))
                      for it in existing_items}

    new_items = []
    for img in new_images:
        for it in by_image[img]:
            key = (it["image"], it["cls"], tuple(it.get("_orig_box", it["box"])))
            if key not in existing_keys:
                # Copy the item and rewrite its image path to be
                # relative to DATASETS_DIR before it's ever written to
                # this person's queue file -- by_image[img] items are
                # shared with the assign-GUI's own in-memory state, so
                # mutating them in place would corrupt sequence/image
                # lookups for the rest of this session.
                portable_it = dict(it)
                portable_it["image"] = portable_image_key(it["image"])
                new_items.append(portable_it)

    with open(queue_path, "w") as f:
        json.dump(existing_items + new_items, f, indent=2, default=str)

    n_img_copied, n_label_copied, n_img_missing = 0, 0, 0
    for image_key in new_images:
        img_ok, label_ok = copy_image_and_label(image_key, person_datasets_dir)
        n_img_copied += int(img_ok)
        n_label_copied += int(label_ok)
        if not img_ok:
            n_img_missing += 1

    for fname in SCRIPT_FILES:
        src = scripts_dir / fname
        if src.exists():
            shutil.copy2(src, person_dir / fname)

    return n_img_copied, n_label_copied, n_img_missing


def write_manifest(person: str, out_dir: Path, all_images_for_person: list,
                    sequences_for_person: int) -> None:
    manifest = {
        "person": person,
        "n_images": len(all_images_for_person),
        "n_batches_cumulative": sequences_for_person,
        "images": sorted(all_images_for_person),
    }
    with open(out_dir / person / "assignment_manifest.json", "w") as f:
        json.dump(manifest, f, indent=2, default=str)


# ------------------------------------------------------------------
# GUI
# ------------------------------------------------------------------

class AssignTab(ttk.Frame):
    def __init__(self, parent, args):
        super().__init__(parent)
        self.root = parent  # top-level window, so _on_close() can destroy it
        self.args = args
        self.out_dir = Path(args.out_dir)
        self.scripts_dir = Path(args.scripts_dir)
        self.state_path = Path(args.state_file) if args.state_file \
            else self.out_dir / "assignment_state.json"

        self.datasets_dir = prepare_datasets.DATASETS_DIR
        self.items = load_queue(self.datasets_dir, args.queue_file)
        self.by_image = {}
        for it in self.items:
            self.by_image.setdefault(it["image"], []).append(it)

        # Only images NOT already fully reviewed are ever eligible to be
        # handed out -- completed images never enter sequence
        # construction, so they can never appear in the checklist or be
        # bundled into anyone's package, no matter what assignment_state
        # says (covers work done before this tool existed, or merged in
        # some other way).
        self.completed_images = load_master_completed_images(self.datasets_dir)
        self.pending_by_image = {img: its for img, its in self.by_image.items()
                                  if img not in self.completed_images}

        self.sequences = build_sequences(self.pending_by_image)
        self.stats = sequence_stats(self.sequences, self.pending_by_image)

        self.n_total_images = len(self.by_image)
        self.n_completed_images = self.n_total_images - len(self.pending_by_image)
        self.n_pending_images = len(self.pending_by_image)
        self.n_pending_items = sum(len(v) for v in self.pending_by_image.values())

        self.state = load_state(self.state_path)
        self.pending = []  # [{"person":, "sequences":[...], "images":[...]}]

        self.seq_vars = {}          # seq_key -> tk.BooleanVar, only currently remaining
        self.row_frames = {}        # seq_key -> ttk.Frame, for filter show/hide
        self.sorted_remaining_keys = []

        self._build_ui()
        self.refresh_remaining()

    def has_pending(self) -> bool:
        return bool(self.pending)

    # ---------------------------------------------------------------- UI

    def _build_ui(self):
        root = self

        self.status_var = tk.StringVar()
        ttk.Label(root, textvariable=self.status_var, font=("", 11, "bold")).pack(
            anchor="w", padx=10, pady=(8, 2))

        self.overview_var = tk.StringVar()
        ttk.Label(root, textvariable=self.overview_var, font=("", 10)).pack(
            anchor="w", padx=10, pady=(0, 6))

        main = ttk.Frame(root)
        main.pack(fill="both", expand=True, padx=10, pady=4)
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

        # --- left: checklist -----------------------------------------
        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.rowconfigure(2, weight=1)
        left.columnconfigure(0, weight=1)

        filt_row = ttk.Frame(left)
        filt_row.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(filt_row, text="Filter:").pack(side="left")
        self.filter_var = tk.StringVar()
        self.filter_var.trace_add("write", lambda *_: self.apply_filter())
        ttk.Entry(filt_row, textvariable=self.filter_var).pack(
            side="left", fill="x", expand=True, padx=6)
        ttk.Button(filt_row, text="Select all shown",
                   command=self.select_all_shown).pack(side="left", padx=2)
        ttk.Button(filt_row, text="Clear all",
                   command=self.clear_all).pack(side="left", padx=2)

        header_row = ttk.Frame(left)
        header_row.grid(row=1, column=0, sticky="ew", pady=(2, 1))
        ttk.Label(header_row, text="   Sequence (source::id)", font=("", 9, "bold"),
                  width=32, anchor="w").pack(side="left")
        ttk.Label(header_row, text="Images in\nsequence", font=("", 9, "bold"),
                  width=10, anchor="center", justify="center").pack(side="left")
        ttk.Label(header_row, text="Boxes still\nto review", font=("", 9, "bold"),
                  width=10, anchor="center", justify="center").pack(side="left")

        list_container = ttk.Frame(left, relief="sunken", borderwidth=1)
        list_container.grid(row=2, column=0, sticky="nsew")
        list_container.rowconfigure(0, weight=1)
        list_container.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(list_container, highlightthickness=0)
        vsb = ttk.Scrollbar(list_container, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=vsb.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")

        self.inner = ttk.Frame(self.canvas)
        self.inner_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.inner.bind("<Configure>",
                         lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>",
                          lambda e: self.canvas.itemconfig(self.inner_id, width=e.width))
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        self.summary_var = tk.StringVar()
        ttk.Label(left, textvariable=self.summary_var, font=("", 10, "italic")).grid(
            row=3, column=0, sticky="w", pady=(4, 0))

        # --- controls row ------------------------------------------------
        controls = ttk.LabelFrame(left, text="Assign selected to")
        controls.grid(row=4, column=0, sticky="ew", pady=(8, 0))

        row0 = ttk.Frame(controls)
        row0.pack(fill="x", padx=4, pady=(6, 2))
        ttk.Label(row0, text="Person:").pack(side="left")
        self.person_var = tk.StringVar()
        self.person_combo = ttk.Combobox(row0, textvariable=self.person_var, width=16)
        self.person_combo.pack(side="left", padx=(4, 12))
        self._refresh_people_list()

        ttk.Label(row0, text="Target images:").pack(side="left")
        self.target_var = tk.StringVar(value=str(self.args.target_chunk_images))
        ttk.Entry(row0, textvariable=self.target_var, width=6).pack(side="left", padx=4)
        ttk.Button(row0, text="Propose ~N", command=self.propose).pack(side="left", padx=6)

        row1 = ttk.Frame(controls)
        row1.pack(fill="x", padx=4, pady=(2, 6))
        add_btn = ttk.Button(row1, text="Add to pending batch", command=self.add_to_pending)
        add_btn.pack(side="left", fill="x", expand=True)

        # --- right: pending + finalize + log ------------------------------
        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.rowconfigure(3, weight=1)
        right.columnconfigure(0, weight=1)

        ttk.Label(right, text="Pending batches (staged, not yet written to disk)",
                  font=("", 10, "bold")).grid(row=0, column=0, sticky="w")

        self.pending_tree = ttk.Treeview(
            right, columns=("person", "seqs", "images", "items", "cumulative"),
            show="headings", height=8)
        headers = {"person": ("Person", 100), "seqs": ("Sequences", 80),
                   "images": ("Images in\nbatch", 80), "items": ("Boxes to\nreview", 80),
                   "cumulative": ("Cumulative\nimages", 90)}
        for col, (label, w) in headers.items():
            self.pending_tree.heading(col, text=label)
            self.pending_tree.column(col, width=w, anchor="center")
        self.pending_tree.grid(row=1, column=0, sticky="nsew", pady=(2, 4))

        pending_btns = ttk.Frame(right)
        pending_btns.grid(row=2, column=0, sticky="ew")
        ttk.Button(pending_btns, text="Undo last pending",
                   command=self.undo_last_pending).pack(side="left")
        ttk.Button(pending_btns, text="Finalize (write + save)",
                   command=self.finalize).pack(side="left", padx=8)

        ttk.Label(right, text="Log", font=("", 10, "bold")).grid(
            row=3, column=0, sticky="nw", pady=(10, 0))
        log_container = ttk.Frame(right)
        log_container.grid(row=4, column=0, sticky="nsew")
        right.rowconfigure(4, weight=1)
        log_container.rowconfigure(0, weight=1)
        log_container.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_container, height=12, wrap="word", state="disabled")
        log_vsb = ttk.Scrollbar(log_container, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_vsb.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_vsb.grid(row=0, column=1, sticky="ns")

        self.log(f"Loaded {len(self.items)} items / {self.n_total_images} images / "
                  f"{len(self.sequences)} sequences from {self.args.queue_file}.")
        self.log(f"{self.n_completed_images} image(s) already fully reviewed -- "
                  f"excluded from assignment entirely. {self.n_pending_images} image(s) "
                  f"({self.n_pending_items} items) still pending review.")
        self.log(f"State file: {self.state_path} "
                  f"({'existing' if self.state_path.exists() else 'new'}, "
                  f"{len(self.state['batches'])} batch(es), "
                  f"people so far: {', '.join(sorted(self.state['people'])) or '(none)'})")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _refresh_people_list(self):
        self.person_combo["values"] = sorted(self.state["people"].keys())

    # ------------------------------------------------------------ logging

    def log(self, msg: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ------------------------------------------------------ checklist logic

    def currently_committed_images(self):
        pending_images = {img for b in self.pending for img in b["images"]}
        return all_assigned_images(self.state) | pending_images

    def refresh_remaining(self):
        for f in self.row_frames.values():
            f.destroy()
        self.row_frames.clear()
        self.seq_vars.clear()

        remaining = remaining_sequences(self.sequences, self.currently_committed_images())
        self.sorted_remaining_keys = sorted(
            remaining, key=lambda s: self.stats[s]["n_items"], reverse=True)

        for seq in self.sorted_remaining_keys:
            var = tk.BooleanVar(value=False)
            s = self.stats[seq]
            row = ttk.Frame(self.inner)
            cb = ttk.Checkbutton(row, variable=var, command=self.update_summary)
            cb.pack(side="left")
            name_lbl = ttk.Label(row, text=seq, width=28, anchor="w")
            name_lbl.pack(side="left")
            img_lbl = ttk.Label(row, text=str(s["n_images"]), width=10, anchor="center")
            img_lbl.pack(side="left")
            items_lbl = ttk.Label(row, text=str(s["n_items"]), width=10, anchor="center")
            items_lbl.pack(side="left")

            def toggle(v=var):
                v.set(not v.get())
                self.update_summary()
            for w in (name_lbl, img_lbl, items_lbl):
                w.bind("<Button-1>", lambda e, t=toggle: t())

            self.seq_vars[seq] = var
            self.row_frames[seq] = row

        self.apply_filter()
        self.update_summary()
        n_remaining_images = sum(self.stats[s]["n_images"] for s in remaining)
        n_already_out = self.n_pending_images - n_remaining_images  # pending images already
                                                                     # handed out in some batch
        self.status_var.set(f"{len(remaining)} sequences / {n_remaining_images} images "
                              f"still unassigned")
        self.overview_var.set(
            f"Set: {self.n_total_images} images total  |  "
            f"{self.n_completed_images} already reviewed (never offered)  |  "
            f"{self.n_pending_images} pending review  |  "
            f"{n_already_out} pending images already handed out in a batch  |  "
            f"{n_remaining_images} pending images still available to assign")

    def apply_filter(self):
        text = self.filter_var.get().strip().lower()
        for f in self.row_frames.values():
            f.pack_forget()
        for seq in self.sorted_remaining_keys:
            if text in seq.lower():
                self.row_frames[seq].pack(fill="x", anchor="w", pady=1, padx=2)

    def checked_sequences(self, visible_only=False):
        if visible_only:
            text = self.filter_var.get().strip().lower()
            keys = [s for s in self.sorted_remaining_keys if text in s.lower()]
        else:
            keys = self.sorted_remaining_keys
        return [s for s in keys if self.seq_vars[s].get()]

    def update_summary(self):
        checked = self.checked_sequences()
        n_img = sum(self.stats[s]["n_images"] for s in checked)
        n_items = sum(self.stats[s]["n_items"] for s in checked)
        self.summary_var.set(f"Selected: {len(checked)} sequences, "
                              f"{n_img} images, {n_items} items")

    def select_all_shown(self):
        text = self.filter_var.get().strip().lower()
        for seq in self.sorted_remaining_keys:
            if text in seq.lower():
                self.seq_vars[seq].set(True)
        self.update_summary()

    def clear_all(self):
        for var in self.seq_vars.values():
            var.set(False)
        self.update_summary()

    def propose(self):
        try:
            target = int(self.target_var.get())
        except ValueError:
            messagebox.showerror("Invalid target", "Target images must be a whole number.")
            return
        self.clear_all()
        proposal = set(greedy_propose(self.sorted_remaining_keys, self.stats, target))
        for seq in proposal:
            self.seq_vars[seq].set(True)
        self.update_summary()
        n_img = sum(self.stats[s]["n_images"] for s in proposal)
        self.log(f"Proposed {len(proposal)} sequence(s), {n_img} images (target {target}).")

    # ------------------------------------------------------------- pending

    def add_to_pending(self):
        person = self.person_var.get().strip()
        if not person:
            messagebox.showwarning("No person", "Type or pick a person name first.")
            return
        checked = self.checked_sequences()
        if not checked:
            messagebox.showwarning("Nothing selected", "Check at least one sequence first.")
            return

        images = sorted(img for seq in checked for img in self.sequences[seq])
        n_items = sum(self.stats[s]["n_items"] for s in checked)
        self.pending.append({"person": person, "sequences": sorted(checked), "images": images})
        self.log(f"Pending: {person} <- {len(checked)} sequence(s), {len(images)} images, "
                  f"{n_items} items to review.")

        # Cumulative = images this person already has on disk (from state)
        # plus anything already staged for them this session plus this batch.
        already_theirs = set(self.state["people"].get(person, {}).get("images", []))
        for b in self.pending[:-1]:
            if b["person"] == person:
                already_theirs.update(b["images"])
        cumulative = len(already_theirs | set(images))

        self.pending_tree.insert(
            "", "end", values=(person, len(checked), len(images), n_items, cumulative))
        self._refresh_people_list()
        self.clear_all()
        self.refresh_remaining()

    def undo_last_pending(self):
        if not self.pending:
            messagebox.showinfo("Nothing pending", "No pending batches to undo.")
            return
        removed = self.pending.pop()
        children = self.pending_tree.get_children()
        if children:
            self.pending_tree.delete(children[-1])
        self.log(f"Undid pending batch: {removed['person']} "
                  f"({len(removed['images'])} images).")
        self.refresh_remaining()

    # ------------------------------------------------------------- finalize

    def finalize(self):
        if not self.pending:
            messagebox.showinfo("Nothing pending", "No pending batches to finalize.")
            return
        if not messagebox.askyesno(
                "Confirm finalize",
                f"Write/update {len(self.pending)} batch(es) to disk under "
                f"{self.out_dir} and save state? This actually copies files."):
            return

        ts = datetime.now().isoformat(timespec="seconds")
        for batch in self.pending:
            person = batch["person"]
            n_copied, n_labels, n_missing = write_or_topup_package(
                person, batch["images"], self.by_image, self.out_dir,
                self.args.queue_file, self.scripts_dir)

            info = self.state["people"].setdefault(person, {"images": [], "n_batches": 0})
            is_new_person = info["n_batches"] == 0
            info["images"] = sorted(set(info["images"]) | set(batch["images"]))
            info["n_batches"] += 1
            write_manifest(person, self.out_dir, info["images"],
                            sequences_for_person=info["n_batches"])

            self.state["batches"].append({
                "person": person,
                "batch_number": info["n_batches"],
                "sequences": batch["sequences"],
                "n_images": len(batch["images"]),
                "assigned_at": ts,
            })

            kind = "NEW package" if is_new_person else f"TOP-UP (batch #{info['n_batches']})"
            self.log(f"{person}: {kind}, +{len(batch['images'])} images "
                      f"({n_copied} images / {n_labels} reference labels copied"
                      + (f", {n_missing} MISSING on disk!" if n_missing else "")
                      + f") -> cumulative {len(info['images'])} images "
                      + f"-> {self.out_dir / person}/")
            if not is_new_person:
                self.log("  New images this batch (send just these if not resending the "
                          "whole folder):")
                for img in sorted(batch["images"])[:10]:
                    self.log(f"    {img}")
                if len(batch["images"]) > 10:
                    self.log(f"    ... and {len(batch['images']) - 10} more")

        write_assignments_master(self.out_dir, self.args.queue_file, self.state)
        save_state(self.state_path, self.state)
        self.log(f"State saved -> {self.state_path}")
        self.log(f"assignments_master.json updated -> {self.out_dir / 'assignments_master.json'}")

        self.pending.clear()
        for child in self.pending_tree.get_children():
            self.pending_tree.delete(child)
        self._refresh_people_list()
        self.refresh_remaining()

    def _on_close(self):
        if self.pending:
            if not messagebox.askyesno(
                    "Discard pending?",
                    f"You have {len(self.pending)} unfinalized pending batch(es). "
                    f"Close without finalizing?"):
                return
        self.root.destroy()


def run_assign(args):
    # NOTE: this used to instantiate a `BatchAssignerApp` class that was
    # never actually defined anywhere in this file -- calling
    # run_assign() raised NameError immediately and the window never
    # rendered (which is why no buttons/labels were visible). AssignTab
    # is a self-contained ttk.Frame, so it just needs to be built and
    # packed directly into the root window.
    root = tk.Tk()
    root.title("Review Batch Assigner -- Assign")
    root.geometry("1200x760")
    tab = AssignTab(root, args)
    tab.pack(fill="both", expand=True)
    root.mainloop()


# ------------------------------------------------------------------
# merge subcommand -- recombines returned per-person folders back into
# the master datasets/ tree. Only import review_labels here (rather
# than at module load) since it pulls in cv2/Pillow/tkinter deps that
# `assign` alone doesn't strictly need beyond tkinter itself -- but in
# practice both subcommands run in the same environment, so this is
# just tidiness, not a hard requirement.
# ------------------------------------------------------------------

def image_of_box_key(key: str) -> str:
    """Reverses item_key()'s 'image|cls|box' shape. Splitting on the
    FIRST '|' is unambiguous since image paths never contain '|'."""
    return key.split("|", 1)[0]


def load_assignments(handoff_dir: Path) -> dict:
    path = handoff_dir / "assignments_master.json"
    if not path.exists():
        raise SystemExit(f"{path} not found -- run `review_batch_assigner.py assign` first.")
    with open(path) as f:
        data = json.load(f)
    return {person: set(images) for person, images in data["people"].items()}


def load_source_lookup(datasets_dir: Path, queue_filename: str) -> dict:
    """{image_key: source} from the given queue file -- used only to
    resolve the right pseudo_labels_reviewed/<SOURCE>/ subtree for each
    image, same lookup `assign` builds for the same reason."""
    path = datasets_dir / queue_filename
    if not path.exists():
        raise SystemExit(f"{path} not found -- run generate_pseudo_labels.py first.")
    with open(path) as f:
        items = json.load(f)
    lookup: dict = {}
    for it in items:
        lookup.setdefault(it["image"], it["source"])
    return lookup


def merge_progress(people: list, handoff_dir: Path, assignments: dict, force: bool, rl):
    """Returns (merged_progress, owner_of_key, unknown_entries, conflicts)."""
    merged: dict = {}
    owner_of_key: dict = {}
    unknown = []     # (person, key, recovered_image)
    conflicts = []   # (key, first_owner, later_person)

    for person in people:
        progress_path = handoff_dir / person / "datasets" / rl.PROGRESS_FILENAME
        person_progress = rl.load_json_dict(progress_path)
        assigned = assignments.get(person, set())

        for key, value in person_progress.items():
            if key.startswith("__manual__"):
                image = key[len("__manual__"):]
            elif key.startswith("__override__"):
                image = key[len("__override__"):]
            else:
                image = image_of_box_key(key)

            if image not in assigned:
                unknown.append((person, key, image))
                continue

            if key in merged:
                conflicts.append((key, owner_of_key[key], person))
                if not force:
                    continue
            merged[key] = value
            owner_of_key[key] = person

    return merged, owner_of_key, unknown, conflicts


def merge_completed(people: list, handoff_dir: Path, assignments: dict, rl):
    """Returns (merged_completed_set, per_person_completed, unknown_entries)."""
    merged: set = set()
    per_person: dict = {}
    unknown = []  # (person, image)

    for person in people:
        completed_path = handoff_dir / person / "datasets" / rl.COMPLETED_FILENAME
        person_completed = rl.load_completed(completed_path)
        assigned = assignments.get(person, set())
        per_person[person] = person_completed

        for image in person_completed:
            if image not in assigned:
                unknown.append((person, image))
                continue
            merged.add(image)

    return merged, per_person, unknown


def collect_progress_candidates(people: list, handoff_dir: Path, assignments: dict, rl) -> tuple:
    """Like merge_progress(), but instead of picking a winner itself it
    returns EVERY (person, value) that claimed each key, so the caller
    (the merge GUI) can show a real conflict-resolution choice to the
    user instead of silently picking first-seen or last-seen.
    Returns (candidates, unknown) where candidates is
    {key: [(person, value), ...]}."""
    candidates: dict = {}
    unknown = []  # (person, key, recovered_image)

    for person in people:
        progress_path = handoff_dir / person / "datasets" / rl.PROGRESS_FILENAME
        person_progress = rl.load_json_dict(progress_path)
        assigned = assignments.get(person, set())

        for key, value in person_progress.items():
            if key.startswith("__manual__"):
                image = key[len("__manual__"):]
            elif key.startswith("__override__"):
                image = key[len("__override__"):]
            else:
                image = image_of_box_key(key)

            if image not in assigned:
                unknown.append((person, key, image))
                continue

            candidates.setdefault(key, []).append((person, value))

    return candidates, unknown


def find_multiply_assigned_images(people: list, assignments: dict) -> dict:
    """{image: [person, ...]} for any image that assignments_master.json
    lists under more than one of the selected people. Assignment is
    disjoint by construction, so this should normally come back empty --
    a non-empty result means the ledger was hand-edited, a folder got
    reused, or something similar, and the GUI should ask the user which
    person's copy of that image (completed status + reviewed label file)
    to trust rather than silently letting whoever is processed last win."""
    owners: dict = {}
    for person in people:
        for image in assignments.get(person, set()):
            owners.setdefault(image, []).append(person)
    return {image: people_list for image, people_list in owners.items() if len(people_list) > 1}


def scan_returned_people(handoff_dir: Path, assignments: dict) -> list:
    """Looks in handoff_dir for per-person folders with data returned
    from review_labels.py, cross-referenced against who assign actually
    handed work out to. Used to populate the merge GUI's checklist so
    the user doesn't have to type --people by hand."""
    found = []
    if not handoff_dir.exists():
        return found
    for child in sorted(handoff_dir.iterdir()):
        if not child.is_dir():
            continue
        person = child.name
        ds = child / "datasets"
        progress_path = ds / "review_progress.json"
        completed_path = ds / "review_completed.json"
        has_progress = progress_path.exists()
        has_completed = completed_path.exists()
        n_completed = 0
        if has_completed:
            try:
                with open(completed_path) as f:
                    n_completed = len(json.load(f))
            except (json.JSONDecodeError, OSError):
                n_completed = -1
        found.append({
            "person": person,
            "has_progress": has_progress,
            "has_completed": has_completed,
            "n_completed": n_completed,
            "n_assigned": len(assignments.get(person, set())),
            "in_assignments": person in assignments,
        })
    return found


def merge_reviewed_labels(people: list, handoff_dir: Path, datasets_dir: Path,
                           assignments: dict, per_person_completed: dict,
                           queue_filename: str, rl, gpl_module):
    """Copies each assigned image's reviewed-label file from the owning
    person's folder into the master pseudo_labels_reviewed/ tree.
    Returns (n_copied, n_removed_stale, n_missing_not_done)."""
    master_reviewed_root = datasets_dir / rl.REVIEWED_DIRNAME
    n_copied, n_removed, n_missing_not_done = 0, 0, 0

    for person in people:
        person_datasets_dir = handoff_dir / person / "datasets"
        person_reviewed_root = person_datasets_dir / rl.REVIEWED_DIRNAME
        assigned = assignments.get(person, set())
        completed_here = per_person_completed.get(person, set())
        source_lookup = load_source_lookup(person_datasets_dir, queue_filename) \
            if (person_datasets_dir / queue_filename).exists() else {}

        for image in sorted(assigned):
            source = source_lookup.get(image)
            if source is None:
                # Person's own filtered queue file didn't have this image
                # (shouldn't happen -- their queue is exactly their
                # assignment) -- skip rather than guess the source dir.
                continue

            real_label_path = gpl_module.image_path_to_label_path(Path(image))
            person_label_path = gpl_module.mirror_under_pseudo_root(
                real_label_path, source, person_reviewed_root)
            master_label_path = gpl_module.mirror_under_pseudo_root(
                real_label_path, source, master_reviewed_root)

            if person_label_path.exists():
                master_label_path.parent.mkdir(parents=True, exist_ok=True)
                master_label_path.write_bytes(person_label_path.read_bytes())
                n_copied += 1
            elif image in completed_here:
                # They finished this image and it has zero accepted boxes
                # -- authoritative; clear any stale master label for it.
                if master_label_path.exists():
                    master_label_path.unlink()
                    n_removed += 1
            else:
                n_missing_not_done += 1

    return n_copied, n_removed, n_missing_not_done


class ConflictResolutionDialog(tk.Toplevel):
    """Modal dialog listing every detected conflict (a review_progress.json
    key claimed by more than one person, or an image assigned to more
    than one person) and letting the user pick, per conflict, whose
    version to keep. Nothing is written to disk from here -- this only
    collects the user's choices; the caller applies them."""

    def __init__(self, parent, progress_conflicts: dict, image_conflicts: dict):
        super().__init__(parent)
        self.title("Resolve merge conflicts")
        self.geometry("820x520")
        self.transient(parent)
        self.grab_set()

        self.progress_conflicts = progress_conflicts  # {key: [(person, value), ...]}
        self.image_conflicts = image_conflicts         # {image: [person, ...]}
        self.choice_vars = {}   # ("progress", key) or ("image", image) -> tk.StringVar
        self.result = None      # set to {"progress": {...}, "image": {...}} on OK

        ttk.Label(
            self, wraplength=780, justify="left",
            text="These entries were claimed by more than one person's returned "
                 "folder. Assignment is supposed to be disjoint, so this normally "
                 "means a folder was reused or the assignment ledger was hand-"
                 "edited. Pick which person's labeling/image to keep for each "
                 "one below. Default is the OLD (first-seen) value; the NEW "
                 "value is whoever showed up later.",
            font=("", 9, "italic")
        ).pack(anchor="w", padx=10, pady=(10, 6))

        container = ttk.Frame(self)
        container.pack(fill="both", expand=True, padx=10)
        canvas = tk.Canvas(container, highlightthickness=0)
        vsb = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        inner = ttk.Frame(canvas)
        inner_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(inner_id, width=e.width))

        if self.image_conflicts:
            ttk.Label(inner, text="Images assigned to more than one person",
                      font=("", 10, "bold")).pack(anchor="w", pady=(6, 2))
            for image, owners in sorted(self.image_conflicts.items()):
                row = ttk.Frame(inner)
                row.pack(fill="x", pady=2)
                ttk.Label(row, text=image, width=52, anchor="w").pack(side="left")
                var = tk.StringVar(value=owners[0])
                labels = [f"OLD ({owners[0]})"] + [f"NEW ({p})" for p in owners[1:]]
                value_map = dict(zip(labels, owners))
                combo = ttk.Combobox(row, values=labels, state="readonly", width=22)
                combo.set(labels[0])
                combo.pack(side="left", padx=6)
                combo.bind("<<ComboboxSelected>>",
                           lambda e, v=var, vm=value_map, c=combo: v.set(vm[c.get()]))
                self.choice_vars[("image", image)] = var

        if self.progress_conflicts:
            ttk.Label(inner, text="Review decisions claimed by more than one person",
                      font=("", 10, "bold")).pack(anchor="w", pady=(14, 2))
            for key, options in sorted(self.progress_conflicts.items()):
                row = ttk.Frame(inner)
                row.pack(fill="x", pady=2)
                short_key = key if len(key) <= 60 else key[:57] + "..."
                ttk.Label(row, text=short_key, width=52, anchor="w").pack(side="left")
                people = [p for p, _ in options]
                labels = [f"OLD ({people[0]})"] + [f"NEW ({p})" for p in people[1:]]
                value_map = dict(zip(labels, people))
                var = tk.StringVar(value=people[0])
                combo = ttk.Combobox(row, values=labels, state="readonly", width=22)
                combo.set(labels[0])
                combo.pack(side="left", padx=6)
                combo.bind("<<ComboboxSelected>>",
                           lambda e, v=var, vm=value_map, c=combo: v.set(vm[c.get()]))
                self.choice_vars[("progress", key)] = var

        btns = ttk.Frame(self)
        btns.pack(fill="x", padx=10, pady=10)
        ttk.Button(btns, text="Cancel merge", command=self._cancel).pack(side="right")
        ttk.Button(btns, text="Apply choices and continue",
                   command=self._ok).pack(side="right", padx=8)

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.wait_window(self)

    def _ok(self):
        progress_choices = {key: var.get() for (kind, key), var in self.choice_vars.items()
                             if kind == "progress"}
        image_choices = {image: var.get() for (kind, image), var in self.choice_vars.items()
                          if kind == "image"}
        self.result = {"progress": progress_choices, "image": image_choices}
        self.destroy()

    def _cancel(self):
        self.result = None
        self.destroy()


class MergeTab(ttk.Frame):
    def __init__(self, parent, args):
        super().__init__(parent)
        self.root = parent
        self.args = args
        self.handoff_dir = Path(args.handoff_dir)

        self.person_vars = {}  # person -> tk.BooleanVar
        self._build_ui()
        self.rescan()

    def _build_ui(self):
        ttk.Label(self, text=f"Handoff / merge folder: {self.handoff_dir}",
                  font=("", 11, "bold")).pack(anchor="w", padx=10, pady=(8, 2))
        ttk.Label(self, text="Drop each person's returned folder (with their filled-in "
                              "datasets/review_progress.json, review_completed.json, and "
                              "pseudo_labels_reviewed/) into this directory, then Rescan.",
                  font=("", 9, "italic")).pack(anchor="w", padx=10, pady=(0, 8))

        top_btns = ttk.Frame(self)
        top_btns.pack(fill="x", padx=10)
        ttk.Button(top_btns, text="Rescan folder", command=self.rescan).pack(side="left")
        ttk.Button(top_btns, text="Select all", command=self.select_all).pack(side="left", padx=6)
        ttk.Button(top_btns, text="Clear all", command=self.clear_all).pack(side="left")

        self.backup_var = tk.BooleanVar(value=not self.args.no_backup)
        ttk.Checkbutton(top_btns, text="Back up master progress/completed files first",
                         variable=self.backup_var).pack(side="left", padx=20)

        header = ttk.Frame(self)
        header.pack(fill="x", padx=10, pady=(10, 1))
        ttk.Label(header, text="   Person", font=("", 9, "bold"), width=22,
                  anchor="w").pack(side="left")
        ttk.Label(header, text="Images\nassigned", font=("", 9, "bold"), width=10,
                  anchor="center", justify="center").pack(side="left")
        ttk.Label(header, text="Images marked\ncomplete (returned)", font=("", 9, "bold"),
                  width=18, anchor="center", justify="center").pack(side="left")
        ttk.Label(header, text="Images left to\nreview (returned)", font=("", 9, "bold"),
                  width=18, anchor="center", justify="center").pack(side="left")
        ttk.Label(header, text="Status", font=("", 9, "bold"), width=16,
                  anchor="w").pack(side="left")

        list_container = ttk.Frame(self, relief="sunken", borderwidth=1)
        list_container.pack(fill="both", expand=True, padx=10, pady=(0, 6))
        self.list_frame = ttk.Frame(list_container)
        self.list_frame.pack(fill="both", expand=True)

        ttk.Button(self, text="Update (merge selected into master datasets/)",
                   command=self.do_merge).pack(anchor="w", padx=10, pady=(0, 6))

        ttk.Label(self, text="Log", font=("", 10, "bold")).pack(anchor="w", padx=10)
        log_container = ttk.Frame(self)
        log_container.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log_text = tk.Text(log_container, height=12, wrap="word", state="disabled")
        log_vsb = ttk.Scrollbar(log_container, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_vsb.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        log_vsb.pack(side="right", fill="y")

    def log(self, msg: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ------------------------------------------------------------ scan

    def rescan(self):
        for w in self.list_frame.winfo_children():
            w.destroy()
        self.person_vars.clear()

        if not (self.handoff_dir / "assignments_master.json").exists():
            self.log(f"[error] {self.handoff_dir / 'assignments_master.json'} not found -- "
                      f"run the `assign` step first (or check --handoff-dir).")
            self.assignments = {}
            return

        self.assignments = load_assignments(self.handoff_dir)
        rows = scan_returned_people(self.handoff_dir, self.assignments)
        preselect = set(self.args.people) if self.args.people else None

        if not rows:
            ttk.Label(self.list_frame,
                      text=f"No person folders found under {self.handoff_dir}.").pack(
                anchor="w", padx=4, pady=4)
            return

        for row in rows:
            frame = ttk.Frame(self.list_frame)
            frame.pack(fill="x", padx=2, pady=1)
            var = tk.BooleanVar(value=(row["person"] in preselect) if preselect
                                 else row["has_progress"] or row["has_completed"])
            cb = ttk.Checkbutton(frame, variable=var)
            cb.pack(side="left")
            ttk.Label(frame, text=row["person"], width=20, anchor="w").pack(side="left")
            ttk.Label(frame, text=str(row["n_assigned"]), width=10,
                      anchor="center").pack(side="left")
            n_left = "?" if row["n_completed"] < 0 else max(row["n_assigned"] - row["n_completed"], 0)
            n_done = "?" if row["n_completed"] < 0 else row["n_completed"]
            ttk.Label(frame, text=str(n_done), width=18, anchor="center").pack(side="left")
            ttk.Label(frame, text=str(n_left), width=18, anchor="center").pack(side="left")

            if not row["in_assignments"]:
                status = "not in assignments_master.json"
            elif not row["has_progress"] and not row["has_completed"]:
                status = "no returned files found"
            else:
                status = "ready to merge"
            ttk.Label(frame, text=status, width=24, anchor="w").pack(side="left")

            self.person_vars[row["person"]] = var

        self.log(f"Found {len(rows)} folder(s) under {self.handoff_dir}.")

    def select_all(self):
        for var in self.person_vars.values():
            var.set(True)

    def clear_all(self):
        for var in self.person_vars.values():
            var.set(False)

    # ----------------------------------------------------------- merge

    def do_merge(self):
        import review_labels as rl  # deferred: only needed for this subcommand

        people = [p for p, var in self.person_vars.items() if var.get()]
        if not people:
            messagebox.showwarning("Nothing selected", "Check at least one person first.")
            return
        if not messagebox.askyesno(
                "Confirm merge",
                f"Merge returned work from {len(people)} person(s) into the master "
                f"datasets/? This writes files."):
            return

        datasets_dir = prepare_datasets.DATASETS_DIR
        progress_path = datasets_dir / rl.PROGRESS_FILENAME
        completed_path = datasets_dir / rl.COMPLETED_FILENAME

        # --- detect conflicts first, before writing anything ------------
        image_conflicts = find_multiply_assigned_images(people, self.assignments)
        candidates, unknown_progress = collect_progress_candidates(
            people, self.handoff_dir, self.assignments, rl)
        progress_conflicts = {k: v for k, v in candidates.items() if len(v) > 1}
        no_conflict_progress = {k: v[0][1] for k, v in candidates.items() if len(v) == 1}

        resolved_progress = dict(no_conflict_progress)
        adjusted_assignments = {p: set(self.assignments.get(p, set())) for p in people}

        if progress_conflicts or image_conflicts:
            dialog = ConflictResolutionDialog(self.root, progress_conflicts, image_conflicts)
            if dialog.result is None:
                self.log("[cancelled] Merge cancelled by user at conflict resolution.")
                return
            for key, options in progress_conflicts.items():
                chosen_person = dialog.result["progress"][key]
                resolved_progress[key] = dict(options)[chosen_person]
            for image, owners in image_conflicts.items():
                winner = dialog.result["image"][image]
                for person in owners:
                    if person != winner:
                        adjusted_assignments[person].discard(image)
            self.log(f"Resolved {len(progress_conflicts)} progress conflict(s) and "
                      f"{len(image_conflicts)} image-ownership conflict(s).")

        # --- back up master files -----------------------------------
        if self.backup_var.get():
            backup_dir = rl.backup_progress_files(datasets_dir, progress_path, completed_path)
            if backup_dir:
                self.log(f"[backup] snapshotted master progress/completed files -> {backup_dir}")

        master_progress = rl.load_json_dict(progress_path)
        master_completed = rl.load_completed(completed_path)

        combined_progress = dict(master_progress)
        combined_progress.update(resolved_progress)

        merged_completed, per_person_completed, unknown_completed = merge_completed(
            people, self.handoff_dir, adjusted_assignments, rl)
        combined_completed = master_completed | merged_completed

        n_copied, n_removed, n_missing_not_done = merge_reviewed_labels(
            people, self.handoff_dir, datasets_dir, adjusted_assignments,
            per_person_completed, self.args.queue_file, rl, gpl)

        rl.save_json(progress_path, combined_progress)
        rl.save_completed(completed_path, combined_completed)

        self.log(f"\nMerged {len(people)} folder(s) into {datasets_dir}:")
        self.log(f"  review_progress.json:  {len(resolved_progress)} entries merged in "
                  f"({len(combined_progress)} total on disk now)")
        self.log(f"  review_completed.json: {len(merged_completed)} image(s) merged in "
                  f"({len(combined_completed)} total on disk now)")
        self.log(f"  pseudo_labels_reviewed/: {n_copied} label file(s) copied, "
                  f"{n_removed} stale file(s) cleared (finished with zero accepted "
                  f"boxes), {n_missing_not_done} image(s) not yet finished (left as-is)")

        if unknown_progress:
            self.log(f"[warn] {len(unknown_progress)} review_progress.json entr(y/ies) "
                      f"referenced an image NOT in that person's assignment -- skipped:")
            for person, key, image in unknown_progress[:10]:
                self.log(f"    {person}: key for '{image}' ({key})")
            if len(unknown_progress) > 10:
                self.log(f"    ... and {len(unknown_progress) - 10} more")

        if unknown_completed:
            self.log(f"[warn] {len(unknown_completed)} review_completed.json entr(y/ies) "
                      f"referenced an image NOT in that person's assignment -- skipped:")
            for person, image in unknown_completed[:10]:
                self.log(f"    {person}: {image}")
            if len(unknown_completed) > 10:
                self.log(f"    ... and {len(unknown_completed) - 10} more")

        self.log("\nNext: run review_labels.py normally against this master datasets/ "
                  "dir once -- its own startup reconciliation will self-heal "
                  "review_completed.json from the merged progress + live queue.")
        messagebox.showinfo("Merge complete", "Merge finished -- see the log for details.")


def run_merge(args):
    root = tk.Tk()
    root.title("Review Batch Assigner -- Merge")
    root.geometry("1000x700")
    tab = MergeTab(root, args)
    tab.pack(fill="both", expand=True)
    root.mainloop()


def main():
    args = parse_args()
    if args.command == "assign":
        run_assign(args)
    elif args.command == "merge":
        run_merge(args)
    else:
        raise SystemExit(f"Unknown command: {args.command}")


if __name__ == "__main__":
    main()