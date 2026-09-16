# `review_labels.py` — interactive review GUI

**File:** `review_labels.py`
**Run:** `python review_labels.py [options]`
**Class:** `ReviewApp` (Tkinter + Pillow)
**Reads:** `datasets/pseudo_labels_review_queue.json` (written by
`generate_pseudo_labels.py`)
**Writes:** `datasets/review_progress.json` (autosaved decisions),
`datasets/review_completed.json`,
`datasets/pseudo_labels_reviewed/<SOURCE>/...`

This is the largest single file in the ML pipeline (and in the project as
a whole) — a full interactive review tool built up over 18 documented
versions. This page covers what it does and why it's shaped the way it
is; it doesn't reproduce the entire in-app UI reference (accessible from
the tool's own Help menu).

## Responsibility

Shows one image at a time from the needs-review queue
`generate_pseudo_labels.py` produced, with its queued boxes overlaid.
Click a box to accept/reject/reclassify it, or drag on empty canvas to
draw a new box the queue missed entirely. Progress autosaves continuously
— close it anytime, rerun later, it picks up exactly where you left off.

## Why a separate output tree

Writes to `datasets/pseudo_labels_reviewed/<SOURCE>/...`, **not**
`datasets/pseudo_labels/` (the auto-accept tier). `generate_pseudo_labels.py`
wipes and overwrites `pseudo_labels/` wholesale on every rerun — if this
tool wrote there too, a later rerun would silently erase manual review
work. `generate_pseudo_labels.py --apply` merges **both** trees into the
real dataset, so this separation costs nothing at apply time (see
[generate-pseudo-labels.md](generate-pseudo-labels.md#--apply-merge--hold-back)).

## Scope

Only images that have at least one queue entry are shown — the much
larger auto-accepted tier is never touched here. `--source`/`--cls` focus
a session on one dataset/class; the tool is explicitly built to be
resumable across many sessions, not a one-sitting task.

## Feature summary

| Feature | Behavior |
|---|---|
| **Queue (model) boxes** | Click opens a menu: Accept / Reject / Reset to Pending / Change Class / "Delete (hide from view, recoverable)". Click-only by default; "Enable Reposition" arms exactly one drag (move or resize), then re-locks. Deleted boxes are hidden unless "Show deleted" is checked, and can be reset back to pending from there |
| **Manual boxes** (cyan) | Drag on empty canvas to draw one; picking its class auto-accepts it. Freely draggable/resizable (8 handles: 4 corners + 4 edge midpoints, minimum size `MIN_BOX_PX`). Ctrl+C/Ctrl+V copies the selected box, offset by `PASTE_OFFSET_PX`; repeated Ctrl+V without a new Ctrl+C cascades further each time |
| **Original dataset labels** (magenta) | Reference only, shown by default (`--hide-original` to hide), never selectable/draggable |
| **Per-model panel** | Visibility checkboxes + a detection-count readout per model for the current image (models with zero detections shown in red), independent of what's currently displayed |
| **Autosave** | Every mutation writes to disk (debounced ~400ms; an immediate un-debounced flush happens on navigate, close, or window focus-loss). Manual "Save Now" also available |
| **Undo/redo** | Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z, scoped per image (stack resets on navigation) |
| **Fast Mode** | Left-click a queue box = Accept, right-click = Reject, no menu (toolbar/View-menu toggle) |
| **Bulk actions** | "Accept all pending"/"Reject all pending" for the current image (confirms first; one undo step reverts the whole batch) |
| **Number keys 1–9** | Apply the Nth class (`CLASS_NAMES` order) to the selected box |
| **Zoom/pan** | Mouse wheel or +/−/0 (0 = fit to window); middle-click-drag or scrollbars to pan |
| **Completed list** | Once every box on an image has a decision, it moves to `datasets/review_completed.json` and is excluded from normal review. `--qa` opens a session sourced only from that list (a second pass/reviewer); "Switch to QA / Completed" swaps live, without restarting. Resetting any box back to pending drops the image out of the completed list again |
| **Sidebar** | Image list split Remaining/Reviewed, grouped by source dataset then by frame set/sequence (color-coded), each set annotated with a reviewed/total count and a warning marker below `MIN_REVIEWED_FRAMES_PER_SEQUENCE`. Click to jump. "Check sequence coverage" gives the same numbers as a text report. Purely navigational — never affects decisions or output |

## Cross-frame sync

Deciding a queue or manual box (accept/reject/delete) looks at nearby
frames in the **same sequence** — parsed from the filename via
`parse_seq_frame()` (e.g. `"M1201_img000322_..."` → sequence `M1201`,
frame `322`) — and applies the same decision to a same-class box at a
matched position, within `--sync-window`. Rules:

- Same class only, one nearest match per neighbor frame.
- Never overwrites an existing decision, never reopens a completed image.
- Only the **decision** is synced, never geometry.
- Uses a constant-velocity (dead-reckoning) position predictor walking
  outward frame-by-frame, so matches track a moving/panning object rather
  than assuming a fixed pixel spot.
- Synced boxes are tagged `[synced]` on canvas.
- Runs **on save**, not on every click, so a burst of edits collapses into
  one sync pass.
- Changing your mind on a source box's decision retracts exactly what it
  had synced (tracked per-box) before applying the new decision forward.
  "Undo Last Sync" reverts the most recent sync batch.

## Startup self-healing

Every launch backs up `review_progress.json` and `review_completed.json`
to `datasets/review_backups/<timestamp>/` (`--no-backup` to skip), then
`reconcile_progress()` reconciles saved decisions against the current
queue: exact key match first, falling back to same-image/same-class
nearest-box matching within a tolerance if a box's stored key no longer
matches exactly (e.g. the queue was regenerated with slightly shifted
coordinates). Unresolved entries are **kept under their old key** (never
deleted) and printed for manual review — nothing from a prior session is
ever silently discarded, only flagged when it can't be automatically
matched. Skip with `--no-key-reconcile` for debugging.

## Notable fixed incidents (from the in-file version history)

- **V11** fixed a real incident where decisions appeared to "disappear"
  after a queue regeneration: manual boxes previously bypassed the
  pending→decided state machine queue boxes went through, which is what
  made them vulnerable to a queue regen. Startup reconciliation,
  automatic per-launch backups, and sync coverage for manual boxes were
  all added in response.
- **V13** was a correctness pass: completion checks moved to using the
  full unfiltered queue (a filtered `--source`/`--cls` session was
  incorrectly marking images "complete" based on only the filtered
  subset); reversing a synced decision now correctly retracts exactly
  what it synced; a reposition-arm was no longer accidentally disarmed by
  a non-drag click; sync now respects manual class overrides; sync writes
  force-flush immediately instead of waiting for the debounce.

## CLI

```bash
python review_labels.py                # everything pending
python review_labels.py --source UAVDT --cls person
python review_labels.py --show-all      # revisit already-decided images too
python review_labels.py --qa            # QA pass: only fully-decided images
python review_labels.py --hide-original # don't overlay real-dataset labels
python review_labels.py --no-sync       # cross-frame sync OFF at startup
python review_labels.py --sync-window 5 # sync up to 5 frames out each direction
python review_labels.py --no-backup         # skip the automatic per-launch backup (not recommended)
python review_labels.py --no-key-reconcile  # skip the startup key-repair pass (debugging only)
```
