"""
audit_sync.py -- one-off audit for review_progress.json's cross-frame
sync entries, to find boxes synced under the PRE-V16 sequence/frame
parsing bug (see review_labels.py's V16 module note).

WHAT IT DOES
For every "synced_from" pointer currently in review_progress.json --
both manual-box syncs (stored under "__manual__<image>" entries) and
queue-decision syncs (stored under normal item_key entries) -- this
re-derives the REAL sequence id and frame number for both sides
(the box's own image, and the image it claims to be synced from)
using the CURRENT parse_seq_frame() from review_labels.py, and
checks:

  1. Same sequence id?          (if not: definitely a stale/bogus
                                  sync from the pre-fix bug -- almost
                                  certainly a different video)
  2. Frame distance <= window?  (if sequence matches but the frame
                                  gap is absurdly large, that's also
                                  suspicious -- could still be a
                                  legitimate wide --sync-window run,
                                  so this is reported separately and
                                  more leniently than #1)

Nothing is deleted or modified. This only reports. Decide what to do
with flagged entries (reset to pending, manually re-review, etc.)
after looking at the report.

USAGE
    cd <same directory as review_labels.py, prepare_datasets.py, etc.>
    python audit_sync.py [--window N] [--out report.json]

    --window sets the frame-distance sanity threshold for check #2
    above (default: SYNC_MAX_WINDOW from review_labels.py, i.e. the
    widest window the tool itself ever allowed -- so anything beyond
    that literally could not have been produced by a legitimate sync,
    regardless of what --sync-window a session was run with).
"""

import argparse
import json
from pathlib import Path

import prepare_datasets
import review_labels as rl


def load_queue(datasets_dir: Path):
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    if not queue_path.exists():
        raise SystemExit(f"{queue_path} not found.")
    with open(queue_path) as f:
        items = json.load(f)
    for it in items:
        it["_orig_box"] = list(it["box"])
    by_image = {}
    for it in items:
        by_image.setdefault(it["image"], []).append(it)
    return by_image


def audit(progress: dict, by_image_full: dict, window: int):
    mismatched_seq = []      # sequence id doesn't match at all
    far_frame = []           # same sequence, but frame gap > window
    unparseable = []         # one or both sides couldn't be parsed at all
    ok = 0
    total_checked = 0

    for key, entry in progress.items():
        if key.startswith("__override__"):
            continue

        if key.startswith("__manual__"):
            image = key[len("__manual__"):]
            for mb in entry:
                synced_from = mb.get("synced_from")
                if not synced_from:
                    continue
                total_checked += 1
                _check_pair(image, synced_from, "manual", mb.get("cls"),
                            window, ok_counter=None,
                            mismatched_seq=mismatched_seq, far_frame=far_frame,
                            unparseable=unparseable,
                            extra={"manual_id": mb.get("_id")})
            continue

        # Regular queue-decision entries: "image" + "synced_from" live
        # directly on the entry.
        synced_from = entry.get("synced_from")
        if not synced_from:
            continue
        total_checked += 1
        image = entry.get("image")
        _check_pair(image, synced_from, "queue", entry.get("cls"),
                    window, ok_counter=None,
                    mismatched_seq=mismatched_seq, far_frame=far_frame,
                    unparseable=unparseable,
                    extra={"progress_key": key})

    ok = total_checked - len(mismatched_seq) - len(far_frame) - len(unparseable)
    return {
        "total_synced_entries_checked": total_checked,
        "ok": ok,
        "mismatched_sequence": mismatched_seq,
        "same_sequence_but_far_frame": far_frame,
        "unparseable": unparseable,
    }


def _check_pair(image, synced_from, kind, cls, window,
                 ok_counter, mismatched_seq, far_frame, unparseable, extra):
    sf_target = rl.parse_seq_frame(image)
    sf_source = rl.parse_seq_frame(synced_from)

    if sf_target is None or sf_source is None:
        unparseable.append({
            "kind": kind, "cls": cls, "image": image, "synced_from": synced_from,
            "target_parsed": sf_target, "source_parsed": sf_source, **extra,
        })
        return

    seq_t, frame_t = sf_target
    seq_s, frame_s = sf_source

    if seq_t != seq_s:
        mismatched_seq.append({
            "kind": kind, "cls": cls, "image": image, "synced_from": synced_from,
            "target_seq": seq_t, "target_frame": frame_t,
            "source_seq": seq_s, "source_frame": frame_s, **extra,
        })
        return

    if abs(frame_t - frame_s) > window:
        far_frame.append({
            "kind": kind, "cls": cls, "image": image, "synced_from": synced_from,
            "seq": seq_t, "target_frame": frame_t, "source_frame": frame_s,
            "frame_gap": abs(frame_t - frame_s), **extra,
        })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=int, default=rl.SYNC_MAX_WINDOW,
                     help=f"Max acceptable frame distance for a legitimate sync "
                          f"(default: {rl.SYNC_MAX_WINDOW}, the tool's own hard "
                          f"ceiling on --sync-window).")
    ap.add_argument("--out", default="sync_audit_report.json")
    args = ap.parse_args()

    datasets_dir = prepare_datasets.DATASETS_DIR
    progress_path = datasets_dir / rl.PROGRESS_FILENAME
    progress = rl.load_json_dict(progress_path)
    by_image_full = load_queue(datasets_dir)

    report = audit(progress, by_image_full, args.window)

    print(f"Checked {report['total_synced_entries_checked']} synced entries.")
    print(f"  OK (same sequence, within window):        {report['ok']}")
    print(f"  MISMATCHED SEQUENCE (different video):     {len(report['mismatched_sequence'])}")
    print(f"  Same sequence but frame gap > {args.window}:          {len(report['same_sequence_but_far_frame'])}")
    print(f"  Unparseable (couldn't derive seq/frame):    {len(report['unparseable'])}")

    with open(args.out, "w") as f:
        json.dump(report, f, indent=2, default=str)
    print(f"\nFull details written to {args.out}")
    print("Nothing was modified -- this is read-only. Decide how to handle "
          "flagged entries (e.g. reset to pending) as a separate step.")


if __name__ == "__main__":
    main()