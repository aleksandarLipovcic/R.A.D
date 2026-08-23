"""
dedupe_real_labels.py -- One-off cleanup for real dataset label files that
already picked up double-labeled objects from a --apply run of
generate_pseudo_labels.py made before its merge dedup was fixed to compare
boxes geometrically instead of as exact strings (see DUPLICATE_IOU_THRESHOLD
in that module).

This does NOT touch labels_backup_original_<split>/ (prepare_datasets.py's
one-time pre-remap snapshot) or read from it -- it only rewrites the real
label files currently in place, in the same unified-taxonomy indices they're
already in. After a successful --apply, labels_backup_reviewed_<split>/ is
just a live mirror of the real files, so it would already contain the same
duplicates; this script re-syncs that mirror automatically once it's done
(same sync_reviewed_backups() the main script calls after --apply), so you
don't need to run anything else afterward.

WHAT IT DOES: for every real label file under datasets/<SOURCE>/<split>/
labels/ (same discovery glob generate_pseudo_labels.py's
sync_reviewed_backups() uses), walks its lines in order and drops any line
that's a geometric duplicate (same class, IoU >= DUPLICATE_IOU_THRESHOLD) of
one already kept earlier in that same file -- keeping the FIRST occurrence,
dropping the rest. Class mismatches or non-overlapping boxes are always
kept, no matter how close together.

USAGE:
    # Dry run -- report what WOULD change, write nothing:
    python dedupe_real_labels.py --datasets-dir datasets

    # Apply for real, then refresh labels_backup_reviewed_<split>/ to match:
    python dedupe_real_labels.py --datasets-dir datasets --apply

    # Everything removed is also written to dedupe_report.json (dry run or
    # --apply) so you can eyeball exactly what was considered a duplicate
    # before trusting it, or re-run against a different threshold:
    python dedupe_real_labels.py --datasets-dir datasets --iou-threshold 0.5
"""

import argparse
import json
from pathlib import Path

import generate_pseudo_labels as gpl


def find_real_label_dirs(datasets_dir: Path) -> list[Path]:
    """Same discovery logic as gpl.sync_reviewed_backups(): Roboflow-style
    datasets/<SOURCE>/<split>/labels/ and datasets/external/<name>/<split>/
    labels/, skipping pseudo-label staging trees and review-tool output --
    those aren't real dataset label files."""
    excluded_tops = ("label_gap_review", "review_backups")
    candidates = set(datasets_dir.glob("*/*/labels")) | set(datasets_dir.glob("*/*/*/labels"))
    out = []
    for labels_dir in sorted(candidates):
        if not labels_dir.is_dir():
            continue
        top = labels_dir.relative_to(datasets_dir).parts[0]
        if top.startswith("pseudo_labels") or top in excluded_tops:
            continue
        out.append(labels_dir)
    return out


def dedupe_file(lines: list[str]) -> tuple[list[str], list[str]]:
    """Returns (kept, removed) -- `lines` filtered down to one line per
    geometric duplicate group (first occurrence wins), and the list of
    lines that were dropped, in the order they were dropped."""
    kept: list[tuple] = []  # (line_text, parsed_or_None)
    removed = []
    for l in lines:
        parsed = gpl._parse_yolo_line(l)
        if gpl._is_duplicate_line(l, parsed, kept):
            removed.append(l)
            continue
        kept.append((l, parsed))
    return [l for l, _ in kept], removed


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--datasets-dir", type=Path, default=Path("datasets"))
    p.add_argument("--apply", action="store_true",
                   help="Actually rewrite the real label files (default: dry "
                        "run -- report only, no files touched).")
    p.add_argument("--iou-threshold", type=float, default=gpl.DUPLICATE_IOU_THRESHOLD,
                   help=f"Same-class IoU at/above which two boxes count as "
                        f"one duplicated object (default: "
                        f"{gpl.DUPLICATE_IOU_THRESHOLD}, matching "
                        f"generate_pseudo_labels.py's own merge threshold).")
    p.add_argument("--report-file", type=Path, default=Path("dedupe_report.json"))
    args = p.parse_args()

    gpl.DUPLICATE_IOU_THRESHOLD = args.iou_threshold

    label_dirs = find_real_label_dirs(args.datasets_dir)
    if not label_dirs:
        print(f"No real label directories found under {args.datasets_dir} "
              f"-- check --datasets-dir.")
        return

    total_files_checked = 0
    total_lines_removed = 0
    report = {}

    for labels_dir in label_dirs:
        for txt_path in sorted(labels_dir.glob("*.txt")):
            total_files_checked += 1
            lines = [l.strip() for l in txt_path.read_text().splitlines() if l.strip()]
            if not lines:
                continue
            kept, removed = dedupe_file(lines)
            if not removed:
                continue
            total_lines_removed += len(removed)
            report[str(txt_path)] = removed
            print(f"  {txt_path}: removing {len(removed)} duplicate line(s)")
            if args.apply:
                txt_path.write_text("\n".join(kept) + "\n" if kept else "")

    args.report_file.write_text(json.dumps(report, indent=2))

    print(f"\n{'[APPLIED]' if args.apply else '[DRY RUN]'} checked "
          f"{total_files_checked} label file(s) across {len(label_dirs)} "
          f"directory(ies), {'removed' if args.apply else 'would remove'} "
          f"{total_lines_removed} duplicate line(s) across "
          f"{len(report)} file(s).")
    print(f"Full before/after detail written to {args.report_file}")

    if not args.apply:
        print("Nothing was written -- rerun with --apply once this looks right.")
    else:
        print("\nRefreshing labels_backup_reviewed_<split>/ to match the "
              "cleaned real label files...")
        stats = gpl.sync_reviewed_backups(args.datasets_dir)
        print(f"  backup sync: {stats}")


if __name__ == "__main__":
    main()
