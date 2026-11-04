"""
restructure_project.py -- one-time reorganisation of Project R.A.D
==================================================================
Run from the repository root (the folder with "Project R.A.D.slnx"),
with Visual Studio CLOSED:

    python _restructure\\restructure_project.py --dry-run   # show the plan, change nothing
    python _restructure\\restructure_project.py             # do it
    python _restructure\\restructure_project.py --undo      # put everything back

What it does
  1. Safety checks: git working tree clean (commit first), every file it is
     going to modify still has the content it was prepared from (SHA-256),
     no destination exists yet.
  2. Moves files into the new layout with `git mv` (history is kept);
     untracked files are moved with a normal file move.
  3. Writes the prepared new/changed files from _restructure/payload/
     (import paths, project files, docs, the command-uplink code) and
     `git add`s them.
  4. Removes the emptied old folders and stale __pycache__ folders.
Every step is recorded in _restructure/journal.json, and every file it
overwrites is backed up in _restructure/backup/, so --undo can restore the
exact previous state. Nothing is committed: review with `git status`,
build in Visual Studio, run the tests, then commit.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PAYLOAD = os.path.join(HERE, "payload")
BACKUP = os.path.join(HERE, "backup")
JOURNAL = os.path.join(HERE, "journal.json")


def p(rel):
    return os.path.join(ROOT, *rel.split("/"))


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def git(*args, check=True):
    r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip() or r.stdout.strip()}")
    return r


def have_git():
    try:
        return git("rev-parse", "--is-inside-work-tree", check=False).stdout.strip() == "true"
    except FileNotFoundError:
        return False


def tracked(rel):
    return git("ls-files", "--error-unmatch", "--", rel, check=False).returncode == 0


def ignored(rel):
    return git("check-ignore", "-q", "--", rel, check=False).returncode == 0


def load_plan():
    with open(os.path.join(HERE, "moves.json"), encoding="utf-8") as f:
        plan = json.load(f)
    with open(os.path.join(HERE, "hashes.json"), encoding="utf-8") as f:
        hashes = json.load(f)
    moves = [tuple(m) for m in plan["moves"]]
    # whole folders (photos, drawings, screenshots): expand to their files
    for src_dir, dst_dir in plan["dir_moves"]:
        base = p(src_dir)
        if not os.path.isdir(base):
            continue
        for r, dirs, files in os.walk(base):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for fn in sorted(files):
                full = os.path.join(r, fn)
                rel = os.path.relpath(full, ROOT).replace(os.sep, "/")
                moves.append((rel, dst_dir + rel[len(src_dir):]))
    payload = []
    for r, _, files in os.walk(PAYLOAD):
        for fn in files:
            payload.append(os.path.relpath(os.path.join(r, fn), PAYLOAD).replace(os.sep, "/"))
    return moves, sorted(payload), hashes, plan["remove_empty_dirs"]


# ═════════════════════════════════════════════════════════════════════════════
def check(moves, payload, hashes, use_git, force):
    problems = []
    if not os.path.exists(p("Project R.A.D.slnx")):
        problems.append("Run this from the Project R.A.D repository (Project R.A.D.slnx not found).")
    if use_git and not force:
        dirty = git("status", "--porcelain", "--untracked-files=no").stdout.strip()
        if dirty:
            problems.append("Uncommitted changes -- commit (or stash) them first:\n" + dirty)
    for rel, h in hashes.items():
        if not os.path.exists(p(rel)):
            problems.append(f"missing: {rel}")
        elif sha(p(rel)) != h:
            problems.append(f"changed since the reorganisation was prepared: {rel}")
    srcs = {s for s, _ in moves}
    for s, d in moves:
        if not os.path.exists(p(s)):
            problems.append(f"missing: {s}")
        if os.path.exists(p(d)):
            problems.append(f"destination already exists: {d}")
    for rel in payload:
        if os.path.exists(p(rel)) and rel not in hashes and rel not in srcs:
            problems.append(f"would overwrite an unexpected file: {rel}")
    return problems


def run(moves, payload, hashes, remove_dirs, use_git):
    journal = {"moves": [], "created": [], "backed_up": [], "git": use_git}

    def save():
        with open(JOURNAL, "w", encoding="utf-8") as f:
            json.dump(journal, f, indent=1)

    # 1. moves
    for s, d in moves:
        os.makedirs(os.path.dirname(p(d)), exist_ok=True)
        via_git = use_git and tracked(s)
        if via_git:
            git("mv", "--", s, d)
        else:
            shutil.move(p(s), p(d))
        journal["moves"].append([s, d, via_git])
        save()
        print(f"  {'git mv' if via_git else 'move  '}  {s}  ->  {d}")

    # 2. payload (new and changed files)
    for rel in payload:
        dst = p(rel)
        if os.path.exists(dst):
            bk = os.path.join(BACKUP, *rel.split("/"))
            os.makedirs(os.path.dirname(bk), exist_ok=True)
            shutil.copy2(dst, bk)
            journal["backed_up"].append(rel)
        else:
            journal["created"].append(rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(os.path.join(PAYLOAD, *rel.split("/")), dst)
        save()
        if use_git and not ignored(rel):
            git("add", "--", rel)
        print(f"  write   {rel}")

    # 3. clean-up of emptied folders
    for r, dirs, _ in os.walk(p("DroneCockpitUI")):
        if "NNTraining" in r:
            continue
        for d in list(dirs):
            if d == "__pycache__":
                shutil.rmtree(os.path.join(r, d), ignore_errors=True)
                dirs.remove(d)
    for rel in remove_dirs:
        remove_empty_tree(p(rel))
    print("\nDone. Nothing is committed yet. Next:")
    print("  1. git status                      -- review")
    print("  2. Visual Studio: open the solution, Rebuild DroneBackend (Release x64)")
    print("  3. cd DroneCockpitUI && pytest     -- unit tests")
    print("  4. run DroneCockpitUI.py")
    print("  5. git commit, then delete the _restructure folder")
    print("  Undo before committing: python _restructure\\restructure_project.py --undo")


def remove_empty_tree(path):
    if not os.path.isdir(path):
        return
    for r, dirs, files in os.walk(path, topdown=False):
        for d in dirs:
            if d == "__pycache__":
                shutil.rmtree(os.path.join(r, d), ignore_errors=True)
        try:
            os.rmdir(r)
        except OSError:
            pass
    if os.path.isdir(path):
        left = sorted(os.listdir(path))
        print(f"  note: {os.path.relpath(path, ROOT)} not removed, still contains: {left}")


def undo():
    if not os.path.exists(JOURNAL):
        sys.exit("No journal -- nothing to undo.")
    with open(JOURNAL, encoding="utf-8") as f:
        j = json.load(f)
    use_git = j.get("git", False)
    for rel in reversed(j["created"]):
        if os.path.exists(p(rel)):
            if use_git and tracked(rel):
                git("rm", "-q", "--cached", "--", rel, check=False)
            os.remove(p(rel))
            print(f"  delete  {rel}")
    for rel in reversed(j["backed_up"]):
        shutil.copy2(os.path.join(BACKUP, *rel.split("/")), p(rel))
        if use_git and not ignored(rel) and tracked(rel):
            git("add", "--", rel)
        print(f"  restore {rel}")
    for s, d, via_git in reversed(j["moves"]):
        os.makedirs(os.path.dirname(p(s)), exist_ok=True)
        if via_git:
            git("mv", "--", d, s)
        else:
            shutil.move(p(d), p(s))
        print(f"  back    {d}  ->  {s}")
    for top in ("DroneBackend/src", "DroneBackend/scripts", "DroneCockpitUI/widgets",
                "DroneCockpitUI/workers", "DroneCockpitUI/core", "DroneCockpitUI/control",
                "DroneCockpitUI/config", "DroneCockpitUI/tools", "DroneCockpitUI/tests",
                "DroneCockpitUI/NNTraining/reports", "Firmware", "docs/doxygen", "docs/hardware",
                "docs/screenshots"):
        remove_empty_tree(p(top))
    os.rename(JOURNAL, JOURNAL + ".undone")
    shutil.rmtree(BACKUP, ignore_errors=True)
    print("\nUndo finished. `git status` should show no changes except _restructure/.")


def main():
    ap = argparse.ArgumentParser(description="Reorganise Project R.A.D (one time).")
    ap.add_argument("--dry-run", action="store_true", help="show the plan only")
    ap.add_argument("--undo", action="store_true", help="revert a previous run")
    ap.add_argument("--yes", action="store_true", help="don't ask for confirmation")
    ap.add_argument("--force", action="store_true", help="skip the clean-working-tree check")
    a = ap.parse_args()

    if a.undo:
        undo()
        return
    if os.path.exists(JOURNAL):
        sys.exit("Already run (journal.json exists). Use --undo first to run again.")

    use_git = have_git()
    if not use_git:
        print("WARNING: git not found -- files will be moved without keeping history.")
    moves, payload, hashes, remove_dirs = load_plan()
    problems = check(moves, payload, hashes, use_git, a.force)
    print(f"Plan: {len(moves)} moves, {len(payload)} new/changed files.")
    if problems:
        print("\nCannot continue:")
        for x in problems:
            print("  -", x)
        sys.exit(1)
    if a.dry_run:
        for s, d in moves:
            print(f"  move   {s}  ->  {d}")
        for rel in payload:
            print(f"  write  {rel}")
        print("\nDry run only, nothing changed.")
        return
    print("Close Visual Studio before continuing (it keeps project files open).")
    if not a.yes and input("Continue? [y/N] ").strip().lower() not in ("y", "yes", "d", "da"):
        sys.exit("Cancelled.")
    run(moves, payload, hashes, remove_dirs, use_git)


if __name__ == "__main__":
    main()
