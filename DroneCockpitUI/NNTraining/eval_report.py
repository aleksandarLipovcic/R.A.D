"""
eval_report.py -- Standalone evaluation of a trained Project R.A.D model
=======================================================================
Produces every number the training/validation/test report needs from ONE
run, saved in machine-readable form (JSON + CSV) plus Ultralytics' own
plots (confusion matrix, PR/F1/P/R curves) for each evaluated set.

WHY THIS EXISTS: train.py runs the per-source evaluation only when a
training run FINISHES. yolom_main_run was stopped by hand at epoch 65, so
that step never ran, and the SARD/UAVDT/... test splits that
prepare_datasets.py remaps and holds out are never evaluated anywhere (the
generated unified.yaml has train/val only). This script covers both.

WHAT IT EVALUATES (each is a separate model.val() call):
  1. blended    -- the full unified val set (datasets/unified.yaml).
  2. per-source -- each dataset's own val set, from datasets/per_source_val.json
                   (same manifest train.py's evaluate_per_source() uses).
  3. test       -- every held-out test/ split found under datasets/
                   (Roboflow-style <root>/test/images, e.g. SARD). These were
                   never used for training OR for choosing best.pt, so they
                   are the only labelled data that is not touched by model
                   selection. Note SARD is person-only, so its test set can
                   only speak for the person class.

USAGE (run from the folder that holds train.py / prepare_datasets.py):
    # 0. See what would be evaluated -- loads no model, takes a second:
    python eval_report.py --weights runs\\detect\\yolom_main_run\\weights\\best.pt --list

    # 1. Real run. CPU because the GPU currently fails with the cuDNN
    #    version mismatch; slow but reliable. Use --device 0 if that's fixed.
    python eval_report.py --weights runs\\detect\\yolom_main_run\\weights\\best.pt --device cpu --out runs\\eval_final

    # Only UAVDT (per-source) + the held-out test set, ~10 min on CPU:
    python eval_report.py --weights ... --device cpu --out runs\\eval_plots --skip-blended --only UAVDT test

    # Only the held-out test sets (fast):
    python eval_report.py --weights ... --skip-blended --skip-per-source

Outputs under --out:
    eval_results.json   everything, incl. checkpoint epoch/fitness, speed
    eval_summary.csv    one row per (set, class) -- easy to chart
    plots/<set>/        confusion matrix + curves per evaluated set
    yamls/              the temporary dataset yamls that were used
Results are written after EVERY set, so a crash or Ctrl+C keeps what
finished. A failing set is recorded and the rest still run.
"""

import argparse
import csv
import gc
import json
import re
import sys
from datetime import datetime
from pathlib import Path

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}
# Top-level dataset folders that are tooling output, not real datasets.
SKIP_TOPS = ("label_gap_review", "review_backups")
# xView is inactive in the current taxonomy -- never evaluate it.
DEFAULT_EXCLUDED_SOURCES = ("xview",)


def default_datasets_dir() -> Path:
    """Same rule prepare_datasets.py / train.py use: Ultralytics' global
    datasets_dir setting, falling back to ./datasets next to this file."""
    here = Path(__file__).resolve().parent
    try:
        from ultralytics.utils import SETTINGS
        return Path(SETTINGS.get("datasets_dir", here / "datasets"))
    except Exception:
        return here / "datasets"


def parse_args():
    ds = default_datasets_dir()
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--weights", required=True, help="Path to best.pt to evaluate.")
    p.add_argument("--datasets-dir", type=Path, default=ds,
                   help=f"Default: {ds} (Ultralytics datasets_dir setting).")
    p.add_argument("--unified-yaml", type=Path, default=None,
                   help="Default: <datasets-dir>/unified.yaml")
    p.add_argument("--out", type=Path, default=Path("runs/eval_final"))
    p.add_argument("--imgsz", type=int, default=960,
                   help="Match training (960).")
    p.add_argument("--batch", type=int, default=4,
                   help="Use 2 on the 6GB GPU; 4-8 is fine on CPU.")
    p.add_argument("--workers", type=int, default=0,
                   help="Keep 0 on Windows (paging-file exhaustion otherwise).")
    p.add_argument("--device", default="cpu",
                   help="'cpu' (default -- GPU currently hits the cuDNN "
                        "mismatch) or a GPU index like 0.")
    p.add_argument("--skip-blended", action="store_true")
    p.add_argument("--skip-per-source", action="store_true")
    p.add_argument("--skip-test", action="store_true")
    p.add_argument("--exclude-sources", nargs="*", default=list(DEFAULT_EXCLUDED_SOURCES),
                   help="Case-insensitive substrings; matching test sets are "
                        f"skipped. Default: {' '.join(DEFAULT_EXCLUDED_SOURCES)}")
    p.add_argument("--only", nargs="*", default=None,
                   help="Run only some sets. Each token matches a set NAME (e.g. UAVDT, "
                        "SARD) or a KIND (blended, per_source, test), case-insensitive. "
                        "Example: --only UAVDT test  -> the UAVDT per-source set plus "
                        "every held-out test set.")
    p.add_argument("--list", action="store_true",
                   help="Only print which sets would be evaluated, then exit "
                        "(does not load the model).")
    return p.parse_args()


# ---------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------

def slug(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", s).strip("_").lower()


def count_images(entries) -> int:
    """Image count behind a list of yaml entries (dirs or .txt path lists)."""
    n = 0
    for e in entries:
        p = Path(e)
        if p.is_dir():
            n += sum(1 for f in p.rglob("*") if f.suffix.lower() in IMAGE_EXTS)
        elif p.is_file():
            n += sum(1 for line in p.read_text(encoding="utf-8").splitlines() if line.strip())
    return n


def _as_list(v):
    if v is None:
        return []
    return list(v) if isinstance(v, (list, tuple)) else [v]


def discover_jobs(args):
    """Returns [(kind, name, entries, data_yaml_or_None)]. data_yaml is set
    only for the blended job (uses unified.yaml directly); the others get a
    temp yaml written at run time."""
    import yaml
    jobs = []
    ds = args.datasets_dir

    if not args.skip_blended:
        uy = args.unified_yaml or (ds / "unified.yaml")
        if uy.exists():
            with open(uy, encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            jobs.append(("blended", "unified_val", _as_list(cfg.get("val")), str(uy)))
        else:
            print(f"[warn] {uy} not found -- skipping blended validation.")

    if not args.skip_per_source:
        manifest = ds / "per_source_val.json"
        if manifest.exists():
            with open(manifest, encoding="utf-8") as f:
                per_source = json.load(f)
            for name, entries in per_source.items():
                jobs.append(("per_source", name, _as_list(entries), None))
        else:
            print(f"[warn] {manifest} not found (run prepare_datasets.py first?) "
                  f"-- skipping per-source validation.")

    if not args.skip_test:
        excluded = [x.lower() for x in args.exclude_sources]
        cands = set(ds.glob("*/test/images")) | set(ds.glob("external/*/test/images"))
        found_any = False
        for images_dir in sorted(cands):
            root = images_dir.parent.parent
            rel = root.relative_to(ds)
            top = rel.parts[0]
            label = "/".join(rel.parts)
            if top.startswith("pseudo_labels") or top in SKIP_TOPS:
                continue
            if any(x in label.lower() for x in excluded):
                continue
            if not any(f.suffix.lower() in IMAGE_EXTS for f in images_dir.iterdir()):
                continue
            # prepare_datasets.py writes test_filtered.txt only when some
            # pending-review images had to be dropped from this split.
            filtered = images_dir.parent / "test_filtered.txt"
            entry = str(filtered) if filtered.exists() else str(images_dir.resolve())
            jobs.append(("test", label, [entry], None))
            found_any = True
        if not found_any:
            print(f"[warn] no held-out test/images folders found under {ds} "
                  f"-- nothing to evaluate as 'test'.")
    if args.only:
        tokens = {t.lower() for t in args.only}
        jobs = [j for j in jobs if j[0].lower() in tokens or j[1].lower() in tokens]
        if not jobs:
            print(f"[warn] --only {args.only} matched no set.")
    return jobs


# ---------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------

def _f(x):
    try:
        return float(x)
    except Exception:
        return None


def extract_metrics(m, names: dict) -> dict:
    b = m.box
    res = {
        "map50": _f(b.map50),
        "map50_95": _f(b.map),
        "precision": _f(b.mp),   # mean over classes, at the confidence that maximises mean F1
        "recall": _f(b.mr),
        "per_class": {},
    }
    present = {}
    try:
        for pos, ci in enumerate(list(b.ap_class_index)):
            p, r, ap50, ap = b.class_result(pos)
            present[int(ci)] = {"precision": _f(p), "recall": _f(r),
                                "ap50": _f(ap50), "ap50_95": _f(ap)}
    except Exception as e:
        res["per_class_error"] = str(e)

    nt = None
    try:
        nt = [int(x) for x in getattr(m, "nt_per_class")]
    except Exception:
        pass

    for ci, cname in names.items():
        entry = present.get(int(ci), {"precision": None, "recall": None,
                                       "ap50": None, "ap50_95": None})
        entry = dict(entry)
        entry["instances"] = nt[int(ci)] if nt is not None and int(ci) < len(nt) else None
        res["per_class"][cname] = entry
    try:
        res["speed_ms_per_image"] = {k: _f(v) for k, v in dict(m.speed).items()}
    except Exception:
        pass
    return res


def clear_cuda():
    gc.collect()
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:
        pass


def run_job(model, job, args, names, out_dir: Path) -> dict:
    import yaml
    kind, name, entries, data_yaml = job
    tag = slug(f"{kind}_{name}")
    if data_yaml is None:
        ydir = out_dir / "yamls"
        ydir.mkdir(parents=True, exist_ok=True)
        data_yaml = str(ydir / f"{tag}.yaml")
        with open(data_yaml, "w", encoding="utf-8") as f:
            # Same trick train.py's evaluate_per_source() uses: the set to
            # score goes in `val`; `train` is required by the loader but
            # unused by val(). Test dirs are scored via split="val" too.
            yaml.safe_dump({"path": None, "train": entries, "val": entries,
                            "nc": len(names),
                            "names": [names[i] for i in sorted(names)]},
                           f, sort_keys=False)

    clear_cuda()
    m = model.val(data=data_yaml, split="val", imgsz=args.imgsz,
                  batch=args.batch, workers=args.workers, device=args.device,
                  plots=True, project=str((out_dir / "plots").resolve()), name=tag,
                  exist_ok=True, verbose=False)
    res = extract_metrics(m, names)
    res.update({"kind": kind, "name": name, "images": count_images(entries),
                "plots_dir": str(getattr(m, "save_dir", out_dir / "plots" / tag))})
    return res


def save_outputs(out_dir: Path, meta: dict, results: list, names: dict):
    def default(o):
        try:
            return float(o)
        except Exception:
            return str(o)

    with open(out_dir / "eval_results.json", "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "results": results}, f, indent=2, default=default)

    with open(out_dir / "eval_summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["kind", "set", "images", "class", "instances",
                    "precision", "recall", "ap50", "ap50_95"])
        for r in results:
            if "error" in r:
                w.writerow([r["kind"], r["name"], r.get("images"), "ERROR", "",
                            "", "", "", r["error"][:200]])
                continue
            inst_total = sum(v["instances"] or 0 for v in r["per_class"].values()) \
                if any(v["instances"] is not None for v in r["per_class"].values()) else ""
            w.writerow([r["kind"], r["name"], r["images"], "ALL", inst_total,
                        r["precision"], r["recall"], r["map50"], r["map50_95"]])
            for cname, v in r["per_class"].items():
                w.writerow([r["kind"], r["name"], r["images"], cname, v["instances"],
                            v["precision"], v["recall"], v["ap50"], v["ap50_95"]])


def fmt(x, nd=3):
    return "  n/a" if x is None else f"{x:.{nd}f}"


def print_table(results: list):
    print(f"\n{'=' * 78}\nSUMMARY\n{'=' * 78}")
    print(f"{'set':32s} {'imgs':>6s} {'mAP50':>7s} {'mAP50-95':>9s} {'P':>7s} {'R':>7s}")
    for r in results:
        if "error" in r:
            print(f"{(r['kind'] + ': ' + r['name'])[:32]:32s}  FAILED: {r['error'][:40]}")
            continue
        print(f"{(r['kind'] + ': ' + r['name'])[:32]:32s} {r['images']:>6d} "
              f"{fmt(r['map50']):>7s} {fmt(r['map50_95']):>9s} "
              f"{fmt(r['precision']):>7s} {fmt(r['recall']):>7s}")
        for cname, v in r["per_class"].items():
            print(f"    {cname:16s} inst={str(v['instances'] if v['instances'] is not None else 'n/a'):>7s} "
                  f"AP50={fmt(v['ap50']):>6s}  AP50-95={fmt(v['ap50_95']):>6s}  "
                  f"P={fmt(v['precision']):>6s}  R={fmt(v['recall']):>6s}")
    print("=" * 78)
    print("Note: a class with no instances in a set shows n/a -- expected for "
          "structurally missing classes\n(e.g. vehicles in SARD). P/R are at "
          "the single confidence that maximises mean F1 across classes, not at a fixed threshold.")


def main():
    args = parse_args()
    jobs = discover_jobs(args)

    print(f"\nDatasets dir : {args.datasets_dir}")
    print(f"Sets to evaluate ({len(jobs)}):")
    for kind, name, entries, _ in jobs:
        print(f"  [{kind:10s}] {name:28s} {count_images(entries):>7d} images")
    if args.list or not jobs:
        if not jobs:
            print("Nothing to evaluate.")
        return 0 if jobs else 1

    from ultralytics import YOLO
    import ultralytics

    args.out.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.weights)
    names = dict(model.names)

    ckpt = getattr(model, "ckpt", None) or {}
    meta = {
        "weights": str(Path(args.weights).resolve()),
        "ultralytics_version": ultralytics.__version__,
        # Ultralytics saves the 0-based epoch index; -1 means final/stripped.
        "ckpt_epoch_index": ckpt.get("epoch") if isinstance(ckpt, dict) else None,
        "ckpt_best_fitness": _f(ckpt.get("best_fitness")) if isinstance(ckpt, dict) else None,
        "classes": [names[i] for i in sorted(names)],
        "imgsz": args.imgsz, "batch": args.batch, "device": str(args.device),
        "run_started": datetime.now().isoformat(timespec="seconds"),
    }
    try:
        meta["parameters"] = int(sum(p.numel() for p in model.model.parameters()))
    except Exception:
        pass
    print(f"\nWeights: {meta['weights']}")
    print(f"Checkpoint epoch index: {meta['ckpt_epoch_index']}  "
          f"best_fitness: {meta['ckpt_best_fitness']}")

    results = []
    for i, job in enumerate(jobs, 1):
        kind, name = job[0], job[1]
        print(f"\n{'=' * 78}\n[{i}/{len(jobs)}] {kind}: {name}\n{'=' * 78}")
        try:
            r = run_job(model, job, args, names, args.out)
            print(f"  mAP50={fmt(r['map50'])}  mAP50-95={fmt(r['map50_95'])}  "
                  f"P={fmt(r['precision'])}  R={fmt(r['recall'])}")
        except KeyboardInterrupt:
            print("\nInterrupted -- saving what finished so far.")
            break
        except Exception as e:
            print(f"  FAILED: {e}")
            r = {"kind": kind, "name": name, "images": count_images(job[2]),
                 "error": str(e)}
            clear_cuda()
        results.append(r)
        save_outputs(args.out, meta, results, names)  # incremental save

    meta["run_finished"] = datetime.now().isoformat(timespec="seconds")
    save_outputs(args.out, meta, results, names)
    print_table(results)
    print(f"\nSaved: {args.out / 'eval_results.json'}\n       {args.out / 'eval_summary.csv'}"
          f"\n       {args.out / 'plots'}")
    failed = [r for r in results if "error" in r]
    if failed:
        print(f"\nWARNING: {len(failed)} set(s) failed: "
              + ", ".join(f"{r['kind']}:{r['name']}" for r in failed))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())