"""
check_split_leakage.py -- Are the valid/test images independent of train?
=======================================================================
Frames cut from the same video look almost identical. If a random frame split
puts neighbouring frames into train AND valid/test, the "held-out" score mostly
measures how well the model remembers scenes it has already seen, not how well
it generalises. This script measures that risk for a Roboflow-style dataset
(<root>/train|valid|test/images), e.g. datasets\\SARD.

Two independent checks, no extra packages beyond numpy + Pillow:

  1. FILE-NAME ADJACENCY. Names like gss1006_jpg.rf.<hash>.jpg carry a frame
     number. For every valid/test image it reports the distance (in frame
     numbers) to the nearest TRAIN frame of the same prefix. Distances of 1-5
     mean "the neighbouring frame is in train".
  2. IMAGE SIMILARITY. Every image is shrunk to a 32x32 grey thumbnail and
     mean-centred; the score is the correlation with the most similar TRAIN
     image (1.0 = identical layout). Near-duplicate video frames score ~0.95+
     while genuinely different scenes score far lower. This is a rough
     screening measure, not proof.

USAGE (from NNTraining):
    python check_split_leakage.py --root "..\\datasets\\SARD"
    python check_split_leakage.py --root "..\\datasets\\SARD" --threshold 0.95 --top 30

OUTPUT: a summary table per split, plus leakage_pairs.csv with the most
similar (valid/test image, train image) pairs so you can open a few and look.
"""

import argparse
import csv
import re
from pathlib import Path

import numpy as np
from PIL import Image

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
NAME_RE = re.compile(r"^([A-Za-z_]*?)(\d+)_(?:jpg|png|jpeg)", re.IGNORECASE)


def list_images(folder: Path):
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS) if folder.exists() else []


def thumb(path: Path, size: int) -> np.ndarray:
    im = Image.open(path).convert("L").resize((size, size), Image.BILINEAR)
    v = np.asarray(im, dtype=np.float32).ravel()
    v -= v.mean()
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def frame_key(path: Path):
    m = NAME_RE.match(path.name)
    return (m.group(1), int(m.group(2))) if m else None


def best_match(query_vecs, train_vecs, chunk=256):
    """For each query, (best similarity, index of the most similar train image)."""
    sims = np.empty(len(query_vecs), dtype=np.float32)
    idx = np.empty(len(query_vecs), dtype=np.int64)
    for i in range(0, len(query_vecs), chunk):
        block = query_vecs[i:i + chunk] @ train_vecs.T
        idx[i:i + chunk] = block.argmax(axis=1)
        sims[i:i + chunk] = block.max(axis=1)
    return sims, idx


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, required=True,
                    help="Dataset root containing train/, valid/ (or val/) and test/ folders.")
    ap.add_argument("--thumb", type=int, default=32, help="Thumbnail side in pixels.")
    ap.add_argument("--threshold", type=float, default=0.95,
                    help="Similarity at/above which a pair counts as a near-duplicate.")
    ap.add_argument("--top", type=int, default=25, help="Pairs written to leakage_pairs.csv.")
    ap.add_argument("--out", type=Path, default=Path("leakage_pairs.csv"))
    args = ap.parse_args()

    train_imgs = list_images(args.root / "train" / "images")
    if not train_imgs:
        raise SystemExit(f"No images found in {args.root / 'train' / 'images'}")
    splits = {}
    for name, folders in (("valid", ("valid", "val")), ("test", ("test",))):
        for f in folders:
            imgs = list_images(args.root / f / "images")
            if imgs:
                splits[name] = imgs
                break
    if not splits:
        raise SystemExit("No valid/test images found.")

    print(f"train images: {len(train_imgs)}  |  " + "  ".join(f"{k}: {len(v)}" for k, v in splits.items()))
    print("Building thumbnails ...")
    train_vecs = np.stack([thumb(p, args.thumb) for p in train_imgs])

    # frame numbers per prefix in train
    train_frames = {}
    for p in train_imgs:
        k = frame_key(p)
        if k:
            train_frames.setdefault(k[0], []).append(k[1])
    train_frames = {k: np.array(sorted(set(v))) for k, v in train_frames.items()}

    rows = []
    print(f"\n{'split':7s} {'images':>7s} {'median sim':>11s} {'sim>=thr':>9s} {'sim>=0.98':>10s} "
          f"{'frame dist<=5':>14s} {'frame dist<=20':>15s}")
    for name, imgs in splits.items():
        vecs = np.stack([thumb(p, args.thumb) for p in imgs])
        sims, idx = best_match(vecs, train_vecs)
        dists = []
        for p in imgs:
            k = frame_key(p)
            if k and k[0] in train_frames:
                dists.append(int(np.abs(train_frames[k[0]] - k[1]).min()))
        dists = np.array(dists)
        near5 = f"{100 * (dists <= 5).mean():5.1f}%" if len(dists) else "n/a"
        near20 = f"{100 * (dists <= 20).mean():5.1f}%" if len(dists) else "n/a"
        print(f"{name:7s} {len(imgs):>7d} {np.median(sims):>11.3f} {100 * (sims >= args.threshold).mean():>8.1f}% "
              f"{100 * (sims >= 0.98).mean():>9.1f}% {near5:>14s} {near20:>15s}")
        for j in np.argsort(-sims)[:args.top]:
            rows.append([name, imgs[j].name, train_imgs[idx[j]].name, f"{sims[j]:.4f}"])

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["split", "image", "most_similar_train_image", "similarity"])
        w.writerows(rows)
    print(f"\nMost similar pairs written to {args.out}  (open a few side by side and look).")
    print("How to read: if most valid/test images have a train neighbour with similarity >= 0.95 and/or a "
          "train frame within a few frame numbers, the split is NOT independent -- the held-out score is "
          "optimistic. Genuinely different scenes typically score well below 0.9.")


if __name__ == "__main__":
    main()