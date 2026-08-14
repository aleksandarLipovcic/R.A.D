"""
auto_filter_queue.py -- Automated pre-decision pass over
datasets/pseudo_labels_review_queue.json, so review_labels.py has far
fewer boxes left needing an actual human click.

WHY THIS EXISTS: the review queue is *already* the leftover after
generate_pseudo_labels.py applied its own vote-count thresholds
(3-4/4 -> auto-accept, 2/4 -> conditional auto-accept, 1/4 -> forced
review). Everything that lands in this queue file either only got 1
or 2 votes, or is tagged "always_review_class" (a class the pipeline
has decided must NEVER be auto-decided, regardless of votes -- e.g.
motorcycle boxes from SARD in the current dataset). So this script
does NOT just re-apply "N+ votes -> accept" -- that would resolve
nothing, since anything that already cleared that bar isn't here.

Instead it does two independent things, run in order, and BOTH are
purely additive on top of whatever review_labels.py already has in
review_progress.json:

  TIER 1 -- vote/confidence rules (no images touched, runs in
  seconds). Two-model spatial+class agreement is itself a real
  signal even when the combined confidence is modest, so a fairly
  small, defensible slice of the "below_threshold" (2-vote) bucket
  can be auto-accepted outright. Single-vote ("single_model_only")
  items get a much higher bar, and only for non-"person" classes by
  default -- see the note on why below.

  TIER 2 -- CLIP zero-shot crop verification (needs a GPU or a
  patient CPU, needs open_clip_torch). For whatever Tier 1 leaves
  pending, crops the box out of the actual image and scores it
  against a small bank of "is this really a <class>" vs. "or is it
  one of these known confusers" prompts (tuned for the aerial/drone
  domain -- poles, lamp posts, tree trunks, shadows, etc. for
  "person"; bicycles, generators, AC units, debris piles for
  "motorcycle"). Confident agreement/disagreement auto-decides it;
  genuine uncertainty is left pending -- that's exactly where your
  review time should go.

WHY "person" IS TREATED DIFFERENTLY: it's ~70% of this queue's
volume, it's the class the search-and-rescue mission actually cares
about most, and it's the class most prone to a specific known
false-positive pattern (a pole/post/pillar/shadow read as a person by
one model). Tier 1's confidence-only rules give that risk nowhere to
hide -- a lone model at 0.6 confidence looks the same whether it's a
real person or a lamp post. So by default Tier 1 leaves EVERY person
box pending (no auto-accept, no auto-reject) and routes it to Tier 2
instead, where the crop can actually be looked at. Override with
--tier1-classes if you disagree, but the default is deliberately
conservative given what this dataset is for.

NOTHING HERE IS DESTRUCTIVE OR FINAL:
  - Only ever touches an item currently "pending" (no saved decision
    at all, or an explicit "pending" entry). Anything you (or a
    previous run of this script) already decided is left completely
    alone -- safe to re-run after every manual session, after
    retuning thresholds, whatever.
  - "always_review_class" items are never touched, full stop -- no
    flag overrides this. That's an explicit decision already baked
    into your pipeline; this script isn't the place to second-guess
    it.
  - Every decision this script makes is a completely normal entry in
    review_progress.json (same schema review_labels.py writes) --
    open review_labels.py afterward and every auto-accepted/rejected
    box is sitting there in its normal color, fully visible, with
    "Reset to Pending" one click away if it got something wrong. QA
    mode (--qa / "Switch to QA / Completed") works on these exactly
    like it works on your own past decisions.
  - Reuses review_labels.py's own item_key(), backup_progress_files(),
    and reconcile_progress() by importing the module directly -- so
    there is zero risk of this tool's key format drifting out of sync
    with the review tool's, and every run gets the same startup
    backup-then-reconcile safety net review_labels.py gets on launch.

USAGE:
    # See exactly what it WOULD do -- no files touched at all.
    python auto_filter_queue.py --dry-run

    # Tier 1 only (fast, no GPU/model needed):
    python auto_filter_queue.py

    # Tier 1 + Tier 2 CLIP verification on whatever Tier 1 leaves
    # pending (needs: pip install open_clip_torch --break-system-packages):
    python auto_filter_queue.py --enable-clip

    # Sanity-check CLIP on a handful of crops BEFORE trusting it on
    # the whole queue -- dumps crops + scores as images, decides
    # nothing:
    python auto_filter_queue.py --enable-clip --sample-crops 40 --dry-run

    # Only these classes go through CLIP (default: all classes still
    # pending after Tier 1):
    python auto_filter_queue.py --enable-clip --clip-classes person,motorcycle

    # Tune thresholds (see --help for every knob and its default):
    python auto_filter_queue.py --accept2-conf 0.5 --accept1-conf 0.85
"""

import argparse
import collections
import json
import random
from pathlib import Path

import cv2
from PIL import Image

import prepare_datasets
import generate_pseudo_labels as gpl
import review_labels as rl   # reuses item_key/backup/reconcile -- see module docstring

# ----------------------------------------------------------------------
# Tier 1: vote / confidence rules
# ----------------------------------------------------------------------

# Classes Tier 1 will NEVER auto-accept/reject by confidence alone --
# left pending for Tier 2 (CLIP) or manual review instead. See the
# module docstring for why "person" is here by default.
DEFAULT_TIER1_SKIP_CLASSES = {"person"}


def tier1_decide(it, accept2_conf, accept1_conf, reject1_conf, skip_classes):
    """Returns ("accept" | "reject" | None, reason_str). None means
    "Tier 1 has no opinion, leave it pending" -- NOT a decision."""
    if it["cls"] in skip_classes:
        return None, "class excluded from tier1"

    votes = [v for v in it["votes"].values() if v is not None]
    if not votes:
        return None, "no confidence values"  # shouldn't happen, but never guess
    avg_conf = sum(votes) / len(votes)
    vc = it["vote_count"]

    if vc >= 2 and avg_conf >= accept2_conf:
        return "accept", f"{vc}-vote agreement, avg_conf={avg_conf:.3f} >= {accept2_conf}"
    if vc == 1 and accept1_conf is not None and avg_conf >= accept1_conf:
        return "accept", f"single-model, conf={avg_conf:.3f} >= {accept1_conf}"
    if vc == 1 and reject1_conf is not None and avg_conf <= reject1_conf:
        return "reject", f"single-model, conf={avg_conf:.3f} <= {reject1_conf}"
    return None, "no rule matched"


# ----------------------------------------------------------------------
# Tier 2: CLIP zero-shot crop verification
# ----------------------------------------------------------------------

# Prompt banks, tuned for aerial/drone imagery and this project's
# actual known failure modes (see module docstring / project history --
# a pole/post/pillar/shadow read as "person" is the recurring one).
# Each list is ensembled into a single averaged embedding (standard
# CLIP prompt-ensembling) rather than compared prompt-by-prompt, so
# adding/removing a phrasing here just nudges the average, it never
# creates a new decision axis.
CLIP_PROMPTS = {
    "person": {
        "positive": [
            "an aerial drone photo of a person standing outdoors",
            "a person seen from directly above by a drone",
            "a human figure walking, seen from a high aerial angle",
            "a person's body and shadow seen from above",
        ],
        "negative": [
            "an aerial photo of a utility pole or lamp post",
            "an aerial photo of a pillar or column",
            "an aerial photo of a tree trunk or small tree",
            "an aerial photo of a fire hydrant",
            "an aerial photo of a fence post or bollard",
            "an aerial photo of a mailbox or small roadside box",
            "an aerial photo of a long shadow on the ground with nothing casting it",
            "an aerial photo of a small rock or debris on the ground",
            "an aerial photo of a traffic sign post",
        ],
    },
    "motorcycle": {
        "positive": [
            "an aerial drone photo of a parked motorcycle",
            "an aerial photo of a motorcycle or moped from above",
            "a scooter or motorbike seen from a drone",
        ],
        "negative": [
            "an aerial photo of a bicycle",
            "an aerial photo of a generator or AC condenser unit",
            "an aerial photo of a water tank or barrel",
            "an aerial photo of a pile of debris or trash",
            "an aerial photo of a small garden cart or wheelbarrow",
            "an aerial photo of a rock or curb shadow",
        ],
    },
    "car": {
        "positive": [
            "an aerial drone photo of a parked car",
            "a sedan or hatchback seen from directly above",
        ],
        "negative": [
            "an aerial photo of a large truck or bus",
            "an aerial photo of a rooftop AC unit or skylight",
            "an aerial photo of a dumpster",
            "an aerial photo of a shadow with no vehicle",
        ],
    },
    "large_vehicle": {
        "positive": [
            "an aerial drone photo of a truck or bus",
            "a large vehicle seen from directly above",
        ],
        "negative": [
            "an aerial photo of a small parked car",
            "an aerial photo of a shipping container",
            "an aerial photo of a building rooftop section",
        ],
    },
    "other_vehicle": {
        "positive": [
            "an aerial drone photo of a vehicle of some kind",
            "a wheeled vehicle seen from directly above",
        ],
        "negative": [
            "an aerial photo of a rock, shadow, or ground debris",
            "an aerial photo of a building rooftop feature",
            "an aerial photo of vegetation or a tree",
        ],
    },
}


class ClipVerifier:
    """Thin wrapper around open_clip: loads once, exposes
    score(crop_pil, cls) -> prob_positive in [0, 1]."""

    def __init__(self, model_name, pretrained, device):
        import torch
        import open_clip
        self.torch = torch
        self.device = device
        print(f"[clip] loading {model_name} ({pretrained}) on {device} ...")
        self.model, _, self.preprocess = open_clip.create_model_and_transforms(
            model_name, pretrained=pretrained)
        self.model = self.model.to(device).eval()
        tokenizer = open_clip.get_tokenizer(model_name)

        self.pos_vecs = {}
        self.neg_vecs = {}
        with torch.no_grad():
            for cls, bank in CLIP_PROMPTS.items():
                pos_tok = tokenizer(bank["positive"]).to(device)
                neg_tok = tokenizer(bank["negative"]).to(device)
                pos_emb = self.model.encode_text(pos_tok)
                neg_emb = self.model.encode_text(neg_tok)
                pos_emb = pos_emb / pos_emb.norm(dim=-1, keepdim=True)
                neg_emb = neg_emb / neg_emb.norm(dim=-1, keepdim=True)
                # Prompt-ensemble: mean, then re-normalize.
                pos_vec = pos_emb.mean(dim=0)
                neg_vec = neg_emb.mean(dim=0)
                self.pos_vecs[cls] = pos_vec / pos_vec.norm()
                self.neg_vecs[cls] = neg_vec / neg_vec.norm()
        print(f"[clip] ready. prompt banks loaded for: {list(CLIP_PROMPTS)}")

    def score_batch(self, crops, classes):
        """crops: list of PIL.Image. classes: parallel list of class
        names. Returns list of prob_positive floats (one per crop),
        each scored against ITS OWN class's prompt bank."""
        torch = self.torch
        with torch.no_grad():
            tensors = torch.stack([self.preprocess(c) for c in crops]).to(self.device)
            img_emb = self.model.encode_image(tensors)
            img_emb = img_emb / img_emb.norm(dim=-1, keepdim=True)

            out = []
            for i, cls in enumerate(classes):
                bank = CLIP_PROMPTS.get(cls, CLIP_PROMPTS["other_vehicle"])
                pos_vec = self.pos_vecs.get(cls, self.pos_vecs["other_vehicle"])
                neg_vec = self.neg_vecs.get(cls, self.neg_vecs["other_vehicle"])
                sim_pos = (img_emb[i] @ pos_vec).item()
                sim_neg = (img_emb[i] @ neg_vec).item()
                # Standard CLIP logit scale for a 2-way softmax.
                logits = torch.tensor([sim_pos, sim_neg]) * 100.0
                probs = torch.softmax(logits, dim=0)
                out.append(probs[0].item())
        return out


def crop_with_padding(img_bgr, box, pad_frac, img_w, img_h):
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    pad_x, pad_y = w * pad_frac, h * pad_frac
    x1 = max(0, int(x1 - pad_x))
    y1 = max(0, int(y1 - pad_y))
    x2 = min(img_w, int(x2 + pad_x))
    y2 = min(img_h, int(y2 + pad_y))
    if x2 <= x1 or y2 <= y1:
        return None
    crop = img_bgr[y1:y2, x1:x2]
    return Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))


# ----------------------------------------------------------------------
# Output writing -- mirrors review_labels.py's own
# _accepted_items_for_image / _write_reviewed_labels_for_image /
# _recompute_completed_for_image exactly (copied logic, not the UI
# methods, since we never instantiate ReviewApp/Tk here).
# ----------------------------------------------------------------------

def accepted_items_for_image(image_key, by_image_full, progress):
    items = by_image_full.get(image_key, [])
    overrides = progress.get(f"__override__{image_key}", {})
    manual = progress.get(f"__manual__{image_key}", [])
    out = []
    for it in items:
        k = rl.item_key(it)
        saved = progress.get(k, {})
        if saved.get("decision") == "accept":
            cls = overrides.get(k, it["cls"])
            box = saved.get("box", it["box"])
            out.append({"cls": cls, "box": box})
    out.extend({"cls": mb["cls"], "box": mb["box"]} for mb in manual
               if mb.get("decision", "accepted") == "accepted")
    return out


def write_reviewed_labels_for_image(image_key, by_image_full, progress, reviewed_root):
    accepted = accepted_items_for_image(image_key, by_image_full, progress)
    img_path = Path(image_key)
    real_label_path = gpl.image_path_to_label_path(img_path)
    items = by_image_full.get(image_key, [])
    source_name = items[0]["source"] if items else None
    reviewed_label_path = gpl.mirror_under_pseudo_root(
        real_label_path, source_name, reviewed_root)

    if not accepted:
        if reviewed_label_path.exists():
            reviewed_label_path.unlink()
        return

    w, h = gpl.get_image_dims(image_key)
    lines = [gpl.box_to_yolo_line(a["cls"], a["box"], w, h) for a in accepted]
    reviewed_label_path.parent.mkdir(parents=True, exist_ok=True)
    reviewed_label_path.write_text("\n".join(lines) + "\n")


def recompute_completed_for_image(image_key, by_image_full, progress, completed_set):
    items = by_image_full.get(image_key, [])
    manual = progress.get(f"__manual__{image_key}", [])
    fully_decided = (
        all(progress.get(rl.item_key(it), {}).get("decision", "pending") != "pending"
            for it in items)
        and all(mb.get("decision", "accepted") != "pending" for mb in manual))
    if fully_decided and image_key not in completed_set:
        completed_set.add(image_key)
    elif not fully_decided and image_key in completed_set:
        completed_set.discard(image_key)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default=None, help="Only process this source (e.g. UAVDT).")
    p.add_argument("--cls", default=None, help="Only process this class (e.g. person).")
    p.add_argument("--dry-run", action="store_true",
                    help="Report exactly what would happen; touch no files.")
    p.add_argument("--limit", type=int, default=None,
                    help="Cap the number of images processed (testing).")

    g1 = p.add_argument_group("Tier 1 (vote/confidence rules)")
    g1.add_argument("--accept2-conf", type=float, default=0.45,
                     help="2-vote items: auto-accept if avg confidence >= this. "
                          "Default 0.45 (real data: resolves ~1450/4300 items in "
                          "this bucket at this bar -- see the printed report for "
                          "your actual dataset's numbers before committing).")
    g1.add_argument("--accept1-conf", type=float, default=0.80,
                     help="Single-vote items: auto-accept if confidence >= this. "
                          "Default 0.80 -- deliberately high since there's no "
                          "second model corroborating.")
    g1.add_argument("--reject1-conf", type=float, default=None,
                     help="Single-vote items: auto-reject if confidence <= this. "
                          "Default: disabled. On the dataset this was built "
                          "against, non-person single-vote confidences never "
                          "actually dropped below ~0.45 -- the very-low-confidence "
                          "single votes are almost all 'person', which Tier 1 "
                          "already routes to Tier 2 by default. Set this "
                          "explicitly if your queue's distribution differs "
                          "(the printed report shows your real percentiles).")
    g1.add_argument("--tier1-classes", default=None,
                     help="Comma-separated list of classes Tier 1 IS allowed to "
                          "decide by confidence. Default: every class except "
                          "'person' (see module docstring for why). Pass "
                          "'all' to include person too, or an explicit list to "
                          "override entirely.")

    g2 = p.add_argument_group("Tier 2 (CLIP crop verification)")
    g2.add_argument("--enable-clip", action="store_true",
                     help="Run CLIP verification on whatever Tier 1 leaves "
                          "pending. Requires: pip install open_clip_torch "
                          "--break-system-packages")
    g2.add_argument("--clip-classes", default=None,
                     help="Comma-separated classes to run through CLIP. "
                          "Default: all classes still pending after Tier 1.")
    g2.add_argument("--clip-model", default="ViT-B-32")
    g2.add_argument("--clip-pretrained", default="laion2b_s34b_b79k")
    g2.add_argument("--clip-device", default=None,
                     help="Default: cuda if available, else cpu.")
    g2.add_argument("--clip-margin", type=float, default=0.15,
                     help="Required deviation from 0.5 in prob_positive to "
                          "auto-decide: accept if prob_positive >= 0.5+margin, "
                          "reject if prob_positive <= 0.5-margin. Default 0.15 "
                          "(accept >=0.65 / reject <=0.35). Lower = more items "
                          "get auto-decided but with more risk; raise it if a "
                          "--sample-crops check shows too many wrong calls.")
    g2.add_argument("--clip-crop-pad", type=float, default=0.15,
                     help="Padding around each box before cropping, as a "
                          "fraction of the box's own width/height (gives CLIP "
                          "a little surrounding context). Default 0.15.")
    g2.add_argument("--clip-batch-size", type=int, default=32)
    g2.add_argument("--sample-crops", type=int, default=0,
                     help="Dump this many random crops (with their CLIP "
                          "prob_positive score in the filename) into "
                          "datasets/auto_filter_samples/ for a visual sanity "
                          "check. Does not affect decisions either way. "
                          "Combine with --dry-run to check CLIP before trusting "
                          "it on the real run.")

    g3 = p.add_argument_group("Safety net (mirrors review_labels.py)")
    g3.add_argument("--no-backup", action="store_true",
                     help="Skip the automatic progress/completed backup. Not recommended.")
    g3.add_argument("--no-key-reconcile", action="store_true",
                     help="Skip the automatic startup key-drift repair pass.")
    return p.parse_args()


def main():
    args = parse_args()
    datasets_dir = prepare_datasets.DATASETS_DIR
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    progress_path = datasets_dir / rl.PROGRESS_FILENAME
    completed_path = datasets_dir / rl.COMPLETED_FILENAME
    reviewed_root = datasets_dir / rl.REVIEWED_DIRNAME

    if not queue_path.exists():
        raise SystemExit(f"{queue_path} not found -- run generate_pseudo_labels.py first.")

    with open(queue_path) as f:
        all_items_full = json.load(f)
    for it in all_items_full:
        it["_orig_box"] = list(it["box"])

    by_image_full: dict = {}
    for it in all_items_full:
        by_image_full.setdefault(it["image"], []).append(it)

    progress = rl.load_json_dict(progress_path)
    completed_set = rl.load_completed(completed_path)

    if not args.no_backup and not args.dry_run:
        backup_dir = rl.backup_progress_files(datasets_dir, progress_path, completed_path)
        if backup_dir:
            print(f"[backup] snapshotted progress/completed files -> {backup_dir}")

    if not args.no_key_reconcile:
        progress, stats, unresolved = rl.reconcile_progress(by_image_full, progress)
        print(f"[reconcile] {stats['exact']} exact, {stats['fuzzy']} fuzzy-matched, "
              f"{stats['unresolved']} unresolved.")
        if not args.dry_run:
            rl.save_json(progress_path, progress)
        recomputed = rl.compute_completed_set(by_image_full, progress)
        if recomputed != completed_set:
            completed_set = recomputed
            if not args.dry_run:
                rl.save_completed(completed_path, completed_set)

    # Scope (--source/--cls) -- matches review_labels.py's own semantics:
    # only items matching the filter are candidates for a NEW decision,
    # but completeness/output for any touched image always considers the
    # image's FULL box set via by_image_full (same reasoning as V13 fix
    # A in review_labels.py -- see that module's docstring).
    candidates = all_items_full
    if args.source:
        candidates = [it for it in candidates if it["source"] == args.source]
    if args.cls:
        candidates = [it for it in candidates if it["cls"] == args.cls]
    if args.limit:
        images_allowed = set(list(dict.fromkeys(it["image"] for it in candidates))[:args.limit])
        candidates = [it for it in candidates if it["image"] in images_allowed]

    def current_decision(it):
        return progress.get(rl.item_key(it), {}).get("decision", "pending")

    pending = [it for it in candidates if current_decision(it) == "pending"]
    already_decided_n = len(candidates) - len(pending)

    # --- Tier 1 -----------------------------------------------------
    if args.tier1_classes is None:
        tier1_skip = set(DEFAULT_TIER1_SKIP_CLASSES)
    elif args.tier1_classes.strip().lower() == "all":
        tier1_skip = set()
    else:
        allowed = {c.strip() for c in args.tier1_classes.split(",") if c.strip()}
        tier1_skip = {c for c in rl.CLASS_NAMES if c not in allowed}

    new_decisions = {}   # item_key -> (decision, tier, reason, item)
    still_pending_after_t1 = []
    forced_manual = 0

    for it in pending:
        if it.get("reason") == "always_review_class":
            forced_manual += 1
            continue
        decision, why = tier1_decide(
            it, args.accept2_conf, args.accept1_conf, args.reject1_conf, tier1_skip)
        if decision:
            new_decisions[rl.item_key(it)] = (decision, "tier1", why, it)
        else:
            still_pending_after_t1.append(it)

    # --- Tier 2 (optional) -------------------------------------------
    sample_dir = datasets_dir / "auto_filter_samples"
    n_clip_scored = 0
    if args.enable_clip or args.sample_crops:
        clip_classes = (set(c.strip() for c in args.clip_classes.split(","))
                         if args.clip_classes else None)
        t2_targets = [it for it in still_pending_after_t1
                      if clip_classes is None or it["cls"] in clip_classes]

        if t2_targets:
            import torch
            device = args.clip_device or ("cuda" if torch.cuda.is_available() else "cpu")
            verifier = ClipVerifier(args.clip_model, args.clip_pretrained, device)

            by_img = collections.defaultdict(list)
            for it in t2_targets:
                by_img[it["image"]].append(it)

            sample_records = []
            sample_quota = args.sample_crops

            crop_buf, cls_buf, item_buf = [], [], []

            def flush_batch():
                nonlocal crop_buf, cls_buf, item_buf, n_clip_scored
                if not crop_buf:
                    return
                probs = verifier.score_batch(crop_buf, cls_buf)
                for it, cls, prob in zip(item_buf, cls_buf, probs):
                    n_clip_scored += 1
                    if prob >= 0.5 + args.clip_margin:
                        new_decisions[rl.item_key(it)] = (
                            "accept", "tier2_clip", f"prob_positive={prob:.3f}", it)
                    elif prob <= 0.5 - args.clip_margin:
                        new_decisions[rl.item_key(it)] = (
                            "reject", "tier2_clip", f"prob_positive={prob:.3f}", it)
                crop_buf, cls_buf, item_buf = [], [], []

            for image_key, items in by_img.items():
                img = cv2.imread(image_key)
                if img is None:
                    print(f"  [warn] could not read {image_key}, skipping its {len(items)} item(s).")
                    continue
                h, w = img.shape[:2]
                for it in items:
                    crop = crop_with_padding(img, it["box"], args.clip_crop_pad, w, h)
                    if crop is None:
                        continue
                    if sample_quota > 0 and random.random() < 0.15:
                        sample_records.append((image_key, it, crop))
                        sample_quota -= 1
                    if args.enable_clip:
                        crop_buf.append(crop)
                        cls_buf.append(it["cls"])
                        item_buf.append(it)
                        if len(crop_buf) >= args.clip_batch_size:
                            flush_batch()
            if args.enable_clip:
                flush_batch()

            if sample_records:
                sample_dir.mkdir(parents=True, exist_ok=True)
                # Score the sampled crops too (even if --enable-clip was off,
                # e.g. a --sample-crops --dry-run check-before-you-trust-it run) --
                # `verifier` is always constructed above whenever t2_targets is
                # non-empty, regardless of which of the two flags got us here.
                probs = verifier.score_batch(
                    [c for _, _, c in sample_records], [it["cls"] for _, it, _ in sample_records])
                for (image_key, it, crop), prob in zip(sample_records, probs):
                    tag = "ACCEPT" if prob >= 0.5 + args.clip_margin else (
                        "REJECT" if prob <= 0.5 - args.clip_margin else "unsure")
                    fname = f"{tag}_{prob:.2f}_{it['cls']}_{Path(image_key).stem}_{int(it['box'][0])}x{int(it['box'][1])}.jpg"
                    crop.save(sample_dir / fname)
                print(f"[sample-crops] wrote {len(sample_records)} crop(s) -> {sample_dir}")

    # --- Report -------------------------------------------------------
    by_tier_cls = collections.Counter()
    for decision, tier, why, it in new_decisions.values():
        by_tier_cls[(tier, decision, it["cls"])] += 1

    print("\n" + "=" * 70)
    print(f"Queue: {len(all_items_full)} total boxes across {len(by_image_full)} images")
    print(f"Scope (--source/--cls/--limit): {len(candidates)} boxes "
          f"({already_decided_n} already had a decision, skipped)")
    print(f"Forced manual (always_review_class): {forced_manual}")
    print(f"Tier 2 CLIP-scored: {n_clip_scored}")
    print("-" * 70)
    total_new = len(new_decisions)
    for (tier, decision, cls), n in sorted(by_tier_cls.items()):
        print(f"  {tier:12s} {decision:8s} {cls:15s} {n}")
    print("-" * 70)
    still_pending = len(pending) - total_new - forced_manual
    print(f"NEW decisions this run: {total_new}")
    print(f"Still pending for manual review: {still_pending}")
    print("=" * 70)

    if args.dry_run:
        print("\n[dry-run] no files were touched. Re-run without --dry-run to apply.")
        return

    if not new_decisions:
        print("\nNothing to write.")
        return

    touched_images = set()
    for key, (decision, tier, why, it) in new_decisions.items():
        entry = {
            "decision": decision,
            "image": it["image"],
            "cls": it["cls"],
            "box": it["box"],
            "orig_box": list(it["_orig_box"]),
            "auto_filter_tier": tier,      # decorative -- harmless extra keys,
            "auto_filter_reason": why,     # dropped the next time a human
        }                                  # edits this box via review_labels.py
        progress[key] = entry
        touched_images.add(it["image"])

    for image_key in touched_images:
        write_reviewed_labels_for_image(image_key, by_image_full, progress, reviewed_root)
        recompute_completed_for_image(image_key, by_image_full, progress, completed_set)

    rl.save_json(progress_path, progress)
    rl.save_completed(completed_path, completed_set)
    print(f"\nWrote {total_new} decision(s) across {len(touched_images)} image(s).")
    print("Open review_labels.py (optionally --qa to spot-check the newly-completed "
          "images) -- every auto-decided box is a normal, fully-reversible entry.")


if __name__ == "__main__":
    main()