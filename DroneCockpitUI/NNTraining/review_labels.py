"""
review_labels.py -- Interactive, resumable review tool for the queue
generate_pseudo_labels.py produces (datasets/pseudo_labels_review_queue.
json). Shows one image at a time with its queued boxes overlaid; you
click to accept/reject each one, drag to draw a box the queue missed
entirely, and move on. Progress saves as you go -- close it anytime,
rerun later, it picks up where you left off.

V3 NOTE: the only change from V2 is cosmetic -- queue items now carry a
"votes" dict (all four cross_reference_gaps.py models' confidence-or-
null) and "vote_count" instead of the old yolo_conf/rfdetr_conf/bucket
fields, so the on-screen label per box now reads e.g.
"car v3/4 yv0.61/rf0.55/yc0.48 [pending]" instead of the old
"car y0.61/r0.55 [pending]". Everything else -- controls, resumability,
the separate pseudo_labels_reviewed/ output tree -- is unchanged.

V4 NOTE: adds an 'o' toggle that overlays the image's EXISTING real-
dataset labels (whatever is already sitting at the normal YOLO label
path for that image, read fresh each time -- not the queue, not the
reviewed tree). Purely a reference layer: not clickable, never edited,
never saved anywhere by this tool. The point is to stop you from
manually drawing a box for something that's already labeled (e.g. it's
outside the --source/--cls filter you're currently working through, or
it belongs to a class the queue never touches). Off by default so it
doesn't clutter images that don't need it; press 'h' to see it in the
legend.

WHY A SEPARATE OUTPUT TREE (see generate_pseudo_labels.py's module
docstring for the full reasoning): this writes to datasets/
pseudo_labels_reviewed/<SOURCE>/..., NOT datasets/pseudo_labels/ (the
auto-accept tree). generate_pseudo_labels.py overwrites pseudo_labels/
wholesale on every rerun -- if this tool wrote there too, a later rerun
(e.g. after another cross_reference_gaps.py pass) would silently erase
your manual review work. generate_pseudo_labels.py's --apply merges
BOTH trees into the real dataset, so this separation costs nothing at
apply time.

CONTROLS (shown in-window too -- press 'h' any time):
  Left-click on a box       cycle its state: pending -> accept ->
                             reject -> pending
  Left-click-drag on empty  draw a new box. On release, press a digit
  space, then release        key (0-4, see class legend in-window) to
                             assign its class -- it's added as an
                             accepted box. Press Esc before the digit
                             to cancel the draw instead.
  n / Space                 next image (saves current image first)
  p                          previous image
  a                          accept all currently-pending boxes in view
  r                          reject all currently-pending boxes in view
  u                          undo the last manually-drawn box (this
                             image, this session only)
  o                          toggle the existing/original dataset
                             labels for this image as a reference
                             overlay (not clickable, never saved)
  s                          save progress now (also autosaves on
                             n/p/quit)
  q / Esc (at image level)  save and quit
  h                          toggle the help/legend overlay

SCOPE: only images that have at least one queue entry are shown -- the
much larger auto-accepted tier is never touched here. Use --source /
--cls to focus a session (e.g. work through UAVDT person first, come
back for other_vehicle later) -- this is built to be resumable across
many sessions, not a one-sitting task.

USAGE:
    python review_labels.py                          # everything pending
    python review_labels.py --source UAVDT            # one source
    python review_labels.py --source UAVDT --cls person
    python review_labels.py --show-all                # revisit decided
                                                        # images too
"""

import argparse
import json
from pathlib import Path

import cv2

import prepare_datasets
from class_map import UNIFIED_CLASSES
import generate_pseudo_labels as gpl

REVIEWED_DIRNAME = "pseudo_labels_reviewed"
PROGRESS_FILENAME = "review_progress.json"

COLOR_PENDING = (0, 165, 255)   # orange
COLOR_ACCEPT = (0, 220, 0)      # green
COLOR_REJECT = (0, 0, 220)      # red
COLOR_MANUAL_DRAW = (255, 200, 0)  # cyan-ish, box currently being drawn
COLOR_ORIGINAL = (255, 0, 255)  # magenta, existing real-dataset labels (reference only)
COLOR_TEXT_BG = (30, 30, 30)

DISPLAY_MAX_W = 1280
DISPLAY_MAX_H = 900

CLASS_KEYS = {str(i): name for i, name in enumerate(UNIFIED_CLASSES)}


def item_key(item: dict) -> str:
    """Stable id for a queue item -- (image, cls, rounded box) is
    deterministic across runs since it's re-read from the same JSON
    each time, no floats get recomputed."""
    box_str = ",".join(f"{v:.1f}" for v in item["box"])
    return f"{item['image']}|{item['cls']}|{box_str}"


def load_progress(path: Path) -> dict:
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return {}


def save_progress(path: Path, progress: dict):
    with open(path, "w") as f:
        json.dump(progress, f, indent=2, default=str)


def load_original_labels(label_path: Path, img_w: int, img_h: int) -> list[dict]:
    """Reads whatever YOLO-format label file already exists at the real
    dataset path for this image (if any) and returns boxes in the same
    {"cls": str, "box": [x1,y1,x2,y2]} pixel-coordinate shape used
    everywhere else here. Purely for reference display -- this tool
    never writes to this path, never edits these boxes, and reloads
    them fresh every time an image is opened (so it always reflects
    whatever's actually in the dataset right now)."""
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


class ImageSession:
    """Holds the mutable state for the image currently on screen."""

    def __init__(self, image_key: str, queue_items: list[dict], progress: dict,
                 original_boxes: list[dict] | None = None):
        self.image_key = image_key
        self.queue_items = queue_items  # original queue entries for this image
        # decision state per queue item, keyed by item_key -- "pending"/"accept"/"reject"
        self.decisions = {}
        for it in queue_items:
            k = item_key(it)
            self.decisions[k] = progress.get(k, {}).get("decision", "pending")
        # manually drawn boxes this session: list of {"cls": str, "box": [x1,y1,x2,y2]}
        # restored from progress if this image was visited before.
        self.manual_boxes = list(progress.get(f"__manual__{image_key}", []))
        self.drawing = False
        self.drag_start = None
        self.pending_manual_box = None  # box awaiting a class-key press
        # existing real-dataset labels for this image, reference-only, never edited
        # or persisted by this tool -- see load_original_labels().
        self.original_boxes = original_boxes or []

    def cycle(self, k: str):
        order = ["pending", "accept", "reject"]
        self.decisions[k] = order[(order.index(self.decisions[k]) + 1) % 3]

    def set_all_pending(self, new_state: str):
        for k, v in self.decisions.items():
            if v == "pending":
                self.decisions[k] = new_state

    def accepted_items(self) -> list[dict]:
        out = []
        for it in self.queue_items:
            if self.decisions[item_key(it)] == "accept":
                out.append({"cls": it["cls"], "box": it["box"]})
        out.extend(self.manual_boxes)
        return out


def scale_for_display(w: int, h: int) -> float:
    return min(DISPLAY_MAX_W / w, DISPLAY_MAX_H / h, 1.0)


def draw_frame(img, session: ImageSession, scale: float, show_help: bool, show_original: bool):
    disp = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))

    if show_original:
        for ob in session.original_boxes:
            x1, y1, x2, y2 = [int(v * scale) for v in ob["box"]]
            cv2.rectangle(disp, (x1, y1), (x2, y2), COLOR_ORIGINAL, 1)
            cv2.putText(disp, f"{ob['cls']} [orig]", (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_ORIGINAL, 1, cv2.LINE_AA)

    for it in session.queue_items:
        state = session.decisions[item_key(it)]
        color = {"pending": COLOR_PENDING, "accept": COLOR_ACCEPT, "reject": COLOR_REJECT}[state]
        x1, y1, x2, y2 = [int(v * scale) for v in it["box"]]
        cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)
        label = f"{it['cls']} {gpl.votes_str(it)} [{state}]"
        cv2.putText(disp, label, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)

    for mb in session.manual_boxes:
        x1, y1, x2, y2 = [int(v * scale) for v in mb["box"]]
        cv2.rectangle(disp, (x1, y1), (x2, y2), COLOR_MANUAL_DRAW, 2)
        cv2.putText(disp, f"{mb['cls']} [manual]", (x1, max(12, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_MANUAL_DRAW, 1, cv2.LINE_AA)

    if session.pending_manual_box:
        x1, y1, x2, y2 = [int(v * scale) for v in session.pending_manual_box]
        cv2.rectangle(disp, (x1, y1), (x2, y2), COLOR_MANUAL_DRAW, 2)
        cv2.putText(disp, "press 0-4 to assign class, Esc to cancel",
                    (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, COLOR_MANUAL_DRAW, 1, cv2.LINE_AA)

    n_pending = sum(1 for v in session.decisions.values() if v == "pending")
    n_accept = sum(1 for v in session.decisions.values() if v == "accept")
    n_reject = sum(1 for v in session.decisions.values() if v == "reject")
    status = (f"{Path(session.image_key).name}  |  "
              f"pending={n_pending} accept={n_accept} reject={n_reject} "
              f"manual={len(session.manual_boxes)} "
              f"orig={len(session.original_boxes)}{'*' if show_original else ' (hidden, press o)'}")
    cv2.rectangle(disp, (0, disp.shape[0] - 22), (disp.shape[1], disp.shape[0]), COLOR_TEXT_BG, -1)
    cv2.putText(disp, status, (6, disp.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)

    if show_help:
        legend = ["classes: " + ", ".join(f"{k}={v}" for k, v in CLASS_KEYS.items()),
                  "vote tags: yv=yolo_visdrone rf=rfdetr_visdrone yc=yolo_coco dn=dino",
                  "click box: cycle pending/accept/reject   drag empty: draw new",
                  "a=accept-all-pending  r=reject-all-pending  u=undo manual",
                  "o=toggle original/existing labels (magenta, reference only)",
                  "n/space=next  p=prev  s=save  q/Esc=save+quit  h=toggle help"]
        for i, line in enumerate(legend):
            y = 20 + i * 20
            cv2.rectangle(disp, (0, y - 15), (disp.shape[1], y + 5), COLOR_TEXT_BG, -1)
            cv2.putText(disp, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

    return disp


def make_mouse_callback(session: ImageSession, scale: float):
    def on_mouse(event, x, y, flags, param):
        ox, oy = x / scale, y / scale  # original image coords

        if event == cv2.EVENT_LBUTTONDOWN:
            hit = find_box_at(session, ox, oy)
            if hit is not None:
                session.cycle(hit)
            else:
                session.drawing = True
                session.drag_start = (ox, oy)

        elif event == cv2.EVENT_LBUTTONUP and session.drawing:
            session.drawing = False
            x1, y1 = session.drag_start
            x2, y2 = ox, oy
            if abs(x2 - x1) > 4 and abs(y2 - y1) > 4:
                box = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
                session.pending_manual_box = box
            session.drag_start = None

    return on_mouse


def find_box_at(session: ImageSession, x: float, y: float):
    # Note: only queue_items are hit-tested -- original_boxes are a
    # reference-only overlay and deliberately not clickable/editable.
    for it in session.queue_items:
        x1, y1, x2, y2 = it["box"]
        if x1 <= x <= x2 and y1 <= y <= y2:
            return item_key(it)
    return None


def write_reviewed_labels(session: ImageSession, reviewed_root: Path, source_name: str):
    """Overwrites this image's file under pseudo_labels_reviewed/ with the
    CURRENT full accepted set (queue-accepted + manual). Safe to fully
    overwrite (not append) because this tree is only ever written by this
    tool, for this image, from complete state each time -- no partial-
    write inconsistency risk like the cross-run auto-accept case."""
    accepted = session.accepted_items()
    img_path = Path(session.image_key)
    real_label_path = gpl.image_path_to_label_path(img_path)
    reviewed_label_path = gpl.mirror_under_pseudo_root(real_label_path, source_name, reviewed_root)

    if not accepted:
        if reviewed_label_path.exists():
            reviewed_label_path.unlink()
        return

    w, h = gpl.get_image_dims(session.image_key)
    lines = [gpl.box_to_yolo_line(a["cls"], a["box"], w, h) for a in accepted]
    reviewed_label_path.parent.mkdir(parents=True, exist_ok=True)
    reviewed_label_path.write_text("\n".join(lines) + "\n")


def flush_session(session: ImageSession, progress: dict, reviewed_root: Path,
                   source_name: str, progress_path: Path):
    for it in session.queue_items:
        k = item_key(it)
        progress[k] = {"decision": session.decisions[k], "image": session.image_key,
                        "cls": it["cls"]}
    progress[f"__manual__{session.image_key}"] = session.manual_boxes
    write_reviewed_labels(session, reviewed_root, source_name)
    save_progress(progress_path, progress)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default=None, help="Only review this source (e.g. UAVDT).")
    p.add_argument("--cls", default=None, help="Only review this class (e.g. person).")
    p.add_argument("--show-all", action="store_true",
                    help="Include images that are already fully decided "
                         "(default: only show images with pending items).")
    p.add_argument("--show-original", action="store_true",
                    help="Start with the existing/original-label overlay ('o') on "
                         "by default, instead of having to toggle it per image.")
    return p.parse_args()


def main():
    args = parse_args()
    datasets_dir = prepare_datasets.DATASETS_DIR
    queue_path = datasets_dir / "pseudo_labels_review_queue.json"
    progress_path = datasets_dir / PROGRESS_FILENAME
    reviewed_root = datasets_dir / REVIEWED_DIRNAME

    if not queue_path.exists():
        raise SystemExit(f"{queue_path} not found -- run generate_pseudo_labels.py first.")

    with open(queue_path) as f:
        all_items = json.load(f)
    progress = load_progress(progress_path)

    if args.source:
        all_items = [it for it in all_items if it["source"] == args.source]
    if args.cls:
        all_items = [it for it in all_items if it["cls"] == args.cls]

    by_image: dict = {}
    for it in all_items:
        by_image.setdefault(it["image"], []).append(it)

    image_keys = list(by_image.keys())
    if not args.show_all:
        def has_pending(image_key):
            return any(progress.get(item_key(it), {}).get("decision", "pending") == "pending"
                       for it in by_image[image_key])
        image_keys = [k for k in image_keys if has_pending(k)]

    if not image_keys:
        print("Nothing to review (try --show-all, or check --source/--cls filters).")
        return

    print(f"{len(image_keys)} images to review. Press 'h' in-window for controls.")

    idx = 0
    show_help = True
    show_original = args.show_original
    window = "review_labels"
    cv2.namedWindow(window)

    while 0 <= idx < len(image_keys):
        image_key = image_keys[idx]
        items = by_image[image_key]
        source_name = items[0]["source"]

        img = cv2.imread(image_key)
        if img is None:
            print(f"  [warn] could not read {image_key}, skipping.")
            idx += 1
            continue

        real_label_path = gpl.image_path_to_label_path(Path(image_key))
        original_boxes = load_original_labels(real_label_path, img.shape[1], img.shape[0])
        session = ImageSession(image_key, items, progress, original_boxes)

        scale = scale_for_display(img.shape[1], img.shape[0])
        cv2.setMouseCallback(window, make_mouse_callback(session, scale))

        advance = None
        while advance is None:
            frame = draw_frame(img, session, scale, show_help, show_original)
            cv2.imshow(window, frame)
            key = cv2.waitKey(20) & 0xFF

            if session.pending_manual_box is not None:
                if key == 27:  # Esc cancels the draw
                    session.pending_manual_box = None
                elif 0 <= key < 256 and chr(key) in CLASS_KEYS:
                    session.manual_boxes.append({
                        "cls": CLASS_KEYS[chr(key)], "box": session.pending_manual_box})
                    session.pending_manual_box = None
                continue  # swallow other keys while a box awaits classification

            if key in (ord('n'), ord(' ')):
                advance = 1
            elif key == ord('p'):
                advance = -1
            elif key == ord('a'):
                session.set_all_pending("accept")
            elif key == ord('r'):
                session.set_all_pending("reject")
            elif key == ord('u'):
                if session.manual_boxes:
                    session.manual_boxes.pop()
            elif key == ord('o'):
                show_original = not show_original
            elif key == ord('s'):
                flush_session(session, progress, reviewed_root, source_name, progress_path)
                print("  Progress saved.")
            elif key == ord('h'):
                show_help = not show_help
            elif key in (ord('q'), 27):
                flush_session(session, progress, reviewed_root, source_name, progress_path)
                cv2.destroyAllWindows()
                print("Saved. Resume any time by rerunning this script.")
                return

        flush_session(session, progress, reviewed_root, source_name, progress_path)
        idx += advance

    cv2.destroyAllWindows()
    print("All matching images reviewed. Progress saved.")


if __name__ == "__main__":
    main()