"""
label_editor.py -- Frame-by-frame review / correction tool for Project R.A.D
video reference labels (YOLO format, unified 5-class taxonomy).

    pip install opencv-python numpy
    python label_editor.py --video rad_fpv_cropped_part0.mp4 --labels labels/part0

Labels: one YOLO txt per frame, named <prefix>_<NNNNNN>.txt with 0-based frame
numbers (part0_000000.txt = first frame). Line format "cls xc yc w h"
(normalized). The tool reads and writes exactly this format, so the edited
folder is directly your ground-truth reference. Progress (which frames you've
marked reviewed) is kept in <labels>/../<prefix>_reviewed.json.

MOUSE
  left-drag on empty area ....... draw a new box (current class)
  left-click a box .............. select it
  left-drag inside selected box . move it
  left-drag a corner / edge ..... resize the selected box
  right-click a box ............. delete it

KEYS
  d / a ............. next / previous frame        (auto-saves)
  e / q ............. +10 / -10 frames
  f / r ............. +60 / -60 frames (1 s)
  g ................. go to frame (type number in console)
  space ............. play / pause
  1..5 .............. set class (selected box, and class for new boxes)
                      1 person 2 car 3 large_vehicle 4 motorcycle 5 other_vehicle
  x / Delete ........ delete selected box
  c ................. copy ALL boxes from previous frame into this frame (replace)
  p ................. push this frame's boxes onto the next frame and step there
  t ................. track this frame's boxes into the next frame (OpenCV CSRT,
                      needs opencv-contrib-python; falls back to plain copy)
  arrows (i/j/k/l) .. nudge selected box 1 px (hold shift for... use I/J/K/L = 5 px)
  [ / ] ............. shrink / grow selected box by 2 px on each side
  u ................. undo last change on this frame
  v ................. mark frame reviewed / unreviewed
  n ................. jump to next NOT-reviewed frame
  h ................. hide / show boxes (to see what is underneath)
  z ................. toggle 2x zoom window around the mouse
  s ................. save now
  ESC ............... save and quit
"""
import argparse, json, re, subprocess, sys, copy
from pathlib import Path

import cv2
import numpy as np

CLASSES = ["person", "car", "large_vehicle", "motorcycle", "other_vehicle"]
COLORS = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (0, 200, 255), (255, 0, 255)]  # BGR
HANDLE = 7
WIN = "R.A.D label editor"


class Video:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            sys.exit(f"cannot open {path}")
        self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30
        self.pos = -1  # index of the frame last returned by read()
        self.cache = {}

    def get(self, i):
        if i in self.cache:
            return self.cache[i]
        if i != self.pos + 1:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = self.cap.read()
        if not ok:
            return None
        self.pos = i
        self.cache[i] = f
        if len(self.cache) > 90:
            self.cache.pop(next(iter(self.cache)))
        return f


class Editor:
    def __init__(self, a):
        self.v = Video(a.video)
        self.dir = Path(a.labels); self.dir.mkdir(parents=True, exist_ok=True)
        files = sorted(self.dir.glob("*.txt"))
        m = re.match(r"(.*?)_?(\d+)$", files[0].stem) if files else None
        self.prefix = a.prefix or (m.group(1) if m else Path(a.video).stem)
        self.digits = len(m.group(2)) if m else 6
        self.rev_path = self.dir.parent / f"{self.prefix}_reviewed.json"
        self.reviewed = set(json.loads(self.rev_path.read_text())) if self.rev_path.exists() else set()
        self.F = a.start
        self.cls = 1
        self.sel = None
        self.drag = None
        self.hide = False
        self.zoom = False
        self.play = False
        self.mouse = (0, 0)
        self.undo = []
        f0 = self.v.get(0); self.H, self.W = f0.shape[:2]
        self.load()

    # ---------- label io ----------
    def path(self, F):
        return self.dir / f"{self.prefix}_{F:0{self.digits}d}.txt"

    def read(self, F):
        p = self.path(F); out = []
        if p.exists():
            for line in p.read_text().split("\n"):
                t = line.split()
                if len(t) < 5: continue
                c = int(float(t[0])); xc, yc, w, h = map(float, t[1:5])
                out.append([c, (xc - w / 2) * self.W, (yc - h / 2) * self.H,
                            (xc + w / 2) * self.W, (yc + h / 2) * self.H])
        return out

    def load(self):
        self.boxes = self.read(self.F); self.sel = None; self.undo = []; self.dirty = False

    def save(self):
        if not self.dirty: return
        lines = []
        for c, x1, y1, x2, y2 in self.boxes:
            x1, x2 = sorted((max(0, min(self.W, x1)), max(0, min(self.W, x2))))
            y1, y2 = sorted((max(0, min(self.H, y1)), max(0, min(self.H, y2))))
            if x2 - x1 < 2 or y2 - y1 < 2: continue
            lines.append(f"{c} {(x1+x2)/2/self.W:.6f} {(y1+y2)/2/self.H:.6f} "
                         f"{(x2-x1)/self.W:.6f} {(y2-y1)/self.H:.6f}")
        self.path(self.F).write_text("\n".join(lines) + ("\n" if lines else ""))
        self.dirty = False

    def save_rev(self):
        self.rev_path.write_text(json.dumps(sorted(self.reviewed)))

    def goto(self, F):
        F = max(0, min(self.v.n - 1, F))
        if F == self.F: return
        self.save(); self.F = F; self.load()

    def change(self):
        self.undo.append(copy.deepcopy(self.boxes)); self.dirty = True

    # ---------- geometry ----------
    def hit(self, x, y):
        """-> (index, mode) mode in move / corner / edge names"""
        best = None
        for i, (c, x1, y1, x2, y2) in enumerate(self.boxes):
            if not (x1 - HANDLE <= x <= x2 + HANDLE and y1 - HANDLE <= y <= y2 + HANDLE): continue
            l, r = abs(x - x1) <= HANDLE, abs(x - x2) <= HANDLE
            t, b = abs(y - y1) <= HANDLE, abs(y - y2) <= HANDLE
            mode = ("l" if l else "r" if r else "") + ("t" if t else "b" if b else "") or "move"
            area = (x2 - x1) * (y2 - y1)
            if best is None or area < best[2]: best = (i, mode, area)
        return (best[0], best[1]) if best else (None, None)

    # ---------- mouse ----------
    def on_mouse(self, ev, x, y, flags, _):
        self.mouse = (x, y)
        if ev == cv2.EVENT_LBUTTONDOWN:
            i, mode = self.hit(x, y)
            if i is not None:
                self.sel = i; self.change(); self.drag = (mode, x, y, list(self.boxes[i]))
            else:
                self.change(); self.boxes.append([self.cls, x, y, x, y])
                self.sel = len(self.boxes) - 1; self.drag = ("rb", x, y, list(self.boxes[-1]))
        elif ev == cv2.EVENT_MOUSEMOVE and self.drag:
            mode, x0, y0, b = self.drag; dx, dy = x - x0, y - y0; nb = list(b)
            if mode == "move": nb[1] += dx; nb[3] += dx; nb[2] += dy; nb[4] += dy
            if "l" in mode: nb[1] = b[1] + dx
            if "r" in mode: nb[3] = b[3] + dx
            if "t" in mode: nb[2] = b[2] + dy
            if "b" in mode: nb[4] = b[4] + dy
            self.boxes[self.sel] = nb
        elif ev == cv2.EVENT_LBUTTONUP and self.drag:
            c, x1, y1, x2, y2 = self.boxes[self.sel]
            x1, x2 = sorted((x1, x2)); y1, y2 = sorted((y1, y2))
            if x2 - x1 < 3 or y2 - y1 < 3:
                self.boxes.pop(self.sel); self.sel = None
            else:
                self.boxes[self.sel] = [c, x1, y1, x2, y2]
            self.drag = None
        elif ev == cv2.EVENT_RBUTTONDOWN:
            i, _ = self.hit(x, y)
            if i is not None:
                self.change(); self.boxes.pop(i); self.sel = None

    # ---------- actions ----------
    def track_next(self):
        if self.F >= self.v.n - 1: return
        f0 = self.v.get(self.F); f1 = self.v.get(self.F + 1); new = []
        for c, x1, y1, x2, y2 in self.boxes:
            nb = [c, x1, y1, x2, y2]
            try:
                t = cv2.TrackerCSRT.create() if hasattr(cv2, "TrackerCSRT") else cv2.TrackerCSRT_create()
                t.init(f0, (int(x1), int(y1), max(2, int(x2 - x1)), max(2, int(y2 - y1))))
                ok, (x, y, w, h) = t.update(f1)
                if ok: nb = [c, x, y, x + w, y + h]
            except Exception:
                pass
            new.append(nb)
        self.save(); self.F += 1; self.boxes = new; self.undo = []; self.sel = None; self.dirty = True

    def draw(self):
        f = self.v.get(self.F)
        img = f.copy() if f is not None else np.zeros((self.H, self.W, 3), np.uint8)
        if not self.hide:
            for i, (c, x1, y1, x2, y2) in enumerate(self.boxes):
                col = COLORS[c % 5]; th = 3 if i == self.sel else 1
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), col, th)
                cv2.putText(img, CLASSES[c], (int(x1), max(12, int(y1) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
                cv2.putText(img, CLASSES[c], (int(x1), max(12, int(y1) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                if i == self.sel:
                    for px, py in ((x1, y1), (x2, y1), (x1, y2), (x2, y2)):
                        cv2.rectangle(img, (int(px) - 3, int(py) - 3), (int(px) + 3, int(py) + 3), (255, 255, 255), -1)
        rv = self.F in self.reviewed
        bar = (f"frame {self.F}/{self.v.n-1}  t={self.F/self.v.fps:.2f}s  boxes={len(self.boxes)}  "
               f"new-box class: {self.cls+1} {CLASSES[self.cls]}  "
               f"{'REVIEWED' if rv else 'not reviewed'}  ({len(self.reviewed)}/{self.v.n} done)"
               f"{'  *unsaved' if self.dirty else ''}")
        cv2.rectangle(img, (0, self.H - 26), (self.W, self.H), (0, 0, 0), -1)
        cv2.putText(img, bar, (6, self.H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if rv else (255, 255, 255), 1, cv2.LINE_AA)
        if self.zoom:
            mx, my = self.mouse; r = 60
            x0, y0 = max(0, min(self.W - 2 * r, mx - r)), max(0, min(self.H - 2 * r, my - r))
            z = cv2.resize(img[y0:y0 + 2 * r, x0:x0 + 2 * r], (4 * r * 2, 4 * r * 2), interpolation=cv2.INTER_NEAREST)
            zx = 10 if mx > self.W / 2 else self.W - z.shape[1] - 10
            img[10:10 + z.shape[0], zx:zx + z.shape[1]] = z
            cv2.rectangle(img, (zx, 10), (zx + z.shape[1], 10 + z.shape[0]), (255, 255, 255), 1)
        return img

    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WIN, self.W, self.H)
        cv2.setMouseCallback(WIN, self.on_mouse)
        print(__doc__)
        while True:
            cv2.imshow(WIN, self.draw())
            k = cv2.waitKeyEx(int(1000 / self.v.fps) if self.play else 30)
            if self.play:
                if self.F >= self.v.n - 1: self.play = False
                else: self.goto(self.F + 1)
            if k == -1: continue
            ch = chr(k & 0xFF) if (k & 0xFF) < 128 else ""
            if k == 27: break
            elif ch == "d" or k in (2555904, 65363): self.goto(self.F + 1)
            elif ch == "a" or k in (2424832, 65361): self.goto(self.F - 1)
            elif ch == "e": self.goto(self.F + 10)
            elif ch == "q": self.goto(self.F - 10)
            elif ch == "f": self.goto(self.F + 60)
            elif ch == "r": self.goto(self.F - 60)
            elif ch == " ": self.play = not self.play
            elif ch == "g":
                try: self.goto(int(input("go to frame: ")))
                except ValueError: pass
            elif ch in "12345" and ch:
                self.cls = int(ch) - 1
                if self.sel is not None:
                    self.change(); self.boxes[self.sel][0] = self.cls
            elif (ch == "x" or k in (3014656, 65535)) and self.sel is not None:
                self.change(); self.boxes.pop(self.sel); self.sel = None
            elif ch == "c" and self.F > 0:
                self.change(); self.boxes = self.read(self.F - 1)
            elif ch == "p" and self.F < self.v.n - 1:
                b = copy.deepcopy(self.boxes); self.save(); self.F += 1
                self.boxes = b; self.undo = []; self.sel = None; self.dirty = True
            elif ch == "t": self.track_next()
            elif ch in "ijklIJKL" and ch and self.sel is not None:
                s = 5 if ch.isupper() else 1
                dx, dy = {"j": (-s, 0), "l": (s, 0), "i": (0, -s), "k": (0, s)}[ch.lower()]
                self.change(); b = self.boxes[self.sel]; b[1] += dx; b[3] += dx; b[2] += dy; b[4] += dy
            elif ch in "[]" and ch and self.sel is not None:
                s = 2 if ch == "]" else -2
                self.change(); b = self.boxes[self.sel]; b[1] -= s; b[2] -= s; b[3] += s; b[4] += s
            elif ch == "u" and self.undo:
                self.boxes = self.undo.pop(); self.sel = None; self.dirty = True
            elif ch == "v":
                self.reviewed ^= {self.F}; self.save_rev()
            elif ch == "n":
                nxt = next((i for i in range(self.F + 1, self.v.n) if i not in self.reviewed), None)
                if nxt is not None: self.goto(nxt)
            elif ch == "h": self.hide = not self.hide
            elif ch == "z": self.zoom = not self.zoom
            elif ch == "s": self.save()
            if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1: break
        self.save(); self.save_rev(); cv2.destroyAllWindows()
        print(f"saved. reviewed {len(self.reviewed)}/{self.v.n} frames.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="R.A.D video label editor")
    p.add_argument("--video", required=True)
    p.add_argument("--labels", required=True, help="folder of per-frame YOLO txt files")
    p.add_argument("--prefix", default=None, help="label filename prefix (auto-detected)")
    p.add_argument("--start", type=int, default=0)
    Editor(p.parse_args()).run()
