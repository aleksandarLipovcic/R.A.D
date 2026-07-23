"""
FPVWidget.py — Live FPV video panel
=====================================
Displays the freshest frame from VideoWorker. Swaps the image onto the
same canvas item every frame (canvas.itemconfig) instead of recreating
canvas items -- recreating items per-frame is what usually causes visible
stutter in Tk video panels.
"""

import tkinter as tk
from PIL import Image, ImageTk


class FPVWidget(tk.Frame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg="#000000", **kwargs)

        self._canvas = tk.Canvas(self, bg="#000000", highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)

        self._img_item = None
        self._photo = None   # keep a live reference -- Tk drops unreferenced PhotoImages

        self._status_item = self._canvas.create_text(
            10, 10, anchor="nw", text="FPV: searching for capture device...",
            fill="#ffaa00", font=("Consolas", 9), tags="status")

        self._fps_item = self._canvas.create_text(
            0, 0, anchor="ne", text="", fill="#00ff88",
            font=("Consolas", 9, "bold"), tags="fps")

        self._canvas.bind("<Configure>", self._on_resize)

    def _on_resize(self, event):
        self._canvas.coords(self._fps_item, event.width - 8, 8)

    def update_fpv(self, frame, fps: float, device_name: str = "") -> None:
        """frame: (H,W,3) BGR uint8 numpy array, or None if not connected yet."""
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        if cw < 2 or ch < 2:
            return

        if frame is None:
            self._canvas.itemconfig(self._status_item,
                                    text="FPV: searching for capture device...")
            self._canvas.tag_raise("status")
            self._canvas.itemconfig(self._fps_item, text="")
            return

        h, w = frame.shape[:2]
        scale = min(cw / w, ch / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))

        img = Image.fromarray(frame[:, :, ::-1])   # BGR -> RGB view, no copy
        img = img.resize((new_w, new_h), Image.BILINEAR)
        self._photo = ImageTk.PhotoImage(img)

        x, y = (cw - new_w) // 2, (ch - new_h) // 2

        if self._img_item is None:
            self._img_item = self._canvas.create_image(
                x, y, anchor="nw", image=self._photo, tags="frame")
        else:
            self._canvas.itemconfig(self._img_item, image=self._photo)
            self._canvas.coords(self._img_item, x, y)

        self._canvas.itemconfig(self._status_item, text="")
        fps_txt = f"{device_name}  {fps:4.1f} fps" if device_name else f"{fps:4.1f} fps"
        self._canvas.itemconfig(self._fps_item, text=fps_txt)
        self._canvas.tag_raise("fps")