"""
FPVWidget.py — Live FPV video panel (native-rendered)
========================================================
The live video feed itself is no longer drawn by this widget at all --
VideoLink paints decoded frames directly into a native Win32 child window
from its C++ capture thread (see VideoLink::attachToWindow /
DroneBackend.VideoLink.attach_to_window()). Python's only job here is:

  1. Own a plain Tk Frame ("host") whose HWND (via winfo_id()) is handed
     to VideoLink once at startup -- attach() does this.
  2. Forward <Configure> events on that host frame to
     VideoLink.resize_window() so the native surface tracks panel
     resizing/dragging.
  3. Show a lightweight status/FPS bar using ordinary Tk labels. This is
     cheap (a couple of label.config() calls per tick) and, critically,
     never touches per-frame pixel data -- that was the whole point of
     moving display off the Python/Tk path.

There is no more update_fpv(frame, ...) that takes pixel data: this
widget never sees a frame. Feed it periodic status only, via
update_status(connected, fps, device_name).
"""

import tkinter as tk


class FPVWidget(tk.Frame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg="#000000", **kwargs)

        self._video_link = None
        self._attached = False
        self._last_size = (0, 0)

        # ── Status bar ────────────────────────────────────────────────────
        # Deliberately a real Tk strip *above* the video area, not an
        # overlay drawn on top of it. The native child window is opaque
        # and is always composited above ordinary Tk canvas/label content
        # in the same screen region, so text placed "on top" of the video
        # area would never actually be visible once VideoLink starts
        # painting into it.
        self._status_bar = tk.Frame(self, bg="#000000")
        self._status_bar.pack(side="top", fill="x")

        self._status_lbl = tk.Label(
            self._status_bar, text="FPV: searching for capture device...",
            fg="#ffaa00", bg="#000000", font=("Consolas", 9), anchor="w",
        )
        self._status_lbl.pack(side="left", padx=6, pady=2)

        self._fps_lbl = tk.Label(
            self._status_bar, text="",
            fg="#00ff88", bg="#000000", font=("Consolas", 9, "bold"), anchor="e",
        )
        self._fps_lbl.pack(side="right", padx=6, pady=2)

        # ── Native video host ────────────────────────────────────────────
        # This frame's HWND is what gets handed to VideoLink via attach().
        # Nothing is ever drawn into it from the Python side -- it exists
        # purely to give the native render window a parent and a place in
        # the panel's layout.
        self._video_host = tk.Frame(self, bg="#000000")
        self._video_host.pack(side="top", fill="both", expand=True)
        self._video_host.bind("<Configure>", self._on_host_resize)

    # =========================================================================
    # Attach / resize / detach
    # =========================================================================

    def attach(self, video_link) -> None:
        """
        Hand the host frame's native window handle to VideoLink so the C++
        capture thread can start painting directly into it.

        Safe to call right after the widget is constructed -- winfo_id()
        forces the underlying platform window to be created if it hasn't
        been already, it does not require the window to be mapped/visible
        on screen first. Safe to call even before a capture device is
        connected: VideoLink simply won't paint anything until frames
        start arriving.
        """
        self._video_link = video_link
        self._video_host.update_idletasks()
        hwnd = self._video_host.winfo_id()
        w = max(1, self._video_host.winfo_width())
        h = max(1, self._video_host.winfo_height())
        video_link.attach_to_window(hwnd, 0, 0, w, h)
        self._attached = True
        self._last_size = (w, h)

    def detach(self) -> None:
        """Destroy the native render window. Call on shutdown."""
        if self._video_link is not None and self._attached:
            try:
                self._video_link.detach_window()
            except Exception:
                pass
        self._attached = False

    def _on_host_resize(self, event) -> None:
        if not self._attached or self._video_link is None:
            return
        w, h = event.width, event.height
        if (w, h) == self._last_size:
            return
        self._last_size = (w, h)
        try:
            self._video_link.resize_window(w, h)
        except Exception:
            pass

    # =========================================================================
    # Status bar — cheap, no pixel data
    # =========================================================================

    def update_status(self, connected: bool, fps: float, device_name: str = "") -> None:
        if not connected:
            self._status_lbl.config(text="FPV: searching for capture device...")
            self._fps_lbl.config(text="")
            return

        self._status_lbl.config(text="")
        fps_txt = f"{device_name}  {fps:4.1f} fps" if device_name else f"{fps:4.1f} fps"
        self._fps_lbl.config(text=fps_txt)