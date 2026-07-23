"""
video_worker.py — Background FPV status-poll thread
=======================================================
The live video frames themselves no longer cross into Python at all --
VideoLink paints them directly into a native window from its own C++
capture thread (see FPVWidget.attach()). This worker's only remaining job
is polling VideoLink's cheap, atomic status fields (connected / fps /
device name) at a steady rate off the Tk thread, so the UI pump never has
to call into VideoLink directly.

Deliberately does NOT call get_latest_frame() any more: that call clones
a full frame under a mutex shared with the capture thread, which is
wasted work (and needless lock contention with the thing painting the
video) now that nothing here does anything with the pixels. If you need
frame data in Python for something else -- e.g. the post-flight
recording / YOLO post-processing pipeline -- call
video_link.get_latest_frame() directly from wherever that pipeline
lives; don't route it through this worker.
"""

import threading
import time


class VideoWorker:
    def __init__(self, video_link, poll_hz: int = 15):
        self._link = video_link
        self._interval = 1.0 / poll_hz
        self._lock = threading.Lock()
        self._latest_fps = 0.0
        self._device_name = ""
        self._running = False
        self._thread = None
        self.is_connected = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()

    def _loop(self):
        while self._running:
            connected = self._link.is_connected()
            self.is_connected = connected
            if connected:
                fps = self._link.get_measured_fps()
                name = self._link.get_device_name()
                with self._lock:
                    self._latest_fps = fps
                    self._device_name = name
            time.sleep(self._interval)

    def get_status(self):
        """Non-blocking. Returns (connected, fps, device_name)."""
        with self._lock:
            return self.is_connected, self._latest_fps, self._device_name