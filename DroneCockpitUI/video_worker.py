"""
video_worker.py — Background FPV capture-poll thread
======================================================
VideoLink's capture loop already runs at the device's native rate inside
C++, on its own thread. This worker's only job is bringing the freshest
frame across the pybind11 boundary at a rate the UI can consume, so the Tk
main thread never blocks on a capture call.
"""

import threading
import time


class VideoWorker:
    def __init__(self, video_link, poll_hz: int = 60):
        self._link = video_link
        self._interval = 1.0 / poll_hz
        self._lock = threading.Lock()
        self._latest_frame = None      # (H,W,3) BGR numpy array, or None
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
                frame = self._link.get_latest_frame()
                if frame is not None and frame.size > 0:
                    with self._lock:
                        self._latest_frame = frame
                        self._latest_fps = self._link.get_measured_fps()
                        self._device_name = self._link.get_device_name()
            time.sleep(self._interval)

    def get_frame(self):
        """Non-blocking. Returns (frame_or_None, fps)."""
        with self._lock:
            return self._latest_frame, self._latest_fps

    def get_device_name(self) -> str:
        with self._lock:
            return self._device_name