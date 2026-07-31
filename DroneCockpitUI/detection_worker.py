"""
detection_worker.py — Background detection status/record-poll thread
=========================================================================
Same role as video_worker.py, one level up: DetectionLink does all the
actual work (frame grab, inference, georeferencing, screenshot saving)
on its own C++ thread. This worker's only job is polling DetectionLink's
cheap getters off the Tk thread at a low, steady rate, and pulling new
DetectionRecords incrementally so DetectionMapWidget never has to
re-render its whole pin list every tick.

Deliberately does NOT touch frames, does NOT call anything that runs
inference, and does NOT write to disk. Every call this worker makes into
DetectionLink is a fast, mutex-protected read of already-computed data
-- the same "route heavy work through C++, poll cheap results from
Python" split as VideoWorker/VideoLink.
"""

import threading
import time


class DetectionWorker:
    def __init__(self, detection_link, poll_hz: int = 4):
        self._link = detection_link
        self._interval = 1.0 / poll_hz
        self._lock = threading.Lock()

        self._running = False
        self._thread = None

        self._detection_count = 0
        self._last_pass_duration_ms = 0.0
        self._last_pass_timestamp_ms = 0
        self._last_seen_id = 0
        self._new_records = []   # records pulled since the last get_new_records() call

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
            count = self._link.get_detection_count()
            duration_ms = self._link.get_last_pass_duration_ms()
            timestamp_ms = self._link.get_last_pass_timestamp_ms()

            # get_records_since() is still a mutex-protected copy, same
            # cost class as get_detection_count() -- these are small
            # structs (a few floats/strings), not frames, so pulling this
            # every poll tick is fine at 4-10Hz.
            fresh = self._link.get_records_since(self._last_seen_id)

            with self._lock:
                self._detection_count = count
                self._last_pass_duration_ms = duration_ms
                self._last_pass_timestamp_ms = timestamp_ms
                if fresh:
                    self._new_records.extend(fresh)
                    self._last_seen_id = fresh[-1].id

            time.sleep(self._interval)

    def get_status(self):
        """Non-blocking. Returns (detection_count, last_pass_ms, last_pass_timestamp_ms)."""
        with self._lock:
            return self._detection_count, self._last_pass_duration_ms, self._last_pass_timestamp_ms

    def get_new_records(self):
        """
        Non-blocking. Returns and clears the list of DetectionRecords
        collected since the last call -- call this from the Tk mainloop
        (e.g. via widget.after()) to drive incremental pin/list updates.
        """
        with self._lock:
            records, self._new_records = self._new_records, []
            return records