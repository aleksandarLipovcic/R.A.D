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
    # TEMPORARY perf diagnostic: print once every _DBG_PRINT_EVERY loop
    # iterations. This worker runs on its own Python thread, separate
    # from Tk's -- if avg_link_calls_ms is ever large, this thread is
    # holding the GIL for a meaningful chunk of its 250ms period (at
    # poll_hz=4), which competes with whatever DetectionMapWidget's
    # _poll_live_frame is doing on the Tk thread at the same time. Delete
    # this whole diagnostic (and the _dbg_* attrs) once the bottleneck is
    # found and fixed.
    _DBG_PRINT_EVERY = 20   # ~5s at poll_hz=4

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

        self._dbg_tick = 0
        self._dbg_link_calls_accum = 0.0
        self._dbg_loop_accum = 0.0
        self._dbg_fresh_records_accum = 0

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
            loop_start = time.monotonic()

            link_calls_start = time.monotonic()
            count = self._link.get_detection_count()
            duration_ms = self._link.get_last_pass_duration_ms()
            timestamp_ms = self._link.get_last_pass_timestamp_ms()

            # get_records_since() is still a mutex-protected copy, same
            # cost class as get_detection_count() -- these are small
            # structs (a few floats/strings), not frames, so pulling this
            # every poll tick is fine at 4-10Hz.
            fresh = self._link.get_records_since(self._last_seen_id)
            link_calls_ms = (time.monotonic() - link_calls_start) * 1000.0

            with self._lock:
                self._detection_count = count
                self._last_pass_duration_ms = duration_ms
                self._last_pass_timestamp_ms = timestamp_ms
                if fresh:
                    self._new_records.extend(fresh)
                    self._last_seen_id = fresh[-1].id

            self._dbg_link_calls_accum += link_calls_ms
            self._dbg_fresh_records_accum += len(fresh)
            self._dbg_loop_accum += (time.monotonic() - loop_start) * 1000.0
            self._dbg_tick += 1
            if self._dbg_tick >= self._DBG_PRINT_EVERY:
                n = self._dbg_tick
                print(
                    f"[DetectionWorker][perf] avg_link_calls={self._dbg_link_calls_accum / n:.2f}ms "
                    f"avg_loop_body={self._dbg_loop_accum / n:.2f}ms "
                    f"last_pass_duration={duration_ms:.1f}ms "
                    f"new_records={self._dbg_fresh_records_accum}"
                )
                self._dbg_tick = 0
                self._dbg_link_calls_accum = 0.0
                self._dbg_loop_accum = 0.0
                self._dbg_fresh_records_accum = 0

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