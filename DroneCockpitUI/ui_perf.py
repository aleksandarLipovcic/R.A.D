"""
ui_perf.py  —  opt-in UI performance monitor for the cockpit
=============================================================

Measures, on the machine the cockpit actually runs on, where the Tk main
thread spends its time and how long repaints are delayed. Off by default;
costs nothing unless enabled:

    set COCKPIT_PERF=1          (Windows cmd)
    python DroneCockpitUI.py

Every REPORT_S seconds it prints one block to the console:

    [PERF] 10 s: 498 ticks (49.8 Hz)  tick mean 3.1 ms  max 14.2 ms
           idle delay p50 0.4 ms  p95 3.0 ms  max 9.8 ms   (repaint latency)
           adi        25.0 Hz  mean 3.2 ms  max 8.1 ms
           ...

What to look for:
  • ticks well below 50 Hz, or tick max > ~20 ms  → something in the UI
    update is too slow (the per-widget lines say what);
  • idle delay p95 > ~20 ms → repaints (moving/resizing panels) are being
    starved; the cause is usually a slow widget or a burst of work.
"""

import os
import time
from collections import defaultdict

REPORT_S = 10.0
_PROBE_MS = 50


def enabled() -> bool:
    return os.environ.get("COCKPIT_PERF", "0") == "1"


class PerfMonitor:
    def __init__(self, root):
        self._root = root
        self._stats = defaultdict(list)
        self._idle = []
        self._t_report = time.monotonic()

    # ── instrumentation ──────────────────────────────────────────────────────
    def wrap(self, obj, method: str, key: str):
        """Replace obj.method with a timed version (instance attribute)."""
        fn = getattr(obj, method, None)
        if fn is None:
            return
        stats = self._stats[key]

        def timed(*a, **kw):
            t = time.perf_counter()
            try:
                return fn(*a, **kw)
            finally:
                stats.append(time.perf_counter() - t)
        setattr(obj, method, timed)

    def start(self):
        self._root.after(_PROBE_MS, self._probe)

    def _probe(self):
        t = time.perf_counter()
        self._root.after_idle(lambda: self._idle.append(time.perf_counter() - t))
        now = time.monotonic()
        if now - self._t_report >= REPORT_S:
            self._report(now - self._t_report)
            self._t_report = now
        self._root.after(_PROBE_MS, self._probe)

    def _report(self, span):
        ticks = self._stats.get("TICK", [])
        lines = []
        if ticks:
            lines.append(f"[PERF] {span:.0f} s: {len(ticks)} ticks ({len(ticks) / span:.1f} Hz)  "
                         f"tick mean {1000 * sum(ticks) / len(ticks):.1f} ms  "
                         f"max {1000 * max(ticks):.1f} ms")
        else:
            lines.append(f"[PERF] {span:.0f} s: no UI ticks")
        if self._idle:
            s = sorted(self._idle)
            lines.append(f"       idle delay p50 {1000 * s[len(s) // 2]:.1f} ms  "
                         f"p95 {1000 * s[int(len(s) * 0.95)]:.1f} ms  "
                         f"max {1000 * s[-1]:.1f} ms   (repaint latency)")
        for key, v in sorted(self._stats.items(), key=lambda kv: -sum(kv[1])):
            if key == "TICK" or not v:
                continue
            lines.append(f"       {key:10s} {len(v) / span:5.1f} Hz  mean {1000 * sum(v) / len(v):5.2f} ms  "
                         f"max {1000 * max(v):6.1f} ms")
        print("\n".join(lines), flush=True)
        for v in self._stats.values():
            v.clear()
        self._idle.clear()


def install(app) -> "PerfMonitor | None":
    """Instrument a DroneCockpitApp if COCKPIT_PERF=1; returns the monitor."""
    if not enabled():
        return None
    mon = PerfMonitor(app.root)
    mon.wrap(app, "_update_loop", "TICK")
    for key, obj, meth in (
            ("imu", app.imu_view, "update_ui"),
            ("imu_radio", app.imu_panel.radio, "update_radio"),
            ("mag", app.mag_view, "update_mag"),
            ("fc", app.fc_status_view, "update_fc_status"),
            ("fc_radio", app.fc_status_panel.radio, "update_radio"),
            ("arming", app.arming_view, "update_arming"),
            ("baro", app.baro_view, "update_baro"),
            ("adi", app.drone_3d, "update_orientation"),
            ("gps", app.gps_view, "update_gps"),
            ("link", app, "_update_link_status")):
        mon.wrap(obj, meth, key)
    mon.start()
    print("[PERF] UI performance monitor on (COCKPIT_PERF=1) — report every "
          f"{REPORT_S:.0f} s", flush=True)
    return mon
