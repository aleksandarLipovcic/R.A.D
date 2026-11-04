"""
radio_link_indicator.py  —  ELRS radio-link traffic light for the toolbar
=========================================================================

    ●  RADIO: OK  LQ 100%   ▶ ELRS

GREEN   telemetry is arriving over ELRS and everything is fresh.
YELLOW  link up but something is missing or weak:
          • uplink LQ below the DEGRADED threshold (CrsfLink, default 70 %)
          • horizon (attitude) data going stale
          • GPS / battery / flight-mode frames missing or older than 3 s
RED     no usable radio link:
          • NO_RADIO        Pocket not found on USB (or wrong USB mode)
          • WAITING         Pocket found, but no telemetry from the drone
          • TELEMETRY_LOST  had telemetry, it stopped  → BLINKS
GREY    this DroneBackend build has no CrsfLink (radio support disabled)

The "▶ USB / ▶ ELRS / ▶ NONE" tag shows which source is feeding the
instruments right now (USB wins while the cable link is healthy).

Pure Tk, main thread only. Fed from TelemetryWorker.get_link_status()
once per UI tick; classify() is pure so it can be unit-tested headless.
"""
import time
import tkinter as tk

GREEN = "#00ff88"
YELLOW = "#ffcc00"
RED = "#ff3344"
GREY = "#667088"
BG = "#0d0d1a"

STALE_MS = 3000   # a data group older than this counts as "missing"


def _missing(age_ms: int) -> bool:
    return age_ms is None or age_ms < 0 or age_ms > STALE_MS


def classify(st: dict):
    """
    Returns (color, headline, detail, blink) for a TelemetryWorker link-status dict.
    """
    status = st.get("radio_status", "NO_RADIO")
    lq = st.get("uplink_lq", 0)
    port = st.get("radio_port") or "-"

    if status == "DISABLED":
        return GREY, "RADIO: N/A", "This DroneBackend build has no CrsfLink.", False

    if status == "NO_RADIO":
        return (RED, "RADIO: NOT CONNECTED",
                "RadioMaster Pocket not found on USB.\n"
                "Plug it in, choose 'USB Serial (VCP)', and set\n"
                "SYS → Hardware → USB-VCP = Telem Mirror.", False)

    if status == "WAITING":
        return (RED, "RADIO: NO DRONE LINK",
                f"Pocket connected on {port}, but no telemetry from the drone.\n"
                "Drone powered and bound? Telemetry enabled in Betaflight?", False)

    if status == "TELEMETRY_LOST":
        age = st.get("age_last_frame_ms", -1)
        age_txt = f"{age / 1000:.1f} s ago" if age >= 0 else "never"
        return (RED, "RADIO: LINK LOST",
                f"No drone telemetry since {age_txt}.\n"
                "Instruments show the LAST KNOWN values (incl. last GPS position).", True)

    # Link is up (TELEMETRY_OK or DEGRADED) — look for what is weak/missing.
    problems = []
    if status == "DEGRADED":
        if st.get("link_stats_valid") and lq < 70:
            problems.append(f"WEAK LQ {lq}%")
        if _missing(st.get("age_attitude_ms", -1)) or st.get("age_attitude_ms", 0) > 1500:
            problems.append("HORIZON STALE")
        if not problems:
            problems.append("DEGRADED")
    for key, name in (("age_gps_ms", "NO GPS DATA"),
                      ("age_battery_ms", "NO BATTERY DATA"),
                      ("age_flight_mode_ms", "NO MODE DATA")):
        if _missing(st.get(key, -1)):
            problems.append(name)

    detail = (f"Port {port}   uplink LQ {lq}%   RSSI {st.get('uplink_rssi1_dbm', 0)}/"
              f"{st.get('uplink_rssi2_dbm', 0)} dBm   SNR {st.get('uplink_snr', 0)} dB\n"
              f"downlink LQ {st.get('downlink_lq', 0)}%   TX {st.get('tx_power_mw', 0)} mW\n"
              f"rates: attitude {st.get('rate_attitude_hz', 0):.1f} Hz, "
              f"GPS {st.get('rate_gps_hz', 0):.1f} Hz, battery {st.get('rate_battery_hz', 0):.1f} Hz\n"
              f"reconnects {st.get('reconnect_count', 0)}   link losses {st.get('link_lost_count', 0)}")

    if problems:
        return YELLOW, "RADIO: " + ", ".join(problems[:2]), detail, False
    return GREEN, f"RADIO: OK  LQ {lq}%", detail, False


class RadioLinkIndicator(tk.Frame):
    """Toolbar widget. Call update_status(worker.get_link_status()) every UI tick."""

    DOT = 14

    def __init__(self, master, **kw):
        super().__init__(master, bg=BG, **kw)
        self._dot = tk.Canvas(self, width=self.DOT + 4, height=self.DOT + 4,
                              bg=BG, highlightthickness=0, bd=0)
        self._dot.pack(side="left", padx=(0, 6))
        self._dot_id = self._dot.create_oval(2, 2, self.DOT + 2, self.DOT + 2,
                                             fill=GREY, outline="")
        self._text = tk.Label(self, text="RADIO: …", fg=GREY, bg=BG,
                              font=("Consolas", 10, "bold"))
        self._text.pack(side="left")
        self._src = tk.Label(self, text="", fg="#a0b8d8", bg=BG,
                             font=("Consolas", 9))
        self._src.pack(side="left", padx=(8, 0))
        self.detail_text = ""
        self._last = None

    def hover_widgets(self):
        """Child widgets to attach hover tooltips to."""
        return (self._dot, self._text, self._src)

    def update_status(self, st: dict) -> None:
        color, headline, detail, blink = classify(st)
        if blink and int(time.monotonic() * 2) % 2:
            shown = BG                           # blink at 1 Hz
        else:
            shown = color
        src = st.get("active_source", "NONE")
        key = (shown, headline, src)
        if key != self._last:                    # avoid needless Tk reconfigures
            self._dot.itemconfig(self._dot_id, fill=shown)
            self._text.config(text=headline, fg=color)
            self._src.config(text=f"▶ {src}",
                             fg="#a0b8d8" if src != "NONE" else RED)
            self._last = key
        self.detail_text = detail
