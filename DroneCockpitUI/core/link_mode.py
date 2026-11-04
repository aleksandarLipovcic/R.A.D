"""
link_mode.py  —  what the active telemetry link can show
=========================================================

The cockpit gets telemetry from one of two sources (see telemetry_worker.py):

  USB   DroneLink, MSP over the USB-C cable — the full data set, refreshed
        every poll.
  ELRS  CrsfLink, CRSF telemetry over the ExpressLRS radio — attitude,
        battery, GPS position, baro/vario, flight mode and link statistics
        only, each at its own (much lower) rate.

Every widget asks this module the same three questions about a ui_data
frame, so they all behave alike:

  is_radio(data)            is the radio feeding the instruments?
  available(data, what)     can this link deliver `what` at all?
                            False → show a grey "USB" placeholder, never a
                            zero that looks like a real reading.
  stale_ms(data, group, …)  how old is this data group, if too old?
                            Only the radio link ages; USB frames are fresh.

Pure functions plus shared colours/texts — no Tk, unit-testable headless.
"""

USB = "USB"
RADIO = "ELRS"

# ── Shared look for "not on this link" and "stale" ───────────────────────────
C_NA_FG    = "#50657a"   # grey-blue: readable, but clearly not a live value
C_NA_BG    = "#0c1420"
C_STALE_FG = "#d0a040"   # muted amber: old value, still shown
C_RADIO    = "#44aaff"   # source tag colour for the radio link
C_USB      = "#8fa8c0"   # source tag colour for the cable link

NA_SHORT = "USB"         # fits in a number cell
NA_LONG  = "USB ONLY"    # for labels with more room

# ── Staleness limits per data group (ms) ─────────────────────────────────────
# Chosen against typical ELRS/Betaflight telemetry rates: attitude arrives
# several times a second, GPS/battery/mode about 1 Hz.
STALE_ATTITUDE_MS = 1500
STALE_SLOW_MS     = 3000     # gps, battery, flight mode, baro

_STALE_LIMITS = {
    "attitude":    STALE_ATTITUDE_MS,
    "gps":         STALE_SLOW_MS,
    "battery":     STALE_SLOW_MS,
    "flight_mode": STALE_SLOW_MS,
    "baro":        STALE_SLOW_MS,
}


def source(data: dict) -> str:
    """'USB' or 'ELRS'. Frames from an older worker have no key → USB."""
    return RADIO if data.get("link_source") == RADIO else USB


def is_radio(data: dict) -> bool:
    return source(data) == RADIO


def available(data: dict, what: str) -> bool:
    """
    what: raw_imu, raw_mag, motors, rc_channels, sat_list, hdop,
          arming_flags, cpu_load. Unknown keys count as available.
    """
    return bool(data.get(f"available_{what}", True))


def age_ms(data: dict, group: str) -> int:
    """Age of a data group in ms; -1 = never received, 0 on USB."""
    if not is_radio(data):
        return 0
    try:
        return int(data.get(f"age_{group}_ms", 0))
    except (TypeError, ValueError):
        return 0


def stale_ms(data: dict, group: str, limit_ms: int = None) -> int:
    """
    0 if the group is fresh (always on USB), otherwise its age in ms, or -1
    if it was never received on the radio link.
    """
    if not is_radio(data):
        return 0
    limit = _STALE_LIMITS.get(group, STALE_SLOW_MS) if limit_ms is None else limit_ms
    a = age_ms(data, group)
    if a < 0:
        return -1
    return a if a > limit else 0


def stale_text(ms: int) -> str:
    """'STALE 3.2s' / 'NO DATA' for a non-zero stale_ms() result."""
    if ms < 0:
        return "NO DATA"
    return f"STALE {ms / 1000:.1f}s"


def source_tag(data: dict) -> tuple:
    """(text, colour) for a small 'where does this come from' tag."""
    if is_radio(data):
        return "RADIO", C_RADIO
    return "USB", C_USB
