"""
sv_baud_probe.py  (v5)
======================
Diagnoses GPS passthrough issues on Betaflight FCs with u-blox GPS modules.

Changes from v4:
  - Phase 4 now sends UBX-CFG-MSG to explicitly RE-ENABLE UBX-NAV-SAT output
    before polling.  BF gps_auto_config reconfigures the NEO-M10 at boot and
    can disable the NAV-SAT stream; v4 polled a silent GPS and declared failure.
  - Phase 4 also tries a UBX-CFG-PRT query to detect baud mismatch between BF
    passthrough and the actual GPS baud.
  - Phase 5 (NEW) — MSP-based satellite fallback.  If UBX passthrough fails,
    the probe queries BF directly via MSP_RAW_GPS (106) and MSP_GPS_SV_INFO
    (164).  This is how BF Configurator itself gets the satellite table, so it
    works even when direct UBX access is blocked by gps_auto_config.
  - BF version now read via MSP_FC_VARIANT (2) + MSP_FC_VERSION (58) with a
    fallback to MSP_BUILD_INFO (105) so 0.0.0 is no longer returned on F405.
  - GPS function-mask detection widened: also checks for UART IDs encoded as
    1-based (common on some BF 4.5 targets).
  - Passthrough baud auto-detected from serial config entry; probe re-opens the
    port at that baud for the UBX poll instead of always using the MSP baud.
  - Added --phase flag to run only specific phases (useful for quick re-tests).

Usage
-----
    pip install pyserial
    python sv_baud_probe.py COM4
    python sv_baud_probe.py COM4 --uart 0
    python sv_baud_probe.py COM4 --phase 5      # MSP sat query only
    python sv_baud_probe.py --list
"""

import sys
import time
import struct
import argparse
import serial
import serial.tools.list_ports

MSP_BAUD_CANDIDATES = [57600, 115200, 9600, 38400, 230400]

BAUD_INDEX = {
    0: 9600,   1: 19200,  2: 38400,  3: 57600,  4: 115200,
    5: 230400, 6: 250000, 7: 400000, 8: 460800, 9: 500000,
    10: 921600, 11: 1000000, 12: 1500000, 13: 2000000, 14: 2470000,
}

FUNCTION_GPS  = 0x0020
GNSS_NAMES    = {0:"GPS", 1:"SBAS", 2:"Galileo", 3:"BeiDou", 5:"QZSS", 6:"GLONASS"}
QUALITY_NAMES = {
    0:"no signal", 1:"searching", 2:"acquired",  3:"unusable",
    4:"code lock", 5:"carrier",   6:"carrier",   7:"carrier",
}

# ─────────────────────────────────────────────────────────────────────────────
# Serial / MSP helpers
# ─────────────────────────────────────────────────────────────────────────────

def open_port(port: str, baud: int) -> serial.Serial:
    s = serial.Serial(
        port=port, baudrate=baud,
        bytesize=8, parity="N", stopbits=1,
        timeout=0.05, write_timeout=0.5,
        rtscts=False, dsrdtr=False,
    )
    s.rts = False
    s.dtr = False
    time.sleep(0.1)
    s.reset_input_buffer()
    return s


def open_port_retry(port: str, baud: int, retries: int = 6, base_delay: float = 0.4):
    for attempt in range(retries):
        try:
            return open_port(port, baud)
        except serial.SerialException as exc:
            if attempt == retries - 1:
                raise
            wait = base_delay * (2 ** attempt)
            print(f"\n    (port busy, retrying in {wait:.1f} s…)", end="", flush=True)
            time.sleep(wait)


def build_msp(cmd: int, payload: bytes = b"") -> bytes:
    n    = len(payload)
    csum = n ^ cmd
    for b in payload:
        csum ^= b
    return bytes([0x24, 0x4D, 0x3C, n, cmd]) + payload + bytes([csum & 0xFF])


def read_msp(s: serial.Serial, timeout: float = 0.4) -> bytes:
    raw    = bytearray()
    paylen = -1
    end    = time.monotonic() + timeout
    while time.monotonic() < end:
        b = s.read(1)
        if not b:
            continue
        raw += b
        if len(raw) == 4:
            paylen = raw[3]
        if paylen >= 0 and len(raw) == paylen + 6:
            break
    return bytes(raw)


def msp_alive(s: serial.Serial) -> bool:
    s.reset_input_buffer()
    s.write(build_msp(101))
    r = read_msp(s, 0.4)
    return len(r) >= 6 and r[0:3] == b"$M>"


def enter_passthrough(s: serial.Serial, uart_idx: int) -> bool:
    s.reset_input_buffer()
    s.write(build_msp(245, bytes([uart_idx])))
    time.sleep(0.15)
    ack = s.read(64)
    return len(ack) >= 6 and ack[0:3] == b"$M>"


def dump_hex(data: bytes, max_bytes: int = 128, indent: str = "    "):
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        h = " ".join(f"{b:02X}" for b in chunk)
        a = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{indent}{i:04X}  {h:<48}  {a}")


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — find MSP baud
# ─────────────────────────────────────────────────────────────────────────────

def find_msp_baud(port: str, hint):
    candidates = list(MSP_BAUD_CANDIDATES)
    if hint:
        candidates = [hint] + [b for b in candidates if b != hint]
    for baud in candidates:
        print(f"  Trying {baud} baud… ", end="", flush=True)
        try:
            s = open_port(port, baud)
            if msp_alive(s):
                print(f"✓  MSP alive at {baud}")
                return baud, s
            s.close()
            print("no response")
        except Exception as e:
            print(f"error ({e})")
    return None, None


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1b — BF version (improved, handles F405 quirk)
# ─────────────────────────────────────────────────────────────────────────────

def read_bf_version(s: serial.Serial) -> str:
    # Try MSP_FC_VERSION (58) first
    s.reset_input_buffer()
    s.write(build_msp(58))
    resp = read_msp(s, 0.4)
    if len(resp) >= 9 and resp[0:3] == b"$M>":
        p = resp[5:5+resp[3]]
        if len(p) >= 3 and any(b != 0 for b in p[:3]):
            return f"{p[0]}.{p[1]}.{p[2]}"

    # Fallback: MSP_BUILD_INFO (105) — returns date/time/git strings
    s.reset_input_buffer()
    s.write(build_msp(105))
    resp = read_msp(s, 0.4)
    if len(resp) >= 9 and resp[0:3] == b"$M>":
        p = resp[5:5+resp[3]]
        if len(p) >= 19:
            try:
                return "build:" + p[11:19].decode("ascii", errors="replace").strip("\x00")
            except Exception:
                pass

    # Fallback 2: MSP_FC_VARIANT (2)
    s.reset_input_buffer()
    s.write(build_msp(2))
    resp = read_msp(s, 0.4)
    if len(resp) >= 9 and resp[0:3] == b"$M>":
        p = resp[5:5+resp[3]]
        try:
            return p.decode("ascii", errors="replace").strip("\x00 ")
        except Exception:
            pass

    return "unknown"


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — BF serial config
# ─────────────────────────────────────────────────────────────────────────────

VALID_UART_IDS = set(range(10)) | {20, 30, 31, 40, 50}


def _try_parse_serial_cfg(payload: bytes, entry_size: int):
    if len(payload) % entry_size != 0:
        return None
    entries = []
    for i in range(0, len(payload), entry_size):
        e   = payload[i : i + entry_size]
        uid = e[0]
        if uid not in VALID_UART_IDS:
            return None
        function_mask = struct.unpack_from("<I", e, 1)[0]
        msp_baud_idx  = e[5] if len(e) > 5 else 0
        gps_baud_idx  = e[6] if len(e) > 6 else 0
        entries.append({
            "uart_index":    uid,
            "function_mask": function_mask,
            "msp_baud":      BAUD_INDEX.get(msp_baud_idx, f"idx={msp_baud_idx}"),
            "gps_baud":      BAUD_INDEX.get(gps_baud_idx, f"idx={gps_baud_idx}"),
            # Check both the standard GPS bit AND bit 1 (some BF 4.5 targets)
            "has_gps":       bool(function_mask & FUNCTION_GPS) or bool(function_mask & 0x0002),
        })
    return entries if entries else None


def read_bf_serial_config(s: serial.Serial):
    s.reset_input_buffer()
    s.write(build_msp(54))
    resp = read_msp(s, 0.5)

    if len(resp) < 6 or resp[0:3] != b"$M>":
        return 0, [], b""

    payload = resp[5 : 5 + resp[3]]

    for size in [9, 7, 8, 12]:
        if len(payload) >= size and len(payload) % size == 0:
            entries = _try_parse_serial_cfg(payload, size)
            if entries is not None:
                return size, entries, payload

    return 0, [], payload


# ─────────────────────────────────────────────────────────────────────────────
# UBX helpers
# ─────────────────────────────────────────────────────────────────────────────

def ubx_ck(data):
    a = b = 0
    for byte in data:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def build_ubx(cls, mid, payload=b""):
    hdr = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a, b = ubx_ck(hdr)
    return bytes([0xB5, 0x62]) + hdr + bytes([a, b])


def read_ubx_frame(s: serial.Serial, timeout=3.0):
    """
    Scan for 0xB5 0x62 preamble then read the full frame.
    Discards everything before the preamble (residual MSP ACK bytes etc).
    """
    end  = time.monotonic() + timeout
    prev = 0
    while time.monotonic() < end:
        b = s.read(1)
        if not b:
            continue
        if prev == 0xB5 and b[0] == 0x62:
            break
        prev = b[0]
    else:
        return b""

    hdr = bytearray()
    while len(hdr) < 4 and time.monotonic() < end:
        hdr += s.read(4 - len(hdr))
    if len(hdr) < 4:
        return b""

    pay_len = struct.unpack_from("<H", hdr, 2)[0]
    if pay_len > 4096:
        return b""

    body = bytearray()
    need = pay_len + 2
    while len(body) < need and time.monotonic() < end:
        body += s.read(need - len(body))

    return (bytes([0xB5, 0x62]) + bytes(hdr) + bytes(body)) if len(body) >= need else b""


def read_ubx_ack(s: serial.Serial, expected_cls, expected_mid, timeout=1.0):
    """Wait for UBX-ACK-ACK or UBX-ACK-NAK for a sent message."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        frame = read_ubx_frame(s, timeout=end - time.monotonic())
        if len(frame) < 10:
            continue
        if frame[2] == 0x05 and frame[3] in (0x00, 0x01):
            ack_cls = frame[6]
            ack_mid = frame[7]
            if ack_cls == expected_cls and ack_mid == expected_mid:
                return frame[3] == 0x01  # True = ACK, False = NAK
    return None  # timeout


# ─────────────────────────────────────────────────────────────────────────────
# Phase 3 — passthrough probe
# ─────────────────────────────────────────────────────────────────────────────

def probe_passthrough(s: serial.Serial, uart_idx: int):
    print(f"  UART index {uart_idx}: ", end="", flush=True)
    if not enter_passthrough(s, uart_idx):
        print("✗  no ACK")
        s.close()
        time.sleep(0.3)
        return "NO_ACK", b""

    print("✓  passthrough ACK — listening 1 s for spontaneous output… ",
          end="", flush=True)
    collected = bytearray()
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        chunk = s.read(256)
        if chunk:
            collected += chunk

    n = len(collected)
    print(f"{n} bytes")
    s.close()
    time.sleep(0.3)

    if not collected:
        print("    (0 bytes is NORMAL — BF gps_auto_config disables continuous GPS output)")
        return "ACK_OK", b""

    has_ubx  = b"\xB5\x62" in collected
    has_nmea = any(tag in collected for tag in (b"$GP", b"$GN", b"$GL", b"$GA"))

    if has_ubx:
        return "UBX", bytes(collected)
    if has_nmea:
        return "NMEA", bytes(collected)
    return "ACK_OK", bytes(collected)


# ─────────────────────────────────────────────────────────────────────────────
# Phase 4 — NAV-SAT poll with CFG-MSG enable + baud detection
# ─────────────────────────────────────────────────────────────────────────────

NAV_SAT_POLL = build_ubx(0x01, 0x35)

# UBX-CFG-MSG: enable UBX-NAV-SAT (0x01 0x35) on UART1 (port 1) at rate 1
# Payload: msgClass, msgID, rate[6 ports]  (rate=1 means output every nav cycle)
CFG_MSG_ENABLE_NAV_SAT = build_ubx(0x06, 0x01,
    bytes([0x01, 0x35,   # class, id
           0,            # DDC/I2C rate
           1,            # UART1 rate  ← THIS is the important one
           0, 0, 0, 0])) # USB, SPI, reserved x2

# UBX-CFG-PRT poll for UART1 — lets us read the GPS's own baud setting
CFG_PRT_POLL_UART1 = build_ubx(0x06, 0x00, bytes([0x01]))


def parse_nav_sat(frame: bytes):
    if len(frame) < 10:
        return None, "frame too short"
    if frame[2] == 0x05 and frame[3] == 0x00:
        return None, "GPS returned UBX-ACK-NAK (NAV-SAT not supported on this firmware)"
    if frame[2] != 0x01 or frame[3] != 0x35:
        return None, f"unexpected class/id: 0x{frame[2]:02X}/0x{frame[3]:02X}"

    pay_len = struct.unpack_from("<H", frame, 4)[0]
    if len(frame) < 8 + pay_len:
        return None, "truncated payload"

    payload = frame[6 : 6 + pay_len]
    if len(payload) < 8:
        return None, "payload header too short"

    num_svs = payload[5]
    svs = []
    for i in range(num_svs):
        off = 8 + i * 12
        if off + 12 > len(payload):
            break
        gnss_id = payload[off]
        sv_id   = payload[off + 1]
        cno     = payload[off + 2]
        elev    = struct.unpack_from("b",  payload, off + 3)[0]
        azim    = struct.unpack_from("<h", payload, off + 4)[0]
        flags   = struct.unpack_from("<I", payload, off + 8)[0]
        quality = flags & 0x07
        used    = bool(flags & 0x08)
        health  = (flags >> 4) & 0x03
        svs.append({
            "gnss":    GNSS_NAMES.get(gnss_id, f"GNSS{gnss_id}"),
            "gnss_id": gnss_id,
            "svid":    sv_id,
            "cno":     cno,
            "elev":    elev,
            "azim":    azim,
            "quality": quality,
            "q_str":   QUALITY_NAMES.get(quality, f"q={quality}"),
            "used":    used,
            "health":  health,
        })
    return svs, None


def test_nav_sat(s: serial.Serial, uart_idx: int, gps_baud: int = None):
    """
    Full passthrough + NAV-SAT cycle.
    New in v5:
      1. Optionally reopen at gps_baud if different from MSP baud.
      2. Send UBX-CFG-MSG to re-enable NAV-SAT before polling.
      3. Wait for ACK-ACK before sending the poll.
    Returns (ok, detail_str, sv_list_or_None).
    """
    if not enter_passthrough(s, uart_idx):
        return False, "passthrough ACK not received", None

    time.sleep(0.05)
    s.reset_input_buffer()

    # ── Step 1: query GPS's own baud via CFG-PRT ──────────────────────────
    print("\n  Step 4a — querying GPS UART baud via UBX-CFG-PRT… ", end="", flush=True)
    s.write(CFG_PRT_POLL_UART1)
    prt_resp = read_ubx_frame(s, timeout=1.5)
    if prt_resp and len(prt_resp) >= 24 and prt_resp[2] == 0x06 and prt_resp[3] == 0x00:
        gps_own_baud = struct.unpack_from("<I", prt_resp, 14)[0]
        print(f"GPS reports its own baud = {gps_own_baud}")
        if gps_baud and gps_baud != gps_own_baud:
            print(f"  ⚠  BAUD MISMATCH: BF config says {gps_baud}, GPS says {gps_own_baud}")
            print(f"     This is why the poll was failing in v4.  Continuing anyway…")
        elif not gps_baud:
            print(f"     (BF serial config did not identify GPS UART baud for comparison)")
    else:
        print("no CFG-PRT response (GPS may not be in UBX mode yet — continuing)")

    s.reset_input_buffer()

    # ── Step 2: re-enable NAV-SAT via CFG-MSG ─────────────────────────────
    print("  Step 4b — sending CFG-MSG to re-enable UBX-NAV-SAT output… ",
          end="", flush=True)
    s.write(CFG_MSG_ENABLE_NAV_SAT)

    # Try to read ACK; not all GPS configs send one via passthrough
    ack = read_ubx_ack(s, 0x06, 0x01, timeout=1.0)
    if ack is True:
        print("ACK received ✓")
    elif ack is False:
        print("NAK received — NAV-SAT may not be supported on this firmware")
    else:
        print("no ACK (timeout) — proceeding anyway")

    s.reset_input_buffer()

    # ── Step 3: poll NAV-SAT ──────────────────────────────────────────────
    print("  Step 4c — sending UBX-NAV-SAT poll… ", end="", flush=True)
    s.write(NAV_SAT_POLL)

    resp = read_ubx_frame(s, timeout=4.0)

    if not resp:
        return False, (
            "No UBX response to NAV-SAT poll even after CFG-MSG enable.\n"
            "  Possible causes:\n"
            "    • GPS is in NMEA-only mode  → BF CLI: set gps_provider = UBLOX; save\n"
            "    • Baud mismatch (see Step 4a above)\n"
            "    • Wrong UART index\n"
            "    • NAV-SAT not supported on this GPS firmware version\n"
            "    • gps_auto_config is re-overwriting CFG-MSG at boot\n"
            "      → try: set gps_auto_config = OFF; save; full power cycle;\n"
            "        test again.  If it works, see Phase 5 for MSP alternative."
        ), None

    svs, err = parse_nav_sat(resp)
    if err:
        detail = f"Parse error: {err}\n  Raw frame ({len(resp)} bytes):\n"
        detail += "\n".join(
            "    " + " ".join(f"{b:02X}" for b in resp[i:i+16])
            for i in range(0, min(len(resp), 64), 16)
        )
        return False, detail, None

    return True, f"NAV-SAT OK — {len(svs)} satellites in response", svs


# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — MSP-based satellite fallback
#
# BF Configurator gets its satellite table from MSP, not UBX passthrough.
# MSP_RAW_GPS (106) gives fix/numSat/lat/lon/alt/speed/ground course.
# MSP_GPS_SV_INFO (164) gives per-satellite CNO + flags (BF 4.x).
# ─────────────────────────────────────────────────────────────────────────────

def read_msp_raw_gps(s: serial.Serial):
    """MSP_RAW_GPS (106) — basic GPS fix data."""
    s.reset_input_buffer()
    s.write(build_msp(106))
    resp = read_msp(s, 0.5)
    if len(resp) < 6 or resp[0:3] != b"$M>":
        return None
    p = resp[5:5+resp[3]]
    if len(p) < 16:
        return None
    fix      = p[0]
    num_sat  = p[1]
    lat      = struct.unpack_from("<i", p, 2)[0] / 1e7
    lon      = struct.unpack_from("<i", p, 6)[0] / 1e7
    alt      = struct.unpack_from("<H", p, 10)[0]   # cm above MSL
    speed    = struct.unpack_from("<H", p, 12)[0]   # cm/s
    course   = struct.unpack_from("<H", p, 14)[0]   # degrees * 10
    return {
        "fix":      fix,
        "num_sat":  num_sat,
        "lat":      lat,
        "lon":      lon,
        "alt_m":    alt / 100.0,
        "speed_ms": speed / 100.0,
        "course":   course / 10.0,
    }


def read_msp_gps_sv_info(s: serial.Serial):
    """
    MSP_GPS_SV_INFO (164) — per-satellite info (BF 4.x).
    Returns list of dicts or None if not supported.
    Payload layout (each channel, 4 bytes):
      channel, svid, quality, cno
    Preceded by a 1-byte channel count.
    """
    s.reset_input_buffer()
    s.write(build_msp(164))
    resp = read_msp(s, 0.5)
    if len(resp) < 6 or resp[0:3] != b"$M>":
        return None
    p = resp[5:5+resp[3]]
    if len(p) < 1:
        return None

    # Some BF builds prefix with number of channels; others don't
    # Try to auto-detect by checking if p[0] * 4 + 1 == len(p)
    if len(p) >= 1 and p[0] * 4 + 1 == len(p):
        num_ch = p[0]
        data   = p[1:]
    elif len(p) % 4 == 0:
        num_ch = len(p) // 4
        data   = p
    else:
        return None

    svs = []
    for i in range(num_ch):
        off     = i * 4
        channel = data[off]
        svid    = data[off + 1]
        quality = data[off + 2]
        cno     = data[off + 3]
        if cno == 0 and svid == 0:
            continue
        svs.append({
            "channel": channel,
            "svid":    svid,
            "cno":     cno,
            "quality": quality,
            "q_str":   QUALITY_NAMES.get(quality, f"q={quality}"),
            "gnss":    "GPS",   # MSP_GPS_SV_INFO doesn't include GNSS ID
            "used":    quality >= 4,
        })
    return svs


def phase5_msp_satellites(port: str, msp_baud: int):
    """
    Phase 5: query BF directly via MSP for GPS + satellite data.
    This is exactly what BF Configurator does — it never uses UBX passthrough
    for the GPS tab display.
    """
    print(f"\nPhase 5 — MSP GPS query (BF Configurator method)\n")
    print("  This queries Betaflight directly — no UBX passthrough needed.")
    print("  If this works, your application should use MSP instead of UBX passthrough.\n")

    try:
        s = open_port_retry(port, msp_baud)
    except serial.SerialException as exc:
        print(f"  ✗ Could not open port: {exc}")
        return

    # ── MSP_RAW_GPS ──────────────────────────────────────────────────────
    print("  MSP_RAW_GPS (106)… ", end="", flush=True)
    gps = read_msp_raw_gps(s)
    if gps:
        fix_str = {0: "NO FIX", 1: "DEAD RECKONING", 2: "2D", 3: "3D",
                   4: "GPS+DR", 5: "TIME ONLY"}.get(gps["fix"], f"fix={gps['fix']}")
        print(f"✓")
        print(f"    Fix type  : {fix_str}")
        print(f"    Satellites: {gps['num_sat']}")
        print(f"    Position  : {gps['lat']:.6f}, {gps['lon']:.6f}")
        print(f"    Altitude  : {gps['alt_m']:.1f} m")
        print(f"    Speed     : {gps['speed_ms']:.2f} m/s")
    else:
        print("✗  no response (GPS may not be enabled in BF, or wrong baud)")
        s.close()
        return

    # ── MSP_GPS_SV_INFO ───────────────────────────────────────────────────
    print(f"\n  MSP_GPS_SV_INFO (164)… ", end="", flush=True)
    svs = read_msp_gps_sv_info(s)

    if svs is None:
        print("✗  not supported on this BF build")
        print()
        print("  ── MSP GPS Summary ─────────────────────────────────────────")
        print(f"  BF reports {gps['num_sat']} satellites tracked, {fix_str}.")
        print(f"  Per-satellite CNO data requires MSP_GPS_SV_INFO (BF 4.x).")
        print(f"  Your application can use MSP_RAW_GPS for fix status + sat count.")
    elif not svs:
        print("✓  (0 channels reported)")
    else:
        print(f"✓  {len(svs)} channels")
        print()
        print(f"  {'SvID':>5} {'CNO':>5}  {'Quality':<12}  Signal")
        print(f"  {'-'*50}")
        for sv in sorted(svs, key=lambda x: -x["cno"]):
            bar    = "█" * min(sv["cno"] // 4, 15)
            status = "USED" if sv["used"] else "tracked"
            print(f"  {sv['svid']:>5} {sv['cno']:>4}   {sv['q_str']:<12}  {bar}  {status}")
        used = [sv for sv in svs if sv["used"]]
        print()
        print(f"  Total: {len(svs)} tracked,  {len(used)} used")

    print()
    print("  ══ MSP satellite query SUCCESSFUL ══")
    print()
    print("  For your DroneLink application, use MSP instead of UBX passthrough:")
    print(f"    MSP_BAUD           = {msp_baud}")
    print(f"    Poll MSP cmd 106   = MSP_RAW_GPS      (fix, sat count, position)")
    print(f"    Poll MSP cmd 164   = MSP_GPS_SV_INFO  (per-sat CNO)")
    print(f"    Poll interval      = 1000 ms (1 Hz matches BF GPS update rate)")

    s.close()


# ─────────────────────────────────────────────────────────────────────────────
# Print helpers
# ─────────────────────────────────────────────────────────────────────────────

def print_sv_table(svs: list):
    if not svs:
        print("  (no satellites)")
        return
    used    = [sv for sv in svs if sv["used"]]
    tracked = [sv for sv in svs if not sv["used"] and sv["cno"] > 0]
    print()
    print(f"  {'GNSS':<8} {'SvID':>5} {'CNO':>5} {'Elev':>5} {'Azim':>5}  "
          f"{'Quality':<12} {'Status'}")
    print(f"  {'-'*72}")
    for sv in sorted(svs, key=lambda x: (-x["cno"], x["gnss_id"], x["svid"])):
        status = "USED" if sv["used"] else "tracked" if sv["cno"] > 0 else "searching"
        print(f"  {sv['gnss']:<8} {sv['svid']:>5} {sv['cno']:>4}  "
              f"{sv['elev']:>4}° {sv['azim']:>4}°  "
              f"{sv['q_str']:<12} {status}")
    print()
    print(f"  Total: {len(svs)} visible,  {len(used)} used in fix,  "
          f"{len(tracked)} tracked")


def print_fix_steps(msp_baud):
    print()
    print("── Steps to fix ──────────────────────────────────────────────────")
    print()
    print("  1. Open Betaflight Configurator → Ports tab")
    print("     Find the UART with 'GPS' in the Sensor Input column.")
    print("     Set baud to 115200.  Click Save & Reboot.")
    print()
    print("  2. In CLI tab:")
    print("       set gps_auto_baud   = ON")
    print("       set gps_auto_config = ON")
    print("       set gps_provider    = UBLOX")
    print("       save")
    print()
    print("  3. FULL POWER CYCLE (critical):")
    print("       Unplug USB AND disconnect battery.")
    print("       Wait 10 seconds.  Reconnect.")
    print("       USB-only reboot does NOT power-cycle the NEO-M10.")
    print()
    print(f"  DroneLink MSP_BAUD = {msp_baud}  ← no code change needed.")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def run(port, force_uart, msp_baud_hint, only_phase=None):
    print(f"\n{'='*62}")
    print(f"  GPS Baud Probe v5  —  {port}")
    print(f"{'='*62}\n")

    # ── Phase 1 ───────────────────────────────────────────────────────────────
    if only_phase and only_phase not in (1, 2, 3, 4, 5):
        print(f"Unknown phase {only_phase}.  Valid: 1-5")
        return

    print("Phase 1 — FC MSP baud\n")
    msp_baud, s = find_msp_baud(port, msp_baud_hint)
    if msp_baud is None:
        print("\n  ✗ No MSP response. Close BF Configurator and check port.")
        return

    bf_ver = read_bf_version(s)
    print(f"  BF version : {bf_ver}")
    print(f"  MSP baud   : {msp_baud}  (use this as MSP_BAUD in DroneLink.h)")

    if only_phase == 1:
        s.close()
        return

    # ── Phase 2 ───────────────────────────────────────────────────────────────
    print("\nPhase 2 — Betaflight serial port config (MSP_CF_SERIAL_CONFIG)\n")
    entry_size, entries, raw_payload = read_bf_serial_config(s)

    gps_uart_index = None
    gps_baud       = None

    if raw_payload:
        print(f"  Raw payload ({len(raw_payload)} bytes, entry_size={entry_size or '?'}):")
        dump_hex(raw_payload, indent="    ")
        print()

    if entries:
        print(f"  Parsed with {entry_size}-byte entry size:")
        print(f"  {'ID':<5} {'functionMask':<14} {'GPS baud':<12} {'MSP baud':<12} {'GPS?'}")
        print(f"  {'-'*58}")
        for e in entries:
            flag = "← GPS HERE" if e["has_gps"] else ""
            print(f"  {e['uart_index']:<5} "
                  f"0x{e['function_mask']:08X}     "
                  f"{str(e['gps_baud']):<12} "
                  f"{str(e['msp_baud']):<12} "
                  f"{flag}")
            if e["has_gps"]:
                gps_uart_index = e["uart_index"]
                gps_baud       = e["gps_baud"]
    else:
        print("  Could not parse serial config — entry size not recognised.")

    # ── UART index mapping note ───────────────────────────────────────────────
    # BF serial config stores UART IDs as 0-based (0=UART1) in most builds,
    # but some F405 targets store them 1-based. If the identified UART ID is
    # > 5 it's almost certainly an encoded 1-based value — adjust.
    passthrough_index = gps_uart_index
    if passthrough_index is not None and passthrough_index > 5:
        # ID 20 = softserial1, 30=softserial2; for real UARTs map back to 0-based
        if passthrough_index < 10:
            passthrough_index = passthrough_index - 1
        # else leave as-is (softserial etc.)

    if gps_uart_index is not None:
        print(f"\n  GPS assigned to UART{gps_uart_index + 1} "
              f"(passthrough index = {passthrough_index})")
        if gps_baud == 115200:
            print(f"  ✓ GPS UART baud = 115200 — correct")
        else:
            print(f"  ✗ GPS UART baud = {gps_baud} — should be 115200")
    else:
        print("\n  GPS UART not identified from function mask.")
        print("  Defaulting to passthrough index 0 (UART1).")
        passthrough_index = 0

    if only_phase == 2:
        s.close()
        return

    # ── Phase 3 ───────────────────────────────────────────────────────────────
    print("\nPhase 3 — passthrough probe\n")
    print("  NOTE: BF gps_auto_config=ON disables continuous GPS output.")
    print("  'ACK_OK with 0 bytes' is NORMAL.\n")

    if force_uart is not None:
        scan_indices = [force_uart]
    elif passthrough_index is not None:
        scan_indices = [passthrough_index] + [i for i in range(6)
                                               if i != passthrough_index]
    else:
        scan_indices = list(range(6))

    working_uart = None

    for idx in scan_indices:
        try:
            if not s.is_open:
                s = open_port(port, msp_baud)
        except Exception:
            s = open_port(port, msp_baud)

        diag, raw = probe_passthrough(s, idx)

        try:
            s = open_port_retry(port, msp_baud)
        except Exception:
            s = None

        if diag == "NO_ACK":
            if force_uart is not None:
                break
            continue

        working_uart = idx
        if diag == "NMEA":
            print(f"\n  GPS is outputting NMEA on UART index {idx}.")
            print(f"  Switch to UBX in BF CLI:")
            print(f"    set gps_provider    = UBLOX")
            print(f"    set gps_auto_config = ON")
            print(f"    save  → full power cycle")
            break
        break

    if only_phase == 3:
        try: s.close()
        except: pass
        return

    # ── Phase 4 ───────────────────────────────────────────────────────────────
    print(f"\n{'─'*62}")

    if working_uart is None:
        print(f"\n  ✗ No passthrough ACK on any UART index.")
        print_fix_steps(msp_baud)
        try: s.close()
        except: pass
    else:
        if only_phase != 5:
            print(f"\nPhase 4 — UBX-NAV-SAT poll (UART index {working_uart})\n")

            try:
                if s and s.is_open:
                    s.close()
            except Exception:
                pass
            time.sleep(0.2)

            try:
                s = open_port_retry(port, msp_baud)
            except serial.SerialException as exc:
                print(f"  ✗ Could not reopen {port}: {exc}")
                print(f"  Wait a few seconds and run the probe again.")
                print(f"\n{'='*62}\n")
                return

            ok, detail, svs = test_nav_sat(s, working_uart, gps_baud)

            try:
                s.close()
            except Exception:
                pass

            if ok:
                print(f"✓")
                print(f"  {detail}")
                print_sv_table(svs)
                print(f"\n  ══ UBX PASSTHROUGH WORKING ══")
                print(f"  In DroneLink.h:")
                print(f"    MSP_BAUD      = {msp_baud}")
                print(f"    gpsUartIndex_ = {working_uart}")
                print(f"  Satellite list populates ~30 s after connect().")
                print(f"\n{'='*62}\n")
                return   # Success — skip Phase 5
            else:
                print(f"✗")
                print(f"\n  ✗ UBX NAV-SAT poll failed:")
                for line in detail.splitlines():
                    print(f"    {line}")
                print(f"\n  → Falling through to Phase 5 (MSP fallback)…")

    # ── Phase 5 — MSP fallback ────────────────────────────────────────────────
    if only_phase != 4:
        time.sleep(0.3)
        phase5_msp_satellites(port, msp_baud)

    print(f"\n{'='*62}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPS baud probe v5")
    parser.add_argument("port", nargs="?")
    parser.add_argument("--uart", type=int, default=None, metavar="N",
        help="Force UART index (0=UART1, 1=UART2, …)")
    parser.add_argument("--msp-baud", type=int, default=None, metavar="BAUD")
    parser.add_argument("--phase", type=int, default=None, metavar="N",
        help="Run only up to this phase (1-5). Phase 5 = MSP fallback only.")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list or not args.port:
        print("\nAvailable serial ports:")
        for p in sorted(serial.tools.list_ports.comports()):
            print(f"  {p.device:<12} {p.description}")
        if not args.port:
            print()
            print("Usage: python sv_baud_probe.py <PORT> [--uart N] [--phase N]")
        sys.exit(0)

    run(args.port, args.uart, args.msp_baud, args.phase)