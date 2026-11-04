"""
sv_baud_probe.py  (v6)
======================
Diagnoses GPS passthrough issues on Betaflight FCs with u-blox GPS modules.

Changes from v5:
  - Phase 2: brute-forces ALL entry sizes 6-14 and dumps every parse so we can
    see the real UART/GPS assignment even when the standard size guess is wrong.
  - Phase 2b (NEW): MSP command scanner — probes cmd 106/119/130/164/166/167
    and raw-dumps any response so we know exactly what BF answers for GPS data.
  - Phase 3: always scans ALL 6 UART indices regardless of config, collecting
    results for all that ACK (not just the first).
  - Phase 4: tries UBX poll on every ACK'd UART, not just index 0.
  - Phase 5: tries MSP GPS commands 106 and 164, raw-dumps response so we can
    identify the correct command number and payload layout.

Usage
-----
    pip install pyserial
    python sv_baud_probe.py COM4
    python sv_baud_probe.py COM4 --uart 0
    python sv_baud_probe.py COM4 --phase 5
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
            print(f"\n    (port busy, retrying in {wait:.1f} s...)", end="", flush=True)
            time.sleep(wait)


def build_msp(cmd: int, payload: bytes = b"") -> bytes:
    n    = len(payload)
    csum = n ^ cmd
    for b in payload:
        csum ^= b
    return bytes([0x24, 0x4D, 0x3C, n, cmd]) + payload + bytes([csum & 0xFF])


def read_msp(s: serial.Serial, timeout: float = 0.6) -> bytes:
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


def msp_query(s: serial.Serial, cmd: int, payload: bytes = b"",
              timeout: float = 0.6):
    """Send MSP request, return payload bytes or None."""
    s.reset_input_buffer()
    s.write(build_msp(cmd, payload))
    resp = read_msp(s, timeout)
    if len(resp) >= 6 and resp[0:3] == b"$M>":
        return resp[5 : 5 + resp[3]]
    return None


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


def dump_hex(data: bytes, max_bytes: int = 256, indent: str = "    "):
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        h = " ".join(f"{b:02X}" for b in chunk)
        a = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{indent}{i:04X}  {h:<48}  {a}")


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — find MSP baud + BF version
# ─────────────────────────────────────────────────────────────────────────────

def find_msp_baud(port: str, hint):
    candidates = list(MSP_BAUD_CANDIDATES)
    if hint:
        candidates = [hint] + [b for b in candidates if b != hint]
    for baud in candidates:
        print(f"  Trying {baud} baud... ", end="", flush=True)
        try:
            s = open_port(port, baud)
            if msp_alive(s):
                print(f"OK  MSP alive at {baud}")
                return baud, s
            s.close()
            print("no response")
        except Exception as e:
            print(f"error ({e})")
    return None, None


def read_bf_version(s: serial.Serial) -> str:
    p = msp_query(s, 58)
    if p and len(p) >= 3 and any(b != 0 for b in p[:3]):
        return f"{p[0]}.{p[1]}.{p[2]}"
    p = msp_query(s, 2)
    if p:
        try:
            return "variant:" + p.decode("ascii", errors="replace").strip("\x00 ")
        except Exception:
            pass
    return "unknown"


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — BF serial config brute-force
# ─────────────────────────────────────────────────────────────────────────────

VALID_UART_IDS = set(range(10)) | {20, 21, 30, 31, 40, 50}


def _parse_entry(e: bytes):
    if len(e) < 7:
        return None
    uid = e[0]
    if uid not in VALID_UART_IDS:
        return None
    function_mask = struct.unpack_from("<I", e, 1)[0]
    msp_baud_idx  = e[5]
    gps_baud_idx  = e[6] if len(e) > 6 else 0
    return {
        "uart_index":    uid,
        "function_mask": function_mask,
        "msp_baud":      BAUD_INDEX.get(msp_baud_idx, f"idx={msp_baud_idx}"),
        "gps_baud":      BAUD_INDEX.get(gps_baud_idx, f"idx={gps_baud_idx}"),
        "has_gps":       bool(function_mask & FUNCTION_GPS)
                         or bool(function_mask & 0x0002),
    }


def brute_parse_serial_cfg(payload: bytes):
    results = []
    for size in range(6, 15):
        if len(payload) == 0 or len(payload) % size != 0:
            continue
        entries = []
        ok = True
        for i in range(0, len(payload), size):
            e = _parse_entry(payload[i : i + size])
            if e is None:
                ok = False
                break
            entries.append(e)
        if ok and entries:
            results.append((size, entries))
    return results


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2b — MSP command scanner
# ─────────────────────────────────────────────────────────────────────────────

GPS_MSP_CMDS = {
    106: "MSP_RAW_GPS",
    107: "MSP_COMP_GPS",
    119: "MSP_ANALOG",
    130: "MSP_STATUS_EX",
    164: "MSP_GPS_SV_INFO",
    166: "MSP_GPS_STATISTICS",
}


def scan_msp_gps_commands(s: serial.Serial):
    results = {}
    for cmd, name in GPS_MSP_CMDS.items():
        p = msp_query(s, cmd, timeout=0.6)
        if p is not None:
            results[cmd] = (name, p)
    return results


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
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        frame = read_ubx_frame(s, timeout=end - time.monotonic())
        if len(frame) < 10:
            continue
        if frame[2] == 0x05 and frame[3] in (0x00, 0x01):
            if frame[6] == expected_cls and frame[7] == expected_mid:
                return frame[3] == 0x01
    return None


NAV_SAT_POLL           = build_ubx(0x01, 0x35)
CFG_MSG_ENABLE_NAV_SAT = build_ubx(0x06, 0x01,
    bytes([0x01, 0x35, 0, 1, 0, 0, 0, 0]))
CFG_PRT_POLL_UART1     = build_ubx(0x06, 0x00, bytes([0x01]))
MON_VER_POLL           = build_ubx(0x0A, 0x04)


def parse_nav_sat(frame: bytes):
    if len(frame) < 10:
        return None, "frame too short"
    if frame[2] == 0x05 and frame[3] == 0x00:
        return None, "GPS returned UBX-ACK-NAK"
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
        })
    return svs, None


# ─────────────────────────────────────────────────────────────────────────────
# Phase 3 — passthrough probe all UART indices
# ─────────────────────────────────────────────────────────────────────────────

def probe_passthrough_all(port: str, msp_baud: int, force_uart=None):
    scan = [force_uart] if force_uart is not None else list(range(6))
    ack_indices = []

    for idx in scan:
        try:
            s = open_port_retry(port, msp_baud)
        except Exception as e:
            print(f"  UART index {idx}: port error ({e})")
            continue

        print(f"  UART index {idx}: ", end="", flush=True)
        if not enter_passthrough(s, idx):
            print("no ACK")
            s.close()
            time.sleep(0.3)
            continue

        print("ACK -- listening 1 s... ", end="", flush=True)
        collected = bytearray()
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            chunk = s.read(256)
            if chunk:
                collected += chunk
        print(f"{len(collected)} bytes", end="")

        if collected:
            has_ubx  = b"\xB5\x62" in collected
            has_nmea = any(t in collected for t in (b"$GP", b"$GN", b"$GL", b"$GA"))
            if has_ubx:
                print("  [UBX data]")
            elif has_nmea:
                print("  [NMEA data]")
            else:
                print("  [unknown data]")
                dump_hex(bytes(collected[:32]), indent="        ")
        else:
            print("  (silent -- normal with gps_auto_config=ON)")

        ack_indices.append(idx)
        s.close()
        time.sleep(0.4)

    return ack_indices


# ─────────────────────────────────────────────────────────────────────────────
# Phase 4 — UBX poll on each ACK'd UART
# ─────────────────────────────────────────────────────────────────────────────

def test_ubx_on_uart(port: str, msp_baud: int, uart_idx: int):
    try:
        s = open_port_retry(port, msp_baud)
    except serial.SerialException as exc:
        return False, f"port open failed: {exc}", None

    if not enter_passthrough(s, uart_idx):
        s.close()
        return False, "passthrough ACK not received", None

    time.sleep(0.05)
    s.reset_input_buffer()

    # MON-VER
    print(f"    MON-VER (GPS firmware)... ", end="", flush=True)
    s.write(MON_VER_POLL)
    ver_frame = read_ubx_frame(s, timeout=2.0)
    if ver_frame and len(ver_frame) >= 10 and ver_frame[2] == 0x0A and ver_frame[3] == 0x04:
        pay = ver_frame[6 : 6 + struct.unpack_from("<H", ver_frame, 4)[0]]
        sw  = pay[:30].rstrip(b"\x00").decode("ascii", errors="replace")
        hw  = pay[30:40].rstrip(b"\x00").decode("ascii", errors="replace") if len(pay) >= 40 else "?"
        print(f"SW={sw}  HW={hw}")
    else:
        print("no response")
    s.reset_input_buffer()

    # CFG-PRT
    print(f"    CFG-PRT (GPS baud)... ", end="", flush=True)
    s.write(CFG_PRT_POLL_UART1)
    prt = read_ubx_frame(s, timeout=1.5)
    if prt and len(prt) >= 24 and prt[2] == 0x06 and prt[3] == 0x00:
        gps_baud = struct.unpack_from("<I", prt, 14)[0]
        print(f"GPS UART baud = {gps_baud}")
    else:
        print("no response")
    s.reset_input_buffer()

    # CFG-MSG enable NAV-SAT
    print(f"    CFG-MSG enable NAV-SAT... ", end="", flush=True)
    s.write(CFG_MSG_ENABLE_NAV_SAT)
    ack = read_ubx_ack(s, 0x06, 0x01, timeout=1.0)
    print("ACK" if ack is True else "NAK" if ack is False else "timeout")
    s.reset_input_buffer()

    # NAV-SAT poll
    print(f"    NAV-SAT poll... ", end="", flush=True)
    s.write(NAV_SAT_POLL)
    resp = read_ubx_frame(s, timeout=4.0)

    s.close()
    time.sleep(0.4)

    if not resp:
        return False, "no UBX response to NAV-SAT poll", None

    svs, err = parse_nav_sat(resp)
    if err:
        raw_hex = " ".join(f"{b:02X}" for b in resp[:32])
        return False, f"parse error: {err}\n    raw: {raw_hex}", None

    return True, f"{len(svs)} satellites", svs


# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — MSP GPS fallback
# ─────────────────────────────────────────────────────────────────────────────

def phase5_msp(port: str, msp_baud: int):
    print(f"\nPhase 5 -- MSP GPS query (BF Configurator method)\n")

    try:
        s = open_port_retry(port, msp_baud)
    except serial.SerialException as exc:
        print(f"  port open failed: {exc}")
        return False

    if not msp_alive(s):
        print("  MSP not responding -- close BF Configurator and retry")
        s.close()
        return False

    print("  Scanning GPS MSP commands...\n")
    responded = scan_msp_gps_commands(s)

    if not responded:
        print("  No GPS MSP commands responded.")
        print()
        print("  IMPORTANT: This means BF is not answering MSP GPS queries.")
        print("  This is the root cause of your problem.")
        print()
        print("  Likely reason: GPS is not assigned in BF Ports tab.")
        print("  Fix:")
        print("    1. Open BF Configurator -> Ports tab")
        print("    2. Find UART3 or UART4 row (check F405 V3 pinout for GPS TX/RX)")
        print("    3. Set 'Sensor Input' dropdown to GPS")
        print("    4. Set baud to 115200")
        print("    5. Save & Reboot, then full power cycle")
        s.close()
        return False

    for cmd, (name, payload) in responded.items():
        print(f"  cmd {cmd:3d} ({name}) -> {len(payload)} bytes")
        if payload:
            dump_hex(payload, max_bytes=48, indent="      ")
        print()

    # Parse MSP_RAW_GPS
    if 106 in responded:
        p = responded[106][1]
        if len(p) >= 16:
            fix     = p[0]
            num_sat = p[1]
            lat     = struct.unpack_from("<i", p, 2)[0] / 1e7
            lon     = struct.unpack_from("<i", p, 6)[0] / 1e7
            alt     = struct.unpack_from("<H", p, 10)[0] / 100.0
            speed   = struct.unpack_from("<H", p, 12)[0] / 100.0
            fix_str = {0:"NO FIX", 1:"DEAD RECK", 2:"2D FIX", 3:"3D FIX",
                       4:"GPS+DR",  5:"TIME ONLY"}.get(fix, f"fix={fix}")
            print(f"  -- MSP_RAW_GPS parsed --")
            print(f"  Fix        : {fix_str}")
            print(f"  Satellites : {num_sat}")
            print(f"  Position   : {lat:.6f}, {lon:.6f}")
            print(f"  Altitude   : {alt:.1f} m")
            print(f"  Speed      : {speed:.2f} m/s")
            print()

    # Parse MSP_GPS_SV_INFO
    if 164 in responded:
        p = responded[164][1]
        svs = None
        if len(p) >= 1 and p[0] * 4 + 1 == len(p):
            num_ch, data = p[0], p[1:]
            svs = [(data[i], data[i+1], data[i+2], data[i+3])
                   for i in range(0, num_ch*4, 4)]
        elif len(p) % 4 == 0:
            data = p
            svs = [(data[i], data[i+1], data[i+2], data[i+3])
                   for i in range(0, len(data), 4)]

        if svs:
            print(f"  -- MSP_GPS_SV_INFO ({len(svs)} channels) --")
            print(f"  {'Ch':>3} {'SvID':>5} {'CNO':>5}  {'Quality':<12}  Signal")
            print(f"  {'-'*52}")
            for ch, svid, qual, cno in sorted(svs, key=lambda x: -x[3]):
                if cno == 0 and svid == 0:
                    continue
                bar    = "#" * min(cno // 4, 15)
                q_str  = QUALITY_NAMES.get(qual, f"q={qual}")
                status = "USED" if qual >= 4 else "tracked" if cno > 0 else ""
                print(f"  {ch:>3} {svid:>5} {cno:>4}   {q_str:<12}  {bar}  {status}")
            print()

    s.close()

    if 106 in responded:
        print(f"  == MSP GPS WORKING ==")
        print(f"  For your application -- use MSP (same as BF Configurator):")
        print(f"    MSP_BAUD = {msp_baud}")
        print(f"    cmd 106  = MSP_RAW_GPS     -> fix, sat count, coordinates")
        print(f"    cmd 164  = MSP_GPS_SV_INFO -> per-satellite CNO")
        print(f"    Interval: 1000 ms")
        return True

    return False


# ─────────────────────────────────────────────────────────────────────────────
# Print helpers
# ─────────────────────────────────────────────────────────────────────────────

def print_sv_table(svs: list):
    if not svs:
        print("    (no satellites)")
        return
    used    = [sv for sv in svs if sv["used"]]
    tracked = [sv for sv in svs if not sv["used"] and sv["cno"] > 0]
    print()
    print(f"    {'GNSS':<8} {'SvID':>5} {'CNO':>5} {'Elev':>5} {'Azim':>5}  "
          f"{'Quality':<12} {'Status'}")
    print(f"    {'-'*68}")
    for sv in sorted(svs, key=lambda x: (-x["cno"], x["gnss_id"], x["svid"])):
        status = "USED" if sv["used"] else "tracked" if sv["cno"] > 0 else "searching"
        print(f"    {sv['gnss']:<8} {sv['svid']:>5} {sv['cno']:>4}  "
              f"{sv['elev']:>4}  {sv['azim']:>4}   "
              f"{sv['q_str']:<12} {status}")
    print()
    print(f"    Total: {len(svs)} visible,  {len(used)} used,  {len(tracked)} tracked")


def print_fix_steps(msp_baud):
    print()
    print("-- Steps to fix --")
    print("  1. BF Configurator -> Ports tab: GPS UART -> baud 115200 -> Save")
    print("  2. CLI:")
    print("       set gps_auto_baud   = ON")
    print("       set gps_auto_config = ON")
    print("       set gps_provider    = UBLOX")
    print("       save")
    print("  3. FULL POWER CYCLE: unplug USB + battery, wait 10 s, reconnect.")
    print(f"  MSP_BAUD = {msp_baud}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def run(port, force_uart, msp_baud_hint, only_phase=None):
    print(f"\n{'='*62}")
    print(f"  GPS Baud Probe v6  --  {port}")
    print(f"{'='*62}\n")

    # Phase 1
    print("Phase 1 -- FC MSP baud\n")
    msp_baud, s = find_msp_baud(port, msp_baud_hint)
    if msp_baud is None:
        print("\n  No MSP response. Close BF Configurator and check port.")
        return

    bf_ver = read_bf_version(s)
    print(f"  BF version : {bf_ver}")
    print(f"  MSP baud   : {msp_baud}")

    if only_phase == 1:
        s.close(); return

    # Phase 2
    print("\nPhase 2 -- Serial port config (all entry sizes)\n")
    raw_payload = msp_query(s, 54, timeout=0.6) or b""

    gps_uart_index = None

    if raw_payload:
        print(f"  Raw payload ({len(raw_payload)} bytes):")
        dump_hex(raw_payload, indent="    ")
        print()

        all_parses = brute_parse_serial_cfg(raw_payload)
        if not all_parses:
            print("  No entry size produced a clean parse.")
        else:
            print(f"  Found {len(all_parses)} valid parse(s):\n")
            for size, entries in all_parses:
                print(f"  -- Entry size {size} bytes --")
                print(f"  {'ID':<5} {'functionMask':<14} {'GPS baud':<12} "
                      f"{'MSP baud':<12} {'GPS?'}")
                print(f"  {'-'*56}")
                for e in entries:
                    flag = "<-- GPS" if e["has_gps"] else ""
                    print(f"  {e['uart_index']:<5} "
                          f"0x{e['function_mask']:08X}     "
                          f"{str(e['gps_baud']):<12} "
                          f"{str(e['msp_baud']):<12} "
                          f"{flag}")
                    if e["has_gps"] and gps_uart_index is None:
                        gps_uart_index = e["uart_index"]
                print()

    # Phase 2b
    print("Phase 2b -- MSP GPS command scan\n")
    responded = scan_msp_gps_commands(s)
    if responded:
        print(f"  Commands that responded:")
        for cmd, (name, payload) in responded.items():
            hex_str = " ".join(f"{b:02X}" for b in payload[:16])
            ellipsis = "..." if len(payload) > 16 else ""
            print(f"    cmd {cmd:3d} ({name}) -> {len(payload)} bytes  [{hex_str}{ellipsis}]")
    else:
        print("  No GPS MSP commands responded.")
        print("  NOTE: If BF Configurator is open, close it and re-run.")

    s.close()
    time.sleep(0.3)

    if only_phase == 2:
        return

    # Phase 3
    print(f"\nPhase 3 -- passthrough probe (all UART indices)\n")
    ack_indices = probe_passthrough_all(port, msp_baud, force_uart)

    if not ack_indices:
        print("\n  No passthrough ACK on any UART index.")
        print_fix_steps(msp_baud)
        print(f"\n{'='*62}\n")
        return

    print(f"\n  ACK received on UART indices: {ack_indices}")

    if only_phase == 3:
        return

    # Phase 4
    print(f"\nPhase 4 -- UBX NAV-SAT poll on each ACK'd UART\n")

    ubx_success = False
    for idx in ack_indices:
        print(f"  Testing UART index {idx}:")
        ok, detail, svs = test_ubx_on_uart(port, msp_baud, idx)
        if ok:
            print(f"    OK -- {detail}")
            print_sv_table(svs)
            print(f"\n  == UBX PASSTHROUGH WORKING on UART index {idx} ==")
            print(f"    MSP_BAUD      = {msp_baud}")
            print(f"    gpsUartIndex_ = {idx}")
            ubx_success = True
            break
        else:
            print(f"    FAIL -- {detail}")
        print()

    if only_phase == 4:
        print(f"\n{'='*62}\n")
        return

    # Phase 5
    if not ubx_success:
        print(f"  UBX passthrough failed on all UARTs.")
        print(f"  Trying Phase 5 (MSP method)...")

    time.sleep(0.3)
    phase5_msp(port, msp_baud)

    print(f"\n{'='*62}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPS baud probe v6")
    parser.add_argument("port", nargs="?")
    parser.add_argument("--uart",     type=int, default=None, metavar="N")
    parser.add_argument("--msp-baud", type=int, default=None, metavar="BAUD")
    parser.add_argument("--phase",    type=int, default=None, metavar="N",
        help="Stop after this phase (1-5).")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list or not args.port:
        print("\nAvailable serial ports:")
        for p in sorted(serial.tools.list_ports.comports()):
            print(f"  {p.device:<12} {p.description}")
        if not args.port:
            print("\nUsage: python sv_baud_probe.py <PORT> [--uart N] [--phase N]")
        sys.exit(0)

    run(args.port, args.uart, args.msp_baud, args.phase)