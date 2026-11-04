"""
sv_passthrough_test.py
======================
Standalone diagnostic script for the MSP_SET_PASSTHROUGH → UBX-NAV-SAT cycle.

Run this script INSTEAD of your main GCS app while the FC is connected.
It tests each step of the protocol independently and prints exactly what
bytes are being sent and received, so you can pinpoint where the failure is.

Usage
-----
    python sv_passthrough_test.py COM5          # replace COM5 with your port
    python sv_passthrough_test.py COM5 --uart 1 # GPS on UART2 (default)
    python sv_passthrough_test.py COM5 --uart 0 # GPS on UART1

IMPORTANT: Close Betaflight Configurator and your main GCS before running.
Only one application can hold the COM port at a time.

What it tests
-------------
  Step 1  — Open serial port at 115200 8N1
  Step 2  — Send a normal MSP_STATUS (101) to confirm MSP is working
  Step 3  — Send MSP_SET_PASSTHROUGH with your GPS UART index
  Step 4  — Read and decode the MSP ACK
  Step 5  — Write the raw UBX-NAV-SAT poll (0x01/0x35)
  Step 6  — Read the raw UBX response, scan for 0xB5 0x62 preamble
  Step 7  — Decode the NAV-SAT payload and print the satellite table
  Step 8  — Close and reopen port (verify MSP recovers)
  Step 9  — Send MSP_STATUS again to confirm recovery
"""

import sys
import time
import struct
import argparse
import serial           # pip install pyserial
import serial.tools.list_ports


# ── Helpers ──────────────────────────────────────────────────────────────────

def hex_dump(data: bytes, label: str = "") -> None:
    if label:
        print(f"  [{label}]  {len(data)} bytes")
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        asc_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"    {i:04X}  {hex_part:<48}  {asc_part}")


def open_port(port: str, timeout_s: float = 0.05) -> serial.Serial:
    s = serial.Serial(
        port        = port,
        baudrate    = 115200,
        bytesize    = 8,
        parity      = "N",
        stopbits    = 1,
        timeout     = timeout_s,   # read timeout in seconds
        write_timeout = 0.5,
        rtscts      = False,
        dsrdtr      = False,
    )
    # Some USB-serial chips assert RTS/DTR by default which can reset the FC.
    s.rts = False
    s.dtr = False
    time.sleep(0.1)   # let the port settle
    s.reset_input_buffer()
    return s


# ── MSP helpers ──────────────────────────────────────────────────────────────

def build_msp_request(cmd: int, payload: bytes = b"") -> bytes:
    """Build an MSP v1 request frame."""
    length = len(payload)
    csum   = length ^ cmd
    for b in payload:
        csum ^= b
    return bytes([ord("$"), ord("M"), ord("<"), length, cmd]) + payload + bytes([csum])


def read_msp_response(s: serial.Serial, timeout_s: float = 0.15) -> bytes:
    """
    Read one complete MSP v1 response frame.
    Returns the raw frame bytes including preamble, or b'' on timeout/error.
    """
    raw    = bytearray()
    paylen = -1
    end    = time.monotonic() + timeout_s

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


# ── UBX helpers ──────────────────────────────────────────────────────────────

def ubx_checksum(data: bytes) -> tuple[int, int]:
    """Fletcher-8 checksum over bytes (excludes preamble 0xB5 0x62)."""
    ck_a = ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def build_ubx_nav_sat_poll() -> bytes:
    """UBX-NAV-SAT (class=0x01, id=0x35) poll frame — zero payload."""
    header  = bytes([0x01, 0x35, 0x00, 0x00])   # class, id, len_lo, len_hi
    ck_a, ck_b = ubx_checksum(header)
    return bytes([0xB5, 0x62]) + header + bytes([ck_a, ck_b])


def read_ubx_frame(s: serial.Serial, timeout_s: float = 0.5) -> bytes:
    """
    Read bytes until a complete UBX frame is assembled.
    Synchronises on the 0xB5 0x62 preamble — discards everything before it.
    Returns the complete frame starting at 0xB5, or b'' on timeout.
    """
    end = time.monotonic() + timeout_s

    # Phase 1: find preamble 0xB5 0x62
    prev = 0
    while time.monotonic() < end:
        b = s.read(1)
        if not b:
            continue
        curr = b[0]
        if prev == 0xB5 and curr == 0x62:
            break
        prev = curr
    else:
        return b""   # timeout before preamble

    # Phase 2: read 4-byte header (class, id, len_lo, len_hi)
    header = b""
    while len(header) < 4 and time.monotonic() < end:
        chunk = s.read(4 - len(header))
        header += chunk
    if len(header) < 4:
        return b""

    pay_len = struct.unpack_from("<H", header, 2)[0]

    if pay_len > 4096:
        print(f"  !! Implausible payload length {pay_len} — framing error?")
        return b""

    # Phase 3: read payload + 2 checksum bytes
    body = b""
    need = pay_len + 2
    while len(body) < need and time.monotonic() < end:
        chunk = s.read(need - len(body))
        body += chunk
    if len(body) < need:
        return b""

    return bytes([0xB5, 0x62]) + header + body


# ── NAV-SAT parser ────────────────────────────────────────────────────────────

GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "Galileo",
              3: "BeiDou", 5: "QZSS", 6: "GLONASS"}

QUALITY_NAMES = [
    "no signal", "searching", "acquired", "unusable",
    "code lock", "carrier lock", "carrier lock", "carrier lock",
]

def parse_nav_sat(payload: bytes) -> list[dict]:
    """
    Parse a UBX-NAV-SAT payload (everything after the 6-byte UBX header).
    Returns a list of dicts with keys: gnss, svid, cno, elev, azim, used, quality, status.
    """
    if len(payload) < 8:
        print(f"  !! Payload too short: {len(payload)} bytes (need ≥ 8)")
        return []

    num_svs = payload[5]   # byte [5] = numSvs  (NOT [4] which is version)
    print(f"  numSvs = {num_svs}  (version byte = {payload[4]})")

    if len(payload) < 8 + num_svs * 12:
        print(f"  !! Payload {len(payload)} bytes, expected {8 + num_svs * 12}")
        return []

    svs = []
    for i in range(num_svs):
        base    = 8 + i * 12
        gnss_id = payload[base + 0]
        svid    = payload[base + 1]
        cno     = payload[base + 2]
        elev    = struct.unpack_from("b", payload, base + 3)[0]
        azim    = struct.unpack_from("<h", payload, base + 4)[0]
        flags32 = struct.unpack_from("<I", payload, base + 8)[0]

        quality = flags32 & 0x07
        used    = bool(flags32 & 0x08)
        health  = (flags32 >> 4) & 0x03

        if used:
            status = "USED"
        elif quality >= 4:
            status = "tracked"
        elif quality >= 2:
            status = "acquired"
        elif quality == 1:
            status = "searching"
        else:
            status = "idle"

        svs.append({
            "gnss":    GNSS_NAMES.get(gnss_id, f"?({gnss_id})"),
            "svid":    svid,
            "cno":     cno,
            "elev":    elev,
            "azim":    azim,
            "used":    used,
            "quality": quality,
            "status":  status,
            "health":  health,
        })

    return svs


def print_sv_table(svs: list[dict]) -> None:
    if not svs:
        print("  (no satellites parsed)")
        return
    print()
    print(f"  {'GNSS':<8} {'SV':>4} {'dBHz':>5} {'Elev':>5} {'Azim':>5}  {'Status':<12} {'Q':>2} {'Health'}")
    print(f"  {'-'*8} {'-'*4} {'-'*5} {'-'*5} {'-'*5}  {'-'*12} {'-'*2} {'-'*6}")
    for sv in sorted(svs, key=lambda x: (not x["used"], -x["cno"])):
        mark = "★" if sv["used"] else " "
        print(f"  {mark}{sv['gnss']:<7} {sv['svid']:>4} {sv['cno']:>5} "
              f"{sv['elev']:>5}° {sv['azim']:>4}°  {sv['status']:<12} "
              f"{sv['quality']:>2}  {'OK' if sv['health'] == 1 else sv['health']}")
    used_count = sum(1 for sv in svs if sv["used"])
    print(f"\n  Total: {len(svs)} SVs visible, {used_count} used in fix")


# ── Main test sequence ────────────────────────────────────────────────────────

def run_test(port: str, gps_uart_idx: int) -> None:
    PASS = "✓"
    FAIL = "✗"

    print(f"\n{'='*60}")
    print(f"  MSP Passthrough Diagnostic  —  {port}  (GPS UART idx={gps_uart_idx})")
    print(f"{'='*60}\n")

    # ── Step 1: open port ─────────────────────────────────────────────────────
    print("Step 1: Open serial port")
    try:
        s = open_port(port)
        print(f"  {PASS} Opened {port} at 115200 8N1")
    except Exception as e:
        print(f"  {FAIL} Cannot open port: {e}")
        print("\n  Check that no other app (BF Configurator, your GCS) holds the port.")
        return

    try:
        # ── Step 2: MSP_STATUS sanity check ───────────────────────────────────
        print("\nStep 2: MSP_STATUS (101) — confirm MSP is alive")
        s.reset_input_buffer()
        req = build_msp_request(101)
        hex_dump(req, "sent")
        s.write(req)
        time.sleep(0.05)
        resp = read_msp_response(s, timeout_s=0.2)
        if len(resp) >= 6 and resp[0:3] == b"$M>":
            print(f"  {PASS} Got MSP response: {len(resp)} bytes, cmd={resp[4]}")
            hex_dump(resp, "received")
        else:
            print(f"  {FAIL} No valid MSP response (got {len(resp)} bytes)")
            hex_dump(resp, "received")
            print("\n  The FC is not responding to MSP. Check:")
            print("  • Is the FC powered and connected?")
            print("  • Is the correct COM port selected?")
            print("  • Does Betaflight Configurator show a working connection?")
            return

        # ── Step 3: MSP_SET_PASSTHROUGH ───────────────────────────────────────
        print(f"\nStep 3: MSP_SET_PASSTHROUGH (245) with GPS UART index = {gps_uart_idx}")
        s.reset_input_buffer()

        pt_payload = bytes([gps_uart_idx])
        pt_frame   = build_msp_request(245, pt_payload)
        hex_dump(pt_frame, "sent")
        s.write(pt_frame)

        # Wait for ACK — up to 200 ms
        time.sleep(0.1)
        ack_raw = s.read(64)   # read whatever arrived

        hex_dump(ack_raw, "received (ACK window)")

        if len(ack_raw) >= 6 and ack_raw[0:3] == b"$M>":
            print(f"  {PASS} Got MSP ACK ({len(ack_raw)} bytes) — FC is now in passthrough mode")
        elif len(ack_raw) == 0:
            print(f"  {FAIL} No ACK received at all!")
            print( "  This usually means the UART index is wrong or GPS is not")
            print( "  assigned in BF Configurator → Ports.")
            print(f"  Try --uart 0 (UART1) if you used --uart 1 (UART2), or vice versa.")
            # Don't return — still try to send UBX in case passthrough activated silently
        else:
            print(f"  ? Unexpected ACK content — proceeding anyway")

        # ── Step 4: write raw UBX-NAV-SAT poll ───────────────────────────────
        print("\nStep 4: Write raw UBX-NAV-SAT poll (0x01/0x35) — NO MSP framing")
        ubx_poll = build_ubx_nav_sat_poll()
        hex_dump(ubx_poll, "sent")
        s.write(ubx_poll)

        # Give the NEO-M10 time to respond via the FC passthrough
        print("  Waiting up to 1.0 s for UBX response…")
        time.sleep(0.05)   # let the first bytes arrive

        # ── Step 5: read UBX response ─────────────────────────────────────────
        print("\nStep 5: Read UBX response (scanning for 0xB5 0x62 preamble)")
        ubx_resp = read_ubx_frame(s, timeout_s=1.0)

        if not ubx_resp:
            print(f"  {FAIL} No UBX frame received within 1.0 s")
            print()
            print("  Raw bytes that did arrive (first 64):")
            leftover = s.read(64)
            if leftover:
                hex_dump(leftover, "leftover")
            else:
                print("  (nothing)")
            print()
            print("  Possible causes:")
            print("  1. Wrong GPS UART index  →  try --uart 0 or --uart 2")
            print("  2. GPS module not wired/powered  →  check hardware")
            print("  3. Betaflight GPS passthrough not enabled  →")
            print("     BF Configurator → Ports → your GPS UART → enable 'GPS' function")
            print("     (The GPS function must be active; passthrough works via that UART)")
            print("  4. NEO-M10 baud rate mismatch  →  check BF GPS baud setting")
        else:
            print(f"  {PASS} Got UBX frame: {len(ubx_resp)} bytes")
            hex_dump(ubx_resp, "received")

            cls = ubx_resp[2]
            mid = ubx_resp[3]
            pay_len = struct.unpack_from("<H", ubx_resp, 4)[0]
            print(f"\n  UBX class=0x{cls:02X}  id=0x{mid:02X}  payload={pay_len} bytes")

            if cls == 0x01 and mid == 0x35:
                print(f"  {PASS} Correct: UBX-NAV-SAT (0x01/0x35)")
            elif cls == 0x01 and mid == 0x30:
                print(f"  !! Got UBX-NAV-SVINFO (0x01/0x30) instead of NAV-SAT.")
                print( "     Your NEO-M10 firmware is very old. Update u-blox firmware,")
                print( "     or change buildNavSvInfoPoll() to use id=0x30 and update")
                print( "     the parser for the SVINFO layout.")
            elif cls == 0x05:
                if mid == 0x00:
                    print(f"  {FAIL} Got UBX-ACK-NAK — NEO-M10 rejected the NAV-SAT poll.")
                    print( "     This means the message is not supported on this firmware.")
                elif mid == 0x01:
                    print(f"  {FAIL} Got UBX-ACK-ACK — unexpected ACK instead of data response.")
            else:
                print(f"  ? Unexpected UBX message class=0x{cls:02X} id=0x{mid:02X}")
                print( "     The GPS may be sending periodic NAV messages instead.")
                print( "     Try increasing the read timeout.")

            # ── Step 6: parse ─────────────────────────────────────────────────
            if cls == 0x01 and mid in (0x35, 0x30):
                print("\nStep 6: Parse NAV-SAT payload")
                payload = ubx_resp[6 : 6 + pay_len]
                svs     = parse_nav_sat(payload)
                print_sv_table(svs)

    finally:
        # ── Step 7: close and reopen port (exit passthrough mode) ─────────────
        print("\nStep 7: Close port to exit passthrough mode")
        s.close()
        print(f"  {PASS} Port closed")

        time.sleep(0.2)

        print("\nStep 8: Reopen port and verify MSP recovers")
        try:
            s2 = open_port(port)
            s2.reset_input_buffer()
            req2 = build_msp_request(101)
            s2.write(req2)
            time.sleep(0.1)
            resp2 = read_msp_response(s2, timeout_s=0.3)
            if len(resp2) >= 6 and resp2[0:3] == b"$M>":
                print(f"  {PASS} MSP recovered after port reopen — all good")
            else:
                print(f"  {FAIL} MSP did not recover ({len(resp2)} bytes received)")
                print( "  Try unplugging and replugging the USB cable.")
            s2.close()
        except Exception as e:
            print(f"  {FAIL} Could not reopen port: {e}")

    print(f"\n{'='*60}\n")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Diagnose MSP_SET_PASSTHROUGH → UBX-NAV-SAT satellite list retrieval")
    parser.add_argument("port",
        nargs="?", default=None,
        help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--uart", type=int, default=1,
        metavar="N",
        help="BF serial port index for the GPS UART "
             "(UART1=0, UART2=1 default, UART3=2)")
    parser.add_argument("--list", action="store_true",
        help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list or args.port is None:
        print("\nAvailable serial ports:")
        ports = sorted(serial.tools.list_ports.comports())
        if ports:
            for p in ports:
                print(f"  {p.device:<12} {p.description}")
        else:
            print("  (none found)")
        print()
        if args.port is None and not args.list:
            print("Usage: python sv_passthrough_test.py <PORT> [--uart N]")
        sys.exit(0)

    run_test(args.port, args.uart)
