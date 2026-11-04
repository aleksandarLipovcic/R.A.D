"""
gps_sv_reader.py
================
Reads satellite info from Betaflight via MSP cmd 164 (MSP_GPS_SV_INFO).
Tested with XFlight Hobby F405 V3 / Betaflight 4.5.3 / NEO-M10.

Usage:
    pip install pyserial
    python gps_sv_reader.py COM4
    python gps_sv_reader.py COM4 --interval 2.0
    python gps_sv_reader.py COM4 --once
"""

import sys
import time
import struct
import argparse
import serial
import serial.tools.list_ports

# ── MSP constants ────────────────────────────────────────────────────────────

MSP_BAUD_CANDIDATES = [57600, 115200, 9600, 38400, 230400]

MSP_RAW_GPS    = 106
MSP_GPS_SV_INFO = 164

# BF quality index -> human label
# In BF's MSP_GPS_SV_INFO the quality byte matches u-blox signal quality flags:
# 0=idle/no signal, 1=searching, 2=acquired, 3=unusable,
# 4=code-lock, 5/6/7=carrier-lock (fully locked)
QUALITY = {
    0: "no signal",
    1: "searching",
    2: "acquired",
    3: "unusable",
    4: "locked",
    5: "fully locked",
    6: "fully locked",
    7: "fully locked",
}

# GNSS type from channel id — BF encodes GNSS in the upper nibble of ch byte
# when numCh > 16 (M10 reports 32 channels with gnss id in high nibble)
GNSS_NAME = {
    0: "GPS",
    1: "SBAS",
    2: "Galileo",
    3: "BeiDou",
    4: "IMES",
    5: "QZSS",
    6: "GLONASS",
}


# ── Serial / MSP helpers ─────────────────────────────────────────────────────

def open_port(port: str, baud: int) -> serial.Serial:
    s = serial.Serial(
        port=port, baudrate=baud,
        bytesize=8, parity="N", stopbits=1,
        timeout=0.1, write_timeout=0.5,
        rtscts=False, dsrdtr=False,
    )
    s.rts = False
    s.dtr = False
    time.sleep(0.1)
    s.reset_input_buffer()
    return s


def build_msp(cmd: int, payload: bytes = b"") -> bytes:
    n = len(payload)
    csum = n ^ cmd
    for b in payload:
        csum ^= b
    return bytes([0x24, 0x4D, 0x3C, n, cmd]) + payload + bytes([csum & 0xFF])


def read_msp(s: serial.Serial, timeout: float = 0.6) -> bytes:
    raw = bytearray()
    paylen = -1
    end = time.monotonic() + timeout
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


def msp_query(s: serial.Serial, cmd: int, timeout: float = 0.6):
    """Send MSP request, return payload bytes or None."""
    s.reset_input_buffer()
    s.write(build_msp(cmd))
    resp = read_msp(s, timeout)
    if len(resp) >= 6 and resp[0:3] == b"$M>":
        return resp[5: 5 + resp[3]]
    return None


def find_msp_baud(port: str, hint=None):
    candidates = list(MSP_BAUD_CANDIDATES)
    if hint:
        candidates = [hint] + [b for b in candidates if b != hint]
    for baud in candidates:
        print(f"  Trying {baud} baud... ", end="", flush=True)
        try:
            s = open_port(port, baud)
            # MSP_IDENT (cmd 101) — lightest alive check
            s.write(build_msp(101))
            r = read_msp(s, 0.4)
            if len(r) >= 6 and r[0:3] == b"$M>":
                print(f"OK")
                return baud, s
            s.close()
            print("no response")
        except Exception as e:
            print(f"error ({e})")
    return None, None


# ── MSP_GPS_SV_INFO parser ───────────────────────────────────────────────────

def parse_sv_info(payload: bytes):
    """
    BF 4.x MSP_GPS_SV_INFO layout (cmd 164):

        Byte 0       : numCh  (number of channels, typically 32 for M10)
        Bytes 1..end : numCh * 4 bytes per channel:
            [0] chn   — channel number  (upper nibble = GNSS id when numCh > 16)
            [1] svid  — satellite vehicle id
            [2] quality — signal quality index (0-7)
            [3] cno   — carrier-to-noise ratio dBHz (0 = not tracked)

    Source: src/main/msp/msp.c  MSP_GPS_SV_INFO handler in Betaflight.
    """
    if not payload or len(payload) < 1:
        return None, "empty payload"

    num_ch = payload[0]
    expected = 1 + num_ch * 4
    if len(payload) < expected:
        return None, f"payload too short: got {len(payload)}, need {expected}"

    # M10 reports 32 channels; GNSS id is packed in the upper nibble of chn
    gnss_in_high_nibble = (num_ch > 16)

    svs = []
    for i in range(num_ch):
        off = 1 + i * 4
        chn  = payload[off]
        svid = payload[off + 1]
        qual = payload[off + 2]
        cno  = payload[off + 3]

        if gnss_in_high_nibble:
            gnss_id = (chn >> 4) & 0x0F
            ch_num  = chn & 0x0F
        else:
            gnss_id = 0   # assume GPS for older 16-ch modules
            ch_num  = chn

        svs.append({
            "ch":      ch_num,
            "gnss_id": gnss_id,
            "gnss":    GNSS_NAME.get(gnss_id, f"GNSS{gnss_id}"),
            "svid":    svid,
            "quality": qual,
            "q_str":   QUALITY.get(qual, f"q={qual}"),
            "cno":     cno,
            "used":    qual >= 4,       # code-lock or better
            "tracked": cno > 0,
        })

    return svs, None


# ── MSP_RAW_GPS parser ───────────────────────────────────────────────────────

FIX_STR = {
    0: "NO FIX", 1: "DEAD RECK", 2: "2D", 3: "3D",
    4: "GPS+DR",  5: "TIME ONLY",
}

def parse_raw_gps(payload: bytes):
    if not payload or len(payload) < 16:
        return None
    fix     = payload[0]
    num_sat = payload[1]
    lat     = struct.unpack_from("<i", payload, 2)[0] / 1e7
    lon     = struct.unpack_from("<i", payload, 6)[0] / 1e7
    alt     = struct.unpack_from("<H", payload, 10)[0] / 100.0   # cm -> m
    speed   = struct.unpack_from("<H", payload, 12)[0] / 100.0   # cm/s -> m/s
    return {
        "fix":     fix,
        "fix_str": FIX_STR.get(fix, f"fix={fix}"),
        "num_sat": num_sat,
        "lat":     lat,
        "lon":     lon,
        "alt_m":   alt,
        "speed_ms": speed,
    }


# ── Display ──────────────────────────────────────────────────────────────────

BAR_WIDTH = 20   # max bar length in chars

def signal_bar(cno: int) -> str:
    """Scale 0-50 dBHz into a bar of BAR_WIDTH chars."""
    filled = min(int(cno / 50.0 * BAR_WIDTH), BAR_WIDTH)
    return "█" * filled + "░" * (BAR_WIDTH - filled)


def print_sv_table(svs: list, gps_info: dict = None):
    if gps_info:
        fix   = gps_info["fix_str"]
        nsats = gps_info["num_sat"]
        lat   = gps_info["lat"]
        lon   = gps_info["lon"]
        alt   = gps_info["alt_m"]
        spd   = gps_info["speed_ms"]
        print(f"  Fix: {fix}  |  Sats used: {nsats}  |  "
              f"Lat: {lat:.6f}  Lon: {lon:.6f}  "
              f"Alt: {alt:.1f} m  Speed: {spd:.2f} m/s")
        print()

    used    = [sv for sv in svs if sv["used"]]
    tracked = [sv for sv in svs if sv["tracked"] and not sv["used"]]
    other   = [sv for sv in svs if not sv["tracked"]]

    hdr = (f"  {'GNSS':<9} {'SvID':>5} {'CNO':>5}  "
           f"{'Signal':<{BAR_WIDTH+2}}  {'Quality':<12}  Status")
    sep = "  " + "-" * (len(hdr) - 2)
    print(hdr)
    print(sep)

    # sort: used first (by cno desc), then tracked, then rest
    ordered = (
        sorted(used,    key=lambda x: -x["cno"]) +
        sorted(tracked, key=lambda x: -x["cno"]) +
        sorted(other,   key=lambda x: (x["gnss_id"], x["svid"]))
    )

    for sv in ordered:
        bar    = signal_bar(sv["cno"])
        status = "USED" if sv["used"] else ("tracked" if sv["tracked"] else "")
        print(f"  {sv['gnss']:<9} {sv['svid']:>5} {sv['cno']:>4}   "
              f"[{bar}]  {sv['q_str']:<12}  {status}")

    print(sep)
    print(f"  Total: {len(svs)} channels  |  "
          f"{len(used)} used  |  {len(tracked)} tracked  |  "
          f"{len(other)} idle/searching")


# ── Main loop ────────────────────────────────────────────────────────────────

def run(port: str, msp_baud_hint=None, interval: float = 1.0, once: bool = False):
    print(f"\nConnecting to {port}...")
    msp_baud, s = find_msp_baud(port, msp_baud_hint)
    if s is None:
        print("Could not find MSP baud. Is BF Configurator closed?")
        sys.exit(1)

    print(f"  MSP baud: {msp_baud}\n")

    try:
        while True:
            t0 = time.monotonic()

            # GPS position (cmd 106)
            raw_gps_payload = msp_query(s, MSP_RAW_GPS)
            gps_info = parse_raw_gps(raw_gps_payload) if raw_gps_payload else None

            # Satellite info (cmd 164)
            sv_payload = msp_query(s, MSP_GPS_SV_INFO)
            if sv_payload is None:
                print("  MSP_GPS_SV_INFO: no response (check port / BF Configurator)")
            else:
                svs, err = parse_sv_info(sv_payload)
                if err:
                    print(f"  Parse error: {err}")
                    print(f"  Raw ({len(sv_payload)} bytes): "
                          + " ".join(f"{b:02X}" for b in sv_payload[:32]) + "...")
                else:
                    ts = time.strftime("%H:%M:%S")
                    print(f"\n[{ts}] ── Satellite view ──────────────────────────────────")
                    print_sv_table(svs, gps_info)

            if once:
                break

            # Wait for next poll cycle
            elapsed = time.monotonic() - t0
            sleep   = max(0.0, interval - elapsed)
            time.sleep(sleep)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        s.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPS satellite reader via MSP cmd 164")
    parser.add_argument("port",       nargs="?",        help="Serial port (e.g. COM4)")
    parser.add_argument("--msp-baud", type=int,         metavar="BAUD")
    parser.add_argument("--interval", type=float,       default=1.0,
                        help="Poll interval in seconds (default: 1.0)")
    parser.add_argument("--once",     action="store_true",
                        help="Query once and exit")
    parser.add_argument("--list",     action="store_true",
                        help="List available serial ports")
    args = parser.parse_args()

    if args.list or not args.port:
        print("\nAvailable serial ports:")
        for p in sorted(serial.tools.list_ports.comports()):
            print(f"  {p.device:<12} {p.description}")
        if not args.port:
            print("\nUsage: python gps_sv_reader.py <PORT> [--interval N] [--once]")
        sys.exit(0)

    run(args.port, args.msp_baud, args.interval, args.once)