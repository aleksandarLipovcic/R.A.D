"""
radio_sniff.py — Project R.A.D.: what is the RadioMaster Pocket sending on USB?

Use when check_backend.py finds the Pocket but reports status=WAITING /
frames=0. Listens to the port twice (DTR on, then DTR off), counts raw
bytes and decodes CRSF frames in Python, independent of DroneBackend.pyd:

    python TestScripts/radio_sniff.py            (port COM4)
    python TestScripts/radio_sniff.py COM7 8     (port, seconds per pass)

Close the cockpit / check_backend.py first — only one program can hold the
COM port. Listen-only: nothing is written to the radio. Paste the output.
"""
import sys
import time
from collections import Counter

import serial

SYNC = {0xC8, 0xEA, 0xEC, 0xEE}
TYPES = {0x02: "GPS", 0x07: "VARIO", 0x08: "BATTERY", 0x09: "BARO_ALT",
         0x0B: "HEARTBEAT", 0x14: "LINK_STATS", 0x16: "RC_CHANNELS",
         0x1E: "ATTITUDE", 0x21: "FLIGHT_MODE", 0x29: "DEVICE_INFO",
         0x2B: "PARAM_ENTRY", 0x3A: "RADIO_ID"}


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _s8(b):
    return b - 256 if b >= 128 else b


def _rssi(b):
    # ELRS: signed int8 dBm; CRSF spec: unsigned magnitude
    return _s8(b) if b >= 128 else -b


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


def decode(ftype, pl):
    """Human-readable summary of the frames the cockpit uses."""
    if ftype == 0x14 and len(pl) >= 10:
        return (f"uplink RSSI {_rssi(pl[0])}/{_rssi(pl[1])} dBm  LQ {pl[2]}%  SNR {_s8(pl[3])} dB  "
                f"ant {pl[4] + 1}  rf {pl[5]}  | downlink RSSI {_rssi(pl[7])} dBm  LQ {pl[8]}%")
    if ftype == 0x1E and len(pl) >= 6:
        k = 57.29578 / 10000
        yaw = (_s16(pl[4], pl[5]) * k) % 360
        return (f"pitch {_s16(pl[0], pl[1]) * k:+.1f}°  roll {_s16(pl[2], pl[3]) * k:+.1f}°  "
                f"yaw {yaw:.1f}°")
    if ftype == 0x08 and len(pl) >= 8:
        return (f"{((pl[0] << 8) | pl[1]) / 10:.1f} V  {((pl[2] << 8) | pl[3]) / 10:.1f} A  "
                f"{(pl[4] << 16) | (pl[5] << 8) | pl[6]} mAh  {pl[7]}%")
    if ftype == 0x02 and len(pl) >= 15:
        lat = _s16(pl[0], pl[1]) * 65536 + ((pl[2] << 8) | pl[3])
        lon = _s16(pl[4], pl[5]) * 65536 + ((pl[6] << 8) | pl[7])
        alt = ((pl[12] << 8) | pl[13]) - 1000
        return (f"lat {lat / 1e7:.6f}  lon {lon / 1e7:.6f}  sats {pl[14]}  "
                f"ALTITUDE FIELD {alt} m")
    if ftype == 0x21:
        return "mode '" + bytes(pl).split(b"\0")[0].decode(errors="replace") + "'"
    if ftype in (0x07, 0x09) and len(pl) >= 2:
        return f"raw {_s16(pl[0], pl[1])}"
    return None


def parse(buf, last=None):
    """Return (Counter of frame types, crc_errors) for a raw byte buffer.
    If `last` is a dict, the last decoded summary per frame type goes in it."""
    types, bad, i = Counter(), 0, 0
    while i + 2 <= len(buf):
        if buf[i] not in SYNC:
            i += 1
            continue
        ln = buf[i + 1]
        if not 2 <= ln <= 62 or i + 2 + ln > len(buf):
            i += 1
            continue
        frame = buf[i + 2:i + 2 + ln]
        if crc8(frame[:-1]) == frame[-1]:
            name = TYPES.get(frame[0], f"0x{frame[0]:02X}")
            types[name] += 1
            if last is not None:
                text = decode(frame[0], frame[1:-1])
                if text:
                    last[name] = text
            i += 2 + ln
        else:
            bad += 1
            i += 1
    return types, bad


def listen(port, seconds, dtr):
    try:
        s = serial.Serial(port, 115200, timeout=0.05)
    except serial.SerialException as e:
        print(f"  cannot open {port}: {e}\n  (close the cockpit / check_backend.py and retry)")
        return None
    s.dtr, s.rts = dtr, True
    s.reset_input_buffer()
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        buf += s.read(512)
    s.close()
    return bytes(buf)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    best = 0
    for dtr in (True, False):
        print(f"\n{port}, DTR {'ON ' if dtr else 'OFF'}, listening {seconds:.0f} s ...")
        buf = listen(port, seconds, dtr)
        if buf is None:
            return
        last = {}
        types, bad = parse(buf, last)
        print(f"  bytes received : {len(buf)}  ({len(buf) / seconds:.0f} B/s)")
        print(f"  CRSF frames    : {sum(types.values())}   CRC errors: {bad}")
        for name, n in types.most_common():
            print(f"      {name:12s} {n:5d}  ({n / seconds:.1f} Hz)")
        if last:
            print("  last decoded   :")
            for name, text in last.items():
                print(f"      {name:12s} {text}")
        if buf:
            print("  first bytes    : " + " ".join(f"{b:02X}" for b in buf[:48]))
            printable = "".join(chr(b) if 32 <= b < 127 else "." for b in buf[:96])
            print(f"  as text        : {printable}")
        best = max(best, len(buf))

    print("\nVERDICT")
    if best == 0:
        print("  The Pocket sends NOTHING on USB. This is a radio setting, not the")
        print("  cockpit software. On the Pocket check: SYS -> Hardware -> USB-VCP =")
        print("  'Telem Mirror' (not None / CLI / LUA), then unplug + replug USB and")
        print("  choose 'USB Serial (VCP)' again.")
    else:
        print("  Bytes arrive. If there are no CRSF frames above, paste this output —")
        print("  the 'first bytes' line shows what format the Pocket is sending.")


if __name__ == "__main__":
    main()
