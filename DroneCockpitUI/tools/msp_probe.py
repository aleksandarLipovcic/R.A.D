"""
msp_probe_fixed.py  —  Raw MSP diagnostic (fixed for BF 4.3+ byte layout)
==========================================================================
Fixes vs original probe:
  1. MSP_ANALOG extended voltage is at frame[13..14] (BF 4.3+), not [11..12].
     BF 4.3 added a uint16 'power' field at [11..12] which shifted the
     voltage field two bytes forward.
  2. Full byte-by-byte payload dump so you can see exactly what each field is.
  3. Sanity clamp on extended voltage: must be 2.0–60.0 V to be accepted.
  4. MSP_BATTERY_STATE diagnosis explains WHY BF returns 0-byte payload.
  5. RC link check uses correct 885 µs YAW threshold (ELRS failsafe value).

Usage:
    python msp_probe.py COM4
    python msp_probe.py COM4 --baud 57600
"""

import serial
import sys
import time
import argparse

# ── MSP command IDs ───────────────────────────────────────────────────────────
MSP_ANALOG        = 110
MSP_BATTERY_STATE = 242
MSP_RC            = 105
MSP_STATUS        = 101

# ── Colour helpers ────────────────────────────────────────────────────────────
GRN  = "\033[92m"
RED  = "\033[91m"
YEL  = "\033[93m"
CYN  = "\033[96m"
RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"

def ok(s):   return f"{GRN}✔  {s}{RST}"
def err(s):  return f"{RED}✗  {s}{RST}"
def warn(s): return f"{YEL}⚠  {s}{RST}"
def hdr(s):  return f"{BOLD}{CYN}{s}{RST}"
def dim(s):  return f"{DIM}{s}{RST}"

# =============================================================================
# Low-level MSP
# =============================================================================

def build_request(cmd: int) -> bytes:
    return bytes([ord('$'), ord('M'), ord('<'), 0, cmd, cmd])

def send_msp(port: serial.Serial, cmd: int, timeout_ms: int = 300) -> bytes:
    port.reset_input_buffer()
    port.write(build_request(cmd))

    deadline = time.monotonic() + timeout_ms / 1000.0

    def read_byte():
        while time.monotonic() < deadline:
            b = port.read(1)
            if b:
                return b[0]
        return None

    for _ in range(64):
        b = read_byte()
        if b is None:
            return b''
        if b != ord('$'):
            continue
        b = read_byte()
        if b != ord('M'):
            continue
        b = read_byte()
        if b == ord('>'):
            break
        if b == ord('!'):
            return b'ERROR'
    else:
        return b''

    payload_len = read_byte()
    cmd_byte    = read_byte()
    if payload_len is None or cmd_byte is None:
        return b''

    body = []
    for _ in range(payload_len + 1):  # +1 for checksum
        b = read_byte()
        if b is None:
            return b''
        body.append(b)

    return bytes([ord('$'), ord('M'), ord('>'), payload_len, cmd_byte] + body)

# =============================================================================
# Helpers
# =============================================================================

def ru16(buf, i):
    return buf[i] | (buf[i + 1] << 8)

def ri16(buf, i):
    v = ru16(buf, i)
    return v if v < 0x8000 else v - 0x10000

def ru32(buf, i):
    return buf[i] | (buf[i+1]<<8) | (buf[i+2]<<16) | (buf[i+3]<<24)

def print_section(title):
    print()
    print(hdr(f"{'─'*4}  {title}  {'─'*(62 - len(title))}"))

def print_kv(k, v, good=None, indent=2):
    pad = " " * indent
    vs = str(v)
    if good is True:
        vs = f"{GRN}{vs}{RST}"
    elif good is False:
        vs = f"{RED}{vs}{RST}"
    print(f"{pad}{k:<34} {vs}")

def print_raw_map(frame, start, definitions):
    """
    Print a colour-coded byte map of the payload.
    definitions: list of (label, start_offset_in_payload, byte_count, decoder_fn)
    start = index in frame where payload begins (always 5 for MSP v1)
    """
    print(f"  {dim('Byte map (payload offsets):')}")
    for label, off, size, decoder in definitions:
        abs_i = start + off
        if abs_i + size > len(frame):
            print(f"    [{off:>2}..{off+size-1:>2}]  {label:<22}  {RED}MISSING{RST}")
            continue
        raw_bytes = " ".join(f"{frame[abs_i + j]:02x}" for j in range(size))
        decoded   = decoder(frame, abs_i)
        print(f"    [{off:>2}..{off+size-1:>2}]  {label:<22}  {DIM}{raw_bytes}{RST}  →  {decoded}")

# =============================================================================
# MSP_ANALOG decoder  (BF 4.3+ layout, payload = 9 bytes)
# =============================================================================
#
# BF 4.3+ MSP_ANALOG payload (9 bytes):
#   [0]     uint8   vbat_legacy   0.1 V units
#   [1..2]  uint16  mAhDrawn
#   [3]     uint8   rssi          0-255 (0 on ELRS/CRSF)
#   [4..5]  int16   amperage      centiamps
#   [6..7]  uint16  power         centi-Watts  ← NEW in BF 4.3, shifts volt field
#   [8..9]  uint16  vbat_10mV     10 mV per LSB ← was at [6..7] in BF 4.1/4.2
#
# Full frame offsets (header = 5 bytes):
#   frame[5]        vbat_legacy
#   frame[6..7]     mAhDrawn
#   frame[8]        rssi
#   frame[9..10]    amperage
#   frame[11..12]   power (centi-Watts)   ← often misidentified as voltage
#   frame[13..14]   vbat_10mV             ← THE REAL VOLTAGE
#   frame[15]       checksum
#
# =============================================================================

def decode_analog_fixed(frame: bytes) -> dict:
    if len(frame) < 6:
        return {"error": "frame too short"}

    result = {
        "frame_len":   len(frame),
        "payload_len": frame[3],
        "raw_hex":     frame.hex(" "),
    }

    if len(frame) < 12:
        result["error"] = "payload too short for legacy fields"
        return result

    # ── Always-present fields ─────────────────────────────────────────────────
    result["vbat_legacy_raw"] = frame[5]
    result["vbat_legacy_V"]   = frame[5] / 10.0
    result["mAhDrawn"]        = ru16(frame, 6)
    result["rssi_raw"]        = frame[8]
    result["amperage_cA"]     = (frame[9] | frame[10] << 8)
    result["amperage_A"]      = result["amperage_cA"] / 100.0

    # ── CONFIRMED layout — hand-verified against live raw hex ────────────────
    #
    # Frame is 15 bytes: header(5) + payload(9) + checksum(1)
    # Payload bytes are at frame[5] through frame[13]. frame[14] = checksum.
    #
    # frame[5]        uint8   vbat_legacy   0.1V units        e.g. 0xa2 = 16.2V
    # frame[6..7]     uint16  mAh drawn     little-endian      e.g. 0x0007 = 7 mAh
    # frame[8]        uint8   rssi          0 on ELRS/CRSF
    # frame[9..10]    uint16  amperage      centiamps LE       e.g. 0x0000 = 0.00A
    # frame[11]       uint8   unknown       value ~0x3e=62
    # frame[12..13]   uint16  vbat_10mV     10mV per LSB LE   e.g. 0x0653=1619→16.19V
    # frame[14]       uint8   checksum
    #
    # Verified: round1 hex ...3e 00 53 06 a9
    #   frame[11]=0x3e=62, frame[12..13]=0x53 0x06 → LE=0x0653=1619 → 16.19V ✔
    # ─────────────────────────────────────────────────────────────────────────

    # Unknown byte at frame[11]
    if len(frame) >= 12:
        result["unknown_field_raw"]  = frame[11]
        result["unknown_field_note"] = "(single byte, purpose unknown)"

    # Extended voltage at frame[12..13] — CONFIRMED
    if len(frame) >= 14:
        v10mV = frame[12] | (frame[13] << 8)
        result["vbat_10mV_raw"]    = v10mV
        result["vbat_10mV_V"]      = v10mV / 100.0
        result["vbat_offset_used"] = "CONFIRMED: frame[12..13]"

    # Best voltage
    ext_v = result.get("vbat_10mV_V", 0.0)
    if 2.0 <= ext_v <= 60.0:
        result["best_voltage_V"]   = ext_v
        result["best_voltage_src"] = "extended 10mV — frame[12..13]"
    else:
        result["best_voltage_V"]   = result["vbat_legacy_V"]
        result["best_voltage_src"] = "legacy 0.1V — frame[5]"

    return result


def check_analog_fixed(frame: bytes, round_n: int, target_v: float, target_cells: int):
    d = decode_analog_fixed(frame) if frame and frame != b'ERROR' else {"error": str(frame)}

    print_section(f"MSP_ANALOG (110)  — round {round_n}")

    if "error" in d:
        print(err(d["error"]))
        return d

    print_kv("Frame length (bytes)",    d["frame_len"],   good=(d["frame_len"] >= 16))
    print_kv("Payload length (bytes)",  d["payload_len"], good=(d["payload_len"] >= 9))
    print_kv("Raw hex",                 d["raw_hex"])
    print()

    # Byte map — confirmed layout, hand-verified from raw hex
    print_raw_map(frame, 5, [
        ("vbat_legacy (0.1V)",        0, 1, lambda f,i: f"{f[i]/10.0:.1f} V"),
        ("mAhDrawn",                  1, 2, lambda f,i: f"{f[i]|(f[i+1]<<8)} mAh"),
        ("rssi",                      3, 1, lambda f,i: f"{f[i]}{'  (ELRS/CRSF normal)' if f[i]==0 else f'  raw={f[i]}'}"),
        ("amperage (cA, LE)",         4, 2, lambda f,i: f"{(f[i]|(f[i+1]<<8))/100.0:.2f} A"),
        ("unknown (1 byte)",          6, 1, lambda f,i: f"0x{f[i]:02x} = {f[i]}"),
        ("vbat_10mV ← CONFIRMED",     7, 2, lambda f,i: f"{f[i]|(f[i+1]<<8)} raw → {(f[i]|(f[i+1]<<8))/100.0:.2f} V"),
    ])
    print()

    # Results
    best_v = d.get("best_voltage_V", 0.0)
    tol    = target_v * 0.10   # 10% tolerance
    v_ok   = abs(best_v - target_v) <= tol

    print_kv("vbat LEGACY frame[5]  (0.1V res)",
             f"{d['vbat_legacy_raw']} raw → {d['vbat_legacy_V']:.1f} V",
             good=(d['vbat_legacy_V'] > 2.0))

    if "unknown_field_raw" in d:
        print_kv("unknown   frame[11]",
                 f"0x{d['unknown_field_raw']:02x} = {d['unknown_field_raw']}  {d['unknown_field_note']}")

    if "vbat_10mV_V" in d:
        ext_v = d["vbat_10mV_V"]
        print_kv("vbat_10mV frame[12..13] ✔ CONFIRMED",
                 f"{d['vbat_10mV_raw']} raw → {ext_v:.2f} V",
                 good=(2.0 <= ext_v <= 60.0))

    print()
    print_kv(f"★ BEST VOLTAGE ({d.get('best_voltage_src','?')})",
             f"{best_v:.2f} V", good=v_ok)

    if target_cells > 0 and best_v > 2.0:
        cell_v = best_v / target_cells
        cell_ok = 3.40 <= cell_v <= 4.25
        print_kv(f"  Per-cell ({target_cells}S)",
                 f"{cell_v:.3f} V/cell", good=cell_ok)
        if not cell_ok:
            if cell_v < 3.40:
                print(err(f"  → Cell voltage critically low! Charge your battery."))
            elif cell_v > 4.25:
                print(err(f"  → Cell voltage too high — wrong cell count? Try {int(best_v//4.2)+1}S"))

    if d["rssi_raw"] == 0:
        print(ok("RSSI = 0  (correct for ELRS/CRSF — not an error)"))

    return d


# =============================================================================
# MSP_BATTERY_STATE decoder
# =============================================================================

def decode_battery_state(frame: bytes) -> dict:
    STATES = {0:"INIT", 1:"OK", 2:"WARNING", 3:"CRITICAL", 4:"NOT_PRESENT"}
    if len(frame) < 6:
        return {"error": "frame too short"}
    result = {
        "frame_len":   len(frame),
        "payload_len": frame[3],
        "raw_hex":     frame.hex(" "),
    }
    if len(frame) >= 15:
        result["cellCount"]     = frame[5]
        result["capacity_mAh"]  = ru16(frame, 6)
        result["vlegacy_raw"]   = frame[8]
        result["vlegacy_V"]     = frame[8] / 10.0
        result["mAhDrawn"]      = ru16(frame, 9)
        result["amperage_cA"]   = ri16(frame, 11)
        result["amperage_A"]    = ri16(frame, 11) / 100.0
        result["battState_raw"] = frame[13]
        result["battState_str"] = STATES.get(frame[13], f"UNKNOWN({frame[13]})")
    if len(frame) >= 17:
        v10 = ru16(frame, 14)
        result["v10mV_raw"] = v10
        result["v10mV_V"]   = v10 / 100.0
    return result


def check_battery_state(frame: bytes, round_n: int):
    d = decode_battery_state(frame) if frame and frame != b'ERROR' else {"error": str(frame)}

    print_section(f"MSP_BATTERY_STATE (242)  — round {round_n}")

    if "error" in d:
        print(err(d["error"]))
        return d

    print_kv("Frame length (bytes)",   d["frame_len"],   good=(d["frame_len"] >= 15))
    print_kv("Payload length (bytes)", d["payload_len"], good=(d["payload_len"] >= 9))
    print_kv("Raw hex",                d["raw_hex"])

    if d["payload_len"] == 0:
        print()
        print(err("BF returned EMPTY payload for MSP_BATTERY_STATE."))
        print(err("This means Betaflight has no battery data to report."))
        print()
        print(f"  {YEL}Run these commands in the Betaflight CLI:{RST}")
        print(f"  {CYN}  get battery_meter_type{RST}   → must be ADC (not NONE)")
        print(f"  {CYN}  get vbat_scale{RST}           → must be ~110 (not 0)")
        print(f"  {CYN}  get bat_capacity{RST}         → set to your pack mAh")
        print()
        print(f"  {YEL}If meter_type is already ADC, try:{RST}")
        print(f"  {CYN}  set battery_meter_type = ADC{RST}")
        print(f"  {CYN}  set vbat_scale = 110{RST}")
        print(f"  {CYN}  set bat_capacity = 5200{RST}   ← your 2×2S 5200mAh in series")
        print(f"  {CYN}  save{RST}")
        print(f"  Then power-cycle the FC with battery plugged in and re-run probe.")
        return d

    if "cellCount" in d:
        print_kv("Cell count",    d["cellCount"],      good=(d["cellCount"] > 0))
        print_kv("Capacity mAh",  d["capacity_mAh"],   good=(d["capacity_mAh"] > 0))
        print_kv("mAh drawn",     d["mAhDrawn"])
        print_kv("Amperage",      f"{d['amperage_A']:.2f} A")
        print_kv("Battery state",
                 f"{d['battState_raw']} → {d['battState_str']}",
                 good=(d["battState_str"] in ("OK", "WARNING")))
        if "v10mV_V" in d:
            print_kv("Voltage (10mV)", f"{d['v10mV_raw']} raw → {d['v10mV_V']:.2f} V",
                     good=(d["v10mV_V"] > 2.0))

    return d


# =============================================================================
# MSP_RC decoder
# =============================================================================

def decode_rc(frame: bytes) -> dict:
    if len(frame) < 6:
        return {"error": "frame too short"}
    payload_len = frame[3]
    ch_count    = payload_len // 2
    channels    = []
    for i in range(ch_count):
        base = 5 + i * 2
        if base + 1 < len(frame):
            channels.append(ru16(frame, base))
    names = ["ROLL","PITCH","THRO","YAW","ARM",
             "AUX1","AUX2","AUX3","AUX4","AUX5",
             "AUX6","AUX7","AUX8","AUX9","AUX10","AUX11"]
    ch_named = {names[i] if i < len(names) else f"CH{i+1}": v
                for i, v in enumerate(channels)}
    return {
        "frame_len":  len(frame),
        "payload_len": payload_len,
        "ch_count":   ch_count,
        "channels":   ch_named,
        "raw_hex":    frame.hex(" "),
    }


def check_rc(frame: bytes, round_n: int):
    d = decode_rc(frame) if frame and frame != b'ERROR' else {"error": str(frame)}

    print_section(f"MSP_RC (105)  — round {round_n}")

    if "error" in d:
        print(err(d["error"]))
        return d

    print_kv("Frame length (bytes)", d["frame_len"],  good=(d["frame_len"] >= 12))
    print_kv("Channel count",        d["ch_count"],   good=(d["ch_count"] >= 4))

    if d["ch_count"] == 0:
        print(err("Zero channels — UART Serial Rx not enabled in BF, or frame purged"))
        return d

    # Check for active link: any of ROLL/PITCH/THRO in 900-2100
    # YAW at 885 is ELRS failsafe trim — not a link failure
    sticks = {k: d["channels"].get(k, 0) for k in ["ROLL","PITCH","THRO","YAW"]}
    link_active = any(900 <= v <= 2100 for v in sticks.values())

    print()
    for ch, val in d["channels"].items():
        in_range = 900 <= val <= 2100
        note = ""
        if ch == "YAW" and val == 885:
            note = f"  {YEL}← ELRS failsafe trim (normal, not a link failure){RST}"
        elif ch == "ARM" and val >= 1800:
            note = f"  {GRN}← ARMED{RST}"
        elif ch == "ARM" and val < 1800:
            note = f"  {DIM}← DISARMED  ({val} < 1800 arm threshold){RST}"
        print_kv(f"  {ch}", f"{val}{note}", good=(in_range or (ch == "YAW" and val == 885)))

    print()
    if link_active:
        print(ok(f"RC LINK ACTIVE — {d['ch_count']} channels  (CRSF/ELRS)"))
        print(f"  {DIM}YAW=885 is ELRS default trim, not failsafe.{RST}")
        print(f"  {DIM}ARM=1000 = disarmed (correct on ground).{RST}")
    else:
        print(err("All sticks out of range — TX off or genuine failsafe"))

    return d


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="MSP fixed diagnostic probe")
    parser.add_argument("port")
    parser.add_argument("--baud", "-b",  type=int,   default=57600)
    parser.add_argument("--rounds","-n", type=int,   default=5)
    parser.add_argument("--delay", "-d", type=float, default=0.25)
    # Target battery for sanity checks
    parser.add_argument("--cells",  type=int,   default=4,    help="Expected cell count (default 4 for 4S)")
    parser.add_argument("--target-v", type=float, default=14.8, help="Expected pack voltage (default 14.8 for 4S)")
    args = parser.parse_args()

    print(hdr(f"\nMSP Probe (fixed)  —  {args.port} @ {args.baud} baud"))
    print(f"Target battery: {args.cells}S  {args.target_v:.1f}V nominal")
    print(f"Rounds: {args.rounds}   Delay: {args.delay}s\n")

    try:
        port = serial.Serial(
            args.port,
            baudrate = args.baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = 0.01,
        )
    except serial.SerialException as e:
        print(err(f"Cannot open {args.port}: {e}"))
        sys.exit(1)

    time.sleep(0.1)

    voltage_readings  = []
    rc_ch_counts      = []
    cell_counts       = []
    batt_states       = []

    for rnd in range(1, args.rounds + 1):

        # MSP_ANALOG
        frame = send_msp(port, MSP_ANALOG)
        d_a   = check_analog_fixed(frame, rnd, args.target_v, args.cells)
        voltage_readings.append(d_a.get("best_voltage_V", 0.0))
        time.sleep(0.02)

        # MSP_BATTERY_STATE
        frame = send_msp(port, MSP_BATTERY_STATE)
        d_b   = check_battery_state(frame, rnd)
        cell_counts.append(d_b.get("cellCount", 0))
        batt_states.append(d_b.get("battState_str", "?"))
        time.sleep(0.02)

        # MSP_RC
        frame = send_msp(port, MSP_RC)
        d_r   = check_rc(frame, rnd)
        rc_ch_counts.append(d_r.get("ch_count", 0))
        time.sleep(args.delay)

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print(hdr("═" * 68))
    print(hdr("  SUMMARY"))
    print(hdr("═" * 68))

    avg_v = sum(voltage_readings) / len(voltage_readings) if voltage_readings else 0
    tol   = args.target_v * 0.10
    v_ok  = abs(avg_v - args.target_v) <= tol
    print_kv("Average voltage (best field)",
             f"{avg_v:.2f} V  (target {args.target_v}V ±10%)",
             good=v_ok)
    if v_ok and avg_v > 2.0:
        cv = avg_v / args.cells
        print_kv(f"  Per cell ({args.cells}S)", f"{cv:.3f} V/cell",
                 good=(3.40 <= cv <= 4.25))
    elif avg_v < 2.0:
        print(err("  Voltage still 0V — run the BF CLI commands shown above"))

    avg_cells = sum(cell_counts) / len(cell_counts) if cell_counts else 0
    print_kv("Average cell count (from BATTERY_STATE)",
             f"{avg_cells:.1f}  (readings: {cell_counts})",
             good=(avg_cells == args.cells))
    if avg_cells == 0:
        print(warn("  → Cell count 0: BF battery_meter_type is NONE or bat_capacity=0"))

    avg_rc = sum(rc_ch_counts) / len(rc_ch_counts) if rc_ch_counts else 0
    print_kv("Average RC channel count",
             f"{avg_rc:.1f}  (readings: {rc_ch_counts})",
             good=(avg_rc >= 4))

    print_kv("Battery states seen", ", ".join(set(batt_states)))

    print()
    print(hdr("  NEXT STEPS"))
    if not v_ok or avg_v < 2.0:
        print(f"  {RED}1. Fix voltage — apply BF CLI commands above and recompile C++ with:{RST}")
        print(f"     parseAnalog(): read vbat_10mV from frame[13..14] (not [11..12])")
    else:
        print(f"  {GRN}1. Voltage OK — update parseAnalog() to use frame[13..14] offset.{RST}")

    if avg_cells == 0:
        print(f"  {RED}2. Fix BATTERY_STATE — set bat_capacity and battery_meter_type in BF CLI{RST}")
    else:
        print(f"  {GRN}2. Battery state OK{RST}")

    if avg_rc >= 4:
        print(f"  {GRN}3. RC link OK — 16 channels active. YAW=885 is normal ELRS trim.{RST}")
    else:
        print(f"  {RED}3. Fix RC — enable Serial Rx on ELRS UART in BF Ports tab{RST}")

    print()
    port.close()


if __name__ == "__main__":
    main()