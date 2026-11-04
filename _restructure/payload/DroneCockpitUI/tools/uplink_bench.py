"""
uplink_bench.py -- bench tool for the laptop -> Pocket -> drone command path
============================================================================
Standalone (pyserial only, no DroneBackend, no cockpit). Talks to radcmd.lua
on the Pocket exactly like CommandLink does, so every bench test of
UPUTSTVO.md can be run before the cockpit integration is built.

    pip install pyserial
    python uplink_bench.py --list
    python uplink_bench.py --port COM7                 # UT-UPL-A1 / A4: monitor
    python uplink_bench.py --port COM7 --tilt -50      # gimbal only
    python uplink_bench.py --port COM7 --sweep         # UT-UPL-A2 (PROPS OFF)
    python uplink_bench.py --port COM7 --latency --msp COM5   # UT-UPL-A3 (PROPS OFF)

--sweep and --latency send stick offsets with "sticks enable" set. They only
reach Betaflight while the pilot switch on the Pocket is ON. PROPELLERS OFF.
"""
import argparse
import math
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pip install pyserial")

# ── CRSF helpers (same format as CrsfProtocol / UplinkProtocol) ─────────────
SYNC = (0xC8, 0xEA, 0xEC, 0xEE)
NAMES = {0x02: "GPS", 0x07: "VARIO", 0x08: "BATTERY", 0x09: "BARO", 0x14: "LINK",
         0x16: "RC", 0x1E: "ATTITUDE", 0x21: "FLIGHT_MODE", 0x7E: "RAD_STATUS"}


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def pack11(ch):
    acc = bits = 0
    out = bytearray()
    for v in ch:
        acc |= (int(v) & 0x7FF) << bits
        bits += 11
        while bits >= 8:
            out.append(acc & 0xFF)
            acc >>= 8
            bits -= 8
    return bytes(out)


def unpack11(p):
    acc = bits = 0
    out = []
    for b in p[:22]:
        acc |= b << bits
        bits += 8
        while bits >= 11 and len(out) < 16:
            out.append(acc & 0x7FF)
            acc >>= 11
            bits -= 11
    return out


def pct(p):
    return max(172, min(1811, round(992 + p * 8.192)))


def us(u):
    return max(172, min(1811, round(992 + (u - 1500) * 1.6)))


def alive(seq):
    return 200 + seq % 1600


def command_frame(seq, roll=0.0, pitch=0.0, yaw=0.0, thr_us=1000.0, tilt=0.0,
                  sticks=False, throttle=False):
    ch = [992] * 16
    ch[0], ch[1], ch[2], ch[3], ch[4] = pct(roll), pct(pitch), us(thr_us), pct(yaw), pct(tilt)
    ch[5] = 1811 if sticks else 172
    ch[6] = 1811 if throttle else 172
    ch[7] = alive(seq)
    body = bytes([0x16]) + pack11(ch)
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)])


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= 2:
            if self.buf[0] not in SYNC or not 2 <= self.buf[1] <= 62:
                del self.buf[0]
                continue
            n = self.buf[1] + 2
            if len(self.buf) < n:
                break
            body = bytes(self.buf[2:n - 1])
            if crc8(body) == self.buf[n - 1]:
                out.append((body[0], body[1:]))
                del self.buf[:n]
            else:
                self.crc_errors += 1
                del self.buf[0]
        return out


def decode_status(p):
    if len(p) < 22:
        return None
    f = p[1]
    s16 = lambda i: struct.unpack(">h", p[i:i + 2])[0]
    return {"fresh": bool(f & 1), "gate": bool(f & 2), "sticks": bool(f & 4),
            "throttle": bool(f & 8), "tlm_live": bool(f & 16),
            "echo": struct.unpack(">H", p[2:4])[0], "rx": struct.unpack(">H", p[4:6])[0],
            "crc_err": struct.unpack(">H", p[6:8])[0], "age_ms": p[8] * 10,
            "thr_stick_us": 1500 + s16(9) / 2, "thr_delta_us": s16(11) / 2,
            "period_ms": p[13] * 10, "out_roll": s16(14), "out_pitch": s16(16),
            "out_yaw": s16(18), "out_tilt": s16(20)}


# ── MSP (Betaflight over USB) for the latency test ──────────────────────────
def msp_rc(port):
    port.reset_input_buffer()
    port.write(b"$M<\x00\x69\x69")                     # MSP_RC (105), size 0
    t0 = time.perf_counter()
    buf = bytearray()
    while time.perf_counter() - t0 < 0.05:
        buf += port.read(port.in_waiting or 1)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 5:
            size = buf[i + 3]
            if len(buf) >= i + 6 + size:
                data = bytes(buf[i + 5:i + 5 + size])
                return [struct.unpack("<H", data[k:k + 2])[0] for k in range(0, size, 2)]
    return None


# ── modes ───────────────────────────────────────────────────────────────────
def find_pocket():
    for p in list_ports.comports():
        name = f"{p.description} {p.manufacturer or ''} {p.product or ''}".lower()
        if any(h in name for h in ("edgetx", "radiomaster", "pocket", "stm32 virtual")):
            return p.device
    return None


class Bench:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.ser.dtr = True
        self.parser = Parser()
        self.seq = 0
        self.sent_at = {}
        self.counts = {}
        self.status = None
        self.last = {}
        self.rtts = []

    def send(self, **kw):
        a = alive(self.seq)
        self.sent_at[a] = time.perf_counter()
        self.ser.write(command_frame(self.seq, **kw))
        self.seq += 1

    def poll(self):
        for t, p in self.parser.feed(self.ser.read(self.ser.in_waiting or 0)):
            self.counts[t] = self.counts.get(t, 0) + 1
            self.last[t] = p
            if t == 0x7E:
                st = decode_status(p)
                if st and self.status and st["echo"] != self.status["echo"] \
                        and st["echo"] in self.sent_at:
                    self.rtts.append((time.perf_counter() - self.sent_at[st["echo"]]) * 1000)
                self.status = st

    def summary(self, dt):
        rates = "  ".join(f"{NAMES.get(t, hex(t))} {n / dt:.1f}/s"
                          for t, n in sorted(self.counts.items()))
        print(f"\n[{time.strftime('%H:%M:%S')}] frames: {rates or 'none'}"
              f"   crc errors {self.parser.crc_errors}")
        st = self.status
        if st:
            rtt = (f"RTT avg {sum(self.rtts) / len(self.rtts):.0f} ms max {max(self.rtts):.0f} ms"
                   if self.rtts else "RTT --")
            print(f"  bridge: fresh={st['fresh']} gate={st['gate']} sticks={st['sticks']}"
                  f" thr={st['throttle']} tlm_live={st['tlm_live']} script period {st['period_ms']} ms"
                  f"  cmd rx {st['rx']} crc {st['crc_err']}  {rtt}")
            print(f"  outputs: roll {st['out_roll']} pitch {st['out_pitch']} yaw {st['out_yaw']}"
                  f" tilt {st['out_tilt']}  thr stick {st['thr_stick_us']:.0f} us"
                  f" delta {st['thr_delta_us']:+.0f} us")
        else:
            print("  no RAD_STATUS yet: is USB-VCP = LUA and radcmd.lua enabled on this model?")
        if 0x16 in self.last:
            ch = unpack11(self.last[0x16])
            print("  radio out: " + "  ".join(f"CH{i + 1} {1500 + (c - 992) * 0.625:.0f}"
                                              for i, c in enumerate(ch[:8])))
        if 0x1E in self.last:
            p = self.last[0x1E]
            pi, ro, ya = (struct.unpack(">h", p[i:i + 2])[0] / 10000 * 57.2958 for i in (0, 2, 4))
            print(f"  attitude: pitch {pi:+.1f}  roll {ro:+.1f}  yaw {ya % 360:.1f} deg")
        if 0x08 in self.last:
            p = self.last[0x08]
            print(f"  battery: {struct.unpack('>H', p[0:2])[0] / 10:.1f} V"
                  f"  {struct.unpack('>H', p[2:4])[0] / 10:.1f} A  {p[7]} %")
        if 0x21 in self.last:
            fm = self.last[0x21].split(b"\x00")[0].decode(errors="replace")
            print(f"  flight mode: {fm}")
        if 0x14 in self.last:
            p = self.last[0x14]
            print(f"  link: LQ {p[2]} %  RSSI1 {-p[0] if p[0] < 128 else p[0] - 256} dBm  SNR {struct.unpack('b', p[3:4])[0]}")
        self.counts.clear()
        self.rtts.clear()


def run_monitor(b, args):
    t_sum = time.perf_counter()
    while True:
        b.send(tilt=args.tilt)
        b.poll()
        time.sleep(0.03)
        if time.perf_counter() - t_sum >= 1.0:
            b.summary(time.perf_counter() - t_sum)
            t_sum = time.perf_counter()


def run_sweep(b, args):
    print("SWEEP: roll/pitch +-20 % sine, 0.2 Hz. Pilot switch ON, PROPS OFF. Ctrl+C to stop.")
    t0 = t_sum = time.perf_counter()
    while True:
        t = time.perf_counter() - t0
        b.send(roll=20 * math.sin(2 * math.pi * 0.2 * t), pitch=20 * math.cos(2 * math.pi * 0.2 * t),
               tilt=args.tilt, sticks=True)
        b.poll()
        time.sleep(0.03)
        if time.perf_counter() - t_sum >= 1.0:
            b.summary(time.perf_counter() - t_sum)
            t_sum = time.perf_counter()


def run_latency(b, args):
    fc = serial.Serial(args.msp, 115200, timeout=0)
    print("LATENCY: roll 0 -> +30 % steps, Betaflight CH1 read over MSP. Pilot switch ON, PROPS OFF.")
    results = []
    for i in range(args.steps):
        t_end = time.perf_counter() + 1.0
        while time.perf_counter() < t_end:            # settle at 0
            b.send(sticks=True); b.poll(); time.sleep(0.03)
        base = msp_rc(fc)
        if not base:
            sys.exit("no MSP_RC answer -- is --msp the flight controller's port?")
        t_step = time.perf_counter()
        seen = None
        next_send = 0.0
        while time.perf_counter() - t_step < 1.0:
            if time.perf_counter() >= next_send:
                b.send(roll=30, sticks=True); b.poll()
                next_send = time.perf_counter() + 0.03
            rc = msp_rc(fc)
            if rc and rc[0] > base[0] + 75:          # +30 % = +150 us, half way
                seen = (time.perf_counter() - t_step) * 1000
                break
        results.append(seen)
        print(f"  step {i + 1}: {'%.0f ms' % seen if seen else 'NOT SEEN within 1 s'}")
    ok = [r for r in results if r]
    if ok:
        ok.sort()
        print(f"latency laptop -> Betaflight CH1: median {ok[len(ok) // 2]:.0f} ms,"
              f" max {ok[-1]:.0f} ms, {len(ok)}/{len(results)} steps seen"
              f"  (target < 100 ms, includes MSP polling ~5-10 ms)")
    for _ in range(20):
        b.send(); time.sleep(0.03)                    # leave it neutral


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--port", help="Pocket COM port (default: auto by USB name)")
    ap.add_argument("--tilt", type=float, default=0.0, help="gimbal tilt %% (-100..100)")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--latency", action="store_true")
    ap.add_argument("--msp", help="flight controller COM port for --latency")
    ap.add_argument("--steps", type=int, default=10)
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device:8} {p.description}  [{p.manufacturer or ''} {p.product or ''}]")
        return
    port = args.port or find_pocket()
    if not port:
        sys.exit("Pocket not found, use --port")
    b = Bench(port)
    print(f"Pocket on {port}. Ctrl+C to stop.")
    try:
        if args.latency:
            if not args.msp:
                sys.exit("--latency needs --msp <FC port>")
            run_latency(b, args)
        elif args.sweep:
            run_sweep(b, args)
        else:
            run_monitor(b, args)
    except KeyboardInterrupt:
        for _ in range(20):
            b.send(tilt=args.tilt)
            time.sleep(0.03)
        print("\nstopped, sent neutral frames")


if __name__ == "__main__":
    main()
