"""
test_lua_bridge.py -- runs Firmware/pocket/SCRIPTS/MIXES/radcmd.lua under Lua
5.2 with a mocked EdgeTX API and checks what it does:

  command encoder --frames--> radcmd.lua --mixer outputs--> checks
  radcmd.lua telemetry --bytes--> CRSF / RAD_STATUS decoders --> checks

The encoder and decoders are a Python reference implementation of the C++
UplinkProtocol / CrsfProtocol code. If the C++ test program is built
(DroneBackend/tests/test_uplink[.exe], or RAD_TEST_UPLINK points at it), the
C++ encoder and decoders are ALSO run on the same bytes and must agree
with the Python ones -- a three-way check Lua / C++ / Python.

Needs:  pip install lupa
Run:  pytest tests/unit/test_lua_bridge.py   or   python tests/unit/test_lua_bridge.py
The mock is only as good as our understanding of the EdgeTX API -- the bench
tests UT-UPL-A1..A5 in UPUTSTVO.md confirm it on the real Pocket.
"""
import json
import os
import subprocess
import sys
import tempfile

# lupa is imported lazily (see _lua52): importing it while pytest collects
# the test modules made the Tk tests of other modules fail on Windows
# ("invalid command name tcl_findLibrary" when a second Tk() is created).
def _lua52():
    try:
        from lupa import lua52
        return lua52
    except ImportError:                  # optional dependency
        return None

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCRIPT = os.path.join(REPO, "Firmware", "pocket", "SCRIPTS", "MIXES", "radcmd.lua")
TOOL = os.environ.get("RAD_TEST_UPLINK") or os.path.join(
    REPO, "DroneBackend", "tests", "test_uplink.exe" if os.name == "nt" else "test_uplink")

PASS = 0
FAIL = 0


def check(cond, what):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
        print("FAIL:", what)


def near(a, b, eps, what):
    check(abs(a - b) <= eps, f"{what}: {a} != {b} (+-{eps})")


class EdgeTxMock:
    """Minimal EdgeTX Lua API: getTime, getValue, serialRead/Write, getOutputValue."""

    def __init__(self):
        self.lua = _lua52().LuaRuntime(unpack_returned_tuples=True, encoding=None)
        self.tick = 1000
        self.values = {}
        self.rx = bytearray()          # laptop -> radio
        self.tx = bytearray()          # radio -> laptop
        self.outputs = [0] * 16
        g = self.lua.globals()
        g.getTime = lambda: self.tick
        g.getValue = self._get_value
        g.serialRead = self._serial_read
        g.serialWrite = self._serial_write
        g.getOutputValue = lambda i: self.outputs[int(i)]
        self.alerts = []
        g.playHaptic = lambda *a: self.alerts.append("haptic")
        g.playTone = lambda *a: self.alerts.append("tone")
        with open(SCRIPT, "rb") as f:
            self.mod = self.lua.execute(f.read())
        self.run_fn = self.mod[b"run"]
        self.mod[b"init"]()
        self.instr = 0

    def _get_value(self, name):
        key = name.decode() if isinstance(name, bytes) else name
        v = self.values.get(key, 0)
        if isinstance(v, dict):
            t = self.lua.table()
            for k, x in v.items():
                t[k.encode()] = x
            return t
        if isinstance(v, str):
            return v.encode()
        return v

    def _serial_read(self, n=64):
        # EdgeTX stops at '\n' -- emulate it so binary data split there is tested.
        n = int(n)
        out = bytearray()
        while self.rx and len(out) < n:
            b = self.rx.pop(0)
            out.append(b)
            if b == 0x0A:
                break
        return bytes(out)

    def _serial_write(self, s):
        self.tx += bytes(s)

    def run(self, ticks=3):
        self.tick += ticks
        return tuple(float(x) for x in self.run_fn())

    def count_instructions(self):
        """Lua VM instructions of one run() (hook every instruction)."""
        counter = self.lua.eval("""
            function(f)
              local n = 0
              debug.sethook(function() n = n + 1 end, "", 1)
              f()
              debug.sethook()
              return n
            end""")
        self.tick += 3
        return int(counter(self.run_fn))


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def rc_frame(ch):
    acc, bits, out = 0, 0, bytearray()
    for v in ch:
        acc |= (v & 0x7FF) << bits
        bits += 11
        while bits >= 8:
            out.append(acc & 0xFF); acc >>= 8; bits -= 8
    body = bytes([0x16]) + bytes(out)
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)])


def tool(*args):
    return subprocess.run([TOOL, *args], check=True, capture_output=True, text=True).stdout


# ── Python reference of the C++ encoder / decoders (UplinkProtocol.cpp,
#    CrsfProtocol.cpp, test_uplink.cpp cmdEncode/cmdDecode) ─────────────────
def _pct(p):
    return max(172, min(1811, round(992 + p * 8.192)))


def _us(u):
    return max(172, min(1811, round(992 + (u - 1500) * 1.6)))


def py_encode():
    """The fixed scenario of test_uplink.cpp cmdEncode(): 3 command frames."""
    out = b""
    for k in range(3):
        ch = [992] * 16
        ch[0], ch[1], ch[2], ch[3], ch[4] = _pct(20), _pct(-15), _us(1450), _pct(40), _pct(-60)
        ch[5], ch[6], ch[7] = 1811, 1811, 200 + k % 1600
        ch[8] = _pct(45)                       # gimbal pan
        out += rc_frame(ch)
    return out


def _parse(data):
    frames, crc_err, i = [], 0, 0
    while i + 2 <= len(data):
        if data[i] not in (0xC8, 0xEA, 0xEC, 0xEE) or not 2 <= data[i + 1] <= 62:
            i += 1
            continue
        n = data[i + 1] + 2
        if i + n > len(data):
            break
        body = data[i + 2:i + n - 1]
        if crc8(body) == data[i + n - 1]:
            frames.append((body[0], body[1:]))
            i += n
        else:
            crc_err += 1
            i += 1
    return frames, crc_err


def _u16(p, i):
    return (p[i] << 8) | p[i + 1]


def _s16(p, i):
    v = _u16(p, i)
    return v - 65536 if v >= 32768 else v


def _s32(p, i):
    v = (p[i] << 24) | (p[i + 1] << 16) | (p[i + 2] << 8) | p[i + 3]
    return v - (1 << 32) if v >= (1 << 31) else v


def _rssi(b):
    return b - 256 if b >= 128 else -b


def py_decode(data):
    """Same dict layout as `test_uplink decode` prints."""
    import math
    frames, crc_err = _parse(data)
    out = []
    for t, p in frames:
        d = {"type": t}
        if t == 0x1E and len(p) >= 6:
            yaw = math.degrees(_s16(p, 4) / 10000.0) % 360.0
            d.update(pitch=round(math.degrees(_s16(p, 0) / 10000.0), 3),
                     roll=round(math.degrees(_s16(p, 2) / 10000.0), 3), yaw=round(yaw, 3))
        elif t == 0x08 and len(p) >= 8:
            d.update(v=round(_u16(p, 0) / 10, 2), a=round(_u16(p, 2) / 10, 2),
                     mah=(p[4] << 16) | (p[5] << 8) | p[6], pct=p[7])
        elif t == 0x02 and len(p) >= 15:
            d.update(lat=round(_s32(p, 0) / 1e7, 7), lon=round(_s32(p, 4) / 1e7, 7),
                     kmh=round(_u16(p, 8) / 10, 1), hdg=round(_u16(p, 10) / 100, 2),
                     alt=_u16(p, 12) - 1000, sats=p[14])
        elif t == 0x14 and len(p) >= 10:
            pw = {0: 0, 1: 10, 2: 25, 3: 100, 4: 500, 5: 1000, 6: 2000, 7: 250, 8: 50}
            d.update(rssi1=_rssi(p[0]), lq=p[2], snr=p[3] - 256 if p[3] >= 128 else p[3],
                     txmw=pw.get(p[6], 0), dlq=p[8])
        elif t == 0x09 and len(p) >= 2:
            v = _u16(p, 0)
            d.update(alt_cm=(v & 0x7FFF) * 100 if v & 0x8000 else (v - 10000) * 10)
        elif t == 0x07 and len(p) >= 2:
            d.update(vario_cms=_s16(p, 0))
        elif t == 0x21 and p:
            d.update(fm=bytes(p).split(b"\x00")[0].decode(errors="replace"))
        elif t == 0x7E and len(p) >= 22:
            f = p[1]
            echo = _u16(p, 2)
            d.update(fresh=int(bool(f & 1)), gate=int(bool(f & 2)), stk=int(bool(f & 4)),
                     thr=int(bool(f & 8)), live=int(bool(f & 16)),
                     override=int(bool(f & 32)), angle=int(bool(f & 64)),
                     echo=-1 if echo == 0xFFFF else echo, rx=_u16(p, 4),
                     thr_stick=_s16(p, 9), thr_delta=_s16(p, 11), out_roll=_s16(p, 14),
                     out_pitch=_s16(p, 16), out_yaw=_s16(p, 18), out_tilt=_s16(p, 20),
                     age_ms=p[8] * 10, version=p[0])
            has_pan = p[0] >= 2 and len(p) >= 24
            d.update(has_pan=int(has_pan), out_pan=_s16(p, 22) if has_pan else 0)
        elif t == 0x16 and len(p) >= 22:
            acc = bits = 0
            ch = []
            for b in p[:22]:
                acc |= b << bits
                bits += 8
                while bits >= 11 and len(ch) < 16:
                    ch.append(acc & 0x7FF)
                    acc >>= 11
                    bits -= 11
            d.update(us=[round(1500 + (c - 992) * 0.625, 1) for c in ch])
        out.append(d)
    return {"crc_errors": crc_err, "frames": out}


def _same(a, b):
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, float) or isinstance(b, float):
        return abs(a - b) <= 2e-3
    return a == b


def have_cpp():
    return os.path.exists(TOOL)


def encode_frames(tmp):
    data = py_encode()
    if have_cpp():
        path = os.path.join(tmp, "cmd.bin")
        tool("encode", path)
        check(open(path, "rb").read() == data, "C++ and Python command encoders agree")
    return data


def decode_frames(data, tmp):
    dec = py_decode(data)
    if have_cpp():
        path = os.path.join(tmp, "tlm.bin")
        with open(path, "wb") as fh:
            fh.write(data)
        check(_same(json.loads(tool("decode", path)), dec), "C++ and Python decoders agree")
    return dec


def main():
    if _lua52() is None:
        sys.exit("pip install lupa")
    print("decoders:", "C++ + Python" if have_cpp() else "Python (C++ test_uplink not built)")
    tmp = tempfile.mkdtemp()
    frames = encode_frames(tmp)
    flen = len(frames) // 3
    f0, f1, f2 = frames[:flen], frames[flen:2 * flen], frames[2 * flen:]

    m = EdgeTxMock()
    m.values.update({"sd": -1024, "thr": 0, "RQly": 0, "ch6": 1024,
                     "ail": 0, "ele": 0, "rud": 0})

    # --- no laptop: everything neutral -------------------------------------
    out = m.run()
    check(out == (0, 0, 0, 0, 0, 0), f"idle outputs {out}")

    # --- fresh command, gate OFF: nothing applied except gimbal ------------
    m.rx += f0
    rol, pit, thr, yaw, tlt, pan = m.run()
    check((rol, pit, thr, yaw) == (0, 0, 0, 0), "gate off -> sticks/throttle zero")
    near(tlt, -0.60 * 1024, 2, "gimbal tilt follows laptop without gate")
    near(pan, 0.45 * 1024, 2, "gimbal pan follows laptop without gate")

    # --- gate ON: offsets applied and clamped ------------------------------
    m.values["sd"] = 1024
    m.rx += f1
    rol, pit, thr, yaw, tlt, pan = m.run()
    near(rol, 0.20 * 1024, 2, "roll offset 20 %")
    near(pit, -0.15 * 1024, 2, "pitch offset -15 %")
    check(yaw == 300, f"yaw 40 % clamped to MAX_STICK 300 (got {yaw})")
    # throttle target 1450 us = mixer -100, stick at 0 -> delta -100
    near(thr, -100, 2, "throttle delta = target - stick")

    m.values["thr"] = 300          # stick 1650 us: delta -400 -> clamp -200
    out = m.run()
    check(out[2] == -200, f"throttle delta clamped to THR_BAND (got {out[2]})")
    m.values["thr"] = -800         # stick ~1100 us: below THR_MIN_STICK
    out = m.run()
    check(out[2] == 0, "no laptop throttle with stick near idle")
    m.values["thr"] = 0

    # --- radio-side ANGLE gate: offsets only with the ANGLE switch on ------
    m.values["ch6"] = -1024                        # ACRO
    out = m.run()
    check(out[:4] == (0, 0, 0, 0), f"ANGLE switch off -> no offsets/throttle {out}")
    near(out[4], -0.60 * 1024, 2, "gimbal still follows without ANGLE")
    m.values["ch6"] = 1024
    out = m.run()
    check(out[0] != 0, "ANGLE back on -> offsets again")

    # --- pilot priority: stick deflection > 30 % stops the laptop ----------
    m.alerts.clear()
    m.values["ele"] = 500
    out = m.run()
    check((out[0], out[1], out[3]) == (0, 0, 0), f"pilot override -> laptop offsets 0 {out}")
    near(out[2], -100, 2, "pilot override keeps the laptop throttle (no height jump)")
    check("haptic" in m.alerts, "override start buzzes the radio")
    m.values["ele"] = 0
    out = m.run()
    check((out[0], out[1], out[3]) == (0, 0, 0), "override held after the pilot releases (0.3 s)")
    m.tx.clear()
    for _ in range(8):                              # let a RAD_STATUS go out
        m.run(0)
    st = [d for d in py_decode(bytes(m.tx))["frames"] if d["type"] == 0x7E]
    check(st and st[-1].get("override") == 1 and st[-1].get("angle") == 1,
          "RAD_STATUS reports override + ANGLE flags")
    out = m.run(28)                                 # 0.31 s after release, frame still fresh
    check(out[0] != 0 and out[3] != 0, f"offsets back 0.3 s after the pilot released {out}")
    m.tick += 101                                   # laptop now stale

    # --- same alive counter again is NOT fresh -----------------------------
    m.tick += 60                   # 600 ms
    m.rx += f1                     # replay: same counter as before
    out = m.run(0)
    check(out[:4] == (0, 0, 0, 0), f"replayed frame is stale {out}")
    near(out[4], -0.60 * 1024, 2, "gimbal holds last tilt when stale")
    near(out[5], 0.45 * 1024, 2, "gimbal holds last pan when stale")

    # --- new counter -> fresh again; then watchdog after 500 ms ------------
    m.rx += f2
    out = m.run()
    check(out[0] != 0, "new counter -> active again")
    m.tick += 49
    out = m.run(0)
    check(out[0] != 0, "still fresh at 490 ms")
    m.alerts.clear()
    m.tick += 2
    out = m.run(0)
    check(out[:4] == (0, 0, 0, 0), "watchdog neutral after 510 ms")
    check("haptic" in m.alerts and "tone" in m.alerts,
          "laptop dropped out while flying -> haptic + tone on the radio")
    near(out[5], 0.45 * 1024, 2, "gimbal pan held after watchdog")

    # --- corrupted frame is rejected ---------------------------------------
    bad = bytearray(f0)
    bad[10] ^= 0x55
    m.rx += bad
    out = m.run()
    check(out[:4] == (0, 0, 0, 0), "corrupted frame ignored")

    # --- '\n' inside binary frames (serialRead stops there) ----------------
    # ch1 = 0x30A puts a 0x0A byte at the start of the payload.
    nl = rc_frame([0x30A] + [992] * 4 + [1811, 172, 0x20A] + [992] * 8)
    check(0x0A in nl[3:], "test frame contains 0x0A")
    m.rx += nl + nl[:5]            # plus a partial frame
    out = m.run()
    near(out[0], (0x30A - 992) * 1.25, 1, "frame with 0x0A parsed (roll), counts as fresh")

    # --- telemetry re-encoding ---------------------------------------------
    m.values.update({
        "RQly": 100, "1RSS": -60, "2RSS": -62, "RSNR": 9, "ANT": 0, "RFMD": 6,
        "TPWR": 100, "TRSS": -55, "TQly": 100, "TSNR": 8,
        "Ptch": 0.1, "Roll": -0.2, "Yaw": 4.0,      # radians, yaw > pi
        "RxBt": 15.6, "Curr": 12.3, "Capa": 850, "Bat%": 72,
        "GPS": {"lat": 44.7722, "lon": 17.1910}, "GSpd": 21.5, "Hdg": 123.45,
        "GAlt": 160, "Sats": 9, "FM": "STAB", "BAlt": 12.3, "VSpd": 0.5,
    })
    m.outputs = [0] * 16
    m.outputs[2] = -400            # CH3 at 1300 us
    m.outputs[4] = 1024            # CH5 at 2012 us
    m.rx += f2
    m.tx.clear()
    for _ in range(30):
        m.run()
    dec = decode_frames(bytes(m.tx), tmp)
    check(dec["crc_errors"] == 0, "telemetry CRC clean")
    by = {}
    for fr in dec["frames"]:
        by.setdefault(fr["type"], []).append(fr)
    for t, name in [(0x1E, "attitude"), (0x08, "battery"), (0x02, "gps"), (0x14, "link"),
                    (0x16, "rc"), (0x21, "fm"), (0x09, "baro"), (0x07, "vario"), (0x7E, "status")]:
        check(t in by, f"{name} frame present")
    a = by[0x1E][-1]
    near(a["pitch"], 5.73, 0.02, "pitch deg")
    near(a["roll"], -11.46, 0.02, "roll deg")
    near(a["yaw"], 229.18, 0.05, "yaw deg (wrapped)")
    b = by[0x08][-1]
    near(b["v"], 15.6, 0.01, "voltage"); near(b["a"], 12.3, 0.01, "current")
    check(b["mah"] == 850 and b["pct"] == 72, "capacity / percent")
    g = by[0x02][-1]
    near(g["lat"], 44.7722, 1e-6, "lat"); near(g["lon"], 17.1910, 1e-6, "lon")
    near(g["kmh"], 21.5, 0.05, "ground speed"); near(g["hdg"], 123.45, 0.01, "heading")
    check(g["alt"] == 160 and g["sats"] == 9, "gps alt / sats")
    lk = by[0x14][-1]
    check(lk["rssi1"] == -60 and lk["lq"] == 100 and lk["txmw"] == 100 and lk["snr"] == 9,
          f"link stats {lk}")
    check(by[0x21][-1]["fm"] == "STAB", "flight mode text")
    check(by[0x09][-1]["alt_cm"] == 1230, "baro altitude cm")
    check(by[0x07][-1]["vario_cms"] == 50, "vario cm/s")
    rc = by[0x16][-1]["us"]
    near(rc[2], 1300, 1.0, "CH3 us from getOutputValue"); near(rc[4], 2012, 1.0, "CH5 us")
    st = by[0x7E][-1]
    check(st["echo"] == 202, f"status echoes alive counter of seq 2 (got {st['echo']})")
    check(st["live"] == 1 and st["gate"] == 1, "status flags")
    check(st["version"] == 2 and st["has_pan"] == 1, f"status v2 with pan ({st['version']})")
    near(st["out_pan"], 0.45 * 1024, 2, "status reports applied pan")

    # attitude dominates the schedule
    per_run = len(dec["frames"]) / 30      # baro slot writes BARO + VARIO
    check(2.0 <= per_run <= 2.2, f"TLM_PER_RUN frames per run ({per_run})")

    # --- drone telemetry lost: only link + status + rc go out ---------------
    m.values["RQly"] = 0
    m.tx.clear()
    for _ in range(30):
        m.run()
    types = {fr["type"] for fr in decode_frames(bytes(m.tx), tmp)["frames"]}
    check(types <= {0x14, 0x16, 0x7E}, f"no drone frames while RQly = 0 ({sorted(types)})")

    # --- cost per run --------------------------------------------------------
    m.values["RQly"] = 100
    costs = []
    for _ in range(14):
        m.rx += f2 if len(costs) % 2 else f1
        costs.append(m.count_instructions())
    print(f"Lua VM instructions per run(): min {min(costs)}, max {max(costs)}")

    print(f"{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


def test_lua_bridge():
    """pytest entry: radcmd.lua against the reference encoder/decoders."""
    import pytest
    if _lua52() is None:
        pytest.skip("lupa not installed (pip install lupa)")
    assert main() == 0


if __name__ == "__main__":
    sys.exit(main())
