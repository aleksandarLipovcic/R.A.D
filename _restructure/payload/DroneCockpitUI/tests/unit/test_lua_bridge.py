"""
test_lua_bridge.py -- runs pocket/SCRIPTS/MIXES/radcmd.lua under Lua 5.2 with
a mocked EdgeTX API and checks it against the C++ side (test_uplink binary):

  C++ CommandLink encoder --frames--> radcmd.lua --mixer outputs--> checks
  radcmd.lua telemetry --bytes--> C++ CrsfProtocol/Uplink decoders --> checks

Needs:  pip install lupa  and the test_uplink binary from DroneBackend/tests
(see test_uplink.cpp; or point RAD_TEST_UPLINK at it). Without them the
pytest test is skipped.
Run:  pytest tests/unit/test_lua_bridge.py   or   python tests/unit/test_lua_bridge.py
The mock is only as good as our understanding of the EdgeTX API -- the bench
tests UT-UPL-A1..A5 in UPUTSTVO.md confirm it on the real Pocket.
"""
import json
import os
import subprocess
import sys
import tempfile

try:
    from lupa import lua52
except ImportError:                      # optional dependency
    lua52 = None

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
        self.lua = lua52.LuaRuntime(unpack_returned_tuples=True, encoding=None)
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


def main():
    if lua52 is None:
        sys.exit("pip install lupa")
    if not os.path.exists(TOOL):
        sys.exit(f"build {TOOL} first (see DroneBackend/tests/test_uplink.cpp)")
    tmp = tempfile.mkdtemp()
    cmd_path = os.path.join(tmp, "cmd.bin")
    tool("encode", cmd_path)
    frames = open(cmd_path, "rb").read()
    flen = len(frames) // 3
    f0, f1, f2 = frames[:flen], frames[flen:2 * flen], frames[2 * flen:]

    m = EdgeTxMock()
    m.values.update({"sd": -1024, "thr": 0, "RQly": 0})

    # --- no laptop: neutral, Lnk -1024 -------------------------------------
    out = m.run()
    check(out == (0, 0, 0, 0, 0, -1024), f"idle outputs {out}")

    # --- fresh command, gate OFF: nothing applied except gimbal ------------
    m.rx += f0
    rol, pit, thr, yaw, tlt, lnk = m.run()
    check((rol, pit, thr, yaw) == (0, 0, 0, 0), "gate off -> sticks/throttle zero")
    check(lnk == 0, f"gate off, fresh -> Lnk 0 (got {lnk})")
    near(tlt, -0.60 * 1024, 2, "gimbal follows laptop without gate")

    # --- gate ON: offsets applied and clamped ------------------------------
    m.values["sd"] = 1024
    m.rx += f1
    rol, pit, thr, yaw, tlt, lnk = m.run()
    check(lnk == 1024, "gate on -> Lnk 1024")
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

    # --- same alive counter again is NOT fresh -----------------------------
    m.tick += 60                   # 600 ms
    m.rx += f1                     # replay: same counter as before
    out = m.run(0)
    check(out[:4] == (0, 0, 0, 0) and out[5] == -1024, f"replayed frame is stale {out}")
    near(out[4], -0.60 * 1024, 2, "gimbal holds last tilt when stale")

    # --- new counter -> fresh again; then watchdog after 500 ms ------------
    m.rx += f2
    out = m.run()
    check(out[5] == 1024 and out[0] != 0, "new counter -> active again")
    m.tick += 49
    out = m.run(0)
    check(out[5] == 1024, "still fresh at 490 ms")
    m.tick += 2
    out = m.run(0)
    check(out[:4] == (0, 0, 0, 0) and out[5] == -1024, "watchdog neutral after 510 ms")

    # --- corrupted frame is rejected ---------------------------------------
    bad = bytearray(f0)
    bad[10] ^= 0x55
    m.rx += bad
    out = m.run()
    check(out[5] == -1024, "corrupted frame ignored")

    # --- '\n' inside binary frames (serialRead stops there) ----------------
    # ch1 = 0x30A puts a 0x0A byte at the start of the payload.
    nl = rc_frame([0x30A] + [992] * 4 + [1811, 172, 0x20A] + [992] * 8)
    check(0x0A in nl[3:], "test frame contains 0x0A")
    m.rx += nl + nl[:5]            # plus a partial frame
    out = m.run()
    near(out[0], (0x30A - 992) * 1.25, 1, "frame with 0x0A parsed (roll)")
    check(out[5] == 1024, "frame with 0x0A counts as fresh")

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
    tlm_path = os.path.join(tmp, "tlm.bin")
    with open(tlm_path, "wb") as fh:
        fh.write(m.tx)
    dec = json.loads(tool("decode", tlm_path))
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

    # attitude dominates the schedule
    per_run = len(dec["frames"]) / 30      # baro slot writes BARO + VARIO
    check(2.0 <= per_run <= 2.2, f"TLM_PER_RUN frames per run ({per_run})")

    # --- drone telemetry lost: only link + status + rc go out ---------------
    m.values["RQly"] = 0
    m.tx.clear()
    for _ in range(30):
        m.run()
    with open(tlm_path, "wb") as fh:
        fh.write(m.tx)
    types = {fr["type"] for fr in json.loads(tool("decode", tlm_path))["frames"]}
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
    """pytest entry: radcmd.lua against the C++ encoder/decoders."""
    import pytest
    if lua52 is None:
        pytest.skip("lupa not installed (pip install lupa)")
    if not os.path.exists(TOOL):
        pytest.skip(f"test_uplink binary not built: {TOOL}")
    assert main() == 0


if __name__ == "__main__":
    sys.exit(main())
