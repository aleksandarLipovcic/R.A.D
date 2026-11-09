-- =============================================================================
-- radcmd.lua -- Project R.A.D. laptop command bridge (EdgeTX mixer script)
--
-- Runs on the RadioMaster Pocket (EdgeTX 2.12). The Pocket's USB-VCP port is
-- set to "LUA", so this script owns the USB link to the laptop in BOTH
-- directions:
--
--   laptop --CRSF RC frame (0x16)--> serialRead --> mixer outputs Rol/Pit/Thr/
--                                                   Yaw/Tlt/Pan --> mixes on
--                                                   CH1-4 + gimbal CH8/CH9
--                                                   ==ELRS==> Betaflight
--                                                   (servos on M5/M6)
--   laptop <--CRSF telemetry frames-- serialWrite <-- EdgeTX telemetry sensors
--
-- Telemetry is re-encoded from the EdgeTX sensors into the SAME CRSF frames
-- that "Telem Mirror" used to forward, so CrsfLink on the laptop parses it
-- unchanged. Two extra frames are added: RC_CHANNELS (0x16, the channel
-- outputs the radio really transmits) and RAD_STATUS (0x7E, bridge state).
--
-- SAFETY (the radio decides, not the laptop):
--   * Every laptop output is 0 unless the pilot's gate switch (CFG.GATE_SW)
--     is ON and a NEW command frame arrived within CFG.STALE_TICKS.
--   * Roll/pitch/yaw are OFFSETS added to the pilot's sticks, clamped to
--     +-CFG.MAX_STICK. Pilot input always adds on top.
--   * Throttle is a DELTA relative to the pilot's throttle stick, clamped to
--     +-CFG.THR_BAND, and only while the stick is above CFG.THR_MIN_STICK.
--     The laptop can never move throttle further than THR_BAND away from
--     where the pilot holds the stick, and switching the gate off returns
--     throttle to the stick with a jump of at most THR_BAND.
--   * Arming, flight modes and failsafe are NOT touched by this script.
--   * Stick offsets and throttle also need the ANGLE switch ON (read on the
--     radio, CH6). Any pilot roll/pitch/yaw deflection > ~30 % stops the
--     laptop's stick offsets until 0.3 s after release (pilot priority);
--     the laptop throttle (altitude hold, landing) keeps running.
--   * The pilot feels/hears it: haptic + tone when the laptop stops while
--     it was flying, a short buzz when a pilot override starts.
--   * Gimbal tilt (Tlt) and pan (Pan) follow the laptop while commands are
--     fresh and hold their last value when they stop. They are not gated by
--     the switch.
--
-- EdgeTX allows at most 6 outputs per mixer script. Version 2 uses the 6th
-- output for the gimbal pan axis; the former "Lnk" output is gone. The
-- mixer lines on CH1-4 are now conditioned on the gate switch itself (SD),
-- which is equivalent: every laptop output is already 0 here unless the
-- switch is ON and commands are fresh.
--
-- Install: /SCRIPTS/MIXES/radcmd.lua (file name max 6 characters), then
-- MODEL -> Custom scripts -> radcmd. Setup and bench tests: UPUTSTVO.md.
-- =============================================================================

-- ---- Configuration ----------------------------------------------------------
-- All values in EdgeTX mixer units (-1024..1024). On the ELRS/CRSF link one
-- unit is ~0.5 us, so 200 units = ~100 us = 10 % of stick travel.
local CFG = {
  GATE_SW        = "sd",   -- pilot switch that enables laptop control
  GATE_ON        = 1024,   -- switch value meaning ON (2-pos: -1024 / 1024)
  STALE_TICKS    = 50,     -- 10 ms ticks: no new command for 500 ms -> neutral
  MAX_STICK      = 300,    -- max roll/pitch/yaw offset (~30 % stick)
  THR_BAND       = 200,    -- max |throttle - pilot stick| (~100 us)
  THR_MIN_STICK  = -600,   -- laptop throttle only above ~1200 us stick
  ATT_IN_DEGREES = false,  -- true if Ptch/Roll/Yaw sensors read in degrees
  TLM_PER_RUN    = 2,      -- telemetry frames written per script run
  -- Radio-side flight-mode gate: stick offsets and throttle only while the
  -- ANGLE switch is ON. Betaflight: "aux 1 1 1 1800 2100" = ANGLE on AUX2
  -- (CH6) at 1800..2100 us = mixer units 600..1200. Checked on the radio
  -- itself, so a late or wrong flight-mode telemetry on the laptop can never
  -- put offsets on the sticks in ACRO (where they would be rotation rates).
  ANGLE_CH       = "ch6",
  ANGLE_MIN      = 600,
  ANGLE_MAX      = 1200,
  -- Pilot priority: if the pilot moves roll/pitch/yaw more than this, the
  -- laptop's stick offsets stop at once and stay off until the pilot's
  -- sticks have been back below the threshold for OVERRIDE_HOLD. The pilot
  -- has full control the whole time (his sticks always pass through); the
  -- hold only stops laptop and pilot from alternating while the stick
  -- springs back. Laptop throttle (altitude hold / landing) is NOT stopped:
  -- touching roll to correct drift must not make the drone jump in height.
  OVERRIDE       = 300,    -- ~30 % stick
  OVERRIDE_HOLD  = 30,     -- 10 ms ticks = 0.3 s
  ALERTS         = true,   -- haptic + tone when the laptop drops out / pilot overrides
}

-- Telemetry sensor names as they appear on the Pocket's TELEMETRY page.
-- EdgeTX names BOTH the GPS altitude and the baro altitude "Alt": rename
-- them on the radio (e.g. "GAlt" and "BAlt") and put the names here.
local S = {
  pitch = "Ptch", roll = "Roll", yaw = "Yaw",
  vbat = "RxBt", curr = "Curr", capa = "Capa", bpct = "Bat%",
  gps = "GPS", gspd = "GSpd", hdg = "Hdg", galt = "GAlt", sats = "Sats",
  fm = "FM", balt = "BAlt", vspd = "VSpd",
  rss1 = "1RSS", rss2 = "2RSS", rqly = "RQly", rsnr = "RSNR", ant = "ANT",
  rfmd = "RFMD", tpwr = "TPWR", trss = "TRSS", tqly = "TQly", tsnr = "TSNR",
}

-- ---- Helpers ----------------------------------------------------------------
local band, bor, bxor = bit32.band, bit32.bor, bit32.bxor
local lsh, rsh = bit32.lshift, bit32.rshift
local unpack = table.unpack or unpack
local floor = math.floor

local SYNC_FC = 0xC8
local T_GPS, T_VARIO, T_BATT, T_BARO = 0x02, 0x07, 0x08, 0x09
local T_LINK, T_RC, T_ATT, T_FM, T_STATUS = 0x14, 0x16, 0x1E, 0x21, 0x7E
local STATUS_VERSION = 2          -- 2: + applied gimbal pan output

-- CRC-8/DVB-S2 (poly 0xD5), precomputed: building it in init() would cost
-- ~16k Lua instructions and risk the script being killed.
local CRC = {
  0, 213, 127, 170, 254, 43, 129, 84, 41, 252, 86, 131, 215, 2, 168, 125,
  82, 135, 45, 248, 172, 121, 211, 6, 123, 174, 4, 209, 133, 80, 250, 47,
  164, 113, 219, 14, 90, 143, 37, 240, 141, 88, 242, 39, 115, 166, 12, 217,
  246, 35, 137, 92, 8, 221, 119, 162, 223, 10, 160, 117, 33, 244, 94, 139,
  157, 72, 226, 55, 99, 182, 28, 201, 180, 97, 203, 30, 74, 159, 53, 224,
  207, 26, 176, 101, 49, 228, 78, 155, 230, 51, 153, 76, 24, 205, 103, 178,
  57, 236, 70, 147, 199, 18, 184, 109, 16, 197, 111, 186, 238, 59, 145, 68,
  107, 190, 20, 193, 149, 64, 234, 63, 66, 151, 61, 232, 188, 105, 195, 22,
  239, 58, 144, 69, 17, 196, 110, 187, 198, 19, 185, 108, 56, 237, 71, 146,
  189, 104, 194, 23, 67, 150, 60, 233, 148, 65, 235, 62, 106, 191, 21, 192,
  75, 158, 52, 225, 181, 96, 202, 31, 98, 183, 29, 200, 156, 73, 227, 54,
  25, 204, 102, 179, 231, 50, 152, 77, 48, 229, 79, 154, 206, 27, 177, 100,
  114, 167, 13, 216, 140, 89, 243, 38, 91, 142, 36, 241, 165, 112, 218, 15,
  32, 245, 95, 138, 222, 11, 161, 116, 9, 220, 118, 163, 247, 34, 136, 93,
  214, 3, 169, 124, 40, 253, 87, 130, 255, 42, 128, 85, 1, 212, 126, 171,
  132, 81, 251, 46, 122, 175, 5, 208, 173, 120, 210, 7, 83, 134, 44, 249
}

local function clamp(v, lim)
  if v > lim then return lim elseif v < -lim then return -lim end
  return v
end

local function rnd(x) return floor(x + 0.5) end

local function num(name)            -- sensor value as a number (0 if absent)
  local v = getValue(name)
  if type(v) == "number" then return v end
  return 0
end

-- Big-endian appenders. Negative values are written as two's complement.
local function u8(p, v)  p[#p + 1] = band(v, 0xFF) end
local function u16(p, v)
  if v < 0 then v = v + 65536 end
  p[#p + 1] = band(rsh(v, 8), 0xFF); p[#p + 1] = band(v, 0xFF)
end
local function u24(p, v)
  p[#p + 1] = band(rsh(v, 16), 0xFF); u16(p, band(v, 0xFFFF))
end
local function u32(p, v)
  if v < 0 then v = v + 4294967296 end
  u16(p, floor(v / 65536)); u16(p, v % 65536)
end
local function s16(v) return clamp(rnd(v), 32767) end

-- ---- State ------------------------------------------------------------------
local cmd = { roll = 0, pitch = 0, thr = 0, yaw = 0, tilt = 0, pan = 0,
              stk = false, thrOn = false }
local st = { ctr = -1, freshT = -10000, rxFrames = 0, crcErr = 0, lastRun = 0,
             ovrT = -10000, wasActive = false, wasOvr = false }
local out = { rol = 0, pit = 0, thr = 0, yaw = 0, tlt = 0, pan = 0 }
local flags = { fresh = false, gate = false, stk = false, thr = false, live = false,
                ovr = false, angle = false }
local thrStick = -1024
local rxbuf = ""
local chIn = {}
local tlmStep = 0

-- ---- Laptop -> radio: CRSF RC_CHANNELS frame --------------------------------
-- Channel map (CRSF 11-bit values, 992 = centre, 172..1811 = -100..100 %):
--   1 roll offset   2 pitch offset   3 throttle target (absolute)
--   4 yaw offset    5 gimbal tilt    6 sticks enable (>1500 = on)
--   7 throttle enable (>1500 = on)   8 alive counter (must change)
--   9 gimbal pan
local function toMix(c) return (c - 992) * 5 / 4 end

local function unpack11(b, off, dst)
  local acc, bits, k = 0, 0, 1
  for i = off, off + 21 do
    acc = bor(acc, lsh(b[i], bits)); bits = bits + 8
    while bits >= 11 and k <= 16 do
      dst[k] = band(acc, 0x7FF); acc = rsh(acc, 11); bits = bits - 11; k = k + 1
    end
  end
end

local function handleFrame(b, len)          -- b = {type, payload..., crc}
  if b[1] ~= T_RC or len ~= 24 then return end
  unpack11(b, 2, chIn)
  cmd.roll  = toMix(chIn[1]); cmd.pitch = toMix(chIn[2])
  cmd.thr   = toMix(chIn[3]); cmd.yaw   = toMix(chIn[4])
  cmd.tilt  = toMix(chIn[5])
  cmd.pan   = toMix(chIn[9])
  cmd.stk   = chIn[6] > 1500
  cmd.thrOn = chIn[7] > 1500
  st.rxFrames = (st.rxFrames + 1) % 65536
  if chIn[8] ~= st.ctr then                 -- a frozen sender is not "fresh"
    st.ctr = chIn[8]
    st.freshT = getTime()
  end
end

local function isSync(a) return a == 0xC8 or a == 0xEA or a == 0xEC or a == 0xEE end

local function feed(s)
  rxbuf = rxbuf .. s
  if #rxbuf > 256 then rxbuf = string.sub(rxbuf, -128) end
  while #rxbuf >= 2 do
    local a, len = string.byte(rxbuf, 1, 2)
    if not isSync(a) or len < 2 or len > 62 then
      rxbuf = string.sub(rxbuf, 2)
    elseif #rxbuf < len + 2 then
      break
    else
      local b = { string.byte(rxbuf, 3, len + 2) }   -- type .. crc
      local c = 0
      for i = 1, len - 1 do c = CRC[bxor(c, b[i]) + 1] end
      if c == b[len] then
        handleFrame(b, len)
        rxbuf = string.sub(rxbuf, len + 3)
      else
        st.crcErr = (st.crcErr + 1) % 65536
        rxbuf = string.sub(rxbuf, 2)
      end
    end
  end
end

-- ---- Radio -> laptop: CRSF telemetry frames ---------------------------------
local function send(ftype, p)
  local f = { SYNC_FC, #p + 2, ftype }
  local c = CRC[ftype + 1]
  for i = 1, #p do
    local v = band(p[i], 0xFF)
    f[i + 3] = v
    c = CRC[bxor(c, v) + 1]
  end
  f[#p + 4] = c
  serialWrite(string.char(unpack(f)))
end

local function rad(v)
  if CFG.ATT_IN_DEGREES then v = v * math.pi / 180 end
  return v
end

local function sendAttitude()
  if not flags.live then return end
  local yaw = rad(num(S.yaw))
  while yaw > math.pi do yaw = yaw - 2 * math.pi end
  while yaw < -math.pi do yaw = yaw + 2 * math.pi end
  local p = {}
  u16(p, s16(rad(num(S.pitch)) * 10000))
  u16(p, s16(rad(num(S.roll)) * 10000))
  u16(p, s16(yaw * 10000))
  send(T_ATT, p)
end

local function sendBattery()
  if not flags.live then return end
  local p = {}
  u16(p, rnd(num(S.vbat) * 10)); u16(p, rnd(num(S.curr) * 10))
  u24(p, rnd(num(S.capa))); u8(p, rnd(num(S.bpct)))
  send(T_BATT, p)
end

local function sendGps()
  if not flags.live then return end
  local g = getValue(S.gps)
  if type(g) ~= "table" or (g.lat == 0 and g.lon == 0) then return end
  local p = {}
  u32(p, rnd(g.lat * 1e7)); u32(p, rnd(g.lon * 1e7))
  u16(p, rnd(num(S.gspd) * 10))                 -- km/h * 10
  u16(p, rnd(num(S.hdg) * 100))                 -- deg * 100
  u16(p, rnd(num(S.galt)) + 1000)               -- m + 1000
  u8(p, rnd(num(S.sats)))
  send(T_GPS, p)
end

local function sendFlightMode()
  if not flags.live then return end
  local fm = getValue(S.fm)
  if type(fm) ~= "string" or #fm == 0 then return end
  local p = { string.byte(fm, 1, -1) }
  p[#p + 1] = 0
  send(T_FM, p)
end

local function sendBaro()
  if not flags.live then return end
  local alt = num(S.balt)
  local p = {}
  local dm = rnd(alt * 10) + 10000
  if dm >= 0 and dm < 32768 then u16(p, dm)      -- fine mode, dm + 10000
  else u16(p, bor(0x8000, clamp(rnd(alt), 32767))) end
  send(T_BARO, p)
  local v = {}
  u16(v, s16(num(S.vspd) * 100))                -- cm/s
  send(T_VARIO, v)
end

local PWR = { [0] = 0, [10] = 1, [25] = 2, [100] = 3, [500] = 4,
              [1000] = 5, [2000] = 6, [250] = 7, [50] = 8 }
local function dbm(v) v = rnd(v); if v < 0 then v = -v end; return v end
local function sendLink()
  local p = {}
  u8(p, dbm(num(S.rss1))); u8(p, dbm(num(S.rss2)))
  u8(p, rnd(num(S.rqly))); u8(p, rnd(num(S.rsnr)))
  u8(p, rnd(num(S.ant))); u8(p, rnd(num(S.rfmd)))
  u8(p, PWR[rnd(num(S.tpwr))] or 0)
  u8(p, dbm(num(S.trss))); u8(p, rnd(num(S.tqly))); u8(p, rnd(num(S.tsnr)))
  send(T_LINK, p)
end

local function sendRcChannels()             -- what the radio transmits now
  if not getOutputValue then return end
  local p, acc, bits = {}, 0, 0
  for k = 0, 15 do
    local c = rnd(992 + getOutputValue(k) * 4 / 5)
    if c < 0 then c = 0 elseif c > 2047 then c = 2047 end
    acc = bor(acc, lsh(c, bits)); bits = bits + 11
    while bits >= 8 do p[#p + 1] = band(acc, 0xFF); acc = rsh(acc, 8); bits = bits - 8 end
  end
  send(T_RC, p)
end

local function sendStatus(now)
  local f = 0
  if flags.fresh then f = bor(f, 1) end
  if flags.gate  then f = bor(f, 2) end
  if flags.stk   then f = bor(f, 4) end
  if flags.thr   then f = bor(f, 8) end
  if flags.live  then f = bor(f, 16) end
  if flags.ovr   then f = bor(f, 32) end        -- pilot override active
  if flags.angle then f = bor(f, 64) end        -- ANGLE switch on (radio side)
  local p = {}
  u8(p, STATUS_VERSION); u8(p, f)
  u16(p, st.ctr >= 0 and st.ctr or 0xFFFF)
  u16(p, st.rxFrames); u16(p, st.crcErr)
  local age = now - st.freshT
  if age > 255 or age < 0 then age = 255 end
  u8(p, age)
  u16(p, s16(thrStick)); u16(p, s16(out.thr))
  local dt = now - st.lastRun
  if dt > 255 or dt < 0 then dt = 255 end
  u8(p, dt)
  u16(p, s16(out.rol)); u16(p, s16(out.pit)); u16(p, s16(out.yaw)); u16(p, s16(out.tlt))
  u16(p, s16(out.pan))
  send(T_STATUS, p)
end

-- Round robin: attitude every other slot, the rest in turn.
local SLOW = { sendStatus, sendBattery, sendGps, sendLink, sendRcChannels,
               sendFlightMode, sendBaro }
local slowIdx = 0

local function sendTelemetry(now)
  for i = 1, CFG.TLM_PER_RUN do
    tlmStep = tlmStep + 1
    if tlmStep % 2 == 1 then
      sendAttitude()
    else
      slowIdx = slowIdx % #SLOW + 1
      SLOW[slowIdx](now)
    end
  end
end

-- Haptic + tone for the pilot. playHaptic/playTone are general EdgeTX
-- functions; guarded so a firmware without them (or the test mock) only
-- skips the alert and never kills the script.
local function alert(kind)
  if not CFG.ALERTS then return end
  if kind == "lost" then
    if playHaptic then pcall(playHaptic, 300, 100) end
    if playTone then pcall(playTone, 600, 400, 100) end
  else
    if playHaptic then pcall(playHaptic, 60, 0) end
  end
end

local function abs(v) return v < 0 and -v or v end

-- ---- Mixer script entry points ----------------------------------------------
local function init()
  for k = 1, 16 do chIn[k] = 992 end
  st.lastRun = getTime()
end

local function run()
  local now = getTime()

  -- 1. Read laptop commands (serialRead may stop early at a 0x0A byte).
  if serialRead then
    for i = 1, 4 do
      local s = serialRead(64)
      if not s or #s == 0 then break end
      feed(s)
    end
  end

  -- 2. Outputs. Everything defaults to neutral.
  flags.fresh = (now - st.freshT) <= CFG.STALE_TICKS
  flags.gate  = math.abs(num(CFG.GATE_SW) - CFG.GATE_ON) < 100
  flags.live  = num(S.rqly) > 0
  thrStick    = num("thr")

  local ang = num(CFG.ANGLE_CH)
  flags.angle = ang >= CFG.ANGLE_MIN and ang <= CFG.ANGLE_MAX
  if abs(num("ail")) > CFG.OVERRIDE or abs(num("ele")) > CFG.OVERRIDE
     or abs(num("rud")) > CFG.OVERRIDE then
    st.ovrT = now
  end
  flags.ovr = (now - st.ovrT) <= CFG.OVERRIDE_HOLD

  out.rol, out.pit, out.yaw, out.thr = 0, 0, 0, 0
  flags.stk, flags.thr = false, false

  if flags.fresh and flags.gate and flags.angle then
    if cmd.stk and not flags.ovr then
      out.rol = clamp(cmd.roll, CFG.MAX_STICK)
      out.pit = clamp(cmd.pitch, CFG.MAX_STICK)
      out.yaw = clamp(cmd.yaw, CFG.MAX_STICK)
      flags.stk = true
    end
    if cmd.thrOn and thrStick > CFG.THR_MIN_STICK then
      out.thr = clamp(cmd.thr - thrStick, CFG.THR_BAND)
      flags.thr = true
    end
  end
  if flags.fresh then                                       -- else hold
    out.tlt = clamp(cmd.tilt, 1024)
    out.pan = clamp(cmd.pan, 1024)
  end

  -- Alerts on the radio: the laptop was flying and its commands stopped
  -- (app frozen/crashed, USB unplugged), or the pilot took over.
  local active = flags.stk or flags.thr
  if st.wasActive and not active and not flags.fresh and flags.gate then
    alert("lost")
  end
  if flags.ovr and not st.wasOvr and flags.gate and flags.fresh then
    alert("override")
  end
  st.wasActive, st.wasOvr = active, flags.ovr

  -- 3. Telemetry back to the laptop.
  if serialWrite then sendTelemetry(now) end
  st.lastRun = now

  return out.rol, out.pit, out.thr, out.yaw, out.tlt, out.pan
end

return {
  output = { "Rol", "Pit", "Thr", "Yaw", "Tlt", "Pan" },
  init = init,
  run = run,
}
