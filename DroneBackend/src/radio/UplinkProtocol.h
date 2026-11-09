#pragma once
// =============================================================================
// UplinkProtocol — laptop → Pocket command frames and the Pocket's replies
//
// Platform-independent (no Win32, no threads), so it is unit-tested on any
// machine together with the Lua bridge (tests/test_lua_bridge.py).
//
// ── Path ────────────────────────────────────────────────────────────────────
//
//   CommandLink ──CRSF RC_CHANNELS 0x16──► Pocket USB-VCP (mode "LUA")
//                                          radcmd.lua → mixer → ELRS → BF
//   CrsfLink    ◄──CRSF telemetry + RC_CHANNELS + RAD_STATUS 0x7E── radcmd.lua
//
// The command frame is a standard, CRC-checked CRSF RC_CHANNELS frame, so the
// same bytes would also drive EdgeTX's "CRSF Trainer over USB-VCP" mode if a
// later EdgeTX release ships it (only the radio-side mixing changes).
//
// ── Command channel map (CRSF 11-bit: 172 = −100 %, 992 = 0, 1811 = +100 %) ─
//
//   ch1 roll offset   ch2 pitch offset   ch3 throttle target (absolute, µs)
//   ch4 yaw offset    ch5 gimbal tilt    ch6 sticks enable (1811 on / 172 off)
//   ch7 throttle enable                  ch8 alive counter (changes every frame)
//   ch9 gimbal pan (outer gimbal axis)   ch10–16 centre (reserved)
//
// The two gimbal values are what the radio puts on its gimbal channels
// (CH8 tilt, CH9 pan in the Pocket mixer); Betaflight forwards those
// channels unchanged to the servo outputs M5/M6 (feature SERVO_TILT,
// "forward from channel"). Converting degrees to percent is the cockpit's
// job (DroneCockpitUI/control/gimbal.py), because it needs the per-servo
// calibration.
//
// Units on the radio side: EdgeTX mixer value v ∈ [−1024, 1024],
//   CRSF = 992 + v·4/5,   µs ≈ 1500 + v/2,   so CRSF = 992 + (µs − 1500)·1.6.
// =============================================================================
#include "CrsfProtocol.h"
#include <array>
#include <cstdint>
#include <vector>

namespace Uplink {

constexpr int NUM_CHANNELS = 16;
constexpr uint16_t CRSF_MIN = 172, CRSF_MID = 992, CRSF_MAX = 1811;

// Frame type of the bridge status frame written by radcmd.lua. Never sent
// over RF: it only exists on the Pocket ↔ laptop USB link.
constexpr uint8_t TYPE_RAD_STATUS = 0x7E;
constexpr uint8_t RAD_STATUS_VERSION = 2;       // 2: + applied gimbal pan
constexpr uint8_t RAD_STATUS_MIN_VERSION = 1;   // v1 bridges still work (no pan)

namespace Ch {                                  // 0-based indices
    constexpr int ROLL = 0, PITCH = 1, THROTTLE = 2, YAW = 3, TILT = 4,
                  STICKS_EN = 5, THROTTLE_EN = 6, ALIVE = 7, PAN = 8;
}

// ── Unit conversions ────────────────────────────────────────────────────────
uint16_t usToCrsf(float us);                  // clamped to 172..1811
float    crsfToUs(uint16_t crsf);
uint16_t pctToCrsf(float pct);                // −100..100 % → 172..1811
float    crsfToPct(uint16_t crsf);

// ── 16 × 11-bit packing (CRSF RC_CHANNELS payload, 22 bytes, LSB first) ─────
std::vector<uint8_t> packChannels(const std::array<uint16_t, NUM_CHANNELS>& ch);
bool unpackChannels(const std::vector<uint8_t>& payload,
                    std::array<uint16_t, NUM_CHANNELS>& ch);

// ── Laptop → Pocket ─────────────────────────────────────────────────────────
struct CommandValues {
    float rollPct = 0.0f;        // offset added to the pilot's stick, −100..100
    float pitchPct = 0.0f;       // + = nose down / forward (stick forward)
    float yawPct = 0.0f;         // + = yaw right
    bool  sticksEnabled = false;
    float throttleUs = 1000.0f;  // absolute target; the radio clamps it to
    bool  throttleEnabled = false;   // ± THR_BAND around the pilot's stick
    float gimbalTiltPct = 0.0f;  // −100..100 % of the gimbal tilt channel (CH8)
    float gimbalPanPct = 0.0f;   // −100..100 % of the gimbal pan channel (CH9)
};

std::array<uint16_t, NUM_CHANNELS> commandToChannels(const CommandValues& c,
                                                     uint32_t sequence);
uint16_t aliveValue(uint32_t sequence);       // ch8 value for a sequence number
std::vector<uint8_t> buildCommandFrame(const CommandValues& c, uint32_t sequence);

// ── Pocket → laptop: RAD_STATUS (0x7E) ──────────────────────────────────────
struct BridgeStatus {
    uint8_t  version = 0;
    bool     fresh = false;          // a new command frame within 500 ms
    bool     gateOn = false;         // pilot's laptop-control switch is ON
    bool     sticksActive = false;   // roll/pitch/yaw offsets are applied
    bool     throttleActive = false; // throttle delta is applied
    bool     telemetryLive = false;  // drone telemetry is arriving (RQly > 0)
    bool     pilotOverride = false;  // pilot moved a stick > ~30 %: laptop offsets off (bridge ≥ v2)
    bool     angleOn = false;        // ANGLE switch on, read on the radio (bridge ≥ v2)
    int      echoAlive = -1;         // last ch8 value the script accepted
    uint16_t rxFrames = 0;           // command frames received (mod 65536)
    uint16_t crcErrors = 0;
    int      cmdAgeMs = -1;          // age of the newest command (≤ 2550)
    int16_t  throttleStick = 0;      // pilot throttle stick, mixer units
    int16_t  throttleDelta = 0;      // applied throttle delta, mixer units
    int      scriptPeriodMs = 0;     // time between two script runs
    int16_t  outRoll = 0, outPitch = 0, outYaw = 0, outTilt = 0;   // mixer units
    int16_t  outPan = 0;             // mixer units (version ≥ 2, else 0)
    bool     hasPan = false;         // the bridge reports the pan output
};
bool decodeBridgeStatus(const Crsf::Frame& f, BridgeStatus& out);

// mixer units (−1024..1024) → µs on the ELRS link
inline float mixToUs(int v) { return 1500.0f + v / 2.0f; }

} // namespace Uplink
