#include "UplinkProtocol.h"
#include <cmath>

namespace Uplink {

namespace {
    uint16_t clampCrsf(long v) {
        if (v < CRSF_MIN) return CRSF_MIN;
        if (v > CRSF_MAX) return CRSF_MAX;
        return static_cast<uint16_t>(v);
    }
    uint16_t be16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
    int16_t  be16s(const uint8_t* p) { return static_cast<int16_t>(be16(p)); }
}

uint16_t usToCrsf(float us) {
    return clampCrsf(std::lround(CRSF_MID + (us - 1500.0f) * 1.6f));
}
float crsfToUs(uint16_t c) { return 1500.0f + (static_cast<float>(c) - CRSF_MID) * 0.625f; }

uint16_t pctToCrsf(float pct) {
    // 100 % = mixer 1024 = CRSF 992 + 819.2
    return clampCrsf(std::lround(CRSF_MID + pct * 8.192f));
}
float crsfToPct(uint16_t c) { return (static_cast<float>(c) - CRSF_MID) / 8.192f; }

std::vector<uint8_t> packChannels(const std::array<uint16_t, NUM_CHANNELS>& ch) {
    std::vector<uint8_t> out;
    out.reserve(22);
    uint32_t acc = 0;
    int bits = 0;
    for (uint16_t v : ch) {
        acc |= static_cast<uint32_t>(v & 0x7FF) << bits;
        bits += 11;
        while (bits >= 8) {
            out.push_back(static_cast<uint8_t>(acc & 0xFF));
            acc >>= 8;
            bits -= 8;
        }
    }
    return out;
}

bool unpackChannels(const std::vector<uint8_t>& p, std::array<uint16_t, NUM_CHANNELS>& ch) {
    if (p.size() < 22) return false;
    uint32_t acc = 0;
    int bits = 0, k = 0;
    for (size_t i = 0; i < 22 && k < NUM_CHANNELS; ++i) {
        acc |= static_cast<uint32_t>(p[i]) << bits;
        bits += 8;
        while (bits >= 11 && k < NUM_CHANNELS) {
            ch[k++] = static_cast<uint16_t>(acc & 0x7FF);
            acc >>= 11;
            bits -= 11;
        }
    }
    return true;
}

uint16_t aliveValue(uint32_t sequence) {
    // Stays inside the normal channel range so a CRSF-trainer radio would
    // accept it too; 1600 distinct values, never repeats twice in a row.
    return static_cast<uint16_t>(200 + sequence % 1600);
}

std::array<uint16_t, NUM_CHANNELS> commandToChannels(const CommandValues& c, uint32_t seq) {
    std::array<uint16_t, NUM_CHANNELS> ch;
    ch.fill(CRSF_MID);
    ch[Ch::ROLL]        = pctToCrsf(c.rollPct);
    ch[Ch::PITCH]       = pctToCrsf(c.pitchPct);
    ch[Ch::THROTTLE]    = usToCrsf(c.throttleUs);
    ch[Ch::YAW]         = pctToCrsf(c.yawPct);
    ch[Ch::TILT]        = pctToCrsf(c.gimbalTiltPct);
    ch[Ch::PAN]         = pctToCrsf(c.gimbalPanPct);
    ch[Ch::STICKS_EN]   = c.sticksEnabled ? CRSF_MAX : CRSF_MIN;
    ch[Ch::THROTTLE_EN] = c.throttleEnabled ? CRSF_MAX : CRSF_MIN;
    ch[Ch::ALIVE]       = aliveValue(seq);
    return ch;
}

std::vector<uint8_t> buildCommandFrame(const CommandValues& c, uint32_t seq) {
    return Crsf::buildFrame(Crsf::ADDR_FLIGHT_CONTROLLER, Crsf::Type::RC_CHANNELS,
                            packChannels(commandToChannels(c, seq)));
}

bool decodeBridgeStatus(const Crsf::Frame& f, BridgeStatus& o) {
    if (f.type != TYPE_RAD_STATUS || f.payload.size() < 22) return false;
    const uint8_t* p = f.payload.data();
    o.version        = p[0];
    const uint8_t fl = p[1];
    o.fresh          = (fl & 0x01) != 0;
    o.gateOn         = (fl & 0x02) != 0;
    o.sticksActive   = (fl & 0x04) != 0;
    o.throttleActive = (fl & 0x08) != 0;
    o.telemetryLive  = (fl & 0x10) != 0;
    o.pilotOverride  = (fl & 0x20) != 0;
    o.angleOn        = (fl & 0x40) != 0;
    const uint16_t echo = be16(p + 2);
    o.echoAlive      = echo == 0xFFFF ? -1 : echo;
    o.rxFrames       = be16(p + 4);
    o.crcErrors      = be16(p + 6);
    o.cmdAgeMs       = p[8] * 10;
    o.throttleStick  = be16s(p + 9);
    o.throttleDelta  = be16s(p + 11);
    o.scriptPeriodMs = p[13] * 10;
    o.outRoll        = be16s(p + 14);
    o.outPitch       = be16s(p + 16);
    o.outYaw         = be16s(p + 18);
    o.outTilt        = be16s(p + 20);
    o.hasPan         = o.version >= 2 && f.payload.size() >= 24;
    o.outPan         = o.hasPan ? be16s(p + 22) : 0;
    return true;
}

} // namespace Uplink
