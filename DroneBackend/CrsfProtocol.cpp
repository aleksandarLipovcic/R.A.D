#include "CrsfProtocol.h"
#include <cmath>

namespace Crsf {

namespace {
    constexpr double RAD_TO_DEG = 57.29577951308232;

    inline uint16_t be16(const uint8_t* p) {
        return static_cast<uint16_t>((p[0] << 8) | p[1]);
    }
    inline int16_t be16s(const uint8_t* p) {
        return static_cast<int16_t>(be16(p));
    }
    inline uint32_t be24(const uint8_t* p) {
        return (static_cast<uint32_t>(p[0]) << 16) |
               (static_cast<uint32_t>(p[1]) << 8) | p[2];
    }
    inline int32_t be32s(const uint8_t* p) {
        return static_cast<int32_t>(
            (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
            (static_cast<uint32_t>(p[2]) << 8)  |  static_cast<uint32_t>(p[3]));
    }
}

// =============================================================================
// crc8 — CRC-8/DVB-S2, poly 0xD5, init 0x00, no reflection
// =============================================================================
uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

// =============================================================================
// StreamParser
// =============================================================================
bool StreamParser::isSyncByte(uint8_t b) {
    return b == ADDR_FLIGHT_CONTROLLER || b == ADDR_RADIO_TRANSMITTER ||
           b == ADDR_CRSF_RECEIVER     || b == ADDR_CRSF_TRANSMITTER;
}

void StreamParser::feed(const uint8_t* data, size_t len, std::vector<Frame>& out) {
    buf_.insert(buf_.end(), data, data + len);

    size_t pos = 0;
    while (true) {
        // 1. Find a plausible start-of-frame.
        while (pos < buf_.size() && !isSyncByte(buf_[pos])) { ++pos; ++bytesDiscarded_; }
        if (buf_.size() - pos < 2) break;                    // need addr + len

        const uint8_t flen = buf_[pos + 1];
        if (flen < MIN_LEN || flen > MAX_LEN) {              // impossible length → not a frame start
            ++pos; ++bytesDiscarded_;
            continue;
        }
        const size_t total = static_cast<size_t>(flen) + 2;
        if (buf_.size() - pos < total) break;                // wait for the rest

        // 2. CRC over type + payload.
        const uint8_t* f = buf_.data() + pos;
        const uint8_t expected = f[total - 1];
        if (crc8(f + 2, flen - 1) != expected) {
            ++crcErrors_;
            ++pos; ++bytesDiscarded_;                        // resync one byte later
            continue;
        }

        Frame fr;
        fr.addr = f[0];
        fr.type = f[2];
        fr.payload.assign(f + 3, f + total - 1);
        out.push_back(std::move(fr));
        ++framesOk_;
        pos += total;
    }

    if (pos > 0) buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
    // Safety valve: a stream of pure garbage can never grow the buffer beyond
    // one maximum frame.
    if (buf_.size() > 4 * MAX_FRAME_SIZE) {
        bytesDiscarded_ += buf_.size();
        buf_.clear();
    }
}

// =============================================================================
// Decoders
// =============================================================================
uint16_t txPowerEnumToMw(uint8_t e) {
    // ELRS / CRSF power index table
    static const uint16_t table[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 };
    return e < sizeof(table) / sizeof(table[0]) ? table[e] : 0;
}

bool decodeLinkStats(const Frame& f, LinkStats& o) {
    if (f.type != Type::LINK_STATISTICS || f.payload.size() < 10) return false;
    const uint8_t* p = f.payload.data();
    o.uplinkRssi1Dbm  = -static_cast<int>(p[0]);
    o.uplinkRssi2Dbm  = -static_cast<int>(p[1]);
    o.uplinkLq        = p[2];
    o.uplinkSnr       = static_cast<int8_t>(p[3]);
    o.activeAntenna   = p[4];
    o.rfModeIndex     = p[5];
    o.txPowerMw       = txPowerEnumToMw(p[6]);
    o.downlinkRssiDbm = -static_cast<int>(p[7]);
    o.downlinkLq      = p[8];
    o.downlinkSnr     = static_cast<int8_t>(p[9]);
    return true;
}

bool decodeBattery(const Frame& f, Battery& o) {
    if (f.type != Type::BATTERY || f.payload.size() < 8) return false;
    const uint8_t* p = f.payload.data();
    o.voltageV     = be16(p) / 10.0f;
    o.currentA     = be16(p + 2) / 10.0f;
    o.mahDrawn     = be24(p + 4);
    o.remainingPct = p[7];
    return true;
}

bool decodeGps(const Frame& f, Gps& o) {
    if (f.type != Type::GPS || f.payload.size() < 15) return false;
    const uint8_t* p = f.payload.data();
    o.latitude       = be32s(p) / 1e7;
    o.longitude      = be32s(p + 4) / 1e7;
    o.groundSpeedKmh = be16(p + 8) / 10.0f;
    o.headingDeg     = be16(p + 10) / 100.0f;
    o.altitudeM      = static_cast<int32_t>(be16(p + 12)) - 1000;
    o.satellites     = p[14];
    return true;
}

bool decodeAttitude(const Frame& f, Attitude& o, bool yawSigned) {
    if (f.type != Type::ATTITUDE || f.payload.size() < 6) return false;
    const uint8_t* p = f.payload.data();
    o.pitchDeg = static_cast<float>(be16s(p)     / 10000.0 * RAD_TO_DEG);
    o.rollDeg  = static_cast<float>(be16s(p + 2) / 10000.0 * RAD_TO_DEG);
    o.rawYaw   = be16s(p + 4);
    double yawRad = yawSigned ? be16s(p + 4) / 10000.0 : be16(p + 4) / 10000.0;
    double yaw = std::fmod(yawRad * RAD_TO_DEG, 360.0);
    if (yaw < 0.0) yaw += 360.0;
    o.yawDeg = static_cast<float>(yaw);
    return true;
}

bool decodeBaroAltitude(const Frame& f, BaroAltitude& o) {
    if (f.type != Type::BARO_ALTITUDE || f.payload.size() < 2) return false;
    uint16_t v = be16(f.payload.data());
    if (v & 0x8000)                       // coarse mode: metres, no offset
        o.altitudeCm = static_cast<int32_t>(v & 0x7FFF) * 100;
    else                                  // fine mode: decimetres + 10000 offset
        o.altitudeCm = (static_cast<int32_t>(v) - 10000) * 10;
    return true;
}

bool decodeVario(const Frame& f, Vario& o) {
    if (f.type != Type::VARIO || f.payload.size() < 2) return false;
    o.verticalSpeedCmS = be16s(f.payload.data());
    return true;
}

bool decodeFlightMode(const Frame& f, std::string& o) {
    if (f.type != Type::FLIGHT_MODE || f.payload.empty()) return false;
    o.clear();
    for (uint8_t c : f.payload) {
        if (c == 0) break;
        o.push_back(static_cast<char>(c));
    }
    return true;
}

const char* typeName(uint8_t t) {
    switch (t) {
    case Type::GPS:             return "GPS";
    case Type::VARIO:           return "VARIO";
    case Type::BATTERY:         return "BATTERY";
    case Type::BARO_ALTITUDE:   return "BARO_ALT";
    case Type::HEARTBEAT:       return "HEARTBEAT";
    case Type::LINK_STATISTICS: return "LINK_STATS";
    case Type::RC_CHANNELS:     return "RC_CHANNELS";
    case Type::ATTITUDE:        return "ATTITUDE";
    case Type::FLIGHT_MODE:     return "FLIGHT_MODE";
    case Type::DEVICE_INFO:     return "DEVICE_INFO";
    default:                    return "OTHER";
    }
}

std::vector<uint8_t> buildFrame(uint8_t addr, uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> f;
    f.reserve(payload.size() + 4);
    f.push_back(addr);
    f.push_back(static_cast<uint8_t>(payload.size() + 2));
    f.push_back(type);
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(crc8(f.data() + 2, payload.size() + 1));
    return f;
}

} // namespace Crsf
