#pragma once
// =============================================================================
// CrsfProtocol — platform-independent CRSF (Crossfire / ExpressLRS) framing
// and telemetry decoders.
//
// No Win32, no threads, no DroneState: this file only turns a raw byte
// stream into validated frames and frames into small plain structs, so it
// can be unit-tested on any machine (see tests/test_crsf.cpp).
//
// ── Where the bytes come from in Project R.A.D. ─────────────────────────────
//
//   Betaflight ──CRSF telemetry──► RP4TD (RX) ══ELRS 2.4 GHz══► Pocket (TX)
//        Pocket EdgeTX:  SYS → Hardware → USB-VCP = "Telem Mirror"
//        ──USB CDC──► laptop ──► CrsfLink ──► DroneState ──► cockpit widgets
//
// EdgeTX mirrors the telemetry frames it receives from the ELRS module
// verbatim to the USB virtual COM port. It is output-only: nothing we write
// to the port is forwarded to the drone, so this is a pure listener.
//
// ── Frame layout (all multi-byte fields BIG-endian, unlike MSP) ─────────────
//
//   [addr/sync][len][type][payload ... ][crc8]
//     len  = number of bytes after itself = 1 (type) + payload + 1 (crc)
//     crc8 = CRC-8/DVB-S2 (poly 0xD5, init 0) over type + payload
//   Maximum total frame size is 64 bytes → len ≤ 62.
// =============================================================================
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace Crsf {

// ── Sync / address bytes accepted as start-of-frame ─────────────────────────
constexpr uint8_t ADDR_FLIGHT_CONTROLLER = 0xC8;  // also the generic "sync" byte
constexpr uint8_t ADDR_RADIO_TRANSMITTER = 0xEA;  // handset
constexpr uint8_t ADDR_CRSF_RECEIVER     = 0xEC;
constexpr uint8_t ADDR_CRSF_TRANSMITTER  = 0xEE;  // TX module

constexpr size_t MAX_FRAME_SIZE = 64;
constexpr uint8_t MIN_LEN = 2;    // type + crc, empty payload
constexpr uint8_t MAX_LEN = 62;

// ── Frame types we decode ───────────────────────────────────────────────────
namespace Type {
    constexpr uint8_t GPS             = 0x02;
    constexpr uint8_t VARIO           = 0x07;
    constexpr uint8_t BATTERY         = 0x08;
    constexpr uint8_t BARO_ALTITUDE   = 0x09;
    constexpr uint8_t HEARTBEAT       = 0x0B;
    constexpr uint8_t LINK_STATISTICS = 0x14;
    constexpr uint8_t RC_CHANNELS     = 0x16;
    constexpr uint8_t ATTITUDE        = 0x1E;
    constexpr uint8_t FLIGHT_MODE     = 0x21;
    constexpr uint8_t DEVICE_INFO     = 0x29;
}

// CRC-8/DVB-S2 (polynomial 0xD5), as used by CRSF.
uint8_t crc8(const uint8_t* data, size_t len);

// One validated frame. `payload` excludes type and CRC.
struct Frame {
    uint8_t addr = 0;
    uint8_t type = 0;
    std::vector<uint8_t> payload;
};

// ── Stream parser ───────────────────────────────────────────────────────────
// Feed arbitrary chunks of bytes (as they come out of ReadFile); complete,
// CRC-valid frames are appended to `out`. Handles frames split across reads,
// garbage between frames, and resyncs one byte at a time after a bad CRC.
class StreamParser {
public:
    void feed(const uint8_t* data, size_t len, std::vector<Frame>& out);
    void reset() { buf_.clear(); }

    uint32_t crcErrors() const { return crcErrors_; }
    uint32_t framesOk() const { return framesOk_; }
    uint64_t bytesDiscarded() const { return bytesDiscarded_; }

private:
    static bool isSyncByte(uint8_t b);
    std::vector<uint8_t> buf_;
    uint32_t crcErrors_ = 0;
    uint32_t framesOk_ = 0;
    uint64_t bytesDiscarded_ = 0;
};

// ── Decoded telemetry structs (units already converted) ─────────────────────

struct LinkStats {                 // 0x14
    int     uplinkRssi1Dbm = 0;    // negative dBm (payload stores magnitude)
    int     uplinkRssi2Dbm = 0;
    uint8_t uplinkLq = 0;          // 0–100 %
    int8_t  uplinkSnr = 0;         // dB
    uint8_t activeAntenna = 0;
    uint8_t rfModeIndex = 0;       // raw ELRS packet-rate index
    uint16_t txPowerMw = 0;        // decoded from power enum
    int     downlinkRssiDbm = 0;
    uint8_t downlinkLq = 0;
    int8_t  downlinkSnr = 0;
};

struct Battery {                   // 0x08
    float    voltageV = 0.0f;      // 0.1 V resolution on the wire
    float    currentA = 0.0f;      // 0.1 A resolution on the wire
    uint32_t mahDrawn = 0;         // uint24
    uint8_t  remainingPct = 0;
};

struct Gps {                       // 0x02
    double   latitude = 0.0;       // degrees
    double   longitude = 0.0;
    float    groundSpeedKmh = 0.0f;
    float    headingDeg = 0.0f;
    int32_t  altitudeM = 0;        // MSL, wire value has +1000 m offset
    uint8_t  satellites = 0;
};

struct Attitude {                  // 0x1E
    // Degrees. Wire format is int16 radians × 10000, order pitch, roll, yaw.
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    float yawDeg = 0.0f;           // normalised to [0, 360)
    int16_t rawYaw = 0;            // kept for bench verification of yaw encoding
};

struct BaroAltitude {              // 0x09
    int32_t altitudeCm = 0;
};

struct Vario {                     // 0x07
    int16_t verticalSpeedCmS = 0;
};

// ── Decoders: return false if the payload is too short ──────────────────────
bool decodeLinkStats(const Frame& f, LinkStats& out);
bool decodeBattery(const Frame& f, Battery& out);
bool decodeGps(const Frame& f, Gps& out);
// yawSigned = true  → yaw is int16 in [-π, π] (CRSF spec).
// yawSigned = false → yaw is uint16 in [0, 2π) (some older FC firmwares).
// Bench test IT-ELRS-004 decides which one Betaflight 4.5 uses.
bool decodeAttitude(const Frame& f, Attitude& out, bool yawSigned = true);
bool decodeBaroAltitude(const Frame& f, BaroAltitude& out);
bool decodeVario(const Frame& f, Vario& out);
bool decodeFlightMode(const Frame& f, std::string& out);

// ELRS TX power enum → milliwatts (0 if unknown).
uint16_t txPowerEnumToMw(uint8_t e);

// Human-readable frame type name, for the sniffer / logs.
const char* typeName(uint8_t type);

// Builds a complete frame (addr, len, type, payload, crc). Used by the unit
// tests and the simulator; never needed in the live read path.
std::vector<uint8_t> buildFrame(uint8_t addr, uint8_t type,
                                const std::vector<uint8_t>& payload);

} // namespace Crsf
