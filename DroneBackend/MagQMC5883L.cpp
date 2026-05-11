#include "MagQMC5883L.h"
#include <cmath>

namespace {
    constexpr float PI = 3.14159265358979323846f;
}

// ─────────────────────────────────────────────────────────────────────────────
// verifyChecksum()
//
// Betaflight MSP v1 checksum: XOR of every byte from [3] (length byte)
// through the last payload byte.  The result must equal the final byte
// in the buffer.
//
// For MSP_RAW_MAG:
//   checksum = buf[3] ^ buf[4] ^ buf[5] ^ buf[6] ^ buf[7]
//            ^ buf[8] ^ buf[9] ^ buf[10]
//   expected in buf[11]
// ─────────────────────────────────────────────────────────────────────────────
bool MagQMC5883L::verifyChecksum(const std::vector<uint8_t>& buf) {
    if (buf.size() < MIN_BUF_LEN) return false;

    // Checksum covers bytes [3 .. size-2], result stored in [size-1].
    uint8_t csum = 0;
    for (size_t i = 3; i < buf.size() - 1; ++i)
        csum ^= buf[i];

    return csum == buf[buf.size() - 1];
}

// ─────────────────────────────────────────────────────────────────────────────
// read16() — little-endian signed 16-bit helper
// ─────────────────────────────────────────────────────────────────────────────
int16_t MagQMC5883L::read16(const std::vector<uint8_t>& buf, size_t i) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(buf[i]) |
        (static_cast<uint16_t>(buf[i + 1]) << 8)
        );
}

// ─────────────────────────────────────────────────────────────────────────────
// computeHeading()
//
// Tilt-uncorrected 2-D heading from the horizontal field components.
// Range: 0-360°, where 0° / 360° = magnetic North.
//
// NOTE: this is accurate only when the drone is level.  If you later add
// attitude-compensated heading, pass roll/pitch from DroneState and apply
// the standard tilt-compensation formula before the atan2 call.
// ─────────────────────────────────────────────────────────────────────────────
float MagQMC5883L::computeHeading(int16_t x, int16_t y) {
    float heading = std::atan2f(static_cast<float>(y),
        static_cast<float>(x))
        * (180.0f / PI);

    if (heading < 0.0f)
        heading += 360.0f;

    return heading;
}

// ─────────────────────────────────────────────────────────────────────────────
// parse()
//
// Full validation + decode of a MSP_RAW_MAG response buffer.
//
// Guard order:
//   1. Buffer long enough?
//   2. MSP preamble correct?     ('$', 'M', '>')
//   3. Command ID matches?       (buf[4] == 130)
//   4. Declared payload length?  (buf[3] == 6)
//   5. Checksum valid?
//   6. Decode and populate MagReading.
// ─────────────────────────────────────────────────────────────────────────────
bool MagQMC5883L::parse(const std::vector<uint8_t>& buf, MagReading& out) {

    // ── Guard 1: minimum length ───────────────────────────────────────────────
    if (buf.size() < MIN_BUF_LEN)
        return false;

    // ── Guard 2: MSP v1 preamble ──────────────────────────────────────────────
    if (buf[0] != '$' || buf[1] != 'M' || buf[2] != '>')
        return false;

    // ── Guard 3: command ID ───────────────────────────────────────────────────
    if (buf[4] != CMD_ID)
        return false;

    // ── Guard 4: payload length field ────────────────────────────────────────
    // buf[3] == 6  (3 × int16_t = 6 bytes)
    if (buf[3] != 6)
        return false;

    // ── Guard 5: checksum ─────────────────────────────────────────────────────
    if (!verifyChecksum(buf))
        return false;

    // ── Decode ────────────────────────────────────────────────────────────────
    out.x = read16(buf, 5);   // bytes [5-6]
    out.y = read16(buf, 7);   // bytes [7-8]
    out.z = read16(buf, 9);   // bytes [9-10]

    out.headingDeg = computeHeading(out.x, out.y);

    return true;
}