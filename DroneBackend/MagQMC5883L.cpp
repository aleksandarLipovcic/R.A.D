#include "MagQMC5883L.h"
#include <cmath>

namespace {
    constexpr float PI = 3.14159265358979323846f;
}

// ─────────────────────────────────────────────────────────────────────────────
// verifyChecksum()
//
// MSP v1 checksum: XOR of every byte from [3] (length byte) through the last
// payload byte.  The result must equal the final byte in the buffer.
// ─────────────────────────────────────────────────────────────────────────────
bool MagQMC5883L::verifyChecksum(const std::vector<uint8_t>& buf) {
    if (buf.size() < 6) return false;

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
// Tilt-uncorrected 2-D heading from horizontal field components.
// Range: 0-360°, where 0° / 360° = magnetic North.
//
// NOTE: Accurate only when the drone is level.  For tilt-compensated heading,
// pass roll and pitch from DroneState and apply the standard tilt-compensation
// formula before the atan2 call.
// ─────────────────────────────────────────────────────────────────────────────
float MagQMC5883L::computeHeading(int16_t x, int16_t y) {
    float heading = std::atan2f(static_cast<float>(y),
        static_cast<float>(x))
        * (180.0f / PI);

    if (heading < 0.0f)
        heading += 360.0f;

    return heading;
}