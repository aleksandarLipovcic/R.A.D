#pragma once
#include <vector>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// MagReading — plain data bag filled by MagQMC5883L::parse()
// ─────────────────────────────────────────────────────────────────────────────
struct MagReading {
    int16_t x = 0;      // raw ADC counts, sensor X axis
    int16_t y = 0;      // raw ADC counts, sensor Y axis
    int16_t z = 0;      // raw ADC counts, sensor Z axis
    float   headingDeg = 0.0f;   // magnetic heading, 0-360°, 0 = North
    // computed as atan2(y, x), tilt-uncorrected
};

// ─────────────────────────────────────────────────────────────────────────────
// MagQMC5883L
//
// Static-only helper — no instances needed.
// Mirrors the pattern of BaroBMP280 so DroneLink::parseMag() stays symmetric.
//
// MSP_RAW_MAG (ID 130) response layout
// ─────────────────────────────────────
//  Byte   Value / Meaning
//  ─────  ─────────────────────────────────────────────────────────────────
//  [0]    '$'           MSP preamble
//  [1]    'M'
//  [2]    '>'           direction: FC → host
//  [3]    0x06          payload length = 6 bytes
//  [4]    0x82 (130)    command ID (MSP_RAW_MAG)
//  [5-6]  int16_t LE    magX  (ADC counts)
//  [7-8]  int16_t LE    magY
//  [9-10] int16_t LE    magZ
//  [11]   uint8_t       XOR checksum over bytes [3..10]
//
// Betaflight prerequisite (CLI):
//   set mag_hardware = QMC5883      (or AUTO if only one mag is present)
//   set align_mag    = CW90         (adjust to your GPS module orientation)
//   feature MAG                     (needed on some BF versions)
//   save
// ─────────────────────────────────────────────────────────────────────────────
class MagQMC5883L {
public:
    // Minimum valid buffer length for MSP_RAW_MAG
    static constexpr size_t MIN_BUF_LEN = 12;

    // MSP command byte we expect in buf[4]
    static constexpr uint8_t CMD_ID = 130;

    // ── parse() ───────────────────────────────────────────────────────────────
    // Validate and decode a raw MSP_RAW_MAG response buffer.
    //
    // Returns true  and populates `out` on success.
    // Returns false if the buffer is too short, carries the wrong command ID,
    // or fails the XOR checksum.
    static bool parse(const std::vector<uint8_t>& buf, MagReading& out);

    // ── verifyChecksum() ──────────────────────────────────────────────────────
    // XOR of bytes [3..N-2] must equal byte [N-1].
    // Exposed so unit tests can exercise it independently.
    static bool verifyChecksum(const std::vector<uint8_t>& buf);

private:
    // Decode a little-endian int16 from buf at offset i.
    static int16_t read16(const std::vector<uint8_t>& buf, size_t i);

    // Compute 0-360° heading from raw X/Y counts.
    static float computeHeading(int16_t x, int16_t y);
};