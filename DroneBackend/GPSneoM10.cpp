#include "GPSNeoM10.h"

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers — little-endian reads
// ─────────────────────────────────────────────────────────────────────────────

int16_t GPSNeoM10::read16(const std::vector<uint8_t>& b, int i) {
    return static_cast<int16_t>(b[i] | (b[i + 1] << 8));
}

uint16_t GPSNeoM10::read16u(const std::vector<uint8_t>& b, int i) {
    return static_cast<uint16_t>(b[i] | (b[i + 1] << 8));
}

int32_t GPSNeoM10::read32(const std::vector<uint8_t>& b, int i) {
    return static_cast<int32_t>(
        b[i]
        | (static_cast<uint32_t>(b[i + 1]) << 8)
        | (static_cast<uint32_t>(b[i + 2]) << 16)
        | (static_cast<uint32_t>(b[i + 3]) << 24)
        );
}

// ─────────────────────────────────────────────────────────────────────────────
// parseRaw() — MSP_RAW_GPS (command ID 106 / 0x6A)
//
// Betaflight 4.x MSP_RAW_GPS payload (16 or 18 bytes):
//
//  Offset  Size  Type      Description
//  ──────  ────  ────────  ──────────────────────────────────────────────────
//  5       1     uint8     GPS fix type: 0 = none, 1 = 2D, 2 = 3D
//  6       1     uint8     Number of satellites used
//  7       4     int32     Latitude  × 10 000 000  (e.g. 452 345 678 = 45.2345678°)
//  11      4     int32     Longitude × 10 000 000
//  15      2     uint16    Altitude MSL, metres
//  17      2     uint16    Ground speed, cm/s
//  19      2     uint16    Ground course, decidegrees (0 – 3599)
//  21      2     uint16    HDOP × 100  (only present in BF ≥ 4.1; size = 18)
//
// Minimum valid frame: header(5) + 16 bytes payload + 1 checksum = 22 bytes.
// With HDOP:           header(5) + 18 bytes payload + 1 checksum = 24 bytes.
// ─────────────────────────────────────────────────────────────────────────────
bool GPSNeoM10::parseRaw(const std::vector<uint8_t>& buf, GPSReading& out) {
    // Guard: need at least the mandatory 16-byte payload
    // Frame: '$'(0) 'M'(1) '>'(2) size(3) cmd(4) payload(5..) crc(last)
    if (buf.size() < 22) return false;
    if (buf[4] != 106)   return false;   // command ID check

    out.fixType = buf[5];
    out.numSat = buf[6];
    out.latitude = read32(buf, 7) / 10000000.0;
    out.longitude = read32(buf, 11) / 10000000.0;
    out.altitudeM = read16u(buf, 15);
    out.groundSpeedMs = read16u(buf, 17);
    out.groundCourse = read16u(buf, 19);

    // HDOP is present when payload size ≥ 18 (buf big enough for offset 21)
    if (buf.size() >= 24) {
        out.hdop = read16u(buf, 21);
    }
    else {
        out.hdop = 9999;   // unknown — older firmware or missing field
    }

    out.rawValid = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// parseComp() — MSP_COMP_GPS (command ID 107 / 0x6B)
//
// Betaflight 4.x MSP_COMP_GPS payload (5 bytes):
//
//  Offset  Size  Type      Description
//  ──────  ────  ────────  ──────────────────────────────────────────────────
//  5       2     uint16    Distance to home point, metres
//  7       2     int16     Bearing to home, degrees (−180 … +180)
//  9       1     uint8     GPS heartbeat — toggles on every fresh GPS frame
//
// Total frame: header(5) + 5 bytes payload + 1 checksum = 11 bytes.
// ─────────────────────────────────────────────────────────────────────────────
bool GPSNeoM10::parseComp(const std::vector<uint8_t>& buf, GPSReading& out) {
    if (buf.size() < 11) return false;
    if (buf[4] != 107)   return false;   // command ID check

    out.distToHomM = read16u(buf, 5);
    out.bearingToHome = read16(buf, 7);
    out.gpsHeartbeat = buf[9];

    out.compValid = true;
    return true;
}