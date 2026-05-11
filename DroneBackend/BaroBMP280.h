#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// BaroBMP280.h
//
// Barometer data structures and the BaroBMP280 helper class.
//
// DATA SOURCE
// ───────────
// The BMP280 is read by Betaflight internally.  We never talk to the sensor
// directly; instead we request the FC-fused baro output via MSP_ALTITUDE
// (command ID 109).
//
// MSP_ALTITUDE payload (6 bytes, little-endian):
//   Bytes 0-3  int32_t  estimatedAltitudeCm   Fused altitude above home, cm
//   Bytes 4-5  int16_t  varioCmPerSec         Vertical speed, cm/s
//
// Full frame: $ M > 06 6D [4-byte alt][2-byte vario] <checksum>  = 12 bytes
//
// DESIGN
// ──────
// BaroBMP280 is a thin value-type wrapper — it holds no state beyond the last
// parsed reading and a user-settable QNH reference (sea-level offset).
// The actual MSP request/response cycle is done by DroneLink::communicationLoop
// exactly as for every other sensor.  BaroBMP280::parse() is called from there.
//
// MSP ID added to the existing MSP namespace in DroneLink.h:
//   static constexpr uint8_t ALTITUDE = 109;   // MSP_ALTITUDE
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdint>
#include <vector>
#include <cmath>

static constexpr double CM_TO_M = 0.01;
static constexpr double M_TO_FT = 3.28084;
static constexpr double CM_TO_FT = CM_TO_M * M_TO_FT;

struct BaroReading {
    int32_t altitudeCm = 0;
    int16_t varioCmPerSec = 0;
    bool    valid = false;
};

struct BaroData {
    // Altitude
    double  altitudeM = 0.0;
    double  altitudeFt = 0.0;
    int32_t altitudeCm = 0;

    // Vertical speed — raw cm/s kept so the Python UI dict can use it directly
    int16_t varioCmPerSec = 0;   // ← NEW: raw value, same unit as BaroReading
    double  varioMps = 0.0;
    double  varioFpm = 0.0;

    // QNH-corrected altitude
    double  altitudeMqnh = 0.0;
    double  altitudeFtqnh = 0.0;

    bool    valid = false;
};

class BaroBMP280 {
public:
    BaroBMP280() = default;

    static bool parse(const std::vector<uint8_t>& buf, BaroReading& out) {
        if (buf.size() < 12 || buf[4] != 109) {
            out.valid = false;
            return false;
        }
        out.altitudeCm =
            static_cast<int32_t>(
                static_cast<uint32_t>(buf[5]) |
                (static_cast<uint32_t>(buf[6]) << 8) |
                (static_cast<uint32_t>(buf[7]) << 16) |
                (static_cast<uint32_t>(buf[8]) << 24));

        out.varioCmPerSec =
            static_cast<int16_t>(
                static_cast<uint16_t>(buf[9]) |
                (static_cast<uint16_t>(buf[10]) << 8));

        out.valid = true;
        return true;
    }

    static BaroData scale(const BaroReading& raw, int32_t qnhOffsetCm = 0) {
        BaroData d;
        d.valid = raw.valid;
        if (!raw.valid) return d;

        d.altitudeCm = raw.altitudeCm;
        d.altitudeM = raw.altitudeCm * CM_TO_M;
        d.altitudeFt = raw.altitudeCm * CM_TO_FT;

        d.varioCmPerSec = raw.varioCmPerSec;          // ← pass through raw
        d.varioMps = raw.varioCmPerSec * CM_TO_M;
        d.varioFpm = d.varioMps * (M_TO_FT * 60.0);

        int32_t qnhCm = raw.altitudeCm - qnhOffsetCm;
        d.altitudeMqnh = qnhCm * CM_TO_M;
        d.altitudeFtqnh = qnhCm * CM_TO_FT;

        return d;
    }
};