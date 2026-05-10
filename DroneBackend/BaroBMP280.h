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
#include <cmath>   // std::round

// ── Physical constants ────────────────────────────────────────────────────────
// ISA standard lapse rate conversion: pressure altitude → metres
// We use the simpler barometric formula P = P0 * (1 - L*h/T0)^(g*M/R*L)
// but since Betaflight already integrates the baro and gives us altitude in cm,
// we just do unit conversions here.
static constexpr double CM_TO_M = 0.01;
static constexpr double M_TO_FT = 3.28084;
static constexpr double CM_TO_FT = CM_TO_M * M_TO_FT;

// ── Raw parsed data from one MSP_ALTITUDE frame ───────────────────────────────
struct BaroReading {
    int32_t altitudeCm = 0;   // FC-fused altitude above home point, cm
    int16_t varioCmPerSec = 0;   // Vertical speed, cm/s (positive = climb)
    bool    valid = false;
};

// ── Scaled / display-ready values ─────────────────────────────────────────────
struct BaroData {
    // Altitude
    double  altitudeM = 0.0;   // metres above home / QNH reference
    double  altitudeFt = 0.0;   // feet   above home / QNH reference
    int32_t altitudeCm = 0;     // raw cm (for logging)

    // Vertical speed
    double  varioMps = 0.0;   // m/s  (positive = climbing)
    double  varioFpm = 0.0;   // ft/min (positive = climbing)

    // QNH-corrected altitude (pilot can zero the altimeter on the ground)
    double  altitudeMqnh = 0.0;   // metres relative to QNH zero point
    double  altitudeFtqnh = 0.0;   // feet   relative to QNH zero point

    bool    valid = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// BaroBMP280 — stateless parser + unit converter
// ─────────────────────────────────────────────────────────────────────────────

class BaroBMP280 {
public:
    BaroBMP280() = default;

    // ── Parse MSP_ALTITUDE response ───────────────────────────────────────────
    // Call from DroneLink::communicationLoop() after sendMSP(MSP::ALTITUDE).
    // Returns true on success; writes parsed raw values into `out`.
    // Frame: $ M > 06 6D [bytes 5..10] checksum  — minimum 12 bytes total.
    static bool parse(const std::vector<uint8_t>& buf, BaroReading& out) {
        // Minimum frame: header(3) + size(1) + cmd(1) + payload(6) + crc(1) = 12
        if (buf.size() < 12 || buf[4] != 109 /* MSP_ALTITUDE */) {
            out.valid = false;
            return false;
        }

        // Payload starts at index 5
        out.altitudeCm =
            static_cast<int32_t>(
                static_cast<uint32_t>(buf[5]) |
                (static_cast<uint32_t>(buf[6]) << 8) |
                (static_cast<uint32_t>(buf[7]) << 16) |
                (static_cast<uint32_t>(buf[8]) << 24)
                );

        out.varioCmPerSec =
            static_cast<int16_t>(
                static_cast<uint16_t>(buf[9]) |
                (static_cast<uint16_t>(buf[10]) << 8)
                );

        out.valid = true;
        return true;
    }

    // ── Convert raw reading → display-ready BaroData ─────────────────────────
    // qnhOffsetCm: altitude bias set when pilot presses "Set QNH" (cm).
    //              Pass 0 to show altitude above FC home point.
    static BaroData scale(const BaroReading& raw, int32_t qnhOffsetCm = 0) {
        BaroData d;
        d.valid = raw.valid;
        if (!raw.valid) return d;

        d.altitudeCm = raw.altitudeCm;
        d.altitudeM = raw.altitudeCm * CM_TO_M;
        d.altitudeFt = raw.altitudeCm * CM_TO_FT;

        d.varioMps = raw.varioCmPerSec * CM_TO_M;
        d.varioFpm = d.varioMps * (M_TO_FT * 60.0);

        int32_t qnhCm = raw.altitudeCm - qnhOffsetCm;
        d.altitudeMqnh = qnhCm * CM_TO_M;
        d.altitudeFtqnh = qnhCm * CM_TO_FT;

        return d;
    }
};