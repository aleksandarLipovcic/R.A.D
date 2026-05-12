#pragma once
#include <cstdint>
#include <vector>
#include <string>

// =============================================================================
// GPSReading
//
// Populated by GPSNeoM10::parseRaw() and GPSNeoM10::parseComp().
//
// Two MSP commands feed this struct:
//   MSP_RAW_GPS  (106) -- fix, satellites, lat/lon, alt, speed, ground course
//   MSP_COMP_GPS (107) -- distance-to-home, bearing-to-home, GPS heartbeat
//
// All fields are in SI-friendly units; raw protocol units are converted
// inside the parser so callers never need to know the wire format.
// =============================================================================
struct GPSReading {
    // -- From MSP_RAW_GPS (106) -----------------------------------------------
    uint8_t  fixType = 0;       // 0 = no fix, 1 = 2D, 2 = 3D
    uint8_t  numSat = 0;       // satellites used in solution
    double   latitude = 0.0;     // decimal degrees  (+ = N, - = S)
    double   longitude = 0.0;     // decimal degrees  (+ = E, - = W)
    uint16_t altitudeM = 0;       // MSL altitude, metres
    uint16_t groundSpeedMs = 0;       // ground speed, cm/s  -> divide by 100 for m/s
    uint16_t groundCourse = 0;       // ground course, decidegrees (0-3599)
    uint16_t hdop = 9999;    // HDOP x100; 9999 = unknown; good fix < 200

    // -- From MSP_COMP_GPS (107) ----------------------------------------------
    uint16_t distToHomM = 0;       // distance to home point, metres
    int16_t  bearingToHome = 0;       // bearing to home, degrees (-180 to +180)
    uint8_t  gpsHeartbeat = 0;       // toggles each time FC gets a fresh GPS frame

    // -- Validity flags -------------------------------------------------------
    bool rawValid = false;   // true after first successful MSP_RAW_GPS parse
    bool compValid = false;   // true after first successful MSP_COMP_GPS parse

    // -- Position usability gate (DO-229E / ICAO Annex 10) -------------------
    // true only when ALL three conditions are met simultaneously:
    //   1. fixType >= 2  -- 3D fix
    //   2. numSat  >= 4  -- minimum constellation for a valid 3D solution
    //   3. hdop    < 500 -- HDOP < 5.0 (x100 wire format)
    //
    // Always gate coordinate display and navigation decisions on this flag,
    // not on rawValid alone. rawValid is set on any parseable frame; this
    // flag is set only when the fix meets minimum integrity thresholds.
    //
    // Note: this GCS is a monitoring tool, NOT a certified navigation system.
    bool positionUsable = false;
};

// =============================================================================
// GPSConstellationFlags
//
// Bitmask selecting which GNSS constellations the NEO-M10 should use.
// These map directly to u-blox UBX-CFG-GNSS gnssId values.
//
// NEO-M10 supports simultaneous use of GPS + up to 2 other constellations
// depending on firmware revision. Enabling too many may cause the receiver to
// silently ignore the extras -- always check numSat after applying config.
//
// BITMASK NOTE: QZSS uses bit 4 (1<<4 = 0x10) in this bitmask, which maps
// to u-blox gnssId 5. The bit position and gnssId are intentionally different;
// the mapping table in buildCfgGNSS() translates between them.
// =============================================================================
enum GPSConstellationFlags : uint8_t {
    GNSS_GPS = (1 << 0),   // u-blox gnssId 0 -- GPS L1 C/A
    GNSS_SBAS = (1 << 1),   // u-blox gnssId 1 -- SBAS (WAAS/EGNOS/etc.)
    GNSS_GALILEO = (1 << 2),   // u-blox gnssId 2 -- Galileo E1
    GNSS_BEIDOU = (1 << 3),   // u-blox gnssId 3 -- BeiDou B1
    GNSS_QZSS = (1 << 4),   // u-blox gnssId 5 -- QZSS (Japan only); bit != gnssId
    GNSS_GLONASS = (1 << 5),   // u-blox gnssId 6 -- GLONASS L1

    // Convenience presets
    GNSS_DEFAULT = GNSS_GPS | GNSS_SBAS,
    GNSS_DUAL = GNSS_GPS | GNSS_SBAS | GNSS_GALILEO,
    GNSS_TRIPLE = GNSS_GPS | GNSS_SBAS | GNSS_GALILEO | GNSS_GLONASS,
    GNSS_ALL = 0x3F
};

// =============================================================================
// GPSProtocol
//
// Output protocol emitted by the NEO-M10 on its UART.
// For Betaflight 4.5.x the recommended setting is UBLOX (UBX binary) as it
// gives the richest data at lowest bandwidth.
//
// IMPORTANT: This enum is defined here (in GPSNeoM10.h) only.
// DroneLink.h includes this file, so do NOT redefine GPSProtocol there.
// =============================================================================
enum class GPSProtocol : uint8_t {
    UBLOX = 0,   // UBX binary (recommended for Betaflight; richest data)
    NMEA = 1,   // ASCII NMEA 0183 sentences
    MSP = 2    // MSP GPS input (legacy; only for injection, not NEO-M10)
};

// =============================================================================
// GPSUpdateRate
//
// Navigation solution rate in Hz. The NEO-M10 supports 1-10 Hz in single-
// constellation mode; multi-constellation reduces the max rate.
//
// UBX-CFG-RATE measRate field is in milliseconds (e.g. 100 ms = 10 Hz).
// =============================================================================
enum class GPSUpdateRate : uint16_t {
    RATE_1HZ = 1,    //  1 Hz -- 1000 ms
    RATE_2HZ = 2,    //  2 Hz --  500 ms -- general GCS telemetry
    RATE_5HZ = 5,    //  5 Hz --  200 ms -- recommended for UAV navigation
    RATE_10HZ = 10     // 10 Hz --  100 ms -- max for single-constellation GPS
};

// =============================================================================
// GPSConfig
//
// Aggregates all run-time configurable parameters for the NEO-M10 receiver.
// Passed to DroneLink::applyGPSConfig() which serialises each field into the
// correct UBX frame(s) and sends them via MSP GPS passthrough.
//
// IMPORTANT: This struct is defined here (in GPSNeoM10.h) only.
// DroneLink.h includes this file, so do NOT redefine GPSConfig there.
// =============================================================================
struct GPSConfig {
    // Constellation selection (bitmask of GPSConstellationFlags)
    // Default: GNSS_DEFAULT (GPS + SBAS) -- safe for any region.
    uint8_t constellations = GPSConstellationFlags::GNSS_DEFAULT;

    // Target navigation solution rate.
    GPSUpdateRate updateRateHz = GPSUpdateRate::RATE_5HZ;

    // Protocol the NEO-M10 emits on its UART to the F405.
    // Must match the gps_provider setting in Betaflight (UBLOX or NMEA).
    GPSProtocol protocol = GPSProtocol::UBLOX;

    // Enable SBAS correction (WAAS / EGNOS / MSAS / GAGAN).
    // Only relevant if GNSS_SBAS is set in constellations.
    bool sbasEnabled = true;

    // Satellites below this elevation (degrees above horizon) are excluded.
    // 5 deg is u-blox default; 10-15 deg is recommended in obstructed areas.
    uint8_t elevationMaskDeg = 5;

    // Minimum signal strength (dBHz). Reserved for future UBX-CFG-GNSS
    // signal quality block implementation -- NOT currently sent to receiver.
    uint8_t signalMaskDbHz = 6;
};

// =============================================================================
// GPSConfigResult
//
// Returned by DroneLink::applyGPSConfig() so Python can show per-step status.
// =============================================================================
struct GPSConfigResult {
    bool gnssAck = false;   // UBX-CFG-GNSS constellation config accepted
    bool rateAck = false;   // UBX-CFG-RATE update rate accepted
    bool protocolAck = false;   // UBX-CFG-PRT protocol change accepted
    bool saveAck = false;   // UBX-CFG-CFG BBR/flash save accepted
    bool overallOk = false;   // true only when all four acks received

    std::string errorDetail;    // human-readable failure reason (empty = success)
};

// =============================================================================
// GPSNeoM10
//
// Stateless parser + UBX frame builder class -- all methods are static.
// Mirrors the BaroBMP280 interface so DroneLink can call it uniformly.
//
// Parser methods:
//   parseRaw()  -- MSP_RAW_GPS  (106) -> GPSReading
//   parseComp() -- MSP_COMP_GPS (107) -> GPSReading
//
// UBX frame builder methods (returns raw bytes ready to write to serial):
//   buildCfgGNSS()  -- UBX-CFG-GNSS  constellation enable/disable
//   buildCfgRate()  -- UBX-CFG-RATE  navigation solution rate
//   buildCfgPrt()   -- UBX-CFG-PRT   UART protocol (UBX / NMEA)
//   buildCfgNav5()  -- UBX-CFG-NAV5  elevation mask + dynamic model
//   buildCfgCfg()   -- UBX-CFG-CFG   save config to BBR and flash
//
// UBX ACK parser:
//   parseAck()      -- checks if buf contains UBX-ACK-ACK for a given class/id
//
// MSP passthrough helper:
//   wrapUbxInMspPassthrough() -- wraps a UBX frame in MSP_PASSTHROUGH (245)
//
// UBX framing reference: u-blox M10 Integration Manual, Section 3.5
//   Preamble: 0xB5 0x62
//   Class (1 byte), ID (1 byte), Length LE uint16, Payload, CK_A, CK_B
//   CK_A/CK_B: 8-bit Fletcher checksum over Class + ID + Length + Payload.
// =============================================================================
class GPSNeoM10 {
public:
    // -- MSP frame parsers ----------------------------------------------------
    static bool parseRaw(const std::vector<uint8_t>& buf, GPSReading& out);
    static bool parseComp(const std::vector<uint8_t>& buf, GPSReading& out);

    // -- UBX config frame builders --------------------------------------------

    // UBX-CFG-GNSS (Class 0x06, ID 0x3E)
    // Enables/disables GNSS systems. GPS is always forced on.
    static std::vector<uint8_t> buildCfgGNSS(uint8_t constellationFlags);

    // UBX-CFG-RATE (Class 0x06, ID 0x08)
    // navRate is always 1. timeRef: 0 = UTC, 1 = GPS time.
    static std::vector<uint8_t> buildCfgRate(GPSUpdateRate rateHz,
        uint16_t timeRef = 1);

    // UBX-CFG-PRT (Class 0x06, ID 0x00) -- UART1 port config
    // Baud rate is fixed at 115200 to match Betaflight GPS serial config.
    static std::vector<uint8_t> buildCfgPrt(GPSProtocol protocol);

    // UBX-CFG-NAV5 (Class 0x06, ID 0x24)
    // dynModel is always 8 (Airborne <4g) -- REQUIRED for UAV applications.
    // Using any other dynamic model causes position errors at UAV speeds.
    static std::vector<uint8_t> buildCfgNav5(uint8_t elevMaskDeg);

    // UBX-CFG-CFG (Class 0x06, ID 0x09)
    // Saves config to BBR and flash.
    static std::vector<uint8_t> buildCfgCfg();

    // -- UBX ACK parser -------------------------------------------------------
    // Returns true if buf contains UBX-ACK-ACK (0x05 0x01) for msgClass/msgId.
    static bool parseAck(const std::vector<uint8_t>& buf,
        uint8_t msgClass, uint8_t msgId);

    // -- MSP GPS passthrough helper -------------------------------------------
    // Wraps a raw UBX frame inside MSP_PASSTHROUGH (245) so it can be sent
    // to Betaflight which forwards it to the GPS UART transparently.
    // Requires BF Configurator -> Ports -> GPS UART -> Passthrough: ON.
    static std::vector<uint8_t> wrapUbxInMspPassthrough(
        const std::vector<uint8_t>& ubxFrame);

private:
    // Internal little-endian read helpers
    static int16_t  read16(const std::vector<uint8_t>& b, int i);
    static uint16_t read16u(const std::vector<uint8_t>& b, int i);
    static int32_t  read32(const std::vector<uint8_t>& b, int i);

    // Fletcher-8 checksum over Class + ID + Length + Payload (UBX spec s3.4)
    static void addUbxChecksum(std::vector<uint8_t>& frame);
};