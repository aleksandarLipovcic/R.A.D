#pragma once
#include <cstdint>
#include <vector>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// GPSReading — populated by GPSNeoM10::parseRaw() and GPSNeoM10::parseComp()
//
// Two MSP commands feed this struct:
//   MSP_RAW_GPS  (106) — fix, satellites, lat/lon, alt, speed, ground course
//   MSP_COMP_GPS (107) — distance-to-home, bearing-to-home, GPS heartbeat
//
// All fields are in SI-friendly units; raw ADC / protocol units are converted
// inside the parser so callers never need to know the wire format.
// ─────────────────────────────────────────────────────────────────────────────
struct GPSReading {
    // ── From MSP_RAW_GPS (106) ────────────────────────────────────────────────
    uint8_t  fixType = 0;       // 0 = no fix, 1 = 2D, 2 = 3D
    uint8_t  numSat = 0;       // satellites used in solution
    double   latitude = 0.0;     // decimal degrees  (+ = N, − = S)
    double   longitude = 0.0;     // decimal degrees  (+ = E, − = W)
    uint16_t altitudeM = 0;       // MSL altitude, metres
    uint16_t groundSpeedMs = 0;       // ground speed, cm/s  → divide by 100 for m/s
    uint16_t groundCourse = 0;       // ground course, decidegrees (0–3599)
    uint16_t hdop = 9999;    // horizontal dilution of precision × 100
    // 9999 = unknown; aviation minimum: < 100 (1.0)

// ── From MSP_COMP_GPS (107) ───────────────────────────────────────────────
    uint16_t distToHomM = 0;       // distance to home point, metres
    int16_t  bearingToHome = 0;       // bearing to home, degrees (−180 … +180)
    uint8_t  gpsHeartbeat = 0;       // toggles each time FC gets a fresh GPS frame

    // ── Validity flags ────────────────────────────────────────────────────────
    bool rawValid = false;   // true after first successful MSP_RAW_GPS parse
    bool compValid = false;   // true after first successful MSP_COMP_GPS parse
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSConstellationFlags
//
// Bitmask selecting which GNSS constellations the NEO-M10 should use.
// These map directly to u-blox UBX-CFG-GNSS gnssId values.
//
// AVIATION NOTE (ICAO Annex 10, Volume I — Radio Navigation Aids):
//   Multi-constellation GNSS improves availability and integrity.
//   GPS (USA) is the primary ICAO-recognised GNSS; Galileo (EU) and GLONASS
//   (Russia) are accepted supplementary systems. BeiDou (China) is not yet
//   universally accepted for certified aviation use in all regions.
//
//   For certified aviation applications, consult applicable TSO/ETSO standards
//   (TSO-C196b, ETSO-C196b) and the aircraft's AFM supplement before enabling
//   non-GPS constellations. This GCS is a ground-support / monitoring tool and
//   does NOT constitute a certified navigation system.
//
// NEO-M10 UBX gnssId values:
//   0 = GPS        (L1 C/A)
//   1 = SBAS       (WAAS, EGNOS, MSAS, GAGAN — augments GPS)
//   2 = Galileo    (E1 B/C)
//   3 = BeiDou     (B1I / B1C)
//   5 = QZSS       (L1 C/A — Japan regional; do not enable outside coverage)
//   6 = GLONASS    (L1 OF)
//
// The NEO-M10 supports simultaneous use of GPS + up to 2 other constellations
// depending on firmware and hardware revision. Enabling too many may cause
// the receiver to silently ignore the extras — always check numSat.
// ─────────────────────────────────────────────────────────────────────────────
enum GPSConstellationFlags : uint8_t {
    GNSS_GPS = (1 << 0),   // u-blox gnssId 0 — GPS L1 C/A
    GNSS_SBAS = (1 << 1),   // u-blox gnssId 1 — SBAS (WAAS/EGNOS/etc.)
    GNSS_GALILEO = (1 << 2),   // u-blox gnssId 2 — Galileo E1
    GNSS_BEIDOU = (1 << 3),   // u-blox gnssId 3 — BeiDou B1
    GNSS_QZSS = (1 << 4),   // u-blox gnssId 5 — QZSS (Japan only)
    GNSS_GLONASS = (1 << 5),   // u-blox gnssId 6 — GLONASS L1

    // Convenience presets
    GNSS_DEFAULT = GNSS_GPS | GNSS_SBAS,                         // safe minimum
    GNSS_DUAL = GNSS_GPS | GNSS_SBAS | GNSS_GALILEO,          // GPS + Galileo
    GNSS_TRIPLE = GNSS_GPS | GNSS_SBAS | GNSS_GALILEO | GNSS_GLONASS,
    GNSS_ALL = 0x3F                                           // all 6 bits set
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSProtocol
//
// Output protocol emitted by the NEO-M10 on its UART.
// Betaflight supports UBLOX (native UBX binary), NMEA, and MSP GPS (legacy).
// For Betaflight 4.5.x the recommended setting is UBLOX (UBX binary) as it
// gives the richest data at lowest bandwidth and is what MSP_RAW_GPS relies on.
//
// AVIATION NOTE:
//   NMEA 0183 is the standard interface for certified avionics but this
//   backend does not implement a certified NMEA parser.  Use UBLOX for GCS
//   monitoring; leave NMEA for any certified instrument feed.
// ─────────────────────────────────────────────────────────────────────────────
enum class GPSProtocol : uint8_t {
    UBLOX = 0,    // UBX binary (recommended for Betaflight; richest data)
    NMEA = 1,    // ASCII NMEA 0183 sentences
    MSP = 2     // MSP GPS input (legacy; only for injection, not NEO-M10)
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSUpdateRate
//
// Navigation solution rate in Hz.  The NEO-M10 supports 1–10 Hz in single-
// constellation mode; multi-constellation reduces the max rate.
//
// AVIATION NOTE (TSO-C129a / DO-229E performance requirements):
//   A 1 Hz position update rate is the minimum for en-route operations.
//   Approach operations require ≥ 5 Hz for adequate obstacle clearance.
//   The NEO-M10 is NOT a certified aviation receiver and MUST NOT be used
//   as the primary navigation sensor in certified aircraft operations.
//   These rate settings affect GCS telemetry display only.
//
// UBX-CFG-RATE measRate field is in milliseconds (e.g. 100 ms = 10 Hz).
// ─────────────────────────────────────────────────────────────────────────────
enum class GPSUpdateRate : uint16_t {
    RATE_1HZ = 1,   //  1 Hz — 1000 ms — minimum; en-route monitoring
    RATE_2HZ = 2,   //  2 Hz —  500 ms — general GCS telemetry
    RATE_5HZ = 5,   //  5 Hz —  200 ms — recommended for UAV navigation
    RATE_10HZ = 10    // 10 Hz —  100 ms — max for single-constellation GPS only
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSConfig
//
// Aggregates all run-time configurable parameters for the NEO-M10 receiver.
// Passed to DroneLink::applyGPSConfig() which serialises each field into the
// correct UBX frame(s) and sends them via MSP GPS passthrough.
//
// Persistence: UBX-CFG-CFG is sent after each config sequence to save the
// settings to the receiver's battery-backed RAM (BBR) so they survive power
// cycles.  Flash save is also attempted if the receiver reports flash is
// available (UBX-ACK-ACK check).
// ─────────────────────────────────────────────────────────────────────────────
struct GPSConfig {
    // ── Constellation selection ───────────────────────────────────────────────
    // Bitmask of GPSConstellationFlags values.
    // Default: GNSS_DEFAULT (GPS + SBAS) — safe for any region.
    uint8_t constellations = GPSConstellationFlags::GNSS_DEFAULT;

    // ── Update rate ───────────────────────────────────────────────────────────
    // Target navigation solution rate in Hz.
    // Default: 5 Hz — good balance between bandwidth and freshness.
    GPSUpdateRate updateRateHz = GPSUpdateRate::RATE_5HZ;

    // ── Output protocol ───────────────────────────────────────────────────────
    // Protocol the NEO-M10 emits on its UART to the F405.
    // Must match the gps_provider setting in Betaflight (UBLOX or NMEA).
    GPSProtocol protocol = GPSProtocol::UBLOX;

    // ── SBAS ──────────────────────────────────────────────────────────────────
    // Enable SBAS correction (WAAS over North America, EGNOS over Europe,
    // MSAS over Japan, GAGAN over India).  Only relevant if GNSS_SBAS is set.
    // SBAS improves horizontal accuracy to < 3 m (95%) in coverage areas.
    bool sbasEnabled = true;

    // ── Minimum elevation mask ────────────────────────────────────────────────
    // Satellites below this elevation (degrees above horizon) are excluded
    // from the navigation solution.
    // 5° is the u-blox default; 10–15° is recommended in urban/obstructed areas
    // to avoid multipath; aviation typically uses 5° per DO-229E.
    uint8_t elevationMaskDeg = 5;

    // ── Minimum signal strength mask ─────────────────────────────────────────
    // Satellites with C/No below this threshold (dBHz) are excluded.
    // NEO-M10 default: 6 dBHz; typical good-sky minimum: 20 dBHz.
    uint8_t signalMaskDbHz = 6;
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSConfigResult
//
// Returned by DroneLink::applyGPSConfig() so Python can show per-step status.
// Each field indicates whether the corresponding UBX config message was
// acknowledged by the receiver (UBX-ACK-ACK received within timeout).
// ─────────────────────────────────────────────────────────────────────────────
struct GPSConfigResult {
    bool gnssAck = false;   // UBX-CFG-GNSS constellation config accepted
    bool rateAck = false;   // UBX-CFG-RATE update rate accepted
    bool protocolAck = false;   // UBX-CFG-PRT protocol change accepted
    bool saveAck = false;   // UBX-CFG-CFG BBR/flash save accepted
    bool overallOk = false;   // true only when all four acks received

    std::string errorDetail;    // human-readable failure reason (empty = success)
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSNeoM10
//
// Stateless parser + UBX frame builder class — all methods are static.
// Mirrors the BaroBMP280 interface so DroneLink can call it uniformly.
//
// Parser methods:
//   parseRaw()  — MSP_RAW_GPS  (106) → GPSReading
//   parseComp() — MSP_COMP_GPS (107) → GPSReading
//
// UBX frame builder methods (returns raw bytes ready to write to serial port):
//   buildCfgGNSS()    — UBX-CFG-GNSS constellation enable/disable
//   buildCfgRate()    — UBX-CFG-RATE navigation solution rate
//   buildCfgPrt()     — UBX-CFG-PRT UART protocol (UBX / NMEA)
//   buildCfgNav5()    — UBX-CFG-NAV5 elevation + signal masks
//   buildCfgCfg()     — UBX-CFG-CFG save config to BBR and flash
//
// UBX ACK parser:
//   parseAck()        — checks if buf contains UBX-ACK-ACK for a given class/id
//
// UBX framing reference: u-blox M10 Integration Manual, Section 3.5
//   Preamble: 0xB5 0x62
//   Class (1 byte), ID (1 byte), Length LE uint16, Payload, CK_A, CK_B
//   CK_A/CK_B: 8-bit Fletcher checksum over Class + ID + Length + Payload.
// ─────────────────────────────────────────────────────────────────────────────
class GPSNeoM10 {
public:
    // ── MSP frame parsers ─────────────────────────────────────────────────────
    static bool parseRaw(const std::vector<uint8_t>& buf, GPSReading& out);
    static bool parseComp(const std::vector<uint8_t>& buf, GPSReading& out);

    // ── UBX config frame builders ─────────────────────────────────────────────
    // Each returns a complete, checksummed UBX binary frame ready for serial.

    // UBX-CFG-GNSS (Class 0x06, ID 0x3E)
    // Enables/disables GNSS systems.  constellationFlags is a bitmask of
    // GPSConstellationFlags.  The NEO-M10 requires GPS to always be enabled;
    // if GNSS_GPS is not set it will be forced on and noted in GPSConfigResult.
    static std::vector<uint8_t> buildCfgGNSS(uint8_t constellationFlags);

    // UBX-CFG-RATE (Class 0x06, ID 0x08)
    // Sets navigation solution rate. measRateHz is converted to ms internally.
    // navRate is always 1 (solution per measurement) per u-blox recommendation.
    // timeRef: 0 = UTC, 1 = GPS time (use GPS time for aviation consistency).
    static std::vector<uint8_t> buildCfgRate(GPSUpdateRate rateHz,
        uint16_t timeRef = 1);

    // UBX-CFG-PRT (Class 0x06, ID 0x00) — UART1 port config
    // Sets output protocol (UBX, NMEA, or both).
    // Baud rate is preserved at 115200 to match Betaflight's GPS serial config.
    // inProto is always UBX so the FC can still send config commands.
    static std::vector<uint8_t> buildCfgPrt(GPSProtocol protocol);

    // UBX-CFG-NAV5 (Class 0x06, ID 0x24)
    // Sets elevation mask and signal minimum.
    // dynModel = 8 (Airborne < 4g) — REQUIRED for UAV applications per u-blox;
    //   DO NOT use pedestrian (0) or automotive (4) models for airborne use.
    //   The Airborne model disables the speed/altitude caps that would cause
    //   loss of fix at UAV speeds and altitudes.
    static std::vector<uint8_t> buildCfgNav5(uint8_t elevMaskDeg,
        uint8_t sigMinDbHz);

    // UBX-CFG-CFG (Class 0x06, ID 0x09)
    // Saves current config to BBR (battery-backed RAM) and flash if available.
    // clearMask = 0x0000 (no clear), saveMask = 0x061F (nav+msg+port+rxm+inf),
    // loadMask = 0x0000 (no load), deviceMask = 0x07 (BBR + flash + EEPROM).
    static std::vector<uint8_t> buildCfgCfg();

    // ── UBX ACK parser ────────────────────────────────────────────────────────
    // Returns true if buf contains a UBX-ACK-ACK (0x05 0x01) frame
    // acknowledging the given msgClass + msgId.
    // Returns false for UBX-ACK-NAK or any non-ACK frame.
    static bool parseAck(const std::vector<uint8_t>& buf,
        uint8_t msgClass, uint8_t msgId);

    // ── MSP GPS passthrough helpers ───────────────────────────────────────────
    // Wraps a raw UBX frame inside an MSP_PASSTHROUGH (245) frame so it can be
    // sent to Betaflight which forwards it transparently to the GPS UART.
    // This avoids the need for a separate serial connection to the GPS module.
    //
    // MSP_PASSTHROUGH is available in Betaflight 4.3+ when GPS passthrough is
    // enabled in the BF Configurator (Ports tab → GPS passthrough).
    static std::vector<uint8_t> wrapUbxInMspPassthrough(
        const std::vector<uint8_t>& ubxFrame);

private:
    // ── Internal helpers ───────────────────────────────────────────────────────
    static int16_t  read16(const std::vector<uint8_t>& b, int i);
    static uint16_t read16u(const std::vector<uint8_t>& b, int i);
    static int32_t  read32(const std::vector<uint8_t>& b, int i);

    // Fletcher-8 checksum over Class + ID + Length + Payload (UBX spec §3.4)
    static void addUbxChecksum(std::vector<uint8_t>& frame);
};