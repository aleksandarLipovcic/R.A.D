#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>

// =============================================================================
// GPS enums — used by GPSConfig
// =============================================================================

enum GPSConstellationFlags : uint32_t {
    GNSS_GPS = (1 << 0),
    GNSS_SBAS = (1 << 1),
    GNSS_GALILEO = (1 << 2),
    GNSS_BEIDOU = (1 << 3),
    GNSS_QZSS = (1 << 4),
    GNSS_GLONASS = (1 << 5),
    GNSS_DEFAULT = GNSS_GPS | GNSS_GALILEO | GNSS_GLONASS,
    GNSS_DUAL = GNSS_GPS | GNSS_GLONASS,
    GNSS_TRIPLE = GNSS_GPS | GNSS_GALILEO | GNSS_GLONASS,
    GNSS_ALL = GNSS_GPS | GNSS_SBAS | GNSS_GALILEO |
    GNSS_BEIDOU | GNSS_QZSS | GNSS_GLONASS,
};

enum class GPSUpdateRate { RATE_1HZ, RATE_2HZ, RATE_5HZ, RATE_10HZ };
enum class GPSProtocol { UBLOX, NMEA, MSP };

// =============================================================================
// GPSConfig — passed to DroneLink::applyGPSConfig()
// =============================================================================
struct GPSConfig {
    uint32_t      constellations = GNSS_DEFAULT;
    GPSUpdateRate updateRateHz = GPSUpdateRate::RATE_1HZ;
    GPSProtocol   protocol = GPSProtocol::UBLOX;
    bool          sbasEnabled = false;
    int           elevationMaskDeg = 5;
    int           signalMaskDbHz = 0;
};

struct GPSConfigResult {
    bool        gnssAck = false;
    bool        rateAck = false;
    bool        protocolAck = false;
    bool        saveAck = false;
    bool        overallOk = false;
    std::string errorDetail;
};

// =============================================================================
// GPSReading — populated by parseRaw() + parseComp()
// =============================================================================
struct GPSReading {
    // MSP_RAW_GPS (106)
    uint8_t  fixType = 0;    // 0=nofix 1=deadreck 2=2D 3=3D 4=gps+dr 5=time
    uint8_t  numSat = 0;
    double   latitude = 0.0;  // decimal degrees
    double   longitude = 0.0;
    float    altitudeM = 0.0f; // metres MSL
    uint16_t groundSpeedMs = 0;    // cm/s
    uint16_t groundCourse = 0;    // decidegrees (0-3599)
    uint16_t hdop = 9999; // x100; 9999 = unknown

    // MSP_COMP_GPS (107)
    uint16_t distToHomM = 0;    // metres
    int16_t  bearingToHome = 0;    // degrees -180..+180
    uint8_t  gpsHeartbeat = 0;    // toggles 0↔1 on each fresh frame

    // Validity flags
    bool rawValid = false;
    bool compValid = false;
    bool positionUsable = false;   // fixType>=2 && numSat>=4 && hdop<500
};

// =============================================================================
// NavStatus — MSP_NAV_STATUS (121)
// =============================================================================
struct NavStatus {
    uint8_t fixType = 0;
    uint8_t gpsFlags = 0;
    bool    fixOk = false;
    bool    dgpsUsed = false;
    uint8_t mapFlags = 0;
    uint8_t hwStatus = 0;
    bool    valid = false;
};

// =============================================================================
// SVInfoEntry — one satellite channel
//
// Populated by GPSNeoM10::parseMspSvInfo() from MSP_GPS_SV_INFO (cmd 164).
//
// Confirmed payload layout (v6 probe, XFlight F405 V3 / BF 4.5.3 / NEO-M10):
//   Byte 0       : numCh = 0x20 (32 channels)
//   Per channel (4 bytes):
//     [0] chn    : channel byte — upper nibble = GNSS ID (when numCh > 16)
//                                 lower nibble = channel index
//     [1] svid   : satellite vehicle ID
//     [2] quality: signal quality 0-7 (matches UBX qualityInd)
//     [3] cno    : carrier-to-noise dBHz (0 = not tracked)
//
// Fields not available in MSP mode (always 0): elev, azim, prRes.
// Python UI should check sv_source == "MSP" and hide Elev/Azim columns.
// =============================================================================
struct SVInfoEntry {
    uint8_t     chn = 0;    // channel index (lower nibble when numCh>16)
    uint8_t     svid = 0;    // satellite vehicle ID / PRN
    uint8_t     flags = 0;    // packed: bits[0:2]=quality, bit[3]=used
    uint8_t     quality = 0;    // 0-7 UBX qualityInd
    uint8_t     cno = 0;    // dBHz signal strength
    int8_t      elev = 0;    // degrees (-90..+90) — 0 in MSP mode
    int16_t     azim = 0;    // degrees (0..360)   — 0 in MSP mode
    int16_t     prRes = 0;    // pseudorange residual — 0 in MSP mode
    uint8_t     gnssId = 0;    // 0=GPS 1=SBAS 2=Galileo 3=BeiDou 5=QZSS 6=GLONASS
    std::string gnssName;         // "GPS", "GLONASS", "Galileo", etc.
    std::string statusStr;        // "used", "tracked", "acquired", "searching", "idle"
    bool        used = false; // true when quality >= 4
};

// =============================================================================
// GPSNeoM10 — static parser / builder namespace
// =============================================================================
class GPSNeoM10 {
public:
    // ── MSP parsers ──────────────────────────────────────────────────────────

    // MSP_RAW_GPS (cmd 106) — 18-byte payload
    static bool parseRaw(const std::vector<uint8_t>& buf, GPSReading& out);

    // MSP_COMP_GPS (cmd 107) — 5-byte payload
    static bool parseComp(const std::vector<uint8_t>& buf, GPSReading& out);

    // MSP_NAV_STATUS (cmd 121)
    static bool parseNavStatus(const std::vector<uint8_t>& buf, NavStatus& out);

    // ── MSP_GPS_SV_INFO (cmd 164) ─────────────────────────────────────────────
    //
    // PRIMARY satellite data path.  No passthrough required.
    // Called from DroneLink::pollSatellitesMSP() every SV_POLL_TICKS.
    //
    // Confirmed format (NEO-M10, BF 4.5.3):
    //   buf[0..4] = MSP header ($M> payLen cmd)
    //   buf[5]    = numCh (32 = 0x20 for M10)
    //   buf[6..6+numCh*4-1] = channel records (4 bytes each)
    //   buf[last] = checksum
    //
    // GNSS ID encoding (numCh > 16):
    //   chn byte = (gnss_id << 4) | channel_index
    //   gnss_id : 0=GPS 1=SBAS 2=Galileo 3=BeiDou 5=QZSS 6=GLONASS
    //
    // Returns true and fills svList on success.
    // Returns false (leaves svList unchanged) on any parse failure.
    static bool parseMspSvInfo(const std::vector<uint8_t>& buf,
        std::vector<SVInfoEntry>& svList);

    // ── UBX parsers (used by applyGPSConfig / legacy passthrough) ────────────

    // UBX-NAV-SAT response parser (passthrough path, kept for applyGPSConfig)
    static bool parseNavSvInfo(const std::vector<uint8_t>& ubxPayload,
        std::vector<SVInfoEntry>& svList);

    // UBX ACK/NAK check
    static bool parseAck(const std::vector<uint8_t>& frame,
        uint8_t expectedCls, uint8_t expectedId);

    // ── UBX frame builders (for applyGPSConfig) ───────────────────────────────
    static std::vector<uint8_t> buildNavSvInfoPoll();
    static std::vector<uint8_t> buildCfgGNSS(uint32_t constellationMask);
    static std::vector<uint8_t> buildCfgRate(GPSUpdateRate rate);
    static std::vector<uint8_t> buildCfgPrt(GPSProtocol protocol);
    static std::vector<uint8_t> buildCfgNav5(int elevationMaskDeg);
    static std::vector<uint8_t> buildCfgCfg();

private:
    // UBX checksum (Fletcher over bytes cls..payload end)
    static void ubxChecksum(std::vector<uint8_t>& frame);

    // GNSS ID → name string
    static const char* gnssName(uint8_t gnssId);

    // Quality index → status string
    static const char* qualityStatus(uint8_t quality);
};