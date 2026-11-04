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
// GPSConfig / GPSConfigResult
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
    uint8_t  fixType = 0;
    uint8_t  numSat = 0;
    double   latitude = 0.0;
    double   longitude = 0.0;
    float    altitudeM = 0.0f;
    uint16_t groundSpeedMs = 0;
    uint16_t groundCourse = 0;
    uint16_t hdop = 9999;

    uint16_t distToHomM = 0;
    int16_t  bearingToHome = 0;
    uint8_t  gpsHeartbeat = 0;

    bool rawValid = false;
    bool compValid = false;
    bool positionUsable = false;
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
// ── DEFINITIVE WIRE FORMAT (BF 4.5.x, M8+ enhanced path) ────────────────────
//
// BF populates GPS_svinfo[] from UBX-NAV-SAT (gps.c):
//   GPS_svinfo[i].chn     = ubxNavSat.svs[i].gnssId   ← gnssId in .chn !
//   GPS_svinfo[i].svid    = ubxNavSat.svs[i].svId
//   GPS_svinfo[i].cno     = ubxNavSat.svs[i].cno
//   GPS_svinfo[i].quality = ubxNavSat.svs[i].flags    ← UBX flags in .quality !
//
// BF serializes to MSP (msp.c):
//   rec[0] = GPS_svinfo[i].chn      → gnssId (0=GPS,1=SBAS,2=Galileo,
//                                              3=BeiDou,5=QZSS,6=GLONASS)
//   rec[1] = GPS_svinfo[i].svid     → satellite vehicle ID
//   rec[2] = GPS_svinfo[i].quality  → UBX flags byte (lower 8 of uint32):
//                                       bits[2:0] = qualityInd (0-7)
//                                       bit[3]    = svUsed
//                                       bits[7:4] = correction flags (ignored)
//   rec[3] = GPS_svinfo[i].cno      → carrier-to-noise dBHz
//
// Enhanced path is active when GPS_numCh > GPS_SV_MAXSATS_LEGACY (16).
// NEO-M10 always reports numCh=32 → always enhanced path.
//
// Fields always 0 in MSP mode (not in 4-byte record): elev, azim, prRes.
// Python UI should hide Elev/Azim columns when sv_source == "MSP".
// =============================================================================
struct SVInfoEntry {
    uint8_t     chn = 0;    // logical slot index within our list
    uint8_t     svid = 0;    // satellite vehicle ID / PRN
    uint8_t     flags = 0;    // compact: bits[2:0]=quality, bit[3]=used
    uint8_t     quality = 0;    // 0-7 UBX qualityInd (from rec[2] bits[2:0])
    uint8_t     cno = 0;    // dBHz signal strength
    int8_t      elev = 0;    // always 0 in MSP mode
    int16_t     azim = 0;    // always 0 in MSP mode
    int16_t     prRes = 0;    // always 0 in MSP mode
    uint8_t     gnssId = 0;    // from rec[0]: 0=GPS,1=SBAS,2=Galileo,
    //              3=BeiDou,5=QZSS,6=GLONASS
    std::string gnssName;       // human-readable GNSS system name
    std::string statusStr;      // "used","fully locked","locked","searching","idle"
    bool        used = false; // true when rec[2] bit[3]=1 OR quality>=4
};

// =============================================================================
// GPSNeoM10 — static parser / builder
// =============================================================================
class GPSNeoM10 {
public:
    // ── MSP parsers ──────────────────────────────────────────────────────────
    static bool parseRaw(const std::vector<uint8_t>& buf, GPSReading& out);
    static bool parseComp(const std::vector<uint8_t>& buf, GPSReading& out);
    static bool parseNavStatus(const std::vector<uint8_t>& buf, NavStatus& out);

    // ── MSP_GPS_SV_INFO (cmd 164) — PRIMARY satellite data path ──────────────
    //
    // Correct rec[0..3] decoding (BF 4.5.x enhanced path, NEO-M10):
    //   gnssId  = rec[0]              (direct — no shift needed)
    //   svid    = rec[1]
    //   quality = rec[2] & 0x07      (UBX qualityInd, lower 3 bits)
    //   svUsed  = (rec[2] & 0x08)!=0 (UBX svUsed flag, bit[3])
    //   cno     = rec[3]
    //
    // DO NOT extract gnssId from rec[2] upper nibble — that byte contains
    // UBX correction status flags, not a packed gnssId+quality field.
    static bool parseMspSvInfo(const std::vector<uint8_t>& buf,
        std::vector<SVInfoEntry>& svList);

    // ── UBX parsers (applyGPSConfig / legacy passthrough) ────────────────────
    static bool parseNavSvInfo(const std::vector<uint8_t>& ubxPayload,
        std::vector<SVInfoEntry>& svList);
    static bool parseAck(const std::vector<uint8_t>& frame,
        uint8_t expectedCls, uint8_t expectedId);

    // ── UBX frame builders ────────────────────────────────────────────────────
    static std::vector<uint8_t> buildNavSvInfoPoll();
    static std::vector<uint8_t> buildCfgGNSS(uint32_t constellationMask);
    static std::vector<uint8_t> buildCfgRate(GPSUpdateRate rate);
    static std::vector<uint8_t> buildCfgPrt(GPSProtocol protocol);
    static std::vector<uint8_t> buildCfgNav5(int elevationMaskDeg);
    static std::vector<uint8_t> buildCfgCfg();

private:
    static void        ubxChecksum(std::vector<uint8_t>& frame);
    static const char* gnssName(uint8_t gnssId);
    static const char* qualityStatus(uint8_t quality);
};