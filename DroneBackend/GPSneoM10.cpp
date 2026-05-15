#include "GPSNeoM10.h"
#include <cstring>
#include <cmath>

// =============================================================================
// Internal helpers
// =============================================================================

static inline uint16_t u16le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
static inline int16_t  i16le(const uint8_t* p) {
    return static_cast<int16_t>(u16le(p));
}
static inline uint32_t u32le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
        | (static_cast<uint32_t>(p[1]) << 8)
        | (static_cast<uint32_t>(p[2]) << 16)
        | (static_cast<uint32_t>(p[3]) << 24);
}
static inline int32_t  i32le(const uint8_t* p) {
    return static_cast<int32_t>(u32le(p));
}

static const uint8_t* mspPayload(const std::vector<uint8_t>& buf,
    uint8_t  expectedCmd,
    size_t   minPayload)
{
    if (buf.size() < 6)             return nullptr;
    if (buf[0] != '$')              return nullptr;
    if (buf[1] != 'M')              return nullptr;
    if (buf[2] != '>')              return nullptr;
    if (buf[4] != expectedCmd)      return nullptr;
    uint8_t payLen = buf[3];
    if (payLen < minPayload)        return nullptr;
    if (buf.size() < (size_t)(6 + payLen)) return nullptr;
    return buf.data() + 5;
}

// =============================================================================
// gnssName() / qualityStatus()
// =============================================================================

const char* GPSNeoM10::gnssName(uint8_t gnssId) {
    switch (gnssId) {
    case 0: return "GPS";
    case 1: return "SBAS";
    case 2: return "Galileo";
    case 3: return "BeiDou";
    case 4: return "IMES";
    case 5: return "QZSS";
    case 6: return "GLONASS";
    default: return "Unknown";
    }
}

const char* GPSNeoM10::qualityStatus(uint8_t quality) {
    switch (quality) {
    case 0: return "idle";
    case 1: return "searching";
    case 2: return "acquired";
    case 3: return "unusable";
    case 4: return "locked";
    case 5:
    case 6:
    case 7: return "fully locked";
    default: return "unknown";
    }
}

// =============================================================================
// parseRaw()  —  MSP_RAW_GPS (cmd 106)
// =============================================================================
bool GPSNeoM10::parseRaw(const std::vector<uint8_t>& buf, GPSReading& out) {
    const uint8_t* p = mspPayload(buf, 106, 16);
    if (!p) return false;

    out.fixType = p[0];
    out.numSat = p[1];
    out.latitude = i32le(p + 2) / 1e7;
    out.longitude = i32le(p + 6) / 1e7;
    out.altitudeM = static_cast<float>(u16le(p + 10)) / 100.0f;
    out.groundSpeedMs = u16le(p + 12);
    out.groundCourse = u16le(p + 14);
    out.hdop = (buf[3] >= 18) ? u16le(p + 16) : 9999;

    out.rawValid = true;
    out.positionUsable = (out.fixType >= 2)
        && (out.numSat >= 4)
        && (out.hdop < 500);
    return true;
}

// =============================================================================
// parseComp()  —  MSP_COMP_GPS (cmd 107)
// =============================================================================
bool GPSNeoM10::parseComp(const std::vector<uint8_t>& buf, GPSReading& out) {
    const uint8_t* p = mspPayload(buf, 107, 5);
    if (!p) return false;

    out.distToHomM = u16le(p);
    out.bearingToHome = i16le(p + 2);
    out.gpsHeartbeat = p[4];
    out.compValid = true;
    return true;
}

// =============================================================================
// parseNavStatus()  —  MSP_NAV_STATUS (cmd 121)
// =============================================================================
bool GPSNeoM10::parseNavStatus(const std::vector<uint8_t>& buf, NavStatus& out) {
    const uint8_t* p = mspPayload(buf, 121, 4);
    if (!p) return false;

    out.fixType = p[0];
    out.gpsFlags = p[1];
    out.fixOk = (p[1] & 0x01) != 0;
    out.dgpsUsed = (p[1] & 0x02) != 0;
    out.mapFlags = p[2];
    out.hwStatus = p[3];
    out.valid = true;
    return true;
}

// =============================================================================
// parseMspSvInfo()  —  MSP_GPS_SV_INFO (cmd 164)
//
// ── DEFINITIVE WIRE FORMAT confirmed from BF 4.5.x source ───────────────────
//
// From gps.c (UBX-NAV-SAT → GPS_svinfo[] population):
//   GPS_svinfo[i].chn     = ubxNavSat.svs[i].gnssId   <-- gnssId in .chn field!
//   GPS_svinfo[i].svid    = ubxNavSat.svs[i].svId
//   GPS_svinfo[i].cno     = ubxNavSat.svs[i].cno
//   GPS_svinfo[i].quality = ubxNavSat.svs[i].flags    <-- full UBX flags in .quality!
//
// From msp.c (GPS_svinfo[] → MSP frame serialization):
//   sbufWriteU8(dst, GPS_svinfo[i].chn);      -> rec[0]
//   sbufWriteU8(dst, GPS_svinfo[i].svid);     -> rec[1]
//   sbufWriteU8(dst, GPS_svinfo[i].quality);  -> rec[2]
//   sbufWriteU8(dst, GPS_svinfo[i].cno);      -> rec[3]
//
// Therefore the 4-byte record wire layout is:
//
//   rec[0]  = gnssId   (0=GPS,1=SBAS,2=Galileo,3=BeiDou,5=QZSS,6=GLONASS)
//             Field is named "chn" in the BF struct but carries gnssId on
//             the M8+ enhanced path (numCh > GPS_SV_MAXSATS_LEGACY=16).
//             NEO-M10 always reports numCh=32, so always enhanced path.
//
//   rec[1]  = svId     (satellite vehicle ID / PRN, direct from UBX)
//
//   rec[2]  = UBX-NAV-SAT flags uint32, lower 8 bits only:
//               bits[2:0] = qualityInd (0=no signal .. 7=fully locked)
//               bit[3]    = svUsed    (1 = contributes to nav solution)
//               bit[4]    = diffSoln
//               bits[7:5] = other correction flags (ignored here)
//
//   rec[3]  = cno      (carrier-to-noise density, dBHz)
//
// ── HOW PREVIOUS PARSERS WERE WRONG ─────────────────────────────────────────
//
//   Versions 1 & 2 assumed rec[0]=channel_index and rec[2]=(gnssId<<4)|quality.
//   This is the LEGACY format (M6/M7 hardware, UBX-NAV-SVINFO). The NEO-M10
//   uses the enhanced format (UBX-NAV-SAT) where BF repurposes the .chn field
//   to hold gnssId, and .quality holds the raw UBX flags byte.
//
//   The symptom: all satellites decoded as SBAS (gnssId=1) because the old
//   code did (rec[2]>>4)&0x0F on the UBX flags byte. For a "fully locked"
//   used GPS satellite, flags≈0x0F, giving upper nibble=0 (GPS, OK), but for
//   "code locked" used satellites, flags=0x09, giving upper nibble=0 still.
//   The actual SBAS/BeiDou misidentification came from rec[2] upper nibble
//   being UBX correction flag bits rather than gnssId.
// =============================================================================
bool GPSNeoM10::parseMspSvInfo(const std::vector<uint8_t>& buf,
    std::vector<SVInfoEntry>& svList)
{
    // ── Validate MSP frame ────────────────────────────────────────────────────
    if (buf.size() < 8) return false;
    if (buf[0] != '$' || buf[1] != 'M' || buf[2] != '>') return false;
    if (buf[4] != 164) return false;

    uint8_t payLen = buf[3];
    if (buf.size() < static_cast<size_t>(6 + payLen)) return false;

    uint8_t numCh = buf[5];  // GPS_numCh from firmware
    if (numCh == 0) {
        svList.clear();
        return true;
    }

    if (static_cast<size_t>(payLen) < 1u + static_cast<size_t>(numCh) * 4u)
        return false;

    // ── Parse satellite records ───────────────────────────────────────────────
    std::vector<SVInfoEntry> result;
    result.reserve(numCh);

    const uint8_t* base = buf.data() + 6;  // buf[6] = first record byte

    for (uint8_t i = 0; i < numCh; ++i) {
        const uint8_t* rec = base + i * 4;

        // rec[0]: gnssId — stored in GPS_svinfo[].chn by BF on M8+ path
        uint8_t gnssId = rec[0];

        // rec[1]: svId — satellite vehicle ID / PRN
        uint8_t svid = rec[1];

        // rec[2]: UBX-NAV-SAT flags (lower 8 bits of the uint32 flags field)
        //   bits[2:0] = qualityInd  (signal quality 0-7)
        //   bit[3]    = svUsed      (1 = used in navigation solution)
        //   bits[7:4] = correction flags (diffSoln, sbasCorrUsed, etc.) — ignored
        uint8_t ubxFlags = rec[2];
        uint8_t quality = ubxFlags & 0x07;
        bool    svUsed = (ubxFlags & 0x08) != 0;

        // rec[3]: cno — carrier-to-noise density in dBHz
        uint8_t cno = rec[3];

        SVInfoEntry sv;
        sv.chn = i;          // our logical slot index
        sv.svid = svid;
        sv.quality = quality;
        sv.cno = cno;
        sv.elev = 0;          // not present in 4-byte MSP record
        sv.azim = 0;
        sv.prRes = 0;
        sv.gnssId = gnssId;
        sv.gnssName = gnssName(gnssId);

        // svUsed (bit[3] of UBX flags) is the primary indicator.
        // quality >= 4 is kept as fallback for any edge-case firmware variant.
        sv.used = svUsed || (quality >= 4);

        // Reconstruct compact flags byte (UBX-compatible) for legacy consumers
        sv.flags = (quality & 0x07) | (sv.used ? 0x08 : 0x00);

        // Status string matching BF Configurator GPS Signal Strength labels
        if (sv.used) {
            sv.statusStr = "used";
        }
        else if (cno > 0) {
            sv.statusStr = qualityStatus(quality);
        }
        else {
            sv.statusStr = (quality == 1 || quality == 2)
                ? qualityStatus(quality) : "idle";
        }

        result.push_back(std::move(sv));
    }

    svList = std::move(result);
    return true;
}

// =============================================================================
// parseNavSvInfo()  —  UBX-NAV-SAT payload parser (legacy passthrough path)
// =============================================================================
bool GPSNeoM10::parseNavSvInfo(const std::vector<uint8_t>& ubxPayload,
    std::vector<SVInfoEntry>& svList)
{
    if (ubxPayload.size() < 8) return false;

    uint8_t numSvs = ubxPayload[5];
    if (ubxPayload.size() < static_cast<size_t>(8 + numSvs * 12)) return false;

    std::vector<SVInfoEntry> result;
    result.reserve(numSvs);

    for (uint8_t i = 0; i < numSvs; ++i) {
        const uint8_t* sv = ubxPayload.data() + 8 + i * 12;

        uint8_t  gnssId = sv[0];
        uint8_t  svid = sv[1];
        uint8_t  cno = sv[2];
        int8_t   elev = static_cast<int8_t>(sv[3]);
        int16_t  azim = i16le(sv + 4);
        int16_t  prRes = i16le(sv + 6);
        uint32_t flags = u32le(sv + 8);
        uint8_t  quality = flags & 0x07;
        bool     used = (flags & 0x08) != 0;

        SVInfoEntry e;
        e.chn = i;
        e.svid = svid;
        e.quality = quality;
        e.cno = cno;
        e.elev = elev;
        e.azim = azim;
        e.prRes = prRes;
        e.gnssId = gnssId;
        e.gnssName = gnssName(gnssId);
        e.used = used;
        e.flags = static_cast<uint8_t>((quality & 0x07) | (used ? 0x08 : 0x00));

        if (used) {
            e.statusStr = "used";
        }
        else if (cno > 0) {
            e.statusStr = qualityStatus(quality);
        }
        else {
            e.statusStr = qualityStatus(quality);
        }

        result.push_back(std::move(e));
    }

    svList = std::move(result);
    return true;
}

// =============================================================================
// parseAck()  —  UBX-ACK-ACK / UBX-ACK-NAK
// =============================================================================
bool GPSNeoM10::parseAck(const std::vector<uint8_t>& frame,
    uint8_t expectedCls, uint8_t expectedId)
{
    if (frame.size() < 10) return false;
    if (frame[0] != 0xB5 || frame[1] != 0x62) return false;
    if (frame[2] != 0x05) return false;
    if (frame[3] != 0x01) return false;
    if (frame[6] != expectedCls) return false;
    if (frame[7] != expectedId)  return false;
    return true;
}

// =============================================================================
// ubxChecksum()
// =============================================================================
void GPSNeoM10::ubxChecksum(std::vector<uint8_t>& frame) {
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        a += frame[i];
        b += a;
    }
    frame.push_back(a);
    frame.push_back(b);
}

// =============================================================================
// buildNavSvInfoPoll()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildNavSvInfoPoll() {
    std::vector<uint8_t> f = { 0xB5, 0x62, 0x01, 0x35, 0x00, 0x00 };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgGNSS()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgGNSS(uint32_t mask) {
    struct Block { uint8_t id, res, max, rsv; uint8_t en; uint8_t f1, f2, f3; };
    const Block blocks[] = {
        { 0, 8, 16, 0, (mask & GNSS_GPS) ? 1u : 0u, 0x01, 0x00, 0x01 },
        { 1, 1,  3, 0, (mask & GNSS_SBAS) ? 1u : 0u, 0x01, 0x00, 0x01 },
        { 2, 4,  8, 0, (mask & GNSS_GALILEO) ? 1u : 0u, 0x01, 0x00, 0x01 },
        { 3, 8, 16, 0, (mask & GNSS_BEIDOU) ? 1u : 0u, 0x01, 0x00, 0x01 },
        { 4, 0,  8, 0, 0,                                0x03, 0x00, 0x01 },
        { 5, 0,  3, 0, (mask & GNSS_QZSS) ? 1u : 0u, 0x05, 0x00, 0x01 },
        { 6, 8, 14, 0, (mask & GNSS_GLONASS) ? 1u : 0u, 0x01, 0x00, 0x01 },
    };

    const uint8_t numBlocks = sizeof(blocks) / sizeof(blocks[0]);
    std::vector<uint8_t> payload;
    payload.push_back(0x00);
    payload.push_back(0x20);
    payload.push_back(0x20);
    payload.push_back(numBlocks);
    for (auto& b : blocks) {
        payload.push_back(b.id);
        payload.push_back(b.res);
        payload.push_back(b.max);
        payload.push_back(b.rsv);
        payload.push_back(b.en);
        payload.push_back(b.f1);
        payload.push_back(b.f2);
        payload.push_back(b.f3);
    }

    uint16_t payLen = static_cast<uint16_t>(payload.size());
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x3E,
        static_cast<uint8_t>(payLen & 0xFF),
        static_cast<uint8_t>(payLen >> 8)
    };
    f.insert(f.end(), payload.begin(), payload.end());
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgRate()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgRate(GPSUpdateRate rate) {
    uint16_t measRateMs;
    switch (rate) {
    case GPSUpdateRate::RATE_2HZ:  measRateMs = 500;  break;
    case GPSUpdateRate::RATE_5HZ:  measRateMs = 200;  break;
    case GPSUpdateRate::RATE_10HZ: measRateMs = 100;  break;
    default:                       measRateMs = 1000; break;
    }
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        static_cast<uint8_t>(measRateMs & 0xFF),
        static_cast<uint8_t>(measRateMs >> 8),
        0x01, 0x00,
        0x01, 0x00
    };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgPrt()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgPrt(GPSProtocol /*protocol*/) {
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,
        0x00, 0xC2, 0x01, 0x00,
        0x01, 0x00,
        0x01, 0x00,
        0x00, 0x00,
        0x00, 0x00
    };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgNav5()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgNav5(int elevationMaskDeg) {
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
        0x03, 0x00, 0x08, 0x03,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        static_cast<uint8_t>(elevationMaskDeg & 0xFF),
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00
    };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgCfg()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgCfg() {
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x1F, 0x1F, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x07
    };
    ubxChecksum(f);
    return f;
}