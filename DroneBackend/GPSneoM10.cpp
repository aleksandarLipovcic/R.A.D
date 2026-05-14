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

// Minimal MSP frame validation.
// Returns pointer to the first payload byte, or nullptr on failure.
// minPayload is the minimum number of payload bytes required.
static const uint8_t* mspPayload(const std::vector<uint8_t>& buf,
    uint8_t  expectedCmd,
    size_t   minPayload)
{
    // Frame: $M> payLen cmd [payload...] csum
    if (buf.size() < 6)             return nullptr;
    if (buf[0] != '$')              return nullptr;
    if (buf[1] != 'M')              return nullptr;
    if (buf[2] != '>')              return nullptr;
    if (buf[4] != expectedCmd)      return nullptr;
    uint8_t payLen = buf[3];
    if (payLen < minPayload)        return nullptr;
    if (buf.size() < (size_t)(6 + payLen)) return nullptr;
    return buf.data() + 5;          // buf[5] = first payload byte
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
    // Mirrors BF Configurator GPS Signal Strength panel labels
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
//
// BF 4.x payload layout (18 bytes confirmed on F405 V3):
//   [0]     fixType       uint8
//   [1]     numSat        uint8
//   [2..5]  lat           int32 (degrees × 1e7)
//   [6..9]  lon           int32 (degrees × 1e7)
//   [10..11] altitude     uint16 (cm MSL, unsigned)
//   [12..13] groundSpeed  uint16 (cm/s)
//   [14..15] groundCourse uint16 (decidegrees 0-3599)
//   [16..17] hdop         uint16 (× 100; 9999 = unknown — present in BF 4.x)
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
        && (out.hdop < 500);   // HDOP < 5.00
    return true;
}

// =============================================================================
// parseComp()  —  MSP_COMP_GPS (cmd 107)
//
//   [0..1]  distToHome   uint16 (metres)
//   [2..3]  bearing      int16  (degrees -180..+180)
//   [4]     gpsHeartbeat uint8  (toggles 0↔1 on each fresh frame)
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
//
//   [0] fixType
//   [1] gpsFlags  (bit0 = fixOk, bit1 = dgpsUsed)
//   [2] mapFlags
//   [3] hwStatus
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
// parseMspSvInfo()  —  MSP_GPS_SV_INFO (cmd 164)   ← PRIMARY SATELLITE PATH
//
// Confirmed payload layout from live probe (XFlight F405 V3 / BF 4.5.3 / NEO-M10):
//
//   buf[3]  payLen  =  1 + numCh * 4
//           For 32 channels (NEO-M10): payLen = 129 (0x81)
//   buf[4]  cmd     = 164
//   buf[5]  numCh   = 32 (0x20) for NEO-M10
//   buf[6+] channel records, 4 bytes each
//
//   Channel record (4 bytes):
//     rec[0]  chn      uint8
//                      bits[7:4] = gnssId    (when numCh > 16)
//                      bits[3:0] = chnIdx
//     rec[1]  svid     uint8   satellite vehicle ID / PRN
//     rec[2]  packed   uint8   PACKED BYTE — confirmed from debug values 0x11, 0x1C:
//                                bits[7:4] = gnssId  (GNSS system ID 0-6)
//                                bits[3:0] = quality (UBX qualityInd 0-7)
//                              Example: 0x1C → gnssId=1 (SBAS), quality=4 (locked)
//                              Example: 0x11 → gnssId=1 (SBAS), quality=1 (searching)
//     rec[3]  cno      uint8   carrier-to-noise density, dBHz (0 = not tracked)
//
//   elev and azim are NOT present in this 4-byte record; always 0 in MSP mode.
//   prRes is also not present; always 0.
//
// "used" flag: BF MSP_GPS_SV_INFO has no separate svUsed bit.
//   quality >= 4 means the satellite contributes to the fix (BF convention).
//
// GNSS IDs (from rec[2] upper nibble, confirmed matches rec[0] upper nibble):
//   0 = GPS      1 = SBAS    2 = Galileo  3 = BeiDou
//   4 = IMES     5 = QZSS    6 = GLONASS
//
// Quality / status mapping (matches BF Configurator GPS Signal Strength labels):
//   quality 0  → "idle"          quality 4  → "locked"  (used)
//   quality 1  → "searching"     quality 5  → "fully locked"  (used)
//   quality 2  → "acquired"      quality 6  → "fully locked"  (used)
//   quality 3  → "unusable"      quality 7  → "fully locked"  (used)
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

    // buf[5] = numCh
    uint8_t numCh = buf[5];
    if (numCh == 0) {
        svList.clear();
        return true;    // valid response, just no channels yet
    }

    // ── Validate stride: 4 bytes per channel + 1 byte numCh header ───────────
    // payLen = 1 + numCh * 4  →  129 for 32 channels on NEO-M10 / BF 4.5.3.
    // Use size_t arithmetic to avoid uint8_t overflow for large numCh values.
    if (static_cast<size_t>(payLen) < 1u + static_cast<size_t>(numCh) * 4u)
        return false;

    // GNSS ID is encoded in the upper nibble of rec[2] (the packed quality byte).
    // rec[0] upper nibble also carries gnssId and can be used for cross-check,
    // but rec[2] is authoritative since it was confirmed from live debug values.
    // For legacy modules with numCh <= 16 the upper nibble of rec[0] is not
    // reliably the GNSS ID; in that case we fall back to gnssId = 0 (GPS).
    const bool hasGnssId = (numCh > 16);

    // ── Parse channels ────────────────────────────────────────────────────────
    std::vector<SVInfoEntry> result;
    result.reserve(numCh);

    const uint8_t* base = buf.data() + 6;   // buf[6] = first channel record

    for (uint8_t i = 0; i < numCh; ++i) {
        const uint8_t* rec = base + i * 4;  // 4 bytes per record

        uint8_t chnByte = rec[0];
        uint8_t svid = rec[1];
        uint8_t packed = rec[2];  // bits[7:4]=gnssId, bits[3:0]=qualityInd
        uint8_t cno = rec[3];

        // Unpack rec[2] — confirmed from debug output (quality=0x11 → gnss=1,qual=1)
        uint8_t gnssId = hasGnssId ? ((packed >> 4) & 0x0F) : 0;
        uint8_t quality = (packed) & 0x0F;

        // chn index from lower nibble of rec[0] when gnss is in upper nibble
        uint8_t chnIdx = hasGnssId ? (chnByte & 0x0F) : chnByte;

        SVInfoEntry sv;
        sv.chn = chnIdx;
        sv.svid = svid;
        sv.quality = quality;          // correctly unpacked 0-7
        sv.cno = cno;
        sv.elev = 0;                // not in 4-byte record
        sv.azim = 0;
        sv.prRes = 0;
        sv.gnssId = gnssId;
        sv.gnssName = gnssName(gnssId);

        // quality >= 4 → satellite contributes to the fix (BF convention)
        sv.used = (quality >= 4);

        // Pack flags byte for UBX-compatible legacy code: bits[0:2]=quality, bit[3]=used
        sv.flags = (quality & 0x07) | (sv.used ? 0x08 : 0x00);

        // Status string — matches BF Configurator GPS Signal Strength panel labels
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
// parseNavSvInfo()  —  UBX-NAV-SAT payload parser
//
// Used by the legacy passthrough path (applyGPSConfig / pollSatellites).
// ubxPayload is the raw payload bytes (after the 6-byte UBX header).
//
// UBX-NAV-SAT payload layout (u-blox M10 protocol spec):
//   [0..3]   iTOW     uint32
//   [4]      version  uint8  (0x01 for M10)
//   [5]      numSvs   uint8
//   [6..7]   reserved
//   Per SV (12 bytes):
//     [0]   gnssId   uint8
//     [1]   svId     uint8
//     [2]   cno      uint8   dBHz
//     [3]   elev     int8    degrees
//     [4..5] azim    int16   degrees
//     [6..7] prRes   int16   × 0.1 m
//     [8..11] flags  uint32  bits[0:2]=qualityInd, bit[3]=svUsed
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
    // UBX frame: 0xB5 0x62 cls id payLen_lo payLen_hi [payload] ckA ckB
    if (frame.size() < 10) return false;
    if (frame[0] != 0xB5 || frame[1] != 0x62) return false;
    if (frame[2] != 0x05) return false;   // ACK class
    if (frame[3] != 0x01) return false;   // 0x01=ACK-ACK; 0x00=ACK-NAK
    if (frame[6] != expectedCls) return false;
    if (frame[7] != expectedId)  return false;
    return true;
}

// =============================================================================
// ubxChecksum()
// Computes Fletcher-8 over bytes [2..end-2] (cls through payload end),
// appends CK_A and CK_B.
// Call after the full frame (including 0xB5 0x62 preamble) is in `frame`
// but before the two checksum bytes have been appended.
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
// buildNavSvInfoPoll()  —  UBX-NAV-SAT poll request
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildNavSvInfoPoll() {
    std::vector<uint8_t> f = { 0xB5, 0x62, 0x01, 0x35, 0x00, 0x00 };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgGNSS()  —  UBX-CFG-GNSS
// Enables/disables constellation blocks based on constellationMask.
// Sends 8 blocks (all known constellations); each block = 8 bytes.
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgGNSS(uint32_t mask) {
    // Block layout: gnssId, resTrkCh, maxTrkCh, reserved1,
    //               flags_lo, flags_hi, flags2_lo, flags2_hi
    // flags bit0 = enable
    struct Block { uint8_t id, res, max, rsv; uint8_t en; uint8_t f1, f2, f3; };
    const Block blocks[] = {
        { 0, 8, 16, 0, (mask & GNSS_GPS) ? 1u : 0u, 0x01, 0x00, 0x01 }, // GPS
        { 1, 1,  3, 0, (mask & GNSS_SBAS) ? 1u : 0u, 0x01, 0x00, 0x01 }, // SBAS
        { 2, 4,  8, 0, (mask & GNSS_GALILEO) ? 1u : 0u, 0x01, 0x00, 0x01 }, // Galileo
        { 3, 8, 16, 0, (mask & GNSS_BEIDOU) ? 1u : 0u, 0x01, 0x00, 0x01 }, // BeiDou
        { 4, 0,  8, 0, 0,                                0x03, 0x00, 0x01 }, // IMES (off)
        { 5, 0,  3, 0, (mask & GNSS_QZSS) ? 1u : 0u, 0x05, 0x00, 0x01 }, // QZSS
        { 6, 8, 14, 0, (mask & GNSS_GLONASS) ? 1u : 0u, 0x01, 0x00, 0x01 }, // GLONASS
    };

    const uint8_t numBlocks = sizeof(blocks) / sizeof(blocks[0]);
    // Payload: msgVer(1) + numTrkChHw(1) + numTrkChUse(1) + msgVer(1) + numBlocks*8
    std::vector<uint8_t> payload;
    payload.push_back(0x00);       // msgVer
    payload.push_back(0x20);       // numTrkChHw  (32 tracking channels)
    payload.push_back(0x20);       // numTrkChUse (32)
    payload.push_back(numBlocks);
    for (auto& b : blocks) {
        payload.push_back(b.id);
        payload.push_back(b.res);
        payload.push_back(b.max);
        payload.push_back(b.rsv);
        payload.push_back(b.en);   // flags byte 0: bit0 = enable
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
// buildCfgRate()  —  UBX-CFG-RATE
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgRate(GPSUpdateRate rate) {
    uint16_t measRateMs;
    switch (rate) {
    case GPSUpdateRate::RATE_2HZ:  measRateMs = 500;  break;
    case GPSUpdateRate::RATE_5HZ:  measRateMs = 200;  break;
    case GPSUpdateRate::RATE_10HZ: measRateMs = 100;  break;
    default:                       measRateMs = 1000; break;  // 1 Hz
    }
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        static_cast<uint8_t>(measRateMs & 0xFF),
        static_cast<uint8_t>(measRateMs >> 8),
        0x01, 0x00,   // navRate = 1 (one nav per measurement)
        0x01, 0x00    // timeRef = 1 (GPS time)
    };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgPrt()  —  UBX-CFG-PRT (UART1, 115200 baud, UBX in+out)
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgPrt(GPSProtocol /*protocol*/) {
    // Always configure for UBX binary; NMEA output disabled.
    // portID=1 (UART1), mode=8N1, baud=115200, inProto=UBX, outProto=UBX
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x01,                           // portID = UART1
        0x00,                           // reserved0
        0x00, 0x00,                     // txReady
        0xD0, 0x08, 0x00, 0x00,        // mode: 8N1
        0x00, 0xC2, 0x01, 0x00,        // baudRate = 115200 (0x0001C200)
        0x01, 0x00,                     // inProtoMask: UBX
        0x01, 0x00,                     // outProtoMask: UBX
        0x00, 0x00,                     // flags
        0x00, 0x00                      // reserved1
    };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgNav5()  —  UBX-CFG-NAV5
// Sets dynamic model (airborne <1g) and elevation mask.
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgNav5(int elevationMaskDeg) {
    // mask: bit0=dyn, bit1=minEl — set both
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
        0x03, 0x00,         // mask: dynModel + minElev
        0x08,               // dynModel = 8 (airborne < 1g)
        0x03,               // fixMode = 3 (auto 2D/3D)
        0x00, 0x00, 0x00, 0x00,  // fixedAlt
        0x00, 0x00, 0x00, 0x00,  // fixedAltVar
        static_cast<uint8_t>(elevationMaskDeg & 0xFF),  // minElev
        0x00,               // drLimit
        0x00, 0x00,         // pDop
        0x00, 0x00,         // tDop
        0x00, 0x00,         // pAcc
        0x00, 0x00,         // tAcc
        0x00,               // staticHoldThresh
        0x00,               // dgnssTimeout
        0x00,               // cnoThreshNumSVs
        0x00,               // cnoThresh
        0x00, 0x00,         // reserved1
        0x00, 0x00,         // staticHoldMaxDist
        0x00,               // utcStandard
        0x00, 0x00, 0x00, 0x00, 0x00  // reserved2
    };
    ubxChecksum(f);
    return f;
}

// =============================================================================
// buildCfgCfg()  —  UBX-CFG-CFG (save all sections to flash + BBR)
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgCfg() {
    // clearMask=0, saveMask=0x1F1F (all), loadMask=0
    // deviceMask = 0x07 (BBR + flash + I2C-EEPROM)
    std::vector<uint8_t> f = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0x00, 0x00, 0x00, 0x00,   // clearMask
        0x1F, 0x1F, 0x00, 0x00,   // saveMask
        0x00, 0x00, 0x00, 0x00,   // loadMask
        0x07                       // deviceMask
    };
    ubxChecksum(f);
    return f;
}