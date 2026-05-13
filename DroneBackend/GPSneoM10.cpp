#include "GPSNeoM10.h"
#include <cstring>

// =============================================================================
// Internal helpers -- little-endian reads
//
// FIX: All shift operands are explicitly cast to uint16_t / uint32_t before
// shifting. Without the cast, uint8_t is promoted to signed int by the C++
// integer promotion rules, and shifting a signed int can overflow (undefined
// behaviour) or trigger MSVC warning C4334 / Int-arith sub-expression overflow.
// =============================================================================

int16_t GPSNeoM10::read16(const std::vector<uint8_t>& b, int i) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(b[i]) |
        (static_cast<uint16_t>(b[i + 1]) << 8)
        );
}

uint16_t GPSNeoM10::read16u(const std::vector<uint8_t>& b, int i) {
    return static_cast<uint16_t>(
        static_cast<uint16_t>(b[i]) |
        (static_cast<uint16_t>(b[i + 1]) << 8)
        );
}

int32_t GPSNeoM10::read32(const std::vector<uint8_t>& b, int i) {
    return static_cast<int32_t>(
        static_cast<uint32_t>(b[i])
        | (static_cast<uint32_t>(b[i + 1]) << 8)
        | (static_cast<uint32_t>(b[i + 2]) << 16)
        | (static_cast<uint32_t>(b[i + 3]) << 24)
        );
}

// =============================================================================
// addUbxChecksum()
//
// Appends CK_A and CK_B (Fletcher-8) to a UBX frame.
// Per u-blox Integration Manual s3.4, the checksum covers bytes from the
// Class field (index 2) to the last payload byte (inclusive).
// Preamble bytes 0xB5, 0x62 at [0] and [1] are excluded.
// =============================================================================
void GPSNeoM10::addUbxChecksum(std::vector<uint8_t>& frame) {
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    frame.push_back(ck_a);
    frame.push_back(ck_b);
}

// =============================================================================
// parseRaw() -- MSP_RAW_GPS (command ID 106 / 0x6A)
//
// Betaflight 4.x MSP_RAW_GPS payload:
//
//   Offset  Size  Type     Description
//   ------  ----  -------  ---------------------------------------------------
//   5       1     uint8    GPS fix type: 0 = none, 1 = 2D, 2 = 3D
//   6       1     uint8    Number of satellites used
//   7       4     int32    Latitude  x 10 000 000  (452345678 = 45.2345678 deg)
//   11      4     int32    Longitude x 10 000 000
//   15      2     uint16   Altitude MSL, metres
//   17      2     uint16   Ground speed, cm/s
//   19      2     uint16   Ground course, decidegrees (0-3599)
//   21      2     uint16   HDOP x100  (BF >= 4.1 only; present when size=18)
//
// Minimum valid frame: header(5) + 16 bytes payload + 1 checksum = 22 bytes.
// With HDOP:           header(5) + 18 bytes payload + 1 checksum = 24 bytes.
//
// positionUsable gate:
//   Set true only when fixType>=2, numSat>=4, and HDOP<500 (5.0).
//   Always gate coordinate display on positionUsable, not rawValid alone.
// =============================================================================
bool GPSNeoM10::parseRaw(const std::vector<uint8_t>& buf, GPSReading& out) {
    if (buf.size() < 22) return false;
    if (buf[4] != 106)   return false;

    out.fixType = buf[5];
    out.numSat = buf[6];
    out.latitude = read32(buf, 7) / 10000000.0;
    out.longitude = read32(buf, 11) / 10000000.0;
    out.altitudeM = read16u(buf, 15);
    out.groundSpeedMs = read16u(buf, 17);
    out.groundCourse = read16u(buf, 19);

    // HDOP present only when payload size >= 18 (frame >= 24 bytes total)
    if (buf.size() >= 24)
        out.hdop = read16u(buf, 21);
    else
        out.hdop = 9999;   // unknown -- older firmware or missing field

    out.rawValid = true;

    // Position usability gate
    // hdop == 9999 (unknown) intentionally fails the < 500 test.
    out.positionUsable = (out.fixType >= 2)
        && (out.numSat >= 4)
        && (out.hdop < 500);

    return true;
}

// =============================================================================
// parseComp() -- MSP_COMP_GPS (command ID 107 / 0x6B)
//
//   Offset  Size  Type     Description
//   ------  ----  -------  ---------------------------------------------------
//   5       2     uint16   Distance to home point, metres
//   7       2     int16    Bearing to home, degrees (-180 to +180)
//   9       1     uint8    GPS heartbeat -- toggles on every fresh GPS frame
//
// Total frame: header(5) + 5 bytes payload + 1 checksum = 11 bytes.
// =============================================================================
bool GPSNeoM10::parseComp(const std::vector<uint8_t>& buf, GPSReading& out) {
    if (buf.size() < 11) return false;
    if (buf[4] != 107)   return false;

    out.distToHomM = read16u(buf, 5);
    out.bearingToHome = read16(buf, 7);
    out.gpsHeartbeat = buf[9];
    out.compValid = true;
    return true;
}

// =============================================================================
// buildNavSvInfoPoll() -- UBX-NAV-SAT (Class 0x01, ID 0x35)
//
// NOTE ON HARDWARE COMPATIBILITY
// ──────────────────────────────
// The u-blox NEO-M10 (and the broader M10/HPG firmware platform) does NOT
// implement UBX-NAV-SVINFO (Class 0x01, ID 0x30).  That message was deprecated
// in u-blox generation 9 and is absent from HPG firmware entirely.  Polling
// 0x30 on a NEO-M10 produces no response — which is why the satellite list
// was always empty.
//
// The correct replacement is UBX-NAV-SAT (Class 0x01, ID 0x35), available on
// all u-blox M8/M9/M10 modules running current firmware.
//
// This function is named buildNavSvInfoPoll() to match its declaration in the
// header; the underlying UBX message ID has been updated to 0x35 accordingly.
//
// NAV-SAT response layout:
//   Header (8 bytes): iTOW(4), version(1), numSvs(1), reserved(2)
//   Per-SV blocks: numSvs × 12 bytes
//     [0]  gnssId  uint8   GNSS system (0=GPS,1=SBAS,2=GAL,3=BDS,5=QZSS,6=GLO)
//     [1]  svId    uint8   satellite vehicle identifier
//     [2]  cno     uint8   carrier-to-noise density, dBHz
//     [3]  elev    int8    elevation, degrees (-90 to +90)
//     [4]  azim    int16   azimuth, degrees (0-360), little-endian
//     [6]  prRes   int16   pseudorange residual x0.1 m, little-endian
//     [8]  flags   uint32  little-endian bitfield:
//                            bits[0:2] qualityInd  (0=no sig .. 7=carrier locked)
//                            bit[3]    svUsed      (1 = contributes to fix)
//                            bits[4:5] health      (0=unknown,1=OK,2=unhealthy)
//                            bit[6]    diffCorr
//                            bit[7]    smoothed
//                            bits[8:10] orbitSource
//                            ...
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildNavSvInfoPoll() {
    std::vector<uint8_t> frame = {
        0xB5, 0x62,
        0x01, 0x35,   // class=NAV, id=SAT  (NOT 0x30 SVINFO -- M10 doesn't support it)
        0x00, 0x00,   // zero payload = poll request
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// parseNavSvInfo() -- UBX-NAV-SAT response payload parser
//
// Parses the payload of a UBX-NAV-SAT (0x01/0x35) response.
//
// IMPORTANT DIFFERENCES FROM UBX-NAV-SVINFO (the old message this replaces):
//
//   Field         SVINFO offset    NAV-SAT offset    Notes
//   ──────────    ────────────     ──────────────    ──────────────────────────
//   SV count      [4] numCh        [5] numSvs         Different byte position!
//   gnssId        inferred         [0] direct         NAV-SAT is authoritative
//   svId          [1]              [1]                same
//   cno           [4]              [2]                different position
//   elev          [5]              [3]                different position
//   azim          [6..7] int16     [4..5] int16       same type, shifted
//   prRes         [8..11] int32    [6..7] int16       NAV-SAT is int16 x0.1m
//   flags byte    [2]              flags32 bits 0-7   flags is uint32 in NAV-SAT
//   quality byte  [3]              flags32 bits 0-2   embedded in flags
//   svUsed        flags bit 0      flags32 bit 3      bit position differs
//
// The function name is kept as parseNavSvInfo() to match the header declaration
// and avoid cascading renames in DroneLink.cpp and bindings.cpp.
//
// ubxPayload: raw UBX payload bytes starting after the 6-byte UBX header
//             and ending before the 2-byte checksum.
//             Minimum valid size = 8 bytes (header only, numSvs=0).
// =============================================================================
bool GPSNeoM10::parseNavSvInfo(const std::vector<uint8_t>& ubxPayload,
    std::vector<SVInfoEntry>& out)
{
    // NAV-SAT header: iTOW(4) + version(1) + numSvs(1) + reserved(2) = 8 bytes
    if (ubxPayload.size() < 8) return false;

    // numSvs is at byte [5] in NAV-SAT.
    // (In the old SVINFO, the count was at byte [4].  Reading [4] here would
    // give the version byte, always 0x01, so we'd only try to parse 1 SV.)
    const uint8_t numSvs = ubxPayload[5];

    if (ubxPayload.size() < static_cast<size_t>(8u + numSvs * 12u)) return false;

    out.clear();
    out.reserve(numSvs);

    // NAV-SAT provides gnssId directly in each SV block — no svId range
    // inference is needed.
    auto gnssIdToName = [](uint8_t gnssId) -> std::string {
        switch (gnssId) {
        case 0:  return "GPS";
        case 1:  return "SBAS";
        case 2:  return "Galileo";
        case 3:  return "BeiDou";
        case 5:  return "QZSS";
        case 6:  return "GLONASS";
        default: return "Unknown";
        }
        };

    for (uint8_t i = 0; i < numSvs; ++i) {
        const size_t base = 8u + static_cast<size_t>(i) * 12u;

        SVInfoEntry e;

        // ── NAV-SAT per-SV block layout ───────────────────────────────────
        e.gnssId = ubxPayload[base + 0];                             // uint8
        e.svid = ubxPayload[base + 1];                             // uint8
        e.cno = ubxPayload[base + 2];                             // uint8, dBHz
        e.elev = static_cast<int8_t>(ubxPayload[base + 3]);        // int8,  deg
        e.azim = read16(ubxPayload, static_cast<int>(base + 4));   // int16, deg
        // prRes: int16 at [6..7], units = 0.1 m  (int32 in old SVINFO)
        e.prRes = static_cast<int32_t>(
            read16(ubxPayload, static_cast<int>(base + 6)));

        // flags: uint32 at [8..11]
        const uint32_t flags32 =
            static_cast<uint32_t>(ubxPayload[base + 8]) |
            (static_cast<uint32_t>(ubxPayload[base + 9]) << 8) |
            (static_cast<uint32_t>(ubxPayload[base + 10]) << 16) |
            (static_cast<uint32_t>(ubxPayload[base + 11]) << 24);

        const uint8_t qualityInd = static_cast<uint8_t>(flags32 & 0x07u);  // bits 0-2
        const bool    svUsed = (flags32 & 0x08u) != 0u;                 // bit 3

        // Store raw flags lower byte for compatibility with SVInfoEntry.flags
        e.flags = static_cast<uint8_t>(flags32 & 0xFFu);
        e.quality = qualityInd;
        e.used = svUsed;
        // NAV-SAT has no tracking-channel field; use the SV index as a proxy
        e.chn = i;

        e.gnssName = gnssIdToName(e.gnssId);

        // qualityInd values (u-blox M10 HPG Integration Manual, s2.x):
        //   0 = no signal
        //   1 = searching
        //   2 = signal acquired
        //   3 = signal detected but unusable
        //   4 = code locked and time synchronized
        //   5..7 = code + carrier locked (full tracking)
        if (svUsed) {
            e.statusStr = "used";
        }
        else if (qualityInd >= 4) {
            e.statusStr = "tracked";
        }
        else if (qualityInd >= 2) {
            e.statusStr = "acquired";
        }
        else if (qualityInd == 1) {
            e.statusStr = "searching";
        }
        else {
            e.statusStr = "idle";
        }

        // Include only SVs with a valid satellite identifier.
        // svId == 0 means an unused receiver channel slot.
        if (e.svid != 0)
            out.push_back(e);
    }

    return true;
}

// =============================================================================
// parseNavStatus() -- MSP_NAV_STATUS (121)
// =============================================================================
bool GPSNeoM10::parseNavStatus(const std::vector<uint8_t>& buf, NavStatus& out) {
    if (buf.size() < 13) return false;
    if (buf[4] != 121)   return false;

    out.fixType = buf[5];
    out.gpsFlags = buf[6];
    out.fixOk = (out.gpsFlags & 0x01) != 0;
    out.dgpsUsed = (out.gpsFlags & 0x02) != 0;
    out.mapFlags = buf[8];
    out.hwStatus = buf[11];
    out.valid = true;
    return true;
}

// =============================================================================
// buildCfgGNSS() -- UBX-CFG-GNSS (Class 0x06, ID 0x3E)
//
// Enables / disables individual GNSS constellations on the NEO-M10.
//
// GPS is mandatory -- if GNSS_GPS is missing from constellationFlags it is
// forced on here. A receiver without GPS would not provide ICAO-compatible
// position data.
//
// NEO-M10 channel budget (firmware >= HPG 1.00):
//   15 total tracking channels. Only valid combos:
//   GPS+GAL, GPS+GLO, GPS+BDS -- not all four simultaneously.
//
// GNSS block layout (8 bytes per constellation):
//   gnssId    uint8   u-blox constellation ID
//   resTrkCh  uint8   reserved tracking channels (minimum)
//   maxTrkCh  uint8   max tracking channels when enabled
//   reserved  uint8   0x00
//   flags     uint32  LE; bit 0 = enable, bits[24:16] = sigCfgMask (L1=0x01)
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgGNSS(uint8_t constellationFlags) {
    // GPS is mandatory
    constellationFlags |= GPSConstellationFlags::GNSS_GPS;

    struct ConstellEntry {
        uint8_t flag;        // GPSConstellationFlags bit
        uint8_t gnssId;      // u-blox UBX gnssId (differs from flag for QZSS)
        uint8_t resTrkCh;
        uint8_t maxTrkCh;
        uint8_t sigCfgMask;  // bits[24:16] of flags word; L1 = 0x01
    };

    // NEO-M10 channel allocation per u-blox M10 Integration Manual s3.13.
    // QZSS: gnssId=5, not 4 -- the gap is intentional in the u-blox spec.
    const ConstellEntry table[] = {
        { GNSS_GPS,     0, 8,  15, 0x01 },   // GPS L1 C/A
        { GNSS_SBAS,    1, 1,   3, 0x01 },   // SBAS (WAAS/EGNOS)
        { GNSS_GALILEO, 2, 4,   8, 0x01 },   // Galileo E1
        { GNSS_BEIDOU,  3, 8,  15, 0x01 },   // BeiDou B1
        { GNSS_QZSS,    5, 0,   3, 0x01 },   // QZSS L1 C/A (Japan regional)
        { GNSS_GLONASS, 6, 8,  15, 0x01 },   // GLONASS L1 OF
    };

    // FIX: explicit cast to uint8_t avoids MSVC C4244 narrowing warning.
    const uint8_t  numBlocks = static_cast<uint8_t>(sizeof(table) / sizeof(table[0]));
    const uint8_t  msgVer = 0x00;
    const uint8_t  numTrkCh = 15;
    const uint16_t payloadLen = 4u + static_cast<uint16_t>(numBlocks * 8u);

    std::vector<uint8_t> frame;
    frame.reserve(6u + payloadLen + 2u);

    // UBX preamble + frame header
    frame.push_back(0xB5);
    frame.push_back(0x62);
    frame.push_back(0x06);   // class: CFG
    frame.push_back(0x3E);   // id:    CFG-GNSS
    frame.push_back(static_cast<uint8_t>(payloadLen & 0xFF));
    frame.push_back(static_cast<uint8_t>(payloadLen >> 8));

    // CFG-GNSS payload header (4 bytes)
    frame.push_back(msgVer);
    frame.push_back(0x00);       // numTrkChHw = 0 (auto)
    frame.push_back(numTrkCh);
    frame.push_back(numBlocks);

    // One 8-byte block per constellation
    for (const auto& e : table) {
        bool     enabled = (constellationFlags & e.flag) != 0;
        uint32_t flags32 = (enabled ? 0x00000001u : 0x00000000u)
            | (static_cast<uint32_t>(e.sigCfgMask) << 16);

        frame.push_back(e.gnssId);
        frame.push_back(e.resTrkCh);
        frame.push_back(enabled ? e.maxTrkCh : static_cast<uint8_t>(0));
        frame.push_back(0x00);   // reserved1
        frame.push_back(static_cast<uint8_t>(flags32 & 0xFF));
        frame.push_back(static_cast<uint8_t>((flags32 >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>((flags32 >> 16) & 0xFF));
        frame.push_back(static_cast<uint8_t>((flags32 >> 24) & 0xFF));
    }

    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgRate() -- UBX-CFG-RATE (Class 0x06, ID 0x08)
//
// Payload (6 bytes):
//   measRate  uint16 LE   measurement period in milliseconds
//   navRate   uint16 LE   number of measurements per nav solution (always 1)
//   timeRef   uint16 LE   0 = UTC, 1 = GPS time
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgRate(GPSUpdateRate rateHz,
    uint16_t timeRef) {
    uint16_t hz = static_cast<uint16_t>(rateHz);
    if (hz == 0) hz = 1;
    uint16_t measMs = static_cast<uint16_t>(1000u / hz);

    std::vector<uint8_t> frame = {
        0xB5, 0x62,
        0x06, 0x08,
        0x06, 0x00,
        static_cast<uint8_t>(measMs & 0xFF),
        static_cast<uint8_t>((measMs >> 8) & 0xFF),
        0x01, 0x00,   // navRate = 1
        static_cast<uint8_t>(timeRef & 0xFF),
        static_cast<uint8_t>((timeRef >> 8) & 0xFF),
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgPrt() -- UBX-CFG-PRT (Class 0x06, ID 0x00) -- UART1 port
//
// Payload (20 bytes):
//   portID       uint8   0x01 = UART1
//   reserved1    uint8   0x00
//   txReady      uint16  0x0000 (disabled)
//   mode         uint32  8N1, no parity (0x000008D0)
//   baudRate     uint32  115200 = 0x0001C200
//   inProtoMask  uint16  0x0001 = UBX in (always; needed for config commands)
//   outProtoMask uint16  0x0001 = UBX, 0x0002 = NMEA
//   flags        uint16  0x0000
//   reserved2    uint16  0x0000
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgPrt(GPSProtocol protocol) {
    uint16_t outProto = 0x0001u;   // default: UBX only
    if (protocol == GPSProtocol::NMEA)
        outProto = 0x0002u;        // NMEA only

    const uint32_t baud = 115200u;
    const uint32_t mode = 0x000008D0u;   // 8N1, no flow control
    const uint16_t inProto = 0x0001u;

    std::vector<uint8_t> frame = {
        0xB5, 0x62,
        0x06, 0x00,
        0x14, 0x00,   // length = 20
        0x01,         // portID = UART1
        0x00,
        0x00, 0x00,   // txReady disabled
        static_cast<uint8_t>(mode & 0xFF),
        static_cast<uint8_t>((mode >> 8) & 0xFF),
        static_cast<uint8_t>((mode >> 16) & 0xFF),
        static_cast<uint8_t>((mode >> 24) & 0xFF),
        static_cast<uint8_t>(baud & 0xFF),
        static_cast<uint8_t>((baud >> 8) & 0xFF),
        static_cast<uint8_t>((baud >> 16) & 0xFF),
        static_cast<uint8_t>((baud >> 24) & 0xFF),
        static_cast<uint8_t>(inProto & 0xFF),
        static_cast<uint8_t>((inProto >> 8) & 0xFF),
        static_cast<uint8_t>(outProto & 0xFF),
        static_cast<uint8_t>((outProto >> 8) & 0xFF),
        0x00, 0x00,   // flags
        0x00, 0x00,   // reserved2
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgNav5() -- UBX-CFG-NAV5 (Class 0x06, ID 0x24)
//
// CRITICAL: dynModel = 8 (Airborne <4g) is MANDATORY for UAV applications.
// This removes the speed cap and 50 000 m altitude cap that consumer models
// impose. Using model 0 (Portable) or 4 (Automotive) causes incorrect
// positions at speeds above ~27 m/s or altitudes above 9000 m.
//
// mask = 0x0005: apply dynModel (bit 0) + elevation mask (bit 2).
// Payload is 36 bytes; unused fields are zero-filled.
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgNav5(uint8_t elevMaskDeg) {
    const uint16_t mask = 0x0005u;
    const uint8_t  dynModel = 8u;    // Airborne <4g -- MANDATORY for UAV
    const uint8_t  fixMode = 3u;    // auto 2D/3D

    std::vector<uint8_t> frame = {
        0xB5, 0x62,
        0x06, 0x24,
        0x24, 0x00,   // length = 36
        static_cast<uint8_t>(mask & 0xFF),
        static_cast<uint8_t>((mask >> 8) & 0xFF),
        dynModel,
        fixMode,
        0x00, 0x00, 0x00, 0x00,   // fixedAlt     (unused; fixMode=3)
        0x00, 0x00, 0x00, 0x00,   // fixedAltVar  (unused)
        elevMaskDeg,
        0x00,                      // drLimit
        0x00, 0x00,                // pDop
        0x00, 0x00,                // tDop
        0x00, 0x00,                // pAcc
        0x00, 0x00,                // tAcc
        0x00,                      // staticHoldThresh
        0x00,                      // dgnssTimeout
        0x00,                      // cnoThreshNumSVs
        0x00,                      // cnoThresh (see GPSConfig::signalMaskDbHz note)
        0x00, 0x00,                // reserved1
        0x00, 0x00,                // staticHoldDist
        0x00,                      // utcStandard (auto)
        0x00, 0x00, 0x00,          // reserved2[3]
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgCfg() -- UBX-CFG-CFG (Class 0x06, ID 0x09)
//
// Saves current config to BBR and flash.
// Payload (13 bytes):
//   clearMask   uint32  0x00000000 -- do not erase
//   saveMask    uint32  0x0000061F -- save IO ports, messages, INF, nav, rxm
//   loadMask    uint32  0x00000000 -- do not load
//   deviceMask  uint8   0x07       -- BBR + Flash + EEPROM
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgCfg() {
    std::vector<uint8_t> frame = {
        0xB5, 0x62,
        0x06, 0x09,
        0x0D, 0x00,   // length = 13
        0x00, 0x00, 0x00, 0x00,   // clearMask = 0
        0x1F, 0x06, 0x00, 0x00,   // saveMask  = 0x0000061F (LE)
        0x00, 0x00, 0x00, 0x00,   // loadMask  = 0
        0x07,                      // deviceMask: BBR + Flash + EEPROM
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// parseAck() -- UBX-ACK-ACK (Class 0x05, ID 0x01)
//
// Frame layout (10 bytes):
//   [0..1]  0xB5 0x62  preamble
//   [2]     0x05       class ACK
//   [3]     0x01       id ACK-ACK (0x00 = ACK-NAK = rejected)
//   [4..5]  0x02 0x00  payload length = 2
//   [6]     clsID      class of the acknowledged message
//   [7]     msgID      id    of the acknowledged message
//   [8..9]  CK_A CK_B
// =============================================================================
bool GPSNeoM10::parseAck(const std::vector<uint8_t>& buf,
    uint8_t msgClass, uint8_t msgId) {
    if (buf.size() < 10)    return false;
    if (buf[0] != 0xB5)     return false;
    if (buf[1] != 0x62)     return false;
    if (buf[2] != 0x05)     return false;   // class ACK
    if (buf[3] != 0x01)     return false;   // id ACK-ACK (not NAK=0x00)
    if (buf[6] != msgClass) return false;
    if (buf[7] != msgId)    return false;
    return true;
}

// =============================================================================
// wrapUbxInMspPassthrough()
//
// MSP_PASSTHROUGH (ID 0xF5 = 245) frame layout:
//   '$' 'M' '<'  payloadLen  0xF5  <UBX frame bytes>  checksum
//
// checksum = XOR of payloadLen ^ 0xF5 ^ all UBX bytes.
//
// Prerequisite: BF Configurator -> Ports -> GPS UART -> Passthrough: ON.
// =============================================================================
std::vector<uint8_t> GPSNeoM10::wrapUbxInMspPassthrough(
    const std::vector<uint8_t>& ubxFrame)
{
    if (ubxFrame.size() > 255) return {};   // MSP payload length is 1 byte

    const auto    payloadLen = static_cast<uint8_t>(ubxFrame.size());
    const uint8_t MSP_PASSTHROUGH = 0xF5u;

    std::vector<uint8_t> frame;
    frame.reserve(6u + ubxFrame.size());
    frame.push_back('$');
    frame.push_back('M');
    frame.push_back('<');
    frame.push_back(payloadLen);
    frame.push_back(MSP_PASSTHROUGH);
    frame.insert(frame.end(), ubxFrame.begin(), ubxFrame.end());

    uint8_t csum = payloadLen ^ MSP_PASSTHROUGH;
    for (auto b : ubxFrame) csum ^= b;
    frame.push_back(csum);

    return frame;
}