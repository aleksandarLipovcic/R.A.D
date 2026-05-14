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
// gnssIdToName()  [private static]
//
// Shared by parseMspSvInfo() and parseNavSvInfo() so the mapping is defined
// exactly once. u-blox gnssId values match UBX-NAV-SAT and the GNSS nibble
// packed into the chn byte of MSP_GPS_SV_INFO (cmd 164) for M10 modules.
// =============================================================================
std::string GPSNeoM10::gnssIdToName(uint8_t gnssId) {
    switch (gnssId) {
    case 0:  return "GPS";
    case 1:  return "SBAS";
    case 2:  return "Galileo";
    case 3:  return "BeiDou";
    case 5:  return "QZSS";
    case 6:  return "GLONASS";
    default: return "Unknown";
    }
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
// parseMspSvInfo() -- MSP_GPS_SV_INFO (command ID 164 / 0xA4)
//
// PRIMARY satellite data source for this project. Polled directly in the
// main communicationLoop() -- no passthrough, no port cycle, always available.
//
// Confirmed working by sv_baud_probe.py:
//   cmd 164 (MSP_GPS_SV_INFO) -> 129 bytes  [20 00 02 1F ...]
//
// MSP frame layout (full frame including header):
//   [0..2]  '$' 'M' '>'    MSP response preamble
//   [3]     payloadLen      number of payload bytes  (0x81 = 129 for M10)
//   [4]     0xA4            command ID 164
//   [5]     numCh           number of channels (0x20 = 32 for M10)
//   [6..]   numCh × 4 bytes per channel (see below)
//   [last]  checksum
//
// Per-channel block (4 bytes):
//   [0] chn     channel byte
//               When numCh > 16 (M10 reports 32):
//                 upper nibble = GNSS id  (0=GPS,1=SBAS,2=Galileo,3=BDS,5=QZSS,6=GLO)
//                 lower nibble = channel index within that constellation
//               When numCh ≤ 16 (legacy 16-ch modules):
//                 full byte = channel index, gnssId inferred from svid range
//   [1] svid    satellite vehicle ID
//   [2] quality signal quality 0-7 (same encoding as UBX qualityInd):
//                 0=no signal, 1=searching, 2=acquired, 3=unusable,
//                 4=code locked, 5/6/7=code+carrier locked (fully locked)
//   [3] cno     carrier-to-noise density, dBHz (0-55)
//
// Fields NOT available in cmd 164 (set to 0 in SVInfoEntry):
//   elev, azim, prRes
//
// The Python GPSWidget should hide the Elev/Azim columns when
// sv_source == "MSP" (see DroneLink.h sv_source field comment).
//
// Channels where svid==0 AND cno==0 are unused receiver slots -- skipped.
// =============================================================================
bool GPSNeoM10::parseMspSvInfo(const std::vector<uint8_t>& buf,
    std::vector<SVInfoEntry>& out)
{
    // Minimum frame: preamble(3) + len(1) + cmd(1) + numCh(1) + checksum(1) = 7
    if (buf.size() < 7)  return false;
    if (buf[4] != 164)   return false;

    const uint8_t numCh = buf[5];
    if (numCh == 0)      return false;

    // Verify frame is large enough for all channels + checksum
    const size_t expectedMin = 6u + static_cast<size_t>(numCh) * 4u + 1u;
    if (buf.size() < expectedMin) return false;

    // M10 reports 32 channels: GNSS id is in the upper nibble of the chn byte.
    // Legacy 16-ch modules pack the full channel index into the chn byte.
    const bool gnssInHighNibble = (numCh > 16);

    out.clear();
    out.reserve(numCh);

    for (uint8_t i = 0; i < numCh; ++i) {
        const size_t base = 6u + static_cast<size_t>(i) * 4u;

        const uint8_t chnByte = buf[base + 0];
        const uint8_t svid = buf[base + 1];
        const uint8_t quality = buf[base + 2];
        const uint8_t cno = buf[base + 3];

        // Skip unused receiver channel slots
        if (svid == 0 && cno == 0) continue;

        SVInfoEntry e;

        if (gnssInHighNibble) {
            e.gnssId = (chnByte >> 4) & 0x0Fu;
            e.chn = chnByte & 0x0Fu;
        }
        else {
            // Legacy path: infer GNSS from svid range (GPS only module)
            e.gnssId = 0;
            e.chn = chnByte;
        }

        e.svid = svid;
        e.quality = quality;
        e.cno = cno;

        // elev, azim, prRes not available via MSP cmd 164 -- remain 0
        e.elev = 0;
        e.azim = 0;
        e.prRes = 0;

        // Reconstruct flags byte to match the SVInfoEntry convention used by
        // the UBX path: bits[0:2] = qualityInd, bit[3] = svUsed.
        // This lets Python code treat both sources identically.
        e.used = (quality >= 4);
        e.flags = static_cast<uint8_t>(
            (quality & 0x07u) | (e.used ? 0x08u : 0x00u));

        e.gnssName = gnssIdToName(e.gnssId);

        // Status string matches BF Configurator terminology
        if (e.used) {
            e.statusStr = "used";
        }
        else if (quality >= 4) {
            e.statusStr = "tracked";    // code locked but not chosen for fix
        }
        else if (quality >= 2) {
            e.statusStr = "acquired";
        }
        else if (quality == 1) {
            e.statusStr = "searching";
        }
        else {
            e.statusStr = "idle";
        }

        out.push_back(e);
    }

    return !out.empty();
}

// =============================================================================
// buildNavSvInfoPoll() -- UBX-NAV-SAT (Class 0x01, ID 0x35)
//
// NOTE ON HARDWARE COMPATIBILITY
// ──────────────────────────────
// The u-blox NEO-M10 (and the broader M10/HPG firmware platform) does NOT
// implement UBX-NAV-SVINFO (Class 0x01, ID 0x30).  That message was deprecated
// in u-blox generation 9 and is absent from HPG firmware entirely.  Polling
// 0x30 on a NEO-M10 produces no response -- which is why the satellite list
// was always empty.
//
// The correct replacement is UBX-NAV-SAT (Class 0x01, ID 0x35), available on
// all u-blox M8/M9/M10 modules running current firmware.
//
// This function is named buildNavSvInfoPoll() to match its declaration in the
// header; the underlying UBX message ID has been updated to 0x35 accordingly.
//
// NOTE ON REACHABILITY
// ────────────────────
// With gps_auto_config=ON (BF 4.5.x default) the GPS UART is locked by BF.
// sv_baud_probe.py confirmed 0 bytes returned on all 6 UART indices.
// This poll frame is therefore only useful when applyGPSConfig() is called
// and the caller knows passthrough will work (gps_auto_config=OFF).
// For satellite polling use MSP cmd 164 (parseMspSvInfo) instead.
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
// FALLBACK PATH -- only reachable when gps_auto_config=OFF.
// On this project (BF 4.5.x, gps_auto_config=ON) this function is never
// reached during normal operation. It is kept for:
//   - applyGPSConfig() verification
//   - Future hardware where passthrough is available
//
// ubxPayload: raw UBX payload bytes starting after the 6-byte UBX header
//             and ending before the 2-byte checksum.
//             Minimum valid size = 8 bytes (header only, numSvs=0).
//
// NAV-SAT header: iTOW(4) + version(1) + numSvs(1) + reserved(2) = 8 bytes
// Per-SV block (12 bytes):
//   [0]  gnssId  uint8
//   [1]  svId    uint8
//   [2]  cno     uint8
//   [3]  elev    int8
//   [4]  azim    int16 LE
//   [6]  prRes   int16 LE  (0.1 m units)
//   [8]  flags   uint32 LE (bits[0:2]=qualityInd, bit[3]=svUsed)
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

    for (uint8_t i = 0; i < numSvs; ++i) {
        const size_t base = 8u + static_cast<size_t>(i) * 12u;

        SVInfoEntry e;

        e.gnssId = ubxPayload[base + 0];
        e.svid = ubxPayload[base + 1];
        e.cno = ubxPayload[base + 2];
        e.elev = static_cast<int8_t>(ubxPayload[base + 3]);
        e.azim = read16(ubxPayload, static_cast<int>(base + 4));
        e.prRes = static_cast<int32_t>(
            read16(ubxPayload, static_cast<int>(base + 6)));

        const uint32_t flags32 =
            static_cast<uint32_t>(ubxPayload[base + 8]) |
            (static_cast<uint32_t>(ubxPayload[base + 9]) << 8) |
            (static_cast<uint32_t>(ubxPayload[base + 10]) << 16) |
            (static_cast<uint32_t>(ubxPayload[base + 11]) << 24);

        const uint8_t qualityInd = static_cast<uint8_t>(flags32 & 0x07u);
        const bool    svUsed = (flags32 & 0x08u) != 0u;

        e.flags = static_cast<uint8_t>(flags32 & 0xFFu);
        e.quality = qualityInd;
        e.used = svUsed;
        e.chn = i;

        e.gnssName = gnssIdToName(e.gnssId);

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
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgGNSS(uint8_t constellationFlags) {
    constellationFlags |= GPSConstellationFlags::GNSS_GPS;

    struct ConstellEntry {
        uint8_t flag;
        uint8_t gnssId;
        uint8_t resTrkCh;
        uint8_t maxTrkCh;
        uint8_t sigCfgMask;
    };

    const ConstellEntry table[] = {
        { GNSS_GPS,     0, 8,  15, 0x01 },
        { GNSS_SBAS,    1, 1,   3, 0x01 },
        { GNSS_GALILEO, 2, 4,   8, 0x01 },
        { GNSS_BEIDOU,  3, 8,  15, 0x01 },
        { GNSS_QZSS,    5, 0,   3, 0x01 },
        { GNSS_GLONASS, 6, 8,  15, 0x01 },
    };

    const uint8_t  numBlocks = static_cast<uint8_t>(sizeof(table) / sizeof(table[0]));
    const uint8_t  msgVer = 0x00;
    const uint8_t  numTrkCh = 15;
    const uint16_t payloadLen = 4u + static_cast<uint16_t>(numBlocks * 8u);

    std::vector<uint8_t> frame;
    frame.reserve(6u + payloadLen + 2u);

    frame.push_back(0xB5); frame.push_back(0x62);
    frame.push_back(0x06); frame.push_back(0x3E);
    frame.push_back(static_cast<uint8_t>(payloadLen & 0xFF));
    frame.push_back(static_cast<uint8_t>(payloadLen >> 8));
    frame.push_back(msgVer);
    frame.push_back(0x00);
    frame.push_back(numTrkCh);
    frame.push_back(numBlocks);

    for (const auto& e : table) {
        bool     enabled = (constellationFlags & e.flag) != 0;
        uint32_t flags32 = (enabled ? 0x00000001u : 0x00000000u)
            | (static_cast<uint32_t>(e.sigCfgMask) << 16);

        frame.push_back(e.gnssId);
        frame.push_back(e.resTrkCh);
        frame.push_back(enabled ? e.maxTrkCh : static_cast<uint8_t>(0));
        frame.push_back(0x00);
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
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgRate(GPSUpdateRate rateHz,
    uint16_t timeRef) {
    uint16_t hz = static_cast<uint16_t>(rateHz);
    if (hz == 0) hz = 1;
    uint16_t measMs = static_cast<uint16_t>(1000u / hz);

    std::vector<uint8_t> frame = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        static_cast<uint8_t>(measMs & 0xFF),
        static_cast<uint8_t>((measMs >> 8) & 0xFF),
        0x01, 0x00,
        static_cast<uint8_t>(timeRef & 0xFF),
        static_cast<uint8_t>((timeRef >> 8) & 0xFF),
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgPrt() -- UBX-CFG-PRT (Class 0x06, ID 0x00)
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgPrt(GPSProtocol protocol) {
    uint16_t outProto = 0x0001u;
    if (protocol == GPSProtocol::NMEA)
        outProto = 0x0002u;

    const uint32_t baud = 115200u;
    const uint32_t mode = 0x000008D0u;
    const uint16_t inProto = 0x0001u;

    std::vector<uint8_t> frame = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x01, 0x00, 0x00, 0x00,
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
        0x00, 0x00, 0x00, 0x00,
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgNav5() -- UBX-CFG-NAV5 (Class 0x06, ID 0x24)
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgNav5(uint8_t elevMaskDeg) {
    const uint16_t mask = 0x0005u;
    const uint8_t  dynModel = 8u;
    const uint8_t  fixMode = 3u;

    std::vector<uint8_t> frame = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
        static_cast<uint8_t>(mask & 0xFF),
        static_cast<uint8_t>((mask >> 8) & 0xFF),
        dynModel, fixMode,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        elevMaskDeg, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// buildCfgCfg() -- UBX-CFG-CFG (Class 0x06, ID 0x09)
// =============================================================================
std::vector<uint8_t> GPSNeoM10::buildCfgCfg() {
    std::vector<uint8_t> frame = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x1F, 0x06, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x07,
    };
    addUbxChecksum(frame);
    return frame;
}

// =============================================================================
// parseAck() -- UBX-ACK-ACK (Class 0x05, ID 0x01)
// =============================================================================
bool GPSNeoM10::parseAck(const std::vector<uint8_t>& buf,
    uint8_t msgClass, uint8_t msgId) {
    if (buf.size() < 10)    return false;
    if (buf[0] != 0xB5)     return false;
    if (buf[1] != 0x62)     return false;
    if (buf[2] != 0x05)     return false;
    if (buf[3] != 0x01)     return false;
    if (buf[6] != msgClass) return false;
    if (buf[7] != msgId)    return false;
    return true;
}

// =============================================================================
// wrapUbxInMspPassthrough()
// =============================================================================
std::vector<uint8_t> GPSNeoM10::wrapUbxInMspPassthrough(
    const std::vector<uint8_t>& ubxFrame)
{
    if (ubxFrame.size() > 255) return {};

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