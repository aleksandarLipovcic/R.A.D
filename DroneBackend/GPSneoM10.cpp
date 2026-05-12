#include "GPSNeoM10.h"
#include <cstring>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers — little-endian reads
// ─────────────────────────────────────────────────────────────────────────────

int16_t GPSNeoM10::read16(const std::vector<uint8_t>& b, int i) {
    return static_cast<int16_t>(b[i] | (b[i + 1] << 8));
}

uint16_t GPSNeoM10::read16u(const std::vector<uint8_t>& b, int i) {
    return static_cast<uint16_t>(b[i] | (b[i + 1] << 8));
}

int32_t GPSNeoM10::read32(const std::vector<uint8_t>& b, int i) {
    return static_cast<int32_t>(
        b[i]
        | (static_cast<uint32_t>(b[i + 1]) << 8)
        | (static_cast<uint32_t>(b[i + 2]) << 16)
        | (static_cast<uint32_t>(b[i + 3]) << 24)
        );
}

// ─────────────────────────────────────────────────────────────────────────────
// addUbxChecksum()
//
// Appends CK_A and CK_B (Fletcher-8) to a UBX frame.
// Per u-blox Integration Manual §3.4, the checksum covers bytes from the
// Class field (index 2) to the last payload byte (inclusive).
// The preamble bytes 0xB5, 0x62 at [0] and [1] are excluded.
// ─────────────────────────────────────────────────────────────────────────────
void GPSNeoM10::addUbxChecksum(std::vector<uint8_t>& frame) {
    uint8_t ck_a = 0, ck_b = 0;
    // Checksum starts at byte 2 (class field), ends at last payload byte
    for (size_t i = 2; i < frame.size(); ++i) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    frame.push_back(ck_a);
    frame.push_back(ck_b);
}

// ─────────────────────────────────────────────────────────────────────────────
// parseRaw() — MSP_RAW_GPS (command ID 106 / 0x6A)
//
// Betaflight 4.x MSP_RAW_GPS payload (16 or 18 bytes):
//
//  Offset  Size  Type      Description
//  ──────  ────  ────────  ────────────────────────────────────────────────────
//  5       1     uint8     GPS fix type: 0 = none, 1 = 2D, 2 = 3D
//  6       1     uint8     Number of satellites used
//  7       4     int32     Latitude  × 10 000 000  (e.g. 452 345 678 = 45.2345678°)
//  11      4     int32     Longitude × 10 000 000
//  15      2     uint16    Altitude MSL, metres
//  17      2     uint16    Ground speed, cm/s
//  19      2     uint16    Ground course, decidegrees (0–3599)
//  21      2     uint16    HDOP × 100  (BF ≥ 4.1; size = 18 when present)
//
// Minimum valid frame: header(5) + 16 bytes payload + 1 checksum = 22 bytes.
// With HDOP:           header(5) + 18 bytes payload + 1 checksum = 24 bytes.
// ─────────────────────────────────────────────────────────────────────────────
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

    if (buf.size() >= 24)
        out.hdop = read16u(buf, 21);
    else
        out.hdop = 9999;   // unknown — older firmware or missing field

    out.rawValid = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// parseComp() — MSP_COMP_GPS (command ID 107 / 0x6B)
//
//  Offset  Size  Type      Description
//  ──────  ────  ────────  ────────────────────────────────────────────────────
//  5       2     uint16    Distance to home point, metres
//  7       2     int16     Bearing to home, degrees (−180 … +180)
//  9       1     uint8     GPS heartbeat — toggles on every fresh GPS frame
//
// Total frame: header(5) + 5 bytes payload + 1 checksum = 11 bytes.
// ─────────────────────────────────────────────────────────────────────────────
bool GPSNeoM10::parseComp(const std::vector<uint8_t>& buf, GPSReading& out) {
    if (buf.size() < 11) return false;
    if (buf[4] != 107)   return false;

    out.distToHomM = read16u(buf, 5);
    out.bearingToHome = read16(buf, 7);
    out.gpsHeartbeat = buf[9];
    out.compValid = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildCfgGNSS() — UBX-CFG-GNSS (Class 0x06, ID 0x3E)
//
// Enables / disables individual GNSS constellations on the NEO-M10.
//
// GNSS block layout (8 bytes per constellation):
//   gnssId    uint8   u-blox constellation ID (see GPSConstellationFlags)
//   resTrkCh  uint8   reserved tracking channels (minimum 8 per enabled GNSS)
//   maxTrkCh  uint8   max tracking channels (up to receiver total)
//   reserved  uint8   0x00
//   flags     uint32  LE; bit 0 = enable, bits[24:16] = sigCfgMask (L1=0x01)
//
// NEO-M10 UART channel budget:
//   The M10 has 15 tracking channels total in firmware ≥ HPG 1.00.
//   GPS: 8 min, Galileo: 4 min, GLONASS: 8 min, BeiDou: 8 min.
//   Only valid combos: GPS+GAL, GPS+GLO, GPS+BDS — not all 4 simultaneously.
//
// AVIATION NOTE:
//   GPS must always remain enabled. If constellationFlags lacks GNSS_GPS it is
//   forced on here — a receiver with only GLONASS/BeiDou would not provide
//   ICAO-compatible GPS position data.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GPSNeoM10::buildCfgGNSS(uint8_t constellationFlags) {
    // GPS is mandatory — enforce it
    constellationFlags |= GPSConstellationFlags::GNSS_GPS;

    // Map from GPSConstellationFlags bit to { gnssId, resTrkCh, maxTrkCh }
    struct ConstellEntry {
        uint8_t flag;
        uint8_t gnssId;
        uint8_t resTrkCh;
        uint8_t maxTrkCh;
        uint8_t sigCfgMask;   // bits[24:16] of flags word; L1=0x01 for all
    };

    // NEO-M10 channel allocation per u-blox M10 Integration Manual §3.13.
    // resTrkCh = minimum guaranteed channels, maxTrkCh = upper bound.
    // Total channels must not exceed 15.
    const ConstellEntry table[] = {
        { GNSS_GPS,     0, 8,  15, 0x01 },  // GPS L1 C/A
        { GNSS_SBAS,    1, 1,   3, 0x01 },  // SBAS (WAAS/EGNOS — augmentation)
        { GNSS_GALILEO, 2, 4,   8, 0x01 },  // Galileo E1
        { GNSS_BEIDOU,  3, 8,  15, 0x01 },  // BeiDou B1I / B1C
        { GNSS_QZSS,    5, 0,   3, 0x01 },  // QZSS L1 C/A (Japan regional)
        { GNSS_GLONASS, 6, 8,  15, 0x01 },  // GLONASS L1 OF
    };

    const int numBlocks = sizeof(table) / sizeof(table[0]);
    uint8_t   msgVer = 0x00;  // message version
    uint8_t   numTrkCh = 15;    // number of tracking channels (M10 total)

    // Header bytes for UBX frame (preamble + class + id + length placeholder)
    // Payload = 4 header bytes + numBlocks × 8 bytes
    const uint16_t payloadLen = 4 + static_cast<uint16_t>(numBlocks * 8);

    std::vector<uint8_t> frame;
    frame.reserve(6 + payloadLen + 2);

    // ── UBX preamble ─────────────────────────────────────────────────────────
    frame.push_back(0xB5);   // sync char 1
    frame.push_back(0x62);   // sync char 2
    frame.push_back(0x06);   // class: CFG
    frame.push_back(0x3E);   // id:    CFG-GNSS
    frame.push_back(static_cast<uint8_t>(payloadLen & 0xFF));
    frame.push_back(static_cast<uint8_t>(payloadLen >> 8));

    // ── CFG-GNSS header fields ────────────────────────────────────────────────
    frame.push_back(msgVer);       // msgVer
    frame.push_back(0x00);         // numTrkChHw  (hardware total — 0 = auto)
    frame.push_back(numTrkCh);     // numTrkChUse (channels we want to use)
    frame.push_back(numBlocks);    // numConfigBlocks

    // ── One block per constellation ───────────────────────────────────────────
    for (const auto& e : table) {
        bool enabled = (constellationFlags & e.flag) != 0;
        uint32_t flags32 = (enabled ? 0x00000001u : 0x00000000u)
            | (static_cast<uint32_t>(e.sigCfgMask) << 16);

        frame.push_back(e.gnssId);
        frame.push_back(e.resTrkCh);
        frame.push_back(enabled ? e.maxTrkCh : 0);
        frame.push_back(0x00);   // reserved1
        // flags32, little-endian
        frame.push_back(static_cast<uint8_t>(flags32 & 0xFF));
        frame.push_back(static_cast<uint8_t>((flags32 >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>((flags32 >> 16) & 0xFF));
        frame.push_back(static_cast<uint8_t>((flags32 >> 24) & 0xFF));
    }

    addUbxChecksum(frame);
    return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildCfgRate() — UBX-CFG-RATE (Class 0x06, ID 0x08)
//
// Sets the navigation solution rate.
//
// Payload (6 bytes):
//   measRate   uint16 LE   measurement period in milliseconds
//   navRate    uint16 LE   number of measurements per navigation solution (= 1)
//   timeRef    uint16 LE   0 = UTC, 1 = GPS time (aviation standard)
//
// AVIATION NOTE:
//   navRate is fixed at 1 (one nav solution per measurement) per u-blox
//   recommendation.  Setting navRate > 1 is only useful for power saving in
//   non-aviation applications and would degrade integrity monitoring.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GPSNeoM10::buildCfgRate(GPSUpdateRate rateHz,
    uint16_t timeRef) {
    uint16_t hz = static_cast<uint16_t>(rateHz);
    if (hz == 0) hz = 1;               // clamp to minimum 1 Hz
    uint16_t measMs = 1000u / hz;      // convert Hz → milliseconds

    std::vector<uint8_t> frame = {
        0xB5, 0x62,         // preamble
        0x06, 0x08,         // class CFG, id RATE
        0x06, 0x00,         // length = 6 bytes (LE)
        // payload ─────────────────────────────────────────────────────────────
        static_cast<uint8_t>(measMs & 0xFF),   // measRate low
        static_cast<uint8_t>((measMs >> 8) & 0xFF),   // measRate high
        0x01, 0x00,                                    // navRate = 1
        static_cast<uint8_t>(timeRef & 0xFF),  // timeRef low
        static_cast<uint8_t>((timeRef >> 8) & 0xFF),  // timeRef high
    };
    addUbxChecksum(frame);
    return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildCfgPrt() — UBX-CFG-PRT (Class 0x06, ID 0x00) — UART1 port
//
// Configures the output protocol on UART1 (the port connected to the F405).
// Input protocol is always UBX so the F405 can continue sending config frames.
// Baud rate is fixed at 115200 to match Betaflight's GPS serial port setting.
//
// Payload (20 bytes):
//   portID      uint8   0x01 = UART1
//   reserved1   uint8   0x00
//   txReady     uint16  0x0000 (txReady disabled)
//   mode        uint32  UART mode: 8N1, no parity (0x000008D0)
//   baudRate    uint32  115200 (0x0001C200)
//   inProtoMask uint16  0x0001 = UBX in (always; needed for config)
//   outProtoMask uint16 0x0001 = UBX, 0x0002 = NMEA, 0x0003 = both
//   flags       uint16  0x0000
//   reserved2   uint16  0x0000
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GPSNeoM10::buildCfgPrt(GPSProtocol protocol) {
    uint16_t outProto = 0x0001;   // UBX
    if (protocol == GPSProtocol::NMEA)
        outProto = 0x0002;        // NMEA only
    // MSP passthrough case: still use UBX on GPS UART; BF handles MSP layer
    // (GPSProtocol::MSP should not be used to configure the GPS itself)

    const uint32_t baud = 115200u;
    const uint32_t mode = 0x000008D0u;  // 8N1
    const uint16_t inProto = 0x0001;       // always accept UBX in

    std::vector<uint8_t> frame = {
        0xB5, 0x62,   // preamble
        0x06, 0x00,   // class CFG, id PRT
        0x14, 0x00,   // length = 20 bytes (LE)
        // payload ─────────────────────────────────────────────────────────────
        0x01,         // portID = UART1
        0x00,         // reserved1
        0x00, 0x00,   // txReady = disabled

        // mode: 8N1 (0x000008D0)
        static_cast<uint8_t>(mode & 0xFF),
        static_cast<uint8_t>((mode >> 8) & 0xFF),
        static_cast<uint8_t>((mode >> 16) & 0xFF),
        static_cast<uint8_t>((mode >> 24) & 0xFF),

        // baudRate: 115200 (0x0001C200)
        static_cast<uint8_t>(baud & 0xFF),
        static_cast<uint8_t>((baud >> 8) & 0xFF),
        static_cast<uint8_t>((baud >> 16) & 0xFF),
        static_cast<uint8_t>((baud >> 24) & 0xFF),

        // inProtoMask (always UBX)
        static_cast<uint8_t>(inProto & 0xFF),
        static_cast<uint8_t>((inProto >> 8) & 0xFF),

        // outProtoMask
        static_cast<uint8_t>(outProto & 0xFF),
        static_cast<uint8_t>((outProto >> 8) & 0xFF),

        0x00, 0x00,   // flags = 0
        0x00, 0x00,   // reserved2
    };
    addUbxChecksum(frame);
    return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildCfgNav5() — UBX-CFG-NAV5 (Class 0x06, ID 0x24)
//
// Sets the dynamic platform model, elevation mask, and signal minimum.
//
// CRITICAL AVIATION NOTE — dynModel = 8 (Airborne < 4g):
//   u-blox defines "Airborne < 4g" as the correct model for UAV / aircraft.
//   This model removes the ground-speed cap (~515 m/s limit retained vs
//   consumer models' ~100 km/h) and altitude cap (50 000 m vs ground models).
//   Using model 0 (Portable) or 4 (Automotive) in a fixed-wing UAV will
//   cause the receiver to output erroneous positions during climbs, turns,
//   and at speeds above ~27 m/s.  This is NOT optional.
//
// Payload (36 bytes, masked write — only mask bits set are applied):
//   mask        uint16  0x0001 = apply dynModel + fixMode (bit 0)
//                       0x0004 = apply elevation mask      (bit 2)
//                       We set 0x0005 to apply dynModel + elevation.
//   dynModel    uint8   8 = Airborne < 4g
//   fixMode     uint8   3 = auto 2D/3D
//   ...remaining 32 bytes are reserved / other fields (set to zero)
//
// elevMaskDeg: elevation mask in integer degrees (0–90).
// sigMinDbHz:  not exposed in CFG-NAV5; sent separately via CFG-ITFM or left
//              at default. The field is included in GPSConfig for future use
//              with CFG-GNSS signal quality blocks.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GPSNeoM10::buildCfgNav5(uint8_t elevMaskDeg,
    uint8_t /*sigMinDbHz*/) {
    const uint16_t mask = 0x0005;  // apply dynModel + elevation mask
    const uint8_t  dynModel = 8;       // Airborne < 4g — MANDATORY for UAV
    const uint8_t  fixMode = 3;       // auto 2D/3D

    std::vector<uint8_t> frame = {
        0xB5, 0x62,   // preamble
        0x06, 0x24,   // class CFG, id NAV5
        0x24, 0x00,   // length = 36 bytes (LE)
        // payload ─────────────────────────────────────────────────────────────
        static_cast<uint8_t>(mask & 0xFF),
        static_cast<uint8_t>((mask >> 8) & 0xFF),
        dynModel,
        fixMode,
        0x00, 0x00, 0x00, 0x00,   // fixedAlt (not used when fixMode=3)
        0x00, 0x00, 0x00, 0x00,   // fixedAltVar
        static_cast<uint8_t>(elevMaskDeg), // minElev
        0x00,                              // drLimit (dead-reckoning, unused)
        0x00, 0x00,                        // pDop
        0x00, 0x00,                        // tDop
        0x00, 0x00,                        // pAcc
        0x00, 0x00,                        // tAcc
        0x00,                              // staticHoldThresh
        0x00,                              // dgnssTimeout (DGNSS not used)
        0x00,                              // cnoThreshNumSVs
        0x00,                              // cnoThresh (use default)
        0x00, 0x00,                        // reserved1
        0x00, 0x00,                        // staticHoldDist
        0x00,                              // utcStandard (auto)
        0x00, 0x00, 0x00,                  // reserved2[3]
    };
    addUbxChecksum(frame);
    return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// buildCfgCfg() — UBX-CFG-CFG (Class 0x06, ID 0x09)
//
// Saves the current configuration to battery-backed RAM and flash (if present).
//
// Payload (13 bytes):
//   clearMask   uint32  0x00000000  — do not erase any config
//   saveMask    uint32  0x0000061F  — save: IO ports, messages, INF settings,
//                                    nav settings, receiver manager, RINV
//   loadMask    uint32  0x00000000  — do not load from storage
//   deviceMask  uint8   0x07        — target: BBR + Flash + EEPROM
//
// If the receiver does not have flash, only BBR is updated (survives power
// cycle as long as the backup battery is charged).
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GPSNeoM10::buildCfgCfg() {
    std::vector<uint8_t> frame = {
        0xB5, 0x62,   // preamble
        0x06, 0x09,   // class CFG, id CFG
        0x0D, 0x00,   // length = 13 bytes (LE)
        // payload ─────────────────────────────────────────────────────────────
        0x00, 0x00, 0x00, 0x00,   // clearMask = 0
        0x1F, 0x06, 0x00, 0x00,   // saveMask  = 0x0000061F (LE)
        0x00, 0x00, 0x00, 0x00,   // loadMask  = 0
        0x07,                      // deviceMask: BBR + Flash + EEPROM
    };
    addUbxChecksum(frame);
    return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// parseAck() — UBX-ACK-ACK (Class 0x05, ID 0x01)
//
// Verifies the receiver acknowledged a specific config message.
//
// UBX-ACK-ACK frame (10 bytes):
//   0xB5 0x62 — preamble
//   0x05      — class ACK
//   0x01      — id   ACK-ACK  (0x00 = ACK-NAK = rejected)
//   0x02 0x00 — length = 2 bytes
//   clsID     — class  of the acknowledged message
//   msgID     — id     of the acknowledged message
//   CK_A CK_B — checksum
// ─────────────────────────────────────────────────────────────────────────────
bool GPSNeoM10::parseAck(const std::vector<uint8_t>& buf,
    uint8_t msgClass, uint8_t msgId) {
    if (buf.size() < 10)     return false;
    if (buf[0] != 0xB5)      return false;
    if (buf[1] != 0x62)      return false;
    if (buf[2] != 0x05)      return false;   // class ACK
    if (buf[3] != 0x01)      return false;   // id ACK-ACK (not NAK)
    if (buf[6] != msgClass)  return false;
    if (buf[7] != msgId)     return false;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// wrapUbxInMspPassthrough()
//
// Betaflight MSP_PASSTHROUGH (ID 245 / 0xF5) frame format:
//   '$' 'M' '<'  payloadLen  0xF5  <UBX frame bytes>  checksum
//
// This allows the GCS to send UBX config frames to the NEO-M10 through the
// F405 without requiring a dedicated serial connection to the GPS module.
//
// Prerequisite: Betaflight GPS passthrough must be enabled in the Ports tab.
//   Configurator → Ports → UART<n> (GPS port) → Passthrough → ON
//
// The response (UBX-ACK-ACK) comes back via the same MSP channel wrapped
// in an MSP response frame; DroneLink::sendMSPRaw() strips the MSP wrapper
// and returns the raw UBX bytes for parseAck() to validate.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> GPSNeoM10::wrapUbxInMspPassthrough(
    const std::vector<uint8_t>& ubxFrame) {
    if (ubxFrame.size() > 255) return {};   // MSP payload field is 1 byte

    auto payloadLen = static_cast<uint8_t>(ubxFrame.size());
    const uint8_t MSP_PASSTHROUGH = 0xF5;

    // Build the MSP frame header
    std::vector<uint8_t> frame;
    frame.reserve(6 + ubxFrame.size());
    frame.push_back('$');
    frame.push_back('M');
    frame.push_back('<');
    frame.push_back(payloadLen);
    frame.push_back(MSP_PASSTHROUGH);

    // Append UBX payload
    frame.insert(frame.end(), ubxFrame.begin(), ubxFrame.end());

    // MSP checksum: XOR of payloadLen ^ cmd ^ all payload bytes
    uint8_t csum = payloadLen ^ MSP_PASSTHROUGH;
    for (auto b : ubxFrame) csum ^= b;
    frame.push_back(csum);

    return frame;
}