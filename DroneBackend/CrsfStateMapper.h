#pragma once
// =============================================================================
// CrsfStateMapper — CRSF telemetry frames → DroneState
//
// Pure logic, no I/O and no threads: CrsfLink owns one of these and calls it
// from its reader thread under its own mutex. Kept separate so every mapping
// rule below is covered by tests/test_crsf.cpp without a radio attached.
//
// ── What CRSF gives us vs. what the widgets expect ──────────────────────────
//
//   DroneState field(s)             ELRS source            Notes
//   ─────────────────────────────── ────────────────────── ──────────────────
//   roll / pitch / yaw              ATTITUDE 0x1E          same units as MSP
//   magHeadingDeg                   ATTITUDE yaw           fused heading, not raw mag
//   batteryVoltage/Current/mAh/%    BATTERY 0x08           0.1 V / 0.1 A resolution
//   batteryCellCount / State        derived here           from voltage per cell
//   gps.lat/lon/alt/speed/course    GPS 0x02
//   gps.numSat                      GPS 0x02
//   gps.fixType / positionUsable    derived here           no fix type / HDOP in CRSF
//   gps.distToHomM / bearingToHome  derived here           home = position at arming
//   baroAltitudeCm                  BARO_ALT 0x09          or GPS alt rel. to home
//   baroVarioCmPerSec               VARIO 0x07
//   armed / flightModeFlags / Name  FLIGHT_MODE 0x21       text → same flags/names as MSP
//   rssi (→ rc_link_quality)        LINK_STATISTICS 0x14   uplink LQ scaled to 0–255
//   radio.*                         LINK_STATISTICS + here ELRS-only extras
//
//   NOT available: ax..gz, magX/Y/Z, motorValues, rcChannels, svList, hdop,
//   armingDisableFlags (only "blocked yes/no"), cpuLoad, i2c errors, cycle time.
// =============================================================================
#include "CrsfProtocol.h"
#include "DroneLink.h"     // DroneState, RadioLinkStats, FlightMode::, BatteryState
#include <cstdint>

class CrsfStateMapper {
public:
    struct Config {
        // Yaw wire encoding — see Crsf::decodeAttitude(). Verify on the bench
        // (IT-ELRS-004) by comparing against the USB yaw between 180° and 360°.
        bool    yawSigned = true;

        // CRSF carries no fix type or HDOP, so "usable position" is decided
        // from the satellite count alone.
        uint8_t minSatsForFix = 5;

        // 0 = auto-detect from the first voltage reading (ceil(V / 4.35)).
        // Auto-detect is wrong if the radio connects to an already-sagging
        // pack mid-flight, so set it explicitly for real missions (4 for 4S).
        uint8_t cellCount = 0;
        float   warningCellV = 3.5f;   // BF 4.5 default vbat_warning_cell_voltage
        float   criticalCellV = 3.3f;  // BF 4.5 default vbat_min_cell_voltage
        uint16_t capacityMah = 0;      // optional, only for display

        // If the FC sends no baro frame, drive the altimeter from GPS altitude
        // relative to the home point instead of leaving it at zero.
        bool    gpsAltitudeFallback = true;
        int     baroStaleMs = 2000;
    };

    struct StatusThresholds {
        int lostTimeoutMs = 1500;      // no drone telemetry for this long → LOST
        int degradedLq = 70;           // uplink LQ below this → DEGRADED
        int instrumentStaleMs = 1500;  // attitude older than this → DEGRADED
    };

    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    // Apply one frame. Returns true if it was a frame type we use.
    bool apply(const Crsf::Frame& f, DroneState& s, int64_t nowMs);

    // Recompute ages, rates, derived altitude, link status. Call ~10 Hz and
    // before handing a snapshot to Python.
    void refresh(DroneState& s, bool portOpen, int64_t nowMs,
                 const StatusThresholds& th);

    // Forget timing/rate history (e.g. a different radio was plugged in).
    // Deliberately keeps the last known position/attitude in DroneState — for
    // search & rescue the last position before a link loss is valuable.
    void resetTiming();

    static RadioLinkStatus evaluateStatus(bool portOpen, const RadioLinkStats& r,
                                          const StatusThresholds& th);
    static const char* statusName(RadioLinkStatus st);

    // Flight-mode text from Betaflight → flags/name in the same style as
    // DroneLink::decodeFlightMode(). Exposed for tests.
    static void mapFlightMode(const std::string& raw, DroneState& s);

    // Great-circle helpers (metres / degrees) used for home distance & bearing.
    static double distanceM(double lat1, double lon1, double lat2, double lon2);
    static double bearingDeg(double lat1, double lon1, double lat2, double lon2);

private:
    void updateHome(DroneState& s);
    void updateBatteryDerived(DroneState& s);

    Config cfg_;
    bool   prevArmed_ = false;
    uint8_t detectedCells_ = 0;
    bool   heartbeat_ = false;

    // Last-update timestamps (ms, -1 = never)
    int64_t tFrame_ = -1, tAtt_ = -1, tGps_ = -1, tBatt_ = -1,
            tMode_ = -1, tBaro_ = -1, tLink_ = -1;

    // Rate counters
    int64_t rateWindowStart_ = -1;
    uint32_t nAtt_ = 0, nGps_ = 0, nBatt_ = 0, nMode_ = 0,
             nBaro_ = 0, nVario_ = 0, nLink_ = 0, nAll_ = 0;

    RadioLinkStatus prevStatus_ = RadioLinkStatus::NO_RADIO;
};
