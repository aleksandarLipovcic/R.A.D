#include "CrsfStateMapper.h"
#include <algorithm>
#include <cmath>

namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG = PI / 180.0;
    constexpr double EARTH_R = 6371000.0;

    int64_t age(int64_t t, int64_t now) { return t < 0 ? -1 : now - t; }
    bool fresher(int64_t ageMs, int limit) { return ageMs >= 0 && ageMs <= limit; }
}

// =============================================================================
// Geometry
// =============================================================================
double CrsfStateMapper::distanceM(double lat1, double lon1, double lat2, double lon2) {
    const double dLat = (lat2 - lat1) * DEG, dLon = (lon2 - lon1) * DEG;
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
        std::cos(lat1 * DEG) * std::cos(lat2 * DEG) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2.0 * EARTH_R * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// Initial bearing from point 1 to point 2, 0 = north, clockwise, [0, 360).
double CrsfStateMapper::bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    const double y = std::sin((lon2 - lon1) * DEG) * std::cos(lat2 * DEG);
    const double x = std::cos(lat1 * DEG) * std::sin(lat2 * DEG) -
        std::sin(lat1 * DEG) * std::cos(lat2 * DEG) * std::cos((lon2 - lon1) * DEG);
    double b = std::atan2(y, x) / DEG;
    return b < 0 ? b + 360.0 : b;
}

// =============================================================================
// Flight mode text → flags / name
//
// Betaflight 4.5 crsfFrameFlightMode() sends one of:
//   "!FS!"  failsafe          "RTH"  GPS rescue      "MANU" passthrough
//   "STAB"  angle             "HOR"  horizon         "AIR"  acro + airmode
//   "ACRO"  acro              "WAIT" disarmed, waiting for GPS fix/home
//   "!ERR"  disarmed and arming is blocked (reasons only via MSP_STATUS_EX)
// with a trailing '*' whenever the craft is DISARMED.
// =============================================================================
void CrsfStateMapper::mapFlightMode(const std::string& rawIn, DroneState& s) {
    std::string raw = rawIn;
    const bool disarmed = !raw.empty() && raw.back() == '*';
    if (disarmed) raw.pop_back();

    uint32_t flags = disarmed ? 0u : FlightMode::ARM;
    std::string name;
    bool emergency = false;

    s.radio.armingBlocked = false;
    s.radio.gpsWaiting = false;

    if (raw == "!FS!") { flags |= FlightMode::FAILSAFE;   name = "FAILSAFE";   emergency = true; }
    else if (raw == "RTH") { flags |= FlightMode::GPS_RESCUE; name = "GPS RESCUE"; emergency = true; }
    else if (raw == "STAB") { flags |= FlightMode::ANGLE;      name = "ANGLE"; }
    else if (raw == "HOR") { flags |= FlightMode::HORIZON;    name = "HORIZON"; }
    else if (raw == "ACRO" || raw == "AIR") { name = "ACRO"; }
    else if (raw == "MANU") { name = "PASSTHRU"; }
    else if (raw == "!ERR") { name = "ARMING BLOCKED"; s.radio.armingBlocked = true; }
    else if (raw == "WAIT") { name = "WAIT GPS";       s.radio.gpsWaiting = true; }
    else { name = raw.empty() ? "UNKNOWN" : raw; }

    (void)emergency;
    if (disarmed) name += " [DISARMED]";

    s.flightModeFlags = flags;
    s.flightModeName = name;
    s.armed = !disarmed;
    s.radio.rawFlightMode = rawIn;

    // CRSF has no arming-disable bitmask. Keep the flags at 0 (unknown) and
    // put an explicit hint in the text so the arming widget can show it.
    s.armingDisableFlags = 0;
    s.armingDisableStr = s.radio.armingBlocked
        ? "ARMING BLOCKED (reasons only via USB)" : "";
}

// =============================================================================
// apply()
// =============================================================================
bool CrsfStateMapper::apply(const Crsf::Frame& f, DroneState& s, int64_t now) {
    s.linkSource = "ELRS";
    tFrame_ = now;
    ++nAll_;

    switch (f.type) {
    case Crsf::Type::ATTITUDE: {
        Crsf::Attitude a;
        if (!Crsf::decodeAttitude(f, a, cfg_.yawSigned)) return false;
        // DroneState keeps MSP units: roll/pitch in decidegrees, yaw in whole degrees.
        s.roll = static_cast<int16_t>(std::lround(a.rollDeg * 10.0f));
        s.pitch = static_cast<int16_t>(std::lround(a.pitchDeg * 10.0f));
        s.yaw = static_cast<int16_t>(std::lround(a.yawDeg) % 360);
        s.magHeadingDeg = a.yawDeg;   // fused heading — compass widget stays alive
        s.magValid = true;            // heading valid; raw magX/Y/Z stay 0
        s.sensorStatus |= SensorStatus::ACC | SensorStatus::GYRO;
        tAtt_ = now; ++nAtt_;
        return true;
    }
    case Crsf::Type::BATTERY: {
        Crsf::Battery b;
        if (!Crsf::decodeBattery(f, b)) return false;
        s.batteryVoltage = b.voltageV;
        s.batteryCurrent = b.currentA;
        s.batteryMahDrawn = static_cast<uint16_t>((std::min<uint32_t>)(b.mahDrawn, 0xFFFF));
        s.batteryPercentage = (std::min<uint8_t>)(b.remainingPct, 100);
        updateBatteryDerived(s);
        tBatt_ = now; ++nBatt_;
        return true;
    }
    case Crsf::Type::GPS: {
        Crsf::Gps g;
        if (!Crsf::decodeGps(f, g)) return false;
        GPSReading& r = s.gps;
        r.latitude = g.latitude;
        r.longitude = g.longitude;
        r.altitudeM = static_cast<float>(g.altitudeM);
        r.groundSpeedMs = static_cast<uint16_t>(std::lround(g.groundSpeedKmh / 3.6f * 100.0f)); // cm/s
        r.groundCourse = static_cast<uint16_t>(std::lround(g.headingDeg * 10.0f));             // decideg
        r.numSat = g.satellites;
        const bool hasPos = (g.latitude != 0.0 || g.longitude != 0.0);
        r.fixType = (hasPos && g.satellites >= 4) ? 2 : 0;
        r.hdop = 9999;                                   // not in CRSF
        r.rawValid = true;
        r.positionUsable = r.fixType >= 2 && g.satellites >= cfg_.minSatsForFix;
        heartbeat_ = !heartbeat_;
        r.gpsHeartbeat = heartbeat_ ? 1 : 0;
        s.navStatus.fixType = r.fixType;
        s.navStatus.fixOk = r.positionUsable;
        s.navStatus.valid = true;
        s.sensorStatus |= SensorStatus::GPS;
        updateHome(s);
        tGps_ = now; ++nGps_;
        return true;
    }
    case Crsf::Type::BARO_ALTITUDE: {
        Crsf::BaroAltitude b;
        if (!Crsf::decodeBaroAltitude(f, b)) return false;
        s.baroAltitudeCm = b.altitudeCm;
        s.baroValid = true;
        s.radio.altitudeSource = "BARO";
        s.sensorStatus |= SensorStatus::BARO;
        tBaro_ = now; ++nBaro_;
        return true;
    }
    case Crsf::Type::VARIO: {
        Crsf::Vario v;
        if (!Crsf::decodeVario(f, v)) return false;
        s.baroVarioCmPerSec = v.verticalSpeedCmS;
        s.sensorStatus |= SensorStatus::BARO;
        ++nVario_;
        return true;
    }
    case Crsf::Type::FLIGHT_MODE: {
        std::string mode;
        if (!Crsf::decodeFlightMode(f, mode)) return false;
        mapFlightMode(mode, s);
        updateHome(s);
        tMode_ = now; ++nMode_;
        return true;
    }
    case Crsf::Type::LINK_STATISTICS: {
        Crsf::LinkStats l;
        if (!Crsf::decodeLinkStats(f, l)) return false;
        RadioLinkStats& r = s.radio;
        r.linkStatsValid = true;
        r.uplinkRssi1Dbm = l.uplinkRssi1Dbm;
        r.uplinkRssi2Dbm = l.uplinkRssi2Dbm;
        r.uplinkLq = l.uplinkLq;
        r.uplinkSnr = l.uplinkSnr;
        r.activeAntenna = l.activeAntenna;
        r.rfModeIndex = l.rfModeIndex;
        r.txPowerMw = l.txPowerMw;
        r.downlinkRssiDbm = l.downlinkRssiDbm;
        r.downlinkLq = l.downlinkLq;
        r.downlinkSnr = l.downlinkSnr;
        // Reuse the existing 0–255 rssi field so rc_link_quality in to_dict()
        // shows the real ELRS LQ % without any widget change.
        s.rssi = static_cast<uint8_t>((std::min<int>)(l.uplinkLq, 100) * 255 / 100);
        tLink_ = now; ++nLink_;
        return true;
    }
    default:
        return false;   // heartbeat, device info, RC channels, ELRS params …
    }
}

// =============================================================================
// Home point — Betaflight sets home at the moment of arming; mirror that.
// If the craft is armed before a usable fix exists, take the first usable
// fix afterwards (BF does the same when GPS comes in late).
// =============================================================================
void CrsfStateMapper::updateHome(DroneState& s) {
    RadioLinkStats& r = s.radio;
    const bool armEdge = s.armed && !prevArmed_;
    prevArmed_ = s.armed;

    if (armEdge) r.homeSet = false;   // new flight → new home
    if (s.armed && !r.homeSet && s.gps.positionUsable) {
        r.homeSet = true;
        r.homeLat = s.gps.latitude;
        r.homeLon = s.gps.longitude;
        r.homeAltM = s.gps.altitudeM;
    }

    if (r.homeSet && s.gps.rawValid) {
        const double d = distanceM(s.gps.latitude, s.gps.longitude, r.homeLat, r.homeLon);
        double b = bearingDeg(s.gps.latitude, s.gps.longitude, r.homeLat, r.homeLon);
        if (b > 180.0) b -= 360.0;    // MSP_COMP_GPS convention: -180..+180
        s.gps.distToHomM = static_cast<uint16_t>((std::min)(d, 65535.0));
        s.gps.bearingToHome = static_cast<int16_t>(std::lround(b));
        s.gps.compValid = true;
    }
}

// =============================================================================
// Battery — cell count and OK/WARNING/CRITICAL derived from pack voltage.
// =============================================================================
void CrsfStateMapper::updateBatteryDerived(DroneState& s) {
    const float v = s.batteryVoltage;
    if (v < 1.0f) {
        s.batteryState = BatteryState::NOT_PRESENT;
        return;
    }
    uint8_t cells = cfg_.cellCount;
    if (cells == 0) {
        if (detectedCells_ == 0)
            detectedCells_ = static_cast<uint8_t>(std::ceil(v / 4.35f));
        cells = detectedCells_;
    }
    s.batteryCellCount = cells;
    if (cfg_.capacityMah) s.batteryCapacityMah = cfg_.capacityMah;

    const float perCell = cells ? v / cells : v;
    if (perCell <= cfg_.criticalCellV)      s.batteryState = BatteryState::CRITICAL;
    else if (perCell <= cfg_.warningCellV)  s.batteryState = BatteryState::WARNING;
    else                                    s.batteryState = BatteryState::OK;
}

// =============================================================================
// Link status
// =============================================================================
const char* CrsfStateMapper::statusName(RadioLinkStatus st) {
    switch (st) {
    case RadioLinkStatus::NO_RADIO:       return "NO_RADIO";
    case RadioLinkStatus::WAITING:        return "WAITING";
    case RadioLinkStatus::TELEMETRY_OK:   return "TELEMETRY_OK";
    case RadioLinkStatus::DEGRADED:       return "DEGRADED";
    case RadioLinkStatus::TELEMETRY_LOST: return "TELEMETRY_LOST";
    }
    return "UNKNOWN";
}

RadioLinkStatus CrsfStateMapper::evaluateStatus(bool portOpen, const RadioLinkStats& r,
    const StatusThresholds& th) {
    if (!portOpen) return RadioLinkStatus::NO_RADIO;

    // "Drone telemetry" = anything the FC itself produces. Link statistics
    // alone can come from the TX module and don't prove the drone is alive.
    int64_t newest = -1;
    for (int64_t a : { r.attitudeAgeMs, r.batteryAgeMs, r.flightModeAgeMs, r.gpsAgeMs })
        if (a >= 0 && (newest < 0 || a < newest)) newest = a;

    if (newest < 0) return RadioLinkStatus::WAITING;
    if (newest > th.lostTimeoutMs) return RadioLinkStatus::TELEMETRY_LOST;
    if (r.linkStatsValid && fresher(r.linkStatsAgeMs, th.lostTimeoutMs) && r.uplinkLq == 0)
        return RadioLinkStatus::TELEMETRY_LOST;

    const bool lqLow = r.linkStatsValid && fresher(r.linkStatsAgeMs, th.lostTimeoutMs)
        && r.uplinkLq < th.degradedLq;
    const bool attStale = !fresher(r.attitudeAgeMs, th.instrumentStaleMs);
    if (lqLow || attStale) return RadioLinkStatus::DEGRADED;
    return RadioLinkStatus::TELEMETRY_OK;
}

void CrsfStateMapper::refresh(DroneState& s, bool portOpen, int64_t now,
    const StatusThresholds& th) {
    RadioLinkStats& r = s.radio;
    r.msSinceLastFrame = age(tFrame_, now);
    r.attitudeAgeMs = age(tAtt_, now);
    r.gpsAgeMs = age(tGps_, now);
    r.batteryAgeMs = age(tBatt_, now);
    r.flightModeAgeMs = age(tMode_, now);
    r.baroAgeMs = age(tBaro_, now);
    r.linkStatsAgeMs = age(tLink_, now);

    // Rates over a rolling ~2 s window
    if (rateWindowStart_ < 0) rateWindowStart_ = now;
    const int64_t win = now - rateWindowStart_;
    if (win >= 2000) {
        const float k = 1000.0f / static_cast<float>(win);
        r.attitudeHz = nAtt_ * k;  r.gpsHz = nGps_ * k;   r.batteryHz = nBatt_ * k;
        r.flightModeHz = nMode_ * k; r.baroHz = nBaro_ * k; r.varioHz = nVario_ * k;
        r.linkStatsHz = nLink_ * k;  r.totalFrameHz = nAll_ * k;
        nAtt_ = nGps_ = nBatt_ = nMode_ = nBaro_ = nVario_ = nLink_ = nAll_ = 0;
        rateWindowStart_ = now;
    }

    // Altimeter fallback: GPS altitude relative to home when no fresh baro.
    if (!fresher(r.baroAgeMs, cfg_.baroStaleMs)) {
        if (cfg_.gpsAltitudeFallback && r.homeSet && s.gps.positionUsable) {
            s.baroAltitudeCm = static_cast<int32_t>(std::lround((s.gps.altitudeM - r.homeAltM) * 100.0f));
            s.baroValid = true;
            r.altitudeSource = "GPS";
        }
        else if (r.baroAgeMs < 0 && r.altitudeSource != "GPS") {
            r.altitudeSource = "NONE";
        }
        // else: keep the last source; its age is visible in baroAgeMs / gpsAgeMs
    }

    // Status + transition counters
    const RadioLinkStatus st = evaluateStatus(portOpen, r, th);
    const bool wasFlowing = prevStatus_ == RadioLinkStatus::TELEMETRY_OK ||
        prevStatus_ == RadioLinkStatus::DEGRADED;
    if (wasFlowing && st == RadioLinkStatus::TELEMETRY_LOST) ++r.linkLostCount;
    prevStatus_ = st;
    r.status = st;
    r.statusStr = statusName(st);

    s.linkSource = "ELRS";
    s.linkHealthy = st == RadioLinkStatus::TELEMETRY_OK || st == RadioLinkStatus::DEGRADED;
}

void CrsfStateMapper::resetTiming() {
    tFrame_ = tAtt_ = tGps_ = tBatt_ = tMode_ = tBaro_ = tLink_ = -1;
    rateWindowStart_ = -1;
    nAtt_ = nGps_ = nBatt_ = nMode_ = nBaro_ = nVario_ = nLink_ = nAll_ = 0;
}