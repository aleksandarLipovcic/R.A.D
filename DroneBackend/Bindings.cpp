#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DroneLink.h"
#include "IMUSensor.h"
#include "GPSNeoM10.h"

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(DroneBackend, m) {
    m.doc() = "Project R.A.D -- threaded drone telemetry backend (GCS edition)";

    // =========================================================================
    // GPSReading
    // =========================================================================
    py::class_<GPSReading>(m, "GPSReading")
        .def_readonly("fix_type", &GPSReading::fixType,
            "0 = no fix, 1 = 2D fix, 2 = 3D fix.")
        .def_readonly("num_sat", &GPSReading::numSat,
            "Number of satellites used in the solution.")
        .def_readonly("latitude", &GPSReading::latitude,
            "Decimal degrees. Positive = North, negative = South.")
        .def_readonly("longitude", &GPSReading::longitude,
            "Decimal degrees. Positive = East, negative = West.")
        .def_readonly("altitude_m", &GPSReading::altitudeM,
            "MSL altitude in metres.")
        .def_readonly("ground_speed_cms", &GPSReading::groundSpeedMs,
            "Ground speed in cm/s. Divide by 100.0 for m/s.")
        .def_readonly("ground_course", &GPSReading::groundCourse,
            "Ground course in decidegrees (0-3599). Divide by 10.0 for degrees.")
        .def_readonly("hdop", &GPSReading::hdop,
            "HDOP x100. 9999 = unknown (BF < 4.1). Good fix is below 200.")
        .def_readonly("dist_to_home_m", &GPSReading::distToHomM,
            "Distance to home point in metres. Valid after arming with 3D fix.")
        .def_readonly("bearing_to_home", &GPSReading::bearingToHome,
            "Bearing to home in degrees (-180 to +180).")
        .def_readonly("gps_heartbeat", &GPSReading::gpsHeartbeat,
            "Toggles 0<->1 on each fresh GPS frame. XOR with prev to detect new data.")
        .def_readonly("raw_valid", &GPSReading::rawValid,
            "True after first successful MSP_RAW_GPS parse.")
        .def_readonly("comp_valid", &GPSReading::compValid,
            "True after first successful MSP_COMP_GPS parse.")
        .def_readonly("position_usable", &GPSReading::positionUsable,
            "True when fixType>=2, numSat>=4, and HDOP<5.0. "
            "Always gate coordinate display on this flag, not raw_valid alone.");

    // =========================================================================
    // SVInfoEntry
    // =========================================================================
    py::class_<SVInfoEntry>(m, "SVInfoEntry")
        .def_readonly("chn", &SVInfoEntry::chn)
        .def_readonly("svid", &SVInfoEntry::svid)
        .def_readonly("flags", &SVInfoEntry::flags)
        .def_readonly("quality", &SVInfoEntry::quality)
        .def_readonly("cno", &SVInfoEntry::cno)
        .def_readonly("elev", &SVInfoEntry::elev)
        .def_readonly("azim", &SVInfoEntry::azim)
        .def_readonly("gnss_id", &SVInfoEntry::gnssId)
        .def_readonly("gnss_name", &SVInfoEntry::gnssName)
        .def_readonly("status_str", &SVInfoEntry::statusStr)
        .def_readonly("used", &SVInfoEntry::used);

    // =========================================================================
    // NavStatus
    // =========================================================================
    py::class_<NavStatus>(m, "NavStatus")
        .def_readonly("fix_type", &NavStatus::fixType)
        .def_readonly("gps_flags", &NavStatus::gpsFlags)
        .def_readonly("fix_ok", &NavStatus::fixOk)
        .def_readonly("dgps_used", &NavStatus::dgpsUsed)
        .def_readonly("map_flags", &NavStatus::mapFlags)
        .def_readonly("hw_status", &NavStatus::hwStatus)
        .def_readonly("valid", &NavStatus::valid);

    // =========================================================================
    // GPSConfig
    // =========================================================================
    py::class_<GPSConfig>(m, "GPSConfig")
        .def(py::init<>())
        .def_readwrite("constellations", &GPSConfig::constellations)
        .def_readwrite("update_rate_hz", &GPSConfig::updateRateHz)
        .def_readwrite("protocol", &GPSConfig::protocol)
        .def_readwrite("sbas_enabled", &GPSConfig::sbasEnabled)
        .def_readwrite("elevation_mask_deg", &GPSConfig::elevationMaskDeg)
        .def_readwrite("signal_mask_dbhz", &GPSConfig::signalMaskDbHz);

    // =========================================================================
    // GPSConfigResult
    // =========================================================================
    py::class_<GPSConfigResult>(m, "GPSConfigResult")
        .def_readonly("gnss_ack", &GPSConfigResult::gnssAck)
        .def_readonly("rate_ack", &GPSConfigResult::rateAck)
        .def_readonly("protocol_ack", &GPSConfigResult::protocolAck)
        .def_readonly("save_ack", &GPSConfigResult::saveAck)
        .def_readonly("overall_ok", &GPSConfigResult::overallOk)
        .def_readonly("error_detail", &GPSConfigResult::errorDetail);

    // =========================================================================
    // GPSConstellationFlags enum
    // =========================================================================
    py::enum_<GPSConstellationFlags>(m, "GPSConstellationFlags", py::arithmetic())
        .value("GNSS_GPS", GNSS_GPS)
        .value("GNSS_SBAS", GNSS_SBAS)
        .value("GNSS_GALILEO", GNSS_GALILEO)
        .value("GNSS_BEIDOU", GNSS_BEIDOU)
        .value("GNSS_QZSS", GNSS_QZSS)
        .value("GNSS_GLONASS", GNSS_GLONASS)
        .value("GNSS_DEFAULT", GNSS_DEFAULT)
        .value("GNSS_DUAL", GNSS_DUAL)
        .value("GNSS_TRIPLE", GNSS_TRIPLE)
        .value("GNSS_ALL", GNSS_ALL)
        .export_values();

    // =========================================================================
    // GPSUpdateRate enum
    // =========================================================================
    py::enum_<GPSUpdateRate>(m, "GPSUpdateRate")
        .value("RATE_1HZ", GPSUpdateRate::RATE_1HZ)
        .value("RATE_2HZ", GPSUpdateRate::RATE_2HZ)
        .value("RATE_5HZ", GPSUpdateRate::RATE_5HZ)
        .value("RATE_10HZ", GPSUpdateRate::RATE_10HZ)
        .export_values();

    // =========================================================================
    // GPSProtocol enum
    // =========================================================================
    py::enum_<GPSProtocol>(m, "GPSProtocol")
        .value("UBLOX", GPSProtocol::UBLOX)
        .value("NMEA", GPSProtocol::NMEA)
        .value("MSP", GPSProtocol::MSP)
        .export_values();

    // =========================================================================
    // BatteryState enum
    // =========================================================================
    py::enum_<BatteryState>(m, "BatteryState")
        .value("OK", BatteryState::OK)
        .value("WARNING", BatteryState::WARNING)
        .value("CRITICAL", BatteryState::CRITICAL)
        .value("NOT_PRESENT", BatteryState::NOT_PRESENT)
        .value("INIT", BatteryState::INIT)
        .export_values();

    // =========================================================================
    // DroneState
    // =========================================================================
    py::class_<DroneState>(m, "DroneState")

        // ── IMU ───────────────────────────────────────────────────────────────
        .def_readonly("ax", &DroneState::ax)
        .def_readonly("ay", &DroneState::ay)
        .def_readonly("az", &DroneState::az)
        .def_readonly("gx", &DroneState::gx)
        .def_readonly("gy", &DroneState::gy)
        .def_readonly("gz", &DroneState::gz)

        // ── Attitude ──────────────────────────────────────────────────────────
        .def_readonly("roll", &DroneState::roll)
        .def_readonly("pitch", &DroneState::pitch)
        .def_readonly("yaw", &DroneState::yaw)

        // ── Power ─────────────────────────────────────────────────────────────
        .def_readonly("battery_voltage", &DroneState::batteryVoltage)
        .def_readonly("battery_current", &DroneState::batteryCurrent)
        .def_readonly("battery_mah_drawn", &DroneState::batteryMahDrawn)
        .def_readonly("rssi", &DroneState::rssi)

        // ── Battery detail ────────────────────────────────────────────────────
        .def_readonly("battery_cell_count", &DroneState::batteryCellCount)
        .def_readonly("battery_capacity_mah", &DroneState::batteryCapacityMah)
        .def_readonly("battery_percentage", &DroneState::batteryPercentage)
        .def_readonly("battery_state", &DroneState::batteryState)

        // ── Barometer ─────────────────────────────────────────────────────────
        .def_readonly("baro_altitude_cm", &DroneState::baroAltitudeCm)
        .def_readonly("baro_vario_cm_per_sec", &DroneState::baroVarioCmPerSec)
        .def_readonly("baro_valid", &DroneState::baroValid)

        // ── Magnetometer ──────────────────────────────────────────────────────
        .def_readonly("mag_x", &DroneState::magX)
        .def_readonly("mag_y", &DroneState::magY)
        .def_readonly("mag_z", &DroneState::magZ)
        .def_readonly("mag_heading_deg", &DroneState::magHeadingDeg)
        .def_readonly("mag_valid", &DroneState::magValid)

        // ── Calibration state ─────────────────────────────────────────────────
        .def_readonly("mag_cal_active", &DroneState::magCalActive)
        .def_readonly("mag_cal_seconds_remaining", &DroneState::magCalSecondsRemaining)
        .def_readonly("acc_cal_active", &DroneState::accCalActive)
        .def_readonly("acc_cal_seconds_remaining", &DroneState::accCalSecondsRemaining)

        // ── FC status ─────────────────────────────────────────────────────────
        .def_readonly("armed", &DroneState::armed)
        .def_readonly("flight_mode_flags", &DroneState::flightModeFlags)
        .def_readonly("flight_mode_name", &DroneState::flightModeName)
        .def_readonly("sensor_status", &DroneState::sensorStatus)
        .def_readonly("i2c_error_count", &DroneState::i2cErrorCount)
        .def_readonly("cpu_load_percent", &DroneState::cpuLoadPercent)
        .def_readonly("pid_profile", &DroneState::pidProfile)

        // ── Arming diagnostics ────────────────────────────────────────────────
        .def_readonly("arming_disable_flags", &DroneState::armingDisableFlags)
        .def_readonly("arming_disable_str", &DroneState::armingDisableStr)

        // ── Motor outputs ─────────────────────────────────────────────────────
        .def_property_readonly("motor_values",
            [](const DroneState& s) {
                return std::vector<uint16_t>(
                    s.motorValues, s.motorValues + MAX_MOTORS);
            })
        .def_readonly("motor_count", &DroneState::motorCount)

        // ── RC channel inputs ─────────────────────────────────────────────────
        .def_property_readonly("rc_channels",
            [](const DroneState& s) {
                return std::vector<uint16_t>(
                    s.rcChannels, s.rcChannels + s.rcChannelCount);
            })
        .def_readonly("rc_channel_count", &DroneState::rcChannelCount)

        // ── GPS ───────────────────────────────────────────────────────────────
        .def_readonly("gps", &DroneState::gps)
        .def_readonly("sv_list", &DroneState::svList)
        .def_readonly("sv_info_valid", &DroneState::svInfoValid)
        .def_readonly("sv_source", &DroneState::svSource)
        .def_readonly("nav_status", &DroneState::navStatus)

        // ── Diagnostics ───────────────────────────────────────────────────────
        .def_readonly("last_rtt_ms", &DroneState::lastRttMs)
        .def_readonly("fc_cycle_ms", &DroneState::fcCycleMs)
        .def_readonly("link_healthy", &DroneState::linkHealthy)
        .def_readonly("packet_count", &DroneState::packetCount)

        // =====================================================================
        // to_dict()
        //
        // KEY NAME CONTRACT — every key here must match what the Python widgets
        // actually read via data.get("key_name"). Mismatches cause silent zeros.
        //
        // FIXED vs original bindings:
        //   "battery_v"         → "battery_voltage"   (FCStatusWidget, ArmingWidget)
        //   "battery_current_a" → "battery_current"   (FCStatusWidget)
        //   battery_state int   → "battery_state" str (FCStatusWidget reads string)
        //   motor_values list   → motor_1_us…motor_4_us individual keys (both widgets)
        //   rc_channels list    → rc_roll/pitch/throttle/yaw/arm individual keys
        //                         (FCStatusWidget)
        //   [MISSING]           → "rc_link_quality"   (ArmingWidget, FCStatusWidget)
        //                         derived from rssi: rssi is 0-255, widgets expect 0-100
        // =====================================================================
        .def("to_dict", [](const DroneState& s) {

        // ── Satellite list ────────────────────────────────────────────────
        py::list sv_list;
        for (const SVInfoEntry& sv : s.svList)
            sv_list.append(py::cast(sv));

        // ── Motor values (fixed 8-element list, AND individual named keys)
        // FCStatusWidget / ArmingWidget read motor_1_us … motor_4_us.
        // to_dict() previously only exported a "motor_values" list —
        // the individual keys were never emitted, so all motor widgets
        // showed "—" and the arming check reported NO MOTOR DATA.
        py::list motor_vals;
        for (int i = 0; i < MAX_MOTORS; ++i)
            motor_vals.append(s.motorValues[i]);

        // ── RC channels (variable length list, AND individual named keys)
        // FCStatusWidget reads rc_roll, rc_pitch, rc_throttle, rc_yaw, rc_arm.
        // RadioMaster Pocket / CRSF layout (MODE 2):
        //   [0]=Roll [1]=Pitch [2]=Throttle [3]=Yaw [4]=ARM switch
        py::list rc_ch;
        for (int i = 0; i < s.rcChannelCount; ++i)
            rc_ch.append(s.rcChannels[i]);

        // Helper: safely read an RC channel by index (0 if not present)
        auto rc = [&](int idx) -> uint16_t {
            return (idx < s.rcChannelCount) ? s.rcChannels[idx] : 0;
            };

        // ── RC link quality
        // DroneState has no dedicated link-quality field — the backend
        // receives RSSI from MSP_ANALOG (0-255 raw). Widgets expect 0-100.
        // Scale: quality = rssi * 100 / 255, clamped to 0-100.
        // When rssi == 0 (no link / not yet received) emit -1 so widgets
        // display "NO SIGNAL" rather than "0%".
        int rc_link_quality;
        if (s.rssi == 0) {
            rc_link_quality = -1;   // no signal — widget shows "NO SIGNAL"
        }
        else {
            rc_link_quality = static_cast<int>(
                static_cast<unsigned>(s.rssi) * 100u / 255u);
        }

        // ── Battery state string
        // FCStatusWidget reads battery_state as a string
        // ("OK"|"WARNING"|"CRITICAL"|"UNKNOWN").
        // The original to_dict() emitted battery_state as a raw integer
        // and battery_state_str as the string — but the widget only reads
        // the plain "battery_state" key and calls .upper() on it.
        const char* batt_state_str = "INIT";
        switch (s.batteryState) {
        case BatteryState::OK:          batt_state_str = "OK";          break;
        case BatteryState::WARNING:     batt_state_str = "WARNING";     break;
        case BatteryState::CRITICAL:    batt_state_str = "CRITICAL";    break;
        case BatteryState::NOT_PRESENT: batt_state_str = "NOT_PRESENT"; break;
        default:                        batt_state_str = "INIT";        break;
        }

        return py::dict(
            // ── IMU ───────────────────────────────────────────────────────
            "ax"_a = s.ax, "ay"_a = s.ay, "az"_a = s.az,
            "gx"_a = s.gx, "gy"_a = s.gy, "gz"_a = s.gz,

            // ── Attitude (pre-divided) ────────────────────────────────────
            "roll_deg"_a = s.roll / 10.0f,
            "pitch_deg"_a = s.pitch / 10.0f,
            "yaw_deg"_a = static_cast<float>(s.yaw),

            // ── Power — MSP_ANALOG (110) ──────────────────────────────────
            // FIX: was "battery_v" — widgets read "battery_voltage"
            "battery_voltage"_a = s.batteryVoltage,
            // FIX: was "battery_current_a" — widgets read "battery_current"
            "battery_current"_a = s.batteryCurrent,
            "battery_mah_drawn"_a = s.batteryMahDrawn,
            "rssi"_a = s.rssi,

            // ── Battery detail — MSP_BATTERY_STATE (242) ──────────────────
            "battery_cell_count"_a = s.batteryCellCount,
            "battery_capacity_mah"_a = s.batteryCapacityMah,
            "battery_percentage"_a = s.batteryPercentage,
            // FIX: was int — FCStatusWidget reads string and calls .upper()
            "battery_state"_a = batt_state_str,
            // Keep integer version under a distinct key for callers that want it
            "battery_state_int"_a = static_cast<uint8_t>(s.batteryState),

            // ── Barometer ─────────────────────────────────────────────────
            "baro_altitude_m"_a = s.baroAltitudeCm * 0.01,
            "baro_altitude_ft"_a = s.baroAltitudeCm * 0.0328084,
            "baro_vario_mps"_a = s.baroVarioCmPerSec * 0.01,
            "baro_vario_fpm"_a = s.baroVarioCmPerSec * 1.9685,
            "baro_valid"_a = s.baroValid,

            // ── Magnetometer ──────────────────────────────────────────────
            "mag_x"_a = s.magX,
            "mag_y"_a = s.magY,
            "mag_z"_a = s.magZ,
            "mag_heading_deg"_a = s.magHeadingDeg,
            "mag_valid"_a = s.magValid,

            // ── Calibration ───────────────────────────────────────────────
            "mag_cal_active"_a = s.magCalActive,
            "mag_cal_seconds_remaining"_a = s.magCalSecondsRemaining,
            "acc_cal_active"_a = s.accCalActive,
            "acc_cal_seconds_remaining"_a = s.accCalSecondsRemaining,

            // ── FC status — MSP_STATUS (101) ──────────────────────────────
            "armed"_a = s.armed,
            "flight_mode_flags"_a = s.flightModeFlags,
            "flight_mode_name"_a = s.flightModeName,
            "sensor_status"_a = s.sensorStatus,
            // Individual sensor-present helpers (avoids bitmask math in Python)
            "sensor_acc_present"_a = (s.sensorStatus & (1u << 0)) != 0,
            "sensor_baro_present"_a = (s.sensorStatus & (1u << 1)) != 0,
            "sensor_mag_present"_a = (s.sensorStatus & (1u << 2)) != 0,
            "sensor_gps_present"_a = (s.sensorStatus & (1u << 3)) != 0,
            "sensor_rangefinder_present"_a = (s.sensorStatus & (1u << 4)) != 0,
            "sensor_gyro_present"_a = (s.sensorStatus & (1u << 5)) != 0,
            "i2c_error_count"_a = s.i2cErrorCount,
            "cpu_load_percent"_a = s.cpuLoadPercent,
            "pid_profile"_a = s.pidProfile,

            // ── Arming diagnostics — MSP_STATUS_EX (150) ─────────────────
            "arming_disable_flags"_a = s.armingDisableFlags,
            "arming_disable_str"_a = s.armingDisableStr,

            // ── Motor outputs — MSP_MOTOR (104) ───────────────────────────
            // FIX: previously only "motor_values" list was emitted.
            // FCStatusWidget and ArmingWidget read individual keys motor_1_us … motor_4_us.
            "motor_values"_a = motor_vals,      // list kept for other consumers
            "motor_count"_a = s.motorCount,
            "motor_1_us"_a = static_cast<int>(s.motorValues[0]),
            "motor_2_us"_a = static_cast<int>(s.motorValues[1]),
            "motor_3_us"_a = static_cast<int>(s.motorValues[2]),
            "motor_4_us"_a = static_cast<int>(s.motorValues[3]),
            // Extra motors for hexacopter/octocopter support
            "motor_5_us"_a = static_cast<int>(s.motorValues[4]),
            "motor_6_us"_a = static_cast<int>(s.motorValues[5]),
            "motor_7_us"_a = static_cast<int>(s.motorValues[6]),
            "motor_8_us"_a = static_cast<int>(s.motorValues[7]),

            // ── RC channels — MSP_RC (105) ────────────────────────────────
            // FIX: previously only "rc_channels" list was emitted.
            // FCStatusWidget reads individual named keys per CRSF layout:
            //   [0]=Roll [1]=Pitch [2]=Throttle [3]=Yaw [4]=ARM
            "rc_channels"_a = rc_ch,         // list kept for other consumers
            "rc_channel_count"_a = s.rcChannelCount,
            "rc_roll"_a = static_cast<int>(rc(0)),
            "rc_pitch"_a = static_cast<int>(rc(1)),
            "rc_throttle"_a = static_cast<int>(rc(2)),
            "rc_yaw"_a = static_cast<int>(rc(3)),
            "rc_arm"_a = static_cast<int>(rc(4)),
            // AUX channels preserved for flight mode switches etc.
            "rc_aux1"_a = static_cast<int>(rc(5)),
            "rc_aux2"_a = static_cast<int>(rc(6)),
            "rc_aux3"_a = static_cast<int>(rc(7)),

            // ── RC link quality ───────────────────────────────────────────
            // FIX: was entirely missing from to_dict().
            // ArmingWidget._check_rc_link() and FCStatusWidget both read
            // "rc_link_quality" as an integer 0-100 (-1 = no signal).
            // Derived from RSSI (MSP_ANALOG rssi field, 0-255 raw):
            //   quality = rssi * 100 / 255  (-1 when rssi == 0)
            "rc_link_quality"_a = rc_link_quality,

            // ── GPS — MSP_RAW_GPS (106) ───────────────────────────────────
            "gps_fix_type"_a = s.gps.fixType,
            "gps_num_sat"_a = s.gps.numSat,
            "gps_latitude"_a = s.gps.latitude,
            "gps_longitude"_a = s.gps.longitude,
            "gps_altitude_m"_a = static_cast<double>(s.gps.altitudeM),
            "gps_altitude_ft"_a = s.gps.altitudeM * 3.28084,
            "gps_speed_mps"_a = s.gps.groundSpeedMs * 0.01,
            "gps_speed_kph"_a = s.gps.groundSpeedMs * 0.036,
            "gps_speed_mph"_a = s.gps.groundSpeedMs * 0.02237,
            "gps_course_deg"_a = s.gps.groundCourse * 0.1,
            "gps_hdop"_a = s.gps.hdop * 0.01,
            "gps_raw_valid"_a = s.gps.rawValid,
            "gps_position_usable"_a = s.gps.positionUsable,
            // Raw units for GPSWidget internals
            "gps_ground_speed_cms"_a = static_cast<int>(s.gps.groundSpeedMs),
            "gps_ground_course"_a = static_cast<int>(s.gps.groundCourse),
            // ArmingWidget reads gps_fix and gps_num_sats (short forms)
            "gps_fix"_a = s.gps.positionUsable,
            "gps_num_sats"_a = s.gps.numSat,

            // ── GPS — MSP_COMP_GPS (107) ──────────────────────────────────
            "gps_dist_home_m"_a = static_cast<double>(s.gps.distToHomM),
            "gps_dist_to_home_m"_a = static_cast<double>(s.gps.distToHomM),
            "gps_dist_home_ft"_a = s.gps.distToHomM * 3.28084,
            "gps_bearing_home"_a = static_cast<int>(s.gps.bearingToHome),
            "gps_bearing_to_home"_a = static_cast<int>(s.gps.bearingToHome),
            "gps_heartbeat"_a = s.gps.gpsHeartbeat,
            "gps_comp_valid"_a = s.gps.compValid,

            // ── GPS — satellite list ──────────────────────────────────────
            "gps_sv_list"_a = sv_list,
            "gps_sv_info_valid"_a = s.svInfoValid,
            "gps_sv_source"_a = s.svSource,

            // ── GPS — MSP_NAV_STATUS (121) ────────────────────────────────
            "gps_nav_fix_ok"_a = s.navStatus.fixOk,
            "gps_nav_dgps"_a = s.navStatus.dgpsUsed,

            // ── Diagnostics ───────────────────────────────────────────────
            "rtt_ms"_a = s.lastRttMs,
            "fc_cycle_ms"_a = s.fcCycleMs,
            "link_healthy"_a = s.linkHealthy,
            "packet_count"_a = s.packetCount
        );
            });

    // =========================================================================
    // DroneLink
    // =========================================================================
    py::class_<DroneLink>(m, "DroneLink")
        .def(py::init<>())
        .def("connect", &DroneLink::connect,
            py::arg("port_name"),
            "Open serial port and start the background polling thread.")
        .def("disconnect", &DroneLink::disconnect,
            "Stop background thread and close serial port.")
        .def("is_connected", &DroneLink::isConnected)
        .def("get_latest_state", &DroneLink::getLatestState,
            "Return a thread-safe snapshot of the latest telemetry.")
        .def("set_poll_interval_ms", &DroneLink::setPollIntervalMs,
            py::arg("ms"),
            "Tune background thread cadence (default 10 ms = 100 Hz).")
        .def("set_gps_uart_index", &DroneLink::setGpsUartIndex,
            py::arg("index"),
            "Set the BF serial port index used by applyGPSConfig() passthrough.\n"
            "UART1=0, UART2=1 (F405 V3 default), UART3=2.\n"
            "Has no effect on satellite polling (uses MSP cmd 164, no UART index needed).\n"
            "Must be called before connect().")
        .def("start_mag_calibration", &DroneLink::startMagCalibration)
        .def("start_acc_calibration", &DroneLink::startAccCalibration)
        .def("apply_gps_config", &DroneLink::applyGPSConfig,
            py::arg("config"),
            "Send UBX CFG-GNSS/RATE/PRT/NAV5/CFG frames to the NEO-M10 via\n"
            "MSP GPS passthrough.  Returns GPSConfigResult with per-step ACK\n"
            "status.")
        // ── DEF-005 rev 2: fault injection for SAT-IMU-005 Scenario B ────────
        // When active, sendMSP() returns {} immediately on every call,
        // simulating a dead link without relying on serial-timing side-effects.
        // This deterministically increments consecutiveFails until
        // FAIL_THRESHOLD is reached and linkHealthy flips False (~50 ms at
        // default POLL_INTERVAL_MS = 10 ms).
        // Scenarios A and C continue to use set_poll_interval_ms(0); only
        // Scenario B uses this method.
        // NEVER ship in flight builds — test use only.
        .def("set_fail_injection", &DroneLink::setFailInjection,
            py::arg("active"),
            "Inject simulated link failure: sendMSP() returns {} immediately\n"
            "on every call when active=True, regardless of FC responsiveness.\n"
            "Used by SAT-IMU-005 Scenario B (DEF-005 rev 2).\n"
            "Call with active=False to restore normal operation.\n"
            "TEST USE ONLY — never enable in flight builds.");

    // =========================================================================
    // IMUSensor
    // =========================================================================
    py::class_<IMUSensor>(m, "IMUSensor")
        .def(py::init<DroneLink*>(), py::arg("hub"))
        .def("get_raw_data", [](IMUSensor& self) {
        auto d = self.getRawData();
        return py::dict(
            "ax"_a = d.accX, "ay"_a = d.accY, "az"_a = d.accZ,
            "gx"_a = d.gyroX, "gy"_a = d.gyroY, "gz"_a = d.gyroZ
        );
            }, "Raw ADC counts from MPU-6500.")
        .def("get_scaled_data", [](IMUSensor& self) {
        auto d = self.getScaledData();
        return py::dict(
            "ax_g"_a = d.accX, "ay_g"_a = d.accY, "az_g"_a = d.accZ,
            "gx_dps"_a = d.gyroX, "gy_dps"_a = d.gyroY, "gz_dps"_a = d.gyroZ
        );
            }, "Scaled data: accel in g, gyro in deg/s.");

    // =========================================================================
    // Free functions
    // =========================================================================
    m.def("auto_detect_f405", &AutoDetectF405,
        "Scan COM1-COM29 and return the first port that opens, or 'NOT_FOUND'.");
}