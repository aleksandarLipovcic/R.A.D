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
    //
    // Populated by MSP_GPS_SV_INFO (cmd 164) via GPSNeoM10::parseMspSvInfo().
    //
    // Fields always populated (MSP and UBX sources):
    //   gnss_name, svid, cno, quality, status_str, used, gnss_id, flags, chn
    //
    // Fields that are 0 when sv_source == "MSP" (not in cmd 164 payload):
    //   elev, azim, prRes
    //
    // Python UI should check state.sv_source before showing Elev/Azim columns:
    //   if state.sv_source == "MSP":
    //       hide elev/azim columns in the satellite table
    // =========================================================================
    py::class_<SVInfoEntry>(m, "SVInfoEntry")
        .def_readonly("chn", &SVInfoEntry::chn,
            "Channel index. For MSP source: SV index (0-31). "
            "For UBX source: receiver tracking channel.")
        .def_readonly("svid", &SVInfoEntry::svid,
            "Satellite vehicle ID (PRN for GPS, slot for GLONASS).")
        .def_readonly("flags", &SVInfoEntry::flags,
            "Packed flags byte: bits[0:2]=quality, bit[3]=svUsed. "
            "Same encoding for both MSP and UBX sources.")
        .def_readonly("quality", &SVInfoEntry::quality,
            "UBX qualityInd 0-7: 0=no sig,1=searching,2=acquired,3=unusable,"
            "4=code locked,5-7=code+carrier locked.")
        .def_readonly("cno", &SVInfoEntry::cno,
            "Carrier-to-noise density, dBHz. 0-55.")
        .def_readonly("elev", &SVInfoEntry::elev,
            "Elevation above horizon, degrees (-90 to +90). "
            "Always 0 when sv_source == 'MSP' (not available in cmd 164).")
        .def_readonly("azim", &SVInfoEntry::azim,
            "Azimuth, degrees (0-360). "
            "Always 0 when sv_source == 'MSP' (not available in cmd 164).")
        .def_readonly("gnss_id", &SVInfoEntry::gnssId,
            "GNSS system ID: 0=GPS,1=SBAS,2=Galileo,3=BeiDou,5=QZSS,6=GLONASS.")
        .def_readonly("gnss_name", &SVInfoEntry::gnssName,
            "Human-readable GNSS name: 'GPS', 'GLONASS', 'Galileo', etc.")
        .def_readonly("status_str", &SVInfoEntry::statusStr,
            "'used', 'tracked', 'acquired', 'searching', or 'idle'.")
        .def_readonly("used", &SVInfoEntry::used,
            "True when this satellite contributes to the fix solution.");

    // =========================================================================
    // NavStatus
    // =========================================================================
    py::class_<NavStatus>(m, "NavStatus")
        .def_readonly("fix_type", &NavStatus::fixType)
        .def_readonly("gps_flags", &NavStatus::gpsFlags)
        .def_readonly("fix_ok", &NavStatus::fixOk,
            "True when GPS fix is valid (gpsFlags bit0).")
        .def_readonly("dgps_used", &NavStatus::dgpsUsed,
            "True when differential GPS correction is active.")
        .def_readonly("map_flags", &NavStatus::mapFlags)
        .def_readonly("hw_status", &NavStatus::hwStatus)
        .def_readonly("valid", &NavStatus::valid);

    // =========================================================================
    // GPSConfig
    // =========================================================================
    py::class_<GPSConfig>(m, "GPSConfig")
        .def(py::init<>())
        .def_readwrite("constellations", &GPSConfig::constellations,
            "Bitmask of GPSConstellationFlags.")
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

        // ── Power — MSP_ANALOG (110) ──────────────────────────────────────────
        .def_readonly("battery_voltage", &DroneState::batteryVoltage,
            "Battery voltage in Volts. Overwritten by parseBatteryState() with "
            "10 mV-resolution value when available.")
        .def_readonly("battery_current", &DroneState::batteryCurrent,
            "Battery current draw in Amps (centiamps / 100).")
        .def_readonly("battery_mah_drawn", &DroneState::batteryMahDrawn,
            "Cumulative charge consumed in mAh since boot.")
        .def_readonly("rssi", &DroneState::rssi,
            "Received signal strength 0-255 from MSP_ANALOG.")

        // ── Battery detail — MSP_BATTERY_STATE (242) ──────────────────────────
        .def_readonly("battery_cell_count", &DroneState::batteryCellCount,
            "Auto-detected LiPo cell count (e.g. 4 for a 4S pack).")
        .def_readonly("battery_capacity_mah", &DroneState::batteryCapacityMah,
            "Design capacity in mAh. 0 if not configured in BF Configurator "
            "(set battery_capacity = <mAh>; save).")
        .def_readonly("battery_percentage", &DroneState::batteryPercentage,
            "Remaining capacity 0-100 %. Computed from mAh drawn vs design capacity. "
            "0 when battery_capacity_mah == 0.")
        .def_readonly("battery_state", &DroneState::batteryState,
            "BatteryState enum: OK / WARNING / CRITICAL / NOT_PRESENT / INIT.")

        // ── Barometer — BMP280 via MSP_ALTITUDE (109) ─────────────────────────
        .def_readonly("baro_altitude_cm", &DroneState::baroAltitudeCm,
            "FC-fused BMP280 altitude above home point, cm.")
        .def_readonly("baro_vario_cm_per_sec", &DroneState::baroVarioCmPerSec,
            "Vertical speed cm/s. Positive = climbing.")
        .def_readonly("baro_valid", &DroneState::baroValid,
            "True when a valid MSP_ALTITUDE frame was received.")

        // ── Magnetometer — QMC5883L via MSP_DEBUG (254) ───────────────────────
        .def_readonly("mag_x", &DroneState::magX,
            "Raw mag X ADC counts. Requires debug_mode=MAG_CALIB in BF CLI.")
        .def_readonly("mag_y", &DroneState::magY)
        .def_readonly("mag_z", &DroneState::magZ)
        .def_readonly("mag_heading_deg", &DroneState::magHeadingDeg,
            "Tilt-uncorrected 2D magnetic heading 0-360 deg.")
        .def_readonly("mag_valid", &DroneState::magValid,
            "True when mag X/Y/Z are non-zero.")

        // ── Calibration state ─────────────────────────────────────────────────
        .def_readonly("mag_cal_active", &DroneState::magCalActive)
        .def_readonly("mag_cal_seconds_remaining", &DroneState::magCalSecondsRemaining)
        .def_readonly("acc_cal_active", &DroneState::accCalActive)
        .def_readonly("acc_cal_seconds_remaining", &DroneState::accCalSecondsRemaining)

        // ── FC status — MSP_STATUS (101) ──────────────────────────────────────
        .def_readonly("armed", &DroneState::armed,
            "True when the ARM box is active (FlightMode::ARM bit set).")
        .def_readonly("flight_mode_flags", &DroneState::flightModeFlags,
            "Raw bitmask from MSP_STATUS. Use FlightMode:: constants to decode.")
        .def_readonly("flight_mode_name", &DroneState::flightModeName,
            "Human-readable flight mode string, e.g. 'ANGLE', 'ACRO+MAG', "
            "'GPS RESCUE', 'ANGLE [DISARMED]'.")
        .def_readonly("sensor_status", &DroneState::sensorStatus,
            "Sensor presence bitmask from MSP_STATUS. "
            "Bits: ACC=0, BARO=1, MAG=2, GPS=3, RANGEFINDER=4, GYRO=5.")
        .def_readonly("i2c_error_count", &DroneState::i2cErrorCount,
            "I2C bus error count since boot. Non-zero suggests a wiring issue.")
        .def_readonly("cpu_load_percent", &DroneState::cpuLoadPercent,
            "Average FC CPU load 0-100 %. >80% risks loop overruns.")
        .def_readonly("pid_profile", &DroneState::pidProfile,
            "Active PID profile index (0-based).")

        // ── Arming diagnostics — MSP_STATUS_EX (150) ─────────────────────────
        .def_readonly("arming_disable_flags", &DroneState::armingDisableFlags,
            "Raw bitmask of reasons the FC refuses to arm. "
            "0 = ready to arm. Use ArmingDisable:: constants to decode.")
        .def_readonly("arming_disable_str", &DroneState::armingDisableStr,
            "Comma-separated human-readable arming-block reasons, "
            "e.g. 'THROTTLE HIGH, NOT LEVEL'. Empty string when ready to arm.")

        // ── Motor outputs — MSP_MOTOR (104) ──────────────────────────────────
        .def_property_readonly("motor_values",
            [](const DroneState& s) {
                return std::vector<uint16_t>(
                    s.motorValues, s.motorValues + MAX_MOTORS);
            },
            "List of 8 motor throttle values in microseconds (1000-2000). "
            "On a quad only indices 0-3 are non-zero. "
            "Use motor_count to know how many motors are active.")
        .def_readonly("motor_count", &DroneState::motorCount,
            "Number of active motors (index of last non-zero motor + 1). "
            "4 for a standard quadcopter.")

        // ── RC channel inputs — MSP_RC (105) ─────────────────────────────────
        .def_property_readonly("rc_channels",
            [](const DroneState& s) {
                return std::vector<uint16_t>(
                    s.rcChannels, s.rcChannels + s.rcChannelCount);
            },
            "List of RC channel values in microseconds (1000-2000). "
            "Length = rc_channel_count. RadioMaster Pocket / CRSF layout (MODE 2): "
            "[0]=Roll, [1]=Pitch, [2]=Throttle, [3]=Yaw, "
            "[4]=ARM switch, [5]=Flight mode AUX, [6+]=AUX3...")
        .def_readonly("rc_channel_count", &DroneState::rcChannelCount,
            "Number of active RC channels (typically 12-16 with ELRS/CRSF).")

        // ── GPS — NEO-M10 ─────────────────────────────────────────────────────
        .def_readonly("gps", &DroneState::gps,
            "GPSReading from NEO-M10. "
            "Check gps.raw_valid before position fields, "
            "gps.comp_valid before home fields, "
            "gps.position_usable before any navigation use.")

        // ── Satellite list ────────────────────────────────────────────────────
        .def_readonly("sv_list", &DroneState::svList,
            "List of SVInfoEntry. Updated every ~1 s via MSP cmd 164. "
            "Fields gnss_name/svid/cno/quality/status_str/used/gnss_id are "
            "always populated. elev/azim are 0 (not available via MSP).")
        .def_readonly("sv_info_valid", &DroneState::svInfoValid,
            "True when sv_list has been populated at least once (~1 s after connect).")
        .def_readonly("sv_source", &DroneState::svSource,
            "'MSP' when populated via cmd 164 (normal). "
            "'UBX' when populated via passthrough (only if gps_auto_config=OFF). "
            "Check this before showing elev/azim columns in the satellite table.")

        // ── GPS nav engine status — MSP_NAV_STATUS (121) ──────────────────────
        .def_readonly("nav_status", &DroneState::navStatus,
            "NavStatus from MSP_NAV_STATUS (121). fix_ok is the authoritative fix flag.")

        // ── Diagnostics ───────────────────────────────────────────────────────
        .def_readonly("last_rtt_ms", &DroneState::lastRttMs)
        .def_readonly("fc_cycle_ms", &DroneState::fcCycleMs,
            "FC loop cycle time in ms from MSP_STATUS (101).")
        .def_readonly("link_healthy", &DroneState::linkHealthy)
        .def_readonly("packet_count", &DroneState::packetCount)

        // ── to_dict() ─────────────────────────────────────────────────────────
        // Convenience snapshot with all values pre-converted to useful units.
        //
        // New keys vs previous version:
        //   battery_current_a       — current draw in Amps
        //   battery_mah_drawn       — mAh consumed
        //   battery_cell_count      — detected cell count
        //   battery_capacity_mah    — design capacity (0 = not configured in BF)
        //   battery_percentage      — 0-100 % (0 if capacity not configured)
        //   battery_state           — BatteryState integer (0=OK,1=WARN,2=CRIT,3=NO BAT,4=INIT)
        //   battery_state_str       — human-readable state string
        //   armed                   — bool
        //   flight_mode_flags       — raw uint32 bitmask
        //   flight_mode_name        — decoded string
        //   sensor_status           — raw uint16 bitmask
        //   sensor_acc/baro/mag/gps_present/rangefinder/gyro — individual bool helpers
        //   i2c_error_count         — I2C bus errors since boot
        //   cpu_load_percent        — FC CPU load 0-100 %
        //   pid_profile             — active PID profile index
        //   arming_disable_flags    — raw uint32 bitmask (0 = ready to arm)
        //   arming_disable_str      — comma-separated reason list or ""
        //   motor_values            — list of 8 uint16 µs values
        //   motor_count             — number of active motors
        //   rc_channels             — list of uint16 µs values, length = rc_channel_count
        //   rc_channel_count        — number of active RC channels
        //
        // gps_sv_list  : py::list of SVInfoEntry objects.
        // gps_sv_source: "MSP" in normal operation; hide Elev/Azim when "MSP".
        .def("to_dict", [](const DroneState& s) {

        // ── satellite list ────────────────────────────────────────────────────
        py::list sv_list;
        for (const SVInfoEntry& sv : s.svList)
            sv_list.append(py::cast(sv));

        // ── motor values (fixed 8-element list) ───────────────────────────────
        py::list motor_vals;
        for (int i = 0; i < MAX_MOTORS; ++i)
            motor_vals.append(s.motorValues[i]);

        // ── RC channels (variable length, only active channels) ───────────────
        py::list rc_ch;
        for (int i = 0; i < s.rcChannelCount; ++i)
            rc_ch.append(s.rcChannels[i]);

        // ── battery state string ──────────────────────────────────────────────
        const char* batt_state_str = "INIT";
        switch (s.batteryState) {
        case BatteryState::OK:          batt_state_str = "OK";          break;
        case BatteryState::WARNING:     batt_state_str = "WARNING";     break;
        case BatteryState::CRITICAL:    batt_state_str = "CRITICAL";    break;
        case BatteryState::NOT_PRESENT: batt_state_str = "NOT_PRESENT"; break;
        default:                        batt_state_str = "INIT";        break;
        }

        return py::dict(
            // ── IMU ───────────────────────────────────────────────────────────
            "ax"_a = s.ax, "ay"_a = s.ay, "az"_a = s.az,
            "gx"_a = s.gx, "gy"_a = s.gy, "gz"_a = s.gz,

            // ── Attitude (pre-divided) ────────────────────────────────────────
            "roll_deg"_a = s.roll / 10.0f,
            "pitch_deg"_a = s.pitch / 10.0f,
            "yaw_deg"_a = static_cast<float>(s.yaw),

            // ── Power — MSP_ANALOG (110) ──────────────────────────────────────
            "battery_v"_a = s.batteryVoltage,
            "battery_current_a"_a = s.batteryCurrent,
            "battery_mah_drawn"_a = s.batteryMahDrawn,
            "rssi"_a = s.rssi,

            // ── Battery detail — MSP_BATTERY_STATE (242) ──────────────────────
            "battery_cell_count"_a = s.batteryCellCount,
            "battery_capacity_mah"_a = s.batteryCapacityMah,
            "battery_percentage"_a = s.batteryPercentage,
            "battery_state"_a = static_cast<uint8_t>(s.batteryState),
            "battery_state_str"_a = batt_state_str,

            // ── Barometer ─────────────────────────────────────────────────────
            "baro_altitude_m"_a = s.baroAltitudeCm * 0.01,
            "baro_altitude_ft"_a = s.baroAltitudeCm * 0.0328084,
            "baro_vario_mps"_a = s.baroVarioCmPerSec * 0.01,
            "baro_vario_fpm"_a = s.baroVarioCmPerSec * 1.9685,
            "baro_valid"_a = s.baroValid,

            // ── Magnetometer ──────────────────────────────────────────────────
            "mag_x"_a = s.magX,
            "mag_y"_a = s.magY,
            "mag_z"_a = s.magZ,
            "mag_heading_deg"_a = s.magHeadingDeg,
            "mag_valid"_a = s.magValid,

            // ── Calibration ───────────────────────────────────────────────────
            "mag_cal_active"_a = s.magCalActive,
            "mag_cal_seconds_remaining"_a = s.magCalSecondsRemaining,
            "acc_cal_active"_a = s.accCalActive,
            "acc_cal_seconds_remaining"_a = s.accCalSecondsRemaining,

            // ── FC status — MSP_STATUS (101) ──────────────────────────────────
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

            // ── Arming diagnostics — MSP_STATUS_EX (150) ─────────────────────
            "arming_disable_flags"_a = s.armingDisableFlags,
            "arming_disable_str"_a = s.armingDisableStr,

            // ── Motor outputs — MSP_MOTOR (104) ───────────────────────────────
            "motor_values"_a = motor_vals,   // always 8 elements; non-motors = 0
            "motor_count"_a = s.motorCount,

            // ── RC channels — MSP_RC (105) ────────────────────────────────────
            "rc_channels"_a = rc_ch,            // variable length list
            "rc_channel_count"_a = s.rcChannelCount,

            // ── GPS — MSP_RAW_GPS (106) ───────────────────────────────────────
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

            // Raw cm/s and decidegrees for GPSWidget._update_navigation()
            "gps_ground_speed_cms"_a = static_cast<int>(s.gps.groundSpeedMs),
            "gps_ground_course"_a = static_cast<int>(s.gps.groundCourse),

            // ── GPS — MSP_COMP_GPS (107) — dual key names for compatibility ───
            "gps_dist_home_m"_a = static_cast<double>(s.gps.distToHomM),
            "gps_dist_to_home_m"_a = static_cast<double>(s.gps.distToHomM),
            "gps_dist_home_ft"_a = s.gps.distToHomM * 3.28084,
            "gps_bearing_home"_a = static_cast<int>(s.gps.bearingToHome),
            "gps_bearing_to_home"_a = static_cast<int>(s.gps.bearingToHome),
            "gps_heartbeat"_a = s.gps.gpsHeartbeat,
            "gps_comp_valid"_a = s.gps.compValid,

            // ── GPS — satellite list (MSP cmd 164, every ~1 s) ────────────────
            "gps_sv_list"_a = sv_list,
            "gps_sv_info_valid"_a = s.svInfoValid,
            "gps_sv_source"_a = s.svSource,   // "MSP" or "UBX"

            // ── GPS — MSP_NAV_STATUS (121) ────────────────────────────────────
            "gps_nav_fix_ok"_a = s.navStatus.fixOk,
            "gps_nav_dgps"_a = s.navStatus.dgpsUsed,

            // ── Diagnostics ───────────────────────────────────────────────────
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
        .def("start_mag_calibration", &DroneLink::startMagCalibration,
            "Send MSP_MAG_CALIBRATION (206) to the FC.\n"
            "FC enters calibration mode for 30 s. Rotate drone on all axes.\n"
            "Monitor state.mag_cal_active and state.mag_cal_seconds_remaining.")
        .def("start_acc_calibration", &DroneLink::startAccCalibration,
            "Send MSP_ACC_CALIBRATION (205) to the FC.\n"
            "Keep drone perfectly level and still for ~5 s.\n"
            "Monitor state.acc_cal_active and state.acc_cal_seconds_remaining.")
        .def("apply_gps_config", &DroneLink::applyGPSConfig,
            py::arg("config"),
            "Send UBX CFG-GNSS/RATE/PRT/NAV5/CFG frames to the NEO-M10 via\n"
            "MSP GPS passthrough.  Returns GPSConfigResult with per-step ACK\n"
            "status.");

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