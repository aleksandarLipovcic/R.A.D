#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <cstring>
#include "DroneLink.h"
#include "IMUSensor.h"
#include "GPSNeoM10.h"
#include "VideoLink.h"
#include "DetectionLink.h"
#include "CrsfLink.h"
#include "SerialPortScan.h"

namespace py = pybind11;
using namespace pybind11::literals;

// =============================================================================
// matToNumpy()
//
// Converts a cv::Mat (BGR8, HxWx3) into a numpy array owned by Python.
// The memcpy here is unavoidable: cv::Mat's underlying buffer isn't
// guaranteed to outlive the Python-side object once we cross the pybind11
// boundary, so we hand numpy its own memory rather than aliasing the
// cv::Mat's buffer. This is the one intentional copy in the whole path
// (capture -> latestFrame is a header swap, latestFrame -> here is the
// copy, here -> PhotoImage in Tk is unavoidable on the Python side too).
//
// NOTE: this path is only used by get_latest_frame() -- e.g. for
// post-flight recording/YOLO post-processing. The live FPV display no
// longer goes through here at all; it's painted directly by VideoLink
// onto a native child window via attach_to_window()/paintFrame(), so it
// never crosses into Python/numpy/Tk in the first place.
// =============================================================================

static py::array_t<uint8_t> matToNumpy(const cv::Mat& mat) {
    if (mat.empty())
        return py::array_t<uint8_t>();

    cv::Mat contiguous = mat.isContinuous() ? mat : mat.clone();
    py::array_t<uint8_t> result({ contiguous.rows, contiguous.cols, contiguous.channels() });
    std::memcpy(result.mutable_data(), contiguous.data,
        contiguous.total() * contiguous.elemSize());
    return result;
}

PYBIND11_MODULE(DroneBackend, m) {
    m.doc() = "Project R.A.D -- threaded drone telemetry + video backend (GCS edition)";

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
    // RadioLinkStatus / RadioLinkStats — ELRS telemetry path (CrsfLink)
    // =========================================================================
    py::enum_<RadioLinkStatus>(m, "RadioLinkStatus")
        .value("NO_RADIO", RadioLinkStatus::NO_RADIO)
        .value("WAITING", RadioLinkStatus::WAITING)
        .value("TELEMETRY_OK", RadioLinkStatus::TELEMETRY_OK)
        .value("DEGRADED", RadioLinkStatus::DEGRADED)
        .value("TELEMETRY_LOST", RadioLinkStatus::TELEMETRY_LOST)
        .export_values();

    py::class_<RadioLinkStats>(m, "RadioLinkStats")
        .def_readonly("status", &RadioLinkStats::status)
        .def_readonly("status_str", &RadioLinkStats::statusStr)
        .def_readonly("port_name", &RadioLinkStats::portName)
        .def_readonly("link_stats_valid", &RadioLinkStats::linkStatsValid)
        .def_readonly("uplink_rssi1_dbm", &RadioLinkStats::uplinkRssi1Dbm)
        .def_readonly("uplink_rssi2_dbm", &RadioLinkStats::uplinkRssi2Dbm)
        .def_readonly("uplink_lq", &RadioLinkStats::uplinkLq)
        .def_readonly("uplink_snr", &RadioLinkStats::uplinkSnr)
        .def_readonly("active_antenna", &RadioLinkStats::activeAntenna)
        .def_readonly("rf_mode_index", &RadioLinkStats::rfModeIndex)
        .def_readonly("tx_power_mw", &RadioLinkStats::txPowerMw)
        .def_readonly("downlink_rssi_dbm", &RadioLinkStats::downlinkRssiDbm)
        .def_readonly("downlink_lq", &RadioLinkStats::downlinkLq)
        .def_readonly("downlink_snr", &RadioLinkStats::downlinkSnr)
        .def_readonly("ms_since_last_frame", &RadioLinkStats::msSinceLastFrame)
        .def_readonly("attitude_age_ms", &RadioLinkStats::attitudeAgeMs)
        .def_readonly("gps_age_ms", &RadioLinkStats::gpsAgeMs)
        .def_readonly("battery_age_ms", &RadioLinkStats::batteryAgeMs)
        .def_readonly("flight_mode_age_ms", &RadioLinkStats::flightModeAgeMs)
        .def_readonly("baro_age_ms", &RadioLinkStats::baroAgeMs)
        .def_readonly("link_stats_age_ms", &RadioLinkStats::linkStatsAgeMs)
        .def_readonly("attitude_hz", &RadioLinkStats::attitudeHz)
        .def_readonly("gps_hz", &RadioLinkStats::gpsHz)
        .def_readonly("battery_hz", &RadioLinkStats::batteryHz)
        .def_readonly("flight_mode_hz", &RadioLinkStats::flightModeHz)
        .def_readonly("baro_hz", &RadioLinkStats::baroHz)
        .def_readonly("vario_hz", &RadioLinkStats::varioHz)
        .def_readonly("link_stats_hz", &RadioLinkStats::linkStatsHz)
        .def_readonly("total_frame_hz", &RadioLinkStats::totalFrameHz)
        .def_readonly("frames_total", &RadioLinkStats::framesTotal)
        .def_readonly("crc_errors", &RadioLinkStats::crcErrors)
        .def_readonly("reconnect_count", &RadioLinkStats::reconnectCount)
        .def_readonly("link_lost_count", &RadioLinkStats::linkLostCount)
        .def_readonly("raw_flight_mode", &RadioLinkStats::rawFlightMode)
        .def_readonly("arming_blocked", &RadioLinkStats::armingBlocked)
        .def_readonly("gps_waiting", &RadioLinkStats::gpsWaiting)
        .def_readonly("home_set", &RadioLinkStats::homeSet)
        .def_readonly("home_lat", &RadioLinkStats::homeLat)
        .def_readonly("home_lon", &RadioLinkStats::homeLon)
        .def_readonly("home_alt_m", &RadioLinkStats::homeAltM)
        .def_readonly("altitude_source", &RadioLinkStats::altitudeSource);

    py::class_<SerialPortInfo>(m, "SerialPortInfo")
        .def_readonly("port", &SerialPortInfo::port)
        .def_readonly("friendly_name", &SerialPortInfo::friendlyName)
        .def_readonly("bus_description", &SerialPortInfo::busDescription,
            "USB product string, e.g. 'Betaflight STM32F405' or the radio's name.")
        .def_readonly("vid", &SerialPortInfo::vid)
        .def_readonly("pid", &SerialPortInfo::pid)
        .def("__repr__", [](const SerialPortInfo& p) {
            return "<SerialPortInfo " + p.port + " '" + p.busDescription + "' / '" + p.friendlyName + "'>";
        });

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

        // ── Telemetry source / ELRS link ──────────────────────────────────────
        .def_readonly("link_source", &DroneState::linkSource,
            "'USB' (DroneLink, MSP) or 'ELRS' (CrsfLink, CRSF via RadioMaster Pocket).")
        .def_readonly("radio", &DroneState::radio,
            "RadioLinkStats — only meaningful when link_source == 'ELRS'.")

        // =====================================================================
        // to_dict()
        //
        // KEY NAME CONTRACT — every key here must match what the Python widgets
        // actually read via data.get("key_name"). Mismatches cause silent zeros.
        // =====================================================================
        .def("to_dict", [](const DroneState& s) {

        // ── Satellite list ────────────────────────────────────────────────
        py::list sv_list;
        for (const SVInfoEntry& sv : s.svList)
            sv_list.append(py::cast(sv));

        // ── Motor values (fixed 8-element list, AND individual named keys)
        py::list motor_vals;
        for (int i = 0; i < MAX_MOTORS; ++i)
            motor_vals.append(s.motorValues[i]);

        // ── RC channels (variable length list, AND individual named keys)
        py::list rc_ch;
        for (int i = 0; i < s.rcChannelCount; ++i)
            rc_ch.append(s.rcChannels[i]);

        auto rc = [&](int idx) -> uint16_t {
            return (idx < s.rcChannelCount) ? s.rcChannels[idx] : 0;
            };

        int rc_link_quality;
        if (s.linkSource == "ELRS" && s.radio.linkStatsValid) {
            rc_link_quality = s.radio.uplinkLq;   // real ELRS LQ, 0 means lost
        }
        else if (s.rssi == 0) {
            rc_link_quality = -1;
        }
        else {
            rc_link_quality = static_cast<int>(
                static_cast<unsigned>(s.rssi) * 100u / 255u);
        }

        const char* batt_state_str = "INIT";
        switch (s.batteryState) {
        case BatteryState::OK:          batt_state_str = "OK";          break;
        case BatteryState::WARNING:     batt_state_str = "WARNING";     break;
        case BatteryState::CRITICAL:    batt_state_str = "CRITICAL";    break;
        case BatteryState::NOT_PRESENT: batt_state_str = "NOT_PRESENT"; break;
        default:                        batt_state_str = "INIT";        break;
        }

        py::dict d(
            // ── IMU ───────────────────────────────────────────────────────
            "ax"_a = s.ax, "ay"_a = s.ay, "az"_a = s.az,
            "gx"_a = s.gx, "gy"_a = s.gy, "gz"_a = s.gz,

            // ── Attitude (pre-divided) ────────────────────────────────────
            "roll_deg"_a = s.roll / 10.0f,
            "pitch_deg"_a = s.pitch / 10.0f,
            "yaw_deg"_a = static_cast<float>(s.yaw),

            // ── Power — MSP_ANALOG (110) ──────────────────────────────────
            "battery_voltage"_a = s.batteryVoltage,
            "battery_current"_a = s.batteryCurrent,
            "battery_mah_drawn"_a = s.batteryMahDrawn,
            "rssi"_a = s.rssi,

            // ── Battery detail — MSP_BATTERY_STATE (242) ──────────────────
            "battery_cell_count"_a = s.batteryCellCount,
            "battery_capacity_mah"_a = s.batteryCapacityMah,
            "battery_percentage"_a = s.batteryPercentage,
            "battery_state"_a = batt_state_str,
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
            "motor_values"_a = motor_vals,
            "motor_count"_a = s.motorCount,
            "motor_1_us"_a = static_cast<int>(s.motorValues[0]),
            "motor_2_us"_a = static_cast<int>(s.motorValues[1]),
            "motor_3_us"_a = static_cast<int>(s.motorValues[2]),
            "motor_4_us"_a = static_cast<int>(s.motorValues[3]),
            "motor_5_us"_a = static_cast<int>(s.motorValues[4]),
            "motor_6_us"_a = static_cast<int>(s.motorValues[5]),
            "motor_7_us"_a = static_cast<int>(s.motorValues[6]),
            "motor_8_us"_a = static_cast<int>(s.motorValues[7]),

            // ── RC channels — MSP_RC (105) ────────────────────────────────
            "rc_channels"_a = rc_ch,
            "rc_channel_count"_a = s.rcChannelCount,
            "rc_roll"_a = static_cast<int>(rc(0)),
            "rc_pitch"_a = static_cast<int>(rc(1)),
            "rc_throttle"_a = static_cast<int>(rc(2)),
            "rc_yaw"_a = static_cast<int>(rc(3)),
            "rc_arm"_a = static_cast<int>(rc(4)),
            "rc_aux1"_a = static_cast<int>(rc(5)),
            "rc_aux2"_a = static_cast<int>(rc(6)),
            "rc_aux3"_a = static_cast<int>(rc(7)),

            // ── RC link quality ───────────────────────────────────────────
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
            "gps_ground_speed_cms"_a = static_cast<int>(s.gps.groundSpeedMs),
            "gps_ground_course"_a = static_cast<int>(s.gps.groundCourse),
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

        // ── Telemetry source + what this source can provide ──────────────
        // Widgets for data the ELRS path cannot carry should check these
        // and show "USB only" instead of silently displaying zeros.
        const bool usb = s.linkSource != "ELRS";
        const RadioLinkStats& r = s.radio;
        d["link_source"] = s.linkSource;
        d["available_raw_imu"] = usb;
        d["available_raw_mag"] = usb;
        d["available_motors"] = usb;
        d["available_rc_channels"] = usb;
        d["available_sat_list"] = usb;
        d["available_hdop"] = usb;
        d["available_arming_flags"] = usb;
        d["available_cpu_load"] = usb;

        // ── ELRS link (CrsfLink) ──────────────────────────────────────────
        d["radio_status"] = r.statusStr;
        d["radio_status_int"] = static_cast<int>(r.status);
        d["radio_port"] = r.portName;
        d["elrs_link_stats_valid"] = r.linkStatsValid;
        d["elrs_uplink_lq"] = static_cast<int>(r.uplinkLq);
        d["elrs_uplink_rssi1_dbm"] = r.uplinkRssi1Dbm;
        d["elrs_uplink_rssi2_dbm"] = r.uplinkRssi2Dbm;
        d["elrs_uplink_snr"] = static_cast<int>(r.uplinkSnr);
        d["elrs_active_antenna"] = static_cast<int>(r.activeAntenna);
        d["elrs_rf_mode_index"] = static_cast<int>(r.rfModeIndex);
        d["elrs_tx_power_mw"] = static_cast<int>(r.txPowerMw);
        d["elrs_downlink_rssi_dbm"] = r.downlinkRssiDbm;
        d["elrs_downlink_lq"] = static_cast<int>(r.downlinkLq);
        d["elrs_downlink_snr"] = static_cast<int>(r.downlinkSnr);

        // Data ages (ms, -1 = never received) — grey out stale instruments
        d["age_last_frame_ms"] = r.msSinceLastFrame;
        d["age_attitude_ms"] = r.attitudeAgeMs;
        d["age_gps_ms"] = r.gpsAgeMs;
        d["age_battery_ms"] = r.batteryAgeMs;
        d["age_flight_mode_ms"] = r.flightModeAgeMs;
        d["age_baro_ms"] = r.baroAgeMs;
        d["age_link_stats_ms"] = r.linkStatsAgeMs;

        d["rate_attitude_hz"] = r.attitudeHz;
        d["rate_gps_hz"] = r.gpsHz;
        d["rate_battery_hz"] = r.batteryHz;
        d["rate_flight_mode_hz"] = r.flightModeHz;
        d["rate_baro_hz"] = r.baroHz;
        d["rate_vario_hz"] = r.varioHz;
        d["rate_link_stats_hz"] = r.linkStatsHz;
        d["rate_total_hz"] = r.totalFrameHz;

        d["radio_frames_total"] = r.framesTotal;
        d["radio_crc_errors"] = r.crcErrors;
        d["radio_reconnect_count"] = r.reconnectCount;
        d["radio_link_lost_count"] = r.linkLostCount;

        d["raw_flight_mode"] = r.rawFlightMode;
        d["arming_blocked"] = r.armingBlocked;
        d["gps_waiting"] = r.gpsWaiting;
        d["home_set"] = r.homeSet;
        d["home_lat"] = r.homeLat;
        d["home_lon"] = r.homeLon;
        d["altitude_source"] = r.altitudeSource;
        return d;
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
        .def("set_fail_injection", &DroneLink::setFailInjection,
            py::arg("active"),
            "Inject simulated link failure: sendMSP() returns {} immediately\n"
            "on every call when active=True, regardless of FC responsiveness.\n"
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
    // CaptureDeviceInfo
    // =========================================================================
    py::class_<CaptureDeviceInfo>(m, "CaptureDeviceInfo")
        .def_readonly("index", &CaptureDeviceInfo::index)
        .def_readonly("name", &CaptureDeviceInfo::name);

    // =========================================================================
    // VideoLink OSD overlay types
    //
    // Registered ahead of VideoLink itself so its methods (which take/
    // return these types) have something to resolve against. See
    // VideoLink.h's "Software OSD overlay" section for the full picture --
    // this replaces the Betaflight-side OSD, drawn entirely in the render
    // path and never touching the frames DetectionLink sees.
    // =========================================================================
    py::enum_<VideoLink::OsdAnchor>(m, "OsdAnchor")
        .value("TOP_LEFT", VideoLink::OsdAnchor::TopLeft)
        .value("TOP_CENTER", VideoLink::OsdAnchor::TopCenter)
        .value("TOP_RIGHT", VideoLink::OsdAnchor::TopRight)
        .value("MIDDLE_LEFT", VideoLink::OsdAnchor::MiddleLeft)
        .value("CENTER", VideoLink::OsdAnchor::Center)
        .value("MIDDLE_RIGHT", VideoLink::OsdAnchor::MiddleRight)
        .value("BOTTOM_LEFT", VideoLink::OsdAnchor::BottomLeft)
        .value("BOTTOM_CENTER", VideoLink::OsdAnchor::BottomCenter)
        .value("BOTTOM_RIGHT", VideoLink::OsdAnchor::BottomRight)
        .value("CUSTOM", VideoLink::OsdAnchor::Custom);

    py::class_<VideoLink::OsdElementLayout>(m, "OsdElementLayout")
        .def_readonly("enabled", &VideoLink::OsdElementLayout::enabled)
        .def_readonly("anchor", &VideoLink::OsdElementLayout::anchor)
        .def_readonly("margin_x", &VideoLink::OsdElementLayout::marginX)
        .def_readonly("margin_y", &VideoLink::OsdElementLayout::marginY)
        .def_readonly("custom_fx", &VideoLink::OsdElementLayout::customFx,
            "Only meaningful when anchor == CUSTOM.")
        .def_readonly("custom_fy", &VideoLink::OsdElementLayout::customFy,
            "Only meaningful when anchor == CUSTOM.");

    // =========================================================================
    // VideoLink
    //
    // Independent of DroneLink -- the FPV analog capture dongle is a
    // separate USB device from the flight controller's serial link, so it
    // gets its own class, its own thread, and its own connect/disconnect
    // lifecycle.
    //
    // Two independent output paths:
    //   1. get_latest_frame() -- frames cross the pybind11 boundary as
    //      (H,W,3) uint8 BGR numpy arrays. Used for anything that needs
    //      pixel data in Python (e.g. post-flight recording / YOLO
    //      post-processing).
    //   2. attach_to_window()/resize_window()/detach_window() -- native
    //      GDI rendering straight into a Win32 child window (typically a
    //      Tk widget's HWND via winfo_id()). This is the path the live
    //      FPV display should use: frames are painted by the capture
    //      thread itself and never cross into Python at all, eliminating
    //      the numpy/PIL/Tk PhotoImage overhead that was the source of
    //      the extra latency versus OBS.
    //
    // raise_window() / lower_window() / show_window() / hide_window() --
    // ADDED. These were previously missing from the bindings even though
    // main.py's DroneCockpitApp._on_panel_zorder/_on_panel_visibility
    // already called them -- every call was throwing AttributeError
    // ("'DroneBackend.VideoLink' object has no attribute 'raise_window'"),
    // which was silently swallowed by a try/except and printed to the
    // console. Because of that, the native render window's OS-level
    // Z-order/visibility was never actually being kept in sync with the
    // Tk panels around it, which is also why the live video appeared to
    // bleed over/"spill" onto unrelated panels whenever a panel was
    // dragged, resized, or brought to front -- both symptoms share this
    // one root cause.
    // =========================================================================
    py::class_<VideoLink>(m, "VideoLink")
        .def(py::init<>())
        .def_static("enumerate_devices", &VideoLink::enumerateDevices,
            "List available video capture devices (index + friendly name).")
        .def("connect_auto", &VideoLink::connectAuto,
            "Auto-detect the FPV capture dongle, skipping devices that look "
            "like the laptop's built-in webcam. Blocking -- call from a "
            "background thread on the Python side.")
        .def("connect", &VideoLink::connect, py::arg("device_index"),
            "Open a specific capture device by index (see enumerate_devices).")
        .def("disconnect", &VideoLink::disconnect,
            "Stop the capture thread and release the device.")
        .def("is_connected", &VideoLink::isConnected)
        .def("get_latest_frame", [](VideoLink& v) { return matToNumpy(v.getLatestFrame()); },
            "Latest frame as an (H, W, 3) uint8 BGR numpy array. Empty "
            "array if nothing has been captured yet. Not used by the live "
            "display path -- see attach_to_window().")
        .def("get_frame_count", &VideoLink::getFrameCount)
        .def("get_measured_fps", &VideoLink::getMeasuredFps,
            "Capture-thread FPS, measured over a rolling ~1s window.")
        .def("get_device_name", &VideoLink::getDeviceName)
        .def("set_preferred_resolution", &VideoLink::setPreferredResolution,
            py::arg("width"), py::arg("height"),
            "Must be called before connect()/connect_auto() to take effect.")
        .def("attach_to_window", &VideoLink::attachToWindow,
            py::arg("parent_hwnd"), py::arg("x"), py::arg("y"),
            py::arg("w"), py::arg("h"),
            "Reparent a native GDI render window under parent_hwnd (a Tk "
            "widget's winfo_id()) and start painting captured frames "
            "directly into it, bypassing Python entirely for display. "
            "Safe to call again to move/recreate the render window; any "
            "previously attached window is destroyed first.")
        .def("resize_window", &VideoLink::resizeWindow,
            py::arg("w"), py::arg("h"),
            "Resize the attached render window. Call on Tk <Configure>.")
        .def("detach_window", &VideoLink::detachWindow,
            "Destroy the attached render window. Safe to call even if none "
            "is currently attached.")
        .def("is_window_attached", &VideoLink::isWindowAttached)
        .def("raise_window", &VideoLink::raiseWindow,
            "Bring the attached native render window to the top of the "
            "OS Z-order among its siblings (SetWindowPos w/ HWND_TOP). "
            "Called whenever the FPV Tk panel is brought to front.")
        .def("lower_window", &VideoLink::lowerWindow,
            "Send the attached native render window to the bottom of the "
            "OS Z-order among its siblings (SetWindowPos w/ HWND_BOTTOM). "
            "Called whenever a different Tk panel is brought to front, so "
            "the video never sits above panels the user just raised.")
        .def("show_window", &VideoLink::showWindow,
            "Show the attached native render window (ShowWindow SW_SHOW). "
            "Called when the FPV Tk panel is toggled visible.")
        .def("hide_window", &VideoLink::hideWindow,
            "Hide the attached native render window (ShowWindow SW_HIDE). "
            "Called when the FPV Tk panel is toggled hidden -- without "
            "this, the native window kept rendering even while its Tk "
            "panel was hidden, since hiding a Tk canvas item has no "
            "effect on a foreign HWND.")

        // ── Software OSD overlay ─────────────────────────────────────
        .def("set_osd_overlay_enabled", &VideoLink::setOsdOverlayEnabled,
            py::arg("enabled"),
            "Master on/off for the whole overlay -- does not forget "
            "individual elements' own enabled flags.")
        .def("is_osd_overlay_enabled", &VideoLink::isOsdOverlayEnabled)
        .def("set_osd_locked", &VideoLink::setOsdLocked, py::arg("locked"),
            "When true, OsdDragOverlay (Python side) must refuse to start "
            "a drag -- checked there, not enforced here, since dragging "
            "itself is pure Tk/mouse handling that never calls into "
            "VideoLink until the drag is released.")
        .def("is_osd_locked", &VideoLink::isOsdLocked)
        .def("get_osd_element_ids", &VideoLink::getOsdElementIds,
            "Every id the overlay knows how to draw, e.g. "
            "['altitude', 'horizon', 'compass'].")
        .def("set_osd_element_enabled", &VideoLink::setOsdElementEnabled,
            py::arg("id"), py::arg("enabled"))
        .def("set_osd_element_anchor", &VideoLink::setOsdElementAnchor,
            py::arg("id"), py::arg("anchor"), py::arg("margin_x") = 16, py::arg("margin_y") = 16,
            "Snap an element to one of the 9 preset positions. Switches "
            "the element out of CUSTOM (free-placed) mode if it was in it.")
        .def("set_osd_element_custom_position", &VideoLink::setOsdElementCustomPosition,
            py::arg("id"), py::arg("fx"), py::arg("fy"),
            "Free-place an element with its center at fractional panel "
            "position (fx, fy), each in [0, 1]. Puts the element in "
            "CUSTOM anchor mode. Called on drag-release by OsdDragOverlay "
            "when the drag did NOT snap to a preset -- see "
            "osd_overlay_controls.py.")
        .def("get_osd_element_layout", &VideoLink::getOsdElementLayout,
            py::arg("id"),
            "Current layout for one element, for the settings panel to "
            "read back on open and for save/load. Persistence itself is "
            "done in Python, not here.")
        .def("set_telemetry_provider", &VideoLink::setTelemetryProvider,
            py::arg("provider"),
            "Python callable, no args, returning a TelemetrySnapshot -- "
            "the SAME callable already passed to "
            "DetectionLink.set_telemetry_provider() should be reused "
            "here (e.g. DroneCockpitApp._get_detection_telemetry()). "
            "Called once per painted frame, on the capture thread, "
            "immediately before OSD compositing.")
        .def("reset_flight_timer", &VideoLink::resetFlightTimer,
            "Reset the OSD flight timer back to 0:00. Safe to call from "
            "any thread (e.g. a Tk button on the UI thread) -- the "
            "reset is applied on the capture thread on the next painted "
            "frame. If the drone is still armed when this is called, "
            "the timer keeps running from zero rather than stopping; "
            "only a disarm ever pauses it.");

    // =========================================================================
    // TelemetrySnapshot
    //
    // Constructible from Python (py::init<>() + all fields readwrite) so a
    // pure-Python trampoline can build one and hand it back through
    // DetectionLink.set_telemetry_provider() -- see that binding below and
    // DroneCockpitApp._get_detection_telemetry() in main.py. Field names
    // are the snake_case equivalent of the camelCase C++ members in
    // DetectionLink.h; see that header's angle-convention comment before
    // wiring headingDeg/rollDeg/pitchDeg/gimbalPanDeg/gimbalTiltDeg from a
    // new telemetry source.
    // =========================================================================
    py::class_<TelemetrySnapshot>(m, "TelemetrySnapshot")
        .def(py::init<>())
        .def_readwrite("valid", &TelemetrySnapshot::valid,
            "Must be set True for DetectionLink::georeference() to run at "
            "all -- leave False (the default) whenever GPS/attitude aren't "
            "trustworthy yet.")
        .def_readwrite("latitude", &TelemetrySnapshot::latitude)
        .def_readwrite("longitude", &TelemetrySnapshot::longitude)
        .def_readwrite("altitude_m", &TelemetrySnapshot::altitudeM,
            "Height above the ground directly below (AGL), metres.")
        .def_readwrite("heading_deg", &TelemetrySnapshot::headingDeg,
            "Compass heading, 0 = north, increasing clockwise.")
        .def_readwrite("roll_deg", &TelemetrySnapshot::rollDeg,
            "Positive = right wing down.")
        .def_readwrite("pitch_deg", &TelemetrySnapshot::pitchDeg,
            "Positive = nose up.")
        .def_readwrite("gimbal_pan_deg", &TelemetrySnapshot::gimbalPanDeg,
            "Relative to body forward, positive = pan right.")
        .def_readwrite("gimbal_tilt_deg", &TelemetrySnapshot::gimbalTiltDeg,
            "0 = forward/level, +90 = straight down.")
        .def_readwrite("battery_voltage", &TelemetrySnapshot::batteryVoltage,
            "Volts. For the OSD battery readout -- same source as "
            "DroneState.battery_voltage.")
        .def_readwrite("battery_percentage", &TelemetrySnapshot::batteryPercentage,
            "0-100, same source as DroneState.battery_percentage.")
        .def_readwrite("rssi", &TelemetrySnapshot::rssi,
            "0-255 raw link quality, same source as DroneState.rssi.")
        .def_readwrite("gps_fix_type", &TelemetrySnapshot::gpsFixType,
            "0 = no fix, 1 = 2D, 2 = 3D -- same source as GPSReading.fix_type.")
        .def_readwrite("gps_num_sat", &TelemetrySnapshot::gpsNumSat,
            "Satellites used in the solution, same source as GPSReading.num_sat.")
        .def_readwrite("home_distance_m", &TelemetrySnapshot::homeDistanceM,
            "Metres, same source as GPSReading.dist_to_home_m.")
        .def_readwrite("ground_speed_ms", &TelemetrySnapshot::groundSpeedMs,
            "Metres/second, derived from GPSReading.ground_speed_cms / 100.0.")
        .def_readwrite("armed", &TelemetrySnapshot::armed,
            "Same source as DroneState.armed. Drives the OSD flight "
            "timer's start/pause edge-detection in VideoLink::drawOsdTimer.");

    // =========================================================================
    // DetectionRecord
    //
    // Read-only on the Python side -- these are produced by DetectionLink's
    // own worker thread, never constructed in Python. Consumed directly by
    // DetectionMapWidget (see rec.class_name, rec.latitude/longitude,
    // rec.telemetry.heading_deg, etc. in that module).
    // =========================================================================
    py::class_<DetectionRecord>(m, "DetectionRecord")
        .def_readonly("id", &DetectionRecord::id)
        .def_readonly("timestamp_ms", &DetectionRecord::timestampMs,
            "Wall-clock ms (epoch) when the source frame was grabbed.")
        .def_readonly("class_name", &DetectionRecord::className)
        .def_readonly("confidence", &DetectionRecord::confidence)
        .def_readonly("track_id", &DetectionRecord::trackId,
            "Identity assigned by DetectionLink's IoU-based tracker, NOT "
            "a fresh id per pass -- every record sharing the same "
            "track_id is (as far as position-based tracking can tell) "
            "the same physical object at a different moment. A given "
            "track only gets a new record when it's first seen, once it "
            "has moved far enough on the map, or periodically if it's "
            "been static a while (see set_track_move_threshold_m / "
            "set_track_refresh_interval_ms). NOT appearance-based re-ID: "
            "an object that leaves frame and comes back gets a new "
            "track_id.")
        .def_readonly("bbox_x", &DetectionRecord::bboxX)
        .def_readonly("bbox_y", &DetectionRecord::bboxY)
        .def_readonly("bbox_w", &DetectionRecord::bboxW)
        .def_readonly("bbox_h", &DetectionRecord::bboxH)
        .def_readonly("latitude", &DetectionRecord::latitude)
        .def_readonly("longitude", &DetectionRecord::longitude)
        .def_readonly("georeferenced", &DetectionRecord::georeferenced,
            "False if telemetry wasn't valid for this pass -- lat/lon are "
            "meaningless when this is False.")
        .def_readonly("range_method", &DetectionRecord::rangeMethod,
            "'ground_plane' or 'object_size' -- which ranging method "
            "produced latitude/longitude/distance_m/bearing_deg. Empty "
            "string when georeferenced is False.")
        .def_readonly("distance_m", &DetectionRecord::distanceM,
            "Estimated distance from the drone to the object, metres. "
            "0 when georeferenced is False.")
        .def_readonly("bearing_deg", &DetectionRecord::bearingDeg,
            "Compass bearing from the drone to the object at detection "
            "time, 0-360. 0 (== due north) when georeferenced is False -- "
            "always check georeferenced first, don't treat 0 as a real "
            "reading.")
        .def_readonly("screenshot_path", &DetectionRecord::screenshotPath,
            "Empty string if screenshot saving is disabled or failed.")
        .def_readonly("telemetry", &DetectionRecord::telemetry,
            "TelemetrySnapshot stored verbatim at detection time, for "
            "later re-derivation/debugging.");

    // =========================================================================
    // DetectionLink
    //
    // Runs YOLO inference on its own C++ thread, fully independent of both
    // the 100 Hz MSP telemetry loop and the FPV capture/paint thread -- same
    // "own thread, own lifecycle" pattern as VideoLink. Frame source and
    // telemetry source are both wired in from Python, decoupling
    // DetectionLink.h/.cpp from ever needing to know about VideoLink or
    // DroneLink directly (see DetectionLink.h's module comment).
    //
    // set_telemetry_provider() takes a plain Python callable returning a
    // TelemetrySnapshot -- pybind11's std::function support (functional.h,
    // included above) wraps it and acquires the GIL automatically each time
    // DetectionLink's worker thread invokes it, so the bound Python
    // function is safe to call from that non-Python-owned thread without
    // any extra locking on the Python side. See
    // DroneCockpitApp._get_detection_telemetry() in main.py for the
    // trampoline that's actually passed in.
    // =========================================================================
    py::class_<DetectionLink>(m, "DetectionLink")
        .def(py::init<>())
        .def("set_video_link_source", &DetectionLink::setVideoLinkSource,
            py::arg("video_link"),
            "Preferred frame source: every detection pass pulls a frame "
            "via VideoLink::getLatestFrame() directly in C++. DetectionLink "
            "does not take ownership and does not outlive the caller's "
            "responsibility to call stop() before the VideoLink instance "
            "is destroyed.")
        .def("set_telemetry_provider", &DetectionLink::setTelemetryProvider,
            py::arg("provider"),
            "Python callable, no args, returning a TelemetrySnapshot. "
            "Called once per detection pass, immediately before inference.")
        .def("set_model_path", &DetectionLink::setModelPath,
            py::arg("path"),
            "Path to an ONNX object-detection model (Ultralytics "
            "end-to-end/NMS-baked export layout, e.g. YOLO26 -- output "
            "shape [1, maxDetections, 6]). A sibling '<stem>.names' file "
            "is loaded automatically if present.")
        .def("set_screenshot_dir", &DetectionLink::setScreenshotDir,
            py::arg("dir"),
            "Directory detection screenshots are written to (created if "
            "missing). Leave empty to disable screenshot saving.")
        .def("set_horizontal_fov_deg", &DetectionLink::setHorizontalFovDeg,
            py::arg("fov_deg"),
            "Horizontal FOV of the camera feeding this link, in degrees -- "
            "needed to turn a pixel offset into a georeferencing ray angle.")
        .def("set_detection_interval_ms", &DetectionLink::setDetectionIntervalMs,
            py::arg("ms"),
            "How often a detection pass runs, independent of the 30fps "
            "capture loop. Safe to change at runtime.")
        .def("set_confidence_threshold", &DetectionLink::setConfidenceThreshold,
            py::arg("threshold"),
            "Raw model confidence below which a detection is discarded.")
        .def("set_input_size", &DetectionLink::setInputSize,
            py::arg("size"),
            "Square side (pixels) the ONNX model expects, letterboxed. "
            "MUST match the imgsz the .onnx was exported at (see "
            "export_model.py) or every box -- and therefore every "
            "georeferenced position -- comes out scaled wrong with no "
            "error or crash to flag it. Does not need to match training "
            "imgsz, only the export.")
        .def("set_use_cuda", &DetectionLink::setUseCuda,
            py::arg("enabled"),
            "Opt into CUDA inference. Only takes effect if this OpenCV "
            "build actually has CUDA/cuDNN support compiled in (the "
            "stock pip wheel does not) -- start() proves the CUDA path "
            "with a dummy forward pass and silently falls back to CPU "
            "if it fails. Check is_using_cuda() after start() to see "
            "which one you actually got.")
        .def("is_using_cuda", &DetectionLink::isUsingCuda,
            "True only if set_use_cuda(True) was called AND start() "
            "proved the CUDA backend actually works on this machine/build.")
        .def("set_known_object_width", &DetectionLink::setKnownObjectWidth,
            py::arg("class_name"), py::arg("width_m"),
            "Real-world width in metres for one class, measured face-on "
            "(e.g. a person's shoulder width, a car's width -- not "
            "length). Enables object-size-based ranging for that class; "
            "class_name must match a name from the model's .names file "
            "exactly. Safe to call at runtime, including while running.")
        .def("clear_known_object_widths", &DetectionLink::clearKnownObjectWidths,
            "Removes every configured class width -- object-size ranging "
            "then falls back to unavailable for all classes until "
            "set_known_object_width() is called again.")
        .def("set_min_ground_ray_component", &DetectionLink::setMinGroundRayComponent,
            py::arg("v"),
            "Ray-downward-component threshold (0-1, default 0.12) below "
            "which ground-plane ranging is skipped in favor of "
            "triangulated/object-size ranging (when available) -- see "
            "the header comment on rangeByGroundPlane() for why a "
            "near-horizontal ray makes ground-plane intersection "
            "unreliable regardless of how accurate the altitude reading is.")
        .def("set_triangulation_min_baseline_m", &DetectionLink::setTriangulationMinBaselineM,
            py::arg("m"),
            "Minimum straight-line distance (metres, default 5.0) the "
            "drone must have moved between the most different pair of a "
            "track's bearing observations before a triangulated fix is "
            "trusted for it -- see rangeByTriangulation()'s header "
            "comment for why this and the bearing-spread threshold below "
            "are both needed to catch a degenerate (no-parallax) case.")
        .def("set_triangulation_min_bearing_spread_deg", &DetectionLink::setTriangulationMinBearingSpreadDeg,
            py::arg("deg"),
            "Minimum angular spread (degrees, default 5.0) between the "
            "most different pair of bearing directions used for a "
            "triangulated fix -- catches the case where the drone moved "
            "plenty of metres but essentially straight at (or past, at "
            "constant bearing) the object, which gives a large baseline "
            "but almost no actual parallax to triangulate with.")
        .def("set_track_iou_threshold", &DetectionLink::setTrackIouThreshold,
            py::arg("iou"),
            "Minimum IoU (0-1, default 0.3) between a raw box and a "
            "track's last matched box, same class, to count as the same "
            "object across passes. Lower catches faster-moving objects "
            "at the cost of more false merges between nearby same-class "
            "objects; raise it if two objects passing near each other "
            "get incorrectly tracked as one.")
        .def("set_track_max_missed_passes", &DetectionLink::setTrackMaxMissedPasses,
            py::arg("passes"),
            "How many consecutive passes a track may go unmatched "
            "(occlusion, a missed detection) before it's dropped. "
            "Default 6 (~1.5s at the default 250ms interval). Once "
            "dropped, the object reappearing starts a brand new "
            "track_id -- this tracker is position/IoU-based, not "
            "appearance re-ID.")
        .def("set_track_move_threshold_m", &DetectionLink::setTrackMoveThresholdM,
            py::arg("meters"),
            "Minimum ground movement, in meters, between a track's last "
            "recorded position and its current one before a new "
            "DetectionRecord is written for it. Default 3.0. Only "
            "compared when both positions are georeferenced.")
        .def("set_track_refresh_interval_ms", &DetectionLink::setTrackRefreshIntervalMs,
            py::arg("ms"),
            "Even a perfectly static, continuously-tracked object gets a "
            "fresh DetectionRecord (and screenshot) at least this often "
            "so it doesn't go stale in 'what's new' views. Default 10000 "
            "(10s).")
        .def("start", &DetectionLink::start,
            "Loads the model and starts the worker thread. Returns False "
            "(and does not start the thread) if the model failed to load "
            "or no frame source has been set.")
        .def("stop", &DetectionLink::stop)
        .def("is_running", &DetectionLink::isRunning)
        .def("get_detection_count", &DetectionLink::getDetectionCount)
        .def("get_last_pass_duration_ms", &DetectionLink::getLastPassDurationMs)
        .def("get_last_pass_timestamp_ms", &DetectionLink::getLastPassTimestampMs)
        .def("get_all_records", &DetectionLink::getAllRecords,
            "Full copy of every record collected since start() (or since "
            "clear_records()). Safe to call at UI refresh rate, not meant "
            "to be called every frame.")
        .def("get_records_since", &DetectionLink::getRecordsSince,
            py::arg("since_id"),
            "Only records with id > since_id, so a poller can pull "
            "incrementally instead of re-fetching the whole list every "
            "tick. Pass 0 to get everything.")
        .def("clear_records", &DetectionLink::clearRecords)
        .def("get_latest_annotated_frame_jpeg",
            [](const DetectionLink& self) -> py::bytes {
                auto jpeg = self.getLatestAnnotatedFrameJpeg();
                if (jpeg.empty())
                    return py::bytes();
                return py::bytes(reinterpret_cast<const char*>(jpeg.data()), jpeg.size());
            },
            "JPEG bytes of the most recent frame this link processed, "
            "with that pass's detection boxes drawn on it. Empty bytes "
            "if no pass has completed yet. A deliberate, narrow exception "
            "to 'pixel data never crosses into Python' -- meant to feed "
            "an OPTIONAL, low-rate (poll at ~1-3Hz, not per-tick) 'live "
            "detections' preview pane, never the pilot's primary FPV "
            "path (that stays on VideoLink.attach_to_window() exactly "
            "as before). Costs one JPEG encode per call.");

    // =========================================================================
    // Free functions
    // =========================================================================
    // =========================================================================
    // CrsfLink — ELRS telemetry via RadioMaster Pocket (USB-VCP Telem Mirror)
    //
    // Blocking calls release the GIL so the Tk UI keeps running.
    // =========================================================================
    py::class_<CrsfLink>(m, "CrsfLink")
        .def(py::init<>())
        .def("start_auto", &CrsfLink::startAuto, py::call_guard<py::gil_scoped_release>(),
            "Non-blocking. Background thread finds the Pocket, connects, and "
            "reconnects automatically after unplug/replug. Preferred entry point.")
        .def("connect_auto", &CrsfLink::connectAuto, py::arg("timeout_ms") = 3000,
            py::call_guard<py::gil_scoped_release>(),
            "Blocking scan (up to timeout_ms). On success runs like start_auto(). "
            "Call from a background thread, like VideoLink.connect_auto().")
        .def("connect", &CrsfLink::connect, py::arg("port_name"),
            py::call_guard<py::gil_scoped_release>(),
            "Open a specific COM port; re-opens that same port after an unplug.")
        .def("disconnect", &CrsfLink::disconnect, py::call_guard<py::gil_scoped_release>())
        .def("is_connected", &CrsfLink::isConnected,
            "True while the Pocket's COM port is open (says nothing about the drone — "
            "use get_status() for that).")
        .def("is_running", &CrsfLink::isRunning)
        .def("get_latest_state", &CrsfLink::getLatestState,
            "Thread-safe DroneState snapshot with link_source == 'ELRS'.")
        .def("get_status", &CrsfLink::getStatus)
        .def("get_port_name", &CrsfLink::getPortName)
        .def("get_last_scan_report", &CrsfLink::getLastScanReport,
            "Text log of the last port scan — which ports were tried and why they "
            "were skipped/selected.")
        .def("set_excluded_ports", &CrsfLink::setExcludedPorts, py::arg("ports"),
            "Ports the scanner must never open, e.g. [drone_link_port].")
        .def("set_port_name_hints", &CrsfLink::setPortNameHints, py::arg("hints"),
            "Case-insensitive fragments of the Pocket's USB name "
            "(default: edgetx, opentx, radiomaster, pocket).")
        .def("set_scan_interval_ms", &CrsfLink::setScanIntervalMs, py::arg("ms"))
        .def("set_probe_ms", &CrsfLink::setProbeMs, py::arg("ms"))
        .def("set_assert_dtr", &CrsfLink::setAssertDtr, py::arg("on"))
        .def("set_lost_timeout_ms", &CrsfLink::setLostTimeoutMs, py::arg("ms"),
            "No drone telemetry for this long -> TELEMETRY_LOST (default 1500).")
        .def("set_degraded_lq", &CrsfLink::setDegradedLq, py::arg("lq_percent"),
            "Uplink LQ below this -> DEGRADED (default 70).")
        .def("set_instrument_stale_ms", &CrsfLink::setInstrumentStaleMs, py::arg("ms"),
            "Attitude older than this -> DEGRADED (default 1500).")
        .def("set_yaw_signed", &CrsfLink::setYawSigned, py::arg("signed"),
            "Yaw wire encoding; decide with bench test IT-ELRS-004 (default True).")
        .def("set_cell_count", &CrsfLink::setCellCount, py::arg("cells"),
            "0 = auto from first voltage. Set 4 for the 4S pack on real missions.")
        .def("set_cell_voltage_thresholds", &CrsfLink::setCellVoltageThresholds,
            py::arg("warning_v"), py::arg("critical_v"))
        .def("set_battery_capacity_mah", &CrsfLink::setBatteryCapacityMah, py::arg("mah"))
        .def("set_min_sats_for_fix", &CrsfLink::setMinSatsForFix, py::arg("sats"))
        .def("set_gps_altitude_fallback", &CrsfLink::setGpsAltitudeFallback, py::arg("on"))
        .def_static("enumerate_ports", &CrsfLink::enumeratePorts);

    m.def("enumerate_serial_ports", &EnumerateSerialPorts,
        "All COM ports with USB product names (tells the FC and the Pocket apart).");
    m.def("auto_detect_fc", &AutoDetectFlightController,
        py::arg("exclude_ports") = std::vector<std::string>{}, py::arg("timeout_ms") = 300,
        py::call_guard<py::gil_scoped_release>(),
        "Returns the first port that answers an MSP request, or 'NOT_FOUND'. "
        "Never picks the radio. Use instead of auto_detect_f405() now that the "
        "Pocket is a second STM32 serial device on the laptop.");

    m.def("auto_detect_f405", &AutoDetectF405,
        "Scan COM1-COM29 and return the first port that opens, or 'NOT_FOUND'.");
}