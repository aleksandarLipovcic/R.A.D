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
    //
    // Must be registered BEFORE DroneState because DroneState holds one by
    // value. Gate all display/navigation code on raw_valid / comp_valid.
    // =========================================================================
    py::class_<GPSReading>(m, "GPSReading")
        // -- From MSP_RAW_GPS (106) -------------------------------------------
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
        // -- From MSP_COMP_GPS (107) ------------------------------------------
        .def_readonly("dist_to_home_m", &GPSReading::distToHomM,
            "Distance to home point in metres. Valid after arming with 3D fix.")
        .def_readonly("bearing_to_home", &GPSReading::bearingToHome,
            "Bearing to home in degrees (-180 to +180).")
        .def_readonly("gps_heartbeat", &GPSReading::gpsHeartbeat,
            "Toggles 0<->1 on each fresh GPS frame. XOR with prev to detect new data.")
        // -- Validity flags ---------------------------------------------------
        .def_readonly("raw_valid", &GPSReading::rawValid,
            "True after first successful MSP_RAW_GPS parse.")
        .def_readonly("comp_valid", &GPSReading::compValid,
            "True after first successful MSP_COMP_GPS parse.")
        .def_readonly("position_usable", &GPSReading::positionUsable,
            "True when fixType>=2, numSat>=4, and HDOP<5.0. "
            "Always gate coordinate display on this flag, not raw_valid alone.");

    // =========================================================================
    // DroneState
    // =========================================================================
    py::class_<DroneState>(m, "DroneState")

        // IMU -- raw ADC counts (MPU-6500)
        .def_readonly("ax", &DroneState::ax)
        .def_readonly("ay", &DroneState::ay)
        .def_readonly("az", &DroneState::az)
        .def_readonly("gx", &DroneState::gx)
        .def_readonly("gy", &DroneState::gy)
        .def_readonly("gz", &DroneState::gz)

        // Attitude -- degrees x10 for roll/pitch; full degrees for yaw
        .def_readonly("roll", &DroneState::roll)
        .def_readonly("pitch", &DroneState::pitch)
        .def_readonly("yaw", &DroneState::yaw)

        // Power
        .def_readonly("battery_voltage", &DroneState::batteryVoltage)
        .def_readonly("rssi", &DroneState::rssi)

        // Barometer (BMP280 via MSP_ALTITUDE)
        .def_readonly("baro_altitude_cm", &DroneState::baroAltitudeCm,
            "FC-fused BMP280 altitude above home point, cm.")
        .def_readonly("baro_vario_cm_per_sec", &DroneState::baroVarioCmPerSec,
            "Vertical speed cm/s. Positive = climbing.")
        .def_readonly("baro_valid", &DroneState::baroValid,
            "True when a valid MSP_ALTITUDE frame was received.")

        // Magnetometer (QMC5883L via MSP_DEBUG + debug_mode=MAG_CALIB)
        .def_readonly("mag_x", &DroneState::magX,
            "Raw mag X ADC counts. Requires debug_mode=MAG_CALIB in BF CLI.")
        .def_readonly("mag_y", &DroneState::magY)
        .def_readonly("mag_z", &DroneState::magZ)
        .def_readonly("mag_heading_deg", &DroneState::magHeadingDeg,
            "Tilt-uncorrected 2D magnetic heading 0-360 deg.")
        .def_readonly("mag_valid", &DroneState::magValid,
            "True when mag X/Y/Z are non-zero.")

        // Magnetometer calibration state
        .def_readonly("mag_cal_active", &DroneState::magCalActive,
            "True while FC is in magnetometer calibration mode (30 s window).")
        .def_readonly("mag_cal_seconds_remaining", &DroneState::magCalSecondsRemaining,
            "Seconds remaining in mag calibration window (30 -> 0).")

        // Gyro/Accel calibration state
        .def_readonly("acc_cal_active", &DroneState::accCalActive,
            "True while FC is performing gyro/accel calibration (~5 s).")
        .def_readonly("acc_cal_seconds_remaining", &DroneState::accCalSecondsRemaining,
            "Seconds remaining in gyro/accel calibration window (5 -> 0).")

        // GPS -- NEO-M10 via MSP_RAW_GPS (106) + MSP_COMP_GPS (107)
        // Returns the full GPSReading sub-object.
        // Usage in Python:
        //   state = link.get_latest_state()
        //   if state.gps.position_usable:
        //       print(state.gps.latitude, state.gps.longitude)
        .def_readonly("gps", &DroneState::gps,
            "GPSReading from NEO-M10. "
            "Check gps.raw_valid before position fields, "
            "gps.comp_valid before home fields, "
            "gps.position_usable before any navigation use.")

        // Diagnostics
        .def_readonly("last_rtt_ms", &DroneState::lastRttMs)
        .def_readonly("fc_cycle_ms", &DroneState::fcCycleMs,
            "FC loop cycle time in ms from MSP_STATUS (101).")
        .def_readonly("link_healthy", &DroneState::linkHealthy)
        .def_readonly("packet_count", &DroneState::packetCount)

        // Convenience dict -- all values pre-converted to useful units.
        // GPS speed is exposed in m/s, km/h, and mph.
        // GPS altitude is exposed in metres and feet.
        // HDOP is the real value (divided by 100).
        .def("to_dict", [](const DroneState& s) {
        return py::dict(
            // IMU
            "ax"_a = s.ax, "ay"_a = s.ay, "az"_a = s.az,
            "gx"_a = s.gx, "gy"_a = s.gy, "gz"_a = s.gz,
            // Attitude (pre-divided)
            "roll_deg"_a = s.roll / 10.0f,
            "pitch_deg"_a = s.pitch / 10.0f,
            "yaw_deg"_a = static_cast<float>(s.yaw),
            // Power
            "battery_v"_a = s.batteryVoltage,
            "rssi"_a = s.rssi,
            // Barometer
            "baro_altitude_m"_a = s.baroAltitudeCm * 0.01,
            "baro_altitude_ft"_a = s.baroAltitudeCm * 0.0328084,
            "baro_vario_mps"_a = s.baroVarioCmPerSec * 0.01,
            "baro_vario_fpm"_a = s.baroVarioCmPerSec * 1.9685,
            "baro_valid"_a = s.baroValid,
            // Magnetometer
            "mag_x"_a = s.magX,
            "mag_y"_a = s.magY,
            "mag_z"_a = s.magZ,
            "mag_heading_deg"_a = s.magHeadingDeg,
            "mag_valid"_a = s.magValid,
            // Mag calibration
            "mag_cal_active"_a = s.magCalActive,
            "mag_cal_seconds_remaining"_a = s.magCalSecondsRemaining,
            // Acc/Gyro calibration
            "acc_cal_active"_a = s.accCalActive,
            "acc_cal_seconds_remaining"_a = s.accCalSecondsRemaining,
            // GPS -- MSP_RAW_GPS (106)
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
            // GPS -- MSP_COMP_GPS (107)
            "gps_dist_home_m"_a = static_cast<double>(s.gps.distToHomM),
            "gps_dist_home_ft"_a = s.gps.distToHomM * 3.28084,
            "gps_bearing_home"_a = static_cast<int>(s.gps.bearingToHome),
            "gps_heartbeat"_a = s.gps.gpsHeartbeat,
            "gps_comp_valid"_a = s.gps.compValid,
            // Diagnostics
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
        .def("start_mag_calibration", &DroneLink::startMagCalibration,
            "Send MSP_MAG_CALIBRATION (206) to the FC.\n"
            "FC enters calibration mode for 30 s. Rotate drone on all axes.\n"
            "Monitor state.mag_cal_active and state.mag_cal_seconds_remaining.")
        .def("start_acc_calibration", &DroneLink::startAccCalibration,
            "Send MSP_ACC_CALIBRATION (205) to the FC.\n"
            "Keep drone perfectly level and still for ~5 s.\n"
            "Monitor state.acc_cal_active and state.acc_cal_seconds_remaining.");

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