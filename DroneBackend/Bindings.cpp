#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DroneLink.h"
#include "IMUSensor.h"

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(DroneBackend, m) {
    m.doc() = "Project R.A.D — threaded drone telemetry backend (GCS edition)";

    // ── DroneState ────────────────────────────────────────────────────────────
    py::class_<DroneState>(m, "DroneState")

        // IMU — raw ADC counts (MPU-6500)
        .def_readonly("ax", &DroneState::ax)
        .def_readonly("ay", &DroneState::ay)
        .def_readonly("az", &DroneState::az)
        .def_readonly("gx", &DroneState::gx)
        .def_readonly("gy", &DroneState::gy)
        .def_readonly("gz", &DroneState::gz)

        // Attitude — degrees × 10 for roll/pitch; full degrees for yaw
        .def_readonly("roll", &DroneState::roll)
        .def_readonly("pitch", &DroneState::pitch)
        .def_readonly("yaw", &DroneState::yaw)

        // Power
        .def_readonly("battery_voltage", &DroneState::batteryVoltage)
        .def_readonly("rssi", &DroneState::rssi)

        // Barometer (BMP280 via MSP_ALTITUDE)
        .def_readonly("baro_altitude_cm",
            &DroneState::baroAltitudeCm,
            "FC-fused BMP280 altitude above home point, cm.")
        .def_readonly("baro_vario_cm_per_sec",
            &DroneState::baroVarioCmPerSec,
            "Vertical speed cm/s. Positive = climbing.")
        .def_readonly("baro_valid",
            &DroneState::baroValid,
            "True when a valid MSP_ALTITUDE frame was received.")

        // Magnetometer (QMC5883L via MSP_DEBUG + debug_mode=MAG_CALIB)
        .def_readonly("mag_x",
            &DroneState::magX,
            "Raw mag X, ADC counts. Requires debug_mode=MAG_CALIB in BF CLI.")
        .def_readonly("mag_y",
            &DroneState::magY,
            "Raw mag Y, ADC counts.")
        .def_readonly("mag_z",
            &DroneState::magZ,
            "Raw mag Z, ADC counts.")
        .def_readonly("mag_heading_deg",
            &DroneState::magHeadingDeg,
            "Tilt-uncorrected 2-D magnetic heading, 0-360°. "
            "Accurate only when drone is level.")
        .def_readonly("mag_valid",
            &DroneState::magValid,
            "True when mag X/Y/Z are non-zero. "
            "False means debug_mode≠MAG_CALIB or mag sensor not detected.")

        // Magnetometer calibration state
        .def_readonly("mag_cal_active",
            &DroneState::magCalActive,
            "True while FC is in magnetometer calibration mode (30 s window). "
            "Rotate drone on all axes during this window.")
        .def_readonly("mag_cal_seconds_remaining",
            &DroneState::magCalSecondsRemaining,
            "Seconds remaining in mag calibration window (30 → 0).")

        // Gyro/Accel calibration state
        .def_readonly("acc_cal_active",
            &DroneState::accCalActive,
            "True while FC is performing gyro/accel calibration (~5 s). "
            "Keep drone perfectly level and still during this window.")
        .def_readonly("acc_cal_seconds_remaining",
            &DroneState::accCalSecondsRemaining,
            "Seconds remaining in gyro/accel calibration window (5 → 0).")

        // Diagnostics
        .def_readonly("last_rtt_ms", &DroneState::lastRttMs)
        .def_readonly("fc_cycle_ms", &DroneState::fcCycleMs,
            "FC loop cycle time in ms, derived from MSP_STATUS (101).")
        .def_readonly("link_healthy", &DroneState::linkHealthy)
        .def_readonly("packet_count", &DroneState::packetCount)

        // Convenience dict — all values pre-converted to useful units
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
            // Diagnostics
            "rtt_ms"_a = s.lastRttMs,
            "fc_cycle_ms"_a = s.fcCycleMs,
            "link_healthy"_a = s.linkHealthy,
            "packet_count"_a = s.packetCount
        );
            });

    // ── DroneLink ─────────────────────────────────────────────────────────────
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
            "The FC enters calibration mode for 30 seconds.\n"
            "Rotate the drone on all axes during this window.\n"
            "Monitor state.mag_cal_active and state.mag_cal_seconds_remaining.")
        .def("start_acc_calibration", &DroneLink::startAccCalibration,
            "Send MSP_ACC_CALIBRATION (205) to the FC.\n"
            "Keep the drone perfectly level and still for ~5 seconds.\n"
            "Monitor state.acc_cal_active and state.acc_cal_seconds_remaining.");

    // ── IMUSensor ─────────────────────────────────────────────────────────────
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

    // ── Free functions ────────────────────────────────────────────────────────
    m.def("auto_detect_f405", &AutoDetectF405,
        "Scan COM1–COM29 and return the first port that opens, or 'NOT_FOUND'.");
}