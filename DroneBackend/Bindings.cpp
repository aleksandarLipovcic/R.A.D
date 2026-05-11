#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DroneLink.h"
#include "IMUSensor.h"

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(DroneBackend, m) {
    m.doc() = "Project R.A.D — threaded drone telemetry backend";

    // ── DroneState ────────────────────────────────────────────────────────────
    py::class_<DroneState>(m, "DroneState")
        // IMU
        .def_readonly("ax", &DroneState::ax)
        .def_readonly("ay", &DroneState::ay)
        .def_readonly("az", &DroneState::az)
        .def_readonly("gx", &DroneState::gx)
        .def_readonly("gy", &DroneState::gy)
        .def_readonly("gz", &DroneState::gz)
        // Attitude
        .def_readonly("roll", &DroneState::roll)
        .def_readonly("pitch", &DroneState::pitch)
        .def_readonly("yaw", &DroneState::yaw)
        // Analog
        .def_readonly("battery_voltage", &DroneState::batteryVoltage)
        .def_readonly("rssi", &DroneState::rssi)
        // Barometer (BMP280 via MSP_ALTITUDE)
        .def_readonly("baro_altitude_cm",
            &DroneState::baroAltitudeCm,
            "FC-fused BMP280 altitude above home point, in cm.")
        .def_readonly("baro_vario_cm_per_sec",
            &DroneState::baroVarioCmPerSec,
            "Vertical speed in cm/s. Positive = climbing.")
        .def_readonly("baro_valid",
            &DroneState::baroValid,
            "True when a valid MSP_ALTITUDE frame was received.")
        // Magnetometer (QMC5883L via NEO-M10 I2C → MSP_RAW_MAG)
        .def_readonly("mag_x",
            &DroneState::magX,
            "Raw magnetic field, X axis (FC sensor frame, ADC counts).")
        .def_readonly("mag_y",
            &DroneState::magY,
            "Raw magnetic field, Y axis (FC sensor frame, ADC counts).")
        .def_readonly("mag_z",
            &DroneState::magZ,
            "Raw magnetic field, Z axis (FC sensor frame, ADC counts).")
        .def_readonly("mag_heading_deg",
            &DroneState::magHeadingDeg,
            "Tilt-uncorrected 2-D magnetic heading, 0-360°. "
            "Accurate only when the drone is level.")
        .def_readonly("mag_valid",
            &DroneState::magValid,
            "True when a valid MSP_RAW_MAG frame was received.")
        // Magnetometer calibration state
        .def_readonly("mag_cal_active",
            &DroneState::magCalActive,
            "True while the FC is in magnetometer calibration mode. "
            "Rotate the drone on all axes during this window.")
        .def_readonly("mag_cal_seconds_remaining",
            &DroneState::magCalSecondsRemaining,
            "Seconds left in the calibration window (counts down from 30 to 0).")
        // Diagnostics
        .def_readonly("last_rtt_ms", &DroneState::lastRttMs)
        .def_readonly("fc_cycle_ms", &DroneState::fcCycleMs)
        .def_readonly("link_healthy", &DroneState::linkHealthy)
        .def_readonly("packet_count", &DroneState::packetCount)
        // Convenience dict
        .def("to_dict", [](const DroneState& s) {
        return py::dict(
            // IMU
            "ax"_a = s.ax, "ay"_a = s.ay, "az"_a = s.az,
            "gx"_a = s.gx, "gy"_a = s.gy, "gz"_a = s.gz,
            // Attitude (pre-divided for convenience)
            "roll_deg"_a = s.roll / 10.0f,
            "pitch_deg"_a = s.pitch / 10.0f,
            "yaw_deg"_a = static_cast<float>(s.yaw),
            // Analog
            "battery_v"_a = s.batteryVoltage,
            "rssi"_a = s.rssi,
            // Barometer — pre-converted to useful units
            "baro_altitude_m"_a = s.baroAltitudeCm * 0.01,
            "baro_altitude_ft"_a = s.baroAltitudeCm * 0.0328084,
            "baro_vario_mps"_a = s.baroVarioCmPerSec * 0.01,
            "baro_vario_fpm"_a = s.baroVarioCmPerSec * 1.9685,
            "baro_valid"_a = s.baroValid,
            // Magnetometer — QMC5883L via NEO-M10 I2C
            "mag_x"_a = s.magX,
            "mag_y"_a = s.magY,
            "mag_z"_a = s.magZ,
            "mag_heading_deg"_a = s.magHeadingDeg,
            "mag_valid"_a = s.magValid,
            // Magnetometer calibration state
            "mag_cal_active"_a = s.magCalActive,
            "mag_cal_seconds_remaining"_a = s.magCalSecondsRemaining,
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
            "Open serial port and start background polling thread.")
        .def("disconnect", &DroneLink::disconnect,
            "Stop background thread and close serial port.")
        .def("is_connected", &DroneLink::isConnected)
        .def("get_latest_state", &DroneLink::getLatestState,
            "Return a thread-safe snapshot of the latest telemetry.")
        .def("set_poll_interval_ms", &DroneLink::setPollIntervalMs,
            py::arg("ms"),
            "Tune background thread cadence (default 10 ms = 100 Hz).")
        .def("start_mag_calibration", &DroneLink::startMagCalibration,
            "Send MSP_MAG_CALIBRATION (205) to the FC and start the 30 s countdown.\n"
            "Rotate the drone on all axes during the calibration window.\n"
            "Monitor state.mag_cal_active and state.mag_cal_seconds_remaining\n"
            "via get_latest_state() to drive a GUI progress indicator.");

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
        "Scan COM1-COM29 and return the first port that opens, or 'NOT_FOUND'.");
}