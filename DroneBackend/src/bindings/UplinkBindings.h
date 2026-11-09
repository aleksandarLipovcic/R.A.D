#pragma once
// =============================================================================
// UplinkBindings.h — pybind11 registration for the command uplink
//
// Used from Bindings.cpp: RegisterUplinkBindings(m) is called inside
// PYBIND11_MODULE before CrsfLink is registered (CrsfLink.set_uplink takes a
// CommandLink), and the CrsfLink class has
//     .def("set_uplink", &CrsfLink::setUplink, py::arg("uplink"), py::keep_alive<1, 2>())
// keep_alive keeps the CommandLink alive as long as the CrsfLink, so the
// worker thread can never use a freed object.
//
// Python names (snake_case, like the rest of DroneBackend):
//   cv = DroneBackend.CommandValues(); cv.roll_pct = 10; cv.sticks_enabled = True
//   link = DroneBackend.CommandLink(); link.set_enabled(True); link.set_command(cv)
//   st = link.get_status(); st.state, st.bridge.gate_on, st.radio_channels_us ...
// =============================================================================
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "CommandLink.h"

inline void RegisterUplinkBindings(pybind11::module_& m) {
    namespace py = pybind11;

    py::class_<Uplink::CommandValues>(m, "CommandValues")
        .def(py::init<>())
        .def_readwrite("roll_pct", &Uplink::CommandValues::rollPct)
        .def_readwrite("pitch_pct", &Uplink::CommandValues::pitchPct)
        .def_readwrite("yaw_pct", &Uplink::CommandValues::yawPct)
        .def_readwrite("sticks_enabled", &Uplink::CommandValues::sticksEnabled)
        .def_readwrite("throttle_us", &Uplink::CommandValues::throttleUs)
        .def_readwrite("throttle_enabled", &Uplink::CommandValues::throttleEnabled)
        .def_readwrite("gimbal_tilt_pct", &Uplink::CommandValues::gimbalTiltPct)
        .def_readwrite("gimbal_pan_pct", &Uplink::CommandValues::gimbalPanPct);

    py::class_<Uplink::BridgeStatus>(m, "BridgeStatus")
        .def_readonly("version", &Uplink::BridgeStatus::version)
        .def_readonly("fresh", &Uplink::BridgeStatus::fresh)
        .def_readonly("gate_on", &Uplink::BridgeStatus::gateOn)
        .def_readonly("sticks_active", &Uplink::BridgeStatus::sticksActive)
        .def_readonly("throttle_active", &Uplink::BridgeStatus::throttleActive)
        .def_readonly("telemetry_live", &Uplink::BridgeStatus::telemetryLive)
        .def_readonly("pilot_override", &Uplink::BridgeStatus::pilotOverride)
        .def_readonly("angle_on", &Uplink::BridgeStatus::angleOn)
        .def_readonly("echo_alive", &Uplink::BridgeStatus::echoAlive)
        .def_readonly("rx_frames", &Uplink::BridgeStatus::rxFrames)
        .def_readonly("crc_errors", &Uplink::BridgeStatus::crcErrors)
        .def_readonly("cmd_age_ms", &Uplink::BridgeStatus::cmdAgeMs)
        .def_readonly("throttle_stick", &Uplink::BridgeStatus::throttleStick)
        .def_readonly("throttle_delta", &Uplink::BridgeStatus::throttleDelta)
        .def_readonly("script_period_ms", &Uplink::BridgeStatus::scriptPeriodMs)
        .def_readonly("out_roll", &Uplink::BridgeStatus::outRoll)
        .def_readonly("out_pitch", &Uplink::BridgeStatus::outPitch)
        .def_readonly("out_yaw", &Uplink::BridgeStatus::outYaw)
        .def_readonly("out_tilt", &Uplink::BridgeStatus::outTilt)
        .def_readonly("out_pan", &Uplink::BridgeStatus::outPan)
        .def_readonly("has_pan", &Uplink::BridgeStatus::hasPan);

    py::class_<CommandLink::Limits>(m, "CommandLimits")
        .def(py::init<>())
        .def_readwrite("max_stick_pct", &CommandLink::Limits::maxStickPct)
        .def_readwrite("max_yaw_pct", &CommandLink::Limits::maxYawPct)
        .def_readwrite("throttle_min_us", &CommandLink::Limits::throttleMinUs)
        .def_readwrite("throttle_max_us", &CommandLink::Limits::throttleMaxUs)
        .def_readwrite("stick_slew_pct_per_s", &CommandLink::Limits::stickSlewPctPerS)
        .def_readwrite("throttle_slew_us_per_s", &CommandLink::Limits::throttleSlewUsPerS)
        .def_readwrite("lease_ms", &CommandLink::Limits::leaseMs)
        .def_readwrite("send_period_ms", &CommandLink::Limits::sendPeriodMs)
        .def_readwrite("bridge_timeout_ms", &CommandLink::Limits::bridgeTimeoutMs);

    py::class_<CommandLink::Status>(m, "CommandLinkStatus")
        .def_readonly("state", &CommandLink::Status::state)
        .def_readonly("bridge_detected", &CommandLink::Status::bridgeDetected)
        .def_readonly("bridge_age_ms", &CommandLink::Status::bridgeAgeMs)
        .def_readonly("enabled", &CommandLink::Status::enabled)
        .def_readonly("lease_valid", &CommandLink::Status::leaseValid)
        .def_readonly("bridge", &CommandLink::Status::bridge)
        .def_readonly("last_sent", &CommandLink::Status::lastSent)
        .def_readonly("radio_channels_us", &CommandLink::Status::radioChannelsUs)
        .def_readonly("radio_channels_valid", &CommandLink::Status::radioChannelsValid)
        .def_readonly("radio_channels_age_ms", &CommandLink::Status::radioChannelsAgeMs)
        .def_readonly("rtt_ms", &CommandLink::Status::rttMs)
        .def_readonly("rtt_avg_ms", &CommandLink::Status::rttAvgMs)
        .def_readonly("frames_sent", &CommandLink::Status::framesSent)
        .def_readonly("write_errors", &CommandLink::Status::writeErrors);

    py::class_<CommandLink>(m, "CommandLink")
        .def(py::init<>())
        .def("set_limits", &CommandLink::setLimits)
        .def("get_limits", &CommandLink::getLimits)
        .def("set_enabled", &CommandLink::setEnabled)
        .def("is_enabled", &CommandLink::isEnabled)
        .def("set_command", &CommandLink::setCommand,
             "Full command; renews the lease (call at least every lease_ms).")
        .def("set_gimbal_tilt_pct", &CommandLink::setGimbalTiltPct)
        .def("set_gimbal_pan_pct", &CommandLink::setGimbalPanPct)
        .def("set_gimbal_pct", &CommandLink::setGimbalPct, py::arg("pan_pct"), py::arg("tilt_pct"),
             "Both gimbal channels at once (no lease, works without set_enabled).")
        .def("neutral", &CommandLink::neutral)
        .def("get_status", &CommandLink::getStatus);
}
