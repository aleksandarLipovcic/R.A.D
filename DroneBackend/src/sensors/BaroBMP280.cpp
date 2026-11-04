// ─────────────────────────────────────────────────────────────────────────────
// BaroBMP280.cpp
//
// All parsing and scaling logic lives in BaroBMP280.h (static methods).
// This translation unit exists so the linker sees the symbol and to hold
// the integration notes for DroneLink.
//
// ─────────────────────────────────────────────────────────────────────────────
// HOW TO INTEGRATE INTO DroneLink
// ─────────────────────────────────────────────────────────────────────────────
//
// 1.  DroneLink.h — add to the MSP namespace and DroneState struct:
//
//         namespace MSP {
//             ...
//             static constexpr uint8_t ALTITUDE = 109;  // ← add this line
//         }
//
//         struct DroneState {
//             ...
//             // Barometer  (BMP280 via Betaflight MSP_ALTITUDE)
//             int32_t baroAltitudeCm    = 0;   // cm above home
//             int16_t baroVarioCmPerSec = 0;   // cm/s vertical speed
//             bool    baroValid         = false;
//         };
//
//     Also add the parseBaro declaration:
//         bool parseBaro(const std::vector<uint8_t>& buf, DroneState& s);
//
// 2.  DroneLink.cpp — inside communicationLoop(), in the SENSOR POLL block:
//
//         // 5. Barometer (BMP280 via MSP_ALTITUDE)
//         {
//             auto buf = sendMSP(MSP::ALTITUDE);
//             parseBaro(buf, pending);
//         }
//
//     And add the parseBaro() implementation (see below).
//
// 3.  DroneLink.cpp — add parseBaro() implementation:
//
//         bool DroneLink::parseBaro(const std::vector<uint8_t>& buf,
//                                   DroneState& s) {
//             BaroReading raw;
//             if (!BaroBMP280::parse(buf, raw)) {
//                 s.baroValid = false;
//                 return false;
//             }
//             s.baroAltitudeCm    = raw.altitudeCm;
//             s.baroVarioCmPerSec = raw.varioCmPerSec;
//             s.baroValid         = true;
//             return true;
//         }
//
// 4.  bindings.cpp (pybind11) — expose in the DroneState class_ block:
//
//         .def_readonly("baro_altitude_cm",     &DroneState::baroAltitudeCm)
//         .def_readonly("baro_vario_cm_per_sec",&DroneState::baroVarioCmPerSec)
//         .def_readonly("baro_valid",            &DroneState::baroValid)
//
// 5.  main.py — read and pass to the GUI widget:
//
//         ui_data["baro_altitude_cm"]      = state.baro_altitude_cm
//         ui_data["baro_vario_cm_per_sec"] = state.baro_vario_cm_per_sec
//         ui_data["baro_valid"]            = state.baro_valid
//
// ─────────────────────────────────────────────────────────────────────────────

#include "BaroBMP280.h"

// No additional definitions needed — all logic is static in the header.