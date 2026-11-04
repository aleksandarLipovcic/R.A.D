#pragma once
#include <vector>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// MagReading — plain data bag populated by parseDebug() in DroneLink.cpp
// ─────────────────────────────────────────────────────────────────────────────
struct MagReading {
    int16_t x = 0;             // raw ADC counts, sensor X axis
    int16_t y = 0;             // raw ADC counts, sensor Y axis
    int16_t z = 0;             // raw ADC counts, sensor Z axis
    float   headingDeg = 0.0f; // magnetic heading, 0-360°, 0 = North
    // computed as atan2(y, x), tilt-uncorrected
};

// ─────────────────────────────────────────────────────────────────────────────
// MagQMC5883L
//
// IMPORTANT — MSP_RAW_MAG DOES NOT EXIST IN BETAFLIGHT 4.x:
//
//   The old MultiWii protocol had an MSP_RAW_MAG command at ID 130.
//   Betaflight repurposed ID 130 as MSP_BATTERY_STATE.
//   There is NO Betaflight MSP command that directly returns raw mag X/Y/Z.
//
// CORRECT DATA PATH for Betaflight 4.5.x:
//
//   Raw mag X/Y/Z is exposed through the debug subsystem:
//     1. In Betaflight CLI:
//          set debug_mode = MAG_CALIB
//          save
//     2. Poll MSP_DEBUG (254):
//          debug[0] = raw mag X  (int16, ADC counts)
//          debug[1] = raw mag Y
//          debug[2] = raw mag Z
//          debug[3] = heading error / calibration quality
//
//   This is exactly what the Betaflight Configurator Sensors tab uses.
//   The parsing is handled directly in DroneLink::parseDebug().
//
// ABOUT THIS CLASS:
//
//   MagQMC5883L is kept as a utility class providing the heading computation
//   and checksum helper so unit tests can exercise them independently.
//   It no longer parses a direct MSP_RAW_MAG frame (there is none), but the
//   MagReading struct is still used as the data bag type throughout the code.
//
// PREREQUISITE CLI COMMANDS (Betaflight 4.5.x):
//   set mag_hardware = QMC5883   (or AUTO if only one mag is present)
//   set align_mag    = CW90      (adjust to your GPS module's orientation)
//   set debug_mode   = MAG_CALIB
//   save
// ─────────────────────────────────────────────────────────────────────────────
class MagQMC5883L {
public:
    // ── computeHeading() ─────────────────────────────────────────────────────
    // Tilt-uncorrected 2-D heading from raw X/Y field counts.
    // Range: 0-360°, where 0° / 360° = magnetic North.
    // Accurate only when the drone is level.
    static float computeHeading(int16_t x, int16_t y);

    // ── verifyChecksum() ─────────────────────────────────────────────────────
    // MSP v1 XOR checksum: XOR of bytes [3..N-2] must equal byte [N-1].
    // Exposed for unit testing.
    static bool verifyChecksum(const std::vector<uint8_t>& buf);

private:
    static int16_t read16(const std::vector<uint8_t>& buf, size_t i);
};