// =============================================================================
// test_imu_parser.cpp
//
// Unit tests for:
//   - DroneLink::parseIMU()        (UT-IMU-001 .. UT-IMU-004)
//   - IMUSensor::getScaledData()   (UT-SCALE-001 .. UT-SCALE-003)
//
// V-Model reference: IMU Subsystem V-Model, Section 6.1 & 6.2
// SRS coverage:      SRS-IMU-002, SRS-IMU-003, SRS-IMU-004a/b, SRS-IMU-008
//
// Build (MSVC + CMake, add to your existing test target):
//   target_sources(imu_tests PRIVATE test_imu_parser.cpp)
//   target_link_libraries(imu_tests PRIVATE DroneBackend_static gtest gtest_main)
//
// Run:
//   ctest --test-dir build -R imu --output-on-failure
//   -- or --
//   ./imu_tests.exe
// =============================================================================

#include <gtest/gtest.h>
#include <vector>
#include <cstdint>
#include <cstring>

#include "DroneLink.h"
#include "IMUSensor.h"
#include "pch.h"
// =============================================================================
// Helpers
// =============================================================================

// Build a well-formed 18-byte MSP_RAW_IMU frame.
// All six fields default to 0; override individual ones as needed.
//
// Frame layout (per SDD Section 4.2):
//   [0]      '$'
//   [1]      'M'
//   [2]      '>'
//   [3]      0x0C  (payloadLen = 12)
//   [4]      0x66  (cmd = 102 = MSP_RAW_IMU)
//   [5..6]   ax  (int16-LE)
//   [7..8]   ay  (int16-LE)
//   [9..10]  az  (int16-LE)
//   [11..12] gx  (int16-LE)
//   [13..14] gy  (int16-LE)
//   [15..16] gz  (int16-LE)
//   [17]     checksum (XOR bytes [3..16] — not validated by parseIMU, but
//            present so buf.size() == 18 always satisfies the guard)

static std::vector<uint8_t> makeIMUFrame(
    int16_t ax = 0, int16_t ay = 0, int16_t az = 0,
    int16_t gx = 0, int16_t gy = 0, int16_t gz = 0,
    uint8_t cmd = 102)
{
    std::vector<uint8_t> buf(18, 0x00);
    buf[0] = '$';
    buf[1] = 'M';
    buf[2] = '>';
    buf[3] = 0x0C;  // payloadLen = 12
    buf[4] = cmd;

    // Helper: store int16 little-endian
    auto put16 = [&](int offset, int16_t v) {
        buf[offset] = static_cast<uint8_t>(v & 0xFF);
        buf[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        };

    put16(5, ax);
    put16(7, ay);
    put16(9, az);
    put16(11, gx);
    put16(13, gy);
    put16(15, gz);

    // Compute checksum (XOR of bytes [3..16]) for correctness
    uint8_t cs = 0;
    for (int i = 3; i <= 16; ++i) cs ^= buf[i];
    buf[17] = cs;

    return buf;
}

// =============================================================================
// Mock DroneLink
//
// IMUSensor only calls drone->getLatestState(), so a minimal subclass
// that injects a preset DroneState is sufficient.
// =============================================================================

class MockDroneLink : public DroneLink {
public:
    DroneState injectedState;

    // Override getLatestState() — returns the injected state directly.
    // No mutex needed in tests (single-threaded).
    DroneState getLatestState() override {
        return injectedState;
    }
};

// =============================================================================
// Test fixture: ParseIMU
//
// Exposes parseIMU() via a thin friend accessor.
// DroneLink declares:  friend class DroneLinkTestAccessor;
// If that friend declaration is not yet in DroneLink.h, add it, or use the
// alternative approach below (direct instantiation + private accessor pattern).
//
// Alternative (no friend needed): call parseIMU via a thin subclass that
// re-exposes it as public.  That is what we do here to keep DroneLink.h clean.
// =============================================================================

class TestableDroneLink : public DroneLink {
public:
    // Re-expose private parser as public for unit testing
    bool callParseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
        return parseIMU(buf, s);
    }
};

// =============================================================================
// UT-IMU-001 : Happy Path — Nominal Frame
// SRS: SRS-IMU-002, SRS-IMU-003
// =============================================================================

TEST(ParseIMU, UT_IMU_001_HappyPathNominalFrame) {
    TestableDroneLink dl;
    DroneState s{};

    // Build the exact buffer from the test spec:
    //   ax=256 (0x0100), ay=-257 (0xFEFF), az=16 (0x0010)
    //   gx=100 (0x0064), gy=-100 (0xFF9C), gz=0
    auto buf = makeIMUFrame(256, -257, 16, 100, -100, 0);

    bool result = dl.callParseIMU(buf, s);

    EXPECT_TRUE(result);
    EXPECT_EQ(s.ax, 256);
    EXPECT_EQ(s.ay, -257);
    EXPECT_EQ(s.az, 16);
    EXPECT_EQ(s.gx, 100);
    EXPECT_EQ(s.gy, -100);
    EXPECT_EQ(s.gz, 0);
}

// =============================================================================
// UT-IMU-002 : Short Frame Rejection
// SRS: SRS-IMU-002, SRS-IMU-008
// =============================================================================

TEST(ParseIMU, UT_IMU_002_ShortFrameRejected) {
    TestableDroneLink dl;
    DroneState s{};  // all zeros by default

    // 17-byte buffer — one byte short of the required 18
    std::vector<uint8_t> buf(17, 0x66);

    bool result = dl.callParseIMU(buf, s);

    EXPECT_FALSE(result);

    // DroneState must be left entirely unmodified
    EXPECT_EQ(s.ax, 0);
    EXPECT_EQ(s.ay, 0);
    EXPECT_EQ(s.az, 0);
    EXPECT_EQ(s.gx, 0);
    EXPECT_EQ(s.gy, 0);
    EXPECT_EQ(s.gz, 0);
}

TEST(ParseIMU, UT_IMU_002_EmptyFrameRejected) {
    TestableDroneLink dl;
    DroneState s{};

    std::vector<uint8_t> buf;  // empty

    EXPECT_FALSE(dl.callParseIMU(buf, s));
}

TEST(ParseIMU, UT_IMU_002_SingleByteFrameRejected) {
    TestableDroneLink dl;
    DroneState s{};

    std::vector<uint8_t> buf = { '$' };

    EXPECT_FALSE(dl.callParseIMU(buf, s));
}

// =============================================================================
// UT-IMU-003 : Wrong Command Byte Rejection
// SRS: SRS-IMU-008
// =============================================================================

TEST(ParseIMU, UT_IMU_003_WrongCmdRejected_STATUS) {
    TestableDroneLink dl;
    DroneState s{};

    // Sufficient length but cmd = 101 (MSP_STATUS) instead of 102
    auto buf = makeIMUFrame(1, 2, 3, 4, 5, 6, /*cmd=*/101);

    EXPECT_FALSE(dl.callParseIMU(buf, s));
    // Fields must remain zero — frame was rejected before extraction
    EXPECT_EQ(s.ax, 0);
}

TEST(ParseIMU, UT_IMU_003_WrongCmdRejected_Zero) {
    TestableDroneLink dl;
    DroneState s{};

    auto buf = makeIMUFrame(0, 0, 0, 0, 0, 0, /*cmd=*/0);

    EXPECT_FALSE(dl.callParseIMU(buf, s));
}

TEST(ParseIMU, UT_IMU_003_WrongCmdRejected_DEBUG) {
    TestableDroneLink dl;
    DroneState s{};

    auto buf = makeIMUFrame(0, 0, 0, 0, 0, 0, /*cmd=*/254);

    EXPECT_FALSE(dl.callParseIMU(buf, s));
}

// =============================================================================
// UT-IMU-004 : Boundary Values — Maximum and Minimum int16
// SRS: SRS-IMU-003
// =============================================================================

TEST(ParseIMU, UT_IMU_004_MaxInt16) {
    TestableDroneLink dl;
    DroneState s{};

    // ax = +32767 (0x7FFF LE: 0xFF, 0x7F)
    auto buf = makeIMUFrame(32767, 0, 0, 0, 0, 0);

    EXPECT_TRUE(dl.callParseIMU(buf, s));
    EXPECT_EQ(s.ax, 32767);
}

TEST(ParseIMU, UT_IMU_004_MinInt16) {
    TestableDroneLink dl;
    DroneState s{};

    // ay = -32768 (0x8000 LE: 0x00, 0x80)
    auto buf = makeIMUFrame(0, -32768, 0, 0, 0, 0);

    EXPECT_TRUE(dl.callParseIMU(buf, s));
    EXPECT_EQ(s.ay, -32768);
}

TEST(ParseIMU, UT_IMU_004_BoundaryCombo) {
    // From test spec: ax=+32767, ay=-32768, az=0
    TestableDroneLink dl;
    DroneState s{};

    auto buf = makeIMUFrame(32767, -32768, 0, 0, 0, 0);

    EXPECT_TRUE(dl.callParseIMU(buf, s));
    EXPECT_EQ(s.ax, 32767);
    EXPECT_EQ(s.ay, -32768);
    EXPECT_EQ(s.az, 0);
}

TEST(ParseIMU, UT_IMU_004_AllNegative) {
    TestableDroneLink dl;
    DroneState s{};

    auto buf = makeIMUFrame(-1, -1, -1, -1, -1, -1);

    EXPECT_TRUE(dl.callParseIMU(buf, s));
    EXPECT_EQ(s.ax, -1);
    EXPECT_EQ(s.ay, -1);
    EXPECT_EQ(s.az, -1);
    EXPECT_EQ(s.gx, -1);
    EXPECT_EQ(s.gy, -1);
    EXPECT_EQ(s.gz, -1);
}

TEST(ParseIMU, UT_IMU_004_AllZero) {
    // Verify all-zero payload is accepted and produces zeros (not a false negative)
    TestableDroneLink dl;
    DroneState s{};
    s.ax = 999;  // pre-set to non-zero to confirm overwrite

    auto buf = makeIMUFrame(0, 0, 0, 0, 0, 0);

    EXPECT_TRUE(dl.callParseIMU(buf, s));
    EXPECT_EQ(s.ax, 0);
    EXPECT_EQ(s.gx, 0);
}

// =============================================================================
// UT-SCALE-001 : Accelerometer Scale — +1g Reference
// SRS: SRS-IMU-004a
// =============================================================================

TEST(IMUSensorScale, UT_SCALE_001_AccelOneg_ZAxis) {
    MockDroneLink mock;
    mock.injectedState.az = 8192;  // +1g on Z axis

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    // 8192 * (1/8192) = 1.0 g  — tolerance ±0.001 g per test spec
    EXPECT_NEAR(scaled.accZ, 1.0f, 0.001f);
}

TEST(IMUSensorScale, UT_SCALE_001_AccelOneg_AllAxes) {
    // Verify the same divisor applies consistently to X and Y axes
    MockDroneLink mock;
    mock.injectedState.ax = 8192;
    mock.injectedState.ay = -8192;
    mock.injectedState.az = 4096;  // 0.5 g

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.accX, 1.0f, 0.001f);
    EXPECT_NEAR(scaled.accY, -1.0f, 0.001f);
    EXPECT_NEAR(scaled.accZ, 0.5f, 0.001f);
}

TEST(IMUSensorScale, UT_SCALE_001_AccelZeroInput) {
    MockDroneLink mock;
    mock.injectedState.ax = 0;
    mock.injectedState.ay = 0;
    mock.injectedState.az = 0;

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.accX, 0.0f, 0.001f);
    EXPECT_NEAR(scaled.accY, 0.0f, 0.001f);
    EXPECT_NEAR(scaled.accZ, 0.0f, 0.001f);
}

// =============================================================================
// UT-SCALE-002 : Gyroscope Scale — 1000 °/s Reference
// SRS: SRS-IMU-004b
// =============================================================================

TEST(IMUSensorScale, UT_SCALE_002_Gyro1000dps_XAxis) {
    MockDroneLink mock;
    mock.injectedState.gx = 16400;  // 16400 * (1/16.4) = 1000.0 °/s

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    // Tolerance ±0.1 °/s per test spec
    EXPECT_NEAR(scaled.gyroX, 1000.0f, 0.1f);
}

TEST(IMUSensorScale, UT_SCALE_002_Gyro_SmallValue) {
    // 164 counts → 10.0 °/s
    MockDroneLink mock;
    mock.injectedState.gy = 164;

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.gyroY, 10.0f, 0.1f);
}

TEST(IMUSensorScale, UT_SCALE_002_GyroZero) {
    MockDroneLink mock;
    mock.injectedState.gx = 0;
    mock.injectedState.gy = 0;
    mock.injectedState.gz = 0;

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.gyroX, 0.0f, 0.001f);
    EXPECT_NEAR(scaled.gyroY, 0.0f, 0.001f);
    EXPECT_NEAR(scaled.gyroZ, 0.0f, 0.001f);
}

// =============================================================================
// UT-SCALE-003 : Negative Gyroscope Value — Sign Preservation
// SRS: SRS-IMU-004b
// =============================================================================

TEST(IMUSensorScale, UT_SCALE_003_NegativeGyro_YAxis) {
    MockDroneLink mock;
    mock.injectedState.gy = -1640;  // -1640 * (1/16.4) = -100.0 °/s

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.gyroY, -100.0f, 0.1f);
}

TEST(IMUSensorScale, UT_SCALE_003_NegativeGyro_AllAxes) {
    MockDroneLink mock;
    mock.injectedState.gx = -16400;
    mock.injectedState.gy = -820;   // -50.0 °/s
    mock.injectedState.gz = 820;   // +50.0 °/s

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.gyroX, -1000.0f, 0.1f);
    EXPECT_NEAR(scaled.gyroY, -50.0f, 0.1f);
    EXPECT_NEAR(scaled.gyroZ, 50.0f, 0.1f);
}

TEST(IMUSensorScale, UT_SCALE_003_MaxGyroNegative) {
    // -32768 counts → -32768 / 16.4 ≈ -1997.6 °/s (within ±2000 range)
    MockDroneLink mock;
    mock.injectedState.gz = -32768;

    IMUSensor imu(&mock);
    auto scaled = imu.getScaledData();

    EXPECT_NEAR(scaled.gyroZ, -32768.0f / 16.4f, 0.5f);
    EXPECT_LT(scaled.gyroZ, 0.0f);  // Must be negative
}

// =============================================================================
// getRawData() smoke tests — verifies pass-through with no scaling
// =============================================================================

TEST(IMUSensorRaw, RawDataPassthrough) {
    MockDroneLink mock;
    mock.injectedState.ax = 100;
    mock.injectedState.ay = -200;
    mock.injectedState.az = 300;
    mock.injectedState.gx = -400;
    mock.injectedState.gy = 500;
    mock.injectedState.gz = -600;

    IMUSensor imu(&mock);
    auto raw = imu.getRawData();

    EXPECT_EQ(raw.accX, 100);
    EXPECT_EQ(raw.accY, -200);
    EXPECT_EQ(raw.accZ, 300);
    EXPECT_EQ(raw.gyroX, -400);
    EXPECT_EQ(raw.gyroY, 500);
    EXPECT_EQ(raw.gyroZ, -600);
}

// =============================================================================
// main() — only needed if not using gtest_main library target
// With CMake: target_link_libraries(imu_tests PRIVATE gtest_main)
// Comment out this main() in that case.
// =============================================================================

// int main(int argc, char** argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }