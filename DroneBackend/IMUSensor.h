#pragma once
#include "DroneLink.h"
#include <cstdint>

class IMUSensor {
public:
    // ── Raw ADC counts directly from MSP_RAW_IMU ─────────────────────────────
    struct IMUData {
        int16_t accX, accY, accZ;
        int16_t gyroX, gyroY, gyroZ;
    };

    // ── Physical units ────────────────────────────────────────────────────────
    // MPU-6500 at ±16g (INV_FSR_16G — confirmed BF 4.5.0 accgyro_mpu6500.c):
    //   Hardware sensitivity = 2048 LSB/g.
    //   Betaflight acc_1G = 512 × 4 = 2048, matching this range directly.
    //   MSP_RAW_IMU transmits accADC without any prescaling — the effective
    //   MSP-layer sensitivity is therefore 2048 LSB/g.
    //   ACC_SCALE = 1/2048 converts MSP counts to g.
    //
    // GYRO_SCALE is unaffected — BF transmits gyro counts without prescaling.
    //   MPU-6500 at ±2000 °/s → hardware sensitivity = 16.4 LSB/(°/s).
    //   GYRO_SCALE = 1/16.4 converts MSP counts to °/s.
    //
    // IMUWidget uses ACCEL_SCALE = 2048 for the same reason.
    struct IMUScaled {
        float accX, accY, accZ;     // g
        float gyroX, gyroY, gyroZ;  // degrees / second
    };

    IMUSensor(DroneLink* hub);

    IMUData   getRawData();
    IMUScaled getScaledData();

private:
    DroneLink* drone;

    // FIX (DEF-002): was 1/8192 (assumed ±4g hardware range, 8192 LSB/g).
    // BF 4.5.x initialises MPU-6500 at ±16g (INV_FSR_16G); hardware
    // sensitivity at that range is 2048 LSB/g directly, with no BF-side
    // prescaling before MSP transmission.  Correct MSP-layer divisor: 2048.
    // Matches IMUWidget.ACCEL_SCALE = 2048 and fixes the 4× under-read
    // observed in IT-IMU-002.
    static constexpr float ACC_SCALE = 1.0f / 2048.0f;  // MSP counts → g
    static constexpr float GYRO_SCALE = 1.0f / 16.4f;    // MSP counts → °/s  (unchanged)
};