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
    // MPU-6500 at ±4g: raw ADC sensitivity = 8192 LSB/g.
    // Betaflight's MSP_RAW_IMU divides accelerometer counts by 4 before
    // transmission, so the effective sensitivity at the MSP wire is
    // 8192 / 4 = 2048 LSB/g.  ACC_SCALE must reflect the MSP-layer value.
    // GYRO_SCALE is unaffected — BF transmits gyro counts without prescaling.
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

    // FIX: was 1/8192 (raw ADC sensitivity).  MSP_RAW_IMU delivers accel
    // values pre-divided by 4, so the correct MSP-layer divisor is 2048.
    // Matches IMUWidget.ACCEL_SCALE = 2048 and the observed 0.25 g → 1.0 g
    // correction required by IT-IMU-002.
    static constexpr float ACC_SCALE = 1.0f / 2048.0f;  // MSP counts → g
    static constexpr float GYRO_SCALE = 1.0f / 16.4f;    // counts → °/s  (unchanged)
};