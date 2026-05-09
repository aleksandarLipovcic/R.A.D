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
    // MPU-6500 defaults used by Betaflight:
    //   Accel:  ±4g  range  → LSB/g  = 8192
    //   Gyro:   ±2000°/s    → LSB/°/s = 16.4
    struct IMUScaled {
        float accX, accY, accZ;   // g
        float gyroX, gyroY, gyroZ;  // degrees / second
    };

    IMUSensor(DroneLink* hub);

    // Non-blocking — reads latest snapshot from the background thread
    IMUData   getRawData();
    IMUScaled getScaledData();

private:
    DroneLink* drone;

    // Scale factors — adjust here if FC accel/gyro range differs
    static constexpr float ACC_SCALE = 1.0f / 8192.0f;  // counts → g
    static constexpr float GYRO_SCALE = 1.0f / 16.4f;    // counts → °/s
};