#include "IMUSensor.h"

IMUSensor::IMUSensor(DroneLink* hub) : drone(hub) {}

// ── getRawData() ──────────────────────────────────────────────────────────────
// Returns ADC counts as-is from the latest thread-safe snapshot.
// Safe to call from any thread at any frequency.

IMUSensor::IMUData IMUSensor::getRawData() {
    DroneState state = drone->getLatestState();

    IMUData d;
    d.accX = state.ax;
    d.accY = state.ay;
    d.accZ = state.az;
    d.gyroX = state.gx;
    d.gyroY = state.gy;
    d.gyroZ = state.gz;
    return d;
}

// ── getScaledData() ───────────────────────────────────────────────────────────
// Converts raw counts to physical units using MPU-6500 / Betaflight 4.5.x defaults:
//
//   Accelerometer — ±16g range (INV_FSR_16G, confirmed BF 4.5.0 accgyro_mpu6500.c):
//     Hardware sensitivity = 2048 LSB/g.
//     Betaflight sets acc_1G = 512 × 4 = 2048 to match this range.
//     MSP_RAW_IMU transmits accADC directly — no prescaling applied before
//     transmission.  Effective MSP-layer sensitivity: 2048 LSB/g → ACC_SCALE = 1/2048.
//
//   Gyroscope — ±2000 °/s range:
//     Hardware sensitivity = 16.4 LSB/(°/s) → GYRO_SCALE = 1/16.4.
//     BF transmits gyro counts without prescaling (unchanged).
//
// If your Betaflight config uses a different range, adjust the constants
// in IMUSensor.h (ACC_SCALE / GYRO_SCALE).

IMUSensor::IMUScaled IMUSensor::getScaledData() {
    DroneState state = drone->getLatestState();

    IMUScaled d;
    d.accX = state.ax * ACC_SCALE;
    d.accY = state.ay * ACC_SCALE;
    d.accZ = state.az * ACC_SCALE;
    d.gyroX = state.gx * GYRO_SCALE;
    d.gyroY = state.gy * GYRO_SCALE;
    d.gyroZ = state.gz * GYRO_SCALE;
    return d;
}