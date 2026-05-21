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
// Converts raw counts to physical units using MPU-6500 defaults:
//   ±4g accel range, MSP-layer: BF pre-divides by 4 → divide by 2048 to get g
//   ±2000°/s gyro     → divide by 16.4  to get °/s
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