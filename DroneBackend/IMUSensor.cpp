#include "IMUSensor.h"

IMUSensor::IMUData IMUSensor::getRawData() {
    // Use the hub to get raw MSP_RAW_IMU (ID 102) data
    std::vector<uint8_t> rx = drone->sendRequest(102);
    IMUData data = { 0, 0, 0, 0, 0, 0 };

    if (rx.size() >= 18 && rx[4] == 102) {
        // Unpack Accel (6 bytes) and Gyro (6 bytes)
        data.accX = (rx[5] | (rx[6] << 8));
        data.accY = (rx[7] | (rx[8] << 8));
        data.accZ = (rx[9] | (rx[10] << 8));
        data.gyroX = (rx[11] | (rx[12] << 8));
        data.gyroY = (rx[13] | (rx[14] << 8));
        data.gyroZ = (rx[15] | (rx[16] << 8));
    }
    return data;
}