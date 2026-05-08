#pragma once
#include "DroneLink.h"
#include <vector>
#include <cstdint>

class IMUSensor {
public:
    struct IMUData {
        int16_t accX, accY, accZ;
        int16_t gyroX, gyroY, gyroZ;
    };

    // Constructor takes a pointer to our Communication Hub
    IMUSensor(DroneLink* hub) : drone(hub) {}

    // Existing method for high-speed raw data
    IMUData getRawData();

private:
    DroneLink* drone;
};