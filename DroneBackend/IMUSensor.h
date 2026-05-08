#pragma once
#include "DroneLink.h"

class IMUSensor {
private:
    DroneLink* drone; // Pointer to the main hub

public:
    IMUSensor(DroneLink* hub) : drone(hub) {}

    struct IMUData {
        int16_t accX, accY, accZ;
        int16_t gyroX, gyroY, gyroZ;
    };

    IMUData getRawData();
};