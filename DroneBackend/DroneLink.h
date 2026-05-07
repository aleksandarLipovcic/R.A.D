#pragma once
#include <windows.h>
#include <vector>
#include <string>

class DroneLink {
private:
    HANDLE hSerial;
    bool connected = false;

public:
    DroneLink();
    ~DroneLink();

    bool connect(std::string portName);
    void disconnect();

    // MSP Commands
    std::vector<float> getAttitude(); // Returns [roll, pitch, yaw]
    float getBatteryVoltage();
};