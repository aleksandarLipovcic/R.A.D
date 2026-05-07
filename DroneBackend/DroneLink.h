#pragma once
#include <windows.h>
#include <string>
#include <vector>

class DroneLink {
private:
    HANDLE hSerial = INVALID_HANDLE_VALUE;
    bool connected = false;

public:
    DroneLink();   // Constructor
    ~DroneLink();  // Destructor

    // Connection methods
    bool connect(std::string portName);
    void disconnect();

    // Data methods
    std::vector<float> getAttitude();
    float getBatteryVoltage();
};

// Standalone helper function
std::string AutoDetectF405();