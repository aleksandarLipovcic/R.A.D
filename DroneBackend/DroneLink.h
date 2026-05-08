#pragma once
#include <windows.h>
#include <string>
#include <vector>

class DroneLink {
private:
    HANDLE hSerial = INVALID_HANDLE_VALUE;
    bool connected = false;

public:
    DroneLink();
    ~DroneLink();

    bool connect(std::string portName);
    void disconnect();

    // The "Pipe" - sends a request and returns the raw byte response
    std::vector<uint8_t> sendRequest(uint8_t mspID);
};