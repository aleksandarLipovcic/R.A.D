#pragma once
#include <windows.h>
#include <string>
#include <vector>

// Struct to hold both the raw bytes and the round-trip latency
struct TelemetryResult {
    std::vector<uint8_t> data;
    double latencyMs;
};

class DroneLink {
private:
    HANDLE hSerial = INVALID_HANDLE_VALUE;
    bool connected = false;

public:
    DroneLink();
    ~DroneLink();

    bool connect(std::string portName);
    void disconnect();

    // Standard communication
    std::vector<uint8_t> sendRequest(uint8_t mspID);

    // Latency-aware communication for thesis measurements
    TelemetryResult sendRequestWithTiming(uint8_t mspID);

    bool isConnected() { return connected; }
};

std::string AutoDetectF405();