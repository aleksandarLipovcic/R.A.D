#pragma once
#include <windows.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// MSP command IDs — extend here as new sensors are added
// ─────────────────────────────────────────────────────────────────────────────
namespace MSP {
    constexpr uint8_t RAW_IMU = 102;
    constexpr uint8_t ATTITUDE = 108;
    constexpr uint8_t ANALOG = 110;  // battery voltage, RSSI
    constexpr uint8_t ALTITUDE = 109;  // BMP280 fused altitude + vario (MSP_ALTITUDE)
    constexpr uint8_t DEBUG = 254;
    // Add future IDs here:
    // constexpr uint8_t GPS = 106;
}

// ─────────────────────────────────────────────────────────────────────────────
// DroneState — written exclusively by the worker thread,
//              read by Python via getLatestState() (mutex-protected copy)
// ─────────────────────────────────────────────────────────────────────────────
struct DroneState {
    // MPU-6500 raw values (ADC counts, not scaled)
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    // Attitude (degrees × 10 as sent by FC, divide in Python)
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;

    // Power
    float   batteryVoltage = 0.0f;  // volts
    uint8_t rssi = 0;     // 0-255

    // Barometer — BMP280 via MSP_ALTITUDE
    // Requires: Barometer enabled in Betaflight Configurator
    //           (Configuration tab -> Sensors -> Barometer)
    int32_t baroAltitudeCm = 0;     // FC-fused altitude above home, cm
    int16_t baroVarioCmPerSec = 0;     // vertical speed (vario), cm/s
    bool    baroValid = false; // false until first successful parse

    // Diagnostics
    double   lastRttMs = 0.0;   // last measured round-trip time
    double   fcCycleMs = 0.0;   // FC internal loop time (from MSP_DEBUG)
    bool     linkHealthy = false; // worker sets false on consecutive failures
    uint32_t packetCount = 0;     // total successful packets received
};

// ─────────────────────────────────────────────────────────────────────────────
// DroneLink
//
// One object, one serial port, one worker thread.
// Python calls connect() once; the worker handles all serial I/O from that
// point on.  Python never touches the serial port directly.
// ─────────────────────────────────────────────────────────────────────────────
class DroneLink {
public:
    // Default poll interval (ms). Lower = more responsive, higher CPU.
    // 10 ms → 100 Hz max, well within MPU-6500's 1 kHz native rate.
    static constexpr int POLL_INTERVAL_MS = 10;

    // How many consecutive failed packets before linkHealthy = false
    static constexpr int FAIL_THRESHOLD = 5;

    DroneLink();
    ~DroneLink();

    // ── Connection ────────────────────────────────────────────────────────────
    // Opens the serial port AND starts the worker thread.
    // portName: "COM3" style (no "\\\\.\\")
    bool connect(const std::string& portName);

    // Stops the worker thread and closes the port cleanly.
    void disconnect();

    bool isConnected() const { return connected.load(); }

    // ── Data access (call from Python / GUI thread) ───────────────────────────
    // Returns a snapshot copy — safe to call from any thread.
    DroneState getLatestState();

    // ── Poll interval tuning (call before connect, or at runtime) ────────────
    void setPollIntervalMs(int ms) { pollIntervalMs.store(ms); }

private:
    // ── Serial port ──────────────────────────────────────────────────────────
    HANDLE            hSerial;
    std::atomic<bool> connected;

    // ── Worker thread ────────────────────────────────────────────────────────
    std::thread       workerThread;
    std::atomic<bool> keepRunning;
    std::atomic<int>  pollIntervalMs;

    // ── Shared state (guarded by dataMutex) ──────────────────────────────────
    mutable std::mutex dataMutex;
    DroneState         currentState;

    // ── Internal helpers ─────────────────────────────────────────────────────

    // Main loop running on workerThread
    void communicationLoop();

    // Build and send an MSP request packet, return raw response bytes.
    // Called ONLY from workerThread.
    std::vector<uint8_t> sendMSP(uint8_t mspID);

    // Parse individual MSP responses and update fields on pendingState.
    // Returns true if the response was valid for that command.
    bool parseIMU(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAttitude(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAnalog(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseDebug(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseBaro(const std::vector<uint8_t>& buf, DroneState& s);  // BMP280

    // Atomically push a fully-populated state snapshot to currentState.
    void commitState(const DroneState& s);
};

// ─────────────────────────────────────────────────────────────────────────────
// Free function — scan COM1-COM29, return first that opens.
// Returns "NOT_FOUND" if nothing responds.
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoDetectF405();