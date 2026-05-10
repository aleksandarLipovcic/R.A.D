#include "DroneLink.h"
#include "BaroBMP280.h"
#include <iostream>
#include <chrono>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

DroneLink::DroneLink()
    : hSerial(INVALID_HANDLE_VALUE)
    , connected(false)
    , keepRunning(false)
    , pollIntervalMs(POLL_INTERVAL_MS)
{
}

DroneLink::~DroneLink() {
    disconnect();
}

// ─────────────────────────────────────────────────────────────────────────────
// connect() — open serial port, configure it, start worker thread
// ─────────────────────────────────────────────────────────────────────────────

bool DroneLink::connect(const std::string& portName) {
    if (connected.load()) disconnect();

    std::string fullPath = "\\\\.\\" + portName;

    hSerial = CreateFileA(
        fullPath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) return false;

    // ── Baud / framing ───────────────────────────────────────────────────────
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    // ── Timeouts ─────────────────────────────────────────────────────────────
    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout = 10;
    to.ReadTotalTimeoutConstant = 50;
    to.ReadTotalTimeoutMultiplier = 2;
    to.WriteTotalTimeoutConstant = 50;
    to.WriteTotalTimeoutMultiplier = 2;
    SetCommTimeouts(hSerial, &to);

    // ── Start worker ─────────────────────────────────────────────────────────
    connected.store(true);
    keepRunning.store(true);
    workerThread = std::thread(&DroneLink::communicationLoop, this);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// disconnect()
// ─────────────────────────────────────────────────────────────────────────────

void DroneLink::disconnect() {
    keepRunning.store(false);
    connected.store(false);

    if (workerThread.joinable())
        workerThread.join();

    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// getLatestState()
// ─────────────────────────────────────────────────────────────────────────────

DroneState DroneLink::getLatestState() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentState;
}

// ─────────────────────────────────────────────────────────────────────────────
// communicationLoop() — runs on workerThread
// ─────────────────────────────────────────────────────────────────────────────

void DroneLink::communicationLoop() {
    int consecutiveFails = 0;

    while (keepRunning.load()) {
        auto loopStart = std::chrono::steady_clock::now();

        DroneState pending;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            pending = currentState;
        }

        bool anySuccess = false;

        // ── SENSOR POLL ──────────────────────────────────────────────────────

        // 1. IMU (MPU-6500 accel + gyro)
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            auto buf = sendMSP(MSP::RAW_IMU);
            auto t1 = std::chrono::high_resolution_clock::now();

            if (parseIMU(buf, pending)) {
                std::chrono::duration<double, std::milli> rtt = t1 - t0;
                pending.lastRttMs = rtt.count();
                anySuccess = true;
            }
        }

        // 2. Attitude (roll / pitch / yaw from FC fusion)
        {
            auto buf = sendMSP(MSP::ATTITUDE);
            parseAttitude(buf, pending);
        }

        // 3. Analog (battery voltage, RSSI)
        {
            auto buf = sendMSP(MSP::ANALOG);
            parseAnalog(buf, pending);
        }

        // 4. FC internal cycle time
        //    Requires: set debug_mode = CYCLETIME  in Betaflight CLI
        {
            auto buf = sendMSP(MSP::DEBUG);
            parseDebug(buf, pending);
        }

        // 5. Barometer — BMP280 altitude + vertical speed via MSP_ALTITUDE
        //    Requires: Barometer toggle ON in Betaflight Configurator
        //              (Configuration tab -> Sensors -> Barometer)
        {
            auto buf = sendMSP(MSP::ALTITUDE);
            parseBaro(buf, pending);
        }

        // ── ADD FUTURE SENSORS HERE ──────────────────────────────────────────

        // ── Health tracking ──────────────────────────────────────────────────
        if (anySuccess) {
            consecutiveFails = 0;
            pending.linkHealthy = true;
            pending.packetCount++;
        }
        else {
            consecutiveFails++;
            if (consecutiveFails >= FAIL_THRESHOLD)
                pending.linkHealthy = false;
        }

        commitState(pending);

        auto elapsed = std::chrono::steady_clock::now() - loopStart;
        auto budget = std::chrono::milliseconds(pollIntervalMs.load());
        if (elapsed < budget)
            std::this_thread::sleep_for(budget - elapsed);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// sendMSP()
// ─────────────────────────────────────────────────────────────────────────────

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL) || written != sizeof(req))
        return {};

    uint8_t buf[64];
    DWORD   rd = 0;
    if (!ReadFile(hSerial, buf, sizeof(buf), &rd, NULL) || rd < 6)
        return {};

    return std::vector<uint8_t>(buf, buf + rd);
}

// ─────────────────────────────────────────────────────────────────────────────
// Parsers
// ─────────────────────────────────────────────────────────────────────────────

bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;

    auto read16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.ax = read16(5);
    s.ay = read16(7);
    s.az = read16(9);
    s.gx = read16(11);
    s.gy = read16(13);
    s.gz = read16(15);
    return true;
}

bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ATTITUDE) return false;

    auto read16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.roll = read16(5);   // degrees * 10
    s.pitch = read16(7);   // degrees * 10
    s.yaw = read16(9);   // degrees * 1
    return true;
}

bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::ANALOG) return false;

    s.batteryVoltage = buf[5] / 10.0f;   // units of 0.1 V
    s.rssi = buf[8];            // 0-255
    return true;
}

bool DroneLink::parseDebug(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::DEBUG) return false;

    uint16_t cycleUs = static_cast<uint16_t>(buf[5] | (buf[6] << 8));
    s.fcCycleMs = cycleUs / 1000.0;
    return true;
}

bool DroneLink::parseBaro(const std::vector<uint8_t>& buf, DroneState& s) {
    // MSP_ALTITUDE response layout:
    //   [0]      '$'
    //   [1]      'M'
    //   [2]      '>'
    //   [3]      0x06        payload size = 6 bytes
    //   [4]      0x6D (109)  MSP_ALTITUDE command ID
    //   [5..8]   int32_t     FC-fused altitude above home, cm  (little-endian)
    //   [9..10]  int16_t     vertical speed (vario), cm/s      (little-endian)
    //   [11]     checksum
    //
    // Betaflight fuses the BMP280 reading internally; we get the result here.
    // Requires Barometer enabled in Betaflight Configurator.
    BaroReading raw;
    if (!BaroBMP280::parse(buf, raw)) {
        s.baroValid = false;
        return false;
    }
    s.baroAltitudeCm = raw.altitudeCm;
    s.baroVarioCmPerSec = raw.varioCmPerSec;
    s.baroValid = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// commitState()
// ─────────────────────────────────────────────────────────────────────────────

void DroneLink::commitState(const DroneState& s) {
    std::lock_guard<std::mutex> lock(dataMutex);
    currentState = s;
}

// ─────────────────────────────────────────────────────────────────────────────
// AutoDetectF405()
// ─────────────────────────────────────────────────────────────────────────────

std::string AutoDetectF405() {
    for (int i = 1; i < 30; ++i) {
        std::string path = "\\\\.\\COM" + std::to_string(i);
        HANDLE h = CreateFileA(path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0, NULL, OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL, NULL);
        if (h != INVALID_HANDLE_VALUE) {
            CloseHandle(h);
            return "COM" + std::to_string(i);
        }
    }
    return "NOT_FOUND";
}