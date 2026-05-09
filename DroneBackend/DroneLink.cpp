#include "DroneLink.h"
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
    // Short absolute timeout so the worker loop can stay responsive and detect
    // link loss quickly.  50 ms total read timeout is generous for 115200 baud.
    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout = 10;   // ms between characters
    to.ReadTotalTimeoutConstant = 50;   // ms base
    to.ReadTotalTimeoutMultiplier = 2;    // ms per byte requested
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
// disconnect() — signal worker to stop, join it, close port
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
// getLatestState() — safe snapshot for the Python / GUI thread
// ─────────────────────────────────────────────────────────────────────────────

DroneState DroneLink::getLatestState() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentState;   // value copy — caller owns it
}

// ─────────────────────────────────────────────────────────────────────────────
// communicationLoop() — runs on workerThread
//
// Design intent:
//   • One iteration = request every active sensor in sequence.
//   • Each sendMSP() call is synchronous (write → read) but only this thread
//     ever touches hSerial, so there is no contention.
//   • After all sensors are polled, a single mutex lock commits the new state.
//   • The thread then sleeps for the remainder of the poll interval, keeping
//     CPU usage low and making the interval easy to tune from Python.
//
// Adding a new sensor later:
//   1. Add an MSP ID to the MSP namespace in the header.
//   2. Add a parseXxx() method declaration in the header.
//   3. Call sendMSP() + parseXxx() in the block below marked "SENSOR POLL".
//   4. Add the new fields to DroneState.
//   That's it — no threading changes required.
// ─────────────────────────────────────────────────────────────────────────────

void DroneLink::communicationLoop() {
    int consecutiveFails = 0;

    while (keepRunning.load()) {
        auto loopStart = std::chrono::steady_clock::now();

        // Accumulate new readings into a local copy so we hold the mutex for
        // the shortest possible time (one swap at the end, not per-sensor).
        DroneState pending;
        {
            // Seed with the previous state so fields we don't poll this tick
            // keep their last known value rather than resetting to zero.
            std::lock_guard<std::mutex> lock(dataMutex);
            pending = currentState;
        }

        bool anySuccess = false;

        // ── SENSOR POLL ──────────────────────────────────────────────────────
        // Each block is independent. A failure on one sensor does not skip
        // the others, so a missing GPS won't break IMU data.

        // 1. IMU (MPU-6500 accel + gyro) — highest priority, poll every tick
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

        // 4. FC internal cycle time via MSP_DEBUG
        //    Requires `set debug_mode = CYCLETIME` in Betaflight CLI.
        {
            auto buf = sendMSP(MSP::DEBUG);
            parseDebug(buf, pending);
        }

        // ── ADD FUTURE SENSORS HERE ──────────────────────────────────────────
        // Example (uncomment when GPS is wired):
        // {
        //     auto buf = sendMSP(MSP::GPS);
        //     parseGPS(buf, pending);
        // }
        // ─────────────────────────────────────────────────────────────────────

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

        // ── Commit ───────────────────────────────────────────────────────────
        commitState(pending);

        // ── Sleep remainder of poll interval ─────────────────────────────────
        // This keeps the loop at ~pollIntervalMs cadence regardless of how
        // long the sensor queries took, and avoids busy-spinning.
        auto elapsed = std::chrono::steady_clock::now() - loopStart;
        auto budget = std::chrono::milliseconds(pollIntervalMs.load());
        if (elapsed < budget)
            std::this_thread::sleep_for(budget - elapsed);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// sendMSP() — build MSP v1 request, write, read response
// Called ONLY from workerThread; no mutex needed.
// ─────────────────────────────────────────────────────────────────────────────

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    // MSP v1 request frame: $ M < <size=0> <cmd> <checksum>
    // For requests with no payload, checksum = cmd XOR 0 = cmd.
    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL) || written != sizeof(req))
        return {};

    // Read up to 64 bytes. The FC will reply with:
    // $ M > <size> <cmd> [payload...] <checksum>
    uint8_t buf[64];
    DWORD   rd = 0;
    if (!ReadFile(hSerial, buf, sizeof(buf), &rd, NULL) || rd < 6)
        return {};

    return std::vector<uint8_t>(buf, buf + rd);
}

// ─────────────────────────────────────────────────────────────────────────────
// Parsers — each validates the response header and extracts fields.
// Return true on success, false on short/malformed packet.
// ─────────────────────────────────────────────────────────────────────────────

bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    // Expected: $ M > 12 102 [6×int16] checksum  → minimum 18 bytes
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;

    // Bytes are little-endian int16 pairs starting at index 5
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
    // MSP_ATTITUDE: $ M > 6 108 [roll pitch yaw] checksum → 12 bytes
    if (buf.size() < 12 || buf[4] != MSP::ATTITUDE) return false;

    auto read16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.roll = read16(5);   // degrees × 10
    s.pitch = read16(7);   // degrees × 10
    s.yaw = read16(9);   // degrees (already ×1 on most FC builds)
    return true;
}

bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    // MSP_ANALOG: $ M > 7 110 [vbat mah_l mah_h rssi pow_l pow_h pow2] → 14 bytes
    if (buf.size() < 14 || buf[4] != MSP::ANALOG) return false;

    // vbat is in units of 0.1 V
    s.batteryVoltage = buf[5] / 10.0f;
    // RSSI is at byte 8 (0-255)
    s.rssi = buf[8];
    return true;
}

bool DroneLink::parseDebug(const std::vector<uint8_t>& buf, DroneState& s) {
    // MSP_DEBUG with CYCLETIME mode: debug[0] = FC loop time in microseconds
    // Frame: $ M > 8 254 [d0_l d0_h d1_l d1_h ...] → minimum 14 bytes
    if (buf.size() < 14 || buf[4] != MSP::DEBUG) return false;

    uint16_t cycleUs = static_cast<uint16_t>(buf[5] | (buf[6] << 8));
    s.fcCycleMs = cycleUs / 1000.0;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// commitState() — swap pending state into shared state under lock
// ─────────────────────────────────────────────────────────────────────────────

void DroneLink::commitState(const DroneState& s) {
    std::lock_guard<std::mutex> lock(dataMutex);
    currentState = s;
}

// ─────────────────────────────────────────────────────────────────────────────
// AutoDetectF405() — scan COM1-COM29, return first that opens
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