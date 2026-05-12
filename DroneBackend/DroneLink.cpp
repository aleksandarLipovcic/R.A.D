#include "DroneLink.h"
#include "BaroBMP280.h"
#include <cmath>
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
    , magCalRequested(false)
    , accCalRequested(false)
    , magCalActive_(false)
    , accCalActive_(false)
{
}

DroneLink::~DroneLink() {
    disconnect();
}

// ─────────────────────────────────────────────────────────────────────────────
// connect()
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
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    // Disable hardware flow control — CP210x / CH340 chips can enable it by
    // default, which stalls reads on certain FC boards.
    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    if (!SetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }

    // ── Timeouts ─────────────────────────────────────────────────────────────
    // MAXDWORD+MAXDWORD+1 → ReadFile returns immediately with whatever bytes
    // are currently in the driver buffer (non-blocking per-byte read).
    // Actual timing is driven by the 80 ms deadline loop in sendMSP().
    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 1;
    to.WriteTotalTimeoutConstant = 50;
    to.WriteTotalTimeoutMultiplier = 2;
    SetCommTimeouts(hSerial, &to);

    // ── Start background worker ───────────────────────────────────────────────
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
// startMagCalibration() / startAccCalibration()
// Called from Python thread — just sets the atomic flag; the worker picks it up.
// ─────────────────────────────────────────────────────────────────────────────

void DroneLink::startMagCalibration() {
    magCalRequested.store(true);
}

void DroneLink::startAccCalibration() {
    accCalRequested.store(true);
}

// ─────────────────────────────────────────────────────────────────────────────
// communicationLoop()
//
// Poll order — 6 MSP commands per loop tick:
//
//   1. MSP_STATUS   (101) — FC cycle time, sensor status flags
//   2. MSP_RAW_IMU  (102) — raw accel + gyro (MPU-6500)
//   3. MSP_ATTITUDE (108) — fused roll / pitch / yaw
//   4. MSP_ANALOG   (110) — battery voltage, RSSI
//   5. MSP_DEBUG    (254) — mag X/Y/Z when debug_mode = MAG_CALIB
//   6. MSP_ALTITUDE (109) — BMP280 altitude + vario
//
// Calibration commands (fire-and-forget, sent when requested):
//   MSP_ACC_CALIBRATION (205) — triggered by startAccCalibration()
//   MSP_MAG_CALIBRATION (206) — triggered by startMagCalibration()
//
// WHY MSP_DEBUG FOR MAG:
//   Betaflight has no MSP_RAW_MAG command. ID 130 = MSP_BATTERY_STATE.
//   The only way to get raw mag X/Y/Z over MSP is through the debug
//   subsystem with  set debug_mode = MAG_CALIB  in the BF CLI.
//   This is the same data source the BF Configurator Sensors tab uses.
//
// USER SETUP REQUIRED (Betaflight CLI — one time):
//   set mag_hardware = QMC5883   (or AUTO if no other mag present)
//   set debug_mode   = MAG_CALIB
//   save
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

        // ── Magnetometer calibration request ─────────────────────────────────
        // MSP_MAG_CALIBRATION (206) — fire-and-forget.
        // FC enters calibration mode for MAG_CAL_DURATION_S seconds.
        // User must rotate the drone on all three axes during this window.
        if (magCalRequested.exchange(false)) {
            sendMSP(MSP::MAG_CAL);
            magCalActive_ = true;
            magCalStartTime_ = std::chrono::steady_clock::now();
        }

        if (magCalActive_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - magCalStartTime_).count();
            int remaining = MAG_CAL_DURATION_S - static_cast<int>(elapsed);
            if (remaining <= 0) {
                magCalActive_ = false;
                pending.magCalActive = false;
                pending.magCalSecondsRemaining = 0;
            }
            else {
                pending.magCalActive = true;
                pending.magCalSecondsRemaining = remaining;
            }
        }
        else {
            pending.magCalActive = false;
            pending.magCalSecondsRemaining = 0;
        }

        // ── Gyro/Accel calibration request ────────────────────────────────────
        // MSP_ACC_CALIBRATION (205) — fire-and-forget.
        // Keep the drone perfectly level and still for ~5 seconds.
        if (accCalRequested.exchange(false)) {
            sendMSP(MSP::ACC_CAL);
            accCalActive_ = true;
            accCalStartTime_ = std::chrono::steady_clock::now();
        }

        if (accCalActive_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - accCalStartTime_).count();
            int remaining = ACC_CAL_DURATION_S - static_cast<int>(elapsed);
            if (remaining <= 0) {
                accCalActive_ = false;
                pending.accCalActive = false;
                pending.accCalSecondsRemaining = 0;
            }
            else {
                pending.accCalActive = true;
                pending.accCalSecondsRemaining = remaining;
            }
        }
        else {
            pending.accCalActive = false;
            pending.accCalSecondsRemaining = 0;
        }

        // ── 1. Status (FC cycle time + sensor flags) ──────────────────────────
        {
            auto buf = sendMSP(MSP::STATUS);
            parseStatus(buf, pending);
        }

        // ── 2. IMU (MPU-6500 accel + gyro) ───────────────────────────────────
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

        // ── 3. Attitude (roll / pitch / yaw from FC fusion) ───────────────────
        {
            auto buf = sendMSP(MSP::ATTITUDE);
            parseAttitude(buf, pending);
        }

        // ── 4. Analog (battery voltage, RSSI) ─────────────────────────────────
        {
            auto buf = sendMSP(MSP::ANALOG);
            parseAnalog(buf, pending);
        }

        // ── 5. Debug → Magnetometer X / Y / Z ────────────────────────────────
        // Requires:  set debug_mode = MAG_CALIB  in BF CLI.
        // When set, debug[0..2] = raw mag X/Y/Z (same as BF Configurator shows).
        // If debug_mode ≠ MAG_CALIB, parseDebug() will set magValid = false.
        {
            auto buf = sendMSP(MSP::DEBUG);
            parseDebug(buf, pending);
        }

        // ── 6. Barometer — BMP280 altitude + vario via MSP_ALTITUDE ──────────
        {
            auto buf = sendMSP(MSP::ALTITUDE);
            parseBaro(buf, pending);
        }

        // ── Health tracking ───────────────────────────────────────────────────
        if (anySuccess) {
            consecutiveFails = 0;
            pending.linkHealthy = true;
            pending.packetCount++;
        }
        else {
            if (++consecutiveFails >= FAIL_THRESHOLD)
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
//
// Builds and sends a MSP v1 request frame, then reads the response
// byte-by-byte until the full frame has arrived or the 80 ms deadline fires.
//
// Two correctness fixes vs naive single-ReadFile approach:
//   1. PurgeComm(PURGE_RXCLEAR) before every write flushes stale bytes from
//      the previous command's response, preventing cross-command contamination.
//   2. Byte-by-byte loop exits exactly at  payloadLen + 6  bytes, so partial
//      frames from loaded USB-serial drivers are never returned.
// ─────────────────────────────────────────────────────────────────────────────

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    PurgeComm(hSerial, PURGE_RXCLEAR);   // flush stale bytes from prev command

    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL) || written != sizeof(req))
        return {};

    constexpr size_t MAX_FRAME = 64;
    uint8_t raw[MAX_FRAME];
    size_t  total = 0;
    int     payloadLen = -1;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);

    while (total < MAX_FRAME) {
        if (std::chrono::steady_clock::now() > deadline)
            break;

        DWORD rd = 0;
        uint8_t b = 0;
        if (!ReadFile(hSerial, &b, 1, &rd, NULL) || rd == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(150));
            continue;
        }

        raw[total++] = b;
        if (total == 4)
            payloadLen = raw[3];

        // Full frame = preamble(3) + len(1) + cmd(1) + payload(N) + csum(1)
        if (payloadLen >= 0 &&
            total == static_cast<size_t>(payloadLen + 6))
            break;
    }

    if (total < 6) return {};
    return std::vector<uint8_t>(raw, raw + total);
}

// ─────────────────────────────────────────────────────────────────────────────
// Parsers
// ─────────────────────────────────────────────────────────────────────────────

// ── MSP_STATUS (101) ─────────────────────────────────────────────────────────
// Response layout (minimum 11 bytes):
//   [0..2]  $ M >
//   [3]     payload length (≥ 11 bytes in BF 4.x; typically 0x0B or more)
//   [4]     0x65 (101)
//   [5..6]  uint16_t  cycleTime   (µs)
//   [7..8]  uint16_t  i2cErrors
//   [9..10] uint16_t  activeSensors bitmask
//   ...     (more fields follow — we only need cycleTime here)
//   [last]  checksum
bool DroneLink::parseStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 11 || buf[4] != MSP::STATUS) return false;
    uint16_t cycleUs = static_cast<uint16_t>(buf[5] | (buf[6] << 8));
    s.fcCycleMs = cycleUs / 1000.0;
    return true;
}

// ── MSP_RAW_IMU (102) ────────────────────────────────────────────────────────
// Payload: 9 × int16_t = 18 bytes  (ax ay az gx gy gz mx my mz)
// We only use ax..gz (first 6 values); mag comes from MSP_DEBUG instead.
bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;
    auto read16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.ax = read16(5);  s.ay = read16(7);  s.az = read16(9);
    s.gx = read16(11); s.gy = read16(13); s.gz = read16(15);
    return true;
}

// ── MSP_ATTITUDE (108) ───────────────────────────────────────────────────────
// Payload: roll(int16) pitch(int16) yaw(int16) = 6 bytes
// roll and pitch are degrees × 10; yaw is full degrees.
bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ATTITUDE) return false;
    auto read16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.roll = read16(5);
    s.pitch = read16(7);
    s.yaw = read16(9);
    return true;
}

// ── MSP_ANALOG (110) ─────────────────────────────────────────────────────────
// Payload (BF 4.x, 9 bytes):
//   [5]     uint8_t   vbat × 10  (e.g. 126 = 12.6 V)
//   [6..7]  uint16_t  mAhDrawn
//   [8]     uint8_t   rssi (0–255)
bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::ANALOG) return false;
    s.batteryVoltage = buf[5] / 10.0f;
    s.rssi = buf[8];
    return true;
}

// ── MSP_DEBUG (254) → Magnetometer X / Y / Z ─────────────────────────────────
//
// PREREQUISITE — Betaflight CLI (one-time setup):
//   set debug_mode = MAG_CALIB
//   save
//
// Response layout (14 bytes total):
//   [0..2]   $ M >
//   [3]      0x08        payload = 8 bytes (4 × int16_t)
//   [4]      0xFE (254)  command ID
//   [5..6]   int16_t     debug[0] = raw mag X
//   [7..8]   int16_t     debug[1] = raw mag Y
//   [9..10]  int16_t     debug[2] = raw mag Z
//   [11..12] int16_t     debug[3] = heading error / calibration quality
//   [13]     checksum
//
// If debug_mode is anything other than MAG_CALIB, debug[0..2] will NOT
// contain mag data.  The parser detects this by checking if all three values
// are exactly zero — a legitimate "all-zero" mag reading is physically very
// unlikely and only occurs if the sensor is undetected or disabled.
// magValid is set false in that case so the GUI shows "NO SIG".
//
// fcCycleMs now comes from MSP_STATUS (101) above, so it is always available
// regardless of which debug_mode is active.
bool DroneLink::parseDebug(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::DEBUG) return false;

    auto read16s = [&](size_t i) -> int16_t {
        return static_cast<int16_t>(
            static_cast<uint16_t>(buf[i]) |
            (static_cast<uint16_t>(buf[i + 1]) << 8));
        };

    int16_t mx = read16s(5);
    int16_t my = read16s(7);
    int16_t mz = read16s(9);

    s.magX = mx;
    s.magY = my;
    s.magZ = mz;

    // Tilt-uncorrected 2-D heading from horizontal field components.
    // Accurate only when the drone is level.
    float heading = std::atan2f(static_cast<float>(my),
        static_cast<float>(mx))
        * (180.0f / 3.14159265358979f);
    if (heading < 0.0f) heading += 360.0f;
    s.magHeadingDeg = heading;

    // All-zero means either:
    //   a) debug_mode ≠ MAG_CALIB (wrong CLI setting), or
    //   b) mag sensor not detected / not enabled in BF configurator.
    // In both cases show "NO SIG" in the GUI.
    s.magValid = (mx != 0 || my != 0 || mz != 0);

    return true;
}

// ── MSP_ALTITUDE (109) ───────────────────────────────────────────────────────
// Decoded by BaroBMP280::parse() — see BaroBMP280.h for layout.
bool DroneLink::parseBaro(const std::vector<uint8_t>& buf, DroneState& s) {
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
// AutoDetectF405() — scan COM1–COM29, return first port that opens
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