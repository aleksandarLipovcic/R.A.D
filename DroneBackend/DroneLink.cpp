#include "DroneLink.h"
#include "BaroBMP280.h"
#include "GPSNeoM10.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>

// =============================================================================
// Constructor / Destructor
// =============================================================================

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

// =============================================================================
// connect()
// =============================================================================

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

    // Baud / framing
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    // Disable hardware flow control -- CP210x / CH340 chips can enable it by
    // default, which stalls reads on certain FC boards.
    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    if (!SetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }

    // MAXDWORD+MAXDWORD+1 makes ReadFile return immediately with whatever bytes
    // are currently in the driver buffer (non-blocking per-byte read).
    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 1;
    to.WriteTotalTimeoutConstant = 50;
    to.WriteTotalTimeoutMultiplier = 2;
    SetCommTimeouts(hSerial, &to);

    connected.store(true);
    keepRunning.store(true);
    workerThread = std::thread(&DroneLink::communicationLoop, this);

    return true;
}

// =============================================================================
// disconnect()
// =============================================================================

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

// =============================================================================
// getLatestState()
// =============================================================================

DroneState DroneLink::getLatestState() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentState;
}

// =============================================================================
// startMagCalibration() / startAccCalibration()
// Called from the Python thread -- sets atomic flag; worker picks it up.
// =============================================================================

void DroneLink::startMagCalibration() {
    magCalRequested.store(true);
}

void DroneLink::startAccCalibration() {
    accCalRequested.store(true);
}

// -- MSP_NAV_STATUS (121) -----------------------------------------------------
bool DroneLink::parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    return GPSNeoM10::parseNavStatus(buf, s.navStatus);
}

// =============================================================================
// communicationLoop()
//
// Poll order per loop tick (8 MSP commands):
//   1. MSP_STATUS   (101) -- FC cycle time, sensor status flags
//   2. MSP_RAW_IMU  (102) -- raw accel + gyro (MPU-6500)
//   3. MSP_ATTITUDE (108) -- fused roll / pitch / yaw
//   4. MSP_ANALOG   (110) -- battery voltage, RSSI
//   5. MSP_DEBUG    (254) -- mag X/Y/Z when debug_mode = MAG_CALIB
//   6. MSP_ALTITUDE (109) -- BMP280 altitude + vario
//   7. MSP_RAW_GPS  (106) -- GPS fix, sats, lat/lon, alt, speed, course, HDOP
//   8. MSP_COMP_GPS (107) -- distance + bearing to home, GPS heartbeat
//
// GPS PREREQUISITES (Betaflight Configurator -- one time):
//   Configuration -> Other Features: enable GPS
//   Ports tab: assign the GPS module UART to "GPS" function
//   Configuration -> GPS: set Provider (UBLOX recommended for Neo-M10)
// =============================================================================

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

        // -- Magnetometer calibration request ---------------------------------
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

        // -- Gyro/Accel calibration request -----------------------------------
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

        // -- 1. Status --------------------------------------------------------
        {
            auto buf = sendMSP(MSP::STATUS);
            parseStatus(buf, pending);
        }

        // -- 2. IMU -----------------------------------------------------------
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

        // -- 3. Attitude ------------------------------------------------------
        {
            auto buf = sendMSP(MSP::ATTITUDE);
            parseAttitude(buf, pending);
        }

        // -- 4. Analog --------------------------------------------------------
        {
            auto buf = sendMSP(MSP::ANALOG);
            parseAnalog(buf, pending);
        }

        // -- 5. Debug -> Magnetometer -----------------------------------------
        {
            auto buf = sendMSP(MSP::DEBUG);
            parseDebug(buf, pending);
        }

        // -- 6. Barometer -----------------------------------------------------
        {
            auto buf = sendMSP(MSP::ALTITUDE);
            parseBaro(buf, pending);
        }

        // -- 7. GPS Raw (MSP_RAW_GPS 106) -------------------------------------
        {
            auto buf = sendMSP(MSP::RAW_GPS);
            parseGPSRaw(buf, pending);
        }

        // -- 8. GPS Computed (MSP_COMP_GPS 107) -------------------------------
        {
            auto buf = sendMSP(MSP::COMP_GPS);
            parseGPSComp(buf, pending);
        }

        // -- 9. GPS Nav Status (MSP_NAV_STATUS 121) -----------------------------------
        {
            auto buf = sendMSP(MSP::NAV_STATUS);
            parseNavStatus(buf, pending);
        }

        // -- 10. SV Info via UBX passthrough (polled at 1 Hz, not 100 Hz) -----------
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastSvPollTime_).count();
            if (elapsed >= 1000) {
                lastSvPollTime_ = now;
                auto ubxPoll = GPSNeoM10::buildNavSvInfoPoll();
                auto mspFrame = GPSNeoM10::wrapUbxInMspPassthrough(ubxPoll);
                // Write the passthrough frame directly
                DWORD written = 0;
                PurgeComm(hSerial, PURGE_RXCLEAR);
                WriteFile(hSerial, mspFrame.data(),
                    static_cast<DWORD>(mspFrame.size()), &written, NULL);
                // Read the UBX response (variable length, needs bigger buffer)
                auto ubxResp = readUbxResponse(200);
                if (ubxResp.size() > 8) {
                    // Strip the 6-byte UBX header to get the payload
                    std::vector<uint8_t> payload(ubxResp.begin() + 6,
                        ubxResp.end() - 2);  // -2 = checksum
                    std::vector<SVInfoEntry> sv;
                    if (GPSNeoM10::parseNavSvInfo(payload, sv)) {
                        pending.svList = std::move(sv);
                        pending.svInfoValid = true;
                    }
                }
            }
        }

        // -- Health tracking --------------------------------------------------
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

// =============================================================================
// sendMSP()
//
// Builds and sends a MSP v1 request frame, reads the response byte-by-byte
// until the full frame arrives or the 80 ms deadline fires.
//
// PurgeComm(PURGE_RXCLEAR) before every write flushes stale bytes from the
// previous command, preventing cross-command frame contamination.
// =============================================================================

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    PurgeComm(hSerial, PURGE_RXCLEAR);

    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL) || written != sizeof(req))
        return {};

    constexpr size_t MAX_FRAME = 512;   // was 64 — UBX-NAV-SVINFO needs 400+
    uint8_t raw[MAX_FRAME];
    size_t  total = 0;
    int     payloadLen = -1;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);

    while (total < MAX_FRAME) {
        if (std::chrono::steady_clock::now() > deadline) break;

        DWORD   rd = 0;
        uint8_t b = 0;
        if (!ReadFile(hSerial, &b, 1, &rd, NULL) || rd == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(150));
            continue;
        }

        raw[total++] = b;
        if (total == 4) payloadLen = raw[3];

        // Full frame = preamble(3) + len(1) + cmd(1) + payload(N) + csum(1)
        if (payloadLen >= 0 &&
            total == static_cast<size_t>(payloadLen + 6))
            break;
    }

    if (total < 6) return {};
    return std::vector<uint8_t>(raw, raw + total);
}

std::vector<uint8_t> DroneLink::readUbxResponse(int timeoutMs) {
    std::vector<uint8_t> buf;
    buf.reserve(512);
    int     payloadLen = -1;
    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    while (buf.size() < 512) {
        if (std::chrono::steady_clock::now() > deadline) break;
        DWORD   rd = 0;
        uint8_t b = 0;
        if (!ReadFile(hSerial, &b, 1, &rd, NULL) || rd == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(150));
            continue;
        }
        buf.push_back(b);

        // UBX length field is at bytes [4..5] (LE uint16)
        if (buf.size() == 6)
            payloadLen = static_cast<int>(buf[4]) | (static_cast<int>(buf[5]) << 8);

        // Full UBX frame: 6 header + payload + 2 checksum
        if (payloadLen >= 0 &&
            buf.size() == static_cast<size_t>(payloadLen + 8))
            break;
    }
    return buf;
}

GPSConfigResult DroneLink::applyGPSConfig(const GPSConfig& cfg) {
    GPSConfigResult result;
    if (!connected.load()) {
        result.errorDetail = "Not connected";
        return result;
    }

    auto sendUbx = [&](const std::vector<uint8_t>& ubx,
        uint8_t cls, uint8_t id) -> bool {
            auto frame = GPSNeoM10::wrapUbxInMspPassthrough(ubx);
            DWORD written = 0;
            PurgeComm(hSerial, PURGE_RXCLEAR);
            if (!WriteFile(hSerial, frame.data(),
                static_cast<DWORD>(frame.size()), &written, NULL))
                return false;
            auto resp = readUbxResponse(300);
            return GPSNeoM10::parseAck(resp, cls, id);
        };

    result.gnssAck = sendUbx(GPSNeoM10::buildCfgGNSS(cfg.constellations), 0x06, 0x3E);
    result.rateAck = sendUbx(GPSNeoM10::buildCfgRate(cfg.updateRateHz), 0x06, 0x08);
    result.protocolAck = sendUbx(GPSNeoM10::buildCfgPrt(cfg.protocol), 0x06, 0x00);
    // Nav5 (elevation mask + airborne model)
    bool nav5Ok = sendUbx(GPSNeoM10::buildCfgNav5(cfg.elevationMaskDeg), 0x06, 0x24);
    result.saveAck = sendUbx(GPSNeoM10::buildCfgCfg(), 0x06, 0x09);

    result.overallOk = result.gnssAck && result.rateAck
        && result.protocolAck && nav5Ok && result.saveAck;
    if (!result.overallOk) {
        result.errorDetail = "ACK failures:";
        if (!result.gnssAck)     result.errorDetail += " CFG-GNSS";
        if (!result.rateAck)     result.errorDetail += " CFG-RATE";
        if (!result.protocolAck) result.errorDetail += " CFG-PRT";
        if (!nav5Ok)             result.errorDetail += " CFG-NAV5";
        if (!result.saveAck)     result.errorDetail += " CFG-CFG";
    }
    return result;
}

// =============================================================================
// Parsers
// =============================================================================

// -- MSP_STATUS (101) ---------------------------------------------------------
// [5..6] uint16_t cycleTime (us)
bool DroneLink::parseStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 11 || buf[4] != MSP::STATUS) return false;
    uint16_t cycleUs = static_cast<uint16_t>(buf[5] | (buf[6] << 8));
    s.fcCycleMs = cycleUs / 1000.0;
    return true;
}

// -- MSP_RAW_IMU (102) --------------------------------------------------------
// Payload: 9 x int16_t = 18 bytes (ax ay az gx gy gz mx my mz)
// We use ax..gz (first 6); mag comes from MSP_DEBUG.
bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.ax = r16(5);  s.ay = r16(7);  s.az = r16(9);
    s.gx = r16(11); s.gy = r16(13); s.gz = r16(15);
    return true;
}

// -- MSP_ATTITUDE (108) -------------------------------------------------------
// Payload: roll(int16) pitch(int16) yaw(int16) = 6 bytes
// roll and pitch are degrees x10; yaw is full degrees.
bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ATTITUDE) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8));
        };
    s.roll = r16(5);
    s.pitch = r16(7);
    s.yaw = r16(9);
    return true;
}

// -- MSP_ANALOG (110) ---------------------------------------------------------
// [5] vbat x10, [6..7] mAhDrawn, [8] rssi
bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::ANALOG) return false;
    s.batteryVoltage = buf[5] / 10.0f;
    s.rssi = buf[8];
    return true;
}

// -- MSP_DEBUG (254) -> Magnetometer X / Y / Z --------------------------------
// PREREQUISITE: set debug_mode = MAG_CALIB in BF CLI.
// debug[0]=magX, debug[1]=magY, debug[2]=magZ (int16, ADC counts)
bool DroneLink::parseDebug(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::DEBUG) return false;

    auto r16s = [&](size_t i) -> int16_t {
        return static_cast<int16_t>(
            static_cast<uint16_t>(buf[i]) |
            (static_cast<uint16_t>(buf[i + 1]) << 8));
        };

    int16_t mx = r16s(5);
    int16_t my = r16s(7);
    int16_t mz = r16s(9);

    s.magX = mx;
    s.magY = my;
    s.magZ = mz;

    float heading = std::atan2f(static_cast<float>(my), static_cast<float>(mx))
        * (180.0f / 3.14159265358979f);
    if (heading < 0.0f) heading += 360.0f;
    s.magHeadingDeg = heading;

    // All-zero means debug_mode != MAG_CALIB, or mag sensor not detected.
    s.magValid = (mx != 0 || my != 0 || mz != 0);
    return true;
}

// -- MSP_ALTITUDE (109) -------------------------------------------------------
// Decoded by BaroBMP280::parse() -- see BaroBMP280.h for layout.
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

// -- MSP_RAW_GPS (106) --------------------------------------------------------
//
// Thin wrapper around GPSNeoM10::parseRaw().
// Writes directly into the DroneState::gps sub-object (GPSReading).
//
// GPSNeoM10::parseRaw() handles both the 16-byte (BF < 4.1) and 18-byte
// (BF >= 4.1, includes HDOP) payload variants.
//
// On success:
//   s.gps.rawValid       = true
//   s.gps.positionUsable = true  when fixType>=2, numSat>=4, HDOP<5.0
//
// On failure:
//   s.gps.rawValid       = false  (no change to other gps fields)
bool DroneLink::parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseRaw(buf, s.gps)) {
        s.gps.rawValid = false;
        return false;
    }
    return true;
}

// -- MSP_COMP_GPS (107) -------------------------------------------------------
//
// Thin wrapper around GPSNeoM10::parseComp().
// Writes distToHomM, bearingToHome, gpsHeartbeat into DroneState::gps.
//
// Home point is set by the FC at arming (requires a valid 3D fix at that time).
// distToHomM and bearingToHome are 0 before arming or before a fix is acquired.
//
// gpsHeartbeat toggles 0<->1 on every fresh GPS frame. XOR with the previous
// value to detect arrival of new GPS data:
//   if s.gps.gpsHeartbeat != prev_heartbeat: <new GPS frame available>
bool DroneLink::parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseComp(buf, s.gps)) {
        s.gps.compValid = false;
        return false;
    }
    return true;
}

// =============================================================================
// commitState()
// =============================================================================

void DroneLink::commitState(const DroneState& s) {
    std::lock_guard<std::mutex> lock(dataMutex);
    currentState = s;
}

// =============================================================================
// AutoDetectF405() -- scan COM1-COM29, return first port that opens
// =============================================================================

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
