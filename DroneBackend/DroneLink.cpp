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
    , gpsUartIndex_(0)
    , magCalRequested(false)
    , accCalRequested(false)
    , svPollTickCounter_(0)
    , slowPollTickCounter_(0)
    , magCalActive_(false)
    , accCalActive_(false)
{
}

DroneLink::~DroneLink() {
    disconnect();
}

// =============================================================================
// openSerialPort()
// =============================================================================

bool DroneLink::openSerialPort(const std::string& portName) {
    std::string fullPath = "\\\\.\\" + portName;

    hSerial = CreateFileA(
        fullPath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) return false;

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }

    dcb.BaudRate = MSP_BAUD;   // 57600 — confirmed by sv_baud_probe.py
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;

    if (!SetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }

    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 1;
    to.WriteTotalTimeoutConstant = 50;
    to.WriteTotalTimeoutMultiplier = 2;
    SetCommTimeouts(hSerial, &to);

    return true;
}

void DroneLink::closeSerialPort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
}

// =============================================================================
// connect() / disconnect()
// =============================================================================

bool DroneLink::connect(const std::string& portName) {
    if (connected.load()) disconnect();

    portName_ = portName;

    if (!openSerialPort(portName)) return false;

    connected.store(true);
    keepRunning.store(true);
    workerThread = std::thread(&DroneLink::communicationLoop, this);
    return true;
}

void DroneLink::disconnect() {
    keepRunning.store(false);
    connected.store(false);

    if (workerThread.joinable())
        workerThread.join();

    closeSerialPort();
}

DroneState DroneLink::getLatestState() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentState;
}

void DroneLink::startMagCalibration() { magCalRequested.store(true); }
void DroneLink::startAccCalibration() { accCalRequested.store(true); }

// =============================================================================
// communicationLoop()
//
// Poll order per 10 ms tick (100 Hz):
//   ── Standard MSP polls (every tick) ──
//   1.  MSP_STATUS       (101)  FC cycle time, flight mode, armed, sensor status
//   2.  MSP_RAW_IMU      (102)  raw accel + gyro (MPU-6500)
//   3.  MSP_ATTITUDE     (108)  fused roll / pitch / yaw
//   4.  MSP_ANALOG       (110)  battery voltage, current, mAh, RSSI
//   5.  MSP_DEBUG        (254)  mag X/Y/Z via debug_mode=MAG_CALIB
//   6.  MSP_ALTITUDE     (109)  BMP280 altitude + vario
//   7.  MSP_RAW_GPS      (106)  GPS fix, sats, lat/lon, alt, speed, course, HDOP
//   8.  MSP_COMP_GPS     (107)  distance + bearing to home, GPS heartbeat
//   9.  MSP_NAV_STATUS   (121)  GPS nav engine status + fix flags
//  10.  MSP_STATUS_EX    (150)  arming-disable flags (extended status)
//  11.  MSP_MOTOR        (104)  motor throttle outputs (1000–2000 µs)
//  12.  MSP_RC           (105)  RC channel inputs (1000–2000 µs)
//
//   ── Throttled polls ──
//  13.  MSP_GPS_SV_INFO  (164)  satellite list, every SV_POLL_TICKS   (~1 s)
//  14.  MSP_BATTERY_STATE(242)  full battery detail, every SLOW_POLL_TICKS (~500 ms)
//
// Budget note: 12 polls × ~1–2 ms each fits the 10 ms budget on F405 hardware.
// If overruns occur (link_healthy flapping), call setPollIntervalMs(20).
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

        // ── Magnetometer calibration request ──────────────────────────────────
        if (magCalRequested.exchange(false)) {
            sendMSP(MSP::MAG_CAL);
            magCalActive_ = true;
            magCalStartTime_ = std::chrono::steady_clock::now();
        }
        if (magCalActive_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - magCalStartTime_).count();
            int  remaining = MAG_CAL_DURATION_S - static_cast<int>(elapsed);
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
        if (accCalRequested.exchange(false)) {
            sendMSP(MSP::ACC_CAL);
            accCalActive_ = true;
            accCalStartTime_ = std::chrono::steady_clock::now();
        }
        if (accCalActive_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - accCalStartTime_).count();
            int  remaining = ACC_CAL_DURATION_S - static_cast<int>(elapsed);
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

        // ── Regular MSP polls ─────────────────────────────────────────────────
        { auto buf = sendMSP(MSP::STATUS);     parseStatus(buf, pending); }

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

        { auto buf = sendMSP(MSP::ATTITUDE);   parseAttitude(buf, pending); }
        { auto buf = sendMSP(MSP::ANALOG);     parseAnalog(buf, pending); }
        { auto buf = sendMSP(MSP::DEBUG);      parseDebug(buf, pending); }
        { auto buf = sendMSP(MSP::ALTITUDE);   parseBaro(buf, pending); }
        { auto buf = sendMSP(MSP::RAW_GPS);    parseGPSRaw(buf, pending); }
        { auto buf = sendMSP(MSP::COMP_GPS);   parseGPSComp(buf, pending); }
        { auto buf = sendMSP(MSP::NAV_STATUS); parseNavStatus(buf, pending); }

        // ── Extended telemetry — every tick ───────────────────────────────────
        //
        // STATUS_EX (150): arming-disable flags change infrequently but we poll
        // every tick so the GCS shows the reason immediately when it changes.
        //
        // MOTOR (104) / RC (105): polled every tick for real-time motor and
        // stick display, essential for autonomous-flight monitoring.
        { auto buf = sendMSP(MSP::STATUS_EX);  parseStatusEx(buf, pending); }
        { auto buf = sendMSP(MSP::MOTOR);      parseMotors(buf, pending); }
        { auto buf = sendMSP(MSP::RC);         parseRCChannels(buf, pending); }

        // ── Satellite list — MSP_GPS_SV_INFO (164), every ~1 s ───────────────
        if (svPollTickCounter_ <= 0) {
            pollSatellitesMSP(pending);
            svPollTickCounter_ = SV_POLL_TICKS;
        }
        --svPollTickCounter_;

        // ── Battery detail — MSP_BATTERY_STATE (242), every ~500 ms ──────────
        // Battery state changes slowly (seconds between WARNING/CRITICAL
        // transitions). Polling at 2 Hz reduces loop pressure while keeping
        // display values fresh.
        if (slowPollTickCounter_ <= 0) {
            auto buf = sendMSP(MSP::BATTERY_STATE);
            parseBatteryState(buf, pending);
            slowPollTickCounter_ = SLOW_POLL_TICKS;
        }
        --slowPollTickCounter_;

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

// =============================================================================
// pollSatellitesMSP()
//
// PRIMARY satellite polling path. Sends MSP_GPS_SV_INFO (cmd 164) and passes
// the response to GPSNeoM10::parseMspSvInfo(). No passthrough, no port cycle.
//
// On success: pending.svList is updated, pending.svInfoValid = true,
//             pending.svSource = "MSP".
// On failure: svList and svInfoValid are left unchanged (retain last good data).
// =============================================================================

bool DroneLink::pollSatellitesMSP(DroneState& pending) {
    auto buf = sendMSP(MSP::GPS_SV_INFO);   // cmd 164

    std::vector<SVInfoEntry> sv;
    if (!GPSNeoM10::parseMspSvInfo(buf, sv))
        return false;

    pending.svList = std::move(sv);
    pending.svInfoValid = true;
    pending.svSource = "MSP";
    return true;
}

// =============================================================================
// pollSatellites() — UBX passthrough path — RETAINED FOR applyGPSConfig()
//
// NOT called from communicationLoop(). sv_baud_probe.py confirmed BF returns
// 0 bytes on all UART indices with gps_auto_config=ON.
// =============================================================================

void DroneLink::pollSatellites(DroneState& pending) {
    if (hSerial == INVALID_HANDLE_VALUE) return;

    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    {
        uint8_t len = 0x01;
        uint8_t cmd = MSP::SET_PASSTHROUGH;
        uint8_t csum = static_cast<uint8_t>(len ^ cmd ^ gpsUartIndex_);
        uint8_t frame[7] = { '$', 'M', '<', len, cmd, gpsUartIndex_, csum };

        DWORD written = 0;
        if (!WriteFile(hSerial, frame, sizeof(frame), &written, NULL)
            || written != sizeof(frame)) {
            closeSerialPort();
            openSerialPort(portName_);
            return;
        }
    }

    {
        auto    deadline = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(150);
        uint8_t ack[6] = {};
        int     got = 0;

        while (got < 6 && std::chrono::steady_clock::now() < deadline) {
            DWORD rd = 0; uint8_t b = 0;
            if (ReadFile(hSerial, &b, 1, &rd, NULL) && rd == 1)
                ack[got++] = b;
            else
                std::this_thread::sleep_for(std::chrono::microseconds(200));
        }

        if (got < 6 || ack[0] != '$' || ack[1] != 'M' || ack[2] != '>') {
            closeSerialPort();
            openSerialPort(portName_);
            return;
        }
    }

    {
        auto  ubxPoll = GPSNeoM10::buildNavSvInfoPoll();
        DWORD written = 0;
        WriteFile(hSerial, ubxPoll.data(),
            static_cast<DWORD>(ubxPoll.size()), &written, NULL);
    }

    auto ubxResp = readUbxResponse(hSerial, 400);

    closeSerialPort();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    openSerialPort(portName_);

    if (ubxResp.size() < 10)                          return;
    if (ubxResp[2] != 0x01 || ubxResp[3] != 0x35)    return;

    uint16_t payLen =
        static_cast<uint16_t>(ubxResp[4]) |
        (static_cast<uint16_t>(ubxResp[5]) << 8);

    if (ubxResp.size() < static_cast<size_t>(8u + payLen)) return;

    std::vector<uint8_t> payload(ubxResp.begin() + 6,
        ubxResp.begin() + 6 + payLen);

    std::vector<SVInfoEntry> sv;
    if (GPSNeoM10::parseNavSvInfo(payload, sv)) {
        pending.svList = std::move(sv);
        pending.svInfoValid = true;
        pending.svSource = "UBX";
    }
}

// =============================================================================
// readUbxResponse()
// =============================================================================

std::vector<uint8_t> DroneLink::readUbxResponse(HANDLE h, int timeoutMs) {
    if (h == INVALID_HANDLE_VALUE) return {};

    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto readByte = [&](uint8_t& out) -> bool {
        while (std::chrono::steady_clock::now() < deadline) {
            DWORD rd = 0;
            if (ReadFile(h, &out, 1, &rd, NULL) && rd == 1) return true;
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        return false;
        };

    {
        uint8_t prev = 0, curr = 0;
        while (true) {
            if (!readByte(curr)) return {};
            if (prev == 0xB5 && curr == 0x62) break;
            prev = curr;
        }
    }

    uint8_t hdr[4] = {};
    for (int i = 0; i < 4; ++i)
        if (!readByte(hdr[i])) return {};

    uint16_t payloadLen =
        static_cast<uint16_t>(hdr[2]) |
        (static_cast<uint16_t>(hdr[3]) << 8);

    if (payloadLen > 2048) return {};

    std::vector<uint8_t> frame;
    frame.reserve(8u + payloadLen);
    frame.push_back(0xB5);
    frame.push_back(0x62);
    for (int i = 0; i < 4; ++i) frame.push_back(hdr[i]);

    for (uint32_t i = 0; i < static_cast<uint32_t>(payloadLen) + 2u; ++i) {
        uint8_t b = 0;
        if (!readByte(b)) return {};
        frame.push_back(b);
    }

    return frame;
}

// =============================================================================
// sendMSP()
// =============================================================================

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    PurgeComm(hSerial, PURGE_RXCLEAR);

    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD   written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL)
        || written != sizeof(req))
        return {};

    constexpr size_t MAX_FRAME = 512;
    uint8_t raw[MAX_FRAME];
    size_t  total = 0;
    int     payloadLen = -1;

    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(80);

    while (total < MAX_FRAME) {
        if (std::chrono::steady_clock::now() > deadline) break;

        DWORD rd = 0; uint8_t b = 0;
        if (!ReadFile(hSerial, &b, 1, &rd, NULL) || rd == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(150));
            continue;
        }

        raw[total++] = b;
        if (total == 4) payloadLen = raw[3];

        if (payloadLen >= 0 &&
            total == static_cast<size_t>(payloadLen + 6))
            break;
    }

    if (total < 6) return {};
    return std::vector<uint8_t>(raw, raw + total);
}

// =============================================================================
// applyGPSConfig()
// =============================================================================

GPSConfigResult DroneLink::applyGPSConfig(const GPSConfig& cfg) {
    GPSConfigResult result;
    if (!connected.load()) {
        result.errorDetail = "Not connected";
        return result;
    }

    auto sendUbxConfig = [&](const std::vector<uint8_t>& ubxFrame,
        uint8_t cls, uint8_t id) -> bool {
            if (hSerial == INVALID_HANDLE_VALUE) return false;

            PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

            uint8_t len = 0x01;
            uint8_t cmd = MSP::SET_PASSTHROUGH;
            uint8_t csum = static_cast<uint8_t>(len ^ cmd ^ gpsUartIndex_);
            uint8_t ptFrame[7] = { '$', 'M', '<', len, cmd, gpsUartIndex_, csum };
            DWORD   written = 0;
            if (!WriteFile(hSerial, ptFrame, sizeof(ptFrame), &written, NULL)
                || written != sizeof(ptFrame))
                return false;

            auto ackDl = std::chrono::steady_clock::now()
                + std::chrono::milliseconds(150);
            int  ackGot = 0;
            while (ackGot < 6 && std::chrono::steady_clock::now() < ackDl) {
                DWORD rd = 0; uint8_t b = 0;
                if (ReadFile(hSerial, &b, 1, &rd, NULL) && rd == 1) ++ackGot;
                else std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
            if (ackGot < 6) {
                closeSerialPort(); openSerialPort(portName_); return false;
            }

            written = 0;
            WriteFile(hSerial, ubxFrame.data(),
                static_cast<DWORD>(ubxFrame.size()), &written, NULL);

            auto resp = readUbxResponse(hSerial, 300);

            closeSerialPort();
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            openSerialPort(portName_);

            return GPSNeoM10::parseAck(resp, cls, id);
        };

    result.gnssAck = sendUbxConfig(GPSNeoM10::buildCfgGNSS(cfg.constellations), 0x06, 0x3E);
    result.rateAck = sendUbxConfig(GPSNeoM10::buildCfgRate(cfg.updateRateHz), 0x06, 0x08);
    result.protocolAck = sendUbxConfig(GPSNeoM10::buildCfgPrt(cfg.protocol), 0x06, 0x00);
    bool nav5Ok = sendUbxConfig(GPSNeoM10::buildCfgNav5(cfg.elevationMaskDeg), 0x06, 0x24);
    result.saveAck = sendUbxConfig(GPSNeoM10::buildCfgCfg(), 0x06, 0x09);

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

// -----------------------------------------------------------------------------
// parseStatus() — MSP_STATUS (101)
//
// Payload layout (BF 4.5.x, all little-endian), offsets from buf[5]:
//   [0..1]  uint16  cycleTime           µs  → fcCycleMs
//   [2..3]  uint16  i2cErrorCount           → i2cErrorCount
//   [4..5]  uint16  sensorStatus            → sensorStatus  (SensorStatus:: bits)
//   [6..9]  uint32  flightModeFlags         → flightModeFlags (FlightMode:: bits)
//   [10]    uint8   currentPidProfile       → pidProfile
//   [11..12]uint16  averageSystemLoad %     → cpuLoadPercent
//
// Derives: armed (ARM bit in flightModeFlags), flightModeName (via decode helper)
//
// Min frame: header(5) + payload(13) + checksum(1) = 19 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 19 || buf[4] != MSP::STATUS) return false;

    auto ru16 = [&](int i) -> uint16_t {
        return static_cast<uint16_t>(buf[i]) |
            (static_cast<uint16_t>(buf[i + 1]) << 8);
        };
    auto ru32 = [&](int i) -> uint32_t {
        return static_cast<uint32_t>(buf[i]) |
            (static_cast<uint32_t>(buf[i + 1]) << 8) |
            (static_cast<uint32_t>(buf[i + 2]) << 16) |
            (static_cast<uint32_t>(buf[i + 3]) << 24);
        };

    s.fcCycleMs = ru16(5) / 1000.0;
    s.i2cErrorCount = ru16(7);
    s.sensorStatus = ru16(9);
    s.flightModeFlags = ru32(11);
    s.pidProfile = buf[15];
    s.cpuLoadPercent = ru16(16);

    s.armed = (s.flightModeFlags & FlightMode::ARM) != 0;
    s.flightModeName = decodeFlightMode(s.flightModeFlags);

    return true;
}

// -----------------------------------------------------------------------------
// parseIMU() — MSP_RAW_IMU (102)
// -----------------------------------------------------------------------------

bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8)); };
    s.ax = r16(5); s.ay = r16(7);  s.az = r16(9);
    s.gx = r16(11); s.gy = r16(13); s.gz = r16(15);
    return true;
}

// -----------------------------------------------------------------------------
// parseAttitude() — MSP_ATTITUDE (108)
// -----------------------------------------------------------------------------

bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ATTITUDE) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8)); };
    s.roll = r16(5);
    s.pitch = r16(7);
    s.yaw = r16(9);
    return true;
}

// -----------------------------------------------------------------------------
// parseAnalog() — MSP_ANALOG (110)
//
// Payload layout (BF 4.5.x):
//   buf[5]     uint8   vbat       0.1 V units
//   buf[6..7]  uint16  mAhDrawn   mAh consumed
//   buf[8]     uint8   rssi       0–255
//   buf[9..10] int16   amperage   centiamps (divide by 100 → Amps)
//
// Min frame: header(5) + payload(6) + checksum(1) = 12 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ANALOG) return false;

    s.batteryVoltage = buf[5] / 10.0f;

    s.batteryMahDrawn = static_cast<uint16_t>(buf[6]) |
        (static_cast<uint16_t>(buf[7]) << 8);

    s.rssi = buf[8];

    // Amperage: signed int16 in centiamps; 100 cA = 1 A
    int16_t centiAmps = static_cast<int16_t>(
        static_cast<uint16_t>(buf[9]) |
        (static_cast<uint16_t>(buf[10]) << 8));
    s.batteryCurrent = centiAmps / 100.0f;

    return true;
}

// -----------------------------------------------------------------------------
// parseDebug() — MSP_DEBUG (254)
// Requires: set debug_mode = MAG_CALIB; save  in BF CLI
// -----------------------------------------------------------------------------

bool DroneLink::parseDebug(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::DEBUG) return false;
    auto r16s = [&](size_t i) -> int16_t {
        return static_cast<int16_t>(
            static_cast<uint16_t>(buf[i]) |
            (static_cast<uint16_t>(buf[i + 1]) << 8)); };
    int16_t mx = r16s(5), my = r16s(7), mz = r16s(9);
    s.magX = mx; s.magY = my; s.magZ = mz;
    float h = std::atan2f(static_cast<float>(my), static_cast<float>(mx))
        * (180.0f / 3.14159265358979f);
    s.magHeadingDeg = h < 0.0f ? h + 360.0f : h;
    s.magValid = (mx != 0 || my != 0 || mz != 0);
    return true;
}

// -----------------------------------------------------------------------------
// parseBaro() — MSP_ALTITUDE (109)
// -----------------------------------------------------------------------------

bool DroneLink::parseBaro(const std::vector<uint8_t>& buf, DroneState& s) {
    BaroReading raw;
    if (!BaroBMP280::parse(buf, raw)) { s.baroValid = false; return false; }
    s.baroAltitudeCm = raw.altitudeCm;
    s.baroVarioCmPerSec = raw.varioCmPerSec;
    s.baroValid = true;
    return true;
}

// -----------------------------------------------------------------------------
// parseGPSRaw() / parseGPSComp() / parseNavStatus()
// -----------------------------------------------------------------------------

bool DroneLink::parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseRaw(buf, s.gps)) { s.gps.rawValid = false; return false; }
    return true;
}

bool DroneLink::parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseComp(buf, s.gps)) { s.gps.compValid = false; return false; }
    return true;
}

bool DroneLink::parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    return GPSNeoM10::parseNavStatus(buf, s.navStatus);
}

// -----------------------------------------------------------------------------
// parseMspSvInfo() — DroneLink wrapper around GPSNeoM10::parseMspSvInfo()
// -----------------------------------------------------------------------------

bool DroneLink::parseMspSvInfo(const std::vector<uint8_t>& buf, DroneState& s) {
    std::vector<SVInfoEntry> sv;
    if (!GPSNeoM10::parseMspSvInfo(buf, sv)) return false;
    s.svList = std::move(sv);
    s.svInfoValid = true;
    s.svSource = "MSP";
    return true;
}

// =============================================================================
// NEW parsers
// =============================================================================

// -----------------------------------------------------------------------------
// parseStatusEx() — MSP_STATUS_EX (150)
//
// Superset of MSP_STATUS (101). Shares the same first 17 payload bytes.
// The extra fields carry the arming-disable reason bitmask, which tells
// the GCS exactly why the FC refuses to arm.
//
// Payload layout (BF 4.5.x, little-endian):
//   buf[5..6]   uint16  cycleTime           (same as STATUS — not re-read)
//   buf[7..8]   uint16  i2cErrorCount       (same as STATUS — not re-read)
//   buf[9..10]  uint16  sensorStatus        (same as STATUS — not re-read)
//   buf[11..14] uint32  flightModeFlags     (same as STATUS — not re-read)
//   buf[15]     uint8   currentPidProfile   (same as STATUS — not re-read)
//   buf[16..17] uint16  averageSystemLoad   (same as STATUS — not re-read)
//   buf[18..19] uint16  armingDisableCount  informational count of set bits
//   buf[20..23] uint32  armingDisableFlags  ArmingDisable:: bitmask ← the one we want
//
// Note: parseStatusEx does NOT re-write fields already set by parseStatus()
// (which runs first). It only enriches the arming-disable information.
//
// Min frame: header(5) + payload(19) + checksum(1) = 25 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseStatusEx(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 25 || buf[4] != MSP::STATUS_EX) return false;

    auto ru32 = [&](int i) -> uint32_t {
        return static_cast<uint32_t>(buf[i]) |
            (static_cast<uint32_t>(buf[i + 1]) << 8) |
            (static_cast<uint32_t>(buf[i + 2]) << 16) |
            (static_cast<uint32_t>(buf[i + 3]) << 24);
        };

    s.armingDisableFlags = ru32(20);
    s.armingDisableStr = decodeArmingDisable(s.armingDisableFlags);
    return true;
}

// -----------------------------------------------------------------------------
// parseBatteryState() — MSP_BATTERY_STATE (242)
//
// Authoritative source for cell count, design capacity, battery health state,
// and high-resolution voltage (10 mV resolution vs 100 mV from ANALOG).
//
// Battery percentage is computed from mAh drawn / design capacity. It is 0
// when batteryCapacityMah == 0 (not configured in BF Configurator).
// Configure in BF: set battery_capacity = <mAh>; save
//
// Payload layout (BF 4.5.x, little-endian):
//   buf[5]      uint8   cellCount
//   buf[6..7]   uint16  capacity    design capacity, mAh
//   buf[8]      uint8   voltage     0.1 V legacy — skipped; use buf[14..15]
//   buf[9..10]  uint16  mAhDrawn    consumed mAh (cross-checks ANALOG)
//   buf[11..12] uint16  amperage    centiamps (cross-checks ANALOG)
//   buf[13]     uint8   battState   BatteryState enum
//   buf[14..15] uint16  voltage10mV 10 mV per unit (0.01 V resolution)
//
// Min frame: header(5) + payload(11) + checksum(1) = 17 bytes
// Full frame with voltage10mV: header(5) + payload(11) + 2 + checksum = 19
// -----------------------------------------------------------------------------

bool DroneLink::parseBatteryState(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 17 || buf[4] != MSP::BATTERY_STATE) return false;

    auto ru16 = [&](int i) -> uint16_t {
        return static_cast<uint16_t>(buf[i]) |
            (static_cast<uint16_t>(buf[i + 1]) << 8);
        };

    s.batteryCellCount = buf[5];
    s.batteryCapacityMah = ru16(6);
    // buf[8] = legacy 0.1 V voltage — skipped; use voltage10mV below
    s.batteryMahDrawn = ru16(9);

    int16_t centiAmps = static_cast<int16_t>(ru16(11));
    s.batteryCurrent = centiAmps / 100.0f;

    s.batteryState = static_cast<BatteryState>(buf[13]);

    // FIX: removed the erroneous "buf.size() >= 18" guard.
    // Minimum frame is 17 bytes (header 5 + payload 11 + checksum 1).
    // buf[14] = voltage10mV LSB and buf[15] = voltage10mV MSB are always
    // present and safe to access once the size-17 check above passes.
    {
        uint16_t v10mV = ru16(14);
        if (v10mV > 0)
            s.batteryVoltage = v10mV / 100.0f;   // 10 mV → V  (0.01 V resolution)
    }

    // Battery percentage: requires battery_capacity set in BF Configurator
    if (s.batteryCapacityMah > 0 && s.batteryMahDrawn <= s.batteryCapacityMah) {
        uint16_t remaining = s.batteryCapacityMah - s.batteryMahDrawn;
        s.batteryPercentage = static_cast<uint8_t>(
            (static_cast<uint32_t>(remaining) * 100u) / s.batteryCapacityMah);
    }
    else {
        s.batteryPercentage = 0;
    }

    return true;
}


// -----------------------------------------------------------------------------
// parseMotors() — MSP_MOTOR (104)
//
// Up to 8 × uint16 motor throttle values (1000–2000 µs).
// On a quad (F405 V3), only motors[0..3] will be non-zero.
// motorCount is set to the index of the last non-zero motor + 1.
//
// Payload layout:
//   buf[5..6]   motor[0]  uint16 µs
//   buf[7..8]   motor[1]  uint16 µs
//   ...
//   buf[19..20] motor[7]  uint16 µs  (16 bytes total payload)
//
// Min frame: header(5) + payload(16) + checksum(1) = 22 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseMotors(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 22 || buf[4] != MSP::MOTOR) return false;

    s.motorCount = 0;
    for (int i = 0; i < MAX_MOTORS; ++i) {
        uint16_t val = static_cast<uint16_t>(buf[5 + i * 2]) |
            (static_cast<uint16_t>(buf[6 + i * 2]) << 8);
        s.motorValues[i] = val;
        if (val > 0) s.motorCount = static_cast<uint8_t>(i + 1);
    }
    return true;
}

// -----------------------------------------------------------------------------
// parseRCChannels() — MSP_RC (105)
//
// N × uint16 RC channel values (1000–2000 µs).
// N is determined from the payload length field (buf[3]).
// With RadioMaster Pocket + ELRS you typically get 12–16 channels.
//
// RadioMaster Pocket / CRSF default channel layout (MODE 2):
//   [0] Roll (Aileron)    [1] Pitch (Elevator)
//   [2] Throttle          [3] Yaw   (Rudder)
//   [4] ARM switch        [5] Flight mode AUX
//   [6..] AUX3, AUX4, ...
//
// Min frame: header(5) + ≥2 bytes payload + checksum(1) = 8 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseRCChannels(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 8 || buf[4] != MSP::RC) return false;

    uint8_t payloadLen = buf[3];
    int chCount = (static_cast<int>(payloadLen / 2) < MAX_RC_CH)
        ? static_cast<int>(payloadLen / 2) : MAX_RC_CH;

    // Sanity: entire frame must be present
    if (buf.size() < static_cast<size_t>(5u + payloadLen + 1u)) return false;

    s.rcChannelCount = static_cast<uint8_t>(chCount);
    for (int i = 0; i < chCount; ++i) {
        s.rcChannels[i] = static_cast<uint16_t>(buf[5 + i * 2]) |
            (static_cast<uint16_t>(buf[6 + i * 2]) << 8);
    }
    return true;
}

// =============================================================================
// Decode helpers (static)
// =============================================================================

// -----------------------------------------------------------------------------
// decodeFlightMode()
//
// Translates the flightModeFlags bitmask from MSP_STATUS into a human-readable
// string for the GCS status bar.
//
// Priority order: FAILSAFE > GPS_RESCUE > ANGLE > HORIZON > ACRO (default)
// Augmentation modes (MAG, HEADFREE, ANTIGRAV) are appended with '+'.
// "[DISARMED]" is appended when the ARM bit is clear.
// -----------------------------------------------------------------------------

/*static*/
std::string DroneLink::decodeFlightMode(uint32_t flags) {
    std::string mode;
    bool isEmergency = false;

    if (flags & FlightMode::FAILSAFE) {
        mode = "FAILSAFE";
        isEmergency = true;
    }
    else if (flags & FlightMode::GPS_RESCUE) {
        mode = "GPS RESCUE";
        isEmergency = true;
    }
    else if (flags & FlightMode::ANGLE) {
        mode = "ANGLE";
    }
    else if (flags & FlightMode::HORIZON) {
        mode = "HORIZON";
    }
    else {
        mode = "ACRO";
    }

    // Augmentation flags only make sense in non-emergency modes
    if (!isEmergency) {
        if (flags & FlightMode::MAG)       mode += "+MAG";
        if (flags & FlightMode::HEADFREE)  mode += "+HEADFREE";
        if (flags & FlightMode::ANTI_GRAV) mode += "+ANTIGRAV";
    }

    if (!(flags & FlightMode::ARM))
        mode += " [DISARMED]";

    return mode;
}

// -----------------------------------------------------------------------------
// decodeArmingDisable()
//
// Translates the armingDisableFlags bitmask from MSP_STATUS_EX into a
// comma-separated string listing every active reason the FC won't arm.
// Returns "" when flags == 0 (ready to arm).
// -----------------------------------------------------------------------------

/*static*/
std::string DroneLink::decodeArmingDisable(uint32_t flags) {
    if (flags == 0) return "";

    struct Entry { uint32_t bit; const char* label; };
    static constexpr Entry table[] = {
        { ArmingDisable::NO_GYRO,          "NO GYRO"         },
        { ArmingDisable::FAILSAFE,          "FAILSAFE"        },
        { ArmingDisable::RX_FAILSAFE,       "RX FAILSAFE"     },
        { ArmingDisable::BAD_RX_RECOVERY,   "BAD RX"          },
        { ArmingDisable::BOXFAILSAFE,       "BOX FAILSAFE"    },
        { ArmingDisable::RUNAWAY_TAKEOFF,   "RUNAWAY TKOFF"   },
        { ArmingDisable::CRASH_DETECTED,    "CRASH DETECT"    },
        { ArmingDisable::THROTTLE,          "THROTTLE HIGH"   },
        { ArmingDisable::ANGLE,             "NOT LEVEL"       },
        { ArmingDisable::BOOT_GRACE_TIME,   "BOOT GRACE"      },
        { ArmingDisable::NOPREARM,          "NO PREARM"       },
        { ArmingDisable::LOAD,              "CPU OVERLOAD"    },
        { ArmingDisable::CALIBRATING,       "CALIBRATING"     },
        { ArmingDisable::CLI,               "CLI ACTIVE"      },
        { ArmingDisable::CMS_MENU,          "OSD MENU"        },
        { ArmingDisable::BST,               "BST"             },
        { ArmingDisable::MSP,               "MSP OVERRIDE"    },
        { ArmingDisable::PARALYZE,          "PARALYZE"        },
        { ArmingDisable::GPS,               "GPS NOT READY"   },
        { ArmingDisable::RESC_SW,           "RESCUE SW"       },
        { ArmingDisable::DSHOT_BITBANG,     "DSHOT BITBANG"   },
        { ArmingDisable::ACC_CALIBRATION,   "ACC CAL NEEDED"  },
        { ArmingDisable::MOTOR_PROTOCOL,    "MOTOR PROTOCOL"  },
        { ArmingDisable::ARM_SWITCH,        "ARM SW OFF"      },
    };

    std::string result;
    for (const auto& e : table) {
        if (flags & e.bit) {
            if (!result.empty()) result += ", ";
            result += e.label;
        }
    }
    return result;
}

// =============================================================================
// commitState()
// =============================================================================

void DroneLink::commitState(const DroneState& s) {
    std::lock_guard<std::mutex> lock(dataMutex);
    currentState = s;
}

// =============================================================================
// AutoDetectF405()
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