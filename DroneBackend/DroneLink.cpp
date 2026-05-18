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
//  11.  MSP_MOTOR        (104)  motor throttle outputs (1000-2000 µs)
//  12.  MSP_RC           (105)  RC channel inputs (1000-2000 µs)
//
//   ── Throttled polls ──
//  13.  MSP_GPS_SV_INFO  (164)  satellite list, every SV_POLL_TICKS   (~1 s)
//  14.  MSP_BATTERY_STATE(242)  full battery detail, every SLOW_POLL_TICKS (~500 ms)
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

        // ── Magnetometer calibration request ─────────────────────────────────
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
// NOT called from communicationLoop().
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
//
// FIX (RC / large-frame reliability):
//   The original code called PurgeComm(PURGE_RXCLEAR) unconditionally at the
//   top of every poll. With 12+ back-to-back polls at 100 Hz, the purge on
//   poll N+1 was flushing the tail of poll N's response before it finished
//   arriving over the serial FIFO.
//
//   Small responses (STATUS, ANALOG, etc. — 12-20 bytes) arrived fast enough
//   to escape truncation most of the time. MSP_RC with ELRS/CRSF returns
//   16 channels = 32 payload bytes → 38-byte frame. At 57600 baud, 38 bytes
//   takes ~6.6 ms to arrive. The next sendMSP() was issued well inside that
//   window, killing the RC frame mid-read and leaving rcChannelCount = 0
//   in DroneState — which ArmingWidget reported as "NO SIGNAL / UART NOT
//   CONFIGURED" even though the receiver was bound and working perfectly in BF.
//
//   Fix: remove the unconditional pre-poll purge. The read loop already drains
//   exactly payloadLen+6 bytes (or times out). Any genuine stale garbage from
//   a previous timeout is discarded by the header-sync logic inside the read
//   loop (we only accept '$','M','>').  A targeted purge is still issued if
//   we detect an unexpected header byte, so bus wedge recovery is preserved.
// =============================================================================

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    // ── Write request ─────────────────────────────────────────────────────────
    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD   written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL)
        || written != sizeof(req))
        return {};

    // ── Read response with header-sync ────────────────────────────────────────
    // We look for the '$','M','>' preamble before trusting payloadLen.
    // This discards any stale bytes left from a previous timed-out poll
    // without a blanket PurgeComm that would also throw away bytes still
    // in transit for the current response.
    constexpr size_t MAX_FRAME = 512;
    constexpr int    SYNC_TRIES = 64;   // bytes to scan before giving up sync

    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(80);

    auto readByte = [&](uint8_t& out) -> bool {
        while (std::chrono::steady_clock::now() < deadline) {
            DWORD rd = 0;
            if (ReadFile(hSerial, &out, 1, &rd, NULL) && rd == 1) return true;
            std::this_thread::sleep_for(std::chrono::microseconds(150));
        }
        return false;
        };

    // Step 1: sync to '$','M','>'
    uint8_t b0 = 0, b1 = 0, b2 = 0;
    int syncTries = 0;
    while (syncTries < SYNC_TRIES) {
        if (!readByte(b0)) return {};
        if (b0 != '$') { ++syncTries; continue; }
        if (!readByte(b1)) return {};
        if (b1 != 'M') { ++syncTries; continue; }
        if (!readByte(b2)) return {};
        if (b2 == '>') break;   // found header
        // b2 was not '>' — could be '<' (echo) or error byte; keep scanning
        ++syncTries;
    }
    if (syncTries >= SYNC_TRIES) return {};

    // Step 2: read payloadLen + cmd bytes
    uint8_t payloadLen = 0, cmdByte = 0;
    if (!readByte(payloadLen)) return {};
    if (!readByte(cmdByte))    return {};

    // Sanity: cmd byte must match what we requested
    if (cmdByte != mspID) {
        // Stale frame from a different command — flush and bail
        PurgeComm(hSerial, PURGE_RXCLEAR);
        return {};
    }

    if (payloadLen > MAX_FRAME - 6) return {};

    // Step 3: read payload + checksum
    std::vector<uint8_t> frame;
    frame.reserve(6u + payloadLen);
    frame.push_back('$');
    frame.push_back('M');
    frame.push_back('>');
    frame.push_back(payloadLen);
    frame.push_back(cmdByte);

    for (int i = 0; i < static_cast<int>(payloadLen) + 1 /*checksum*/; ++i) {
        uint8_t pb = 0;
        if (!readByte(pb)) return {};
        frame.push_back(pb);
    }

    return frame;
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
//   [4..5]  uint16  sensorStatus            → sensorStatus
//   [6..9]  uint32  flightModeFlags         → flightModeFlags
//   [10]    uint8   currentPidProfile       → pidProfile
//   [11..12]uint16  averageSystemLoad %     → cpuLoadPercent
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
// BF 4.x payload layout (little-endian):
//   buf[5]      uint8   vbat_legacy    0.1 V units  (max 25.5 V — kept for
//                                      compatibility; unreliable above 25 V)
//   buf[6..7]   uint16  mAhDrawn       mAh consumed
//   buf[8]      uint8   rssi           0-255
//   buf[9..10]  int16   amperage       centiamps  (divide by 100 → Amps)
//   buf[11..12] uint16  vbat_10mV      10 mV per LSB  (BF 4.x extended field)
//                                      → divide by 100 → Volts (0.01 V res.)
//
// FIX: The original parser only read buf[5] (the legacy single-byte 0.1 V
// field).  On BF 4.x the authoritative voltage is the uint16 at buf[11..12]
// (10 mV resolution).  For a 4S pack at 16.24 V the raw value is 1624
// (0x0658); buf[5] legacy field holds 162 → 16.2 V which is passable, but
// if BF clips the legacy field to uint8 max (255 → 25.5 V) or the cell
// voltage ever exceeded ~2.55 V × cells the legacy read would be wrong.
//
// We now prefer the 10 mV field and fall back to the legacy byte only when
// the extended field is absent (older BF firmware / very short frame).
//
// FIX: rssi from MSP_ANALOG is always 0 for ELRS/CRSF receivers because
// ELRS does not populate the RSSI byte in the MSP_ANALOG response — it uses
// a dedicated link statistics packet instead.  The rc_link_quality field in
// to_dict() must therefore emit -1 when rssi == 0 so the widget switches to
// the channel-count / stick-range path rather than showing "0% quality".
// That logic already lives in to_dict(); parseAnalog() just stores rssi as-is.
//
// Min frame (legacy only):  header(5) + payload(6)  + checksum(1) = 12 bytes
// Min frame (with 10mV ext): header(5) + payload(8) + checksum(1) = 14 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ANALOG) return false;

    float legacyVoltage = buf[5] / 10.0f;

    s.batteryMahDrawn = static_cast<uint16_t>(buf[6]) |
        (static_cast<uint16_t>(buf[7]) << 8);

    s.rssi = buf[8];

    int16_t centiAmps = static_cast<int16_t>(
        static_cast<uint16_t>(buf[9]) |
        (static_cast<uint16_t>(buf[10]) << 8));
    s.batteryCurrent = centiAmps / 100.0f;

    // FIX: was buf[11..12] — correct offset is buf[12..13].
    // Need the full 15-byte frame (header 5 + payload 9 + checksum 1).
    if (buf.size() >= 15) {
        uint16_t v10mV = static_cast<uint16_t>(buf[12]) |
            (static_cast<uint16_t>(buf[13]) << 8);

        // Sanity clamp: reject readings outside plausible LiPo range (1S–12S).
        // This catches the old off-by-one garbage value of 207 V.
        float extV = v10mV / 100.0f;
        s.batteryVoltage = (extV >= 2.0f && extV <= 60.0f) ? extV : legacyVoltage;
    }
    else {
        s.batteryVoltage = legacyVoltage;
    }

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
// parseMspSvInfo() — DroneLink wrapper
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
// NEW / FIXED parsers
// =============================================================================

// -----------------------------------------------------------------------------
// parseStatusEx() — MSP_STATUS_EX (150)
//
// Payload layout (BF 4.5.x, little-endian):
//   buf[5..6]   uint16  cycleTime           (same as STATUS — not re-read)
//   buf[7..8]   uint16  i2cErrorCount       (same as STATUS — not re-read)
//   buf[9..10]  uint16  sensorStatus        (same as STATUS — not re-read)
//   buf[11..14] uint32  flightModeFlags     (same as STATUS — not re-read)
//   buf[15]     uint8   currentPidProfile   (same as STATUS — not re-read)
//   buf[16..17] uint16  averageSystemLoad   (same as STATUS — not re-read)
//   buf[18..19] uint16  armingDisableCount
//   buf[20..23] uint32  armingDisableFlags  ← the field we want
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
// Payload layout (BF 4.5.x, little-endian):
//   buf[5]      uint8   cellCount
//   buf[6..7]   uint16  capacity        design capacity mAh
//   buf[8]      uint8   voltage_legacy  0.1 V — skipped; use buf[14..15]
//   buf[9..10]  uint16  mAhDrawn
//   buf[11..12] uint16  amperage        centiamps
//   buf[13]     uint8   battState       BatteryState enum
//   buf[14..15] uint16  voltage_10mV    10 mV per unit  (BF 4.1+)
//
// FIX: The original code had a minimum size guard of < 17 bytes.
// On BF 4.5 the frame is header(5) + payload(9) + checksum(1) = 15 bytes.
// The guard of 17 caused every parseBatteryState() call to return false
// immediately, so batteryVoltage, batteryCellCount, and batteryState were
// never written — leaving the widgets showing "—" and "INIT" forever.
//
// Corrected guard:
//   >= 15 bytes → can read cellCount through battState  (buf[5..13])
//   >= 17 bytes → can also read voltage_10mV            (buf[14..15])
// -----------------------------------------------------------------------------

bool DroneLink::parseBatteryState(const std::vector<uint8_t>& buf, DroneState& s) {
    // Minimum: header(5) + cellCount(1) + capacity(2) + vlegacy(1)
    //          + mAhDrawn(2) + amperage(2) + battState(1) + checksum(1) = 15
    if (buf.size() < 15 || buf[4] != MSP::BATTERY_STATE) return false;

    auto ru16 = [&](int i) -> uint16_t {
        return static_cast<uint16_t>(buf[i]) |
            (static_cast<uint16_t>(buf[i + 1]) << 8);
        };

    s.batteryCellCount = buf[5];
    s.batteryCapacityMah = ru16(6);
    // buf[8] = legacy 0.1 V — skip in favour of 10 mV field below
    s.batteryMahDrawn = ru16(9);

    int16_t centiAmps = static_cast<int16_t>(ru16(11));
    s.batteryCurrent = centiAmps / 100.0f;

    s.batteryState = static_cast<BatteryState>(buf[13]);

    // ── High-resolution voltage (10 mV, BF 4.1+) ─────────────────────────────
    // Only present when frame is long enough. Guard is >= 17 because we need
    // buf[14] (LSB) AND buf[15] (MSB), and the frame vector is 0-indexed.
    if (buf.size() >= 17) {
        uint16_t v10mV = ru16(14);
        if (v10mV > 0)
            s.batteryVoltage = v10mV / 100.0f;   // 10 mV → V  (0.01 V resolution)
    }
    // If the 10mV field is absent (older BF or short frame), batteryVoltage
    // retains the value already written by parseAnalog() this tick — so we
    // always have at least the 0.1 V resolution reading available.

    // ── Battery percentage (requires battery_capacity set in BF) ─────────────
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
// Up to 8 × uint16 motor throttle values (1000-2000 µs).
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
// N × uint16 RC channel values (1000-2000 µs).
// N = payloadLen / 2.  With RadioMaster Pocket + ELRS, BF reports 16 channels
// = 32-byte payload → 38-byte frame.
//
// FIX: The original sendMSP() called PurgeComm(PURGE_RXCLEAR) at the top of
// every poll. At 57 600 baud a 38-byte MSP_RC frame takes ~6.6 ms to arrive.
// The unconditional purge in the immediately following sendMSP() call was
// flushing the tail of the RC frame before it was fully read, leaving
// rcChannelCount = 0 and causing ArmingWidget to report "NO SIGNAL".
// The fix is in sendMSP() (see its comment above); parseRCChannels() itself
// is unchanged except for added documentation.
//
// RadioMaster Pocket / CRSF channel layout (MODE 2):
//   [0] Roll   [1] Pitch   [2] Throttle   [3] Yaw
//   [4] ARM switch         [5] Flight mode AUX
//   [6..15] AUX3-AUX12
//
// Min frame: header(5) + 2-byte payload (1 ch) + checksum(1) = 8 bytes
// Typical:   header(5) + 32-byte payload (16 ch) + checksum(1) = 38 bytes
// -----------------------------------------------------------------------------

bool DroneLink::parseRCChannels(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 8 || buf[4] != MSP::RC) return false;

    uint8_t payloadLen = buf[3];
    int chCount = static_cast<int>(payloadLen / 2);
    if (chCount > MAX_RC_CH) chCount = MAX_RC_CH;

    // Sanity: the entire declared frame must have arrived
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

    if (!isEmergency) {
        if (flags & FlightMode::MAG)       mode += "+MAG";
        if (flags & FlightMode::HEADFREE)  mode += "+HEADFREE";
        if (flags & FlightMode::ANTI_GRAV) mode += "+ANTIGRAV";
    }

    if (!(flags & FlightMode::ARM))
        mode += " [DISARMED]";

    return mode;
}

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