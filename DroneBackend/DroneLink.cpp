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
    , gpsUartIndex_(0)           // UART1 — confirmed GPS UART for this project
    , magCalRequested(false)
    , accCalRequested(false)
    , magCalActive_(false)
    , accCalActive_(false)
{
    // Delay the first SV poll so the GPS has time to acquire a fix before
    // we attempt the passthrough cycle.  First poll fires at SV_POLL_INTERVAL_S
    // seconds after connect().
    lastSvPollTime_ = std::chrono::steady_clock::now();
}

DroneLink::~DroneLink() {
    disconnect();
}

// =============================================================================
// openSerialPort()
//
// Opens portName and configures it at MSP_BAUD (57600), 8N1, no flow control,
// non-blocking COMMTIMEOUTS.
//
// ── WHY 57600 AND NOT 115200 ─────────────────────────────────────────────────
//
// sv_baud_probe.py confirmed the FC's USB/MSP port runs at 57600.
// This is the baud DroneLink uses for ALL communication with the FC —
// normal MSP telemetry AND the passthrough handshake bytes.
//
// The GPS UART baud (BF ↔ NEO-M10 inside the FC) is a completely separate
// setting configured in BF Configurator → Ports → UART1 = 115200.
// BF bridges the two speeds transparently during passthrough mode.
//
//   DroneLink ──57600──► FC USB  ──115200──► NEO-M10
//
// If you ever move to a different FC or firmware, re-run sv_baud_probe.py
// and update MSP_BAUD in DroneLink.h accordingly.
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

    // MSP_BAUD = 57600 — confirmed by sv_baud_probe.py.
    // This was previously hardcoded as CBR_115200 which caused all MSP
    // communication to fail silently (garbled bytes in both directions).
    dcb.BaudRate = MSP_BAUD;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    // Disable hardware flow control.
    // CP210x / CH340 chips can assert RTS/CTS by default, which stalls reads
    // on certain FC boards.
    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;

    if (!SetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial); hSerial = INVALID_HANDLE_VALUE; return false;
    }

    // Non-blocking: ReadFile returns immediately with whatever bytes are
    // already in the driver buffer.  The worker loop sleeps between polls.
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

    portName_ = portName;   // save for reopen after passthrough cycle

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
// Poll order per 10 ms tick:
//   1.  MSP_STATUS      (101)  FC cycle time, sensor status flags
//   2.  MSP_RAW_IMU     (102)  raw accel + gyro (MPU-6500)
//   3.  MSP_ATTITUDE    (108)  fused roll / pitch / yaw
//   4.  MSP_ANALOG      (110)  battery voltage, RSSI
//   5.  MSP_DEBUG       (254)  mag X/Y/Z via debug_mode=MAG_CALIB
//   6.  MSP_ALTITUDE    (109)  BMP280 altitude + vario
//   7.  MSP_RAW_GPS     (106)  GPS fix, sats, lat/lon, alt, speed, course, HDOP
//   8.  MSP_COMP_GPS    (107)  distance + bearing to home, GPS heartbeat
//   9.  MSP_NAV_STATUS  (121)  GPS nav engine status + fix flags
//  10.  pollSatellites()       UBX-NAV-SAT via passthrough, every 30 s
//
// pollSatellites() closes and reopens hSerial to exit BF passthrough mode.
// The entire cycle takes ~300–600 ms and drops ~1–3 MSP frames per poll.
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

        // ── Satellite list — passthrough cycle every SV_POLL_INTERVAL_S ───────
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - lastSvPollTime_).count();

            if (elapsed >= SV_POLL_INTERVAL_S) {
                lastSvPollTime_ = now;
                pollSatellites(pending);

                // pollSatellites() closes and reopens hSerial.
                // If the reopen failed, try once before the next tick.
                if (hSerial == INVALID_HANDLE_VALUE) {
                    pending.linkHealthy = false;
                    commitState(pending);
                    openSerialPort(portName_);
                }
            }
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

// =============================================================================
// pollSatellites()
//
// Runs the full MSP_SET_PASSTHROUGH → UBX-NAV-SAT → close/reopen cycle.
//
// PROTOCOL
// ────────
// Step 1  Flush RX and TX buffers.
//
// Step 2  Send MSP_SET_PASSTHROUGH frame with gpsUartIndex_ as payload.
//         Frame: '$' 'M' '<' 0x01 0xF5 <uartIdx> <checksum>
//         Checksum = 0x01 ^ 0xF5 ^ uartIdx
//         This is sent at MSP_BAUD (57600) — the normal MSP speed.
//
// Step 3  Read and verify the 6-byte MSP ACK ($M> 0x00 0xF5 cksum).
//         After the ACK, BF is in raw forwarding mode on gpsUartIndex_.
//         Bail out if the ACK is not received within 150 ms.
//
// Step 4  Write the raw UBX-NAV-SAT poll (8 bytes, NO MSP framing).
//         BF forwards these bytes verbatim to the GPS UART at 115200.
//
// Step 5  Read the raw UBX response with readUbxResponse().
//         Allow 400 ms — typical NEO-M10 response arrives in < 80 ms.
//         The response arrives from the GPS at 115200 and is forwarded
//         back to us at 57600 (BF does the buffering).
//
// Step 6  Close and reopen the serial port to exit passthrough mode.
//         BF has no "exit passthrough" command; the port reset is the
//         only reliable method.  Takes ~150 ms (CP210x/CH340 reset).
//
// Step 7  Parse the UBX-NAV-SAT payload.
//         Frame layout: [0]=0xB5 [1]=0x62 [2]=class [3]=id [4..5]=len …
//         Verify class=0x01 (NAV), id=0x35 (SAT) before parsing.
// =============================================================================

void DroneLink::pollSatellites(DroneState& pending) {
    if (hSerial == INVALID_HANDLE_VALUE) return;

    // Step 1 — flush
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    // Step 2 — MSP_SET_PASSTHROUGH
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

    // Step 3 — read MSP ACK (6 bytes: $ M > 0x00 0xF5 checksum)
    {
        auto    deadline = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(150);
        uint8_t ack[6] = {};
        int     got = 0;

        while (got < 6 && std::chrono::steady_clock::now() < deadline) {
            DWORD   rd = 0;
            uint8_t b = 0;
            if (ReadFile(hSerial, &b, 1, &rd, NULL) && rd == 1)
                ack[got++] = b;
            else
                std::this_thread::sleep_for(std::chrono::microseconds(200));
        }

        if (got < 6 || ack[0] != '$' || ack[1] != 'M' || ack[2] != '>') {
            // No valid ACK — restore MSP mode and bail.
            closeSerialPort();
            openSerialPort(portName_);
            return;
        }
    }

    // Step 4 — write raw UBX-NAV-SAT poll (no MSP framing)
    {
        auto  ubxPoll = GPSNeoM10::buildNavSvInfoPoll();
        DWORD written = 0;
        WriteFile(hSerial,
            ubxPoll.data(),
            static_cast<DWORD>(ubxPoll.size()),
            &written, NULL);
    }

    // Step 5 — read raw UBX response
    auto ubxResp = readUbxResponse(hSerial, 400);

    // Step 6 — close and reopen to exit passthrough, restore MSP mode
    closeSerialPort();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    openSerialPort(portName_);

    // Step 7 — parse
    if (ubxResp.size() < 10) return;
    if (ubxResp[2] != 0x01 || ubxResp[3] != 0x35) return;   // must be NAV-SAT

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
    }
}

// =============================================================================
// readUbxResponse()
//
// Reads bytes from h, scanning for the UBX sync sequence 0xB5 0x62.
// All bytes before the preamble are silently discarded — this handles
// any residual MSP ACK bytes or forwarding noise that precedes the UBX frame.
//
// After the preamble:
//   bytes [2..3] = class, id
//   bytes [4..5] = payload length (LE uint16)
//   bytes [6 .. 6+payLen-1] = payload
//   bytes [6+payLen, 6+payLen+1] = CK_A, CK_B
//
// Returns the complete frame starting at 0xB5, or empty on timeout/error.
// =============================================================================

std::vector<uint8_t> DroneLink::readUbxResponse(HANDLE h, int timeoutMs) {
    if (h == INVALID_HANDLE_VALUE) return {};

    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    // Helper: read one byte, spinning until it arrives or deadline passes.
    auto readByte = [&](uint8_t& out) -> bool {
        while (std::chrono::steady_clock::now() < deadline) {
            DWORD rd = 0;
            if (ReadFile(h, &out, 1, &rd, NULL) && rd == 1) return true;
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        return false;
        };

    // Phase 1 — scan for preamble 0xB5 0x62
    {
        uint8_t prev = 0, curr = 0;
        while (true) {
            if (!readByte(curr)) return {};
            if (prev == 0xB5 && curr == 0x62) break;
            prev = curr;
        }
    }

    // Phase 2 — read 4-byte UBX header (class, id, lenLo, lenHi)
    uint8_t hdr[4] = {};
    for (int i = 0; i < 4; ++i)
        if (!readByte(hdr[i])) return {};

    uint16_t payloadLen =
        static_cast<uint16_t>(hdr[2]) |
        (static_cast<uint16_t>(hdr[3]) << 8);

    // Safety cap: NAV-SAT for 32 SVs = 8 + 32×12 = 392 bytes payload.
    // 2048 gives generous headroom for any future message.
    if (payloadLen > 2048) return {};

    // Phase 3 — assemble complete frame (preamble + header + payload + checksum)
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
// Builds and sends an MSP v1 request frame, then reads the response
// byte-by-byte until the full frame arrives or the 80 ms deadline fires.
//
// PurgeComm(PURGE_RXCLEAR) before every write flushes stale bytes from the
// previous command, preventing cross-command frame contamination.
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

// =============================================================================
// applyGPSConfig()
//
// Sends a sequence of UBX configuration frames to the NEO-M10 via the same
// MSP_SET_PASSTHROUGH cycle used by pollSatellites().  Each UBX message gets
// its own passthrough session because BF must exit passthrough between them.
// =============================================================================

GPSConfigResult DroneLink::applyGPSConfig(const GPSConfig& cfg) {
    GPSConfigResult result;
    if (!connected.load()) {
        result.errorDetail = "Not connected";
        return result;
    }

    // Helper lambda — one UBX config frame per passthrough session.
    auto sendUbxConfig = [&](const std::vector<uint8_t>& ubxFrame,
        uint8_t cls, uint8_t id) -> bool {
            if (hSerial == INVALID_HANDLE_VALUE) return false;

            PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

            // Send MSP_SET_PASSTHROUGH
            uint8_t len = 0x01;
            uint8_t cmd = MSP::SET_PASSTHROUGH;
            uint8_t csum = static_cast<uint8_t>(len ^ cmd ^ gpsUartIndex_);
            uint8_t ptFrame[7] = { '$', 'M', '<', len, cmd, gpsUartIndex_, csum };
            DWORD   written = 0;
            if (!WriteFile(hSerial, ptFrame, sizeof(ptFrame), &written, NULL)
                || written != sizeof(ptFrame))
                return false;

            // Discard MSP ACK (6 bytes)
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

            // Write raw UBX config frame (no MSP framing)
            written = 0;
            WriteFile(hSerial, ubxFrame.data(),
                static_cast<DWORD>(ubxFrame.size()), &written, NULL);

            // Read UBX-ACK-ACK or UBX-ACK-NAK response
            auto resp = readUbxResponse(hSerial, 300);

            // Exit passthrough
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

// -- MSP_STATUS (101) ----------------------------------------------------------
// [5..6] uint16_t cycleTime (µs) → fcCycleMs
bool DroneLink::parseStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 11 || buf[4] != MSP::STATUS) return false;
    s.fcCycleMs = static_cast<uint16_t>(buf[5] | (buf[6] << 8)) / 1000.0;
    return true;
}

// -- MSP_RAW_IMU (102) ---------------------------------------------------------
// Payload: 9 × int16_t = 18 bytes (ax ay az gx gy gz mx my mz)
bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8)); };
    s.ax = r16(5); s.ay = r16(7); s.az = r16(9);
    s.gx = r16(11); s.gy = r16(13); s.gz = r16(15);
    return true;
}

// -- MSP_ATTITUDE (108) --------------------------------------------------------
// roll/pitch: degrees × 10.  yaw: full degrees.
bool DroneLink::parseAttitude(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 12 || buf[4] != MSP::ATTITUDE) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8)); };
    s.roll = r16(5); s.pitch = r16(7); s.yaw = r16(9);
    return true;
}

// -- MSP_ANALOG (110) ----------------------------------------------------------
// [5] vbat × 10,  [8] rssi
bool DroneLink::parseAnalog(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 14 || buf[4] != MSP::ANALOG) return false;
    s.batteryVoltage = buf[5] / 10.0f;
    s.rssi = buf[8];
    return true;
}

// -- MSP_DEBUG (254) → mag X/Y/Z -----------------------------------------------
// Requires: set debug_mode = MAG_CALIB; save  in BF CLI
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

// -- MSP_ALTITUDE (109) --------------------------------------------------------
bool DroneLink::parseBaro(const std::vector<uint8_t>& buf, DroneState& s) {
    BaroReading raw;
    if (!BaroBMP280::parse(buf, raw)) { s.baroValid = false; return false; }
    s.baroAltitudeCm = raw.altitudeCm;
    s.baroVarioCmPerSec = raw.varioCmPerSec;
    s.baroValid = true;
    return true;
}

// -- MSP_RAW_GPS (106) ---------------------------------------------------------
bool DroneLink::parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseRaw(buf, s.gps)) { s.gps.rawValid = false; return false; }
    return true;
}

// -- MSP_COMP_GPS (107) --------------------------------------------------------
bool DroneLink::parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseComp(buf, s.gps)) { s.gps.compValid = false; return false; }
    return true;
}

// -- MSP_NAV_STATUS (121) ------------------------------------------------------
bool DroneLink::parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    return GPSNeoM10::parseNavStatus(buf, s.navStatus);
}

// =============================================================================
// commitState()
// =============================================================================

void DroneLink::commitState(const DroneState& s) {
    std::lock_guard<std::mutex> lock(dataMutex);
    currentState = s;
}

// =============================================================================
// AutoDetectF405() — scan COM1–COM29, return first port that opens
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