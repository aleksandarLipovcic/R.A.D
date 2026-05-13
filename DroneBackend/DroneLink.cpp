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
    // Initialise lastSvPollTime_ to the epoch so the first tick fires
    // immediately rather than waiting a full second.
    lastSvPollTime_ = std::chrono::steady_clock::time_point{};
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

// =============================================================================
// communicationLoop()
//
// Poll order per loop tick (10 MSP/UBX commands):
//   1.  MSP_STATUS        (101) -- FC cycle time, sensor status flags
//   2.  MSP_RAW_IMU       (102) -- raw accel + gyro (MPU-6500)
//   3.  MSP_ATTITUDE      (108) -- fused roll / pitch / yaw
//   4.  MSP_ANALOG        (110) -- battery voltage, RSSI
//   5.  MSP_DEBUG         (254) -- mag X/Y/Z when debug_mode = MAG_CALIB
//   6.  MSP_ALTITUDE      (109) -- BMP280 altitude + vario
//   7.  MSP_RAW_GPS       (106) -- GPS fix, sats, lat/lon, alt, speed, HDOP
//   8.  MSP_COMP_GPS      (107) -- distance + bearing to home, GPS heartbeat
//   9.  MSP_NAV_STATUS    (121) -- GPS nav engine status + fix flags
//  10.  UBX-NAV-SVINFO via MSP passthrough (1 Hz, not every tick)
//
// UBX-NAV-SVINFO passthrough detail
// -----------------------------------
// Betaflight's MSP passthrough mechanism works as follows when a
// UBX poll is sent:
//   GCS → FC: MSP_PASSTHROUGH frame containing the 8-byte UBX poll
//   FC  → GCS: (a) 6-byte MSP ACK ($M> 0x00 0xF5 checksum)
//              (b) raw UBX response bytes forwarded from NEO-M10
//
// readUbxResponse() handles this by scanning for the UBX preamble
// 0xB5 0x62 in the incoming byte stream, discarding everything before
// it (including the MSP ACK). Only after locking onto the preamble
// does it start framing the UBX packet.
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

        // -- 9. GPS Nav Status (MSP_NAV_STATUS 121) ---------------------------
        {
            auto buf = sendMSP(MSP::NAV_STATUS);
            parseNavStatus(buf, pending);
        }

        // -- 10. SV Info via UBX passthrough (rate-limited to 1 Hz) ----------
        //
        // Isolation from the MSP poll cycle:
        //   The MSP commands above all use sendMSP() which calls PurgeComm()
        //   before each write. By the time we reach here the RX buffer has
        //   been flushed by the last sendMSP() call (#9). We do one more
        //   explicit PurgeComm() immediately before our write to guarantee
        //   a clean buffer, then call readUbxResponse() which scans for the
        //   UBX preamble 0xB5 0x62, correctly skipping the MSP ACK that
        //   Betaflight prepends to the forwarded UBX data.
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastSvPollTime_).count();

            if (elapsed >= 1000 && hSerial != INVALID_HANDLE_VALUE) {
                lastSvPollTime_ = now;

                auto ubxPoll = GPSNeoM10::buildNavSvInfoPoll();
                auto mspFrame = GPSNeoM10::wrapUbxInMspPassthrough(ubxPoll);

                // Flush RX buffer immediately before writing so the
                // readUbxResponse() call below sees a clean stream.
                PurgeComm(hSerial, PURGE_RXCLEAR);

                DWORD written = 0;
                WriteFile(hSerial,
                    mspFrame.data(),
                    static_cast<DWORD>(mspFrame.size()),
                    &written, NULL);

                // Give the FC time to:
                //   (a) send the 6-byte MSP ACK for the passthrough command
                //   (b) forward the 8-byte UBX poll to the NEO-M10 UART
                //   (c) receive the UBX-NAV-SVINFO response from the NEO-M10
                //       and forward it back to the host
                // The NEO-M10 responds within ~50 ms; 20 ms covers (a)+(b).
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                // Read the raw byte stream, scanning for the UBX preamble.
                // The MSP ACK bytes ($M>...) are silently discarded by the
                // preamble-sync logic inside readUbxResponse().
                auto ubxResp = readUbxResponse(220);

                if (ubxResp.size() >= 10) {
                    // ubxResp starts at 0xB5 (preamble byte 1).
                    // Layout: [0]=0xB5 [1]=0x62 [2]=class [3]=id
                    //         [4]=len_lo [5]=len_hi [6..N-2]=payload [N-1][N]=CK
                    // Verify it is the SVINFO response (class=0x01, id=0x30).
                    if (ubxResp[2] == 0x01 && ubxResp[3] == 0x30) {
                        uint16_t payLen =
                            static_cast<uint16_t>(ubxResp[4]) |
                            (static_cast<uint16_t>(ubxResp[5]) << 8);

                        // Slice out payload bytes [6 .. 6+payLen-1]
                        if (ubxResp.size() >= static_cast<size_t>(6 + payLen + 2)) {
                            std::vector<uint8_t> payload(
                                ubxResp.begin() + 6,
                                ubxResp.begin() + 6 + payLen);

                            std::vector<SVInfoEntry> sv;
                            if (GPSNeoM10::parseNavSvInfo(payload, sv)) {
                                pending.svList = std::move(sv);
                                pending.svInfoValid = true;
                            }
                        }
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
//
// MAX_FRAME is set to 512 to accommodate any MSP response. Note that
// UBX-NAV-SVINFO responses (up to ~400 bytes) are NOT read by sendMSP() --
// they are handled by readUbxResponse() which has its own larger buffer.
// =============================================================================

std::vector<uint8_t> DroneLink::sendMSP(uint8_t mspID) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    PurgeComm(hSerial, PURGE_RXCLEAR);

    uint8_t req[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written = 0;
    if (!WriteFile(hSerial, req, sizeof(req), &written, NULL) || written != sizeof(req))
        return {};

    // 512 bytes is sufficient for all standard MSP responses.
    // MSP frames larger than this do not exist in Betaflight 4.x.
    constexpr size_t MAX_FRAME = 512;
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

        // Full MSP frame = preamble(3) + len(1) + cmd(1) + payload(N) + csum(1)
        if (payloadLen >= 0 &&
            total == static_cast<size_t>(payloadLen + 6))
            break;
    }

    if (total < 6) return {};
    return std::vector<uint8_t>(raw, raw + total);
}

// =============================================================================
// readUbxResponse()
//
// Reads a complete UBX frame from the serial port, synchronising on the
// two-byte preamble 0xB5 0x62.
//
// WHY PREAMBLE SYNC IS REQUIRED
// ──────────────────────────────
// When a UBX poll is sent via MSP_PASSTHROUGH, Betaflight first sends a
// 6-byte MSP ACK response ($M> 0x00 0xF5 checksum) before forwarding the
// raw UBX bytes from the NEO-M10. Without preamble sync, the ACK bytes
// would be misinterpreted as part of a UBX frame:
//
//   '$'=0x24 'M'=0x4D '>'=0x3E 0x00 0xF5 0xF5   <- MSP ACK (6 bytes)
//   0xB5 0x62 0x01 0x30 ...                       <- UBX-NAV-SVINFO starts here
//
// If the reader starts at byte 0 ($), then at byte 4 it reads 0xF5 0xF5 as
// the UBX length field = 0xF5F5 = 62965. The loop would then try to read
// 62971 bytes, hit the buffer cap, and return garbage.
//
// The sync loop discards bytes until it sees 0xB5 followed immediately by
// 0x62. From that point the UBX frame is read deterministically.
//
// FRAME LAYOUT (after sync)
// ─────────────────────────
//   [0] 0xB5        preamble byte 1  (already consumed by sync)
//   [1] 0x62        preamble byte 2  (already consumed by sync)
//   [2] class       message class
//   [3] id          message id
//   [4] len_lo      payload length, little-endian low byte
//   [5] len_hi      payload length, little-endian high byte
//   [6..6+N-1]      payload (N = len_lo | len_hi<<8)
//   [6+N]  CK_A     Fletcher-8 checksum byte A
//   [6+N+1] CK_B    Fletcher-8 checksum byte B
//
// The returned vector starts at the 0xB5 preamble byte and includes both
// checksum bytes, so total size = 8 + payloadLength.
// =============================================================================

std::vector<uint8_t> DroneLink::readUbxResponse(int timeoutMs) {
    if (hSerial == INVALID_HANDLE_VALUE) return {};

    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(timeoutMs);

    auto readByte = [&](uint8_t& out) -> bool {
        while (std::chrono::steady_clock::now() < deadline) {
            DWORD rd = 0;
            if (ReadFile(hSerial, &out, 1, &rd, NULL) && rd == 1)
                return true;
            std::this_thread::sleep_for(std::chrono::microseconds(150));
        }
        return false;
        };

    // ── Phase 1: scan for preamble 0xB5 0x62 ─────────────────────────────────
    // Discards all bytes until the two-byte UBX sync sequence is found.
    // This cleanly skips the 6-byte MSP ACK that Betaflight prepends.
    {
        uint8_t prev = 0, curr = 0;
        while (true) {
            if (!readByte(curr)) return {};   // timeout
            if (prev == 0xB5 && curr == 0x62)
                break;                         // preamble found
            prev = curr;
        }
    }

    // ── Phase 2: read the fixed 4-byte UBX header (class + id + length) ──────
    uint8_t header[4];
    for (int i = 0; i < 4; ++i) {
        if (!readByte(header[i])) return {};
    }

    uint16_t payloadLen =
        static_cast<uint16_t>(header[2]) |
        (static_cast<uint16_t>(header[3]) << 8);

    // Sanity check: UBX-NAV-SVINFO for 32 SVs = 8+384 = 392 bytes payload.
    // Reject anything implausibly large to guard against framing errors.
    if (payloadLen > 2048) return {};

    // ── Phase 3: read payload + 2 checksum bytes ──────────────────────────────
    std::vector<uint8_t> frame;
    frame.reserve(8 + payloadLen);

    // Re-prepend the preamble so the caller can index the frame from byte 0
    // with the standard UBX layout (makes payload slicing straightforward).
    frame.push_back(0xB5);
    frame.push_back(0x62);
    for (int i = 0; i < 4; ++i) frame.push_back(header[i]);

    for (uint32_t i = 0; i < static_cast<uint32_t>(payloadLen) + 2u; ++i) {
        uint8_t b = 0;
        if (!readByte(b)) return {};
        frame.push_back(b);
    }

    return frame;   // starts at 0xB5, total size = 8 + payloadLen
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
bool DroneLink::parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseRaw(buf, s.gps)) {
        s.gps.rawValid = false;
        return false;
    }
    return true;
}

// -- MSP_COMP_GPS (107) -------------------------------------------------------
bool DroneLink::parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s) {
    if (!GPSNeoM10::parseComp(buf, s.gps)) {
        s.gps.compValid = false;
        return false;
    }
    return true;
}

// -- MSP_NAV_STATUS (121) -----------------------------------------------------
bool DroneLink::parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s) {
    return GPSNeoM10::parseNavStatus(buf, s.navStatus);
}

// =============================================================================
// applyGPSConfig()
//
// Sends each UBX config frame in sequence via MSP GPS passthrough,
// waiting for a UBX-ACK-ACK after each one.
//
// The sequence is:
//   1. CFG-GNSS  (constellation selection)
//   2. CFG-RATE  (navigation solution rate)
//   3. CFG-PRT   (UART protocol)
//   4. CFG-NAV5  (elevation mask + airborne dynamic model)
//   5. CFG-CFG   (save to BBR + flash)
//
// Each sendUbx() call:
//   1. Wraps the UBX frame in MSP_PASSTHROUGH
//   2. Flushes RX with PurgeComm
//   3. Writes to serial
//   4. Waits 20 ms for FC to relay to NEO-M10
//   5. Calls readUbxResponse() with 300 ms timeout
//   6. Checks response with parseAck()
// =============================================================================

GPSConfigResult DroneLink::applyGPSConfig(const GPSConfig& cfg) {
    GPSConfigResult result;
    if (!connected.load()) {
        result.errorDetail = "Not connected";
        return result;
    }

    auto sendUbx = [&](const std::vector<uint8_t>& ubx,
        uint8_t cls, uint8_t id) -> bool {
            auto frame = GPSNeoM10::wrapUbxInMspPassthrough(ubx);
            if (frame.empty()) return false;

            PurgeComm(hSerial, PURGE_RXCLEAR);

            DWORD written = 0;
            if (!WriteFile(hSerial, frame.data(),
                static_cast<DWORD>(frame.size()),
                &written, NULL))
                return false;

            // Let the FC relay the frame and the NEO-M10 prepare its ACK.
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            auto resp = readUbxResponse(300);
            return GPSNeoM10::parseAck(resp, cls, id);
        };

    result.gnssAck = sendUbx(GPSNeoM10::buildCfgGNSS(cfg.constellations), 0x06, 0x3E);
    result.rateAck = sendUbx(GPSNeoM10::buildCfgRate(cfg.updateRateHz), 0x06, 0x08);
    result.protocolAck = sendUbx(GPSNeoM10::buildCfgPrt(cfg.protocol), 0x06, 0x00);
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