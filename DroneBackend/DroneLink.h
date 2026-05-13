#pragma once
#include <windows.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include "GPSNeoM10.h"           // GPSReading + GPSNeoM10 static parsers

// ─────────────────────────────────────────────────────────────────────────────
// MSP Command IDs — verified against Betaflight 4.5.x msp_protocol.h
//
// IMPORTANT — MSP_RAW_MAG DOES NOT EXIST IN BETAFLIGHT:
//   ID 130 = MSP_BATTERY_STATE in all Betaflight 4.x releases.
//   There is no MSP command that directly returns raw mag X/Y/Z.
//
// CORRECT MAG DATA PATH (official — same as BF Configurator Sensors tab):
//   1. Set  debug_mode = MAG_CALIB  in the Betaflight CLI.
//   2. Poll MSP_DEBUG (254) → debug[0]=magX, debug[1]=magY, debug[2]=magZ.
//
// DEBUG MODE MANAGEMENT:
//   The GCS must switch debug_mode between MAG_CALIB and FLIGHT_CONTROLLER
//   (or whatever the user's preferred mode is) so that:
//     - During normal operation: FLIGHT_CONTROLLER debug data is available.
//     - While mag widget is open / mag cal active: MAG_CALIB provides X/Y/Z.
//   This is done via MSP_SET_DEBUG_CONFIG (not a real MSP command — we use
//   the Betaflight CLI passthrough approach via MSP_DEBUGMSG / direct CLI).
//   Simplest GCS approach: keep MAG_CALIB set permanently in BF CLI, and
//   expose fcCycleMs via MSP_STATUS (101) instead of MSP_DEBUG.
//
// CALIBRATION COMMANDS:
//   MSP_ACC_CALIBRATION (205) — triggers accelerometer/gyro calibration.
//   MSP_MAG_CALIBRATION (206) — triggers magnetometer calibration (30s window).
//   Both are fire-and-forget: FC acks with an empty response frame.
//
// GPS SATELLITE DATA:
//   Per-satellite data (GNSS | SV | Signal | dBHz | Status | Quality) is NOT
//   available via standard MSP in Betaflight 4.5.x.  It is obtained via
//   MSP_SET_PASSTHROUGH (0xF5) which puts BF into GPS UART passthrough mode.
//   DroneLink uses this to poll UBX-NAV-SAT directly from the NEO-M10 every
//   30 seconds.  The MSP loop is briefly interrupted (~1.5 s) for each poll.
//   See step 10 in communicationLoop() for the full protocol sequence.
//
//   REQUIRED CONFIGURATION (one time in BF Configurator):
//     Ports tab → GPS UART row → activate "Serial Rx" OR keep it as GPS function.
//     The passthrough works as long as the GPS UART is assigned in Ports.
//     Set gpsUartIndex_ to match: UART1=0, UART2=1, UART3=2, etc.
//     F405 V3 ships with GPS on UART2 by default → gpsUartIndex_ = 1.
// ─────────────────────────────────────────────────────────────────────────────
namespace MSP {
    // ── Telemetry — out messages (FC → host) ─────────────────────────────────
    constexpr uint8_t STATUS = 101;  // FC cycle time, arming flags, sensor status
    constexpr uint8_t RAW_IMU = 102;  // accel + gyro raw (9 DOF)
    constexpr uint8_t RAW_GPS = 106;  // fix, sats, lat/lon, alt, speed, course, HDOP
    constexpr uint8_t COMP_GPS = 107;  // distance-to-home, bearing, heartbeat
    constexpr uint8_t ATTITUDE = 108;  // fused roll / pitch / yaw (deg × 10)
    constexpr uint8_t ALTITUDE = 109;  // BMP280 fused altitude (cm) + vario (cm/s)
    constexpr uint8_t ANALOG = 110;  // battery voltage, mAh drawn, RSSI
    constexpr uint8_t NAV_STATUS = 121;  // GPS nav engine status + fix flags
    constexpr uint8_t DEBUG = 254;  // 4 × int16_t debug values (mode-dependent)

    // ── Calibration — in messages (host → FC, fire-and-forget) ───────────────
    constexpr uint8_t ACC_CAL = 205;  // MSP_ACC_CALIBRATION — gyro + accel
    constexpr uint8_t MAG_CAL = 206;  // MSP_MAG_CALIBRATION — magnetometer

    // ── GPS UART passthrough ──────────────────────────────────────────────────
    // MSP_SET_PASSTHROUGH (245 = 0xF5)
    // Payload: [serialPortIdentifier (1 byte)]
    //   0 = UART1, 1 = UART2, 2 = UART3, etc.
    // After sending this command, BF:
    //   1. Sends an MSP ACK ($M> ...).
    //   2. Enters raw passthrough mode: bytes on the MSP UART are forwarded
    //      verbatim to the target UART, and vice versa.
    //   3. MSP protocol processing is SUSPENDED until the host serial port
    //      disconnects (USB virtual COM close) or BF is reset.
    // IMPORTANT: the payload is ONE byte (UART index), NOT the UBX frame.
    //   Wrapping UBX data as the MSP payload is WRONG — BF will interpret
    //   0xB5 (the UBX preamble) as UART index 181, which doesn't exist.
    constexpr uint8_t SET_PASSTHROUGH = 245;  // 0xF5

    // ── Battery (for reference — do NOT send as a telemetry request) ─────────
    // constexpr uint8_t BATTERY_STATE = 130;  // NOT RAW_MAG — battery data only
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibration durations
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int MAG_CAL_DURATION_S = 30;  // BF holds mag-cal mode for 30 s
static constexpr int ACC_CAL_DURATION_S = 5;  // gyro+accel cal completes in ~5 s

// ─────────────────────────────────────────────────────────────────────────────
// DroneState
//
// Written exclusively by the worker thread.
// Read by Python via getLatestState() — mutex-protected snapshot copy.
// ─────────────────────────────────────────────────────────────────────────────
struct DroneState {
    // ── IMU — MPU-6500 raw ADC counts (not scaled) ───────────────────────────
    int16_t ax = 0, ay = 0, az = 0;   // accelerometer
    int16_t gx = 0, gy = 0, gz = 0;   // gyroscope

    // ── Attitude — degrees × 10 as sent by FC; divide by 10.0 in Python ─────
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;   // yaw is already in full degrees (not × 10)

    // ── Power ─────────────────────────────────────────────────────────────────
    float   batteryVoltage = 0.0f;   // volts (buf[5] / 10.0)
    uint8_t rssi = 0;      // 0–255

    // ── Barometer — BMP280 via MSP_ALTITUDE (109) ────────────────────────────
    // Requires: Barometer enabled in BF Configurator → Configuration → Sensors
    int32_t baroAltitudeCm = 0;
    int16_t baroVarioCmPerSec = 0;
    bool    baroValid = false;

    // ── Magnetometer — QMC5883L via MSP_DEBUG (254) + debug_mode=MAG_CALIB ──
    int16_t magX = 0;
    int16_t magY = 0;
    int16_t magZ = 0;
    float   magHeadingDeg = 0.0f;
    bool    magValid = false;

    // ── Magnetometer calibration state ───────────────────────────────────────
    bool magCalActive = false;
    int  magCalSecondsRemaining = 0;

    // ── Gyro/Accel calibration state ─────────────────────────────────────────
    bool accCalActive = false;
    int  accCalSecondsRemaining = 0;

    // ── GPS — NEO-M10 via MSP_RAW_GPS (106) + MSP_COMP_GPS (107) ────────────
    GPSReading gps;

    // ── Diagnostics ──────────────────────────────────────────────────────────
    double   lastRttMs = 0.0;
    double   fcCycleMs = 0.0;
    bool     linkHealthy = false;
    uint32_t packetCount = 0;

    // ── GPS satellite list (UBX-NAV-SAT via BF GPS UART passthrough) ─────────
    // Populated every 30 s via the MSP_SET_PASSTHROUGH → UBX-NAV-SAT cycle.
    // Empty until the first successful passthrough poll completes.
    std::vector<SVInfoEntry> svList;
    bool                     svInfoValid = false;

    // ── GPS nav engine status (MSP_NAV_STATUS 121) ────────────────────────────
    NavStatus navStatus;
};

// ─────────────────────────────────────────────────────────────────────────────
// DroneLink
// ─────────────────────────────────────────────────────────────────────────────
class DroneLink {
public:
    static constexpr int POLL_INTERVAL_MS = 10;  // 100 Hz poll rate
    static constexpr int FAIL_THRESHOLD = 5;  // consecutive failures before unhealthy

    DroneLink();
    ~DroneLink();

    // ── Connection ────────────────────────────────────────────────────────────
    bool connect(const std::string& portName);
    void disconnect();
    bool isConnected() const { return connected.load(); }

    // ── State snapshot ────────────────────────────────────────────────────────
    DroneState getLatestState();

    // ── Calibration triggers ─────────────────────────────────────────────────
    void startMagCalibration();
    void startAccCalibration();

    // ── Tuning ────────────────────────────────────────────────────────────────
    void setPollIntervalMs(int ms) { pollIntervalMs.store(ms); }

    // ── GPS satellite passthrough configuration ───────────────────────────────
    // Set the BF serial port index that the GPS module is assigned to.
    // Match this to the UART shown in BF Configurator → Ports tab.
    //   UART1 → index 0
    //   UART2 → index 1  (F405 V3 default)
    //   UART3 → index 2
    // Default: 1 (UART2). Call before connect() if your FC uses a different UART.
    void setGpsUartIndex(uint8_t idx) { gpsUartIndex_ = idx; }

    // ── GPS config via UBX passthrough ────────────────────────────────────────
    GPSConfigResult applyGPSConfig(const GPSConfig& cfg);

private:
    HANDLE            hSerial;
    std::atomic<bool> connected;
    std::thread       workerThread;
    std::atomic<bool> keepRunning;
    std::atomic<int>  pollIntervalMs;

    // Saved for serial reconnects inside the passthrough cycle.
    std::string portName_;

    // GPS UART index for MSP_SET_PASSTHROUGH.
    // Matches BF Configurator Ports tab: UART1=0, UART2=1, UART3=2, …
    uint8_t gpsUartIndex_;

    // Calibration request flags (set from Python thread, consumed by worker)
    std::atomic<bool> magCalRequested;
    std::atomic<bool> accCalRequested;

    // SV info poll timestamp (satellite data is refreshed every 30 s)
    std::chrono::steady_clock::time_point lastSvPollTime_;

    // Variable-length UBX response reader (used for SVINFO passthrough)
    std::vector<uint8_t> readUbxResponse(int timeoutMs = 150);

    // Calibration countdown state (worker-thread only, committed via commitState)
    bool                                  magCalActive_;
    std::chrono::steady_clock::time_point magCalStartTime_;
    bool                                  accCalActive_;
    std::chrono::steady_clock::time_point accCalStartTime_;

    mutable std::mutex dataMutex;
    DroneState         currentState;

    // ── Worker ────────────────────────────────────────────────────────────────
    void communicationLoop();

    // ── Serial port helpers ───────────────────────────────────────────────────
    // Opens portName, configures DCB (115200 8N1, no flow control), and sets
    // non-blocking read timeouts.  Used by both connect() and the passthrough
    // reconnect cycle in step 10 of communicationLoop().
    // Does NOT start the worker thread — that is connect()'s responsibility.
    // Returns false if the port cannot be opened or configured.
    bool setupSerialPort(const std::string& portName);

    // ── MSP transport ─────────────────────────────────────────────────────────
    std::vector<uint8_t> sendMSP(uint8_t mspID);

    // ── Parsers — one per MSP response type ───────────────────────────────────
    bool parseStatus(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseIMU(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAttitude(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAnalog(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseDebug(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseBaro(const std::vector<uint8_t>& buf, DroneState& s);

    // GPS parsers — thin wrappers around GPSNeoM10 static methods
    bool parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s);

    void commitState(const DroneState& s);
};

// ─────────────────────────────────────────────────────────────────────────────
// AutoDetectF405() — scans COM1–COM29, returns first port that opens
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoDetectF405();