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
// ─────────────────────────────────────────────────────────────────────────────
namespace MSP {
    // ── Telemetry — out messages (FC → host) ─────────────────────────────────
    constexpr uint8_t STATUS = 101;  // FC cycle time, arming flags, sensor status
    constexpr uint8_t RAW_IMU = 102;  // accel + gyro raw (9 DOF)
    constexpr uint8_t ATTITUDE = 108;  // fused roll / pitch / yaw (deg × 10)
    constexpr uint8_t ALTITUDE = 109;  // BMP280 fused altitude (cm) + vario (cm/s)
    constexpr uint8_t ANALOG = 110;  // battery voltage, mAh drawn, RSSI
    constexpr uint8_t DEBUG = 254;  // 4 × int16_t debug values (mode-dependent)

    // ── Calibration — in messages (host → FC, fire-and-forget) ───────────────
    constexpr uint8_t ACC_CAL = 205;  // MSP_ACC_CALIBRATION — gyro + accel
    constexpr uint8_t MAG_CAL = 206;  // MSP_MAG_CALIBRATION — magnetometer

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
    //
    // PREREQUISITE (Betaflight CLI — one time):
    //   set debug_mode = MAG_CALIB
    //   save
    //
    // When set, MSP_DEBUG returns:
    //   debug[0] = raw mag X (int16, ADC counts)
    //   debug[1] = raw mag Y
    //   debug[2] = raw mag Z
    //   debug[3] = heading error / calibration quality
    //
    // magValid is false (and all values are 0) when:
    //   a) debug_mode ≠ MAG_CALIB, OR
    //   b) Magnetometer sensor is not detected/enabled in BF
    int16_t magX = 0;
    int16_t magY = 0;
    int16_t magZ = 0;
    float   magHeadingDeg = 0.0f;   // tilt-uncorrected 2-D heading, 0–360°
    bool    magValid = false;

    // ── Magnetometer calibration state ───────────────────────────────────────
    bool magCalActive = false;
    int  magCalSecondsRemaining = 0;

    // ── Gyro/Accel calibration state ─────────────────────────────────────────
    bool accCalActive = false;
    int  accCalSecondsRemaining = 0;

    // ── Diagnostics ──────────────────────────────────────────────────────────
    double   lastRttMs = 0.0;
    double   fcCycleMs = 0.0;   // from MSP_STATUS (101); 0 if STATUS not polled
    bool     linkHealthy = false;
    uint32_t packetCount = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// DroneLink
// ─────────────────────────────────────────────────────────────────────────────
class DroneLink {
public:
    static constexpr int POLL_INTERVAL_MS = 10;   // 100 Hz poll rate
    static constexpr int FAIL_THRESHOLD = 5;   // consecutive failures before unhealthy

    DroneLink();
    ~DroneLink();

    // ── Connection ────────────────────────────────────────────────────────────
    bool connect(const std::string& portName);
    void disconnect();
    bool isConnected() const { return connected.load(); }

    // ── State snapshot ────────────────────────────────────────────────────────
    DroneState getLatestState();

    // ── Calibration triggers ─────────────────────────────────────────────────
    // startMagCalibration()  — sends MSP_MAG_CALIBRATION (206) to FC and starts
    //                          the 30-second countdown in DroneState.
    //                          Rotate the drone on all axes during this window.
    //
    // startAccCalibration()  — sends MSP_ACC_CALIBRATION (205) to FC and starts
    //                          the ~5-second countdown in DroneState.
    //                          Keep the drone perfectly level and still.
    void startMagCalibration();
    void startAccCalibration();

    // ── Tuning ────────────────────────────────────────────────────────────────
    void setPollIntervalMs(int ms) { pollIntervalMs.store(ms); }

private:
    HANDLE            hSerial;
    std::atomic<bool> connected;
    std::thread       workerThread;
    std::atomic<bool> keepRunning;
    std::atomic<int>  pollIntervalMs;

    // Calibration request flags (set from Python thread, consumed by worker)
    std::atomic<bool> magCalRequested;
    std::atomic<bool> accCalRequested;

    // Calibration countdown state (worker-thread only, committed via commitState)
    bool                                  magCalActive_;
    std::chrono::steady_clock::time_point magCalStartTime_;
    bool                                  accCalActive_;
    std::chrono::steady_clock::time_point accCalStartTime_;

    mutable std::mutex dataMutex;
    DroneState         currentState;

    // ── Worker ────────────────────────────────────────────────────────────────
    void communicationLoop();

    // ── MSP transport ────────────────────────────────────────────────────────
    std::vector<uint8_t> sendMSP(uint8_t mspID);

    // ── Parsers — one per MSP response type ──────────────────────────────────
    bool parseStatus(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseIMU(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAttitude(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAnalog(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseDebug(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseBaro(const std::vector<uint8_t>& buf, DroneState& s);

    void commitState(const DroneState& s);
};

// ─────────────────────────────────────────────────────────────────────────────
// AutoDetectF405() — scans COM1–COM29, returns first port that opens
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoDetectF405();