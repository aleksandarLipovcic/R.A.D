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
// ─────────────────────────────────────────────────────────────────────────────
namespace MSP {
    constexpr uint8_t STATUS = 101;
    constexpr uint8_t RAW_IMU = 102;
    constexpr uint8_t RAW_GPS = 106;
    constexpr uint8_t COMP_GPS = 107;
    constexpr uint8_t ATTITUDE = 108;
    constexpr uint8_t ALTITUDE = 109;
    constexpr uint8_t ANALOG = 110;
    constexpr uint8_t NAV_STATUS = 121;
    constexpr uint8_t GPS_SV_INFO = 164;   // MSP_GPS_SV_INFO -- satellite list
    constexpr uint8_t DEBUG = 254;
    constexpr uint8_t ACC_CAL = 205;
    constexpr uint8_t MAG_CAL = 206;

    // ── MSP_SET_PASSTHROUGH (245 / 0xF5) ─────────────────────────────────────
    //
    // HOW BETAFLIGHT GPS PASSTHROUGH ACTUALLY WORKS (BF 4.5.x)
    // ──────────────────────────────────────────────────────────
    // TWO-STEP protocol — NOT "wrap UBX in MSP payload".
    //
    // STEP 1 — Activate passthrough
    //   Host → FC:  MSP_SET_PASSTHROUGH with ONE-BYTE payload = GPS UART index
    //               UART1=0, UART2=1, UART3=2 (match BF Configurator Ports tab)
    //   FC → Host:  Standard MSP ACK ($M> 0x00 0xF5 checksum)
    //   After ACK:  BF suspends MSP and enters raw byte-forwarding mode.
    //
    // STEP 2 — Talk directly to GPS
    //   Host → FC:  Raw UBX bytes — NO MSP framing. FC forwards verbatim.
    //   FC → Host:  Raw UBX response bytes — NO MSP framing.
    //
    // CRITICAL: BF 4.x NEVER exits passthrough on its own.
    //   Only recovery = close + reopen the serial port.
    //   The USB-serial chip reset (CP210x/CH340) drives the FC state machine
    //   back to MSP mode.  Takes ~150–250 ms per cycle.
    //
    // ── BAUD RATE ARCHITECTURE (two completely independent settings) ──────────
    //
    //   MSP baud (DroneLink ↔ FC over USB)
    //   ────────────────────────────────────
    //   Confirmed by sv_baud_probe.py: FC USB/MSP port = 57600.
    //   Set in openSerialPort() → dcb.BaudRate = MSP_BAUD.
    //   This is what DroneLink uses for ALL communication with the FC,
    //   including the passthrough handshake bytes.
    //
    //   GPS UART baud (BF ↔ NEO-M10 inside the FC)
    //   ─────────────────────────────────────────────
    //   Set in BF Configurator → Ports → UART1 (GPS row) = 115200.
    //   BF negotiates this with the NEO-M10 on boot via gps_auto_baud=ON.
    //   DroneLink never touches this — BF handles it transparently.
    //   During passthrough BF bridges the two UARTs at their respective speeds.
    //
    //   Summary for this project:
    //     DroneLink ──57600──► FC USB  ──115200──► NEO-M10
    //     (MSP baud)            (BF internal GPS UART baud)
    constexpr uint8_t SET_PASSTHROUGH = 245;   // 0xF5
}

// ─────────────────────────────────────────────────────────────────────────────
// Baud rates
// ─────────────────────────────────────────────────────────────────────────────

// MSP_BAUD — confirmed by sv_baud_probe.py for this FC.
// This is the baud DroneLink uses to talk to the FC over USB.
// Change only if the probe reports a different baud for a new FC.
static constexpr DWORD MSP_BAUD = 57600;

// ─────────────────────────────────────────────────────────────────────────────
// Calibration durations
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int MAG_CAL_DURATION_S = 30;
static constexpr int ACC_CAL_DURATION_S = 5;

// ─────────────────────────────────────────────────────────────────────────────
// SV poll cadence
//
// SV_POLL_TICKS — how many communicationLoop() iterations between satellite
// list refreshes via MSP_GPS_SV_INFO (cmd 164).
//
// At POLL_INTERVAL_MS = 10 ms (100 Hz loop):
//   100 ticks = ~1 s refresh  (recommended: matches BF Configurator rate)
//   500 ticks = ~5 s refresh
//
// Unlike the old UBX passthrough approach, MSP cmd 164 is a normal MSP query
// that takes ~1-2 ms and does NOT close or reopen the serial port.
// There is no penalty for polling frequently.
//
// The old SV_POLL_INTERVAL_S (30 s) was necessary because the passthrough
// cycle required a port close/reopen cycle (~300-600 ms of link interruption).
// That constraint is gone -- MSP cmd 164 is fire-and-forget.
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int SV_POLL_TICKS = 100;   // ~1 s at 100 Hz poll rate

// ─────────────────────────────────────────────────────────────────────────────
// DroneState — written exclusively by the worker thread,
// read by Python via getLatestState() (mutex-protected snapshot copy).
// ─────────────────────────────────────────────────────────────────────────────
struct DroneState {
    // ── IMU — MPU-6500 raw ADC counts ────────────────────────────────────────
    int16_t ax = 0, ay = 0, az = 0;   // accelerometer
    int16_t gx = 0, gy = 0, gz = 0;   // gyroscope

    // ── Attitude — degrees × 10 (roll/pitch); full degrees (yaw) ─────────────
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;

    // ── Power ─────────────────────────────────────────────────────────────────
    float   batteryVoltage = 0.0f;   // volts (buf[5] / 10.0)
    uint8_t rssi = 0;      // 0–255

    // ── Barometer — BMP280 via MSP_ALTITUDE (109) ─────────────────────────────
    int32_t baroAltitudeCm = 0;
    int16_t baroVarioCmPerSec = 0;
    bool    baroValid = false;

    // ── Magnetometer — QMC5883L via MSP_DEBUG (254) + debug_mode=MAG_CALIB ───
    // Requires: set debug_mode = MAG_CALIB; save  in BF CLI
    int16_t magX = 0, magY = 0, magZ = 0;
    float   magHeadingDeg = 0.0f;    // tilt-uncorrected 2-D heading, 0–360°
    bool    magValid = false;

    // ── Calibration state ─────────────────────────────────────────────────────
    bool magCalActive = false;
    int  magCalSecondsRemaining = 0;
    bool accCalActive = false;
    int  accCalSecondsRemaining = 0;

    // ── GPS — NEO-M10 via MSP_RAW_GPS (106) + MSP_COMP_GPS (107) ─────────────
    GPSReading gps;

    // ── Satellite list — MSP_GPS_SV_INFO (cmd 164), polled every SV_POLL_TICKS
    //
    // Populated every ~1 s (SV_POLL_TICKS iterations at 100 Hz).
    // Uses a direct MSP query — no passthrough, no port cycle.
    // sv_info_valid becomes true on the first successful parse (~1 s after
    // connect, as opposed to the old 30 s passthrough delay).
    //
    // sv_source identifies which data path filled the list:
    //   "MSP"  — MSP_GPS_SV_INFO cmd 164 (primary, always available)
    //   "UBX"  — UBX-NAV-SAT passthrough (fallback, only if passthrough works)
    //   ""     — not yet populated
    //
    // Fields available in MSP mode: gnss_name, svid, cno, quality, status_str,
    //   used, gnss_id, flags, chn.
    // Fields always 0 in MSP mode (not in cmd 164 payload): elev, azim, prRes.
    std::vector<SVInfoEntry> svList;
    bool                     svInfoValid = false;
    std::string              svSource;      // "MSP" or "UBX"

    // ── GPS nav engine status — MSP_NAV_STATUS (121) ──────────────────────────
    NavStatus navStatus;

    // ── Diagnostics ──────────────────────────────────────────────────────────
    double   lastRttMs = 0.0;
    double   fcCycleMs = 0.0;   // from MSP_STATUS (101)
    bool     linkHealthy = false;
    uint32_t packetCount = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// DroneLink
// ─────────────────────────────────────────────────────────────────────────────
class DroneLink {
public:
    static constexpr int POLL_INTERVAL_MS = 10;   // 100 Hz MSP poll rate
    static constexpr int FAIL_THRESHOLD = 5;    // consecutive fails → unhealthy

    DroneLink();
    ~DroneLink();

    // ── Connection ────────────────────────────────────────────────────────────
    bool connect(const std::string& portName);
    void disconnect();
    bool isConnected() const { return connected.load(); }

    // ── State snapshot (thread-safe copy) ─────────────────────────────────────
    DroneState getLatestState();

    // ── Calibration triggers ─────────────────────────────────────────────────
    void startMagCalibration();
    void startAccCalibration();

    // ── GPS UART index for MSP_SET_PASSTHROUGH ────────────────────────────────
    // Only used by applyGPSConfig() (UBX config writes).
    // NOT used for satellite polling (MSP cmd 164 needs no UART index).
    //   UART1 → 0   ← GPS confirmed on UART1 for this project
    //   UART2 → 1
    //   UART3 → 2
    // Default = 0.  Call before connect().
    void setGpsUartIndex(uint8_t idx) { gpsUartIndex_ = idx; }

    // ── Poll rate tuning ──────────────────────────────────────────────────────
    void setPollIntervalMs(int ms) { pollIntervalMs.store(ms); }

    // ── GPS configuration via UBX passthrough ─────────────────────────────────
    GPSConfigResult applyGPSConfig(const GPSConfig& cfg);

private:
    HANDLE            hSerial;
    std::atomic<bool> connected;
    std::thread       workerThread;
    std::atomic<bool> keepRunning;
    std::atomic<int>  pollIntervalMs;

    std::string portName_;          // saved for reopen after passthrough cycle

    // GPS UART index — only used by applyGPSConfig() passthrough sessions.
    // MSP cmd 164 satellite polling does NOT use this.
    uint8_t gpsUartIndex_ = 0;

    // Calibration request flags (set from Python thread, consumed by worker)
    std::atomic<bool> magCalRequested;
    std::atomic<bool> accCalRequested;

    // SV poll tick counter — incremented every communicationLoop() iteration.
    // When it reaches SV_POLL_TICKS the satellite list is refreshed and
    // the counter resets to 0.  No time-based scheduling needed.
    int svPollTickCounter_ = 0;

    // Calibration countdown state (worker-thread only, committed via commitState)
    bool                                  magCalActive_;
    std::chrono::steady_clock::time_point magCalStartTime_;
    bool                                  accCalActive_;
    std::chrono::steady_clock::time_point accCalStartTime_;

    mutable std::mutex dataMutex;
    DroneState         currentState;

    // ── Worker thread ─────────────────────────────────────────────────────────
    void communicationLoop();

    // ── Serial port helpers ───────────────────────────────────────────────────
    bool openSerialPort(const std::string& portName);
    void closeSerialPort();

    // ── MSP transport ─────────────────────────────────────────────────────────
    std::vector<uint8_t> sendMSP(uint8_t mspID);

    // ── Satellite list — MSP path (primary) ───────────────────────────────────
    // Sends MSP_GPS_SV_INFO (cmd 164) and parses the response directly.
    // No passthrough, no port cycle.  Called every SV_POLL_TICKS iterations.
    bool pollSatellitesMSP(DroneState& pending);

    // ── UBX passthrough cycle (kept for applyGPSConfig only) ─────────────────
    // pollSatellites() is no longer called from communicationLoop().
    // It is retained so applyGPSConfig() can reuse readUbxResponse().
    void pollSatellites(DroneState& pending);

    // readUbxResponse() synchronises on the 0xB5 0x62 preamble and returns
    // the complete UBX frame (preamble + header + payload + checksums).
    std::vector<uint8_t> readUbxResponse(HANDLE h, int timeoutMs = 300);

    // ── Parsers — one per MSP response type ───────────────────────────────────
    bool parseStatus(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseIMU(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAttitude(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAnalog(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseDebug(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseBaro(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseMspSvInfo(const std::vector<uint8_t>& buf, DroneState& s);
    void commitState(const DroneState& s);
};

// ─────────────────────────────────────────────────────────────────────────────
// AutoDetectF405() — scans COM1–COM29, returns first port that opens
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoDetectF405();