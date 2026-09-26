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

// =============================================================================
// MSP Command IDs — verified against Betaflight 4.5.x msp_protocol.h
// =============================================================================
namespace MSP {
    constexpr uint8_t STATUS = 101;
    constexpr uint8_t RAW_IMU = 102;
    constexpr uint8_t MOTOR = 104;   // MSP_MOTOR         — motor throttle outputs
    constexpr uint8_t RC = 105;   // MSP_RC            — RC channel inputs
    constexpr uint8_t RAW_GPS = 106;
    constexpr uint8_t COMP_GPS = 107;
    constexpr uint8_t ATTITUDE = 108;
    constexpr uint8_t ALTITUDE = 109;
    constexpr uint8_t ANALOG = 110;
    constexpr uint8_t NAV_STATUS = 121;
    constexpr uint8_t STATUS_EX = 150;   // MSP_STATUS_EX     — extended status + arming flags
    constexpr uint8_t GPS_SV_INFO = 164;   // MSP_GPS_SV_INFO   — satellite list
    constexpr uint8_t DEBUG = 254;
    constexpr uint8_t ACC_CAL = 205;
    constexpr uint8_t MAG_CAL = 206;
    constexpr uint8_t BATTERY_STATE = 242;   // MSP_BATTERY_STATE — full battery telemetry

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
    //
    //   GPS UART baud (BF ↔ NEO-M10 inside the FC)
    //   ─────────────────────────────────────────────
    //   Set in BF Configurator → Ports → UART1 (GPS row) = 115200.
    //   BF negotiates this with the NEO-M10 on boot via gps_auto_baud=ON.
    //   DroneLink never touches this — BF handles it transparently.
    //
    //   Summary for this project:
    //     DroneLink ──57600──► FC USB  ──115200──► NEO-M10
    //     (MSP baud)            (BF internal GPS UART baud)
    constexpr uint8_t SET_PASSTHROUGH = 245;   // 0xF5
}

// =============================================================================
// Baud rates
// =============================================================================

// MSP_BAUD — confirmed by sv_baud_probe.py for this FC.
static constexpr DWORD MSP_BAUD = 57600;

// =============================================================================
// Calibration durations
// =============================================================================
static constexpr int MAG_CAL_DURATION_S = 30;
static constexpr int ACC_CAL_DURATION_S = 5;

// =============================================================================
// Poll cadence constants
//
// SV_POLL_TICKS — satellite list refresh via MSP_GPS_SV_INFO (cmd 164).
//   100 ticks = ~1 s at 100 Hz.  No port-cycle cost; fire-and-forget.
//
// SLOW_POLL_TICKS — for MSP_BATTERY_STATE (242).
//   50 ticks = ~500 ms at 100 Hz.  Battery state changes slowly;
//   polling at 2 Hz is more than adequate and reduces loop pressure.
//
// Loop budget note: with 13 MSP polls per tick at ~1–2 ms each, the
// 10 ms (100 Hz) budget is tight on slow hardware. If communicationLoop()
// overruns, call setPollIntervalMs(20) for a 50 Hz rate which gives
// comfortable headroom.
// =============================================================================
static constexpr int SV_POLL_TICKS = 100;   // ~1 s at 100 Hz
static constexpr int SLOW_POLL_TICKS = 50;   // ~500 ms at 100 Hz

// =============================================================================
// Motor / RC channel array sizes
// =============================================================================
static constexpr int MAX_MOTORS = 8;    // Betaflight supports up to 8 motors
static constexpr int MAX_RC_CH = 18;   // up to 18 CRSF/ELRS channels

// =============================================================================
// BatteryState — MSP_BATTERY_STATE buf[13]
// =============================================================================
enum class BatteryState : uint8_t {
    OK = 0,
    WARNING = 1,
    CRITICAL = 2,
    NOT_PRESENT = 3,
    INIT = 4
};

// =============================================================================
// ArmingDisable bitmask — MSP_STATUS_EX buf[20..23]
// Set bits indicate WHY the FC refuses to arm.
// Source: BF 4.5.x src/main/fc/runtime_config.h
// =============================================================================
namespace ArmingDisable {
    constexpr uint32_t NO_GYRO = (1u << 0);
    constexpr uint32_t FAILSAFE = (1u << 1);
    constexpr uint32_t RX_FAILSAFE = (1u << 2);
    constexpr uint32_t BAD_RX_RECOVERY = (1u << 3);
    constexpr uint32_t BOXFAILSAFE = (1u << 4);
    constexpr uint32_t RUNAWAY_TAKEOFF = (1u << 5);
    constexpr uint32_t CRASH_DETECTED = (1u << 6);
    constexpr uint32_t THROTTLE = (1u << 7);   // throttle not at minimum
    constexpr uint32_t ANGLE = (1u << 8);   // FC not level enough
    constexpr uint32_t BOOT_GRACE_TIME = (1u << 9);
    constexpr uint32_t NOPREARM = (1u << 10);
    constexpr uint32_t LOAD = (1u << 11);   // CPU overloaded
    constexpr uint32_t CALIBRATING = (1u << 12);
    constexpr uint32_t CLI = (1u << 13);
    constexpr uint32_t CMS_MENU = (1u << 14);
    constexpr uint32_t BST = (1u << 15);
    constexpr uint32_t MSP = (1u << 16);
    constexpr uint32_t PARALYZE = (1u << 17);
    constexpr uint32_t GPS = (1u << 18);   // GPS not ready (GPS-rescue mode)
    constexpr uint32_t RESC_SW = (1u << 21);
    constexpr uint32_t DSHOT_BITBANG = (1u << 23);
    constexpr uint32_t ACC_CALIBRATION = (1u << 24);
    constexpr uint32_t MOTOR_PROTOCOL = (1u << 25);
    constexpr uint32_t ARM_SWITCH = (1u << 26);   // ARM switch is off
}

// =============================================================================
// SensorStatus bitmask — MSP_STATUS buf[9..10]
// =============================================================================
namespace SensorStatus {
    constexpr uint16_t ACC = (1u << 0);
    constexpr uint16_t BARO = (1u << 1);
    constexpr uint16_t MAG = (1u << 2);
    constexpr uint16_t GPS = (1u << 3);
    constexpr uint16_t RANGEFINDER = (1u << 4);
    constexpr uint16_t GYRO = (1u << 5);
}

// =============================================================================
// FlightMode bitmask — MSP_STATUS buf[11..14]
// Permanent box IDs: BF 4.5.x src/main/msp/msp_box.h
// =============================================================================
namespace FlightMode {
    constexpr uint32_t ARM = (1u << 0);
    constexpr uint32_t ANGLE = (1u << 1);
    constexpr uint32_t HORIZON = (1u << 2);
    constexpr uint32_t MAG = (1u << 3);   // mag-hold (requires compass)
    constexpr uint32_t HEADFREE = (1u << 4);
    constexpr uint32_t FAILSAFE = (1u << 8);
    constexpr uint32_t GPS_RESCUE = (1u << 16);
    constexpr uint32_t ANTI_GRAV = (1u << 24);
}

// =============================================================================
// RadioLinkStatus / RadioLinkStats — ELRS telemetry path (CrsfLink)
//
// Only filled when DroneState::linkSource == "ELRS". On the USB/MSP path
// (DroneLink) this block stays at its defaults.
// =============================================================================
enum class RadioLinkStatus : uint8_t {
    NO_RADIO = 0,   // no Pocket found on any COM port (unplugged / wrong USB mode)
    WAITING = 1,   // Pocket's COM port is open, but no telemetry frames yet
                    //   → drone off, not bound, or USB-VCP not set to Telem Mirror
    TELEMETRY_OK = 2,
    DEGRADED = 3,   // frames arriving, but LQ low or instruments going stale
    TELEMETRY_LOST = 4,   // had telemetry, frames stopped (drone out of range / RX lost)
};

struct RadioLinkStats {
    RadioLinkStatus status = RadioLinkStatus::NO_RADIO;
    std::string     statusStr = "NO_RADIO";
    std::string     portName;

    // ── ELRS LINK_STATISTICS (0x14) ──────────────────────────────────────────
    bool     linkStatsValid = false;
    int      uplinkRssi1Dbm = 0;     // drone RX antenna 1, dBm (negative)
    int      uplinkRssi2Dbm = 0;     // drone RX antenna 2 (RP4TD has diversity)
    uint8_t  uplinkLq = 0;     // % — THE number to watch in flight
    int8_t   uplinkSnr = 0;
    uint8_t  activeAntenna = 0;
    uint8_t  rfModeIndex = 0;
    uint16_t txPowerMw = 0;
    int      downlinkRssiDbm = 0;     // what the Pocket hears from the drone
    uint8_t  downlinkLq = 0;     // % of telemetry packets arriving
    int8_t   downlinkSnr = 0;

    // ── Freshness: ms since each data group was last updated (-1 = never) ────
    int64_t msSinceLastFrame = -1;
    int64_t attitudeAgeMs = -1;
    int64_t gpsAgeMs = -1;
    int64_t batteryAgeMs = -1;
    int64_t flightModeAgeMs = -1;
    int64_t baroAgeMs = -1;
    int64_t linkStatsAgeMs = -1;

    // ── Measured update rates (Hz, rolling ~2 s window) ─────────────────────
    float attitudeHz = 0.0f, gpsHz = 0.0f, batteryHz = 0.0f, flightModeHz = 0.0f;
    float baroHz = 0.0f, varioHz = 0.0f, linkStatsHz = 0.0f, totalFrameHz = 0.0f;

    // ── Counters ─────────────────────────────────────────────────────────────
    uint32_t framesTotal = 0;
    uint32_t crcErrors = 0;
    uint32_t reconnectCount = 0;   // times the Pocket was re-found after a USB drop
    uint32_t linkLostCount = 0;   // TELEMETRY_OK/DEGRADED → TELEMETRY_LOST transitions

    // ── Things CRSF only hints at (details need the USB link) ────────────────
    std::string rawFlightMode;           // e.g. "STAB", "ACRO*", "!ERR*"
    bool        armingBlocked = false;   // BF reports "!ERR" — reason list is MSP-only
    bool        gpsWaiting = false;   // BF reports "WAIT" — no GPS fix / home yet

    // ── Home point (computed on the laptop — CRSF has no home distance) ──────
    bool   homeSet = false;
    double homeLat = 0.0, homeLon = 0.0;
    float  homeAltM = 0.0f;

    // "BARO" (FC baro frame), "GPS" (GPS altitude relative to home), "NONE"
    std::string altitudeSource = "NONE";
};

// =============================================================================
// DroneState — written exclusively by the worker thread,
// read by Python via getLatestState() (mutex-protected snapshot copy).
// =============================================================================
struct DroneState {

    // ── IMU — MPU-6500 raw ADC counts ────────────────────────────────────────
    int16_t ax = 0, ay = 0, az = 0;   // accelerometer
    int16_t gx = 0, gy = 0, gz = 0;   // gyroscope

    // ── Attitude — degrees × 10 (roll/pitch); full degrees (yaw) ─────────────
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0;

    // ── Power — MSP_ANALOG (110) ──────────────────────────────────────────────
    float    batteryVoltage = 0.0f;   // Volts  (buf[5] / 10.0)
    float    batteryCurrent = 0.0f;   // Amps   (amperage centiamps / 100)
    uint16_t batteryMahDrawn = 0;      // mAh consumed
    uint8_t  rssi = 0;      // 0–255

    // ── Battery detail — MSP_BATTERY_STATE (242), polled every ~500 ms ────────
    //
    // batteryVoltage and batteryMahDrawn are shared with parseAnalog().
    // parseBatteryState() overwrites batteryVoltage with 10 mV-resolution
    // data (more accurate) and adds the fields below.
    uint8_t      batteryCellCount = 0;     // auto-detected cell count
    uint16_t     batteryCapacityMah = 0;     // design capacity in mAh
    uint8_t      batteryPercentage = 0;     // 0–100 % (computed from mAh)
    BatteryState batteryState = BatteryState::INIT;  // OK/WARNING/CRITICAL/NOT_PRESENT

    // ── Barometer — BMP280 via MSP_ALTITUDE (109) ─────────────────────────────
    int32_t baroAltitudeCm = 0;
    int16_t baroVarioCmPerSec = 0;
    bool    baroValid = false;

    // ── Magnetometer — QMC5883L via MSP_DEBUG (254) + debug_mode=MAG_CALIB ───
    // Requires: set debug_mode = MAG_CALIB; save  in BF CLI
    int16_t magX = 0, magY = 0, magZ = 0;
    float   magHeadingDeg = 0.0f;      // tilt-uncorrected 2-D heading, 0–360°
    bool    magValid = false;

    // ── Calibration state ─────────────────────────────────────────────────────
    bool magCalActive = false;
    int  magCalSecondsRemaining = 0;
    bool accCalActive = false;
    int  accCalSecondsRemaining = 0;

    // ── FC status — MSP_STATUS (101) extended ─────────────────────────────────
    //
    // armed / flightModeFlags / flightModeName come from STATUS (101).
    // armingDisableFlags / armingDisableStr come from STATUS_EX (150).
    bool        armed = false;
    uint32_t    flightModeFlags = 0;     // raw bitmask — use FlightMode:: constants
    std::string flightModeName;           // human-readable: "ANGLE", "ACRO [DISARMED]", etc.
    uint16_t    sensorStatus = 0;     // use SensorStatus:: constants to decode
    uint16_t    i2cErrorCount = 0;     // I2C bus errors since boot
    uint16_t    cpuLoadPercent = 0;     // average system load 0–100 %
    uint8_t     pidProfile = 0;     // active PID profile index (0-based)

    // ── Arming diagnostics — MSP_STATUS_EX (150) ─────────────────────────────
    uint32_t    armingDisableFlags = 0;   // use ArmingDisable:: constants
    std::string armingDisableStr;         // comma-separated reason list, or "" if clear

    // ── Motor outputs — MSP_MOTOR (104) ──────────────────────────────────────
    // Values in microseconds: 1000 = min throttle, 2000 = max throttle.
    // On a quad (F405 V3 with 4 motors) indices 0–3 are populated;
    // motorCount reflects the number of non-zero entries.
    uint16_t motorValues[MAX_MOTORS] = {};
    uint8_t  motorCount = 0;

    // ── RC channel inputs — MSP_RC (105) ─────────────────────────────────────
    // Raw receiver channel values 1000–2000 µs.
    // RadioMaster Pocket / CRSF default layout (MODE 2):
    //   [0] Roll (Aileron)    [1] Pitch (Elevator)
    //   [2] Throttle          [3] Yaw   (Rudder)
    //   [4] ARM switch        [5] Flight mode
    //   [6..] AUX3, AUX4, ...
    uint16_t rcChannels[MAX_RC_CH] = {};
    uint8_t  rcChannelCount = 0;

    // ── GPS — NEO-M10 via MSP_RAW_GPS (106) + MSP_COMP_GPS (107) ─────────────
    GPSReading gps;

    // ── Satellite list — MSP_GPS_SV_INFO (cmd 164), every ~1 s ───────────────
    //
    // sv_source: "MSP" in normal operation (gps_auto_config=ON).
    //            "UBX" only if passthrough is active (gps_auto_config=OFF).
    //            ""    not yet populated.
    // Fields elev / azim are always 0 in MSP mode (not in cmd 164 payload).
    std::vector<SVInfoEntry> svList;
    bool                     svInfoValid = false;
    std::string              svSource;

    // ── GPS nav engine status — MSP_NAV_STATUS (121) ──────────────────────────
    NavStatus navStatus;

    // ── Diagnostics ───────────────────────────────────────────────────────────
    double   lastRttMs = 0.0;
    double   fcCycleMs = 0.0;    // from MSP_STATUS (101)
    bool     linkHealthy = false;
    uint32_t packetCount = 0;

    // ── Telemetry source ──────────────────────────────────────────────────────
    // "USB"  — DroneLink, MSP over the USB-C cable (full data set)
    // "ELRS" — CrsfLink, CRSF telemetry mirrored by the RadioMaster Pocket.
    //          Raw IMU, raw mag, motors, RC channels, satellite list, HDOP,
    //          arming-disable flags and CPU load are NOT available on this path.
    std::string    linkSource = "USB";
    RadioLinkStats radio;
};

// =============================================================================
// DroneLink
// =============================================================================
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
    virtual DroneState getLatestState();

    // ── Calibration triggers ──────────────────────────────────────────────────
    void startMagCalibration();
    void startAccCalibration();

    // ── GPS UART index for MSP_SET_PASSTHROUGH ────────────────────────────────
    // Only used by applyGPSConfig() (UBX config writes).
    // NOT used for satellite polling (MSP cmd 164 needs no UART index).
    //   UART1 → 0  ← GPS confirmed on UART1 for this project
    //   UART2 → 1
    //   UART3 → 2
    // Default = 0.  Call before connect().
    void setGpsUartIndex(uint8_t idx) { gpsUartIndex_ = idx; }

    // ── Poll rate tuning ──────────────────────────────────────────────────────
    // If the loop overruns at 100 Hz with 13 MSP polls per tick,
    // call setPollIntervalMs(20) for a comfortable 50 Hz rate.
    void setPollIntervalMs(int ms) { pollIntervalMs.store(ms); }

    // ── Fault injection (test use only) ──────────────────────────────────────
    // DEF-005 rev 2: when active, sendMSP() returns {} immediately on every
    // call, deterministically simulating a dead link without relying on serial
    // timing side-effects.  Used by SAT-IMU-005 Scenario B.
    // Always restore with setFailInjection(false) in test teardown.
    // Must NOT be called from production flight code.
    void setFailInjection(bool active) { failInjectionActive.store(active); }

    // ── GPS configuration via UBX passthrough ─────────────────────────────────
    GPSConfigResult applyGPSConfig(const GPSConfig& cfg);

    // Moved to protected so the tests can access it without making it public in the header.
protected:
    bool parseIMU(const std::vector<uint8_t>& buf, DroneState& s);

private:
    HANDLE            hSerial;
    std::atomic<bool> connected;
    std::thread       workerThread;
    std::atomic<bool> keepRunning;
    std::atomic<int>  pollIntervalMs;

    // ── Fault injection flag (DEF-005 rev 2) ─────────────────────────────────
    // Checked at the top of sendMSP(). Atomic so Python thread and worker
    // thread can access it without a lock.
    std::atomic<bool> failInjectionActive{ false };

    std::string portName_;          // saved for reopen after passthrough cycle

    // GPS UART index — only used by applyGPSConfig() passthrough sessions.
    uint8_t gpsUartIndex_ = 0;

    // Calibration request flags (set from Python thread, consumed by worker)
    std::atomic<bool> magCalRequested;
    std::atomic<bool> accCalRequested;

    // ── Tick counters for throttled polls ─────────────────────────────────────
    // svPollTickCounter_   : satellite list, fires every SV_POLL_TICKS   (~1 s)
    // slowPollTickCounter_ : battery state,  fires every SLOW_POLL_TICKS (~500 ms)
    int svPollTickCounter_ = 0;
    int slowPollTickCounter_ = 0;

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
    bool pollSatellitesMSP(DroneState& pending);

    // ── UBX passthrough cycle (kept for applyGPSConfig only) ─────────────────
    void pollSatellites(DroneState& pending);
    std::vector<uint8_t> readUbxResponse(HANDLE h, int timeoutMs = 300);

    // ── Parsers — one per MSP response type ───────────────────────────────────
    bool parseStatus(const std::vector<uint8_t>& buf, DroneState& s);
    //bool parseIMU(const std::vector<uint8_t>& buf, DroneState& s); // Uncomment when not doing testing
    bool parseAttitude(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseAnalog(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseDebug(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseBaro(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseGPSRaw(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseGPSComp(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseNavStatus(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseMspSvInfo(const std::vector<uint8_t>& buf, DroneState& s);

    // ── New parsers ───────────────────────────────────────────────────────────
    bool parseStatusEx(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseBatteryState(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseMotors(const std::vector<uint8_t>& buf, DroneState& s);
    bool parseRCChannels(const std::vector<uint8_t>& buf, DroneState& s);

    // ── Decode helpers (static — no hardware access) ──────────────────────────
    // decodeFlightMode: translates flightModeFlags bitmask → display string.
    //   Returns e.g. "ANGLE", "ACRO+MAG", "GPS RESCUE", "ANGLE [DISARMED]".
    static std::string decodeFlightMode(uint32_t flags);

    // decodeArmingDisable: translates armingDisableFlags → comma-separated string.
    //   Returns "" when flags == 0 (ready to arm).
    static std::string decodeArmingDisable(uint32_t flags);

    void commitState(const DroneState& s);
};

// =============================================================================
// AutoDetectF405() — scans COM1–COM29, returns first port that opens
// =============================================================================
std::string AutoDetectF405();