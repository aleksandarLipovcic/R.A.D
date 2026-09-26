#pragma once
// =============================================================================
// CrsfLink — wireless telemetry over ExpressLRS via the RadioMaster Pocket
//
// Sibling of DroneLink (USB/MSP) with the same shape: own thread, own
// lifecycle, thread-safe DroneState snapshot. The cockpit picks one of the
// two as the active source (see tools/telemetry_source.py).
//
//   Betaflight → RP4TD ══ELRS 2.4 GHz══► Pocket ──USB-VCP "Telem Mirror"──► CrsfLink
//
// ── Safety behaviour (same idea as VideoLink's auto-connect) ────────────────
//   • Auto-detect: finds the Pocket's COM port by USB name and/or by the CRSF
//     stream on it. Never keeps a silent unknown port (that could be the FC).
//   • Auto-reconnect: USB unplug is detected within ~1 s (ReadFile error or
//     the COM device disappearing); the worker keeps scanning and re-attaches
//     when the Pocket comes back. reconnectCount counts these events.
//   • Link status (RadioLinkStatus) distinguishes "no radio", "radio but no
//     drone telemetry", "OK", "degraded" and "lost", with per-instrument data
//     ages so the UI can grey out a frozen horizon instead of showing it as live.
//   • On TELEMETRY_LOST the last known values are KEPT (last GPS position is
//     the most important thing to have if the drone goes down).
//
// Listen-only: nothing is ever written to the Pocket's port.
// =============================================================================
#include "DroneLink.h"          // DroneState, RadioLinkStats (includes windows.h)
#include "CrsfProtocol.h"
#include "CrsfStateMapper.h"
#include "SerialPortScan.h"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class CrsfLink {
public:
    CrsfLink();
    ~CrsfLink();
    CrsfLink(const CrsfLink&) = delete;
    CrsfLink& operator=(const CrsfLink&) = delete;

    // ── Lifecycle ────────────────────────────────────────────────────────────
    // Non-blocking. Starts the worker: scans for the Pocket, connects, and
    // re-connects forever after unplug/replug. Preferred entry point.
    void startAuto();

    // Blocking (up to timeoutMs): scan until the Pocket is found. On success
    // the worker is started in auto mode (so it still auto-reconnects).
    // Same contract as VideoLink::connectAuto — call from a background thread.
    bool connectAuto(int timeoutMs = 3000);

    // Blocking: open a specific port. The worker re-opens THIS port name after
    // an unplug (no scanning of other ports).
    bool connect(const std::string& portName);

    void disconnect();
    bool isConnected() const { return portOpen_.load(); }   // Pocket's COM port is open
    bool isRunning() const { return running_.load(); }

    // ── Data ─────────────────────────────────────────────────────────────────
    DroneState getLatestState();
    RadioLinkStatus getStatus();
    std::string getPortName();
    // Human-readable log of the last port scan — paste this when debugging detection.
    std::string getLastScanReport();

    // ── Configuration (safe to call any time) ────────────────────────────────
    void setExcludedPorts(const std::vector<std::string>& ports);   // e.g. DroneLink's port
    void setPortNameHints(const std::vector<std::string>& hints);   // default: EdgeTX/RadioMaster/Pocket
    void setScanIntervalMs(int ms) { scanIntervalMs_.store(ms > 100 ? ms : 100); }
    void setProbeMs(int ms) { probeMs_.store(ms > 100 ? ms : 100); }
    void setAssertDtr(bool on) { assertDtr_.store(on); }            // default true (like pyserial)

    void setLostTimeoutMs(int ms);
    void setDegradedLq(int lqPercent);
    void setInstrumentStaleMs(int ms);

    void setYawSigned(bool s);
    void setCellCount(int cells);                 // 0 = auto
    void setCellVoltageThresholds(float warningV, float criticalV);
    void setBatteryCapacityMah(int mah);
    void setMinSatsForFix(int sats);
    void setGpsAltitudeFallback(bool on);

    static std::vector<SerialPortInfo> enumeratePorts() { return EnumerateSerialPorts(); }

private:
    void startWorker();
    void workerLoop();

    // Port handling (worker thread only, except connect*/disconnect before/after the thread)
    HANDLE openPort(const std::string& name);
    void   closePort();
    bool   scanForRadio(std::string& foundPort, bool& nameMatched);   // opens h_ on success
    int    countFrames(HANDLE h, int listenMs);

    static int64_t nowMs();

    // Worker
    std::thread       worker_;
    std::atomic<bool> running_{ false };
    std::atomic<bool> portOpen_{ false };
    HANDLE            h_ = INVALID_HANDLE_VALUE;
    bool              nameMatched_ = false;
    bool              everConnected_ = false;
    int64_t           lastByteMs_ = 0;
    int64_t           lastPresenceCheckMs_ = 0;

    // Config
    std::mutex               cfgMutex_;
    std::string              fixedPort_;          // empty → auto scan
    std::vector<std::string> excluded_;
    std::vector<std::string> hints_;
    std::string              lastScanReport_;
    std::atomic<int>         scanIntervalMs_{ 1000 };
    std::atomic<int>         probeMs_{ 500 };
    std::atomic<bool>        assertDtr_{ true };

    // State
    std::mutex                         stateMutex_;
    DroneState                         state_;
    CrsfStateMapper                    mapper_;
    CrsfStateMapper::StatusThresholds  thresholds_;
    Crsf::StreamParser                 parser_;
    uint32_t                           reconnectCount_ = 0;
    std::string                        portName_;
};
