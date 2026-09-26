#include "CrsfLink.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <sstream>

namespace {
    std::string lower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }
    constexpr int READ_CHUNK = 512;
    constexpr int REFRESH_PERIOD_MS = 100;       // status/age/rate refresh, 10 Hz
    constexpr int PRESENCE_CHECK_MS = 1000;      // QueryDosDevice unplug check
    constexpr int WRONG_PORT_SILENCE_MS = 5000;  // unnamed port silent this long → rescan
}

int64_t CrsfLink::nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

CrsfLink::CrsfLink() : hints_(DefaultRadioPortHints()) {
    state_.linkSource = "ELRS";
}

CrsfLink::~CrsfLink() { disconnect(); }

// =============================================================================
// Port helpers
// =============================================================================
HANDLE CrsfLink::openPort(const std::string& name) {
    const std::string path = "\\\\.\\" + name;
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return h;

    // USB CDC ignores the baud rate, but set a sane line config anyway.
    DCB dcb = {}; dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = 115200; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
    dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
    // Some CDC firmwares only transmit once the host asserts DTR ("port opened").
    dcb.fDtrControl = assertDtr_.load() ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    SetCommState(h, &dcb);

    // ReadFile returns as soon as any byte is available, or after 20 ms.
    COMMTIMEOUTS to = {};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 20;
    to.WriteTotalTimeoutConstant = 50;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

void CrsfLink::closePort() {
    if (h_ != INVALID_HANDLE_VALUE) {
        CloseHandle(h_);
        h_ = INVALID_HANDLE_VALUE;
    }
    portOpen_.store(false);
}

int CrsfLink::countFrames(HANDLE h, int listenMs) {
    Crsf::StreamParser p;
    std::vector<Crsf::Frame> frames;
    const int64_t end = nowMs() + listenMs;
    uint8_t buf[READ_CHUNK];
    while (nowMs() < end) {
        DWORD rd = 0;
        if (!ReadFile(h, buf, sizeof(buf), &rd, nullptr)) return -1;
        if (rd) p.feed(buf, rd, frames);
    }
    return static_cast<int>(frames.size());
}

// =============================================================================
// scanForRadio — one pass over all COM ports.
//   1. Ports whose USB name matches a hint (EdgeTX/RadioMaster/Pocket) are
//      accepted as soon as they open, even if silent (drone may be off).
//   2. Other ports are accepted only if they carry a valid CRSF stream.
//   Excluded ports (DroneLink's), Bluetooth ports and ports naming
//   themselves Betaflight/INAV are never touched.
// =============================================================================
bool CrsfLink::scanForRadio(std::string& found, bool& nameMatched) {
    std::vector<std::string> excluded, hints;
    {
        std::lock_guard<std::mutex> lk(cfgMutex_);
        excluded = excluded_;
        hints = hints_;
    }
    auto isExcluded = [&](const std::string& port) {
        for (const auto& e : excluded) if (lower(e) == lower(port)) return true;
        return false;
        };

    auto ports = EnumerateSerialPorts();
    std::stable_partition(ports.begin(), ports.end(),
        [&](const SerialPortInfo& p) { return PortMatchesHints(p, hints); });

    std::ostringstream rep;
    rep << "scan: " << ports.size() << " COM port(s)\n";
    bool ok = false;

    for (const auto& p : ports) {
        const std::string names = lower(p.friendlyName + " " + p.busDescription);
        rep << "  " << p.port << " [" << p.busDescription << " | " << p.friendlyName << "] ";
        if (isExcluded(p.port)) { rep << "skip: excluded\n"; continue; }
        if (names.find("bluetooth") != std::string::npos) { rep << "skip: bluetooth\n"; continue; }
        if (names.find("betaflight") != std::string::npos ||
            names.find("inav") != std::string::npos) {
            rep << "skip: flight controller\n"; continue;
        }

        HANDLE h = openPort(p.port);
        if (h == INVALID_HANDLE_VALUE) {
            rep << "cannot open (busy?) err=" << GetLastError() << "\n";
            continue;
        }
        const bool hinted = PortMatchesHints(p, hints);
        const int n = countFrames(h, probeMs_.load());
        rep << (hinted ? "name match, " : "") << n << " CRSF frames in " << probeMs_.load() << " ms";
        if (hinted || n >= 2) {
            rep << " -> SELECTED\n";
            h_ = h;
            found = p.port;
            nameMatched = hinted;
            ok = true;
            break;
        }
        rep << "\n";
        CloseHandle(h);
    }
    if (!ok) rep << "  no radio found\n";

    std::lock_guard<std::mutex> lk(cfgMutex_);
    lastScanReport_ = rep.str();
    return ok;
}

// =============================================================================
// Lifecycle
// =============================================================================
void CrsfLink::startWorker() {
    running_.store(true);
    worker_ = std::thread(&CrsfLink::workerLoop, this);
}

void CrsfLink::startAuto() {
    disconnect();
    { std::lock_guard<std::mutex> lk(cfgMutex_); fixedPort_.clear(); }
    startWorker();
}

bool CrsfLink::connectAuto(int timeoutMs) {
    disconnect();
    { std::lock_guard<std::mutex> lk(cfgMutex_); fixedPort_.clear(); }

    const int64_t end = nowMs() + timeoutMs;
    std::string port; bool hinted = false;
    do {
        if (scanForRadio(port, hinted)) {
            nameMatched_ = hinted;
            everConnected_ = true;
            lastByteMs_ = nowMs();
            {
                std::lock_guard<std::mutex> lk(stateMutex_);
                portName_ = port;
            }
            portOpen_.store(true);
            startWorker();
            return true;
        }
        Sleep(200);
    } while (nowMs() < end);
    return false;
}

bool CrsfLink::connect(const std::string& portName) {
    disconnect();
    { std::lock_guard<std::mutex> lk(cfgMutex_); fixedPort_ = portName; }
    h_ = openPort(portName);
    const bool ok = h_ != INVALID_HANDLE_VALUE;
    if (ok) {
        nameMatched_ = true;            // explicitly chosen: keep it even if silent
        everConnected_ = true;
        lastByteMs_ = nowMs();
        { std::lock_guard<std::mutex> lk(stateMutex_); portName_ = portName; }
        portOpen_.store(true);
    }
    startWorker();                      // even on failure: keeps retrying this port
    return ok;
}

void CrsfLink::disconnect() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    closePort();
    std::lock_guard<std::mutex> lk(stateMutex_);
    portName_.clear();
    mapper_.refresh(state_, false, nowMs(), thresholds_);
    state_.radio.portName.clear();
}

// =============================================================================
// workerLoop
// =============================================================================
void CrsfLink::workerLoop() {
    int64_t nextScanMs = 0, lastRefreshMs = 0;
    uint8_t buf[READ_CHUNK];
    std::vector<Crsf::Frame> frames;

    while (running_.load()) {
        const int64_t now = nowMs();

        // ── Not connected: scan / re-open ───────────────────────────────────
        if (h_ == INVALID_HANDLE_VALUE) {
            if (now >= nextScanMs) {
                std::string fixed;
                { std::lock_guard<std::mutex> lk(cfgMutex_); fixed = fixedPort_; }

                std::string port; bool hinted = false; bool ok = false;
                if (fixed.empty()) {
                    ok = scanForRadio(port, hinted);
                }
                else {
                    h_ = openPort(fixed);
                    ok = h_ != INVALID_HANDLE_VALUE; port = fixed; hinted = true;
                }
                if (ok) {
                    nameMatched_ = hinted;
                    if (everConnected_) ++reconnectCount_;
                    everConnected_ = true;
                    lastByteMs_ = nowMs();
                    lastPresenceCheckMs_ = lastByteMs_;
                    parser_.reset();
                    { std::lock_guard<std::mutex> lk(stateMutex_); portName_ = port; }
                    portOpen_.store(true);
                }
                else {
                    nextScanMs = now + scanIntervalMs_.load();
                }
            }
            if (h_ == INVALID_HANDLE_VALUE) {
                if (now - lastRefreshMs >= REFRESH_PERIOD_MS) {
                    std::lock_guard<std::mutex> lk(stateMutex_);
                    mapper_.refresh(state_, false, now, thresholds_);
                    state_.radio.portName.clear();
                    state_.radio.reconnectCount = reconnectCount_;
                    lastRefreshMs = now;
                }
                Sleep(50);
                continue;
            }
        }

        // ── Connected: read ─────────────────────────────────────────────────
        DWORD errs = 0; COMSTAT cs = {};
        DWORD rd = 0;
        bool dead = !ClearCommError(h_, &errs, &cs) ||
            !ReadFile(h_, buf, sizeof(buf), &rd, nullptr);

        const int64_t t = nowMs();
        if (!dead && t - lastPresenceCheckMs_ >= PRESENCE_CHECK_MS) {
            lastPresenceCheckMs_ = t;
            std::string pn;
            { std::lock_guard<std::mutex> lk(stateMutex_); pn = portName_; }
            if (!SerialPortExists(pn)) dead = true;          // unplugged
        }
        if (dead) {
            closePort();
            nextScanMs = t + 300;                            // give Windows a moment
            continue;
        }

        if (rd) {
            lastByteMs_ = t;
            frames.clear();
            parser_.feed(buf, rd, frames);
            std::lock_guard<std::mutex> lk(stateMutex_);
            for (const auto& f : frames) mapper_.apply(f, state_, t);
            state_.radio.framesTotal = parser_.framesOk();
            state_.radio.crcErrors = parser_.crcErrors();
            state_.packetCount = parser_.framesOk();
        }

        // Auto mode safety: an unnamed port that went silent may not be the
        // radio at all — release it (it could be the FC) and rescan.
        bool fixedMode;
        { std::lock_guard<std::mutex> lk(cfgMutex_); fixedMode = !fixedPort_.empty(); }
        if (!fixedMode && !nameMatched_ && t - lastByteMs_ > WRONG_PORT_SILENCE_MS) {
            closePort();
            nextScanMs = t;
            continue;
        }

        if (t - lastRefreshMs >= REFRESH_PERIOD_MS) {
            std::lock_guard<std::mutex> lk(stateMutex_);
            mapper_.refresh(state_, true, t, thresholds_);
            state_.radio.portName = portName_;
            state_.radio.reconnectCount = reconnectCount_;
            lastRefreshMs = t;
        }
    }
    closePort();
}

// =============================================================================
// Accessors
// =============================================================================
DroneState CrsfLink::getLatestState() {
    std::lock_guard<std::mutex> lk(stateMutex_);
    // Ages must be current at the moment Python reads them, not up to 100 ms old.
    mapper_.refresh(state_, portOpen_.load(), nowMs(), thresholds_);
    state_.radio.portName = portOpen_.load() ? portName_ : std::string();
    return state_;
}

RadioLinkStatus CrsfLink::getStatus() {
    std::lock_guard<std::mutex> lk(stateMutex_);
    mapper_.refresh(state_, portOpen_.load(), nowMs(), thresholds_);
    return state_.radio.status;
}

std::string CrsfLink::getPortName() {
    std::lock_guard<std::mutex> lk(stateMutex_);
    return portOpen_.load() ? portName_ : std::string();
}

std::string CrsfLink::getLastScanReport() {
    std::lock_guard<std::mutex> lk(cfgMutex_);
    return lastScanReport_;
}

void CrsfLink::setExcludedPorts(const std::vector<std::string>& p) {
    std::lock_guard<std::mutex> lk(cfgMutex_); excluded_ = p;
}
void CrsfLink::setPortNameHints(const std::vector<std::string>& h) {
    std::lock_guard<std::mutex> lk(cfgMutex_); hints_ = h;
}

void CrsfLink::setLostTimeoutMs(int ms) { std::lock_guard<std::mutex> lk(stateMutex_); thresholds_.lostTimeoutMs = ms; }
void CrsfLink::setDegradedLq(int lq) { std::lock_guard<std::mutex> lk(stateMutex_); thresholds_.degradedLq = lq; }
void CrsfLink::setInstrumentStaleMs(int ms) { std::lock_guard<std::mutex> lk(stateMutex_); thresholds_.instrumentStaleMs = ms; }

#define CRSF_CFG(stmt) do { std::lock_guard<std::mutex> lk(stateMutex_); \
    auto c = mapper_.config(); stmt; mapper_.setConfig(c); } while (0)

void CrsfLink::setYawSigned(bool s) { CRSF_CFG(c.yawSigned = s); }
void CrsfLink::setCellCount(int n) { CRSF_CFG(c.cellCount = static_cast<uint8_t>((std::max)(0, n))); }
void CrsfLink::setCellVoltageThresholds(float w, float cr) { CRSF_CFG(c.warningCellV = w; c.criticalCellV = cr); }
void CrsfLink::setBatteryCapacityMah(int mah) { CRSF_CFG(c.capacityMah = static_cast<uint16_t>((std::max)(0, mah))); }
void CrsfLink::setMinSatsForFix(int s) { CRSF_CFG(c.minSatsForFix = static_cast<uint8_t>((std::max)(0, s))); }
void CrsfLink::setGpsAltitudeFallback(bool on) { CRSF_CFG(c.gpsAltitudeFallback = on); }
#undef CRSF_CFG