#pragma once
// =============================================================================
// CommandLink — laptop → drone commands through the RadioMaster Pocket
//
// Holds "what the laptop wants now" and turns it into CRSF command frames for
// the Lua bridge (pocket/SCRIPTS/MIXES/radcmd.lua). It owns NO port: CrsfLink
// already owns the Pocket's COM port, so CrsfLink's worker thread asks this
// object for the next frame, writes it, and hands every received frame back
// via onFrame(). One port owner, two directions (see command-uplink.md).
//
//   Python (UplinkController, 25 Hz) ─setCommand()─► CommandLink
//   CrsfLink worker ─nextFrame()─► WriteFile ─► Pocket ─► mixer ─► ELRS ─► BF
//   CrsfLink worker ◄─ RAD_STATUS / RC_CHANNELS frames ─ Pocket  ─onFrame()─►
//
// ── Safety layers on the laptop side (the radio has its own, see radcmd.lua) ─
//   1. Nothing is ever written until a RAD_STATUS frame proved the port is
//      the Lua bridge (never into Telem Mirror, a CLI or a flight controller).
//   2. Master enable (setEnabled). Off → sticks/throttle disabled at once.
//   3. Lease: a stick/throttle command expires leaseMs after the last
//      setCommand(). A hung Python thread can't leave an old command active.
//   4. Limits: |roll|,|pitch| ≤ maxStickPct, |yaw| ≤ maxYawPct, throttle
//      inside [throttleMinUs, throttleMaxUs], all slew-rate limited.
//   5. Frames go out at a fixed rate with an alive counter; if this process
//      dies, the radio goes neutral within 500 ms on its own.
// No Win32 here: tested on any machine (tests/test_uplink.cpp).
// =============================================================================
#include "CrsfProtocol.h"
#include "UplinkProtocol.h"
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class CommandLink {
public:
    struct Limits {
        float maxStickPct = 25.0f;          // roll / pitch offset
        float maxYawPct = 30.0f;
        float throttleMinUs = 1150.0f;
        float throttleMaxUs = 1750.0f;
        float stickSlewPctPerS = 150.0f;    // 0 → 25 % in ~170 ms
        float throttleSlewUsPerS = 250.0f;
        int   leaseMs = 300;                // setCommand() must come this often
        int   sendPeriodMs = 30;            // ~33 frames/s
        int   bridgeTimeoutMs = 1500;       // no RAD_STATUS this long → stop
    };

    struct Status {
        std::string state;                  // NO_BRIDGE / STANDBY / GATE_ON / ACTIVE
        bool  bridgeDetected = false;
        int   bridgeAgeMs = -1;             // age of the newest RAD_STATUS
        bool  enabled = false;              // master enable (laptop side)
        bool  leaseValid = false;
        Uplink::BridgeStatus bridge;        // what the radio reports it is doing
        Uplink::CommandValues lastSent;     // after limits and slew
        // Channels the Pocket is actually transmitting (RC_CHANNELS from the
        // bridge), in µs. Index 0 = CH1.
        std::array<float, Uplink::NUM_CHANNELS> radioChannelsUs{};
        bool  radioChannelsValid = false;
        int   radioChannelsAgeMs = -1;
        float rttMs = -1.0f;                // laptop → script → laptop, last
        float rttAvgMs = -1.0f;             // exponential average
        uint32_t framesSent = 0;
        uint32_t writeErrors = 0;
    };

    CommandLink() = default;

    // ── Python side ──────────────────────────────────────────────────────────
    void setLimits(const Limits& l);
    Limits getLimits() const;
    void setEnabled(bool on);
    bool isEnabled() const;
    // Full command; renews the lease. The gimbal value in it is used too.
    void setCommand(const Uplink::CommandValues& c);
    // Gimbal only (no lease: the radio holds the last gimbal values anyway,
    // and gimbal commands work without the master enable).
    void setGimbalTiltPct(float pct);
    void setGimbalPanPct(float pct);
    void setGimbalPct(float panPct, float tiltPct);
    // Sticks to zero and throttle hold off immediately (keeps gimbal).
    void neutral();
    Status getStatus() const;

    // ── CrsfLink worker side ────────────────────────────────────────────────
    bool frameDue(int64_t nowMs) const;
    std::vector<uint8_t> nextFrame(int64_t nowMs);   // empty if nothing to send
    void onWriteResult(bool ok);
    void onFrame(const Crsf::Frame& f, int64_t nowMs);
    void onPortClosed();

    // Explicit-time variants (tests / simulation)
    void setCommandAt(const Uplink::CommandValues& c, int64_t nowMs);
    Status getStatusAt(int64_t nowMs) const;

    static int64_t nowMs();

private:
    static float approach(float cur, float target, float maxStep);
    bool bridgeAliveLocked(int64_t now) const;

    mutable std::mutex m_;
    Limits lim_;
    bool enabled_ = false;
    Uplink::CommandValues target_;
    int64_t leaseT_ = -1;                   // time of the last setCommand()
    Uplink::CommandValues sent_;            // slewed output of the last frame
    bool prevThrottleOn_ = false;
    int64_t lastFrameT_ = -1;
    uint32_t seq_ = 0;

    // bridge
    int64_t statusT_ = -1;
    Uplink::BridgeStatus bridge_;
    std::array<float, Uplink::NUM_CHANNELS> radioUs_{};
    int64_t radioT_ = -1;

    // round-trip measurement: alive value → send time
    struct Sent { uint16_t alive = 0; int64_t t = -1; };
    std::array<Sent, 64> sentRing_{};
    int lastEcho_ = -1;
    float rtt_ = -1.0f, rttAvg_ = -1.0f;
    uint32_t framesSent_ = 0, writeErrors_ = 0;
};
