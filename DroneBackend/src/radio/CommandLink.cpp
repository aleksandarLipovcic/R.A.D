#include "CommandLink.h"
#include <algorithm>
#include <chrono>
#include <cmath>

int64_t CommandLink::nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

float CommandLink::approach(float cur, float target, float maxStep) {
    if (target > cur + maxStep) return cur + maxStep;
    if (target < cur - maxStep) return cur - maxStep;
    return target;
}

// ── Python side ─────────────────────────────────────────────────────────────
void CommandLink::setLimits(const Limits& l) {
    std::lock_guard<std::mutex> lk(m_);
    lim_ = l;
    lim_.maxStickPct = (std::min)((std::max)(lim_.maxStickPct, 0.0f), 100.0f);
    lim_.maxYawPct = (std::min)((std::max)(lim_.maxYawPct, 0.0f), 100.0f);
    lim_.throttleMinUs = (std::max)(lim_.throttleMinUs, 1000.0f);
    lim_.throttleMaxUs = (std::min)((std::max)(lim_.throttleMaxUs, lim_.throttleMinUs), 2000.0f);
    lim_.leaseMs = (std::max)(lim_.leaseMs, 50);
    lim_.sendPeriodMs = (std::min)((std::max)(lim_.sendPeriodMs, 10), 200);
}

CommandLink::Limits CommandLink::getLimits() const {
    std::lock_guard<std::mutex> lk(m_);
    return lim_;
}

void CommandLink::setEnabled(bool on) {
    std::lock_guard<std::mutex> lk(m_);
    enabled_ = on;
    if (!on) leaseT_ = -1;
}

bool CommandLink::isEnabled() const {
    std::lock_guard<std::mutex> lk(m_);
    return enabled_;
}

void CommandLink::setCommand(const Uplink::CommandValues& c) { setCommandAt(c, nowMs()); }

void CommandLink::setCommandAt(const Uplink::CommandValues& c, int64_t now) {
    std::lock_guard<std::mutex> lk(m_);
    target_ = c;
    leaseT_ = now;
}

namespace {
    float clampPct(float v) { return (std::min)((std::max)(v, -100.0f), 100.0f); }
}

void CommandLink::setGimbalTiltPct(float pct) {
    std::lock_guard<std::mutex> lk(m_);
    target_.gimbalTiltPct = clampPct(pct);
}

void CommandLink::setGimbalPanPct(float pct) {
    std::lock_guard<std::mutex> lk(m_);
    target_.gimbalPanPct = clampPct(pct);
}

void CommandLink::setGimbalPct(float panPct, float tiltPct) {
    std::lock_guard<std::mutex> lk(m_);
    target_.gimbalPanPct = clampPct(panPct);
    target_.gimbalTiltPct = clampPct(tiltPct);
}

void CommandLink::neutral() {
    std::lock_guard<std::mutex> lk(m_);
    const float tilt = target_.gimbalTiltPct, pan = target_.gimbalPanPct;
    target_ = Uplink::CommandValues{};
    target_.gimbalTiltPct = tilt;
    target_.gimbalPanPct = pan;
    leaseT_ = -1;
}

// ── Worker side ─────────────────────────────────────────────────────────────
bool CommandLink::bridgeAliveLocked(int64_t now) const {
    return statusT_ >= 0 && now - statusT_ <= lim_.bridgeTimeoutMs;
}

bool CommandLink::frameDue(int64_t now) const {
    std::lock_guard<std::mutex> lk(m_);
    if (!bridgeAliveLocked(now)) return false;
    return lastFrameT_ < 0 || now - lastFrameT_ >= lim_.sendPeriodMs;
}

std::vector<uint8_t> CommandLink::nextFrame(int64_t now) {
    std::lock_guard<std::mutex> lk(m_);
    if (!bridgeAliveLocked(now)) return {};

    const float dt = lastFrameT_ < 0 ? 0.0f
        : (std::min)(static_cast<float>(now - lastFrameT_), 100.0f) / 1000.0f;
    lastFrameT_ = now;

    const bool lease = enabled_ && leaseT_ >= 0 && now - leaseT_ <= lim_.leaseMs;
    const bool sticksOn = lease && target_.sticksEnabled;
    const bool thrOn = lease && target_.throttleEnabled;

    Uplink::CommandValues out = sent_;
    out.gimbalTiltPct = clampPct(target_.gimbalTiltPct);
    out.gimbalPanPct = clampPct(target_.gimbalPanPct);

    // Sticks: slew toward the clamped target; drop to zero at once when off.
    if (sticksOn) {
        const float step = lim_.stickSlewPctPerS * dt;
        auto lim = [](float v, float m) { return (std::min)((std::max)(v, -m), m); };
        out.rollPct  = approach(sent_.rollPct,  lim(target_.rollPct,  lim_.maxStickPct), step);
        out.pitchPct = approach(sent_.pitchPct, lim(target_.pitchPct, lim_.maxStickPct), step);
        out.yawPct   = approach(sent_.yawPct,   lim(target_.yawPct,   lim_.maxYawPct),   step);
    } else {
        out.rollPct = out.pitchPct = out.yawPct = 0.0f;
    }
    out.sticksEnabled = sticksOn;

    // Throttle: no slew on the engage edge (Python hands over the throttle the
    // radio is already sending), slew-limited afterwards.
    const float thrTarget = (std::min)((std::max)(target_.throttleUs, lim_.throttleMinUs),
                                       lim_.throttleMaxUs);
    if (thrOn) {
        out.throttleUs = prevThrottleOn_
            ? approach(sent_.throttleUs, thrTarget, lim_.throttleSlewUsPerS * dt)
            : thrTarget;
    } else {
        out.throttleUs = thrTarget;
    }
    out.throttleEnabled = thrOn;
    prevThrottleOn_ = thrOn;
    sent_ = out;

    const uint32_t seq = seq_++;
    Sent& s = sentRing_[seq % sentRing_.size()];
    s.alive = Uplink::aliveValue(seq);
    s.t = now;
    return Uplink::buildCommandFrame(out, seq);
}

void CommandLink::onWriteResult(bool ok) {
    std::lock_guard<std::mutex> lk(m_);
    if (ok) ++framesSent_; else ++writeErrors_;
}

void CommandLink::onFrame(const Crsf::Frame& f, int64_t now) {
    if (f.type == Uplink::TYPE_RAD_STATUS) {
        Uplink::BridgeStatus b;
        if (!Uplink::decodeBridgeStatus(f, b) || b.version < Uplink::RAD_STATUS_MIN_VERSION
            || b.version > Uplink::RAD_STATUS_VERSION) return;
        std::lock_guard<std::mutex> lk(m_);
        bridge_ = b;
        statusT_ = now;
        if (b.echoAlive >= 0 && b.echoAlive != lastEcho_) {
            lastEcho_ = b.echoAlive;
            for (const Sent& s : sentRing_) {
                if (s.t >= 0 && s.alive == b.echoAlive) {
                    rtt_ = static_cast<float>(now - s.t);
                    rttAvg_ = rttAvg_ < 0 ? rtt_ : rttAvg_ * 0.9f + rtt_ * 0.1f;
                    break;
                }
            }
        }
    }
    else if (f.type == Crsf::Type::RC_CHANNELS) {
        std::array<uint16_t, Uplink::NUM_CHANNELS> ch;
        if (!Uplink::unpackChannels(f.payload, ch)) return;
        std::lock_guard<std::mutex> lk(m_);
        for (int i = 0; i < Uplink::NUM_CHANNELS; ++i) radioUs_[i] = Uplink::crsfToUs(ch[i]);
        radioT_ = now;
    }
}

void CommandLink::onPortClosed() {
    std::lock_guard<std::mutex> lk(m_);
    statusT_ = -1;
    radioT_ = -1;
    lastFrameT_ = -1;
    prevThrottleOn_ = false;
    sent_ = Uplink::CommandValues{};
    lastEcho_ = -1;
    for (Sent& s : sentRing_) s.t = -1;
}

// ── Status ──────────────────────────────────────────────────────────────────
CommandLink::Status CommandLink::getStatus() const { return getStatusAt(nowMs()); }

CommandLink::Status CommandLink::getStatusAt(int64_t now) const {
    std::lock_guard<std::mutex> lk(m_);
    Status s;
    s.bridgeDetected = bridgeAliveLocked(now);
    s.bridgeAgeMs = statusT_ < 0 ? -1 : static_cast<int>(now - statusT_);
    s.enabled = enabled_;
    s.leaseValid = enabled_ && leaseT_ >= 0 && now - leaseT_ <= lim_.leaseMs;
    s.bridge = bridge_;
    s.lastSent = sent_;
    s.radioChannelsUs = radioUs_;
    s.radioChannelsValid = radioT_ >= 0 && now - radioT_ <= lim_.bridgeTimeoutMs;
    s.radioChannelsAgeMs = radioT_ < 0 ? -1 : static_cast<int>(now - radioT_);
    s.rttMs = rtt_;
    s.rttAvgMs = rttAvg_;
    s.framesSent = framesSent_;
    s.writeErrors = writeErrors_;

    if (!s.bridgeDetected)                                   s.state = "NO_BRIDGE";
    else if (!bridge_.fresh || !bridge_.gateOn)              s.state = "STANDBY";
    else if (bridge_.sticksActive || bridge_.throttleActive) s.state = "ACTIVE";
    else                                                     s.state = "GATE_ON";
    return s;
}
