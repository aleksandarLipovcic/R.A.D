// =============================================================================
// test_uplink.cpp — unit tests for UplinkProtocol + CommandLink
//
// Build (any OS, no Win32), from DroneBackend/tests:
//   g++ -std=c++17 -Wall -Wextra -I../src/radio test_uplink.cpp
//       ../src/radio/UplinkProtocol.cpp ../src/radio/CommandLink.cpp
//       ../src/radio/CrsfProtocol.cpp -o test_uplink -pthread
//   Race check (Linux/WSL): add -fsanitize=thread -g
//   (MSVC: cl /std:c++17 /EHsc /I..\src\radio test_uplink.cpp
//          ..\src\radio\UplinkProtocol.cpp ..\src\radio\CommandLink.cpp
//          ..\src\radio\CrsfProtocol.cpp)
// The same binary is used by test_lua_bridge.py as a frame encoder/decoder:
//   test_uplink encode <out.bin>   command frames for the Lua cross-check
//   test_uplink decode <in.bin>    parse frames written by radcmd.lua
// =============================================================================
#include "CommandLink.h"
#include "UplinkProtocol.h"
#include "CrsfProtocol.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <random>
#include <atomic>
#include <chrono>
#include <thread>

static int g_fail = 0, g_pass = 0;
#define CHECK(cond) do { if (cond) ++g_pass; else { ++g_fail; \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

static std::vector<Crsf::Frame> parseAll(const std::vector<uint8_t>& bytes) {
    Crsf::StreamParser p;
    std::vector<Crsf::Frame> out;
    p.feed(bytes.data(), bytes.size(), out);
    return out;
}

static Crsf::Frame statusFrame(uint8_t flags, int echo, uint8_t version = Uplink::RAD_STATUS_VERSION) {
    std::vector<uint8_t> p(version >= 2 ? 24 : 22, 0);
    p[0] = version;
    p[1] = flags;
    const uint16_t e = echo < 0 ? 0xFFFF : static_cast<uint16_t>(echo);
    p[2] = e >> 8; p[3] = e & 0xFF;
    Crsf::Frame f;
    f.addr = 0xC8; f.type = Uplink::TYPE_RAD_STATUS; f.payload = p;
    return f;
}

static Uplink::CommandValues sentOf(const std::vector<uint8_t>& frame) {
    auto fr = parseAll(frame);
    Uplink::CommandValues c;
    if (fr.size() != 1) return c;
    std::array<uint16_t, 16> ch;
    Uplink::unpackChannels(fr[0].payload, ch);
    c.rollPct = Uplink::crsfToPct(ch[0]);
    c.pitchPct = Uplink::crsfToPct(ch[1]);
    c.throttleUs = Uplink::crsfToUs(ch[2]);
    c.yawPct = Uplink::crsfToPct(ch[3]);
    c.gimbalTiltPct = Uplink::crsfToPct(ch[4]);
    c.gimbalPanPct = Uplink::crsfToPct(ch[8]);
    c.sticksEnabled = ch[5] > 1500;
    c.throttleEnabled = ch[6] > 1500;
    return c;
}

static void testProtocol() {
    // conversions
    CHECK(Uplink::usToCrsf(1500) == 992);
    CHECK(Uplink::usToCrsf(2012) == 1811);
    CHECK(Uplink::usToCrsf(2500) == 1811);
    CHECK(Uplink::usToCrsf(500) == 172);
    CHECK(Uplink::pctToCrsf(100) == 1811);
    CHECK(Uplink::pctToCrsf(-100) == 173);
    NEAR(Uplink::crsfToUs(Uplink::usToCrsf(1350)), 1350.0f, 0.7f);
    NEAR(Uplink::crsfToPct(Uplink::pctToCrsf(-37.5f)), -37.5f, 0.15f);

    // pack / unpack round trip
    std::mt19937 rng(7);
    for (int n = 0; n < 200; ++n) {
        std::array<uint16_t, 16> a, b;
        for (auto& v : a) v = rng() & 0x7FF;
        auto bytes = Uplink::packChannels(a);
        CHECK(bytes.size() == 22);
        CHECK(Uplink::unpackChannels(bytes, b));
        CHECK(a == b);
    }

    // command frame is a valid CRSF RC_CHANNELS frame
    Uplink::CommandValues c;
    c.rollPct = 10; c.pitchPct = -20; c.yawPct = 5; c.throttleUs = 1420;
    c.sticksEnabled = true; c.throttleEnabled = false; c.gimbalTiltPct = -50;
    c.gimbalPanPct = 35;
    auto fr = parseAll(Uplink::buildCommandFrame(c, 41));
    CHECK(fr.size() == 1);
    CHECK(fr[0].type == Crsf::Type::RC_CHANNELS);
    CHECK(fr[0].payload.size() == 22);
    std::array<uint16_t, 16> ch;
    Uplink::unpackChannels(fr[0].payload, ch);
    CHECK(ch[Uplink::Ch::STICKS_EN] == 1811);
    CHECK(ch[Uplink::Ch::THROTTLE_EN] == 172);
    CHECK(ch[Uplink::Ch::ALIVE] == Uplink::aliveValue(41));
    CHECK(Uplink::aliveValue(41) != Uplink::aliveValue(42));
    NEAR(Uplink::crsfToPct(ch[Uplink::Ch::TILT]), -50.0f, 0.15f);
    NEAR(Uplink::crsfToPct(ch[Uplink::Ch::PAN]), 35.0f, 0.15f);
    for (int i = 9; i < 16; ++i) CHECK(ch[i] == 992);

    // status decode
    auto sf = statusFrame(0x1F, 777);
    sf.payload[8] = 12;                       // 120 ms
    sf.payload[9] = 0xFF; sf.payload[10] = 0x38;   // -200
    Uplink::BridgeStatus b;
    CHECK(Uplink::decodeBridgeStatus(sf, b));
    CHECK(b.fresh && b.gateOn && b.sticksActive && b.throttleActive && b.telemetryLive);
    CHECK(b.echoAlive == 777);
    CHECK(b.cmdAgeMs == 120);
    CHECK(b.throttleStick == -200);
    auto none = statusFrame(0, -1);
    CHECK(Uplink::decodeBridgeStatus(none, b) && b.echoAlive == -1 && !b.fresh);

    // v2 carries the applied pan output; v1 (22 bytes) still decodes, no pan
    auto v2 = statusFrame(0x01, 5);
    v2.payload[22] = 0xFE; v2.payload[23] = 0x0C;          // -500
    CHECK(Uplink::decodeBridgeStatus(v2, b) && b.hasPan && b.outPan == -500);
    auto v1 = statusFrame(0x01, 5, 1);
    CHECK(Uplink::decodeBridgeStatus(v1, b) && !b.hasPan && b.outPan == 0);
}

static void testCommandLink() {
    CommandLink cl;
    int64_t t = 1000;

    // 1. No bridge → never writes
    CHECK(!cl.frameDue(t));
    CHECK(cl.nextFrame(t).empty());
    CHECK(cl.getStatusAt(t).state == "NO_BRIDGE");

    // 2. Bridge seen → frames, but disabled = neutral, enables off
    cl.onFrame(statusFrame(0x01, -1), t);
    CHECK(cl.frameDue(t));
    Uplink::CommandValues want;
    want.rollPct = 50; want.pitchPct = -10; want.yawPct = 80;
    want.sticksEnabled = true; want.throttleEnabled = true; want.throttleUs = 1450;
    want.gimbalTiltPct = 30; want.gimbalPanPct = -45;
    cl.setCommandAt(want, t);
    auto s = sentOf(cl.nextFrame(t));
    CHECK(!s.sticksEnabled && !s.throttleEnabled);
    NEAR(s.rollPct, 0.0f, 0.2f);
    NEAR(s.gimbalTiltPct, 30.0f, 0.2f);          // gimbal works without enable
    NEAR(s.gimbalPanPct, -45.0f, 0.2f);
    CHECK(cl.getStatusAt(t).state == "STANDBY");

    // The bridge answers every frame, as radcmd.lua does on the radio.
    auto tick = [&](int64_t dtMs, bool renew) {
        t += dtMs;
        cl.onFrame(statusFrame(0x01, -1), t);
        if (renew) cl.setCommandAt(want, t);
        return sentOf(cl.nextFrame(t));
    };

    // 3. Enabled: slew + clamp
    cl.setEnabled(true);
    t += 30; cl.setCommandAt(want, t);
    s = sentOf(cl.nextFrame(t));                 // dt 30 ms → step 4.5 %
    CHECK(s.sticksEnabled && s.throttleEnabled);
    NEAR(s.rollPct, 4.5f, 0.2f);
    NEAR(s.pitchPct, -4.5f, 0.2f);
    NEAR(s.throttleUs, 1450.0f, 0.7f);           // engage edge: no slew
    for (int i = 0; i < 20; ++i) s = tick(30, true);
    NEAR(s.rollPct, 25.0f, 0.2f);                // maxStickPct
    NEAR(s.pitchPct, -10.0f, 0.2f);
    NEAR(s.yawPct, 30.0f, 0.2f);                 // maxYawPct

    // 4. Throttle slew after engage, and clamp
    want.throttleUs = 1600;
    s = tick(100, true);                         // 250 us/s * 0.1 s
    NEAR(s.throttleUs, 1475.0f, 0.7f);
    want.throttleUs = 1990;
    for (int i = 0; i < 100; ++i) s = tick(30, true);
    NEAR(s.throttleUs, 1750.0f, 0.7f);           // throttleMaxUs

    // 5. Lease: Python stops calling setCommand → neutral within leaseMs
    const int64_t last = t;
    s = tick(250, false);
    CHECK(s.sticksEnabled);
    s = tick(last + 301 - t, false);
    CHECK(!s.sticksEnabled && !s.throttleEnabled);
    NEAR(s.rollPct, 0.0f, 0.2f);
    CHECK(!cl.getStatusAt(t).leaseValid);

    // 6. Master disable is immediate
    s = tick(30, true);
    CHECK(s.sticksEnabled);
    cl.setEnabled(false);
    s = tick(30, true);
    CHECK(!s.sticksEnabled && !s.throttleEnabled);

    // 7. Round-trip time from the echoed alive value
    CommandLink rt;
    rt.onFrame(statusFrame(0x01, -1), 5000);
    rt.nextFrame(5000);                          // seq 0 at t=5000
    rt.nextFrame(5030);                          // seq 1 at t=5030
    rt.onFrame(statusFrame(0x01, Uplink::aliveValue(0)), 5085);
    NEAR(rt.getStatusAt(5085).rttMs, 85.0f, 0.01f);

    // 8. Bridge timeout and port close stop writing
    CHECK(rt.frameDue(5085));
    CHECK(!rt.frameDue(5085 + 1501));
    rt.onFrame(statusFrame(0x01, -1), 7000);
    CHECK(rt.frameDue(7000));
    rt.onPortClosed();
    CHECK(!rt.frameDue(7000));

    // 9. Radio channel echo
    std::array<uint16_t, 16> ch;
    ch.fill(992);
    ch[2] = Uplink::usToCrsf(1400);
    Crsf::Frame rc;
    rc.type = Crsf::Type::RC_CHANNELS;
    rc.payload = Uplink::packChannels(ch);
    rt.onFrame(rc, 7100);
    auto st = rt.getStatusAt(7100);
    CHECK(st.radioChannelsValid);
    NEAR(st.radioChannelsUs[2], 1400.0f, 0.7f);

    // 10. Status state names
    CommandLink sn;
    sn.onFrame(statusFrame(0x03, -1), 100);      // fresh + gate
    CHECK(sn.getStatusAt(100).state == "GATE_ON");
    sn.onFrame(statusFrame(0x07, -1), 110);
    CHECK(sn.getStatusAt(110).state == "ACTIVE");

    // 11. Gimbal: setGimbalPct without enable, kept by neutral(), clamped,
    //     and an old v1 bridge is still accepted
    CommandLink gb;
    gb.onFrame(statusFrame(0x01, -1, 1), 200);
    CHECK(gb.getStatusAt(200).bridgeDetected);
    gb.setGimbalPct(60.0f, -20.0f);
    auto g = sentOf(gb.nextFrame(200));
    NEAR(g.gimbalPanPct, 60.0f, 0.2f);
    NEAR(g.gimbalTiltPct, -20.0f, 0.2f);
    gb.neutral();
    gb.setGimbalPanPct(150.0f);
    g = sentOf(gb.nextFrame(240));
    NEAR(g.gimbalPanPct, 100.0f, 0.2f);
    NEAR(g.gimbalTiltPct, -20.0f, 0.2f);
    CommandLink old;
    Crsf::Frame bad = statusFrame(0x01, -1, 9);   // unknown future version
    old.onFrame(bad, 300);
    CHECK(!old.getStatusAt(300).bridgeDetected);
}

// ── CLI helpers for the Lua cross-check ─────────────────────────────────────
static int cmdEncode(const char* path) {
    // Fixed scenario read by test_lua_bridge.py (keep both in sync):
    //   frame k: roll 20, pitch -15, yaw 40, thr 1450, tilt -60, pan 45,
    //            sticks on, throttle on, sequence k
    std::ofstream f(path, std::ios::binary);
    Uplink::CommandValues c;
    c.rollPct = 20; c.pitchPct = -15; c.yawPct = 40; c.throttleUs = 1450;
    c.gimbalTiltPct = -60; c.gimbalPanPct = 45; c.sticksEnabled = true; c.throttleEnabled = true;
    for (uint32_t k = 0; k < 3; ++k) {
        auto fr = Uplink::buildCommandFrame(c, k);
        f.write(reinterpret_cast<const char*>(fr.data()), static_cast<std::streamsize>(fr.size()));
    }
    return 0;
}

static int cmdDecode(const char* path) {
    std::ifstream f(path, std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    Crsf::StreamParser p;
    std::vector<Crsf::Frame> frames;
    p.feed(bytes.data(), bytes.size(), frames);
    std::printf("{\"crc_errors\": %u, \"frames\": [\n", p.crcErrors());
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& fr = frames[i];
        std::printf("  {\"type\": %d", fr.type);
        Crsf::Attitude a; Crsf::Battery b; Crsf::Gps g; Crsf::LinkStats l;
        Crsf::BaroAltitude ba; Crsf::Vario v; std::string fm; Uplink::BridgeStatus bs;
        std::array<uint16_t, 16> ch;
        if (Crsf::decodeAttitude(fr, a))
            std::printf(", \"pitch\": %.3f, \"roll\": %.3f, \"yaw\": %.3f", a.pitchDeg, a.rollDeg, a.yawDeg);
        else if (Crsf::decodeBattery(fr, b))
            std::printf(", \"v\": %.2f, \"a\": %.2f, \"mah\": %u, \"pct\": %u", b.voltageV, b.currentA, b.mahDrawn, b.remainingPct);
        else if (Crsf::decodeGps(fr, g))
            std::printf(", \"lat\": %.7f, \"lon\": %.7f, \"kmh\": %.1f, \"hdg\": %.2f, \"alt\": %d, \"sats\": %u",
                        g.latitude, g.longitude, g.groundSpeedKmh, g.headingDeg, g.altitudeM, g.satellites);
        else if (Crsf::decodeLinkStats(fr, l))
            std::printf(", \"rssi1\": %d, \"lq\": %u, \"snr\": %d, \"txmw\": %u, \"dlq\": %u",
                        l.uplinkRssi1Dbm, l.uplinkLq, l.uplinkSnr, l.txPowerMw, l.downlinkLq);
        else if (Crsf::decodeBaroAltitude(fr, ba))
            std::printf(", \"alt_cm\": %d", ba.altitudeCm);
        else if (Crsf::decodeVario(fr, v))
            std::printf(", \"vario_cms\": %d", v.verticalSpeedCmS);
        else if (Crsf::decodeFlightMode(fr, fm))
            std::printf(", \"fm\": \"%s\"", fm.c_str());
        else if (Uplink::decodeBridgeStatus(fr, bs))
            std::printf(", \"fresh\": %d, \"gate\": %d, \"stk\": %d, \"thr\": %d, \"live\": %d, \"echo\": %d,"
                        " \"rx\": %u, \"thr_stick\": %d, \"thr_delta\": %d, \"out_roll\": %d,"
                        " \"out_pitch\": %d, \"out_yaw\": %d, \"out_tilt\": %d, \"age_ms\": %d,"
                        " \"version\": %d, \"has_pan\": %d, \"out_pan\": %d",
                        bs.fresh, bs.gateOn, bs.sticksActive, bs.throttleActive, bs.telemetryLive,
                        bs.echoAlive, bs.rxFrames, bs.throttleStick, bs.throttleDelta, bs.outRoll,
                        bs.outPitch, bs.outYaw, bs.outTilt, bs.cmdAgeMs,
                        bs.version, bs.hasPan, bs.outPan);
        else if (fr.type == Crsf::Type::RC_CHANNELS && Uplink::unpackChannels(fr.payload, ch)) {
            std::printf(", \"us\": [");
            for (int k = 0; k < 16; ++k) std::printf("%s%.1f", k ? ", " : "", Uplink::crsfToUs(ch[k]));
            std::printf("]");
        }
        std::printf("}%s\n", i + 1 < frames.size() ? "," : "");
    }
    std::printf("]}\n");
    return 0;
}

// -----------------------------------------------------------------------------
// Concurrency: the cockpit calls CommandLink from three threads at once
// (UplinkController 25 Hz, Tk status 10 Hz, CrsfLink worker ~33 Hz + every
// received frame). Run them for real, faster than in the app, and check that
//   - no call ever waits noticeably (all locks are short, none nested),
//   - every frame on the wire is valid and inside the limits,
//   - when the commanding thread stops, the lease drops to neutral in time.
// Build with -fsanitize=thread to also prove there are no data races.
// -----------------------------------------------------------------------------
static void testConcurrency() {
    using clk = std::chrono::steady_clock;
    CommandLink cl;
    std::atomic<bool> stopCmd{false}, stopAll{false};
    std::atomic<int64_t> worstUs{0};
    auto timed = [&](auto&& fn) {
        const auto a = clk::now();
        fn();
        const int64_t us = std::chrono::duration_cast<std::chrono::microseconds>(clk::now() - a).count();
        int64_t prev = worstUs.load();
        while (us > prev && !worstUs.compare_exchange_weak(prev, us)) {}
    };

    cl.onFrame(statusFrame(0x01, 1500), CommandLink::nowMs());
    cl.setEnabled(true);

    std::atomic<int> badFrames{0}, outOfLimits{0}, frames{0};
    std::thread worker([&] {                       // CrsfLink worker
        while (!stopAll) {
            const int64_t now = CommandLink::nowMs();
            timed([&] { cl.onFrame(statusFrame(0x01, 1500), now); });
            bool due = false;
            timed([&] { due = cl.frameDue(now); });
            if (due) {
                std::vector<uint8_t> f;
                timed([&] { f = cl.nextFrame(now); });
                if (!f.empty()) {
                    ++frames;
                    if (parseAll(f).size() != 1) ++badFrames;
                    const auto v = sentOf(f);
                    if (std::fabs(v.rollPct) > 25.3f || std::fabs(v.pitchPct) > 25.3f ||
                        std::fabs(v.yawPct) > 30.3f ||
                        (v.throttleEnabled && (v.throttleUs < 1149 || v.throttleUs > 1751)))
                        ++outOfLimits;
                    timed([&] { cl.onWriteResult(true); });
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
    std::thread commander([&] {                    // UplinkController (fast)
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> u(-100.f, 100.f);
        int i = 0;
        while (!stopCmd) {
            Uplink::CommandValues c;
            c.sticksEnabled = true; c.throttleEnabled = (i / 50) % 2 == 0;
            c.rollPct = u(rng); c.pitchPct = u(rng); c.yawPct = u(rng);
            c.throttleUs = 1450.f + u(rng) * 5.f;  // also out of range on purpose
            c.gimbalTiltPct = u(rng);
            timed([&] { cl.setCommand(c); });
            if (i % 97 == 0) timed([&] { cl.neutral(); });
            ++i;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
    std::thread ui([&] {                           // Tk status + gimbal slider
        int i = 0;
        while (!stopAll) {
            timed([&] { (void)cl.getStatus(); });
            if (i++ % 10 == 0) timed([&] { cl.setGimbalTiltPct(static_cast<float>(i % 100)); });
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    stopCmd = true;                                // controller thread "hangs"
    commander.join();
    const int64_t stoppedAt = CommandLink::nowMs();
    bool neutralSeen = false;
    while (CommandLink::nowMs() - stoppedAt < 1000) {
        if (!cl.getStatus().leaseValid) { neutralSeen = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const int64_t dropMs = CommandLink::nowMs() - stoppedAt;
    stopAll = true;
    worker.join();
    ui.join();

    std::printf("  concurrency: %d frames, worst call %lld us, lease dropped after %lld ms\n",
                frames.load(), static_cast<long long>(worstUs.load()),
                static_cast<long long>(dropMs));
    CHECK(frames > 20);
    CHECK(badFrames == 0);
    CHECK(outOfLimits == 0);
    CHECK(neutralSeen && dropMs <= 300 + 50);
    CHECK(worstUs < 20000);                        // generous: CI/sanitizer scheduling
}

int main(int argc, char** argv) {
    if (argc == 3 && std::strcmp(argv[1], "encode") == 0) return cmdEncode(argv[2]);
    if (argc == 3 && std::strcmp(argv[1], "decode") == 0) return cmdDecode(argv[2]);
    testProtocol();
    testCommandLink();
    testConcurrency();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
