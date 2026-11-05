// Host unit test of the portable application layer: settings blob, JSON reader, leader frame, command flow,
// and fault injection against the emulated RCB runtime (sim_rcb.h) with an ideal leader and follower:
// raw gripper telemetry, unreachable poses, angle branches, encoder wrap, duplicate / frozen state packets,
// a stalled control task, unusable telemetry values, clutch, precision, locks, the target jump guard, strict
// boot with a missing servo, replay, range calibration, barriers and the settings guards.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "app.h"
#include "arm_design.h"
#include "boot.h"
#include "config_store.h"
#include "json_lite.h"
#include "leader_frame.h"
#include "loop_stats.h"
#include "sim_rcb.h"

using namespace raft;

static int g_fail = 0;
static void check(bool ok, const char* what) { std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) ++g_fail; }

static const double kPiD = 3.14159265358979323846;
static double rad(double d) { return d * kPiD / 180.0; }
static double deg(double r) { return r * 180.0 / kPiD; }

struct RamStorage : Storage {
    uint8_t blob[512]; size_t len = 0; int saves = 0;
    bool load(PersistentConfig& c) override { return len && deserializeConfig(blob, len, c); }
    bool save(const PersistentConfig& c) override { len = serializeConfig(c, blob, sizeof(blob)); ++saves; return len > 0; }
};

// The settings layout before 2026-10-31 (config_store.cpp keeps its own copy to upgrade from)
struct JointCalV1 { int16_t zeroTicks; int8_t dir; uint8_t flags; float ktNmPerA; float frictionNm; };
// The version 2 layout, as config_store.cpp keeps it
struct PersistentConfigV2 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    JointCal joint[kNumJoints];
    float    mAPerUnit;
    float    currentRegPerUnit;
    float    gravityScale;
    SpringModel spring;
    uint8_t  autoRest;
    float    autoRestIdleS;
    char     piHost[40];
    char     streamHost[40];
    uint16_t streamPort;
    uint16_t reserved;
    TeleopSettings teleop;
    uint32_t crc;
};

// The version 3 layout: version 4 without the arm name
struct PersistentConfigV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    JointCal joint[kNumJoints];
    float    mAPerUnit;
    float    currentRegPerUnit;
    float    gravityScale;
    SpringModel spring;
    uint8_t  autoRest;
    float    autoRestIdleS;
    char     piHost[40];
    char     streamHost[40];
    uint16_t streamPort;
    uint16_t reserved;
    TeleopSettings teleop;
    float    holdFriction;
    uint32_t crc;
};

struct PersistentConfigV1 {
    uint32_t magic; uint16_t version; uint16_t size; JointCalV1 joint[kNumJoints];
    float mAPerUnit, currentRegPerUnit, gravityScale; SpringModel spring; uint8_t autoRest; float autoRestIdleS;
    char piHost[40]; char streamHost[40]; uint16_t streamPort; uint16_t reserved; uint32_t crc;
};

// Fake bus for the strict boot sequence: records the order of calls
struct FakeBootBus : BootBus {
    std::string log; uint8_t silent = 0; uint32_t t = 0;
    void torqueOffAll() override { log += 'T'; }
    uint8_t missing() override { log += 'P'; return silent; }
    void waitMs(uint32_t ms) override { t += ms; }
    uint32_t nowMs() override { return t; }
};

// App + emulated PAR6, 1 kHz steps: control at 200 Hz (the App is time-based), robot loop at 250 Hz, network every step.
// The leader is ideal: in MOVE it is where the controller wants it, otherwise where the test (the hand) puts it.
struct Rig {
    RamStorage st;
    simrcb::FakeRcb rcb;
    App app;
    RawServoState raw = {};
    double t = 0.0;
    long ms = 0;
    double qL[7] = {0};          // leader, CAD (rad), unwrapped
    double fCad[6] = {0};        // follower, CAD
    double text[6] = {0};
    double gp = 0.0;
    bool controlRuns = true, followerFollows = true;
    int16_t out[7] = {0};
    bool on = false, need = false;

    static simrcb::NetConfig net() { simrcb::NetConfig n; return n; }
    Rig() : rcb(net()), app(st, rcb)
    {
        PersistentConfig c;
        defaultPersistentConfig(c);
        for (int i = 0; i < 7; ++i) { c.joint[i].zeroTicks = 2048; c.joint[i].dir = 1; c.joint[i].flags = CAL_ZERO | CAL_DIR; }
        st.save(c);
        app.begin(0.0, true);
        for (int i = 0; i < 7; ++i) { raw.ok[i] = true; raw.tempC[i] = 30; raw.voltage[i] = 60; }
    }
    std::string cmd(const char* line) { char rep[4096]; app.handleCommand(line, rep, sizeof(rep), t, "10.0.0.9"); return rep; }
    void encode()
    {
        for (int i = 0; i < 7; ++i) {
            long ticks = std::lround(kJointDir[i] * qL[i] / (2.0 * kPiD / 4096.0)) + 2048;   // servos as the design says
            ticks %= 4096; if (ticks < 0) ticks += 4096;
            raw.pos[i] = (int16_t)ticks;
        }
    }
    void run(double seconds)
    {
        const long steps = std::lround(seconds * 1000.0);
        for (long k = 0; k < steps; ++k) {
            t += 0.001;
            rcb.setTime(t);
            if (ms % 4 == 0) {
                double q[6], qd[6], dh[6], z[6] = {0};
                if (followerFollows && rcb.target(q, qd)) {
                    const double off[6] = {0, -kPiD / 2, kPiD, 0, 0, kPiD};
                    for (int i = 0; i < 6; ++i) fCad[i] = q[i] - off[i];
                }
                const double off[6] = {0, -kPiD / 2, kPiD, 0, 0, kPiD};
                for (int i = 0; i < 6; ++i) dh[i] = fCad[i] + off[i];
                rcb.robotTick(dh, z, text, gp, 0.0);
            }
            app.networkTick(t);
            if (controlRuns && ms % 5 == 0) {
                encode();
                app.controlTick(raw, t, out, on, need);
                if (app.mode() == Mode::Move) for (int i = 0; i < 7; ++i) qL[i] = app.lastOutput().qRef[i];
            }
            ++ms;
        }
    }
    TeleopSession::Stage stage() const { return app.session().stage(); }
    bool runUntil(TeleopSession::Stage s, double maxS)
    {
        for (double w = 0; w < maxS; w += 0.05) { if (stage() == s) return true; run(0.05); }
        return stage() == s;
    }
    void setFollowerDeg(const double d[6]) { for (int i = 0; i < 6; ++i) fCad[i] = rad(d[i]); }
};

static void testBlobAndParsers()
{
    std::printf("=== Settings blob ===\n");
    {
        PersistentConfig a; defaultPersistentConfig(a);
        a.joint[2].zeroTicks = 1234; a.joint[2].dir = -1; a.spring.k = 824.0f; std::strcpy(a.piHost, "192.168.1.20");
        a.teleop.forceGain = 0.04f; a.joint[1].flags |= CAL_RANGE; a.joint[1].qMaxRad = 1.2f;
        uint8_t buf[512]; const size_t n = serializeConfig(a, buf, sizeof(buf));
        PersistentConfig b; defaultPersistentConfig(b);
        check(n == sizeof(PersistentConfig) && deserializeConfig(buf, n, b) && b.joint[2].zeroTicks == 1234 && b.joint[2].dir == kJointDir[2]
              && b.spring.k == 824.0f && !std::strcmp(b.piHost, "192.168.1.20") && b.teleop.forceGain == 0.04f && b.joint[1].qMaxRad == 1.2f, "round trip; a stored direction is replaced by the design's");
        buf[20] ^= 0x40;
        check(!deserializeConfig(buf, n, b), "a flipped bit is rejected (CRC)");
        check(!deserializeConfig(buf, n - 1, b), "a short blob is rejected");
        check(!fullyCalibrated(a), "defaults are not calibrated");
        std::printf("  (blob %u bytes, version %u)\n", (unsigned)sizeof(PersistentConfig), (unsigned)kConfigVersion);

        PersistentConfigV1 v; std::memset(&v, 0, sizeof(v));
        v.magic = kConfigMagic; v.version = 1; v.size = sizeof(PersistentConfigV1);
        for (int i = 0; i < 7; ++i) { v.joint[i].zeroTicks = (int16_t)(1000 + i); v.joint[i].dir = (int8_t)(i % 2 ? -1 : 1); v.joint[i].flags = CAL_ZERO | CAL_DIR; v.joint[i].ktNmPerA = 0.45f; }
        v.mAPerUnit = 6.0f; v.currentRegPerUnit = 1.1f; v.gravityScale = 0.9f; v.spring.type = SpringType::ZeroLengthCable; v.spring.k = 718.0f;
        std::strcpy(v.piHost, "par6.local"); v.streamPort = 6000;
        v.crc = crc32((const uint8_t*)&v, offsetof(PersistentConfigV1, crc));
        PersistentConfig u; defaultPersistentConfig(u);
        check(deserializeConfig((const uint8_t*)&v, sizeof(v), u) && u.joint[3].zeroTicks == 1003 && u.joint[3].dir == kJointDir[3] && fullyCalibrated(u)
              && u.spring.k == 718.0f && !std::strcmp(u.piHost, "par6.local") && u.streamPort == 6000 && u.teleop.gripperClosedRaw == 255,
              "a version 1 blob (saved before the teleop settings) still loads, new fields at their defaults");

        PersistentConfig cur; defaultPersistentConfig(cur);
        cur.joint[2].zeroTicks = 777; cur.gravityScale = 1.31f; cur.teleop.forceGain = 0.03f; cur.mAPerUnit = 3.25f;
        PersistentConfigV2 w; std::memset(&w, 0, sizeof(w));
        std::memcpy(&w, &cur, offsetof(PersistentConfigV2, crc));   // the same layout up to the new field
        w.version = 2; w.size = sizeof(PersistentConfigV2);
        w.crc = crc32((const uint8_t*)&w, offsetof(PersistentConfigV2, crc));
        PersistentConfig x; defaultPersistentConfig(x);
        check(deserializeConfig((const uint8_t*)&w, sizeof(w), x) && x.joint[2].zeroTicks == 777 && x.gravityScale == 1.0f
              && std::fabs(x.holdFriction - 0.31f) < 1e-5f && x.teleop.forceGain == 0.03f && x.mAPerUnit == 3.25f,
              "a version 2 blob loads: its gravity 1.31 becomes gravity 1.00 + friction compensation (holding) 0.31");

        cur.holdFriction = 0.27f; std::strcpy(cur.streamHost, "10.0.0.5");
        PersistentConfigV3 w3; std::memset(&w3, 0, sizeof(w3));
        std::memcpy(&w3, &cur, offsetof(PersistentConfigV3, crc));
        w3.version = 3; w3.size = sizeof(PersistentConfigV3);
        w3.crc = crc32((const uint8_t*)&w3, offsetof(PersistentConfigV3, crc));
        PersistentConfig y; defaultPersistentConfig(y); std::strcpy(y.armName, "stale");
        check(sizeof(PersistentConfigV3) != sizeof(PersistentConfig) && deserializeConfig((const uint8_t*)&w3, sizeof(w3), y)
              && y.joint[2].zeroTicks == 777 && y.holdFriction == 0.27f && !std::strcmp(y.streamHost, "10.0.0.5") && y.armName[0] == '\0'
              && y.version == kConfigVersion, "a version 3 blob loads with everything kept and no name");

        PersistentConfig nmd; defaultPersistentConfig(nmd); std::strcpy(nmd.armName, "left");
        uint8_t nb[512]; size_t nn = serializeConfig(nmd, nb, sizeof(nb));
        PersistentConfig back; defaultPersistentConfig(back);
        check(deserializeConfig(nb, nn, back) && !std::strcmp(back.armName, "left"), "the name survives the round trip");
        std::strcpy(nmd.armName, "Bad Name");
        nn = serializeConfig(nmd, nb, sizeof(nb));
        check(deserializeConfig(nb, nn, back) && back.armName[0] == '\0', "a stored name that breaks the rules loads as no name");
    }
    std::printf("=== Arm name rules ===\n");
    {
        char o[kArmNameMax + 1], h[32];
        check(normalizeArmName("Left-2", o, sizeof(o)) && !std::strcmp(o, "left-2"), "letters are lowercased, digits and '-' kept");
        check(normalizeArmName("", o, sizeof(o)) && o[0] == '\0', "empty is valid (no name)");
        check(!normalizeArmName("no space", o, sizeof(o)) && !normalizeArmName("-left", o, sizeof(o)) && !normalizeArmName("left-", o, sizeof(o))
              && !normalizeArmName("a_b", o, sizeof(o)) && !normalizeArmName("arm.1", o, sizeof(o)), "space, '_', '.' and a leading or trailing '-' are refused");
        check(normalizeArmName("abcdefghijklmnopqrst", o, sizeof(o)) && !normalizeArmName("abcdefghijklmnopqrstu", o, sizeof(o)), "20 characters fit, 21 do not");
        armHostname("left", h, sizeof(h));
        const bool named = !std::strcmp(h, "raft-left");
        armHostname("", h, sizeof(h));
        check(named && !std::strcmp(h, "raft-leader"), "hostname raft-left, or raft-leader without a name");
    }
    std::printf("=== JSON reader ===\n");
    {
        const char* l = "{\"cmd\": \"move\", \"q_deg\": [0, 20.5, -3e1, 4, 5, 6, 7], \"speed_deg\": 30, \"on\": true, \"host\": \"pi.local\"}";
        char s[32]; double v, a[8]; bool b;
        check(json::getString(l, "cmd", s, sizeof(s)) && !std::strcmp(s, "move"), "string");
        check(json::getNumber(l, "speed_deg", v) && v == 30.0, "number");
        check(json::getNumberArray(l, "q_deg", a, 7) == 7 && a[1] == 20.5 && a[2] == -30.0, "number array");
        check(json::getBool(l, "on", b) && b, "bool");
        check(json::getString(l, "host", s, sizeof(s)) && !std::strcmp(s, "pi.local"), "host string");
        check(!json::getNumber(l, "missing", v) && json::getNumberArray(l, "missing", a, 7) == -1, "missing keys");
        const char* bad = "{\"a\": inf, \"b\": nan, \"c\": 1e999, \"d\": -1e10, \"e\": [1, nan, 3], \"f\": 1e9}";
        check(!json::getNumber(bad, "a", v) && !json::getNumber(bad, "b", v) && !json::getNumber(bad, "c", v) && !json::getNumber(bad, "d", v)
              && json::getNumberArray(bad, "e", a, 3) == -1 && json::getNumber(bad, "f", v), "inf, nan, overflow and > 1e9 are not numbers");
        check(std::isnan(wrapPi(INFINITY)) && std::fabs(wrapPi(7.0f) - (7.0f - 2.0f * 3.14159265f)) < 1e-5f && wrapPi(3.14159265f) > 3.0f,
              "wrapPi returns NaN for inf instead of looping, wraps finite values to (-pi, pi]");
    }
    std::printf("=== Leader frame ===\n");
    {
        LeaderFrame f = {}; f.mode = 2; f.flags = LF_CALIBRATED | LF_SPRING | LF_FOLLOWER; f.seq = 77; f.q[3] = 0.5f; f.tcp[2] = 0.175f; f.tempC[6] = 41; f.events = 3;
        f.currentmA[1] = -321; f.voltage[4] = 61; f.stage = 4; f.teleopFlags = TF_ENGAGED | TF_EXT_VALID; f.locks = 5; f.qFollower[2] = 1.25f;
        f.followerTarget[5] = 3.5f; f.tauExt[1] = -2.5f; f.gripperRaw = 128.0f; f.followerTick = 4000000000u; f.precision = 0.25f; f.guide = 3;
        f.pcEnergyJ = -0.001f; f.offsetRad = 0.2f;
        uint8_t buf[400]; LeaderFrame g = {};
        const size_t n = packLeaderFrame(f, buf, sizeof(buf));
        check(n == kLeaderFrameSize && n == 288 && !std::memcmp(buf, "RAFT", 4) && buf[4] == 2 && unpackLeaderFrame(buf, n, g) && g.seq == 77 && g.q[3] == 0.5f
              && g.tcp[2] == 0.175f && g.tempC[6] == 41 && g.events == 3 && g.flags == (LF_CALIBRATED | LF_SPRING | LF_FOLLOWER), "pack / unpack 288 bytes (version 1 part)");
        check(g.currentmA[1] == -321 && g.voltage[4] == 61 && g.stage == 4 && g.teleopFlags == (TF_ENGAGED | TF_EXT_VALID) && g.locks == 5 && g.qFollower[2] == 1.25f
              && g.followerTarget[5] == 3.5f && g.tauExt[1] == -2.5f && g.gripperRaw == 128.0f && g.followerTick == 4000000000u && g.precision == 0.25f
              && g.guide == 3 && g.pcEnergyJ == -0.001f && g.offsetRad == 0.2f, "version 2 fields survive the round trip");
        check(!unpackLeaderFrame(buf, 160, g), "a 160-byte (version 1) frame is refused");
    }
    std::printf("=== Strict boot (fake bus) ===\n");
    {
        FakeBootBus ok;
        check(strictBoot(ok, 10000, 500) == 0 && ok.log == "TP", "torque off goes out before the first presence check");
        FakeBootBus miss; miss.silent = 1u << 3;
        const uint8_t m = strictBoot(miss, 10000, 500);
        check(m == (1u << 3) && miss.log.size() > 4 && miss.log[0] == 'T' && miss.log.find("PP") == std::string::npos,
              "a missing servo (J4): the mask comes back after the timeout, torque off before every retry");
    }
}

static void testCommandFlow()
{
    std::printf("=== Command flow (synthetic servo readings) ===\n");
    RamStorage st; simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    App app(st, rcb);
    app.begin(0.0, true);
    RawServoState raw = {};
    for (int i = 0; i < kNumJoints; ++i) { raw.ok[i] = true; raw.pos[i] = 2048; raw.tempC[i] = 30; }
    int16_t out[kNumJoints]; bool on, need;
    char rep[4096];
    double t = 0;
    auto tick = [&](int n) { for (int k = 0; k < n; ++k) { t += 0.005; app.controlTick(raw, t, out, on, need); } };
    tick(2);
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, "10.0.0.9");
    check(std::strstr(rep, "\"ok\": false") && std::strstr(rep, "not calibrated"), "gravity refused before calibration");
    app.handleCommand("{\"cmd\": \"calib_zero\"}", rep, sizeof(rep), t, nullptr);
    tick(120);
    check(app.config().joint[0].flags & CAL_ZERO, "calib_zero stores the zero pose");
    check(fullyCalibrated(app.config()) && app.config().joint[0].dir == kJointDir[0] && app.config().joint[6].dir == kJointDir[6],
          "after the zero the arm counts as calibrated, with the design's directions");
    app.handleCommand("{\"cmd\": \"calib_dir\", \"joint\": 2}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && std::strstr(rep, "fixed"), "calib_dir is refused: the directions are fixed");
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr);
    tick(10);
    check(std::strstr(rep, "\"ok\": true") && on && app.mode() == Mode::Gravity, "gravity accepted once calibrated, torque on");
    app.handleCommand("{\"cmd\": \"status\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"mode\": \"gravity\"") && std::strstr(rep, "\"calibrated\": true") && rep[std::strlen(rep) - 1] == '}', "status reply");
    std::printf("  status: %u bytes, %.160s...\n", (unsigned)std::strlen(rep), rep);
    // settings guards
    app.handleCommand("{\"cmd\": \"set_kt\", \"joint\": 2, \"kt\": 0.5}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && std::strstr(rep, "limp") && app.config().joint[1].ktNmPerA != 0.5f, "set_dir refused while the torque is on (it would flip the compensation)");
    app.handleCommand("{\"cmd\": \"set_kt\", \"joint\": 2, \"kt\": 0.2}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false"), "set_kt refused while the torque is on");
    app.handleCommand("{\"cmd\": \"load\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && std::strstr(rep, "limp"), "load refused while the torque is on");
    app.handleCommand("{\"cmd\": \"spring\", \"type\": \"cable\", \"k\": 824, \"a_mm\": 13, \"b_mm\": 20}", rep, sizeof(rep), t, nullptr);
    check(app.config().spring.type == SpringType::ZeroLengthCable && std::fabs(app.config().spring.b - 0.020f) < 1e-6f, "spring command (a bench knob, allowed in gravity)");
    app.handleCommand("{\"cmd\": \"spring\", \"k\": 1e999}", rep, sizeof(rep), t, nullptr);
    check(app.config().spring.k == 824.0f, "an overflowing number does not reach the settings");
    app.handleCommand("{\"cmd\": \"spring\", \"k\": -5}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && app.config().spring.k == 824.0f, "an out-of-range spring is refused");
    app.handleCommand("{\"cmd\": \"save\"}", rep, sizeof(rep), t, nullptr);
    check(st.saves == 1 && std::strstr(rep, "\"saved\": true"), "save writes the blob");
    App app2(st, rcb); app2.begin(0.0, true);
    check(fullyCalibrated(app2.config()) && app2.config().spring.k == 824.0f, "a fresh boot loads the saved calibration");
    app.handleCommand("{\"cmd\": \"calib_torque\", \"joint\": 2}", rep, sizeof(rep), t, nullptr);
    tick(2);
    PersistentConfig snap; char why[120];
    check(!app.prepareSave(snap, why, sizeof(why)) && std::strstr(why, "calibration") && !app.maintenanceAllowed(why, sizeof(why)),
          "save and servo_info are refused during a calibration (flash and bus pauses)");
    app.handleCommand("{\"cmd\": \"calib_abort\"}", rep, sizeof(rep), t, nullptr);
    app.handleCommand("{\"cmd\": \"stream\", \"port\": 6000}", rep, sizeof(rep), t, "10.0.0.9");
    check(!std::strcmp(app.streamHost(), "10.0.0.9") && app.streamPort() == 6000, "stream defaults to the command client's address");
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr);
    tick(2);
    raw.pos[3] = 2100; tick(1);                           // J4 turning: a speed, then its reads fail
    const float q4 = app.lastInput().q[3];
    raw.speed[3] = 40; tick(1); raw.speed[3] = 0;
    raw.ok[3] = false; tick(49);
    check(on && app.mode() == Mode::Gravity && !app.fault()[0] && app.lastInput().q[3] == q4 && app.lastInput().qd[3] == 0.0f,
          "49 missed reads in a row with torque on: still running on J4's last angle, its speed taken as 0");
    tick(1);
    check(app.mode() == Mode::Limp && !std::strcmp(app.fault(), "servo J4 not answering"), "the 50th (200 ms) faults the arm to limp, naming only it");
    raw.ok[3] = true;
    app.handleCommand("{\"cmd\": \"clear_fault\"}", rep, sizeof(rep), t, nullptr);
    check(!app.fault()[0], "clear_fault");
    raw.ok[5] = false; tick(100);
    check(!app.fault()[0], "limp: a missing servo only counts, no fault");
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && std::strstr(rep, "J6 is not answering") && app.mode() == Mode::Limp, "torque is refused while it is missing");
    raw.ok[5] = true; tick(1);
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr);
    tick(2);
    check(app.mode() == Mode::Gravity && on, "and accepted once it answers");
    for (int i = 3; i < 7; ++i) raw.ok[i] = false;
    tick(50);
    check(!std::strcmp(app.fault(), "servo J4 J5 J6 J7 not answering"), "a broken chain names every servo behind it");
    for (int i = 3; i < 7; ++i) raw.ok[i] = true;
    app.handleCommand("{\"cmd\": \"status\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"missed\": [0, 0, 0, 100, 50, 150, 50]") != nullptr, "status counts the missed reads per servo");
    app.handleCommand("{\"cmd\": \"clear_fault\"}", rep, sizeof(rep), t, nullptr);
    raw.tempC[1] = 76; tick(1);
    check(app.mode() == Mode::Limp && std::strstr(app.fault(), "76"), "over-temperature faults the arm");
    raw.tempC[1] = 30;
    app.handleCommand("{\"cmd\": \"clear_fault\"}", rep, sizeof(rep), t, nullptr);
    raw.current[2] = 100; raw.voltage[2] = 61; tick(1);
    uint8_t fb[400];
    LeaderFrame lf = {};
    check(app.leaderFrame(fb, sizeof(fb)) == kLeaderFrameSize && unpackLeaderFrame(fb, kLeaderFrameSize, lf) && (lf.flags & LF_CALIBRATED), "leader frame from the app");
    check(lf.currentmA[2] == kCurrentSign * 325 && lf.voltage[2] == 61, "measured current (100 units x 3.25 mA, servo and joint sign) and voltage in the frame");
    // encoder wrap: J1 turned through +180 deg keeps counting
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"limp\"}", rep, sizeof(rep), t, nullptr);
    bool smooth = true;
    auto j1 = [](int d) { return (int16_t)(((2048 + kJointDir[0] * d) % 4096 + 4096) % 4096); };   // J1 turns the other way round
    raw.pos[0] = j1(1800); tick(1);                       // +158 deg
    float prev = app.lastInput().q[0];
    for (int k = 1; k <= 400; ++k) {
        raw.pos[0] = j1(1800 + 2 * k); tick(1);           // across 180 deg to +228 deg
        const float q = app.lastInput().q[0];
        if (std::fabs(q - prev) > 0.01f) smooth = false;
        prev = q;
    }
    check(smooth && std::fabs(deg(app.lastInput().q[0]) - 228.5) < 0.2, "a joint turned through +-180 deg keeps counting (unwrapped, no jump)");
    app.handleCommand("{\"cmd\": \"bogus\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "unknown cmd"), "unknown command is refused");
}

static void testRangeAndReplay()
{
    std::printf("=== Range calibration and replay (ideal leader) ===\n");
    Rig r;
    std::string s = r.cmd("{\"cmd\": \"calib_range\"}");
    check(s.find("\"ok\": true") != std::string::npos, "calib_range starts while limp");
    for (int k = 0; k <= 100; ++k) { r.qL[1] = rad(-50.0 + 1.3 * k); r.run(0.01); }    // J2 from -50 to +80 deg
    r.cmd("{\"cmd\": \"calib_range_stop\"}");
    r.run(0.01);
    const ControlConfig& cc = r.app.controlConfig();
    check((r.app.config().joint[1].flags & CAL_RANGE) && std::fabs(deg(cc.joint[1].qMinRad) + 48.0) < 0.3 && std::fabs(deg(cc.joint[1].qMaxRad) - 78.0) < 0.3
          && !(r.app.config().joint[0].flags & CAL_RANGE), "J2 range measured -50..80 deg (stored 2 deg inside); unmoved joints keep the defaults");
    r.qL[1] = 0.0;
    r.run(0.05);
    // replay: 1 s of J1 0 -> 20 deg at 50 Hz, uploaded in two commands
    r.cmd("{\"cmd\": \"replay_clear\"}");
    std::string a = "{\"cmd\": \"replay_add\", \"dt_ms\": 20, \"q_cdeg\": [", b = "{\"cmd\": \"replay_add\", \"q_cdeg\": [";
    for (int k = 0; k <= 50; ++k) {
        char one[80];
        std::snprintf(one, sizeof(one), "%s%d, 0, 0, 0, 0, 0, 0", (k == 0 || k == 26) ? "" : ", ", 40 * k);
        (k < 26 ? a : b) += one;
    }
    a += "]}"; b += "]}";
    r.cmd(a.c_str());
    s = r.cmd(b.c_str());
    check(s.find("\"n\": 51") != std::string::npos, "51 samples uploaded in two replay_add commands");
    r.cmd("{\"cmd\": \"mode\", \"mode\": \"gravity\"}");
    s = r.cmd("{\"cmd\": \"set_kt\", \"joint\": 1, \"kt\": 0.5}");
    check(s.find("\"ok\": false") != std::string::npos, "set_kt refused in gravity");
    s = r.cmd("{\"cmd\": \"replay_start\"}");
    check(s.find("\"ok\": true") != std::string::npos && r.app.replaying(), "replay_start: moves to the first sample, then plays");
    double peak = 0;
    for (int k = 0; k < 60 && r.app.replaying(); ++k) { r.run(0.05); peak = std::fmax(peak, deg(r.qL[0])); }
    check(!r.app.replaying() && std::fabs(deg(r.qL[0]) - 20.0) < 0.5 && r.app.mode() == Mode::Gravity, "the replay ends on the last sample (J1 20 deg) and hands back to gravity");
    s = r.cmd("{\"cmd\": \"status\"}");
    check(s.find("\"note\": \"done\"") != std::string::npos, "status reports the finished replay");
}

static void testTeleop()
{
    std::printf("=== Teleoperation fault injection (emulated PAR6) ===\n");
    {
        Rig r;
        const double f[6] = {10, 5, 20, 0, -15, 0};
        r.setFollowerDeg(f);
        r.gp = 128.0;                                     // the gripper half closed, raw scale
        r.cmd("{\"cmd\": \"teleop_start\", \"host\": \"par6.local\"}");
        check(r.runUntil(TeleopSession::Stage::Coupled, 5.0), "aligns onto the follower and couples");
        const float trig = r.app.session().followerQ()[6];
        check(std::fabs(deg(trig) - 128.0 / 255.0 * 60.0) < 0.3, "g_p is the raw 0..255 position: 128 maps to half the trigger travel (30 deg)");
        r.rcb.gpField = "-99999999";                       // the runtime's 'not reported yet' sentinel
        r.run(0.1);
        check(std::fabs(r.app.session().followerQ()[6] - trig) < 1e-6f && r.stage() == TeleopSession::Stage::Coupled,
              "the g_p sentinel is dropped: the trigger keeps its last value, the coupling stays");
        r.rcb.gpField.clear();
        r.rcb.textField = "nan,0,0,0,0,0";
        r.run(0.1);
        uint8_t fb[400]; LeaderFrame lf = {};
        r.app.leaderFrame(fb, sizeof(fb)); unpackLeaderFrame(fb, kLeaderFrameSize, lf);
        check(r.stage() == TeleopSession::Stage::Coupled && !(lf.teleopFlags & TF_EXT_VALID), "a NaN in t_ext drops t_ext only (still coupled)");
        r.rcb.textField.clear();
        // duplicates
        r.rcb.repeatPackets = true;
        const uint32_t before = r.app.link().staleStates();
        r.run(0.5);
        check(r.app.link().staleStates() > before + 50 && r.stage() == TeleopSession::Stage::Coupled, "duplicated state packets are dropped by their tick, the coupling stays");
        r.rcb.repeatPackets = false;
        // clutch
        const float tgt0 = r.app.session().followerTarget()[0];
        r.app.setButtons(false, true);
        r.run(0.05);
        check(r.stage() == TeleopSession::Stage::Held && r.app.mode() == Mode::Gravity, "clutch pressed: HELD, the leader is free");
        r.qL[0] += rad(15.0);
        r.run(0.2);
        check(std::fabs(r.app.session().followerTarget()[0] - tgt0) < 1e-4f, "while HELD the follower target stays put");
        r.app.setButtons(false, false);
        r.run(0.05);
        const float tgt1 = r.app.session().followerTarget()[0];
        check(r.stage() == TeleopSession::Stage::Coupled && std::fabs(tgt1 - tgt0) < rad(0.5), "clutch released: re-anchored where the leader is, no jump");
        std::string s = r.cmd("{\"cmd\": \"status\"}");
        check(s.find("\"offset_deg\": 15") != std::string::npos || s.find("\"offset_deg\": 14.") != std::string::npos, "the 15 deg offset is reported");
        // precision
        r.cmd("{\"cmd\": \"teleop_precision\", \"value\": 0.25}");
        r.run(0.05);
        const float p0 = r.app.session().followerTarget()[0];
        r.qL[0] += rad(8.0);
        r.run(0.1);
        check(std::fabs(deg(r.app.session().followerTarget()[0] - p0) - 2.0) < 0.1, "precision 0.25: 8 deg on the leader move the follower 2 deg");
        r.cmd("{\"cmd\": \"teleop_precision\", \"value\": 1}");
        // locks
        r.cmd("{\"cmd\": \"teleop_lock\", \"joints\": [1]}");
        r.run(0.05);
        const float l0 = r.app.session().followerTarget()[0], l1 = r.app.session().followerTarget()[1];
        r.qL[0] += rad(5.0); r.qL[1] += rad(3.0);
        r.run(0.1);
        check(std::fabs(r.app.session().followerTarget()[0] - l0) < 1e-4f && std::fabs(deg(r.app.session().followerTarget()[1] - l1) - 3.0) < 0.1,
              "J1 locked: the follower's J1 stays, J2 still follows");
        r.cmd("{\"cmd\": \"teleop_lock\", \"joints\": []}");
        r.run(0.05);
        // realign removes the offsets
        s = r.cmd("{\"cmd\": \"teleop_realign\"}");
        check(s.find("REALIGNING") != std::string::npos, "realign starts");
        check(r.runUntil(TeleopSession::Stage::Coupled, 10.0) && r.app.session().offsetMaxRad() < rad(2.5), "realign: the leader moved back onto the follower, offset gone");
        // the jump guard (an encoder slip)
        r.qL[3] += rad(40.0);
        r.run(0.02);
        check(r.stage() == TeleopSession::Stage::Held && std::strstr(r.app.session().note(), "jumped"), "a 40 deg target jump in one tick: HELD with a note");
        r.qL[3] -= rad(40.0);
        r.run(0.05);
        check(r.stage() == TeleopSession::Stage::Held, "... and it does not re-engage by itself");
        r.app.setButtons(false, true); r.run(0.05); r.app.setButtons(false, false); r.run(0.05);
        check(r.stage() == TeleopSession::Stage::Coupled, "a clutch press and release engages again");
        // PAR6 blocked (it stops following) while the leader moves on: realign goes to where PAR6 really is
        r.followerFollows = false;
        const double j1 = r.qL[0];
        for (int k = 1; k <= 50; ++k) { r.qL[0] = j1 + rad(12.0) * k / 50.0; r.run(0.01); }
        r.cmd("{\"cmd\": \"teleop_realign\"}");
        const bool back = r.runUntil(TeleopSession::Stage::Coupled, 10.0);
        check(back && std::fabs(deg(r.qL[0] - r.fCad[0])) < 2.5 && std::fabs(deg(r.app.session().followerTarget()[0] - r.fCad[0])) < 0.5,
              "a blocked PAR6: realign holds it where it is and brings the leader onto it (not onto the old target)");
        r.followerFollows = true;
        // save and servo_info refused while teleoperating
        PersistentConfig snap; char why[120];
        check(!r.app.prepareSave(snap, why, sizeof(why)) && !r.app.maintenanceAllowed(why, sizeof(why)), "save and servo_info are refused during teleoperation");
        // barrier: move the leader's J2 slowly toward PAR6's soft limit (83.6 deg CAD)
        const double j2 = r.qL[1];
        for (int k = 1; k <= 100; ++k) { r.qL[1] = j2 + (rad(80.0) - j2) * k / 100.0; r.run(0.01); }
        check(r.stage() == TeleopSession::Stage::Coupled && r.app.lastOutput().tauAssist[1] < -0.05f, "near PAR6's J2 limit the barrier pushes the leader back");
        for (int k = 1; k <= 100; ++k) { r.qL[1] = rad(80.0) + (j2 - rad(80.0)) * k / 100.0; r.run(0.01); }
        // frozen robot: same packet over and over
        r.rcb.freezeState = true;
        r.run(0.4);
        check(r.stage() == TeleopSession::Stage::Lost, "a frozen state stream (same tick) times out: LOST");
        r.rcb.freezeState = false;
        r.run(3.0);
        check(r.stage() == TeleopSession::Stage::Lost && r.app.mode() == Mode::Gravity, "after an outage it waits for the operator (no automatic re-claim)");
        s = r.cmd("{\"cmd\": \"teleop_resume\"}");
        check(s.find("\"ok\": true") != std::string::npos && r.runUntil(TeleopSession::Stage::Coupled, 8.0), "teleop_resume realigns and reclaims");
        // stalled control task: the network keeps running, the leader target ages
        r.controlRuns = false;
        r.run(0.05);                                      // the newest target is 50 ms old now
        const uint32_t c0 = r.app.link().commandsSent();
        r.run(0.5);
        check(r.app.link().staleTargets() > 0 && r.app.link().commandsSent() == c0 && r.app.link().phase() != rti::Phase::Active,
              "a stalled control task: targets older than 50 ms are not sent, the claim is dropped");
        std::printf("         link error: \"%s\"\n", r.app.link().lastError());
        r.controlRuns = true;
        r.cmd("{\"cmd\": \"teleop_stop\"}");
    }
    {
        Rig r;
        const double f[6] = {0, 0, 160, 0, 0, 0};          // PAR6's J3 beyond RAFT's 150 deg
        r.setFollowerDeg(f);
        r.qL[2] = rad(140.0);
        r.cmd("{\"cmd\": \"teleop_start\", \"host\": \"par6.local\"}");
        r.run(2.0);
        std::string s = r.cmd("{\"cmd\": \"status\"}");
        check(r.stage() == TeleopSession::Stage::WaitPaired && r.app.mode() != Mode::Move && s.find("J3: PAR6 is at 160.0 deg") != std::string::npos,
              "PAR6 outside RAFT's range: reported, nothing moves");
        r.cmd("{\"cmd\": \"teleop_offset\", \"on\": true}");
        check(r.runUntil(TeleopSession::Stage::Coupled, 6.0), "with an offset allowed it couples");
        check(std::fabs(deg(r.qL[2]) - 147.0) < 1.0 && std::fabs(deg(r.app.session().followerTarget()[2]) - 160.0) < 1.0,
              "leader J3 at 147 deg (3 deg inside its range) drives PAR6's J3 at 160 deg");
        r.cmd("{\"cmd\": \"teleop_stop\"}");
    }
    {
        Rig r;
        const double f[6] = {0, 0, 0, 0, 0, 200};          // PAR6's J6 at +200 deg (its range is +-229)
        r.setFollowerDeg(f);
        r.qL[5] = rad(-150.0);
        r.cmd("{\"cmd\": \"teleop_start\", \"host\": \"par6.local\"}");
        check(r.runUntil(TeleopSession::Stage::Coupled, 6.0), "PAR6's J6 at 200 deg: couples");
        check(std::fabs(deg(r.qL[5]) + 160.0) < 1.0 && std::fabs(deg(r.app.session().followerTarget()[5]) - 200.0) < 1.0,
              "RAFT's J6 at -160 deg (the same wrist roll) drives PAR6 at +200 deg, not -160");
        r.cmd("{\"cmd\": \"teleop_stop\"}");
    }
    {
        Rig r;
        r.cmd("{\"cmd\": \"teleop_settings\", \"auto_reclaim\": true}");
        const double f[6] = {5, 0, 0, 0, 0, 0};
        r.setFollowerDeg(f);
        r.cmd("{\"cmd\": \"teleop_start\", \"host\": \"par6.local\"}");
        r.runUntil(TeleopSession::Stage::Coupled, 5.0);
        r.rcb.streamMuted = true; r.run(0.3); r.rcb.streamMuted = false;
        check(r.runUntil(TeleopSession::Stage::Coupled, 8.0), "with auto_reclaim on, the session reclaims by itself after an outage");
        std::string s = r.cmd("{\"cmd\": \"teleop_settings\", \"require_enable\": true}");
        check(s.find("\"ok\": false") != std::string::npos, "require_enable is refused without an enable button");
        r.cmd("{\"cmd\": \"teleop_stop\"}");
    }
}

// calib_stop: every joint held at its mechanical stop, against servos with known zeros and the design's directions
// (J1 and the trigger reversed). Afterwards the standby pose must read 0 on every joint.
static void testStopCalibration()
{
    std::printf("=== Stop calibration ===\n");
    RamStorage st; simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    App app(st, rcb);
    app.begin(0.0, true);
    const int trueZero[kNumJoints] = {2970, 2544, 606, 1027, 4000, 100, 1155};
    int trueDir[kNumJoints];
    for (int i = 0; i < kNumJoints; ++i) trueDir[i] = kJointDir[i];
    const double kTick = 2.0 * 3.14159265358979 / 4096.0;
    auto rawAt = [&](int j, double qRad) {      // the servo reading for joint angle q (CAD)
        int r = trueZero[j] + (int)std::lround(trueDir[j] * qRad / kTick);
        r %= 4096; if (r < 0) r += 4096;
        return (int16_t)r;
    };
    RawServoState raw = {};
    for (int i = 0; i < kNumJoints; ++i) { raw.ok[i] = true; raw.pos[i] = rawAt(i, 0.0); raw.tempC[i] = 30; }
    int16_t out[kNumJoints]; bool on, need;
    char rep[4096];
    double t = 0;
    auto tick = [&](int n) { for (int k = 0; k < n; ++k) { t += 0.005; app.controlTick(raw, t, out, on, need); } };
    tick(2);
    bool allOk = true;
    for (int j = 0; j < kNumJoints; ++j) {
        const double stop = kStopDeg[j] * 3.14159265358979 / 180.0;
        raw.pos[j] = rawAt(j, stop);
        char c[64]; std::snprintf(c, sizeof(c), "{\"cmd\": \"calib_stop\", \"joint\": %d}", j + 1);
        app.handleCommand(c, rep, sizeof(rep), t, nullptr);
        if (!std::strstr(rep, "\"ok\": true")) { std::printf("  J%d: %s\n", j + 1, rep); allOk = false; continue; }
        tick(70);                                // 0.35 s held at the stop
        raw.pos[j] = rawAt(j, 0.0);
        const JointCal& jc = app.config().joint[j];
        int dz = std::abs(jc.zeroTicks - trueZero[j]); if (dz > 2048) dz = 4096 - dz;
        if (jc.dir != trueDir[j] || dz > 1 || !(jc.flags & CAL_ZERO)) {
            std::printf("  J%d: dir %d (true %d), zero %d (true %d)\n", j + 1, jc.dir, trueDir[j], jc.zeroTicks, trueZero[j]);
            allOk = false;
        }
    }
    check(allOk && fullyCalibrated(app.config()), "calib_stop finds every zero from the stops (J1 and the trigger reversed)");
    tick(4);
    app.handleCommand("{\"cmd\": \"status\"}", rep, sizeof(rep), t, nullptr);
    double q[kNumJoints] = {};
    const int nq = json::getNumberArray(rep, "q_deg", q, kNumJoints);
    bool zeroAtStandby = nq == kNumJoints;
    for (int i = 0; i < nq; ++i) if (std::fabs(q[i]) > 0.1) zeroAtStandby = false;
    check(zeroAtStandby, "the standby pose then reads 0 on every joint");
    check(std::strstr(rep, "\"stop_deg\": [-136.28, -83.96, 92.81") != nullptr, "status lists the stop angles");

    // refusals: not limp, a stop given beyond half a turn, a joint that is not held still
    app.handleCommand("{\"cmd\": \"calib_stop\", \"joint\": 2, \"deg\": 200}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") != nullptr, "a stop angle beyond half a turn is refused");
    app.handleCommand("{\"cmd\": \"calib_stop\", \"joint\": 2}", rep, sizeof(rep), t, nullptr);
    for (int k = 0; k < 70; ++k) { raw.pos[1] = (int16_t)(rawAt(1, 0.0) + (k % 2 ? 15 : 0)); tick(1); }
    raw.pos[1] = rawAt(1, 0.0);
    tick(2);
    app.handleCommand("{\"cmd\": \"calib_result\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "hold it still") != nullptr, "a joint that shakes while sampling is refused");
}

// Servo position test: what the App hands the control task (start, goals, stop) and when it refuses.
static void testServoTest()
{
    std::printf("=== Servo position test ===\n");
    RamStorage st; simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    App app(st, rcb);
    app.begin(0.0, true);
    RawServoState raw = {};
    for (int i = 0; i < kNumJoints; ++i) { raw.ok[i] = true; raw.pos[i] = 2048; raw.tempC[i] = 30; }
    raw.pos[4] = 1500;
    int16_t out[kNumJoints]; bool on, need;
    char rep[4096];
    double t = 0;
    auto tick = [&](int n) { for (int k = 0; k < n; ++k) { t += 0.005; app.controlTick(raw, t, out, on, need); } };
    tick(2);
    App::ServoTestOp op;
    app.handleCommand("{\"cmd\": \"servo_test\", \"joint\": 5, \"torque_ma\": 650, \"speed_deg\": 30}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": true") && app.takeServoTestOp(op) && op.step == App::kServoTestStart && op.joint == 4 && op.goalRaw == 1500
          && op.torque == 200 && op.speed == 7 && !app.takeServoTestOp(op), "start: J5 holds where it is (650 mA = 200 units, 30 deg/s = 7 units), once");
    app.handleCommand("{\"cmd\": \"servo_jog\", \"deg\": 10}", rep, sizeof(rep), t, nullptr);
    const int expect = 1500 + kJointDir[4] * (int)std::lround(10.0 * 4096.0 / 360.0);
    check(app.takeServoTestOp(op) && op.step == App::kServoTestGoal && op.goalRaw == expect, "jog +10 deg: a new goal from the start position");
    app.handleCommand("{\"cmd\": \"servo_jog\", \"deg\": -180}", rep, sizeof(rep), t, nullptr);
    check(app.takeServoTestOp(op) && (op.goalRaw == 0 || op.goalRaw == 4095) && std::strstr(rep, "clamped"), "a goal past the encoder's end is clamped, not wrapped");
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && app.mode() == Mode::Limp, "no torque mode while a servo test runs");
    app.handleCommand("{\"cmd\": \"status\"}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"servo_test\": {\"joint\": 5") != nullptr, "status shows the test");
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"limp\"}", rep, sizeof(rep), t, nullptr);
    check(app.takeServoTestOp(op) && op.step == App::kServoTestStop && op.joint == 4 && app.servoTestJoint() < 0, "LIMP (Esc) stops it: torque off, back to current mode");
    app.handleCommand("{\"cmd\": \"servo_test\", \"joint\": 3}", rep, sizeof(rep), t, nullptr);
    raw.ok[3] = false; tick(50); raw.ok[3] = true;
    check(app.takeServoTestOp(op) && op.step == App::kServoTestStop && op.joint == 2, "a fault (50 missed reads) stops it too");
    app.handleCommand("{\"cmd\": \"clear_fault\"}", rep, sizeof(rep), t, nullptr);
    app.handleCommand("{\"cmd\": \"calib_zero\"}", rep, sizeof(rep), t, nullptr); tick(120);
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr); tick(2);
    app.handleCommand("{\"cmd\": \"servo_test\", \"joint\": 2}", rep, sizeof(rep), t, nullptr);
    check(std::strstr(rep, "\"ok\": false") && !app.takeServoTestOp(op), "refused unless the arm is limp");
}

// Hold in the servos' own position mode: which joints get goals, and the goal values.
static void testHoldPosition()
{
    std::printf("=== Hold in the servos' position mode ===\n");
    RamStorage st; simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    App app(st, rcb);
    app.begin(0.0, true);
    RawServoState raw = {};
    for (int i = 0; i < kNumJoints; ++i) { raw.ok[i] = true; raw.pos[i] = 2048; raw.tempC[i] = 30; }
    raw.pos[2] = 40;                 // J3's zero right next to the encoder's 0/4095 wrap
    int16_t out[kNumJoints]; bool on, need;
    char rep[4096];
    double t = 0;
    // ideal position servos: every joint in position mode is where its goal says
    auto tick = [&](int n) {
        for (int k = 0; k < n; ++k) {
            t += 0.005;
            app.controlTick(raw, t, out, on, need);
            const App::ServoGoals& g = app.servoGoals();
            for (int i = 0; i < kNumJoints; ++i) if (g.mask & (1 << i)) raw.pos[i] = g.raw[i];
        }
    };
    tick(2);
    app.handleCommand("{\"cmd\": \"calib_zero\"}", rep, sizeof(rep), t, nullptr); tick(120);
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"gravity\"}", rep, sizeof(rep), t, nullptr); tick(4);
    check(on && app.servoGoals().mask == 0, "Gravity: current mode, no position goals");
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"move\"}", rep, sizeof(rep), t, nullptr); tick(4);
    const App::ServoGoals& g = app.servoGoals();
    check(g.mask == (0x7f & ~(1 << 2)) && g.raw[0] == 2048 && g.raw[4] == 2048 && g.limit == 400,
          "Hold: position mode where it is, at the stall current (400 units); J3 next to the wrap keeps the computed Hold");
    app.handleCommand("{\"cmd\": \"move\", \"q_deg\": [0, 0, 0, 0, 20, 0, 0], \"speed_deg\": 40}", rep, sizeof(rep), t, nullptr);
    tick(60);
    const int mid = app.servoGoals().raw[4];
    tick(400);
    const int end = app.servoGoals().raw[4];
    const int expectEnd = 2048 + kJointDir[4] * (int)std::lround(20.0 * 4096.0 / 360.0);
    check(app.mode() == Mode::Move && (app.servoGoals().mask & (1 << 4)), "a move runs in position mode too");
    check((mid - 2048) * (expectEnd - 2048) > 0 && std::abs(mid - 2048) < std::abs(expectEnd - 2048) && std::abs(end - expectEnd) <= 1,
          "its goal travels to the target (J5 +20 deg)");
    {   // a missed read keeps the servo in position mode on its goal: no mode switch, no torque blip
        const int goal = app.servoGoals().raw[5];
        raw.ok[5] = false; tick(3);
        const bool kept = (app.servoGoals().mask & (1 << 5)) && app.servoGoals().raw[5] == goal;
        raw.ok[5] = true; tick(1);
        check(kept && (app.servoGoals().mask & (1 << 5)), "J6 misses 3 reads: it stays in position mode on the same goal");
    }
    app.handleCommand("{\"cmd\": \"hold_position\", \"on\": false}", rep, sizeof(rep), t, nullptr); tick(1);
    check(app.servoGoals().mask == 0, "hold_position off: the computed Hold everywhere");
    app.handleCommand("{\"cmd\": \"hold_position\", \"on\": true}", rep, sizeof(rep), t, nullptr);
    app.handleCommand("{\"cmd\": \"mode\", \"mode\": \"limp\"}", rep, sizeof(rep), t, nullptr); tick(1);
    check(!on && app.servoGoals().mask == 0, "limp: no goals");
}

// The control loop figures of the Health page: one-second windows of tick timing.
static void testLoopStats()
{
    std::printf("=== Loop statistics ===\n");
    raft::LoopStats st(5000);
    uint32_t t = 4294000000u;                  // close to the 32-bit wrap of micros()
    bool closed = false;
    for (int k = 0; k < 300 && !closed; ++k) {
        const uint32_t period = k == 100 ? 7000u : (k == 101 ? 3000u : 5000u);   // one late tick, then it catches up
        if (k) t += period;
        st.wake(t, k == 101);
        if (k == 50) st.failedRead();
        st.part(raft::kPartRead, 3700);
        st.part(raft::kPartApp, k == 100 ? 900 : 300);
        closed = st.done(t + 4100);
    }
    const raft::LoopReport& r = st.report();
    check(closed && r.seq == 1 && r.ticks == 201, "a window closes after one second of ticks");
    check(std::fabs(r.hz - 200.0f) < 1.5f, "about 200 ticks per second");
    check(r.periodMinUs == 3000 && r.periodMaxUs == 7000 && std::fabs(r.periodMeanUs - 5000.0f) < 1.0f, "period min, mean and max");
    check(std::fabs(r.jitterRmsUs - (float)std::sqrt(2.0 * 2000.0 * 2000.0 / 200.0)) < 1.0f, "jitter: RMS deviation from 5 ms");
    check(r.partMaxUs[raft::kPartApp] == 900 && std::fabs(r.partMeanUs[raft::kPartRead] - 3700.0f) < 0.5f && r.workMaxUs == 4100, "the parts of a tick: mean and max");
    check(r.overruns == 1 && r.failedReads == 1 && r.overrunsTotal == 1, "overruns and failed reads");
    st.skipPeriod();
    t += 50000;                                // a pause (servo_info) is not a period
    st.wake(t, false);
    st.done(t + 4000);
    for (int k = 0; k < 200; ++k) { t += 5000; st.wake(t, k == 3); closed = st.done(t + 4000); if (closed) break; }
    check(closed && st.report().seq == 2 && st.report().periodMaxUs == 5000 && st.report().overrunsTotal == 2, "the next window: the pause left out, totals carried on");
    char js[512];
    const size_t n = raft::formatLoopReport(st.report(), js, sizeof(js));
    std::string body = "{" + std::string(js, n) + "}";
    double v = 0, arr[2] = {0, 0};
    check(n > 0 && raft::json::getNumber(body.c_str(), "hz", v) && v > 190 && raft::json::getNumberArray(body.c_str(), "overruns", arr, 2) == 2 && arr[1] == 2,
          "the status's loop object");
}

// PAR6's two streaming contracts and the target filter.
static void testRealRti()
{
    std::printf("=== Real RTI and the target filter (emulated PAR6) ===\n");
    const std::string::size_type npos = std::string::npos;
    {
        Rig r;
        check(r.cmd("{\"cmd\": \"teleop_rate\", \"hz\": 100}").find("\"ok\": false") != npos, "100 Hz is neither contract: refused");
        check(r.cmd("{\"cmd\": \"teleop_rate\", \"hz\": 250}").find("\"link_mode\": \"RTI\"") != npos, "250 Hz selects Real RTI");
        const double f[6] = {10, 5, 20, 0, -15, 0};
        r.setFollowerDeg(f);
        r.cmd("{\"cmd\": \"teleop_start\", \"host\": \"par6.local\"}");
        check(r.cmd("{\"cmd\": \"teleop_rate\", \"hz\": 50}").find("stop teleoperation first") != npos, "the rate cannot change while teleoperating");
        check(r.runUntil(TeleopSession::Stage::Coupled, 5.0) && r.rcb.mode() == "RTI", "pairs in Real RTI (no rate in rti_connect) and couples");
        const unsigned c0 = r.rcb.commandsReceived();
        const uint32_t s0 = r.app.link().stateCount();
        r.run(1.0);
        const unsigned cmds = r.rcb.commandsReceived() - c0;
        const uint32_t states = r.app.link().stateCount() - s0;
        char what[160];
        std::snprintf(what, sizeof(what), "one command per PAR6 state packet: %u commands for %u state packets in 1 s", cmds, (unsigned)states);
        check(states > 240 && cmds + 2 >= states && cmds <= states + 1, what);
        check(r.stage() == TeleopSession::Stage::Coupled, "stays coupled at 250 Hz");
        r.cmd("{\"cmd\": \"teleop_stop\"}");
        r.run(0.2);
        check(r.cmd("{\"cmd\": \"teleop_rate\", \"hz\": 25}").find("\"link_mode\": \"SOFT_RTI\"") != npos, "back to SOFT_RTI 25 Hz once stopped");
    }
    {
        // the filter: a 20 deg step of the leader's J1 reaches the target smoothly and a few ms later
        Rig r;
        r.cmd("{\"cmd\": \"teleop_rate\", \"hz\": 250}");
        check(r.cmd("{\"cmd\": \"teleop_filter\", \"hz\": 25}").find("\"teleop_filter_hz\": 25.0") != npos, "teleop_filter 25 Hz");
        r.cmd("{\"cmd\": \"teleop_start\", \"host\": \"par6.local\"}");
        check(r.runUntil(TeleopSession::Stage::Coupled, 5.0), "couples with the filter on");
        r.run(0.3);
        double q[6], qd[6];
        r.rcb.target(q, qd);
        const double before = q[0];
        r.qL[0] += rad(2.0);                       // a 2 deg step of the leader (below the jump guard)
        r.run(0.004);
        r.rcb.target(q, qd);
        const double early = q[0] - before;
        r.run(0.1);
        r.rcb.target(q, qd);
        const double late = q[0] - before;
        char what[160];
        std::snprintf(what, sizeof(what), "a 2 deg step: %.2f deg after one tick, %.2f deg after 0.1 s", deg(early), deg(late));
        check(std::fabs(deg(early)) < 1.0 && std::fabs(deg(late) - 2.0) < 0.05, what);
    }
}

// The web server prints the status for a page only when App::statusKey changes: live values must not change it,
// settings, modes and texts must.
static void testStatusKey()
{
    std::printf("=== Status key (status sent on change) ===\n");
    Rig r;
    App::StatusSnapshot s;
    auto key = [&]() { r.app.snapshot(s); return App::statusKey(s); };
    r.run(0.2);
    const uint32_t k0 = key();
    for (int i = 0; i < 7; ++i) { r.qL[i] = rad(5.0 * (i + 1)); r.raw.tempC[i] = 41; }
    r.run(0.3);
    r.app.snapshot(s);
    check(App::statusKey(s) == k0 && std::fabs(s.q[0] - rad(5.0)) < 0.01, "angles and temperatures move: the key stays");
    r.raw.ok[5] = false;
    r.run(0.012);                                   // a few missed reads of J6, not yet a fault
    r.raw.ok[5] = true;
    r.run(0.1);
    r.app.snapshot(s);
    check(App::statusKey(s) == k0 && s.missed[5] > 0, "missed reads count up: the key stays");
    r.cmd("{\"cmd\": \"gravity_scale\", \"value\": 0.8}");
    const uint32_t k1 = key();
    check(k1 != k0, "a setting changes it");
    r.cmd("{\"cmd\": \"mode\", \"mode\": \"gravity\"}");
    r.run(0.1);
    const uint32_t k2 = key();
    check(k2 != k1, "a mode change changes it");
    r.run(0.5);
    check(key() == k2, "and it stays while nothing else changes");
}

static void testArmName()
{
    std::printf("=== Arm name (set_name) ===\n");
    RamStorage st; simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    App app(st, rcb);
    app.begin(0.0, true);
    char rep[4096];
    app.handleCommand("{\"cmd\": \"set_name\", \"name\": \"Left\"}", rep, sizeof(rep), 0.0, nullptr);
    check(std::strstr(rep, "\"ok\": true") && std::strstr(rep, "\"host\": \"raft-left.local\"") && !std::strcmp(app.config().armName, "left"),
          "set_name Left: stored as left, the reply names raft-left.local");
    app.handleCommand("{\"cmd\": \"status\"}", rep, sizeof(rep), 0.0, nullptr);
    check(std::strstr(rep, "\"name\": \"left\"") && std::strstr(rep, "\"unsaved\": true"), "the status carries the name and shows it unsaved");
    app.handleCommand("{\"cmd\": \"set_name\", \"name\": \"two words\"}", rep, sizeof(rep), 0.0, nullptr);
    check(std::strstr(rep, "\"ok\": false") && !std::strcmp(app.config().armName, "left"), "a name with a space is refused, the old one stays");
    app.handleCommand("{\"cmd\": \"set_name\"}", rep, sizeof(rep), 0.0, nullptr);
    check(std::strstr(rep, "\"ok\": false") != nullptr, "set_name without a name is refused");
    app.handleCommand("{\"cmd\": \"defaults\"}", rep, sizeof(rep), 0.0, nullptr);
    check(std::strstr(rep, "\"ok\": true") && !std::strcmp(app.config().armName, "left"), "factory defaults keep the name");
    app.handleCommand("{\"cmd\": \"save\"}", rep, sizeof(rep), 0.0, nullptr);
    App again(st, rcb);
    again.begin(0.0, true);
    check(!std::strcmp(again.config().armName, "left"), "saved and loaded again after a restart");
    again.handleCommand("{\"cmd\": \"set_name\", \"name\": \"\"}", rep, sizeof(rep), 0.0, nullptr);
    check(std::strstr(rep, "\"host\": \"raft-leader.local\"") && again.config().armName[0] == '\0', "an empty name removes it: raft-leader again");
}

int main()
{
    testBlobAndParsers();
    testArmName();
    testStatusKey();
    testCommandFlow();
    testStopCalibration();
    testServoTest();
    testHoldPosition();
    testLoopStats();
    testRangeAndReplay();
    testTeleop();
    testRealRti();
    std::printf("%s\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return g_fail ? 1 : 0;
}
