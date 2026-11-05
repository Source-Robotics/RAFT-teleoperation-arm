// Host unit test of the firmware's RCB link code (src/main/rti_protocol.cpp, rti_link.cpp) against
// golden bytes from the RCB runtime's own Python packer and an emulated RCB runtime (sim_rcb.h).
#include <cmath>
#include <cstdio>
#include <cstring>

#include "rti_link.h"
#include "rti_protocol.h"
#include "sim_rcb.h"

using namespace raft::rti;

static int g_fail = 0;
static void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

// communication/rti_protocol.py pack_command_packet(7, 1, [.1,-.2,.3,-.4,.5,-.6], [.01..06], timestamp=12.5)
static const uint8_t kGolden109[] = {0x07,0x00,0x00,0x00,0x01,0x9a,0x99,0x99,0x99,0x99,0x99,0xb9,0x3f,0x9a,0x99,0x99,0x99,0x99,0x99,0xc9,0xbf,0x33,0x33,0x33,0x33,0x33,0x33,0xd3,0x3f,0x9a,0x99,0x99,0x99,0x99,0x99,0xd9,0xbf,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x3f,0x33,0x33,0x33,0x33,0x33,0x33,0xe3,0xbf,0x7b,0x14,0xae,0x47,0xe1,0x7a,0x84,0x3f,0x7b,0x14,0xae,0x47,0xe1,0x7a,0x94,0x3f,0xb8,0x1e,0x85,0xeb,0x51,0xb8,0x9e,0x3f,0x7b,0x14,0xae,0x47,0xe1,0x7a,0xa4,0x3f,0x9a,0x99,0x99,0x99,0x99,0x99,0xa9,0x3f,0xb8,0x1e,0x85,0xeb,0x51,0xb8,0xae,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x29,0x40};
// pack_command_packet(4294967295, 3, [1..6], [0]*6)
static const uint8_t kGolden101Head[] = {0xff,0xff,0xff,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40};

static void testProtocol()
{
    std::printf("=== Wire formats vs the RCB reference ===\n");
    uint8_t buf[200];
    const double p[6] = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6}, ff[6] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
    size_t n = packCommand(buf, sizeof(buf), 7, CMD_JOINT_POSITION, p, ff, true, 12.5);
    check(n == 109 && std::memcmp(buf, kGolden109, 109) == 0, "109-byte command packet identical to rti_protocol.py");
    const double p2[6] = {1, 2, 3, 4, 5, 6}, z[6] = {0, 0, 0, 0, 0, 0};
    n = packCommand(buf, sizeof(buf), 4294967295u, CMD_JOINT_TORQUE, p2, z, false, 0);
    check(n == 101 && std::memcmp(buf, kGolden101Head, sizeof(kGolden101Head)) == 0, "101-byte packet (no send time) identical");
    uint32_t seq; uint8_t type; double q[6], f[6], t; bool hasT;
    packCommand(buf, sizeof(buf), 7, CMD_JOINT_POSITION, p, ff, true, 12.5);
    check(unpackCommand(buf, 109, &seq, &type, q, f, &hasT, &t) && seq == 7 && type == 1 && hasT && t == 12.5 && q[5] == -0.6 && f[0] == 0.01,
          "unpack round trip");
    check(packCommand(buf, 100, 1, 1, p, ff, false, 0) == 0, "too-small buffer is refused");

    char line[200];
    buildRtiConnect(line, sizeof(line), "rf0001", "192.168.1.50", 50.0f, "raft");
    check(std::strcmp(line, "{\"id\": \"rf0001\", \"cmd\": \"rti_connect\", \"args\": [\"192.168.1.50\", 50.0, \"udp\"], \"client\": \"raft\"}\n") == 0,
          "rti_connect (SOFT_RTI) line identical to the RCB client's json.dumps");
    buildRtiConnect(line, sizeof(line), "rf0002", "192.168.1.50", 0.0f, "raft");
    check(std::strcmp(line, "{\"id\": \"rf0002\", \"cmd\": \"rti_connect\", \"args\": [\"192.168.1.50\", \"\", \"udp\"], \"client\": \"raft\"}\n") == 0,
          "rti_connect (hard RTI, empty rate) identical");
    buildNoArgs(line, sizeof(line), "rf0003", "rti_claim", "raft");
    check(std::strcmp(line, "{\"id\": \"rf0003\", \"cmd\": \"rti_claim\", \"args\": [], \"client\": \"raft\"}\n") == 0, "rti_claim identical");
    buildSetRecipe(line, sizeof(line), "rf0004", "raft", "raft");
    check(std::strcmp(line, "{\"id\": \"rf0004\", \"cmd\": \"set_recipe\", \"args\": [\"raft\"], \"client\": \"raft\"}\n") == 0, "set_recipe identical");
    buildGripperSet(line, sizeof(line), "rf0005", 128, 200, 400, 1, "raft");
    check(std::strcmp(line, "{\"id\": \"rf0005\", \"cmd\": \"gripper_set\", \"args\": [128, 200, 400, 1], \"client\": \"raft\"}\n") == 0, "gripper_set identical");

    // docs/udp_telemetry_format.md worked example (standard recipe)
    const char* ex = "j_p:0.000,0.523,-1.234,0.000,1.571,-0.785,j_s:0.00,0.01,-0.02,0.00,0.00,0.00,j_err:0,0,0,0,0,0,tick:42301,TCP_p:0.4523,0.1200,0.3450,3.1416,0.0000,0.0000";
    State s;
    check(parseState(ex, std::strlen(ex), s) && (s.present & F_JP) && (s.present & F_JS) && (s.present & F_TICK) && !(s.present & F_MODE)
          && s.jp[2] == -1.234 && s.js[2] == -0.02 && s.tick == 42301, "parses the spec's worked example, skips unknown tags");
    const char* full = "j_p:0.1,0.2,0.3,0.4,0.5,0.6,t_ext:-0.5,0,0,0,0,1.25,g_p:0.012,g_c:250.5,rti_seq:99,rti_state:2,mode:SOFT_RTI";
    check(parseState(full, std::strlen(full), s) && s.text[0] == -0.5 && s.text[5] == 1.25 && s.gp == 0.012 && s.gc == 250.5
          && s.rtiSeq == 99 && s.rtiState == 2 && std::strcmp(s.mode, "SOFT_RTI") == 0, "parses the raft recipe tags");
    const char* bad1 = "j_p:0.1,0.2,tick:5";
    const char* bad2 = "j_p:0.1,x,0.3,0.4,0.5,0.6";
    check(!parseState(bad1, std::strlen(bad1), s) && !parseState(bad2, std::strlen(bad2), s), "rejects a short j_p and a non-number");
    const char* inf = "j_p:inf,0,0,0,0,0,tick:5";
    const char* huge = "j_p:1e999,0,0,0,0,0,tick:5";
    const char* tick = "j_p:0,0,0,0,0,0,tick:1e20";
    const char* st = "j_p:0,0,0,0,0,0,rti_state:2.5";
    check(!parseState(inf, std::strlen(inf), s) && !parseState(huge, std::strlen(huge), s) && !parseState(tick, std::strlen(tick), s)
          && !parseState(st, std::strlen(st), s), "rejects inf / overflow in j_p, a tick beyond 32 bits, a fractional rti_state");
    const char* opt = "j_p:0.1,0.2,0.3,0.4,0.5,0.6,t_ext:nan,0,0,0,0,0,g_p:-99999999,g_c:12,tick:7";
    check(parseState(opt, std::strlen(opt), s) && (s.present & F_JP) && !(s.present & F_TEXT) && !(s.present & F_GP) && (s.present & F_GC)
          && (s.dropped & F_TEXT) && (s.dropped & F_GP) && s.tick == 7, "a NaN t_ext and the g_p sentinel are dropped, the joint state is kept");
    const char* gp = "j_p:0,0,0,0,0,0,g_p:128.0";
    check(parseState(gp, std::strlen(gp), s) && (s.present & F_GP) && s.gp == 128.0, "g_p on its raw 0..255 scale");

    Reply r;
    const char* ok = "{\"id\": \"rf0007\", \"status\": \"ok\", \"data\": null}";
    const char* err = "{\"id\": \"rf0008\", \"status\": \"error\", \"code\": \"BAD_ARGS\", \"message\": \"x\"}";
    const char* forb = "{\"type\": \"error\", \"code\": \"FORBIDDEN\", \"message\": \"not on the list\"}";
    check(parseReply(ok, std::strlen(ok), r) && r.hasId && std::strcmp(r.id, "rf0007") == 0 && r.status == Reply::OK, "reply ok");
    check(parseReply(err, std::strlen(err), r) && r.status == Reply::ERROR && std::strcmp(r.code, "BAD_ARGS") == 0, "reply error with code");
    check(parseReply(forb, std::strlen(forb), r) && !r.hasId && std::strcmp(r.code, "FORBIDDEN") == 0, "access-list refusal event");
}

// Run the link against the emulated runtime at 1 kHz; the robot loop ticks at 250 Hz.
struct Rig {
    simrcb::FakeRcb rcb;
    Link link;
    double t = 0.0;
    double meas[6] = {0.1, -1.4, 3.0, 0.2, -0.3, 3.1};   // follower pose, DH
    bool pollLeader = true;
    explicit Rig(const simrcb::NetConfig& n, const LinkConfig& c) : rcb(n), link(rcb, c) {}
    void run(double seconds, bool aligned = true)
    {
        const long steps = std::lround(seconds * 1000.0);
        for (long k = 0; k < steps; ++k) {
            t += 0.001;
            rcb.setTime(t);
            if (k % 4 == 0) {
                double q[6], qd[6], z[6] = {0, 0, 0, 0, 0, 0};
                if (rcb.target(q, qd)) for (int i = 0; i < 6; ++i) meas[i] = q[i];   // ideal follower
                (void)qd;
                rcb.robotTick(meas, z, z, 0.0, 0.0);
            }
            if (pollLeader) {
                double tgt[6], v[6] = {0, 0, 0, 0, 0, 0};
                for (int i = 0; i < 6; ++i) tgt[i] = meas[i] + (aligned ? 0.0 : 0.2);
                link.setTarget(tgt, v, t);
                link.poll(t);
            }
        }
    }
};

static void testLink()
{
    std::printf("=== Link state machine vs the emulated RCB runtime ===\n");
    simrcb::NetConfig net;
    LinkConfig cfg = defaultLinkConfig();
    {
        Rig rig(net, cfg);
        rig.link.start(0.0);
        rig.run(0.2);
        check(rig.link.phase() == Phase::Paired, "connects, selects the recipe, pairs in SOFT_RTI and stops at PAIRED");
        check(rig.rcb.rtiState() == 1 && rig.rcb.mode() == "SOFT_RTI", "robot reports rti_state 1, mode SOFT_RTI (not claimed)");
        rig.link.claim(rig.t);
        rig.run(0.1);
        check(rig.link.phase() == Phase::Active && rig.rcb.rtiState() == 2, "claim -> ACTIVE, robot rti_state 2");
        const unsigned c0 = rig.rcb.commandsReceived();
        rig.run(1.0);
        const unsigned perSec = rig.rcb.commandsReceived() - c0;
        char msg[96];
        std::snprintf(msg, sizeof(msg), "streams joint targets at the link rate (%u per second, expect ~50)", perSec);
        check(perSec >= 48 && perSec <= 51, msg);
        check(rig.link.state().rtiSeq + 3 >= rig.link.commandsSent(), "robot echoes the latest command seq (rti_seq)");
        rig.link.setGripper(128, 200, 400);
        rig.run(0.2);
        check(rig.rcb.gripperRaw() == 128 && rig.rcb.gripperCommands() == 1, "gripper_set sent once on change");
        for (int k = 0; k < 40; ++k) { rig.link.setGripper(k * 5, 200, 400); rig.run(0.005); }
        std::snprintf(msg, sizeof(msg), "gripper commands rate-limited (%d in 0.2 s, limit 15/s)", rig.rcb.gripperCommands() - 1);
        check(rig.rcb.gripperCommands() - 1 <= 4, msg);

        rig.rcb.streamMuted = true;
        rig.run(0.3);
        check(rig.link.phase() == Phase::Backoff || rig.link.phase() == Phase::Connecting || rig.link.phase() == Phase::Pairing,
              "state stream lost -> leaves ACTIVE");
        rig.rcb.streamMuted = false;
        rig.run(2.0);
        check(rig.link.phase() == Phase::Paired, "reconnects by itself, but only up to PAIRED (never re-claims)");

        rig.pollLeader = false;                         // leader stalls while in control
        rig.link.claim(rig.t);
        rig.pollLeader = true; rig.run(0.1);
        rig.pollLeader = false; rig.run(0.2);
        check(rig.rcb.rtiState() == 4, "robot watchdog stops a silent leader (rti_state 4)");
        rig.pollLeader = true; rig.run(0.05);
        check(rig.link.phase() != Phase::Active, "leader notices the revoked claim");
    }
    {
        Rig rig(net, cfg);
        rig.link.start(0.0);
        rig.run(0.2);
        rig.link.claim(rig.t);
        rig.run(0.2, false);                             // leader 11 deg away from the follower
        check(rig.rcb.lastEvent() == "START_POSE" && rig.link.phase() != Phase::Active, "robot's start-pose check rejects a misaligned leader");
    }
    {
        Rig rig(net, cfg);
        rig.rcb.forbidden = true;
        rig.link.start(0.0);
        rig.run(0.1);
        check(std::strstr(rig.link.lastError(), "FORBIDDEN") != nullptr && rig.link.phase() != Phase::Paired, "access-list refusal is reported");
        std::printf("         error text: \"%s\"\n", rig.link.lastError());
    }
    {
        Rig rig(net, cfg);
        rig.rcb.recipe = "full";                         // raft.xml not installed on the robot
        rig.link.start(0.0);
        rig.run(0.1);
        check(std::strstr(rig.link.lastError(), "BAD_ARGS") != nullptr, "missing recipe on the robot is reported");
        std::printf("         error text: \"%s\"\n", rig.link.lastError());
    }
    {
        simrcb::NetConfig lossy; lossy.latencyS = 0.010; lossy.jitterS = 0.003; lossy.udpLoss = 0.05;
        Rig rig(lossy, cfg);
        rig.link.start(0.0);
        rig.run(0.3);
        rig.link.claim(rig.t);
        rig.run(3.0);
        check(rig.link.phase() == Phase::Active, "stays ACTIVE with 10 ms latency, 3 ms jitter and 5 % UDP loss");
    }
}

int main()
{
    testProtocol();
    testLink();
    std::printf("%s\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return g_fail ? 1 : 0;
}
