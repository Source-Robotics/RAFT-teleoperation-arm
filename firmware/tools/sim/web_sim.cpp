// RAFT web UI simulator (dev tool, Windows; nothing here runs on the arm). The firmware App and the web UI
// server run on this PC against the MuJoCo arm, so the page can be tried without hardware:
//   web_sim.exe [--port 8080] [--bind 127.0.0.1] [--calibrated] [--fresh] [--spring-k N] [--friction Nm] [--scene file]
//   then open http://localhost:8080/
// The simulated servos have unknown zeros and torque constants like real ones (as in calib_sim.cpp), and the
// design's directions (arm_design.h),
// so the calibration wizard works end to end. A simulated hand stands in for the person at the bench:
//   - it holds the arm whenever the arm is limp (catches it when torque goes off) and lets go when torque comes on
//   - during calib_zero it puts the arm in the zero pose
//   - the page's simulator card and Shift-drag send {"cmd": "sim_hand", "mode": "hold"|"zero"|"free"|"pull", "p": [x,y,z]}
// The PAR6 is the emulated RCB runtime (sim_rcb.h) with a follower that tracks the leader's commands, so the
// Teleop works with any host name. Settings persist in out/web_sim_settings.bin (--fresh starts over,
// --calibrated starts with zero and signs already measured).
#include "arm_design.h"
#include "win_web.h"

#include <mmsystem.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "app.h"
#include "config_store.h"
#include "json_lite.h"
#include "loop_stats.h"
#include "model_selftest.h"
#include "sim_core.h"
#include "sim_rcb.h"
#include "version.h"
#include "web_assets.h"
#include "servo_info.h"
#include "web_bridge.h"

using namespace sim;

namespace {

const int    kZero[7] = {1517, 2903, 310, 3790, 2421, 1006, 2640};
const int8_t (&kDir)[7] = raft::kJointDir;   // the servos sit as the design says
const double kKtScale[7] = {1.0, 0.90, 1.10, 1.0, 1.0, 1.0, 1.0};

double wall()
{
    static LARGE_INTEGER f = {}, t0 = {};
    if (!f.QuadPart) { QueryPerformanceFrequency(&f); QueryPerformanceCounter(&t0); }
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return (double)(t.QuadPart - t0.QuadPart) / (double)f.QuadPart;
}

struct FileStorage : raft::Storage {
    std::string path;
    bool load(raft::PersistentConfig& c) override
    {
        FILE* fp = std::fopen(path.c_str(), "rb");
        if (!fp) return false;
        uint8_t buf[512];
        const size_t n = std::fread(buf, 1, sizeof(buf), fp);
        std::fclose(fp);
        return raft::deserializeConfig(buf, n, c);
    }
    bool save(const raft::PersistentConfig& c) override
    {
        uint8_t buf[512];
        const size_t n = raft::serializeConfig(c, buf, sizeof(buf));
        FILE* fp = std::fopen(path.c_str(), "wb");
        if (!fp) return false;
        const bool ok = std::fwrite(buf, 1, n, fp) == n;
        std::fclose(fp);
        return ok;
    }
};

enum class HandMode { Free, Hold, Zero, Pull };
const char* handName(HandMode h)
{
    switch (h) {
    case HandMode::Free: return "free";
    case HandMode::Hold: return "holding";
    case HandMode::Zero: return "zero pose";
    case HandMode::Pull: return "pulling";
    }
    return "?";
}

struct Sim {
    explicit Sim(const std::string& scene) : arm(scene) {}
    Arm arm;
    ServoBus bus;
    long step = 0;
    HandMode hand = HandMode::Hold;
    double holdQ[7] = {0};      // where the hand holds the arm
    // simulated servos in their own position mode (like the HLS3606's mode 0): the servo test and Hold
    int posTest = -1;                 // the servo test's joint
    uint8_t posMask = 0;              // joints in position mode now
    double posGoalQ[7] = {0}, posRefQ[7] = {0}, posSpeed[7] = {0};
    int posLimitRaw[7] = {0};
    void positionGoal(int j, int16_t goalRaw, uint16_t speed, uint16_t limit, bool start)
    {
        // the raw goal through the simulated servo's own zero and sign, as the real servo reads it
        posGoalQ[j] = bus.truth.dir[j] * raft::ticksFromZero(goalRaw, bus.truth.zeroTicks[j]) * kRadPerTick;
        if (start || !(posMask & (1u << j))) posRefQ[j] = arm.q(j);
        posSpeed[j] = speed * kRadPsPerSpeedUnit;
        posLimitRaw[j] = limit;
        posMask |= (uint8_t)(1u << j);
    }
    double pull[3] = {0, 0, 0};
    double prevTcp[3] = {0, 0, 0};
    bool torqueWas = false;
    double dt() const { return arm.m->opt.timestep; }
    double t() const { return step * dt(); }
    void holdHere() { for (int i = 0; i < 7; ++i) holdQ[i] = arm.q(i); hand = HandMode::Hold; }
};

class SimBridge : public raft::WebBridge {
public:
    SimBridge(raft::App& app, Sim& sim) : WebBridge(app, "RAFT " RAFT_VERSION " (web_sim, built " __DATE__ " " __TIME__ ")", true), sim_(sim) {}
    raft::web::Server* server = nullptr;
    double rtf = 1.0;
    bool enableDown = false, clutchDown = false;
    raft::LoopReport loop;      // the simulator's control ticks in wall time (it runs them in bursts between naps)

protected:
    double now() override { return sim_.t(); }
    size_t extraCommand(const char* json, char* reply, size_t cap) override
    {
        char cmd[24] = "";
        raft::json::getString(json, "cmd", cmd, sizeof(cmd));
        if (!std::strcmp(cmd, "sim_reset")) {
            const double q0[7] = {0};
            sim_.arm.reset(q0);
            sim_.holdHere();
            return (size_t)std::snprintf(reply, cap, "{\"ok\": true, \"sim\": \"arm back in the zero pose, held\"}");
        }
        if (!std::strcmp(cmd, "servo_info")) {          // register tables made up from the simulated bus
            raft::RawServoState raw;
            sim_.bus.readRaw(sim_.arm, raw);
            raft::ServoRegisters regs[raft::kNumJoints];
            auto put16 = [](uint8_t* r, int a, int v) { r[a] = (uint8_t)(v & 0xff); r[a + 1] = (uint8_t)((v >> 8) & 0xff); };
            auto sm = [](int v) { return v < 0 ? (0x8000 | -v) : v; };
            for (int i = 0; i < raft::kNumJoints; ++i) {
                raft::ServoRegisters& s = regs[i];
                std::memset(&s, 0, sizeof(s));
                s.ok = true;
                s.id = (uint8_t)i;
                uint8_t* r = s.r;
                r[0] = 3; r[1] = 10;                        // firmware version (made up)
                put16(r, 3, 2566);                          // model (made up)
                r[5] = s.id;
                put16(r, 11, 4095);
                r[33] = sim_.torqueWas ? 2 : 0;
                r[40] = sim_.torqueWas ? 1 : 0;
                put16(r, 44, sm(raw.current[i]));
                put16(r, 48, 1000);
                r[55] = 1;
                put16(r, 56, raw.pos[i]);
                r[62] = raw.voltage[i];
                r[63] = raw.tempC[i];
                put16(r, 69, sm(raw.current[i]));
            }
            double j = 0.0;
            raft::json::getNumber(json, "joint", j);
            return raft::formatServoInfo(regs, (int)j, reply, cap);
        }
        if (!std::strcmp(cmd, "sim_button")) {        // the enable / clutch buttons a real handle would have
            char name[16] = "";
            bool down = false;
            raft::json::getString(json, "name", name, sizeof(name));
            if (!raft::json::getBool(json, "down", down) || (std::strcmp(name, "enable") && std::strcmp(name, "clutch")))
                return (size_t)std::snprintf(reply, cap, "{\"ok\": false, \"error\": \"name: enable | clutch, down: true | false\"}");
            (!std::strcmp(name, "enable") ? enableDown : clutchDown) = down;
            app_.setButtons(enableDown, clutchDown);
            return (size_t)std::snprintf(reply, cap, "{\"ok\": true, \"enable\": %s, \"clutch\": %s}", enableDown ? "true" : "false", clutchDown ? "true" : "false");
        }
        if (std::strcmp(cmd, "sim_hand")) return 0;
        char mode[16] = "";
        raft::json::getString(json, "mode", mode, sizeof(mode));
        if (!std::strcmp(mode, "hold")) sim_.holdHere();
        else if (!std::strcmp(mode, "zero")) { for (double& v : sim_.holdQ) v = 0.0; sim_.hand = HandMode::Hold; }
        else if (!std::strcmp(mode, "free")) sim_.hand = HandMode::Free;
        else if (!std::strcmp(mode, "pull")) {
            double p[3];
            if (raft::json::getNumberArray(json, "p", p, 3) != 3) return (size_t)std::snprintf(reply, cap, "{\"ok\": false, \"error\": \"pull needs p: [x, y, z] in m\"}");
            for (int k = 0; k < 3; ++k) sim_.pull[k] = p[k];
            sim_.hand = HandMode::Pull;
        } else {
            return (size_t)std::snprintf(reply, cap, "{\"ok\": false, \"error\": \"mode: hold | zero | free | pull\"}");
        }
        return (size_t)std::snprintf(reply, cap, "{\"ok\": true, \"hand\": \"%s\"}", handName(sim_.hand));
    }
    size_t sys(char* out, size_t cap) override
    {
        char lp[420];
        if (!raft::formatLoopReport(loop, lp, sizeof(lp))) lp[0] = '\0';
        return (size_t)std::snprintf(out, cap, "\"simulator\": \"MuJoCo, real time x%.2f\", \"uptime_s\": %.0f, \"hand\": \"%s\", \"loop\": {%s}, "
                                     "\"net\": {\"ip\": \"this PC\", \"web_clients\": %d, \"web_frames\": %lu}",
                                     rtf, sim_.t(), handName(sim_.hand), lp, server ? server->wsClients() : 0,
                                     server ? (unsigned long)server->framesSent() : 0ul);
    }
    size_t helloSys(char* out, size_t cap) override
    {
        return (size_t)std::snprintf(out, cap, "\"simulator\": \"MuJoCo\", \"ip\": \"this PC\", \"host\": \"web_sim\"");
    }
    bool healthFigures(raft::HealthFigures& f) override
    {
        f.loop = loop;
        f.uptimeS = (uint32_t)sim_.t();
        f.flags = raft::HF_NET_UP;
        f.webClients = server ? (uint8_t)server->wsClients() : 0;
        f.webFrames = server ? server->framesSent() : 0u;
        return true;
    }

private:
    Sim& sim_;
};

}  // namespace

int main(int argc, char** argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::string scene = defaultScene(argv[0], "scene.xml");
    const char* bindIp = "127.0.0.1";
    int port = 8080;
    bool calibrated = false, fresh = false;
    double springK = 824.0 * 1.10, friction = -1.0;
    for (int i = 1; i < argc; ++i) {
        const bool more = i + 1 < argc;
        if (!std::strcmp(argv[i], "--port") && more) port = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--bind") && more) bindIp = argv[++i];
        else if (!std::strcmp(argv[i], "--calibrated")) calibrated = true;
        else if (!std::strcmp(argv[i], "--fresh")) fresh = true;
        else if (!std::strcmp(argv[i], "--spring-k") && more) springK = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--friction") && more) friction = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--scene") && more) scene = argv[++i];
        else {
            std::printf("usage: web_sim [--port 8080] [--bind 127.0.0.1] [--calibrated] [--fresh] [--spring-k N] [--friction Nm] [--scene file]\n");
            return 2;
        }
    }

    Sim s(scene);
    if (!s.arm.ok()) return 1;
    s.arm.configure(friction, 1.0);
    s.arm.setSpringK(springK);
    for (int i = 0; i < 7; ++i) { s.bus.truth.zeroTicks[i] = kZero[i]; s.bus.truth.dir[i] = kDir[i]; s.bus.truth.ktScale[i] = kKtScale[i]; }
    const double q0[7] = {0};
    s.arm.reset(q0);
    s.bus.reset();
    s.bus.thermal.reset(30.0);

    std::string exe = argv[0];
    const size_t cut = exe.find_last_of("/\\");
    FileStorage storage;
    storage.path = (cut == std::string::npos ? std::string(".") : exe.substr(0, cut)) + "/web_sim_settings.bin";
    if (fresh) std::remove(storage.path.c_str());
    if (calibrated) {
        raft::PersistentConfig c;
        raft::defaultPersistentConfig(c);
        for (int i = 0; i < 7; ++i) { c.joint[i].zeroTicks = (int16_t)kZero[i]; c.joint[i].dir = (int8_t)kDir[i]; c.joint[i].flags = raft::CAL_ZERO | raft::CAL_DIR; }
        storage.save(c);
    }

    simrcb::NetConfig net;
    net.latencyS = 0.002; net.jitterS = 0.0005; net.udpLoss = 0.0;
    simrcb::FakeRcb rcb(net);
    raft::App app(storage, rcb);
    app.begin(0.0, raft::runModelSelfTest().pass);
    {   // the simulated servos have no lopsided gear friction: no friction compensation (holding)
        char r[256];
        app.handleCommand("{\"cmd\": \"hold_friction\", \"value\": 0}", r, sizeof(r), 0.0, nullptr);
    }
    app.setButtonPresence(true, true);                    // simulated handle buttons (sim_button)

    timeBeginPeriod(1);
    wintr::WsaInit wsa;
    wintr::WinListener listener;
    if (!wsa.ok || !listener.begin(bindIp, (uint16_t)port)) { std::printf("cannot listen on %s:%d (in use?)\n", bindIp, port); return 2; }
    const raft::web::Asset assets[] = {
        {"/", "text/html; charset=utf-8", raft::webui::kIndexGz, raft::webui::kIndexGzLen, true, raft::webui::kIndexEtag},
        {"/index.html", "text/html; charset=utf-8", raft::webui::kIndexGz, raft::webui::kIndexGzLen, true, raft::webui::kIndexEtag},
        {"/arm.bin", "application/octet-stream", raft::webui::kArmGz, raft::webui::kArmGzLen, true, raft::webui::kArmEtag},
    };
    SimBridge bridge(app, s);
    raft::web::Server server(listener, bridge, assets, (int)(sizeof(assets) / sizeof(assets[0])));
    bridge.server = &server;
    std::printf("RAFT web simulator: open http://%s:%d/   (settings: %s, %s)\n",
                std::strcmp(bindIp, "0.0.0.0") ? bindIp : "<this-pc-ip>", port, storage.path.c_str(),
                raft::fullyCalibrated(app.config()) ? "calibrated" : "not calibrated: try the Calibrate page");

    // follower for the emulated PAR6: starts in the DH pose of the CAD zero pose, tracks the SOFT_RTI target.
    // A table stops its J2 at 60 deg forward (CAD), where its motor pushes and t_ext rises; its gripper closes
    // on an object at raw 150 (0 open .. 255 closed), where the gripper current rises.
    double fq[6] = {0.0, -kPi / 2, kPi, 0.0, 0.0, kPi}, fqd[6] = {0}, text[6] = {0};
    const double kWallDh = rad(60.0) - kPi / 2, kWallK = 20.0, kObjectRaw = 150.0;
    double gripRaw = 0.0, gripCur = 0.0;
    double nextRobot = 0.0, lastPrint = 0.0, rtfT0 = wall(), rtfS0 = 0.0;
    int16_t out[7] = {0};
    bool on = false, need = false;
    const double t0 = wall();
    raft::LoopStats loopStats(raft::kControlPeriodUs);
    const auto wallUs = []() { return (uint32_t)(long long)(wall() * 1e6); };
    for (;;) {
        const double target = wall() - t0;
        int budget = 50;                                  // catch up at most 50 ms per pass
        while (s.t() < target && budget-- > 0) {
            if (s.step % kCtrlEvery == 0) {
                raft::RawServoState r;
                const uint32_t tw = wallUs();
                loopStats.wake(tw, false);
                s.bus.readRaw(s.arm, r);
                const uint32_t ta = wallUs();
                app.controlTick(r, s.t(), out, on, need);
                const uint32_t te = wallUs();
                loopStats.part(raft::kPartRead, ta - tw);
                loopStats.part(raft::kPartApp, te - ta);
                if (loopStats.done(te)) bridge.loop = loopStats.report();
                raft::App::ServoTestOp op;
                if (app.takeServoTestOp(op)) {
                    if (op.step == raft::App::kServoTestStop) {
                        s.posTest = -1;
                        s.posMask &= (uint8_t)~(1u << op.joint);
                        s.bus.setActive(op.joint, 0);
                        s.holdQ[op.joint] = s.arm.q(op.joint);      // the hand takes that joint again where it is
                    } else {
                        s.positionGoal(op.joint, op.goalRaw, op.speed, op.torque, op.step == raft::App::kServoTestStart);
                        s.posTest = op.joint;
                    }
                }
                {   // Hold, moves and replay in position mode (with torque on), like ServoBus::setPositionMask on the arm
                    const raft::App::ServoGoals& g = app.servoGoals();
                    for (int j = 0; j < 7; ++j) {
                        if (j == s.posTest) continue;
                        const uint8_t bit = (uint8_t)(1u << j);
                        if (on && (g.mask & bit)) s.positionGoal(j, g.raw[j], g.speed, g.limit, false);
                        else if (s.posMask & bit) { s.posMask &= (uint8_t)~bit; s.bus.setActive(j, 0); }
                    }
                    app.setServoModes(on ? (uint8_t)(s.posMask & g.mask) : 0);
                    s.bus.selfMask = s.posMask;
                }
                int16_t cmd[7];
                for (int i = 0; i < 7; ++i) cmd[i] = on ? out[i] : 0;
                s.bus.command(cmd, s.step);
                // the person: catch the arm when torque goes off, let go when it comes on
                if (on != s.torqueWas) {
                    if (on && (s.hand == HandMode::Hold)) s.hand = HandMode::Free;
                    if (!on && s.hand == HandMode::Free) s.holdHere();
                    s.torqueWas = on;
                }
                // calibration helpers
                const raft::Calibrator& cal = app.calibrator();
                if (cal.kind() == raft::CalibKind::Zero && s.hand != HandMode::Zero) s.hand = HandMode::Zero;
                if (cal.kind() != raft::CalibKind::Zero && s.hand == HandMode::Zero) { for (double& v : s.holdQ) v = 0.0; s.hand = HandMode::Hold; }
            }
            for (int j = 0; j < 7; ++j) {   // each position-mode servo's own loop: its reference moves at the set speed,
                if (!(s.posMask & (1u << j))) continue;   // PD up to the current limit
                const double step = s.posSpeed[j] * s.dt(), d = s.posGoalQ[j] - s.posRefQ[j];
                s.posRefQ[j] += std::fabs(d) <= step ? d : (d > 0 ? step : -step);
                const double tau = 20.0 * (s.posRefQ[j] - s.arm.q(j)) - 0.3 * s.arm.qd(j);   // stiff, like the real servo
                const Truth& tr = s.bus.truth;
                double raw = tau / (tr.currentSign * tr.dir[j] * tr.mAPerUnit * 1e-3 * tr.ktNmPerA * tr.ktScale[j]);
                raw = std::fmax(-s.posLimitRaw[j], std::fmin((double)s.posLimitRaw[j], raw));
                s.bus.setActive(j, (int)std::lround(raw));
            }
            double ctrl[7];
            s.bus.apply(s.step, s.dt(), ctrl);
            s.arm.clearApplied();
            bool frozen = false;
            if (s.hand == HandMode::Zero) {
                for (int i = 0; i < 7; ++i) s.arm.setQ(i, 0.0);
                frozen = true;
            } else if (s.hand == HandMode::Hold) {
                // a hand compensates gravity and corrects the rest, like a person holding the arm still
                for (int i = 0; i < 7; ++i)
                    if (!(s.posMask & (1u << i))) s.arm.applyJointTorque(i, s.arm.bias(i) + 10.0 * (s.holdQ[i] - s.arm.q(i)) - 0.15 * s.arm.qd(i));
            } else if (s.hand == HandMode::Pull) {
                double p[3], f[3];
                s.arm.tcp(p);
                for (int k = 0; k < 3; ++k) f[k] = 300.0 * (s.pull[k] - p[k]) - 6.0 * (p[k] - s.prevTcp[k]) / s.dt();
                s.arm.applyTcpForce(f);
            }
            s.arm.tcp(s.prevTcp);
            s.arm.step(ctrl, frozen);
            ++s.step;
        }
        // the emulated PAR6 at 250 Hz
        rcb.setTime(s.t());
        while (s.t() >= nextRobot) {
            double tg[6], tgv[6];
            for (double& v : text) v = 0.0;
            if (rcb.target(tg, tgv)) {
                const double a = 1.0 - std::exp(-0.004 / 0.03);
                for (int i = 0; i < 6; ++i) { const double qn = fq[i] + a * (tg[i] - fq[i]); fqd[i] = (qn - fq[i]) / 0.004; fq[i] = qn; }
                if (fq[1] > kWallDh) { fq[1] = kWallDh; fqd[1] = 0.0; }
                if (tg[1] > kWallDh) text[1] = kWallK * (tg[1] - kWallDh);   // measured minus model torque: the motor pushing
            } else {
                for (double& v : fqd) v = 0.0;
            }
            const double ga = 1.0 - std::exp(-0.004 / 0.15);
            gripRaw += ga * (std::fmin((double)rcb.gripperRaw(), kObjectRaw) - gripRaw);
            gripCur = rcb.gripperRaw() > kObjectRaw ? 4.0 * (rcb.gripperRaw() - kObjectRaw) : 50.0;
            rcb.robotTick(fq, fqd, text, gripRaw, gripCur);
            nextRobot += 0.004;
        }
        app.networkTick(s.t());
        server.poll(s.t());
        if (wall() - lastPrint >= 5.0) {
            const double w = wall();
            bridge.rtf = (s.t() - rtfS0) / std::fmax(1e-3, w - rtfT0);
            rtfT0 = w; rtfS0 = s.t();
            char line[400];
            app.statusLine(line, sizeof(line));
            std::printf("%7.1f s  %s | hand %s | web clients %d, frames %u\n", s.t(), line, handName(s.hand), server.wsClients(), (unsigned)server.framesSent());
            lastPrint = w;
        }
        wintr::napMs();
    }
}
