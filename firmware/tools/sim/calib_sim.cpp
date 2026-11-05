// Bench calibration rehearsal (dev tool, nothing here runs on the arm). The firmware App drives the MuJoCo
// arm through a simulated servo bus whose servos have unknown encoder zeros, flipped signs and torque-
// constant errors, with a J2 spring 10 % stiffer than ordered. The scenario sends the same JSON commands
// as the bench procedure (docs/calibration.md) and checks that:
//   - torque is refused until zero and direction are calibrated
//   - calib_zero recovers every zero (the signs are the design's, arm_design.h)
//   - calib_torque recovers the J2/J3 torque constants, and refuses a joint without gravity load
//   - a wrong sign during a torque ramp is caught and the arm goes limp
//   - calib_spring recovers how strong the spring really is
//   - the saved result loads on the next boot and cuts the gravity-compensation error
// Usage: calib_sim.exe [scene.xml] [friction Nm per joint, default the MJCF estimate 0.02]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "arm_design.h"
#include "app.h"
#include "config_store.h"
#include "json_lite.h"
#include "model_selftest.h"
#include "sim_core.h"
#include "sim_rcb.h"

using namespace sim;

namespace {

int g_fail = 0;
double g_friction = -1.0;
void check(bool ok, const char* what) { std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) ++g_fail; }

struct RamStorage : raft::Storage {
    uint8_t blob[512]; size_t len = 0;
    bool load(raft::PersistentConfig& c) override { return len && raft::deserializeConfig(blob, len, c); }
    bool save(const raft::PersistentConfig& c) override { len = raft::serializeConfig(c, blob, sizeof(blob)); return len > 0; }
};

// The bench truth: what the real servos and spring are, which the firmware has to find out.
const int    kZero[7] = {1517, 2903, 310, 3790, 2421, 1006, 2640};
const int8_t (&kDir)[7] = raft::kJointDir;   // the servos sit as the design says
const double kKtScale[7] = {1.0, 0.90, 1.10, 1.0, 1.0, 1.0, 1.0};
const double kSpringOrdered = 824.0;      // Z-078I, N/m
const double kSpringTrue = 824.0 * 1.10;

// MuJoCo arm + simulated bus + the firmware App: physics at 1 kHz, App at 250 Hz.
struct Bench {
    Bench(const std::string& scene, raft::Storage& st, raft::rti::Transport& tr) : arm(scene), app(st, tr) {}
    Arm arm;
    ServoBus bus;
    raft::App app;
    long step = 0;
    int16_t out[7] = {0};
    bool on = false, need = false;
    // a hand supporting every joint except freeJoint at supportQ (only when support is set)
    bool support = false; int freeJoint = -1; double supportQ[7] = {0};

    bool setUp()
    {
        if (!arm.ok()) return false;
        arm.configure(g_friction, 1.0);             // < 0: MJCF friction estimate (0.02 Nm)
        arm.setSpringK(kSpringTrue);
        for (int i = 0; i < 7; ++i) { bus.truth.zeroTicks[i] = kZero[i]; bus.truth.dir[i] = kDir[i]; bus.truth.ktScale[i] = kKtScale[i]; }
        const double q0[7] = {0};
        arm.reset(q0); bus.reset(); bus.thermal.reset(30.0);
        return true;
    }
    double dt() const { return arm.m->opt.timestep; }
    double t() const { return step * dt(); }
    void stepOnce(bool frozen)
    {
        if (step % kCtrlEvery == 0) {
            raft::RawServoState r;
            bus.readRaw(arm, r);
            app.controlTick(r, t(), out, on, need);
            int16_t cmd[7];
            for (int i = 0; i < 7; ++i) cmd[i] = on ? out[i] : 0;
            bus.command(cmd, step);
        }
        double ctrl[7];
        bus.apply(step, dt(), ctrl);
        arm.clearApplied();
        if (!frozen && support)
            for (int i = 0; i < 7; ++i) if (i != freeJoint) arm.applyJointTorque(i, 3.0 * (supportQ[i] - arm.q(i)) - 0.08 * arm.qd(i));
        arm.step(ctrl, frozen);
        ++step;
    }
    void run(double s, bool frozen = false) { const long n = std::lround(s / dt()); for (long k = 0; k < n; ++k) stepOnce(frozen); }
    double runCalib(double maxS, int traceJoint = -1)
    {
        const double t0 = t();
        const bool trace = traceJoint >= 0 && std::getenv("CALIB_TRACE");
        while (app.calibrator().active() && t() - t0 < maxS) {
            if (trace && step % 20 == 0)
                std::printf("    t %.2f  raw %4d  q %8.3f deg  qd %7.3f  %s\n", t() - t0, out[traceJoint], deg(arm.q(traceJoint)),
                            arm.qd(traceJoint), app.calibrator().phaseText());
            stepOnce(false);
        }
        return t() - t0;
    }
    std::string cmd(const char* line) { char rep[1600]; app.handleCommand(line, rep, sizeof(rep), t(), "10.0.0.5"); return rep; }
    std::string lastCalib()
    {
        const std::string r = cmd("{\"cmd\": \"calib_result\"}");
        char m[160] = "";
        raft::json::getString(r.c_str(), "last", m, sizeof(m));
        return m;
    }
};

bool has(const std::string& s, const char* what) { return s.find(what) != std::string::npos; }
double truthKt(int j) { return 4.5 * 0.0980665 * kKtScale[j]; }

// Gravity-compensation error at a few poses: actuator + spring - gravity, arm held still, GRAVITY mode.
const int kPoses = 6;
const double kPoseDeg[kPoses][2] = {{0, 0}, {45, 45}, {90, 90}, {-45, -45}, {60, 0}, {30, -60}};
void residuals(const std::string& scene, const raft::PersistentConfig& cfg, double res[kPoses][2], double load[kPoses][2])
{
    RamStorage s; s.save(cfg);
    simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    Bench R(scene, s, rcb);
    R.setUp();
    R.app.begin(0.0, true);
    for (int p = 0; p < kPoses; ++p) {
        double q[7] = {0};
        q[1] = rad(kPoseDeg[p][0]); q[2] = rad(kPoseDeg[p][1]);
        R.arm.reset(q);
        if (p == 0) R.cmd("{\"cmd\": \"mode\", \"mode\": \"gravity\"}");
        R.run(0.3, true);
        for (int k = 0; k < 2; ++k) { res[p][k] = R.arm.residual(1 + k); load[p][k] = R.arm.bias(1 + k); }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string scene = argc > 1 && std::strcmp(argv[1], "-") ? argv[1] : defaultScene(argv[0], "scene.xml");
    if (argc > 2) g_friction = std::atof(argv[2]);
    RamStorage st; simrcb::NetConfig net; simrcb::FakeRcb rcb(net);
    Bench B(scene, st, rcb);
    if (!B.setUp()) return 1;
    const raft::ModelSelfTestResult selftest = raft::runModelSelfTest();
    B.app.begin(0.0, selftest.pass);
    {   // the simulated servos have no lopsided gear friction: no friction compensation (holding)
        char r[256];
        B.app.handleCommand("{\"cmd\": \"hold_friction\", \"value\": 0}", r, sizeof(r), 0.0, nullptr);
    }
    char line[200];

    std::printf("=== Bench truth (unknown to the firmware) ===\n");
    std::printf("  zero ticks  "); for (int i = 0; i < 7; ++i) std::printf(" %5d", kZero[i]); std::printf("\n");
    std::printf("  sign        "); for (int i = 0; i < 7; ++i) std::printf(" %+5d", kDir[i]); std::printf("\n");
    std::printf("  Kt Nm/A     "); for (int i = 0; i < 7; ++i) std::printf(" %5.3f", truthKt(i)); std::printf("   (firmware assumes 0.441)\n");
    const double fr = g_friction < 0 ? 0.02 : g_friction;
    std::printf("  spring      %.0f N/m installed, %.0f N/m configured; friction %.3f Nm per joint\n", kSpringTrue, kSpringOrdered, fr);

    std::printf("\n=== 1. Fresh boot ===\n");
    B.run(0.1, true);
    {
        const std::string r = B.cmd("{\"cmd\": \"mode\", \"mode\": \"gravity\"}");
        check(has(r, "\"ok\": false") && has(r, "not calibrated"), "torque refused before calibration");
        check(!B.on, "servos stay limp");
    }

    std::printf("\n=== 2. calib_zero (arm held in the CAD zero pose) ===\n");
    {
        const std::string r = B.cmd("{\"cmd\": \"calib_zero\"}");
        B.run(0.7, true);
        int worst = 0;
        for (int i = 0; i < 7; ++i) worst = std::max(worst, std::abs(raft::ticksFromZero(B.app.config().joint[i].zeroTicks, kZero[i])));
        std::printf("  %s\n", B.lastCalib().c_str());
        check(has(r, "\"ok\": true") && worst == 0, "every zero recovered exactly");
    }

    std::printf("\n=== 3. Joint readings (the directions are the design's) ===\n");
    {
        check(raft::fullyCalibrated(B.app.config()), "the zero is enough, the arm counts as calibrated");
        // the firmware's joint reading now matches the arm, including the zeros near the tick wrap
        const double qTest[7] = {10, 20, -30, 40, -50, 60, 15};
        for (int i = 0; i < 7; ++i) B.arm.setQ(i, rad(qTest[i]));
        B.run(0.02, true);
        double worst = 0;
        for (int i = 0; i < 7; ++i) worst = std::fmax(worst, std::fabs(deg(B.app.lastInput().q[i]) - qTest[i]));
        check(worst < 0.09, "joint reading at a test pose within one tick (0.088 deg)");
        for (int i = 0; i < 7; ++i) B.arm.setQ(i, 0.0);
        B.run(0.02, true);
    }

    std::printf("\n=== 4. Spring as ordered, torque on ===\n");
    B.cmd("{\"cmd\": \"spring\", \"type\": \"cable\", \"k\": 824, \"a_mm\": 13, \"b_mm\": 20}");
    const raft::PersistentConfig beforeTorqueCal = B.app.config();
    {
        const std::string r = B.cmd("{\"cmd\": \"mode\", \"mode\": \"move\"}");
        B.run(0.3, true);              // the person still holds the arm while torque comes on
        B.run(1.0, false);
        double dev = 0;
        for (int i = 0; i < 7; ++i) dev = std::fmax(dev, std::fabs(deg(B.arm.q(i))));
        std::printf("  MOVE holds the zero pose after letting go: max deviation %.2f deg\n", dev);
        check(has(r, "\"ok\": true") && B.on && dev < 1.0, "torque accepted once calibrated, arm holds");
    }

    std::printf("\n=== 5. Refusals ===\n");
    std::printf("  model gravity load at the zero pose, Nm:");
    for (int i = 0; i < 7; ++i) std::printf(" J%d %+.3f", i + 1, B.app.lastOutput().tauGravity[i]);
    std::printf("\n");
    {
        B.cmd("{\"cmd\": \"calib_torque\", \"joint\": 1}");
        B.run(0.05);
        const std::string m = B.lastCalib();
        std::printf("  J1: %s\n", m.c_str());
        check(has(m, "carries only"), "calib_torque refuses a joint without gravity load");
        B.cmd("{\"cmd\": \"calib_spring\"}");
        B.run(0.05);
        const std::string s = B.lastCalib();
        std::printf("  spring at q2 = 0: %s\n", s.c_str());
        check(has(s, "tilt the upper arm"), "calib_spring refuses with the upper arm vertical");
    }

    std::printf("\n=== 6. calib_torque J3, then J2 (zero pose, MOVE holds the other joints) ===\n");
    for (int j : {2, 1}) {
        std::snprintf(line, sizeof(line), "{\"cmd\": \"calib_torque\", \"joint\": %d}", j + 1);
        B.cmd(line);
        const double took = B.runCalib(30.0, j);
        const raft::JointCal& c = B.app.config().joint[j];
        const double err = 100.0 * (c.ktNmPerA / truthKt(j) - 1.0);
        std::printf("  J%d (%.1f s): %s\n", j + 1, took, B.lastCalib().c_str());
        std::printf("      Kt %.4f vs true %.4f (%+.1f %%), friction %.4f Nm vs %.3f\n", c.ktNmPerA, truthKt(j), err, c.frictionNm, fr);
        std::snprintf(line, sizeof(line), "J%d torque constant within 3 %% of the truth", j + 1);
        check((c.flags & raft::CAL_KT) && std::fabs(err) < 3.0, line);
    }
    std::printf("  present-current register per commanded unit: %.3f (truth %.3f)\n", B.app.config().currentRegPerUnit, B.bus.truth.regPerUnit);

    std::printf("\n=== 7. A wrong sign during a torque sweep ===\n");
    {
        const int j = 2;
        for (int i = 0; i < 7; ++i) B.supportQ[i] = B.arm.q(i);
        B.support = true; B.freeJoint = j;
        std::snprintf(line, sizeof(line), "{\"cmd\": \"set_dir\", \"joint\": %d, \"dir\": %d}", j + 1, -kDir[j]);
        check(B.cmd(line).find("fixed") != std::string::npos, "set_dir is refused: the directions are fixed");
        B.cmd("{\"cmd\": \"mode\", \"mode\": \"limp\"}");
        B.bus.truth.dir[j] = -kDir[j];   // a servo mounted the other way round than the design says
        std::snprintf(line, sizeof(line), "{\"cmd\": \"calib_torque\", \"joint\": %d}", j + 1);
        B.cmd(line);                   // same tick: the ramp owns the joint from its first tick
        const double q0 = B.arm.q(j), t0 = B.t();
        double peak = 0, tripped = -1;
        while (B.t() - t0 < 2.0) {
            B.stepOnce(false);
            if (tripped < 0) peak = std::fmax(peak, std::fabs(deg(B.arm.q(j) - q0)));
            if (tripped < 0 && !B.app.calibrator().active()) { tripped = B.t() - t0; B.freeJoint = -1; }   // the hand catches it
        }
        const std::string m = B.lastCalib();
        std::printf("  %s\n  tripped after %.2f s with the joint %.1f deg from its start, mode now %s\n",
                    m.c_str(), tripped, peak, B.app.mode() == raft::Mode::Limp ? "LIMP" : "not limp");
        check(has(m, "did not follow") && B.app.mode() == raft::Mode::Limp && !B.on, "runaway caught, arm goes limp");
        // put things right: the servo as the design says, arm back in the zero pose, torque on again
        B.bus.truth.dir[j] = kDir[j];
        for (int i = 0; i < 7; ++i) B.arm.setQ(i, 0.0);
        B.support = false;
        B.cmd("{\"cmd\": \"mode\", \"mode\": \"move\"}");
        B.run(0.3, true);
        B.run(0.5, false);
    }

    std::printf("\n=== 8. calib_spring (upper arm tilted 45 deg, forearm level) ===\n");
    {
        B.cmd("{\"cmd\": \"move\", \"q_deg\": [0, 45, 45, 0, 0, 0, 0], \"speed_deg\": 30}");
        B.run(3.0);
        std::printf("  reached q2 %.1f, q3 %.1f deg\n", deg(B.arm.q(1)), deg(B.arm.q(2)));
        B.cmd("{\"cmd\": \"calib_spring\"}");
        const double took = B.runCalib(30.0);
        const double scale = B.app.config().spring.scale;
        std::printf("  (%.1f s) %s\n  spring.scale now %.3f (truth %.3f)\n", took, B.lastCalib().c_str(), scale, kSpringTrue / kSpringOrdered);
        check(std::fabs(scale / (kSpringTrue / kSpringOrdered) - 1.0) < 0.05, "spring strength within 5 % of the truth");
    }

    std::printf("\n=== 9. save, reboot ===\n");
    {
        const std::string r = B.cmd("{\"cmd\": \"save\"}");
        simrcb::FakeRcb rcb2(net);
        raft::App fresh(st, rcb2);
        fresh.begin(0.0, true);
        const raft::PersistentConfig& a = B.app.config();
        const raft::PersistentConfig& b = fresh.config();
        check(has(r, "\"saved\": true") && raft::fullyCalibrated(b) && b.joint[1].ktNmPerA == a.joint[1].ktNmPerA
              && b.joint[2].zeroTicks == a.joint[2].zeroTicks && b.spring.scale == a.spring.scale, "the next boot loads the whole calibration");
    }

    std::printf("\n=== 10. Gravity-compensation error, arm held still in GRAVITY ===\n");
    {
        double resA[kPoses][2], resB[kPoses][2], load[kPoses][2];
        residuals(scene, beforeTorqueCal, resA, load);
        residuals(scene, B.app.config(), resB, load);
        std::printf("  %-16s %16s %22s %24s\n", "pose q2/q3 deg", "load J2/J3 Nm", "zero+sign only J2/J3", "fully calibrated J2/J3");
        double worstA = 0, worstB = 0;
        for (int p = 0; p < kPoses; ++p) {
            std::printf("  %5.0f / %-8.0f %8.3f %7.3f %11.4f %10.4f %12.4f %11.4f\n", kPoseDeg[p][0], kPoseDeg[p][1],
                        load[p][0], load[p][1], resA[p][0], resA[p][1], resB[p][0], resB[p][1]);
            for (int k = 0; k < 2; ++k) { worstA = std::fmax(worstA, std::fabs(resA[p][k])); worstB = std::fmax(worstB, std::fabs(resB[p][k])); }
        }
        std::printf("  worst error: %.4f Nm before, %.4f Nm after torque + spring calibration\n", worstA, worstB);
        check(worstB < 0.006 && worstB < 0.5 * worstA, "calibration cuts the worst J2/J3 error at least in half, below 6 mNm");
    }

    std::printf("\nSimulated bench time %.1f s\n%s\n", B.t(), g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return g_fail ? 1 : 0;
}
