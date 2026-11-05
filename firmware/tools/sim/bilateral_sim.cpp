// Bilateral teleoperation simulator (dev tool). Everything on the leader side is firmware code:
//   control.cpp (BILATERAL), teleop.cpp (align -> claim -> couple), rti_link.cpp + rti_protocol.cpp
// talking over an emulated network to an emulated RCB runtime (sim_rcb.h), whose robot is a second
// MuJoCo arm standing in for PAR6: stiff joint-position control with gravity compensation, SOFT_RTI
// interpolation, a block in front of it, and a "gripper" (its trigger joint) that an object stops at 10 deg.
// Build: tools\sim\build.cmd    Run: tools\sim\out\bilateral_sim.exe
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "control.h"
#include "kinematics.h"
#include "rti_link.h"
#include "sim_core.h"
#include "sim_rcb.h"
#include "teleop.h"

using namespace sim;

namespace {

const double kDhOffset[6] = {0.0, -kPi / 2, kPi, 0.0, 0.0, kPi};
const double kTrigClosed = rad(60.0);

double wrap(double x) { while (x > kPi) x -= 2 * kPi; while (x <= -kPi) x += 2 * kPi; return x; }

// PAR6 stand-in: position control with perfect gravity compensation, strong motors.
struct Follower {
    explicit Follower(const std::string& scene) : arm(scene) {}
    Arm arm;
    double Kp[7] = {20, 20, 20, 8, 8, 8, 2}, Kd[7] = {0.4, 0.45, 0.4, 0.15, 0.15, 0.12, 0.04};
    double hold[7] = {0}, tauPd[7] = {0};
    bool controlled = false;

    void control(simrcb::FakeRcb& rcb, double ctrl[7])
    {
        double qDh[6], qdDh[6], qT[7], qdT[7];
        const bool active = rcb.target(qDh, qdDh);
        if (active) { for (int i = 0; i < 6; ++i) { qT[i] = qDh[i] - kDhOffset[i]; qdT[i] = qdDh[i]; } }
        else {
            if (controlled) for (int i = 0; i < 6; ++i) hold[i] = arm.q(i);   // hold where control stopped
            for (int i = 0; i < 6; ++i) { qT[i] = hold[i]; qdT[i] = 0; }
        }
        controlled = active;
        qT[6] = rcb.gripperRaw() / 255.0 * kTrigClosed; qdT[6] = 0;
        for (int i = 0; i < 7; ++i) {
            tauPd[i] = Kp[i] * wrap(qT[i] - arm.q(i)) + Kd[i] * (qdT[i] - arm.qd(i));
            ctrl[i] = arm.bias(i) + tauPd[i];
        }
    }
    void report(simrcb::FakeRcb& rcb)
    {
        double q[6], qd[6], te[6];
        // t_ext as the runtime defines it: measured motor torque minus the model's (here: the PD part)
        for (int i = 0; i < 6; ++i) { q[i] = arm.q(i) + kDhOffset[i]; qd[i] = arm.qd(i); te[i] = tauPd[i]; }
        // g_p on the gripper's raw firmware-mode scale, 0 open .. 255 closed
        const double gp = std::fmin(255.0, std::fmax(0.0, arm.q(6)) / kTrigClosed * 255.0);
        const double gc = std::fabs(tauPd[6]) / (4.5 * 0.0980665) * 1000.0;
        rcb.robotTick(q, qd, te, gp, gc);
    }
};

struct Metrics {
    double tCoupled = -1, alignErrDeg = 0;
    double freeTrackMm = 0, freeHandMean = 0, freeHandPeak = 0;
    double contactHandN = 0, contactGapMm = 0, followerDropMm = 0;
    double releaseSpeed = 0, releaseGapMm = 0;
    double fingerFree = 0, fingerBlocked = 0, trigLeaderDeg = 0, trigFollowerDeg = 0;
    double lossJumpMm = 0, recoupleS = -1, waitedForResume = 0;
    double extPeakNm = 0, pcMinJ = 0;
    unsigned cmds = 0;
};

Metrics runScenario(const std::string& leaderScene, const std::string& followerScene, const simrcb::NetConfig& net, bool verbose,
                    const raft::ControlConfig& cfg)
{
    Metrics M;
    Leader L(leaderScene);
    Follower F(followerScene);
    if (!L.arm.ok() || !F.arm.ok()) return M;
    L.arm.configure(-1, 1.0);
    F.arm.configure(-1, 1.0);
    F.arm.setTorqueLimit(3.0);
    F.arm.setJointRange(6, rad(-90), rad(10));          // the gripper closes on an object at 10 deg

    simrcb::FakeRcb rcb(net);
    raft::rti::LinkConfig lc = raft::rti::defaultLinkConfig();
    raft::rti::Link link(rcb, lc);
    raft::Controller ctl(cfg);
    raft::TeleopSession session(ctl, link, raft::defaultTeleopConfig());

    const double qL0[7] = {0, 0, 0, 0, 0, 0, 0};
    const double qF0[7] = {rad(10), 0, 0, 0, rad(-15), 0, 0};
    L.reset(qL0);
    F.arm.reset(qF0);
    for (int i = 0; i < 7; ++i) F.hold[i] = qF0[i];
    ctl.setMode(raft::Mode::Gravity);
    session.begin(0.0);

    Hand hand; hand.tRelease = 0;                       // released
    double t = 0;
    auto stepAll = [&]() {
        t = L.time();
        rcb.setTime(t);
        double fc[7];
        F.control(rcb, fc);
        if (L.step % 4 == 0) F.report(rcb);
        F.arm.clearApplied();
        F.arm.step(fc, false);
        link.poll(t);
        L.stepOnce(ctl, &hand, nullptr, false,
                   [&](raft::ControlInput& in) { session.beforeControl(in, t); },
                   [&](const raft::ControlInput& in, const raft::ControlOutput& o) {
                       session.afterControl(in, t);
                       for (int i = 0; i < 6; ++i) M.extPeakNm = std::fmax(M.extPeakNm, std::fabs(o.tauExt[i]));
                       M.pcMinJ = std::fmin(M.pcMinJ, o.pcEnergyJ);
                   });
    };
    auto runFor = [&](double s, auto&& each) { const double t0 = L.time(); while (L.time() - t0 < s) { stepAll(); each(); } };
    auto nothing = [] {};
    auto tcpGap = [&]() { double a[3], b[3]; L.arm.tcp(a); F.arm.tcp(b); return 1e3 * dist3(a, b); };

    // P0: align and couple
    runFor(8.0, [&] {
        if (M.tCoupled < 0 && session.stage() == raft::TeleopSession::Stage::Coupled) {
            M.tCoupled = L.time();
            for (int i = 0; i < 6; ++i) M.alignErrDeg = std::fmax(M.alignErrDeg, std::fabs(deg(wrap(L.arm.q(i) - F.arm.q(i)))));
        }
    });
    if (M.tCoupled < 0) { if (verbose) std::printf("  never coupled: stage %s, link %s, error \"%s\"\n",
                                                   raft::TeleopSession::stageName(session.stage()), raft::rti::phaseName(link.phase()), link.lastError()); return M; }
    runFor(0.5, nothing);

    // P1: free motion, drag the handle 40 mm sideways and back
    L.arm.tcp(hand.p0); hand.delta[0] = 0; hand.delta[1] = -0.04; hand.delta[2] = 0;
    hand.t0 = L.time(); hand.t1 = L.time() + 1.0; hand.tRelease = L.time() + 2.4; hand.peak = hand.sum = 0; hand.n = 0;
    runFor(1.2, [&] { M.freeTrackMm = std::fmax(M.freeTrackMm, tcpGap()); });
    double back0[3]; L.arm.tcp(back0);
    std::memcpy(hand.p0, back0, sizeof(back0)); hand.delta[1] = 0.04; hand.t0 = L.time(); hand.t1 = L.time() + 1.0;
    runFor(1.2, [&] { M.freeTrackMm = std::fmax(M.freeTrackMm, tcpGap()); });
    M.freeHandMean = hand.n ? hand.sum / hand.n : 0; M.freeHandPeak = hand.peak;
    runFor(0.5, nothing);

    // P2: push the handle 60 mm down; the follower meets the block on the way
    double f0[3]; F.arm.tcp(f0);
    L.arm.tcp(hand.p0); hand.delta[0] = 0; hand.delta[1] = 0; hand.delta[2] = -0.06;
    hand.t0 = L.time(); hand.t1 = L.time() + 1.5; hand.tRelease = L.time() + 2.5;
    runFor(2.4, nothing);
    M.contactHandN = hand.last; M.contactGapMm = tcpGap();
    double f1[3]; F.arm.tcp(f1); M.followerDropMm = 1e3 * (f0[2] - f1[2]);

    // P3: let go while pressing: the coupling pulls the leader back onto the follower
    runFor(0.2, nothing);                                // hand releases at the end of P2 + 0.1 s
    runFor(0.5, nothing);
    runFor(1.5, [&] { for (int i = 0; i < 6; ++i) M.releaseSpeed = std::fmax(M.releaseSpeed, std::fabs(L.arm.qd(i))); });
    M.releaseGapMm = tcpGap();

    // P4: trigger: squeeze the leader trigger toward 40 deg; the follower's gripper stops at 10 deg
    hand.fingerOn = true; hand.fingerQ = rad(40); hand.fingerK = 0.5;
    runFor(2.0, nothing);
    M.fingerBlocked = std::fabs(hand.fingerTau); M.trigLeaderDeg = deg(L.arm.q(6)); M.trigFollowerDeg = deg(F.arm.q(6));
    hand.fingerQ = rad(5);                               // squeeze lightly, below the object: no resistance expected
    runFor(2.0, nothing);
    M.fingerFree = std::fabs(hand.fingerTau);
    hand.fingerOn = false;
    runFor(0.5, nothing);

    // P5: the state stream drops out for 0.5 s
    double l0[3]; L.arm.tcp(l0);
    rcb.streamMuted = true;
    runFor(0.5, [&] { double p[3]; L.arm.tcp(p); M.lossJumpMm = std::fmax(M.lossJumpMm, 1e3 * dist3(p, l0)); });
    const raft::TeleopSession::Stage during = session.stage();
    rcb.streamMuted = false;
    // the session waits for the operator (autoReclaim off): 1 s later it must still be LOST; then "resume"
    runFor(1.0, [&] { double p[3]; L.arm.tcp(p); M.lossJumpMm = std::fmax(M.lossJumpMm, 1e3 * dist3(p, l0)); });
    M.waitedForResume = session.stage() == raft::TeleopSession::Stage::Lost ? 1.0 : 0.0;
    session.resume(L.time());
    const double tBack = L.time();
    runFor(8.0, [&] {
        double p[3]; L.arm.tcp(p); M.lossJumpMm = std::fmax(M.lossJumpMm, 1e3 * dist3(p, l0));
        if (M.recoupleS < 0 && session.stage() == raft::TeleopSession::Stage::Coupled) M.recoupleS = L.time() - tBack;
    });
    M.cmds = rcb.commandsReceived();
    if (verbose) std::printf("  during the outage the session was %s; after it: %s\n", raft::TeleopSession::stageName(during),
                             raft::TeleopSession::stageName(session.stage()));
    return M;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string leaderScene = defaultScene(argv[0], "scene.xml");
    const std::string followerScene = defaultScene(argv[0], "follower_scene.xml");
    std::printf("MuJoCo %s; leader %s\n", mj_versionString(), leaderScene.c_str());
    std::printf("Bilateral: firmware controller + teleop session + RTI link -> emulated RCB runtime -> MuJoCo follower\n");
    std::printf("  follower: PD 20 Nm/rad (wrist 8) + gravity, 3 Nm motors, block 30 mm below its start TCP, gripper stops at 10 deg\n");
    const double lat[] = {0.002, 0.010, 0.030};
    const bool verbose = argc > 1;
    raft::ControlConfig stiff = raft::defaultControlConfig();
    const float stiffMax[7] = {0.30f, 0.35f, 0.30f, 0.12f, 0.12f, 0.10f, 0.15f};
    for (int i = 0; i < 7; ++i) { stiff.joint[i].bilatK *= 4.0f; stiff.joint[i].bilatD *= 2.0f; stiff.joint[i].bilatTauMaxNm = stiffMax[i]; }
    raft::ControlConfig stiffNoPc = stiff;
    stiffNoPc.passivity = false;
    raft::ControlConfig force = raft::defaultControlConfig();
    force.extGain = 0.05f;                                // contact reflection: 0.05 leader Nm per PAR6 Nm
    for (int i = 0; i < 7; ++i) force.joint[i].extTauMaxNm = stiffMax[i];
    raft::ControlConfig harsh = stiff;                    // well past the default: where the passivity damper matters
    for (int i = 0; i < 7; ++i) { harsh.joint[i].bilatK *= 2.0f; harsh.joint[i].bilatD *= 0.25f; }
    raft::ControlConfig harshNoPc = harsh;
    harshNoPc.passivity = false;
    const struct { const char* name; raft::ControlConfig cfg; } cfgs[] = {
        {"default coupling (J2: Kb 1.2 Nm/rad, clamp 0.20 Nm), passivity on", raft::defaultControlConfig()},
        {"stiffer coupling (Kb x4, Db x2, J2 clamp 0.35 Nm), passivity on", stiff},
        {"stiffer coupling, passivity off", stiffNoPc},
        {"default coupling + contact force reflection from t_ext (gain 0.05)", force},
        {"harsh coupling (Kb x8, Db x0.5), passivity on", harsh},
        {"harsh coupling (Kb x8, Db x0.5), passivity off", harshNoPc},
    };
    Metrics m[3];
    for (const auto& c : cfgs) {
    std::printf("\n  %s\n", c.name);
    std::printf("  %-22s %9s %10s %10s\n", "one-way latency", "2 ms", "10 ms", "30 ms");
    for (int k = 0; k < 3; ++k) {
        simrcb::NetConfig n; n.latencyS = lat[k]; n.jitterS = 0.2 * lat[k]; n.udpLoss = 0.01;
        m[k] = runScenario(leaderScene, followerScene, n, verbose, c.cfg);
    }
    auto row = [&](const char* name, const char* unit, double Metrics::*f) {
        std::printf("  %-22s", name);
        for (int k = 0; k < 3; ++k) std::printf(" %7.2f %-3s", m[k].*f, unit);
        std::printf("\n");
    };
    row("coupled after", "s", &Metrics::tCoupled);
    row("align error at claim", "deg", &Metrics::alignErrDeg);
    row("free: follower lag", "mm", &Metrics::freeTrackMm);
    row("free: hand force mean", "N", &Metrics::freeHandMean);
    row("free: hand force peak", "N", &Metrics::freeHandPeak);
    row("contact: follower drop", "mm", &Metrics::followerDropMm);
    row("contact: hand force", "N", &Metrics::contactHandN);
    row("contact: leader-follower", "mm", &Metrics::contactGapMm);
    row("release: max speed", "r/s", &Metrics::releaseSpeed);
    row("release: final gap", "mm", &Metrics::releaseGapMm);
    row("trigger blocked: finger", "Nm", &Metrics::fingerBlocked);
    row("trigger: leader angle", "deg", &Metrics::trigLeaderDeg);
    row("trigger: gripper angle", "deg", &Metrics::trigFollowerDeg);
    row("trigger free: finger", "Nm", &Metrics::fingerFree);
    row("link loss: leader moved", "mm", &Metrics::lossJumpMm);
    row("link loss: waited (1=yes)", "", &Metrics::waitedForResume);
    row("link loss: recoupled in", "s", &Metrics::recoupleS);
    row("reflected torque peak", "Nm", &Metrics::extPeakNm);
    row("passivity: lowest credit", "J", &Metrics::pcMinJ);
    }
    std::printf("  (1 %% UDP loss, jitter 20 %% of latency; 'release: max speed' is the leader's largest joint speed 0.5-2 s after letting go;\n"
                "   'recoupled in' counts from the operator's resume; 'lowest credit' < 0 means the passivity damper was working)\n");
    return 0;
}
