// RAFT leader simulator: the firmware's controller (src/main/control.cpp, kinematics.cpp, dynamics.cpp,
// the same files the ESP32 builds) drives the MuJoCo model of the arm through a simulated HLS3606 bus.
// Dev tool: nothing here runs on the arm. Build: tools\sim\build.cmd   Run: tools\sim\out\raft_sim.exe
//
// Groups: gravity (hold, calibration errors, guiding, enabling), spring (J2 counterbalance),
// thermal (15-minute sessions), move (self-driven moves, grabbing, automatic rest), friction (compensation).
// Run one group with e.g. "raft_sim.exe thermal".
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "control.h"
#include "model_selftest.h"
#include "sim_core.h"

using namespace sim;

namespace {

struct Pose { const char* name; double q[7]; };

const Pose kPoses[] = {
    {"standby",          {0, 0, 0, 0, 0, 0, 0}},
    {"handle in front",  {0, rad(20), 0, 0, 0, 0, 0}},
    {"reach forward",    {0, rad(45), rad(45), 0, 0, 0, 0}},
    {"stretched",        {0, rad(90), rad(90), 0, 0, 0, 0}},
    {"forearm down 70",  {0, 0, rad(-70), 0, 0, 0, 0}},
    {"wrist twisted",    {rad(35), rad(15), rad(-10), rad(60), rad(-45), rad(90), rad(25)}},
    {"side reach",       {rad(-70), rad(30), rad(10), rad(-40), rad(70), rad(-30), 0}},
};
const int kNumPoses = sizeof(kPoses) / sizeof(kPoses[0]);

raft::ControlConfig openClamps()
{
    raft::ControlConfig c = raft::defaultControlConfig();
    for (int i = 0; i < 7; ++i) c.joint[i].tauMaxNm = 0.58f;
    return c;
}

raft::ControlConfig withSpring(raft::ControlConfig c, float k)
{
    c.spring.type = raft::SpringType::ZeroLengthCable;
    c.spring.k = k; c.spring.a = 0.013f; c.spring.b = 0.020f; c.spring.scale = 1.0f;   // must match SPRING_A/B in gen_model.py
    return c;
}

struct HoldResult { double driftDeg = 0, tcpDriftMm = 0, tcpDzMm = 0, j2mA = 0, j3mA = 0; };

// Hold the arm while the compensation comes up, let go, measure how far it wanders in 5 s.
HoldResult hold(Leader& L, const Pose& p, const raft::ControlConfig& cfg)
{
    raft::Controller ctl(cfg);
    ctl.setMode(raft::Mode::Gravity);
    L.reset(p.q);
    L.run(0.5, ctl, nullptr, nullptr, true);
    HoldResult r;
    r.j2mA = 1e3 * L.bus.current[1]; r.j3mA = 1e3 * L.bus.current[2];
    double p0[3]; L.arm.tcp(p0);
    L.run(5.0, ctl);
    double p1[3]; L.arm.tcp(p1);
    for (int i = 0; i < 6; ++i) r.driftDeg = std::fmax(r.driftDeg, std::fabs(deg(L.arm.q(i) - p.q[i])));
    r.tcpDriftMm = 1e3 * dist3(p1, p0);
    r.tcpDzMm = 1e3 * (p1[2] - p0[2]);
    return r;
}

void holdTable(Leader& L, const raft::ControlConfig& cfg, double friction, double mass, const Truth& truth, double springK, const char* title)
{
    L.arm.configure(friction, mass);
    L.arm.setSpringK(springK);
    L.bus.truth = truth;
    std::printf("\n--- %s ---\n", title);
    std::printf("  %-18s %12s %11s %10s %9s %9s\n", "pose", "joint drift", "TCP drift", "TCP dz", "J2 mA", "J3 mA");
    for (int k = 0; k < kNumPoses; ++k) {
        const HoldResult r = hold(L, kPoses[k], cfg);
        std::printf("  %-18s %8.3f deg %8.2f mm %7.2f mm %9.0f %9.0f\n", kPoses[k].name, r.driftDeg, r.tcpDriftMm, r.tcpDzMm, r.j2mA, r.j3mA);
    }
    L.arm.setSpringK(0.0);
    L.bus.truth = Truth();
}

void groupGravity(Leader& L, const std::string& outDir)
{
    const Truth exact;
    holdTable(L, openClamps(), 0.0, 1.0, exact, 0.0, "G1. Hold 5 s after letting go, NO friction (model + bus quantization only)");
    holdTable(L, raft::defaultControlConfig(), -1, 1.0, exact, 0.0, "G2. Same with the MJCF friction estimate, default config (J2 unclamped)");
    holdTable(L, openClamps(), -1, 1.10, exact, 0.0, "G3. Real arm 10 % heavier than the CAD model");
    Truth weak; weak.ktNmPerA *= 0.95;
    holdTable(L, openClamps(), -1, 1.0, weak, 0.0, "G4. Real Kt 5 % lower than assumed");

    L.arm.configure(-1, 1.0);
    std::printf("\n--- G5. Hand-guiding from standby: drag the handle 60 mm in 1 s, let go (default config) ---\n");
    std::printf("  %-10s %12s %12s %14s %18s\n", "direction", "peak force", "mean force", "lag at end", "drift 2 s after");
    const struct { const char* name; double d[3]; } moves[] = {
        {"sideways", {0, -0.06, 0}}, {"down", {0, 0, -0.06}}, {"up", {0, 0, 0.06}}, {"forward", {0.06, 0, 0}}, {"back", {-0.06, 0, 0}},
    };
    for (const auto& mv : moves) {
        raft::Controller ctl(raft::defaultControlConfig()); ctl.setMode(raft::Mode::Gravity);
        L.reset(kPoses[0].q);
        L.run(0.5, ctl, nullptr, nullptr, true);
        Hand h; L.arm.tcp(h.p0);
        std::memcpy(h.delta, mv.d, sizeof(h.delta));
        h.t0 = 0.6; h.t1 = 1.6; h.tRelease = 1.8;
        L.run(1.3, ctl, &h);
        double pr[3]; L.arm.tcp(pr);
        double tg[3]; for (int i = 0; i < 3; ++i) tg[i] = h.p0[i] + h.delta[i];
        L.run(2.0, ctl, &h);
        double pe[3]; L.arm.tcp(pe);
        std::printf("  %-10s %10.2f N %10.2f N %11.1f mm %15.2f mm\n", mv.name, h.peak, h.n ? h.sum / h.n : 0.0, 1e3 * dist3(pr, tg), 1e3 * dist3(pe, pr));
    }

    std::printf("\n--- G6. Turning compensation on while the arm is free, at 'handle in front' ---\n");
    std::printf("  %-34s %18s %16s\n", "ramp / slew", "TCP drop at 0.1 s", "TCP drop at 1 s");
    const struct { float ramp, slew; const char* name; } en[] = {
        {0.3f, 5.0f, "0.3 s ramp, 5 Nm/s slew (old plan)"}, {0.0f, 5.0f, "no ramp, 5 Nm/s slew"},
        {0.0f, 20.0f, "no ramp, 20 Nm/s slew (default)"}, {0.0f, 50.0f, "no ramp, 50 Nm/s slew"},
    };
    for (const auto& e : en) {
        raft::ControlConfig c = raft::defaultControlConfig(); c.rampTimeS = e.ramp;
        for (int i = 0; i < 7; ++i) c.joint[i].slewNmPerS = e.slew;
        raft::Controller ctl(c); ctl.setMode(raft::Mode::Gravity);
        L.reset(kPoses[1].q);
        double p0[3]; L.arm.tcp(p0);
        L.run(0.1, ctl);
        double pa[3]; L.arm.tcp(pa);
        L.run(0.9, ctl);
        double pb[3]; L.arm.tcp(pb);
        std::printf("  %-34s %15.1f mm %13.1f mm\n", e.name, 1e3 * (p0[2] - pa[2]), 1e3 * (p0[2] - pb[2]));
    }
    {
        raft::Controller ctl(raft::defaultControlConfig());
        L.reset(kPoses[0].q);
        double p0[3]; L.arm.tcp(p0);
        double t = 0, p[3];
        for (; t < 3.0; t += 0.01) { L.run(0.01, ctl); L.arm.tcp(p); if (p0[2] - p[2] > 0.05) break; }
        std::printf("\n--- G7. Sanity: torque off at standby -> TCP falls 50 mm in %.2f s ---\n", t);
    }
    (void)outDir;
}

void groupSpring(Leader& L)
{
    std::printf("\n=== Spring test set (federnshop, 0.8 mm wire) on the cable balancer, anchor a = 13 mm, eyelet b = 20 mm ===\n");
    std::printf("    (MuJoCo tendon = the real spring; firmware SpringModel = what the firmware believes)\n");
    const Truth exact;
    holdTable(L, raft::defaultControlConfig(), -1, 1.0, exact, 0.0, "S0. No spring (reference)");
    const struct { const char* code; float k; } set[] = {{"Z-075NI", 565.0f}, {"Z-078AI", 718.0f}, {"Z-078I", 824.0f}, {"Z-077AI", 1027.0f}};
    for (const auto& s : set) {
        char title[120];
        std::snprintf(title, sizeof(title), "%s, %.3f N/mm = %.0f %% balance, modelled exactly", s.code, s.k / 1000, 100.0 * s.k * 0.013 * 0.020 / 0.2269);
        holdTable(L, withSpring(raft::defaultControlConfig(), s.k), -1, 1.0, exact, s.k, title);
    }
    holdTable(L, withSpring(raft::defaultControlConfig(), 824.0f), -1, 1.0, exact, 1.10 * 824.0,
              "Z-078I 10 % stiffer than its catalog rate (untrimmed)");
    holdTable(L, raft::defaultControlConfig(), -1, 1.0, exact, 824.0, "Z-078I fitted but the firmware does not know (model off)");
}

// 15-minute session: the operator holds working poses and lets go, in 90 s cycles.
struct SessionResult { double j2Peak = 0, j3Peak = 0, j2DerateS = 0, j2RmsmA = 0, j2End = 0, j3End = 0; int restMoves = 0; };

SessionResult session(Leader& L, const raft::ControlConfig& cfg, double springK, bool continuousHold, double minutes, double wobbleDeg = 3.0)
{
    L.arm.configure(-1, 1.0);
    L.arm.setSpringK(springK);
    raft::Controller ctl(cfg);
    ctl.setMode(raft::Mode::Gravity);
    L.reset(kPoses[0].q);
    Operator op;
    struct Seg { double dur; bool held; int pose; };
    const Seg cycle[] = {{30, true, 1}, {15, true, 2}, {15, true, 0}, {30, false, 0}};
    const Seg cont[] = {{90, true, 1}};
    const Seg* segs = continuousHold ? cont : cycle;
    const int nSeg = continuousHold ? 1 : 4;
    double cycleLen = 0; for (int s = 0; s < nSeg; ++s) cycleLen += segs[s].dur;

    SessionResult r;
    double sumI2 = 0; long n = 0;
    int curSeg = -1;
    double segStart = 0, from[7] = {0};
    const long steps = std::lround(minutes * 60.0 / L.dt());
    for (long k = 0; k < steps; ++k) {
        const double t = L.time();
        double tc = std::fmod(t, cycleLen);
        int s = 0; double acc = 0;
        while (s < nSeg - 1 && tc >= acc + segs[s].dur) { acc += segs[s].dur; ++s; }
        if (s != curSeg) { curSeg = s; segStart = t; for (int i = 0; i < 7; ++i) from[i] = L.arm.q(i); }
        op.active = segs[s].held;
        const double u = minJerk((t - segStart) / 2.0);              // 2 s to move to the next pose
        for (int i = 0; i < 7; ++i) op.q[i] = from[i] + u * (kPoses[segs[s].pose].q[i] - from[i]);
        if (wobbleDeg > 0) {                                         // a working hand is never perfectly still
            op.q[0] += rad(wobbleDeg) * std::sin(2 * kPi * 0.25 * t);
            op.q[1] += rad(0.7 * wobbleDeg) * std::sin(2 * kPi * 0.2 * t + 1.0);
        }
        L.stepOnce(ctl, nullptr, &op, false, [](raft::ControlInput&) {},
                   [&](const raft::ControlInput&, const raft::ControlOutput& o) { if (o.events & raft::EV_AUTO_REST) ++r.restMoves; });
        const double T2 = L.bus.thermal.T[1], T3 = L.bus.thermal.T[2];
        r.j2Peak = std::fmax(r.j2Peak, T2); r.j3Peak = std::fmax(r.j3Peak, T3);
        if (T2 > cfg.derateStartC) r.j2DerateS += L.dt();
        sumI2 += L.bus.current[1] * L.bus.current[1]; ++n;
    }
    r.j2RmsmA = 1e3 * std::sqrt(sumI2 / n);
    r.j2End = L.bus.thermal.T[1]; r.j3End = L.bus.thermal.T[2];
    L.arm.setSpringK(0.0);
    return r;
}

void groupThermal(Leader& L)
{
    const float k100 = 824.0f;                                   // Z-078I (94 % balance) at a = 13, b = 20 mm
    std::printf("\n=== Thermal: 15-minute sessions (heating model is an ESTIMATE: 131 K/W, 10 min, see sim_core.h) ===\n");
    std::printf("    cycle of 90 s: 30 s handle in front, 15 s reach forward, 15 s standby, 30 s let go\n");
    std::printf("  %-44s %9s %10s %10s %9s %9s %9s %6s\n", "setup", "J2 peak", "J2 >60 C", "J2 RMS", "J3 peak", "J2 end", "J3 end", "rests");
    raft::ControlConfig rest = raft::defaultControlConfig();
    rest.autoRest.enabled = true; rest.autoRest.idleTimeS = 10.0f;
    const struct { const char* name; raft::ControlConfig cfg; double k; bool cont; } runs[] = {
        {"no spring, no auto-rest", raft::defaultControlConfig(), 0.0, false},
        {"no spring, auto-rest after 10 s", rest, 0.0, false},
        {"spring Z-078I, no auto-rest", withSpring(raft::defaultControlConfig(), k100), k100, false},
        {"spring Z-078I + auto-rest", withSpring(rest, k100), k100, false},
        {"held in front for 15 min, no spring", raft::defaultControlConfig(), 0.0, true},
        {"held in front for 15 min, spring Z-078I", withSpring(raft::defaultControlConfig(), k100), k100, true},
    };
    for (const auto& run : runs) {
        const SessionResult r = session(L, run.cfg, run.k, run.cont, 15.0);
        std::printf("  %-44s %7.1f C %8.0f s %7.0f mA %7.1f C %7.1f C %7.1f C %6d\n", run.name, r.j2Peak, r.j2DerateS, r.j2RmsmA, r.j3Peak, r.j2End, r.j3End, r.restMoves);
    }
    // limitation: a hand that holds perfectly still looks exactly like a released arm
    const SessionResult still = session(L, rest, 0.0, false, 15.0, 0.0);
    std::printf("  %-44s %7.1f C %8.0f s %7.0f mA %7.1f C %7.1f C %7.1f C %6d\n", "auto-rest, operator holds perfectly still", still.j2Peak,
                still.j2DerateS, still.j2RmsmA, still.j3Peak, still.j2End, still.j3End, still.restMoves);
    std::printf("    (the operator holds the working poses with a 2-3 deg wobble; 10 let-go periods per session)\n");
}

void groupMove(Leader& L)
{
    L.arm.configure(-1, 1.0);
    std::printf("\n=== Self-driven moves (MOVE mode, firmware impedance K/D, friction estimate) ===\n");
    std::printf("  %-34s %9s %14s %14s %14s\n", "move", "duration", "max track err", "final err", "max speed");
    const struct { const char* name; int from, to; float v; } mv[] = {
        {"standby -> reach forward", 0, 2, 0.6f}, {"reach forward -> forearm down", 2, 4, 0.6f},
        {"forearm down -> wrist twisted", 4, 5, 0.6f}, {"wrist twisted -> standby (fast)", 5, 0, 1.2f},
    };
    for (const auto& m : mv) {
        raft::Controller ctl(raft::defaultControlConfig()); ctl.setMode(raft::Mode::Gravity);
        L.reset(kPoses[m.from].q);
        L.run(0.3, ctl, nullptr, nullptr, true);
        float goal[7]; for (int i = 0; i < 7; ++i) goal[i] = (float)kPoses[m.to].q[i];
        ctl.moveTo(goal, m.v);
        double maxErr = 0, maxV = 0, tDone = -1;
        const double t0 = L.time();
        while (L.time() - t0 < 10.0 && (tDone < 0 || L.time() < tDone + 1.0)) {
            L.stepOnce(ctl, nullptr, nullptr, false, [](raft::ControlInput&) {},
                       [&](const raft::ControlInput& in, const raft::ControlOutput& o) {
                           for (int i = 0; i < 7; ++i) { maxErr = std::fmax(maxErr, std::fabs(o.qRef[i] - in.q[i])); maxV = std::fmax(maxV, std::fabs(in.qd[i])); }
                           if ((o.events & raft::EV_MOVE_DONE) && tDone < 0) tDone = L.time();
                       });
        }
        double fin = 0; for (int i = 0; i < 7; ++i) fin = std::fmax(fin, std::fabs(L.arm.q(i) - goal[i]));
        std::printf("  %-34s %7.2f s %10.2f deg %10.2f deg %8.2f rad/s\n", m.name, tDone - t0, deg(maxErr), deg(fin), maxV);
    }

    {   // grabbing the arm mid-move
        raft::Controller ctl(raft::defaultControlConfig()); ctl.setMode(raft::Mode::Gravity);
        L.reset(kPoses[0].q);
        L.run(0.3, ctl, nullptr, nullptr, true);
        float goal[7]; for (int i = 0; i < 7; ++i) goal[i] = (float)kPoses[3].q[i];
        ctl.moveTo(goal, 0.6f);
        L.run(0.8, ctl);
        Hand h; L.arm.tcp(h.p0); h.t0 = L.time(); h.t1 = L.time() + 3.0; h.tRelease = L.time() + 3.0;   // grab and hold still
        const double tGrab = L.time();
        double tOver = -1;
        while (L.time() - tGrab < 2.0) {
            L.stepOnce(ctl, &h, nullptr, false, [](raft::ControlInput&) {},
                       [&](const raft::ControlInput&, const raft::ControlOutput& o) { if ((o.events & raft::EV_OVERRIDDEN) && tOver < 0) tOver = L.time(); });
        }
        std::printf("\n  grabbed mid-move: move aborted to GRAVITY %.2f s after the grab, peak hand force %.2f N, mode now %s\n",
                    tOver - tGrab, h.peak, ctl.mode() == raft::Mode::Gravity ? "GRAVITY" : "not gravity");
    }

    {   // automatic rest
        raft::ControlConfig c = raft::defaultControlConfig();
        c.autoRest.enabled = true; c.autoRest.idleTimeS = 5.0f;
        raft::Controller ctl(c); ctl.setMode(raft::Mode::Gravity);
        L.reset(kPoses[1].q);
        L.run(0.3, ctl, nullptr, nullptr, true);
        const double i2a = 1e3 * L.bus.current[1], i3a = 1e3 * L.bus.current[2];
        const double t0 = L.time();
        double tRest = -1, tPark = -1, err = 0;
        while (L.time() - t0 < 20.0 && (tPark < 0 || L.time() < tPark + 1.0)) {
            L.stepOnce(ctl, nullptr, nullptr, false, [](raft::ControlInput&) {},
                       [&](const raft::ControlInput&, const raft::ControlOutput& o) {
                           if ((o.events & raft::EV_AUTO_REST) && tRest < 0) tRest = L.time();
                           if ((o.events & raft::EV_MOVE_DONE) && tPark < 0 && tRest > 0) {
                               tPark = L.time();   // the pose error on arrival: after that the arm is limp (and the model has no stops)
                               for (int i = 0; i < 7; ++i) err = std::fmax(err, std::fabs(L.arm.q(i) - c.autoRest.qRest[i]));
                           }
                       });
        }
        std::printf("  auto-rest: left alone at 'handle in front' -> rest move starts after %.1f s, parked %.1f s later, pose error on arrival %.2f deg, then %s\n",
                    tRest - t0, tPark - tRest, deg(err), ctl.mode() == raft::Mode::Limp ? "LIMP" : ctl.mode() == raft::Mode::Gravity ? "GRAVITY" : "other");
        std::printf("             J2 current %.0f -> %.0f mA, J3 %.0f -> %.0f mA\n", i2a, 1e3 * L.bus.current[1], i3a, 1e3 * L.bus.current[2]);
    }
}

// Friction compensation (frictionScale x the Coulomb friction the torque calibration measures): how much less
// a hand has to push to guide the arm, and whether a released arm creeps.
void groupFriction(Leader& L)
{
    std::printf("\n=== Friction compensation (GRAVITY, friction = the MJCF estimate, assumed measured exactly) ===\n");
    std::printf("  scale  v_s rad/s   hand torque RMS J1/J2   lag RMS J1/J2      drift 3 s after letting go\n");
    L.arm.configure(-1, 1.0);
    const double runs[][2] = {{0.0, 0.05}, {0.5, 0.05}, {1.0, 0.05}, {0.5, 0.2}, {1.0, 0.2}, {0.5, 0.5}, {1.0, 0.5}};
    for (const auto& run : runs) {
        const double scale = run[0];
        raft::ControlConfig cfg = raft::defaultControlConfig();
        for (int i = 0; i < 7; ++i) cfg.joint[i].frictionNm = (float)L.arm.m->dof_frictionloss[L.arm.vadr[i]];
        cfg.frictionScale = (float)scale;
        cfg.frictionVelRadS = (float)run[1];
        raft::Controller ctl(cfg);
        ctl.setMode(raft::Mode::Gravity);
        L.reset(kPoses[1].q);
        Operator op;
        op.active = true;
        double sumT[2] = {0, 0}, sumE[2] = {0, 0};
        long n = 0;
        const double t0 = L.time();
        while (L.time() - t0 < 10.0) {                     // the hand sways J1 and J2 +-15 deg at 0.3 Hz
            const double t = L.time() - t0;
            for (int i = 0; i < 7; ++i) op.q[i] = kPoses[1].q[i];
            op.q[0] += rad(15.0) * std::sin(2 * kPi * 0.3 * t);
            op.q[1] += rad(15.0) * std::sin(2 * kPi * 0.3 * t + 0.7);
            L.stepOnce(ctl, nullptr, &op);
            if (t > 2.0) {
                for (int k = 0; k < 2; ++k) { sumT[k] += op.lastTau[k] * op.lastTau[k]; const double e = op.q[k] - L.arm.q(k); sumE[k] += e * e; }
                ++n;
            }
        }
        op.active = false;
        double q0[7];
        for (int i = 0; i < 7; ++i) q0[i] = L.arm.q(i);
        L.run(3.0, ctl, nullptr, &op);
        double drift = 0;
        for (int i = 0; i < 7; ++i) drift = std::fmax(drift, std::fabs(deg(L.arm.q(i) - q0[i])));
        std::printf("  %4.1f   %5.2f      %6.4f / %6.4f Nm      %5.2f / %5.2f deg    %6.2f deg\n", scale, run[1], std::sqrt(sumT[0] / n), std::sqrt(sumT[1] / n),
                    deg(std::sqrt(sumE[0] / n)), deg(std::sqrt(sumE[1] / n)), drift);
    }
    std::printf("  (scale 1 assumes the measured friction is exact; a real joint's friction varies, so start at 0.5)\n");
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string group = argc > 1 ? argv[1] : "all";
    const std::string scene = argc > 2 ? argv[2] : defaultScene(argv[0], "scene.xml");
    std::string outDir = argv[0];
    { const size_t cut = outDir.find_last_of("/\\"); outDir = cut == std::string::npos ? "." : outDir.substr(0, cut); }
    std::printf("MuJoCo %s, scene %s\n", mj_versionString(), scene.c_str());
    const raft::ModelSelfTestResult st = raft::runModelSelfTest();
    std::printf("firmware model self-test: %s (gravity err %.1e Nm)\n", st.pass ? "PASS" : "FAIL", st.maxTauErr);
    if (!st.pass) return 1;
    Leader L(scene);
    if (!L.arm.ok()) return 1;
    std::printf("MJCF servo placeholders (estimates): armature %.4f kg m^2, frictionloss %.3f Nm, damping %.3f Nm s/rad\n",
                L.arm.m->dof_armature[0], L.arm.m->dof_frictionloss[0], L.arm.m->dof_damping[0]);
    if (group == "all" || group == "gravity") groupGravity(L, outDir);
    if (group == "all" || group == "spring") groupSpring(L);
    if (group == "all" || group == "thermal") groupThermal(L);
    if (group == "all" || group == "move") groupMove(L);
    if (group == "all" || group == "friction") groupFriction(L);
    return 0;
}
