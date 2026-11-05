// Host unit test for the firmware's model and controller (no MuJoCo, no hardware).
// Build and run: tools\sim\build.cmd   then   tools\sim\out\test_model.exe
#include <chrono>
#include <cmath>
#include <cstdio>

#include "control.h"
#include "dynamics.h"
#include "kinematics.h"
#include "model_selftest.h"

static int g_fail = 0;
static void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

int main()
{
    std::printf("=== Model self-test against golden_vectors.h ===\n");
    const raft::ModelSelfTestResult r = raft::runModelSelfTest();
    std::printf("  %d cases: TCP pos %.2e m, rot %.2e, gravity %.2e Nm, rpy %.2e rad\n",
                r.cases, r.maxPosErr, r.maxRotErr, r.maxTauErr, r.maxRpyErr);
    check(r.pass, "model matches the generator within the boot thresholds");

    std::printf("=== Controller ===\n");
    raft::ControlConfig cfg = raft::defaultControlConfig();
    cfg.rampTimeS = 0.3f;                 // bring-up setting, to exercise the ramp
    raft::Controller ctl(cfg);
    raft::ControlInput in = {};
    for (int i = 0; i < raft::kNumJoints; ++i) in.tempC[i] = 35.0f;
    raft::ControlOutput out;

    ctl.step(in, 0.005f, out);
    bool zero = true;
    for (int i = 0; i < raft::kNumJoints; ++i) zero = zero && out.raw[i] == 0;
    check(zero, "limp sends zero current");

    ctl.setMode(raft::Mode::Gravity);
    ctl.step(in, 0.005f, out);
    check(std::fabs(out.tauCmd[1]) < 0.2f * std::fabs(out.tauGravity[1]), "first tick after enabling is still ramping");
    for (int k = 0; k < 100; ++k) ctl.step(in, 0.005f, out);
    check(std::fabs(out.tauCmd[1] - out.tauGravity[1]) < 1e-6f, "after the ramp J2 commands the full gravity torque at standby");
    check(std::fabs(out.tauGravity[1] + 0.1596f) < 2e-4f, "standby J2 gravity torque is -0.160 Nm");
    const int expectRaw = (int)std::lround(raft::kCurrentSign * cfg.joint[1].dir * out.tauGravity[1] / cfg.joint[1].ktNmPerA * 1000.0f / cfg.mAPerUnit);
    check(out.raw[1] == expectRaw, "raw register value = servo sign x tau / Kt / 6.5 mA");

    in.q[1] = 1.5708f; in.q[2] = 1.5708f;   // stretched: J2 needs 0.387 Nm
    for (int k = 0; k < 200; ++k) ctl.step(in, 0.005f, out);
    check(std::fabs(out.tauCmd[1] - out.tauGravity[1]) < 1e-5f && std::fabs(out.tauCmd[1]) > 0.38f,
          "stretched arm: J2 commands the full 0.387 Nm (no J2 clamp)");

    in.tempC[1] = 65.0f;
    ctl.step(in, 0.005f, out);
    check(std::fabs(out.derate[1] - 0.5f) < 1e-6f, "65 C derates J2 to 50 %");
    in.tempC[1] = 72.0f;
    ctl.step(in, 0.005f, out);
    check(out.raw[1] == 0, "72 C: J2 torque off");

    ctl.setMode(raft::Mode::Limp);
    ctl.step(in, 0.005f, out);
    zero = true;
    for (int i = 0; i < raft::kNumJoints; ++i) zero = zero && out.raw[i] == 0;
    check(zero, "back to limp drops all current at once");

    float th[6], qb[6];
    const float q6[6] = {0.1f, -0.2f, 0.3f, 2.9f, -3.0f, 1.0f};
    raft::cadToDh(q6, th);
    raft::dhToCad(th, qb);
    bool rt = true;
    for (int i = 0; i < 6; ++i) rt = rt && std::fabs(raft::wrapPi(qb[i] - q6[i])) < 1e-6f;
    check(rt && std::fabs(th[1] - (-0.2f - 1.5707963f)) < 1e-6f, "CAD <-> PAR6 DH angle conversion");
    {
        // just either side of the standby pose J3 and J6 must stay near +pi (the PAR6 has J3 in 1.90..6.65 rad)
        const float a[6] = {0.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.001f}, b[6] = {0.0f, 0.0f, -0.001f, 0.0f, 0.0f, -0.001f};
        float ta[6], tb[6];
        raft::cadToDh(a, ta);
        raft::cadToDh(b, tb);
        char msg[120];
        std::snprintf(msg, sizeof(msg), "DH angles stay continuous through standby (J3 %.4f / %.4f rad)", tb[2], ta[2]);
        check(std::fabs(ta[2] - tb[2]) < 0.01f && std::fabs(ta[5] - tb[5]) < 0.01f && ta[2] > 1.904f && ta[2] < 6.65f, msg);
    }

    std::printf("=== Jacobian, spring, MOVE, auto-rest, BILATERAL ===\n");
    {
        const float q0[7] = {0.3f, 0.5f, -0.4f, 0.7f, -0.6f, 0.9f, 0.1f};
        raft::Frame T[raft::kNumFrames], Tp[raft::kNumFrames], Tm[raft::kNumFrames];
        raft::forwardKinematics(q0, T);
        float J[6][6];
        raft::tcpJacobian(T, J);
        float worst = 0.0f;
        for (int i = 0; i < 6; ++i) {
            float qp[7], qm[7];
            for (int k = 0; k < 7; ++k) { qp[k] = q0[k]; qm[k] = q0[k]; }
            const float h = 1e-3f;
            qp[i] += h; qm[i] -= h;
            raft::forwardKinematics(qp, Tp); raft::forwardKinematics(qm, Tm);
            for (int rr = 0; rr < 3; ++rr) worst = std::fmax(worst, std::fabs((Tp[6].p[rr] - Tm[6].p[rr]) / (2 * h) - J[rr][i]));
            float dR[9];                                  // angular part: skew(w) = dR * R^T
            for (int k = 0; k < 9; ++k) dR[k] = (Tp[6].R[k] - Tm[6].R[k]) / (2 * h);
            const float* R = T[6].R;
            auto W = [&](int a, int b) { return dR[3 * a] * R[3 * b] + dR[3 * a + 1] * R[3 * b + 1] + dR[3 * a + 2] * R[3 * b + 2]; };
            worst = std::fmax(worst, std::fabs(W(2, 1) - J[3][i]));
            worst = std::fmax(worst, std::fabs(W(0, 2) - J[4][i]));
            worst = std::fmax(worst, std::fabs(W(1, 0) - J[5][i]));
        }
        char msg[96];
        std::snprintf(msg, sizeof(msg), "TCP Jacobian matches finite differences (max diff %.1e)", worst);
        check(worst < 2e-3f, msg);
    }
    {
        raft::SpringModel sp = cfg.spring;
        sp.type = raft::SpringType::ZeroLengthCable; sp.k = 363.0f; sp.a = 0.025f; sp.b = 0.025f;
        const float t = raft::springTorque(sp, 0.349f);
        check(std::fabs(t + 363.0f * 0.025f * 0.025f * std::sin(0.349f)) < 1e-6f && t < 0.0f, "zero-length spring torque = -k a b sin(q2), same sign as the gravity load");
        sp.type = raft::SpringType::OneSidedTorsion; sp.k = 0.19f; sp.q0 = 0.0f;
        check(raft::springTorque(sp, -0.3f) == 0.0f && std::fabs(raft::springTorque(sp, 0.5f) + 0.095f) < 1e-6f, "one-sided torsion spring loads only for forward tilt");
    }
    {
        raft::Controller mc(raft::defaultControlConfig());
        raft::ControlInput mi = {};
        for (int i = 0; i < raft::kNumJoints; ++i) mi.tempC[i] = 35.0f;
        mc.setMode(raft::Mode::Gravity);
        mc.step(mi, 0.005f, out);
        const float goal[7] = {0.2f, 0.3f, -0.2f, 0.4f, 0.0f, 0.0f, 0.0f};
        check(mc.moveTo(goal, 0.5f) && mc.mode() == raft::Mode::Move, "moveTo switches to MOVE");
        bool done = false; int ticks = 0;
        while (!done && ticks < 1000) {                   // ideal tracking: q follows the reference
            mc.step(mi, 0.005f, out);
            for (int i = 0; i < 7; ++i) mi.q[i] = out.qRef[i];
            done = (out.events & raft::EV_MOVE_DONE) != 0; ++ticks;
        }
        char msg[96];
        std::snprintf(msg, sizeof(msg), "minimum-jerk move ends on the goal (%.2f s; expect 1.875*0.4/0.5 = 1.50 s)", ticks * 0.005f);
        check(done && std::fabs(mi.q[3] - 0.4f) < 1e-4f && std::fabs(ticks * 0.005f - 1.5f) < 0.02f, msg);
        check(mc.mode() == raft::Mode::Move, "after a commanded move the arm holds the goal (stays in MOVE)");
        const float back[7] = {0, 0, 0, 0, 0, 0, 0};
        mc.moveTo(back, 0.5f);
        int t2 = 0; bool over = false;
        while (!over && t2 < 1000) { mc.step(mi, 0.005f, out); over = (out.events & raft::EV_OVERRIDDEN) != 0; ++t2; }   // q held still
        check(over && mc.mode() == raft::Mode::Gravity, "a held arm aborts the move to GRAVITY (someone grabbed it)");
    }
    {
        raft::ControlConfig ac = raft::defaultControlConfig();
        ac.autoRest.enabled = true; ac.autoRest.idleTimeS = 1.0f;
        raft::Controller rc(ac);
        raft::ControlInput ri = {};
        for (int i = 0; i < raft::kNumJoints; ++i) ri.tempC[i] = 35.0f;
        ri.q[1] = 0.35f;                                  // handle in front, left alone
        rc.setMode(raft::Mode::Gravity);
        int k = 0; bool started = false;
        while (!started && k < 400) { rc.step(ri, 0.005f, out); started = (out.events & raft::EV_AUTO_REST) != 0; ++k; }
        check(started && std::fabs(k * 0.005f - 1.0f) < 0.02f && rc.mode() == raft::Mode::Move, "left alone 1 s -> automatic move to the rest pose");
        bool parked = false; k = 0;
        while (!parked && k < 2000) { rc.step(ri, 0.005f, out); for (int i = 0; i < 7; ++i) ri.q[i] = out.qRef[i]; parked = (out.events & raft::EV_MOVE_DONE) != 0; ++k; }
        check(parked && rc.mode() == raft::Mode::Gravity && std::fabs(ri.q[2] + 1.2217f) < 1e-3f, "parks at the rest pose (forearm 70 deg down) and is weightless again");
        for (int j = 0; j < 400; ++j) rc.step(ri, 0.005f, out);
        check(rc.mode() == raft::Mode::Gravity, "does not re-trigger while parked");
    }
    {
        raft::Controller bc(raft::defaultControlConfig());
        raft::ControlInput bi = {};
        for (int i = 0; i < raft::kNumJoints; ++i) bi.tempC[i] = 35.0f;
        bc.setMode(raft::Mode::Bilateral);
        bi.followerFresh = true;
        bi.qFollower[1] = 0.05f;                          // follower 2.9 deg ahead on J2
        for (int k = 0; k < 20; ++k) { bc.step(bi, 0.005f, out); bi.followerFresh = (k % 2 == 0); }
        const float coupling = out.tauCmd[1] - out.tauGravity[1];
        check(std::fabs(coupling - 1.2f * 0.05f) < 2e-3f, "BILATERAL adds Kb * (q_f - q) on J2");
        bi.qFollower[1] = 1.0f; bi.followerFresh = true;
        for (int k = 0; k < 20; ++k) bc.step(bi, 0.005f, out);
        check(std::fabs(out.tauCmd[1] - out.tauGravity[1] - 0.20f) < 2e-3f, "coupling torque is clamped (J2 0.20 Nm)");
        bi.followerFresh = false;
        bool lost = false;
        for (int k = 0; k < 40 && !lost; ++k) { bc.step(bi, 0.005f, out); lost = (out.events & raft::EV_FOLLOWER_LOST) != 0; }
        for (int k = 0; k < 10; ++k) bc.step(bi, 0.005f, out);
        check(lost && std::fabs(out.tauCmd[1] - out.tauGravity[1]) < 1e-5f, "follower silent > 0.1 s: coupling off, gravity only");
    }
    std::printf("=== Friction compensation, barriers, contact reflection, orientation lock, guide, passivity ===\n");
    {
        raft::ControlInput hi = {};
        for (int i = 0; i < raft::kNumJoints; ++i) hi.tempC[i] = 35.0f;
        raft::ControlConfig fcfg = raft::defaultControlConfig();
        fcfg.frictionScale = 1.0f;
        for (int i = 0; i < raft::kNumJoints; ++i) fcfg.joint[i].frictionNm = 0.02f;
        raft::Controller fcl(fcfg);
        fcl.setMode(raft::Mode::Gravity);
        hi.qd[0] = 1.0f;
        for (int k = 0; k < 5; ++k) fcl.step(hi, 0.005f, out);
        const float fr = out.tauCmd[0] - out.tauGravity[0];
        hi.qd[0] = 0.0f;
        for (int k = 0; k < 5; ++k) fcl.step(hi, 0.005f, out);
        check(std::fabs(fr - 0.02f * std::tanh(1.0f / 0.2f)) < 1e-4f && std::fabs(out.tauCmd[0] - out.tauGravity[0]) < 1e-5f,
              "friction compensation: Fc tanh(qd / v_s) in the direction of motion, nothing at rest");

        raft::Controller bcl(raft::defaultControlConfig());
        bcl.setMode(raft::Mode::Gravity);
        bcl.setBarrier(1, -0.5f, 0.5f);
        hi.q[1] = 0.46f;                                  // 0.10 rad into the 0.14 rad margin
        for (int k = 0; k < 5; ++k) bcl.step(hi, 0.005f, out);
        const float bar = out.tauAssist[1];
        hi.q[1] = 0.0f;
        for (int k = 0; k < 5; ++k) bcl.step(hi, 0.005f, out);
        check(std::fabs(bar + 1.5f * (0.46f - (0.5f - 0.13962634f))) < 1e-4f && std::fabs(out.tauAssist[1]) < 1e-6f, "barrier: K x depth inside the margin, nothing in the middle");

        raft::ControlConfig ecfg = raft::defaultControlConfig();
        ecfg.extGain = 0.05f;
        raft::Controller ecl(ecfg);
        ecl.setMode(raft::Mode::Bilateral);
        hi.followerFresh = true; hi.extValid = true;
        hi.tauExtFollower[1] = 2.3f;                      // PAR6's J2 motor pushes 2.3 Nm against something
        for (int k = 0; k < 100; ++k) { ecl.step(hi, 0.005f, out); hi.followerFresh = hi.extValid = (k % 4 == 0); }
        const float ext = out.tauExt[1];
        hi.tauExtFollower[1] = 0.2f;                      // inside the 0.3 Nm deadband
        for (int k = 0; k < 200; ++k) { ecl.step(hi, 0.005f, out); hi.followerFresh = hi.extValid = (k % 4 == 0); }
        check(std::fabs(ext + 0.05f * 2.0f) < 2e-3f && std::fabs(out.tauExt[1]) < 1e-3f,
              "contact reflection: -gain x (t_ext - deadband) = -0.10 Nm; below the deadband nothing");
        hi.followerFresh = hi.extValid = false; hi.tauExtFollower[1] = 0.0f;

        raft::Controller ocl(raft::defaultControlConfig());
        ocl.setMode(raft::Mode::Gravity);
        for (int i = 0; i < raft::kNumJoints; ++i) hi.q[i] = 0.0f;
        ocl.setOrientLock(true);
        ocl.step(hi, 0.005f, out);                        // anchored here
        hi.q[5] = 0.1f;                                   // the handle rolled 0.1 rad
        ocl.step(hi, 0.005f, out);
        check(std::fabs(out.tauAssist[5] + 0.2f * 0.1f) < 1e-3f, "orientation lock: a 0.1 rad roll is pushed back with 0.2 Nm/rad on J6");
        hi.q[5] = 0.0f;
        // J1 turns the handle too; with the wrist bent (J5 = 0 is the wrist singularity: J4 and J6 line up and
        // cannot make a yaw) the wrist joints answer the turn
        raft::Controller ocl2(raft::defaultControlConfig());
        ocl2.setMode(raft::Mode::Gravity);
        hi.q[4] = 0.6f;
        ocl2.setOrientLock(true);
        ocl2.step(hi, 0.005f, out);
        hi.q[0] = 0.3f;
        ocl2.step(hi, 0.005f, out);
        check(std::fabs(out.tauAssist[3]) + std::fabs(out.tauAssist[4]) + std::fabs(out.tauAssist[5]) > 0.02f && std::fabs(out.tauAssist[0]) < 1e-6f,
              "orientation lock, wrist bent: turning J1 makes the wrist joints push against the turn (J1 itself is free)");
        hi.q[0] = 0.0f; hi.q[4] = 0.0f;

        raft::Controller gcl(raft::defaultControlConfig());
        gcl.setMode(raft::Mode::Gravity);
        gcl.setGuide(raft::Guide::PlaneHorizontal);
        gcl.step(hi, 0.005f, out);
        hi.q[0] = 0.3f;                                   // J1 keeps the wrist centre's height
        gcl.step(hi, 0.005f, out);
        const float horiz = out.tauAssist[0];
        hi.q[0] = 0.0f; hi.q[1] = 0.1f;                   // J2 forward lowers it
        gcl.step(hi, 0.005f, out);
        check(std::fabs(horiz) < 1e-4f && out.tauAssist[1] < -0.01f, "horizontal-plane guide: free around J1, J2 is pushed back up");
        hi.q[1] = 0.0f;

        raft::ControlConfig pon = raft::defaultControlConfig(), poff = pon;
        poff.passivity = false;
        raft::Controller pc1(pon), pc2(poff);
        pc1.setMode(raft::Mode::Bilateral); pc2.setMode(raft::Mode::Bilateral);
        hi.followerFresh = true;
        hi.qFollower[1] = 0.1f;                           // the coupling pulls the leader the way it is already moving:
        hi.qd[1] = 0.5f;                                  // it gives energy, nothing absorbed it before
        raft::ControlOutput o2;
        for (int k = 0; k < 20; ++k) { pc1.step(hi, 0.005f, out); pc2.step(hi, 0.005f, o2); }
        check(out.pcEnergyJ < 0.0f && out.tauCmd[1] < o2.tauCmd[1] - 0.01f, "passivity: a coupling that only gives energy gets damped");
    }

    std::printf("=== Cost on this PC (the ESP32 will be ~50-100x slower) ===\n");
    float q[7] = {0.3f, 0.4f, -0.5f, 0.6f, -0.7f, 0.8f, 0.2f}, tau[7], acc = 0.0f;
    raft::Frame T[raft::kNumFrames];
    const int N = 200000;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < N; ++k) {
        q[0] = 1e-6f * (float)k;
        raft::forwardKinematics(q, T);
        raft::gravityTorque(T, tau);
        acc += tau[1];
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / N;
    std::printf("  FK + gravity: %.3f us per call (checksum %.3f)\n", us, acc);

    std::printf("%s\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return g_fail ? 1 : 0;
}
