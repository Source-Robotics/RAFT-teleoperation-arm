#include "control.h"

#include <math.h>
#include <string.h>

#include "dynamics.h"

namespace raft {
namespace {

const float kPi = 3.14159265358979f;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

}  // namespace

const char* guideName(Guide g)
{
    switch (g) {
    case Guide::None: return "none";
    case Guide::PlaneHorizontal: return "plane_h";
    case Guide::PlaneTool: return "plane_tool";
    case Guide::AxisTool: return "axis_tool";
    case Guide::AxisVertical: return "axis_z";
    }
    return "?";
}

bool guideFromName(const char* s, Guide& g)
{
    const Guide all[] = {Guide::None, Guide::PlaneHorizontal, Guide::PlaneTool, Guide::AxisTool, Guide::AxisVertical};
    for (Guide x : all)
        if (!strcmp(s, guideName(x))) { g = x; return true; }
    return false;
}

ControlConfig defaultControlConfig()
{
    ControlConfig c;
    const float kt = 4.5f * 0.0980665f;      // datasheet 4.5 kg cm/A at the output
    const float stall = 6.0f * 0.0980665f;   // 0.588 Nm
    // Clamps: none below the servo's own stall torque on any joint (since 2026-11-02: 0.10-0.30 Nm
    // caps made Hold give way at about 300 mA). The coupling and assist terms keep their own clamps.
    const float tauMax[kNumJoints] = {stall, stall, stall, stall, stall, stall, stall};
    // MOVE: spring-like holding. K is set so the ~0.02 Nm gearbox friction (MJCF estimate) leaves
    // about 1 deg of error; softer wrist gains let friction alone reach the 8 deg grab threshold in
    // the simulator. D is near-critical for the servo armature (~0.003 kg m^2) plus the links.
    const float moveK[kNumJoints]  = {1.5f, 1.5f, 1.5f, 1.2f, 1.2f, 1.2f, 1.0f};
    const float moveD[kNumJoints]  = {0.20f, 0.25f, 0.20f, 0.12f, 0.12f, 0.10f, 0.08f};
    const float bilK[kNumJoints]   = {1.0f, 1.2f, 1.0f, 0.5f, 0.5f, 0.4f, 0.3f};
    const float bilD[kNumJoints]   = {0.05f, 0.06f, 0.05f, 0.025f, 0.025f, 0.02f, 0.015f};
    const float bilMax[kNumJoints] = {0.15f, 0.20f, 0.15f, 0.08f, 0.08f, 0.06f, 0.06f};
    const float assist[kNumJoints] = {0.25f, 0.30f, 0.25f, 0.10f, 0.10f, 0.08f, 0.05f};
    for (int i = 0; i < kNumJoints; ++i) {
        JointControl& j = c.joint[i];
        j.dir = 1;
        j.ktNmPerA = kt;
        j.tauMaxNm = tauMax[i];
        j.slewNmPerS = 20.0f;              // 0.08 Nm per 4 ms tick: full compensation within ~4 ticks
        j.dampingNmsPerRad = 0.0f;
        j.qMinRad = -kPi;                  // placeholders until the real ranges are measured (calib_range)
        j.qMaxRad = kPi;
        j.moveK = moveK[i];
        j.moveD = moveD[i];
        j.bilatK = bilK[i];
        j.bilatD = bilD[i];
        j.bilatTauMaxNm = bilMax[i];
        j.frictionNm = 0.0f;
        j.viscousNmsPerRad = 0.0f;
        j.extTauMaxNm = bilMax[i];
        j.assistTauMaxNm = assist[i];
    }
    // J2/J3 from part-to-part collisions of the CAD meshes (tools/gen_model.py JOINT_LIMITS), until measured
    c.joint[1].qMinRad = -105.0f * kPi / 180.0f;
    c.joint[1].qMaxRad = 130.0f * kPi / 180.0f;
    c.joint[2].qMinRad = -75.0f * kPi / 180.0f;
    c.joint[2].qMaxRad = 150.0f * kPi / 180.0f;
    c.joint[6].qMinRad = -0.5f * kPi;
    c.joint[6].qMaxRad = 0.5f * kPi;
    c.gravityScale = 1.0f;
    c.holdFriction = 0.0f;   // the model as it is; a new arm's stored settings start at kBenchHoldFriction (config_store)
    c.moveStiffness = 1.0f;
    for (int i = 0; i < kNumJoints; ++i) c.gravityJoint[i] = 1.0f;
    // No ramp by default: the MuJoCo simulator showed a 0.3 s ramp lets a free arm collapse ~240 mm
    // before the torque catches it, while switching on at once drops it ~1 mm. Use a ramp only on
    // first bring-up, with the arm resting on its stand.
    c.rampTimeS = 0.0f;
    // 3.25 mA per register unit: with the vendor example's 6.5 the arm needed twice the model's current to hold (J2, J3
    // and J5 alike, 2026-11-02), while the datasheet's locked-rotor Kt (4.5 kg cm/A) and the CAD masses fit 3.25.
    c.mAPerUnit = 3.25f;
    c.rawLimit = (int16_t)lroundf(kStallCurrentMa / c.mAPerUnit);   // 400: the stall current
    c.derateStartC = 60.0f;
    c.derateEndC = 70.0f;
    c.moveSpeedRadS = 0.6f;
    c.overrideRad = 8.0f * kPi / 180.0f;
    c.followerTimeoutS = 0.1f;
    c.frictionScale = 0.0f;                // off until the friction is measured on the bench
    c.frictionVelRadS = 0.2f;             // raft_sim friction, scale 1: a released arm coasts 12 deg (20 deg at 0.05)
    c.extGain = 0.0f;                      // off until the sign is checked on the real PAR6 (docs/bringup.md)
    c.extSign = -1.0f;
    c.extDeadbandNm = 0.3f;
    c.extCutoffHz = 8.0f;
    c.extTimeoutS = 0.1f;
    c.passivity = true;
    c.passivityMaxDampNmsPerRad = 0.1f;
    c.passivityMaxEnergyJ = 0.005f;
    c.barrierMarginRad = 8.0f * kPi / 180.0f;
    c.barrierKNmPerRad = 1.5f;
    c.barrierDNmsPerRad = 0.05f;
    c.orientKNmPerRad = 0.2f;
    c.orientDNmsPerRad = 0.01f;
    c.guideKNPerM = 200.0f;
    c.guideDNsPerM = 4.0f;
    c.spring.type = SpringType::None;
    c.spring.k = 0.0f;
    c.spring.a = 0.013f;                   // plan 4.4 layout v2: anchor 13 mm above J2, eyelet 20 mm up the arm
    c.spring.b = 0.020f;
    c.spring.q0 = 0.0f;
    c.spring.scale = 1.0f;
    c.autoRest.enabled = false;            // off until tried on the bench
    c.autoRest.idleTimeS = 30.0f;
    c.autoRest.stillSpeedRadS = 0.05f;
    c.autoRest.stillAngleRad = 2.0f * kPi / 180.0f;
    c.autoRest.speedRadS = 0.4f;
    // the arm lying back the way it rests limp (Petar, 2026-11-05): upper arm 2 deg short of J2's stop (-83.96), forearm
    // folded down to J3 -80, wrist straight. In the CAD every part keeps 8 mm from the others and the handle stays 41 mm
    // above the table, also on the straight move there from the usual poses; the arm then goes limp and settles the
    // last few degrees onto its stops. (Forearm 70 deg down with the upper arm vertical put the handle into the table.)
    const float rest[kNumJoints] = {0.0f, -82.0f * kPi / 180.0f, -80.0f * kPi / 180.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < kNumJoints; ++i) c.autoRest.qRest[i] = rest[i];
    return c;
}

float springTorque(const SpringModel& s, float q2)
{
    // Sign: the spring pulls the upper arm back toward vertical, i.e. it does the same job as the
    // servo's gravity holding torque, which is negative for a forward tilt (q2 > 0). Verified against
    // the MuJoCo tendon in tools/gen_model.py and tools/sim.
    switch (s.type) {
    case SpringType::ZeroLengthCable:
        return -s.scale * s.k * s.a * s.b * sinf(q2);
    case SpringType::OneSidedTorsion:
        return (q2 > s.q0) ? -s.scale * s.k * (q2 - s.q0) : 0.0f;
    default:
        return 0.0f;
    }
}

int16_t torqueToRaw(float tauNm, const JointControl& j, float mAPerUnit, int16_t rawLimit)
{
    const float amps = tauNm / j.ktNmPerA;
    float raw = (float)(kCurrentSign * j.dir) * amps * 1000.0f / mAPerUnit;
    raw = clampf(raw, -(float)rawLimit, (float)rawLimit);
    return (int16_t)lroundf(raw);
}

Controller::Controller(const ControlConfig& cfg)
    : cfg_(cfg), mode_(Mode::Limp), ramp_(0.0f), haveQ_(false), moveT_(0.0f), moveDur_(0.0f), moveIsRest_(false),
      tracking_(false), idleT_(0.0f), sinceFollower_(1e9f), followerOk_(false), sinceExt_(1e9f), orientOn_(false),
      orientPending_(false), guide_(Guide::None), guidePending_(false)
{
    for (int i = 0; i < kNumJoints; ++i) {
        prevTau_[i] = 0.0f; lastQ_[i] = 0.0f; moveStart_[i] = 0.0f; moveGoal_[i] = 0.0f;
        idleAnchor_[i] = 0.0f; qF_[i] = 0.0f; qdF_[i] = 0.0f; trackQ_[i] = 0.0f; trackQd_[i] = 0.0f;
        extIn_[i] = 0.0f; extF_[i] = 0.0f; energy_[i] = 0.0f;
        barOn_[i] = false; barLo_[i] = -kPi; barHi_[i] = kPi;
    }
    for (int k = 0; k < 9; ++k) orientR_[k] = (k % 4 == 0) ? 1.0f : 0.0f;
    for (int k = 0; k < 3; ++k) { guideP0_[k] = 0.0f; guideDir_[k] = 0.0f; }
}

void Controller::setMode(Mode m)
{
    if (m == Mode::Gravity && mode_ != Mode::Gravity) ramp_ = 0.0f;
    if (m == Mode::Gravity) { idleT_ = 0.0f; for (int i = 0; i < kNumJoints; ++i) idleAnchor_[i] = lastQ_[i]; }
    if (m == Mode::Bilateral && mode_ != Mode::Bilateral) {
        sinceFollower_ = 1e9f; followerOk_ = false; sinceExt_ = 1e9f;
        for (int i = 0; i < kNumJoints; ++i) { energy_[i] = 0.0f; extIn_[i] = 0.0f; extF_[i] = 0.0f; }
    }
    if (m == Mode::Move && mode_ != Mode::Move) {            // a bare MOVE holds the current pose
        for (int i = 0; i < kNumJoints; ++i) { moveStart_[i] = lastQ_[i]; moveGoal_[i] = lastQ_[i]; }
        moveT_ = moveDur_ = 0.0f; moveIsRest_ = false;
    }
    if (m != Mode::Move) tracking_ = false;
    mode_ = m;
}

bool Controller::moveTo(const float qTarget[kNumJoints], float speedRadS, bool rest)
{
    if (!haveQ_) return false;
    const float v = speedRadS > 0.0f ? speedRadS : cfg_.moveSpeedRadS;
    float maxDelta = 0.0f;
    for (int i = 0; i < kNumJoints; ++i) {
        moveStart_[i] = lastQ_[i];
        moveGoal_[i] = rest ? qTarget[i] : clampf(qTarget[i], cfg_.joint[i].qMinRad, cfg_.joint[i].qMaxRad);
        maxDelta = fmaxf(maxDelta, fabsf(moveGoal_[i] - moveStart_[i]));
    }
    moveDur_ = fmaxf(0.2f, 1.875f * maxDelta / v);   // minimum-jerk peak speed = 1.875 * delta / T
    moveT_ = 0.0f;
    moveIsRest_ = rest;
    tracking_ = false;
    if (mode_ != Mode::Move) { ramp_ = 1.0f; }
    mode_ = Mode::Move;
    return true;
}

bool Controller::track(const float q[kNumJoints], const float qd[kNumJoints])
{
    if (!haveQ_) return false;
    for (int i = 0; i < kNumJoints; ++i) {
        trackQ_[i] = clampf(q[i], cfg_.joint[i].qMinRad, cfg_.joint[i].qMaxRad);
        trackQd_[i] = qd[i];
    }
    if (mode_ != Mode::Move) { ramp_ = 1.0f; moveIsRest_ = false; }
    mode_ = Mode::Move;
    tracking_ = true;
    return true;
}

void Controller::moveReference(float* qRef, float* qdRef) const
{
    if (tracking_) {
        for (int i = 0; i < kNumJoints; ++i) { qRef[i] = trackQ_[i]; qdRef[i] = trackQd_[i]; }
        return;
    }
    const float u = (moveDur_ > 0.0f) ? clampf(moveT_ / moveDur_, 0.0f, 1.0f) : 1.0f;
    const float s = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
    const float ds = (moveDur_ > 0.0f && u < 1.0f) ? 30.0f * u * u * (1.0f - u) * (1.0f - u) / moveDur_ : 0.0f;
    for (int i = 0; i < kNumJoints; ++i) {
        const float d = moveGoal_[i] - moveStart_[i];
        qRef[i] = moveStart_[i] + s * d;
        qdRef[i] = ds * d;
    }
}

// Barriers, orientation lock and the wrist-centre guide (GRAVITY and BILATERAL). Adds into tau.
void Controller::assistTorques(const ControlInput& in, const Frame* T, float* tau)
{
    // barriers: a spring that starts barrierMarginRad inside the usable range, damping only toward the limit
    for (int i = 0; i < kNumJoints; ++i) {
        if (!barOn_[i]) continue;
        const float m = fminf(cfg_.barrierMarginRad, 0.4f * (barHi_[i] - barLo_[i]));
        const float up = in.q[i] - (barHi_[i] - m), down = (barLo_[i] + m) - in.q[i];
        if (up > 0.0f) tau[i] -= cfg_.barrierKNmPerRad * up + (in.qd[i] > 0.0f ? cfg_.barrierDNmsPerRad * in.qd[i] : 0.0f);
        if (down > 0.0f) tau[i] += cfg_.barrierKNmPerRad * down - (in.qd[i] < 0.0f ? cfg_.barrierDNmsPerRad * in.qd[i] : 0.0f);
    }
    // orientation lock: rotation vector e from the current TCP orientation to the anchor; J4..J6 get z_j . (K e)
    if (orientOn_) {
        const float* A = orientR_;
        const float* C = T[6].R;
        float E[9];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) E[3 * r + c] = A[3 * r] * C[3 * c] + A[3 * r + 1] * C[3 * c + 1] + A[3 * r + 2] * C[3 * c + 2];
        const float ang = acosf(clampf(0.5f * (E[0] + E[4] + E[8] - 1.0f), -1.0f, 1.0f));
        const float sa = sinf(ang);
        const float k = sa > 1e-4f ? ang / (2.0f * sa) : 0.5f;
        const float e[3] = {k * (E[7] - E[5]), k * (E[2] - E[6]), k * (E[3] - E[1])};
        for (int j = 3; j < 6; ++j) {
            const float* R = T[j].R;                  // joint j+1 turns about z of DH frame j
            tau[j] += cfg_.orientKNmPerRad * (R[2] * e[0] + R[5] * e[1] + R[8] * e[2]) - cfg_.orientDNmsPerRad * in.qd[j];
        }
    }
    // guide: spring on the wrist centre (frame 4 origin) toward its plane or line, through J1..J3
    if (guide_ != Guide::None) {
        const float* p = T[4].p;
        float Jv[3][3];                               // columns: joints 1..3
        for (int j = 0; j < 3; ++j) {
            const float zx = T[j].R[2], zy = T[j].R[5], zz = T[j].R[8];
            const float rx = p[0] - T[j].p[0], ry = p[1] - T[j].p[1], rz = p[2] - T[j].p[2];
            Jv[0][j] = zy * rz - zz * ry;
            Jv[1][j] = zz * rx - zx * rz;
            Jv[2][j] = zx * ry - zy * rx;
        }
        float d[3], v[3];
        for (int k = 0; k < 3; ++k) {
            d[k] = p[k] - guideP0_[k];
            v[k] = Jv[k][0] * in.qd[0] + Jv[k][1] * in.qd[1] + Jv[k][2] * in.qd[2];
        }
        // the constrained part: along the normal for a plane, across the line for an axis
        const float* u = guideDir_;
        const bool plane = guide_ == Guide::PlaneHorizontal || guide_ == Guide::PlaneTool;
        const float du = d[0] * u[0] + d[1] * u[1] + d[2] * u[2], vu = v[0] * u[0] + v[1] * u[1] + v[2] * u[2];
        float F[3];
        for (int k = 0; k < 3; ++k) {
            const float dc = plane ? du * u[k] : d[k] - du * u[k];
            const float vc = plane ? vu * u[k] : v[k] - vu * u[k];
            F[k] = -cfg_.guideKNPerM * dc - cfg_.guideDNsPerM * vc;
        }
        for (int j = 0; j < 3; ++j) tau[j] += Jv[0][j] * F[0] + Jv[1][j] * F[1] + Jv[2][j] * F[2];
    }
}

void Controller::step(const ControlInput& in, float dt, ControlOutput& out)
{
    out.events = EV_NONE;
    for (int i = 0; i < kNumJoints; ++i) lastQ_[i] = in.q[i];
    if (!haveQ_) { haveQ_ = true; for (int i = 0; i < kNumJoints; ++i) idleAnchor_[i] = in.q[i]; }

    forwardKinematics(in.q, out.frames);
    gravityTorque(out.frames, out.tauGravity);
    out.tauSpring2 = springTorque(cfg_.spring, in.q[1]);

    // anchors of the orientation lock and the guide: the pose of the tick they were switched on
    if (orientPending_) { for (int k = 0; k < 9; ++k) orientR_[k] = out.frames[6].R[k]; orientPending_ = false; }
    if (guidePending_) {
        for (int k = 0; k < 3; ++k) guideP0_[k] = out.frames[4].p[k];
        const bool tool = guide_ == Guide::PlaneTool || guide_ == Guide::AxisTool;
        const float* R = out.frames[6].R;
        guideDir_[0] = tool ? R[2] : 0.0f;
        guideDir_[1] = tool ? R[5] : 0.0f;
        guideDir_[2] = tool ? R[8] : 1.0f;
        guidePending_ = false;
    }

    if (mode_ == Mode::Gravity) {
        ramp_ = (cfg_.rampTimeS > 0.0f) ? clampf(ramp_ + dt / cfg_.rampTimeS, 0.0f, 1.0f) : 1.0f;
    } else if (mode_ == Mode::Limp) {
        ramp_ = 0.0f;
    } else {
        ramp_ = 1.0f;          // MOVE and BILATERAL always carry full compensation (the ramp is a GRAVITY bring-up aid)
    }

    // --- supervisors -----------------------------------------------------------------------
    float qRef[kNumJoints], qdRef[kNumJoints];
    for (int i = 0; i < kNumJoints; ++i) { qRef[i] = in.q[i]; qdRef[i] = 0.0f; }

    if (mode_ == Mode::Gravity && cfg_.autoRest.enabled) {
        bool still = true;
        for (int i = 0; i < kNumJoints; ++i) {
            if (fabsf(in.qd[i]) > cfg_.autoRest.stillSpeedRadS || fabsf(in.q[i] - idleAnchor_[i]) > cfg_.autoRest.stillAngleRad) still = false;
        }
        if (still) {
            idleT_ += dt;
        } else {
            idleT_ = 0.0f;
            for (int i = 0; i < kNumJoints; ++i) idleAnchor_[i] = in.q[i];
        }
        float dist = 0.0f;
        for (int i = 0; i < kNumJoints; ++i) dist = fmaxf(dist, fabsf(in.q[i] - cfg_.autoRest.qRest[i]));
        if (idleT_ >= cfg_.autoRest.idleTimeS && dist > cfg_.autoRest.stillAngleRad) {
            moveTo(cfg_.autoRest.qRest, cfg_.autoRest.speedRadS, true);
            out.events |= EV_AUTO_REST;
            idleT_ = 0.0f;
        }
    }

    if (mode_ == Mode::Move) {
        moveT_ += dt;
        moveReference(qRef, qdRef);
        float worst = 0.0f;
        for (int i = 0; i < kNumJoints; ++i) worst = fmaxf(worst, fabsf(qRef[i] - in.q[i]));
        if (worst > cfg_.overrideRad) {                      // someone is holding or pushing the arm
            out.events |= EV_OVERRIDDEN;
            setMode(Mode::Gravity);
            ramp_ = 1.0f;
            for (int i = 0; i < kNumJoints; ++i) { qRef[i] = in.q[i]; qdRef[i] = 0.0f; }
        } else if (!tracking_ && moveT_ >= moveDur_ && moveT_ - dt < moveDur_) {
            out.events |= EV_MOVE_DONE;
            if (moveIsRest_) setMode(Mode::Limp);      // parked: torque off, the arm settles onto its stops
        }
    }

    bool extOk = false;
    if (mode_ == Mode::Bilateral) {
        sinceFollower_ += dt;
        sinceExt_ += dt;
        if (in.followerFresh) {
            sinceFollower_ = 0.0f;
            for (int i = 0; i < kNumJoints; ++i) { qF_[i] = in.qFollower[i]; qdF_[i] = in.qdFollower[i]; }
            if (in.extValid) {
                sinceExt_ = 0.0f;
                for (int i = 0; i < kNumJoints; ++i) {
                    const float x = in.tauExtFollower[i], m = fabsf(x) - cfg_.extDeadbandNm;
                    extIn_[i] = m > 0.0f ? (x > 0.0f ? m : -m) : 0.0f;
                }
            }
            if (!followerOk_) out.events |= EV_FOLLOWER_BACK;
            followerOk_ = true;
        } else if (followerOk_ && sinceFollower_ > cfg_.followerTimeoutS) {
            followerOk_ = false;
            out.events |= EV_FOLLOWER_LOST;
        }
        if (followerOk_) for (int i = 0; i < kNumJoints; ++i) { qRef[i] = qF_[i]; qdRef[i] = qdF_[i]; }
        extOk = followerOk_ && sinceExt_ <= cfg_.extTimeoutS && cfg_.extGain != 0.0f;
        // low-pass toward the latest sample, or fade out without one
        const float a = 1.0f - expf(-2.0f * kPi * cfg_.extCutoffHz * dt);
        for (int i = 0; i < kNumJoints; ++i) extF_[i] += a * ((extOk ? extIn_[i] : 0.0f) - extF_[i]);
    }

    float assist[kNumJoints] = {0};
    if (mode_ == Mode::Gravity || mode_ == Mode::Bilateral) assistTorques(in, out.frames, assist);

    // --- torque law --------------------------------------------------------------------------
    out.pcEnergyJ = 0.0f;
    for (int i = 0; i < kNumJoints; ++i) {
        const JointControl& j = cfg_.joint[i];
        float tau = 0.0f;
        out.tauExt[i] = 0.0f;
        out.tauAssist[i] = 0.0f;
        if (mode_ != Mode::Limp) {
            const float load = out.tauGravity[i] - (i == 1 ? out.tauSpring2 : 0.0f);
            tau = ramp_ * (cfg_.gravityScale + cfg_.holdFriction) * cfg_.gravityJoint[i] * load;
            const float fric = cfg_.frictionScale * (j.frictionNm * tanhf(in.qd[i] / cfg_.frictionVelRadS) + j.viscousNmsPerRad * in.qd[i]);
            if (mode_ == Mode::Gravity) {
                tau += fric - j.dampingNmsPerRad * in.qd[i];
            } else if (mode_ == Mode::Move) {
                tau += cfg_.moveStiffness * j.moveK * (qRef[i] - in.q[i]) + sqrtf(cfg_.moveStiffness) * j.moveD * (qdRef[i] - in.qd[i]);
            } else if (mode_ == Mode::Bilateral) {
                tau += fric;
                if (followerOk_) {
                    float c = clampf(j.bilatK * (qRef[i] - in.q[i]) + j.bilatD * (qdRef[i] - in.qd[i]), -j.bilatTauMaxNm, j.bilatTauMaxNm);
                    const float ext = clampf(cfg_.extSign * cfg_.extGain * extF_[i], -j.extTauMaxNm, j.extTauMaxNm);
                    out.tauExt[i] = ext;
                    c += ext;
                    if (cfg_.passivity && i < kNumJoints - 1) {
                        // energy the coupling took from the leader (J); negative = it gave out more than it took
                        energy_[i] -= c * in.qd[i] * dt;
                        if (energy_[i] < 0.0f && fabsf(in.qd[i]) > 1e-3f) {
                            const float alpha = fminf(cfg_.passivityMaxDampNmsPerRad, -energy_[i] / (in.qd[i] * in.qd[i] * dt));
                            c -= alpha * in.qd[i];
                            energy_[i] += alpha * in.qd[i] * in.qd[i] * dt;
                        }
                        energy_[i] = fminf(energy_[i], cfg_.passivityMaxEnergyJ);
                        out.pcEnergyJ = fminf(out.pcEnergyJ, energy_[i]);
                    }
                    tau += c;
                }
            }
            const float as = clampf(assist[i], -j.assistTauMaxNm, j.assistTauMaxNm);
            out.tauAssist[i] = as;
            tau += as;
        }
        const float maxStep = j.slewNmPerS * dt;
        tau = clampf(tau, prevTau_[i] - maxStep, prevTau_[i] + maxStep);
        tau = clampf(tau, -j.tauMaxNm, j.tauMaxNm);
        if (mode_ == Mode::Limp) tau = 0.0f;           // limp is immediate, no slew
        prevTau_[i] = tau;

        const float span = cfg_.derateEndC - cfg_.derateStartC;
        const float d = (span > 0.0f) ? clampf((cfg_.derateEndC - in.tempC[i]) / span, 0.0f, 1.0f) : 1.0f;
        out.derate[i] = d;
        if (i == ovJoint_) {                           // calibration drives this joint directly
            const float tauOv = (float)ovRaw_ * cfg_.mAPerUnit * 1e-3f * j.ktNmPerA * (float)(kCurrentSign * j.dir);
            prevTau_[i] = tauOv;
            out.tauCmd[i] = d * tauOv;
            out.raw[i] = (int16_t)lroundf(d * (float)ovRaw_);
            out.qRef[i] = qRef[i];
            continue;
        }
        out.tauCmd[i] = d * tau;
        out.raw[i] = torqueToRaw(out.tauCmd[i], j, cfg_.mAPerUnit, cfg_.rawLimit);
        out.qRef[i] = qRef[i];
    }
}

}  // namespace raft
