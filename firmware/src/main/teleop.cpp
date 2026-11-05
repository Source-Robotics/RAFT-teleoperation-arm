#include "teleop.h"

#include "arm_design.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace raft {
namespace {

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
const float kPi = 3.14159265358979f;
float deg(float r) { return r * 180.0f / kPi; }

}  // namespace

TeleopConfig defaultTeleopConfig()
{
    TeleopConfig c;
    c.alignTolRad = 2.0f * kPi / 180.0f;
    c.alignSpeedRadS = 0.4f;
    c.realignDelayS = 1.0f;
    c.triggerOpenRad = 0.0f;                 // placeholders until the trigger's travel is measured
    c.triggerClosedRad = 60.0f * kPi / 180.0f;
    c.gripperOpenRaw = 0;                    // firmware mode: 0 open, 255 closed, for commands and g_p alike
    c.gripperClosedRaw = 255;
    c.gripperSpeed = 200;
    c.gripperCurrentmA = 400;
    // RCB robots/PAR6.xml software_joint_limit_*, DH theta minus RAFT's DH offsets [0, -pi/2, pi, 0, 0, pi]
    const float fMin[6] = {-2.8647335f, -2.4407335f + 0.5f * kPi, 1.9912665f - kPi, -2.6147335f, -1.73f, -0.85f - kPi};
    const float fMax[6] = {2.8647335f, -0.1122665f + 0.5f * kPi, 6.5627335f - kPi, 2.5547335f, 1.6f, 7.14f - kPi};
    for (int i = 0; i < 6; ++i) { c.followerMinRad[i] = fMin[i]; c.followerMaxRad[i] = fMax[i]; }
    c.limitMarginRad = 3.0f * kPi / 180.0f;
    c.autoReclaim = false;                   // an outage waits for the operator (Resume)
    c.maxTargetStepRad = 0.15f;              // 37 rad/s at 250 Hz: no hand does that, a 2 pi encoder slip does
    return c;
}

const char* TeleopSession::stageName(Stage s)
{
    switch (s) {
    case Stage::Off: return "OFF";
    case Stage::WaitPaired: return "WAIT_PAIRED";
    case Stage::Aligning: return "ALIGNING";
    case Stage::Claiming: return "CLAIMING";
    case Stage::Coupled: return "COUPLED";
    case Stage::Held: return "HELD";
    case Stage::Realigning: return "REALIGNING";
    case Stage::Lost: return "LOST";
    }
    return "?";
}

TeleopSession::TeleopSession(Controller& ctl, rti::Link& link, const TeleopConfig& cfg)
    : ctl_(ctl), link_(link), cfg_(cfg), stage_(Stage::Off), stageT_(0.0), lastStateCount_(0), haveCmd_(false),
      precision_(1.0f), locks_(0), engaged_(true), needRelease_(false), offsetAlign_(false)
{
    for (int i = 0; i < kNumJoints; ++i) { qF_[i] = 0.0f; target_[i] = 0.0f; qL_[i] = 0.0f; }
    for (int i = 0; i < 6; ++i) { aL_[i] = aF_[i] = branch_[i] = hold_[i] = cmdF_[i] = sent_[i] = 0.0f; s_[i] = 1.0f; }
    note_[0] = '\0';
}

void TeleopSession::setNote(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(note_, sizeof(note_), fmt, ap);
    va_end(ap);
}

void TeleopSession::setFilter(float hz)
{
    const float fs = (float)kControlHz;
    filterHz_ = hz > 0.0f ? (hz < 0.45f * fs ? hz : 0.45f * fs) : 0.0f;
    filtInit_ = false;
    if (filterHz_ <= 0.0f) return;
    // bilinear transform with prewarping (the textbook 2nd-order Butterworth biquad)
    const float k = tanf(3.14159265f * filterHz_ / fs), r2 = 1.41421356f;
    const float norm = 1.0f / (1.0f + r2 * k + k * k);
    fb0_ = k * k * norm;
    fb1_ = 2.0f * fb0_;
    fb2_ = fb0_;
    fa1_ = 2.0f * (k * k - 1.0f) * norm;
    fa2_ = (1.0f - r2 * k + k * k) * norm;
}

float TeleopSession::lp(Lp2& s, float x) const
{
    const float y = fb0_ * x + fb1_ * s.x1 + fb2_ * s.x2 - fa1_ * s.y1 - fa2_ * s.y2;
    s.x2 = s.x1; s.x1 = x;
    s.y2 = s.y1; s.y1 = y;
    return y;
}

void TeleopSession::begin(double now)
{
    link_.start(now);
    stage_ = Stage::WaitPaired;
    stageT_ = now;
    haveCmd_ = false;
    filtInit_ = false;
    needRelease_ = false;
    note_[0] = '\0';
    for (int i = 0; i < 6; ++i) { aL_[i] = aF_[i] = branch_[i] = 0.0f; s_[i] = 1.0f; }
}

void TeleopSession::end(double now)
{
    link_.stop(now);
    ctl_.setMode(Mode::Gravity);
    stage_ = Stage::Off;
    stageT_ = now;
    note_[0] = '\0';
}

bool TeleopSession::followerPose(float q[kNumJoints], float qd[kNumJoints], float tau[kNumJoints], bool& extOk) const
{
    const rti::State& s = link_.state();
    if (!(s.present & rti::F_JP)) return false;
    float th[6], qc[6];
    for (int i = 0; i < 6; ++i) th[i] = (float)s.jp[i];
    dhToCad(th, qc);
    extOk = (s.present & rti::F_TEXT) != 0;
    for (int i = 0; i < 6; ++i) {
        q[i] = qc[i];
        qd[i] = (s.present & rti::F_JS) ? (float)s.js[i] : 0.0f;   // the DH offset is constant
        tau[i] = extOk ? (float)s.text[i] : 0.0f;                   // ... so torques map 1:1 too
    }
    // gripper position (raw, same scale as gripper_set) -> equivalent trigger angle; without g_p (or with the
    // runtime's "not reported yet" sentinel, which the parser drops) it stays at the last known value
    if (s.present & rti::F_GP) {
        const float span = (float)(cfg_.gripperClosedRaw - cfg_.gripperOpenRaw);
        const float u = span != 0.0f ? ((float)s.gp - (float)cfg_.gripperOpenRaw) / span : 0.0f;
        q[6] = cfg_.triggerOpenRad + clampf(u, 0.0f, 1.0f) * (cfg_.triggerClosedRad - cfg_.triggerOpenRad);
    } else {
        q[6] = qF_[6];
    }
    qd[6] = 0.0f;
    tau[6] = 0.0f;
    return true;
}

uint8_t TeleopSession::planAlignment(const float qf[kNumJoints], const float qNow[kNumJoints], float target[kNumJoints], float branch[6])
{
    uint8_t bad = 0;
    const ControlConfig& cc = ctl_.config();
    for (int i = 0; i < 6; ++i) {
        const float lo = cc.joint[i].qMinRad, hi = cc.joint[i].qMaxRad;
        // q and q + 2 pi k are the same arm configuration: take the branch inside RAFT's range nearest to where
        // the leader is (PAR6's J6 at 200 deg is RAFT's J6 at -160 deg)
        int best = 99;
        float bestD = 1e9f;
        for (int k = -2; k <= 2; ++k) {
            const float q = qf[i] - 2.0f * kPi * (float)k;
            if (q < lo || q > hi) continue;
            const float d = fabsf(q - qNow[i]);
            if (d < bestD) { bestD = d; best = k; }
        }
        if (best != 99) {
            branch[i] = 2.0f * kPi * (float)best;
            target[i] = qf[i] - branch[i];
            continue;
        }
        // unreachable: the branch that comes closest, and a target clamped just inside RAFT's range
        int kb = 0;
        float bestOut = 1e9f;
        for (int k = -2; k <= 2; ++k) {
            const float q = qf[i] - 2.0f * kPi * (float)k;
            const float out = q < lo ? lo - q : q - hi;
            if (out < bestOut) { bestOut = out; kb = k; }
        }
        branch[i] = 2.0f * kPi * (float)kb;
        const float m = fminf(cfg_.limitMarginRad, 0.25f * (hi - lo));
        target[i] = clampf(qf[i] - branch[i], lo + m, hi - m);
        if (!bad) setNote("J%d: PAR6 is at %.1f deg, RAFT reaches %.0f..%.0f deg", i + 1, (double)deg(qf[i] - branch[i]), (double)deg(lo), (double)deg(hi));
        bad |= (uint8_t)(1u << i);
    }
    target[6] = qNow[6];                                 // the trigger stays where the hand has it
    return bad;
}

void TeleopSession::reanchor(const float qL[kNumJoints], const float cmd[6])
{
    // the follower target stays exactly where it is (no step), and from here on moves s times the leader
    for (int i = 0; i < 6; ++i) {
        aL_[i] = qL[i];
        aF_[i] = cmd[i];
        s_[i] = (locks_ & (1u << i)) ? 0.0f : precision_;
    }
}

void TeleopSession::setPrecision(float s)
{
    precision_ = clampf(s, 0.05f, 1.0f);
    if (stage_ == Stage::Coupled) reanchor(qL_, cmdF_);
    else if (stage_ == Stage::Held || stage_ == Stage::Realigning) for (int i = 0; i < 6; ++i) s_[i] = (locks_ & (1u << i)) ? 0.0f : precision_;
}

void TeleopSession::setLocks(uint8_t mask)
{
    locks_ = (uint8_t)(mask & 0x3f);
    if (stage_ == Stage::Coupled) reanchor(qL_, cmdF_);
    else if (stage_ == Stage::Held || stage_ == Stage::Realigning) for (int i = 0; i < 6; ++i) s_[i] = (locks_ & (1u << i)) ? 0.0f : precision_;
}

float TeleopSession::offsetMaxRad() const
{
    float m = 0.0f;
    for (int i = 0; i < 6; ++i) m = fmaxf(m, fabsf(cmdF_[i] - (qL_[i] + branch_[i])));
    return m;
}

void TeleopSession::usableRange(int i, float& lo, float& hi) const
{
    const JointControl& j = ctl_.config().joint[i];
    lo = j.qMinRad;
    hi = j.qMaxRad;
    if (i >= 6 || s_[i] <= 0.0f) return;                 // the trigger, or a locked joint (the coupling holds it)
    lo = fmaxf(lo, mapToLeader(i, cfg_.followerMinRad[i]));
    hi = fminf(hi, mapToLeader(i, cfg_.followerMaxRad[i]));
}

void TeleopSession::lose(double now, const char* why)
{
    ctl_.setMode(Mode::Gravity);
    stageT_ = now;
    if (cfg_.autoReclaim) {
        stage_ = Stage::WaitPaired;
        note_[0] = '\0';
    } else {
        stage_ = Stage::Lost;
        setNote("%s (%s): resume to realign and reclaim", why, link_.lastError());
    }
}

bool TeleopSession::resume(double now)
{
    if (stage_ != Stage::Lost) return false;
    stage_ = Stage::WaitPaired;
    stageT_ = now;
    note_[0] = '\0';
    return true;
}

bool TeleopSession::realign(double now)
{
    if (stage_ != Stage::Coupled && stage_ != Stage::Held) return false;
    // PAR6 holds where it really is (a blocked or clamped follower is not where it was told to be, and holding
    // the old target would keep it pushing), and the leader aligns with that
    if (link_.stateFresh(now)) {
        for (int i = 0; i < 6; ++i) hold_[i] = clampf(qF_[i], cfg_.followerMinRad[i], cfg_.followerMaxRad[i]);
    } else if (stage_ == Stage::Coupled) {
        for (int i = 0; i < 6; ++i) hold_[i] = cmdF_[i];
    }
    float f[kNumJoints], target[kNumJoints], branch[6];
    for (int i = 0; i < 6; ++i) f[i] = hold_[i];
    f[6] = qL_[6];
    if (planAlignment(f, qL_, target, branch)) {
        char why[120];
        snprintf(why, sizeof(why), "cannot realign: %s", note_);
        snprintf(note_, sizeof(note_), "%s", why);
        return false;
    }
    if (!ctl_.moveTo(target, cfg_.alignSpeedRadS)) return false;
    for (int i = 0; i < 6; ++i) branch_[i] = branch[i];
    for (int i = 0; i < kNumJoints; ++i) target_[i] = target[i];
    stage_ = Stage::Realigning;
    stageT_ = now;
    needRelease_ = false;
    note_[0] = '\0';
    return true;
}

void TeleopSession::beforeControl(ControlInput& in, double now)
{
    in.followerFresh = false;
    in.extValid = false;
    const bool linkActive = link_.phase() == rti::Phase::Active;
    const bool linkPaired = link_.phase() == rti::Phase::Paired;
    float qf[kNumJoints], qdf[kNumJoints], tau[kNumJoints];
    bool extOk = false;
    const bool haveF = link_.stateFresh(now) && followerPose(qf, qdf, tau, extOk);
    if (haveF) for (int i = 0; i < kNumJoints; ++i) qF_[i] = qf[i];

    switch (stage_) {
    case Stage::Off:
    case Stage::Lost:
        break;
    case Stage::WaitPaired:
        if (linkPaired && haveF) {
            float target[kNumJoints], branch[6];
            const uint8_t bad = planAlignment(qf, in.q, target, branch);
            if (bad && !offsetAlign_) {
                const size_t n = strlen(note_);
                snprintf(note_ + n, sizeof(note_) - n, ": move PAR6 into range, or allow an offset");
                break;                                   // nothing moves; checked again every tick
            }
            if (!bad) {
                note_[0] = '\0';
            } else {
                char why[120];
                snprintf(why, sizeof(why), "coupling with an offset: %s", note_);
                snprintf(note_, sizeof(note_), "%s", why);
            }
            // identity mapping (plus the branch); for an unreachable joint the offset follower - target
            for (int i = 0; i < 6; ++i) { branch_[i] = branch[i]; s_[i] = 1.0f; aL_[i] = target[i]; aF_[i] = qf[i]; }
            for (int i = 0; i < kNumJoints; ++i) target_[i] = target[i];
            if (ctl_.moveTo(target, cfg_.alignSpeedRadS)) { stage_ = Stage::Aligning; stageT_ = now; haveCmd_ = false; }
        }
        break;
    case Stage::Aligning: {
        if (!linkPaired) { ctl_.setMode(Mode::Gravity); stage_ = Stage::WaitPaired; stageT_ = now; break; }
        if (ctl_.mode() != Mode::Move) {             // someone grabbed the arm: try again a bit later
            if (now - stageT_ > cfg_.realignDelayS) { stage_ = Stage::WaitPaired; stageT_ = now; }
            break;
        }
        if (haveF) {                                 // PAR6 moved meanwhile (jogged by someone): plan again
            bool moved = false;
            for (int i = 0; i < 6; ++i) moved = moved || fabsf(qf[i] - aF_[i]) > cfg_.alignTolRad;
            if (moved) { stage_ = Stage::WaitPaired; stageT_ = now; break; }
        }
        float worst = 0.0f;
        for (int i = 0; i < 6; ++i) worst = fmaxf(worst, fabsf(in.q[i] - target_[i]));
        if (!ctl_.moveActive() && worst < cfg_.alignTolRad && link_.claim(now)) { stage_ = Stage::Claiming; stageT_ = now; }
        break;
    }
    case Stage::Claiming:
        if (linkActive) {
            stageT_ = now;
            lastStateCount_ = link_.stateCount() - 1;   // feed the current follower state on this tick
            if (precision_ != 1.0f || locks_) {
                float cmd[6];
                for (int i = 0; i < 6; ++i) cmd[i] = mapToFollower(i, in.q[i]);
                reanchor(in.q, cmd);
            }
            if (engaged_) { ctl_.setMode(Mode::Bilateral); stage_ = Stage::Coupled; }
            else { for (int i = 0; i < 6; ++i) hold_[i] = mapToFollower(i, in.q[i]); ctl_.setMode(Mode::Gravity); stage_ = Stage::Held; }
        } else if (link_.phase() != rti::Phase::Claiming) {
            ctl_.setMode(Mode::Gravity);
            stage_ = Stage::WaitPaired;
            stageT_ = now;
        }
        break;
    case Stage::Coupled:
        if (!linkActive) { lose(now, "PAR6 link lost"); break; }
        if (!engaged_) {                             // clutch: the follower stays, the leader is free
            for (int i = 0; i < 6; ++i) hold_[i] = cmdF_[i];
            ctl_.setMode(Mode::Gravity);
            stage_ = Stage::Held;
            stageT_ = now;
            break;
        }
        if (haveF && link_.stateCount() != lastStateCount_) {
            lastStateCount_ = link_.stateCount();
            in.followerFresh = true;
            in.extValid = extOk;
            for (int i = 0; i < 6; ++i) {
                in.qFollower[i] = mapToLeader(i, qf[i]);
                in.qdFollower[i] = s_[i] > 0.0f ? qdf[i] / s_[i] : 0.0f;
                in.tauExtFollower[i] = s_[i] * tau[i];          // virtual work: tau_L dq_L = tau_F dq_F
            }
            in.qFollower[6] = qf[6];
            in.qdFollower[6] = 0.0f;
            in.tauExtFollower[6] = 0.0f;
        }
        break;
    case Stage::Held:
        if (!linkActive) { lose(now, "PAR6 link lost"); break; }
        if (!engaged_) needRelease_ = false;
        if (engaged_ && !needRelease_) {             // re-engage where the leader is now
            reanchor(in.q, hold_);
            ctl_.setMode(Mode::Bilateral);
            stage_ = Stage::Coupled;
            stageT_ = now;
            lastStateCount_ = link_.stateCount() - 1;
            note_[0] = '\0';
        }
        break;
    case Stage::Realigning: {
        if (!linkActive) { lose(now, "PAR6 link lost"); break; }
        if (ctl_.mode() != Mode::Move) {             // grabbed during the move
            stage_ = Stage::Held;
            stageT_ = now;
            setNote("realign interrupted (the arm was moved by hand)");
            break;
        }
        float worst = 0.0f;
        for (int i = 0; i < 6; ++i) worst = fmaxf(worst, fabsf(in.q[i] - target_[i]));
        if (!ctl_.moveActive() && worst < cfg_.alignTolRad) {
            reanchor(in.q, hold_);                   // what is left of the offset is within the alignment tolerance
            stageT_ = now;
            if (engaged_) { ctl_.setMode(Mode::Bilateral); stage_ = Stage::Coupled; lastStateCount_ = link_.stateCount() - 1; }
            else { ctl_.setMode(Mode::Gravity); stage_ = Stage::Held; }
        }
        break;
    }
    }
}

void TeleopSession::afterControl(const ControlInput& in, double now)
{
    if (stage_ == Stage::Off) return;
    for (int i = 0; i < kNumJoints; ++i) qL_[i] = in.q[i];
    float cmd[6];
    double qdDh[6];
    if (stage_ == Stage::Held || stage_ == Stage::Realigning) {
        for (int i = 0; i < 6; ++i) { cmd[i] = hold_[i]; qdDh[i] = 0.0; }
    } else {
        // keep the link's target current from CLAIMING on, so the first command is the live leader pose
        for (int i = 0; i < 6; ++i) {
            cmd[i] = clampf(mapToFollower(i, in.q[i]), cfg_.followerMinRad[i], cfg_.followerMaxRad[i]);
            qdDh[i] = s_[i] * in.qd[i];
        }
    }
    if (stage_ == Stage::Coupled && haveCmd_) {
        for (int i = 0; i < 6; ++i) {
            const float step = fabsf(cmd[i] - cmdF_[i]);
            if (step <= cfg_.maxTargetStepRad) continue;
            setNote("J%d target jumped %.0f deg in one tick: holding (release and engage, or realign)", i + 1, (double)deg(step));
            for (int k = 0; k < 6; ++k) { hold_[k] = cmdF_[k]; cmd[k] = hold_[k]; qdDh[k] = 0.0; }
            ctl_.setMode(Mode::Gravity);
            stage_ = Stage::Held;
            stageT_ = now;
            needRelease_ = true;
            break;
        }
    }
    for (int i = 0; i < 6; ++i) cmdF_[i] = cmd[i];
    haveCmd_ = true;
    if (filterHz_ > 0.0f) {   // after the jump guard, which must see the raw target
        if (!filtInit_) {
            for (int i = 0; i < 6; ++i) {
                lpQ_[i] = {cmd[i], cmd[i], cmd[i], cmd[i]};
                const float v = (float)qdDh[i];
                lpV_[i] = {v, v, v, v};
            }
            filtInit_ = true;
        }
        for (int i = 0; i < 6; ++i) { cmd[i] = lp(lpQ_[i], cmd[i]); qdDh[i] = lp(lpV_[i], (float)qdDh[i]); }
    }
    for (int i = 0; i < 6; ++i) sent_[i] = cmd[i];
    float th[6];
    cadToDh(cmd, th);
    double qDh[6];
    for (int i = 0; i < 6; ++i) qDh[i] = th[i];
    link_.setTarget(qDh, qdDh, now);
    if (stage_ == Stage::Coupled) {
        const float u = (cfg_.triggerClosedRad != cfg_.triggerOpenRad)
                            ? (in.q[6] - cfg_.triggerOpenRad) / (cfg_.triggerClosedRad - cfg_.triggerOpenRad) : 0.0f;
        const int pos = (int)lroundf((float)cfg_.gripperOpenRaw + clampf(u, 0.0f, 1.0f) * (float)(cfg_.gripperClosedRaw - cfg_.gripperOpenRaw));
        link_.setGripper(pos, cfg_.gripperSpeed, cfg_.gripperCurrentmA);
    }
}

}  // namespace raft
