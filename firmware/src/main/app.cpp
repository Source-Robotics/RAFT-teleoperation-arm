#include "app.h"

#include "arm_design.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "json_lite.h"

namespace raft {
namespace {

const float kPi = 3.14159265358979f;
const float kDeg = 180.0f / kPi;
// A servo that misses a read keeps its last angle (speed taken as 0) and, in Hold, its last position goal, like the
// RCB runtime does with old motor data (a warning at 10 ticks, an error at 50). Consecutive failed reads of one
// servo that fault the arm to limp: with torque on 50 (200 ms at 250 Hz); during a calibration 6 (24 ms), so no
// capture uses an old angle. Limp, a missing servo only counts; torque is refused while one has missed its last 10.
const int kMissingFault = 50;
const int kMissingFaultCalib = 6;
const int kMissingRefuse = 10;
// A servo that answers again after this many missed reads in a row (about 0.25 s) may have been turned by hand in the
// meantime (its supply was off): its angle is taken to be within half a turn of zero again, as at power-up, instead of
// counting a turn from its last angle.
const int kRewrapMissing = 63;
const float kTempFaultC = 75.0f;     // derating ends at 70 C; this is the hard stop

const char* modeName(Mode m)
{
    switch (m) {
    case Mode::Limp: return "limp";
    case Mode::Gravity: return "gravity";
    case Mode::Move: return "move";
    case Mode::Bilateral: return "bilateral";
    }
    return "?";
}

// appends printf output to a fixed buffer; once full, everything after is dropped and ok() turns false
struct Out {
    char* b; size_t cap; size_t n; bool full;
    Out(char* buf, size_t c) : b(buf), cap(c), n(0), full(c == 0) { if (c) b[0] = '\0'; }
    void add(const char* fmt, ...)
    {
        if (full) return;
        va_list ap;
        va_start(ap, fmt);
        const int m = vsnprintf(b + n, cap - n, fmt, ap);
        va_end(ap);
        if (m < 0 || n + (size_t)m >= cap) { full = true; return; }
        n += (size_t)m;
    }
    void degs(const char* key, const float* v, int count)
    {
        add("\"%s\": [", key);
        for (int i = 0; i < count; ++i) add("%s%.2f", i ? ", " : "", (double)(v[i] * kDeg));
        add("]");
    }
};

// host names are typed by people: keep them JSON-safe
void jsonSafe(const char* in, char* out, size_t cap)
{
    size_t i = 0;
    for (; i + 1 < cap && in[i]; ++i) out[i] = (in[i] == '"' || in[i] == '\\' || in[i] < ' ') ? '?' : in[i];
    out[i] = '\0';
}

}  // namespace

constexpr int Replay::kMax;

App::App(Storage& storage, rti::Transport& transport)
    : storage_(storage), ctl_(defaultControlConfig()), link_(transport, rti::defaultLinkConfig()),
      session_(ctl_, link_, defaultTeleopConfig()), teleop_(false), modelOk_(false), torqueWasOn_(false),
      lastTick_(0.0), haveTick_(false)
{
    fault_[0] = lastCalib_[0] = streamHost_[0] = replayNote_[0] = '\0';
    memset(missing_, 0, sizeof(missing_));
    for (int i = 0; i < kNumJoints; ++i) jointDamping_[i] = kGravityDampingNms[i];
    memset(&in_, 0, sizeof(in_));
    memset(&out_, 0, sizeof(out_));
    memset(&frame_, 0, sizeof(frame_));
    memset(seq_, 0, sizeof(seq_));
    memset(events_, 0, sizeof(events_));
    memset(currentmA_, 0, sizeof(currentmA_));
    memset(voltage_, 0, sizeof(voltage_));
    memset(lastWrapped_, 0, sizeof(lastWrapped_));
    resetUnwrap();
    defaultPersistentConfig(pc_);
    cc_ = defaultControlConfig();
}

App::~App()
{
    free(replay_.q);
    free(scope_.buf);
}

void App::scopeTick(double now)
{
    if (scope_.state != Scope::Recording) return;
    if (!scope_.n) scope_.t0 = now;
    int16_t* w = scope_.buf + (size_t)scope_.n * Scope::kWords;
    const auto q16 = [](float v, float unit) {
        const float x = v / unit;
        return (int16_t)(x > 32767.0f ? 32767 : (x < -32767.0f ? -32767 : lroundf(x)));
    };
    w[0] = (int16_t)(uint16_t)lround((now - scope_.t0) * 1e4);
    for (int i = 0; i < kNumJoints; ++i) { w[1 + i] = q16(in_.q[i], 1e-4f); w[8 + i] = q16(in_.qd[i], 1e-3f); }
    const bool sending = teleop_ && session_.streaming();
    for (int i = 0; i < 6; ++i) w[15 + i] = sending ? q16(session_.sentTarget()[i], 1e-4f) : (int16_t)INT16_MIN;
    for (int i = 0; i < kNumJoints; ++i) w[21 + i] = currentmA_[i];
    if (++scope_.n >= scope_.total) scope_.state = Scope::Done;
}

void App::begin(double now, bool modelOk)
{
    (void)now;
    modelOk_ = modelOk;
    PersistentConfig loaded;
    if (storage_.load(loaded)) pc_ = loaded;
    else defaultPersistentConfig(pc_);
    rebuildControlConfig();
    strncpy(streamHost_, pc_.streamHost, sizeof(streamHost_) - 1);
    if (!modelOk_) setFault("model self-test failed");
}

void App::rebuildControlConfig()
{
    cc_ = defaultControlConfig();
    applyConfig(pc_, cc_);
    cc_.moveStiffness = holdStiffness_;
    for (int i = 0; i < kNumJoints; ++i) { cc_.gravityJoint[i] = jointGravity_[i]; cc_.joint[i].dampingNmsPerRad = jointDamping_[i]; }
    ctl_.mutableConfig() = cc_;
    TeleopConfig& t = session_.config();
    t.autoReclaim = pc_.teleop.autoReclaim != 0;
    t.gripperOpenRaw = pc_.teleop.gripperOpenRaw;
    t.gripperClosedRaw = pc_.teleop.gripperClosedRaw;
    t.triggerOpenRad = pc_.teleop.triggerOpenRad;
    t.triggerClosedRad = pc_.teleop.triggerClosedRad;
}

void App::queueServoTest(uint8_t step, int16_t goalRaw)
{
    if (step == kServoTestGoal && stOp_.step == kServoTestStart) { stOp_.goalRaw = goalRaw; return; }   // not started yet
    stOp_.step = step;
    stOp_.goalRaw = goalRaw;
}

void App::stopServoTest()
{
    if (stJoint_ < 0) return;
    stOp_.joint = stJoint_;
    queueServoTest(kServoTestStop, 0);
    stJoint_ = -1;
}

bool App::takeServoTestOp(ServoTestOp& op)
{
    if (stOp_.step == kServoTestNone) return false;
    op = stOp_;
    stOp_.step = kServoTestNone;
    return true;
}

void App::setFault(const char* why)
{
    stopServoTest();
    if (!fault_[0]) {
        strncpy(fault_, why, sizeof(fault_) - 1);
        fault_[sizeof(fault_) - 1] = '\0';
    }
    if (teleop_) { session_.end(lastTick_); teleop_ = false; }
    cal_.abort("fault");
    stopReplay("fault");
    ctl_.setMode(Mode::Limp);
}

bool App::canEnableTorque(char* why, size_t cap) const
{
    if (fault_[0]) { snprintf(why, cap, "fault: %s (clear_fault first)", fault_); return false; }
    for (int i = 0; i < kNumJoints; ++i)
        if (missing_[i] >= kMissingRefuse) { snprintf(why, cap, "servo J%d is not answering right now (its lead? Health shows the misses)", i + 1); return false; }
    if (!modelOk_) { snprintf(why, cap, "model self-test failed"); return false; }
    if (!fullyCalibrated(pc_)) { snprintf(why, cap, "not calibrated: capture every joint at its stop (calib_stop), or calib_zero"); return false; }
    return true;
}

bool App::busy(char* why, size_t cap, bool needLimp) const
{
    if (cal_.active()) { snprintf(why, cap, "a calibration is running (calib_abort first)"); return true; }
    if (teleop_) { snprintf(why, cap, "teleoperation is running (teleop_stop first)"); return true; }
    if (replaying()) { snprintf(why, cap, "a replay is running (replay_stop first)"); return true; }
    if (needLimp && ctl_.mode() != Mode::Limp) { snprintf(why, cap, "set mode limp first: this changes how torque is computed"); return true; }
    return false;
}

bool App::prepareSave(PersistentConfig& snapshot, char* why, size_t cap) const
{
    // writing flash stalls both cores for tens of ms (the cache is off meanwhile): only while the arm is quiet
    if (busy(why, cap, false)) return false;
    if (ctl_.moving()) { snprintf(why, cap, "a move is running"); return false; }
    snapshot = pc_;
    return true;
}

void App::finishSave(const PersistentConfig& snapshot, bool ok)
{
    if (ok && memcmp(&snapshot, &pc_, sizeof(pc_)) == 0) dirty_ = false;
}

bool App::maintenanceAllowed(char* why, size_t cap) const
{
    if (busy(why, cap, false)) return false;
    if (ctl_.moving()) { snprintf(why, cap, "a move is running"); return false; }
    return true;
}

void App::applyCalibration(const CalibResult& r)
{
    strncpy(lastCalib_, r.msg, sizeof(lastCalib_) - 1);
    lastCalib_[sizeof(lastCalib_) - 1] = '\0';
    lastCalibOk_ = r.ok;
    ++calibSeq_;
    if (r.ok) dirty_ = true;
    if (!r.ok && r.limp) ctl_.setMode(Mode::Limp);   // let go: the bench procedure has the person supporting the arm
    if (r.ok) {
        if (r.kind == CalibKind::Zero) {
            for (int i = 0; i < kNumJoints; ++i) { pc_.joint[i].zeroTicks = r.zeroTicks[i]; pc_.joint[i].flags |= CAL_ZERO; }
            resetUnwrap();
        } else if (r.kind == CalibKind::Stop) {
            pc_.joint[r.joint].zeroTicks = r.zeroTicks[r.joint]; pc_.joint[r.joint].flags |= CAL_ZERO;
            resetUnwrap();
        } else if (r.kind == CalibKind::Torque) {
            pc_.joint[r.joint].ktNmPerA = r.ktNmPerA; pc_.joint[r.joint].frictionNm = r.frictionNm; pc_.joint[r.joint].flags |= CAL_KT;
            if (r.currentRegPerUnit > 0.0f) pc_.currentRegPerUnit = r.currentRegPerUnit;
        } else if (r.kind == CalibKind::Spring) {
            pc_.spring.scale *= r.springScale;
        } else if (r.kind == CalibKind::Range) {
            for (int i = 0; i < kNumJoints; ++i) {
                if (!(r.rangeMask & (1u << i))) continue;
                pc_.joint[i].qMinRad = r.qMinRad[i]; pc_.joint[i].qMaxRad = r.qMaxRad[i]; pc_.joint[i].flags |= CAL_RANGE;
            }
        }
        rebuildControlConfig();          // takes effect at once; "save" stores it
    }
}

void App::stopReplay(const char* why)
{
    if (replay_.state == Replay::Idle) return;
    replay_.state = Replay::Idle;
    snprintf(replayNote_, sizeof(replayNote_), "%s", why);
    if (teleop_ && session_.stage() == TeleopSession::Stage::Coupled) ctl_.setMode(Mode::Bilateral);
    else if (ctl_.mode() == Mode::Move) ctl_.setMode(Mode::Gravity);
}

void App::replayTick(float dt)
{
    if (replay_.state == Replay::Idle) return;
    if (ctl_.mode() != Mode::Move) { stopReplay("stopped: the arm was moved by hand"); return; }
    if (teleop_ && session_.claimed() && session_.stage() != TeleopSession::Stage::Coupled) { stopReplay("stopped: teleoperation not engaged"); return; }
    if (replay_.state == Replay::ToStart) {
        if (ctl_.moveActive()) return;
        replay_.state = Replay::Playing;
        replay_.t = 0.0f;
    }
    const float pos = replay_.t * replay_.speed / replay_.dtS;
    const int k = (int)pos;
    float q[kNumJoints], qd[kNumJoints];
    if (k >= replay_.n - 1) {
        for (int i = 0; i < kNumJoints; ++i) { q[i] = replay_.q[replay_.n - 1][i] * 0.01f / kDeg; qd[i] = 0.0f; }
        ctl_.track(q, qd);
        stopReplay("done");
        return;
    }
    const float u = pos - (float)k;
    for (int i = 0; i < kNumJoints; ++i) {
        const float a = replay_.q[k][i] * 0.01f / kDeg, b = replay_.q[k + 1][i] * 0.01f / kDeg;
        q[i] = a + u * (b - a);
        qd[i] = (b - a) / replay_.dtS * replay_.speed;
    }
    ctl_.track(q, qd);
    replay_.t += dt;
}

void App::controlTick(const RawServoState& raw, double now, int16_t rawOut[kNumJoints], bool& torqueOn, bool& needCurrentMode)
{
    const float dt = haveTick_ ? (float)(now - lastTick_) : 0.005f;
    lastTick_ = now;
    haveTick_ = true;
    const float dts = dt > 0.0f && dt < 0.1f ? dt : 0.005f;

    // servo health
    for (int i = 0; i < kNumJoints; ++i) if (raw.ok[i]) lastRaw_[i] = raw.pos[i];
    bool lost = false;
    const int faultAfter = cal_.active() ? kMissingFaultCalib : (torqueWasOn_ || stJoint_ >= 0) ? kMissingFault : 0;   // 0: limp, never
    for (int i = 0; i < kNumJoints; ++i) {
        if (raw.ok[i] && missing_[i] >= kRewrapMissing) { haveWrap_[i] = false; turns_[i] = 0; }
        missing_[i] = raw.ok[i] ? 0 : (uint8_t)(missing_[i] < 255 ? missing_[i] + 1 : 255);
        if (!raw.ok[i]) ++missedTotal_[i];
        if (faultAfter && missing_[i] >= faultAfter) lost = true;
    }
    if (lost) {   // name every servo missing right now: J4 alone points at its own lead, J4..J7 at the chain cable before J4
        char m[80];
        int n = snprintf(m, sizeof(m), "servo");
        for (int i = 0; i < kNumJoints && n > 0 && n < (int)sizeof(m); ++i)
            if (missing_[i]) n += snprintf(m + n, sizeof(m) - (size_t)n, " J%d", i + 1);
        if (n > 0 && n < (int)sizeof(m)) snprintf(m + n, sizeof(m) - (size_t)n, " not answering");
        setFault(m);
    }
    for (int i = 0; i < kNumJoints; ++i) {
        if (raw.ok[i] && raw.tempC[i] >= kTempFaultC) { char m[64]; snprintf(m, sizeof(m), "servo J%d at %d C", i + 1, raw.tempC[i]); setFault(m); }
    }

    // raw -> joint space (CAD convention) with the stored zero and sign, unwrapped across +-180 deg
    for (int i = 0; i < kNumJoints; ++i) {
        if (!raw.ok[i]) { in_.qd[i] = 0.0f; continue; }   // the last angle stays; no damping or friction push from an old speed
        const JointCal& c = pc_.joint[i];
        const float w = (float)c.dir * (float)ticksFromZero(raw.pos[i], c.zeroTicks) * kRadPerTick;
        if (haveWrap_[i]) {
            const float d = w - lastWrapped_[i];
            if (d > kPi) --turns_[i];
            else if (d < -kPi) ++turns_[i];
        }
        lastWrapped_[i] = w;
        haveWrap_[i] = true;
        in_.q[i] = w + 2.0f * kPi * (float)turns_[i];
        in_.qd[i] = (float)c.dir * (float)raw.speed[i] * kRadPsPerSpeedUnit;
        in_.tempC[i] = (float)raw.tempC[i];
        const float regPerUnit = pc_.currentRegPerUnit > 0.0f ? pc_.currentRegPerUnit : 1.0f;
        currentmA_[i] = (int16_t)lroundf((float)(kCurrentSign * c.dir) * (float)raw.current[i] * pc_.mAPerUnit / regPerUnit);
        voltage_[i] = raw.voltage[i];
    }

    // operator inputs: engaged = enable held (when required) and no clutch; an enable press also resumes after an outage
    const bool engaged = (!pc_.teleop.requireEnable || enableDown_) && !clutchDown_ && !uiHold_;
    if (teleop_) {
        session_.setEngaged(engaged);
        if (enableDown_ && !enableWas_ && session_.stage() == TeleopSession::Stage::Lost) session_.resume(now);
    }
    enableWas_ = enableDown_;

    in_.followerFresh = false;
    in_.extValid = false;
    if (teleop_) session_.beforeControl(in_, now);

    // haptic barriers: the usable range while coupled (RAFT's range and PAR6's mapped limits), else RAFT's own
    // range if asked for
    if (teleop_ && session_.stage() == TeleopSession::Stage::Coupled) {
        for (int i = 0; i < kNumJoints; ++i) { float lo, hi; session_.usableRange(i, lo, hi); ctl_.setBarrier(i, lo, hi); }
    } else if (pc_.teleop.barriersAlways) {
        for (int i = 0; i < kNumJoints; ++i) ctl_.setBarrier(i, cc_.joint[i].qMinRad, cc_.joint[i].qMaxRad);
    } else {
        ctl_.clearBarriers();
    }

    replayTick(dts);
    ctl_.step(in_, dts, out_);

    CalibOverride ov = {false, -1, 0};
    cal_.tick(raw, in_, out_, dt, ov);
    if (ov.active) {
        ctl_.setRawOverride(ov.joint, ov.raw);
        out_.raw[ov.joint] = (int16_t)lroundf(out_.derate[ov.joint] * (float)ov.raw);
    } else {
        ctl_.clearRawOverride();
    }
    {
        CalibResult r;
        if (cal_.takeResult(r)) applyCalibration(r);
    }
    if (teleop_) session_.afterControl(in_, now);

    const bool on = ctl_.mode() != Mode::Limp || ov.active;
    // Hold, moves and replay: the servos' own position mode, per joint, while the goal stays clear of the encoder wrap
    // (a joint that would cross it keeps the computed Hold torque). Not during a calibration (it drives one joint by
    // current) and only with torque on.
    const uint8_t prevMask = goals_.mask;
    goals_.mask = 0;
    goals_.speed = (uint16_t)lroundf(180.0f * kPi / 180.0f / kRadPsPerSpeedUnit);   // 180 deg/s: above any planned move
    goals_.acc = 50;
    goals_.limit = (uint16_t)cc_.rawLimit;                                         // the stall current
    if (holdPosition_ && on && ctl_.mode() == Mode::Move && !cal_.active()) {
        for (int i = 0; i < kNumJoints; ++i) {
            // a missed read: the servo stays in position mode on its last goal (switching modes takes acknowledged
            // writes from that same servo, and drops its torque for a moment)
            if (!raw.ok[i]) { goals_.mask |= (uint8_t)(prevMask & (1u << i)); continue; }
            const int cur = raw.pos[i];
            const long g = cur + pc_.joint[i].dir * lroundf((out_.qRef[i] - in_.q[i]) / kRadPerTick);
            const int lo = kPositionWrapMarginTicks, hi = 4095 - kPositionWrapMarginTicks;
            if (cur < lo || cur > hi || g < lo || g > hi) continue;
            goals_.mask |= (uint8_t)(1u << i);
            goals_.raw[i] = (int16_t)g;
        }
    }
    if (!on) servoModes_ = 0;
    needCurrentMode = on && !torqueWasOn_;
    torqueWasOn_ = on;
    torqueOn = on;
    for (int i = 0; i < kNumJoints; ++i) rawOut[i] = on ? out_.raw[i] : 0;

    // leader frame snapshot
    const bool fresh = teleop_ && link_.stateFresh(now);
    const rti::State& st = link_.state();
    frame_.mode = (uint8_t)ctl_.mode();
    uint16_t fl = 0;
    if (fullyCalibrated(pc_)) fl |= LF_CALIBRATED;
    if (on) fl |= LF_TORQUE_ON;
    if (fault_[0]) fl |= LF_FAULT;
    if (teleop_ && session_.stage() == TeleopSession::Stage::Coupled) fl |= LF_COUPLED;
    if (pc_.spring.type != SpringType::None) fl |= LF_SPRING;
    if (modelOk_) fl |= LF_MODEL_OK;
    if (cal_.active()) fl |= LF_CALIBRATING;
    for (int i = 0; i < kNumJoints; ++i) if (out_.derate[i] < 1.0f) fl |= LF_DERATING;
    if (fresh) fl |= LF_FOLLOWER;
    frame_.flags = fl;
    frame_.tUs = (uint32_t)(now * 1e6);
    for (int i = 0; i < kNumJoints; ++i) {
        frame_.q[i] = in_.q[i]; frame_.qd[i] = in_.qd[i];
        frame_.tauCmd[i] = out_.tauCmd[i]; frame_.tauGravity[i] = out_.tauGravity[i];
        frame_.tempC[i] = raw.tempC[i];
        frame_.currentmA[i] = currentmA_[i];
        frame_.voltage[i] = voltage_[i];
        frame_.qFollower[i] = session_.followerQ()[i];
    }
    float rpy[3];
    rpyXyz(out_.frames[6].R, rpy);
    for (int k = 0; k < 3; ++k) { frame_.tcp[k] = out_.frames[6].p[k]; frame_.tcp[3 + k] = rpy[k]; }
    frame_.stage = teleop_ ? (uint8_t)session_.stage() : 0;
    uint8_t tf = 0;
    if (engaged) tf |= TF_ENGAGED;
    if (fresh && (st.present & rti::F_TEXT)) tf |= TF_EXT_VALID;
    if (ctl_.orientLock()) tf |= TF_ORIENT_LOCK;
    if (replaying()) tf |= TF_REPLAYING;
    if (uiHold_) tf |= TF_HOLD;
    if (enableDown_) tf |= TF_ENABLE_BTN;
    if (clutchDown_) tf |= TF_CLUTCH_BTN;
    frame_.teleopFlags = tf;
    frame_.locks = session_.locks();
    for (int i = 0; i < 6; ++i) {
        frame_.followerTarget[i] = session_.followerTarget()[i];
        frame_.tauExt[i] = (st.present & rti::F_TEXT) ? (float)st.text[i] : 0.0f;
    }
    frame_.gripperRaw = (st.present & rti::F_GP) ? (float)st.gp : -1.0f;
    frame_.followerTick = st.tick;
    frame_.precision = session_.precision();
    frame_.guide = (uint8_t)ctl_.guide();
    frame_.pcEnergyJ = out_.pcEnergyJ;
    frame_.offsetRad = teleop_ && session_.claimed() ? session_.offsetMaxRad() : 0.0f;
    for (int k = 0; k < kFrameConsumers; ++k) events_[k] |= out_.events;   // kept until that consumer takes a frame
    scopeTick(now);
}

void App::networkTick(double now)
{
    if (teleop_) link_.poll(now);
}

size_t App::leaderFrame(uint8_t* buf, size_t cap, int consumer)
{
    const int c = consumer < 0 || consumer >= kFrameConsumers ? 0 : consumer;
    frame_.seq = ++seq_[c];
    frame_.events = events_[c];
    const size_t n = packLeaderFrame(frame_, buf, cap);
    if (n) events_[c] = 0;
    return n;
}

size_t App::reply(char* out, size_t cap, bool ok, const char* fmt, ...) const
{
    int n = snprintf(out, cap, "{\"ok\": %s", ok ? "true" : "false");
    if (n < 0 || (size_t)n >= cap) return 0;
    if (fmt && fmt[0]) {
        out[n++] = ',';
        out[n++] = ' ';
        va_list ap;
        va_start(ap, fmt);
        const int m = vsnprintf(out + n, cap - (size_t)n, fmt, ap);
        va_end(ap);
        if (m < 0 || (size_t)(n + m) >= cap - 2) return 0;
        n += m;
    }
    out[n++] = '}';
    out[n] = '\0';
    return (size_t)n;
}

void App::snapshot(StatusSnapshot& s) const
{
    s.mode = ctl_.mode();
    memcpy(s.fault, fault_, sizeof(s.fault));
    s.modelOk = modelOk_;
    s.calibrated = fullyCalibrated(pc_);
    memcpy(&s.pc, &pc_, sizeof(pc_));
    for (int i = 0; i < kNumJoints; ++i) {
        s.q[i] = in_.q[i];
        s.tempC[i] = in_.tempC[i];
        s.jointGravity[i] = jointGravity_[i];
        s.jointDamping[i] = jointDamping_[i];
        s.missed[i] = missedTotal_[i];
        s.qMin[i] = cc_.joint[i].qMinRad;
        s.qMax[i] = cc_.joint[i].qMaxRad;
        s.followerQ[i] = session_.followerQ()[i];
    }
    s.tcp = out_.frames[6];
    s.j2mA = fabsf(out_.tauCmd[1]) / cc_.joint[1].ktNmPerA * 1000.0f;
    s.stJoint = stJoint_;
    s.stGoalDeg = stGoalDeg_;
    s.holdPosition = holdPosition_;
    s.dirty = dirty_;
    s.servoModes = servoModes_;
    s.holdStiffness = holdStiffness_;
    memcpy(s.streamHost, streamHost_, sizeof(s.streamHost));
    s.calib = cal_.phaseText();
    s.calibJoint = cal_.joint() + 1;
    memcpy(s.lastCalib, lastCalib_, sizeof(s.lastCalib));
    s.lastCalibOk = lastCalibOk_;
    s.calibSeq = calibSeq_;
    s.teleop = teleop_;
    s.stage = teleop_ ? TeleopSession::stageName(session_.stage()) : "OFF";
    s.linkPhase = rti::phaseName(link_.phase());
    strncpy(s.note, teleop_ ? session_.note() : "", sizeof(s.note) - 1);
    s.note[sizeof(s.note) - 1] = '\0';
    strncpy(s.linkError, link_.lastError(), sizeof(s.linkError) - 1);
    s.linkError[sizeof(s.linkError) - 1] = '\0';
    s.staleStates = link_.staleStates();
    s.staleTargets = link_.staleTargets();
    s.linkRateHz = link_.config().rateHz;
    s.filterHz = session_.filterHz();
    s.followerFresh = teleop_ && link_.stateFresh(lastTick_);
    s.engaged = (!pc_.teleop.requireEnable || enableDown_) && !clutchDown_ && !uiHold_;
    s.uiHold = uiHold_;
    s.orientLock = ctl_.orientLock();
    s.offsetAlign = session_.offsetAlign();
    s.precision = session_.precision();
    s.offsetDeg = teleop_ && session_.claimed() ? session_.offsetMaxRad() * kDeg : 0.0f;
    s.locks = session_.locks();
    s.guide = guideName(ctl_.guide());
    s.hasEnable = hasEnable_;
    s.hasClutch = hasClutch_;
    s.enableDown = enableDown_;
    s.clutchDown = clutchDown_;
    s.replayN = replay_.n;
    s.replayState = (uint8_t)replay_.state;
    s.replayDtS = replay_.dtS;
    s.replayT = replay_.t;
    s.replaySpeed = replay_.speed;
    memcpy(s.replayNote, replayNote_, sizeof(s.replayNote));
}

size_t App::status(char* out, size_t cap) const
{
    StatusSnapshot s;
    snapshot(s);
    return formatStatus(s, out, cap);
}

size_t App::formatStatus(const StatusSnapshot& s, char* out, size_t cap)
{
    Out o(out, cap);
    const PersistentConfig& pc = s.pc;
    float rpy[3];
    rpyXyz(s.tcp.R, rpy);
    char pi[40], st[40];
    jsonSafe(pc.piHost, pi, sizeof(pi));
    jsonSafe(s.streamHost, st, sizeof(st));
    char name[sizeof(pc.armName)];
    jsonSafe(pc.armName, name, sizeof(name));
    o.add("{\"ok\": true, \"name\": \"%s\", \"mode\": \"%s\", \"fault\": \"%s\", \"model_ok\": %s, \"calibrated\": %s, \"cal_flags\": [",
          name, modeName(s.mode), s.fault, s.modelOk ? "true" : "false", s.calibrated ? "true" : "false");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%d", i ? ", " : "", (int)pc.joint[i].flags);
    o.add("], ");
    o.degs("q_deg", s.q, kNumJoints);
    o.add(", \"tcp_mm\": [%.1f, %.1f, %.1f], \"rpy_deg\": [%.1f, %.1f, %.1f], \"temp_c\": [",
          (double)(s.tcp.p[0] * 1e3f), (double)(s.tcp.p[1] * 1e3f), (double)(s.tcp.p[2] * 1e3f),
          (double)(rpy[0] * kDeg), (double)(rpy[1] * kDeg), (double)(rpy[2] * kDeg));
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%d", i ? ", " : "", (int)s.tempC[i]);
    o.add("], \"j2_mA\": %.0f, \"spring\": {\"type\": %d, \"k\": %.1f, \"a_mm\": %.1f, \"b_mm\": %.1f, \"scale\": %.3f}, \"kt\": [",
          (double)s.j2mA, (int)pc.spring.type, (double)pc.spring.k, (double)(pc.spring.a * 1e3f), (double)(pc.spring.b * 1e3f), (double)pc.spring.scale);
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%.3f", i ? ", " : "", (double)pc.joint[i].ktNmPerA);
    o.add("], \"dir\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%d", i ? ", " : "", (int)pc.joint[i].dir);
    o.add("], \"joint_gravity\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%.2f", i ? ", " : "", (double)s.jointGravity[i]);
    o.add("], \"joint_damping\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%.3f", i ? ", " : "", (double)s.jointDamping[i]);
    if (s.stJoint >= 0) o.add("], \"servo_test\": {\"joint\": %d, \"goal_deg\": %.2f}, \"missed\": [", s.stJoint + 1, (double)s.stGoalDeg);
    else o.add("], \"servo_test\": null, \"missed\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%lu", i ? ", " : "", (unsigned long)s.missed[i]);
    o.add("], \"friction_nm\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%.3f", i ? ", " : "", (double)pc.joint[i].frictionNm);
    o.add("], \"range_deg\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s[%.1f, %.1f]", i ? ", " : "", (double)(s.qMin[i] * kDeg), (double)(s.qMax[i] * kDeg));
    o.add("], \"stop_deg\": [");
    for (int i = 0; i < kNumJoints; ++i) o.add("%s%.2f", i ? ", " : "", (double)kStopDeg[i]);
    o.add("], \"cur_reg_per_unit\": %.3f, \"gravity_scale\": %.3f, \"hold_friction\": %.2f, \"hold_position\": %s, \"pos_mask\": %u, \"hold_stiffness\": %.2f, \"auto_rest\": %s, \"idle_s\": %.0f, \"ma_per_unit\": %.2f, "
          "\"pi_host\": \"%s\", \"stream\": \"%s:%u\", \"unsaved\": %s, ",
          (double)pc.currentRegPerUnit, (double)pc.gravityScale, (double)pc.holdFriction, s.holdPosition ? "true" : "false", (unsigned)s.servoModes, (double)s.holdStiffness, pc.autoRest ? "true" : "false", (double)pc.autoRestIdleS,
          (double)pc.mAPerUnit, pi, st, (unsigned)pc.streamPort, s.dirty ? "true" : "false");
    o.add("\"calib\": \"%s\", \"calib_joint\": %d, \"last_calib\": \"%s\", \"last_calib_ok\": %s, \"calib_seq\": %u, ",
          s.calib, s.calibJoint, s.lastCalib, s.lastCalibOk ? "true" : "false", (unsigned)s.calibSeq);
    // teleoperation
    const TeleopSettings& ts = pc.teleop;
    char note[120];
    jsonSafe(s.note, note, sizeof(note));
    o.add("\"teleop\": \"%s\", \"teleop_note\": \"%s\", \"link\": \"%s\", \"link_error\": \"%s\", \"link_stale\": [%u, %u], ",
          s.stage, note, s.linkPhase, s.linkError, (unsigned)s.staleStates, (unsigned)s.staleTargets);
    o.add("\"link_rate_hz\": %.0f, \"link_mode\": \"%s\", \"teleop_filter_hz\": %.1f, ",
          s.linkRateHz > 0.0f ? (double)s.linkRateHz : 250.0, s.linkRateHz > 0.0f ? "SOFT_RTI" : "RTI", (double)s.filterHz);
    if (s.followerFresh) { o.degs("follower_deg", s.followerQ, kNumJoints); o.add(", "); }
    else o.add("\"follower_deg\": null, ");
    o.add("\"engaged\": %s, \"hold\": %s, \"precision\": %.2f, \"locks\": %u, \"orient_lock\": %s, \"guide\": \"%s\", \"offset_deg\": %.1f, "
          "\"offset_align\": %s, ",
          s.engaged ? "true" : "false", s.uiHold ? "true" : "false", (double)s.precision, (unsigned)s.locks, s.orientLock ? "true" : "false", s.guide,
          (double)s.offsetDeg, s.offsetAlign ? "true" : "false");
    o.add("\"settings\": {\"auto_reclaim\": %s, \"require_enable\": %s, \"passivity\": %s, \"barriers_always\": %s, \"force_gain\": %.3f, "
          "\"force_sign\": %d, \"force_deadband\": %.2f, \"friction_scale\": %.2f, \"trigger_deg\": [%.1f, %.1f], \"gripper_raw\": [%u, %u]}, ",
          ts.autoReclaim ? "true" : "false", ts.requireEnable ? "true" : "false", ts.passivity ? "true" : "false", ts.barriersAlways ? "true" : "false",
          (double)ts.forceGain, (int)ts.forceSign, (double)ts.forceDeadbandNm, (double)ts.frictionScale,
          (double)(ts.triggerOpenRad * kDeg), (double)(ts.triggerClosedRad * kDeg), (unsigned)ts.gripperOpenRaw, (unsigned)ts.gripperClosedRaw);
    o.add("\"buttons\": {\"enable\": %s, \"clutch\": %s, \"enable_down\": %s, \"clutch_down\": %s}, ",
          s.hasEnable ? "true" : "false", s.hasClutch ? "true" : "false", s.enableDown ? "true" : "false", s.clutchDown ? "true" : "false");
    const char* rs = s.replayState == Replay::Playing ? "playing" : (s.replayState == Replay::ToStart ? "to_start" : "idle");
    o.add("\"replay\": {\"n\": %d, \"dt_ms\": %.0f, \"state\": \"%s\", \"t\": %.2f, \"len_s\": %.2f, \"speed\": %.2f, \"note\": \"%s\"}}",
          s.replayN, (double)(s.replayDtS * 1e3f), rs, (double)(s.replayT * s.replaySpeed),
          (double)(s.replayN > 1 ? (s.replayN - 1) * s.replayDtS : 0.0f), (double)s.replaySpeed, s.replayNote);
    return o.full ? 0 : o.n;
}

size_t App::statusLine(char* buf, size_t cap) const
{
    StatusSnapshot s;
    snapshot(s);
    return formatStatusLine(s, buf, cap);
}

size_t App::formatStatusLine(const StatusSnapshot& s, char* buf, size_t cap)
{
    const int n = snprintf(buf, cap, "mode %s%s%s | q deg %.1f %.1f %.1f %.1f %.1f %.1f %.1f | J2 %.0f mA | calib %s | teleop %s/%s",
                           modeName(s.mode), s.fault[0] ? " FAULT " : "", s.fault,
                           (double)(s.q[0] * kDeg), (double)(s.q[1] * kDeg), (double)(s.q[2] * kDeg),
                           (double)(s.q[3] * kDeg), (double)(s.q[4] * kDeg), (double)(s.q[5] * kDeg),
                           (double)(s.q[6] * kDeg), (double)s.j2mA, s.calib, s.stage, s.linkPhase);
    return n > 0 ? (size_t)n : 0;
}

namespace {
struct Fnv {                         // FNV-1a, 32 bit
    uint32_t h = 2166136261u;
    void bytes(const void* p, size_t n)
    {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 16777619u; }
    }
    template <typename T> void val(const T& v) { bytes(&v, sizeof(v)); }
    void str(const char* s) { if (s) bytes(s, strlen(s) + 1); else val(0); }
};
}  // namespace

uint32_t App::statusKey(const StatusSnapshot& s)
{
    // field by field: the snapshot's padding bytes are not defined. Strings to their end, static texts by content.
    Fnv f;
    f.val(s.mode); f.str(s.fault); f.val(s.modelOk); f.val(s.calibrated);
    f.bytes(&s.pc, sizeof(s.pc));    // a memcpy of the App's settings: its padding only changes with them
    f.bytes(s.jointGravity, sizeof(s.jointGravity)); f.bytes(s.jointDamping, sizeof(s.jointDamping));
    f.val(s.stJoint); f.val(s.stGoalDeg);
    f.bytes(s.qMin, sizeof(s.qMin)); f.bytes(s.qMax, sizeof(s.qMax));
    f.val(s.holdPosition); f.val(s.dirty); f.val(s.servoModes); f.val(s.holdStiffness); f.str(s.streamHost);
    f.str(s.calib); f.val(s.calibJoint); f.str(s.lastCalib); f.val(s.lastCalibOk); f.val(s.calibSeq);
    f.val(s.teleop); f.str(s.stage); f.str(s.linkPhase); f.str(s.note); f.str(s.linkError);
    f.val(s.linkRateHz); f.val(s.filterHz);
    f.val(s.engaged); f.val(s.uiHold); f.val(s.orientLock); f.val(s.offsetAlign); f.val(s.precision); f.val(s.locks); f.str(s.guide);
    f.val(s.hasEnable); f.val(s.hasClutch); f.val(s.enableDown); f.val(s.clutchDown);
    f.val(s.replayN); f.val(s.replayState); f.val(s.replayDtS); f.val(s.replaySpeed); f.str(s.replayNote);
    return f.h;
}

size_t App::handleCommand(const char* line, char* out, size_t cap, double now, const char* clientIp)
{
    PersistentConfig before;
    memcpy(&before, &pc_, sizeof(pc_));
    const size_t n = handleCommandImpl(line, out, cap, now, clientIp);
    char cmd[16] = "";
    json::getString(line, "cmd", cmd, sizeof(cmd));
    if (!strcmp(cmd, "save") || !strcmp(cmd, "load")) {
        if (n && strstr(out, "\"ok\": true")) dirty_ = false;
    } else if (memcmp(&before, &pc_, sizeof(pc_)) != 0) {
        dirty_ = true;
    }
    return n;
}

size_t App::handleCommandImpl(const char* line, char* out, size_t cap, double now, const char* clientIp)
{
    char cmd[32];
    if (!json::getString(line, "cmd", cmd, sizeof(cmd))) return reply(out, cap, false, "\"error\": \"no cmd\"");
    char why[120];
    double v = 0.0;
    bool b = false;
    int joint = 0;
    {
        double jd;
        if (json::getNumber(line, "joint", jd)) joint = (int)jd;      // 1-based
    }
    const bool jointOk = joint >= 1 && joint <= kNumJoints;

    if (!strcmp(cmd, "status")) return status(out, cap);

    if (!strcmp(cmd, "mode")) {
        char m[16];
        if (!json::getString(line, "mode", m, sizeof(m))) return reply(out, cap, false, "\"error\": \"mode: limp|gravity|move\"");
        if (!strcmp(m, "limp")) {
            stopServoTest();
            if (teleop_) { session_.end(now); teleop_ = false; }
            cal_.abort("stopped");
            stopReplay("stopped");
            ctl_.setMode(Mode::Limp);
            return reply(out, cap, true, "\"mode\": \"limp\"");
        }
        if (!canEnableTorque(why, sizeof(why))) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (teleop_) return reply(out, cap, false, "\"error\": \"teleoperation is running (teleop_stop first)\"");
        if (stJoint_ >= 0) return reply(out, cap, false, "\"error\": \"a servo position test is running (stop it first)\"");
        stopReplay("stopped");
        if (!strcmp(m, "gravity")) { ctl_.setMode(Mode::Gravity); return reply(out, cap, true, "\"mode\": \"gravity\""); }
        if (!strcmp(m, "move")) { ctl_.setMode(Mode::Move); return reply(out, cap, true, "\"mode\": \"move\", \"note\": \"holding the current pose\""); }
        return reply(out, cap, false, "\"error\": \"unknown mode %s\"", m);
    }
    if (!strcmp(cmd, "move") || !strcmp(cmd, "rest")) {
        if (!canEnableTorque(why, sizeof(why))) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (teleop_) return reply(out, cap, false, "\"error\": \"teleoperation is running\"");
        if (replaying()) return reply(out, cap, false, "\"error\": \"a replay is running\"");
        if (cal_.active()) return reply(out, cap, false, "\"error\": \"a calibration is running\"");
        float q[kNumJoints];
        for (int i = 0; i < kNumJoints; ++i) q[i] = in_.q[i];
        if (!strcmp(cmd, "rest")) {
            for (int i = 0; i < kNumJoints; ++i) q[i] = cc_.autoRest.qRest[i];
        } else {
            double a[kNumJoints];
            int n = json::getNumberArray(line, "q_deg", a, kNumJoints);
            const float scale = (n >= 0) ? kPi / 180.0f : 1.0f;
            if (n < 0) n = json::getNumberArray(line, "q", a, kNumJoints);
            if (n < 6) return reply(out, cap, false, "\"error\": \"move needs q_deg (or q in rad) with 6 or 7 values\"");
            for (int i = 0; i < n; ++i) q[i] = (float)a[i] * scale;
        }
        float speed = 0.0f;
        if (json::getNumber(line, "speed_deg", v) && v > 0.0 && v <= 180.0) speed = (float)v * kPi / 180.0f;
        if (ctl_.mode() == Mode::Limp) ctl_.setMode(Mode::Gravity);
        if (!ctl_.moveTo(q, speed)) return reply(out, cap, false, "\"error\": \"no joint state yet\"");
        return reply(out, cap, true, "\"mode\": \"move\"");
    }
    if (!strcmp(cmd, "gravity_scale")) {
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (!json::getNumber(line, "value", v) || v < 0.0 || v > 1.5) return reply(out, cap, false, "\"error\": \"value 0..1.5\"");
        pc_.gravityScale = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"gravity_scale\": %.3f", v);
    }
    if (!strcmp(cmd, "hold_friction")) {
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (!json::getNumber(line, "value", v) || v < 0.0 || v > 1.0) return reply(out, cap, false, "\"error\": \"value 0..1\"");
        pc_.holdFriction = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"hold_friction\": %.3f", v);
    }
    if (!strcmp(cmd, "hold_stiffness")) {
        if (!json::getNumber(line, "value", v) || v < 0.25 || v > 8.0) return reply(out, cap, false, "\"error\": \"value 0.25..8\"");
        holdStiffness_ = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"hold_stiffness\": %.2f", v);
    }
    if (!strcmp(cmd, "servo_test")) {
        bool on = true;
        json::getBool(line, "on", on);
        if (!on) { stopServoTest(); return reply(out, cap, true, "\"servo_test\": null"); }
        if (!jointOk) return reply(out, cap, false, "\"error\": \"joint 1..7\"");
        if (stJoint_ >= 0) return reply(out, cap, false, "\"error\": \"a servo test is running (servo_test off first)\"");
        if (missing_[joint - 1]) return reply(out, cap, false, "\"error\": \"servo J%d is not answering right now\"", joint);
        if (busy(why, sizeof(why), true)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        double sp = 30.0, tq = 650.0;
        json::getNumber(line, "speed_deg", sp);
        json::getNumber(line, "torque_ma", tq);
        if (!(sp >= 1.0 && sp <= 300.0) || !(tq >= 50.0 && tq <= kStallCurrentMa))
            return reply(out, cap, false, "\"error\": \"speed_deg 1..300, torque_ma 50..1300\"");
        const int j = joint - 1;
        stJoint_ = (int8_t)j;
        stStartRaw_ = lastRaw_[j];
        stGoalDeg_ = 0.0f;
        stOp_.joint = (int8_t)j;
        stOp_.speed = (uint16_t)fmax(1.0, lround(sp * kPi / 180.0 / kRadPsPerSpeedUnit));   // servo speed units
        stOp_.acc = 20;                                                                       // about 170 deg/s^2
        stOp_.torque = (uint16_t)lround(tq / (pc_.mAPerUnit > 0.5f ? pc_.mAPerUnit : 3.25f));  // current register units
        queueServoTest(kServoTestStart, stStartRaw_);
        return reply(out, cap, true, "\"servo_test\": {\"joint\": %d, \"start_raw\": %d, \"speed_deg\": %.0f, \"torque_ma\": %.0f}",
                     joint, stStartRaw_, sp, tq);
    }
    if (!strcmp(cmd, "servo_jog")) {
        if (stJoint_ < 0) return reply(out, cap, false, "\"error\": \"no servo test running\"");
        if (!json::getNumber(line, "deg", v) || !(v >= -180.0 && v <= 180.0)) return reply(out, cap, false, "\"error\": \"deg -180..180 from the start\"");
        const int j = stJoint_;
        long g = stStartRaw_ + pc_.joint[j].dir * lround(v * kPi / 180.0 / kRadPerTick);
        const bool clamped = g < 0 || g > 4095;   // the servo's single-turn goal range: never ask it to wrap
        g = g < 0 ? 0 : (g > 4095 ? 4095 : g);
        stGoalDeg_ = (float)(pc_.joint[j].dir * (g - stStartRaw_) * kRadPerTick * 180.0f / kPi);
        queueServoTest(kServoTestGoal, (int16_t)g);
        return reply(out, cap, true, "\"goal_deg\": %.2f, \"goal_raw\": %ld%s", (double)stGoalDeg_, g, clamped ? ", \"note\": \"clamped at the encoder's end\"" : "");
    }
    if (!strcmp(cmd, "hold_position")) {
        if (!json::getBool(line, "on", b)) return reply(out, cap, false, "\"error\": \"on: true | false\"");
        holdPosition_ = b;
        return reply(out, cap, true, "\"hold_position\": %s", b ? "true" : "false");
    }
    if (!strcmp(cmd, "joint_damping")) {
        if (!jointOk || !json::getNumber(line, "value", v) || v < 0.0 || v > 0.2) return reply(out, cap, false, "\"error\": \"joint 1..7, value 0..0.2 Nm s/rad\"");
        jointDamping_[joint - 1] = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"joint\": %d, \"joint_damping\": %.3f", joint, v);
    }
    if (!strcmp(cmd, "joint_gravity")) {
        if (!jointOk || !json::getNumber(line, "value", v) || v < 0.0 || v > 3.0) return reply(out, cap, false, "\"error\": \"joint 1..7, value 0..3\"");
        jointGravity_[joint - 1] = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"joint\": %d, \"joint_gravity\": %.2f", joint, v);
    }
    if (!strcmp(cmd, "spring")) {
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        SpringModel s = pc_.spring;
        char type[16] = "";
        json::getString(line, "type", type, sizeof(type));
        if (!strcmp(type, "none")) s.type = SpringType::None;
        else if (!strcmp(type, "cable")) s.type = SpringType::ZeroLengthCable;
        else if (!strcmp(type, "torsion")) s.type = SpringType::OneSidedTorsion;
        if (json::getNumber(line, "k", v)) s.k = (float)v;
        if (json::getNumber(line, "a_mm", v)) s.a = (float)v * 1e-3f;
        if (json::getNumber(line, "b_mm", v)) s.b = (float)v * 1e-3f;
        if (json::getNumber(line, "q0_deg", v)) s.q0 = (float)v * kPi / 180.0f;
        if (json::getNumber(line, "scale", v)) s.scale = (float)v;
        if (s.k < 0.0f || s.k > 5000.0f || s.a < 0.0f || s.a > 0.1f || s.b < 0.0f || s.b > 0.1f || s.scale < 0.0f || s.scale > 2.0f)
            return reply(out, cap, false, "\"error\": \"k 0..5000, a_mm and b_mm 0..100, scale 0..2\"");
        pc_.spring = s;
        rebuildControlConfig();
        return reply(out, cap, true, "\"spring\": {\"type\": %d, \"k\": %.1f, \"a_mm\": %.1f, \"b_mm\": %.1f, \"scale\": %.3f}",
                     (int)pc_.spring.type, (double)pc_.spring.k, (double)(pc_.spring.a * 1e3f), (double)(pc_.spring.b * 1e3f), (double)pc_.spring.scale);
    }
    if (!strcmp(cmd, "auto_rest")) {
        if (json::getBool(line, "on", b)) pc_.autoRest = b ? 1 : 0;
        if (json::getNumber(line, "idle_s", v) && v >= 3.0 && v <= 3600.0) pc_.autoRestIdleS = (float)v;
        rebuildControlConfig();
        return reply(out, cap, true, "\"auto_rest\": %s, \"idle_s\": %.0f", pc_.autoRest ? "true" : "false", (double)pc_.autoRestIdleS);
    }
    // ---- calibration
    if (!strcmp(cmd, "calib_zero")) {
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (ctl_.mode() != Mode::Limp) return reply(out, cap, false, "\"error\": \"set mode limp and hold the arm in the CAD zero pose\"");
        return cal_.startZero() ? reply(out, cap, true, "\"calib\": \"zero: sampling 0.5 s\"") : reply(out, cap, false, "\"error\": \"busy\"");
    }
    if (!strcmp(cmd, "calib_dir") || !strcmp(cmd, "set_dir"))
        return reply(out, cap, false, "\"error\": \"the joint directions are fixed for this design (kJointDir in arm_design.h)\"");
    if (!strcmp(cmd, "calib_stop")) {
        if (!jointOk) return reply(out, cap, false, "\"error\": \"joint 1..7\"");
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (ctl_.mode() != Mode::Limp) return reply(out, cap, false, "\"error\": \"set mode limp first\"");
        double deg = kStopDeg[joint - 1];       // a measured stop angle may be given instead (CAD convention)
        if (json::getNumber(line, "deg", v)) {
            if (!(v > -179.0 && v < 179.0)) return reply(out, cap, false, "\"error\": \"deg -179..179 (0 = standby)\"");
            deg = v;
        }
        return cal_.startStop(joint - 1, (float)(deg * kPi / 180.0), kJointDir[joint - 1])
                   ? reply(out, cap, true, "\"calib\": \"hold J%d still against its stop (%.2f deg)\"", joint, deg)
                   : reply(out, cap, false, "\"error\": \"busy\"");
    }
    if (!strcmp(cmd, "calib_torque") || !strcmp(cmd, "calib_spring")) {
        if (!canEnableTorque(why, sizeof(why))) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        const bool spring = !strcmp(cmd, "calib_spring");
        if (!spring && !jointOk) return reply(out, cap, false, "\"error\": \"joint 1..7\"");
        if (ctl_.mode() != Mode::Move) ctl_.setMode(Mode::Move);   // the other joints hold their pose (Hold) during the sweep
        const bool ok = spring ? cal_.startSpring(cc_) : cal_.startTorque(joint - 1, cc_);
        return ok ? reply(out, cap, true, "\"calib\": \"ramp started, support the arm\"") : reply(out, cap, false, "\"error\": \"busy or no spring configured\"");
    }
    if (!strcmp(cmd, "calib_range")) {
        if (busy(why, sizeof(why), false)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (ctl_.mode() == Mode::Move) return reply(out, cap, false, "\"error\": \"limp or gravity: the joints are moved by hand\"");
        return cal_.startRange() ? reply(out, cap, true, "\"calib\": \"range: move every joint gently to both ends, then calib_range_stop\"")
                                 : reply(out, cap, false, "\"error\": \"busy\"");
    }
    if (!strcmp(cmd, "calib_range_stop")) {
        return cal_.stopRange() ? reply(out, cap, true, "\"calib\": \"range: stopped\"") : reply(out, cap, false, "\"error\": \"no range recording\"");
    }
    if (!strcmp(cmd, "calib_abort")) { cal_.abort("aborted"); return reply(out, cap, true, ""); }
    if (!strcmp(cmd, "calib_result")) return reply(out, cap, true, "\"calib\": \"%s\", \"last\": \"%s\"", cal_.phaseText(), lastCalib_);
    if (!strcmp(cmd, "set_kt")) {
        if (!jointOk || !json::getNumber(line, "kt", v) || v < 0.1 || v > 2.0) return reply(out, cap, false, "\"error\": \"joint 1..7, kt 0.1..2 Nm/A\"");
        if (busy(why, sizeof(why), true)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        pc_.joint[joint - 1].ktNmPerA = (float)v; pc_.joint[joint - 1].flags |= CAL_KT; rebuildControlConfig();
        return reply(out, cap, true, "\"joint\": %d, \"kt\": %.4f", joint, v);
    }
    if (!strcmp(cmd, "set_ma_per_unit")) {
        if (!json::getNumber(line, "value", v) || v < 1.0 || v > 20.0) return reply(out, cap, false, "\"error\": \"value 1..20 mA\"");
        if (busy(why, sizeof(why), true)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        pc_.mAPerUnit = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"ma_per_unit\": %.3f", v);
    }
    if (!strcmp(cmd, "set_friction")) {
        if (!jointOk || !json::getNumber(line, "nm", v) || v < 0.0 || v > 0.2) return reply(out, cap, false, "\"error\": \"joint 1..7, nm 0..0.2\"");
        if (cal_.active()) return reply(out, cap, false, "\"error\": \"a calibration is running\"");
        pc_.joint[joint - 1].frictionNm = (float)v; rebuildControlConfig();
        return reply(out, cap, true, "\"joint\": %d, \"friction_nm\": %.3f", joint, v);
    }
    if (!strcmp(cmd, "save")) {
        PersistentConfig snap;
        if (!prepareSave(snap, why, sizeof(why))) return reply(out, cap, false, "\"error\": \"%s\"", why);
        const bool ok = storage_.save(snap);
        finishSave(snap, ok);
        return ok ? reply(out, cap, true, "\"saved\": true") : reply(out, cap, false, "\"error\": \"flash write failed\"");
    }
    if (!strcmp(cmd, "load")) {
        if (busy(why, sizeof(why), true)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        PersistentConfig p;
        if (!storage_.load(p)) return reply(out, cap, false, "\"error\": \"nothing stored\"");
        pc_ = p; rebuildControlConfig(); resetUnwrap();
        return reply(out, cap, true, "\"loaded\": true");
    }
    if (!strcmp(cmd, "defaults")) {
        if (busy(why, sizeof(why), true)) return reply(out, cap, false, "\"error\": \"%s\"", why);
        char name[sizeof(pc_.armName)];
        memcpy(name, pc_.armName, sizeof(name));
        defaultPersistentConfig(pc_); rebuildControlConfig(); resetUnwrap();
        memcpy(pc_.armName, name, sizeof(name));   // the name stays: it is the arm's address on the network
        return reply(out, cap, true, "\"defaults\": true, \"note\": \"not saved\"");
    }
    // ---- teleoperation and streaming
    if (!strcmp(cmd, "teleop_start")) {
        if (!canEnableTorque(why, sizeof(why))) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (cal_.active()) return reply(out, cap, false, "\"error\": \"a calibration is running\"");
        char host[40];
        if (json::getString(line, "host", host, sizeof(host))) { strncpy(pc_.piHost, host, sizeof(pc_.piHost) - 1); pc_.piHost[sizeof(pc_.piHost) - 1] = '\0'; }
        if (!pc_.piHost[0]) return reply(out, cap, false, "\"error\": \"give host (the PAR6 Pi)\"");
        if (teleop_) return reply(out, cap, false, "\"error\": \"already running\"");
        rti::LinkConfig lc = link_.config();
        memset(lc.host, 0, sizeof(lc.host));
        strncpy(lc.host, pc_.piHost, sizeof(lc.host) - 1);
        if (!link_.setConfig(lc)) return reply(out, cap, false, "\"error\": \"link busy\"");
        stopReplay("stopped");
        session_.setOffsetAlign(json::getBool(line, "offset", b) && b);
        if (ctl_.mode() != Mode::Move) ctl_.setMode(Mode::Move);   // the other joints hold their pose (Hold) during the sweep
        session_.begin(now);
        teleop_ = true;
        return reply(out, cap, true, "\"teleop\": \"connecting to %s\"", pc_.piHost);
    }
    if (!strcmp(cmd, "teleop_stop")) {
        stopReplay("stopped");
        if (teleop_) { session_.end(now); teleop_ = false; }
        return reply(out, cap, true, "\"teleop\": \"OFF\"");
    }
    if (!strcmp(cmd, "teleop_resume")) {
        if (!teleop_ || !session_.resume(now)) return reply(out, cap, false, "\"error\": \"nothing to resume (the session is not LOST)\"");
        return reply(out, cap, true, "\"teleop\": \"%s\"", TeleopSession::stageName(session_.stage()));
    }
    if (!strcmp(cmd, "teleop_realign")) {
        if (!teleop_) return reply(out, cap, false, "\"error\": \"teleoperation is off\"");
        stopReplay("stopped");
        if (!session_.realign(now)) {
            char note[120];
            jsonSafe(session_.note(), note, sizeof(note));
            return reply(out, cap, false, "\"error\": \"%s\"", note[0] ? note : "only while COUPLED or HELD");
        }
        return reply(out, cap, true, "\"teleop\": \"REALIGNING\"");
    }
    if (!strcmp(cmd, "teleop_hold")) {
        if (!json::getBool(line, "on", b)) return reply(out, cap, false, "\"error\": \"on: true|false\"");
        uiHold_ = b;
        return reply(out, cap, true, "\"hold\": %s", uiHold_ ? "true" : "false");
    }
    if (!strcmp(cmd, "teleop_offset")) {
        if (!json::getBool(line, "on", b)) return reply(out, cap, false, "\"error\": \"on: true|false\"");
        session_.setOffsetAlign(b);
        return reply(out, cap, true, "\"offset_align\": %s", b ? "true" : "false");
    }
    if (!strcmp(cmd, "teleop_rate")) {
        // PAR6's two streaming contracts: SOFT_RTI 5..50 Hz (it fills in between) or Real RTI at its 250 Hz loop
        if (!json::getNumber(line, "hz", v) || !((v >= 5.0 && v <= 50.0) || v == 0.0 || v == 250.0))
            return reply(out, cap, false, "\"error\": \"hz: 250 (Real RTI, one command per PAR6 tick) or 5..50 (SOFT_RTI)\"");
        if (teleop_) return reply(out, cap, false, "\"error\": \"stop teleoperation first: the rate is agreed when pairing\"");
        rti::LinkConfig lc = link_.config();
        lc.rateHz = v >= 250.0 || v == 0.0 ? 0.0f : (float)v;
        if (!link_.setConfig(lc)) return reply(out, cap, false, "\"error\": \"the link is not idle\"");
        return reply(out, cap, true, "\"link_rate_hz\": %.0f, \"link_mode\": \"%s\"", lc.rateHz > 0.0f ? (double)lc.rateHz : 250.0,
                     lc.rateHz > 0.0f ? "SOFT_RTI" : "RTI");
    }
    if (!strcmp(cmd, "teleop_filter")) {
        if (!json::getNumber(line, "hz", v) || v < 0.0 || v > 110.0) return reply(out, cap, false, "\"error\": \"hz: 0 (off) .. 110\"");
        session_.setFilter((float)v);
        return reply(out, cap, true, "\"teleop_filter_hz\": %.1f", (double)session_.filterHz());
    }
    if (!strcmp(cmd, "teleop_precision")) {
        if (!json::getNumber(line, "value", v) || v < 0.05 || v > 1.0) return reply(out, cap, false, "\"error\": \"value 0.05..1 (follower motion per leader motion)\"");
        session_.setPrecision((float)v);
        return reply(out, cap, true, "\"precision\": %.2f", (double)session_.precision());
    }
    if (!strcmp(cmd, "teleop_lock")) {
        double js[6];
        unsigned mask = 0;
        const int n = json::getNumberArray(line, "joints", js, 6);
        if (n >= 0) {
            for (int i = 0; i < n; ++i) {
                if (js[i] < 1.0 || js[i] > 6.0) return reply(out, cap, false, "\"error\": \"joints: arm joints 1..6\"");
                mask |= 1u << ((int)js[i] - 1);
            }
        } else if (json::getNumber(line, "mask", v) && v >= 0.0 && v < 64.0) {
            mask = (unsigned)v;
        } else {
            return reply(out, cap, false, "\"error\": \"joints: [..] or mask 0..63\"");
        }
        session_.setLocks((uint8_t)mask);
        return reply(out, cap, true, "\"locks\": %u", (unsigned)session_.locks());
    }
    if (!strcmp(cmd, "orient_lock")) {
        if (!json::getBool(line, "on", b)) return reply(out, cap, false, "\"error\": \"on: true|false\"");
        ctl_.setOrientLock(b);
        return reply(out, cap, true, "\"orient_lock\": %s", b ? "true" : "false");
    }
    if (!strcmp(cmd, "guide")) {
        char type[16] = "";
        Guide g;
        if (!json::getString(line, "type", type, sizeof(type)) || !guideFromName(type, g))
            return reply(out, cap, false, "\"error\": \"type: none | plane_h | plane_tool | axis_tool | axis_z\"");
        ctl_.setGuide(g);
        return reply(out, cap, true, "\"guide\": \"%s\"", guideName(g));
    }
    if (!strcmp(cmd, "teleop_settings")) {
        if (cal_.active()) return reply(out, cap, false, "\"error\": \"a calibration is running\"");
        TeleopSettings s = pc_.teleop;
        if (json::getBool(line, "auto_reclaim", b)) s.autoReclaim = b ? 1 : 0;
        if (json::getBool(line, "require_enable", b)) {
            if (b && !hasEnable_) return reply(out, cap, false, "\"error\": \"require_enable needs an enable button (none is wired)\"");
            s.requireEnable = b ? 1 : 0;
        }
        if (json::getBool(line, "passivity", b)) s.passivity = b ? 1 : 0;
        if (json::getBool(line, "barriers_always", b)) s.barriersAlways = b ? 1 : 0;
        if (json::getNumber(line, "force_gain", v)) { if (v < 0.0 || v > 0.2) return reply(out, cap, false, "\"error\": \"force_gain 0..0.2\""); s.forceGain = (float)v; }
        if (json::getNumber(line, "force_sign", v)) { if (v != 1.0 && v != -1.0) return reply(out, cap, false, "\"error\": \"force_sign +1/-1\""); s.forceSign = (int8_t)v; }
        if (json::getNumber(line, "force_deadband", v)) { if (v < 0.0 || v > 5.0) return reply(out, cap, false, "\"error\": \"force_deadband 0..5 Nm\""); s.forceDeadbandNm = (float)v; }
        if (json::getNumber(line, "friction_scale", v)) { if (v < 0.0 || v > 1.0) return reply(out, cap, false, "\"error\": \"friction_scale 0..1\""); s.frictionScale = (float)v; }
        double a[2];
        if (json::getNumberArray(line, "trigger_deg", a, 2) == 2) {
            if (a[0] < -90.0 || a[0] > 90.0 || a[1] < -90.0 || a[1] > 90.0 || fabs(a[1] - a[0]) < 5.0)
                return reply(out, cap, false, "\"error\": \"trigger_deg [open, closed], -90..90, at least 5 deg apart\"");
            s.triggerOpenRad = (float)a[0] * kPi / 180.0f; s.triggerClosedRad = (float)a[1] * kPi / 180.0f;
        }
        if (json::getNumberArray(line, "gripper_raw", a, 2) == 2) {
            if (a[0] < 0.0 || a[0] > 255.0 || a[1] < 0.0 || a[1] > 255.0 || fabs(a[1] - a[0]) < 10.0)
                return reply(out, cap, false, "\"error\": \"gripper_raw [open, closed], 0..255, at least 10 apart\"");
            s.gripperOpenRaw = (uint8_t)a[0]; s.gripperClosedRaw = (uint8_t)a[1];
        }
        pc_.teleop = s;
        rebuildControlConfig();
        return reply(out, cap, true, "\"settings\": \"applied\"");
    }
    // ---- replay
    if (!strcmp(cmd, "scope_start")) {
        double secs = 2.0;
        json::getNumber(line, "seconds", secs);
        const int total = (int)lround(secs * kControlHz);
        if (total < 25 || total > Scope::kMaxSamples) return reply(out, cap, false, "\"error\": \"seconds: 0.1 .. 3\"");
        free(scope_.buf);
        scope_.buf = (int16_t*)malloc((size_t)total * Scope::kWords * sizeof(int16_t));
        if (!scope_.buf) { scope_.state = Scope::Idle; return reply(out, cap, false, "\"error\": \"not enough memory\""); }
        scope_.n = 0;
        scope_.total = total;
        scope_.state = Scope::Recording;
        return reply(out, cap, true, "\"total\": %d, \"hz\": %d", total, kControlHz);
    }
    if (!strcmp(cmd, "scope_read")) {
        // samples [from, from + n) as base64 of int16 little-endian, kWords per sample; n <= 45 keeps a reply under 4 kB
        static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        const char* st = scope_.state == Scope::Recording ? "recording" : (scope_.state == Scope::Done ? "done" : "idle");
        double from = 0.0, count = 0.0;
        json::getNumber(line, "from", from);
        json::getNumber(line, "n", count);
        const int f = (int)from;
        int k = (int)count;
        if (scope_.state != Scope::Done || f < 0 || f >= scope_.n) k = 0;
        if (k > 45) k = 45;
        if (k > scope_.n - f) k = scope_.n - f;
        Out o(out, cap);
        o.add("{\"ok\": true, \"state\": \"%s\", \"have\": %d, \"total\": %d, \"hz\": %d, \"words\": %d, \"from\": %d, \"n\": %d, \"b64\": \"",
              st, scope_.n, scope_.total, kControlHz, Scope::kWords, f, k);
        const uint8_t* bytes = k > 0 ? (const uint8_t*)(scope_.buf + (size_t)f * Scope::kWords) : nullptr;
        const size_t len = (size_t)k * Scope::kWords * sizeof(int16_t);
        char quad[5] = {0, 0, 0, 0, 0};
        for (size_t i = 0; i < len; i += 3) {
            const uint32_t w = (uint32_t)bytes[i] << 16 | (i + 1 < len ? (uint32_t)bytes[i + 1] << 8 : 0) | (i + 2 < len ? bytes[i + 2] : 0);
            quad[0] = kB64[(w >> 18) & 63];
            quad[1] = kB64[(w >> 12) & 63];
            quad[2] = i + 1 < len ? kB64[(w >> 6) & 63] : '=';
            quad[3] = i + 2 < len ? kB64[w & 63] : '=';
            o.add("%s", quad);
        }
        o.add("\"}");
        return o.full ? reply(out, cap, false, "\"error\": \"reply too long\"") : o.n;
    }
    if (!strcmp(cmd, "scope_free")) {
        free(scope_.buf);
        scope_.buf = nullptr;
        scope_.n = scope_.total = 0;
        scope_.state = Scope::Idle;
        return reply(out, cap, true, "");
    }
    if (!strcmp(cmd, "replay_clear")) {
        stopReplay("cleared");
        replay_.n = 0;
        free(replay_.q);
        replay_.q = nullptr;
        replayNote_[0] = '\0';
        return reply(out, cap, true, "\"n\": 0");
    }
    if (!strcmp(cmd, "replay_add")) {
        if (replaying()) return reply(out, cap, false, "\"error\": \"a replay is running\"");
        if (!replay_.q) {
            replay_.q = (int16_t(*)[kNumJoints])malloc(sizeof(int16_t) * kNumJoints * Replay::kMax);
            if (!replay_.q) return reply(out, cap, false, "\"error\": \"no memory for a replay (42 kB)\"");
            replay_.n = 0;
        }
        if (json::getNumber(line, "dt_ms", v)) {
            if (v < 5.0 || v > 200.0) return reply(out, cap, false, "\"error\": \"dt_ms 5..200\"");
            if (replay_.n > 0 && fabsf((float)v * 1e-3f - replay_.dtS) > 1e-4f) return reply(out, cap, false, "\"error\": \"dt_ms differs from the samples already loaded (replay_clear)\"");
            replay_.dtS = (float)v * 1e-3f;
        }
        static double a[7 * 40];
        const int n = json::getNumberArray(line, "q_cdeg", a, 7 * 40);
        if (n <= 0 || n % 7) return reply(out, cap, false, "\"error\": \"q_cdeg: 7 values per sample (centidegrees), at most 40 samples per command\"");
        if (replay_.n + n / 7 > Replay::kMax) return reply(out, cap, false, "\"error\": \"too long: at most %d samples\"", Replay::kMax);
        for (int s = 0; s < n / 7; ++s)
            for (int i = 0; i < kNumJoints; ++i) {
                const double c = a[7 * s + i];
                replay_.q[replay_.n + s][i] = (int16_t)(c > 32000.0 ? 32000 : (c < -32000.0 ? -32000 : (int)lround(c)));
            }
        replay_.n += n / 7;
        return reply(out, cap, true, "\"n\": %d", replay_.n);
    }
    if (!strcmp(cmd, "replay_start")) {
        if (!canEnableTorque(why, sizeof(why))) return reply(out, cap, false, "\"error\": \"%s\"", why);
        if (replay_.n < 2) return reply(out, cap, false, "\"error\": \"nothing loaded (replay_add)\"");
        if (cal_.active()) return reply(out, cap, false, "\"error\": \"a calibration is running\"");
        if (replaying()) return reply(out, cap, false, "\"error\": \"already playing\"");
        if (teleop_ && session_.stage() != TeleopSession::Stage::Coupled) return reply(out, cap, false, "\"error\": \"teleoperation is running but not COUPLED\"");
        replay_.speed = 1.0f;
        if (json::getNumber(line, "speed", v)) { if (v < 0.1 || v > 1.0) return reply(out, cap, false, "\"error\": \"speed 0.1..1\""); replay_.speed = (float)v; }
        float q0[kNumJoints];
        for (int i = 0; i < kNumJoints; ++i) q0[i] = replay_.q[0][i] * 0.01f / kDeg;
        if (ctl_.mode() == Mode::Limp) ctl_.setMode(Mode::Gravity);
        if (!ctl_.moveTo(q0, 0.4f)) return reply(out, cap, false, "\"error\": \"no joint state yet\"");
        replay_.state = Replay::ToStart;
        replay_.t = 0.0f;
        replayNote_[0] = '\0';
        return reply(out, cap, true, "\"replay\": \"moving to the first sample\"%s", teleop_ ? ", \"note\": \"PAR6 follows\"" : "");
    }
    if (!strcmp(cmd, "replay_stop")) {
        stopReplay("stopped");
        return reply(out, cap, true, "\"replay\": \"idle\"");
    }
    if (!strcmp(cmd, "set_name")) {
        char in[64], name[kArmNameMax + 1], host[32];
        if (!json::getString(line, "name", in, sizeof(in))) return reply(out, cap, false, "\"error\": \"give name (\\\"\\\" to remove it)\"");
        if (!normalizeArmName(in, name, sizeof(name)))
            return reply(out, cap, false, "\"error\": \"a name is up to %u letters, digits and '-', not starting or ending with '-'\"", (unsigned)kArmNameMax);
        memset(pc_.armName, 0, sizeof(pc_.armName));
        memcpy(pc_.armName, name, strlen(name));
        armHostname(pc_.armName, host, sizeof(host));
        return reply(out, cap, true, "\"name\": \"%s\", \"host\": \"%s.local\", \"note\": \"save, then restart: the network name changes at the next boot\"",
                     pc_.armName, host);
    }
    if (!strcmp(cmd, "stream")) {
        char host[40];
        if (json::getString(line, "host", host, sizeof(host))) { strncpy(pc_.streamHost, host, sizeof(pc_.streamHost) - 1); }
        else if (clientIp) { strncpy(pc_.streamHost, clientIp, sizeof(pc_.streamHost) - 1); }
        if (json::getNumber(line, "port", v) && v > 0 && v < 65536) pc_.streamPort = (uint16_t)v;
        strncpy(streamHost_, pc_.streamHost, sizeof(streamHost_) - 1);
        return reply(out, cap, true, "\"stream\": \"%s:%u\"", streamHost_, (unsigned)pc_.streamPort);
    }
    if (!strcmp(cmd, "clear_fault")) {
        fault_[0] = '\0';
        memset(missing_, 0, sizeof(missing_));
        if (!modelOk_) setFault("model self-test failed");
        return reply(out, cap, !fault_[0], "\"fault\": \"%s\"", fault_);
    }
    return reply(out, cap, false, "\"error\": \"unknown cmd %s\"", cmd);
}

}  // namespace raft
