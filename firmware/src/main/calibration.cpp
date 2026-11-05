#include "calibration.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace raft {
namespace {

const float kPi = 3.14159265358979f;
const float kRunaway = 8.0f * kPi / 180.0f;      // abort if the joint gets this far from where it started
const float kTrackErr = 6.0f * kPi / 180.0f;     // abort if the joint lags the sweep by this much (3 deg was tripped by the real gear friction)
const float kMinLoad = 0.04f;                    // Nm: too little gravity load to calibrate against
const float kIntegMax = 0.25f;                   // Nm: PID integrator clamp
const int16_t kRawMax = 400;                     // register clamp during the sweep

}  // namespace

constexpr float Calibrator::kSweepAmpRad;
constexpr float Calibrator::kSweepSpeedRadS;
constexpr float Calibrator::kHoldS;
constexpr float Calibrator::kSkipS;

Calibrator::Calibrator()
    : kind_(CalibKind::None), joint_(-1), t_(0.0f), n_(0), stopRad_(0.0f), stopDir_(1), phase_(P_IDLE), phaseT_(0.0f), qStart_(0.0f),
      qMinSeen_(0.0f), qMaxSeen_(0.0f), integ_(0.0f), kp_(0.0f), ki_(0.0f), kd_(0.0f), tauNeed0_(0.0f), tauSpring0_(0.0f), sumCur_(0.0f), sumReg_(0.0f),
      mAPerUnit_(3.25f), ktAssumed_(0.441f), dirJ_(1), haveResult_(false)
{
    memset(&result_, 0, sizeof(result_));
    for (int i = 0; i < kNumJoints; ++i) { sum_[i] = 0; ref_[i] = 0; minD_[i] = maxD_[i] = 0; rMin_[i] = rMax_[i] = 0.0f; }
    for (int b = 0; b < 2; ++b) { sumCmd_[b] = sumGrav_[b] = sumSpring_[b] = 0.0f; nAvg_[b] = 0; }
}

const char* Calibrator::phaseText() const
{
    if (kind_ == CalibKind::None) return "idle";
    if (kind_ == CalibKind::Zero) return "zero: hold the arm in the CAD zero pose";
    if (kind_ == CalibKind::Stop) return "stop: hold the joint still against its stop";
    if (kind_ == CalibKind::Range) return "range: move every joint gently to both ends, then stop";
    switch (phase_) {
    case P_HOLD: return "sweep: holding still";
    case P_UP: return "sweep: moving + 4 deg";
    case P_DOWN: return "sweep: moving - 8 deg";
    case P_BACK: return "sweep: returning";
    default: return "sweep";
    }
}

void Calibrator::finish(bool ok, const char* msg)
{
    result_.kind = kind_;
    result_.joint = joint_;
    result_.ok = ok;
    strncpy(result_.msg, msg, sizeof(result_.msg) - 1);
    result_.msg[sizeof(result_.msg) - 1] = '\0';
    haveResult_ = true;
    kind_ = CalibKind::None;
    phase_ = P_IDLE;
}

void Calibrator::abort(const char* why)
{
    if (kind_ == CalibKind::None) return;
    finish(false, why);
}

bool Calibrator::startZero()
{
    if (active()) return false;
    memset(&result_, 0, sizeof(result_));
    haveResult_ = false;
    kind_ = CalibKind::Zero; joint_ = -1; t_ = 0.0f; n_ = 0;
    for (int i = 0; i < kNumJoints; ++i) { sum_[i] = 0; minD_[i] = 32767; maxD_[i] = -32767; }
    return true;
}

bool Calibrator::startStop(int joint, float stopRad, int dir)
{
    if (active() || joint < 0 || joint >= kNumJoints || (dir != 1 && dir != -1)) return false;
    if (!(stopRad > -kPi && stopRad < kPi)) return false;   // a reading is half a turn either way of the zero
    memset(&result_, 0, sizeof(result_));
    haveResult_ = false;
    kind_ = CalibKind::Stop; joint_ = (int8_t)joint; t_ = 0.0f; n_ = 0;
    stopRad_ = stopRad; stopDir_ = (int8_t)dir;
    sum_[joint] = 0; minD_[joint] = 32767; maxD_[joint] = -32767;
    return true;
}

void Calibrator::startSweep(int joint, const ControlConfig& cfg, CalibKind kind)
{
    memset(&result_, 0, sizeof(result_));
    haveResult_ = false;
    kind_ = kind; joint_ = (int8_t)joint; t_ = 0.0f; n_ = 0;
    phase_ = P_HOLD;
    phaseT_ = 0.0f;
    integ_ = 0.0f;
    // the MOVE impedance of this joint (validated in the simulator) plus an integrator with a 0.25 s time constant
    kp_ = cfg.joint[joint].moveK;
    kd_ = cfg.joint[joint].moveD;
    ki_ = 4.0f * kp_;
    for (int b = 0; b < 2; ++b) { sumCmd_[b] = sumGrav_[b] = sumSpring_[b] = 0.0f; nAvg_[b] = 0; }
    sumCur_ = sumReg_ = 0.0f;
    mAPerUnit_ = cfg.mAPerUnit;
    ktAssumed_ = cfg.joint[joint].ktNmPerA;
    dirJ_ = cfg.joint[joint].dir;
}

bool Calibrator::startTorque(int joint, const ControlConfig& cfg)
{
    if (active() || joint < 0 || joint >= kNumJoints) return false;
    startSweep(joint, cfg, CalibKind::Torque);
    return true;
}

bool Calibrator::startSpring(const ControlConfig& cfg)
{
    if (active() || cfg.spring.type == SpringType::None) return false;
    startSweep(1, cfg, CalibKind::Spring);
    return true;
}

bool Calibrator::startRange()
{
    if (active()) return false;
    memset(&result_, 0, sizeof(result_));
    haveResult_ = false;
    kind_ = CalibKind::Range; joint_ = -1; t_ = 0.0f; n_ = 0;
    return true;
}

bool Calibrator::stopRange()
{
    if (kind_ != CalibKind::Range) return false;
    finishRange();
    return true;
}

void Calibrator::finishRange()
{
    const float minSpan = 10.0f * kPi / 180.0f;
    result_.rangeMask = 0;
    char msg[120];
    int m = snprintf(msg, sizeof(msg), "range: measured");
    for (int i = 0; i < kNumJoints; ++i) {
        result_.qMinRad[i] = rMin_[i];
        result_.qMaxRad[i] = rMax_[i];
        if (n_ > 0 && rMax_[i] - rMin_[i] >= minSpan) {
            result_.rangeMask |= (uint8_t)(1u << i);
            if (m > 0 && m < (int)sizeof(msg))
                m += snprintf(msg + m, sizeof(msg) - (size_t)m, " J%d %.0f..%.0f", i + 1, (double)(rMin_[i] * 180.0f / kPi), (double)(rMax_[i] * 180.0f / kPi));
        }
    }
    if (!result_.rangeMask) { finish(false, "range: no joint was moved through 10 deg"); return; }
    finish(true, msg);
}

void Calibrator::tick(const RawServoState& raw, const ControlInput& in, const ControlOutput& out, float dt, CalibOverride& ov)
{
    ov.active = false;
    if (!active()) return;
    t_ += dt;
    char msg[120];

    if (kind_ == CalibKind::Zero) {
        for (int i = 0; i < kNumJoints; ++i) {
            if (!raw.ok[i]) { finish(false, "zero: a servo did not answer"); return; }
            if (n_ == 0) ref_[i] = raw.pos[i];
            const int16_t d = (int16_t)ticksFromZero(raw.pos[i], ref_[i]);
            sum_[i] += d;
            if (d < minD_[i]) minD_[i] = d;
            if (d > maxD_[i]) maxD_[i] = d;
        }
        ++n_;
        if (t_ < 0.5f) return;
        for (int i = 0; i < kNumJoints; ++i) {
            if (maxD_[i] - minD_[i] > 10) {
                snprintf(msg, sizeof(msg), "zero: joint %d moved by %d ticks while sampling, hold the arm still", i + 1, maxD_[i] - minD_[i]);
                finish(false, msg); return;
            }
            const int avg = (int)lroundf((float)sum_[i] / (float)n_);
            int z = (ref_[i] + avg) % 4096; if (z < 0) z += 4096;
            result_.zeroTicks[i] = (int16_t)z;
        }
        finish(true, "zero: stored the CAD zero pose for all joints");
        return;
    }

    if (kind_ == CalibKind::Range) {
        for (int i = 0; i < kNumJoints; ++i) {
            if (n_ == 0) { rMin_[i] = rMax_[i] = in.q[i]; continue; }
            if (in.q[i] < rMin_[i]) rMin_[i] = in.q[i];
            if (in.q[i] > rMax_[i]) rMax_[i] = in.q[i];
        }
        ++n_;
        if (t_ > 180.0f) finishRange();
        return;
    }

    if (kind_ == CalibKind::Stop) {
        const int j = joint_;
        if (!raw.ok[j]) { finish(false, "stop: servo did not answer"); return; }
        if (n_ == 0) ref_[j] = raw.pos[j];     // average the reading at the stop
        const int16_t d = (int16_t)ticksFromZero(raw.pos[j], ref_[j]);
        sum_[j] += d;
        if (d < minD_[j]) minD_[j] = d;
        if (d > maxD_[j]) maxD_[j] = d;
        ++n_;
        if (t_ < 0.3f) return;
        if (maxD_[j] - minD_[j] > 10) {
            snprintf(msg, sizeof(msg), "stop: J%d moved by %d ticks while sampling, hold it still against its stop", j + 1, maxD_[j] - minD_[j]);
            finish(false, msg); return;
        }
        int s = (ref_[j] + (int)lroundf((float)sum_[j] / (float)n_)) % 4096; if (s < 0) s += 4096;
        // the reading at the stop must give the stop angle: dir * (stopTicks - zero) * kRadPerTick = stopRad
        int z = (s - stopDir_ * (int)lroundf(stopRad_ / kRadPerTick)) % 4096; if (z < 0) z += 4096;
        result_.zeroTicks[j] = (int16_t)z;
        snprintf(msg, sizeof(msg), "stop: J%d zero %d ticks (stop at %.2f deg)", j + 1, z, (double)(stopRad_ * 180.0f / kPi));
        finish(true, msg);
        return;
    }

    // ---- sweep (Torque / Spring) on joint_
    const int j = joint_;
    if (!raw.ok[j]) { finish(false, "sweep: servo did not answer"); return; }
    const float q = in.q[j];
    const float grav = out.tauGravity[j];
    const float spring = (j == 1) ? out.tauSpring2 : 0.0f;    // what the firmware assumes, incl. spring.scale
    if (n_++ == 0) {
        tauNeed0_ = grav - spring;
        tauSpring0_ = spring;
        if (kind_ == CalibKind::Spring && fabsf(tauSpring0_) < 0.04f) {
            finish(false, "spring: tilt the upper arm at least 40 deg so the spring carries load"); return;
        }
        if (kind_ == CalibKind::Torque && fabsf(tauNeed0_) < kMinLoad) {
            snprintf(msg, sizeof(msg), "torque: joint %d carries only %.3f Nm here; pose it with its link near horizontal", j + 1, (double)tauNeed0_);
            finish(false, msg); return;
        }
        qStart_ = q;
        qMinSeen_ = qMaxSeen_ = q;
    }
    if (q < qMinSeen_) qMinSeen_ = q;
    if (q > qMaxSeen_) qMaxSeen_ = q;

    // reference: hold, +A, down to -A, back to the start
    const float A = kSweepAmpRad, v = kSweepSpeedRadS;
    phaseT_ += dt;
    float qRef = qStart_;
    int bucket = -1;                       // which average this tick feeds
    switch (phase_) {
    case P_HOLD:
        if (phaseT_ >= kHoldS) { phase_ = P_UP; phaseT_ = 0.0f; }
        break;
    case P_UP:
        qRef = qStart_ + v * phaseT_;
        if (phaseT_ > kSkipS) bucket = 0;
        if (phaseT_ >= A / v) { phase_ = P_DOWN; phaseT_ = 0.0f; }
        break;
    case P_DOWN:
        qRef = qStart_ + A - v * phaseT_;
        if (phaseT_ > kSkipS) bucket = 1;
        if (phaseT_ >= 2.0f * A / v) { phase_ = P_BACK; phaseT_ = 0.0f; }
        break;
    case P_BACK:
        qRef = qStart_ - A + v * phaseT_;
        if (phaseT_ > kSkipS) bucket = 0;
        if (phaseT_ >= A / v) phase_ = P_DONE;
        break;
    default:
        break;
    }
    const float e = qRef - q;
    if (fabsf(q - qStart_) > kRunaway || fabsf(e) > kTrackErr) {
        result_.limp = true;
        finish(false, "sweep: the joint did not follow (wrong sign or zero? blocked?), stopped and let go");
        return;
    }

    if (phase_ == P_DONE) {
        if (nAvg_[0] < 50 || nAvg_[1] < 50) { finish(false, "sweep: too few samples"); return; }
        if (qMaxSeen_ - qMinSeen_ < 1.5f * kSweepAmpRad) {   // a joint held by friction or a block gives no torque constant
            snprintf(msg, sizeof(msg), "sweep: joint %d moved only %.1f of %.0f deg (gear friction or blocked): no result", j + 1,
                     (double)((qMaxSeen_ - qMinSeen_) * 180.0f / kPi), (double)(2.0f * kSweepAmpRad * 180.0f / kPi));
            finish(false, msg); return;
        }
        const float U = sumCmd_[0] / nAvg_[0], D = sumCmd_[1] / nAvg_[1];          // commanded, assumed-Kt Nm
        const float Gu = sumGrav_[0] / nAvg_[0], Gd = sumGrav_[1] / nAvg_[1];
        const float Su = sumSpring_[0] / nAvg_[0], Sd = sumSpring_[1] / nAvg_[1];
        // real torque = kappa * commanded; moving +q it equals load + friction, moving -q load - friction
        if (kind_ == CalibKind::Torque) {
            const float loadU = Gu - Su, loadD = Gd - Sd;
            if (fabsf(U + D) < 1e-4f) { finish(false, "torque: no net command, check the sign"); return; }
            const float kappa = (loadU + loadD) / (U + D);
            if (kappa < 0.4f || kappa > 2.5f) {
                snprintf(msg, sizeof(msg), "torque: implausible result (real/assumed torque %.2f), check sign and mA per unit", (double)kappa);
                finish(false, msg); return;
            }
            result_.ktNmPerA = kappa * ktAssumed_;
            result_.frictionNm = fmaxf(0.0f, 0.5f * ((kappa * U - loadU) + (loadD - kappa * D)));
            result_.currentRegPerUnit = sumReg_ > 0.0f ? sumCur_ / sumReg_ : 0.0f;
            snprintf(msg, sizeof(msg), "torque: joint %d Kt %.3f Nm/A (assumed %.3f), friction %.3f Nm at %.3f Nm load",
                     j + 1, (double)result_.ktNmPerA, (double)ktAssumed_, (double)result_.frictionNm, (double)fabsf(0.5f * (loadU + loadD)));
            finish(true, msg);
        } else {
            if (fabsf(Su + Sd) < 0.08f) { finish(false, "spring: the spring carried too little load during the sweep"); return; }
            const float s = ((Gu + Gd) - (U + D)) / (Su + Sd);
            if (s < 0.3f || s > 3.0f) {
                snprintf(msg, sizeof(msg), "spring: implausible result (%.2f x the configured spring), check k, a, b", (double)s);
                finish(false, msg); return;
            }
            result_.springScale = s;
            snprintf(msg, sizeof(msg), "spring: gives %.3f Nm where the model says %.3f Nm -> multiply spring.scale by %.3f",
                     (double)(0.5f * s * (Su + Sd)), (double)(0.5f * (Su + Sd)), (double)s);
            finish(true, msg);
        }
        return;
    }

    // model load feedforward + PID on position, in assumed-Kt Nm
    integ_ += ki_ * e * dt;
    if (integ_ > kIntegMax) integ_ = kIntegMax;
    if (integ_ < -kIntegMax) integ_ = -kIntegMax;
    float tau = (grav - spring) + kp_ * e + integ_ - kd_ * in.qd[j];
    const float nmPerUnit = ktAssumed_ * mAPerUnit_ * 1e-3f;
    float units = tau / nmPerUnit;
    if (units > kRawMax) units = kRawMax;
    if (units < -kRawMax) units = -kRawMax;
    tau = units * nmPerUnit;
    if (bucket >= 0) {
        sumCmd_[bucket] += tau; sumGrav_[bucket] += grav; sumSpring_[bucket] += spring; ++nAvg_[bucket];
        sumCur_ += fabsf((float)raw.current[j]);
        sumReg_ += fabsf(units);
    }
    ov.active = true;
    ov.joint = (int8_t)j;
    ov.raw = (int16_t)lroundf((float)(kCurrentSign * dirJ_) * units);
}

}  // namespace raft
