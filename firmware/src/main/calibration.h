// Bench calibration procedures (docs/calibration.md), run inside the 250 Hz control loop. Portable: the MuJoCo
// simulator (tools/sim/calib_sim.cpp) runs them against a servo bus with deliberately wrong zero, sign,
// torque constant and spring.
// The joint directions are design constants (kJointDir, arm_design.h), not calibrated.
//   Stop       one joint held against its mechanical stop, whose angle is known (kStopDeg): average its raw ticks
//              for 0.3 s; with the joint's direction the stop angle gives its zero. No torque.
//   Zero       arm held in the CAD zero pose: average every joint's raw ticks for 0.5 s -> zeroTicks
//   Torque     at a pose where the joint carries a clear gravity load (>= 0.04 Nm): a slow closed-loop
//              sweep of that joint, +-4 deg at 3 deg/s, driving its current directly (model feedforward +
//              PID on position). Moving +q the servo supplies load + friction, moving -q load - friction:
//              the two averages against the model load give Kt and friction. No breakaway detection, so
//              the joint never jumps.
//   Spring     the same sweep on J2 with the upper arm tilted >= 40 deg: with J2's Kt known, the servo's
//              share tells how much the spring really gives -> spring.scale. Run the J2 torque calibration
//              first (arm vertical, where the spring gives nothing).
//   Range      limp or weightless: the person moves every joint gently to both of its ends; stopRange() (or
//              3 minutes) ends it. Joints moved through at least 10 deg get their measured range.
#pragma once

#include <stdint.h>

#include "arm_design.h"
#include "control.h"
#include "servo_types.h"

namespace raft {

enum class CalibKind : uint8_t { None = 0, Zero, Torque, Spring, Range, Stop };

struct CalibResult {
    CalibKind kind;
    int8_t    joint;          // -1 for all joints (zero)
    bool      ok;
    bool      limp;                    // the joint ran away (wrong sign?): the App drops the arm to LIMP
    char      msg[120];
    int16_t   zeroTicks[kNumJoints];   // Zero (Stop: the joint's entry)
    float     ktNmPerA;                // Torque
    float     frictionNm;              // Torque
    float     currentRegPerUnit;       // Torque: present-current register per commanded unit
    float     springScale;             // Spring: factor to multiply the configured spring.scale by
    uint8_t   rangeMask;               // Range: joints whose range was measured (bit i = joint i+1)
    float     qMinRad[kNumJoints];     // Range: the measured ends (CAD convention)
    float     qMaxRad[kNumJoints];
};

// A request to drive one joint's goal-current register directly this tick (the controller applies it).
struct CalibOverride { bool active; int8_t joint; int16_t raw; };

class Calibrator {
public:
    Calibrator();
    bool active() const { return kind_ != CalibKind::None; }
    CalibKind kind() const { return kind_; }
    int joint() const { return active() ? joint_ : -1; }   // joint being calibrated (0-based), -1 none or all
    // false if a procedure is already running or the arguments are invalid
    bool startZero();
    bool startStop(int joint, float stopRad, int dir);   // dir: the joint's kJointDir
    bool startTorque(int joint, const ControlConfig& cfg);
    bool startSpring(const ControlConfig& cfg);
    bool startRange();
    bool stopRange();                  // ends a range recording with what it has
    void abort(const char* why);
    // Every control tick, after Controller::step (uses its gravity torques and the current q).
    void tick(const RawServoState& raw, const ControlInput& in, const ControlOutput& out, float dt, CalibOverride& ov);
    // The result of the procedure that just ended, handed out once.
    bool takeResult(CalibResult& r) { if (!haveResult_) return false; r = result_; haveResult_ = false; return true; }
    // Progress text for status queries
    const char* phaseText() const;

    // sweep shape
    static constexpr float kSweepAmpRad = 4.0f * 3.14159265f / 180.0f;
    static constexpr float kSweepSpeedRadS = 3.0f * 3.14159265f / 180.0f;
    static constexpr float kHoldS = 0.5f;          // hold at the start before moving
    static constexpr float kSkipS = 0.5f;          // after each reversal, before averaging

private:
    enum Phase : uint8_t { P_IDLE, P_HOLD, P_UP, P_DOWN, P_BACK, P_DONE };
    void finish(bool ok, const char* msg);
    void startSweep(int joint, const ControlConfig& cfg, CalibKind kind);

    CalibKind kind_;
    int8_t joint_;
    float t_;
    // zero
    int32_t sum_[kNumJoints];
    int16_t ref_[kNumJoints];
    int16_t minD_[kNumJoints], maxD_[kNumJoints];
    int32_t n_;
    // stop: the stop angle and the joint's direction
    float stopRad_;
    int8_t stopDir_;
    // range
    float rMin_[kNumJoints], rMax_[kNumJoints];
    void finishRange();
    // sweep
    Phase phase_;
    float phaseT_;
    float qStart_;
    float qMinSeen_, qMaxSeen_;        // the range the joint really covered during the sweep
    float integ_;                      // PID integrator, Nm
    float kp_, ki_, kd_;               // Nm/rad, Nm/(rad s), Nm s/rad
    float tauNeed0_, tauSpring0_;      // model load and assumed spring torque at the start
    // averages over the steady parts: [0] moving +q, [1] moving -q
    float sumCmd_[2], sumGrav_[2], sumSpring_[2];
    int32_t nAvg_[2];
    float sumCur_, sumReg_;
    float mAPerUnit_, ktAssumed_;
    int8_t dirJ_;
    bool haveResult_;
    CalibResult result_;
};

}  // namespace raft
