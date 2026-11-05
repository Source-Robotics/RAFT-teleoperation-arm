// RAFT torque controller. One law, four modes:
//   LIMP       tau = 0
//   GRAVITY    tau = s_g (g(q) - spring(q2)) - D qd + friction                    weightless leader
//   MOVE       tau = s_g (g(q) - spring(q2)) + K (q_ref - q) + D (qd_ref - qd)   self-driven move or replay
//   BILATERAL  tau = s_g (g(q) - spring(q2)) + clamp(Kb (q_f - q) + Db (qd_f - qd))  coupled to PAR6
//                    + contact force reflection (PAR6's t_ext) + passivity damping + friction
// plus, in GRAVITY and BILATERAL, the haptic helpers:
//   barriers       a spring pushing back within barrierMarginRad of the usable joint range
//   orientation    a rotational spring through the wrist joints that holds the handle's orientation
//   guide          a spring on the wrist centre that keeps it on a plane or a line (virtual fixture)
// then slew limit, per-joint clamp, thermal derating, and conversion to the servo's goal-current
// register (mode 2): raw = dir * tau / Kt / (mA per unit).
// Friction compensation: frictionScale * (Fc tanh(qd / v_s) + B qd), Fc from the torque calibration. Off by default.
// Passivity: a time-domain passivity observer on each arm joint's coupling torque (coupling + reflection): the
// energy the coupling has given to the leader beyond what it took is dissipated by an adaptive damper. The
// trigger is left out (in bilateral_sim, damping it weakened the blocked-gripper feel from 0.07 to 0.05 Nm).
// GRAVITY can hand over to an automatic rest move after the arm has been left alone (plan 4.4b);
// MOVE aborts to GRAVITY when someone grabs the arm.
// Plain C++11, no Arduino dependencies: the MuJoCo simulator (tools/sim) runs this exact code.
#pragma once

#include <stdint.h>

#include "kinematics.h"

namespace raft {

enum class Mode : uint8_t { Limp = 0, Gravity = 1, Move = 2, Bilateral = 3 };

struct JointControl {
    int8_t dir;              // +1 or -1: servo positive current turns the joint along model +z (calibrated in P1)
    float  ktNmPerA;         // output torque per amp; datasheet 0.441, trimmed per joint in P3
    float  tauMaxNm;         // clamp on the commanded torque (the servo's stall torque: no lower software clamp)
    float  slewNmPerS;       // limit on how fast the command may change
    float  dampingNmsPerRad; // GRAVITY mode viscous term -D * qd (0 = pure gravity compensation)
    float  qMinRad, qMaxRad; // RAFT's range (CAD convention): MOVE targets are clamped to it, the barriers use it
    float  moveK, moveD;     // MOVE impedance: Nm/rad, Nm s/rad
    float  bilatK, bilatD;   // BILATERAL coupling: Nm/rad, Nm s/rad
    float  bilatTauMaxNm;    // clamp on the coupling term alone
    float  frictionNm;       // Coulomb friction (torque calibration), used by the friction compensation
    float  viscousNmsPerRad; // viscous friction for the compensation
    float  extTauMaxNm;      // clamp on the contact force reflection
    float  assistTauMaxNm;   // clamp on barrier + orientation + guide torque
};

// Passive J2 counterbalance (plan 4.4c). spring torque about +z of J2, added to the servo's work:
// the servo then commands g(q) - springTorque(q2).
enum class SpringType : uint8_t { None = 0, ZeroLengthCable = 1, OneSidedTorsion = 2 };
struct SpringModel {
    SpringType type;
    float k;        // ZeroLengthCable: N/m.  OneSidedTorsion: Nm/rad
    float a, b;     // ZeroLengthCable: anchor height above J2 and pin distance up the upper arm, m
    float q0;       // OneSidedTorsion: q2 where the leg starts to load (rad, CAD)
    float scale;    // bench trim (P3); 1 = as specified
};
float springTorque(const SpringModel& s, float q2);

struct AutoRest {
    bool  enabled;
    float idleTimeS;          // left alone this long in GRAVITY -> move to qRest
    float stillSpeedRadS;     // "left alone": every joint slower than this ...
    float stillAngleRad;      // ... and none moved more than this since the timer started
    float speedRadS;          // speed of the rest move
    float qRest[kNumJoints];  // low-load pose (default: forearm folded down, J2/J3 at ~95 mA)
};

// Virtual fixture on the wrist centre (DH frame 4 origin: J4..J6 axes meet there, and with the scaled PAR6
// geometry the follower's wrist centre moves on the same kind of plane or line).
enum class Guide : uint8_t { None = 0, PlaneHorizontal = 1, PlaneTool = 2, AxisTool = 3, AxisVertical = 4 };
const char* guideName(Guide g);
bool guideFromName(const char* s, Guide& g);

struct ControlConfig {
    JointControl joint[kNumJoints];
    float   gravityScale;     // s_g: 1.0 = real gravity, a bench knob (0.5 sags, 1.2 drifts up)
    float   holdFriction;     // friction compensation (holding): extra push against gravity, x the gravity load; 0 = none
    float   moveStiffness;    // Hold / MOVE: K x this, D x its square root (same damping ratio); a bench knob, 1 nominal
    float   gravityJoint[kNumJoints];   // per-joint factor on top of gravityScale (wrist loads the CAD misses); 1 nominal
    float   rampTimeS;        // optional ramp-in when entering GRAVITY; 0 = on at once (default, see plan 1.2-3)
    float   mAPerUnit;        // servo current register unit: 3.25 mA (measured on the arm 2026-11-02, not the vendor example's 6.5)
    int16_t rawLimit;         // clamp on the register value (200 = 1.3 A stall at 6.5 mA/unit)
    float   derateStartC;     // all torque derates linearly from 1 at this temperature ...
    float   derateEndC;       // ... to 0 at this one
    float   moveSpeedRadS;    // default MOVE speed
    float   overrideRad;      // MOVE aborts to GRAVITY when any joint is this far from its reference
    float   followerTimeoutS; // BILATERAL drops the coupling when the follower is silent this long
    // friction compensation
    float   frictionScale;    // 0 = off, 1 = the measured friction in full
    float   frictionVelRadS;  // speed at which the Coulomb term reaches 76 % (tanh 1)
    // contact force reflection from PAR6's t_ext (measured motor torque - model torque)
    float   extGain;          // leader Nm per follower Nm; 0 = off
    float   extSign;          // -1: push back against what PAR6's motors push (t_ext's sign convention)
    float   extDeadbandNm;    // follower torques below this are model error, not contact
    float   extCutoffHz;      // first-order low-pass on the reflected torque
    float   extTimeoutS;      // no t_ext for this long -> reflection fades out
    // passivity observer / controller on the coupling
    bool    passivity;
    float   passivityMaxDampNmsPerRad;
    float   passivityMaxEnergyJ;   // the observer's credit is capped, so a long passive stretch cannot pay for a burst
    // haptic barriers at the usable range
    float   barrierMarginRad;
    float   barrierKNmPerRad;
    float   barrierDNmsPerRad;
    // orientation lock and the wrist-centre guide
    float   orientKNmPerRad;
    float   orientDNmsPerRad;
    float   guideKNPerM;
    float   guideDNsPerM;
    SpringModel spring;
    AutoRest autoRest;
};

ControlConfig defaultControlConfig();

struct ControlInput {
    float q[kNumJoints];      // rad, CAD convention
    float qd[kNumJoints];     // rad/s
    float tempC[kNumJoints];  // servo temperature
    // BILATERAL only: latest follower state, already mapped to this arm's joint space
    bool  followerFresh;      // true on the tick a new follower state arrived
    float qFollower[kNumJoints];
    float qdFollower[kNumJoints];
    bool  extValid;           // with followerFresh: tauExtFollower holds a fresh t_ext
    float tauExtFollower[kNumJoints];   // PAR6's t_ext in this arm's joint space, follower Nm
};

enum ControlEvent : uint8_t {
    EV_NONE            = 0,
    EV_MOVE_DONE       = 1 << 0,
    EV_OVERRIDDEN      = 1 << 1,   // someone grabbed the arm during a move
    EV_AUTO_REST       = 1 << 2,   // idle timeout started the rest move
    EV_FOLLOWER_LOST   = 1 << 3,
    EV_FOLLOWER_BACK   = 1 << 4,
};

struct ControlOutput {
    Frame   frames[kNumFrames];       // forward kinematics of this tick (TCP = frames[6])
    float   tauGravity[kNumJoints];   // model gravity torque, Nm
    float   tauSpring2;               // counterbalance torque on J2, Nm
    float   tauCmd[kNumJoints];       // final commanded torque, Nm
    float   tauExt[kNumJoints];       // contact force reflection in tauCmd, Nm
    float   tauAssist[kNumJoints];    // barriers + orientation lock + guide in tauCmd, Nm
    int16_t raw[kNumJoints];          // servo register values to send
    float   derate[kNumJoints];       // 1 = full torque
    float   qRef[kNumJoints];         // MOVE / BILATERAL reference (q otherwise)
    float   pcEnergyJ;                // passivity observer: smallest joint energy credit (negative = it is damping)
    uint8_t events;                   // ControlEvent bits raised this tick
};

// The HLS3606 turns toward lower encoder counts for a positive goal current, and reports its present current with
// the sign of the goal (measured on the arm 2026-11-02: +30 units -> -205 ticks, -30 units -> +147 ticks). With a
// joint's direction dir (ticks vs the model's +), register = kCurrentSign * dir * current in the model's sense.
constexpr int kCurrentSign = -1;

// Datasheet stall current at 6 V. The goal-current register is clamped to this (rawLimit = kStallCurrentMa / mAPerUnit).
constexpr float kStallCurrentMa = 1300.0f;

int16_t torqueToRaw(float tauNm, const JointControl& j, float mAPerUnit, int16_t rawLimit);

class Controller {
public:
    explicit Controller(const ControlConfig& cfg);
    void setMode(Mode m);                       // entering GRAVITY restarts the optional ramp
    Mode mode() const { return mode_; }
    // MOVE on its way to a target (or following a replay); a plain Hold of the current pose is not moving
    bool moving() const { return mode_ == Mode::Move && (tracking_ || moveT_ < moveDur_); }
    // Start a smooth (minimum-jerk) joint move from the last measured q. speed <= 0 uses the default.
    // Targets are clamped to the soft limits. Returns false before the first step() (no q yet).
    bool moveTo(const float qTarget[kNumJoints], float speedRadS = 0.0f);
    bool moveActive() const { return mode_ == Mode::Move && !tracking_ && moveT_ < moveDur_; }
    // Replay: MOVE follows this reference directly (call every tick; the grab abort still applies).
    bool track(const float q[kNumJoints], const float qd[kNumJoints]);
    bool tracking() const { return mode_ == Mode::Move && tracking_; }
    void setGravityScale(float s) { cfg_.gravityScale = s; }
    const ControlConfig& config() const { return cfg_; }
    ControlConfig& mutableConfig() { return cfg_; }
    // Haptic barriers: the usable range of a joint (set every tick by the owner; cleared = none)
    void setBarrier(int joint, float lo, float hi) { barLo_[joint] = lo; barHi_[joint] = hi; barOn_[joint] = true; }
    void clearBarriers() { for (int i = 0; i < kNumJoints; ++i) barOn_[i] = false; }
    // Orientation lock / guide: anchored at the pose of the next step()
    void setOrientLock(bool on) { orientOn_ = on; orientPending_ = on; }
    bool orientLock() const { return orientOn_; }
    void setGuide(Guide g) { guide_ = g; guidePending_ = g != Guide::None; }
    Guide guide() const { return guide_; }
    // One control tick, dt in seconds.
    void step(const ControlInput& in, float dt, ControlOutput& out);
    // Calibration: drive one joint's goal-current register directly (the law is bypassed for that joint,
    // derating still applies). The slew state follows, so handing the joint back is smooth.
    void setRawOverride(int joint, int16_t raw) { ovJoint_ = (int8_t)joint; ovRaw_ = raw; }
    void clearRawOverride() { ovJoint_ = -1; }

private:
    void moveReference(float* qRef, float* qdRef) const;
    void assistTorques(const ControlInput& in, const Frame* T, float* tau);

    ControlConfig cfg_;
    Mode  mode_;
    float ramp_;
    float prevTau_[kNumJoints];
    bool  haveQ_;
    float lastQ_[kNumJoints];
    // move
    float moveStart_[kNumJoints], moveGoal_[kNumJoints];
    float moveT_, moveDur_;
    bool  moveIsRest_;
    bool  tracking_;
    float trackQ_[kNumJoints], trackQd_[kNumJoints];
    // auto rest
    float idleT_;
    float idleAnchor_[kNumJoints];
    // bilateral
    float sinceFollower_;
    bool  followerOk_;
    float qF_[kNumJoints], qdF_[kNumJoints];
    float extIn_[kNumJoints], extF_[kNumJoints];
    float sinceExt_;
    float energy_[kNumJoints];
    // haptic helpers
    bool  barOn_[kNumJoints];
    float barLo_[kNumJoints], barHi_[kNumJoints];
    bool  orientOn_, orientPending_;
    float orientR_[9];
    Guide guide_;
    bool  guidePending_;
    float guideP0_[3], guideDir_[3];
    // calibration override
    int8_t  ovJoint_ = -1;
    int16_t ovRaw_ = 0;
};

}  // namespace raft
