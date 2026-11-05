// Bilateral teleoperation session: glues the RTI link to the controller.
//   link PAIRED   -> check that RAFT can reach the follower's pose, then self-move onto it (the RCB runtime
//                    checks the start pose); an unreachable joint is reported and nothing moves
//   aligned       -> claim control
//   link ACTIVE   -> COUPLED: controller BILATERAL, follower state in, follower targets out at the link rate
//   not engaged   -> HELD: the follower holds its last target, the leader is free (GRAVITY). Engaging again
//                    re-anchors the mapping where the leader is now (an indexing clutch).
//   realign       -> REALIGNING: the follower holds, the leader self-moves back onto it (removes offsets)
//   link lost     -> controller GRAVITY at once, then LOST until the operator resumes (or, with autoReclaim,
//                    straight back to realigning and reclaiming as soon as the link is PAIRED again)
//
// Joint mapping, per arm joint, in RAFT's CAD convention:  q_follower = aF + s (q_leader - aL)
//   identity: s = 1 and aF - aL = the 2 pi branch that puts the follower's angle inside RAFT's range (PAR6's J6
//   runs to +-229 deg, RAFT reads its J6 within one turn); precision mode: s < 1; a locked joint: s = 0
//   (the follower joint stays put and the coupling holds the leader joint where it was locked); an offset
//   (after the clutch, or when coupling to a pose outside RAFT's range): aF - aL differs from the branch.
// The coupling feeds the follower back through the inverse map, t_ext through s (virtual work), and the
// usable range of every joint (RAFT's own range intersected with PAR6's mapped soft limits) goes to the
// controller's haptic barriers. Follower targets are clamped to PAR6's soft limits.
// Trigger <-> gripper: the trigger angle maps linearly to the gripper's raw position command (0..255); the
// gripper's measured position g_p is on the same raw scale (firmware mode, 0 open, 255 closed) and maps back
// to a trigger angle, so a blocked gripper is felt on the trigger through the joint-7 coupling.
// Plain C++11; the ESP32 calls beforeControl/afterControl from the control task and Link::poll from
// the network task (with a lock around the shared state, P4).
#pragma once

#include "control.h"
#include "rti_link.h"

namespace raft {

struct TeleopConfig {
    float alignTolRad;        // claim when every joint is this close to its alignment target
    float alignSpeedRadS;     // self-move speed onto the follower's pose
    float realignDelayS;      // after someone grabs the arm during alignment, retry this much later
    float triggerOpenRad;     // leader trigger angle for an open gripper ...
    float triggerClosedRad;   // ... and for a closed one
    int   gripperOpenRaw;     // gripper position (0..255) for open: the gripper_set command AND the g_p reading ...
    int   gripperClosedRaw;   // ... and closed (the gripper's firmware mode uses one raw scale for both)
    int   gripperSpeed;       // 0..255
    int   gripperCurrentmA;
    float followerMinRad[6];  // PAR6 soft joint limits in RAFT's CAD convention (robots/PAR6.xml)
    float followerMaxRad[6];
    float limitMarginRad;     // an alignment target clamped into RAFT's range stays this far inside it
    bool  autoReclaim;        // after an outage, realign and reclaim without waiting for resume()
    float maxTargetStepRad;   // a follower target jumping more than this in one control tick -> HELD
};
TeleopConfig defaultTeleopConfig();

class TeleopSession {
public:
    enum class Stage : uint8_t { Off, WaitPaired, Aligning, Claiming, Coupled, Held, Realigning, Lost };

    TeleopSession(Controller& ctl, rti::Link& link, const TeleopConfig& cfg);
    void begin(double now);    // start the link; the controller keeps its mode until coupling
    void end(double now);      // stop the link, controller -> GRAVITY
    // Control tick: call before Controller::step (fills the follower fields of `in`) ...
    void beforeControl(ControlInput& in, double now);
    // ... and after it (streams the follower target and trigger).
    void afterControl(const ControlInput& in, double now);

    // Operator inputs
    void setEngaged(bool on) { engaged_ = on; }      // every tick: enable held and clutch not pressed
    bool resume(double now);                         // LOST -> realign and reclaim
    bool realign(double now);                        // COUPLED / HELD -> REALIGNING (removes offsets)
    void setPrecision(float s);                      // 0.05..1, joints scale around where they are now
    void setLocks(uint8_t mask);                     // bit i: arm joint i+1 does not move the follower
    void setOffsetAlign(bool on) { offsetAlign_ = on; }   // couple with an offset when PAR6 is outside RAFT's range
    // Low-pass on the targets sent to PAR6: 2nd-order Butterworth at the control rate, on position and velocity alike;
    // 0 = off (the default). It lags about 0.23 / hz s at low frequencies (25 Hz: 9 ms, 50 Hz: 4.5 ms). It smooths the
    // encoder steps and the hand's tremor, and is the anti-alias filter in front of a SOFT_RTI rate below 250 Hz.
    void setFilter(float hz);
    float filterHz() const { return filterHz_; }
    TeleopConfig& config() { return cfg_; }
    const TeleopConfig& config() const { return cfg_; }

    Stage stage() const { return stage_; }
    static const char* stageName(Stage s);
    bool claimed() const { return stage_ == Stage::Coupled || stage_ == Stage::Held || stage_ == Stage::Realigning; }
    const char* note() const { return note_; }                       // why the session waits, "" otherwise
    const float* followerQ() const { return qF_; }                   // follower pose, CAD (valid once the link has state)
    const float* followerTarget() const { return cmdF_; }            // last follower target, CAD, before the filter
    const float* sentTarget() const { return sent_; }                // ... and as sent, after the filter
    bool streaming() const { return stage_ != Stage::Off && haveCmd_; }
    float precision() const { return precision_; }
    uint8_t locks() const { return locks_; }
    bool engaged() const { return engaged_; }
    bool offsetAlign() const { return offsetAlign_; }
    float offsetMaxRad() const;                                      // largest |target - (leader + branch)|
    void usableRange(int joint, float& lo, float& hi) const;         // leader space, for the barriers

private:
    bool followerPose(float q[kNumJoints], float qd[kNumJoints], float tauExt[kNumJoints], bool& extOk) const;
    float mapToFollower(int i, float qL) const { return aF_[i] + s_[i] * (qL - aL_[i]); }
    float mapToLeader(int i, float qF) const { return s_[i] > 0.0f ? aL_[i] + (qF - aF_[i]) / s_[i] : aL_[i]; }
    // alignment targets for the identity mapping (plus offsets for unreachable joints when allowed);
    // returns the mask of joints RAFT cannot reach and writes the first one into note_
    uint8_t planAlignment(const float qf[kNumJoints], const float qNow[kNumJoints], float target[kNumJoints], float branch[6]);
    void reanchor(const float qL[kNumJoints], const float cmd[6]);
    void lose(double now, const char* why);
    void setNote(const char* fmt, ...);

    Controller& ctl_;
    rti::Link& link_;
    TeleopConfig cfg_;
    Stage stage_;
    double stageT_;
    uint32_t lastStateCount_;
    float qF_[kNumJoints];
    float aL_[6], aF_[6], s_[6], branch_[6];
    float target_[kNumJoints];
    float hold_[6], cmdF_[6], sent_[6];
    bool  haveCmd_;
    float qL_[kNumJoints];
    float precision_;
    uint8_t locks_;
    bool  engaged_;
    bool  needRelease_;        // HELD by the jump guard: engaging again needs a release first (or realign)
    bool  offsetAlign_;
    char  note_[120];
    // target filter
    struct Lp2 { float x1, x2, y1, y2; };
    float filterHz_ = 0.0f;
    float fb0_ = 1.0f, fb1_ = 0.0f, fb2_ = 0.0f, fa1_ = 0.0f, fa2_ = 0.0f;
    Lp2 lpQ_[6], lpV_[6];
    bool filtInit_ = false;
    float lp(Lp2& s, float x) const;
};

}  // namespace raft
