// RAFT application: everything between the servo bus and the network, in one portable class.
//   controlTick()   250 Hz, control task: raw servo readings -> calibration -> controller -> register values
//   networkTick()   network task: PAR6 link
//   handleCommand() one JSON command line from the TCP command channel -> one JSON reply line
//   leaderFrame()   the UDP leader frame of the latest tick
// The ESP32 calls these from two cores with one mutex around them (main.cpp); the PC simulator calls them
// from one thread against MuJoCo (tools/sim). Commands: docs/teleop_protocol.md.
// Joint angles are unwrapped: a joint turned past +-180 deg keeps counting (the encoder reads one turn), so the
// follower target never jumps by a turn. At power-up every joint is taken to be within half a turn of zero.
// Settings that change how torque is computed (signs, Kt, register unit, load, defaults) are accepted only
// while limp; nothing that changes the calibration runs during teleoperation, a calibration or a replay.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "calibration.h"
#include "config_store.h"
#include "control.h"
#include "leader_frame.h"
#include "rti_link.h"
#include "arm_design.h"
#include "servo_types.h"
#include "teleop.h"

namespace raft {

// Teach-and-replay: a joint trajectory uploaded by the web UI (replay_add), played on the device. With the
// teleoperation coupled, PAR6 follows the replayed leader. The 42 kB of samples come from the heap on the first
// replay_add and go back on replay_clear (static RAM on the ESP32 is too small for them).
struct Replay {
    static constexpr int kMax = 3000;          // 60 s at 50 Hz
    int16_t (*q)[kNumJoints] = nullptr;        // centidegrees, kMax rows once allocated
    int     n = 0;
    float   dtS = 0.02f;
    enum State : uint8_t { Idle, ToStart, Playing } state = Idle;
    float   t = 0.0f;
    float   speed = 1.0f;
};

// The Plots page's 250 Hz capture: every control tick, the leader's angles and speeds, the target sent to PAR6 (CAD,
// after the target filter; absent without teleoperation) and the measured currents, as int16 (1e-4 rad, 1e-3 rad/s,
// mA) plus the time. The buffer comes from the heap on scope_start (56 bytes a sample, 3 s at most = 42 kB) and goes
// back on scope_free.
struct Scope {
    static constexpr int kMaxSamples = 750;
    static constexpr int kWords = 1 + 7 + 7 + 6 + 7;   // t (0.1 ms), q[7], qd[7], sent[6] (INT16_MIN = none), mA[7]
    int16_t* buf = nullptr;
    int n = 0, total = 0;
    double t0 = 0.0;
    enum State : uint8_t { Idle, Recording, Done } state = Idle;
};

class App {
public:
    App(Storage& storage, rti::Transport& transport);
    ~App();
    App(const App&) = delete;
    App& operator=(const App&) = delete;
    // Load the stored configuration (defaults if none or invalid). modelOk = boot self-test result.
    void begin(double now, bool modelOk);

    // Control task. torqueOn: servos must have torque; needCurrentMode: put them in mode 2 before enabling.
    void controlTick(const RawServoState& raw, double now, int16_t rawOut[kNumJoints], bool& torqueOn, bool& needCurrentMode);
    void networkTick(double now);
    size_t handleCommand(const char* line, char* reply, size_t cap, double now, const char* clientIp);
    // consumer: 0 = the UDP stream, 1 = the web UI; each has its own sequence number and event bits
    static constexpr int kFrameConsumers = 2;
    size_t leaderFrame(uint8_t* buf, size_t cap, int consumer = 0);
    size_t statusLine(char* buf, size_t cap) const;            // one line for the serial console

    // The status in two steps, so the slow part runs without the App lock: snapshot() copies what the status shows
    // (microseconds, under the lock), formatStatus() / formatStatusLine() print it. The printing is ~110 decimal
    // numbers, ~3-4 ms on the ESP32; under the lock the control task waited for all of it (2026-11-03).
    struct StatusSnapshot {
        Mode mode;
        char fault[80];
        bool modelOk, calibrated;
        PersistentConfig pc;
        float q[kNumJoints];
        Frame tcp;
        uint8_t tempC[kNumJoints];
        float j2mA;
        float jointGravity[kNumJoints], jointDamping[kNumJoints];
        int8_t stJoint;
        float stGoalDeg;
        uint32_t missed[kNumJoints];
        float qMin[kNumJoints], qMax[kNumJoints];
        bool holdPosition, dirty;
        uint8_t servoModes;
        float holdStiffness;
        char streamHost[40];
        const char* calib;            // static texts
        int calibJoint;
        char lastCalib[120];
        bool lastCalibOk;
        uint32_t calibSeq;
        bool teleop;
        const char* stage;
        const char* linkPhase;
        char note[120];
        char linkError[48];
        uint32_t staleStates, staleTargets;
        float linkRateHz, filterHz;   // 0 = Real RTI
        bool followerFresh;
        float followerQ[kNumJoints];
        bool engaged, uiHold, orientLock, offsetAlign;
        float precision, offsetDeg;
        uint8_t locks;
        const char* guide;
        bool hasEnable, hasClutch, enableDown, clutchDown;
        int replayN;
        uint8_t replayState;
        float replayDtS, replayT, replaySpeed;
        char replayNote[64];
    };
    void snapshot(StatusSnapshot& s) const;
    static size_t formatStatus(const StatusSnapshot& s, char* out, size_t cap);
    static size_t formatStatusLine(const StatusSnapshot& s, char* buf, size_t cap);
    // A hash of everything in the status except the values that change every tick (angles, TCP, temperatures, J2
    // current, missed reads, follower pose, offset, dropped packets, replay time): the web server prints the status
    // for a page only when this changes. The leader frame and the web health message carry the live values.
    static uint32_t statusKey(const StatusSnapshot& s);
    const uint32_t* missedReads() const { return missedTotal_; }   // failed reads per servo since boot
    // A fault found outside the App (e.g. a servo refused constant-current mode): limp until clear_fault.
    void reportFault(const char* why) { setFault(why); }

    // Servo position test (Servos page): one servo in its own position mode (mode 0) while the arm is limp. The App
    // keeps the state; the control task performs the bus operation handed out here, once (ESP32: ServoBus).
    enum ServoTestStep : uint8_t { kServoTestNone = 0, kServoTestStart, kServoTestGoal, kServoTestStop };
    struct ServoTestOp { int8_t joint; uint8_t step; int16_t goalRaw; uint16_t speed; uint8_t acc; uint16_t torque; };
    bool takeServoTestOp(ServoTestOp& op);
    // Servo position mode for Hold, moves and replay: the goals of this tick (bit i of mask = joint i in position
    // mode), computed in controlTick. The control task switches the servos (ServoBus::setPositionMask) and reports
    // which really are in position mode (setServoModes, for the status).
    struct ServoGoals { uint8_t mask; int16_t raw[kNumJoints]; uint16_t speed; uint8_t acc; uint16_t limit; };
    const ServoGoals& servoGoals() const { return goals_; }
    void setServoModes(uint8_t positionMask) { servoModes_ = positionMask; }
    int servoTestJoint() const { return stJoint_; }   // 0-based, -1 = none

    // Physical buttons (main.cpp reads them every tick; the simulators set them too). Which ones exist decides
    // what the settings allow (require_enable needs an enable button).
    void setButtonPresence(bool enable, bool clutch) { hasEnable_ = enable; hasClutch_ = clutch; }
    void setButtons(bool enableDown, bool clutchDown) { enableDown_ = enableDown; clutchDown_ = clutchDown; }
    // Saving to flash, split so the ESP32 writes outside the App lock: prepareSave() refuses while the arm is
    // busy (teleoperation, calibration, a move or replay) and hands out the settings, finishSave() records it.
    bool prepareSave(PersistentConfig& snapshot, char* why, size_t cap) const;
    void finishSave(const PersistentConfig& snapshot, bool ok);
    // Maintenance that pauses the servo bus (servo_info): not while the arm is busy.
    bool maintenanceAllowed(char* why, size_t cap) const;

    const PersistentConfig& config() const { return pc_; }
    const char* streamHost() const { return streamHost_; }
    uint16_t streamPort() const { return pc_.streamPort; }
    Mode mode() const { return ctl_.mode(); }
    const char* fault() const { return fault_; }
    const ControlOutput& lastOutput() const { return out_; }
    const ControlInput& lastInput() const { return in_; }
    const ControlConfig& controlConfig() const { return cc_; }
    const Calibrator& calibrator() const { return cal_; }
    const TeleopSession& session() const { return session_; }
    const rti::Link& link() const { return link_; }
    bool teleopOn() const { return teleop_; }
    bool replaying() const { return replay_.state != Replay::Idle; }

private:
    void rebuildControlConfig();
    void resetUnwrap() { for (int i = 0; i < kNumJoints; ++i) { turns_[i] = 0; haveWrap_[i] = false; } }
    void setFault(const char* why);
    size_t handleCommandImpl(const char* line, char* reply, size_t cap, double now, const char* clientIp);
    bool canEnableTorque(char* why, size_t cap) const;
    bool busy(char* why, size_t cap, bool needLimp) const;      // refuses settings changes while busy
    void applyCalibration(const CalibResult& r);
    void replayTick(float dt);
    void stopReplay(const char* why);
    size_t reply(char* out, size_t cap, bool ok, const char* fmt, ...) const;
    size_t status(char* out, size_t cap) const;

    Storage& storage_;
    PersistentConfig pc_;
    ControlConfig cc_;
    float holdStiffness_ = 1.0f;   // runtime only (hold_stiffness), until the bench settles a default
    float jointGravity_[kNumJoints] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};   // runtime only (joint_gravity), likewise
    float jointDamping_[kNumJoints];   // Nm s/rad in Gravity mode (joint_damping), from kGravityDampingNms
    // servo position mode for Hold
    bool holdPosition_ = kHoldServoPosition;
    ServoGoals goals_ = {0, {0, 0, 0, 0, 0, 0, 0}, 0, 0, 0};
    uint8_t servoModes_ = 0;
    // servo position test
    int8_t stJoint_ = -1;
    int16_t stStartRaw_ = 0;
    float stGoalDeg_ = 0.0f;
    ServoTestOp stOp_ = {-1, kServoTestNone, 0, 0, 0, 0};
    int16_t lastRaw_[kNumJoints] = {0, 0, 0, 0, 0, 0, 0};
    void queueServoTest(uint8_t step, int16_t goalRaw);
    void stopServoTest();
    uint32_t missedTotal_[kNumJoints] = {0, 0, 0, 0, 0, 0, 0};   // failed reads per servo since boot (diagnostics)
    Controller ctl_;
    Calibrator cal_;
    rti::Link link_;
    TeleopSession session_;
    bool teleop_;
    bool modelOk_;
    bool torqueWasOn_;
    char fault_[80];
    char lastCalib_[120];
    bool lastCalibOk_ = false;
    uint32_t calibSeq_ = 0;     // counts calibration results, so a UI can tell a new one from the last
    bool dirty_ = false;        // settings changed since the last save or load
    char streamHost_[40];
    uint8_t missing_[kNumJoints];
    double lastTick_;
    bool haveTick_;
    // unwrapping
    int16_t turns_[kNumJoints];
    float lastWrapped_[kNumJoints];
    bool haveWrap_[kNumJoints];
    // operator inputs
    bool hasEnable_ = false, hasClutch_ = false;
    bool enableDown_ = false, clutchDown_ = false, enableWas_ = false;
    bool uiHold_ = false;
    // the Plots page's capture
    Scope scope_;
    void scopeTick(double now);
    // replay
    Replay replay_;
    char replayNote_[64];
    ControlInput in_;
    ControlOutput out_;
    int16_t currentmA_[kNumJoints];
    uint8_t voltage_[kNumJoints];
    LeaderFrame frame_;
    uint32_t seq_[kFrameConsumers];
    uint8_t events_[kFrameConsumers];
};

}  // namespace raft
