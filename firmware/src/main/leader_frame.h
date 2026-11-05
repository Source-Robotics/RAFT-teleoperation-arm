// RAFT leader frame: one UDP datagram per control tick with the arm's state (docs/teleop_protocol.md).
// Little-endian, fixed 288 bytes (version 2). The first 160 bytes keep version 1's layout; version 2 adds the
// measured servo current and voltage and the teleoperation state (follower pose and target, t_ext, gripper),
// so one frame is one synchronized sample for recording. Portable.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "kinematics.h"

namespace raft {

constexpr uint8_t kLeaderFrameVersion = 2;
constexpr size_t kLeaderFrameSize = 288;

enum LeaderFlags : uint16_t {
    LF_CALIBRATED = 1 << 0, LF_TORQUE_ON = 1 << 1, LF_FAULT = 1 << 2, LF_COUPLED = 1 << 3,
    LF_SPRING = 1 << 4, LF_MODEL_OK = 1 << 5, LF_CALIBRATING = 1 << 6, LF_DERATING = 1 << 7,
    LF_FOLLOWER = 1 << 8,        // the follower fields hold fresh PAR6 state
};

enum TeleopFlags : uint8_t {
    TF_ENGAGED = 1 << 0,         // enable held (if required) and no clutch / hold
    TF_EXT_VALID = 1 << 1,       // tauExt is fresh
    TF_ORIENT_LOCK = 1 << 2,
    TF_REPLAYING = 1 << 3,
    TF_HOLD = 1 << 4,            // the web UI's hold (clutch) is on
    TF_ENABLE_BTN = 1 << 5,      // the enable button is pressed
    TF_CLUTCH_BTN = 1 << 6,      // the clutch button is pressed
};

struct LeaderFrame {
    uint8_t  mode;                    // 0 limp, 1 gravity, 2 move, 3 bilateral
    uint16_t flags;                   // LeaderFlags
    uint32_t seq;
    uint32_t tUs;
    float    q[kNumJoints];           // rad, CAD convention (q = 0: standby pose)
    float    qd[kNumJoints];          // rad/s
    float    tauCmd[kNumJoints];      // Nm, commanded
    float    tauGravity[kNumJoints];  // Nm, model gravity load (before the spring)
    float    tcp[6];                  // m, rad (RCB rpy 'xyz'), base frame
    uint8_t  tempC[kNumJoints];
    uint8_t  events;                  // ControlEvent bits raised since the previous frame
    // version 2
    int16_t  currentmA[kNumJoints];   // measured (present current register), joint sign convention
    uint8_t  voltage[kNumJoints];     // 0.1 V
    uint8_t  stage;                   // TeleopSession::Stage (0 = off)
    uint8_t  teleopFlags;             // TeleopFlags
    uint8_t  locks;                   // locked arm joints (bit i = joint i+1)
    float    qFollower[kNumJoints];   // PAR6 pose in RAFT's CAD convention; [6] = gripper as a trigger angle
    float    followerTarget[6];       // last target sent to PAR6, CAD convention
    float    tauExt[6];               // PAR6's t_ext, Nm
    float    gripperRaw;              // PAR6's g_p, 0..255 (-1 = not reported)
    uint32_t followerTick;            // PAR6's loop tick of that state
    float    precision;               // follower motion per leader motion (1 = 1:1)
    uint8_t  guide;                   // Guide
    float    pcEnergyJ;               // passivity observer, smallest joint credit (negative: damping now)
    float    offsetRad;               // largest leader-follower offset of the mapping
};

size_t packLeaderFrame(const LeaderFrame& f, uint8_t* buf, size_t cap);   // returns 288 or 0
bool unpackLeaderFrame(const uint8_t* buf, size_t len, LeaderFrame& f);

}  // namespace raft
