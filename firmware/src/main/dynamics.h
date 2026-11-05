// RAFT static gravity torque g(q), the on-device equivalent of the RCB runtime's RNE gravity term.
// Plain C++11, no Arduino dependencies.
#pragma once

#include "kinematics.h"

namespace raft {

// Torque (Nm) each joint must apply to hold the arm still against gravity, about the joint's
// +z axis (the model's positive rotation direction). Reference: tau_rne() in tools/gen_model.py.
void gravityTorque(const Frame T[kNumFrames], float tau[kNumJoints]);   // reuses a forward-kinematics result
void gravityTorque(const float q[kNumJoints], float tau[kNumJoints]);

}  // namespace raft
