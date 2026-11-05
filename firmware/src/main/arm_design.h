// RAFT design constants: the same on every arm built to the CAD, so they live in the firmware and not in the
// per-arm settings in flash (config_store.h: encoder zeros, torque constants, ranges, spring, teleop settings).
// Plain C++11, no Arduino dependencies.
#pragma once

#include <stdint.h>

#include "kinematics.h"

namespace raft {

// Servo ticks against the model's + direction (PAR6's): how each servo sits in the arm. J1 and the trigger turn
// the other way round (first arm, 2026-11-02).
constexpr int8_t kJointDir[kNumJoints] = {-1, 1, 1, 1, 1, 1, -1};

// The mechanical stop of every joint, as the angle the joint reads there (CAD convention: 0 in the standby pose).
// Measured on the first arm: how far each joint moves from its stop to standby (2026-11-02). The trigger's stop is 39.63 deg
// past the pose where the trigger is parallel to the handle's face, which is its zero.
constexpr float kStopDeg[kNumJoints] = {-136.28f, -83.96f, 92.81f, -158.56f, 119.72f, 149.37f, 39.63f};

// Damping in Gravity mode, Nm s/rad: the servos' gears run so easily from the joint side that a light wrist joint keeps
// coasting after you let go. Tuned on the first arm (2026-11-02); the heavier joints stop by themselves.
constexpr float kGravityDampingNms[kNumJoints] = {0.0f, 0.0f, 0.0f, 0.0f, 0.03f, 0.02f, 0.035f};

// Hold, moves and replay use the servos' own position mode (stiff, full current), not a spring computed over the bus
// (soft: the bus delay makes more than about 2x shake). Gravity and teleoperation stay in current mode.
constexpr bool kHoldServoPosition = true;
constexpr int kPositionWrapMarginTicks = 64;   // position goals stay this far from the encoder's 0/4095 wrap

// The control loop, like PAR6's: 250 Hz. One tick needs about 3 ms of bus time (servo_bus.h), the rest is margin.
constexpr int kControlHz = 250;
constexpr uint32_t kControlPeriodUs = 1000000 / kControlHz;

}  // namespace raft
