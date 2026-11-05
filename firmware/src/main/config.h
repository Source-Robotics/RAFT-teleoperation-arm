#pragma once

// =============================================================================
// config.h
// Per-joint configuration for the RAFT 6-axis robotic arm + gripper
//
// Joint layout:
//   Index 0 → Servo ID 0 → Base rotation         (J1)
//   Index 1 → Servo ID 1 → Shoulder              (J2)
//   Index 2 → Servo ID 2 → Elbow                 (J3)
//   Index 3 → Servo ID 3 → Wrist pitch            (J4)
//   Index 4 → Servo ID 4 → Wrist roll             (J5)
//   Index 5 → Servo ID 5 → End-effector rotation  (J6)
//   Index 6 → Servo ID 6 → Gripper               (G)
//
// Servo IDs start at 0, as on PAROL6 and PAR6.
//
// All angle values in radians.
// Values marked TODO must be measured on the physical robot and updated here.
// =============================================================================

#include <Arduino.h>
#include "units.h"

// Total number of actuators (6 arm joints + 1 gripper)
static constexpr uint8_t NUM_JOINTS = 7;

// Index of the gripper in the JOINTS array
static constexpr uint8_t GRIPPER_INDEX = 6;

// =============================================================================
// JointConfig struct
// All parameters that describe one actuator.
// =============================================================================
struct JointConfig
{
    uint8_t     id;                    // Servo bus ID (1-7)
    const char* name;                  // Human-readable label for debug output

    // --- Soft position limits (enforced before every move command) ---
    float       minRad;                // Minimum allowed position (rad) TODO: measure per joint
    float       maxRad;                // Maximum allowed position (rad) TODO: measure per joint

    // --- Normal operation ---
    float       maxSpeedRadPerSec;     // Maximum commanded speed (rad/s)
    uint16_t    maxTorqueLimit;        // Maximum torque limit for normal moves (raw, 0-1000)

    // --- Homing ---
    int8_t      homingDirection;       // Direction to seek limit: +1 = positive ticks, -1 = negative ticks
                                       // TODO: determine correct direction for each joint
    uint16_t    homingSpeed;           // Raw speed units during homing (slow, ~50 units ≈ 0.55 rad/s)
    int         homingTorqueThreshold; // Raw load threshold that indicates limit contact (abs value)
                                       // TODO: tune per joint during Phase 4.2 testing
    int16_t     homingOffsetTicks;     // Ticks to travel FROM the hard limit TO the 0-rad position
                                       // TODO: measure per joint in Phase 4.3 (positive = away from limit)
};

// =============================================================================
// JOINTS[NUM_JOINTS] — configuration array
//
// PLACEHOLDER VALUES — update after physical testing:
//   - minRad / maxRad : set to ±π until real range-of-motion is measured
//   - homingDirection : all set to -1 (verify on each joint before first homing run)
//   - homingTorqueThreshold : 400 raw units = 40% stall (conservative start)
//   - homingOffsetTicks : 0 (unknown — must be filled after Phase 4.3)
//   - maxTorqueLimit : 600 (60% stall) — reduce if a joint has lower load requirements
// =============================================================================
static const JointConfig JOINTS[NUM_JOINTS] = {
    //  id  name         minRad   maxRad  maxSpd  maxTrq  homDir  homSpd  homTrqThr  homOfsT
    {   0,  "J1_Base",   -PI,     PI,     1.5f,   600,    -1,     50,     400,       0   },  // TODO: verify all values
    {   1,  "J2_Shoulder",-PI,    PI,     1.5f,   600,    -1,     50,     400,       0   },  // TODO: verify all values
    {   2,  "J3_Elbow",  -PI,     PI,     1.5f,   600,    -1,     50,     400,       0   },  // TODO: verify all values
    {   3,  "J4_WristP", -PI,     PI,     1.5f,   500,    -1,     50,     400,       0   },  // TODO: verify all values
    {   4,  "J5_WristR", -PI,     PI,     1.5f,   500,    -1,     50,     400,       0   },  // TODO: verify all values
    {   5,  "J6_EErot",  -PI,     PI,     1.5f,   500,    -1,     50,     400,       0   },  // TODO: verify all values
    {   6,  "G_Gripper",  0.0f,   PI,     1.0f,   400,    -1,     30,     300,       0   },  // TODO: verify all values
};

// =============================================================================
// Safety thresholds (shared across all joints)
// =============================================================================

// Overcurrent: disable joint immediately (raw current units)
// 1.3A stall / 0.0065 = 200 raw units; 15% margin → 230
static constexpr int   CURRENT_FAULT_RAW       = 230;

// Overtemperature: above this, cut max torque to 50%
static constexpr float TEMP_WARN_C             = 60.0f;

// Overtemperature: above this, disable the joint entirely
static constexpr float TEMP_FAULT_C            = 70.0f;

// Communication fault: if ReadPos/FeedBack fails this many times in a row, flag the joint
static constexpr int   COMM_FAULT_COUNT        = 5;

// Command watchdog: if no command is received for this duration, hold position (ms)
static constexpr uint32_t CMD_WATCHDOG_MS      = 5000;

// =============================================================================
// Helper: look up the JOINTS index for a given servo ID.
// Returns -1 if the ID is not found.
// =============================================================================
inline int jointIndexByID(uint8_t servoID)
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        if (JOINTS[i].id == servoID) return i;
    }
    return -1;
}

// =============================================================================
// Helper: returns true if the given position (rad) is within the soft limits
// for the joint at `index`.
// =============================================================================
inline bool isWithinLimits(int index, float posRad)
{
    if (index < 0 || index >= NUM_JOINTS) return false;
    return (posRad >= JOINTS[index].minRad && posRad <= JOINTS[index].maxRad);
}
