#pragma once

// =============================================================================
// robot_state.h
// Real-time snapshot of the entire robotic arm state.
//
// One global instance `g_state` is declared in main.cpp.
// The servo feedback loop (Phase 1) writes to it every 10 ms.
// The command interface, telemetry output, and web dashboard all read from it.
//
// In Phase 7 this will be protected by a FreeRTOS mutex for dual-core safety.
// =============================================================================

#include <Arduino.h>
#include "config.h"

// =============================================================================
// Error flag bitmask — stored per joint in errorFlags[NUM_JOINTS]
// Multiple flags can be set simultaneously via bitwise OR.
// =============================================================================
static constexpr uint8_t ERR_NONE        = 0x00;  // No error
static constexpr uint8_t ERR_OVERCURRENT = 0x01;  // Joint current exceeded CURRENT_FAULT_RAW
static constexpr uint8_t ERR_OVERTEMP    = 0x02;  // Joint temperature exceeded TEMP_FAULT_C
static constexpr uint8_t ERR_TEMP_WARN   = 0x04;  // Joint temperature exceeded TEMP_WARN_C
static constexpr uint8_t ERR_COMM_FAIL   = 0x08;  // Serial communication failure on this joint
static constexpr uint8_t ERR_NOT_HOMED   = 0x10;  // Joint has not been homed yet
static constexpr uint8_t ERR_JOINT_LIMIT = 0x20;  // Commanded position was outside soft limits

// =============================================================================
// Homing state — one per joint, used by the Phase 4 homing state machine
// =============================================================================
enum class HomingState : uint8_t
{
    IDLE,             // Not homing
    SEEKING_LIMIT,    // Moving toward mechanical hard stop at low speed
    AT_LIMIT,         // Hard stop detected via torque threshold
    MOVING_TO_OFFSET, // Moving from hard stop to the defined zero position
    HOMED             // Homing complete, zero calibrated
};

// =============================================================================
// RobotState struct
// All values are in SI / real-world units (rad, rad/s, Nm, A, V, °C).
// =============================================================================
struct RobotState
{
    // --- Real-time feedback (updated at ~100 Hz by readAllServos) ---
    float    position[NUM_JOINTS];     // Current position  (rad)
    float    speed[NUM_JOINTS];        // Current speed     (rad/s)
    float    torque[NUM_JOINTS];       // Current load/torque estimate (Nm)
    float    current[NUM_JOINTS];      // Motor current     (A)
    float    voltage[NUM_JOINTS];      // Supply voltage    (V)
    float    temperature[NUM_JOINTS];  // Motor temperature (°C)

    // --- Status ---
    bool     isHomed;                  // true when all joints have been successfully homed
    bool     torqueEnabled;            // true when torque output is active on all joints
    bool     ethConnected;             // true when Ethernet link is up

    // --- Error / warning tracking ---
    uint8_t  errorFlags[NUM_JOINTS];   // Bitmask per joint (ERR_* constants above)
    int      commFailCount[NUM_JOINTS];// Consecutive communication failure counter per joint

    // --- Homing state machine ---
    HomingState homingState[NUM_JOINTS]; // Current homing state for each joint

    // --- Timing ---
    uint32_t timestamp_ms;             // millis() at the time of the last full state update

    // --- Loop performance ---
    uint32_t lastLoopDuration_us;      // Duration of the last servo read cycle (microseconds)
};

// =============================================================================
// Helpers
// =============================================================================

// Set an error flag on a joint (non-destructive — preserves other flags)
inline void setError(RobotState& state, int jointIndex, uint8_t flag)
{
    if (jointIndex >= 0 && jointIndex < NUM_JOINTS)
        state.errorFlags[jointIndex] |= flag;
}

// Clear an error flag on a joint
inline void clearError(RobotState& state, int jointIndex, uint8_t flag)
{
    if (jointIndex >= 0 && jointIndex < NUM_JOINTS)
        state.errorFlags[jointIndex] &= ~flag;
}

// Returns true if a specific error flag is set on a joint
inline bool hasError(const RobotState& state, int jointIndex, uint8_t flag)
{
    if (jointIndex < 0 || jointIndex >= NUM_JOINTS) return false;
    return (state.errorFlags[jointIndex] & flag) != 0;
}

// Returns true if any joint has any error flag set
inline bool hasAnyError(const RobotState& state)
{
    for (int i = 0; i < NUM_JOINTS; i++)
        if (state.errorFlags[i] != ERR_NONE) return true;
    return false;
}

// Zero-initialise the whole state (call once in setup before first read)
inline void initRobotState(RobotState& state)
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        state.position[i]      = 0.0f;
        state.speed[i]         = 0.0f;
        state.torque[i]        = 0.0f;
        state.current[i]       = 0.0f;
        state.voltage[i]       = 0.0f;
        state.temperature[i]   = 0.0f;
        state.errorFlags[i]    = ERR_NOT_HOMED;  // All joints start as not-homed
        state.commFailCount[i] = 0;
        state.homingState[i]   = HomingState::IDLE;
    }
    state.isHomed            = false;
    state.torqueEnabled      = false;
    state.ethConnected       = false;
    state.timestamp_ms       = 0;
    state.lastLoopDuration_us = 0;
}
