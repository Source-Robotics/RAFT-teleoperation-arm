// Raw servo bus readings as the HLS3606 reports them (one SyncRead of registers 56..70). Portable: the
// ESP32 fills it from the bus (servo_bus.cpp), the PC simulator from MuJoCo (tools/sim).
#pragma once

#include <stdint.h>

#include "kinematics.h"

namespace raft {

struct RawServoState {
    bool     ok[kNumJoints];       // the servo answered this read
    int16_t  pos[kNumJoints];      // ticks 0..4095 (4096 per turn)
    int16_t  speed[kNumJoints];    // 50 steps/s units (0.0767 rad/s), signed
    int16_t  current[kNumJoints];  // present current register, signed (unit to calibrate)
    uint8_t  tempC[kNumJoints];
    uint8_t  voltage[kNumJoints];  // 0.1 V units
};

constexpr float kRadPerTick = 6.28318530718f / 4096.0f;
constexpr float kRadPsPerSpeedUnit = 0.732f * 6.28318530718f / 60.0f;

// Signed tick difference raw - zero, wrapped to [-2048, 2047] (half a turn either way of the zero pose).
inline int ticksFromZero(int raw, int zero)
{
    int d = (raw - zero + 2048) % 4096;
    if (d < 0) d += 4096;
    return d - 2048;
}

}  // namespace raft
