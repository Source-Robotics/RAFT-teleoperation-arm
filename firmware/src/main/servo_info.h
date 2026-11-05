// servo_info: what each HLS3606 reports about itself, for bring-up (docs/bringup.md). Portable: the ESP32 reads
// the register table in its control task, which owns the bus (main.cpp); the simulator makes up plausible
// tables (tools/sim/web_sim.cpp). Register numbers are those of the vendored driver (src/HLSCL.h); words are
// little-endian. Fields the driver does not name are only in the raw dump (one joint at a time).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "kinematics.h"

namespace raft {

constexpr int kServoRegs = 71;            // registers 0..70

struct ServoRegisters {
    bool    ok;                           // both reads answered
    uint8_t id;
    uint8_t r[kServoRegs];
};

// JSON reply ({"ok": true, "servos": [...], "notes": [...]}). joint 1..7 adds that servo's raw registers
// as hex ("regs"); joint <= 0 lists all servos without the dump.
size_t formatServoInfo(const ServoRegisters regs[kNumJoints], int joint, char* out, size_t cap);

}  // namespace raft
