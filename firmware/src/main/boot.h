// Strict boot (no bypass), portable so the PC test (tools/sim/test_app.cpp) runs the same sequence
// against a fake bus. Torque goes off before the first presence check and again before every retry: after an
// ESP32-only reset the servos are still powered and keep their last goal current until told otherwise, and
// a servo that comes up late gets the same command. The caller halts (with torque off) when servos are missing.
#pragma once

#include <stdint.h>

namespace raft {

class BootBus {
public:
    virtual ~BootBus() {}
    virtual void torqueOffAll() = 0;      // zero goal current and torque off on every configured ID (no reply needed)
    virtual uint8_t missing() = 0;        // one presence check of all servos: bitmask of the silent ones
    virtual void waitMs(uint32_t ms) = 0;
    virtual uint32_t nowMs() = 0;
};

// Returns the bitmask of servos that never answered within timeoutMs (0 = all present).
uint8_t strictBoot(BootBus& bus, uint32_t timeoutMs, uint32_t periodMs);

}  // namespace raft
