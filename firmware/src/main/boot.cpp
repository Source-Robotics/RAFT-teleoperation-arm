#include "boot.h"

namespace raft {

uint8_t strictBoot(BootBus& bus, uint32_t timeoutMs, uint32_t periodMs)
{
    const uint32_t t0 = bus.nowMs();
    uint8_t m = 0;
    for (;;) {
        bus.torqueOffAll();
        m = bus.missing();
        if (!m || bus.nowMs() - t0 >= timeoutMs) return m;
        bus.waitMs(periodMs);
    }
}

}  // namespace raft
