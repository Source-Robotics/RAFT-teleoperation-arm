#include "leader_frame.h"

#include <string.h>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "leader_frame.cpp assumes a little-endian target"
#endif

namespace raft {

size_t packLeaderFrame(const LeaderFrame& f, uint8_t* b, size_t cap)
{
    if (cap < kLeaderFrameSize) return 0;
    memset(b, 0, kLeaderFrameSize);
    memcpy(b, "RAFT", 4);
    b[4] = kLeaderFrameVersion;
    b[5] = f.mode;
    memcpy(b + 6, &f.flags, 2);
    memcpy(b + 8, &f.seq, 4);
    memcpy(b + 12, &f.tUs, 4);
    memcpy(b + 16, f.q, 28);
    memcpy(b + 44, f.qd, 28);
    memcpy(b + 72, f.tauCmd, 28);
    memcpy(b + 100, f.tauGravity, 28);
    memcpy(b + 128, f.tcp, 24);
    memcpy(b + 152, f.tempC, 7);
    b[159] = f.events;
    memcpy(b + 160, f.currentmA, 14);
    memcpy(b + 174, f.voltage, 7);
    b[181] = f.stage;
    b[182] = f.teleopFlags;
    b[183] = f.locks;
    memcpy(b + 184, f.qFollower, 28);
    memcpy(b + 212, f.followerTarget, 24);
    memcpy(b + 236, f.tauExt, 24);
    memcpy(b + 260, &f.gripperRaw, 4);
    memcpy(b + 264, &f.followerTick, 4);
    memcpy(b + 268, &f.precision, 4);
    b[272] = f.guide;
    memcpy(b + 276, &f.pcEnergyJ, 4);
    memcpy(b + 280, &f.offsetRad, 4);
    return kLeaderFrameSize;
}

bool unpackLeaderFrame(const uint8_t* b, size_t len, LeaderFrame& f)
{
    if (len != kLeaderFrameSize || memcmp(b, "RAFT", 4) != 0 || b[4] != kLeaderFrameVersion) return false;
    f.mode = b[5];
    memcpy(&f.flags, b + 6, 2);
    memcpy(&f.seq, b + 8, 4);
    memcpy(&f.tUs, b + 12, 4);
    memcpy(f.q, b + 16, 28);
    memcpy(f.qd, b + 44, 28);
    memcpy(f.tauCmd, b + 72, 28);
    memcpy(f.tauGravity, b + 100, 28);
    memcpy(f.tcp, b + 128, 24);
    memcpy(f.tempC, b + 152, 7);
    f.events = b[159];
    memcpy(f.currentmA, b + 160, 14);
    memcpy(f.voltage, b + 174, 7);
    f.stage = b[181];
    f.teleopFlags = b[182];
    f.locks = b[183];
    memcpy(f.qFollower, b + 184, 28);
    memcpy(f.followerTarget, b + 212, 24);
    memcpy(f.tauExt, b + 236, 24);
    memcpy(&f.gripperRaw, b + 260, 4);
    memcpy(&f.followerTick, b + 264, 4);
    memcpy(&f.precision, b + 268, 4);
    f.guide = b[272];
    memcpy(&f.pcEnergyJ, b + 276, 4);
    memcpy(&f.offsetRad, b + 280, 4);
    return true;
}

}  // namespace raft
