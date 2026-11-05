#include "web_bridge.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "raft_model.h"

namespace raft {
namespace {

const float kRadToDeg = 57.2957795f;

struct SpringPreset { const char* name; const char* shop; float k, aMm, bMm; };
// docs/spring_mount.md: federnshop set on the a = 13 / b = 20 mount, Springs Online on 16 / 27
const SpringPreset kSprings[] = {
    {"Z-075NI", "federnshop", 565.0f, 13.0f, 20.0f},
    {"Z-078AI", "federnshop", 718.0f, 13.0f, 20.0f},
    {"Z-078I", "federnshop", 824.0f, 13.0f, 20.0f},
    {"Z-077AI", "federnshop", 1027.0f, 13.0f, 20.0f},
    {"SOTRI0073", "Springs Online", 340.0f, 16.0f, 27.0f},
    {"SOTR0073", "Springs Online", 410.0f, 16.0f, 27.0f},
    {"SOTRI0072", "Springs Online", 530.0f, 16.0f, 27.0f},
    {"SOTR0072", "Springs Online", 640.0f, 16.0f, 27.0f},
};

// appends printf output; returns false once the buffer is full
bool add(char* out, size_t cap, size_t& n, const char* fmt, ...)
{
    if (n >= cap) return false;
    va_list ap;
    va_start(ap, fmt);
    const int m = vsnprintf(out + n, cap - n, fmt, ap);
    va_end(ap);
    if (m < 0 || n + (size_t)m >= cap) { n = cap; return false; }
    n += (size_t)m;
    return true;
}

void put16(uint8_t* b, size_t o, uint16_t v) { b[o] = (uint8_t)v; b[o + 1] = (uint8_t)(v >> 8); }
void put32(uint8_t* b, size_t o, uint32_t v) { for (int k = 0; k < 4; ++k) b[o + k] = (uint8_t)(v >> (8 * k)); }
void putF(uint8_t* b, size_t o, float v) { uint32_t u; memcpy(&u, &v, 4); put32(b, o, u); }

}  // namespace

size_t packHealth(const HealthFigures& f, const uint32_t missed[kNumJoints], uint32_t staleStates, uint32_t staleTargets, uint8_t* b, size_t cap)
{
    if (cap < kHealthSize) return 0;
    memset(b, 0, kHealthSize);
    const LoopReport& L = f.loop;
    memcpy(b, "RFTH", 4);
    b[4] = 1;
    b[5] = f.flags;
    put16(b, 6, (uint16_t)kHealthSize);
    put32(b, 8, L.seq); put32(b, 12, L.ticks); putF(b, 16, L.targetHz); putF(b, 20, L.hz);
    put32(b, 24, L.periodMinUs); put32(b, 28, L.periodMaxUs); putF(b, 32, L.periodMeanUs); putF(b, 36, L.jitterRmsUs);
    putF(b, 40, L.workMeanUs); put32(b, 44, L.workMaxUs);
    for (int k = 0; k < kNumLoopParts; ++k) { putF(b, 48 + 4 * k, L.partMeanUs[k]); put32(b, 64 + 4 * k, L.partMaxUs[k]); }
    put32(b, 80, L.overruns); put32(b, 84, L.failedReads); put32(b, 88, L.overrunsTotal); put32(b, 92, L.failedReadsTotal);
    putF(b, 96, L.cpuLoad[0]); putF(b, 100, L.cpuLoad[1]);
    put32(b, 104, f.uptimeS); put32(b, 108, f.heapFree); put32(b, 112, f.heapMin); put32(b, 116, f.heapSize);
    put32(b, 120, f.stackControl); put32(b, 124, f.stackNetwork); put32(b, 128, f.stackLoop);
    put16(b, 132, (uint16_t)f.linkMbps); b[134] = f.webClients; b[135] = f.tcpClients;
    put32(b, 136, f.webFrames); put32(b, 140, f.udpFrames);
    for (int i = 0; i < kNumJoints; ++i) put32(b, 144 + 4 * i, missed[i]);
    put32(b, 172, staleStates); put32(b, 176, staleTargets);
    return kHealthSize;
}

uint32_t WebBridge::statusKey()
{
    lock();
    app_.snapshot(snap_);
    unlock();
    return App::statusKey(snap_);
}

size_t WebBridge::health(uint8_t* buf, size_t cap)
{
    HealthFigures f;
    if (!healthFigures(f)) return 0;
    uint32_t missed[kNumJoints];
    lock();
    memcpy(missed, app_.missedReads(), sizeof(missed));
    const uint32_t ss = app_.link().staleStates(), stt = app_.link().staleTargets();
    unlock();
    return packHealth(f, missed, ss, stt, buf, cap);
}

size_t WebBridge::command(const char* json, char* reply, size_t cap, const char* clientIp)
{
    const size_t direct = outsideLockCommand(json, reply, cap);
    if (direct) return direct;
    lock();
    size_t n = extraCommand(json, reply, cap);
    if (!n) n = app_.handleCommand(json, reply, cap, now(), clientIp);
    unlock();
    return n;
}

size_t WebBridge::status(char* out, size_t cap)
{
    // only the copy holds the lock: the control task waits for every microsecond it is held
    char s[1200];
    const size_t m = sys(s, sizeof(s));
    lock();
    app_.snapshot(snap_);
    unlock();
    size_t n = App::formatStatus(snap_, out, cap);
    if (n > 1 && out[n - 1] == '}' && m && n - 1 + m + 12 < cap) {
        n -= 1;
        n += (size_t)snprintf(out + n, cap - n, ", \"sys\": {%s}}", s);
    }
    return n;
}

size_t WebBridge::frame(uint8_t* buf, size_t cap)
{
    lock();
    const size_t n = app_.leaderFrame(buf, cap, 1);
    unlock();
    return n;
}

size_t WebBridge::hello(char* out, size_t cap)
{
    float qMin[kNumJoints], qMax[kNumJoints], rest[kNumJoints];
    lock();
    const ControlConfig& c = app_.controlConfig();
    for (int i = 0; i < kNumJoints; ++i) { qMin[i] = c.joint[i].qMinRad; qMax[i] = c.joint[i].qMaxRad; rest[i] = c.autoRest.qRest[i]; }
    unlock();
    size_t n = 0;
    add(out, cap, n, "{\"fw\": \"%s\", \"sim\": %s, \"dh\": [", firmware_, sim_ ? "true" : "false");
    for (int i = 0; i < raft_model::NUM_ARM_JOINTS; ++i) {
        const raft_model::DhLink& L = raft_model::DH[i];
        add(out, cap, n, "%s[%.6f, %.6f, %.8f, %.8f]", i ? ", " : "", (double)L.d, (double)L.a, (double)L.alpha, (double)L.offset);
    }
    add(out, cap, n, "], \"grip_r\": [");
    for (int i = 0; i < 9; ++i) add(out, cap, n, "%s%.8f", i ? ", " : "", (double)raft_model::GRIP_R[i]);
    add(out, cap, n, "], \"grip_p\": [%.6f, %.6f, %.6f], \"lim_deg\": [",
        (double)raft_model::GRIP_P[0], (double)raft_model::GRIP_P[1], (double)raft_model::GRIP_P[2]);
    for (int i = 0; i < kNumJoints; ++i)
        add(out, cap, n, "%s[%.1f, %.1f]", i ? ", " : "", (double)(qMin[i] * kRadToDeg), (double)(qMax[i] * kRadToDeg));
    add(out, cap, n, "], \"rest_deg\": [");
    for (int i = 0; i < kNumJoints; ++i) add(out, cap, n, "%s%.1f", i ? ", " : "", (double)(rest[i] * kRadToDeg));
    add(out, cap, n, "], \"springs\": [");
    for (size_t i = 0; i < sizeof(kSprings) / sizeof(kSprings[0]); ++i)
        add(out, cap, n, "%s{\"name\": \"%s\", \"shop\": \"%s\", \"k\": %.0f, \"a_mm\": %.0f, \"b_mm\": %.0f}", i ? ", " : "",
            kSprings[i].name, kSprings[i].shop, (double)kSprings[i].k, (double)kSprings[i].aMm, (double)kSprings[i].bMm);
    add(out, cap, n, "]");
    char sys[200];
    const size_t m = helloSys(sys, sizeof(sys));
    if (m) add(out, cap, n, ", \"sys\": {%s}", sys);
    add(out, cap, n, "}");
    return n < cap ? n : 0;
}

}  // namespace raft
