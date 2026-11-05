#include "servo_info.h"

#include <stdarg.h>
#include <stdio.h>

namespace raft {
namespace {

// register numbers from src/HLSCL.h
enum : int {
    R_MODEL = 3, R_ID = 5, R_BAUD = 6, R_MIN_ANGLE = 9, R_MAX_ANGLE = 11, R_CW_DEAD = 26, R_CCW_DEAD = 27,
    R_OFS = 31, R_MODE = 33, R_TORQUE_ENABLE = 40, R_GOAL_TORQUE = 44, R_TORQUE_LIMIT = 48, R_LOCK = 55,
    R_POS = 56, R_VOLT = 62, R_TEMP = 63, R_CURRENT = 69,
};

int word(const uint8_t* r, int a) { return r[a] | (r[a + 1] << 8); }
int signed15(int w) { return (w & 0x8000) ? -(w & 0x7FFF) : w; }   // sign-magnitude, bit 15

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

}  // namespace

size_t formatServoInfo(const ServoRegisters regs[kNumJoints], int joint, char* out, size_t cap)
{
    size_t n = 0;
    add(out, cap, n, "{\"ok\": true, \"servos\": [");
    bool first = true;
    for (int i = 0; i < kNumJoints; ++i) {
        if (joint > 0 && i != joint - 1) continue;
        const ServoRegisters& s = regs[i];
        add(out, cap, n, "%s{\"j\": %d, \"id\": %d, \"ok\": %s", first ? "" : ", ", i + 1, s.id, s.ok ? "true" : "false");
        first = false;
        if (s.ok) {
            const uint8_t* r = s.r;
            add(out, cap, n,
                ", \"fw\": [%d, %d], \"model\": %d, \"id_reg\": %d, \"baud\": %d, \"mode\": %d, \"lim\": [%d, %d], \"dead\": [%d, %d], "
                "\"ofs_raw\": %d, \"torque\": %d, \"goal_cur\": %d, \"tq_limit\": %d, \"lock\": %d, \"pos\": %d, \"volt\": %.1f, "
                "\"temp\": %d, \"cur\": %d",
                r[0], r[1], word(r, R_MODEL), r[R_ID], r[R_BAUD], r[R_MODE], word(r, R_MIN_ANGLE), word(r, R_MAX_ANGLE),
                r[R_CW_DEAD], r[R_CCW_DEAD], word(r, R_OFS), r[R_TORQUE_ENABLE], signed15(word(r, R_GOAL_TORQUE)),
                word(r, R_TORQUE_LIMIT), r[R_LOCK], word(r, R_POS), r[R_VOLT] * 0.1, r[R_TEMP], signed15(word(r, R_CURRENT)));
            if (joint > 0) {
                add(out, cap, n, ", \"regs\": \"");
                for (int k = 0; k < kServoRegs; ++k) add(out, cap, n, "%02x", r[k]);
                add(out, cap, n, "\"");
            }
        }
        add(out, cap, n, "}");
    }
    // plain-language notes on what stands out; the firmware itself switches each servo to mode 2 on enable
    add(out, cap, n, "], \"notes\": [");
    first = true;
    auto note = [&](const char* fmt, int a, int b) {
        char t[160];
        snprintf(t, sizeof(t), fmt, a, b);
        add(out, cap, n, "%s\"%s\"", first ? "" : ", ", t);
        first = false;
    };
    int volts = -1;
    for (int i = 0; i < kNumJoints; ++i) {
        const ServoRegisters& s = regs[i];
        if (!s.ok) { note("J%d (ID %d) did not answer the register read", i + 1, s.id); continue; }
        const uint8_t* r = s.r;
        if (r[R_ID] != s.id) note("J%d answers but reports ID %d", i + 1, r[R_ID]);
        // HLS3606: rated 4.5 to 7.4 V, torque figures at 6 V
        if (r[R_VOLT] < 55) note("J%d supply is low: %d tenths of a volt (the torque figures are at 6 V)", i + 1, r[R_VOLT]);
        if (r[R_VOLT] > 76) note("J%d supply is above the 7.4 V rating: %d tenths of a volt", i + 1, r[R_VOLT]);
        if (volts >= 0 && (r[R_VOLT] > volts + 5 || r[R_VOLT] + 5 < volts)) note("J%d supply differs from J1 by more than 0.5 V (%d tenths)", i + 1, r[R_VOLT]);
        if (volts < 0) volts = r[R_VOLT];
        if (word(r, R_TORQUE_LIMIT) < 1000) note("J%d torque limit register 48 is %d (1000 is full scale on Feetech servos): it may cap the current", i + 1, word(r, R_TORQUE_LIMIT));
        if (r[R_TEMP] >= 50) note("J%d is already at %d C", i + 1, r[R_TEMP]);
    }
    add(out, cap, n, "]}");
    return n < cap ? n : 0;
}

}  // namespace raft
