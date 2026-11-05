#include "rti_protocol.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The packet is little-endian ('<' in the Python struct format). ESP32 (Xtensa) and x86 both are,
// so memcpy of the native representation is the wire format.
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "rti_protocol.cpp assumes a little-endian target"
#endif

namespace raft {
namespace rti {

size_t packCommand(uint8_t* buf, size_t cap, uint32_t seq, uint8_t type, const double payload[6],
                   const double feedforward[6], bool withSendTime, double sendTime)
{
    const size_t n = withSendTime ? kCmdTsSize : kCmdSize;
    if (cap < n) return 0;
    memcpy(buf, &seq, 4);
    buf[4] = type;
    memcpy(buf + 5, payload, 48);
    memcpy(buf + 53, feedforward, 48);
    if (withSendTime) memcpy(buf + 101, &sendTime, 8);
    return n;
}

bool unpackCommand(const uint8_t* buf, size_t len, uint32_t* seq, uint8_t* type, double payload[6],
                   double feedforward[6], bool* hasSendTime, double* sendTime)
{
    if (len < kCmdSize) return false;
    memcpy(seq, buf, 4);
    *type = buf[4];
    memcpy(payload, buf + 5, 48);
    memcpy(feedforward, buf + 53, 48);
    const bool ts = (len == kCmdTsSize);
    if (hasSendTime) *hasSendTime = ts;
    if (sendTime) { double t = 0.0; if (ts) memcpy(&t, buf + 101, 8); *sendTime = t; }
    return true;
}

namespace {

// Parse one numeric token [s, e) into v. Tokens are short ("-1.2345"), copy to terminate.
bool toNumber(const char* s, const char* e, double& v)
{
    char tmp[40];
    const size_t n = (size_t)(e - s);
    if (n == 0 || n >= sizeof(tmp)) return false;
    memcpy(tmp, s, n);
    tmp[n] = '\0';
    char* end = nullptr;
    v = strtod(tmp, &end);
    return end == tmp + n;
}

enum Tag { T_NONE, T_JP, T_JS, T_TEXT, T_GP, T_GC, T_RTI_STATE, T_RTI_SEQ, T_TICK, T_MODE, T_OTHER };

Tag tagOf(const char* s, size_t n)
{
    struct { const char* name; Tag tag; } map[] = {
        {"j_p", T_JP}, {"j_s", T_JS}, {"t_ext", T_TEXT}, {"g_p", T_GP}, {"g_c", T_GC},
        {"rti_state", T_RTI_STATE}, {"rti_seq", T_RTI_SEQ}, {"tick", T_TICK}, {"mode", T_MODE},
    };
    for (const auto& m : map)
        if (strlen(m.name) == n && memcmp(m.name, s, n) == 0) return m.tag;
    return T_OTHER;
}

// strtod reads "nan" and "inf" (Python prints a NaN as "nan"), and a huge number would overflow the integer
// conversions below: every value is checked against its tag's plausible range first.
bool usable(Tag t, double v)
{
    if (!isfinite(v)) return false;
    switch (t) {
    case T_JP: return fabs(v) <= kMaxJointRad;
    case T_JS: return fabs(v) <= kMaxJointRadS;
    case T_TEXT: return fabs(v) <= kMaxTorqueNm;
    case T_GP: return v >= kMinGripperRaw && v <= kMaxGripperRaw;
    case T_GC: return fabs(v) <= 1e5;
    case T_RTI_STATE: return v == floor(v) && v >= 0.0 && v <= 4.0;
    case T_RTI_SEQ:
    case T_TICK: return v >= 0.0 && v < 4294967296.0;
    default: return true;
    }
}

}  // namespace

bool parseState(const char* text, size_t len, State& out)
{
    memset(&out, 0, sizeof(out));
    Tag cur = T_NONE;
    int idx = 0;
    int count[T_OTHER + 1] = {0};
    bool bad[T_OTHER + 1] = {false};
    const char* p = text;
    const char* end = text + len;
    while (p < end) {
        const char* q = p;
        while (q < end && *q != ',') ++q;              // token [p, q)
        const char* colon = (const char*)memchr(p, ':', (size_t)(q - p));
        const char* vs = p;
        if (colon) {
            cur = tagOf(p, (size_t)(colon - p));
            idx = 0;
            vs = colon + 1;
        }
        if (cur != T_NONE && cur != T_OTHER) {
            double v = 0.0;
            if (cur == T_MODE) {
                const size_t n = (size_t)(q - vs);
                if (n == 0 || n >= sizeof(out.mode)) return false;
                memcpy(out.mode, vs, n);
                out.mode[n] = '\0';
            } else {
                if (!toNumber(vs, q, v)) return false;
                const bool ok = usable(cur, v);
                if (!ok) bad[cur] = true;
                double* arr = (cur == T_JP) ? out.jp : (cur == T_JS) ? out.js : (cur == T_TEXT) ? out.text : nullptr;
                if (arr) {
                    if (idx >= 6) return false;
                    arr[idx] = ok ? v : 0.0;
                } else if (idx > 0) {
                    return false;                     // scalar tag with more than one value
                } else if (!ok) {
                    // checked after the loop
                } else if (cur == T_GP) out.gp = v;
                else if (cur == T_GC) out.gc = v;
                else if (cur == T_RTI_STATE) out.rtiState = (int)v;
                else if (cur == T_RTI_SEQ) out.rtiSeq = (uint32_t)v;
                else if (cur == T_TICK) out.tick = (uint32_t)v;
            }
            ++idx;
            count[cur] = idx;
        }
        p = q + 1;
    }
    // the joint state and the link bookkeeping must be usable; the optional tags are dropped one by one
    if (bad[T_JP] || bad[T_JS] || bad[T_RTI_STATE] || bad[T_RTI_SEQ] || bad[T_TICK]) return false;
    if (count[T_JP])        { if (count[T_JP] != 6) return false; out.present |= F_JP; }
    if (count[T_JS])        { if (count[T_JS] != 6) return false; out.present |= F_JS; }
    if (count[T_TEXT])      { if (count[T_TEXT] != 6) return false; if (bad[T_TEXT]) out.dropped |= F_TEXT; else out.present |= F_TEXT; }
    if (count[T_GP])        { if (bad[T_GP]) out.dropped |= F_GP; else out.present |= F_GP; }
    if (count[T_GC])        { if (bad[T_GC]) out.dropped |= F_GC; else out.present |= F_GC; }
    if (count[T_RTI_STATE]) out.present |= F_RTI_STATE;
    if (count[T_RTI_SEQ])   out.present |= F_RTI_SEQ;
    if (count[T_TICK])      out.present |= F_TICK;
    if (count[T_MODE])      out.present |= F_MODE;
    return true;
}

static size_t finish(int n, size_t cap) { return (n > 0 && (size_t)n < cap) ? (size_t)n : 0; }

size_t buildRtiConnect(char* buf, size_t cap, const char* id, const char* myIp, float rateHz, const char* client)
{
    // all three args always sent; "" = no rate (hard RTI). The executor checks exact arity.
    if (rateHz > 0.0f)
        return finish(snprintf(buf, cap, "{\"id\": \"%s\", \"cmd\": \"rti_connect\", \"args\": [\"%s\", %.1f, \"udp\"], \"client\": \"%s\"}\n",
                               id, myIp, (double)rateHz, client), cap);
    return finish(snprintf(buf, cap, "{\"id\": \"%s\", \"cmd\": \"rti_connect\", \"args\": [\"%s\", \"\", \"udp\"], \"client\": \"%s\"}\n",
                           id, myIp, client), cap);
}

size_t buildNoArgs(char* buf, size_t cap, const char* id, const char* cmd, const char* client)
{
    return finish(snprintf(buf, cap, "{\"id\": \"%s\", \"cmd\": \"%s\", \"args\": [], \"client\": \"%s\"}\n", id, cmd, client), cap);
}

size_t buildSetRecipe(char* buf, size_t cap, const char* id, const char* recipe, const char* client)
{
    return finish(snprintf(buf, cap, "{\"id\": \"%s\", \"cmd\": \"set_recipe\", \"args\": [\"%s\"], \"client\": \"%s\"}\n", id, recipe, client), cap);
}

size_t buildGripperSet(char* buf, size_t cap, const char* id, int position, int speed, int currentmA, int action, const char* client)
{
    return finish(snprintf(buf, cap, "{\"id\": \"%s\", \"cmd\": \"gripper_set\", \"args\": [%d, %d, %d, %d], \"client\": \"%s\"}\n",
                           id, position, speed, currentmA, action, client), cap);
}

namespace {

// Find "key": in [s, e) and copy its string value, or report a number. Flat scan; the executor's
// replies put id/status/code/type at the top level.
const char* findKey(const char* s, const char* e, const char* key)
{
    char pat[32];
    const int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0) return nullptr;
    for (const char* p = s; p + n <= e; ++p) {
        if (memcmp(p, pat, (size_t)n) == 0) {
            const char* q = p + n;
            while (q < e && (*q == ' ' || *q == '\t')) ++q;
            if (q < e && *q == ':') {
                ++q;
                while (q < e && (*q == ' ' || *q == '\t')) ++q;
                return q;
            }
        }
    }
    return nullptr;
}

bool stringValue(const char* s, const char* e, const char* key, char* out, size_t cap)
{
    const char* v = findKey(s, e, key);
    if (!v) return false;
    if (*v == '"') {
        const char* q = v + 1;
        while (q < e && *q != '"') ++q;
        const size_t n = (size_t)(q - v - 1);
        if (q >= e || n >= cap) return false;
        memcpy(out, v + 1, n);
        out[n] = '\0';
        return true;
    }
    // a number id: copy digits
    const char* q = v;
    while (q < e && ((*q >= '0' && *q <= '9') || *q == '-')) ++q;
    const size_t n = (size_t)(q - v);
    if (n == 0 || n >= cap) return false;
    memcpy(out, v, n);
    out[n] = '\0';
    return true;
}

}  // namespace

bool parseReply(const char* line, size_t len, Reply& out)
{
    memset(&out, 0, sizeof(out));
    const char* e = line + len;
    const char* b = (const char*)memchr(line, '{', len);
    if (!b) return false;
    out.hasId = stringValue(b, e, "id", out.id, sizeof(out.id));
    char st[16] = "";
    if (stringValue(b, e, "status", st, sizeof(st))) {
        if (strcmp(st, "ok") == 0) out.status = Reply::OK;
        else if (strcmp(st, "error") == 0) out.status = Reply::ERROR;
        else if (strcmp(st, "queued") == 0) out.status = Reply::QUEUED;
        else if (strcmp(st, "complete") == 0) out.status = Reply::COMPLETE;
    }
    stringValue(b, e, "code", out.code, sizeof(out.code));
    stringValue(b, e, "type", out.type, sizeof(out.type));
    return true;
}

}  // namespace rti
}  // namespace raft
