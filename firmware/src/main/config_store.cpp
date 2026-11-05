#include "config_store.h"

#include <math.h>
#include <stdio.h>

#include "arm_design.h"

#include <string.h>

namespace raft {
namespace {

const float kPi = 3.14159265358979f;
const float kRangeMarginRad = 2.0f * kPi / 180.0f;   // MOVE targets and barriers stay this far inside a measured end

// The version 1 layout, kept to upgrade blobs saved before 2026-10-31
struct JointCalV1 {
    int16_t zeroTicks;
    int8_t  dir;
    uint8_t flags;
    float   ktNmPerA;
    float   frictionNm;
};
struct PersistentConfigV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    JointCalV1 joint[kNumJoints];
    float    mAPerUnit;
    float    currentRegPerUnit;
    float    gravityScale;
    SpringModel spring;
    uint8_t  autoRest;
    float    autoRestIdleS;
    char     piHost[40];
    char     streamHost[40];
    uint16_t streamPort;
    uint16_t reserved;
    uint32_t crc;
};

// The version 2 layout, kept to upgrade blobs saved before 2026-11-03 (no holdFriction)
struct PersistentConfigV2 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    JointCal joint[kNumJoints];
    float    mAPerUnit;
    float    currentRegPerUnit;
    float    gravityScale;
    SpringModel spring;
    uint8_t  autoRest;
    float    autoRestIdleS;
    char     piHost[40];
    char     streamHost[40];
    uint16_t streamPort;
    uint16_t reserved;
    TeleopSettings teleop;
    uint32_t crc;
};

// The version 3 layout, kept to upgrade blobs saved before 2026-11-05 (no armName): version 4 up to holdFriction
struct PersistentConfigV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    JointCal joint[kNumJoints];
    float    mAPerUnit;
    float    currentRegPerUnit;
    float    gravityScale;
    SpringModel spring;
    uint8_t  autoRest;
    float    autoRestIdleS;
    char     piHost[40];
    char     streamHost[40];
    uint16_t streamPort;
    uint16_t reserved;
    TeleopSettings teleop;
    float    holdFriction;
    uint32_t crc;
};
static_assert(offsetof(PersistentConfigV3, crc) == offsetof(PersistentConfig, armName), "version 4 = version 3 + armName");

bool upgradeV3(const uint8_t* buf, PersistentConfig& out)
{
    PersistentConfigV3 v;
    memcpy(&v, buf, sizeof(v));
    if (v.magic != kConfigMagic || v.version != 3 || v.size != sizeof(PersistentConfigV3)) return false;
    if (v.crc != crc32((const uint8_t*)&v, offsetof(PersistentConfigV3, crc))) return false;
    PersistentConfig c;
    defaultPersistentConfig(c);
    memcpy(&c, &v, offsetof(PersistentConfigV3, crc));   // the same layout up to the new field
    c.version = kConfigVersion;
    c.size = (uint16_t)sizeof(PersistentConfig);
    c.piHost[sizeof(c.piHost) - 1] = '\0';
    c.streamHost[sizeof(c.streamHost) - 1] = '\0';
    for (int i = 0; i < kNumJoints; ++i) { c.joint[i].dir = kJointDir[i]; c.joint[i].flags |= CAL_DIR; }
    out = c;
    return true;
}

bool upgradeV2(const uint8_t* buf, PersistentConfig& out)
{
    PersistentConfigV2 v;
    memcpy(&v, buf, sizeof(v));
    if (v.magic != kConfigMagic || v.version != 2 || v.size != sizeof(PersistentConfigV2)) return false;
    if (v.crc != crc32((const uint8_t*)&v, offsetof(PersistentConfigV2, crc))) return false;
    PersistentConfig c;
    defaultPersistentConfig(c);
    for (int i = 0; i < kNumJoints; ++i) { c.joint[i] = v.joint[i]; c.joint[i].dir = kJointDir[i]; c.joint[i].flags |= CAL_DIR; }
    c.mAPerUnit = v.mAPerUnit;
    c.currentRegPerUnit = v.currentRegPerUnit;
    // version 2 had one knob: 1.3 meant real gravity plus 0.3 for the gear friction
    c.gravityScale = v.gravityScale > 1.0f ? 1.0f : v.gravityScale;
    c.holdFriction = v.gravityScale > 1.0f ? v.gravityScale - 1.0f : kBenchHoldFriction;
    c.spring = v.spring;
    c.autoRest = v.autoRest;
    c.autoRestIdleS = v.autoRestIdleS;
    memcpy(c.piHost, v.piHost, sizeof(c.piHost));
    memcpy(c.streamHost, v.streamHost, sizeof(c.streamHost));
    c.piHost[sizeof(c.piHost) - 1] = '\0';
    c.streamHost[sizeof(c.streamHost) - 1] = '\0';
    c.streamPort = v.streamPort;
    c.teleop = v.teleop;
    out = c;
    return true;
}

bool upgradeV1(const uint8_t* buf, PersistentConfig& out)
{
    PersistentConfigV1 v;
    memcpy(&v, buf, sizeof(v));
    if (v.magic != kConfigMagic || v.version != 1 || v.size != sizeof(PersistentConfigV1)) return false;
    if (v.crc != crc32((const uint8_t*)&v, offsetof(PersistentConfigV1, crc))) return false;
    PersistentConfig c;
    defaultPersistentConfig(c);
    for (int i = 0; i < kNumJoints; ++i) {
        c.joint[i].zeroTicks = v.joint[i].zeroTicks;
        c.joint[i].dir = kJointDir[i];
        c.joint[i].flags = (uint8_t)(v.joint[i].flags & (CAL_ZERO | CAL_DIR | CAL_KT));
        c.joint[i].ktNmPerA = v.joint[i].ktNmPerA;
        c.joint[i].frictionNm = v.joint[i].frictionNm;
    }
    c.mAPerUnit = v.mAPerUnit;
    c.currentRegPerUnit = v.currentRegPerUnit;
    c.gravityScale = v.gravityScale;
    c.spring = v.spring;
    c.autoRest = v.autoRest;
    c.autoRestIdleS = v.autoRestIdleS;
    memcpy(c.piHost, v.piHost, sizeof(c.piHost));
    memcpy(c.streamHost, v.streamHost, sizeof(c.streamHost));
    c.piHost[sizeof(c.piHost) - 1] = '\0';
    c.streamHost[sizeof(c.streamHost) - 1] = '\0';
    c.streamPort = v.streamPort;
    out = c;
    return true;
}

}  // namespace

void defaultPersistentConfig(PersistentConfig& c)
{
    memset(&c, 0, sizeof(c));
    c.magic = kConfigMagic;
    c.version = kConfigVersion;
    c.size = (uint16_t)sizeof(PersistentConfig);
    const ControlConfig d = defaultControlConfig();
    for (int i = 0; i < kNumJoints; ++i) {
        c.joint[i].zeroTicks = 2048;
        c.joint[i].dir = kJointDir[i];
        c.joint[i].flags = CAL_DIR;
        c.joint[i].ktNmPerA = d.joint[i].ktNmPerA;
        c.joint[i].frictionNm = 0.0f;
        c.joint[i].qMinRad = d.joint[i].qMinRad;
        c.joint[i].qMaxRad = d.joint[i].qMaxRad;
    }
    c.mAPerUnit = d.mAPerUnit;
    c.currentRegPerUnit = 1.0f;
    c.gravityScale = d.gravityScale;   // 1: real gravity
    // Friction compensation (holding) 0.3: the HLS3606's gears run easily when gravity drives them from the joint,
    // but the motor needs about 0.3 Nm more to drive them itself (J3 lifted its hanging forearm only at ~650 mA,
    // 2026-11-02). With gravity alone the arm sinks wherever the model is a little short; 30 % more push held most
    // poses on the arm without creeping up.
    c.holdFriction = kBenchHoldFriction;
    c.spring = d.spring;
    c.autoRest = 0;
    c.autoRestIdleS = d.autoRest.idleTimeS;
    c.streamPort = 5005;
    c.teleop.autoReclaim = 0;
    c.teleop.requireEnable = 0;
    c.teleop.passivity = d.passivity ? 1 : 0;
    c.teleop.barriersAlways = 0;
    c.teleop.forceGain = d.extGain;
    c.teleop.forceDeadbandNm = d.extDeadbandNm;
    c.teleop.forceSign = d.extSign < 0.0f ? -1 : 1;
    c.teleop.gripperOpenRaw = 0;
    c.teleop.gripperClosedRaw = 255;
    c.teleop.triggerOpenRad = 0.0f;                    // placeholders until the trigger's travel is measured
    c.teleop.triggerClosedRad = 60.0f * kPi / 180.0f;
    c.teleop.frictionScale = d.frictionScale;
}

uint32_t crc32(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

size_t serializeConfig(const PersistentConfig& c, uint8_t* buf, size_t cap)
{
    if (cap < sizeof(PersistentConfig)) return 0;
    PersistentConfig t = c;
    t.magic = kConfigMagic;
    t.version = kConfigVersion;
    t.size = (uint16_t)sizeof(PersistentConfig);
    t.crc = crc32((const uint8_t*)&t, offsetof(PersistentConfig, crc));
    memcpy(buf, &t, sizeof(t));
    return sizeof(t);
}

bool deserializeConfig(const uint8_t* buf, size_t len, PersistentConfig& out)
{
    if (len == sizeof(PersistentConfigV1)) return upgradeV1(buf, out);
    if (len == sizeof(PersistentConfigV2)) return upgradeV2(buf, out);
    if (len == sizeof(PersistentConfigV3)) return upgradeV3(buf, out);
    if (len != sizeof(PersistentConfig)) return false;
    PersistentConfig t;
    memcpy(&t, buf, sizeof(t));
    if (t.magic != kConfigMagic || t.version != kConfigVersion || t.size != sizeof(PersistentConfig)) return false;
    if (t.crc != crc32((const uint8_t*)&t, offsetof(PersistentConfig, crc))) return false;
    t.piHost[sizeof(t.piHost) - 1] = '\0';
    t.streamHost[sizeof(t.streamHost) - 1] = '\0';
    t.armName[sizeof(t.armName) - 1] = '\0';
    if (!armNameValid(t.armName)) t.armName[0] = '\0';
    for (int i = 0; i < kNumJoints; ++i) { t.joint[i].dir = kJointDir[i]; t.joint[i].flags |= CAL_DIR; }   // design, not stored
    out = t;
    return true;
}

bool normalizeArmName(const char* in, char* out, size_t cap)
{
    if (!in || !out || cap < kArmNameMax + 1) return false;
    size_t n = 0;
    for (; in[n]; ++n) {
        if (n >= kArmNameMax) { out[0] = '\0'; return false; }
        const char ch = in[n];
        out[n] = (ch >= 'A' && ch <= 'Z') ? (char)(ch - 'A' + 'a') : ch;
    }
    out[n] = '\0';
    if (!armNameValid(out)) { out[0] = '\0'; return false; }
    return true;
}

bool armNameValid(const char* name)
{
    size_t n = 0;
    for (; name[n]; ++n) {
        const char ch = name[n];
        if (n >= kArmNameMax) return false;
        if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-')) return false;
    }
    return n == 0 || (name[0] != '-' && name[n - 1] != '-');
}

void armHostname(const char* name, char* out, size_t cap)
{
    if (!cap) return;
    snprintf(out, cap, "raft-%s", name && name[0] && armNameValid(name) ? name : "leader");
}

void applyConfig(const PersistentConfig& p, ControlConfig& c)
{
    for (int i = 0; i < kNumJoints; ++i) {
        const JointCal& j = p.joint[i];
        c.joint[i].dir = kJointDir[i];
        c.joint[i].ktNmPerA = j.ktNmPerA;
        c.joint[i].frictionNm = j.frictionNm;
        if ((j.flags & CAL_RANGE) && j.qMaxRad - j.qMinRad > 4.0f * kRangeMarginRad) {
            c.joint[i].qMinRad = j.qMinRad + kRangeMarginRad;
            c.joint[i].qMaxRad = j.qMaxRad - kRangeMarginRad;
        }
    }
    c.mAPerUnit = p.mAPerUnit;
    c.rawLimit = (int16_t)fminf(1000.0f, (float)lroundf(kStallCurrentMa / (p.mAPerUnit > 0.5f ? p.mAPerUnit : 3.25f)));
    c.gravityScale = p.gravityScale;
    c.holdFriction = p.holdFriction;
    c.spring = p.spring;
    c.autoRest.enabled = p.autoRest != 0;
    c.autoRest.idleTimeS = p.autoRestIdleS;
    c.frictionScale = p.teleop.frictionScale;
    c.extGain = p.teleop.forceGain;
    c.extSign = p.teleop.forceSign < 0 ? -1.0f : 1.0f;
    c.extDeadbandNm = p.teleop.forceDeadbandNm;
    c.passivity = p.teleop.passivity != 0;
}

bool fullyCalibrated(const PersistentConfig& p)
{
    for (int i = 0; i < kNumJoints; ++i)
        if (!(p.joint[i].flags & CAL_ZERO)) return false;
    return true;
}

}  // namespace raft
