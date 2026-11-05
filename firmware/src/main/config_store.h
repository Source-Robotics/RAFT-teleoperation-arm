// Everything the RAFT keeps across power cycles: calibration, spring, network, behaviour settings.
// One POD blob with a magic, a version and a CRC32; the ESP32 keeps it in NVS (nvs_store.cpp), the PC tests
// in RAM. Older blobs (version 1: before the measured ranges and the teleop settings; 2: before the friction
// compensation; 3: before the arm name) still load. Portable.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "control.h"

namespace raft {

constexpr uint32_t kConfigMagic = 0x52414654;   // "RAFT"
constexpr uint16_t kConfigVersion = 4;
constexpr float kBenchHoldFriction = 0.3f;   // a new arm's friction compensation (holding), see defaultPersistentConfig
constexpr size_t kArmNameMax = 20;           // characters of the arm name (set_name); the hostname is raft-<name>

struct JointCal {
    int16_t zeroTicks;     // raw reading in the CAD zero pose
    int8_t  dir;           // +1 / -1: servo ticks and current vs model +z. Always kJointDir (arm_design.h); kept for the blob layout
    uint8_t flags;         // CAL_ZERO | CAL_DIR | CAL_KT | CAL_RANGE
    float   ktNmPerA;      // output torque per amp (trimmed by the torque calibration)
    float   frictionNm;    // Coulomb friction from the torque calibration (friction compensation, simulator)
    float   qMinRad;       // measured range (calib_range), CAD convention; used when CAL_RANGE is set
    float   qMaxRad;
};
enum CalFlags : uint8_t { CAL_ZERO = 1, CAL_DIR = 2, CAL_KT = 4, CAL_RANGE = 8 };

struct TeleopSettings {
    uint8_t autoReclaim;       // 0/1: after an outage, realign and reclaim without the operator
    uint8_t requireEnable;     // 0/1: the follower moves only while the enable button is held
    uint8_t passivity;         // 0/1: passivity observer on the coupling
    uint8_t barriersAlways;    // 0/1: haptic barriers at RAFT's own range outside teleoperation too
    float   forceGain;         // contact force reflection from t_ext, leader Nm per follower Nm (0 = off)
    float   forceDeadbandNm;
    int8_t  forceSign;         // +1 / -1
    uint8_t gripperOpenRaw;    // gripper position for an open / closed gripper (0..255, commands and g_p)
    uint8_t gripperClosedRaw;
    uint8_t reserved;
    float   triggerOpenRad;    // leader trigger angle for an open / closed gripper
    float   triggerClosedRad;
    float   frictionScale;     // friction compensation 0..1
};

struct PersistentConfig {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    JointCal joint[kNumJoints];
    float    mAPerUnit;          // goal-current register unit
    float    currentRegPerUnit;  // present-current register reading per commanded unit (measured)
    float    gravityScale;
    SpringModel spring;
    uint8_t  autoRest;           // 0/1
    float    autoRestIdleS;
    char     piHost[40];         // PAR6 runtime host for teleoperation ("" = not set)
    char     streamHost[40];     // leader frame destination ("" = the command client's address)
    uint16_t streamPort;
    uint16_t reserved;
    TeleopSettings teleop;
    float    holdFriction;       // friction compensation (holding): extra push against gravity, x the gravity load (v3)
    char     armName[24];        // "" = unnamed (hostname raft-leader); otherwise a-z, 0-9 and '-', see armNameValid (v4)
    uint32_t crc;                // CRC32 of everything before this field
};

void defaultPersistentConfig(PersistentConfig& c);
uint32_t crc32(const uint8_t* data, size_t len);
// Blob I/O. deserialize() rejects a wrong magic, version, size or CRC and then leaves `out` untouched; a valid
// version 1, 2 or 3 blob is upgraded (new fields at their defaults; a version 2 gravity scale above 1 is split into
// gravity 1 and that much friction compensation; a version 3 blob gets no name).
size_t serializeConfig(const PersistentConfig& c, uint8_t* buf, size_t cap);
bool deserializeConfig(const uint8_t* buf, size_t len, PersistentConfig& out);
// Persistent settings -> controller configuration (everything else keeps the controller defaults).
void applyConfig(const PersistentConfig& p, ControlConfig& c);
bool fullyCalibrated(const PersistentConfig& p);    // a zero on every joint (the directions are fixed)
// The arm name, so several arms on one network can be told apart: it becomes the network name raft-<name>(.local)
// and shows in the web UI. Valid: empty, or 1 to kArmNameMax of a-z, 0-9 and '-', not starting or ending with '-'
// (a DNS label). normalizeArmName() lowercases `in` into `out` (cap >= kArmNameMax + 1) and checks it.
bool normalizeArmName(const char* in, char* out, size_t cap);
bool armNameValid(const char* name);
// The network name for a stored name: "raft-<name>", or "raft-leader" while unnamed or invalid.
void armHostname(const char* name, char* out, size_t cap);

class Storage {
public:
    virtual ~Storage() {}
    virtual bool load(PersistentConfig& c) = 0;
    virtual bool save(const PersistentConfig& c) = 0;
};

}  // namespace raft
