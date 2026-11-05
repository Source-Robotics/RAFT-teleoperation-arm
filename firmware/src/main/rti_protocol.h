// RCB runtime real-time interface (RTI), wire formats only. Mirrors, byte for byte:
//   communication/rti_protocol.py   101/109-byte binary command packet
//   docs/udp_telemetry_format.md    text state stream "tag:v,v,...,tag:v"
//   robotclient/connection.py       newline-delimited JSON commands to the executor (TCP 29000)
// Plain C++11, no Arduino dependencies, so it is unit-tested on the PC (tools/sim/test_rti.cpp).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace raft {
namespace rti {

constexpr uint16_t kPortExecutor = 29000;   // TCP, JSON commands
constexpr uint16_t kPortCommand  = 30001;   // UDP, robot receives command packets
constexpr uint16_t kPortState    = 30002;   // UDP, robot sends state packets to us

constexpr uint8_t CMD_JOINT_POSITION = 0x01;
constexpr uint8_t CMD_JOINT_VELOCITY = 0x02;
constexpr uint8_t CMD_JOINT_TORQUE   = 0x03;
constexpr uint8_t CMD_STOP           = 0xFF;

constexpr size_t kCmdSize   = 101;   // '<I B 12d'
constexpr size_t kCmdTsSize = 109;   // + send time (s, sender's monotonic clock)

enum RtiState : int { RTI_UNPAIRED = 0, RTI_CONNECTED = 1, RTI_CONTROL_ACTIVE = 2, RTI_STOPPING_CLEAN = 3, RTI_STOPPING_ERROR = 4 };

// Command packet. Returns bytes written (101 or 109), 0 if buf is too small.
size_t packCommand(uint8_t* buf, size_t cap, uint32_t seq, uint8_t type, const double payload[6],
                   const double feedforward[6], bool withSendTime, double sendTime);
// Inverse, for tests and the simulator's emulated robot. sendTime may be null.
bool unpackCommand(const uint8_t* buf, size_t len, uint32_t* seq, uint8_t* type, double payload[6],
                   double feedforward[6], bool* hasSendTime, double* sendTime);

// One parsed state packet. Only the tags the RAFT uses; others are skipped.
struct State {
    uint32_t present;            // StateField bits
    uint32_t dropped;            // StateField bits of optional tags that came with unusable values (see parseState)
    double   jp[6];              // j_p   joint positions, rad (DH convention)
    double   js[6];              // j_s   joint speeds, rad/s
    double   text[6];            // t_ext external joint torques, Nm: measured motor torque - model torque (robotics/dynamics.py)
    double   gp;                 // g_p   gripper position, RAW 0..255 (0 open, 255 closed) in the gripper's firmware mode.
                                 //       The runtime's tag list calls it metres; it is not (data/update_shared_data.py).
                                 //       Before the gripper reports, the runtime sends the sentinel -99999999.
    double   gc;                 // g_c   gripper current, mA
    int      rtiState;           // rti_state
    uint32_t rtiSeq;             // rti_seq: last accepted command seq
    uint32_t tick;               // tick
    char     mode[20];           // mode string, e.g. "SOFT_RTI"
};
enum StateField : uint32_t {
    F_JP = 1u << 0, F_JS = 1u << 1, F_TEXT = 1u << 2, F_GP = 1u << 3, F_GC = 1u << 4,
    F_RTI_STATE = 1u << 5, F_RTI_SEQ = 1u << 6, F_TICK = 1u << 7, F_MODE = 1u << 8,
};
// Plausible magnitudes. A number outside them, or "nan" / "inf", is unusable.
constexpr double kMaxJointRad = 50.0, kMaxJointRadS = 100.0, kMaxTorqueNm = 1000.0;
constexpr double kMinGripperRaw = -5.0, kMaxGripperRaw = 260.0;
// Returns false for a malformed packet: a known tag with the wrong count or a non-number, or an unusable value
// in j_p, j_s, rti_state, rti_seq or tick. An unusable t_ext, g_p or g_c only drops that tag (its bit goes from
// `present` to `dropped`), so a not-yet-reporting gripper or observer does not cost the joint state.
bool parseState(const char* text, size_t len, State& out);

// Executor command lines, newline-terminated JSON. Return length written, 0 if buf is too small.
// rti_connect: rateHz > 0 selects SOFT_RTI (prefer 50, 25 or 10, which divide the 250 Hz robot loop).
size_t buildRtiConnect(char* buf, size_t cap, const char* id, const char* myIp, float rateHz, const char* client);
size_t buildNoArgs(char* buf, size_t cap, const char* id, const char* cmd, const char* client);   // rti_claim, rti_release, rti_disconnect, reload_recipes
size_t buildSetRecipe(char* buf, size_t cap, const char* id, const char* recipe, const char* client);
size_t buildGripperSet(char* buf, size_t cap, const char* id, int position, int speed, int currentmA, int action, const char* client);

// One executor reply or event line.
struct Reply {
    bool hasId;
    char id[16];
    enum Status : uint8_t { NONE, OK, ERROR, QUEUED, COMPLETE } status;
    char code[24];               // error code, e.g. "BAD_ARGS", "FORBIDDEN"
    char type[12];               // events (no id): "status", "error"
};
bool parseReply(const char* line, size_t len, Reply& out);

}  // namespace rti
}  // namespace raft
