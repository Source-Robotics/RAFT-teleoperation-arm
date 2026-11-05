// RAFT -> PAR6 link: native C++ client of the RCB runtime's real-time interface.
// Handshake on the executor's TCP port (reload_recipes, set_recipe, rti_connect = SOFT_RTI), then the
// state stream arrives on UDP 30002. The link stops at PAIRED: it only claims control when the
// application calls claim(), i.e. once the leader has moved itself onto the follower's pose (the RCB
// runtime checks the start pose). While ACTIVE it streams joint-position targets at rateHz.
// Losing the state stream, or the robot revoking control, drops back to BACKOFF and reconnects up to
// PAIRED again; it never re-claims by itself.
// Freshness: a state packet whose tick is not newer than the last accepted one (a duplicate, or one that
// arrived out of order) is dropped, so a robot that keeps repeating one packet looks silent and times out.
// Every leader target carries the time it was computed: an ACTIVE link sends only targets younger than
// targetMaxAgeS and gives up the claim when none came for targetLostS (a stalled control task must not
// keep the robot moving to its last target).
//
// Sockets are behind Transport: the ESP32 binds it to ETH/WiFiClient/WiFiUDP (P4), the PC test
// harness to an emulated RCB runtime (tools/sim/sim_rcb.h). Plain C++11.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rti_protocol.h"

namespace raft {
namespace rti {

class Transport {
public:
    virtual ~Transport() {}
    // Start connecting (name lookup included); false = could not even start. It may complete at once (PC
    // tests) or later (ESP32, a worker task): tcpConnected() turns true when the connection is up.
    virtual bool tcpConnect(const char* host, uint16_t port) = 0;
    virtual bool tcpConnected() = 0;
    virtual void tcpClose() = 0;
    virtual bool tcpSend(const char* data, size_t len) = 0;
    virtual size_t tcpReadLine(char* buf, size_t cap) = 0;          // one complete line without '\n', 0 if none
    virtual bool udpSend(const uint8_t* data, size_t len, uint16_t port) = 0;   // to the robot
    virtual size_t udpRecv(char* buf, size_t cap) = 0;              // one state datagram, 0 if none
    virtual const char* localIp() = 0;
};

struct LinkConfig {
    char  host[40];            // robot (Pi) address
    float rateHz;              // SOFT_RTI command rate, 5..50 Hz (50, 25, 10 divide PAR6's 250 Hz loop); 0 = Real
                               // (hard) RTI: one command in reply to every state packet, PAR6's 250 Hz, 40 ms watchdog
    float stateTimeoutS;       // no state packet this long -> lost
    float handshakeTimeoutS;   // per handshake step
    float connectTimeoutS;     // TCP connect incl. the name lookup
    float retryS;              // wait in BACKOFF before reconnecting
    float gripperRateHz;       // max gripper_set rate
    float targetMaxAgeS;       // ACTIVE sends a leader target only while it is younger than this ...
    float targetLostS;         // ... and drops the claim when the newest one is older than this
    char  client[16];          // name the executor logs
    char  recipe[24];          // state recipe to select ("" = keep the robot's); see docs/rcb/raft.xml
};
LinkConfig defaultLinkConfig();

enum class Phase : uint8_t { Idle, Connecting, Pairing, Paired, Claiming, Active, Backoff };
const char* phaseName(Phase p);

class Link {
public:
    Link(Transport& t, const LinkConfig& cfg);
    bool setConfig(const LinkConfig& cfg);   // only while IDLE
    const LinkConfig& config() const { return cfg_; }
    void start(double now);            // IDLE -> CONNECTING
    void stop(double now);             // release and disconnect, -> IDLE
    bool claim(double now);            // PAIRED -> CLAIMING; false in any other phase
    void poll(double now);             // call every few ms from the network task

    // latest target, DH convention, computed at time `now` (control task); sent at rateHz (Real RTI: on every state
    // packet) while ACTIVE and fresh
    void setTarget(const double qDh[6], const double qdDh[6], double now);
    // gripper target in the robot's raw units; sent only on change, rate limited
    void setGripper(int position, int speed, int currentmA);

    Phase phase() const { return phase_; }
    bool stateFresh(double now) const { return haveState_ && now - lastStateT_ <= cfg_.stateTimeoutS; }
    const State& state() const { return state_; }
    uint32_t stateCount() const { return stateCount_; }
    uint32_t commandsSent() const { return cmdSeq_; }
    uint32_t staleStates() const { return staleStates_; }       // duplicate / out-of-order state packets dropped
    uint32_t staleTargets() const { return staleTargets_; }     // command slots skipped because the target was old
    const char* lastError() const { return err_; }

private:
    enum Step : uint8_t { S_RELOAD, S_RECIPE, S_CONNECT, S_DONE };
    void enter(Phase p, double now);
    void fail(const char* why, double now);
    void sendStep(double now);
    void nextId();
    void handleReply(const Reply& r, double now);

    Transport& t_;
    LinkConfig cfg_;
    Phase phase_;
    double phaseT_;
    Step step_;
    char  pendingId_[16];
    bool  replyOk_;
    uint32_t idCounter_;
    bool  connectStarted_;
    char  err_[48];

    State state_;
    bool  haveState_;
    double lastStateT_;
    uint32_t stateCount_;
    bool  haveTick_;
    uint32_t lastTick_;
    uint32_t staleStates_;

    double target_[6], targetVel_[6];
    bool  haveTarget_;
    double targetT_;
    uint32_t staleTargets_;
    uint32_t cmdSeq_;
    double lastCmdT_;
    uint32_t cmdForState_;   // Real RTI: the state packet the last command answered

    int   grip_[3];
    int   gripSent_[3];
    bool  gripPending_;
    double lastGripT_;
};

}  // namespace rti
}  // namespace raft
