#include "rti_link.h"

#include <stdio.h>
#include <string.h>

namespace raft {
namespace rti {

LinkConfig defaultLinkConfig()
{
    LinkConfig c;
    memset(&c, 0, sizeof(c));
    strncpy(c.host, "par6.local", sizeof(c.host) - 1);
    c.rateHz = 50.0f;
    c.stateTimeoutS = 0.1f;
    c.handshakeTimeoutS = 2.0f;
    c.connectTimeoutS = 5.0f;
    c.retryS = 1.0f;
    c.gripperRateHz = 15.0f;
    c.targetMaxAgeS = 0.05f;       // 10 control ticks
    c.targetLostS = 0.3f;
    strncpy(c.client, "raft", sizeof(c.client) - 1);
    strncpy(c.recipe, "raft", sizeof(c.recipe) - 1);
    return c;
}

const char* phaseName(Phase p)
{
    switch (p) {
    case Phase::Idle: return "IDLE";
    case Phase::Connecting: return "CONNECTING";
    case Phase::Pairing: return "PAIRING";
    case Phase::Paired: return "PAIRED";
    case Phase::Claiming: return "CLAIMING";
    case Phase::Active: return "ACTIVE";
    case Phase::Backoff: return "BACKOFF";
    }
    return "?";
}

Link::Link(Transport& t, const LinkConfig& cfg)
    : t_(t), cfg_(cfg), phase_(Phase::Idle), phaseT_(0.0), step_(S_DONE), replyOk_(false), idCounter_(0), connectStarted_(false),
      haveState_(false), lastStateT_(-1e9), stateCount_(0), haveTick_(false), lastTick_(0), staleStates_(0),
      haveTarget_(false), targetT_(-1e9), staleTargets_(0), cmdSeq_(0), lastCmdT_(-1e9), cmdForState_(0),
      gripPending_(false), lastGripT_(-1e9)
{
    pendingId_[0] = '\0';
    err_[0] = '\0';
    memset(&state_, 0, sizeof(state_));
    for (int i = 0; i < 6; ++i) { target_[i] = 0.0; targetVel_[i] = 0.0; }
    for (int i = 0; i < 3; ++i) { grip_[i] = 0; gripSent_[i] = -1000; }
}

bool Link::setConfig(const LinkConfig& cfg)
{
    if (phase_ != Phase::Idle) return false;
    cfg_ = cfg;
    return true;
}

void Link::nextId()
{
    // short ids like the RCB client's (6 chars); "rf" + 4 base-36 digits of a counter
    static const char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    uint32_t n = ++idCounter_;
    char s[7] = "rf0000";
    for (int i = 5; i >= 2; --i) { s[i] = digits[n % 36]; n /= 36; }
    memcpy(pendingId_, s, 7);
}

void Link::enter(Phase p, double now)
{
    phase_ = p;
    phaseT_ = now;
    connectStarted_ = false;
}

void Link::fail(const char* why, double now)
{
    strncpy(err_, why, sizeof(err_) - 1);
    err_[sizeof(err_) - 1] = '\0';
    if (t_.tcpConnected()) {
        char line[160];
        nextId();
        const size_t n = buildNoArgs(line, sizeof(line), pendingId_, "rti_disconnect", cfg_.client);
        if (n) t_.tcpSend(line, n);
    }
    t_.tcpClose();                   // also cancels a connect still in progress
    haveState_ = false;
    haveTick_ = false;               // a restarted runtime counts its ticks from 0 again
    enter(Phase::Backoff, now);
}

void Link::start(double now)
{
    if (phase_ != Phase::Idle) return;
    err_[0] = '\0';
    enter(Phase::Connecting, now);
}

void Link::stop(double now)
{
    if (t_.tcpConnected()) {
        char line[160];
        if (phase_ == Phase::Active || phase_ == Phase::Claiming) {
            nextId();
            const size_t n = buildNoArgs(line, sizeof(line), pendingId_, "rti_release", cfg_.client);
            if (n) t_.tcpSend(line, n);
        }
        nextId();
        const size_t n = buildNoArgs(line, sizeof(line), pendingId_, "rti_disconnect", cfg_.client);
        if (n) t_.tcpSend(line, n);
    }
    t_.tcpClose();
    haveState_ = false;
    haveTick_ = false;
    enter(Phase::Idle, now);
}

bool Link::claim(double now)
{
    if (phase_ != Phase::Paired) return false;
    char line[160];
    nextId();
    const size_t n = buildNoArgs(line, sizeof(line), pendingId_, "rti_claim", cfg_.client);
    if (!n || !t_.tcpSend(line, n)) { fail("send rti_claim", now); return false; }
    replyOk_ = false;
    haveTarget_ = false;          // the first command must come from a fresh leader target
    enter(Phase::Claiming, now);
    return true;
}

void Link::setTarget(const double qDh[6], const double qdDh[6], double now)
{
    for (int i = 0; i < 6; ++i) { target_[i] = qDh[i]; targetVel_[i] = qdDh[i]; }
    haveTarget_ = true;
    targetT_ = now;
}

void Link::setGripper(int position, int speed, int currentmA)
{
    grip_[0] = position; grip_[1] = speed; grip_[2] = currentmA;
    if (position < gripSent_[0] - 1 || position > gripSent_[0] + 1 || speed != gripSent_[1] || currentmA != gripSent_[2])
        gripPending_ = true;
}

void Link::sendStep(double now)
{
    char line[200];
    size_t n = 0;
    nextId();
    replyOk_ = false;
    if (step_ == S_RELOAD) n = buildNoArgs(line, sizeof(line), pendingId_, "reload_recipes", cfg_.client);
    else if (step_ == S_RECIPE) n = buildSetRecipe(line, sizeof(line), pendingId_, cfg_.recipe, cfg_.client);
    else if (step_ == S_CONNECT) n = buildRtiConnect(line, sizeof(line), pendingId_, t_.localIp(), cfg_.rateHz, cfg_.client);
    if (!n || !t_.tcpSend(line, n)) fail("tcp send", now);
    phaseT_ = now;
}

void Link::handleReply(const Reply& r, double now)
{
    if (!r.hasId) {
        if (strcmp(r.code, "FORBIDDEN") == 0) fail("FORBIDDEN: add this IP to RCB_ALLOWED_IPS", now);
        return;                                   // status / error events
    }
    if (strcmp(r.id, pendingId_) != 0) return;    // gripper acks and late replies
    if (r.status == Reply::ERROR) {
        char why[48];
        snprintf(why, sizeof(why), "%s refused: %s", phase_ == Phase::Claiming ? "rti_claim" : "handshake", r.code);
        fail(why, now);
        return;
    }
    if (r.status == Reply::OK || r.status == Reply::COMPLETE) replyOk_ = true;
}

void Link::poll(double now)
{
    // incoming: executor replies, then state packets
    char buf[600];
    if (t_.tcpConnected()) {
        size_t n;
        while ((n = t_.tcpReadLine(buf, sizeof(buf))) > 0) {
            Reply r;
            if (parseReply(buf, n, r)) handleReply(r, now);
            if (phase_ == Phase::Backoff || phase_ == Phase::Idle) return;
        }
    }
    {
        size_t n;
        while ((n = t_.udpRecv(buf, sizeof(buf) - 1)) > 0) {
            State s;
            if (parseState(buf, n, s) && (s.present & F_JP)) {
                if (s.present & F_TICK) {
                    // not newer than the last one: a duplicate or a late packet, not a fresh measurement
                    if (haveTick_ && (int32_t)(s.tick - lastTick_) <= 0) { ++staleStates_; continue; }
                    lastTick_ = s.tick;
                    haveTick_ = true;
                }
                state_ = s;
                haveState_ = true;
                lastStateT_ = now;
                ++stateCount_;
            }
        }
    }

    const double inPhase = now - phaseT_;
    switch (phase_) {
    case Phase::Idle:
        break;
    case Phase::Connecting:
        if (!connectStarted_) {
            if (!t_.tcpConnect(cfg_.host, kPortExecutor)) { fail("tcp connect", now); break; }
            connectStarted_ = true;
        }
        if (!t_.tcpConnected()) {
            if (inPhase > cfg_.connectTimeoutS) fail("tcp connect timeout (host name? executor running?)", now);
            break;
        }
        enter(Phase::Pairing, now);
        step_ = cfg_.recipe[0] ? S_RELOAD : S_CONNECT;
        haveState_ = false;
        haveTick_ = false;
        sendStep(now);
        break;
    case Phase::Pairing:
        if (!t_.tcpConnected()) { fail("executor hung up (not on RCB_ALLOWED_IPS?)", now); break; }
        if (step_ != S_DONE && replyOk_) {
            step_ = (Step)(step_ + 1);
            if (step_ != S_DONE) sendStep(now);
            else phaseT_ = now;
        }
        if (step_ == S_DONE && stateFresh(now)) {
            if ((state_.present & F_MODE) && strcmp(state_.mode, cfg_.rateHz > 0.0f ? "SOFT_RTI" : "RTI") != 0) break;   // not in the mode yet
            if (!(state_.present & F_RTI_STATE)) { fail("recipe lacks rti_state", now); break; }
            enter(Phase::Paired, now);
        } else if (inPhase > cfg_.handshakeTimeoutS) {
            fail(step_ == S_DONE ? "no state stream (UDP 30002)" : "executor reply timeout", now);
        }
        break;
    case Phase::Paired:
        if (!stateFresh(now)) fail("state stream lost", now);
        break;
    case Phase::Claiming:
        if (!stateFresh(now)) { fail("state stream lost", now); break; }
        if (replyOk_ && state_.rtiState == RTI_CONTROL_ACTIVE) { enter(Phase::Active, now); lastCmdT_ = -1e9; }
        else if (inPhase > cfg_.handshakeTimeoutS) fail("claim timeout", now);
        break;
    case Phase::Active:
        if (!stateFresh(now)) { fail("state stream lost", now); break; }
        if (state_.rtiState == RTI_STOPPING_ERROR || state_.rtiState == RTI_UNPAIRED) { fail("robot revoked control", now); break; }
        {
            // Real RTI: PAR6 clocks the stream, one command answers each state packet (it expects one per 4 ms tick);
            // SOFT_RTI: our own schedule at rateHz
            const bool hard = cfg_.rateHz <= 0.0f;
            const bool due = hard ? stateCount_ != cmdForState_ : now - lastCmdT_ >= 1.0 / cfg_.rateHz - 1e-6;
            // the age of the leader's newest target (or of the claim, before the first one)
            const double age = haveTarget_ ? now - targetT_ : inPhase;
            if (age > cfg_.targetLostS) { fail("leader target stale (control loop stalled?)", now); break; }
            if (haveTarget_ && age > cfg_.targetMaxAgeS && due) {
                ++staleTargets_;              // skip this slot; the robot holds its last target meanwhile
                lastCmdT_ = now;
                cmdForState_ = stateCount_;
                break;
            }
            if (haveTarget_ && due) {
                uint8_t pkt[kCmdTsSize];
                const size_t n = packCommand(pkt, sizeof(pkt), ++cmdSeq_, CMD_JOINT_POSITION, target_, targetVel_, true, now);
                t_.udpSend(pkt, n, kPortCommand);
                cmdForState_ = stateCount_;
                if (hard) {
                    lastCmdT_ = now;
                } else {   // a fixed schedule keeps the average rate when poll() runs late; resync after a long gap
                    const double period = 1.0 / cfg_.rateHz;
                    lastCmdT_ = (now - lastCmdT_ > 2.0 * period) ? now : lastCmdT_ + period;
                }
            }
        }
        if (gripPending_ && now - lastGripT_ >= 1.0 / cfg_.gripperRateHz) {
            char line[160];
            char id[16];
            memcpy(id, pendingId_, sizeof(id));        // keep the claim id; gripper acks are ignored
            nextId();
            const size_t n = buildGripperSet(line, sizeof(line), pendingId_, grip_[0], grip_[1], grip_[2], 1, cfg_.client);
            memcpy(pendingId_, id, sizeof(id));
            if (n) t_.tcpSend(line, n);
            for (int i = 0; i < 3; ++i) gripSent_[i] = grip_[i];
            gripPending_ = false;
            lastGripT_ = now;
        }
        break;
    case Phase::Backoff:
        if (inPhase >= cfg_.retryS) enter(Phase::Connecting, now);
        break;
    }
}

}  // namespace rti
}  // namespace raft
