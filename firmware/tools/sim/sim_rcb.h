// Emulated RCB runtime for PC tests and the bilateral simulator (dev tool, not part of the arm).
// It plays the robot side of the real-time interface the way the PAR6 runtime does it:
//   executor TCP: reload_recipes, set_recipe, rti_connect (SOFT_RTI), rti_claim, rti_release,
//                 rti_disconnect, gripper_set; access-list refusal ({"code": "FORBIDDEN"}, no id)
//   state stream: text recipe "raft" (j_p, j_s, t_ext, g_p, g_c, tick, rti_seq, rti_state, mode)
//                 at the robot loop rate (250 Hz) to the client's UDP port; g_p on the gripper's raw
//                 0..255 scale like the real runtime in firmware mode (its tag list says metres, it is not)
//   commands:     109-byte joint-position packets; SOFT_RTI interpolation (linear over one command
//                 interval), start-pose check on the first packet after a claim, command watchdog
//                 (3 command intervals) -> rti_state 4
// The wire adds one-way latency, jitter and UDP loss. It implements raft::rti::Transport, so the
// firmware's rti::Link talks to it unchanged.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <random>
#include <string>

#include "rti_link.h"
#include "rti_protocol.h"

namespace simrcb {

struct NetConfig {
    double latencyS = 0.003;     // one way
    double jitterS = 0.0005;
    double udpLoss = 0.0;        // probability per datagram
    unsigned seed = 7;
};

class FakeRcb : public raft::rti::Transport {
public:
    explicit FakeRcb(const NetConfig& n) : net_(n), rng_(n.seed) {}

    // knobs
    bool forbidden = false;
    std::string recipe = "raft";
    bool streamMuted = false;           // robot keeps running but its state packets vanish
    double startPoseTolRad = 3.0 * 3.14159265358979 / 180.0;
    // fault injection
    bool repeatPackets = false;         // every state packet goes out twice (a duplicating network)
    bool freezeState = false;           // the robot keeps sending its last packet, same tick (a stuck runtime)
    std::string textField;              // non-empty: sent verbatim as t_ext's six values (e.g. "nan,0,0,0,0,0")
    std::string gpField;                // non-empty: sent verbatim as g_p (e.g. the runtime's "-99999999")

    void setTime(double t) { now_ = t; }
    double time() const { return now_; }

    // Robot loop: call at 250 Hz with the follower's measurement (DH convention). gpRaw is the gripper's
    // firmware-mode position, 0 open .. 255 closed, as the real runtime reports g_p.
    void robotTick(const double qDh[6], const double qdDh[6], const double tExt[6], double gpRaw, double gcMA)
    {
        for (int i = 0; i < 6; ++i) { meas_[i] = qDh[i]; measVel_[i] = qdDh[i]; text_[i] = tExt[i]; }
        gp_ = gpRaw; gc_ = gcMA;
        haveMeas_ = true;
        processInbox();
        if (rtiState_ == 2 && cmdCount_ > 0 && now_ - lastCmdArrival_ > 3.0 / rate_) {
            rtiState_ = 4;                               // command watchdog
        }
        if (streaming_ && !streamMuted) {
            if (!freezeState || lastPacket_.empty()) {
                char te[160], gp[40];
                if (textField.empty()) std::snprintf(te, sizeof(te), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", text_[0], text_[1], text_[2], text_[3], text_[4], text_[5]);
                else std::snprintf(te, sizeof(te), "%s", textField.c_str());
                if (gpField.empty()) std::snprintf(gp, sizeof(gp), "%.1f", gp_);
                else std::snprintf(gp, sizeof(gp), "%s", gpField.c_str());
                char buf[700];
                int n = std::snprintf(buf, sizeof(buf),
                    "j_p:%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,j_s:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,t_ext:%s,"
                    "g_p:%s,g_c:%.1f,tick:%u,rti_seq:%u,rti_state:%d,mode:%s",
                    meas_[0], meas_[1], meas_[2], meas_[3], meas_[4], meas_[5],
                    measVel_[0], measVel_[1], measVel_[2], measVel_[3], measVel_[4], measVel_[5],
                    te, gp, gc_, tick_, lastSeq_, rtiState_, mode_.c_str());
                if (n > 0) lastPacket_.assign(buf, (size_t)n);
            }
            sendUdpToLeader(lastPacket_);
            if (repeatPackets) sendUdpToLeader(lastPacket_);
        }
        ++tick_;
    }

    // SOFT_RTI target for the robot's own controller (DH). false = not under control (hold).
    bool target(double qDh[6], double qdDh[6]) const
    {
        if (rtiState_ != 2 || cmdCount_ == 0) return false;
        const double interval = 1.0 / rate_;
        if (cmdCount_ == 1) { for (int i = 0; i < 6; ++i) { qDh[i] = last_[i]; qdDh[i] = 0.0; } return true; }
        double a = (now_ - lastCmdArrival_) / interval;
        a = a < 0 ? 0 : (a > 1 ? 1 : a);
        for (int i = 0; i < 6; ++i) {
            qDh[i] = prev_[i] + a * (last_[i] - prev_[i]);
            qdDh[i] = (a < 1.0) ? (last_[i] - prev_[i]) / interval : 0.0;
        }
        return true;
    }

    int gripperRaw() const { return grip_; }
    int gripperCommands() const { return gripCmds_; }
    int rtiState() const { return rtiState_; }
    const std::string& mode() const { return mode_; }
    unsigned commandsReceived() const { return cmdCount_; }
    const std::string& lastEvent() const { return lastEvent_; }

    // ---- raft::rti::Transport: the leader's end of the wire ----
    bool tcpConnect(const char*, uint16_t port) override
    {
        if (port != raft::rti::kPortExecutor) return false;
        connected_ = true;
        closeAfterRead_ = false;
        toLeaderTcp_.clear();
        if (forbidden) {
            push(toLeaderTcp_, "{\"type\": \"error\", \"code\": \"FORBIDDEN\", \"message\": \"192.168.1.50 is not on the access list\"}");
            closeAfterRead_ = true;
        }
        return true;
    }
    bool tcpConnected() override { return connected_; }
    void tcpClose() override
    {
        connected_ = false;
        toLeaderTcp_.clear();
        // the executor drops the RTI pairing when its client goes away
        streaming_ = false; mode_ = "IDLE"; rtiState_ = 0;
    }
    bool tcpSend(const char* data, size_t len) override
    {
        if (!connected_) return false;
        std::string s(data, len);
        size_t p = 0, q;
        while ((q = s.find('\n', p)) != std::string::npos) { push(toRobotTcp_, s.substr(p, q - p)); p = q + 1; }
        return true;
    }
    size_t tcpReadLine(char* buf, size_t cap) override
    {
        if (toLeaderTcp_.empty() || toLeaderTcp_.front().at > now_) return 0;
        const std::string s = toLeaderTcp_.front().data;
        toLeaderTcp_.pop_front();
        if (closeAfterRead_ && toLeaderTcp_.empty()) connected_ = false;
        const size_t n = s.size() < cap - 1 ? s.size() : cap - 1;
        std::memcpy(buf, s.data(), n);
        buf[n] = '\0';
        return n;
    }
    bool udpSend(const uint8_t* data, size_t len, uint16_t port) override
    {
        if (port != raft::rti::kPortCommand || lost()) return true;
        push(toRobotUdp_, std::string((const char*)data, len));
        return true;
    }
    size_t udpRecv(char* buf, size_t cap) override
    {
        if (toLeaderUdp_.empty() || toLeaderUdp_.front().at > now_) return 0;
        const std::string s = toLeaderUdp_.front().data;
        toLeaderUdp_.pop_front();
        const size_t n = s.size() < cap ? s.size() : cap;
        std::memcpy(buf, s.data(), n);
        return n;
    }
    const char* localIp() override { return "192.168.1.50"; }

private:
    struct Msg { double at; std::string data; };

    double delay()
    {
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        return std::fmax(0.0, net_.latencyS + net_.jitterS * u(rng_));
    }
    bool lost()
    {
        std::uniform_real_distribution<double> u(0.0, 1.0);
        return net_.udpLoss > 0 && u(rng_) < net_.udpLoss;
    }
    void push(std::deque<Msg>& q, const std::string& s)
    {
        // keep order: a message never overtakes the previous one on the same stream
        double at = now_ + delay();
        if (!q.empty() && at < q.back().at) at = q.back().at;
        q.push_back({at, s});
    }
    void sendUdpToLeader(const std::string& s) { if (!lost()) push(toLeaderUdp_, s); }

    static std::string field(const std::string& s, const char* key)
    {
        const std::string pat = std::string("\"") + key + "\": \"";
        const size_t p = s.find(pat);
        if (p == std::string::npos) return "";
        const size_t b = p + pat.size(), e = s.find('"', b);
        return e == std::string::npos ? "" : s.substr(b, e - b);
    }
    static std::string args(const std::string& s)
    {
        const size_t b = s.find('['), e = s.find(']');
        return (b == std::string::npos || e == std::string::npos) ? "" : s.substr(b + 1, e - b - 1);
    }
    void reply(const std::string& id, bool ok, const char* code = "")
    {
        char buf[200];
        if (ok) std::snprintf(buf, sizeof(buf), "{\"id\": \"%s\", \"status\": \"ok\", \"data\": null}", id.c_str());
        else std::snprintf(buf, sizeof(buf), "{\"id\": \"%s\", \"status\": \"error\", \"code\": \"%s\", \"message\": \"refused\"}", id.c_str(), code);
        push(toLeaderTcp_, buf);
    }

    void processInbox()
    {
        while (connected_ && !toRobotTcp_.empty() && toRobotTcp_.front().at <= now_) {
            const std::string line = toRobotTcp_.front().data;
            toRobotTcp_.pop_front();
            if (forbidden) continue;
            const std::string id = field(line, "id"), cmd = field(line, "cmd"), a = args(line);
            if (cmd == "reload_recipes") reply(id, true);
            else if (cmd == "set_recipe") reply(id, a == "\"" + recipe + "\"", "BAD_ARGS");
            else if (cmd == "rti_connect") {
                double r = 0.0;
                const size_t c1 = a.find(',');
                if (c1 != std::string::npos) r = std::atof(a.c_str() + c1 + 1);
                rate_ = r > 0 ? r : 250.0;
                mode_ = r > 0 ? "SOFT_RTI" : "RTI";
                rtiState_ = 1; streaming_ = true; reply(id, true);
            } else if (cmd == "rti_claim") {
                if (rtiState_ == 1) { rtiState_ = 2; cmdCount_ = 0; reply(id, true); }
                else reply(id, false, "NOT_PAIRED");
            } else if (cmd == "rti_release") { if (rtiState_ == 2 || rtiState_ == 4) rtiState_ = 1; reply(id, true); }
            else if (cmd == "rti_disconnect") { rtiState_ = 0; streaming_ = false; mode_ = "IDLE"; reply(id, true); }
            else if (cmd == "gripper_set") { grip_ = std::atoi(a.c_str()); ++gripCmds_; reply(id, true); }
            else reply(id, false, "UNKNOWN_COMMAND");
        }
        while (!toRobotUdp_.empty() && toRobotUdp_.front().at <= now_) {
            const std::string pkt = toRobotUdp_.front().data;
            toRobotUdp_.pop_front();
            if (rtiState_ != 2) continue;
            uint32_t seq; uint8_t type; double q[6], ff[6]; bool hasT; double t;
            if (!raft::rti::unpackCommand((const uint8_t*)pkt.data(), pkt.size(), &seq, &type, q, ff, &hasT, &t)) continue;
            if (type != raft::rti::CMD_JOINT_POSITION || (cmdCount_ > 0 && seq <= lastSeq_)) continue;
            if (cmdCount_ == 0 && haveMeas_) {           // start-pose check
                for (int i = 0; i < 6; ++i)
                    if (std::fabs(q[i] - meas_[i]) > startPoseTolRad) { rtiState_ = 4; lastEvent_ = "START_POSE"; break; }
                if (rtiState_ != 2) continue;
                for (int i = 0; i < 6; ++i) prev_[i] = meas_[i];
            } else {
                for (int i = 0; i < 6; ++i) prev_[i] = last_[i];
            }
            for (int i = 0; i < 6; ++i) last_[i] = q[i];
            lastSeq_ = seq;
            lastCmdArrival_ = now_;
            ++cmdCount_;
        }
    }

    NetConfig net_;
    std::mt19937 rng_;
    double now_ = 0.0;
    bool connected_ = false, closeAfterRead_ = false, streaming_ = false, haveMeas_ = false;
    std::string mode_ = "IDLE";
    int rtiState_ = 0;
    double rate_ = 50.0;
    unsigned tick_ = 0, lastSeq_ = 0, cmdCount_ = 0;
    double lastCmdArrival_ = -1e9;
    double meas_[6] = {0}, measVel_[6] = {0}, text_[6] = {0}, gp_ = 0, gc_ = 0;
    double prev_[6] = {0}, last_[6] = {0};
    int grip_ = 0, gripCmds_ = 0;
    std::string lastEvent_, lastPacket_;
    std::deque<Msg> toRobotTcp_, toLeaderTcp_, toRobotUdp_, toLeaderUdp_;
};

}  // namespace simrcb
