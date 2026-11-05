// Emulated RCB runtime on real sockets (dev tool, Windows): the robot side of the PAR6 real-time
// interface, for a loopback test of rti_probe.exe, or of the ESP32 firmware pointed at this PC. The protocol
// logic is FakeRcb (tools/sim/sim_rcb.h); this file only carries its messages over TCP 29000, UDP 30001
// (commands in) and UDP 30002 (state out), and adds a follower that tracks the commanded joint positions.
//   fake_rcb_server [--bind IP] [--seconds S] [--forbidden] [--recipe NAME]
//     --bind IP      listen address (default 127.0.0.1; 0.0.0.0 to serve the ESP32 over the LAN)
//     --seconds S    stop after S seconds (default: run until Ctrl+C)
//     --forbidden    refuse the client like an access-list miss (FORBIDDEN)
#define NOMINMAX
#include "win_transport.h"

#include <mmsystem.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <condition_variable>
#include <cstdarg>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "sim_rcb.h"

namespace {

const double kPi = 3.14159265358979323846;

// Output from the robot loop goes through this background thread: a blocked stdout (a pipe nobody reads)
// must not stall the 250 Hz loop, or the client sees its state stream stop.
class Logger {
public:
    Logger() : th_([this] { run(); }) {}
    ~Logger()
    {
        { std::lock_guard<std::mutex> l(m_); done_ = true; }
        cv_.notify_one();
        th_.join();
    }
    void operator()(const char* fmt, ...)
    {
        char b[400];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        { std::lock_guard<std::mutex> l(m_); q_.push_back(b); }
        cv_.notify_one();
    }

private:
    void run()
    {
        std::unique_lock<std::mutex> l(m_);
        for (;;) {
            cv_.wait(l, [this] { return done_ || !q_.empty(); });
            while (!q_.empty()) {
                const std::string s = q_.front();
                q_.pop_front();
                l.unlock();
                std::fputs(s.c_str(), stdout);
                std::fflush(stdout);
                l.lock();
            }
            if (done_) return;
        }
    }
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<std::string> q_;
    bool done_ = false;
    std::thread th_;
};

double seconds()
{
    static LARGE_INTEGER f = {}, t0 = {};
    if (!f.QuadPart) { QueryPerformanceFrequency(&f); QueryPerformanceCounter(&t0); }
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return (double)(t.QuadPart - t0.QuadPart) / (double)f.QuadPart;
}

SOCKET bound(int type, const char* ip, uint16_t port)
{
    SOCKET s = socket(AF_INET, type, type == SOCK_STREAM ? IPPROTO_TCP : IPPROTO_UDP);
    if (s == INVALID_SOCKET) return s;
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    inet_pton(AF_INET, ip, &a.sin_addr);
    if (bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) { closesocket(s); return INVALID_SOCKET; }
    wintr::setNonBlocking(s);
    return s;
}

}  // namespace

int main(int argc, char** argv)
{
    const char* bindIp = "127.0.0.1";
    double runS = 0.0;
    simrcb::NetConfig net;
    net.latencyS = 0.0; net.jitterS = 0.0; net.udpLoss = 0.0;     // the real network adds its own
    simrcb::FakeRcb rcb(net);
    for (int i = 1; i < argc; ++i) {
        const bool more = i + 1 < argc;
        if (!std::strcmp(argv[i], "--bind") && more) bindIp = argv[++i];
        else if (!std::strcmp(argv[i], "--seconds") && more) runS = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--forbidden")) rcb.forbidden = true;
        else if (!std::strcmp(argv[i], "--recipe") && more) rcb.recipe = argv[++i];
        else { std::printf("usage: fake_rcb_server [--bind IP] [--seconds S] [--forbidden] [--recipe NAME]\n"); return 2; }
    }

    std::setvbuf(stdout, nullptr, _IONBF, 0);      // live output, also when redirected
    timeBeginPeriod(1);
    wintr::WsaInit wsa;
    SOCKET lis = bound(SOCK_STREAM, bindIp, raft::rti::kPortExecutor);
    SOCKET udp = bound(SOCK_DGRAM, bindIp, raft::rti::kPortCommand);
    if (!wsa.ok || lis == INVALID_SOCKET || udp == INVALID_SOCKET || listen(lis, 1) != 0) {
        std::printf("cannot bind %s:29000 / :30001 (in use?)\n", bindIp);
        return 2;
    }
    std::printf("fake RCB runtime on %s: executor TCP 29000, commands UDP 30001, state to the client's UDP 30002%s\n",
                bindIp, rcb.forbidden ? " (refusing clients)" : "");

    Logger say;
    SOCKET cli = INVALID_SOCKET;
    sockaddr_in cliAddr = {};
    std::string rx;
    // follower: DH pose of the CAD standby pose (q_DH = q_CAD + [0, -90, 180, 0, 0, 180] deg)
    double q[6] = {0.0, -kPi / 2, kPi, 0.0, 0.0, kPi}, qd[6] = {0}, text[6] = {0};
    double nextTick = 0.0, lastPrint = 0.0;
    unsigned lastCmds = 0;
    for (;;) {
        const double t = seconds();
        if (runS > 0 && t > runS) break;
        rcb.setTime(t);

        if (cli == INVALID_SOCKET) {
            int al = sizeof(cliAddr);
            cli = accept(lis, reinterpret_cast<sockaddr*>(&cliAddr), &al);
            if (cli != INVALID_SOCKET) {
                wintr::setNonBlocking(cli);
                BOOL nd = TRUE;
                setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
                rx.clear();
                rcb.tcpConnect("leader", raft::rti::kPortExecutor);
                say("%8.3f s  client %s connected\n", t, wintr::ipString(cliAddr).c_str());
            }
        }
        if (cli != INVALID_SOCKET) {
            char buf[2048];
            bool closed = false;
            for (;;) {
                const int n = recv(cli, buf, sizeof(buf), 0);
                if (n > 0) { rx.append(buf, (size_t)n); continue; }
                if (n == 0 || WSAGetLastError() != WSAEWOULDBLOCK) closed = true;
                break;
            }
            size_t nl;
            while ((nl = rx.find('\n')) != std::string::npos) {
                const std::string line = rx.substr(0, nl + 1);
                rx.erase(0, nl + 1);
                rcb.tcpSend(line.data(), line.size());
            }
            if (closed) {
                closesocket(cli); cli = INVALID_SOCKET;
                rcb.tcpClose();
                say("%8.3f s  client disconnected\n", t);
            }
        }
        {
            char buf[512];
            sockaddr_in from = {};
            int fl = sizeof(from);
            int n;
            while ((n = recvfrom(udp, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &fl)) > 0)
                rcb.udpSend(reinterpret_cast<const uint8_t*>(buf), (size_t)n, raft::rti::kPortCommand);
        }
        // the robot loop at 250 Hz: follow the SOFT_RTI target with a 30 ms lag, then publish the state
        while (t >= nextTick) {
            const double dt = 0.004;
            double tg[6], tgv[6];
            if (rcb.target(tg, tgv)) {
                const double a = 1.0 - std::exp(-dt / 0.03);
                for (int i = 0; i < 6; ++i) { const double qn = q[i] + a * (tg[i] - q[i]); qd[i] = (qn - q[i]) / dt; q[i] = qn; }
            } else {
                for (double& v : qd) v = 0.0;
            }
            rcb.robotTick(q, qd, text, 0.0, 0.0);
            nextTick += dt;
        }
        {
            char buf[700];
            size_t n;
            while ((n = rcb.tcpReadLine(buf, sizeof(buf))) > 0) {
                if (cli == INVALID_SOCKET) continue;
                buf[n] = '\n';
                send(cli, buf, (int)n + 1, 0);
            }
            if (cli != INVALID_SOCKET && !rcb.tcpConnected()) {      // FORBIDDEN: the executor hangs up
                // graceful close: send FIN, read what the client already sent, then close. Closing with
                // unread input makes Windows send RST, and the client may lose the FORBIDDEN line.
                shutdown(cli, SD_SEND);
                const double t1 = seconds();
                char junk[512];
                while (seconds() - t1 < 0.2) {
                    const int got = recv(cli, junk, sizeof(junk), 0);
                    if (got == 0 || (got < 0 && WSAGetLastError() != WSAEWOULDBLOCK)) break;
                    if (got < 0) Sleep(1);
                }
                closesocket(cli); cli = INVALID_SOCKET;
                say("%8.3f s  client refused and disconnected\n", t);
            }
            sockaddr_in to = cliAddr;
            to.sin_port = htons(raft::rti::kPortState);
            while ((n = rcb.udpRecv(buf, sizeof(buf))) > 0)
                if (cliAddr.sin_family == AF_INET) sendto(udp, buf, (int)n, 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
        }
        if (t - lastPrint >= 2.0) {
            say("%8.3f s  mode %-9s rti_state %d  commands %5u (+%u)  follower deg %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f%s%s\n", t,
                        rcb.mode().c_str(), rcb.rtiState(), rcb.commandsReceived(), rcb.commandsReceived() - lastCmds,
                        q[0] * 180 / kPi, q[1] * 180 / kPi, q[2] * 180 / kPi, q[3] * 180 / kPi, q[4] * 180 / kPi, q[5] * 180 / kPi,
                        rcb.lastEvent().empty() ? "" : "  event ", rcb.lastEvent().c_str());
            lastCmds = rcb.commandsReceived();
            lastPrint = t;
        }
        wintr::napMs();
    }
    if (cli != INVALID_SOCKET) closesocket(cli);
    closesocket(udp);
    closesocket(lis);
    timeEndPeriod(1);
    return 0;
}
