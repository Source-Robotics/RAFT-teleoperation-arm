// PAR6 link probe (dev tool, Windows). Runs the firmware's rti::Link, the same code the ESP32 runs, over
// Winsock against the RCB runtime on the Pi (or fake_rcb_server.exe), to check pairing and, optionally,
// a small claimed motion before the leader arm itself is wired up. Procedure: docs/rcb_link_test.md.
//   rti_probe <host> [options]
//     --claim          claim control once PAIRED and move one joint in a slow sine. THIS MOVES THE ROBOT.
//     --joint N        joint for the sine, 1..6 (default 6, the last wrist joint)
//     --amp DEG        sine amplitude, at most 10 (default 2); 0.2 Hz, faded in and out over 2 s
//     --seconds S      how long to stay PAIRED (or ACTIVE with --claim), default 10
//     --rate HZ        SOFT_RTI command rate 50 | 25 | 10 (default 50)
//     --recipe NAME    state recipe to select (default raft, docs/rcb/raft.xml; "-" keeps the robot's)
//     --bind IP        local address for the state port 30002 (default 0.0.0.0; 127.0.0.1 for loopback)
// Exit code 0 when the link reached PAIRED (and ACTIVE with --claim) and ended cleanly.
#define NOMINMAX
#include "win_transport.h"

#include <mmsystem.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "rti_link.h"

using raft::rti::Link;
using raft::rti::Phase;

namespace {

const double kPi = 3.14159265358979323846;

double seconds()
{
    static LARGE_INTEGER f = {}, t0 = {};
    if (!f.QuadPart) { QueryPerformanceFrequency(&f); QueryPerformanceCounter(&t0); }
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return (double)(t.QuadPart - t0.QuadPart) / (double)f.QuadPart;
}

int usage()
{
    std::printf("usage: rti_probe <host> [--claim] [--joint N] [--amp DEG] [--seconds S] [--rate HZ] [--recipe NAME|-] [--bind IP]\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argv[1][0] == '-') return usage();
    const char* host = argv[1];
    bool claim = false;
    int joint = 6;
    double ampDeg = 2.0, runS = 10.0, rate = 50.0;
    const char* recipe = "raft";
    const char* bindIp = "0.0.0.0";
    for (int i = 2; i < argc; ++i) {
        const bool more = i + 1 < argc;
        if (!std::strcmp(argv[i], "--claim")) claim = true;
        else if (!std::strcmp(argv[i], "--joint") && more) joint = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--amp") && more) ampDeg = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--seconds") && more) runS = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--rate") && more) rate = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--recipe") && more) recipe = argv[++i];
        else if (!std::strcmp(argv[i], "--bind") && more) bindIp = argv[++i];
        else return usage();
    }
    if (joint < 1 || joint > 6 || ampDeg < 0 || ampDeg > 10 || runS <= 0) return usage();

    std::setvbuf(stdout, nullptr, _IONBF, 0);      // live output, also when redirected
    timeBeginPeriod(1);
    wintr::WsaInit wsa;
    wintr::WinTransport tr(bindIp);
    if (!wsa.ok || !tr.ok()) { std::printf("socket setup failed: %s\n", tr.error()); return 2; }

    raft::rti::LinkConfig lc = raft::rti::defaultLinkConfig();
    std::memset(lc.host, 0, sizeof(lc.host));
    std::strncpy(lc.host, host, sizeof(lc.host) - 1);
    lc.rateHz = (float)rate;
    std::memset(lc.recipe, 0, sizeof(lc.recipe));
    if (std::strcmp(recipe, "-")) std::strncpy(lc.recipe, recipe, sizeof(lc.recipe) - 1);
    std::strncpy(lc.client, "raft-probe", sizeof(lc.client) - 1);
    Link link(tr, lc);

    std::printf("rti_probe -> %s:29000, SOFT_RTI %.0f Hz, recipe %s, %s\n", host, rate, lc.recipe[0] ? lc.recipe : "(robot's)",
                claim ? "WILL CLAIM AND MOVE" : "read-only (no claim)");
    if (claim) std::printf("  sine on joint %d, %.1f deg, 0.2 Hz, for %.0f s\n", joint, ampDeg, runS);

    link.start(seconds());
    Phase last = Phase::Idle;
    bool paired = false, active = false, claimed = false, lost = false;
    double pairedAt = -1, activeAt = -1, lastPrint = 0;
    double q0[6] = {0};
    uint32_t lastStates = 0;
    const int j = joint - 1;
    const double amp = ampDeg * kPi / 180.0, w = 2.0 * kPi * 0.2;

    for (;;) {
        const double t = seconds();
        link.poll(t);
        const Phase ph = link.phase();
        if (ph != last) {
            std::printf("%8.3f s  %-10s -> %-10s %s%s\n", t, raft::rti::phaseName(last), raft::rti::phaseName(ph),
                        ph == Phase::Backoff ? "error: " : "", ph == Phase::Backoff ? link.lastError() : "");
            last = ph;
        }
        if (ph == Phase::Paired && !paired) { paired = true; pairedAt = t; }
        if (claim && ph == Phase::Paired && !claimed && t - pairedAt > 1.0) {
            const raft::rti::State& s = link.state();
            for (int i = 0; i < 6; ++i) q0[i] = s.jp[i];
            const double zero[6] = {0};
            link.setTarget(q0, zero, t);                    // first command = the measured pose (start-pose check)
            link.claim(t);
            claimed = true;
        }
        if (ph == Phase::Active) {
            if (!active) { active = true; activeAt = t; }
            const double u = t - activeAt;
            const double env = std::fmin(1.0, std::fmin(u / 2.0, (runS - u) / 2.0));
            const double e = env > 0 ? env : 0.0;
            double q[6], qd[6];
            for (int i = 0; i < 6; ++i) { q[i] = q0[i]; qd[i] = 0.0; }
            q[j] += e * amp * std::sin(w * u);
            qd[j] = e * amp * w * std::cos(w * u);
            link.setTarget(q, qd, t);
        }
        if (claimed && active && ph == Phase::Backoff) { lost = true; std::printf("control lost: %s\n", link.lastError()); break; }
        if (claim ? (active && t - activeAt >= runS) : (paired && t - pairedAt >= runS)) break;
        if (!paired && t > 20.0) { std::printf("did not pair within 20 s (last error: %s)\n", link.lastError()); break; }
        if (claimed && !active && t - pairedAt > 10.0) { std::printf("claim did not become ACTIVE (last error: %s)\n", link.lastError()); break; }

        if (t - lastPrint >= 1.0) {
            const raft::rti::State& s = link.state();
            const uint32_t n = link.stateCount();
            std::printf("%8.3f s  %-8s state %3u/s  mode %-9s rti_state %d  seq %5u  sent %5u  j_p deg %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
                        t, raft::rti::phaseName(ph), (unsigned)(n - lastStates), s.mode, s.rtiState, (unsigned)s.rtiSeq,
                        (unsigned)link.commandsSent(), s.jp[0] * 180 / kPi, s.jp[1] * 180 / kPi, s.jp[2] * 180 / kPi,
                        s.jp[3] * 180 / kPi, s.jp[4] * 180 / kPi, s.jp[5] * 180 / kPi);
            lastStates = n;
            lastPrint = t;
        }
        wintr::napMs();
    }
    link.stop(seconds());
    Sleep(100);                                          // let release / disconnect leave
    const bool ok = paired && (!claim || (active && !lost));
    std::printf("summary: paired %s, active %s, %u state packets, %u commands sent -> %s\n", paired ? "yes" : "no",
                active ? "yes" : "no", (unsigned)link.stateCount(), (unsigned)link.commandsSent(), ok ? "OK" : "FAILED");
    timeEndPeriod(1);
    return ok ? 0 : 1;
}
