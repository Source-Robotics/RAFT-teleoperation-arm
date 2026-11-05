// The web UI's Handler on top of the App: commands, status, leader frames, and a hello message with the
// kinematic model (from raft_model.h), joint limits, rest pose and spring presets, so the page draws exactly
// the model the firmware uses. Portable: the ESP32 subclass adds the App lock and system figures
// (main.cpp), the simulator adds its hand (tools/sim/web_sim.cpp).
#pragma once

#include <stddef.h>

#include "app.h"
#include "loop_stats.h"
#include "web_server.h"

namespace raft {

// The web UI's health message: one binary WebSocket message per one-second loop window, for the status bar and
// the Health page (docs/web_ui.md). Little-endian, kHealthSize bytes:
//     0  u32  magic "RFTH"              4  u8   version (1)        5  u8  flags (HF_*)      6  u16  size
//     8  u32  loop window seq          12  u32  ticks              16  f32 target Hz       20  f32  Hz
//    24  u32  period min us            28  u32  period max us      32  f32 period mean us  36  f32  jitter rms us
//    40  f32  work mean us             44  u32  work max us        48  f32 part mean us [4: read, lock, app, write]
//    64  u32  part max us [4]          80  u32  late ticks, failed reads (last s), late ticks, failed reads (total)
//    96  f32  CPU load % [2]          104  u32  uptime s          108  u32 heap free, min, size (bytes; 0 = none)
//   120  u32  stack left: control, network, loop (bytes)          132  i16 link Mbit/s    134  u8   web clients
//   135  u8   TCP clients             136  u32  web frames sent  140  u32 UDP frames sent
//   144  u32  missed reads per servo since boot [7]               172  u32 PAR6 old states, stale targets
constexpr size_t kHealthSize = 180;
enum HealthFlags : uint8_t { HF_NET_UP = 1, HF_FULL_DUPLEX = 2, HF_HEAP = 4, HF_STACKS = 8, HF_TCP = 16, HF_UDP = 32 };

// what the platform measures (the ESP32 in main.cpp, web_sim on the PC)
struct HealthFigures {
    LoopReport loop;
    uint32_t uptimeS = 0;
    uint32_t heapFree = 0, heapMin = 0, heapSize = 0;
    uint32_t stackControl = 0, stackNetwork = 0, stackLoop = 0;
    int16_t linkMbps = 0;
    uint8_t flags = 0;               // HF_*
    uint8_t webClients = 0, tcpClients = 0;
    uint32_t webFrames = 0, udpFrames = 0;
};
size_t packHealth(const HealthFigures& f, const uint32_t missed[kNumJoints], uint32_t staleStates, uint32_t staleTargets, uint8_t* buf, size_t cap);

class WebBridge : public web::Handler {
public:
    WebBridge(App& app, const char* firmware, bool sim) : app_(app), firmware_(firmware), sim_(sim) {}
    size_t command(const char* json, char* reply, size_t cap, const char* clientIp) override;
    size_t status(char* out, size_t cap) override;
    size_t frame(uint8_t* buf, size_t cap) override;
    size_t hello(char* out, size_t cap) override;
    uint32_t statusKey() override;
    size_t health(uint8_t* buf, size_t cap) override;

protected:
    virtual void lock() {}
    virtual void unlock() {}
    // commands that must run without the App lock (servo_info waits for the control task); 0 = not handled
    virtual size_t outsideLockCommand(const char* json, char* reply, size_t cap) { (void)json; (void)reply; (void)cap; return 0; }
    // commands the App does not know (the simulator's hand); 0 = not handled. Called with the lock held.
    virtual size_t extraCommand(const char* json, char* reply, size_t cap) { (void)json; (void)reply; (void)cap; return 0; }
    // body of the "sys" object in the status (uptime, heap, loop timing ...); 0 = none. Called without the lock.
    virtual size_t sys(char* out, size_t cap) { (void)out; (void)cap; return 0; }
    // body of the "sys" object in the hello message (what does not change while connected: reset reason, clock,
    // address); 0 = none
    virtual size_t helloSys(char* out, size_t cap) { (void)out; (void)cap; return 0; }
    // the health message's platform figures; false = none (no health messages)
    virtual bool healthFigures(HealthFigures& f) { (void)f; return false; }
    // wall time for App::handleCommand
    virtual double now() { return 0.0; }

    App& app_;
    const char* firmware_;
    bool sim_;
    App::StatusSnapshot snap_;   // the status is printed from this copy, without the lock (and hashed for statusKey)
};

}  // namespace raft
