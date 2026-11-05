// Network services on the ESP32-PoE's Ethernet. Everything here runs in the network task.
//   Ethernet    DHCP, hostname raft-leader
//   mDNS        raft-leader.local, service _raft._tcp on the command port
//   OTA         ArduinoOTA (pio run -e main -t upload --upload-port raft-leader.local); torque goes off first
//   commands    TCP 5000, one JSON command per line, one JSON reply per line (docs/teleop_protocol.md)
//   stream      the leader frame over UDP, one per control tick (250 Hz), to the host:port the App names
// The App is reached through callbacks that take the App lock themselves; nothing here blocks while
// holding it. ESP32 / Arduino only.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace raft {
namespace net {

constexpr uint16_t kCommandPort = 5000;
constexpr int kMaxClients = 2;

struct Callbacks {
    // one command line -> one reply line (no '\n'); clientIp = the sender's address
    size_t (*command)(const char* line, char* reply, size_t cap, const char* clientIp);
    // the next leader frame waiting to go out and where it goes; 0 = none (or no stream host)
    size_t (*leaderFrame)(uint8_t* buf, size_t cap, char* host, size_t hostCap, uint16_t* port);
    void (*otaStarting)();      // switch torque off before the flash is rewritten
    void (*networkUp)();        // first time the interface has an address
};

void begin(const char* hostname, const Callbacks& cb);   // starts Ethernet; services start once it has an address
void poll(uint32_t nowMs);                                // network task, every 1-2 ms
bool up();
const char* ip();
uint32_t framesSent();
int clients();
int linkMbps();             // Ethernet link speed, 0 = no link
bool fullDuplex();

}  // namespace net
}  // namespace raft
