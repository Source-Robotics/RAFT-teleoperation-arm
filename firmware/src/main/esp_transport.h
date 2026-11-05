// rti::Transport on the ESP32-PoE's Ethernet: the PAR6 link's sockets.
//   TCP  executor, port 29000 (newline JSON)
//   UDP  bound to 30002 for the state stream; commands go to the robot's 30001
// Name lookup (".local" via mDNS, other names via DNS) and the TCP connect run in a small worker task, so
// tcpConnect() returns at once and the network task never blocks while it holds the App lock.
// ESP32 / Arduino only; the PC tools use tools/sim/win_transport.h.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "rti_link.h"

namespace raft {

class EspTransport : public rti::Transport {
public:
    // Start the worker task and bind the state port. Call once the network is up.
    bool begin(uint16_t statePort = rti::kPortState);

    bool tcpConnect(const char* host, uint16_t port) override;
    bool tcpConnected() override;
    void tcpClose() override;
    bool tcpSend(const char* data, size_t len) override;
    size_t tcpReadLine(char* buf, size_t cap) override;
    bool udpSend(const uint8_t* data, size_t len, uint16_t port) override;
    size_t udpRecv(char* buf, size_t cap) override;
    const char* localIp() override;

    // last connect problem, for the status line ("" = none)
    const char* lastError() const { return err_; }

private:
    enum State : uint8_t { IDLE = 0, REQUESTED, CONNECTED, FAILED };
    static void workerMain(void* self);
    void work();
    static bool resolve(const char* host, IPAddress& ip);

    WiFiClient client_;
    WiFiUDP udp_;
    IPAddress robotIp_;
    TaskHandle_t worker_ = nullptr;
    volatile uint8_t state_ = IDLE;
    volatile bool cancel_ = false;
    char host_[40] = "";
    uint16_t port_ = 0;
    char line_[700];
    size_t lineLen_ = 0;
    char ip_[16] = "0.0.0.0";
    char err_[48] = "";
    bool udpOk_ = false;
};

}  // namespace raft
