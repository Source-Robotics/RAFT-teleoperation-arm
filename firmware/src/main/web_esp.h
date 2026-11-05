// ESP32 bindings of the web UI server (web_server.h): WiFiServer on port 80 over the Ethernet interface, and
// connections whose writes never block (lwIP send with MSG_DONTWAIT), so a slow browser cannot stall the
// network task that also runs the PAR6 link. ESP32 / Arduino only.
#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "web_server.h"

namespace raft {

class EspWebConn : public web::Conn {
public:
    explicit EspWebConn(const WiFiClient& c);
    int read(uint8_t* buf, size_t cap) override;
    int write(const uint8_t* buf, size_t len) override;
    void close() override;
    const char* remoteIp() const override { return ip_; }

private:
    WiFiClient c_;
    char ip_[16];
};

class EspWebListener : public web::Listener {
public:
    explicit EspWebListener(uint16_t port = 80) : server_(port) {}
    void begin();
    web::Conn* accept() override;

private:
    WiFiServer server_;
    bool started_ = false;
};

}  // namespace raft
