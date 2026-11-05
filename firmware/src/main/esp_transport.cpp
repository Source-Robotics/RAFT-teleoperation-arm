#include "esp_transport.h"

#include <ESPmDNS.h>
#include <ETH.h>
#include <errno.h>
#include <lwip/sockets.h>
#include <string.h>
#include <strings.h>

namespace raft {

bool EspTransport::begin(uint16_t statePort)
{
    if (!worker_) xTaskCreatePinnedToCore(workerMain, "rti_conn", 4096, this, 1, &worker_, 0);
    if (!udpOk_) udpOk_ = udp_.begin(statePort) == 1;
    return worker_ != nullptr && udpOk_;
}

bool EspTransport::resolve(const char* host, IPAddress& ip)
{
    if (ip.fromString(host)) return true;
    const size_t n = strlen(host);
    if (n > 6 && n < 40 && !strcasecmp(host + n - 6, ".local")) {
        char name[40];
        memcpy(name, host, n - 6);
        name[n - 6] = '\0';
        ip = MDNS.queryHost(name, 2000);
        return ip != IPAddress();
    }
    return WiFi.hostByName(host, ip) == 1 && ip != IPAddress();
}

void EspTransport::workerMain(void* self) { static_cast<EspTransport*>(self)->work(); }

void EspTransport::work()
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (state_ != REQUESTED) continue;
        IPAddress ip;
        const bool named = resolve(host_, ip);
        bool ok = named && !cancel_ && client_.connect(ip, port_, 3000) == 1;
        if (cancel_) {
            client_.stop();
            cancel_ = false;
            state_ = IDLE;
            continue;
        }
        if (ok) {
            client_.setNoDelay(true);
            robotIp_ = ip;
            lineLen_ = 0;
            err_[0] = '\0';
            state_ = CONNECTED;
        } else {
            snprintf(err_, sizeof(err_), named ? "no executor at %s:%u" : "cannot resolve %s", host_, (unsigned)port_);
            state_ = FAILED;
        }
    }
}

bool EspTransport::tcpConnect(const char* host, uint16_t port)
{
    if (!worker_ || state_ == REQUESTED) return false;   // previous attempt still running: the link retries
    if (state_ == CONNECTED) client_.stop();
    strncpy(host_, host, sizeof(host_) - 1);
    host_[sizeof(host_) - 1] = '\0';
    port_ = port;
    cancel_ = false;
    state_ = REQUESTED;
    xTaskNotifyGive(worker_);
    return true;
}

bool EspTransport::tcpConnected()
{
    return state_ == CONNECTED && client_.connected();
}

void EspTransport::tcpClose()
{
    if (state_ == REQUESTED) { cancel_ = true; return; }   // the worker owns the socket until it is done
    if (state_ == CONNECTED) client_.stop();
    state_ = IDLE;
}

bool EspTransport::tcpSend(const char* data, size_t len)
{
    if (!tcpConnected()) return false;
    // Never block: this runs inside App::networkTick under the App lock, and the control task waits for that
    // lock. A send buffer too full for one short line means the executor stopped reading: close, and the
    // link reconnects.
    const int fd = client_.fd();
    const int n = fd >= 0 ? ::send(fd, data, len, MSG_DONTWAIT) : -1;
    if (n == (int)len) return true;
    snprintf(err_, sizeof(err_), "executor stopped reading, closed");
    client_.stop();
    state_ = IDLE;
    return false;
}

size_t EspTransport::tcpReadLine(char* buf, size_t cap)
{
    if (state_ != CONNECTED || cap == 0) return 0;
    while (client_.available() > 0) {
        const int c = client_.read();
        if (c < 0) break;
        if (c == '\n') {
            const size_t n = lineLen_ < cap - 1 ? lineLen_ : cap - 1;
            memcpy(buf, line_, n);
            buf[n] = '\0';
            lineLen_ = 0;
            return n;
        }
        if (c != '\r' && lineLen_ < sizeof(line_) - 1) line_[lineLen_++] = (char)c;
    }
    return 0;
}

bool EspTransport::udpSend(const uint8_t* data, size_t len, uint16_t port)
{
    if (!udpOk_ || robotIp_ == IPAddress()) return false;
    if (!udp_.beginPacket(robotIp_, port)) return false;
    udp_.write(data, len);
    return udp_.endPacket() == 1;
}

size_t EspTransport::udpRecv(char* buf, size_t cap)
{
    if (!udpOk_) return 0;
    for (int guard = 0; guard < 8; ++guard) {
        const int n = udp_.parsePacket();
        if (n <= 0) return 0;
        if (robotIp_ != IPAddress() && udp_.remoteIP() != robotIp_) continue;   // not from the robot: drop
        const int m = udp_.read(reinterpret_cast<uint8_t*>(buf), cap);
        return m > 0 ? (size_t)m : 0;
    }
    return 0;
}

const char* EspTransport::localIp()
{
    strncpy(ip_, ETH.localIP().toString().c_str(), sizeof(ip_) - 1);
    ip_[sizeof(ip_) - 1] = '\0';
    return ip_;
}

}  // namespace raft
