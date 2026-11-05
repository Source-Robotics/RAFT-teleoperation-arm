#include "web_esp.h"

#include <errno.h>
#include <lwip/sockets.h>
#include <string.h>

namespace raft {

EspWebConn::EspWebConn(const WiFiClient& c) : c_(c)
{
    strncpy(ip_, c_.remoteIP().toString().c_str(), sizeof(ip_) - 1);
    ip_[sizeof(ip_) - 1] = '\0';
    c_.setNoDelay(true);
}

int EspWebConn::read(uint8_t* buf, size_t cap)
{
    const int avail = c_.available();
    if (avail > 0) {
        const int n = c_.read(buf, (size_t)avail < cap ? (size_t)avail : cap);
        return n > 0 ? n : 0;
    }
    return c_.connected() ? 0 : -1;
}

int EspWebConn::write(const uint8_t* buf, size_t len)
{
    const int fd = c_.fd();
    if (fd < 0) return -1;
    const int n = ::send(fd, buf, len, MSG_DONTWAIT);
    if (n >= 0) return n;
    return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
}

void EspWebConn::close() { c_.stop(); }

void EspWebListener::begin()
{
    if (started_) return;
    server_.begin();
    server_.setNoDelay(true);
    started_ = true;
}

web::Conn* EspWebListener::accept()
{
    if (!started_) return nullptr;
    WiFiClient c = server_.available();
    if (!c) return nullptr;
    return new EspWebConn(c);
}

}  // namespace raft
