// Winsock bindings of the web UI server's Conn / Listener (dev tool, Windows only). Non-blocking sockets,
// like the ESP32 binding (src/main/web_esp.cpp). Header-only; link with ws2_32.lib.
#pragma once

#include "win_transport.h"     // Winsock includes, WsaInit, setNonBlocking
#include "web_server.h"

namespace wintr {

class WinConn : public raft::web::Conn {
public:
    WinConn(SOCKET s, const sockaddr_in& peer) : s_(s) { std::strncpy(ip_, ipString(peer).c_str(), sizeof(ip_) - 1); }
    ~WinConn() override { close(); }
    int read(uint8_t* buf, size_t cap) override
    {
        if (s_ == INVALID_SOCKET) return -1;
        const int n = recv(s_, reinterpret_cast<char*>(buf), (int)cap, 0);
        if (n > 0) return n;
        if (n == 0) return -1;
        return WSAGetLastError() == WSAEWOULDBLOCK ? 0 : -1;
    }
    int write(const uint8_t* buf, size_t len) override
    {
        if (s_ == INVALID_SOCKET) return -1;
        const int n = send(s_, reinterpret_cast<const char*>(buf), (int)len, 0);
        if (n >= 0) return n;
        return WSAGetLastError() == WSAEWOULDBLOCK ? 0 : -1;
    }
    void close() override { if (s_ != INVALID_SOCKET) { closesocket(s_); s_ = INVALID_SOCKET; } }
    const char* remoteIp() const override { return ip_; }

private:
    SOCKET s_;
    char ip_[INET_ADDRSTRLEN] = "";
};

class WinListener : public raft::web::Listener {
public:
    bool begin(const char* bindIp, uint16_t port)
    {
        ls_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (ls_ == INVALID_SOCKET) return false;
        sockaddr_in a = {};
        a.sin_family = AF_INET;
        a.sin_port = htons(port);
        inet_pton(AF_INET, bindIp, &a.sin_addr);
        if (bind(ls_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 || listen(ls_, 8) != 0) { closesocket(ls_); ls_ = INVALID_SOCKET; return false; }
        setNonBlocking(ls_);
        return true;
    }
    ~WinListener() override { if (ls_ != INVALID_SOCKET) closesocket(ls_); }
    raft::web::Conn* accept() override
    {
        if (ls_ == INVALID_SOCKET) return nullptr;
        sockaddr_in peer = {};
        int pl = sizeof(peer);
        SOCKET c = ::accept(ls_, reinterpret_cast<sockaddr*>(&peer), &pl);
        if (c == INVALID_SOCKET) return nullptr;
        setNonBlocking(c);
        BOOL nd = TRUE;
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
        return new WinConn(c, peer);
    }

private:
    SOCKET ls_ = INVALID_SOCKET;
};

}  // namespace wintr
