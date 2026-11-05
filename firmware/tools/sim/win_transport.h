// rti::Transport over Winsock (dev tool, Windows only): lets the PC run the firmware's PAR6 link
// (src/main/rti_link.cpp, the same code the ESP32 runs) against the real RCB runtime on the Pi, or
// against fake_rcb_server.exe on this PC.
//   TCP  executor port 29000, connect with a 3 s timeout (names via getaddrinfo; Windows 10+ resolves .local)
//   UDP  bound to the state port (30002); commands go to the robot's 30001
// Header-only; link with ws2_32.lib.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOGDI
#define NOGDI          // wingdi.h defines ERROR, which collides with rti::Reply::ERROR
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "rti_link.h"

namespace wintr {

struct WsaInit {
    bool ok = false;
    WsaInit() { WSADATA d; ok = WSAStartup(MAKEWORD(2, 2), &d) == 0; }
    ~WsaInit() { if (ok) WSACleanup(); }
};

inline void setNonBlocking(SOCKET s) { u_long on = 1; ioctlsocket(s, FIONBIO, &on); }

// Sleep about 1 ms. Sleep(1) can take 15.6 ms on Windows 11 when the system ignores a background process's
// timeBeginPeriod; a high-resolution waitable timer (Windows 10 1803+) does not.
inline void napMs()
{
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
    static HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    if (!timer) { Sleep(1); return; }
    LARGE_INTEGER due;
    due.QuadPart = -10000;                 // 1 ms, relative, in 100 ns units
    SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE);
    WaitForSingleObject(timer, INFINITE);
}

inline bool resolve4(const char* host, sockaddr_in& out)
{
    addrinfo hints = {}, *res = nullptr;
    hints.ai_family = AF_INET;
    if (getaddrinfo(host, nullptr, &hints, &res) != 0 || !res) return false;
    out = *reinterpret_cast<sockaddr_in*>(res->ai_addr);
    freeaddrinfo(res);
    return true;
}

inline std::string ipString(const sockaddr_in& a)
{
    char b[INET_ADDRSTRLEN] = "";
    inet_ntop(AF_INET, &a.sin_addr, b, sizeof(b));
    return b;
}

class WinTransport : public raft::rti::Transport {
public:
    // bindIp: address to bind the state port on ("0.0.0.0" for a real robot, "127.0.0.1" for loopback tests)
    explicit WinTransport(const char* bindIp = "0.0.0.0", uint16_t statePort = raft::rti::kPortState)
    {
        udp_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        sockaddr_in a = {};
        a.sin_family = AF_INET;
        a.sin_port = htons(statePort);
        inet_pton(AF_INET, bindIp, &a.sin_addr);
        udpOk_ = udp_ != INVALID_SOCKET && bind(udp_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
        if (udpOk_) setNonBlocking(udp_);
        else std::snprintf(err_, sizeof(err_), "cannot bind UDP %s:%u (in use?)", bindIp, (unsigned)statePort);
    }
    ~WinTransport() override
    {
        tcpClose();
        if (udp_ != INVALID_SOCKET) closesocket(udp_);
    }
    bool ok() const { return udpOk_; }
    const char* error() const { return err_; }

    bool tcpConnect(const char* host, uint16_t port) override
    {
        tcpClose();
        if (!resolve4(host, peer_)) { std::snprintf(err_, sizeof(err_), "cannot resolve %s", host); return false; }
        peer_.sin_port = htons(port);
        tcp_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcp_ == INVALID_SOCKET) return false;
        setNonBlocking(tcp_);
        connect(tcp_, reinterpret_cast<sockaddr*>(&peer_), sizeof(peer_));
        fd_set w, e;
        FD_ZERO(&w); FD_ZERO(&e); FD_SET(tcp_, &w); FD_SET(tcp_, &e);
        timeval tv = {3, 0};
        int soErr = 0, len = sizeof(soErr);
        if (select(0, nullptr, &w, &e, &tv) <= 0 || !FD_ISSET(tcp_, &w)
            || getsockopt(tcp_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&soErr), &len) != 0 || soErr != 0) {
            std::snprintf(err_, sizeof(err_), "no executor at %s:%u", ipString(peer_).c_str(), (unsigned)port);
            tcpClose();
            return false;
        }
        BOOL nd = TRUE;
        setsockopt(tcp_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
        sockaddr_in me = {};
        int ml = sizeof(me);
        getsockname(tcp_, reinterpret_cast<sockaddr*>(&me), &ml);
        std::strncpy(localIp_, ipString(me).c_str(), sizeof(localIp_) - 1);
        robot_ = peer_;
        haveRobot_ = true;
        rx_.clear();
        return true;
    }
    bool tcpConnected() override { return tcp_ != INVALID_SOCKET; }
    void tcpClose() override
    {
        if (tcp_ != INVALID_SOCKET) { closesocket(tcp_); tcp_ = INVALID_SOCKET; }
        rx_.clear();
        closeAfterDrain_ = false;
    }
    bool tcpSend(const char* data, size_t len) override
    {
        if (tcp_ == INVALID_SOCKET) return false;
        size_t done = 0;
        while (done < len) {
            const int n = send(tcp_, data + done, (int)(len - done), 0);
            if (n > 0) { done += (size_t)n; continue; }
            if (WSAGetLastError() != WSAEWOULDBLOCK) { tcpClose(); return false; }
            Sleep(1);
        }
        return true;
    }
    size_t tcpReadLine(char* buf, size_t cap) override
    {
        if (tcp_ != INVALID_SOCKET) {
            char tmp[2048];
            for (;;) {
                const int n = recv(tcp_, tmp, sizeof(tmp), 0);
                if (n > 0) { rx_.append(tmp, (size_t)n); continue; }
                if (n == 0 || WSAGetLastError() != WSAEWOULDBLOCK) closeAfterDrain_ = true;   // peer closed
                break;
            }
        }
        const size_t nl = rx_.find('\n');
        if (nl == std::string::npos) {
            if (closeAfterDrain_) { closeAfterDrain_ = false; tcpClose(); }
            return 0;
        }
        std::string line = rx_.substr(0, nl);
        rx_.erase(0, nl + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const size_t n = line.size() < cap - 1 ? line.size() : cap - 1;
        std::memcpy(buf, line.data(), n);
        buf[n] = '\0';
        return n;
    }
    bool udpSend(const uint8_t* data, size_t len, uint16_t port) override
    {
        if (!udpOk_ || !haveRobot_) return false;
        sockaddr_in to = robot_;
        to.sin_port = htons(port);
        return sendto(udp_, reinterpret_cast<const char*>(data), (int)len, 0, reinterpret_cast<sockaddr*>(&to), sizeof(to)) == (int)len;
    }
    size_t udpRecv(char* buf, size_t cap) override
    {
        if (!udpOk_) return 0;
        for (int guard = 0; guard < 8; ++guard) {
            sockaddr_in from = {};
            int fl = sizeof(from);
            const int n = recvfrom(udp_, buf, (int)cap, 0, reinterpret_cast<sockaddr*>(&from), &fl);
            if (n <= 0) return 0;
            if (haveRobot_ && from.sin_addr.s_addr != robot_.sin_addr.s_addr) continue;   // not from the robot
            ++udpIn_;
            return (size_t)n;
        }
        return 0;
    }
    const char* localIp() override { return localIp_; }
    unsigned long datagramsIn() const { return udpIn_; }

private:
    SOCKET tcp_ = INVALID_SOCKET, udp_ = INVALID_SOCKET;
    bool udpOk_ = false, haveRobot_ = false, closeAfterDrain_ = false;
    sockaddr_in peer_ = {}, robot_ = {};
    std::string rx_;
    char localIp_[INET_ADDRSTRLEN] = "0.0.0.0";
    char err_[96] = "";
    unsigned long udpIn_ = 0;
};

}  // namespace wintr
