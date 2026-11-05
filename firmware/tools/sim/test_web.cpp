// Host unit test of the web UI server (src/main/web_server.cpp) over in-memory connections: HTTP routes,
// ETag, the WebSocket handshake and framing, command replies, pushed frames and status, slow clients.
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

#include "web_bridge.h"
#include "web_server.h"

using raft::web::Conn;
using raft::web::Server;

static int g_fail = 0;
static void check(bool ok, const char* what) { std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) ++g_fail; }

// both ends of one in-memory TCP connection; the test keeps it, the server owns the Conn
struct Pipe {
    std::string toServer, fromServer;
    bool closedByServer = false, peerClosed = false;
    size_t writeLimit = (size_t)-1;          // bytes the "socket" accepts per write call
};

struct MemConn : Conn {
    explicit MemConn(Pipe* p) : p_(p) {}
    int read(uint8_t* buf, size_t cap) override
    {
        if (p_->toServer.empty()) return p_->peerClosed ? -1 : 0;
        const size_t n = p_->toServer.size() < cap ? p_->toServer.size() : cap;
        std::memcpy(buf, p_->toServer.data(), n);
        p_->toServer.erase(0, n);
        return (int)n;
    }
    int write(const uint8_t* buf, size_t len) override
    {
        const size_t n = len < p_->writeLimit ? len : p_->writeLimit;
        p_->fromServer.append(reinterpret_cast<const char*>(buf), n);
        return (int)n;
    }
    void close() override { p_->closedByServer = true; }
    const char* remoteIp() const override { return "10.0.0.7"; }
    Pipe* p_;
};

struct MemListener : raft::web::Listener {
    std::deque<Pipe*> pending;
    Conn* accept() override
    {
        if (pending.empty()) return nullptr;
        Pipe* p = pending.front();
        pending.pop_front();
        return new MemConn(p);
    }
};

struct FakeHandler : raft::web::Handler {
    std::string lastCommand, lastIp;
    int frames = 0;
    size_t command(const char* json, char* reply, size_t cap, const char* ip) override
    {
        lastCommand = json; lastIp = ip ? ip : "";
        return (size_t)std::snprintf(reply, cap, "{\"ok\": true, \"echo\": %u}", (unsigned)std::strlen(json));
    }
    size_t status(char* out, size_t cap) override { return (size_t)std::snprintf(out, cap, "{\"ok\": true, \"mode\": \"limp\"}"); }
    size_t frame(uint8_t* buf, size_t cap) override
    {
        if (cap < 160) return 0;
        std::memset(buf, 0, 160);
        std::memcpy(buf, "RAFT", 4);
        ++frames;
        return 160;
    }
    size_t hello(char* out, size_t cap) override { return (size_t)std::snprintf(out, cap, "{\"fw\": \"test\"}"); }
    uint32_t key = 1, healthSeq = 1;
    int statuses = 0, healthCalls = 0;
    uint32_t statusKey() override { return key; }
    size_t health(uint8_t* buf, size_t cap) override
    {
        ++healthCalls;
        if (cap < 16) return 0;
        std::memset(buf, 0, 16);
        std::memcpy(buf, "RFTH", 4);
        for (int k = 0; k < 4; ++k) buf[8 + k] = (uint8_t)(healthSeq >> (8 * k));
        return 16;
    }
};

// a masked client frame
static std::string clientFrame(uint8_t opcode, const std::string& payload)
{
    std::string f;
    f += (char)(0x80 | opcode);
    const uint8_t mk[4] = {0x12, 0x34, 0x56, 0x78};
    if (payload.size() < 126) f += (char)(0x80 | payload.size());
    else { f += static_cast<char>(0xFE); f += static_cast<char>(payload.size() >> 8); f += static_cast<char>(payload.size() & 0xff); }
    f.append(reinterpret_cast<const char*>(mk), 4);
    for (size_t i = 0; i < payload.size(); ++i) f += (char)(payload[i] ^ mk[i & 3]);
    return f;
}

// pop one server frame from s: returns opcode, -1 if incomplete
static int serverFrame(std::string& s, std::string& payload)
{
    if (s.size() < 2) return -1;
    const uint8_t b0 = (uint8_t)s[0], b1 = (uint8_t)s[1];
    size_t len = b1 & 0x7f, hl = 2;
    if (len == 126) { if (s.size() < 4) return -1; len = ((size_t)(uint8_t)s[2] << 8) | (uint8_t)s[3]; hl = 4; }
    if (s.size() < hl + len) return -1;
    payload = s.substr(hl, len);
    s.erase(0, hl + len);
    return b0 & 0x0f;
}

int main()
{
    std::printf("=== SHA-1, base64, WebSocket accept key ===\n");
    {
        uint8_t d[20];
        Server::sha1(reinterpret_cast<const uint8_t*>("abc"), 3, d);
        char hex[41];
        for (int i = 0; i < 20; ++i) std::snprintf(hex + 2 * i, 3, "%02x", d[i]);
        check(!std::strcmp(hex, "a9993e364706816aba3e25717850c26c9cd0d89d"), "sha1(\"abc\")");
        const std::string m(1000, 'a');
        Server::sha1(reinterpret_cast<const uint8_t*>(m.data()), m.size(), d);
        for (int i = 0; i < 20; ++i) std::snprintf(hex + 2 * i, 3, "%02x", d[i]);
        check(!std::strcmp(hex, "291e9a6c66994949b57ba5e650361e98fc36b1ba"), "sha1 of 1000 bytes (multi-block)");
        char acc[29];
        Server::acceptKey("dGhlIHNhbXBsZSBub25jZQ==", acc);
        check(!std::strcmp(acc, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="), "RFC 6455 sample accept key");
    }

    const uint8_t page[] = {0x1f, 0x8b, 8, 0, 1, 2, 3, 4, 5};
    const raft::web::Asset assets[] = {{"/", "text/html; charset=utf-8", page, sizeof(page), true, "\"abc123\""}};
    MemListener lis;
    FakeHandler h;
    Server srv(lis, h, assets, 1);
    double t = 0.0;
    auto run = [&](double s) { for (double end = t + s; t < end; t += 0.001) srv.poll(t); };

    std::printf("=== HTTP ===\n");
    {
        Pipe p; p.toServer = "GET / HTTP/1.1\r\nHost: x\r\nAccept-Encoding: gzip\r\n\r\n";
        lis.pending.push_back(&p); run(0.01);
        const size_t hdr = p.fromServer.find("\r\n\r\n");
        check(p.fromServer.rfind("HTTP/1.1 200 OK", 0) == 0 && p.fromServer.find("Content-Encoding: gzip") != std::string::npos
              && p.fromServer.find("ETag: \"abc123\"") != std::string::npos && hdr != std::string::npos
              && p.fromServer.substr(hdr + 4) == std::string(reinterpret_cast<const char*>(page), sizeof(page)) && p.closedByServer,
              "GET / serves the gzipped page and closes");
    }
    {
        Pipe p; p.toServer = "GET /?x=1 HTTP/1.1\r\nIf-None-Match: \"abc123\"\r\n\r\n";
        lis.pending.push_back(&p); run(0.01);
        check(p.fromServer.rfind("HTTP/1.1 304", 0) == 0 && p.fromServer.find("\r\n\r\n") + 4 == p.fromServer.size(), "matching ETag -> 304, no body");
    }
    {
        Pipe p; p.toServer = "GET /nope HTTP/1.1\r\n\r\n";
        lis.pending.push_back(&p); run(0.01);
        check(p.fromServer.rfind("HTTP/1.1 404", 0) == 0, "unknown path -> 404");
    }
    {
        Pipe p; p.toServer = "GET /api/status HTTP/1.1\r\n\r\n";
        lis.pending.push_back(&p); run(0.01);
        check(p.fromServer.find("\"mode\": \"limp\"") != std::string::npos && p.fromServer.find("application/json") != std::string::npos, "GET /api/status");
    }
    {
        Pipe p; p.toServer = "POST /api HTTP/1.1\r\nContent-Length: 17\r\n\r\n{\"cmd\": \"st";
        lis.pending.push_back(&p); run(0.01);
        const bool waited = p.fromServer.empty();
        p.toServer += "atus\"}"; run(0.01);
        check(waited && h.lastCommand == "{\"cmd\": \"status\"}" && h.lastIp == "10.0.0.7" && p.fromServer.find("\"echo\": 17") != std::string::npos,
              "POST /api waits for the whole body, forwards it with the client address");
    }
    {
        Pipe p; p.toServer = "GET / HTTP/1.1\r\n";        // never finishes its header
        lis.pending.push_back(&p); run(11.0);
        check(p.closedByServer, "an idle half-sent request is dropped after 10 s");
    }

    std::printf("=== WebSocket ===\n");
    Pipe w;
    w.toServer = "GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: keep-alive, Upgrade\r\n"
                 "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";
    lis.pending.push_back(&w);
    run(0.002);
    {
        const size_t hdr = w.fromServer.find("\r\n\r\n");
        const bool ok = w.fromServer.rfind("HTTP/1.1 101", 0) == 0 && w.fromServer.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != std::string::npos;
        w.fromServer.erase(0, hdr + 4);
        std::string pl;
        const int op = serverFrame(w.fromServer, pl);
        check(ok && op == 1 && pl == "{\"t\":\"hello\",\"r\":{\"fw\": \"test\"}}" && srv.wsClients() == 1, "handshake, then the hello message");
    }
    {
        w.toServer += clientFrame(1, "{\"cmd\": \"ws_rate\", \"frame_hz\": 10, \"status_hz\": 1, \"id\": 3}");
        w.fromServer.clear();
        h.frames = 0;
        run(1.0005);
        int frames = 0, statuses = 0, replies = 0;
        bool replyOk = false;
        std::string pl;
        for (int op; (op = serverFrame(w.fromServer, pl)) >= 0;) {
            if (op == 2 && pl.size() == 160 && pl.compare(0, 4, "RAFT") == 0) ++frames;
            if (op == 1 && pl.rfind("{\"t\":\"s\"", 0) == 0) ++statuses;
            if (op == 1 && pl.rfind("{\"t\":\"r\",\"id\":3,", 0) == 0) { ++replies; replyOk = pl.find("\"frame_hz\": 10.0") != std::string::npos; }
        }
        std::printf("  (1 s: %d frames, %d status messages)\n", frames, statuses);
        check(replies == 1 && replyOk, "ws_rate is answered with the new rates");
        check(frames >= 10 && frames <= 11 && statuses >= 1 && statuses <= 2, "frames at 10 Hz, status at 1 Hz");
    }
    {
        w.toServer += clientFrame(1, "{\"cmd\": \"mode\", \"mode\": \"gravity\", \"id\": 42}");
        w.toServer += clientFrame(9, "hi");
        run(0.003);
        std::string pl;
        bool reply = false, pong = false;
        for (int op; (op = serverFrame(w.fromServer, pl)) >= 0;) {
            if (op == 1 && pl.rfind("{\"t\":\"r\",\"id\":42,\"r\":{\"ok\": true", 0) == 0) reply = true;
            if (op == 10 && pl == "hi") pong = true;
        }
        check(reply && h.lastCommand.find("\"mode\": \"gravity\"") != std::string::npos, "a command is forwarded and answered with its id");
        check(pong, "ping -> pong");
    }
    {
        const std::string big = "{\"cmd\": \"spring\", \"pad\": \"" + std::string(260, 'x') + "\", \"id\": 5}";
        const std::string f = clientFrame(1, big);
        w.toServer += f.substr(0, 100);                       // the frame arrives in two pieces
        run(0.002);
        w.toServer += f.substr(100);
        run(0.003);
        std::string pl;
        bool reply = false;
        for (int op; (op = serverFrame(w.fromServer, pl)) >= 0;) if (op == 1 && pl.rfind("{\"t\":\"r\",\"id\":5,", 0) == 0) reply = true;
        check(reply, "a 300-byte message (16-bit length) is handled");
    }
    {
        // a page that asks for the status on change and the health message
        Pipe c;
        c.toServer = "GET /ws HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n";
        lis.pending.push_back(&c);
        run(0.002);
        c.toServer += clientFrame(1, "{\"cmd\": \"ws_rate\", \"frame_hz\": 0, \"status\": \"change\", \"health\": true, \"id\": 7}");
        c.fromServer.clear();
        auto count = [&](int& st, int& hm, bool& replyOk) {
            std::string pl;
            for (int op; (op = serverFrame(c.fromServer, pl)) >= 0;) {
                if (op == 1 && pl.rfind("{\"t\":\"s\"", 0) == 0) ++st;
                if (op == 2 && pl.size() == 16 && pl.compare(0, 4, "RFTH") == 0) ++hm;
                if (op == 1 && pl.rfind("{\"t\":\"r\",\"id\":7,", 0) == 0) replyOk = pl.find("\"status\": \"change\", \"health\": true") != std::string::npos;
            }
        };
        int st = 0, hm = 0;
        bool replyOk = false;
        // 3 s with a new health window every second and the status key changing once at 1.5 s
        for (int k = 0; k < 3000; ++k) {
            if (k && k % 1000 == 0) ++h.healthSeq;
            if (k == 1500) h.key = 2;
            run(0.001);
        }
        count(st, hm, replyOk);
        std::printf("  (3 s: %d status messages, %d health messages, %d health calls)\n", st, hm, h.healthCalls);
        check(replyOk, "ws_rate status change, health true is answered");
        check(st == 2, "on change: one status at the start, one when the key changed, none in between");
        check(hm == 3 && h.healthCalls >= 11 && h.healthCalls <= 13, "health: once per new window (looked for every 0.25 s)");
        run(5.0);
        st = 0; hm = 0;
        count(st, hm, replyOk);
        check(st == 1, "an unchanged status is sent again after 5 s");
        c.peerClosed = true;
        run(0.01);
    }
    std::printf("=== Health message layout ===\n");
    {
        raft::HealthFigures f;
        f.loop.seq = 77; f.loop.hz = 250.0f; f.loop.jitterRmsUs = 1.25f; f.loop.partMaxUs[raft::kPartWrite] = 333; f.loop.failedReadsTotal = 9;
        f.loop.cpuLoad[1] = 25.5f; f.heapFree = 152500; f.linkMbps = 100; f.flags = raft::HF_NET_UP | raft::HF_FULL_DUPLEX; f.webClients = 2; f.udpFrames = 123456;
        const uint32_t missed[7] = {0, 0, 0, 12, 0, 412, 0};
        uint8_t b[raft::kHealthSize];
        const size_t n = raft::packHealth(f, missed, 3, 4, b, sizeof(b));
        auto u32 = [&](size_t o) { return (uint32_t)b[o] | ((uint32_t)b[o + 1] << 8) | ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24); };
        auto f32 = [&](size_t o) { const uint32_t u = u32(o); float v; std::memcpy(&v, &u, 4); return v; };
        check(n == 180 && !std::memcmp(b, "RFTH", 4) && b[4] == 1 && b[5] == 3 && (b[6] | (b[7] << 8)) == 180, "header: magic, version, flags, size");
        check(u32(8) == 77 && f32(20) == 250.0f && f32(36) == 1.25f && u32(76) == 333 && u32(92) == 9 && f32(100) == 25.5f, "loop window fields");
        check(u32(108) == 152500 && (b[132] | (b[133] << 8)) == 100 && b[134] == 2 && u32(140) == 123456, "platform fields");
        check(u32(156) == 12 && u32(164) == 412 && u32(172) == 3 && u32(176) == 4, "missed reads per servo and PAR6 counters");
        check(raft::packHealth(f, missed, 0, 0, b, 100) == 0, "too small a buffer: nothing");
    }
    {
        // a client that stops reading: frames stop being queued, and after 30 s the server drops it
        w.writeLimit = 0;
        const int before = h.frames;
        run(5.0);
        const int during = h.frames - before;
        run(26.0);
        check(during < 10 && w.closedByServer && srv.wsClients() == 0, "a stalled client gets few frames and is dropped after 30 s");
    }
    {
        Pipe c;
        c.toServer = "GET /ws HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n";
        lis.pending.push_back(&c);
        run(0.002);
        c.toServer += clientFrame(8, "\x03\xe8");
        run(0.003);
        check(c.closedByServer && srv.wsClients() == 0, "close frame -> close back and disconnect");
    }
    {
        Pipe p[6];
        for (Pipe& x : p) { x.toServer = "GET /ws HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: a2V5a2V5a2V5a2V5a2V5aw==\r\n\r\n"; lis.pending.push_back(&x); }
        run(0.005);
        int open = 0;
        for (Pipe& x : p) if (!x.closedByServer) ++open;
        check(open == Server::kMaxConns && srv.wsClients() == Server::kMaxConns, "connections beyond the limit are refused");
        for (Pipe& x : p) x.peerClosed = true;
        run(0.005);
        check(srv.wsClients() == 0, "closed peers free their slots");
    }
    {
        Pipe p; p.toServer = "GET /ws HTTP/1.1\r\nConnection: Upgrade\r\n\r\n";
        lis.pending.push_back(&p); run(0.01);
        check(p.fromServer.rfind("HTTP/1.1 400", 0) == 0, "/ws without the upgrade headers -> 400");
    }
    std::printf("%s\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return g_fail ? 1 : 0;
}
