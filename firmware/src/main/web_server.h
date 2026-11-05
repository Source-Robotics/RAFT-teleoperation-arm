// Small HTTP/1.1 + WebSocket server for the RAFT web UI (docs/web_ui.md). Portable C++11: the ESP32 binds
// it to WiFiServer/WiFiClient (web_esp.cpp), the PC simulator to Winsock (tools/sim/win_web.h), the unit
// test to in-memory connections (tools/sim/test_web.cpp).
//   GET  /             the UI page, gzipped, from flash
//   GET  /api/status   status JSON
//   POST /api          one JSON command in the body, the App's JSON reply back (for scripts)
//   GET  /ws           WebSocket:
//     server -> client  binary: the 288-byte leader frame ("RAFT"), at the rate the client asks for;
//                               the health message ("RFTH", web_bridge.h) once a second if asked for
//                       text:   {"t":"hello",...} once, {"t":"s","r":{status}} periodically or when it
//                               changes, {"t":"r","id":N,"r":{reply}} for every command
//     client -> server  text:   {"cmd":"...", ..., "id":N}; "ws_rate" with "frame_hz", "status_hz",
//                               "status": "change" and "health": true is handled here, everything else
//                               goes to the Handler
// Nothing is sent unless a client asks, and writes never block: a client that cannot keep up gets fewer
// leader frames, and one that stops reading is dropped.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace raft {
namespace web {

class Conn {
public:
    virtual ~Conn() {}
    virtual int read(uint8_t* buf, size_t cap) = 0;          // > 0 bytes, 0 nothing yet, < 0 closed
    virtual int write(const uint8_t* buf, size_t len) = 0;   // bytes accepted (0 = would block), < 0 broken
    virtual void close() = 0;
    virtual const char* remoteIp() const = 0;
};

class Listener {
public:
    virtual ~Listener() {}
    virtual Conn* accept() = 0;                              // a new connection (owned by the server) or null
};

class Handler {
public:
    virtual ~Handler() {}
    virtual size_t command(const char* json, char* reply, size_t cap, const char* clientIp) = 0;
    virtual size_t status(char* out, size_t cap) = 0;        // one JSON object
    virtual size_t frame(uint8_t* buf, size_t cap) = 0;      // the latest leader frame
    virtual size_t hello(char* out, size_t cap) = 0;         // one JSON object with the static facts
    // A number that changes whenever the status changes, apart from its live values (App::statusKey): a client
    // that asked for the status on change gets it only then, and every kKeepaliveS. 0 = no key: only then.
    virtual uint32_t statusKey() { return 0; }
    // The binary health message, with its window sequence number at bytes 8..11 (web_bridge.h); 0 = none
    virtual size_t health(uint8_t* buf, size_t cap) { (void)buf; (void)cap; return 0; }
};

struct Asset {
    const char* path;          // "/"
    const char* type;          // "text/html; charset=utf-8"
    const uint8_t* data;
    size_t len;
    bool gzip;
    const char* etag;          // quoted, e.g. "\"1a2b3c4d\""
};

class Server {
public:
    static constexpr int kMaxConns = 4;
    static constexpr size_t kRxCap = 1536;
    static constexpr size_t kTxCap = 8192;
    static constexpr double kKeyCheckS = 0.1;      // how often an on-change client's status is checked for a change
    static constexpr double kKeepaliveS = 5.0;     // and sent anyway
    static constexpr double kHealthCheckS = 0.25;  // how often a new health window is looked for

    Server(Listener& listener, Handler& handler, const Asset* assets, int nAssets);
    ~Server();
    void poll(double now);
    int wsClients() const;
    uint32_t framesSent() const { return framesSent_; }

    // helpers, public for the unit test
    static void sha1(const uint8_t* data, size_t len, uint8_t out[20]);
    static size_t base64(const uint8_t* in, size_t len, char* out, size_t cap);
    static void acceptKey(const char* clientKey, char out[29]);

private:
    enum State : uint8_t { FREE, HTTP, WS };
    struct Slot {
        Conn* c;
        State st;
        uint8_t rx[kRxCap];
        size_t rxLen;
        uint8_t tx[kTxCap];
        size_t txLen, txOff;
        const uint8_t* body;
        size_t bodyLen, bodyOff;
        bool closeWhenSent;
        double lastIo;
        double frameHz, statusHz, nextFrame, nextStatus;
        bool statusOnChange, health;
        uint32_t sentKey, healthSeq;
        double nextKeepalive, nextHealth;
    };

    void drop(Slot& s);
    bool queue(Slot& s, const void* data, size_t len);
    bool queueWs(Slot& s, uint8_t opcode, const void* data, size_t len);
    void flush(Slot& s, double now);
    void handleHttp(Slot& s, double now);
    void handleWs(Slot& s, double now);
    void wsText(Slot& s, char* text, size_t len, double now);
    void push(Slot& s, double now);
    bool sendStatus(Slot& s);
    uint32_t statusKey(double now);
    void respond(Slot& s, int code, const char* type, const char* body, size_t len, const char* extra);

    Listener& listener_;
    Handler& handler_;
    const Asset* assets_;
    int nAssets_;
    Slot slots_[kMaxConns];
    uint32_t framesSent_;
    uint32_t key_;                 // the handler's status key, shared by all clients for kKeyCheckS / 2
    double keyAt_;
    char scratch_[4096];
    char scratch2_[3072];
};

}  // namespace web
}  // namespace raft
