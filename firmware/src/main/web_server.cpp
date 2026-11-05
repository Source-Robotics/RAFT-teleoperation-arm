#include "web_server.h"

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "json_lite.h"

namespace raft {
namespace web {
namespace {

const double kHttpIdleS = 10.0;          // an HTTP connection that sends nothing for this long is dropped
const double kWsIdleS = 30.0;            // a WebSocket that cannot be written to for this long is dropped
const size_t kFrameRoom = 512;           // a leader frame is queued only when less than this is pending

bool ieq(const char* a, const char* b)
{
    for (; *a && *b; ++a, ++b)
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    return *a == *b;
}

uint32_t rol(uint32_t x, int s) { return (x << s) | (x >> (32 - s)); }

void sha1Block(uint32_t h[5], const uint8_t* p)
{
    uint32_t w[80];
    for (int i = 0; i < 16; ++i)
        w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
    for (int i = 16; i < 80; ++i) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20) { f = (b & c) | (~b & d); k = 0x5A827999u; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1u; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6u; }
        const uint32_t t = rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol(b, 30); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

// a header line's value, trimmed, into out; false when the header is absent
bool header(const char* head, const char* name, char* out, size_t cap)
{
    const size_t n = strlen(name);
    for (const char* line = strstr(head, "\r\n"); line; line = strstr(line, "\r\n")) {
        line += 2;
        if (!*line || (line[0] == '\r' && line[1] == '\n')) break;
        if (strlen(line) > n && line[n] == ':') {
            char nm[40];
            if (n >= sizeof(nm)) continue;
            memcpy(nm, line, n);
            nm[n] = '\0';
            if (!ieq(nm, name)) continue;
            const char* v = line + n + 1;
            while (*v == ' ' || *v == '\t') ++v;
            size_t k = 0;
            while (v[k] && v[k] != '\r' && k + 1 < cap) { out[k] = v[k]; ++k; }
            while (k && (out[k - 1] == ' ' || out[k - 1] == '\t')) --k;
            out[k] = '\0';
            return true;
        }
    }
    return false;
}

bool containsToken(const char* value, const char* token)
{
    // case-insensitive search for a comma-separated token ("Upgrade" in "keep-alive, Upgrade")
    const size_t n = strlen(token);
    for (const char* p = value; *p; ++p) {
        size_t k = 0;
        while (k < n && p[k] && tolower((unsigned char)p[k]) == tolower((unsigned char)token[k])) ++k;
        if (k == n) return true;
    }
    return false;
}

}  // namespace

constexpr int Server::kMaxConns;
constexpr size_t Server::kRxCap;
constexpr size_t Server::kTxCap;
constexpr double Server::kKeyCheckS;
constexpr double Server::kKeepaliveS;
constexpr double Server::kHealthCheckS;

void Server::sha1(const uint8_t* data, size_t len, uint8_t out[20])
{
    uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    size_t i = 0;
    for (; i + 64 <= len; i += 64) sha1Block(h, data + i);
    uint8_t blk[128];
    const size_t rest = len - i;
    memset(blk, 0, sizeof(blk));
    memcpy(blk, data + i, rest);
    blk[rest] = 0x80;
    const size_t total = rest + 9 <= 64 ? 64 : 128;
    const uint64_t bits = (uint64_t)len * 8u;
    for (int k = 0; k < 8; ++k) blk[total - 1 - k] = (uint8_t)(bits >> (8 * k));
    sha1Block(h, blk);
    if (total == 128) sha1Block(h, blk + 64);
    for (int k = 0; k < 5; ++k) {
        out[4 * k] = (uint8_t)(h[k] >> 24); out[4 * k + 1] = (uint8_t)(h[k] >> 16);
        out[4 * k + 2] = (uint8_t)(h[k] >> 8); out[4 * k + 3] = (uint8_t)h[k];
    }
}

size_t Server::base64(const uint8_t* in, size_t len, char* out, size_t cap)
{
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < len; i += 3) {
        if (o + 4 >= cap) return 0;
        const uint32_t v = ((uint32_t)in[i] << 16) | (i + 1 < len ? (uint32_t)in[i + 1] << 8 : 0) | (i + 2 < len ? in[i + 2] : 0);
        out[o++] = T[(v >> 18) & 63];
        out[o++] = T[(v >> 12) & 63];
        out[o++] = i + 1 < len ? T[(v >> 6) & 63] : '=';
        out[o++] = i + 2 < len ? T[v & 63] : '=';
    }
    out[o] = '\0';
    return o;
}

void Server::acceptKey(const char* clientKey, char out[29])
{
    char buf[96];
    const int n = snprintf(buf, sizeof(buf), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", clientKey);
    uint8_t d[20];
    sha1(reinterpret_cast<const uint8_t*>(buf), n > 0 ? (size_t)n : 0, d);
    base64(d, sizeof(d), out, 29);
}

Server::Server(Listener& listener, Handler& handler, const Asset* assets, int nAssets)
    : listener_(listener), handler_(handler), assets_(assets), nAssets_(nAssets), framesSent_(0), key_(0), keyAt_(-1e9)
{
    for (Slot& s : slots_) { s.c = nullptr; s.st = FREE; }
}

Server::~Server()
{
    for (Slot& s : slots_) if (s.st != FREE) drop(s);
}

int Server::wsClients() const
{
    int n = 0;
    for (const Slot& s : slots_) if (s.st == WS) ++n;
    return n;
}

void Server::drop(Slot& s)
{
    if (s.c) { s.c->close(); delete s.c; }
    s.c = nullptr;
    s.st = FREE;
}

bool Server::queue(Slot& s, const void* data, size_t len)
{
    if (s.txOff > 0) {                                     // compact
        memmove(s.tx, s.tx + s.txOff, s.txLen - s.txOff);
        s.txLen -= s.txOff;
        s.txOff = 0;
    }
    if (s.txLen + len > kTxCap) return false;
    memcpy(s.tx + s.txLen, data, len);
    s.txLen += len;
    return true;
}

bool Server::queueWs(Slot& s, uint8_t opcode, const void* data, size_t len)
{
    uint8_t h[4];
    size_t hl;
    h[0] = (uint8_t)(0x80 | opcode);
    if (len < 126) { h[1] = (uint8_t)len; hl = 2; }
    else if (len <= 0xFFFF) { h[1] = 126; h[2] = (uint8_t)(len >> 8); h[3] = (uint8_t)len; hl = 4; }
    else return false;
    if ((s.txLen - s.txOff) + hl + len > kTxCap) return false;
    return queue(s, h, hl) && queue(s, data, len);
}

void Server::respond(Slot& s, int code, const char* type, const char* body, size_t len, const char* extra)
{
    const char* text = code == 200 ? "OK" : code == 304 ? "Not Modified" : code == 400 ? "Bad Request"
                     : code == 404 ? "Not Found" : code == 413 ? "Payload Too Large" : "Error";
    char h[400];
    const int n = snprintf(h, sizeof(h),
                           "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %u\r\nCache-Control: no-cache\r\n"
                           "Access-Control-Allow-Origin: *\r\n%sConnection: close\r\n\r\n",
                           code, text, type, (unsigned)len, extra ? extra : "");
    if (n > 0) queue(s, h, (size_t)n);
    if (body && len) queue(s, body, len);
    s.closeWhenSent = true;
}

void Server::handleHttp(Slot& s, double now)
{
    if (s.closeWhenSent || s.body) return;                 // already answered
    // the end of the header block
    size_t end = 0;
    for (size_t i = 3; i < s.rxLen; ++i)
        if (s.rx[i - 3] == '\r' && s.rx[i - 2] == '\n' && s.rx[i - 1] == '\r' && s.rx[i] == '\n') { end = i + 1; break; }
    if (!end) {
        if (s.rxLen >= kRxCap) respond(s, 400, "text/plain", "header too long\n", 16, nullptr);
        return;
    }
    char head[kRxCap + 1];
    memcpy(head, s.rx, end);
    head[end] = '\0';

    char method[8] = "", path[80] = "";
    {
        const char* sp1 = strchr(head, ' ');
        const char* sp2 = sp1 ? strchr(sp1 + 1, ' ') : nullptr;
        if (!sp1 || !sp2 || (size_t)(sp1 - head) >= sizeof(method) || (size_t)(sp2 - sp1 - 1) >= sizeof(path)) {
            respond(s, 400, "text/plain", "bad request\n", 12, nullptr);
            return;
        }
        memcpy(method, head, (size_t)(sp1 - head));
        method[sp1 - head] = '\0';
        memcpy(path, sp1 + 1, (size_t)(sp2 - sp1 - 1));
        path[sp2 - sp1 - 1] = '\0';
        char* q = strchr(path, '?');
        if (q) *q = '\0';
    }
    char upgrade[32] = "", connection[64] = "", key[64] = "", etag[48] = "", clen[16] = "";
    header(head, "Upgrade", upgrade, sizeof(upgrade));
    header(head, "Connection", connection, sizeof(connection));
    header(head, "Sec-WebSocket-Key", key, sizeof(key));
    header(head, "If-None-Match", etag, sizeof(etag));
    header(head, "Content-Length", clen, sizeof(clen));
    const long contentLength = clen[0] ? strtol(clen, nullptr, 10) : 0;

    if (!strcmp(method, "GET") && !strcmp(path, "/ws")) {
        if (!ieq(upgrade, "websocket") || !containsToken(connection, "upgrade") || !key[0]) {
            respond(s, 400, "text/plain", "websocket upgrade expected\n", 27, nullptr);
            return;
        }
        char acc[29];
        acceptKey(key, acc);
        char h[200];
        const int n = snprintf(h, sizeof(h),
                               "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                               "Sec-WebSocket-Accept: %s\r\n\r\n", acc);
        queue(s, h, (size_t)n);
        s.st = WS;
        memmove(s.rx, s.rx + end, s.rxLen - end);
        s.rxLen -= end;
        s.frameHz = 20.0;
        s.statusHz = 2.0;
        s.nextFrame = s.nextStatus = now;
        static const char pre[] = "{\"t\":\"hello\",\"r\":";
        const size_t room = sizeof(scratch_) - sizeof(pre) - 2;
        size_t m = handler_.hello(scratch_ + sizeof(pre) - 1, room);
        if (m) {
            memcpy(scratch_, pre, sizeof(pre) - 1);
            m += sizeof(pre) - 1;
            scratch_[m++] = '}';
            queueWs(s, 1, scratch_, m);
        }
        return;
    }
    if (!strcmp(method, "POST") && !strcmp(path, "/api")) {
        if (contentLength <= 0 || contentLength > 1024) { respond(s, 413, "text/plain", "body 1..1024 bytes\n", 19, nullptr); return; }
        if (s.rxLen < end + (size_t)contentLength) {
            if (s.rxLen >= kRxCap) respond(s, 413, "text/plain", "too large\n", 10, nullptr);
            return;                                        // wait for the rest of the body
        }
        memcpy(scratch2_, s.rx + end, (size_t)contentLength);
        scratch2_[contentLength] = '\0';
        const size_t n = handler_.command(scratch2_, scratch_, sizeof(scratch_), s.c->remoteIp());
        respond(s, 200, "application/json", scratch_, n, nullptr);
        return;
    }
    if (!strcmp(method, "GET") && !strcmp(path, "/api/status")) {
        const size_t n = handler_.status(scratch_, sizeof(scratch_));
        respond(s, 200, "application/json", scratch_, n, nullptr);
        return;
    }
    if (!strcmp(method, "GET") || !strcmp(method, "HEAD")) {
        for (int i = 0; i < nAssets_; ++i) {
            const Asset& a = assets_[i];
            if (strcmp(path, a.path)) continue;
            if (a.etag && etag[0] && !strcmp(etag, a.etag)) {
                char ex[80];
                snprintf(ex, sizeof(ex), "ETag: %s\r\n", a.etag);
                respond(s, 304, a.type, nullptr, 0, ex);
                return;
            }
            char ex[120];
            snprintf(ex, sizeof(ex), "%sETag: %s\r\n", a.gzip ? "Content-Encoding: gzip\r\n" : "", a.etag ? a.etag : "\"0\"");
            respond(s, 200, a.type, nullptr, a.len, ex);
            if (!strcmp(method, "GET")) { s.body = a.data; s.bodyLen = a.len; s.bodyOff = 0; }
            return;
        }
    }
    respond(s, 404, "text/plain", "not found\n", 10, nullptr);
}

void Server::wsText(Slot& s, char* text, size_t len, double now)
{
    (void)len;
    double id = 0.0;
    json::getNumber(text, "id", id);
    char cmd[24] = "";
    json::getString(text, "cmd", cmd, sizeof(cmd));
    // reply body goes to scratch_ + 48, the wrapper prefix right before it
    char* body = scratch_ + 48;
    const size_t cap = sizeof(scratch_) - 48 - 2;
    size_t n;
    if (!strcmp(cmd, "ws_rate")) {
        double v;
        if (json::getNumber(text, "frame_hz", v)) s.frameHz = v < 0 ? 0 : (v > 100 ? 100 : v);
        if (json::getNumber(text, "status_hz", v)) { s.statusHz = v < 0 ? 0 : (v > 10 ? 10 : v); s.statusOnChange = false; }
        char mode[16] = "";
        if (json::getString(text, "status", mode, sizeof(mode))) s.statusOnChange = !strcmp(mode, "change");
        bool b = false;
        if (json::getBool(text, "health", b)) s.health = b;
        s.nextFrame = s.nextStatus = s.nextKeepalive = s.nextHealth = now;
        s.healthSeq = 0xffffffffu;
        const int m = snprintf(body, cap, "{\"ok\": true, \"frame_hz\": %.1f, \"status_hz\": %.1f, \"status\": \"%s\", \"health\": %s}", s.frameHz,
                               s.statusHz, s.statusOnChange ? "change" : "periodic", s.health ? "true" : "false");
        n = m > 0 ? (size_t)m : 0;
    } else if (!strcmp(cmd, "ping")) {     // the page's round-trip probe: answered here, without the App lock
        const int m = snprintf(body, cap, "{\"ok\": true}");
        n = m > 0 ? (size_t)m : 0;
    } else {
        n = handler_.command(text, body, cap, s.c->remoteIp());
    }
    if (!n) return;
    char pre[48];
    const int pl = snprintf(pre, sizeof(pre), "{\"t\":\"r\",\"id\":%.0f,\"r\":", id);
    char* start = body - pl;
    memcpy(start, pre, (size_t)pl);
    body[n] = '}';
    if (!queueWs(s, 1, start, (size_t)pl + n + 1)) drop(s);   // a client that does not read its replies goes
}

void Server::handleWs(Slot& s, double now)
{
    for (;;) {
        if (s.st != WS || s.rxLen < 2) return;
        const uint8_t b0 = s.rx[0], b1 = s.rx[1];
        const uint8_t op = b0 & 0x0F;
        const bool fin = (b0 & 0x80) != 0;
        size_t len = b1 & 0x7F, hl = 2;
        if (!(b1 & 0x80)) { drop(s); return; }              // client frames must be masked
        if (len == 127) { drop(s); return; }                // far too big for us
        if (len == 126) {
            if (s.rxLen < 4) return;
            len = ((size_t)s.rx[2] << 8) | s.rx[3];
            hl = 4;
        }
        if (hl + 4 + len > kRxCap) { drop(s); return; }
        if (s.rxLen < hl + 4 + len) return;
        uint8_t* p = s.rx + hl + 4;
        const uint8_t* mk = s.rx + hl;
        for (size_t i = 0; i < len; ++i) p[i] ^= mk[i & 3];
        if (op == 1 && fin) {
            const size_t m = len < sizeof(scratch2_) - 1 ? len : sizeof(scratch2_) - 1;
            memcpy(scratch2_, p, m);
            scratch2_[m] = '\0';
            wsText(s, scratch2_, m, now);
            if (s.st != WS) return;
        } else if (op == 8) {
            queueWs(s, 8, p, len < 2 ? len : 2);
            s.closeWhenSent = true;
        } else if (op == 9) {
            queueWs(s, 10, p, len);
        }
        const size_t used = hl + 4 + len;
        memmove(s.rx, s.rx + used, s.rxLen - used);
        s.rxLen -= used;
    }
}

uint32_t Server::statusKey(double now)
{
    if (now - keyAt_ >= kKeyCheckS / 2 || now < keyAt_) { key_ = handler_.statusKey(); keyAt_ = now; }
    return key_;
}

bool Server::sendStatus(Slot& s)
{
    if (kTxCap - (s.txLen - s.txOff) <= sizeof(scratch_) + 16) return false;   // no room: try again later
    static const char pre[] = "{\"t\":\"s\",\"r\":";
    size_t n = handler_.status(scratch_ + sizeof(pre) - 1, sizeof(scratch_) - sizeof(pre) - 2);
    if (n) {
        memcpy(scratch_, pre, sizeof(pre) - 1);
        n += sizeof(pre) - 1;
        scratch_[n++] = '}';
        queueWs(s, 1, scratch_, n);
    }
    return true;
}

void Server::push(Slot& s, double now)
{
    if (s.st != WS || s.closeWhenSent) return;
    if (s.statusOnChange) {
        // printing the status is the costly part (~110 decimal numbers): only when something in it changed
        if (now >= s.nextStatus) {
            s.nextStatus = now + kKeyCheckS;
            const uint32_t k = statusKey(now);
            if ((k != s.sentKey || now >= s.nextKeepalive) && sendStatus(s)) { s.sentKey = k; s.nextKeepalive = now + kKeepaliveS; }
        }
    } else if (s.statusHz > 0 && now >= s.nextStatus) {
        if (sendStatus(s)) {
            const double period = 1.0 / s.statusHz;
            s.nextStatus = (s.nextStatus + period < now) ? now + period : s.nextStatus + period;
        }
    }
    if (s.health && now >= s.nextHealth) {
        s.nextHealth = now + kHealthCheckS;
        uint8_t b[256];
        const size_t n = kTxCap - (s.txLen - s.txOff) > sizeof(b) + 8 ? handler_.health(b, sizeof(b)) : 0;
        if (n >= 12) {
            const uint32_t seq = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24);
            if (seq != s.healthSeq && queueWs(s, 2, b, n)) s.healthSeq = seq;
        }
    }
    if (s.frameHz > 0 && now >= s.nextFrame) {
        if (s.txLen - s.txOff < kFrameRoom) {               // skip the frame for a client that is behind
            uint8_t f[512];
            const size_t n = handler_.frame(f, sizeof(f));
            if (n && queueWs(s, 2, f, n)) ++framesSent_;
        }
        const double period = 1.0 / s.frameHz;
        s.nextFrame = (s.nextFrame + period < now) ? now + period : s.nextFrame + period;
    }
}

void Server::flush(Slot& s, double now)
{
    while (s.st != FREE && s.txLen > s.txOff) {
        const int n = s.c->write(s.tx + s.txOff, s.txLen - s.txOff);
        if (n < 0) { drop(s); return; }
        if (n == 0) return;
        s.txOff += (size_t)n;
        s.lastIo = now;
        if (s.txOff == s.txLen) s.txOff = s.txLen = 0;
    }
    while (s.st != FREE && s.body && s.bodyOff < s.bodyLen) {
        const size_t chunk = s.bodyLen - s.bodyOff < 2920 ? s.bodyLen - s.bodyOff : 2920;
        const int n = s.c->write(s.body + s.bodyOff, chunk);
        if (n < 0) { drop(s); return; }
        if (n == 0) return;
        s.bodyOff += (size_t)n;
        s.lastIo = now;
    }
    if (s.st != FREE && s.body && s.bodyOff >= s.bodyLen) s.body = nullptr;
    if (s.st != FREE && s.closeWhenSent && s.txLen == s.txOff && !s.body) drop(s);
}

void Server::poll(double now)
{
    for (int k = 0; k < kMaxConns; ++k) {
        Slot* free = nullptr;
        for (Slot& s : slots_) if (s.st == FREE) { free = &s; break; }
        Conn* c = listener_.accept();
        if (!c) break;
        if (!free) { c->close(); delete c; continue; }     // full: the browser retries
        free->c = c;
        free->st = HTTP;
        free->rxLen = free->txLen = free->txOff = 0;
        free->body = nullptr;
        free->bodyLen = free->bodyOff = 0;
        free->closeWhenSent = false;
        free->lastIo = now;
        free->frameHz = free->statusHz = 0.0;
        free->nextFrame = free->nextStatus = now;
        free->statusOnChange = free->health = false;
        free->sentKey = 0;
        free->healthSeq = 0xffffffffu;
        free->nextKeepalive = free->nextHealth = now;
    }
    for (Slot& s : slots_) {
        if (s.st == FREE) continue;
        while (s.rxLen < kRxCap) {
            const int n = s.c->read(s.rx + s.rxLen, kRxCap - s.rxLen);
            if (n < 0) { drop(s); break; }
            if (n == 0) break;
            s.rxLen += (size_t)n;
            s.lastIo = now;
        }
        if (s.st == FREE) continue;
        if (s.st == HTTP) handleHttp(s, now);
        if (s.st == WS) { handleWs(s, now); push(s, now); }
        if (s.st == FREE) continue;
        flush(s, now);
        if (s.st == FREE) continue;
        const bool pending = s.txLen > s.txOff || s.body;
        if (s.st == HTTP && now - s.lastIo > kHttpIdleS) drop(s);
        else if (s.st == WS && pending && now - s.lastIo > kWsIdleS) drop(s);
    }
}

}  // namespace web
}  // namespace raft
