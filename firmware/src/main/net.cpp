#include "net.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/sockets.h>
#include <string.h>
#include <strings.h>

#include "leader_frame.h"

namespace raft {
namespace net {
namespace {

Callbacks g_cb = {};
char g_hostname[32] = "raft-leader";
volatile bool g_up = false;
bool g_started = false;
char g_ip[16] = "0.0.0.0";

WiFiServer g_server(kCommandPort);
struct Client {
    WiFiClient sock;
    char line[512];
    size_t len = 0;
    char ip[16] = "";
};
Client g_clients[kMaxClients];

WiFiUDP g_stream;
bool g_streamOk = false;
uint32_t g_lastFrameMs = 0;
uint32_t g_frames = 0;
char g_streamHost[40] = "";
IPAddress g_streamIp;
uint32_t g_lastResolveMs = 0;

void onEvent(WiFiEvent_t event)
{
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
        ETH.setHostname(g_hostname);
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        strncpy(g_ip, ETH.localIP().toString().c_str(), sizeof(g_ip) - 1);
        g_up = true;
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
        g_up = false;
        break;
    default:
        break;
    }
}

void startServices()
{
    ArduinoOTA.setHostname(g_hostname);         // also starts mDNS under this name
#ifdef RAFT_OTA_PASSWORD
    ArduinoOTA.setPassword(RAFT_OTA_PASSWORD);
#endif
    ArduinoOTA.onStart([]() {
        if (g_cb.otaStarting) g_cb.otaStarting();
        Serial.println("OTA update starting: torque off");
    });
    ArduinoOTA.onEnd([]() { Serial.println("OTA update done, rebooting"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", (unsigned)e); });
    ArduinoOTA.begin();
    MDNS.addService("raft", "tcp", kCommandPort);
    MDNS.addService("http", "tcp", 80);          // the web UI (main.cpp)
    g_server.begin();
    g_server.setNoDelay(true);
    g_streamOk = g_stream.begin(kCommandPort + 6) == 1;      // local port for the leader stream (5006)
    if (g_cb.networkUp) g_cb.networkUp();
    Serial.printf("Network up: %s (%s.local), commands on TCP %u\n", g_ip, g_hostname, (unsigned)kCommandPort);
    g_started = true;
}

void acceptClients()
{
    WiFiClient c = g_server.available();
    if (!c) return;
    for (Client& s : g_clients) {
        if (!s.sock.connected()) {
            s.sock.stop();
            s.sock = c;
            s.sock.setNoDelay(true);
            s.len = 0;
            strncpy(s.ip, c.remoteIP().toString().c_str(), sizeof(s.ip) - 1);
            return;
        }
    }
    c.print("{\"ok\": false, \"error\": \"too many clients\"}\n");
    c.stop();
}

void serveClient(Client& s)
{
    if (!s.sock.connected()) return;
    int budget = 512;                               // bytes per poll, so one chatty client cannot stall the task
    while (budget-- > 0 && s.sock.available() > 0) {
        const int ch = s.sock.read();
        if (ch < 0) break;
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (s.len < sizeof(s.line) - 1) s.line[s.len++] = (char)ch;
            continue;
        }
        s.line[s.len] = '\0';
        const bool empty = s.len == 0;
        s.len = 0;
        if (empty || !g_cb.command) continue;
        static char reply[4096];
        const size_t n = g_cb.command(s.line, reply, sizeof(reply) - 1, s.ip);
        if (n) {
            reply[n] = '\n';
            // never block the network task (it also runs the PAR6 link): a client whose receive window is
            // full has stopped reading, so it is dropped instead of waited for
            const int fd = s.sock.fd();
            const int sent = fd >= 0 ? ::send(fd, reply, n + 1, MSG_DONTWAIT) : -1;
            if (sent != (int)(n + 1)) { s.sock.stop(); return; }
        }
    }
}

bool resolveStream(const char* host, uint32_t nowMs)
{
    if (strcmp(host, g_streamHost) == 0 && g_streamIp != IPAddress()) return true;
    if (strcmp(host, g_streamHost) == 0 && nowMs - g_lastResolveMs < 5000) return false;   // failed recently
    strncpy(g_streamHost, host, sizeof(g_streamHost) - 1);
    g_streamHost[sizeof(g_streamHost) - 1] = '\0';
    g_lastResolveMs = nowMs;
    IPAddress ip;
    if (!ip.fromString(host)) {
        // names block this task for up to 1 s, outside the App lock; use an IP address while teleoperating
        const size_t n = strlen(host);
        if (n > 6 && n < 40 && !strcasecmp(host + n - 6, ".local")) {
            char name[40];
            memcpy(name, host, n - 6);
            name[n - 6] = '\0';
            ip = MDNS.queryHost(name, 1000);
        } else if (WiFi.hostByName(host, ip) != 1) {
            ip = IPAddress();
        }
    }
    g_streamIp = ip;
    return ip != IPAddress();
}

void stream(uint32_t nowMs)
{
    if (!g_streamOk || !g_cb.leaderFrame) return;
    g_lastFrameMs = nowMs;
    // every frame the control task left since the last pass (one per tick), at most a ring's worth
    for (int k = 0; k < 8; ++k) {
        uint8_t buf[kLeaderFrameSize + 8];
        char host[40] = "";
        uint16_t port = 0;
        const size_t n = g_cb.leaderFrame(buf, sizeof(buf), host, sizeof(host), &port);
        if (!n) return;
        if (!host[0] || !port || !resolveStream(host, nowMs)) return;
        if (!g_stream.beginPacket(g_streamIp, port)) return;
        g_stream.write(buf, n);
        if (g_stream.endPacket() == 1) ++g_frames;
    }
}

}  // namespace

void begin(const char* hostname, const Callbacks& cb)
{
    g_cb = cb;
    strncpy(g_hostname, hostname, sizeof(g_hostname) - 1);
    WiFi.onEvent(onEvent);
    ETH.begin();                                    // board defaults: LAN8720, power GPIO12, clock GPIO17 out
}

void poll(uint32_t nowMs)
{
    if (!g_started) {
        if (g_up) startServices();
        return;
    }
    ArduinoOTA.handle();
    acceptClients();
    for (Client& s : g_clients) serveClient(s);
    stream(nowMs);
}

bool up() { return g_up; }
const char* ip() { return g_ip; }
uint32_t framesSent() { return g_frames; }
int linkMbps() { return g_started && ETH.linkUp() ? ETH.linkSpeed() : 0; }
bool fullDuplex() { return g_started && ETH.linkUp() && ETH.fullDuplex(); }
int clients()
{
    int n = 0;
    for (Client& s : g_clients) if (s.sock.connected()) ++n;
    return n;
}

}  // namespace net
}  // namespace raft
