// RAFT leader arm firmware (Olimex ESP32-PoE, Arduino core 2.0.x). Everything runs here; no PC program is
// needed at runtime: all of it is C/C++ on the ESP32.
//   core 1  control task, 250 Hz: collect the 7 servos' replies -> App::controlTick -> SyncWrite goals, next SyncRead
//   core 0  network task: Ethernet services (net.cpp), web UI on port 80 (web_server.cpp), PAR6 link
//           (App::networkTick), leader stream
//   loop()  serial console (the same JSON commands as TCP 5000, plus a shorthand) and a 1 Hz status line
// One mutex around every App call. Strict boot: all 7 servos must answer, there is no bypass;
// torque goes off before the first check (boot.cpp). The control task is on the task watchdog: if it stalls
// for 2 s the ESP32 resets, and the boot sequence switches the torque off again.
// Saving to flash and servo_info run outside the App lock and only while the arm is quiet (App::prepareSave,
// App::maintenanceAllowed).
#include <Arduino.h>
#include <SCServo.h>
#include <esp_freertos_hooks.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <stdarg.h>
#include <string.h>

#include "app.h"
#include "arm_design.h"
#include "boot.h"
#include "config.h"
#include "dynamics.h"
#include "esp_transport.h"
#include "kinematics.h"
#include "loop_stats.h"
#include "model_selftest.h"
#include "net.h"
#include "nvs_store.h"
#include "json_lite.h"
#include "servo_bus.h"
#include "servo_info.h"
#include "version.h"
#include "web_assets.h"
#include "web_bridge.h"
#include "web_esp.h"

// Servo bus: GPIO33 = RX from the servo board, GPIO32 = TX to it, 1 Mbit/s
#define SERVO_RX_PIN 33
#define SERVO_TX_PIN 32
#define SERVO_BAUD   1000000

// Handle buttons, active low (pressed = LOW); -1 = not wired. This build has none: the web UI's Hold PAR6 is the
// clutch and Resume reclaims after an outage. To add them later:
//   enable (deadman): e.g. GPIO4 (UEXT pin 3) to GND, internal pull-up.
//   clutch: BUT1 on the ESP32-PoE board is GPIO34 (input only, pull-up on the board).
// With an enable button the "require_enable" setting makes PAR6 move only while it is held.
#define ENABLE_BUTTON_PIN -1
#define CLUTCH_BUTTON_PIN -1

HardwareSerial servoSerial(2);
HLSCL hlscl;

namespace {

char g_hostname[32] = "raft-leader";   // raft-<arm name> (set_name), fixed at boot from the stored settings
const uint32_t kWatchdogS = 2;              // control task silent this long -> reset (then the boot turns torque off)

// A button state changes after it has read the same for 4 control ticks (16 ms).
struct Button {
    int pin;
    bool state = false;
    uint8_t same = 0;
    explicit Button(int p) : pin(p) {}
    bool read()
    {
        if (pin < 0) return false;
        const bool down = digitalRead(pin) == LOW;
        if (down == state) { same = 0; return state; }
        if (++same >= 4) { state = down; same = 0; }
        return state;
    }
};
Button g_enableBtn(ENABLE_BUTTON_PIN), g_clutchBtn(CLUTCH_BUTTON_PIN);

uint8_t g_ids[raft::kNumJoints];
raft::ServoBus* g_bus = nullptr;
raft::NvsStorage g_storage;
raft::EspTransport g_transport;
raft::App* g_app = nullptr;
SemaphoreHandle_t g_lock = nullptr;
volatile bool g_otaActive = false;
bool g_modelOk = false;

// ---- health figures for the web UI's Health page
// Control loop timing: the control task fills a LoopStats and publishes each finished second here.
portMUX_TYPE g_statsMux = portMUX_INITIALIZER_UNLOCKED;
raft::LoopReport g_loop;
TaskHandle_t g_controlTask = nullptr, g_networkTask = nullptr;
volatile uint32_t g_loopStackFree = 0;
uint32_t g_heapSize = 0;   // read once at start: heap_caps_get_info walks the whole heap with interrupts off, and an
                           // interrupt held off on core 0 delays the control task's wake-up on core 1 (2026-11-03)

raft::LoopReport loopReport()
{
    portENTER_CRITICAL(&g_statsMux);
    const raft::LoopReport r = g_loop;
    portEXIT_CRITICAL(&g_statsMux);
    return r;
}

// CPU load per core: each core's idle task calls its hook over and over (the hook returns false, so the idle
// task spins instead of sleeping until the next interrupt). The time between two calls close together is idle
// time; a longer gap means a task or an interrupt ran in between. Costs some power, no time anything else needs.
volatile uint32_t g_idleUs[2] = {0, 0};
uint32_t g_idlePrevUs[2] = {0, 0};
const uint32_t kIdleGapUs = 20;

template <int Core>
bool idleHook()
{
    const uint32_t t = (uint32_t)esp_timer_get_time();
    const uint32_t d = t - g_idlePrevUs[Core];
    g_idlePrevUs[Core] = t;
    if (d < kIdleGapUs) g_idleUs[Core] = g_idleUs[Core] + d;
    return false;
}

const char* resetReason()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_EXT: return "reset pin";
    case ESP_RST_SW: return "restart (update or command)";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT: return "brownout (supply dipped)";
    case ESP_RST_SDIO: return "SDIO";
    default: return "unknown";
    }
}

struct AppLock {
    AppLock() { xSemaphoreTake(g_lock, portMAX_DELAY); }
    ~AppLock() { xSemaphoreGive(g_lock); }
};

double nowS() { return (double)esp_timer_get_time() * 1e-6; }

// ---- the UDP leader stream: the control task packs every tick's frame into this ring (only while a stream host is
// set), the network task sends what is in it, so the stream carries each tick exactly once at 250 Hz
const int kStreamRing = 8;                     // 32 ms of frames; older ones are dropped if the network task lags
const size_t kRingSlot = raft::kLeaderFrameSize + 8;
uint8_t* g_ring = nullptr;                     // kStreamRing slots, from the heap at start (static RAM is full)
uint16_t g_ringLen[kStreamRing];
volatile uint32_t g_ringW = 0, g_ringR = 0;
volatile bool g_streamOn = false;

// ---- servo_info: the control task owns the bus, so it reads the register tables between two ticks when
// asked. The asking task waits without the App lock (the control task needs that lock for its tick).
raft::ServoRegisters g_regs[raft::kNumJoints];
volatile bool g_regsWanted = false, g_regsReady = false;

bool isCmd(const char* json, const char* name)
{
    char c[16] = "";
    return raft::json::getString(json, "cmd", c, sizeof(c)) && !strcmp(c, name);
}

size_t refuse(char* reply, size_t cap, const char* why)
{
    const int n = snprintf(reply, cap, "{\"ok\": false, \"error\": \"%s\"}", why);
    return n > 0 ? (size_t)n : 0;
}

size_t servoInfo(const char* json, char* reply, size_t cap)
{
    {
        char why[120];
        AppLock l;
        if (!g_app->maintenanceAllowed(why, sizeof(why))) return refuse(reply, cap, why);   // it pauses the bus ~8 ms
    }
    g_regsReady = false;
    g_regsWanted = true;
    for (int k = 0; k < 60 && !g_regsReady; ++k) vTaskDelay(pdMS_TO_TICKS(5));
    if (!g_regsReady) {
        g_regsWanted = false;
        const int n = snprintf(reply, cap, "{\"ok\": false, \"error\": \"the control task did not read the servos within 0.3 s\"}");
        return n > 0 ? (size_t)n : 0;
    }
    double j = 0.0;
    raft::json::getNumber(json, "joint", j);
    return raft::formatServoInfo(g_regs, (int)j, reply, cap);
}

// "save" writes NVS without holding the App lock, so the control task keeps its ticks except while the flash
// itself is busy (both cores stall then, tens of ms): the App only allows it while the arm is quiet.
size_t saveSettings(char* reply, size_t cap)
{
    raft::PersistentConfig snap;
    char why[120];
    {
        AppLock l;
        if (!g_app->prepareSave(snap, why, sizeof(why))) return refuse(reply, cap, why);
    }
    const bool ok = g_storage.save(snap);
    {
        AppLock l;
        g_app->finishSave(snap, ok);
    }
    const int n = ok ? snprintf(reply, cap, "{\"ok\": true, \"saved\": true}") : snprintf(reply, cap, "{\"ok\": false, \"error\": \"flash write failed\"}");
    return n > 0 ? (size_t)n : 0;
}

// "status" copies the App's state under the lock and prints it after (the printing takes milliseconds)
size_t statusCommand(char* reply, size_t cap)
{
    static raft::App::StatusSnapshot snap;   // 1.2 kB; callers of outsideLock never run it at the same time
    static SemaphoreHandle_t mine = xSemaphoreCreateMutex();
    xSemaphoreTake(mine, portMAX_DELAY);
    {
        AppLock l;
        g_app->snapshot(snap);
    }
    const size_t n = raft::App::formatStatus(snap, reply, cap);
    xSemaphoreGive(mine);
    return n;
}

// appends printf output to a reply buffer; size() = 0 once it overflowed
struct Printer {
    char* b; size_t cap, n = 0;
    Printer(char* buf, size_t c) : b(buf), cap(c) {}
    void add(const char* fmt, ...) __attribute__((format(printf, 2, 3)))
    {
        if (n >= cap) return;
        va_list ap;
        va_start(ap, fmt);
        const int m = vsnprintf(b + n, cap - n, fmt, ap);
        va_end(ap);
        n = m < 0 ? cap : n + (size_t)m;
    }
    size_t size() const { return n < cap ? n : 0; }
};

// The servo bus diagnosis (servo_bus.h): {"cmd": "bus_diag"} the counts per servo; "trace": N watches the replies of
// the next N SyncReads come in; "capture": k one recorded SyncRead (0..3 newest first that missed a servo, 4 a clean one).
// {"cmd": "bus_retry", "on": true|false} switches the in-tick retry. Neither needs the App lock.
size_t busDiag(const char* json, char* reply, size_t cap)
{
    static raft::ServoBus::Diag d;
    static raft::ServoBus::Capture caps[raft::ServoBus::kCaptures], clean;
    int nCaps = 0;
    bool haveClean = false;
    uint32_t traceLeft = 0;
    double v = 0.0;
    if (raft::json::getNumber(json, "trace", v) && v > 0) g_bus->trace((uint32_t)(v < 5000 ? v : 5000));
    g_bus->diagnostics(d, caps, nCaps, clean, haveClean, traceLeft);
    Printer o(reply, cap);
    if (raft::json::getNumber(json, "capture", v)) {
        const int k = (int)v;
        const raft::ServoBus::Capture* c = k >= 0 && k < nCaps ? &caps[k] : (k == raft::ServoBus::kCaptures && haveClean ? &clean : nullptr);
        if (!c) return refuse(reply, cap, "no such capture yet");
        o.add("{\"ok\": true, \"read\": %lu, \"missing\": [", (unsigned long)c->read);
        bool first = true;
        for (int i = 0; i < raft::kNumJoints; ++i) if ((c->missing >> i) & 1u) { o.add("%s%d", first ? "" : ", ", i + 1); first = false; }
        o.add("], \"len\": %u, \"hex\": \"", (unsigned)c->len);
        for (int b = 0; b < c->len; ++b) o.add("%02x", c->b[b]);
        o.add("\", \"trace\": [");
        for (int t = 0; t < c->traceN; ++t) o.add("%s[%u, %u]", t ? ", " : "", (unsigned)c->traceUs[t], (unsigned)c->traceLen[t]);
        o.add("]}");
        return o.size();
    }
    o.add("{\"ok\": true, \"full_read\": %s, \"servo_poll_ms\": %lu, \"retry\": %s, \"reads\": %lu, \"stray_bytes\": %lu, \"captures\": %d, \"clean\": %s, \"trace_left\": %lu, \"servos\": [",
        g_bus->fullRead() ? "true" : "false", (unsigned long)g_bus->slowIntervalMs(), g_bus->retry() ? "true" : "false", (unsigned long)d.reads, (unsigned long)d.strayBytes, nCaps, haveClean ? "true" : "false",
        (unsigned long)traceLeft);
    for (int i = 0; i < raft::kNumJoints; ++i)
        o.add("%s{\"j\": %d, \"id\": %u, \"missed\": %lu, \"silent\": %lu, \"garbled\": %lu, \"runs\": [%lu, %lu, %lu], \"longest\": %lu, "
            "\"flagged\": %lu, \"flags\": %u, \"retried\": %lu, \"recovered\": %lu}",
            i ? ", " : "", i + 1, (unsigned)g_bus->ids()[i], (unsigned long)d.missed[i], (unsigned long)d.silent[i], (unsigned long)d.garbled[i],
            (unsigned long)d.runs[i][0], (unsigned long)d.runs[i][1], (unsigned long)d.runs[i][2], (unsigned long)d.longest[i],
            (unsigned long)d.flagged[i], (unsigned)d.lastFlags[i], (unsigned long)d.retried[i], (unsigned long)d.recovered[i]);
    o.add("]}");
    return o.size();
}

// {"cmd": "servo_poll", "ms": 2000}: voltage, temperature and current of each servo every this many ms (servo_bus.h)
size_t servoPoll(const char* json, char* reply, size_t cap)
{
    double v = 0.0;
    if (raft::json::getNumber(json, "ms", v)) g_bus->setSlowInterval((uint32_t)(v < 0 ? 0 : v));
    const int n = snprintf(reply, cap, "{\"ok\": true, \"servo_poll_ms\": %lu}", (unsigned long)g_bus->slowIntervalMs());
    return n > 0 ? (size_t)n : 0;
}

// {"cmd": "bus_read", "full": true|false}: what the SyncRead covers (servo_bus.h)
size_t busRead(const char* json, char* reply, size_t cap)
{
    bool on = true;
    if (raft::json::getBool(json, "full", on)) g_bus->setFullRead(on);
    const int n = snprintf(reply, cap, "{\"ok\": true, \"full_read\": %s}", g_bus->fullRead() ? "true" : "false");
    return n > 0 ? (size_t)n : 0;
}

size_t busRetry(const char* json, char* reply, size_t cap)
{
    bool on = true;
    if (!raft::json::getBool(json, "on", on)) return refuse(reply, cap, "bus_retry needs on: true or false");
    g_bus->setRetry(on);
    const int n = snprintf(reply, cap, "{\"ok\": true, \"retry\": %s}", on ? "true" : "false");
    return n > 0 ? (size_t)n : 0;
}

// commands that must not run under the App lock; 0 = not one of them
size_t outsideLock(const char* json, char* reply, size_t cap)
{
    if (isCmd(json, "status")) return statusCommand(reply, cap);
    if (isCmd(json, "bus_diag")) return busDiag(json, reply, cap);
    if (isCmd(json, "bus_retry")) return busRetry(json, reply, cap);
    if (isCmd(json, "servo_poll")) return servoPoll(json, reply, cap);
    if (isCmd(json, "bus_read")) return busRead(json, reply, cap);
    if (isCmd(json, "servo_info")) return servoInfo(json, reply, cap);
    if (isCmd(json, "save")) return saveSettings(reply, cap);
    return 0;
}

// The web UI's view of the App: same lock as everything else, plus the system figures for the Health page.
class EspWebBridge : public raft::WebBridge {
public:
    explicit EspWebBridge(raft::App& app) : WebBridge(app, "RAFT " RAFT_VERSION " (ESP32, built " __DATE__ " " __TIME__ ")", false) {}
    raft::web::Server* server = nullptr;

protected:
    void lock() override { xSemaphoreTake(g_lock, portMAX_DELAY); }
    void unlock() override { xSemaphoreGive(g_lock); }
    double now() override { return nowS(); }
    size_t outsideLockCommand(const char* json, char* reply, size_t cap) override { return outsideLock(json, reply, cap); }
    size_t sys(char* out, size_t cap) override
    {
        const raft::LoopReport r = loopReport();
        char loop[420];
        if (!raft::formatLoopReport(r, loop, sizeof(loop))) loop[0] = '\0';
        const int n = snprintf(out, cap,
                               "\"uptime_s\": %lu, \"reset\": \"%s\", \"cpu_mhz\": %lu, \"cpu_load\": [%.0f, %.0f], \"loop\": {%s}, "
                               "\"heap\": {\"free\": %u, \"min\": %u, \"size\": %u}, "
                               "\"stack_free\": {\"control\": %u, \"network\": %u, \"loop\": %u}, "
                               "\"net\": {\"ip\": \"%s\", \"link_mbps\": %d, \"full_duplex\": %s, \"web_clients\": %d, \"tcp_clients\": %d, "
                               "\"web_frames\": %lu, \"udp_frames\": %lu}",
                               (unsigned long)(millis() / 1000), resetReason(), (unsigned long)getCpuFrequencyMhz(), (double)r.cpuLoad[0],
                               (double)r.cpuLoad[1], loop, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                               (unsigned)g_heapSize, g_controlTask ? (unsigned)uxTaskGetStackHighWaterMark(g_controlTask) : 0u,
                               g_networkTask ? (unsigned)uxTaskGetStackHighWaterMark(g_networkTask) : 0u, (unsigned)g_loopStackFree,
                               raft::net::up() ? raft::net::ip() : "down", raft::net::linkMbps(), raft::net::fullDuplex() ? "true" : "false",
                               server ? server->wsClients() : 0, raft::net::clients(), server ? (unsigned long)server->framesSent() : 0ul,
                               (unsigned long)raft::net::framesSent());
        return n > 0 && (size_t)n < cap ? (size_t)n : 0;
    }
    size_t helloSys(char* out, size_t cap) override
    {
        const int n = snprintf(out, cap, "\"reset\": \"%s\", \"cpu_mhz\": %lu, \"ip\": \"%s\", \"host\": \"%s.local\"", resetReason(),
                               (unsigned long)getCpuFrequencyMhz(), raft::net::up() ? raft::net::ip() : "down", g_hostname);
        return n > 0 && (size_t)n < cap ? (size_t)n : 0;
    }
    // the health message (web_bridge.h): counters only, no printing
    bool healthFigures(raft::HealthFigures& f) override
    {
        f.loop = loopReport();
        f.uptimeS = (uint32_t)(millis() / 1000);
        f.heapFree = (uint32_t)ESP.getFreeHeap(); f.heapMin = (uint32_t)ESP.getMinFreeHeap(); f.heapSize = g_heapSize;
        f.stackControl = g_controlTask ? (uint32_t)uxTaskGetStackHighWaterMark(g_controlTask) : 0u;
        f.stackNetwork = g_networkTask ? (uint32_t)uxTaskGetStackHighWaterMark(g_networkTask) : 0u;
        f.stackLoop = g_loopStackFree;
        f.linkMbps = (int16_t)raft::net::linkMbps();
        f.flags = raft::HF_HEAP | raft::HF_STACKS | raft::HF_TCP | raft::HF_UDP | (raft::net::up() ? raft::HF_NET_UP : 0) |
                  (raft::net::fullDuplex() ? raft::HF_FULL_DUPLEX : 0);
        f.webClients = server ? (uint8_t)server->wsClients() : 0;
        f.tcpClients = (uint8_t)raft::net::clients();
        f.webFrames = server ? server->framesSent() : 0u;
        f.udpFrames = (uint32_t)raft::net::framesSent();
        return true;
    }
};

raft::EspWebListener g_webListener(80);
EspWebBridge* g_webBridge = nullptr;
raft::web::Server* g_web = nullptr;
volatile bool g_webReady = false;

// Boot self-test of the on-device kinematics/gravity model against the golden vectors generated by
// tools/gen_model.py. Torque stays off when it fails. Also times one FK + gravity call.
void checkModel()
{
    const uint32_t t0 = micros();
    const raft::ModelSelfTestResult r = raft::runModelSelfTest();
    const uint32_t t1 = micros();
    Serial.printf("Model self-test: %s  (%d cases, TCP %.1e m, gravity %.1e Nm, rpy %.1e rad, %lu us)\n",
                  r.pass ? "PASS" : "FAIL", r.cases, r.maxPosErr, r.maxTauErr, r.maxRpyErr, (unsigned long)(t1 - t0));
    float q[raft::kNumJoints] = {0.3f, 0.4f, -0.5f, 0.6f, -0.7f, 0.8f, 0.2f};
    float tau[raft::kNumJoints];
    raft::Frame T[raft::kNumFrames];
    const uint32_t t2 = micros();
    for (int k = 0; k < 100; ++k) {
        q[0] += 1e-4f;
        raft::forwardKinematics(q, T);
        raft::gravityTorque(T, tau);
    }
    Serial.printf("FK + gravity: %.1f us per call (J2 %.3f Nm)\n", (micros() - t2) / 100.0f, tau[1]);
    g_modelOk = r.pass;
}

// ---- control task (core 1)
// Woken by hardware timer 0 every 4 ms. Its interrupt is allocated on core 1 (the control task starts it there), so the
// wake-up does not wait for core 0: the FreeRTOS tick that xTaskDelayUntil uses is processed by core 0, and core 0's
// interrupt latency under network load moved the ticks by up to 0.7 ms (2026-11-03).
TaskHandle_t g_tickTask = nullptr;
hw_timer_t* g_tickTimer = nullptr;

void IRAM_ATTR onTickTimer()
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_tickTask, &woken);
    if (woken) portYIELD_FROM_ISR();
}

void controlTask(void*)
{
    raft::RawServoState raw;
    memset(&raw, 0, sizeof(raw));
    int16_t out[raft::kNumJoints];
    const int16_t zeros[raft::kNumJoints] = {0};
    bool torqueOn = false;
    esp_task_wdt_init(kWatchdogS, true);   // IDF 4.4: reconfigures the task watchdog if it already runs
    esp_task_wdt_add(nullptr);
    raft::LoopStats stats(raft::kControlPeriodUs);
    uint32_t idle0[2] = {g_idleUs[0], g_idleUs[1]}, windowStart = micros();
    g_tickTask = xTaskGetCurrentTaskHandle();
    g_tickTimer = timerBegin(0, 80, true);                 // 1 MHz
    timerAttachInterrupt(g_tickTimer, &onTickTimer, true);
    timerAlarmWrite(g_tickTimer, raft::kControlPeriodUs, true);
    timerAlarmEnable(g_tickTimer);
    for (;;) {
        // more than one pending: the last tick ran past a timer period (an overrun); its catch-up ticks are dropped
        const uint32_t pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        const bool late = pending != 1;
        const uint32_t t0 = micros();
        stats.wake(t0, late);
        esp_task_wdt_reset();
        // the servos' replies to the SyncRead sent at the end of the last tick (servo_bus.h): already in, no waiting
        if (g_bus->collectSync(raw) != 0x7f) stats.failedRead();
        const bool regs = g_regsWanted;
        if (regs) {                        // servo_info: ~8 ms once; the servos keep their last current meanwhile
            for (int i = 0; i < raft::kNumJoints; ++i) {
                g_regs[i].id = g_bus->ids()[i];
                g_regs[i].ok = g_bus->readRegisters(i, g_regs[i].r);
            }
            g_regsWanted = false;
            g_regsReady = true;
        } else {
            g_bus->requestSlow();          // one servo's voltage, temperature, current: comes in during the App tick
        }
        const bool enableDown = g_enableBtn.read(), clutchDown = g_clutchBtn.read();
        bool on = false, needMode = false, haveOp;
        raft::App::ServoGoals goals;
        raft::App::ServoTestOp op;
        const uint32_t tRead = micros();
        uint32_t tApp;
        {
            AppLock l;
            tApp = micros();
            g_app->setButtons(enableDown, clutchDown);
            g_app->setServoModes(g_bus->positionMask());
            g_app->controlTick(raw, nowS(), out, on, needMode);
            goals = g_app->servoGoals();
            haveOp = g_app->takeServoTestOp(op);   // servo position test (allowed only while the arm is limp)
            g_streamOn = g_app->streamHost()[0] != '\0';
            if (g_streamOn && g_ring) {
                const uint32_t w = g_ringW;
                g_ringLen[w % kStreamRing] = (uint16_t)g_app->leaderFrame(g_ring + (w % kStreamRing) * kRingSlot, kRingSlot, 0);
                __sync_synchronize();               // the frame before the index, for the other core
                g_ringW = w + 1;
            }
            g_bus->setSlowFocus(g_app->servoTestJoint());   // its live current: read it every 8 ticks
        }
        const uint32_t tSlow = micros();
        g_bus->collectSlow(raw);
        const uint32_t tWrite = micros();
        if (g_otaActive) on = false;
        if (on && !torqueOn) {
            g_bus->writeCurrent(zeros);
            if (needMode && !g_bus->currentMode()) {
                AppLock l;
                g_app->reportFault("a servo refused constant-current mode");
                on = false;
            } else {
                g_bus->torque(true);
                torqueOn = true;
            }
        } else if (!on && torqueOn) {
            g_bus->writeCurrent(zeros);
            g_bus->torque(false);
            torqueOn = false;
        }
        if (torqueOn) {   // position mode for the joints the App holds that way, goal currents for the others
            const uint8_t pos = g_bus->setPositionMask(goals.mask, goals.raw, goals.speed, goals.acc, goals.limit);
            g_bus->writeGoals(goals.raw, pos);
            g_bus->writeCurrent(out, (uint8_t)(~pos & 0x7f));
        }
        if (haveOp) g_bus->servoTest(op.joint, op.step, op.goalRaw, op.speed, op.acc, op.torque);
        g_bus->requestSync();              // after the writes; the replies come in while this task sleeps
        if (regs) {
            ulTaskNotifyTake(pdTRUE, 0);   // do not count the servo_info pause as overruns, nor as a period
            stats.skipPeriod();
        }
        const uint32_t tEnd = micros();
        stats.part(raft::kPartRead, (tRead - t0) + (tWrite - tSlow));
        stats.part(raft::kPartLock, tApp - tRead);
        stats.part(raft::kPartApp, tSlow - tApp);
        stats.part(raft::kPartWrite, tEnd - tWrite);
        if (stats.done(tEnd)) {
            raft::LoopReport r = stats.report();
            const uint32_t span = tEnd - windowStart;
            for (int c = 0; c < 2; ++c) {
                const uint32_t idle = g_idleUs[c];
                r.cpuLoad[c] = 100.0f * (1.0f - (float)(idle - idle0[c]) / (float)span);
                if (r.cpuLoad[c] < 0.0f) r.cpuLoad[c] = 0.0f;
                idle0[c] = idle;
            }
            windowStart = tEnd;
            portENTER_CRITICAL(&g_statsMux);
            g_loop = r;
            portEXIT_CRITICAL(&g_statsMux);
        }
    }
}

// ---- network callbacks (network task, core 0)
size_t onCommand(const char* line, char* reply, size_t cap, const char* clientIp)
{
    const size_t direct = outsideLock(line, reply, cap);
    if (direct) return direct;
    AppLock l;
    return g_app->handleCommand(line, reply, cap, nowS(), clientIp);
}

// the next frame of the ring (0 = none waiting), and where it goes
size_t onLeaderFrame(uint8_t* buf, size_t cap, char* host, size_t hostCap, uint16_t* port)
{
    if (!g_streamOn || !g_ring) { g_ringR = g_ringW; return 0; }
    uint32_t r = g_ringR;
    const uint32_t w = g_ringW;
    if (r == w) return 0;
    if (w - r > (uint32_t)kStreamRing) r = w - kStreamRing;   // fell behind: skip the overwritten ones
    __sync_synchronize();
    const size_t n = g_ringLen[r % kStreamRing] < cap ? g_ringLen[r % kStreamRing] : cap;
    memcpy(buf, g_ring + (r % kStreamRing) * kRingSlot, n);
    g_ringR = r + 1;
    {
        AppLock l;
        strncpy(host, g_app->streamHost(), hostCap - 1);
        host[hostCap - 1] = '\0';
        *port = g_app->streamPort();
    }
    return n;
}

void onOtaStarting()
{
    g_otaActive = true;
    AppLock l;
    char r[160];
    g_app->handleCommand("{\"cmd\": \"mode\", \"mode\": \"limp\"}", r, sizeof(r), nowS(), nullptr);
}

void onNetworkUp()
{
    if (!g_transport.begin()) Serial.println("PAR6 link transport: could not bind UDP 30002");
    g_webListener.begin();
    g_webReady = true;
    Serial.printf("Web UI: http://%s/  or  http://%s.local/\n", raft::net::ip(), g_hostname);
}

void networkTask(void*)
{
    for (;;) {
        raft::net::poll(millis());
        if (g_webReady) g_web->poll(nowS());      // takes the App lock itself, per call
        {
            AppLock l;
            g_app->networkTick(nowS());
        }
        vTaskDelay(1);
    }
}

// ---- serial console (loop task)
// Accepts the JSON commands of docs/teleop_protocol.md, or a shorthand "cmd [arg]":
//   status | save | calib_stop 3 | mode gravity | calib_torque 2 | teleop_start 192.168.1.20
// plus "quiet" to toggle the 1 Hz status line.
bool g_quiet = false;

void serialCommand(char* line)
{
    while (*line == ' ') ++line;
    size_t n = strlen(line);
    while (n && line[n - 1] == ' ') line[--n] = '\0';
    if (!n) return;
    if (!strcmp(line, "quiet")) { g_quiet = !g_quiet; Serial.printf("status line %s\n", g_quiet ? "off" : "on"); return; }
    char json[200];
    const char* cmd = line;
    if (line[0] != '{') {
        char* arg = strchr(line, ' ');
        if (arg) { *arg++ = '\0'; while (*arg == ' ') ++arg; }
        if (!arg || !*arg) {
            snprintf(json, sizeof(json), "{\"cmd\": \"%s\"}", line);
        } else if ((arg[0] >= '0' && arg[0] <= '9') || arg[0] == '-' || arg[0] == '.') {
            const bool joint = !strcmp(line, "calib_stop") || !strcmp(line, "calib_torque") || !strcmp(line, "servo_info");
            snprintf(json, sizeof(json), "{\"cmd\": \"%s\", \"%s\": %s}", line, joint ? "joint" : "value", arg);
        } else {
            const char* key = !strcmp(line, "mode") ? "mode" : (!strcmp(line, "spring") ? "type" : "host");
            snprintf(json, sizeof(json), "{\"cmd\": \"%s\", \"%s\": \"%s\"}", line, key, arg);
        }
        cmd = json;
    }
    static char reply[4096];
    size_t r = outsideLock(cmd, reply, sizeof(reply));
    if (!r) {
        AppLock l;
        r = g_app->handleCommand(cmd, reply, sizeof(reply), nowS(), nullptr);
    }
    if (r) Serial.println(reply);
}

// The strict boot sequence (boot.cpp) on the real bus
class EspBootBus : public raft::BootBus {
public:
    explicit EspBootBus(raft::ServoBus& b) : bus_(b) {}
    void torqueOffAll() override
    {
        const int16_t zeros[raft::kNumJoints] = {0};
        bus_.writeCurrent(zeros);
        bus_.torque(false);
    }
    uint8_t missing() override { return bus_.pingAll(); }
    void waitMs(uint32_t ms) override { delay(ms); }
    uint32_t nowMs() override { return millis(); }

private:
    raft::ServoBus& bus_;
};

}  // namespace

void setup()
{
    Serial.begin(115200);
    // the servo bus first: after an ESP32-only reset the servos are still powered and keep their last goal
    // current, so torque goes off before anything else (no reply needed; a servo still powering up misses it,
    // the boot sequence repeats it)
    servoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    hlscl.pSerial = &servoSerial;
    for (int i = 0; i < raft::kNumJoints; i++) g_ids[i] = JOINTS[i].id;
    static raft::ServoBus bus(hlscl, g_ids);
    g_bus = &bus;
    bus.begin();
    static EspBootBus bootBus(bus);
    bootBus.torqueOffAll();

    const uint32_t t = millis();
    while (!Serial && (millis() - t < 3000)) delay(10);
    delay(200);
    Serial.println();
    Serial.println("=== RAFT leader arm " RAFT_VERSION " booting ===");
    Serial.println("Servo bus up, torque OFF sent to every servo.");

    checkModel();

    Serial.println("Waiting for all servos...");
    const uint8_t missing = raft::strictBoot(bootBus, 10000, 500);
    if (missing) {
        // Strict boot: halt with torque off and keep saying what is missing.
        for (;;) {
            bootBus.torqueOffAll();
            Serial.print("BOOT HALTED: not all servos answer. Missing:");
            for (int i = 0; i < raft::kNumJoints; i++)
                if (missing & (1u << i)) { Serial.print(' '); Serial.print(JOINTS[i].name); Serial.print(" (ID "); Serial.print(g_ids[i]); Serial.print(')'); }
            Serial.println();
            delay(2000);
        }
    }
    Serial.println("All servos online, torque OFF.");

    if (ENABLE_BUTTON_PIN >= 0) pinMode(ENABLE_BUTTON_PIN, INPUT_PULLUP);
    if (CLUTCH_BUTTON_PIN >= 0) pinMode(CLUTCH_BUTTON_PIN, CLUTCH_BUTTON_PIN >= 34 ? INPUT : INPUT_PULLUP);   // GPIO34-39: no internal pull-up

    g_lock = xSemaphoreCreateMutex();
    static raft::App app(g_storage, g_transport);
    g_app = &app;
    app.begin(nowS(), g_modelOk);
    app.setButtonPresence(ENABLE_BUTTON_PIN >= 0, CLUTCH_BUTTON_PIN >= 0);
    Serial.printf("Settings: %s, %s\n", raft::fullyCalibrated(app.config()) ? "calibrated" : "NOT calibrated (see docs/calibration.md)",
                  g_modelOk ? "model OK" : "MODEL SELF-TEST FAILED: torque stays off");

    static EspWebBridge webBridge(app);
    static const raft::web::Asset assets[] = {
        {"/", "text/html; charset=utf-8", raft::webui::kIndexGz, raft::webui::kIndexGzLen, true, raft::webui::kIndexEtag},
        {"/index.html", "text/html; charset=utf-8", raft::webui::kIndexGz, raft::webui::kIndexGzLen, true, raft::webui::kIndexEtag},
        {"/arm.bin", "application/octet-stream", raft::webui::kArmGz, raft::webui::kArmGzLen, true, raft::webui::kArmEtag},
    };
    // on the heap: its four connections' buffers are 46 kB, and the static RAM region is capped by the build (part of
    // it is held for Bluetooth, which the heap gets back at start)
    g_web = new raft::web::Server(g_webListener, webBridge, assets, (int)(sizeof(assets) / sizeof(assets[0])));
    webBridge.server = g_web;
    g_webBridge = &webBridge;

    const raft::net::Callbacks cb = {onCommand, onLeaderFrame, onOtaStarting, onNetworkUp};
    raft::armHostname(app.config().armName, g_hostname, sizeof(g_hostname));
    raft::net::begin(g_hostname, cb);

    // Priority 20: above the Ethernet driver's receive task (15, no core affinity), which otherwise preempts the servo
    // bus work on core 1 whenever a packet arrives, and above the TCP/IP task (18, core 0): when the control task waits
    // for the App lock, the network task holding it inherits 20 and the TCP/IP task cannot interrupt its copy
    // (at 16 the wait reached 0.9 ms, 2026-11-03).
    g_ring = (uint8_t*)malloc(kStreamRing * kRingSlot);
    g_heapSize = ESP.getHeapSize();
    esp_register_freertos_idle_hook_for_cpu(idleHook<0>, 0);
    esp_register_freertos_idle_hook_for_cpu(idleHook<1>, 1);
    xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 20, &g_controlTask, 1);
    // 12 kB: the status with its Health figures is built on this stack (8 kB left 1.4 kB free, measured 2026-11-03)
    xTaskCreatePinnedToCore(networkTask, "network", 12288, nullptr, 3, &g_networkTask, 0);
    Serial.println("Running. Type 'status', or see docs/teleop_protocol.md. 'quiet' stops the status line.");
}

void loop()
{
    static char line[300];
    static size_t n = 0;
    static uint32_t lastStatus = 0;
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') { line[n] = '\0'; serialCommand(line); n = 0; continue; }
        if (n < sizeof(line) - 1) line[n++] = (char)c;
    }
    if (millis() - lastStatus >= 1000) {
        lastStatus = millis();
        g_loopStackFree = uxTaskGetStackHighWaterMark(nullptr);
        if (!g_quiet) {
            const raft::LoopReport lr = loopReport();
            char s[320];
            static raft::App::StatusSnapshot snap;
            {
                AppLock l;
                g_app->snapshot(snap);
            }
            raft::App::formatStatusLine(snap, s, sizeof(s));
            Serial.printf("%s | net %s, %d TCP + %d web client(s), %lu frames | loop %.1f Hz, max %lu us, read max %lu us, overruns %lu, "
                          "failed reads %lu, CPU %.0f / %.0f %%\n", s,
                          raft::net::up() ? raft::net::ip() : "down", raft::net::clients(), g_web ? g_web->wsClients() : 0,
                          (unsigned long)raft::net::framesSent(), (double)lr.hz, (unsigned long)lr.workMaxUs,
                          (unsigned long)lr.partMaxUs[raft::kPartRead], (unsigned long)lr.overruns, (unsigned long)lr.failedReads,
                          (double)lr.cpuLoad[0], (double)lr.cpuLoad[1]);
        }
    }
    delay(5);
}
