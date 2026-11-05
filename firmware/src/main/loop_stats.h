// Control loop timing in one-second windows, for the web UI's Health page: tick rate, period (min, mean, max and
// the RMS deviation from the nominal period, the jitter), the work per tick split into its parts, overruns and
// failed bus reads. Portable: the ESP32 control task and web_sim feed it microsecond timestamps.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace raft {

// the parts of one control tick
enum LoopPart { kPartRead, kPartLock, kPartApp, kPartWrite, kNumLoopParts };

struct LoopReport {              // one finished window
    uint32_t seq = 0;            // counts windows; 0 = none yet
    uint32_t ticks = 0;
    float targetHz = 0.0f;       // the nominal rate
    float hz = 0.0f;
    uint32_t periodMinUs = 0, periodMaxUs = 0;
    float periodMeanUs = 0.0f, jitterRmsUs = 0.0f;
    float workMeanUs = 0.0f;
    uint32_t workMaxUs = 0;
    float partMeanUs[kNumLoopParts] = {};
    uint32_t partMaxUs[kNumLoopParts] = {};
    uint32_t overruns = 0, failedReads = 0, overrunsTotal = 0, failedReadsTotal = 0;
    float cpuLoad[2] = {-1.0f, -1.0f};   // percent per core, -1 = not measured
};

class LoopStats {
public:
    explicit LoopStats(uint32_t nominalUs, uint32_t windowUs = 1000000) : nominalUs_(nominalUs), windowUs_(windowUs) {}

    // start of a tick; overrun = the tick came late (the previous one ran past its period)
    void wake(uint32_t tUs, bool overrun);
    // the period up to the next wake is not a normal one (e.g. the servo_info pause): leave it out
    void skipPeriod() { havePrev_ = false; }
    void failedRead() { ++w_.failedReads; }
    void part(LoopPart p, uint32_t us) { w_.part[p].add(us); }
    // end of a tick; true when it closed a window, which report() then holds
    bool done(uint32_t tUs);
    const LoopReport& report() const { return r_; }

private:
    struct Span {
        uint32_t max = 0;
        uint64_t sum = 0;
        void add(uint32_t v) { sum += v; if (v > max) max = v; }
    };
    struct Window {
        uint32_t ticks = 0, periods = 0, periodMin = 0xffffffffu, overruns = 0, failedReads = 0;
        Span period, work, part[kNumLoopParts];
        uint64_t devSq = 0;      // sum of (period - nominal)^2, us^2
    };
    uint32_t nominalUs_, windowUs_;
    Window w_;
    LoopReport r_;
    uint32_t wakeUs_ = 0, prevWakeUs_ = 0, startUs_ = 0;
    bool havePrev_ = false, started_ = false;
};

// the body of the status's "loop" object: {"seq": .., "target_hz": .., "hz": .., "period_us": [min, mean, max], "jitter_us": ..,
// "work_us": [mean, max], "read_us", "lock_us", "app_us", "write_us": [mean, max], "overruns": [last s, total],
// "failed_reads": [last s, total]}, without the braces; 0 when it does not fit
size_t formatLoopReport(const LoopReport& r, char* out, size_t cap);

}  // namespace raft
