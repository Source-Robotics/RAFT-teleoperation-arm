#include "loop_stats.h"

#include <math.h>
#include <stdio.h>

namespace raft {

void LoopStats::wake(uint32_t tUs, bool overrun)
{
    if (!started_) { startUs_ = tUs; started_ = true; }
    if (havePrev_) {
        const uint32_t p = tUs - prevWakeUs_;
        w_.period.add(p);
        if (p < w_.periodMin) w_.periodMin = p;
        const int64_t d = (int64_t)p - (int64_t)nominalUs_;
        w_.devSq += (uint64_t)(d * d);
        ++w_.periods;
    }
    if (overrun) ++w_.overruns;
    prevWakeUs_ = wakeUs_ = tUs;
    havePrev_ = true;
}

bool LoopStats::done(uint32_t tUs)
{
    w_.work.add(tUs - wakeUs_);
    ++w_.ticks;
    const uint32_t span = tUs - startUs_;
    if (span < windowUs_) return false;
    LoopReport r;
    r.seq = r_.seq + 1;
    r.ticks = w_.ticks;
    r.targetHz = (float)(1e6 / nominalUs_);
    r.hz = (float)(w_.ticks * 1e6 / span);
    if (w_.periods) {
        r.periodMinUs = w_.periodMin;
        r.periodMaxUs = w_.period.max;
        r.periodMeanUs = (float)((double)w_.period.sum / w_.periods);
        r.jitterRmsUs = (float)sqrt((double)w_.devSq / w_.periods);
    }
    r.workMeanUs = (float)((double)w_.work.sum / w_.ticks);
    r.workMaxUs = w_.work.max;
    for (int k = 0; k < kNumLoopParts; ++k) {
        r.partMeanUs[k] = (float)((double)w_.part[k].sum / w_.ticks);
        r.partMaxUs[k] = w_.part[k].max;
    }
    r.overruns = w_.overruns;
    r.failedReads = w_.failedReads;
    r.overrunsTotal = r_.overrunsTotal + w_.overruns;
    r.failedReadsTotal = r_.failedReadsTotal + w_.failedReads;
    r_ = r;
    w_ = Window();
    startUs_ = tUs;
    return true;
}

size_t formatLoopReport(const LoopReport& r, char* out, size_t cap)
{
    const int n = snprintf(out, cap,
                           "\"seq\": %lu, \"target_hz\": %.0f, \"hz\": %.1f, \"period_us\": [%lu, %.0f, %lu], \"jitter_us\": %.1f, \"work_us\": [%.0f, %lu], "
                           "\"read_us\": [%.0f, %lu], \"lock_us\": [%.0f, %lu], \"app_us\": [%.0f, %lu], \"write_us\": [%.0f, %lu], "
                           "\"overruns\": [%lu, %lu], \"failed_reads\": [%lu, %lu]",
                           (unsigned long)r.seq, (double)r.targetHz, (double)r.hz, (unsigned long)r.periodMinUs, (double)r.periodMeanUs, (unsigned long)r.periodMaxUs,
                           (double)r.jitterRmsUs, (double)r.workMeanUs, (unsigned long)r.workMaxUs,
                           (double)r.partMeanUs[kPartRead], (unsigned long)r.partMaxUs[kPartRead],
                           (double)r.partMeanUs[kPartLock], (unsigned long)r.partMaxUs[kPartLock],
                           (double)r.partMeanUs[kPartApp], (unsigned long)r.partMaxUs[kPartApp],
                           (double)r.partMeanUs[kPartWrite], (unsigned long)r.partMaxUs[kPartWrite],
                           (unsigned long)r.overruns, (unsigned long)r.overrunsTotal, (unsigned long)r.failedReads, (unsigned long)r.failedReadsTotal);
    return n > 0 && (size_t)n < cap ? (size_t)n : 0;
}

}  // namespace raft
