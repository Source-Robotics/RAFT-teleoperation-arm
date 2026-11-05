#include "servo_bus.h"

namespace raft {
namespace {

const uint8_t kFirstReg = HLSCL_PRESENT_POSITION_L;                               // 56
const uint8_t kShortLen = HLSCL_PRESENT_SPEED_H - HLSCL_PRESENT_POSITION_L + 1;    // 56..59 = 4 bytes: position, speed
const uint8_t kFullLen = HLSCL_PRESENT_CURRENT_H - HLSCL_PRESENT_POSITION_L + 1;    // 56..70 = 15 bytes: ... to current
const uint8_t kSlowReg = HLSCL_PRESENT_VOLTAGE;                                   // 62
const uint8_t kSlowLen = HLSCL_PRESENT_CURRENT_H - HLSCL_PRESENT_VOLTAGE + 1;     // 62..70 = 9 bytes, one servo

const uint8_t kInstRead = 0x02, kInstSyncRead = 0x82, kBroadcast = 0xfe;
// reply timing (measured 2026-11-02): the first byte 0.3 ms after the request, then ~21 us per byte, ~20 us between
// the replies of a SyncRead; a reply that has not come this long after it was due does not come any more
const uint32_t kFirstReplyUs = 300, kByteUs = 21, kGapUs = 20, kLateUs = 400;
// the SyncRead's replies as measured (bus_diag trace, 2026-11-03): the first byte ~0.2 ms after the request, 10 us per
// byte on the wire and ~60 us between replies; a little more is allowed per byte
const uint32_t kSyncFirstUs = 200, kSyncByteUs = 12, kSyncGapUs = 70;

// Sign-magnitude register word, bit 15 = sign, little-endian (as the driver's syncReadRxPacketToWrod(15))
int16_t signMag(uint8_t lo, uint8_t hi)
{
    const uint16_t w = (uint16_t)(lo | (hi << 8));
    return (w & 0x8000) ? (int16_t)-(int16_t)(w & 0x7fff) : (int16_t)w;
}

// The status packet of servo `id` with `len` data bytes in b[0..n): FF FF id len+2 error data checksum. Anything else
// (another servo's reply, an echo of our own request, noise) is skipped. Returns its data, or nullptr.
const uint8_t* findReply(const uint8_t* b, size_t n, uint8_t id, uint8_t len)
{
    const size_t size = (size_t)len + 6;
    for (size_t k = 0; k + size <= n; ++k) {
        if (b[k] != 0xff || b[k + 1] != 0xff || b[k + 2] != id || b[k + 3] != len + 2) continue;
        uint8_t sum = 0;
        for (size_t j = 2; j < size - 1; ++j) sum = (uint8_t)(sum + b[k + j]);
        if ((uint8_t)~sum == b[k + size - 1]) return b + k + 5;
    }
    return nullptr;
}

}  // namespace

ServoBus::ServoBus(HLSCL& hlscl, const uint8_t ids[kNumJoints]) : bus_(hlscl)
{
    for (int i = 0; i < kNumJoints; ++i) ids_[i] = ids[i];
}

void ServoBus::begin(uint32_t ioTimeoutMs)
{
    bus_.IOTimeOut = ioTimeoutMs;   // the driver's default of 10 ms would stall a tick that long when a servo is missing
    ser_ = bus_.pSerial;
}

void ServoBus::send(const uint8_t* p, size_t n)
{
    ser_->write(p, n);
    ser_->flush();                  // until the UART is idle: the writes queued before this have left too
}

void ServoBus::startReceive()
{
    while (ser_->available() > 0) ser_->read();   // leftovers: a late reply, an echo, noise
    rxLen_ = 0;
}

void ServoBus::pump()
{
    const int avail = ser_->available();
    if (avail <= 0 || rxLen_ >= sizeof(rx_)) return;
    const size_t want = (size_t)avail < sizeof(rx_) - rxLen_ ? (size_t)avail : sizeof(rx_) - rxLen_;
    rxLen_ += ser_->read(rx_ + rxLen_, want);
}

void ServoBus::requestSync()
{
    uint8_t p[8 + kNumJoints];
    const uint8_t kReadLen = readLen();
    syncLen_ = kReadLen;             // the replies are parsed with the length asked for, even if bus_read changes it
    uint8_t sum = (uint8_t)(kBroadcast + (kNumJoints + 4) + kInstSyncRead + kFirstReg + kReadLen);
    p[0] = p[1] = 0xff;
    p[2] = kBroadcast;
    p[3] = kNumJoints + 4;
    p[4] = kInstSyncRead;
    p[5] = kFirstReg;
    p[6] = kReadLen;
    for (int i = 0; i < kNumJoints; ++i) { p[7 + i] = ids_[i]; sum = (uint8_t)(sum + ids_[i]); }
    p[7 + kNumJoints] = (uint8_t)~sum;
    startReceive();
    send(p, sizeof(p));
    sentUs_ = micros();
    syncDueUs_ = sentUs_ + kSyncFirstUs + kNumJoints * ((kReadLen + 6) * kSyncByteUs + kSyncGapUs);
    syncPending_ = true;
    traced_ = false;
    if (traceLeft_) {                // bus_diag trace: watch the replies come in (busy until they are due, ~2.3 ms)
        --traceLeft_;
        cur_.traceN = 0;
        size_t last = 0;
        while ((int32_t)(micros() - (syncDueUs_ + kLateUs)) < 0) {
            pump();
            if (rxLen_ >= (size_t)kNumJoints * (kReadLen + 6u)) {   // all seven in: stop watching
                if (cur_.traceN < 40) { cur_.traceUs[cur_.traceN] = (uint16_t)(micros() - sentUs_); cur_.traceLen[cur_.traceN] = (uint8_t)rxLen_; ++cur_.traceN; }
                break;
            }
            if (rxLen_ != last && cur_.traceN < 40) {
                cur_.traceUs[cur_.traceN] = (uint16_t)(micros() - sentUs_);
                cur_.traceLen[cur_.traceN] = (uint8_t)rxLen_;
                ++cur_.traceN;
                last = rxLen_;
            }
        }
        traced_ = true;
    }
}

void ServoBus::record(uint8_t seen)
{
    const uint8_t kReadLen = syncLen_;
    ++diag_.reads;
    int answered = 0;
    for (int i = 0; i < kNumJoints; ++i) answered += (seen >> i) & 1;
    const int stray = (int)rxLen_ - answered * (kReadLen + 6);
    if (seen == 0x7f) diag_.strayBytes += stray > 0 ? (uint32_t)stray : 0u;
    for (int i = 0; i < kNumJoints; ++i) {
        if ((seen >> i) & 1u) {
            const uint8_t* d = findReply(rx_, rxLen_, ids_[i], kReadLen);
            if (d && d[-1]) { ++diag_.flagged[i]; diag_.lastFlags[i] = d[-1]; }   // the error byte before the data
            if (gap_[i]) {
                ++diag_.runs[i][gap_[i] == 1 ? 0 : (gap_[i] <= 5 ? 1 : 2)];
                gap_[i] = 0;
            }
            continue;
        }
        ++diag_.missed[i];
        if (stray > 0) ++diag_.garbled[i]; else ++diag_.silent[i];
        if (gap_[i] < 0xffff) ++gap_[i];
        if (gap_[i] > diag_.longest[i]) diag_.longest[i] = gap_[i];
    }
    // the raw bytes: of reads that missed a servo, and one clean one for comparison
    const bool miss = seen != 0x7f;
    if (!miss && haveClean_ && !traced_) return;
    if (!traced_) cur_.traceN = 0;
    cur_.read = diag_.reads;
    cur_.missing = (uint8_t)(~seen & 0x7f);
    cur_.len = (uint8_t)(rxLen_ < sizeof(cur_.b) ? rxLen_ : sizeof(cur_.b));
    memcpy(cur_.b, rx_, cur_.len);
    portENTER_CRITICAL(&mux_);
    if (miss) {
        caps_[capHead_] = cur_;
        capHead_ = (capHead_ + 1) % kCaptures;
        if (nCaps_ < kCaptures) ++nCaps_;
    } else if (!haveClean_ || traced_) {
        clean_ = cur_;
        haveClean_ = true;
    }
    portEXIT_CRITICAL(&mux_);
}

bool ServoBus::retryOne(uint8_t& seen, RawServoState& r)
{
    int i = -1;
    for (int k = 1; k <= kNumJoints; ++k) {           // round robin over the missing ones
        const int j = (lastRetry_ + k) % kNumJoints;
        if (!((seen >> j) & 1u)) { i = j; break; }
    }
    if (i < 0) return false;
    lastRetry_ = i;
    ++diag_.retried[i];
    const uint8_t id = ids_[i], kReadLen = readLen();
    const uint8_t p[8] = {0xff, 0xff, id, 4, kInstRead, kFirstReg, kReadLen,
                          (uint8_t)~(uint8_t)(id + 4 + kInstRead + kFirstReg + kReadLen)};
    startReceive();
    send(p, sizeof(p));
    const uint32_t due = micros() + kFirstReplyUs + (kReadLen + 6) * kByteUs + kLateUs;
    for (;;) {
        pump();
        const uint8_t* d = findReply(rx_, rxLen_, id, kReadLen);
        if (d) {
            parseReply(d, i, r, kReadLen);
            seen |= (uint8_t)(1u << i);
            ++diag_.recovered[i];
            return true;
        }
        if ((int32_t)(micros() - due) >= 0) return false;
    }
}

void ServoBus::diagnostics(Diag& d, Capture caps[kCaptures], int& nCaps, Capture& clean, bool& haveClean, uint32_t& traceLeft)
{
    portENTER_CRITICAL(&mux_);
    d = diag_;
    nCaps = nCaps_;
    for (int k = 0; k < nCaps_; ++k) caps[k] = caps_[(capHead_ - 1 - k + kCaptures) % kCaptures];
    clean = clean_;
    haveClean = haveClean_;
    traceLeft = traceLeft_;
    portEXIT_CRITICAL(&mux_);
}

void ServoBus::parseReply(const uint8_t* d, int i, RawServoState& r, uint8_t len) const
{
    r.pos[i] = signMag(d[0], d[1]);              // 56-57
    r.speed[i] = signMag(d[2], d[3]);            // 58-59
    if (len < kFullLen) return;
    r.voltage[i] = d[6];                         // 62
    r.tempC[i] = d[7];                           // 63
    r.current[i] = signMag(d[13], d[14]);        // 69-70
}

uint8_t ServoBus::collectSync(RawServoState& r)
{
    if (!syncPending_) requestSync();
    syncPending_ = false;
    uint8_t seen = 0;
    size_t parsed = (size_t)-1;
    for (;;) {
        pump();
        if (rxLen_ != parsed) {
            parsed = rxLen_;
            for (int i = 0; i < kNumJoints; ++i) {
                const uint8_t bit = (uint8_t)(1u << i);
                if (seen & bit) continue;
                const uint8_t* d = findReply(rx_, rxLen_, ids_[i], syncLen_);
                if (!d) continue;
                parseReply(d, i, r, syncLen_);
                seen |= bit;
            }
        }
        if (seen == 0x7f || (int32_t)(micros() - (syncDueUs_ + kLateUs)) >= 0) break;
    }
    record(seen);
    retriedThisTick_ = false;
    if (retry_ && seen != 0x7f) {
        retriedThisTick_ = true;
        retryOne(seen, r);
    }
    for (int i = 0; i < kNumJoints; ++i) r.ok[i] = (seen >> i) & 1u;
    return seen;
}

void ServoBus::setSlowInterval(uint32_t ms)
{
    const uint32_t tickUs = kControlPeriodUs;
    slowMs_ = ms < 28 ? 28 : (ms > 10000 ? 10000 : ms);
    // one Read per slowEvery_ ticks, round robin over the seven
    slowEvery_ = (uint32_t)((uint64_t)slowMs_ * 1000 / ((uint64_t)tickUs * kNumJoints));
    if (slowEvery_ < 1) slowEvery_ = 1;
    slowNext_ = slowTick_;
}

void ServoBus::requestSlow()
{
    ++slowTick_;
    if (full_) return;              // the SyncRead brings voltage, temperature and current of every servo
    if (retriedThisTick_) return;   // the retry used this tick's bus time
    if (focus_ >= 0 && focus_ < kNumJoints && slowTick_ % 8 == 0) {
        slowWho_ = focus_;
        slowAdvance_ = false;
    } else if (firstRound_ > 0 || (int32_t)(slowTick_ - slowNext_) >= 0) {   // all seven once at the start
        slowWho_ = slow_;
        slowAdvance_ = true;
        slowNext_ = slowTick_ + slowEvery_;
        if (firstRound_ > 0) --firstRound_;
    } else {
        return;
    }
    const uint8_t id = ids_[slowWho_];
    const uint8_t p[8] = {0xff, 0xff, id, 4, kInstRead, kSlowReg, kSlowLen,
                          (uint8_t)~(uint8_t)(id + 4 + kInstRead + kSlowReg + kSlowLen)};
    startReceive();
    send(p, sizeof(p));
    slowDueUs_ = micros() + kFirstReplyUs + (kSlowLen + 6) * kByteUs;
    slowPending_ = true;
}

void ServoBus::collectSlow(RawServoState& r)
{
    if (!slowPending_) return;
    slowPending_ = false;
    for (;;) {
        pump();
        const uint8_t* d = findReply(rx_, rxLen_, ids_[slowWho_], kSlowLen);
        if (d) {
            r.voltage[slowWho_] = d[0];                    // 62
            r.tempC[slowWho_] = d[1];                      // 63
            r.current[slowWho_] = signMag(d[7], d[8]);     // 69-70
            break;
        }
        if ((int32_t)(micros() - (slowDueUs_ + kLateUs)) >= 0) break;
    }
    if (slowAdvance_) slow_ = (slow_ + 1) % kNumJoints;
}

uint8_t ServoBus::pingAll()
{
    uint8_t missing = 0;
    for (int i = 0; i < kNumJoints; ++i) {
        const int r = bus_.Ping(ids_[i]);
        if (bus_.getLastError() || r != ids_[i]) missing |= (uint8_t)(1u << i);
        delay(2);
    }
    return missing;
}

void ServoBus::writeCurrent(const int16_t raw[kNumJoints], uint8_t mask)
{
    // register 44 is sign-magnitude: bit 15 = direction (as HLSCL::WriteEle encodes it), little-endian
    uint8_t ids[kNumJoints], data[2 * kNumJoints];
    int n = 0;
    for (int i = 0; i < kNumJoints; ++i) {
        if (!(mask & (1u << i))) continue;
        uint16_t v = (uint16_t)(raw[i] < 0 ? -raw[i] : raw[i]);
        if (raw[i] < 0) v |= (1u << 15);
        ids[n] = ids_[i];
        data[2 * n] = (uint8_t)(v & 0xff);
        data[2 * n + 1] = (uint8_t)(v >> 8);
        ++n;
    }
    if (n) bus_.syncWrite(ids, (uint8_t)n, HLSCL_GOAL_TORQUE_L, data, 2);
}

void ServoBus::writeGoals(const int16_t goalRaw[kNumJoints], uint8_t mask)
{
    uint8_t ids[kNumJoints], data[2 * kNumJoints];
    int n = 0;
    for (int i = 0; i < kNumJoints; ++i) {
        if (!(mask & (1u << i))) continue;
        const uint16_t v = (uint16_t)(goalRaw[i] < 0 ? 0 : goalRaw[i]);
        ids[n] = ids_[i];
        data[2 * n] = (uint8_t)(v & 0xff);
        data[2 * n + 1] = (uint8_t)(v >> 8);
        ++n;
    }
    if (n) bus_.syncWrite(ids, (uint8_t)n, HLSCL_GOAL_POSITION_L, data, 2);
}

uint8_t ServoBus::setPositionMask(uint8_t want, const int16_t goalRaw[kNumJoints], uint16_t speed, uint8_t acc, uint16_t limit)
{
    for (int i = 0; i < kNumJoints; ++i) {
        const uint8_t bit = (uint8_t)(1u << i);
        if ((want & bit) == (posMask_ & bit)) continue;
        const uint8_t id = ids_[i];
        bus_.EnableTorque(id, 0);
        if (want & bit) {
            // the goal block (register 44 = the current limit in mode 0) only after the servo acknowledged mode 0
            if (bus_.ServoMode(id) && bus_.WritePosEx(id, goalRaw[i], speed, acc, limit)) posMask_ |= bit;
            else { bus_.EleMode(id); bus_.WriteEle(id, 0); }
        } else {
            bus_.EleMode(id);
            bus_.WriteEle(id, 0);
            posMask_ &= (uint8_t)~bit;
        }
        bus_.EnableTorque(id, 1);
    }
    return posMask_;
}

void ServoBus::torque(bool on)
{
    uint8_t data[kNumJoints];
    for (int i = 0; i < kNumJoints; ++i) data[i] = on ? 1 : 0;
    bus_.syncWrite(ids_, kNumJoints, HLSCL_TORQUE_ENABLE, data, 1);
}

void ServoBus::servoTest(int i, uint8_t step, int16_t goalRaw, uint16_t speed, uint8_t acc, uint16_t torque)
{
    if (i < 0 || i >= kNumJoints) return;
    const uint8_t id = ids_[i];
    if (step == 1) {
        bus_.EnableTorque(id, 0);
        bus_.ServoMode(id);
        bus_.WritePosEx(id, goalRaw, speed, acc, torque);
        bus_.EnableTorque(id, 1);
    } else if (step == 2) {
        bus_.WritePosEx(id, goalRaw, speed, acc, torque);
    } else if (step == 3) {
        bus_.EnableTorque(id, 0);
        bus_.EleMode(id);
    }
}

bool ServoBus::readRegisters(int i, uint8_t out[71])
{
    if (i < 0 || i >= kNumJoints) return false;
    return bus_.Read(ids_[i], 0, out, 40) == 40 && bus_.Read(ids_[i], 40, out + 40, 31) == 31;
}

bool ServoBus::currentMode()
{
    bool ok = true;
    for (int i = 0; i < kNumJoints; ++i) ok = (bus_.EleMode(ids_[i]) != 0) && ok;   // Ack() returns 0 on failure
    posMask_ = 0;
    return ok;
}

}  // namespace raft
