// RAFT servo bus: the 7 HLS3606 servos on one half-duplex 1 Mbit/s bus.
// Writes and acknowledged commands go through the vendored Feetech HLSCL driver (unmodified); the control loop's reads
// talk to the UART directly, pipelined so that the CPU never waits for the slow servos. Per 4 ms tick (250 Hz):
//   tick start  collectSync: the replies to the SyncRead sent at the end of the last tick, which came in while the
//               control task slept: registers 56..70 (position, speed, load, voltage, temperature, status, current) of
//               every servo, ~2 ms of the bus; with the short read 56..59 only, plus requestSlow: a Read of 62..70 of one
//               servo now and then
//   App tick    the slow reply comes in meanwhile; collectSlow right after
//   tick end    the SyncWrites (goal currents, goal positions), then requestSync for the next tick
// Timing on the arm (bus_diag trace, 2026-11-03): the first reply is in 0.3 ms after the SyncRead request has left, all
// seven by ~1.15 ms (one every ~0.16 ms); the deadlines here allow 1.9 ms plus 0.4. With the slow Read and the writes a
// tick uses ~2 ms of the bus and ~1 ms of CPU. (The old driver's byte-by-byte reads made it look like 21 us per byte.)
// A servo that does not answer costs no time: its reply is simply missing at the next tick. The positions are the ones
// the servos sent during the previous tick, about as old as with a blocking read at the start of the tick.
// It fills the portable RawServoState (servo_types.h); zero, sign and units are applied by the App.
// ESP32 / Arduino only (the PC simulator models the bus in tools/sim/sim_core.h instead).
#pragma once

#include <Arduino.h>
#include <SCServo.h>

#include "arm_design.h"
#include "kinematics.h"
#include "servo_types.h"

namespace raft {

class ServoBus {
public:
    ServoBus(HLSCL& hlscl, const uint8_t ids[kNumJoints]);
    // The driver's acknowledged commands time out on millis(), so N ms waits N to N + 1 ms; a single reply (ping, write
    // ack) takes at most about 1.3 ms. Call after hlscl.pSerial is set.
    void begin(uint32_t ioTimeoutMs = 3);

    // One ping of every servo (strict boot, boot.cpp): bitmask of the ones that did not answer (0 = all present).
    uint8_t pingAll();

    // Send the SyncRead of position and speed for every servo; returns at once (after the request has left).
    void requestSync();
    // The replies to the last requestSync: position and speed into r, ok = answered (the others keep their previous
    // values). Waits only while a reply can still come; without a pending request it asks first and waits (~2.3 ms).
    // Returns the servos that answered (bit i = servo i).
    uint8_t collectSync(RawServoState& r);
    // Voltage, temperature and current, one servo at a time: send the Read, do other work, then collect its reply
    // (waits at most until it can no longer come). Nothing else may use the bus in between. requestSlow() is called
    // every tick and only asks when one is due: each servo every slowIntervalMs (temperature drives the derate and the
    // fault and changes over seconds; voltage and current are only shown), and the servo of a position test every
    // 8 ticks so its live current stays live.
    void requestSlow();
    void collectSlow(RawServoState& r);
    void setSlowInterval(uint32_t ms);                // per servo, 28 ms (every tick) .. 10 s
    uint32_t slowIntervalMs() const { return slowMs_; }
    void setSlowFocus(int servo) { focus_ = servo; }  // -1 = none

    // What the SyncRead covers. Full (the default): registers 56..70, position, speed, load, voltage, temperature, status
    // and current of every servo every tick (21-byte replies, ~2.2 ms of the bus); the slow Read is then not needed.
    // Short: 56..59, position and speed (10-byte replies, ~1.2 ms), voltage / temperature / current by the slow Read.
    void setFullRead(bool on) { full_ = on; }
    bool fullRead() const { return full_; }

    // Retry: a servo missing from the SyncRead is asked once more on its own, in the same tick (~0.55 ms; one servo per
    // tick, and that tick skips the slow read so the bus stays inside the tick). On by default.
    void setRetry(bool on) { retry_ = on; }
    bool retry() const { return retry_; }

    // Diagnostics of the SyncRead since boot, per servo (bus_diag): why replies were missing and how.
    struct Diag {
        uint32_t reads;                        // SyncReads collected
        uint32_t missed[kNumJoints];           // no valid reply to the SyncRead
        uint32_t silent[kNumJoints];           // ... and no stray bytes either: nothing came from it
        uint32_t garbled[kNumJoints];          // ... but bytes came that were not a valid reply (noise, a damaged reply)
        uint32_t runs[kNumJoints][3];          // gaps of 1, 2-5 and 6+ SyncReads in a row (6 = the fault)
        uint32_t longest[kNumJoints];          // longest gap, SyncReads
        uint32_t flagged[kNumJoints];          // valid replies with a non-zero error byte
        uint8_t lastFlags[kNumJoints];         // the last such error byte
        uint32_t retried[kNumJoints], recovered[kNumJoints];
        uint32_t strayBytes;                   // stray bytes seen on reads where every servo answered (echo, noise)
    };
    // The received bytes of one SyncRead, with when they came in (traces only while bus_diag asked for them).
    struct Capture {
        uint32_t read;                         // which SyncRead (Diag::reads)
        uint8_t missing, len, traceN;
        uint16_t traceUs[40];                  // us after the request had left ...
        uint8_t traceLen[40];                  // ... this many bytes were in
        uint8_t b[160];
    };
    static constexpr int kCaptures = 4;
    // Busy-wait at the end of the next `reads` ticks and log when the reply bytes come in (bus_diag trace).
    void trace(uint32_t reads) { traceLeft_ = reads; }
    // A copy for the network task. caps: the last kCaptures SyncReads that missed a servo, newest first.
    void diagnostics(Diag& d, Capture caps[kCaptures], int& nCaps, Capture& clean, bool& haveClean, uint32_t& traceLeft);
    void writeCurrent(const int16_t raw[kNumJoints], uint8_t mask = 0x7f);   // goal current, signed register units
    // Position mode per servo (bit i = servo i in mode 0, else current mode 2). Switches the servos whose bit changed,
    // one acknowledged write at a time (torque off, mode, goal or zero current, torque on), so a servo that missed its
    // mode change never sees a position-mode goal block. Returns the servos really in position mode.
    uint8_t setPositionMask(uint8_t want, const int16_t goalRaw[kNumJoints], uint16_t speed, uint8_t acc, uint16_t limit);
    void writeGoals(const int16_t goalRaw[kNumJoints], uint8_t mask);   // goal positions of the servos in mask
    uint8_t positionMask() const { return posMask_; }
    void torque(bool on);                             // TORQUE_ENABLE on all servos
    // Servo position test on servo i: step 1 start (torque off, position mode 0, goal, torque on), 2 a new goal,
    // 3 stop (torque off, back to current mode 2). speed / acc / torque in the servo's register units.
    void servoTest(int i, uint8_t step, int16_t goalRaw, uint16_t speed, uint8_t acc, uint16_t torque);
    bool currentMode();                               // mode 2 on all servos (call with torque off)
    // Registers 0..70 of servo i in two reads (servo_info). About 1 ms per servo; only between control ticks.
    bool readRegisters(int i, uint8_t out[71]);

    const uint8_t* ids() const { return ids_; }

private:
    void send(const uint8_t* p, size_t n);   // queue, wait until everything queued has left the UART
    void startReceive();                     // forget what came in so far
    void pump();                             // move what came in into rx_
    void record(uint8_t seen);               // the diagnostics of one collected SyncRead
    uint8_t readLen() const { return full_ ? 15 : 4; }
    void parseReply(const uint8_t* d, int i, RawServoState& r, uint8_t len) const;
    bool retryOne(uint8_t& seen, RawServoState& r);

    HLSCL& bus_;
    HardwareSerial* ser_ = nullptr;
    uint8_t ids_[kNumJoints];
    uint8_t posMask_ = 0;
    int slow_ = 0;   // the servo whose voltage, temperature and current are read next
    int slowWho_ = 0, focus_ = -1;
    bool slowAdvance_ = false;
    uint32_t slowMs_ = 2000, slowEvery_ = 71, slowTick_ = 0, slowNext_ = 0;
    int firstRound_ = kNumJoints;
    bool syncPending_ = false, slowPending_ = false, full_ = true;
    uint8_t syncLen_ = 15;                   // what the pending SyncRead asked for
    uint32_t syncDueUs_ = 0, slowDueUs_ = 0;   // when the last reply should be in
    uint8_t rx_[160];
    size_t rxLen_ = 0;
    // diagnostics and retry
    bool retry_ = true, retriedThisTick_ = false;
    int lastRetry_ = -1;
    uint16_t gap_[kNumJoints] = {0, 0, 0, 0, 0, 0, 0};
    Diag diag_ = {};
    Capture caps_[kCaptures] = {};
    Capture clean_ = {}, cur_ = {};
    int nCaps_ = 0, capHead_ = 0;
    bool haveClean_ = false, traced_ = false;
    volatile uint32_t traceLeft_ = 0;
    uint32_t sentUs_ = 0;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace raft
