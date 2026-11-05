#include <Arduino.h>
#include <SCServo.h>

// GPIO 32 -> servo TX, GPIO 33 -> servo RX
constexpr int kServoRxPin = 33;
constexpr int kServoTxPin = 32;
constexpr uint32_t kServoBaud = 1000000;
constexpr uint32_t kUsbBaud = 115200;
constexpr uint8_t kMinId = 0;
constexpr uint8_t kMaxId = 253; // 0xFE is broadcast, avoid 254/255
constexpr int kNumJoints = 7;   // the firmware expects IDs 0..6 in joint order, as on PAROL6 and PAR6
constexpr int kPingRepeats = 5;
const char *const kJointNames[kNumJoints] = {"J1 base", "J2 shoulder", "J3 elbow", "J4", "J5", "J6", "J7 trigger"};

HLSCL hlscl;
HardwareSerial servoBus(2);

void printHelp()
{
  Serial.println();
  Serial.println("Type a command, then Enter: s = scan the bus, w = watch joint positions, t = bus timing,");
  Serial.println("c <id> <raw> <ms> = current test on one servo, a number = change that servo's ID, x = exit");
}

// Reads one line from the monitor and echoes it (the monitor does not show what you type). Until the first key the
// help repeats every 5 s, so a monitor opened after the start-up scan still shows what to type.
String readLine()
{
  static bool typedOnce = false;
  String input;
  uint32_t idleSince = millis();
  while (true) {
    if (!Serial.available()) {
      if (!typedOnce && millis() - idleSince > 5000) {
        printHelp();
        Serial.print("> ");
        idleSince = millis();
      }
      delay(5);
      continue;
    }
    const char c = Serial.read();
    typedOnce = true;
    if (c == '\r' || c == '\n') {
      if (input.length() == 0) {
        continue; // the \n after \r, or Enter on an empty line
      }
      Serial.println();
      input.trim();
      return input;
    }
    if (c == '\b' || c == 127) {
      if (input.length() > 0) {
        input.remove(input.length() - 1);
        Serial.print("\b \b");
      }
      continue;
    }
    input += c;
    Serial.print(c);
  }
}

// An ID typed as digits only, 0..253, or -1. (String::toInt() reads anything else as 0, which is a real ID here.)
int parseId(const String &input)
{
  if (input.length() == 0 || input.length() > 3) {
    return -1;
  }
  for (unsigned i = 0; i < input.length(); ++i) {
    if (!isDigit(input[i])) {
      return -1;
    }
  }
  const int value = input.toInt();
  return (value >= kMinId && value <= kMaxId) ? value : -1;
}

// The ID typed by the user, or -1 when they type x to cancel.
int readIdFromUser(const char *prompt)
{
  while (true) {
    Serial.println(prompt);
    Serial.print("> ");
    String input = readLine();
    if (input.equalsIgnoreCase("x")) {
      return -1;
    }
    const int value = parseId(input);
    if (value >= 0) {
      return value;
    }
    Serial.println("Invalid ID. Enter a value between 0 and 253 (x cancels).");
  }
}

// Present position 0..4095, or -1 when the servo does not answer (ReadPos cannot tell a failed read apart).
int readPosition(uint8_t id)
{
  int raw = hlscl.readWord(id, HLSCL_PRESENT_POSITION_L);
  if (raw < 0) {
    return -1;
  }
  return (raw & (1 << 15)) ? -(raw & ~(1 << 15)) : raw;
}

// Pings every ID. An ID that answers is pinged kPingRepeats times: a flaky line, or two servos sharing one ID
// (their replies collide), shows up as missed answers. With all seven servos connected, IDs 0..6 each answering
// every ping and nothing else means the numbering is right.
void scanBus()
{
  Serial.println("Scanning IDs 0-253 (about 5 s)...");
  bool jointFound[kNumJoints] = {};
  int found = 0, flaky = 0, extra = 0;
  for (int id = kMinId; id <= kMaxId; ++id) {
    if (hlscl.Ping(id) < 0 && hlscl.Ping(id) < 0) {
      continue;
    }
    int answers = 0;
    for (int k = 0; k < kPingRepeats; ++k) {
      if (hlscl.Ping(id) >= 0) {
        ++answers;
      }
    }
    ++found;
    if (answers < kPingRepeats) {
      ++flaky;
    }
    const bool isJoint = id < kNumJoints;
    if (isJoint) {
      jointFound[id] = true;
    } else {
      ++extra;
    }
    Serial.printf("  ID %3d  %-12s answered %d/%d pings, position %d%s\n", id, isJoint ? kJointNames[id] : "(not a joint)",
                  answers, kPingRepeats, readPosition(id), answers < kPingRepeats ? "  <- unreliable" : "");
  }

  Serial.printf("%d servo ID(s) answer.\n", found);
  int missing = 0;
  for (int j = 0; j < kNumJoints; ++j) {
    if (!jointFound[j]) {
      Serial.printf("  MISSING: %s (ID %d)\n", kJointNames[j], j);
      ++missing;
    }
  }
  if (missing == 0 && flaky == 0 && extra == 0) {
    Serial.println("OK: IDs 0-6 all answer, nothing else on the bus. Type w to check which joint is which.");
  } else {
    if (flaky > 0) {
      Serial.println("Unreliable answers: two servos may share that ID, or see the GPIO32 note in docs/bringup.md 2.1.");
    }
    if (missing > 0 || extra > 0) {
      Serial.println("Renumber in place only if exactly one joint ID is missing and exactly one ID above 6 answers every ping.");
      Serial.println("Otherwise connect one servo at a time (two servos with the same ID both take a new ID).");
    }
  }
}

// Prints the positions of IDs 0..6 five times a second until a key is pressed. Turn one joint by hand: only its
// column should change.
void watchJoints()
{
  Serial.println("Turn one joint at a time by hand; only its column should change. Press Enter to stop.");
  while (Serial.available()) {
    Serial.read();
  }
  while (!Serial.available()) {
    for (int j = 0; j < kNumJoints; ++j) {
      int pos = readPosition(j);
      if (pos < 0) {
        Serial.printf("J%d (ID %d)  --   ", j + 1, j);
      } else {
        Serial.printf("J%d (ID %d) %4d   ", j + 1, j, pos);
      }
    }
    Serial.println();
    delay(200);
  }
  while (Serial.available()) {
    Serial.read();
  }
}

// Bus timing: the main firmware's SyncRead (registers 56..70 of IDs 0..6), sent by hand so every received byte gets a
// timestamp. Prints when each servo's reply ends after the request, averaged over kTimingRuns reads, and how long a
// whole read takes. The firmware's driver gives up after 3-4 ms; its control tick is 5 ms.
constexpr uint8_t kSyncAddr = HLSCL_PRESENT_POSITION_L;
constexpr uint8_t kSyncLen = HLSCL_PRESENT_CURRENT_H - HLSCL_PRESENT_POSITION_L + 1;
constexpr int kTimingRuns = 100;
constexpr uint32_t kTimingWaitUs = 20000;

void busTiming()
{
  uint8_t pkt[8 + kNumJoints];
  int n = 0;
  pkt[n++] = 0xff;
  pkt[n++] = 0xff;
  pkt[n++] = 0xfe;
  pkt[n++] = kNumJoints + 4;
  pkt[n++] = INST_SYNC_READ;
  pkt[n++] = kSyncAddr;
  pkt[n++] = kSyncLen;
  uint8_t sum = static_cast<uint8_t>(0xfe + kNumJoints + 4 + INST_SYNC_READ + kSyncAddr + kSyncLen);
  for (int j = 0; j < kNumJoints; ++j) {
    pkt[n++] = j;
    sum += j;
  }
  pkt[n++] = ~sum;

  const int replyLen = kSyncLen + 6;
  const int total = kNumJoints * replyLen;
  static uint8_t buf[256];
  static uint32_t at[256];
  uint32_t startSum[kNumJoints] = {}, endSum[kNumJoints] = {};
  int seen[kNumJoints] = {};
  uint32_t doneMin = 0xffffffff, doneMax = 0, doneSum = 0, txSum = 0;
  int complete = 0;
  Serial.printf("Bus timing: %d SyncReads of %d bytes from IDs 0-%d...\n", kTimingRuns, kSyncLen, kNumJoints - 1);
  for (int run = 0; run < kTimingRuns; ++run) {
    while (servoBus.available()) {
      servoBus.read();
    }
    const uint32_t tTx = micros();
    servoBus.write(pkt, n);
    servoBus.flush();
    const uint32_t t0 = micros();
    txSum += t0 - tTx;
    int got = 0;
    while (got < total && micros() - t0 < kTimingWaitUs) {
      const int c = servoBus.read();
      if (c >= 0) {
        at[got] = micros() - t0;
        buf[got++] = static_cast<uint8_t>(c);
      }
    }
    for (int i = 0; i + replyLen <= got;) {
      if (buf[i] == 0xff && buf[i + 1] == 0xff && buf[i + 2] < kNumJoints) {
        const int id = buf[i + 2];
        startSum[id] += at[i];
        endSum[id] += at[i + replyLen - 1];
        ++seen[id];
        i += replyLen;
      } else {
        ++i;
      }
    }
    if (got >= total) {
      const uint32_t d = at[total - 1];
      ++complete;
      doneSum += d;
      doneMin = d < doneMin ? d : doneMin;
      doneMax = d > doneMax ? d : doneMax;
    }
    delay(5);
  }
  Serial.printf("  sending the request: %lu us on average\n", (unsigned long)(txSum / kTimingRuns));
  for (int j = 0; j < kNumJoints; ++j) {
    if (seen[j] == 0) {
      Serial.printf("  ID %d (%s): no reply\n", j, kJointNames[j]);
    } else {
      Serial.printf("  ID %d (%-11s): reply %4lu..%4lu us after the request, %d/%d replies\n", j, kJointNames[j],
                    (unsigned long)(startSum[j] / seen[j]), (unsigned long)(endSum[j] / seen[j]), seen[j], kTimingRuns);
    }
  }
  if (complete > 0) {
    Serial.printf("  whole read: %lu us average, %lu..%lu us, %d/%d complete\n", (unsigned long)(doneSum / complete),
                  (unsigned long)doneMin, (unsigned long)doneMax, complete, kTimingRuns);
  } else {
    Serial.printf("  no read was complete within %lu us\n", (unsigned long)kTimingWaitUs);
  }

  // The same read through the driver, as the firmware does it (4 ms timeout).
  uint8_t ids[kNumJoints];
  for (int j = 0; j < kNumJoints; ++j) {
    ids[j] = j;
  }
  hlscl.syncReadBegin(kNumJoints, kSyncLen, 4);
  uint32_t drvSum = 0, drvMax = 0;
  int drvComplete = 0;
  for (int run = 0; run < kTimingRuns; ++run) {
    const uint32_t t = micros();
    const int len = hlscl.syncReadPacketTx(ids, kNumJoints, kSyncAddr, kSyncLen);
    const uint32_t d = micros() - t;
    drvSum += d;
    drvMax = d > drvMax ? d : drvMax;
    if (len >= total) {
      ++drvComplete;
    }
    delay(5);
  }
  hlscl.syncReadEnd();
  Serial.printf("  through the driver: %lu us average, %lu us max, %d/%d complete\n", (unsigned long)(drvSum / kTimingRuns),
                (unsigned long)drvMax, drvComplete, kTimingRuns);
}

// Current test: one servo in constant-current mode (2) with the goal current `raw` (register units, about 6.5 mA
// each; sign as the driver's WriteEle encodes it) for `ms`, then torque off. Prints which way the encoder counted
// and the present current it reported, so the sign and unit of the current register can be checked on the arm.
void driveCurrent(int id, int raw, int ms, int rewriteMs)
{
  if (raw < -300 || raw > 300 || ms < 50 || ms > 1000) {
    Serial.println("c <id> <raw -300..300> <ms 50..1000>");
    return;
  }
  const int p0 = readPosition(id);
  if (p0 < 0) {
    Serial.printf("ID %d does not answer.\n", id);
    return;
  }
  hlscl.EnableTorque(id, 0);
  hlscl.EleMode(id);
  hlscl.WriteEle(id, 0);
  hlscl.EnableTorque(id, 1);
  hlscl.WriteEle(id, raw);
  int curMin = 0, curMax = 0, pMin = p0, pMax = p0;
  const uint32_t t0 = millis();
  uint32_t lastWrite = millis();
  while (millis() - t0 < (uint32_t)ms) {
    if (rewriteMs > 0 && millis() - lastWrite >= (uint32_t)rewriteMs) {   // rewrite the goal like the control loop does
      hlscl.WriteEle(id, raw);
      lastWrite = millis();
    }
    const int p = readPosition(id);
    const int c = hlscl.readWord(id, HLSCL_PRESENT_CURRENT_L);
    if (p >= 0) {
      pMin = p < pMin ? p : pMin;
      pMax = p > pMax ? p : pMax;
      int dp = p - p0;
      if (dp > 2048) dp -= 4096;
      if (dp < -2048) dp += 4096;
      if (dp > 60 || dp < -60) {   // moved 5 deg: stop before it swings far
        Serial.printf("  moved %+d ticks after %lu ms: stopping\n", dp, (unsigned long)(millis() - t0));
        break;
      }
    }
    if (c >= 0) {
      const int cs = (c & (1 << 15)) ? -(c & 0x7fff) : c;
      curMin = cs < curMin ? cs : curMin;
      curMax = cs > curMax ? cs : curMax;
    }
    delay(rewriteMs > 0 && rewriteMs < 10 ? rewriteMs : 10);
  }
  hlscl.WriteEle(id, 0);
  hlscl.EnableTorque(id, 0);
  delay(50);
  const int p1 = readPosition(id);
  int d = p1 - p0;
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  Serial.printf("ID %d goal current %+d for %d ms: position %d -> %d (%+d ticks, range %d..%d), present current %d..%d\n",
                id, raw, ms, p0, p1, d, pMin, pMax, curMin, curMax);
}

bool changeServoId(uint8_t currentId, uint8_t newId)
{
  if (currentId == newId) {
    Serial.println("Current ID and new ID are identical.");
    return false;
  }

  Serial.print("Pinging servo with ID ");
  Serial.println(currentId);
  if (hlscl.Ping(currentId) < 0) {
    Serial.println("No response from servo. Verify the wiring, and type s to see which IDs answer.");
    return false;
  }

  hlscl.EnableTorque(currentId, 0);
  delay(20);
  hlscl.unLockEprom(currentId);
  delay(20);

  Serial.print("Writing new ID ");
  Serial.print(newId);
  Serial.println("...");
  int result = hlscl.writeByte(currentId, HLSCL_ID, newId);
  if (result < 0) {
    Serial.println("Failed to write new ID. Try again.");
    return false;
  }
  delay(20);

  // After the ID write succeeds the servo will respond under its new ID. Torque stays off: the servo may sit in
  // the assembled arm.
  hlscl.LockEprom(newId);
  delay(50);

  if (hlscl.Ping(newId) >= 0) {
    Serial.println("ID change complete.");
    return true;
  }

  Serial.println("Unable to ping servo using the new ID. Power cycle the servo and try again.");
  return false;
}

void setup()
{
  Serial.begin(kUsbBaud);
  servoBus.begin(kServoBaud, SERIAL_8N1, kServoRxPin, kServoTxPin);
  hlscl.pSerial = &servoBus;
  delay(500);

  Serial.println();
  Serial.println("=== Feetech Servo ID Utility ===");
  Serial.println("Expected IDs: J1 base = 0, J2 shoulder = 1, J3 elbow = 2, J4 = 3, J5 = 4, J6 = 5, J7 trigger = 6.");
  scanBus();
  printHelp();
}

void loop()
{
  Serial.print("> ");
  String input = readLine();
  if (input.equalsIgnoreCase("s")) {
    scanBus();
    printHelp();
    return;
  }
  if (input.equalsIgnoreCase("w")) {
    watchJoints();
    printHelp();
    return;
  }
  if (input.equalsIgnoreCase("t")) {
    busTiming();
    printHelp();
    return;
  }
  if (input.length() > 2 && (input[0] == 'c' || input[0] == 'C') && input[1] == ' ') {
    int id = -1, raw = 0, ms = 0, rewriteMs = 0;
    if (sscanf(input.c_str() + 2, "%d %d %d %d", &id, &raw, &ms, &rewriteMs) >= 3 && id >= 0 && id <= 253) {
      driveCurrent(id, raw, ms, rewriteMs);
    } else {
      Serial.println("c <id> <raw -300..300> <ms 50..1000>");
    }
    return;
  }
  if (input.equalsIgnoreCase("x") || input.equalsIgnoreCase("exit")) {
    Serial.println("Exiting. Flash the main firmware for normal use.");
    while (true) {
      delay(1000);
    }
  }
  const int currentId = parseId(input);
  if (currentId < 0) {
    printHelp();
    return;
  }

  const int newId = readIdFromUser("Enter NEW servo ID (x cancels)");
  if (newId < 0) {
    Serial.println("Cancelled.");
    return;
  }
  if (newId == currentId) {
    Serial.println("New ID matches current ID. Choose a different number.");
    return;
  }

  if (changeServoId(static_cast<uint8_t>(currentId), static_cast<uint8_t>(newId))) {
    Serial.println("Type s to scan again.");
  } else {
    Serial.println("ID change failed. Check connections and try again.");
  }
}
