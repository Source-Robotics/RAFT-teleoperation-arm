# RAFT leader arm: command channel and leader stream

Everything runs on the ESP32-PoE. A PC is only needed to send commands or to watch the stream, never to keep
the arm working. Source: `src/main/app.cpp` (commands), `src/main/net.cpp` (network), `src/main/leader_frame.h`
(stream), `src/main/main.cpp` (serial console).

## Ways in

| Channel | Where | Format |
|---|---|---|
| Web UI | http://raft-leader.local/ (port 80), up to 4 connections | the control page, plus `GET /api/status`, `POST /api` and a WebSocket (`docs/web_ui.md`) |
| Command channel | TCP port 5000 on `raft-leader.local` (or its DHCP address), up to 2 clients | one JSON command per line (at most 511 characters), one JSON reply per line |
| Serial console | USB, 115200 baud (PlatformIO monitor, Arduino serial monitor) | the same JSON lines, or the shorthand below |
| Leader stream | UDP from the ESP32 to the host and port set with `stream`, one frame per control tick (250 Hz) | 288-byte binary frame (version 2) |
| Firmware update | ArduinoOTA on `raft-leader.local` | `pio run -e main -t upload --upload-port raft-leader.local` |

The ESP32 announces itself over mDNS as `raft-leader.local`, with the services `_http._tcp` on port 80 and
`_raft._tcp` on port 5000. The serial console prints the address at boot ("Network up: ...") and in the 1 Hz
status line.

To talk to port 5000 from Windows, use PuTTY with connection type "Raw", or `ncat raft-leader.local 5000` (ncat
comes with Nmap). Type one command per line.

An OTA update switches torque off before it rewrites the flash. Build with `-D RAFT_OTA_PASSWORD=\"...\"` in
`build_flags` to require a password.

## Replies

Every reply is one line of valid JSON: `{"ok": true, ...}` or `{"ok": false, "error": "..."}`. Joint numbers
are 1 to 7 (7 is the gripper trigger). Angles in commands are degrees unless the key says otherwise. String
values are plain words and host names: a backslash escape or a control character in one arrives as `?`.
Numbers must be finite and at most 1e9 in magnitude: `inf`, `nan` and an overflowing `1e999` count as missing.

## When commands are accepted

The arm refuses changes that would surprise it while it works:

| Command group | Accepted |
|---|---|
| `set_kt`, `set_ma_per_unit`, `load`, `defaults`, `calib_zero`, `calib_stop` | only in LIMP, with no calibration, teleoperation or replay running. A sign flipped while gravity compensation runs would push the arm the wrong way. |
| `spring`, `gravity_scale`, `calib_torque`, `calib_spring`, `calib_range` | not during a calibration, teleoperation or replay |
| `save`, `servo_info` | not during a calibration, teleoperation, a move or a replay. Writing flash stalls both cores for tens of ms; `servo_info` pauses the servo bus for about 10 ms. On the ESP32 both run outside the application lock. |
| teleoperation settings and haptics | any time except during a calibration (they are meant to be tuned live) |

## Commands

### Status and modes

| Command | Arguments | What it does |
|---|---|---|
| `status` | | Everything at a glance (see below). |
| `mode` | `mode`: `limp`, `gravity`, `move` | `limp` always works and also stops teleoperation, calibration and replay. `gravity` and `move` need a calibrated arm, a passed model self-test and no fault. `move` holds the current pose. |
| `move` | `q_deg`: 6 or 7 values (or `q` in rad), `speed_deg` optional (up to 180) | Smooth minimum-jerk move to the target, clamped to the joint ranges. Grabbing the arm during the move hands it back to GRAVITY. |
| `rest` | | Move to the rest pose (J2 -82, J3 -80 deg, the rest 0: the arm lying back the way it rests limp), then go limp. |
| `gravity_scale` | `value` 0 to 1.5 | Scales gravity compensation. 1 = real gravity. |
| `hold_friction` | `value` 0 to 1 | Friction compensation (holding): extra push against gravity, as a share of the gravity load, because the servos' gears slip when gravity drives them. 0.3 on a new arm. `save` keeps it (settings blob v3). |
| `hold_stiffness` | `value` 0.25 to 8 | Hold (MOVE) stiffness factor: K x value, D x its square root. 1 is nominal (1.5 Nm/rad on J1-J3). Not saved yet; status shows `hold_stiffness`. |
| `hold_position` | `on` true/false | Hold, moves and replay with the servos' own position mode (mode 0) at the stall current, per joint while its goal stays clear of the encoder's 0/4095 wrap; other joints keep the computed Hold. Default on (`kHoldServoPosition`). Status: `hold_position`, and `pos_mask` = joints in position mode now. |
| `joint_gravity` | `joint` 1..7, `value` 0 to 3 | Per-joint factor on top of `gravity_scale`, for a joint whose real gravity load the CAD misses (wrist: cables, screws). 1 is nominal. Not saved yet; status shows `joint_gravity`, and `missed` = failed reads per servo since boot. |
| `joint_damping` | `joint` 1..7, `value` 0 to 0.2 Nm s/rad | Viscous damping in Gravity mode, a gentle brake on a joint that keeps coasting (the servos' gears run very easily from the joint side). Defaults from `kGravityDampingNms` in `arm_design.h` (J5 0.03, J6 0.02, trigger 0.035); changes are not saved; status shows `joint_damping`. |
| `servo_test` | `joint` 1..7, optional `torque_ma` 50..1300 (650), `speed_deg` 1..300 (30); or `on`: false | Arm in LIMP: that servo goes into its own position mode (mode 0) and holds where it is, with that current limit and speed; the others stay limp. `on: false`, LIMP, any mode change or a fault ends it (torque off, back to current mode). Status: `servo_test` {joint, goal_deg} or null. |
| `servo_jog` | `deg` -180..180 | During a servo test: the target, in degrees from where the test started (joint sense; clamped to the servo's 0..4095 goal range). |
| `clear_fault` | | Clears a fault. A failed model self-test cannot be cleared. |
| `bus_diag` | `trace` N, or `capture` 0..4, optional | The servo bus diagnosis since the start, per servo: SyncRead misses, how many were silent (nothing came) or garbled (bytes that were not a valid reply), gaps of 1 / 2-5 / 6+ reads in a row and the longest, replies with a non-zero error byte, in-tick retries and how many answered. `trace`: log when the reply bytes come in for the next N reads. `capture` k: the raw bytes of one recorded read (0..3 the newest that missed a servo, 4 a clean one), with the trace if there was one. Runs outside the application lock. |
| `servo_poll` | `ms` 28 to 10000 | Voltage, temperature and current: each servo every this many ms, one servo at a time (default 2000; temperature drives the derate from 60 deg C and the fault at 75, voltage and current are only shown). The servo of a running position test is read every 32 ms. Not saved. |
| `scope_start` | `seconds` 0.1 to 3 (default 2) | The Plots page's 250 Hz capture: every control tick, the leader's angles and speeds, the target sent to PAR6 (while teleoperating) and the measured currents, into 56 bytes a sample from the heap. |
| `scope_read` | `from`, `n` (45 at most) | The capture's state (`recording`, `done`, `idle`), and samples as base64 of int16 little-endian, `words` per sample: time in 0.1 ms, q[7] in 1e-4 rad, qd[7] in 1e-3 rad/s, sent target[6] in 1e-4 rad (-32768 = none), current[7] in mA. |
| `bus_read` | `full` true / false | What the SyncRead covers every tick. Full (default): registers 56..70, so position, speed, voltage, temperature and current of every servo at 250 Hz (21-byte replies, ~2.2 ms of the bus). Short: 56..59, position and speed only; voltage, temperature and current then come from `servo_poll`. Not saved. |
| `scope_free` | | Gives the capture's memory back. |
| `bus_retry` | `on` true / false | A servo missing from the SyncRead is asked again on its own in the same tick (one per tick, that tick skips the slow read). On by default. |
| `servo_info` | `joint` optional | Reads every servo's register table (about 10 ms, between two control ticks): ID, firmware, model, mode, supply, temperature, position, torque limit (register 48), angle limits, and notes on anything that stands out. With `joint`, also that servo's raw registers 0..70 in hex. Used in `docs/bringup.md`. |

`status` returns: the arm `name` ("" when unnamed), mode, fault, calibration flags, joint angles, TCP position and rpy, temperatures, J2 current,
spring, Kt, signs, measured friction per joint (`friction_nm`), the joint ranges in use (`range_deg`), gravity
scale, auto-rest, mA per unit, PAR6 host, stream target, `unsaved`, calibration progress (`calib`,
`calib_joint`, `last_calib`, `last_calib_ok`, `calib_seq`); for teleoperation the stage (`teleop`), why it
waits (`teleop_note`), the link phase and error, `link_stale` (state packets dropped as duplicates or late,
command slots skipped because the leader target was old), `follower_deg` (the PAR6 pose in the RAFT's
convention, or null), `engaged`, `hold`, `precision`, `locks`, `orient_lock`, `guide`, `offset_deg`,
`offset_align`, the saved `settings`, the `buttons` (which are wired and which are pressed), and `replay`.

### Spring and behaviour

| Command | Arguments | What it does |
|---|---|---|
| `spring` | `type` (`cable`, `torsion`, `none`), `k` 0..5000, `a_mm` and `b_mm` 0..100, `q0_deg`, `scale` 0..2 | Sets the J2 counterbalance model. For the zero-length cable: `k` in N/m, `a_mm` = anchor height above J2 (13), `b_mm` = eyelet distance up the upper arm (20). Rates of the federnshop set: 565, 718, 824, 1027 N/m. |
| `auto_rest` | `on` true/false, `idle_s` 3..3600 | Moves to the rest pose after the arm was left alone in GRAVITY, then goes limp. Off by default. |

### Calibration (see `docs/calibration.md`)

| Command | Arguments | What it does |
|---|---|---|
| `calib_zero` | | Arm in LIMP, held in the CAD zero pose: stores every joint's zero. Takes 0.5 s. |
| `calib_stop` | `joint`, optional `deg` | Arm in LIMP: hold that joint still against its mechanical stop for 0.3 s. Stores its zero from the stop angle (`kStopDeg` in `arm_design.h`, or `deg` in the CAD convention). Status lists the table as `stop_deg`. The joint directions are fixed (`kJointDir`); `calib_dir` and `set_dir` are refused. |
| `calib_torque` | `joint` | Slow +-4 deg sweep of that joint with model feedforward. Measures its torque constant and friction. Needs at least 0.04 Nm of gravity load at the current pose. About 6 s. |
| `calib_spring` | | The same sweep on J2 with the upper arm tilted at least 40 deg. Corrects the spring strength. Run `calib_torque` on J2 first. |
| `calib_range` | | LIMP or GRAVITY: move every joint gently to both of its ends, then `calib_range_stop` (it also ends by itself after 3 minutes). Joints moved through at least 10 deg get their measured range, stored 2 deg inside the ends. The ranges clamp move targets and place the haptic barriers. |
| `calib_range_stop` | | Ends the range recording. |
| `calib_abort` | | Stops a running calibration. |
| `calib_result` | | Progress text and the last result message. |
| `set_kt` | `joint`, `kt` 0.1 to 2 Nm/A | Sets a torque constant by hand (LIMP only). |
| `set_ma_per_unit` | `value` 1 to 20 | Goal-current register unit in mA (LIMP only). Default 3.25, measured on the arm (Feetech's documents say 6.5). |
| `set_friction` | `joint`, `nm` 0 to 0.2 | Sets the Coulomb friction the compensation uses (normally measured by `calib_torque`). |
| `save` | | Writes all settings to flash (NVS). Refused while the arm is busy (see above). |
| `load` | | Reloads the saved settings (LIMP only). |
| `defaults` | | Back to factory settings, not saved (LIMP only). The arm name stays. |
| `set_name` | `name`: up to 20 of a-z, 0-9 and '-' (capitals are lowercased), `""` to remove it | Names the arm, so several arms on one network can be told apart: the web UI shows the name at once; after `save` and a restart the arm answers at raft-*name*.local (unnamed: raft-leader.local). Allowed at any time. |

A calibration result takes effect at once. Use `save` to keep it across power cycles. Settings saved by an older
firmware (blob versions 1 to 3) still load; the new fields start at their defaults.

If a sweep cannot follow its reference, for example because a sign is wrong, the firmware stops it and drops the
arm to LIMP. Support the arm during every torque or spring calibration.

### Teleoperation

| Command | Arguments | What it does |
|---|---|---|
| `teleop_start` | `host`: the PAR6 Pi, e.g. `192.168.1.20` or `par6.local`; `offset` true/false (optional) | Connects to the RCB runtime and pairs. Then it checks that the RAFT can reach the follower's pose: if a joint is outside the RAFT's range, `teleop_note` says which and nothing moves (unless `offset` is allowed). Otherwise the leader moves onto the follower's pose, claims control and couples. The host is remembered, and `save` keeps it. |
| `teleop_stop` | | Releases control and disconnects. The arm stays in GRAVITY. |
| `teleop_resume` | | After an outage the session waits in LOST with the leader weightless; this realigns and reclaims. An enable-button press does the same. |
| `teleop_realign` | | COUPLED or HELD: PAR6 holds where it really is (its measured pose, so a blocked PAR6 stops pushing), the leader moves onto it (removes any offset), then couples again. |
| `teleop_hold` | `on` true/false | The web UI's clutch: while on, PAR6 holds its last target and the leader is free (HELD). Off: coupled again, re-anchored where the leader is. |
| `teleop_offset` | `on` true/false | Allow coupling with an offset when PAR6 is outside the RAFT's range: the leader moves to the nearest pose inside its range and drives PAR6 relative to it. |
| `teleop_rate` | `hz`: 250, or 5 to 50 | How the targets go to PAR6, agreed when pairing, so only while teleoperation is off. 250: Real RTI, `rti_connect` without a rate, one target in reply to every PAR6 state packet (its 250 Hz loop; PAR6 stops after 40 ms without one). 5..50: SOFT_RTI at that rate, PAR6 fills in between (50, 25, 10 divide its loop). Default 50; not saved yet. Status: `link_rate_hz`, `link_mode`. |
| `teleop_filter` | `hz`: 0 (off) to 110 | 2nd-order Butterworth low-pass on the targets sent to PAR6, at the control rate, on position and velocity; lags about 230 / hz ms. Off by default, not saved yet. Status: `teleop_filter_hz`. |
| `teleop_precision` | `value` 0.05 to 1 | Follower motion per leader motion, around where both are now (1 = 1:1). Leaves an offset behind; `teleop_realign` removes it. |
| `teleop_lock` | `joints`: e.g. `[1, 6]` (arm joints 1..6), or `mask` 0..63 | Locked joints do not move PAR6; the coupling holds the leader joint where it was locked. `[]` unlocks all. |
| `orient_lock` | `on` true/false | Holds the handle's orientation as it is now: the wrist joints push against any turn. PAR6 copies the joint angles, so its tool keeps the same orientation. At J5 = 0 (wrist singular) a yaw about the vertical cannot be held. |
| `guide` | `type`: `none`, `plane_h` (keep the height), `plane_tool` (move only across the tool axis), `axis_tool` (move only along the tool axis: insertion), `axis_z` (only up and down) | A spring on the wrist centre toward that plane or line, anchored where it is now. With `orient_lock` the tool point follows the same plane or line. Works in GRAVITY too. |
| `teleop_settings` | any of: `auto_reclaim`, `require_enable`, `passivity`, `barriers_always` (true/false), `force_gain` 0..0.2, `force_sign` +1/-1, `force_deadband` 0..5 Nm, `friction_scale` 0..1, `trigger_deg` [open, closed], `gripper_raw` [open, closed] | See below. `save` keeps them. |
| `stream` | `host` (default: the address that sent the command), `port` (default 5005) | Starts the leader stream to that address. Use an IP address while teleoperating: a name lookup can pause the network task for up to 1 s. |

Settings:
- `auto_reclaim` (off): after an outage, realign and reclaim without waiting for the operator.
- `require_enable` (off): PAR6 moves only while the enable button on the handle is held. Needs a wired enable
  button (`ENABLE_BUTTON_PIN` in `main.cpp`).
- `passivity` (on): a passivity observer on each arm joint's coupling adds damping when the coupling gives the
  hand more energy than it took (delay, stiff coupling).
- `barriers_always` (off): the haptic barriers at the RAFT's own joint ranges also outside teleoperation. While
  coupled they are always on, at the intersection of the RAFT's range and PAR6's soft limits.
- `force_gain` (0, off), `force_sign` (-1), `force_deadband` (0.3 Nm): contact force reflection from PAR6's
  `t_ext` (measured motor torque minus model torque): leader torque = sign x gain x (t_ext beyond the deadband),
  low-passed at 8 Hz, clamped per joint. Check the sign on the real PAR6 first (`docs/bringup.md`).
- `friction_scale` (0, off): friction compensation with the measured Coulomb friction. Start at 0.5.
- `trigger_deg`, `gripper_raw`: the trigger angles and the gripper positions (raw 0..255, 0 open, 255 closed)
  for an open and a closed gripper. The same raw scale is used for `gripper_set` and for the gripper position
  the runtime reports (`g_p`).

Stages (`status.teleop`, frame byte 181): `WAIT_PAIRED`, `ALIGNING`, `CLAIMING`, `COUPLED`, `HELD` (PAR6 holds,
leader free), `REALIGNING`, `LOST` (outage; waiting for `teleop_resume`).

The PAR6 must allow the ESP32's address: see `RCB_ALLOWED_IPS` in `docs/rcb_link_test.md`.

### Replay (teach and replay)

| Command | Arguments | What it does |
|---|---|---|
| `replay_clear` | | Empties the trajectory and frees its memory. |
| `replay_add` | `dt_ms` 5..200 (with the first chunk), `q_cdeg`: 7 values per sample in centidegrees, at most 40 samples per command | Appends samples (up to 3000, e.g. 60 s at 50 Hz). The web UI's Record tab uploads recorded episodes this way. |
| `replay_start` | `speed` 0.1 to 1 (optional) | Moves to the first sample, then plays the trajectory as MOVE references (grabbing the arm stops it). With teleoperation COUPLED, PAR6 follows. |
| `replay_stop` | | Stops; the arm goes back to GRAVITY (or BILATERAL when coupled). |

## Serial shorthand

On the USB console a line that does not start with `{` is read as `command [argument]`:

| Typed | Sent to the App |
|---|---|
| `status` | `{"cmd": "status"}` |
| `mode gravity` | `{"cmd": "mode", "mode": "gravity"}` |
| `calib_stop 3` | `{"cmd": "calib_stop", "joint": 3}` |
| `calib_torque 2` | `{"cmd": "calib_torque", "joint": 2}` |
| `servo_info 3` | `{"cmd": "servo_info", "joint": 3}` |
| `gravity_scale 0.9` | `{"cmd": "gravity_scale", "value": 0.9}` |
| `teleop_precision 0.3` | `{"cmd": "teleop_precision", "value": 0.3}` |
| `teleop_start 192.168.1.20` | `{"cmd": "teleop_start", "host": "192.168.1.20"}` |
| `spring none` | `{"cmd": "spring", "type": "none"}` |

`quiet` switches the 1 Hz status line off and on. Commands with several arguments, such as `move` or a full
`spring`, need the JSON form.

## Leader frame

One UDP datagram, 288 bytes, little-endian, floats are IEEE 754 single precision, one per control tick (250 Hz):
the control task leaves every tick's frame in a ring that the network task sends from, so `seq` counts up by one per
tick and a gap means a lost datagram. With the full SyncRead (`bus_read`, the default) the measured currents,
voltages and temperatures are from that same tick; measured torque = current x Kt (`kt` in the status). The web UI's
WebSocket carries the same frame at its own rate. Version 2; the first 160 bytes are laid out as in version 1.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 4 | magic | `RAFT` |
| 4 | 1 | version | 2 |
| 5 | 1 | mode | 0 limp, 1 gravity, 2 move, 3 bilateral |
| 6 | 2 | flags | see below |
| 8 | 4 | seq | frame counter |
| 12 | 4 | t_us | ESP32 time, microseconds |
| 16 | 28 | q[7] | joint angles, rad, CAD convention (q = 0 is the standby pose), unwrapped past +-180 deg |
| 44 | 28 | qd[7] | joint speeds, rad/s |
| 72 | 28 | tau_cmd[7] | commanded torque, Nm |
| 100 | 28 | tau_gravity[7] | model gravity load, Nm, before the spring |
| 128 | 24 | tcp[6] | x, y, z in m and roll, pitch, yaw in rad (RCB rpy `xyz`), base frame |
| 152 | 7 | temp[7] | servo temperatures, deg C |
| 159 | 1 | events | events since the previous frame |
| 160 | 14 | current_mA[7] | measured current (present-current register, int16), joint sign convention |
| 174 | 7 | voltage[7] | servo supply, 0.1 V |
| 181 | 1 | stage | teleoperation stage: 0 off, 1 WAIT_PAIRED, 2 ALIGNING, 3 CLAIMING, 4 COUPLED, 5 HELD, 6 REALIGNING, 7 LOST |
| 182 | 1 | teleop_flags | bit 0 engaged, 1 t_ext valid, 2 orientation lock, 3 replaying, 4 web hold, 5 enable button pressed, 6 clutch button pressed |
| 183 | 1 | locks | locked arm joints, bit i = joint i+1 |
| 184 | 28 | q_follower[7] | PAR6 pose in the RAFT's CAD convention, rad; [6] = the gripper as a trigger angle |
| 212 | 24 | follower_target[6] | last target sent to PAR6, CAD convention, rad |
| 236 | 24 | tau_ext[6] | PAR6's t_ext, Nm |
| 260 | 4 | gripper_raw | PAR6's gripper position g_p, 0..255 (-1 = not reported) |
| 264 | 4 | follower_tick | PAR6's loop tick of that state (uint32) |
| 268 | 4 | precision | follower motion per leader motion |
| 272 | 1 | guide | 0 none, 1 plane_h, 2 plane_tool, 3 axis_tool, 4 axis_z |
| 276 | 4 | pc_energy | passivity observer: the smallest joint energy credit, J (negative: it is damping) |
| 280 | 4 | offset | largest leader-follower offset of the mapping, rad |
| 284 | 4 | reserved | 0 |

Flags: bit 0 calibrated, 1 torque on, 2 fault, 3 coupled to the follower, 4 spring configured, 5 model self-test
passed, 6 calibrating, 7 derating (a servo is above 60 deg C), 8 the follower fields hold fresh PAR6 state.

Events: bit 0 move done, 1 move overridden by a hand, 2 auto-rest started, 3 follower lost, 4 follower back.

## Safety rules the firmware enforces

- **Strict boot.** Torque goes off on every servo before anything else (an ESP32-only reset leaves the servos
  powered with their last current), then all 7 must answer, or the firmware halts with torque off and keeps
  naming the missing ones.
- **Watchdog.** If the control task stalls for 2 s the ESP32 resets, and the boot switches torque off.
- **Torque needs calibration.** Torque stays off until every joint has a zero and a sign, and the model self-test passed.
- **Faults go limp.** With torque on, a servo that misses 50 reads in a row (200 ms; 6 during a calibration) or reaches
  75 deg C faults the arm to LIMP until `clear_fault`. Through shorter gaps the joint keeps its last angle with its
  speed taken as 0, and in Hold its last position goal (like the RCB runtime with old motor data: a warning at 10
  ticks, an error at 50). Limp, a missing servo only counts; torque is refused while one has missed its last 10 reads.
- **Derating.** Torque derates linearly from 60 deg C to 70 deg C.
- **Calibration runaway.** A sweep that cannot follow its reference stops and goes limp.
- **Settings guards.** Signs, Kt, the register unit, `load` and `defaults` change only in LIMP (see the table above).
- **Telemetry checks.** A PAR6 state packet with an unusable joint value (NaN, infinite, implausible) is dropped;
  an unusable `t_ext`, `g_p` or `g_c` drops only that field. A packet whose tick is not newer than the last one
  (a duplicate or a late one) is dropped, so a stuck runtime that repeats itself times out.
- **Fresh targets only.** The link sends PAR6 only leader targets younger than 50 ms and drops the claim when none
  came for 300 ms (a stalled control task).
- **Reachability.** The session never couples to a pose the RAFT cannot reach without saying so; PAR6 targets are
  clamped to PAR6's soft limits, and the leader feels barriers before it gets there.
- **Jump guard.** A PAR6 target that would jump more than 0.15 rad in one control tick (an encoder slip) puts the
  session in HELD; it needs a release and re-engage (or a realign) to continue.
- **After an outage** the session waits for the operator (`teleop_resume` or the enable button), unless `auto_reclaim` is on.
- **OTA.** An update switches torque off first.
