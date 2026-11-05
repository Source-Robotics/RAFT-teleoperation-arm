# RAFT on the real arm, step by step

From loose parts on the desk to teleoperating PAR6, one checked step at a time. Every step says what you should
see. If you see something else, stop there: the table at the end says what it usually means.

Steps 1 to 12 and 14 have run on the first arm. The spring (13) and the PAR6 link (15) have been tested against the
MuJoCo simulator, an emulated PAR6 and the RCB runtime's protocol only, not yet on a real PAR6.

| Phase | What | Takes | Needs PAR6 |
|---|---|---|---|
| [0](#0-safety-rules) | Safety rules | read once | |
| [1](#1-parts-and-tools) | Parts and tools | | |
| [2](#2-wiring) | Wiring: supply, bus adapter, ESP32 | 1 h | |
| [3](#3-check-the-servo-ids) | Check the servo IDs (also the first bus test) | 10 min | |
| [4](#4-flash-and-watch-the-boot)-[7](#7-move-the-limp-arm-by-hand) | Flash, boot, bus timing, servo check, limp arm | 30 min | |
| [8](#8-zero-at-the-stops) | Calibration at the mechanical stops | 15 min | |
| [9](#9-first-torque-at-half-gravity)-[12](#12-friction-compensation-while-moving) | Gravity compensation, safety checks | 1 h | |
| [13](#13-the-spring-when-it-arrives)-[14](#14-running-on-its-own-poe) | Spring, running on PoE alone | later | |
| [15](#15-the-par6-link-in-three-stages) | PAR6 link: simulated robot first, then the real one | later | yes |

## USB and Ethernet: what goes where

At the bench the ESP32 has both cables plugged in, and each does its own job:

| Cable | Goes to | Used for |
|---|---|---|
| Micro-USB | the PC | Power for the ESP32. Flashing (`pio run ... -t upload`) and the serial console (`pio device monitor`). The `set_id` tool (step 3) works only here. Every command typed in this guide (`status`, `quiet`, `servo_info`, `mode limp`, the JSON commands) goes into the serial monitor. |
| Ethernet | the router, same network as the PC, a port **without** PoE | The web UI from step 6 on, and TCP port 5000. The web UI's Console (under the 3D view) takes the same commands as the serial monitor. |

Without a network, everything up to step 5 and every console command still works over USB; only the web UI needs
Ethernet. The Ethernet side needs a DHCP server (a router). A cable straight into the PC works only with Windows
Internet Connection Sharing switched on for that port, which hands out 192.168.137.x addresses.

Both at once is safe only because the Ethernet port gives no power: never USB together with PoE (step 0).

## 0. Safety rules

- **The hard stop.** The servo supply switch is the only hard stop: keep it within reach. The arm has no buttons;
  in software, the LIMP button or Esc in the web UI and `mode limp` on the console switch torque off (and end
  teleoperation). With PAR6 coupled, PAR6's own E-stop stops the follower.
- **Never USB and PoE together.** The ESP32-PoE (the version without -ISO) is not isolated: never connect USB while
  it is powered over Ethernet. Olimex: "not safe and should be avoided at all costs". At the bench, power it from
  USB and plug the network cable into a port without PoE (or switch PoE off for that port). Check the board's name:
  ESP32-PoE-ISO is the isolated one.
- **Hold the arm when torque changes.** Support the arm whenever torque goes on or off: LIMP and every fault drop it
  at once.
- **Clamp it down.** Clamp the base plate to the table, with cable slack for J1 and the wrist.
- **Rest.** The Rest pose button (and auto-rest, off by default) moves the arm to its parking pose, the way it rests
  limp: upper arm lying back 2 deg short of J2's stop, forearm folded down, wrist straight. There it goes limp and
  settles the last few degrees onto its stops. Keep a hand near it the first time.
- **Hands clear.** Keep fingers out of the trigger hinge and the elbow while a move runs.

## 1. Parts and tools

| Item | Notes |
|---|---|
| Olimex ESP32-PoE | Check that the metal can says ESP32-WROOM-32: the firmware's Ethernet setup assumes it (the WROVER version moves the Ethernet clock to GPIO0). |
| TTLinker mini | Half-duplex bus adapter (74HC126), `datasheets/TTLINKER.pdf`. Its logic runs at 5 V: see 2.1. |
| 6 V servo supply | Any DC adapter of 1.5 A or more with a floating output, 6 V best (5 to 7 V works); 2 to 3 A leaves headroom. A bench supply with a current limit is fine too. The servos are rated 4.5 to 7.4 V (torque figures at 6 V); the TTLinker's datasheet asks for 6 V or more for its 5 V regulator. |
| Supply leads | 0.5 mm² (20 AWG) or thicker, with a switch in the + lead if the supply has none. |
| Micro-USB cable | Power and serial console for the ESP32: flashing, the `set_id` tool, console commands. |
| Ethernet cable, router or switch with DHCP | Same network as the PC. At the bench, a port without PoE. |
| Multimeter | For the checks in 2.5. |
| Jumper wires | Female to female, for the header pins. |

## 2. Wiring

### 2.1 Signal levels

The TTLinker's logic runs at 5 V, the ESP32's at 3.3 V. Both signal lines are plain wires.

- **TTLinker to ESP32 (RX0 to GPIO33).** The TTLinker drives this line to 5 V, above the ESP32's 3.3 V rating
  (3.6 V at most). The first arm runs it this way. For margin, a divider brings it to 3.3 V: 1 kOhm from RX0 to
  GPIO33 and 2 kOhm from GPIO33 to GND.
- **ESP32 to TTLinker (GPIO32 to TX1).** 3.3 V into a 5 V 74HC chip is at the edge of what it reads as high: it
  usually works, but is not guaranteed. The scan in step 3 tells: every servo should answer 5 of 5 pings. If they
  do not, add a 74AHCT125 buffer (TTL-level input) powered from the TTLinker's 5 V pin, or use a level shifter
  rated for 1 Mbit/s.

The servos themselves accept 2 to 5 V signals (HLS3606 datasheet), so only the adapter is the issue.

### 2.2 Diagram

```
  6.0 V supply (+) --[switch]-----------------------> TTLinker VIN
  6.0 V supply (-) ---------------------------------> TTLinker GND <------ ESP32-PoE GND   (left header, pin 3)

  ESP32-PoE GPIO32 (right header) ------------------> TTLinker TX1        (ESP32 transmits)
  TTLinker RX0 -------------------------------------> ESP32-PoE GPIO33    (right header, ESP32 receives)

  TTLinker servo port (S V G) ---> J1 ---> J2 ---> ... ---> J7 (trigger)     all 7 on one bus
  TTLinker 5V pin: NOT connected (it is 150 mA at most, and the ESP32 has its own supply)

  ESP32-PoE micro-USB ---> PC (power + serial console)
  ESP32-PoE Ethernet  ---> router or switch, port WITHOUT PoE
```

### 2.3 Connections

| From | To | Notes |
|---|---|---|
| Supply + (through the switch) | TTLinker VIN | 6 V (5 to 7 V works). The servos get their power through the TTLinker's servo ports. |
| Supply - | TTLinker GND | |
| TTLinker GND | ESP32-PoE GND, left header pin 3 | Common ground: without it nothing works. |
| ESP32-PoE GPIO32, right header | TTLinker TX1 | The TTLinker's pins are named after the Arduino pins they go to: TX1 takes the controller's TX. |
| TTLinker RX0 | ESP32-PoE GPIO33, right header | A plain wire. It carries 5 V: see 2.1. |
| TTLinker servo port | J1 servo, then the chain to J7 | Servo cable (5264-3P): pin 1 GND, 2 Vcc, 3 signal. Plug and unplug servos only with the supply off. |
| ESP32-PoE micro-USB | PC | |
| ESP32-PoE Ethernet | network port without PoE | |

The firmware uses UART2 on GPIO33 (RX) and GPIO32 (TX) at 1 Mbit/s (`main.cpp`, `src/tools/set_id/main.cpp`).

### 2.4 No buttons

This setup has no buttons, and the firmware reads none (`CLUTCH_BUTTON_PIN` and `ENABLE_BUTTON_PIN` are -1 in
`src/main/main.cpp`; the board's BUT1 does nothing). The web UI does their jobs: **Hold PAR6** / **Release hold** on
the Teleop page is the clutch, and **Resume** reclaims PAR6 after an outage.

### 2.5 Checks before the first power-up

1. **Supply.** Set it to 6.0 V and the current limit to 2 A, with the output off. Nothing is connected yet.
2. **No short.** With everything wired and the supply off, measure between Vcc and GND at a servo connector: no
   short (more than a few hundred ohms).
3. **Ground.** Beep through from the supply's minus to the TTLinker GND and to the ESP32's GND pin.

## 3. Check the servo IDs

The firmware expects IDs 0 to 6 in joint order, as on PAROL6 and PAR6:

| Joint | J1 base | J2 shoulder | J3 elbow | J4 | J5 | J6 | J7 trigger |
|---|---|---|---|---|---|---|---|
| Servo ID | 0 | 1 | 2 | 3 | 4 | 5 | 6 |

This step checks them and fixes any that are wrong. It is also the first test of the wiring: a servo that answers
means both directions of the bus work.

1. **Flash the ID tool** (over USB). pio must run in `firmware/`, where `platformio.ini` is; the upload
   ends with SUCCESS:
   ```
   cd firmware
   pio run -e set_id -t upload
   pio device monitor -b 115200
   ```
   It scans the bus when it starts (about 5 s), usually before the monitor is open. If the screen stays empty, wait
   for the help line (it repeats every 5 s until you type) or just type `s` and Enter.
2. **All seven connected, supply on: scan.** You want every joint ID answering 5 of 5 pings and nothing else:
   ```
     ID   0  J1 base      answered 5/5 pings, position 2047
     ID   1  J2 shoulder  answered 5/5 pings, position 1022
     ...
     ID   6  J7 trigger   answered 5/5 pings, position 1210
   7 servo ID(s) answer.
   OK: IDs 0-6 all answer, nothing else on the bus. Type w to check which joint is which.
   ```
   Seven servos on IDs 0 to 6 also means no two share an ID. If so, go on to 4.
3. **If the scan is not OK,** add the servos one at a time. Switch the supply off whenever you plug a cable in or out.
   1. Start with one servo on the bus (plug the bus cable straight into it if needed) and type `s`.
   2. To renumber, type the ID the scan shows, Enter, then the joint's ID from the table, Enter. For example, the
      trigger on ID 2: `2`, then `6`. Then `s` to confirm. `x` cancels.
   3. Supply off, plug in the next servo, supply on, `s`. Exactly one new ID should appear: renumber it the same way.
      - If an ID that was there disappears, the new servo has the same ID. Disconnect the others and renumber the new
        one alone.
      - If nothing new appears, that servo answers on no ID: no power or signal at its connector, or a baud rate
        other than 1 Mbit/s.
   4. New servos come as ID 1, which is J2's ID. Leave J2 for last, or renumber every other servo that shows up as
      ID 1 before adding the next one.
   5. **Power-cycle check.** At the end, switch the supply off and on and scan again: the IDs must survive it.
4. **Which joint is which.** Type `w`: the tool prints the seven positions five times a second. Turn J1 by hand: only
   the J1 column may change. Then J2, and so on to the trigger. Enter stops it. If two joints are swapped (turning J3
   changes the J5 column), swap their IDs through a free one: `2` then `20`, `4` then `2`, `20` then `4`.

If nothing answers at all: supply on, common ground, TX1 and RX0 not swapped (2.3, 2.5). If every servo
is "unreliable": the GPIO32 note in 2.1.

`t` measures every servo's reply time (see step 5). The tool leaves torque off. Type `x` when done; step 4 flashes the real firmware over it.

## 4. Flash and watch the boot

```
cd firmware
pio run -e main -t upload
pio device monitor -b 115200
```

Over USB, as in step 3. Switch the servo supply on before the ESP32 starts, or press the ESP32's reset button after
switching it on (the firmware waits 10 s for all servos, then halts). You should see, in this order:

```
=== RAFT leader arm booting ===
Servo bus up, torque OFF sent to every servo.
Model self-test: PASS  (42 cases, ...)
FK + gravity: ... us per call
Waiting for all servos...
All servos online, torque OFF.
Settings: NOT calibrated (see docs/calibration.md), model OK
Running. Type 'status', or see docs/teleop_protocol.md. 'quiet' stops the status line.
Network up: 192.168.x.y (raft-leader.local), commands on TCP 5000
Web UI: http://192.168.x.y/  or  http://raft-leader.local/
```

Then one status line per second. Type `quiet` to stop it while you type commands.

In the router, give the ESP32 a fixed address (a DHCP reservation for its MAC). PAR6's allow-list needs it in
stage 15.

**More than one arm on the network.** Every arm starts as raft-leader.local, so name each one: Settings, This
arm (or `set_name` on the console), then Save and a restart. An arm named `left` answers at raft-left.local and shows
its name in the header and the browser tab. Each arm serves its own page: open one tab per arm. For a network update,
pass that arm's address: `pio run -e main_ota -t upload --upload-port raft-left.local`.

## 5. Check the bus timing

The end of each status line has the control loop's numbers for the last second:

```
loop max 3700 us, read max 3100 us, overruns 0, failed reads 0
```

| Value | Expect (first arm, limp) | Why it matters |
|---|---|---|
| read max | about 3100 us | One SyncRead of position and speed from all 7 servos (about 1.9 ms), plus voltage, temperature and current from one servo in turn. The driver gives up after 3 to 4 ms. |
| loop max | about 1500 us | The CPU part of the 250 Hz tick (4000 us budget); the servo replies come in while the task sleeps. |
| overruns | 0 | Ticks that started late. |
| failed reads | 0 | Reads where at least one servo did not answer. |

The servos answer slowly: about 0.3 ms after a request, then about 21 us per byte. The ID tool's `t` command (step 3)
measures every servo's reply time if these numbers ever change.

## 6. Open the web UI and read the servos

Open the address from the boot log, or http://raft-leader.local/. The header shows a green "live" dot and the 3D
arm. The view says "Not calibrated yet": the angles are uncorrected until step 8.

Go to **Settings, Servos, Read servo settings** (or type `servo_info` on the console). You should see:
- **Answers.** All 7 servos answer, each with the ID of its joint.
- **Supply.** About 6 V, and the same on every servo within about 0.5 V. The HLS3606 is rated 4.5 to 7.4 V; its
  torque figures are at 6 V.
- **Temperature.** Room temperature.
- **Notes.** The line under the table says "Nothing stands out", or explains what does.

Press **raw** on one row and copy the register dump. It tells us the HLS3606's settings that the driver does not
name, such as protection limits.

## 7. Move the limp arm by hand

The arm is limp. Turn each joint slowly by hand and watch the **Joints** panel under the 3D view: its angle should change smoothly as
you turn, with no jumps, for every joint. The 3D arm moves too, but in wrong directions and from a wrong zero
until the next step. The Joints panel under the 3D view also shows the measured current (near 0 now) and each servo's voltage.

## 8. Zero at the stops

Every joint has a mechanical stop at a known angle, and every servo turns a known way round. Both are the same on
every RAFT, so they are in the firmware (`src/main/arm_design.h`), measured on the first arm:

| Joint | J1 | J2 | J3 | J4 | J5 | J6 | Trigger |
|---|---|---|---|---|---|---|---|
| Reads at its stop (deg, standby = 0) | -136.28 | -83.96 | 92.81 | -158.56 | 119.72 | 149.37 | 39.63 |
| Servo direction | reversed | | | | | | reversed |

Only the encoder zeros are per arm. On the **Calibrate** page, step 1 guides you through them (details in
`docs/calibration.md`). The arm stays limp:
1. **One joint at a time.** Turn the joint shown gently against its stop and keep it there. Press **Capture** and
   hold it still for a moment. The joint gets a ✓ and the guide moves on to the next one.
2. **Check standby.** Put the arm in the standby pose: upper arm vertical, forearm level and pointing forward, wrist
   straight, trigger parallel to the handle's face. Every joint should read about 0 deg in the Joints panel.

Now the strongest check of the whole day: **the 3D arm must move exactly like the real one.** Move each joint by
hand; the on-screen joint must turn the same way and by the same amount. If one turns the opposite way, its servo
is mounted differently from the design: check it against the [assembly manual](../../Assembly%20manual/readme.md)
before going on. Then step 3 on the page (choose "No spring") and
step 4, Save. The zeros go into the ESP32's flash and stay there through power-offs and firmware updates.

This is done once. The servos' encoders are absolute and drive their joints directly, so the zeros survive every
power-off. Redo a joint only when its servo, its horn or its M3 stop screw was taken off. Every joint must be within half a turn of the
standby pose when the arm powers up: the firmware counts turns from where it starts.

The joint ranges are not measured per arm: they belong to the design, like the stops. Until they are final, the
firmware uses the CAD estimates (J2 -105..130, J3 -75..150 deg) and +-180 deg placeholders for the other joints.
They clamp move targets and place the haptic barriers.

## 9. First torque, at half gravity

1. **Half compensation.** On the **Drive** page, set Gravity compensation to 0.50, before any torque is on.
2. **Gravity on.** Hold the arm in the standby pose, press **Gravity**, and keep holding.
3. **Let go slowly.** The arm should feel lighter but still sink slowly.
4. **Abort if needed.** If a joint pushes away or kicks, press Esc at once, then check that joint's sign.
5. **Raise in steps.** Go to 1.00 (real gravity). Friction compensation (holding) below it starts at 0.30 on a new
   arm: with both, the arm stays in most poses. Without the spring, J2 still sinks in the stretched poses.
6. **Check the load.** The Joints panel shows, per joint, the commanded (Cmd), measured (Current) and model (Model) current in mA. In the standby pose the
   model expects about 360 mA on J2 and J3; cmd is 1.3 times that (gravity 1.00 + friction 0.30).
7. **Hold mode.** Press **Hold**: the arm keeps its pose and pushes back when you move it. Hold stiffness 1 on the
   Drive page; above about 2 the real arm starts to shake.

Why friction compensation (holding) on top of real gravity (measured on the first arm): the HLS3606's gears run easily when gravity drives them from the joint,
but the motor needs about 0.3 Nm more to drive them itself (J3 lifted its hanging forearm only at about 650 mA). At
gravity alone the arm sinks wherever the model is a little short, and an extra 30 % is far too little to lift anything
through that friction. Pushing a joint more than 8 deg away from its pose in Hold hands the arm over to Gravity (the grab
rule).

If the firmware faults with "a servo refused constant-current mode", the mode switch (register 33) did not take:
read the servo settings (Servos page) and check the Mode column; if it stays, open an issue with the servo_info dump.

## 10. What the servos do when the ESP32 stops (two short checks)

Hold the arm in the standby pose for both, with gravity on at 1.00.

1. **Reset.** Press the ESP32-PoE's reset button. The servos stay powered; the boot sends torque off before anything
   else, so the arm must go limp within about half a second (you feel it drop into your hand). If it keeps pushing
   for longer, note how long.
2. **Bus lost.** Pull the servo bus signal wire for about a second, then plug it back. With torque on the firmware faults to LIMP
   after 200 ms ("servo ... not answering"), but the servos cannot hear that while the wire is out: note whether they
   keep their last current (expected: the HLS3606 has no command watchdog we know of) or let go by themselves.
   When the wire is back, they must let go at once. Then `clear_fault`.

The answer tells us whether a dead ESP32 leaves the arm pushing. The firmware's task watchdog resets the ESP32 when
the control task stalls for 2 s, and the reset switches torque off (check 1).

## 11. Torque constants

Nothing to do: every joint uses the datasheet value, 0.441 Nm/A (4.5 kg.cm/A), with the 3.25 mA current register
unit. On the first arm that matched a direct test of the J3 servo. The web UI has no measuring step
for it any more; the firmware's torque sweep (`calib_torque`) still exists for the console. On that arm the gear
friction kept J3 from moving even at 0.5 Nm, so the sweep had no result ("moved only 0.0 of 8 deg").

## 12. Friction compensation while moving

Not in use yet. Settings, Teleoperation and haptics, "Friction (moving)" scales a measured friction per joint, and
nothing measures it now (the torque sweep did): with friction 0 for every joint the setting does nothing. The
holding friction on the Drive page is a different thing and is in use.

## 13. The spring, when it arrives

The ordered springs (AliExpress, 0.8 x 6 mm, 35 mm main, 40 mm spare; 0.8 x 7 x 30 mm backup) come without data.

1. **Measure two or three.** The water-bottle test in `docs/spring_mount.md` ("Any other spring"): 0.5 l, then 1.5 l.
   Rate = 9.8 / (L2 - L1) N/mm; expected about 0.8-0.9 N/mm for the 35 mm. Initial tension / rate must be under
   about 5.5 mm, or the spring goes slack near vertical: then take the 40 mm or the 7 mm spring.
2. **Mount.** Mount the spring and the cable as in `docs/spring_mount.md` (anchor 13 mm above J2, eyelet 20 mm up the
   upper arm). With the upper arm vertical the spring is stretched by 7 mm - (initial tension / rate).
3. **Tell the firmware.** Calibrate step 3, or `{"cmd": "spring", "type": "cable", "k": 850, "a_mm": 13, "b_mm": 20}`
   with your measured rate in N/m instead of 850.
4. **Correct it.** Calibrate step 7 (`calib_spring`, upper arm tilted at least 40 deg). Save.
5. **Check.** Gravity mode should hold everywhere, and J2's current should drop to about J3's level (about 385 mA)
   when the arm leans forward.

## 14. Running on its own (PoE)

Once the arm is calibrated and saved, it needs no PC:
1. **Power down.** Servo supply off, USB **unplugged**.
2. **PoE.** Network cable into a PoE port (802.3af).
3. **Servo supply on.** The ESP32 boots, finds the servos and comes up limp. Use the web UI or TCP port 5000; the
   serial console is gone with the USB.

With the board on PoE, its ground is not isolated from the PoE side: the servo supply's output must be floating
(not connected to mains earth). A bench supply's output usually is; check the adapter if you use one. Never plug
USB in while PoE powers the board. Firmware updates over the network (OTA) work in this setup.

## 15. The PAR6 link, in three stages

Each stage needs the one before it.

### 15.1 PAR6 side, once (no arm needed)

1. **Allow the ESP32.** Add the ESP32's fixed address to `RCB_ALLOWED_IPS` on the Pi (environment variable, never
   committed; `docs/rcb_link_test.md`).
2. **The recipe.** Copy `docs/rcb/raft.xml` into the runtime's `data_recipe/` folder.
3. **Test B.** Run test B in `docs/rcb_link_test.md`: `rti_probe.exe` on the PC against the Pi's simulated runtime
   (`sim_run.py`). It checks pairing, the state stream and a claimed motion without any real robot moving. This
   stage can be done now, before the arm is wired.

### 15.2 The leader against the simulated runtime

The Pi runs `sim_run.py` (no real drives). On the Teleop page enter the Pi's address and press Start:
1. **Couple.** RAFT moves itself onto the simulated PAR6's pose, claims control and shows COUPLED.
2. **Follow.** Move the leader: the simulated PAR6 follows (the view can show it: "Show the follower").
3. **Hold.** Press Hold PAR6: the stage goes to HELD. Press Release hold: COUPLED again.
4. **Realign, precision, lock.** Try Realign, the precision slider and a joint lock.
5. **Outage.** Unplug the ESP32's network cable for 2 s: the leader goes weightless, the stage shows LOST. Plug it
   back and press Resume.

### 15.3 The real PAR6

**Before you start:**
- PAR6 in a clear area, near its standby pose, with its own E-stop in reach of a second person (or yours).
- The leader first moves itself onto PAR6's pose; PAR6 moves only after the claim, and only as you move the leader.
- If PAR6 stands where RAFT cannot follow, the Teleop page says which joint (for example `J3: PAR6 is at 160.0 deg,
  RAFT reaches -75..150 deg`) and nothing moves. Jog PAR6 into range, or tick "couple with an offset".

**The first session:**
1. **Small and slow.** Couple, then move the leader slowly and in small steps. Check that every PAR6 joint follows in
   the right direction.
2. **Gripper scale.** Squeeze the trigger slowly: the gripper should close over the trigger's travel, and the follower
   pose's trigger value should run from open to closed as the gripper's raw position goes 0 to 255. Set the
   trigger's real travel and the gripper's open / closed positions in Settings, Teleoperation and haptics.
3. **Limits.** Move the leader toward a joint limit: before PAR6 reaches its soft limit, the leader starts to push back
   (the barriers).
4. **Clutch.** Press Hold PAR6, move the leader, press Release hold: PAR6 stays put while held, and continues from
   where it is when you release. Realign removes the offset.
5. **Contact reflection sign.** Set the force gain to 0.02. While coupled, push PAR6's forearm gently by hand: the
   leader must push your hand the same way you pushed PAR6. If it pulls the other way, set the force sign to +1.
   Then raise the gain slowly (0.05 felt right in the simulator).
6. **Outage.** Unplug the ESP32's network cable for 2 s: PAR6 stops (its own watchdog), the leader goes weightless,
   the stage shows LOST. Plug it back and press Resume.

## Reporting a problem

When you open an [issue](https://github.com/Source-Robotics/RAFT-teleoperation-arm/issues) or ask on Discord, these help:

- **IDs.** The scan from step 3.
- **Boot log.** The first screen of the serial monitor.
- **Bus timing.** The numbers from step 5.
- **Servos.** The servo_info notes and one raw register dump.
- **Currents.** Commanded and measured J2 / J3 current in the standby pose (step 9).
- **Stops.** What the servos did in the two checks of step 10.
- **Health.** The Health page after a few minutes in gravity mode: loop rate, jitter, late ticks, failed bus reads
  and the missed counts per servo.
- **Odd things.** Anything that surprised you, with the console output.

## When something is off

| You see | Usually means |
|---|---|
| The set_id scan finds nothing | Supply off, no common ground, or TX1 / RX0 swapped. |
| The scan marks a servo "unreliable" | One ID only: two servos probably share it (step 3, one at a time). Every servo: GPIO32's 3.3 V is at the edge of what the TTLinker reads as high (2.1); add the 74AHCT125 buffer. |
| `BOOT HALTED: not all servos answer. Missing: J4_WristP (ID 3)` | No servo power, a loose bus cable, or a wrong ID. Check with the `set_id` tool's scan (step 3). The firmware retries for 10 s, then waits for a reset. |
| All seven missing at boot | The same as the first row: supply, ground, TX / RX. Or the servo supply came on after the 10 s: press reset. |
| No "Network up" line appears | No DHCP lease. The arm runs and the serial console works for everything; the web UI needs a network with a DHCP server. |
| failed reads > 0 every second, all servos present, mostly the last one (J7) | The SyncRead takes longer than the driver's 3 to 4 ms timeout. Measure with the ID tool's `t` command (step 3) and include the output; the timeout is the `syncTimeoutMs` default of `ServoBus::begin()` in `servo_bus.h`. |
| overruns > 0 every second | Ticks come late: the Health page shows which part of the tick takes the time. |
| A joint's angle jumps by about 180 deg when you turn it | Not calibrated yet. Step 8 fixes it. |
| The 3D arm turns a joint the wrong way after step 8 | That servo is mounted the other way round than the design (`kJointDir` in `arm_design.h`): check it against the assembly manual. |
| `stop: J3 moved by 25 ticks while sampling` | The joint was not held still against the stop for the 0.3 s after Capture. Press Capture again. |
| A joint does not read about 0 in the standby pose after step 8 | It was not fully against its stop at Capture, or the stop angle in `kStopDeg` is off. Redo it; if the offset stays, open an issue with the reading. A joint off by about twice its stop angle turns the other way round than `kJointDir` says. |
| "fault: servo J2 not answering" | With torque on, a servo missed 50 reads in a row (200 ms; 6 during a calibration). Shorter gaps are ridden through on its last angle. Torque goes off. Check the cable, then `clear_fault`. Limp, a missing servo only counts, and torque is refused while it has missed its last 10 reads. |
| The arm sinks with gravity 1.00 | Expected on these servos (step 9): friction compensation (holding) 0.30 on the Drive page. Still sinking in most poses: check `ma_per_unit` is 3.25 (Settings, Behaviour). |
| The arm floats up | Friction compensation (holding) or gravity compensation too high, or a spring stronger than configured. Lower it by 0.05 at a time. |
| Measured and commanded current disagree by a constant factor | The current register unit is not 3.25 mA (Settings, Behaviour). |
| A joint goes weak after holding a heavy pose for a few seconds | The servo's own overload protection (about 80 % load for 2 s, datasheet). Mostly J2 with the arm stretched: the spring helps. |
| A joint gets hot (amber marker at 55 deg C) | J2 carries most of the load. Torque derates from 60 deg C and faults at 75. The spring will fix most of this. |
| The web page does not load | Try the IP address instead of raft-leader.local. Windows needs mDNS, which Windows 10 and later have. |
| `set mode limp first: this changes how torque is computed` | Signs, Kt, the current unit, `load` and `defaults` change only while limp. Press LIMP (support the arm), then retry. |
| `save` answers "teleoperation is running" or "a calibration is running" | Writing flash stalls the control loop for tens of ms, so it waits until the arm is quiet. Stop first, then save. |
| Teleop says `J3: PAR6 is at 160.0 deg, RAFT reaches -75..150 deg` | PAR6 stands where the RAFT cannot follow. Jog PAR6 into range, or allow an offset. |
| Teleop: `FORBIDDEN: add this IP to RCB_ALLOWED_IPS` | The ESP32's address is not on PAR6's allow-list (15.1). With DHCP the address may have changed: reserve it. |
| Teleop stage LOST | The link dropped (see `link_error`). The leader is weightless; press Resume to realign and reclaim. |
| Teleop stage HELD with "target jumped" | A joint reading jumped (an encoder slip or a loose cable). Check the joints, then press Hold PAR6 and Release hold, or Realign. |
