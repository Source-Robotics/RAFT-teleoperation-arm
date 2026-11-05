# RAFT web UI

The ESP32 serves its own control page. Open **http://raft-leader.local/** or the arm's IP address from any
browser on the same network: PC, tablet or phone. The page comes from the ESP32's flash, so it needs no internet
and no PC software. It uses the system's own fonts (Segoe UI on Windows, San Francisco on Apple, Roboto on Android),
so no font file sits in flash.

## What is on the page

The page is built like an engineering tool, in the Source Robotics colours, light or dark (Settings, Display):
- **Header.** The connection, a search box (Ctrl K), state chips (COUPLED, HELD, DERATING, CALIBRATING, MODEL FAILED), Save (highlighted while there are unsaved changes) and the red **LIMP** button.
- **Menu on the left.** Grouped as Operate (Drive, Teleop), Set up (Calibrate, Servos, Settings) and Diagnose (Plots, Health, Console). Badges show the teleoperation stage, a missing calibration, unsaved changes and the servos that miss reads.
- **Middle.** The 3D view, with the bottom panel under it: **Joints**, **Live plot** and **Console**. Drag its top edge to resize it, or fold it away.
- **Inspector on the right.** The settings of the page chosen in the menu.
- **Status bar at the bottom.** Always shows the control loop's rate, jitter, tick work and late ticks, the CPU load of both cores, missed servo reads per second, the ping round trip, free memory and the Ethernet link; clicking it opens Health.

Plots and Health use the whole width: they replace the 3D view, so the page asks for 5 live frames a second while one is open.

LIMP and the Esc key switch the torque off from anywhere. With the search box or a confirmation open, Esc first closes it, so dismissing a box never drops the arm; the next Esc goes limp.

The 3D view shows the real arm live: the CAD meshes, reduced to 20,000 triangles, drawn with WebGL at the joint
angles the firmware reports, moved by the same forward kinematics the firmware uses. Drag to turn it, right-drag
or Ctrl-drag to pan, scroll or pinch to zoom, and double-click to reset; 3D, Side, Front and Top set the camera. A
joint whose servo runs at 55 °C or more gets an amber marker with its temperature, red from 65 °C. An amber
see-through arm shows a move target while you edit it, and a cyan one shows the PAR6 follower during teleoperation
(the PAR6 switch in the view's toolbar). Without WebGL, or before the model has loaded, the view draws a simple
stick arm instead. The top left corner shows the tool point (X, Y, Z in mm and Rx, Ry, Rz) in degrees or radians
(DEG / RAD, remembered by the browser). The bottom left corner counts how often the page redrew the arm, the live
frames per second, and how old the pose on screen is (half the best round trip, plus how late frames arrive, plus
half a frame interval).

The **Joints** panel has one row per joint: angle, where it is in its range, speed, the current the servo measures
(the bar marks the rated 350 mA; amber above it, red above twice it), the commanded current (torque / Kt, or "pos"
while the servo runs its own position mode), what the model's gravity load needs, torque, temperature, supply voltage
and missed reads per second over the last 10 s. A gap between commanded and measured current points at saturation, a
supply problem or a wrong Kt.

**Search (Ctrl K or /)** finds every card on every page by its title or any word on it ("stiffness" finds Drive ›
Hold) and opens it, and runs actions: go to a page, a mode, save, standby or rest pose, teleop start, stop, hold and
realign, the 250 Hz capture, the bus trace, reading the servo settings, the theme. Anything else typed there, like
`mode gravity` or a JSON command, runs on the arm like in the Console.

The longer explanations sit behind the (?) on each card; Settings, Display shows them all at once.

| Page | What it does |
|---|---|
| Drive | Mode (Limp, Gravity, Hold), gravity compensation and friction compensation (holding), Hold stiffness and the servos' own position mode, per joint the gravity factor and damping, move to a pose with sliders and a speed, standby and rest poses. |
| Teleop | PAR6 host, start and stop, the stage (WAIT_PAIRED, ALIGNING, CLAIMING, COUPLED, HELD, REALIGNING, LOST) as four steps and the session's note (why it waits, e.g. "J3: PAR6 is at 160.0 deg, RAFT reaches -75..150 deg"), a large Hold PAR6 / Release hold button (a clutch in the page), Realign (removes the offset), Resume after a lost link, coupling with an offset when PAR6 is outside RAFT's range. The link (Real RTI at 250 Hz, or SOFT_RTI at 50, 25 or 10 Hz), a low-pass filter on the targets sent, link errors, coupling error, offset, engaged, the handle buttons, dropped packets. Assist: precision (0.05 to 1), joint locks J1 to J6, the wrist-centre guide (horizontal plane, plane across the tool axis, line along the tool axis, vertical line), orientation lock, PAR6's live t_ext, gripper position and the passivity credit. Where the UDP leader stream goes. |
| Calibrate | Seven steps that tick themselves off from the arm's state, with live progress while a calibration runs: zero at the stops, the fixed directions, the spring, save, first torque, spring strength (with a spring), save and test. The torque constants come from the datasheet and the joint ranges from the design, so neither is measured here (`calib_torque` and `calib_range` still work from the Console). |
| Servos | The servo position test (one servo in its own position mode, jogged with a slider, with its live position, current, torque, temperature and supply), the servo register tables (`servo_info`) with plain-language notes and raw registers, and Kt per joint. |
| Settings | Save, reload or reset the settings (save is refused while teleoperation, a calibration, a move or a replay runs), the arm's name (header, tab title and raft-*name*.local), teleoperation and haptics (reclaim by itself after an outage, require the enable button, passivity damper, barriers outside teleoperation, contact-force gain, sign and deadband, friction compensation while moving, trigger and gripper travel), auto-rest, the current register unit, theme, live rate, explanations on every card. |
| Plots | A 250 Hz capture of 2 s recorded on the arm (`scope_start`): one joint's angle, speed, measured current or torque raw, through two filter cutoffs computed in the page with the firmware's own target filter, and the target really sent to PAR6, with each filter's measured lag. The live 30 s chart from the frames (angles, speeds, tool point, currents, torques, voltages, temperatures) is the Live plot tab under the 3D view. |
| Health | The control loop over the last second: rate, period, jitter (RMS deviation from the 4 ms period), tick work, late ticks and failed reads, one tick drawn as a timeline split into servo read, waiting for the App lock, controller and servo writes, with a two-minute chart of the longest period and the longest tick. The servo bus: missed reads per servo, per second and since the start. The processor: CPU load per core, free memory and its low point, stack left per task, uptime, why it last reset, clock, firmware, model check. The network: live frames, live data delay, frame spacing jitter, ping round trip, link speed, address, clients. The bus diagnosis (`bus_diag`, on the arm only): per servo the misses, silent or garbled, gap lengths, error flags and retries, a 20 s byte trace and the raw bytes of the last misses. |
| Console | Under the 3D view: any command from `docs/teleop_protocol.md`, as JSON or shorthand like `mode gravity`, with a log of replies and events. |

## Power and network use

The ESP32 sends nothing until a page is open, so with no page open the web UI costs only a few kB of idle
RAM structures. With a page open:

| Item | Cost |
|---|---|
| Page download | 53 kB gzipped, plus 86 kB for the arm model, once; later loads are answered with "not modified" |
| Live data | the 288-byte leader frame at 20 frames/s by default (5 to 50 in Settings), about 6 kB/s; 5 a second while Plots or Health is open |
| Status | about 1.9 kB, only when something in it changes (a setting, the mode, a stage, a text: checked 10 times a second), and every 5 s regardless |
| Health | one 180-byte binary message a second: loop timing, CPU load, memory, network counters, missed reads per servo |
| Hidden tab | frames and health messages stop; the status still comes when it changes |
| ESP32 RAM | about 43 kB of heap for the server (4 connections with 1.5 kB receive and 8 kB send buffers, a 4 kB status buffer); static RAM 76 kB (23.2 %) in total |
| ESP32 flash | 139 kB for the page and the arm model; the firmware uses 83.7 % of one over-the-air slot |
| Browser | the arm is redrawn only when a frame arrives or the view moves; 20,000 triangles are light work for any phone's GPU |

Printing the status (about 110 decimal numbers) is the costly part of serving a page on the ESP32's network core, so a
page takes it only when it changes and gets the figures that change every second as a binary message instead. The
control loop runs on the other core and never waits for either: the status is printed from a copy taken under the App
lock in microseconds.

Writes to the browser never block. A slow or stalled browser gets fewer frames, and one that stops reading for
30 s is dropped. The PAR6 link runs in the same network task and cannot be held up by a browser. Up to four
connections are served at once.

## Security

Like TCP port 5000, the page has no login: anyone on the network can drive the arm. Keep the arm on a bench
network you trust.

## Protocol

| Request | Answer |
|---|---|
| `GET /` | the page, gzipped, with an ETag |
| `GET /arm.bin` | the arm model, gzipped, with an ETag |
| `GET /api/status` | the status JSON |
| `POST /api` with one JSON command as the body | the command's JSON reply, for scripts and `curl` |
| `GET /ws` | WebSocket, see below |

On the WebSocket the page sends text commands, the same JSON as TCP port 5000, with an extra `"id"`. The
ESP32 sends:

| Message | Meaning |
|---|---|
| `{"t":"hello","r":{...}}` | once: firmware, DH model, gripper hinge, joint limits, rest pose, spring presets, and `sys` (why the ESP32 last started, clock, address, host name) |
| `{"t":"s","r":{...}}` | the status: when it changes (`"status": "change"`) or a number of times a second (`status_hz`) |
| `{"t":"r","id":N,"r":{...}}` | the reply to command N |
| binary, 288 bytes, starts "RAFT" | the leader frame, version 2 (layout in `docs/teleop_protocol.md` and `src/main/leader_frame.h`) |
| binary, 180 bytes, starts "RFTH" | the health message, once a second with `"health": true` (layout in `src/main/web_bridge.h`) |

`{"cmd":"ws_rate","frame_hz":20,"status":"change","health":true}` is what the page sends, for itself only; `"status_hz": N`
(0 to 10) asks for the status N times a second instead, as scripts and `web_flow.mjs` do. A new connection gets the
status twice a second until it says otherwise. `{"cmd":"ping"}` answers
`{"ok": true}` straight from the web server, without the App lock: the page times it once a second.

On the arm the status carries a `sys` object, for scripts and `GET /api/status`: `uptime_s`, `reset` (why the ESP32 last
started), `cpu_mhz`, `cpu_load` (percent per core, from the idle tasks), `loop` (the last one-second window:
`seq`, `target_hz`, `hz`, `period_us` [min, mean, max], `jitter_us`, `work_us`, `read_us`, `lock_us`, `app_us`, `write_us`
[mean, max], `overruns` and `failed_reads` [last second, since start]), `heap` (`free`, `min`, `largest`,
`size`), `stack_free` per task in bytes, and `net` (`ip`, `link_mbps`, `full_duplex`, `web_clients`,
`tcp_clients`, `web_frames`, `udp_frames`). web_sim sends its own loop timing and network counts. The page reads the
same figures from the health message, and the missed reads and PAR6's dropped packets too, which change every tick.

## Changing the page

The page is one file, `web/index.html`, with no external libraries. PlatformIO re-embeds it and the arm model
into `src/main/web_assets.h` before every `pio run -e main` when either changed (`tools/pio_build_web.py`).
`python tools/build_web.py --force` does the same by hand.

Opening `web/index.html` straight from disk shows a demo with an animated arm, for working on the layout. It
shows the stick arm, because a page opened from disk may not read `arm.bin`.

## The arm model

`web/arm.bin` is made from the CAD by `python tools/build_arm_mesh.py`. Rerun it after a CAD change, after
`tools/gen_model.py`. It:
- **Places the meshes.** It loads the MuJoCo model, so every mesh sits where the simulator puts it.
- **Re-expresses them.** It stores each link's vertices in the DH frame that link rides on, so the page only applies forward kinematics.
- **Checks the mapping.** It compares against MuJoCo at 25 random poses; the worst vertex error is 3 nm.
- **Shrinks them.** It decimates the 84,232 CAD triangles to 20,000, keeps 16 bits per coordinate, and delta-encodes vertices and indices so gzip works well.

| Triangles (`--tris`) | arm.bin gzipped | Look |
|---|---|---|
| 20,000 (default) | 86 kB | indistinguishable from the CAD at normal zoom |
| 30,000 | 124 kB | same |
| 40,000 | 160 kB | same |
| 84,232 (all) | 485 kB | does not fit the over-the-air slot next to the firmware |

Decimation needs `pip install fast-simplification`. Its aggressiveness stays low (`--agg 2`): the fast default
crumples the CAD's thin features.

## Trying it without the arm

`tools/sim/out/web_sim.exe`, built by `tools/sim/build.cmd`, runs the real firmware App and the same web server
on the PC, against the MuJoCo arm and an emulated PAR6. Start it and open http://localhost:8080/.

| Option | Meaning |
|---|---|
| `--fresh` | forget saved settings: start uncalibrated, so Calibrate can be walked through |
| `--calibrated` | start with the zeros already measured |
| `--bind 0.0.0.0` | serve the whole network, e.g. a phone; Windows asks for firewall permission |
| `--port N` | another port than 8080 |
| `--spring-k N` | the installed spring's real rate in N/m, default 906 (Z-078I plus 10 %) |

The simulated servos have unknown zeros and torque constants 10 % off, like real ones, and the design's directions. A simulated
hand stands in for the person at the bench, and the simulator card in the 3D view controls it:
- **Holding.** It holds the arm whenever the arm is limp, and lets go when torque comes on.
- **Calibration.** During zero calibration it puts the arm in the zero pose, and during direction calibration it turns the joint.
- **Pulling.** "Drag pulls the arm" or Shift-drag pulls the arm by its tool point. In gravity mode the arm floats along. In Hold mode, a pull of more than 8° hands the arm back to gravity.

Teleop works with any host name, against an emulated PAR6 whose follower tracks the leader. Its J2 meets a
table at 60° forward (CAD), where its motor pushes and t_ext rises, and its gripper closes on an object at raw
150 of 255. `{"cmd": "sim_button", "name": "enable" | "clutch", "down": true | false}` presses the handle
buttons a real arm would have (the simulator reports both as wired).

## Tests

| Test | What it checks |
|---|---|
| `tools/sim/out/test_web.exe` | the server: HTTP, ETag, WebSocket handshake and framing, rates, the status on change and the health message (and its byte layout), slow and stalled clients, limits |
| `tools/sim/out/test_app.exe` | among the App tests: the status key stays while angles, temperatures and missed reads change, and changes with a setting or the mode |
| `node tools/sim/web_smoke.mjs [url]` | with `web_sim.exe --calibrated` running: headless Chrome (software WebGL) loads the page and checks the connection, version 2 frames, the status only on change (none in 3 quiet seconds, a change within 0.4 s), the health message, the layout (menu, 3D view, bottom panel, inspector, status bar on one screen), the Joints panel (commanded, measured and model current, voltage, missed reads), the Live plot and Console tabs, the tool point readout and its DEG / RAD switch, every page, 5 frames a second on Plots, the 250 Hz capture, Health, the cards, the search box (Ctrl K, a setting, a page) and Esc (closes the box, then goes limp), the move target, the servo tables, that the real arm model is drawn, that the page's kinematics matches the firmware's TCP, no horizontal overflow at 390 px, pulling the arm, teleop with the Hold button, precision and a lock, no script errors; screenshots light and dark, desktop, laptop and phone in `tools/sim/out/shots` |
| `node tools/sim/web_flow.mjs [ws-url]` | with `web_sim.exe --fresh` running: the whole calibration wizard, then teleop through the page's WebSocket: save refused while teleoperating, hold, the clutch button, precision, locks, orientation lock and guide, the emulated table showing up as t_ext in the frame, realign, and a replay that PAR6 follows |

All four pass. What they measure:
- **Calibration:** through the web UI it recovers the simulated torque constants within 0.3 % and the spring within 0.2 %.
- **Teleoperation:** PAR6's t_ext on J2 reaches about 8 Nm against the emulated table, and a replay moves PAR6's J1
  to within 0.1° of its target.
- **Live data:** an idle page gets one status in 3 s; a setting change shows up about 60 ms later.

On the arm, with pages open: period 4.00 ms, jitter about 1 µs rms (the hardware timer tick), tick work about 0.9 ms
of 4 ms, no late ticks. A status print costs about 4.5 ms on the network core, outside the App lock, so the control
loop on the other core does not wait for it. The firmware's `replay_*` commands are for the Console only.
