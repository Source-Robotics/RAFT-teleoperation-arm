# RAFT firmware

Everything RAFT runs, on an Olimex ESP32-PoE (Arduino core 2.0.17 via PlatformIO, platform pinned in
`platformio.ini`): the servo bus, gravity compensation and the controller, calibration, settings in flash, the web UI,
the commands on TCP port 5000, the leader frame stream and the PAR6 link. No PC is needed at runtime.

New to the arm? Start with [`docs/bringup.md`](docs/bringup.md): wiring, servo IDs, flashing, calibration at the
mechanical stops and the first torque, one checked step at a time.

## Build and flash

[PlatformIO](https://platformio.org/) (the VS Code extension or `pip install platformio`), from this folder:

| | |
|---|---|
| `pio run -e main -t upload` | Build and flash over USB |
| `pio run -e main_ota -t upload` | Over the network, for the board on PoE (`upload_port` in `platformio.ini` is raft-leader.local; a named arm is raft-*name*.local, or give `--upload-port <the arm's address>`). The firmware switches torque off first, so park the arm. |
| `pio device monitor -b 115200` | Serial console (USB only) |
| `pio run -e set_id -t upload` | The servo ID tool: scan the bus, renumber servos, bus timing, a current test |

The ESP32-PoE is not isolated: never connect USB while it runs on PoE.

Network updates on Windows: the arm answers the upload tool by connecting back to the PC, so the firewall must let
PlatformIO's Python receive connections. If `main_ota` stops at "Sending invitation" / "No response from device",
allow `~/.platformio/penv/Scripts/python.exe`, or run the tool with PlatformIO's base Python:
`~/.platformio/python3/python.exe ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py -i <the arm's address>
-p 3232 -f .pio/build/main_ota/firmware.bin -r`.

The web page (`web/index.html`) is embedded into the firmware at build time (`tools/pio_build_web.py`). Open
`http://<the arm's address>/` once it is on the network.

## Layout

| | |
|---|---|
| `src/main/` | The firmware. `app.*` holds everything between the servo bus and the network (portable: the PC tests and simulators run it too); `main.cpp` is the ESP32 side (tasks, bus, network). `arm_design.h`: what is the same on every RAFT (joint directions, stop angles, wrist damping). `version.h`: the firmware version, shown at boot and in the web UI ([changelog](../CHANGELOG.md)). |
| `src/HLSCL.*`, `SCS*.*`, `SMS_STS.*`, `INST.h`, `SCServo.h` | Feetech's servo driver, unchanged |
| `src/tools/set_id/` | The servo ID tool |
| `web/` | The web UI (one page) and the arm mesh it draws |
| `docs/` | Bring-up, calibration, web UI, protocol, PAR6 link test, spring, the J2 spring mount |
| `tools/sim/` | PC tests and simulators (below) |
| `tools/*.py` | `build_web.py` (embeds the page), and the model generators `gen_model.py`, `build_arm_mesh.py`, `build_viewer.py`, which need the SolidWorks export of the arm (not in this repository) |

## Tests and simulators

`tools/sim/build.cmd` (Windows, Visual Studio Build Tools) or `tools/sim/build.sh` (Linux, g++) build the host tools
into `tools/sim/out/`. The unit tests need nothing else; the simulators need MuJoCo (`pip install mujoco`).

| | |
|---|---|
| `test_model`, `test_app`, `test_rti`, `test_web` | Model and controller, the application (commands, calibration, settings, faults, teleoperation), the PAR6 link, the web server |
| `calib_sim` | The whole bench calibration against a MuJoCo arm with wrong zeros and torque constants |
| `bilateral_sim` | Teleoperation against a simulated PAR6 |
| `web_sim` | The web UI on the PC against the MuJoCo arm: `http://localhost:8080/` (see `docs/web_ui.md`); `web_smoke.mjs` and `web_flow.mjs` drive it headless |

## Licence

GPL-3.0 ([`../LICENSE-GPL-3.0`](../LICENSE-GPL-3.0)), except Feetech's servo driver in `src/` (`HLSCL`, `SCS`, `SCSCL`,
`SMS_STS`, `SCSerial`, `INST.h`, `SCServo.h`), which is MIT-licensed, © 2024 ftservo, from
[FTServo_Arduino](https://github.com/ftservo/FTServo_Arduino), unchanged: [`src/LICENSE-FTServo.txt`](src/LICENSE-FTServo.txt).
