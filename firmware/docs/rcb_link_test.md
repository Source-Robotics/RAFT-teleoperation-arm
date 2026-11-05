# Testing the PAR6 link before the leader arm is wired

The RAFT's link to the PAR6 (`src/main/rti_link.cpp`) is plain C++ that runs unchanged on the ESP32 and on a PC.
`rti_probe.exe` runs it on Windows over real sockets, so the handshake, the state stream and a claimed motion can
be tested against the real RCB runtime without the leader arm. Build everything with `tools/sim/build.cmd`; the
programs land in `tools/sim/out/`.

Run the tests in this order: A needs nothing, B needs the Pi but moves only a simulated arm, C moves the real PAR6.

## A. Loopback on this PC

`fake_rcb_server.exe` plays the robot side (`tools/sim/sim_rcb.h`) on 127.0.0.1. Use two terminals in
`tools/sim/out`:

```
fake_rcb_server.exe
rti_probe.exe 127.0.0.1 --bind 127.0.0.1 --claim --amp 5 --seconds 8
```

Result, fake server on the same PC:

| Check | Result |
|---|---|
| Pairing (reload_recipes, set_recipe raft, rti_connect SOFT_RTI) | PAIRED after 12 ms |
| State stream | 250 packets/s |
| Claim, then 8 s of 0.2 Hz sine on joint 6 | ACTIVE, 400 of 400 commands accepted (50 Hz), clean release |
| Server started with `--forbidden` | every retry reports `FORBIDDEN: add this IP to RCB_ALLOWED_IPS` |
| Server killed while ACTIVE | `state stream lost` after 0.1 s, back to BACKOFF, no re-claim |
| No server | `tcp connect`, retried about every 2 s |

## B. The real runtime on the Pi, simulated drives

`tools/sim_run.py` runs the real RTI, executor and collision worker against simulated drives on `vcan0`. The
protocol is exactly what the robot speaks.

**Stopping `autorun.service` makes the real PAR6 go limp. Park it or support it first.**

On the Pi, first SSH session:

```
sudo systemctl stop autorun.service
sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up    # once per boot
cd ~/Desktop/RCB-Runtime
RCB_ALLOWED_IPS=<this-pc-ip> .venv/bin/python tools/sim_run.py
```

Copy the RAFT state recipe from this PC once:

```
scp docs/rcb/raft.xml <user>@<pi>:~/Desktop/RCB-Runtime/data_recipe/
```

The link sends `reload_recipes` before `set_recipe raft`, so no restart is needed after copying.

Second SSH session, to enable and home the simulated arm, which a claim needs:

```
cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/01_mode.py --go
```

On this PC:

```
rti_probe.exe <pi-ip>
rti_probe.exe <pi-ip> --claim
```

The first run only pairs and watches: expect PAIRED, about 250 state packets/s, mode `SOFT_RTI`, rti_state 1. The
second also claims and moves the simulated joint 6 by 2 deg. Expect ACTIVE, rti_state 2, and `seq` climbing with
`sent`.

Windows Defender Firewall asks the first time `rti_probe.exe` listens on UDP 30002. Allow it on private networks,
or no state packets arrive.

Afterwards stop `sim_run.py` with Ctrl-C, run `sudo systemctl start autorun.service`, and re-home the arm.

## C. The real PAR6

Only after B passes. The robot runs `autorun.service` as usual and must be ENABLED and homed.

The executor accepts only addresses on its access list. Never edit `ALLOWED_IPS` in `command_executor.py`: it is a
tracked file in a public repository. Add this PC and the ESP32 with a systemd drop-in instead:

```
sudo bash scripts/setup_system.sh --allowed-ips <this-pc-ip>,<esp32-ip>
sudo systemctl restart autorun.service
```

Run it with the arm disabled: its package check slows the runtime's ticks. `bash scripts/setup_system.sh --check`
shows what is set now, without sudo. The robot may already allow any address.

Then run the probe read-only first, and with a hand on the E-stop:

```
rti_probe.exe <pi-ip>
rti_probe.exe <pi-ip> --claim --amp 2
```

## D. The ESP32 against the fake server

Once the arm is calibrated (`docs/calibration.md`), the firmware's own link can be tested without the PAR6:

```
fake_rcb_server.exe --bind 0.0.0.0
```

Allow it through the firewall, then type `teleop_start <this-pc-ip>` on the ESP32's console. The fake follower
tracks the leader with a 30 ms lag.

## Probe options

| Option | Default | Meaning |
|---|---|---|
| `--claim` | off | claim control once PAIRED and move one joint in a slow sine. This moves the robot. |
| `--joint N` | 6 | joint for the sine, 1 to 6 |
| `--amp DEG` | 2 | amplitude, at most 10; 0.2 Hz, faded in and out over 2 s |
| `--seconds S` | 10 | time to stay PAIRED, or ACTIVE with `--claim` |
| `--rate HZ` | 50 | SOFT_RTI command rate: 50, 25 or 10 |
| `--recipe NAME` | raft | state recipe to select; `-` keeps the robot's current one |
| `--bind IP` | 0.0.0.0 | local address for UDP 30002; 127.0.0.1 for test A |

Exit code 0 means the link reached PAIRED, and ACTIVE with `--claim`, and ended cleanly.

## Link errors

| Error | Likely cause |
|---|---|
| `tcp connect` or `tcp connect timeout` | Wrong address, the runtime is not running, or a firewall blocks TCP 29000. |
| `FORBIDDEN: add this IP to RCB_ALLOWED_IPS` | This machine is not on the executor's access list. |
| `executor hung up (not on RCB_ALLOWED_IPS?)` | Usually the same refusal, when the refusal line was lost as the connection closed. |
| `executor reply timeout` | The executor did not answer a handshake step within 2 s. |
| `no state stream (UDP 30002)` | Paired, but no state packets: a firewall on this PC, or a wrong address in rti_connect. |
| `recipe lacks rti_state` | The robot streams a recipe without `rti_state`: copy `raft.xml` and keep `--recipe raft`. |
| `claim timeout` | The robot is not ENABLED and homed, or it refused the start pose. |
| `robot revoked control` | The robot stopped the session: a command watchdog, an error, or another client. |
| `state stream lost` | No state packet for 0.1 s: the runtime stopped, or the network dropped out. |
