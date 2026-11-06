# Changelog

## 1.0.0 (2026-11-04)

The first release of RAFT with its software. Everything below has run on the first arm, except where it says
simulation.

- **Firmware for the Olimex ESP32-PoE.**
  - 250 Hz control loop on its own core, woken by a hardware timer, with a pipelined servo bus.
  - Gravity compensation, with friction compensation for holding and damping per joint.
  - Hold in the servos' own position mode, and moves to a pose. Rest parks the arm lying back, the way it rests limp,
    and turns the torque off.
  - Calibration at the mechanical stops; settings in flash.
  - A strict boot: torque stays off until all seven servos answer.
  - Short servo dropouts ridden through; 200 ms of silence goes limp.
  - A servo that answers again after its supply was off is read within half a turn of zero, as at power-up, so a
    joint turned by hand meanwhile is not a full turn off.
  - Updates over the network.
  - An arm name for several arms on one network: raft-*name*.local, and the name in the web UI's header and tab.
- **Teleoperation of PAR6** through its RCB runtime, tested in simulation only, not yet on a real PAR6.
  - Real RTI at 250 Hz or SOFT_RTI, with a filter on the targets sent.
  - Hold PAR6 (a clutch), realign, precision, joint locks and guides.
  - Haptic walls at the joint limits, and optional contact force feedback.
- **Web UI served by the arm.**
  - A 3D view of the real arm in the model's colours (printed parts, servos, horns, screws), with a joints table,
    a live plot and a console under it.
  - Pages: Drive, Teleop, Calibrate, Servos, Settings, a 250 Hz capture (Plots) and loop and bus health.
  - Search with Ctrl K.
  - The status is sent when it changes, plus a binary health message once a second.
- **Robot models in [robot_descriptions](https://github.com/Source-Robotics/robot_descriptions)**, like PAR6's and PAROL6's
  ([model guide](raft_description/README.md)).
  - A MuJoCo and a URDF / ROS 2 package with PAR6's joint names, built there from
    `firmware/tools/sim/model/raft_reference.json`, which the firmware's model generator writes.
  - Coloured printed parts, servos, horns and screws from the shared palette; joint ranges from the measured stops
    and a sweep of the CAD; `home` and `parked` (the rest pose) keyframes. CC BY 4.0.
- **PC tools:** a MuJoCo simulator of the arm, an emulated PAR6, the web UI against the simulator, unit tests and
  browser tests.
- **Hardware:** STEP and print files, the BOM, and an assembly manual with a picture for every step.
