"""RAFT floating in MuJoCo: the motors cancel gravity, as the firmware does on the ESP32 (there in C++, at 250 Hz).

    pip install mujoco
    python examples/gravity_comp.py                  # move the arm: double-click a link, then Ctrl + right-drag
    python examples/gravity_comp.py --spring 718     # with the J2 counterbalance spring, rate in N/m
    python examples/gravity_comp.py --headless       # no window: 5 s from every keyframe, print drift and motor load

The motor command is the model's gravity torque minus what the spring already provides. The servos' gearbox
friction and inertia in the model are estimates; the real arm's values come from its calibration.
"""
import argparse
import time
from pathlib import Path

import mujoco
import numpy as np

SCENE = Path(__file__).resolve().parents[1] / "firmware" / "tools" / "sim" / "model" / "scene.xml"   # the simulators' model


def compensate(m, d):
    """Gravity (and velocity) torque minus the spring's torque, limited to the servos' stall torque."""
    tau = d.qfrc_bias[:m.nu] - d.qfrc_spring[:m.nu]
    d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spring", type=float, default=0.0, help="J2 spring rate in N/m (federnshop set: 565, 718, 824, 1027)")
    ap.add_argument("--headless", action="store_true", help="no window, print a summary")
    ap.add_argument("--key", default="handle_in_front", help="start keyframe")
    a = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(str(SCENE)); d = mujoco.MjData(m)
    m.tendon_stiffness[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "j2_spring")] = a.spring
    print(f"J2 spring: {a.spring:.0f} N/m" if a.spring else "no J2 spring")

    if a.headless:
        print(f"{'keyframe':16s} {'drift (deg)':>11s} {'J2 motor (Nm)':>14s} {'J3 motor (Nm)':>14s}")
        for k in range(m.nkey):
            mujoco.mj_resetDataKeyframe(m, d, k)
            q0 = d.qpos.copy()
            for _ in range(int(5.0 / m.opt.timestep)):
                mujoco.mj_forward(m, d); compensate(m, d); mujoco.mj_step(m, d)
            print(f"{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, k):16s} {np.rad2deg(np.abs(d.qpos - q0).max()):11.4f} "
                  f"{d.ctrl[1]:+14.3f} {d.ctrl[2]:+14.3f}")
        return

    from mujoco import viewer
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, a.key))
    with viewer.launch_passive(m, d) as v:
        if a.spring > 0:
            v.opt.geomgroup[3] = 0; v.opt.sitegroup[3] = 1      # show the spring cable, not the collision hulls
        last = time.time()
        while v.is_running():
            t0 = time.time()
            for _ in range(int(round(0.01 / m.opt.timestep))):  # 10 ms of simulation per frame
                mujoco.mj_forward(m, d); compensate(m, d); mujoco.mj_step(m, d)
            v.sync()
            if t0 - last > 1.0:
                last = t0
                print(f"\rJ2 motor {d.ctrl[1]:+.3f} Nm   J3 motor {d.ctrl[2]:+.3f} Nm   ", end="", flush=True)
            time.sleep(max(0.0, 0.01 - (time.time() - t0)))


if __name__ == "__main__":
    main()
