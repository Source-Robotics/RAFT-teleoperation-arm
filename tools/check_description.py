"""Check that the RAFT descriptions agree with each other and with the published DH table.

    pip install mujoco numpy            (required)
    pip install yourdfpy xacro          (optional: an independent URDF reader, xacro expansion)
    python tools/check_description.py

Exits with 1 if any check fails. Runs in CI on every push (.github/workflows/check.yml).
"""
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from math import pi
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1] / "raft_description"
URDF = ROOT / "urdf" / "raft.urdf"
XACRO = ROOT / "urdf" / "raft.urdf.xacro"
SCENE = ROOT / "mjcf" / "scene.xml"
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"]

# Published model (raft_description/README.md): standard DH (CM6 table), metres; theta_DH = q + offset
DH_D = [0.05122, 0.0, 0.0, -0.09792, 0.0, -0.078277]
DH_A = [0.013, 0.100, -0.02416, 0.0, 0.0, 0.0]
DH_ALPHA = [-pi / 2, pi, pi / 2, -pi / 2, pi / 2, pi]
DH_OFFSET = [0.0, -pi / 2, pi, 0.0, 0.0, pi]
TRIGGER_XYZ, TRIGGER_RPY = [0.0, -0.01129, -0.042567], [pi / 2, 0.0, -pi / 2]   # hinge frame in DH frame 6
MASS_TOTAL, MASS_MOVING = 0.3713, 0.28354                                         # kg (moving = links 1-6 + trigger)

failures = []


def check(name, ok, detail=""):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
    if not ok:
        failures.append(name)


def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    R = {"x": [[1, 0, 0], [0, c, -s], [0, s, c]], "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]], "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
    return np.array(R)


def T(R=np.eye(3), p=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p
    return M


def dh_frames(q):
    """Base -> DH frame 6, and base -> trigger hinge frame (turned by q7)."""
    M = np.eye(4)
    for i in range(6):
        M = M @ T(rot("z", q[i] + DH_OFFSET[i])) @ T(p=(DH_A[i], 0, DH_D[i])) @ T(rot("x", DH_ALPHA[i]))
    r, p, y = TRIGGER_RPY
    hinge = M @ T(rot("z", y) @ rot("y", p) @ rot("x", r), TRIGGER_XYZ) @ T(rot("z", q[6]))
    return M, hinge


def random_q(rng, lo, hi, n):
    return [rng.uniform(lo, hi) for _ in range(n)]


def load(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    return m, mujoco.MjData(m)


def joint_index(m):
    ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS]
    return [m.jnt_qposadr[i] for i in ids], [m.jnt_dofadr[i] for i in ids], np.array([m.jnt_range[i] for i in ids])


def pose_of(m, d, kind, name):
    if kind == "site":
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name); return T(d.site_xmat[i].reshape(3, 3), d.site_xpos[i])
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name); return T(d.xmat[i].reshape(3, 3), d.xpos[i])


def set_q(m, d, adr, q):
    d.qpos[:] = 0; d.qvel[:] = 0
    for k, a in enumerate(adr):
        d.qpos[a] = q[k]
    mujoco.mj_forward(m, d)


def main():
    print(f"MuJoCo {mujoco.__version__}, description in {ROOT}")
    rng = np.random.default_rng(1)

    print("\nFiles")
    meshes = {Path(e.get("filename").replace("package://raft_description/", "")) for e in ET.parse(URDF).getroot().iter("mesh")}
    missing = [str(p) for p in meshes if not (ROOT / p).is_file()]
    check("every mesh the URDF names exists", not missing, ", ".join(missing))

    print("\nMJCF (mjcf/scene.xml) against the DH table")
    mj, dj = load(SCENE)
    adr, dof, rng_mj = joint_index(mj)
    worst_tcp = worst_trig = 0.0
    for q in random_q(rng, rng_mj[:, 0], rng_mj[:, 1], 500):
        set_q(mj, dj, adr, q)
        F6, H = dh_frames(q)
        worst_tcp = max(worst_tcp, np.abs(pose_of(mj, dj, "site", "tcp") - F6).max())
        worst_trig = max(worst_trig, np.abs(pose_of(mj, dj, "body", "gripper_trigger") - H).max())
    check("tcp site = DH frame 6 (500 poses)", worst_tcp < 1e-9, f"max diff {worst_tcp:.1e}")
    check("trigger body = hinge frame (500 poses)", worst_trig < 1e-9, f"max diff {worst_trig:.1e}")
    set_q(mj, dj, adr, np.zeros(7))
    tcp0 = dj.site_xpos[mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_SITE, "tcp")] * 1e3
    check("tcp at the standby pose is (189.197, 0, 175.380) mm", np.allclose(tcp0, [189.197, 0, 175.38], atol=1e-3), f"{np.round(tcp0, 3)}")
    total = mj.body_mass.sum()
    moving = sum(mj.body_mass[mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_BODY, b)] for b in
                 ["link1", "link2", "link3", "link4", "link5", "link6", "gripper_trigger"])
    check("mass: 371.30 g total, 283.54 g moving", abs(total - MASS_TOTAL) < 1e-5 and abs(moving - MASS_MOVING) < 1e-5,
          f"{total * 1e3:.2f} g, {moving * 1e3:.2f} g")

    print("\nURDF (urdf/raft.urdf, loaded by MuJoCo) against the MJCF")
    mu, du = load(URDF)
    adr_u, dof_u, rng_u = joint_index(mu)
    check("same joint limits", np.allclose(rng_u, rng_mj, atol=1e-5),
          " ".join(f"J{k + 1} {np.rad2deg(a):.0f}..{np.rad2deg(b):.0f}" for k, (a, b) in enumerate(rng_mj)))
    wf = wg = 0.0
    for q in random_q(rng, rng_mj[:, 0], rng_mj[:, 1], 500):
        set_q(mj, dj, adr, q); set_q(mu, du, adr_u, q)
        wf = max(wf, np.abs(pose_of(mu, du, "body", "tcp") - pose_of(mj, dj, "site", "tcp")).max(),
                 np.abs(pose_of(mu, du, "body", "gripper_trigger") - pose_of(mj, dj, "body", "gripper_trigger")).max())
        wg = max(wg, np.abs(du.qfrc_bias[dof_u] - dj.qfrc_bias[dof]).max())
    check("same tcp and trigger frames (500 poses)", wf < 1e-9, f"max diff {wf:.1e}")
    check("same gravity torques (500 poses)", wg < 1e-9, f"max diff {wg:.1e} Nm")

    print("\nScene behaviour (MJCF)")
    for k in range(mj.nkey):
        mujoco.mj_resetDataKeyframe(mj, dj, k); mujoco.mj_forward(mj, dj)
        name = mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_KEY, k)
        check(f"keyframe '{name}': no contacts", dj.ncon == 0, f"{dj.ncon} contacts" if dj.ncon else "")
    # gravity compensation through the actuators: the arm must stay where it is put
    drift = 0.0
    for k in range(mj.nkey):
        mujoco.mj_resetDataKeyframe(mj, dj, k)
        q0 = dj.qpos.copy()
        for _ in range(2000):                                   # 2 s
            mujoco.mj_forward(mj, dj)
            dj.ctrl[:] = np.clip(dj.qfrc_bias[dof], *mj.actuator_ctrlrange.T)
            mujoco.mj_step(mj, dj)
        drift = max(drift, np.abs(dj.qpos - q0).max())
    check("motors cancel gravity (2 s at every keyframe)", drift < np.deg2rad(0.1), f"max drift {np.rad2deg(drift):.4f} deg")
    print("  gravity load (Nm) at the keyframes:")
    for k in range(mj.nkey):
        mujoco.mj_resetDataKeyframe(mj, dj, k); mujoco.mj_forward(mj, dj)
        t = dj.qfrc_bias[dof]
        print(f"    {mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_KEY, k):16s} J2 {t[1]:+.3f}  J3 {t[2]:+.3f}  J4..J7 max {np.abs(t[3:]).max():.3f}")

    print("\nOptional: yourdfpy (independent URDF reader)")
    try:
        import yourdfpy
    except ImportError:
        print("  (yourdfpy not installed, skipped)")
    else:
        handler = lambda fn: str(ROOT / fn.replace("package://raft_description/", ""))
        robot = yourdfpy.URDF.load(str(URDF), filename_handler=handler, load_meshes=False, build_scene_graph=True)
        w = 0.0
        for q in random_q(rng, rng_mj[:, 0], rng_mj[:, 1], 200):
            robot.update_cfg(dict(zip(JOINTS, q)))
            w = max(w, np.abs(robot.get_transform("tcp", "base_link") - dh_frames(q)[0]).max())
        check("yourdfpy tcp = DH frame 6 (200 poses)", w < 1e-9, f"max diff {w:.1e}")

    print("\nOptional: xacro")
    try:
        import xacro
    except ImportError:
        print("  (xacro not installed, skipped)")
    else:
        def tree(el):
            """element structure with sorted attributes; text, comments and visual/collision names left out"""
            attrs = {k: v for k, v in el.attrib.items() if not (el.tag in ("visual", "collision") and k == "name")}
            return (el.tag, tuple(sorted(attrs.items())), tuple(tree(c) for c in el))
        def canon(root):
            return sorted(tree(el) for el in root if el.tag in ("link", "joint"))
        expanded = ET.fromstring(xacro.process_file(str(XACRO)).toxml())
        check("raft.urdf.xacro expands to raft.urdf", canon(expanded) == canon(ET.parse(URDF).getroot()))
        two = ('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="pair"><link name="world"/>'
               f'<xacro:include filename="{(ROOT / "urdf" / "raft_macro.xacro").as_posix()}"/>'
               '<xacro:raft prefix="a_" parent="world"/><xacro:raft prefix="b_" parent="world" xyz="0.4 0 0" rpy="0 0 3.14159265"/></robot>')
        with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as f:
            f.write(two)
        try:
            pair = ET.fromstring(xacro.process_file(f.name).toxml())
        finally:
            os.unlink(f.name)
        names = [l.get("name") for l in pair.iter("link")]
        mounts = [j for j in pair.iter("joint") if j.get("name", "").endswith("mount_joint")]
        check("macro: two prefixed arms on one parent", len(names) == 1 + 2 * 9 and len(set(names)) == len(names) and len(mounts) == 2,
              f"{len(names)} links, {len(mounts)} mount joints")

    print("\n" + ("All checks passed." if not failures else f"{len(failures)} check(s) FAILED: " + "; ".join(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
