"""Bake the RAFT's CAD meshes into a small binary for the web UI (offline dev tool, nothing here runs on the arm).

1. Loads the MJCF made by gen_model.py (tools/sim/model/RAFT.xml) with MuJoCo, so every mesh sits exactly
   where the simulator puts it.
2. Expresses each link's vertices in the DH frame that link rides on (frames 0..6 of kinematics.cpp, 7 = the
   trigger hinge), so the page only has to apply its own forward kinematics, the same as the firmware's.
3. Checks that mapping against MuJoCo at random poses (full-resolution vertices, error must be < 1e-5 m).
4. Decimates every part (quadric error, fast-simplification), quantizes vertices to 16 bit per axis and writes
   web/arm.bin, which tools/build_web.py embeds next to the page.

    python tools/build_arm_mesh.py [--tris N] [--agg A]
        N = total triangle budget (default 20000; the CAD export has 84232)
        A = decimation aggressiveness, 0 careful .. 10 fast and rough (default 2)

Needs: numpy, mujoco, fast-simplification (pip install fast-simplification).

web/arm.bin, little-endian:
    "RAFTMESH" u16 version=2, u16 parts
    per part: u8 frame, u8 material (0 base, 1 link, 2 trigger), u16 vertices, u16 triangles,
              f32 min[3], f32 step[3]  (position = min + q * step, q u16 per axis)
              u16 dq[vertices][3]      zig-zag deltas of q from the previous vertex, per axis, mod 65536
              u16 di[triangles * 3]    zig-zag deltas of the index from the previous index, mod 65536
    Vertices are numbered in order of first use, so both delta streams stay small and gzip well.
"""
import argparse
import math
import os
import re
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # firmware/
MJCF = os.path.join(ROOT, "tools", "sim", "model", "RAFT.xml")
MODEL_H = os.path.join(ROOT, "src", "main", "raft_model.h")
OUT = os.path.join(ROOT, "web", "arm.bin")
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"]
# share of the triangle budget per mesh (by visual importance, not by source size)
SHARE = {"base_link": 0.11, "link1": 0.15, "link2": 0.06, "link3": 0.20, "link4": 0.14, "link5": 0.13,
         "link6": 0.15, "gripper_trigger": 0.06}
MATERIAL = {"base_link": 0, "gripper_trigger": 2}


def model_from_header():
    """DH rows and the trigger hinge from the generated raft_model.h, so this stays in step with the firmware."""
    text = open(MODEL_H, encoding="utf-8").read()
    body = text[text.index("DH[NUM_ARM_JOINTS]"):]
    body = body[:body.index("};")]
    rows = [[float(x) for x in re.findall(r"(-?[\d.]+(?:e-?\d+)?)f", r)] for r in re.findall(r"\{([^{}]*)\}", body)]
    grip_r = [float(x) for x in re.findall(r"(-?[\d.]+(?:e-?\d+)?)f", re.search(r"GRIP_R\[9\] = \{([^}]*)\}", text).group(1))]
    grip_p = [float(x) for x in re.findall(r"(-?[\d.]+(?:e-?\d+)?)f", re.search(r"GRIP_P\[3\] = \{([^}]*)\}", text).group(1))]
    assert len(rows) == 6 and all(len(r) == 4 for r in rows) and len(grip_r) == 9 and len(grip_p) == 3
    return rows, np.array(grip_r).reshape(3, 3), np.array(grip_p)


def dh_a(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1.0]])


def fk(q, dh, grip_r, grip_p):
    """Frames 0..6 and the trigger hinge (7), like raft::forwardKinematics. q in rad, CAD convention."""
    T = [np.eye(4)]
    for i in range(6):
        d, a, alpha, off = dh[i]
        T.append(T[i] @ dh_a(q[i] + off, d, a, alpha))
    H = np.eye(4)
    H[:3, :3], H[:3, 3] = grip_r, grip_p
    c, s = math.cos(q[6]), math.sin(q[6])
    Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    T.append(T[6] @ H @ Rz)
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tris", type=int, default=20000)
    ap.add_argument("--agg", type=float, default=2.0)
    args = ap.parse_args()
    import mujoco
    import fast_simplification

    dh, grip_r, grip_p = model_from_header()
    m = mujoco.MjModel.from_xml_path(MJCF)
    d = mujoco.MjData(m)
    jid = [m.joint(n).id for n in JOINTS]
    qadr = [m.jnt_qposadr[j] for j in jid]

    def frame_of_body(b):
        while b > 0:
            for k, j in enumerate(jid):
                if m.jnt_bodyid[j] == b:
                    return k + 1
            b = m.body_parentid[b]
        return 0

    parts = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid)
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        parts.append(dict(name=name, geom=g, frame=frame_of_body(m.geom_bodyid[g]),
                          v=np.array(m.mesh_vert[va:va + vn], dtype=np.float64), f=np.array(m.mesh_face[fa:fa + fn], dtype=np.int64)))

    def world(p):
        R = d.geom_xmat[p["geom"]].reshape(3, 3)
        return p["v"] @ R.T + d.geom_xpos[p["geom"]]

    # bake at q = 0, then check against MuJoCo at random poses
    d.qpos[:] = 0
    mujoco.mj_forward(m, d)
    T0 = fk([0.0] * 7, dh, grip_r, grip_p)
    for p in parts:
        Tinv = np.linalg.inv(T0[p["frame"]])
        w = world(p)
        p["vk"] = w @ Tinv[:3, :3].T + Tinv[:3, 3]
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(25):
        q = [rng.uniform(-1.5, 1.5) for _ in range(6)] + [rng.uniform(-1.2, 1.2)]
        d.qpos[:] = 0
        for i in range(7):
            d.qpos[qadr[i]] = q[i]
        mujoco.mj_forward(m, d)
        T = fk(q, dh, grip_r, grip_p)
        for p in parts:
            Tk = T[p["frame"]]
            mine = p["vk"] @ Tk[:3, :3].T + Tk[:3, 3]
            worst = max(worst, float(np.abs(mine - world(p)).max()))
    print("mapping check against MuJoCo at 25 random poses: worst vertex error %.2e m" % worst)
    if worst > 1e-5:
        sys.exit("mesh-to-DH-frame mapping is wrong")

    # decimate, quantize, write
    def zigzag_deltas(a):
        """int array -> u16 zig-zag deltas from the previous element (mod 65536)."""
        d = np.diff(np.concatenate([[0], a.astype(np.int64)]))
        d = ((d + 32768) % 65536) - 32768                    # wrap into int16
        return np.where(d >= 0, 2 * d, -2 * d - 1).astype("<u2")

    blob = bytearray(b"RAFTMESH" + struct.pack("<HH", 2, len(parts)))
    total_in = total_out = 0
    for p in parts:
        n_in = len(p["f"])
        target = max(60, int(args.tris * SHARE.get(p["name"], 0.1)))
        if n_in > target:
            v, f = fast_simplification.simplify(p["vk"], p["f"].astype(np.int32), target_count=target, agg=args.agg)
        else:
            v, f = p["vk"], p["f"]
        v, f = np.asarray(v, dtype=np.float64), np.asarray(f, dtype=np.int64)
        # number the vertices in order of first use (drops unused ones)
        flat = f.reshape(-1)
        _, first = np.unique(flat, return_index=True)
        order = flat[np.sort(first)]
        remap = -np.ones(len(v), dtype=np.int64)
        remap[order] = np.arange(len(order))
        v, f = v[order], remap[f]
        assert len(v) < 65536 and len(f) < 65536
        lo, hi = v.min(axis=0), v.max(axis=0)
        step = np.maximum(hi - lo, 1e-9) / 65535.0
        qv = np.clip(np.round((v - lo) / step), 0, 65535).astype(np.int64)
        blob += struct.pack("<BBHH", p["frame"], MATERIAL.get(p["name"], 1), len(v), len(f))
        blob += struct.pack("<6f", *lo, *step)
        dq = np.stack([zigzag_deltas(qv[:, k]) for k in range(3)], axis=1)
        blob += dq.astype("<u2").tobytes() + zigzag_deltas(f.reshape(-1)).tobytes()
        total_in += n_in
        total_out += len(f)
        print("  %-16s frame %d  %6d -> %5d triangles, %5d vertices, %5.0f x %5.0f x %5.0f mm"
              % (p["name"], p["frame"], n_in, len(f), len(v), *(1000 * (hi - lo))))
    with open(OUT, "wb") as fh:
        fh.write(blob)
    import gzip
    print("total %d -> %d triangles; %s %d bytes (%d gzipped)" % (total_in, total_out, os.path.relpath(OUT, ROOT), len(blob), len(gzip.compress(bytes(blob), 9))))


if __name__ == "__main__":
    main()
