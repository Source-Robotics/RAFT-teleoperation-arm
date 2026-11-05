"""RAFT model generator (offline dev tool, nothing from here runs on the arm).

Sources of truth
  * CM6 standard DH table with the RAFT lengths (PAROL6_PAR6_DH_TABLE.png + RAFT inertials.pdf p.9)
  * Mass properties from RAFT inertials.pdf p.2-8 (SolidWorks, corrected masses), given in the
    assembly's "default" coordinate system at the CAD zero pose
  * The SW2URDF export (tools/cad/Assem1.SLDASM.urdf), only for the mesh file names and the gripper hinge
    transform; its STL meshes are raft_description/meshes (lowercase names), or tools/cad/meshes after a new export

Outputs
  * a corrected URDF (joint axes, joint origins, limits, PDF inertials, meshes re-homed)
  * src/main/raft_model.h   : DH table + link inertials in DH link frames (SI, float)
  * src/main/golden_vectors.h: random q -> FK + gravity torque, for the firmware boot self-test
  * the model for the simulators: tools/sim/model (RAFT.urdf, RAFT.xml, scene.xml, follower_scene.xml)
  * the public description package raft_description/ (urdf, xacro macro, mjcf with collision geoms, meshes); its
    README, package.xml, launch and rviz files are hand-written. Check it with tools/check_description.py.
  * a sizing printout (which poses load J2/J3 how much)
"""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from math import pi
import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=150)

# --------------------------------------------------------------------------------------
# 1. Geometry: standard DH (CM6 table), RAFT lengths in metres
# --------------------------------------------------------------------------------------
A1, A2, A3, A4, A5, A6, A7 = 51.22e-3, 13e-3, 100e-3, 24.16e-3, 97.92e-3, 78.277e-3, 0.0
#        d      a     alpha   theta offset (q_DH = q_CAD + offset)
DH = [(A1,     A2,  -pi/2,   0.0),
      (0.0,    A3,   pi,    -pi/2),
      (0.0,   -A4,   pi/2,   pi),
      (-A5,    0.0, -pi/2,   0.0),
      (0.0,    0.0,  pi/2,   0.0),
      (-A6,   -A7,   pi,     pi)]
# gripper trigger hinge frame relative to DH frame 6 (from the SW export: Coordinate System8)
GRIP_XYZ = np.array([0.0, -0.01129, -0.042567])
GRIP_RPY = (pi/2, 0.0, -pi/2)
G = 9.81


def rotx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def roty(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def rotz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def rpy_to_R(r, p, y): return rotz(y) @ roty(p) @ rotx(r)
def R_to_rpy(R):
    R = np.where(np.abs(R) < 1e-12, 0.0, R)
    p = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    r = np.arctan2(R[2, 1], R[2, 2]); y = np.arctan2(R[1, 0], R[0, 0])
    return r, p, y
def T_of(R, p): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return T
def dh_A(theta, d, a, alpha):
    return T_of(rotz(theta), [0, 0, 0]) @ T_of(np.eye(3), [0, 0, d]) @ T_of(np.eye(3), [a, 0, 0]) @ T_of(rotx(alpha), [0, 0, 0])
T_6H = T_of(rpy_to_R(*GRIP_RPY), GRIP_XYZ)


def frames(q_cad):
    """Zero-based list: T[0]=base, T[1..6]=DH frames 1..6, T[7]=gripper hinge frame (rotated by q7)."""
    T = [np.eye(4)]
    for i in range(6):
        d, a, al, off = DH[i]
        T.append(T[-1] @ dh_A(q_cad[i] + off, d, a, al))
    T.append(T[6] @ T_6H @ T_of(rotz(q_cad[6]), [0, 0, 0]))
    return T


# --------------------------------------------------------------------------------------
# 2. Mass properties from RAFT inertials.pdf (SolidWorks "default" coordinate system)
#    units: g, mm, g*mm^2 ; L = inertia at the COM aligned with that coordinate system
# --------------------------------------------------------------------------------------
PDF = {  # name: (mass, COM xyz, [Lxx, Lyy, Lzz, Lxy, Lyz, Lxz])
    "link1":           (43.14, (14.16,  -2.50,   6.83), [8812.03,  8881.16,  4908.53,   -12.94,  -149.90,    -79.71]),
    "link2":           (18.20, (-8.89,  50.00,  13.00), [24045.05,  782.23, 23590.99,     0.00,     0.00,      0.00]),
    "link3":           (71.40, (11.76, 114.84,   5.56), [18140.60, 13758.01, 17383.44, -1537.69,   -74.69,   -297.59]),
    "link4":           (46.10, (-7.09, 124.18,  84.39), [38388.21, 46672.37, 10756.63,    15.77,   -41.16, -15386.09]),
    "link5":           (35.10, (13.78, 119.52, 110.95), [5272.48,  3420.08,  4478.32,  -136.94,    32.15,     41.61]),
    "link6":           (55.30, (-11.91, 122.54, 156.63), [18751.33, 90165.35, 76914.96, -2779.67,  1008.02, -30531.83]),
    "gripper_trigger": (14.30, (12.45, 111.38, 161.83), [4941.75,  4546.86,  4419.35, -1423.86,  1719.48,  -1376.42]),
}
TOTAL = (371.30, (5.87, 63.88, 51.30), [3543444.64, 1840072.90, 2075109.82, -154482.42, 1139615.57, -212092.70])
NAMES = list(PDF.keys())

# SolidWorks default axes -> DH base axes. The SW frame has Y up, Z along the forearm (+x DH), X lateral.
P_SW2DH = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)   # v_dh = P v_sw (+ translation)
assert abs(np.linalg.det(P_SW2DH) - 1) < 1e-12


def tensor(L):
    """SolidWorks prints PRODUCTS of inertia (Lxy = integral of x*y dm). The inertia tensor carries them
    with a minus sign. Verified: only this convention reproduces the principal moments and axes the PDF
    prints for every link (with the naive sign the gripper's eigenvalues come out 2762/3494/7651 instead
    of 1592/6082/6234)."""
    Lxx, Lyy, Lzz, Lxy, Lyz, Lxz = L
    return np.array([[Lxx, -Lxy, -Lxz], [-Lxy, Lyy, -Lyz], [-Lxz, -Lyz, Lzz]])
def six(I):
    return [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[1, 2], I[0, 2]]


# --------------------------------------------------------------------------------------
# 3. Locate the SolidWorks default frame relative to the DH base frame
#    Compare the PDF COMs with the old export's COMs (link frames = DH frames at zero pose).
#    The old export used uniform density, so per-link COMs differ a little; the translation
#    must come out consistent across links.
# --------------------------------------------------------------------------------------
import os
from pathlib import Path
_REPO = Path(__file__).resolve().parents[2]          # the repository root (this file: firmware/tools/)
_CAD = Path(__file__).resolve().parent / "cad"
_SIM_MODEL = Path(__file__).resolve().parent / "sim" / "model"
OLD_URDF = os.environ.get("RAFT_OLD_URDF", str(_CAD / "Assem1.SLDASM.urdf"))
old = {}
for lk in ET.parse(OLD_URDF).getroot().findall("link"):
    inr = lk.find("inertial")
    old[lk.get("name")] = (float(inr.find("mass").get("value")) * 1e3,
                           np.array([float(v) for v in inr.find("origin").get("xyz").split()]) * 1e3)
T0 = frames(np.zeros(7))
print("=== SolidWorks default frame vs DH base frame: implied translation per link (mm, DH axes) ===")
print("(old export COM mapped through the DH zero-pose frames, minus the PDF COM rotated into DH axes)")
implied = []
for i, n in enumerate(NAMES):
    Ti = T0[i + 1]
    c_old = Ti[:3, :3] @ old[n][1] + Ti[:3, 3] * 1e3
    c_pdf = P_SW2DH @ np.array(PDF[n][1])
    implied.append(c_old - c_pdf)
    print(f"  {n:16s} old-export mass {old[n][0]:6.2f} g  PDF {PDF[n][0]:6.2f} g   t = {c_old - c_pdf}")
implied = np.array(implied)
print("  median:", np.median(implied, axis=0), " spread (max-min):", implied.max(0) - implied.min(0))

# Decision: take the translation from link2 (upper arm + two couplers). Its products of inertia are
# exactly zero in the SolidWorks frame and its COM sits at exactly half the arm length, so it has
# symmetry planes through its COM: the density overrides cannot move its COM, and the implied
# translation from it is exact: (0, -14.26, 51.22) mm. The other links scatter around it because the
# old export used uniform density (servo-bearing links shift laterally).
T_SW = np.array([0.0, -14.26, A1 * 1e3])
assert np.allclose(implied[1], T_SW, atol=0.02), implied[1]
print("  USING translation t_sw->dh (mm):", T_SW)


def sw_to_dh_point(c_mm):
    return (P_SW2DH @ np.array(c_mm) + T_SW) * 1e-3          # metres, DH base frame


# --------------------------------------------------------------------------------------
# 4. Link inertials in the DH link frames (what standard-DH RNE and raft_model.h want)
# --------------------------------------------------------------------------------------
links = []   # per link: m [kg], r [m] in frame i, I [kg m^2] about COM in frame i axes
for i, n in enumerate(NAMES):
    m_g, c_sw, L = PDF[n]
    Ti = T0[i + 1]
    Ri, pi_ = Ti[:3, :3], Ti[:3, 3]
    c0 = sw_to_dh_point(c_sw)
    r = Ri.T @ (c0 - pi_)
    I0 = P_SW2DH @ tensor(L) @ P_SW2DH.T * 1e-9          # kg m^2, DH base axes
    Ii = Ri.T @ I0 @ Ri
    links.append(dict(name=n, m=m_g * 1e-3, r=r, I=Ii))
    ev = np.linalg.eigvalsh(Ii)
    assert ev.min() > 0 and ev.sum() > 2 * ev.max() - 1e-12, f"{n}: inertia not physical {ev * 1e9} g mm^2"

# base link by subtraction (static, only for the URDF)
m_tot = TOTAL[0]; c_tot = np.array(TOTAL[1]); L_tot = tensor(TOTAL[2])
m_b = m_tot - sum(PDF[n][0] for n in NAMES)
c_b = (m_tot * c_tot - sum(PDF[n][0] * np.array(PDF[n][1]) for n in NAMES)) / m_b
def shift(L, m, d):  # inertia at COM -> at a point displaced by d (parallel axis)
    d = np.asarray(d); return L + m * (d @ d * np.eye(3) - np.outer(d, d))
L_b = L_tot - sum(shift(tensor(PDF[n][2]), PDF[n][0], np.array(PDF[n][1]) - c_tot) for n in NAMES)
L_b = L_b - m_b * ((c_b - c_tot) @ (c_b - c_tot) * np.eye(3) - np.outer(c_b - c_tot, c_b - c_tot))
print(f"\nbase by subtraction: m = {m_b:.2f} g, COM (SW) = {c_b}, inertia eigenvalues (g mm^2) = {np.linalg.eigvalsh(L_b)}")
base = dict(name="base_link", m=m_b * 1e-3, r=sw_to_dh_point(c_b), I=P_SW2DH @ L_b @ P_SW2DH.T * 1e-9)

print("\n=== Link inertials in DH link frames ===")
for lk in links:
    print(f"  {lk['name']:16s} m={lk['m']*1e3:6.2f} g  r={lk['r']*1e3} mm  I diag={np.diag(lk['I'])*1e6} g m^2... (kg m^2 *1e6)")

# --------------------------------------------------------------------------------------
# 5. Reference gravity torque: potential energy gradient on the 7-link chain (exact, independent
#    of any RNE); frames() is the only geometry it uses.
# --------------------------------------------------------------------------------------
def com_world(q):
    T = frames(q)
    return [T[i + 1][:3, :3] @ links[i]["r"] + T[i + 1][:3, 3] for i in range(7)]
def U(q):
    return sum(links[i]["m"] * G * c[2] for i, c in enumerate(com_world(q)))
def tau_g(q, h=1e-6):
    t = np.zeros(7)
    for i in range(7):
        qp, qm = q.copy(), q.copy(); qp[i] += h; qm[i] -= h
        t[i] = (U(qp) - U(qm)) / (2 * h)
    return t


def tau_rne(q):
    """Static recursive Newton-Euler, the algorithm the firmware will run: backward recursion of
    gravity forces and moments through the chain, projected on each joint axis."""
    T = frames(q)
    f = np.zeros(3); n_ = np.zeros(3); tau = np.zeros(7)   # total distal gravity force, moment about the world origin
    for i in range(6, -1, -1):
        Ti = T[i + 1]; c = Ti[:3, :3] @ links[i]["r"] + Ti[:3, 3]
        fg = np.array([0, 0, -links[i]["m"] * G])
        f = f + fg
        n_ = n_ + np.cross(c, fg)
        # joint i rotates about z of its parent frame (DH frame i-1); the trigger about z of the hinge frame
        if i < 6:
            z, o = T[i][:3, 2], T[i][:3, 3]
        else:
            Th = T[6] @ T_6H; z, o = Th[:3, 2], Th[:3, 3]
        tau[i] = -(z @ (n_ - np.cross(o, f)))                # holding torque = -(axis . moment about the joint)
    return tau


print("\n=== Reference checks ===")
rng = np.random.default_rng(7)
worst = 0
for _ in range(200):
    q = rng.uniform(-pi, pi, 7)
    worst = max(worst, np.abs(tau_rne(q) - tau_g(q)).max())
print(f"  static RNE vs potential gradient, 200 random poses: max |diff| = {worst:.2e} Nm")
# rtb cross-check (links 1-6 only, trigger lumped into link 6 at q7 = 0)
try:
    import warnings; warnings.filterwarnings("ignore")
    from roboticstoolbox import DHRobot, RevoluteDH
    dhl = [RevoluteDH(d=d, a=a, alpha=al, offset=off) for (d, a, al, off) in DH]
    for i in range(6):
        dhl[i].m = links[i]["m"]; dhl[i].r = links[i]["r"].copy(); dhl[i].I = six(links[i]["I"])
    # lump the trigger (q7 = 0) into link 6
    T6h = T_6H; m6, m7 = links[5]["m"], links[6]["m"]
    r7_in6 = T6h[:3, :3] @ links[6]["r"] + T6h[:3, 3]
    I7_in6 = T6h[:3, :3] @ links[6]["I"] @ T6h[:3, :3].T
    rc = (m6 * links[5]["r"] + m7 * r7_in6) / (m6 + m7)
    Ic = shift(links[5]["I"], m6, links[5]["r"] - rc) + shift(I7_in6, m7, r7_in6 - rc)
    dhl[5].m = m6 + m7; dhl[5].r = rc; dhl[5].I = six(Ic)
    rob = DHRobot(dhl, name="RAFT", gravity=[0, 0, -G])
    w = 0
    for _ in range(200):
        q = rng.uniform(-pi, pi, 7); q[6] = 0
        w = max(w, np.abs(rob.gravload(q[:6]) - tau_g(q)[:6]).max())
    print(f"  rtb DHRobot.gravload vs potential gradient (q7=0), 200 poses: max |diff| = {w:.2e} Nm")
    print("  rtb FK frame 6 at CAD zero (mm):", np.round(rob.fkine(np.zeros(6)).t * 1e3, 3), " ours:", np.round(T0[6][:3, 3] * 1e3, 3))
except ImportError:
    print("  (roboticstoolbox not available, rtb cross-check skipped)")

# --------------------------------------------------------------------------------------
# 6. Sizing: which poses load J2/J3
# --------------------------------------------------------------------------------------
STALL, RATED, KT, R_W = 6.0 * 0.0980665, 1.5 * 0.0980665, 4.5 * 0.0980665, 2.8
def row(label, q):
    t = tau_g(q); mA = 1e3 * np.abs(t) / KT
    print(f"  {label:44s} J2 {t[1]:+.3f} Nm ({mA[1]:4.0f} mA, {np.abs(t[1])/RATED:3.1f}x rated)   J3 {t[2]:+.3f} Nm ({mA[2]:4.0f} mA)   J4..J7 max {np.abs(t[3:]).max():.3f} Nm")
deg = np.deg2rad
def arm_pose(tilt, drop, q5=0.0):
    """Pose from what you see: upper arm tilted `tilt` deg forward from vertical, forearm pointing `drop`
    deg below horizontal (in the arm's plane). Joint 3's axis is opposite to joint 2's (DH alpha2 = pi),
    so the forearm's world angle is q2 - q3, which gives q3 = tilt - drop. Checked on rendered meshes."""
    q = np.zeros(7); q[1] = deg(tilt); q[2] = deg(tilt - drop); q[4] = deg(q5); return q
print("\n=== Gravity torque in named poses (CAD convention, q = 0 is the standby pose) ===")
print("    tilt = upper arm forward from vertical, drop = forearm below horizontal (deg)")
row("standby: tilt 0, drop 0 (all q = 0)", np.zeros(7))
row("teleop, handle in front: tilt 20, drop 20", arm_pose(20, 20))
row("teleop, forearm level: tilt 20, drop 0", arm_pose(20, 0))
row("reaching forward: tilt 45, drop 0", arm_pose(45, 0))
row("reaching far: tilt 60, drop -10", arm_pose(60, -10))
row("FULLY STRETCHED horizontal: tilt 90, drop 0", arm_pose(90, 0))
row("upper arm forward, forearm folded back (q2 90, q3 -90)", arm_pose(90, 180))
row("forearm straight down: tilt 0, drop 90", arm_pose(0, 90))
row("forearm straight up: tilt 0, drop -90", arm_pose(0, -90))

tmax = np.zeros(7); argq = [np.zeros(7)] * 7
for _ in range(30000):
    q = rng.uniform(-pi, pi, 7); t = np.abs(tau_g(q))
    for i in range(7):
        if t[i] > tmax[i]: tmax[i], argq[i] = t[i], q
print("\n=== Worst case over 30000 random poses (J1 is vertical: always 0) ===")
for i in range(1, 7):
    print(f"  J{i+1}: {tmax[i]:.3f} Nm = {100*tmax[i]/STALL:5.1f}% stall, {tmax[i]/RATED:4.2f}x rated, {1e3*tmax[i]/KT:4.0f} mA, I^2R ~{R_W*(tmax[i]/KT)**2:.2f} W   at q(deg)={np.rad2deg(argq[i]).round(0)}")

print("\nTCP (DH frame 6) at the CAD zero pose (mm):", np.round(T0[6][:3, 3] * 1e3, 3))
print("arm COM (links 1-7 + base) at zero, DH base frame (mm):",
      np.round((sum(l['m'] * c for l, c in zip(links, com_world(np.zeros(7)))) + base['m'] * base['r']) / (sum(l['m'] for l in links) + base['m']) * 1e3, 2),
      "  (PDF total COM mapped: ", np.round(sw_to_dh_point(c_tot) * 1e3, 2), ")")

# --------------------------------------------------------------------------------------
# 7. Corrected URDF.  URDF link i frame := DH frame i-1 (the joint i axis is its z); everything the
#    SW export expressed in DH frame i (inertials, meshes) is moved by A_i(q=0).  A fixed 'tcp' link
#    carries DH frame 6 so simulators report the same TCP as the firmware.
# --------------------------------------------------------------------------------------
def fmt(v): return " ".join(f"{x:.12g}" for x in np.where(np.abs(v) < 1e-12, 0.0, v))
def write_urdf(path, mesh_prefix="../../../../raft_description/meshes/", ext=".stl", header=None, extra=(), named=False):
    L = []
    L.append('<?xml version="1.0" encoding="utf-8"?>')
    L += header or [
        '<!-- RAFT leader arm. GENERATED by firmware/tools/gen_model.py from the CM6 DH table,',
        '     the RAFT link lengths and the SolidWorks mass properties in RAFT inertials.pdf. Do not hand-edit.',
        '     Joint i rotates about the z axis of DH frame i-1; q = 0 is the CAD / PAR6 standby pose.',
        '     PAR6/RCB DH angles: theta = q + [0, -pi/2, pi, 0, 0, pi]. Joint limits are placeholders until measured. -->']
    L.append('<robot name="RAFT">')
    L += extra
    def link_xml(name, m, r, I, visuals):
        I = np.where(np.abs(I) < 1e-14, 0.0, I)      # rounding dust from the frame rotations
        L.append(f'  <link name="{name}">')
        L.append('    <inertial>')
        L.append(f'      <origin xyz="{fmt(r)}" rpy="0 0 0"/>')
        L.append(f'      <mass value="{m:.6g}"/>')
        L.append(f'      <inertia ixx="{I[0,0]:.6g}" ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" iyy="{I[1,1]:.6g}" iyz="{I[1,2]:.6g}" izz="{I[2,2]:.6g}"/>')
        L.append('    </inertial>')
        for mesh, Tm in visuals:
            xyz, rpy = Tm[:3, 3], R_to_rpy(Tm[:3, :3])
            for tag in ("visual", "collision"):
                L.append(f'    <{tag} name="{name}_{tag}">' if named else f'    <{tag}>')
                L.append(f'      <origin xyz="{fmt(xyz)}" rpy="{fmt(np.array(rpy))}"/>')
                L.append(f'      <geometry><mesh filename="{mesh_prefix}{mesh}{ext}"/></geometry>')
                if tag == "visual":
                    L.append('      <material name="grey"><color rgba="0.8 0.8 0.8 1"/></material>')
                L.append(f'    </{tag}>')
        L.append('  </link>')
    def joint_xml(name, jtype, parent, child, T, axis="0 0 1", limit=None):
        xyz, rpy = T[:3, 3], R_to_rpy(T[:3, :3])
        L.append(f'  <joint name="{name}" type="{jtype}">')
        L.append(f'    <origin xyz="{fmt(xyz)}" rpy="{fmt(np.array(rpy))}"/>')
        L.append(f'    <parent link="{parent}"/>')
        L.append(f'    <child link="{child}"/>')
        if jtype == "revolute":
            L.append(f'    <axis xyz="{axis}"/>')
            lo, hi, eff, vel = limit
            L.append(f'    <limit lower="{lo:.6g}" upper="{hi:.6g}" effort="{eff:.6g}" velocity="{vel:.6g}"/>')
        L.append('  </joint>')

    link_xml("base_link", base["m"], base["r"], base["I"], [("base_link", np.eye(4))])
    A0 = [dh_A(off, d, a, al) for (d, a, al, off) in DH]      # A_i at q = 0, i = 1..6
    LIMITS = [(lo, hi, 0.588, 6.0) for lo, hi in JOINT_LIMITS[:6]]
    for i in range(6):
        n = NAMES[i]; Ai = A0[i]
        r_in = Ai[:3, :3] @ links[i]["r"] + Ai[:3, 3]; I_in = Ai[:3, :3] @ links[i]["I"] @ Ai[:3, :3].T
        link_xml(n, links[i]["m"], r_in, I_in, [(n, Ai)])
        joint_xml(f"joint{i+1}", "revolute", "base_link" if i == 0 else NAMES[i - 1], n,
                  np.eye(4) if i == 0 else A0[i - 1], limit=LIMITS[i])
    # fixed TCP frame = DH frame 6
    L.append('  <link name="tcp"/>')
    joint_xml("tcp_joint", "fixed", "link6", "tcp", A0[5])
    # gripper trigger: its link frame is the hinge frame (SW Coordinate System8), inertials already there
    n = "gripper_trigger"
    link_xml(n, links[6]["m"], links[6]["r"], links[6]["I"], [(n, np.eye(4))])
    joint_xml("gripper_trigger_joint", "revolute", "link6", n, A0[5] @ T_6H, limit=(-pi/2, pi/2, 0.588, 6.0))
    L.append('</robot>')
    open(path, "w", encoding="utf-8").write("\n".join(L) + "\n")
    return path


def check_urdf(path):
    """Read the generated URDF back with plain URDF semantics (joint rotates about its axis at the joint
    origin, child frame = joint frame) and compare FK of 'tcp' and the potential-gradient gravity torque
    against the DH model. Independent of frames()."""
    root = ET.parse(path).getroot()
    linfo = {}
    for lk in root.findall("link"):
        inr = lk.find("inertial")
        if inr is None: continue
        linfo[lk.get("name")] = (float(inr.find("mass").get("value")),
                                 np.array([float(v) for v in inr.find("origin").get("xyz").split()]))
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin"); xyz = np.array([float(v) for v in o.get("xyz").split()]); rpy = [float(v) for v in o.get("rpy").split()]
        ax = j.find("axis"); ax = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None
        joints.append((j.get("name"), j.get("type"), j.find("parent").get("link"), j.find("child").get("link"), T_of(rpy_to_R(*rpy), xyz), ax))
    order = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"]
    jd = {j[0]: j for j in joints}
    def fk(q):
        Tw = {"base_link": np.eye(4)}
        for k, jn in enumerate(order):
            _, _, par, ch, To, ax = jd[jn]
            assert np.allclose(ax, [0, 0, 1])
            Tw[ch] = Tw[par] @ To @ T_of(rotz(q[k]), [0, 0, 0])
        _, _, par, ch, To, _ = jd["tcp_joint"]; Tw["tcp"] = Tw[par] @ To
        return Tw
    def Uu(q):
        Tw = fk(q)
        return sum(m * G * (Tw[n][:3, :3] @ r + Tw[n][:3, 3])[2] for n, (m, r) in linfo.items() if n != "base_link")
    def tg(q, h=1e-6):
        t = np.zeros(7)
        for i in range(7):
            qp, qm = q.copy(), q.copy(); qp[i] += h; qm[i] -= h; t[i] = (Uu(qp) - Uu(qm)) / (2 * h)
        return t
    wf = wt = 0
    for _ in range(200):
        q = rng.uniform(-pi, pi, 7)
        Tw = fk(q); T = frames(q)
        wf = max(wf, np.abs(Tw["tcp"] - T[6]).max(), np.abs(Tw["gripper_trigger"] - T[7]).max())
        wt = max(wt, np.abs(tg(q) - tau_g(q)).max())
    print(f"  generated URDF read back with URDF semantics: max FK diff (tcp, trigger) {wf:.2e}, max gravity torque diff {wt:.2e} Nm")
    # the old export's visual meshes live in DH frames; the new file moves them by A_i(0). Check one: the
    # mesh origin of link i in world must equal DH frame i at q = 0.
    Tw = fk(np.zeros(7))
    for i, n in enumerate(NAMES[:6]):
        vis = [lk for lk in root.findall("link") if lk.get("name") == n][0].find("visual").find("origin")
        Tm = T_of(rpy_to_R(*[float(v) for v in vis.get("rpy").split()]), [float(v) for v in vis.get("xyz").split()])
        assert np.allclose(Tw[n] @ Tm, T0[i + 1], atol=1e-7), n     # 12 significant digits in the file
    print("  mesh frames at q = 0 coincide with the SolidWorks link coordinate systems (DH frames 1..6): OK")


# --------------------------------------------------------------------------------------
# 8. MJCF (MuJoCo). Same body frames as the URDF (body i frame = DH frame i-1, joint axis = z),
#    plus torque actuators, servo placeholders and keyframes. scene.xml adds floor and lights.
# --------------------------------------------------------------------------------------
# Servo-side placeholders, NOT measured; identify on the bench (plan P3) and update here:
#   armature     reflected motor inertia at the output: ~0.7 g cm^2 rotor x 205^2 gear -> 0.003 kg m^2
#   frictionloss gearbox Coulomb friction at the output, guess 0.02 Nm
#   damping      viscous friction at the output, guess 0.01 Nm s/rad
SERVO_ARMATURE, SERVO_FRICTION, SERVO_DAMPING = 0.003, 0.02, 0.01
# Optional J2 counterbalance, a zero-free-length cable spring (plan 4.4c): anchor SPRING_A above the J2
# axis on the shoulder bracket, pin SPRING_B up the upper arm. The MJCF carries it as a tendon with
# stiffness 0 (no effect); the simulator sets the stiffness when it tests a spring. Placeholder geometry.
SPRING_A, SPRING_B = 0.013, 0.020          # anchor height above J2, eyelet distance up the arm (plan 4.4, layout v2)
SPRING_Z = -0.033                           # cable plane along the J2 axis (DH frame 1 z), 4.8 mm outside the bar's outer face
# Joint ranges: J2 and J3 estimated from part-to-part collisions of the CAD meshes (convex hulls of the
# non-watertight parts, so slightly conservative; J3 at q2 = 0); the others are placeholders until measured (P1).
JOINT_LIMITS = [(-pi, pi), (np.deg2rad(-105), np.deg2rad(130)), (np.deg2rad(-75), np.deg2rad(150)),
                (-pi, pi), (-pi, pi), (-pi, pi), (-pi / 2, pi / 2)]
STALL_NM = 6.0 * 0.0980665
LINK_RGBA = {"base_link": "0.35 0.40 0.45 1", "link1": "0.48 0.53 0.58 1", "link2": "0.65 0.69 0.73 1",
             "link3": "0.38 0.50 0.60 1", "link4": "0.60 0.65 0.69 1", "link5": "0.44 0.52 0.60 1",
             "link6": "0.69 0.73 0.76 1", "gripper_trigger": "0.78 0.31 0.10 1"}
# Public MJCF only. Keyframes must clear the table (the base plate's underside is z = 0): the firmware's rest pose
# (forearm 70 deg down) puts the handle 19 mm below it, so the public model folds to 50 deg (handle 18 mm above).
RELEASE_KEYS = [("standby", np.zeros(7)), ("handle_in_front", arm_pose(20, 20)), ("reach_forward", arm_pose(45, 0)),
                ("stretched", arm_pose(90, 0)), ("folded", arm_pose(0, 50))]
# convex-hull pairs that touch in normal poses (3000 random poses: base/link1 always, wrist pairs 17-33 %)
RELEASE_EXCLUDES = [("base_link", "link1"), ("link4", "link6"), ("link4", "gripper_trigger"), ("link5", "gripper_trigger")]


def R_to_quat(R):
    """Rotation matrix -> unit quaternion (w, x, y, z), w >= 0 (MuJoCo order)."""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]); q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]); q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]); q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q) / np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def write_mjcf(robot_path, scene_path, release=False, meshdir="../meshes"):
    """release=True writes the public raft_description variant: lowercase mesh files, separate visual and
    collision geoms, no tcp_probe, no follower scene, keyframes that clear the table."""
    A0 = [dh_A(off, d, a, al) for (d, a, al, off) in DH]
    def at(T): return f'pos="{fmt(T[:3, 3])}" quat="{fmt(R_to_quat(T[:3, :3]))}"'
    def inertial(m, r, I):
        I = np.where(np.abs(I) < 1e-14, 0.0, I)
        return (f'<inertial pos="{fmt(r)}" mass="{m:.6g}" fullinertia="{I[0,0]:.6g} {I[1,1]:.6g} {I[2,2]:.6g} '
                f'{I[0,1]:.6g} {I[0,2]:.6g} {I[1,2]:.6g}"/>')
    def geoms(ind, n, pose=""):
        if not release:
            return [f'{ind}<geom mesh="{n}" material="{n}"{pose}/>']
        return [f'{ind}<geom class="visual" mesh="{n}" material="{n}"{pose}/>', f'{ind}<geom class="collision" mesh="{n}"{pose}/>']
    if release:
        head = ['  <!-- RAFT leader arm (raft_description). GENERATED by tools/gen_model.py from the CM6 DH table, the RAFT',
                '       link lengths and the SolidWorks mass properties. Do not hand-edit; see ../README.md.',
                '       Body i frame = DH frame i-1, joint i turns about its z; qpos = 0 is the standby pose (= PAR6 standby).',
                '       PAR6/RCB DH angles: theta = q + [0, -pi/2, pi, 0, 0, pi], not wrapped. Actuators take output torque in Nm.',
                f'       Servo armature {SERVO_ARMATURE}, frictionloss {SERVO_FRICTION}, damping {SERVO_DAMPING} are ESTIMATES, not measured.',
                '       Ranges: J2 and J3 from CAD collision checks; J1 and J4-J6 are +-180 deg placeholders until measured.',
                '       Collision geoms are the meshes\' convex hulls (class "collision", group 3). -->']
        geom_default = ['      <default class="visual">',
                        '        <geom type="mesh" contype="0" conaffinity="0" group="2"/>',
                        '      </default>',
                        '      <default class="collision">',
                        '        <geom type="mesh" group="3"/>',
                        '      </default>']
    else:
        head = ['  <!-- RAFT leader arm. GENERATED by firmware/tools/gen_model.py from the CM6 DH table, the RAFT',
                '       link lengths and the SolidWorks mass properties in RAFT inertials.pdf. Do not hand-edit.',
                '       Body i frame = DH frame i-1, joint i turns about its z; qpos = 0 is the CAD / PAR6 standby pose.',
                '       PAR6/RCB DH angles: theta = q + [0, -pi/2, pi, 0, 0, pi]. Actuators take output torque in Nm.',
                f'       Servo armature {SERVO_ARMATURE}, frictionloss {SERVO_FRICTION}, damping {SERVO_DAMPING} are ESTIMATES, not measured.',
                '       Joint ranges are placeholders until measured. -->']
        geom_default = ['      <geom type="mesh" contype="0" conaffinity="0" group="2"/>']
    X = ['<mujoco model="RAFT">'] + head + [
         f'  <compiler angle="radian" meshdir="{meshdir}" autolimits="true" inertiafromgeom="false"/>',
         '  <option timestep="0.001" integrator="implicitfast"/>',
         '  <default>',
         '    <default class="raft">',
         f'      <joint axis="0 0 1" armature="{SERVO_ARMATURE}" frictionloss="{SERVO_FRICTION}" damping="{SERVO_DAMPING}"/>',
         f'      <motor ctrlrange="{-STALL_NM:.4f} {STALL_NM:.4f}"/>'] + (geom_default if not release else []) + [
         '      <site group="3" size="0.004" rgba="0.9 0.2 0.1 1"/>'] + (geom_default if release else []) + [
         '    </default>',
         '  </default>',
         '  <asset>']
    for n in ["base_link"] + NAMES:
        X.append(f'    <material name="{n}" rgba="{LINK_RGBA[n]}" specular="0.2" shininess="0.3"/>')
    for n in ["base_link"] + NAMES:
        X.append(f'    <mesh name="{n}" file="{n}.stl"/>')
    X += ['  </asset>', '  <worldbody>',
          '    <body name="base_link" childclass="raft">',
          f'      {inertial(base["m"], base["r"], base["I"])}']
    X += geoms("      ", "base_link")
    ind = "      "
    for i in range(6):
        n = NAMES[i]; Ai = A0[i]
        r_in = Ai[:3, :3] @ links[i]["r"] + Ai[:3, 3]; I_in = Ai[:3, :3] @ links[i]["I"] @ Ai[:3, :3].T
        X.append(f'{ind}<body name="{n}" {at(np.eye(4) if i == 0 else A0[i - 1])}>')
        ind += "  "
        X.append(f'{ind}{inertial(links[i]["m"], r_in, I_in)}')
        X.append(f'{ind}<joint name="joint{i+1}" range="{JOINT_LIMITS[i][0]:.6f} {JOINT_LIMITS[i][1]:.6f}"/>')
        X += geoms(ind, n, " " + at(Ai))
        if i == 0:      # spring anchor: SPRING_A straight up from the J2 axis (up = -y of DH frame 1)
            X.append(f'{ind}<site name="j2_spring_anchor" pos="{fmt((A0[0] @ [0, -SPRING_A, SPRING_Z, 1])[:3])}"/>')
        if i == 1:      # spring pin: SPRING_B from the J2 axis toward the elbow (J2 axis at x2 = -a2)
            X.append(f'{ind}<site name="j2_spring_pin" pos="{fmt((A0[1] @ [-A3 + SPRING_B, 0, -SPRING_Z, 1])[:3])}"/>')   # z2 = -z1
    X.append(f'{ind}<site name="tcp" {at(A0[5])}/>')
    # contact probe at the TCP: collides only with geoms that set conaffinity bit 2 (the follower
    # scene's obstacle), so it changes nothing in the leader scene
    if not release:
        X.append(f'{ind}<geom name="tcp_probe" type="sphere" size="0.01" pos="{fmt(A0[5][:3, 3])}" contype="2" conaffinity="0" group="4" rgba="1 0.5 0 0.3"/>')
    X.append(f'{ind}<body name="gripper_trigger" {at(A0[5] @ T_6H)}>')
    X.append(f'{ind}  {inertial(links[6]["m"], links[6]["r"], links[6]["I"])}')
    X.append(f'{ind}  <joint name="gripper_trigger_joint" range="{-pi/2:.6f} {pi/2:.6f}"/>')
    X += geoms(ind + "  ", "gripper_trigger")
    X.append(f'{ind}</body>')
    for _ in range(6):
        ind = ind[:-2]; X.append(f'{ind}</body>')
    X += ['    </body>', '  </worldbody>']
    if release:
        # hulls that overlap where the real parts do not: the base and the shoulder share the J1 bearing, and
        # the wrist pairs touch at J5 angles the real wrist reaches (its range is not measured yet)
        X += ['  <contact>'] + [f'    <exclude body1="{a}" body2="{b}"/>' for a, b in RELEASE_EXCLUDES] + ['  </contact>']
    X += ['  <tendon>',
          '    <!-- optional J2 counterbalance (zero-length spring), stiffness 0 = not fitted -->' if not release else
          '    <!-- optional J2 counterbalance (zero-length cable spring, docs: spring_mount.md); stiffness = spring rate in N/m, 0 = not fitted -->',
          '    <spatial name="j2_spring" stiffness="0" springlength="0 0" width="0.0012" rgba="0.95 0.75 0.1 1" group="3">',
          '      <site site="j2_spring_anchor"/>',
          '      <site site="j2_spring_pin"/>',
          '    </spatial>',
          '  </tendon>',
          '  <actuator>']
    for k, jn in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"]):
        X.append(f'    <motor name="m{k+1}" joint="{jn}" class="raft"/>')
    X += ['  </actuator>', '  <sensor>',
          '    <framepos name="tcp_pos" objtype="site" objname="tcp"/>',
          '    <framequat name="tcp_quat" objtype="site" objname="tcp"/>',
          '  </sensor>', '  <keyframe>']
    KEYS = [("standby", np.zeros(7)), ("handle_in_front", arm_pose(20, 20)), ("reach_forward", arm_pose(45, 0)),
            ("stretched", arm_pose(90, 0)), ("rest", arm_pose(0, 70))]          # forearm 70 deg down: lowest reachable fold
    for name, q in (RELEASE_KEYS if release else KEYS):
        X.append(f'    <key name="{name}" qpos="{fmt(q)}"/>')
    X += ['  </keyframe>', '</mujoco>']
    open(robot_path, "w", encoding="utf-8").write("\n".join(X) + "\n")

    S = ['<mujoco model="RAFT scene">',
         '  <!-- GENERATED by tools/gen_model.py. Open with:  python -m mujoco.viewer --mjcf=<this file> -->',
         f'  <include file="{os.path.basename(robot_path)}"/>',
         '  <statistic center="0.08 0 0.13" extent="0.32"/>',
         '  <visual>',
         '    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0 0 0"/>',
         '    <global azimuth="130" elevation="-22" offwidth="1280" offheight="960"/>',
         '  </visual>',
         '  <asset>',
         '    <texture type="skybox" builtin="gradient" rgb1="0.32 0.38 0.44" rgb2="0.08 0.09 0.11" width="512" height="3072"/>',
         '    <texture type="2d" name="floor" builtin="checker" mark="edge" rgb1="0.24 0.27 0.30" rgb2="0.20 0.23 0.26" markrgb="0.45 0.48 0.52" width="300" height="300"/>',
         '    <material name="floor" texture="floor" texuniform="true" texrepeat="20 20" reflectance="0"/>',
         '  </asset>',
         '  <worldbody>',
         '    <light pos="0.2 -0.3 1.5" dir="-0.12 0.2 -1" directional="true"/>',
         '    <geom name="floor" size="0.6 0.6 0.01" type="plane" material="floor"/>',
         '  </worldbody>',
         '</mujoco>']
    open(scene_path, "w", encoding="utf-8").write("\n".join(S) + "\n")
    if release:
        return

    # follower scene for the bilateral simulator: same arm plus a block in front of the standby TCP
    # (top 40 mm below it), which only the TCP probe can touch
    F = [l for l in S]
    F[0] = '<mujoco model="RAFT follower scene">'
    F[1] = '  <!-- GENERATED by tools/gen_model.py. Follower arm for tools/sim/bilateral_sim.cpp: the obstacle is touched only by tcp_probe. -->'
    k = F.index('    <geom name="floor" size="0.6 0.6 0.01" type="plane" material="floor"/>')
    F.insert(k + 1, '    <body name="obstacle" pos="0.19 0 0.1104">')
    F.insert(k + 2, '      <geom name="obstacle" type="box" size="0.04 0.06 0.025" contype="0" conaffinity="2" solref="0.004 1" rgba="0.55 0.3 0.25 1"/>')
    F.insert(k + 3, '    </body>')
    open(os.path.join(os.path.dirname(scene_path), "follower_scene.xml"), "w", encoding="utf-8").write("\n".join(F) + "\n")


def check_mjcf(scene_path):
    """Load the MJCF in MuJoCo and compare its static gravity force (qfrc_bias at qvel = 0), the TCP site and
    the trigger body against the DH model."""
    try:
        import mujoco
    except ImportError:
        print("  (mujoco not installed, MJCF check skipped)"); return
    m = mujoco.MjModel.from_xml_path(scene_path); d = mujoco.MjData(m)
    jn = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"]
    adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in jn]
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in jn]
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp"); bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "gripper_trigger")
    wt = wf = 0.0
    for _ in range(200):
        q = rng.uniform(-pi, pi, 7); q[6] = rng.uniform(-pi / 2, pi / 2)
        d.qpos[:] = 0; d.qvel[:] = 0
        for k in range(7): d.qpos[adr[k]] = q[k]
        mujoco.mj_forward(m, d)
        wt = max(wt, np.abs(np.array([d.qfrc_bias[dof[k]] for k in range(7)]) - tau_g(q)).max())
        T = frames(q)
        wf = max(wf, np.abs(d.site_xpos[sid] - T[6][:3, 3]).max(), np.abs(d.site_xmat[sid].reshape(3, 3) - T[6][:3, :3]).max(),
                 np.abs(d.xpos[bid] - T[7][:3, 3]).max(), np.abs(d.xmat[bid].reshape(3, 3) - T[7][:3, :3]).max())
    moving = sum(m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)] for n in NAMES)
    print(f"  MJCF in MuJoCo {mujoco.__version__}: nq={m.nq}, nu={m.nu}, moving mass {moving*1e3:.2f} g, total {m.body_mass.sum()*1e3:.2f} g")
    print(f"  MJCF vs DH model, 200 random poses: max gravity torque diff {wt:.2e} Nm, max TCP/trigger frame diff {wf:.2e}")
    # counterbalance tendon: with stiffness k it must give tau_J2 = -k a b sin(q2) and nothing on the other joints
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "j2_spring")
    k = 300.0; m.tendon_stiffness[tid] = k; ws = 0.0; wo = 0.0
    for _ in range(200):
        q = rng.uniform(-pi, pi, 7)
        d.qpos[:] = 0; d.qvel[:] = 0
        for kk in range(7): d.qpos[adr[kk]] = q[kk]
        mujoco.mj_forward(m, d)
        ws = max(ws, abs(d.qfrc_passive[dof[1]] - (-k * SPRING_A * SPRING_B * np.sin(q[1]))))
        wo = max(wo, max(abs(d.qfrc_passive[dof[kk]]) for kk in range(7) if kk != 1))
    m.tendon_stiffness[tid] = 0.0
    print(f"  J2 spring tendon vs -k a b sin(q2): max diff {ws:.2e} Nm, max torque on other joints {wo:.2e} Nm")


# --------------------------------------------------------------------------------------
# 9. C++ headers
# --------------------------------------------------------------------------------------
def write_headers(model_h, golden_h, n_vec=40):
    from spatialmath.base import tr2rpy        # the reference the RCB runtime's write_rpy_xyz mirrors
    def f(x):
        s = f"{float(x):.9g}"
        if "." not in s and "e" not in s:      # "0" / "1" are not valid float literals with an f suffix
            s += ".0"
        return s + "f"
    H = ["// GENERATED by tools/gen_model.py - do not edit. RAFT kinematic + inertial model.",
         "// Standard DH (CM6 table), SI units. q_CAD = 0 is the CAD / PAR6 standby pose; theta_DH = q_CAD + offset.",
         "// Link inertials are in DH link frame i (link i's frame), inertia about the COM.",
         "#pragma once", "", "namespace raft_model {", "", "constexpr int NUM_ARM_JOINTS = 6;", "constexpr int NUM_LINKS = 7;",
         f"constexpr float GRAVITY = {f(G)};", "",
         "struct DhLink { float d, a, alpha, offset; };",
         "struct LinkInertial { float m; float r[3]; float I[6]; };  // I = {Ixx, Iyy, Izz, Ixy, Iyz, Ixz}", "",
         "constexpr DhLink DH[NUM_ARM_JOINTS] = {"]
    for (d, a, al, off) in DH:
        H.append(f"    {{{f(d)}, {f(a)}, {f(al)}, {f(off)}}},")
    H += ["};", "",
          "// gripper trigger hinge frame relative to DH frame 6 (rotation matrix row-major, translation), joint 7 rotates about its z",
          "constexpr float GRIP_R[9] = {" + ", ".join(f(x) for x in T_6H[:3, :3].reshape(-1)) + "};",
          "constexpr float GRIP_P[3] = {" + ", ".join(f(x) for x in T_6H[:3, 3]) + "};", "",
          "constexpr LinkInertial LINKS[NUM_LINKS] = {"]
    for lk in links:
        H.append(f"    {{{f(lk['m'])}, {{{', '.join(f(x) for x in lk['r'])}}}, {{{', '.join(f(x) for x in six(lk['I']))}}}}},  // {lk['name']}")
    H += ["};", "", "}  // namespace raft_model", ""]
    open(model_h, "w", encoding="utf-8").write("\n".join(H))

    rng2 = np.random.default_rng(2026)
    Gh = ["// GENERATED by tools/gen_model.py - do not edit. Boot self-test vectors for kinematics/dynamics.",
          "// For each case: q_CAD[7] (rad), T06 (row-major 3x4: R | p) in metres, tau_g[7] (Nm, static gravity torque).",
          "#pragma once", "", "namespace raft_model {", "",
          f"constexpr int NUM_GOLDEN = {n_vec + 2};",
          "// rpy: TCP roll/pitch/yaw (rad), SpatialMath tr2rpy(order='xyz') = RCB write_rpy_xyz; compare only when |R02| < 0.999",
          "struct GoldenCase { float q[7]; float T06[12]; float tau_g[7]; float rpy[3]; };", "",
          "constexpr GoldenCase GOLDEN[NUM_GOLDEN] = {"]
    # case 0: zero pose; case 1: arm fully stretched forward, horizontal (q2 = q3 = +90 deg); then random
    cases = [np.zeros(7), np.array([0, pi/2, pi/2, 0, 0, 0, 0.0])] + [rng2.uniform(-pi, pi, 7) for _ in range(n_vec)]
    for q in cases:
        T = frames(q)[6]; t = tau_g(q); rpy = tr2rpy(T[:3, :3], order="xyz")
        Gh.append("    {{" + ", ".join(f(x) for x in q) + "}, {" + ", ".join(f(x) for x in T[:3, :4].reshape(-1)) + "}, {"
                  + ", ".join(f(x) for x in t) + "}, {" + ", ".join(f(x) for x in rpy) + "}},")
    Gh += ["};", "", "}  // namespace raft_model", ""]
    open(golden_h, "w", encoding="utf-8").write("\n".join(Gh))


# --------------------------------------------------------------------------------------
# 10. Public description package (raft_description/): the same model with lowercase mesh files and
#     package:// paths, a xacro macro for scenes with several robots, and the MJCF with collision geoms.
#     The package's hand-written files (README, package.xml, launch, rviz, raft.urdf.xacro) are not touched.
# --------------------------------------------------------------------------------------
def write_macro(urdf_path, macro_path):
    """raft_macro.xacro: the links and joints of raft.urdf inside a macro; every link, joint and material name
    gets ${prefix}, and a non-empty `parent` adds a fixed joint parent -> base_link."""
    lines = Path(urdf_path).read_text(encoding="utf-8").splitlines()
    body = lines[lines.index('<robot name="RAFT">') + 1:lines.index('</robot>')]
    body = [l for l in body if "<mujoco>" not in l and "lets MuJoCo" not in l]
    body = [re.sub(r'(<(?:link|joint|material) name="|<(?:parent|child) link=")', r'\1${prefix}', l) for l in body]
    X = ['<?xml version="1.0" encoding="utf-8"?>',
         '<!-- RAFT leader arm as a xacro macro (raft_description). GENERATED by tools/gen_model.py, do not hand-edit.',
         '     Same links, joints and numbers as raft.urdf, with every link, joint and material name prefixed.',
         '     Use:  <xacro:include filename="$(find raft_description)/urdf/raft_macro.xacro"/>',
         '           <xacro:raft prefix="leader_" parent="world" xyz="0 0 0" rpy="0 0 0"/>',
         '     With parent left empty, base_link is the root link. -->',
         '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
         "  <xacro:macro name=\"raft\" params=\"prefix:='' parent:='' xyz:='0 0 0' rpy:='0 0 0'\">",
         "    <xacro:if value=\"${parent != ''}\">",
         '      <joint name="${prefix}mount_joint" type="fixed">',
         '        <parent link="${parent}"/>',
         '        <child link="${prefix}base_link"/>',
         '        <origin xyz="${xyz}" rpy="${rpy}"/>',
         '      </joint>',
         '    </xacro:if>']
    X += ["  " + l for l in body]
    X += ['  </xacro:macro>', '</robot>']
    Path(macro_path).write_text("\n".join(X) + "\n", encoding="utf-8")


def write_release(root):
    root = Path(root)
    for sub in ("urdf", "mjcf", "meshes"):
        (root / sub).mkdir(parents=True, exist_ok=True)
    # a new SolidWorks export brings its meshes in tools/cad/meshes (upper-case .STL); without one, the meshes
    # already in raft_description/meshes stay as they are
    mesh_src = _CAD / "meshes"
    if mesh_src.is_dir():
        for n in ["base_link"] + NAMES:
            shutil.copyfile(mesh_src / f"{n}.STL", root / "meshes" / f"{n}.stl")
    header = ['<!-- RAFT leader arm (raft_description). GENERATED by tools/gen_model.py from the CM6 DH table, the RAFT link',
              '     lengths and the SolidWorks mass properties. Do not hand-edit; see ../README.md.',
              '     Link i frame = DH frame i-1 and joint i turns about its z axis; q = 0 is the standby pose (= PAR6 standby).',
              '     PAR6 / RCB DH angles: theta = q + [0, -pi/2, pi, 0, 0, pi], not wrapped. Effort = servo stall torque.',
              '     Limits: J2 and J3 from CAD collision checks; J1 and J4-J6 are +-180 deg placeholders until measured. -->']
    urdf = root / "urdf" / "raft.urdf"
    write_urdf(str(urdf), mesh_prefix="package://raft_description/meshes/", ext=".stl", header=header,
               extra=['  <!-- lets MuJoCo load this file directly; it drops the package:// part of mesh paths -->',
                      '  <mujoco><compiler meshdir="../meshes/" fusestatic="false"/></mujoco>'], named=True)
    print("  ", urdf); check_urdf(str(urdf))
    write_macro(urdf, root / "urdf" / "raft_macro.xacro"); print("  ", root / "urdf" / "raft_macro.xacro")
    write_mjcf(str(root / "mjcf" / "raft.xml"), str(root / "mjcf" / "scene.xml"), release=True)
    print("  ", root / "mjcf" / "raft.xml"); print("  ", root / "mjcf" / "scene.xml"); check_mjcf(str(root / "mjcf" / "scene.xml"))
    print("  ", root / "meshes", f"({len(NAMES) + 1} STL files)")


if __name__ == "__main__":
    _SIM_MODEL.mkdir(parents=True, exist_ok=True)
    out_urdf = os.environ.get("RAFT_URDF_OUT", str(_SIM_MODEL / "RAFT.urdf"))
    out_dir = os.environ.get("RAFT_SRC_OUT", str(Path(__file__).resolve().parents[1] / "src" / "main"))
    print("\n=== Writing ===")
    write_urdf(out_urdf); print("  ", out_urdf); check_urdf(out_urdf)
    mj_dir = Path(os.environ.get("RAFT_MJCF_OUT", str(_SIM_MODEL)))
    mj_dir.mkdir(exist_ok=True)
    write_mjcf(str(mj_dir / "RAFT.xml"), str(mj_dir / "scene.xml"), meshdir="../../../../raft_description/meshes")
    print("  ", mj_dir / "RAFT.xml"); print("  ", mj_dir / "scene.xml"); check_mjcf(str(mj_dir / "scene.xml"))
    write_headers(os.path.join(out_dir, "raft_model.h"), os.path.join(out_dir, "golden_vectors.h"))
    print("  ", os.path.join(out_dir, "raft_model.h")); print("  ", os.path.join(out_dir, "golden_vectors.h"))
    rel = os.environ.get("RAFT_RELEASE_OUT", str(_REPO / "raft_description"))
    print("\n=== Public description package ===")
    write_release(rel)
