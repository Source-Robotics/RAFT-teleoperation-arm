"""Render the assembly manual's pictures from the robot description into "Assembly manual/images/".

Every link mesh of the arm's model (the simulators' MJCF, CAD link names) is split into its shells and sorted into printed parts, servos (one piece per
servo, with its 4 mounting screws), horns and M3 screws (the joint stops and the base lid's). The pictures show the
arm in its standby pose, built up step by step:
- **Fitted parts:** light grey.
- **New parts:** in colour.
- **A part that goes onto a horn:** drawn pulled back along that joint's axis, with an arrow to where it goes.

The ESP32-PoE is not in the arm's model. Given its STEP (Olimex ESP32-PoE rev L, `ESP32-PoE_Rev_L.step`), the
pictures show it on the Base's three posts, in its own colours (reading STEP needs `pip install cascadio`):

    pip install mujoco trimesh pillow numpy
    python tools/render_assembly.py [--esp32 path/to/ESP32-PoE_Rev_L.step]
"""
import argparse
import math
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
MJCF = ROOT / "firmware" / "tools" / "sim" / "model" / "RAFT.xml"   # the CAD link names and one mesh per link
OUT = ROOT / "Assembly manual" / "images"
W, H = 1500, 1100                      # rendered size; saved 1000 wide
FOVY = 30.0
LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_trigger"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"]
SERVO_LINK = dict(zip(JOINTS, ["base_link", "link1", "link3", "link3", "link4", "link5", "link6"]))   # where each servo sits

# Source Robotics palette: new printed parts Moderate Cyan, new servos Dark Gray, new horns Soft Blue, new screws
# Dusty Purple; fitted parts in neutral greys
COL = {
    ("printed", "new"): (0.216, 0.776, 0.647, 1), ("printed", "done"): (0.72, 0.735, 0.76, 1), ("printed", "ghost"): (0.72, 0.735, 0.76, 0.25),
    ("servo", "new"): (0.16, 0.17, 0.22, 1), ("servo", "done"): (0.42, 0.43, 0.48, 1), ("servo", "ghost"): (0.42, 0.43, 0.48, 0.3),
    ("horn", "new"): (0.57, 0.74, 0.84, 1), ("horn", "done"): (0.70, 0.72, 0.76, 1), ("horn", "ghost"): (0.70, 0.72, 0.76, 0.3),
    ("screw", "new"): (0.43, 0.37, 0.61, 1), ("screw", "done"): (0.58, 0.59, 0.63, 1), ("screw", "ghost"): (0.58, 0.59, 0.63, 0.3),
}
# the ESP32-PoE on the Base: the board's three 2.2 mm mounting holes, in ESP32-PoE_Rev_L.step's own frame (2.55, 52.68),
# (25.42, 52.69) and (7.37, 2.62) mm, land on the Base's three posts (-3.68, 41.94), (-3.68, 19.10) and (-53.71, 37.12) mm
# (9 mm tall) with the board turned -90 deg about z: 0.03 mm off at most. Found with height maps of both meshes.
ESP_ROT_DEG, ESP_SHIFT = -90.0, np.array([-0.05636, 0.04449, 0.009])
ESP_POSTS = np.array([[-0.00368, 0.04194, 0.009], [-0.00368, 0.01910, 0.009], [-0.05371, 0.03712, 0.009]])
INK = (41, 43, 55)                      # Dark Gray
CYAN = (55, 198, 165)                   # Moderate Cyan
CORE = (20, 57, 82)                     # Core Blue


# ---------------------------------------------------------------- the model's shells in world coordinates (standby pose)
def world_meshes():
    m = mujoco.MjModel.from_xml_path(str(MJCF))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    meshes = {}
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 2:
            continue
        k = m.geom_dataid[g]
        v0, nv, f0, nf = m.mesh_vertadr[k], m.mesh_vertnum[k], m.mesh_faceadr[k], m.mesh_facenum[k]
        v = m.mesh_vert[v0:v0 + nv] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        meshes[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, k)] = trimesh.Trimesh(v, m.mesh_face[f0:f0 + nf], process=True)
    axes = {}
    for j in JOINTS:
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        axes[j] = (d.xanchor[i].copy(), d.xaxis[i].copy())
    return meshes, axes


def kind(shell):
    e = np.sort(shell.extents * 1000)[::-1]
    if shell.is_watertight and abs(e[0] - 18) < 2 and abs(e[1] - 18) < 2 and e[2] < 6:
        return "horn"                                    # 25-tooth disc horn, 18 mm across
    if abs(e[0] - 8) < 1 and abs(e[1] - 5.5) < 0.6 and abs(e[2] - 5.5) < 0.6:
        return "m3"                                      # M3 x 8 socket head (a joint stop)
    if abs(e[0] - 10) < 1 and abs(e[1] - 2.8) < 0.4 and abs(e[2] - 2.8) < 0.4:
        return "m3"                                      # M3 drawn as a plain shank (the base lid)
    if abs(e[0] - 6.4) < 0.5 and abs(e[1] - 1.9) < 0.3:
        return "servo_screw"                             # the servo's own mounting screws
    if (shell.is_watertight and abs(shell.volume) * 1e9 > 2000) or e[0] > 50:
        return "printed"
    return "servo"


def joint_dist(p, anchor, axis):
    """how far p is from a joint: off its axis line, plus a quarter of the way along it from the joint's anchor
    (J4 and J6 share one line in the standby pose)"""
    r = p - anchor
    along = np.dot(r, axis)
    return np.linalg.norm(r - along * axis) + 0.25 * abs(along)


def clusters(shells, gap=0.0004):
    """groups shells whose bounding boxes touch (a servo drawn as many surface pieces)"""
    n = len(shells)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    b = [s.bounds for s in shells]
    for i in range(n):
        for k in range(i + 1, n):
            if np.all(b[i][0] - gap <= b[k][1]) and np.all(b[k][0] - gap <= b[i][1]):
                parent[find(i)] = find(k)
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(shells[i])
    return list(groups.values())


def build_pieces(meshes, axes):
    """pieces: name -> dict(mesh, cls)"""
    pieces, shells = {}, {}
    for link in LINKS:
        shells[link] = {"printed": [], "servo": [], "servo_screw": [], "m3": [], "horn": []}
        for s in meshes[link].split(only_watertight=False):
            shells[link][kind(s)].append(s)
    for link in LINKS:
        for h in shells[link]["horn"]:
            j = min(JOINTS, key=lambda jn: joint_dist(h.centroid, *axes[jn]))
            pieces[f"horn_{j}"] = dict(mesh=h, cls="horn")
    for link in LINKS:
        sh = shells[link]
        lids = [s for s in sh["printed"] if link == "base_link" and s.extents.min() < 0.003]
        body = [s for s in sh["printed"] if s not in lids]
        pieces[f"{link}_printed"] = dict(mesh=trimesh.util.concatenate(body), cls="printed")
        if lids:
            pieces["base_lid"] = dict(mesh=trimesh.util.concatenate(lids), cls="printed")
        # servos: the shells that touch make one servo; the biggest groups are the servos of this link's joints
        joints = [j for j in JOINTS if SERVO_LINK[j] == link]
        if joints:
            gs = sorted(clusters(sh["servo"]), key=lambda g: -sum(x.area for x in g))
            seeds = [list(g) for g in gs[:len(joints)]]
            seed_c = [np.mean([x.centroid for x in sd], axis=0) for sd in seeds]
            for g in gs[len(joints):]:
                c = np.mean([x.centroid for x in g], axis=0)
                k = min(range(len(seeds)), key=lambda i: np.linalg.norm(seed_c[i] - c))
                if np.linalg.norm(seed_c[k] - c) < 0.03:
                    seeds[k].extend(g)
                else:                                    # loose surface pieces of the printed part itself
                    pieces[f"{link}_printed"]["mesh"] = trimesh.util.concatenate([pieces[f"{link}_printed"]["mesh"], *g])
            cent = [np.mean([x.centroid for x in sd], axis=0) for sd in seeds]
            order = min(([0, 1], [1, 0]) if len(joints) == 2 else ([0],),
                        key=lambda o: sum(np.linalg.norm(cent[o[i]] - pieces[f"horn_{j}"]["mesh"].centroid) for i, j in enumerate(joints)))
            for i, j in enumerate(joints):
                sd = seeds[order[i]]
                c = cent[order[i]]
                own = [x for x in sh["servo_screw"] if min(cent, key=lambda cc: np.linalg.norm(cc - x.centroid)) is c]
                pieces[f"servo_{j}"] = dict(mesh=trimesh.util.concatenate(sd), cls="servo")
                if own:
                    pieces[f"sscrew_{j}"] = dict(mesh=trimesh.util.concatenate(own), cls="screw")
        for k, s in enumerate(sh["m3"]):
            pieces[f"m3_{link}_{k}"] = dict(mesh=s, cls="screw")
    return pieces


def board_pieces(path):
    """the ESP32-PoE from its STEP (or a GLB), one piece per colour, placed on the Base's posts"""
    path = Path(path)
    if path.suffix.lower() in (".step", ".stp"):
        import cascadio                                  # pip install cascadio
        glb = Path(tempfile.gettempdir()) / (path.stem + ".glb")
        cascadio.step_to_glb(str(path), str(glb), tol_linear=0.05, tol_angular=0.5)
        path = glb
    sc = trimesh.load(path)
    c, s_ = math.cos(math.radians(ESP_ROT_DEG)), math.sin(math.radians(ESP_ROT_DEG))
    place = np.array([[c, -s_, 0, ESP_SHIFT[0]], [s_, c, 0, ESP_SHIFT[1]], [0, 0, 1, ESP_SHIFT[2]], [0, 0, 0, 1]])
    by_colour = {}
    for node in sc.graph.nodes_geometry:
        tf, name = sc.graph[node]
        g = sc.geometry[name].copy()
        g.apply_transform(place @ tf)
        try:
            rgba = np.array(g.visual.material.baseColorFactor, float)
            rgba = rgba / 255.0 if rgba.max() > 1 else rgba
        except AttributeError:
            rgba = np.array([0.55, 0.56, 0.6, 1.0])
        by_colour.setdefault(tuple(np.round(rgba, 2)), []).append(trimesh.Trimesh(g.vertices, g.faces))
    return {f"esp_{k}": dict(mesh=trimesh.util.concatenate(ms), cls="electronics", rgba=tuple(col))
            for k, (col, ms) in enumerate(by_colour.items())}


# ---------------------------------------------------------------- a static scene: every piece a world geom
def scene(pieces, tmp):
    assets, geoms = [], []
    for name, p in pieces.items():
        path = Path(tmp) / f"{name}.stl"
        p["mesh"].export(path)
        assets.append(f'<mesh name="{name}" file="{path.as_posix()}" inertia="shell"/>')
        geoms.append(f'<geom name="{name}" type="mesh" mesh="{name}" rgba="1 1 1 1" contype="0" conaffinity="0"/>')
    xml = f"""<mujoco>
  <visual>
    <global offwidth="{W}" offheight="{H}" fovy="{FOVY}"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.42 0.42 0.42" diffuse="0.45 0.45 0.45" specular="0.08 0.08 0.08"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2="1 1 1" width="256" height="1536"/>
    <material name="floor" rgba="0.89 0.895 0.905 1" specular="0" reflectance="0"/>
    {''.join(assets)}
  </asset>
  <worldbody>
    <light pos="0.3 -0.4 1.0" dir="-0.25 0.35 -1" directional="true" castshadow="true" diffuse="0.45 0.45 0.45"/>
    <light pos="-0.4 0.3 0.8" dir="0.4 -0.3 -0.8" directional="true" castshadow="false" diffuse="0.18 0.18 0.2"/>
    <geom name="floor" type="plane" size="6 6 0.01" pos="0 0 -0.0005" material="floor" contype="0" conaffinity="0"/>
    {''.join(geoms)}
  </worldbody>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


# ---------------------------------------------------------------- camera and projection
def cam_frame(az, el):
    a, e = math.radians(az), math.radians(el)
    fwd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    return fwd, right, np.cross(right, fwd)


def project(p, cam):
    fwd, right, up = cam_frame(cam.azimuth, cam.elevation)
    dv = np.asarray(p) - (np.array(cam.lookat) - cam.distance * fwd)
    f = (H / 2) / math.tan(math.radians(FOVY) / 2)
    z = dv @ fwd
    return np.array([W / 2 + f * (dv @ right) / z, H / 2 - f * (dv @ up) / z])


def fit_camera(points, az, el, margin):
    pts = np.asarray(points)
    lo, hi = pts.min(0), pts.max(0)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (lo + hi) / 2
    cam.azimuth, cam.elevation = az, el
    cam.distance = margin * max(np.linalg.norm(hi - lo) / 2, 0.035) / math.sin(math.radians(FOVY) / 2)
    return cam


# ---------------------------------------------------------------- 2D overlay: arrows, labels, title
def font(size, bold=False):
    for n in (["segoeuib.ttf", "arialbd.ttf", "DejaVuSans-Bold.ttf"] if bold else ["segoeui.ttf", "arial.ttf", "DejaVuSans.ttf"]):
        try:
            return ImageFont.truetype(n, size)
        except OSError:
            pass
    return ImageFont.load_default()


def arrow(dr, a, b, col=CYAN, width=6):
    a, b = np.asarray(a, float), np.asarray(b, float)
    v = b - a
    L = np.linalg.norm(v)
    if L < 40:
        return
    u = v / L
    n = np.array([-u[1], u[0]])
    s = 0.0
    while s < L - 36:
        dr.line([tuple(a + u * s), tuple(a + u * min(s + 18, L - 36))], fill=col, width=width)
        s += 30
    tip = b
    dr.polygon([tuple(tip), tuple(tip - u * 36 + n * 16), tuple(tip - u * 36 - n * 16)], fill=col)


def label(dr, anchor, text, offset, fnt):
    ax, ay = anchor
    lx, ly = ax + offset[0], ay + offset[1]
    tw, th = dr.textbbox((0, 0), text, font=fnt)[2:]
    pad = 12
    box = [lx, ly - th / 2 - pad, lx + tw + 2 * pad, ly + th / 2 + pad] if offset[0] >= 0 else [lx - tw - 2 * pad, ly - th / 2 - pad, lx, ly + th / 2 + pad]
    shift = min(0, W - 12 - box[2]) + max(0, 12 - box[0])          # keep the box inside the picture
    box = [box[0] + shift, box[1], box[2] + shift, box[3]]
    lx += shift
    dr.line([(ax, ay), (lx, ly)], fill=INK, width=3)
    dr.ellipse([ax - 6, ay - 6, ax + 6, ay + 6], fill=INK)
    dr.rounded_rectangle(box, radius=10, fill=(255, 255, 255), outline=(200, 202, 208), width=2)
    dr.text((box[0] + pad, ly - th / 2 - 5), text, fill=INK, font=fnt)


def title(dr, number, text):
    f1, f2 = font(46, True), font(40)
    nw = dr.textbbox((0, 0), number, font=f1)[2]
    dr.rounded_rectangle([40, 36, 40 + nw + 36, 106], radius=14, fill=CORE)
    dr.text((58, 42), number, fill=(255, 255, 255), font=f1)
    dr.text((40 + nw + 60, 48), text, fill=INK, font=f2)


# ---------------------------------------------------------------- the steps
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--esp32", help="the ESP32-PoE STEP (or GLB), to show the board on the Base")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    meshes, axes = world_meshes()
    P = build_pieces(meshes, axes)
    if args.esp32:
        P.update(board_pieces(args.esp32))
    for k, p in P.items():
        e = np.sort(p["mesh"].extents * 1000)[::-1]
        print(f"  {k:28s} {p['cls']:8s} {e[0]:6.1f} x {e[1]:5.1f} x {e[2]:5.1f} mm")
    J = dict(zip(["J1", "J2", "J3", "J4", "J5", "J6", "J7"], JOINTS))
    box_centre = lambda n: P[n]["mesh"].bounds.mean(axis=0)

    def away(jk):
        """along joint jk's axis, from its servo through its horn"""
        j = J[jk]
        a = axes[j][1]
        return a if np.dot(P[f"horn_{j}"]["mesh"].centroid - P[f"servo_{j}"]["mesh"].centroid, a) > 0 else -a

    def servo_set(jk):
        j = J[jk]
        return [n for n in (f"servo_{j}", f"sscrew_{j}") if n in P]

    def stops(link):
        return [n for n in P if n.startswith(f"m3_{link}_")]

    built, steps = set(), []

    def step(key, number, text, new, move=(), along=None, dist=0.05, ghost=(), labels=(), az=135, el=-20, margin=1.5, at=None, only=None):
        new = [n for n in new if n in P]
        steps.append(dict(key=key, number=number, text=text, new=new, done=set(built), move=[n for n in move if n in P],
                          along=along, dist=dist, ghost=set(ghost), labels=list(labels), az=az, el=el, margin=margin, at=at, only=only))
        built.update(new)

    hJ = lambda jk: f"horn_{J[jk]}"
    step("01_base", "1", "J1 servo into the Base", ["base_link_printed", *servo_set("J1"), hJ("J1")], move=[hJ("J1")],
         along=away("J1"), dist=0.035, ghost=["base_link_printed"],
         labels=[(f"servo_{J['J1']}", "J1 servo, ID 0", (-300, 140)), (hJ("J1"), "J1 horn", (190, -60)),
                 ("base_link_printed", "Base", (260, 150))], az=130, el=-26)
    step("02a_shoulder", "2a", "Shoulder onto J1's horn", ["link1_printed"], move=["link1_printed"], along=away("J1"), dist=0.055,
         labels=[("link1_printed", "Shoulder", (230, -70)), (hJ("J1"), "4 × M2", (-260, 40))], at=hJ("J1"), az=130, el=-22)
    step("02b_j2_servo", "2b", "J2 servo and stop screw into the Shoulder", [*servo_set("J2"), hJ("J2"), *stops("link1")], move=[hJ("J2")],
         along=away("J2"), dist=0.035, ghost=["link1_printed"],
         labels=[(f"servo_{J['J2']}", "J2 servo, ID 1", (230, -170)), (hJ("J2"), "J2 horn", (-250, 110)),
                 *[(n, "M3 × 8 stop screw", (-250, -150)) for n in stops("link1")]], az=130, el=-20)
    step("03_upper_arm", "3", "Upper arm onto J2's horn", ["link2_printed"], move=["link2_printed"], along=away("J2"), dist=0.055,
         labels=[("link2_printed", "Upper arm", (-260, -160)), (hJ("J2"), "4 × M2", (230, 110))], at=hJ("J2"), az=130, el=-14, margin=1.35)
    elbow = ["link3_printed", *servo_set("J3"), *servo_set("J4"), hJ("J3"), hJ("J4"), *stops("link3")]
    step("04a_elbow_servos", "4a", "J3 and J4 servos and stop screw into the Elbow", elbow, ghost=["link3_printed"],
         labels=[(f"servo_{J['J3']}", "J3 servo, ID 2", (-310, 210)), (f"servo_{J['J4']}", "J4 servo, ID 3", (230, -230)),
                 (hJ("J3"), "J3 horn", (-290, -90)), (hJ("J4"), "J4 horn", (250, -70)),
                 *[(n, "M3 × 8 stop screw", (240, 120)) for n in stops("link3")]], az=150, el=-22, margin=1.6)
    # the elbow, with both servos in it, goes onto the upper arm through J3's horn: it moves from the horn toward its servo
    steps.append(dict(key="04b_elbow_on", number="4b", text="Elbow onto the Upper arm", new=[], done=set(built), move=elbow,
                      along=-away("J3"), dist=0.055, ghost=set(), az=140, el=-16, margin=1.45, at=hJ("J3"), only=None,
                      labels=[("link3_printed", "Elbow", (230, -150)), (hJ("J3"), "J3 horn to the Upper arm: 4 × M2", (-140, 230))]))
    step("05a_lower_arm", "5a", "Lower arm onto J4's horn", ["link4_printed"], move=["link4_printed"], along=away("J4"), dist=0.055,
         labels=[("link4_printed", "Lower arm", (200, -170)), (hJ("J4"), "4 × M2", (-250, 120))], at=hJ("J4"), az=130, el=-22)
    step("05b_j5_servo", "5b", "J5 servo and stop screw into the Lower arm", [*servo_set("J5"), hJ("J5"), *stops("link4")], move=[hJ("J5")],
         along=away("J5"), dist=0.035, ghost=["link4_printed"],
         labels=[(f"servo_{J['J5']}", "J5 servo, ID 4", (-320, 140)), (hJ("J5"), "J5 horn", (220, -110)),
                 *[(n, "M3 × 8 stop screw", (240, 90)) for n in stops("link4")]], az=130, el=-22)
    step("06a_wrist", "6a", "Wrist onto J5's horn", ["link5_printed"], move=["link5_printed"], along=away("J5"), dist=0.05,
         labels=[("link5_printed", "Wrist", (220, -150)), (hJ("J5"), "4 × M2", (-260, 120))], at=hJ("J5"), az=130, el=-22)
    step("06b_j6_servo", "6b", "J6 servo and stop screw into the Wrist", [*servo_set("J6"), hJ("J6"), *stops("link5")], move=[hJ("J6")],
         along=away("J6"), dist=0.035, ghost=["link5_printed"],
         labels=[(f"servo_{J['J6']}", "J6 servo, ID 5", (-320, 150)), (hJ("J6"), "J6 horn", (220, 110)),
                 *[(n, "M3 × 8 stop screw", (220, -160)) for n in stops("link5")]], az=130, el=-22)
    step("07a_handle", "7a", "Gripper handle onto J6's horn", ["link6_printed"], move=["link6_printed"], along=away("J6"), dist=0.055,
         labels=[("link6_printed", "Gripper handle", (210, -190)), (hJ("J6"), "4 × M2", (-260, 120))], at=hJ("J6"), az=130, el=-20)
    step("07b_j7_servo", "7b", "Trigger servo into the handle", [*servo_set("J7"), hJ("J7")], move=[hJ("J7")],
         along=away("J7"), dist=0.035, ghost=["link6_printed"],
         labels=[(f"servo_{J['J7']}", "Trigger servo, ID 6", (-330, -150)), (hJ("J7"), "Trigger horn", (230, 110))], az=115, el=-20, margin=2.6)
    step("08_trigger", "8", "Gripper trigger onto its horn", ["gripper_trigger_printed"], move=["gripper_trigger_printed"],
         along=away("J7"), dist=0.045, labels=[("gripper_trigger_printed", "Gripper trigger", (230, 120)), (hJ("J7"), "4 × M2", (-260, -130))],
         at=hJ("J7"), az=130, el=-20)
    board = [n for n in P if n.startswith("esp_")]
    if board:
        base_only = ["base_link_printed", *servo_set("J1"), hJ("J1"), "base_lid", *stops("base_link")]
        pcb = max(board, key=lambda n: P[n]["mesh"].area)
        steps.append(dict(key="09_esp32", number="9", text="ESP32-PoE onto its three posts", new=board, done=set(built), move=board,
                          along=np.array([0.0, 0.0, 1.0]), dist=0.035, ghost=set(), az=200, el=-34, margin=1.25, at=None,
                          only=[*base_only, *board], labels=[(pcb, "ESP32-PoE", (230, -150)),
                                                          (ESP_POSTS[0], "3 posts", (220, 150))]))
        built.update(board)
    lid = ["base_lid", *stops("base_link")]
    step("10_base_lid", "10", "Base lid", lid, move=lid, along=np.array([0.0, 0.0, 1.0]), dist=0.03,
         labels=[("base_lid", "Base lid", (-280, -130)), *[(n, "2 × M3 × 8", (200, -120)) for n in stops("base_link")[:1]]], az=120, el=-30,
         margin=1.3, only=["base_link_printed", *servo_set("J1"), hJ("J1"), *lid, *board])
    names = {"base_link_printed": "Base", "link1_printed": "Shoulder", "link2_printed": "Upper arm", "link3_printed": "Elbow",
             "link4_printed": "Lower arm", "link5_printed": "Wrist", "link6_printed": "Gripper handle", "gripper_trigger_printed": "Gripper trigger"}
    offs = {"base_link_printed": (-260, 90), "link1_printed": (-260, 40), "link2_printed": (-280, 0), "link3_printed": (-230, -130),
            "link4_printed": (-170, -150), "link5_printed": (150, -190), "link6_printed": (220, -90), "gripper_trigger_printed": (200, 130)}
    overview = dict(key="00_arm", number="", text="", new=[], done=set(built), move=[], along=None, dist=0, ghost=set(),
                    labels=[(k, v, offs[k]) for k, v in names.items()], az=128, el=-16, margin=1.12, at=None, only=None)

    with tempfile.TemporaryDirectory() as tmp:
        m, d = scene(P, tmp)
        r = mujoco.Renderer(m, H, W)
        gid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in P}
        pos0 = {n: m.geom_pos[g].copy() for n, g in gid.items()}     # the compiler's offset to each mesh's centre
        for s in [overview] + steps:
            off = s["along"] * s["dist"] if s["along"] is not None else np.zeros(3)
            for n, g in gid.items():
                state = "ghost" if n in s["ghost"] else "new" if n in s["new"] or n in s["move"] else "done" if n in s["done"] else None
                if s["only"] is not None and n not in s["only"]:
                    state = None
                if state and "rgba" in P[n]:                 # the board in its own colours
                    m.geom_rgba[g] = P[n]["rgba"][:3] + ((0.3,) if state == "ghost" else (1.0,))
                else:
                    m.geom_rgba[g] = COL[(P[n]["cls"], state)] if state else (0, 0, 0, 0)
                m.geom_pos[g] = pos0[n] + (off if n in s["move"] else 0)
            mujoco.mj_forward(m, d)
            focus = s["new"] + s["move"] if s["new"] or s["move"] else list(s["done"])
            pts = []
            for n in focus:
                b = P[n]["mesh"].bounds
                pts += [b[0], b[1]] + ([b[0] + off, b[1] + off] if n in s["move"] else [])
            cam = fit_camera(pts, s["az"], s["el"], s["margin"])
            r.update_scene(d, camera=cam)
            img = Image.fromarray(r.render())
            dr = ImageDraw.Draw(img)
            if s["move"]:
                c = box_centre(s["at"]) if s["at"] else np.mean([box_centre(n) for n in s["move"]], axis=0)
                arrow(dr, project(c + off, cam), project(c, cam))
            fl = font(34)
            for n, text, o in s["labels"]:
                if isinstance(n, np.ndarray):                   # a point, not a piece
                    label(dr, project(n, cam), text, o, fl)
                elif n in P:
                    label(dr, project(box_centre(n) + (off if n in s["move"] else 0), cam), text, o, fl)
            if s["number"]:
                title(dr, s["number"], s["text"])
            img.resize((1000, round(1000 * H / W)), Image.LANCZOS).save(OUT / f"{s['key']}.png", optimize=True)
            print(f"  wrote {s['key']}.png")
        r.close()


if __name__ == "__main__":
    main()
