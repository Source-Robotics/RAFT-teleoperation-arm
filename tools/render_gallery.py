"""Render the README's pictures of RAFT from its MuJoCo model into "Photos/": two views of the whole arm and the arm
in five poses, each in a light and a dark studio (GitHub shows the one matching the reader's theme), and the cover
at the top, RAFT_arm_img.png (the dark view with the handle held in front).

The arm's meshes are split into printed parts, servos, horns and screws as in render_assembly.py, each with its own
material in the colours of RAFT's palette in robot_descriptions (the raft_* keys), and posed with the simulators'
model's keyframes. The ESP32-PoE is not in the model; given Olimex's board STEP
(ESP32-PoE_Rev_L.step, or a GLB made from it) it sits on the Base as in the assembly manual (STEP needs cascadio).

    pip install mujoco trimesh pillow numpy
    python tools/render_gallery.py [--esp32 path/to/ESP32-PoE_Rev_L.step]
"""
import argparse
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_assembly as ra  # noqa: E402  (shell sorting, the board, fonts)

OUT = ra.ROOT / "Photos"
SS = 2                                                   # rendered at twice the saved size, then scaled down
BODIES = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_trigger"]
POSES = [("standby", "Standby"), ("handle_in_front", "Handle in front"), ("reach_forward", "Reach forward"),
         ("stretched", "Stretched"), ("rest", "Rest")]

# Source Robotics palette for the studio: Ghost Gray / Light Gray (light), Dark Charcoal / Dark Gray (dark)
THEMES = {
    "light": dict(top=(0.961, 0.961, 0.961), bottom=(0.894, 0.894, 0.894), floor=(0.93, 0.93, 0.935), refl=0.0,
                  ink=(41, 43, 55), key=0.58, fill=0.26, rim=0.22, amb=0.34),
    "dark": dict(top=(0.161, 0.169, 0.216), bottom=(0.067, 0.071, 0.086), floor=(0.09, 0.094, 0.11), refl=0.22,
                 ink=(228, 228, 228), key=0.6, fill=0.18, rim=0.5, amb=0.24),
}
# the model's palette: printed parts, servos, horns, screws (render_assembly's piece classes -> palette keys)
# RAFT's starting colours in robot_descriptions' palette (source/appearance/colors.json: raft_* keys, motors)
PALETTE = {"structure": [0.3, 0.31, 0.335, 1], "motors": [0.06, 0.06, 0.065, 1], "horns": [0.74, 0.76, 0.79, 1],
           "fasteners": [0.42, 0.43, 0.46, 1]}
def _rgba(v):                                            # "#RRGGBB" or [r, g, b, 1], as gen_model accepts
    return [int(v[i:i + 2], 16) / 255 for i in (1, 3, 5)] + [1] if isinstance(v, str) else v


MATS = {cls: dict(rgba=" ".join(f"{c:.4g}" for c in _rgba(PALETTE[key])), specular=sp, shininess=sh)
        for cls, key, sp, sh in (("printed", "structure", "0.25", "0.35"), ("servo", "motors", "0.45", "0.6"),
                                 ("horn", "horns", "0.9", "0.9"), ("screw", "fasteners", "0.8", "0.8"))}


def pieces_with_links(esp32):
    """the arm's pieces (standby pose, world coordinates) with the link each one rides on"""
    meshes, axes = ra.world_meshes()
    pieces = ra.build_pieces(meshes, axes)
    owner = {}
    for link in ra.LINKS:                                # every piece is cut from one link's mesh: match a vertex
        for v in np.round(meshes[link].vertices, 7):
            owner.setdefault(tuple(v), link)
    for p in pieces.values():
        p["link"] = owner.get(tuple(np.round(p["mesh"].vertices[0], 7)), "base_link")
    if esp32:
        for name, p in ra.board_pieces(esp32).items():
            p["link"] = "base_link"
            pieces[name] = p
    return pieces


def build_model(pieces, theme, tmp):
    t = THEMES[theme]
    mats, assets, bodies = [], [], {b: [] for b in BODIES}
    for k, v in MATS.items():
        mats.append(f'<material name="m_{k}" rgba="{v["rgba"]}" specular="{v["specular"]}" shininess="{v["shininess"]}" reflectance="0"/>')
    for name, p in pieces.items():
        path = Path(tmp) / f"{name}.stl"
        p["mesh"].export(path)
        assets.append(f'<mesh name="{name}" file="{path.as_posix()}" inertia="shell"/>')
        if p["cls"] == "electronics":
            r, g, b, a = p["rgba"]
            mats.append(f'<material name="m_{name}" rgba="{r} {g} {b} {a}" specular="0.4" shininess="0.5"/>')
            mat = f"m_{name}"
        else:
            mat = f"m_{p['cls']}"
        bodies[p["link"]].append(f'<geom type="mesh" mesh="{name}" material="{mat}" contype="0" conaffinity="0"/>')
    hz = tuple((a + b) / 2 for a, b in zip(t["top"], t["bottom"]))     # the sky gradient at the horizon
    top, bot, fl, hor = (" ".join(f"{c:.3f}" for c in x) for x in (t["top"], t["bottom"], t["floor"], hz))
    body_xml = "".join(f'<body name="{b}" mocap="true">{"".join(g)}</body>' for b, g in bodies.items() if g)
    xml = f"""<mujoco>
  <statistic center="0.1 0 0.06" extent="0.45"/>
  <visual>
    <global offwidth="{1600 * SS}" offheight="{1000 * SS}" fovy="{FOVY}"/>
    <quality shadowsize="16384" offsamples="8"/>
    <map shadowclip="0.75" shadowscale="0.6" fogstart="2.2" fogend="6" znear="0.01"/>
    <rgba fog="{hor} 1" haze="{hor} 1"/>
    <headlight ambient="{t['amb']} {t['amb']} {t['amb']}" diffuse="0.18 0.18 0.18" specular="0.05 0.05 0.05"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="{top}" rgb2="{bot}" width="512" height="3072"/>
    <material name="floor" rgba="{fl} 1" specular="0.1" shininess="0.2" reflectance="{t['refl']}"/>
    {''.join(mats)}
    {''.join(assets)}
  </asset>
  <worldbody>
    <light pos="0.25 -0.35 1.2" dir="-0.2 0.32 -1" directional="true" castshadow="true" diffuse="{t['key']} {t['key']} {t['key']}" specular="0.35 0.35 0.35"/>
    <light pos="0.6 0.8 0.5" dir="-0.55 -0.75 -0.4" directional="true" castshadow="false" diffuse="{t['fill']} {t['fill']} {t['fill']}" specular="0.05 0.05 0.05"/>
    <light pos="-0.8 0.5 0.7" dir="0.75 -0.45 -0.6" directional="true" castshadow="false" diffuse="{t['rim']} {t['rim']} {t['rim']}" specular="0.5 0.5 0.5"/>
    <geom type="plane" size="8 8 0.01" pos="0 0 -0.0005" material="floor" contype="0" conaffinity="0"/>
    {body_xml}
  </worldbody>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


class Poser:
    """moves every link's mocap body from its standby placement to a keyframe of the robot model"""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(ra.MJCF))
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.T0 = {b: self.pose(b) for b in BODIES}

    def pose(self, b):
        i = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, b)
        T = np.eye(4); T[:3, :3] = self.d.xmat[i].reshape(3, 3); T[:3, 3] = self.d.xpos[i]
        return T

    def apply(self, key, sm, sd):
        k = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_KEY, key)
        self.d.qpos[:] = self.m.key_qpos[k]
        mujoco.mj_forward(self.m, self.d)
        for b in BODIES:
            bid = mujoco.mj_name2id(sm, mujoco.mjtObj.mjOBJ_BODY, b)
            if bid < 0:
                continue
            M = self.pose(b) @ np.linalg.inv(self.T0[b])
            q = np.zeros(4); mujoco.mju_mat2Quat(q, M[:3, :3].ravel())
            mid = sm.body_mocapid[bid]
            sd.mocap_pos[mid] = M[:3, 3]; sd.mocap_quat[mid] = q
        mujoco.mj_forward(sm, sd)


FOVY = 24.0


def cam_axes(az, el):
    a, e = np.radians(az), np.radians(el)
    fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.cross(fwd, [0, 0, 1]); right /= np.linalg.norm(right)
    return fwd, right, np.cross(right, fwd)


def fit(points, az, el, aspect, margin=1.12):
    """lookat and distance so the points fill the frame (perspective, FOVY vertical)"""
    fwd, right, up = cam_axes(az, el)
    c = (points.min(0) + points.max(0)) / 2
    rel = points - c
    tv = np.tan(np.radians(FOVY) / 2); th = tv * aspect
    x, y, z = rel @ right, rel @ up, rel @ fwd
    d = max(np.max((np.abs(x) / th) - z), np.max((np.abs(y) / tv) - z)) * margin   # every point inside the frustum
    return c, d


def render(r, sd, lookat, distance, azimuth, elevation, size):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = distance; cam.azimuth = azimuth; cam.elevation = elevation
    r.update_scene(sd, camera=cam)
    return Image.fromarray(r.render()).resize(size, Image.LANCZOS)


def posed_points(pieces, poser, key, links=None):
    """a sample of the arm's vertices in a keyframe (only the given links, if any)"""
    k = mujoco.mj_name2id(poser.m, mujoco.mjtObj.mjOBJ_KEY, key)
    poser.d.qpos[:] = poser.m.key_qpos[k]
    mujoco.mj_forward(poser.m, poser.d)
    pts = []
    for p in pieces.values():
        if links and p["link"] not in links:
            continue
        v = p["mesh"].vertices[::7]
        M = poser.pose(p["link"]) @ np.linalg.inv(poser.T0[p["link"]])
        pts.append(v @ M[:3, :3].T + M[:3, 3])
    return np.vstack(pts)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--esp32", help="the ESP32-PoE STEP (or GLB), to show the board on the Base")
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    pieces = pieces_with_links(a.esp32)
    poser = Poser()
    HERO = ("handle_in_front", 116, -18)
    BACK = ("reach_forward", 312, -24)
    SIDE = (94, -13)
    TILE = (600, 470)
    # one scale for every pose tile: the largest distance any pose needs
    side = {k: fit(posed_points(pieces, poser, k), *SIDE, TILE[0] / TILE[1], 1.04) for k, _ in POSES}
    dside = max(d for _, d in side.values())
    hero = fit(posed_points(pieces, poser, HERO[0]), HERO[1], HERO[2], 1.6, 1.16)
    back = fit(posed_points(pieces, poser, BACK[0]), BACK[1], BACK[2], 1.6, 1.08)
    with tempfile.TemporaryDirectory() as tmp:
        for theme in THEMES:
            sm = build_model(pieces, theme, tmp)
            sd = mujoco.MjData(sm)
            r = mujoco.Renderer(sm, 1000 * SS, 1600 * SS)
            ink = THEMES[theme]["ink"]
            poser.apply(HERO[0], sm, sd)
            img = render(r, sd, hero[0], hero[1], HERO[1], HERO[2], (1600, 1000))
            img.save(OUT / f"render_hero_{theme}.png", optimize=True)
            if theme == "dark":
                img.save(OUT / "RAFT_arm_img.png", optimize=True)      # the README's cover
            poser.apply(BACK[0], sm, sd)
            render(r, sd, back[0], back[1], BACK[1], BACK[2], (1600, 1000)).save(OUT / f"render_back_{theme}.png", optimize=True)
            r.close()
            r = mujoco.Renderer(sm, TILE[1] * SS, TILE[0] * SS)
            strip = Image.new("RGB", (TILE[0] * len(POSES), TILE[1]))
            for i, (key, label) in enumerate(POSES):
                poser.apply(key, sm, sd)
                c = side[key][0].copy(); c[2] = 0.105                     # the same height for every tile
                t = render(r, sd, c, dside, SIDE[0], SIDE[1], TILE)
                ImageDraw.Draw(t).text((28, 24), label, fill=ink, font=ra.font(28, bold=True))
                strip.paste(t, (TILE[0] * i, 0))
            strip.save(OUT / f"render_poses_{theme}.png", optimize=True)
            r.close()
            print("wrote", theme)


if __name__ == "__main__":
    main()
