"""Render the README pictures from the MJCF into raft_description/media/.

    pip install mujoco pillow
    python tools/render_media.py
"""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1] / "raft_description"
OUT = ROOT / "media"
W, H = 1280, 960                 # the scene's offscreen buffer (scene.xml <global offwidth/offheight>)


def camera(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = distance; cam.azimuth = azimuth; cam.elevation = elevation
    return cam


def shot(r, m, d, key, cam, opt=None):
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, key))
    mujoco.mj_forward(m, d)
    r.update_scene(d, camera=cam, scene_option=opt)
    return Image.fromarray(r.render())


def font(size):
    for name in ("DejaVuSans.ttf", "arial.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def main():
    OUT.mkdir(exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(ROOT / "mjcf" / "scene.xml")); d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)

    # 1. hero: the handle in front, as it is held for teleoperation
    img = shot(r, m, d, "handle_in_front", camera([0.095, 0.0, 0.115], 0.40, 128, -14))
    img.resize((960, 720), Image.LANCZOS).save(OUT / "raft.png", optimize=True)

    # 2. the keyframes side by side, all from one camera
    keys = ["standby", "handle_in_front", "reach_forward", "stretched", "folded"]
    tiles = []
    for k in keys:
        t = shot(r, m, d, k, camera([0.13, 0.0, 0.12], 0.62, 90, -8)).crop((240, 60, 1040, 900)).resize((400, 420), Image.LANCZOS)
        ImageDraw.Draw(t).text((14, 10), k, fill=(235, 238, 242), font=font(22))
        tiles.append(t)
    strip = Image.new("RGB", (400 * len(tiles), 420))
    for i, t in enumerate(tiles):
        strip.paste(t, (400 * i, 0))
    strip.save(OUT / "keyframes.png", optimize=True)

    # 3. joint frames: body i carries DH frame i-1 (red x, green y, blue z = joint axis), plus the tcp site
    opt = mujoco.MjvOption()
    opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    m.vis.scale.framelength = 0.7; m.vis.scale.framewidth = 0.045
    m.mat_rgba[:, 3] = 0.18                                                   # see-through links
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MATERIAL, "floor"); m.mat_rgba[floor, 3] = 1.0
    img = shot(r, m, d, "standby", camera([0.09, 0.0, 0.11], 0.38, 150, -18), opt)
    img.resize((960, 720), Image.LANCZOS).save(OUT / "frames.png", optimize=True)

    r.close()
    for p in sorted(OUT.glob("*.png")):
        print(f"  {p.relative_to(ROOT.parent)}  {p.stat().st_size // 1024} kB")


if __name__ == "__main__":
    main()
