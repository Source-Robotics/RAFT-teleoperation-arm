"""Build the RAFT URDF viewer (dev tool, run on a PC after gen_model.py).

Packs tools/sim/model/RAFT.urdf and its CAD meshes (tools/sim/model/meshes) into one HTML page (three.js from a CDN):
  docs/raft_viewer.html            standalone page, open it in a browser (needs internet for three.js); not in git
  <--fragment PATH>                the same page without the html/head/body wrapper (for publishing)

Meshes are quantized to 16-bit positions per mesh bounding box (about 2 um steps) with 16-bit indices.
"""
import argparse
import base64
import json
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
URDF = HERE / "sim" / "model" / "RAFT.urdf"
MESH_DIR = HERE / "sim" / "model" / "meshes"


def pack_mesh(path):
    m = trimesh.load(path)                          # merges duplicate vertices
    v = np.asarray(m.vertices, dtype=np.float64)
    f = np.asarray(m.faces, dtype=np.int64)
    assert len(v) < 65536, f"{path.name}: too many vertices for 16-bit indices"
    lo, hi = v.min(0), v.max(0)
    span = np.where(hi - lo > 0, hi - lo, 1.0)
    q = np.round((v - lo) / span * 65535).astype("<u2")
    err = np.abs(lo + q / 65535.0 * span - v).max()
    return {"min": lo.tolist(), "max": hi.tolist(),
            "pos": base64.b64encode(q.tobytes()).decode("ascii"),
            "idx": base64.b64encode(f.astype("<u2").tobytes()).decode("ascii")}, len(f), err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fragment", help="also write the page without the html wrapper here")
    ap.add_argument("--out", default=str(HERE.parent / "docs" / "raft_viewer.html"))
    a = ap.parse_args()

    meshes, tris, worst = {}, 0, 0.0
    for p in sorted(MESH_DIR.glob("*.stl")):
        meshes[p.stem], n, err = pack_mesh(p)
        tris += n; worst = max(worst, err)
    data = {"urdf": URDF.read_text(encoding="utf-8"), "meshes": meshes}
    page = (HERE / "viewer_template.html").read_text(encoding="utf-8")
    assert "/*__RAFT_DATA__*/null" in page
    page = page.replace("/*__RAFT_DATA__*/null", json.dumps(data, separators=(",", ":")))

    standalone = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
                  '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
                  '</head>\n<body>\n' + page + '\n</body>\n</html>\n')
    Path(a.out).write_text(standalone, encoding="utf-8")
    print(f"{len(meshes)} meshes, {tris} triangles, max quantization error {worst * 1e6:.2f} um")
    print(f"wrote {a.out} ({len(standalone) / 1e6:.2f} MB)")
    if a.fragment:
        Path(a.fragment).write_text(page, encoding="utf-8")
        print(f"wrote {a.fragment}")


if __name__ == "__main__":
    main()
