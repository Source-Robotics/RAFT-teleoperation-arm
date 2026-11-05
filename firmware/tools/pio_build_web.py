# PlatformIO pre-build hook (env:main): re-embed web/index.html into src/main/web_assets.h when the page
# changed. Build-time dev tool only; see tools/build_web.py.
Import("env")  # noqa: F821  (provided by PlatformIO / SCons)

import os
import sys

_tools = os.path.join(env.subst("$PROJECT_DIR"), "tools")  # noqa: F821
if _tools not in sys.path:
    sys.path.insert(0, _tools)
import build_web  # noqa: E402

if build_web.build(root=env.subst("$PROJECT_DIR")):  # noqa: F821
    print("web UI embedded into src/main/web_assets.h")
