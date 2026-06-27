"""
Run this first on your local machine to check everything is set up correctly.

    python check_setup.py
"""
from __future__ import annotations

import os
import platform
import sys


def check(name: str, fn) -> tuple[bool, str]:
    try:
        result = fn()
        return True, result or "ok"
    except Exception as e:
        return False, str(e)


checks: list[tuple[str, bool, str]] = []

# Python
v = sys.version_info
checks.append(("Python >= 3.10", v >= (3, 10), f"{v.major}.{v.minor}.{v.micro}"))

# Core packages
for pkg, import_name in [
    ("mujoco", "mujoco"),
    ("numpy", "numpy"),
    ("imageio", "imageio"),
    ("PyYAML", "yaml"),
]:
    ok, detail = check(
        pkg,
        lambda n=import_name: __import__(n) and getattr(__import__(n), "__version__", "ok"),
    )
    checks.append((pkg, ok, detail if ok else f"pip install {pkg.lower()}"))

# GLFW (needed for live window)
try:
    import glfw
    checks.append(("glfw (live window)", True, "ok"))
except ImportError:
    checks.append(("glfw (live window)", False, "pip install glfw"))

# imageio-ffmpeg (video export)
try:
    import imageio_ffmpeg
    checks.append(("imageio-ffmpeg (video)", True, imageio_ffmpeg.__version__))
except ImportError:
    checks.append(("imageio-ffmpeg (video)", False, "pip install imageio-ffmpeg"))

# Display environment
system = platform.system()
if system == "Linux":
    display = os.environ.get("DISPLAY", "")
    wayland = os.environ.get("WAYLAND_DISPLAY", "")
    if display:
        checks.append(("Display (Linux/X11)", True, display))
    elif wayland:
        checks.append(("Display (Linux/Wayland)", True, wayland))
    else:
        checks.append((
            "Display (Linux)",
            False,
            "DISPLAY not set — are you in a desktop session? (not SSH)"
        ))
elif system == "Darwin":
    checks.append(("Display (macOS)", True, "native — works out of the box"))
elif system == "Windows":
    checks.append(("Display (Windows)", True, "native — works out of the box"))

# loophole-arm package itself
try:
    sys.path.insert(0, "src")
    import loophole_arm
    checks.append(("loophole-arm", True, loophole_arm.__version__))
except ImportError:
    checks.append(("loophole-arm", False, "pip install -e .   (from repo root)"))

# MuJoCo live viewer
try:
    import mujoco.viewer
    checks.append(("mujoco.viewer", True, "available"))
except ImportError:
    checks.append(("mujoco.viewer", False, "upgrade: pip install mujoco>=3.2"))

print("\n" + "=" * 55)
print("  Loophole Arm — local setup check")
print("=" * 55 + "\n")

all_ok = True
for name, ok, detail in checks:
    icon = "✅" if ok else "❌"
    print(f"  {icon}  {name:<35} {detail}")
    if not ok:
        all_ok = False

print()
if all_ok:
    print("  All good! Run the viewer:\n")
    print("    # Live scene (free-roam, drag camera with mouse):")
    print("    python viewer.py\n")
    print("    # Optimize then watch live:")
    print("    python viewer.py --optimize --reward shaped_lift\n")
    print("    # Replay a saved run:")
    print("    python viewer.py --params runs/<timestamp>_shaped_lift/best_params.npy\n")
else:
    print("  Fix the ❌ items above, then run python viewer.py")
    print()
    print("  Quick fix — install everything:")
    print("    pip install -r requirements.txt")
    print("    pip install -e .")
