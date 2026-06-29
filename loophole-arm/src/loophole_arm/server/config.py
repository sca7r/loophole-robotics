"""Load a scene + arm configuration from a YAML file.

  loophole-armd --scene examples/scenes/pickplace_dual.yaml

The YAML mirrors the in-code API one-for-one — anything you can build with
``Scene().add_table(...).add_object(...)`` plus a list of
:class:`ArmInstance`s, you can also write as YAML. Per-arm safety limits are
optional; arms that omit them use ``SafetyLimits.feetech_default()``.

Schema (all sections optional; ``arms`` must have at least one entry)::

    arms:
      - name: arm_a
        kind: feetech                       # default: feetech
        mount_pos: [0.0, 0.0, 0.10]
        safety:                             # optional, overrides defaults
          workspace_min: [-0.05, -0.30, 0.10]
          workspace_max: [0.35, 0.30, 0.45]
          max_joint_step: 0.15              # scalar applies to all 6 joints
          joint_lower: [-3.14, -1.57, ...]  # optional, 6 values
          joint_upper: [ 3.14,  1.57, ...]
          joint_margin: 0.05

    scene:
      reference_axes: true
      reference_axes_origin: [0.0, 0.0, 0.10]
      table_grid: true

      tables:
        - size: [0.35, 0.45]
          height: 0.10
          pos: [0.0, 0.0]

      objects:
        - kind: cube                        # cube | sphere | cylinder
          size: 0.025                       # scalar; tuple for cylinder [r, h/2]
          pos: [0.18, 0.08, 0.13]
          color: orange                     # named, or [r,g,b] / [r,g,b,a]
          mass: 0.05                        # optional
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

from loophole_arm.control.limits import SafetyLimits
from loophole_arm.control.scene import Scene
from loophole_arm.control.workcell import ArmInstance


class SceneConfigError(ValueError):
    """Raised when a scene YAML is malformed."""


def _xyz(value: Any, field: str) -> tuple[float, float, float]:
    """Require a 3-element list/tuple of floats."""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise SceneConfigError(f"{field!r} must be a 3-element list [x, y, z]")
    return (float(value[0]), float(value[1]), float(value[2]))


def _xy(value: Any, field: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise SceneConfigError(f"{field!r} must be a 2-element list [x, y]")
    return (float(value[0]), float(value[1]))


def _limits_from_dict(d: dict, arm_name: str) -> SafetyLimits:
    """Build a SafetyLimits, filling in feetech defaults for omitted fields."""
    base = SafetyLimits.feetech_default()
    # Optional overrides — anything missing stays at the default.
    joint_lower = np.array(d["joint_lower"], dtype=float) if "joint_lower" in d else base.joint_lower
    joint_upper = np.array(d["joint_upper"], dtype=float) if "joint_upper" in d else base.joint_upper
    if "max_joint_step" in d:
        step = d["max_joint_step"]
        max_joint_step = np.full(6, float(step)) if isinstance(step, (int, float)) else np.array(step, dtype=float)
    else:
        max_joint_step = base.max_joint_step
    workspace_min = np.array(d["workspace_min"], dtype=float) if "workspace_min" in d else base.workspace_min
    workspace_max = np.array(d["workspace_max"], dtype=float) if "workspace_max" in d else base.workspace_max
    joint_margin = float(d.get("joint_margin", base.joint_margin))

    for arr, name, expected in [
        (joint_lower, "joint_lower", 6), (joint_upper, "joint_upper", 6),
        (max_joint_step, "max_joint_step", 6),
        (workspace_min, "workspace_min", 3), (workspace_max, "workspace_max", 3),
    ]:
        if arr.shape != (expected,):
            raise SceneConfigError(
                f"arm {arm_name!r} safety.{name}: expected {expected} values, got {arr.shape[0]}"
            )
    return SafetyLimits(
        joint_lower=joint_lower, joint_upper=joint_upper,
        max_joint_step=max_joint_step,
        workspace_min=workspace_min, workspace_max=workspace_max,
        joint_margin=joint_margin,
    )


def _scene_from_dict(d: dict) -> Scene:
    """Build a Scene from the 'scene' YAML section."""
    scene = Scene(
        reference_axes=bool(d.get("reference_axes", False)),
        reference_axes_origin=_xyz(d.get("reference_axes_origin", [0.0, 0.0, 0.0]),
                                    "scene.reference_axes_origin"),
        table_grid=bool(d.get("table_grid", False)),
    )
    for i, t in enumerate(d.get("tables", []) or []):
        if "size" not in t or "height" not in t:
            raise SceneConfigError(f"tables[{i}]: 'size' and 'height' are required")
        scene.add_table(
            size=_xy(t["size"], f"tables[{i}].size"),
            height=float(t["height"]),
            pos=_xy(t.get("pos", [0.0, 0.0]), f"tables[{i}].pos"),
            name=str(t.get("name", "")),
        )
    for i, o in enumerate(d.get("objects", []) or []):
        if "pos" not in o:
            raise SceneConfigError(f"objects[{i}]: 'pos' is required")
        size = o.get("size", 0.025)
        # cylinders need a 2-tuple (radius, half-height); pass lists through as tuples
        if isinstance(size, list):
            size = tuple(float(s) for s in size)
        scene.add_object(
            kind=o.get("kind", "cube"),
            size=size,
            pos=_xyz(o["pos"], f"objects[{i}].pos"),
            color=o.get("color", "orange"),
            mass=float(o.get("mass", 0.05)),
            name=str(o.get("name", "")),
        )
    return scene


def load_scene_config(
    path: str | Path,
) -> tuple[Scene, list[ArmInstance], dict[str, SafetyLimits]]:
    """Load a scene config from YAML.

    Returns
    -------
    scene
        The :class:`Scene` (tables, objects, lighting flags).
    arms
        Ordered :class:`ArmInstance` list — feeds straight into
        :func:`build_multi_arm_model`.
    per_arm_limits
        Mapping ``arm_name -> SafetyLimits``. Every arm has an entry; arms that
        omitted the ``safety:`` section get ``SafetyLimits.feetech_default()``.
    """
    path = Path(path)
    if not path.exists():
        raise SceneConfigError(f"scene config not found: {path}")
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise SceneConfigError(f"{path}: top-level YAML must be a mapping")

    arms_raw = data.get("arms") or []
    if not arms_raw:
        raise SceneConfigError(f"{path}: at least one entry under 'arms' is required")

    arms: list[ArmInstance] = []
    per_arm_limits: dict[str, SafetyLimits] = {}
    for i, a in enumerate(arms_raw):
        if "name" not in a:
            raise SceneConfigError(f"arms[{i}]: 'name' is required")
        name = str(a["name"])
        inst = ArmInstance(
            name=name,
            kind=str(a.get("kind", "feetech")),
            mount_pos=_xyz(a.get("mount_pos", [0.0, 0.0, 0.10]), f"arms[{i}].mount_pos"),
        )
        arms.append(inst)
        safety = a.get("safety") or {}
        per_arm_limits[name] = (
            _limits_from_dict(safety, name) if safety else SafetyLimits.feetech_default()
        )

    scene = _scene_from_dict(data.get("scene") or {})
    return scene, arms, per_arm_limits
