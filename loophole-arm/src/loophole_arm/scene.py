"""Programmatic scene composition for UR5e + Robotiq 2F-85 + tabletop cup.

We build the scene with ``mujoco.MjSpec`` rather than a static XML so the
canonical robot models stay vendored under ``assets/menagerie/`` and their
mesh paths resolve correctly. The compiled :class:`mujoco.MjModel` is the
single artifact the rest of the code consumes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import mujoco

# Resolve menagerie at import time so consumers don't need to know the layout.
_PKG_ROOT = Path(__file__).resolve().parents[2]
_MENAGERIE = _PKG_ROOT / "assets" / "menagerie"
_UR5E_XML = _MENAGERIE / "universal_robots_ur5e" / "ur5e.xml"
_GRIPPER_XML = _MENAGERIE / "robotiq_2f85" / "2f85.xml"


@dataclass(frozen=True)
class SceneConfig:
    """Tunable parameters for the cup-lift scene."""

    cup_pos: tuple[float, float, float] = (0.55, 0.0, 0.46)
    cup_radius: float = 0.032
    cup_half_height: float = 0.05
    cup_density: float = 250.0
    cup_friction: tuple[float, float, float] = (1.2, 0.05, 0.001)
    table_pos: tuple[float, float, float] = (0.55, 0.0, 0.20)
    table_half_size: tuple[float, float, float] = (0.25, 0.30, 0.20)
    # Initial joint configuration (radians) — neutral "ready" pose
    home_qpos: tuple[float, ...] = field(
        default_factory=lambda: (0.0, -1.2, 1.6, -1.8, -1.57, 0.0)
    )


def build_spec(cfg: SceneConfig | None = None) -> mujoco.MjSpec:
    """Compose UR5e + Robotiq 2F-85 + table + free-body cup into a single spec."""
    cfg = cfg or SceneConfig()

    if not _UR5E_XML.exists():
        raise FileNotFoundError(
            f"UR5e model not found at {_UR5E_XML}. "
            "Run `make fetch-assets` to vendor Menagerie."
        )

    arm = mujoco.MjSpec.from_file(str(_UR5E_XML))
    gripper = mujoco.MjSpec.from_file(str(_GRIPPER_XML))

    # Reconcile physics options the gripper relies on.
    arm.option.impratio = 10.0
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC

    # Attach gripper at the UR5e's wrist site.
    arm.site("attachment_site").attach_body(gripper.body("base_mount"), "gripper_", "")

    world = arm.worldbody

    # Lighting
    world.add_light(
        pos=[0, 0, 2.0],
        dir=[0, 0, -1],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    )
    world.add_light(pos=[0.8, -0.6, 1.5], dir=[-0.5, 0.4, -1])

    # Floor (Menagerie's ur5e.xml omits one)
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[3, 3, 0.05],
        rgba=[0.88, 0.88, 0.88, 1],
    )

    # Table sized within the UR5e reach envelope (~85 cm)
    world.add_geom(
        name="table",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=list(cfg.table_pos),
        size=list(cfg.table_half_size),
        rgba=[0.62, 0.45, 0.30, 1],
    )

    # Free-body cup
    cup = world.add_body(name="cup", pos=list(cfg.cup_pos))
    cup.add_freejoint(name="cup_free")
    cup.add_geom(
        name="cup_geom",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[cfg.cup_radius, cfg.cup_half_height, 0.0],
        rgba=[0.85, 0.25, 0.25, 1],
        density=cfg.cup_density,
        friction=list(cfg.cup_friction),
    )

    return arm


def build_model(cfg: SceneConfig | None = None) -> mujoco.MjModel:
    """Convenience: compose, compile, return a ready-to-use ``MjModel``."""
    return build_spec(cfg).compile()
