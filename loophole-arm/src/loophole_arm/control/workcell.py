"""The workcell — composes a :class:`Scene` with one or more robots.

This is the thin assembly layer. It owns the industrial styling (materials,
floor/skybox textures) shared across all scenes; the *content* of the stage
(tables, objects, lighting) lives in :mod:`loophole_arm.control.scene`, and
the *robot* definitions live in the existing per-arm builders.

Two ways to build a workcell:

    # Simple, single-arm — backwards-compatible:
    model = build_workcell_model(WorkcellConfig(arm="feetech"))

    # Composable:
    scene = Scene().add_table(...).add_object(...)
    model = build_workcell_from_scene(scene, arm="feetech")
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco

from loophole_arm.control.scene import (
    Scene,
    TableSpec,
    add_scene_to_spec,
)
from loophole_arm.sim.scene import _build_feetech_spec, _build_ur5e_spec


@dataclass(frozen=True)
class WorkcellConfig:
    """Single-arm convenience config — kept for the existing entry points.

    The arm is mounted on a single workbench. For multi-table scenes, multi-
    robot setups, or custom object placement, build a :class:`Scene` and call
    :func:`build_workcell_from_scene`.
    """
    arm: str = "feetech"
    table_height: float = 0.10
    table_size: tuple[float, float] = (0.35, 0.45)
    add_work_objects: bool = True


# ── Industrial styling (materials, skybox, floor texture) ────────────────
def _apply_industrial_styling(spec: mujoco.MjSpec) -> None:
    """Studio lighting defaults, skybox, floor texture, neutral materials."""
    spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    spec.visual.headlight.ambient = [0.3, 0.3, 0.3]
    spec.visual.headlight.specular = [0.1, 0.1, 0.1]
    spec.visual.rgba.haze = [0.85, 0.87, 0.90, 1.0]
    spec.visual.global_.azimuth = 140
    spec.visual.global_.elevation = -22
    spec.visual.quality.shadowsize = 8192

    spec.add_texture(
        name="skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.78, 0.80, 0.82],
        rgb2=[0.55, 0.57, 0.60],
        width=512,
        height=3072,
    )
    spec.add_texture(
        name="floor_tex",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.35, 0.36, 0.38],
        rgb2=[0.29, 0.30, 0.32],
        markrgb=[0.45, 0.46, 0.48],
        mark=mujoco.mjtMark.mjMARK_EDGE,
        width=400,
        height=400,
    )
    spec.add_material(
        name="floor_mat",
        textures=["", "floor_tex"] + [""] * 8,
        texuniform=True,
        texrepeat=[6, 6],
        reflectance=0.12,
        specular=0.3,
        shininess=0.1,
    )
    spec.add_material(
        name="steel_mat",
        rgba=[0.62, 0.64, 0.67, 1.0],
        reflectance=0.25,
        specular=0.6,
        shininess=0.6,
    )
    spec.add_material(
        name="frame_mat",
        rgba=[0.20, 0.21, 0.23, 1.0],
        reflectance=0.05,
        specular=0.2,
        shininess=0.2,
    )
    spec.add_material(
        name="workpiece_mat",
        rgba=[0.90, 0.45, 0.10, 1.0],
        reflectance=0.05,
        specular=0.3,
        shininess=0.3,
    )


# ── TCP / arm coloring helpers ───────────────────────────────────────────
TCP_SITE = "tcp"

_TCP_OFFSET: dict[str, tuple[float, float, float]] = {
    "feetech": (0.045, 0.0, 0.005),
    "ur5e": (0.0, 0.0, 0.10),
}
_TCP_PARENT: dict[str, str] = {
    "feetech": "Gripper",
    "ur5e": "gripper_base_mount",
}
_GRIPPER_BODIES: dict[str, set[str]] = {
    "feetech": {"Gripper"},
    "ur5e": {"gripper_base_mount"},
}


def _color_arm_geoms(spec: mujoco.MjSpec, arm: str) -> None:
    """Apply the arm body + accent gripper colors to the imported geoms.

    URDF import gives each geom an explicit ``rgba`` which takes precedence
    over assigned materials in MuJoCo, so we set ``rgba`` directly on each
    geom. This guarantees the arm reads as a clean industrial robot at the
    pitch (clean off-white body, bright orange accent gripper).
    """
    arm_body_rgba = [0.93, 0.94, 0.96, 1.0]
    gripper_rgba = [0.95, 0.45, 0.10, 1.0]
    gripper_bodies = _GRIPPER_BODIES.get(arm, set())
    for geom in spec.geoms:
        body_name = geom.parent.name if geom.parent is not None else ""
        if body_name.startswith("Link"):
            geom.rgba = arm_body_rgba
        elif body_name in gripper_bodies:
            geom.rgba = gripper_rgba


def _add_tcp_site(spec: mujoco.MjSpec, arm: str) -> None:
    """Attach a TCP site to the gripper body as the IK control frame."""
    parent_name = _TCP_PARENT[arm]
    offset = _TCP_OFFSET[arm]
    parent = next(b for b in spec.bodies if b.name == parent_name)
    parent.add_site(
        name=TCP_SITE,
        pos=list(offset),
        size=[0.005, 0.005, 0.005],
        group=4,
    )


def _build_arm_spec(arm: str) -> mujoco.MjSpec:
    """Build the bare arm spec (no scene yet), with control-grade physics."""
    if arm == "feetech":
        return _build_feetech_spec(
            kp_scale=2.0,
            add_velocity_damping=True,
            force_limit=10.0,
            armature=0.05,
            joint_damping=2.0,
            disable_self_collision=True,
        )
    if arm == "ur5e":
        return _build_ur5e_spec()
    raise ValueError(f"unknown arm {arm!r}")


# ── Public builders ──────────────────────────────────────────────────────
def build_workcell_spec(
    scene: Scene,
    arm: str = "feetech",
    arm_mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> mujoco.MjSpec:
    """Build the MjSpec for a scene + arm (does not compile).

    Useful when you need the spec itself — e.g. to dump XML so a remote client
    can load the same kinematic model.
    """
    spec = _build_arm_spec(arm)
    _apply_industrial_styling(spec)
    _color_arm_geoms(spec, arm)
    root = spec.worldbody.bodies[0]
    root.pos = list(arm_mount_pos)
    _add_tcp_site(spec, arm)
    add_scene_to_spec(spec, scene)
    return spec


def build_workcell_from_scene(
    scene: Scene,
    arm: str = "feetech",
    arm_mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> mujoco.MjModel:
    """Compile a :class:`Scene` + a single arm into a ready :class:`MjModel`.

    The arm is mounted at ``arm_mount_pos`` (defaults to origin). If the scene
    has tables, the arm typically sits on the first one — pass an
    ``arm_mount_pos`` whose Z equals the table height.
    """
    return build_workcell_spec(scene, arm=arm, arm_mount_pos=arm_mount_pos).compile()


def build_workcell_model(cfg: WorkcellConfig | None = None) -> mujoco.MjModel:
    """Compile a single-arm workcell from a :class:`WorkcellConfig`.

    Backwards-compatible entry point. Builds a scene with one table (sized
    by ``cfg``) and mounts the arm on top of it.
    """
    cfg = cfg or WorkcellConfig()
    scene = Scene()
    scene.tables.append(TableSpec(
        size=cfg.table_size,
        height=cfg.table_height,
        pos=(0.0, 0.0),
        name="worktable",
    ))
    return build_workcell_from_scene(
        scene,
        arm=cfg.arm,
        arm_mount_pos=(0.0, 0.0, cfg.table_height),
    )


# ── Multi-arm builder ────────────────────────────────────────────────────
# One simulation, N independent arms. Each arm is attached to the scene with a
# name prefix (e.g. "arm_a/") so its joints, bodies, actuators, and TCP site
# get a unique namespace. Per-arm name maps tell the server how to wire each
# SimBackend.
@dataclass(frozen=True)
class ArmInstance:
    """One arm in a multi-arm scene."""
    name: str                                # endpoint name, becomes the prefix
    kind: str = "feetech"                    # which arm model
    mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ArmHandle:
    """How the server addresses one attached arm in the compiled model.

    All names are already prefixed with the arm's ``name`` (e.g. ``"arm_a/"``)
    by MuJoCo's ``MjSpec.attach``.
    """
    name: str
    arm_joints: list[str]
    gripper_actuator: str
    tcp_site: str


def build_multi_arm_model(
    scene: Scene,
    arms: list[ArmInstance],
) -> tuple[mujoco.MjModel, mujoco.MjSpec, list[ArmHandle]]:
    """Compile a scene + N independently-prefixed arms.

    Returns the compiled model, the spec (for XML dump to clients), and one
    :class:`ArmHandle` per arm with the already-prefixed names the server
    needs to wire its :class:`SimBackend`s.
    """
    if not arms:
        raise ValueError("at least one arm is required")
    seen: set[str] = set()
    for a in arms:
        if a.name in seen:
            raise ValueError(f"duplicate arm name {a.name!r}")
        if "/" in a.name:
            raise ValueError(f"arm name {a.name!r} cannot contain '/'")
        seen.add(a.name)

    # Parent (scene-only) spec: styling, lighting, floor, tables, objects.
    parent = mujoco.MjSpec()
    _apply_industrial_styling(parent)
    add_scene_to_spec(parent, scene)

    # Attach each arm with its prefix at its mount position.
    handles: list[ArmHandle] = []
    for inst in arms:
        child = _build_arm_spec(inst.kind)
        # Match parent's integrator default to silence the cosmetic "integrator
        # differs" warning on attach. The arm's URDF picks implicitfast; the
        # parent's empty default is Euler — both are fine for our control
        # rates, and the parent's choice wins after attach anyway.
        child.option.integrator = parent.option.integrator
        # Clear the child's modelname so two attached copies don't warn about
        # a duplicate "arm_description" name (cosmetic — same model, twice).
        child.modelname = ""
        _color_arm_geoms(child, inst.kind)
        _add_tcp_site(child, inst.kind)
        frame = parent.worldbody.add_frame(pos=list(inst.mount_pos))
        parent.attach(child, prefix=f"{inst.name}/", frame=frame)
        prefix = f"{inst.name}/"
        handles.append(ArmHandle(
            name=inst.name,
            arm_joints=[f"{prefix}Joint_{i}" for i in range(1, 7)],
            gripper_actuator=f"{prefix}Joint_Gripper",
            tcp_site=f"{prefix}{TCP_SITE}",
        ))

    return parent.compile(), parent, handles
