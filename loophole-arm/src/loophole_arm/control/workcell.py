"""Professional industrial workcell scene.

A clean, neutral factory-cell aesthetic following MuJoCo Menagerie conventions:
proper materials (specular/shininess/reflectance), studio lighting, a textured
floor, and a defined work surface. The robot model is *included*, not redefined
here — same separation Menagerie uses (model file vs. scene file).

This scene is GENERIC. It never changes between tasks. Task-specific object
placement and motion live in the command layer.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco

from loophole_arm.sim.scene import _build_feetech_spec, _build_ur5e_spec


@dataclass(frozen=True)
class WorkcellConfig:
    """Layout of the industrial workcell (all dimensions in metres).

    The arm is mounted ON the work surface (standard tabletop manipulation),
    not on the floor. Table dimensions are scaled to the arm's reach so the
    workspace is fully reachable.
    """

    arm: str = "feetech"
    # Table is a low platform the arm sits on; sized to the arm's footprint.
    table_height: float = 0.10
    table_size: tuple[float, float] = (0.35, 0.45)   # half-extents (x, y)
    add_work_objects: bool = True


# ── Professional materials & visual settings ───────────────────────────────
# Injected into the spec so the scene reads as a real workcell, not primitives.
def _apply_industrial_styling(spec: mujoco.MjSpec) -> None:
    """Add studio lighting, skybox, floor texture, and PBR-ish materials."""
    # Visual/global settings — soft headlight, slight haze for depth.
    spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    spec.visual.headlight.ambient = [0.3, 0.3, 0.3]
    spec.visual.headlight.specular = [0.1, 0.1, 0.1]
    spec.visual.rgba.haze = [0.85, 0.87, 0.90, 1.0]
    spec.visual.global_.azimuth = 140
    spec.visual.global_.elevation = -22
    spec.visual.quality.shadowsize = 8192   # crisp shadows

    # Skybox — neutral light-grey gradient (clean industrial, not a blue sky).
    spec.add_texture(
        name="skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.78, 0.80, 0.82],
        rgb2=[0.55, 0.57, 0.60],
        width=512,
        height=3072,
    )

    # Floor — fine checker, low reflectance (polished concrete look).
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
        textures=["", "floor_tex"] + [""] * 8,  # texture slot 1 (2D)
        texuniform=True,
        texrepeat=[6, 6],
        reflectance=0.12,
        specular=0.3,
        shininess=0.1,
    )

    # Work surface — brushed steel table top.
    spec.add_material(
        name="steel_mat",
        rgba=[0.62, 0.64, 0.67, 1.0],
        reflectance=0.25,
        specular=0.6,
        shininess=0.6,
    )
    # Table frame — matte dark grey.
    spec.add_material(
        name="frame_mat",
        rgba=[0.20, 0.21, 0.23, 1.0],
        reflectance=0.05,
        specular=0.2,
        shininess=0.2,
    )
    # Work object — safety-orange, matte.
    spec.add_material(
        name="workpiece_mat",
        rgba=[0.90, 0.45, 0.10, 1.0],
        reflectance=0.05,
        specular=0.3,
        shininess=0.3,
    )


# The canonical control-frame (Tool Center Point) site name. The IK solver and
# any hardware TCP calibration refer to this single name.
TCP_SITE = "tcp"

# Per-arm TCP placement relative to the gripper body origin (metres). Tuned so
# the site sits between the gripper fingers at the grasp point.
_TCP_OFFSET: dict[str, tuple[float, float, float]] = {
    "feetech": (0.045, 0.0, 0.005),
    "ur5e": (0.0, 0.0, 0.10),
}

# The body each arm's TCP site attaches to.
_TCP_PARENT: dict[str, str] = {
    "feetech": "Gripper",
    "ur5e": "gripper_base_mount",
}


def _add_tcp_site(spec: mujoco.MjSpec, arm: str) -> None:
    """Attach a TCP site to the gripper body as the IK control frame."""
    parent_name = _TCP_PARENT[arm]
    offset = _TCP_OFFSET[arm]
    parent = next(b for b in spec.bodies if b.name == parent_name)
    parent.add_site(
        name=TCP_SITE,
        pos=list(offset),
        size=[0.005, 0.005, 0.005],
        group=4,  # group 4 = hidden by default in the viewer; not clutter
    )


def _build_industrial_workcell(cfg: WorkcellConfig) -> mujoco.MjSpec:
    if cfg.arm == "feetech":
        # Control needs the arm to hold poses against gravity. Real Feetech
        # servos achieve this via high gear-ratio (reflected rotor inertia),
        # modelled here with armature. Moderate kp + velocity damping +
        # armature gives stable, non-oscillatory position holding at the
        # URDF's 2 ms timestep.
        spec = _build_feetech_spec(
            kp_scale=2.0,
            add_velocity_damping=True,
            force_limit=10.0,
            armature=0.05,
            joint_damping=2.0,
            disable_self_collision=True,
        )
    else:
        spec = _build_ur5e_spec()
    _apply_industrial_styling(spec)

    # Raise the arm base so it stands ON the table surface, not the floor.
    # The arm's root body is the first non-world body in the kinematic tree.
    root = spec.worldbody.bodies[0]
    root.pos = [root.pos[0], root.pos[1], cfg.table_height]

    # Add a Tool Center Point (TCP) site on the gripper. This is the control
    # frame the IK solver tracks — placed between the fingers where grasping
    # actually happens, NOT at the wrist link origin. Defining an explicit TCP
    # site is standard practice: it is the single source of truth for "where
    # the hand is", identical in sim and on hardware.
    _add_tcp_site(spec, cfg.arm)

    world = spec.worldbody

    # Studio three-point lighting (key, fill, rim) for clean shadows.
    world.add_light(
        pos=[0.6, -0.6, 1.4], dir=[-0.4, 0.4, -1.0],
        diffuse=[0.7, 0.7, 0.7], specular=[0.3, 0.3, 0.3],
        castshadow=True,
    )
    world.add_light(
        pos=[-0.6, -0.4, 1.2], dir=[0.4, 0.3, -1.0],
        diffuse=[0.3, 0.3, 0.35], specular=[0.1, 0.1, 0.1],
        castshadow=False,
    )
    world.add_light(
        pos=[0.0, 0.8, 1.0], dir=[0.0, -0.6, -1.0],
        diffuse=[0.25, 0.25, 0.28], castshadow=False,
    )

    # Floor.
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05],
        material="floor_mat",
    )

    # ── Work platform: steel top on a dark frame, arm mounted on top ───
    h = cfg.table_height
    sx, sy = cfg.table_size
    top_thickness = 0.015
    leg = 0.025

    table = world.add_body(name="worktable", pos=[0.0, 0.0, 0.0])
    # Steel work surface (top face at z = h)
    table.add_geom(
        name="table_top",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0, 0, h - top_thickness],
        size=[sx, sy, top_thickness],
        material="steel_mat",
    )
    # Four legs (dark frame)
    for i, (lx, ly) in enumerate([(sx - leg, sy - leg), (sx - leg, -(sy - leg)),
                                  (-(sx - leg), sy - leg), (-(sx - leg), -(sy - leg))]):
        table.add_geom(
            name=f"table_leg_{i}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[lx, ly, (h - top_thickness) / 2],
            size=[leg, leg, (h - top_thickness) / 2],
            material="frame_mat",
        )

    return spec


def build_workcell_model(cfg: WorkcellConfig | None = None) -> mujoco.MjModel:
    """Compose and compile the industrial workcell. Returns a ready MjModel."""
    cfg = cfg or WorkcellConfig()
    return _build_industrial_workcell(cfg).compile()
