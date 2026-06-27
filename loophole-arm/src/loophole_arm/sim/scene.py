"""Programmatic scene composition.

Two arms are supported:

* ``ur5e`` — Universal Robots UR5e + Robotiq 2F-85 from DeepMind Menagerie.
  Used for high-fidelity benchmarking against an industrial-grade reference.
* ``feetech`` — Custom 6-DOF Feetech-servo arm with a 1-DOF prismatic gripper.
  The deployment target: low-cost, ROS 2-compatible, real hardware reachable.

Scene composition is done with :class:`mujoco.MjSpec` so vendored model
files stay drop-in replaceable and mesh paths resolve correctly.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import mujoco

_PKG_ROOT = Path(__file__).resolve().parents[3]
_MENAGERIE = _PKG_ROOT / "assets" / "menagerie"
_FEETECH = _PKG_ROOT / "assets" / "feetech_arm"

ArmName = Literal["ur5e", "feetech"]


# ── Per-arm tuning ─────────────────────────────────────────────────────────
# Feetech actuator gains, tuned per joint. The servos have ~3 N·m peak,
# so kp values are small; the shoulder lift fights gravity and gets more.
_FEETECH_KP = {
    "Joint_1": 30.0,
    "Joint_2": 50.0,
    "Joint_3": 40.0,
    "Joint_4": 25.0,
    "Joint_5": 20.0,
    "Joint_6": 15.0,
    "Joint_Gripper": 80.0,
}

# Home pose (radians) and scene placement tuned for each arm's reach envelope.
# Feetech reach ≈ 35 cm; UR5e reach ≈ 85 cm.
_ARM_DEFAULTS: dict[str, dict] = {
    "feetech": {
        "home_qpos": (0.0, -0.5, 1.0, 0.0, 0.0, 0.0, 0.0),
        "cup_pos": (0.18, 0.0, 0.16),
        "table_pos": (0.18, 0.0, 0.05),
        "table_half_size": (0.10, 0.12, 0.05),
    },
    "ur5e": {
        "home_qpos": (0.0, -1.2, 1.6, -1.8, -1.57, 0.0),
        "cup_pos": (0.55, 0.0, 0.46),
        "table_pos": (0.55, 0.0, 0.20),
        "table_half_size": (0.25, 0.30, 0.20),
    },
}


@dataclass(frozen=True)
class SceneConfig:
    """Tunable parameters for the cup-lift scene."""

    arm: ArmName = "feetech"
    cup_radius: float = 0.022
    cup_half_height: float = 0.04
    cup_density: float = 250.0
    cup_friction: tuple[float, float, float] = (1.2, 0.05, 0.001)

    # Optional overrides. None means "use the arm's default."
    cup_pos: tuple[float, float, float] | None = None
    table_pos: tuple[float, float, float] | None = None
    table_half_size: tuple[float, float, float] | None = None
    home_qpos: tuple[float, ...] | None = None

    def resolved(self) -> ResolvedScene:
        d = _ARM_DEFAULTS[self.arm]
        return ResolvedScene(
            arm=self.arm,
            cup_pos=self.cup_pos or d["cup_pos"],
            table_pos=self.table_pos or d["table_pos"],
            table_half_size=self.table_half_size or d["table_half_size"],
            home_qpos=self.home_qpos or d["home_qpos"],
            cup_radius=self.cup_radius,
            cup_half_height=self.cup_half_height,
            cup_density=self.cup_density,
            cup_friction=self.cup_friction,
        )


@dataclass(frozen=True)
class ResolvedScene:
    """Scene with all defaults applied — convenient for env code."""

    arm: ArmName
    cup_pos: tuple[float, float, float]
    table_pos: tuple[float, float, float]
    table_half_size: tuple[float, float, float]
    home_qpos: tuple[float, ...]
    cup_radius: float
    cup_half_height: float
    cup_density: float
    cup_friction: tuple[float, float, float]


# ── Builders ───────────────────────────────────────────────────────────────
def _build_feetech_spec(
    *,
    kp_scale: float = 1.0,
    add_velocity_damping: bool = False,
    force_limit: float = 3.0,
    armature: float = 0.0,
    joint_damping: float = 0.0,
    disable_self_collision: bool = False,
) -> mujoco.MjSpec:
    """Build the Feetech arm spec with position actuators.

    Parameters
    ----------
    kp_scale:
        Multiplier on the base position gains. The reward-hacking sim uses
        1.0 (weak, realistic servos). The control workcell uses a higher
        value so the arm holds commanded poses against gravity.
    add_velocity_damping:
        Add a velocity-feedback term (kv) for a critically-damped PD response.
    force_limit:
        Per-joint torque clamp (N·m).
    armature:
        Reflected rotor inertia (models the servo gearbox). Critical for
        stable position control — without it, stiff gains are unstable.
    joint_damping:
        Per-joint viscous damping.
    """
    urdf = _FEETECH / "arm_mujoco.urdf"
    if not urdf.exists():
        raise FileNotFoundError(
            f"Feetech URDF not found at {urdf}. Run `python scripts/convert_feetech_urdf.py` first."
        )
    spec = mujoco.MjSpec.from_file(str(urdf))

    # Joint-level damping + armature for numerical stability under position
    # control. Without armature, stiff actuators are unstable at 2 ms.
    if armature > 0 or joint_damping > 0:
        for joint in spec.joints:
            if joint.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                joint.damping[0] = joint_damping
                joint.armature = armature

    # Disable arm self-collision. Adjacent serial-link geoms overlap by design;
    # checking them produces spurious contact forces that fight the actuators.
    # Standard idiom: put all robot geoms in contype/conaffinity group 2, and
    # the world (floor/table, group 1) in group 1. Group 2 geoms collide with
    # group 1 but not with each other.
    if disable_self_collision:
        for geom in spec.geoms:
            geom.contype = 2
            geom.conaffinity = 1   # collides only with group-1 (world) geoms

    # Add position actuators (URDF doesn't carry these for MuJoCo).
    joint_ranges = {j.name: j.range for j in spec.joints}
    for joint_name, base_kp in _FEETECH_KP.items():
        lo, hi = joint_ranges[joint_name]
        kp = base_kp * kp_scale
        # Critically-damped PD: kv ≈ 2·sqrt(kp) gives a smooth, non-oscillatory
        # approach to the setpoint. biasprm = [b0, b1(=-kp), b2(=-kv)].
        kv = 2.0 * (kp ** 0.5) if add_velocity_damping else 0.0
        act = spec.add_actuator(
            name=joint_name,
            target=joint_name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            gainprm=[kp, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            biasprm=[0, -kp, -kv, 0, 0, 0, 0, 0, 0, 0],
            forcerange=[-force_limit, force_limit],
            ctrlrange=[float(lo), float(hi)],
            ctrllimited=True,
        )
        # CRITICAL: a position servo needs gaintype=FIXED + biastype=AFFINE so
        # that force = kp*(ctrl - qpos) - kv*qvel. Without setting biastype the
        # bias is ignored and the actuator applies raw kp*ctrl, which saturates.
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    return spec


def _build_ur5e_spec() -> mujoco.MjSpec:
    ur5e_xml = _MENAGERIE / "universal_robots_ur5e" / "ur5e.xml"
    gripper_xml = _MENAGERIE / "robotiq_2f85" / "2f85.xml"
    if not ur5e_xml.exists():
        raise FileNotFoundError(f"UR5e model not found at {ur5e_xml}. Run `make fetch-assets`.")
    arm = mujoco.MjSpec.from_file(str(ur5e_xml))
    gripper = mujoco.MjSpec.from_file(str(gripper_xml))
    arm.option.impratio = 10.0
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    arm.site("attachment_site").attach_body(gripper.body("base_mount"), "gripper_", "")
    return arm


# ── Public API ─────────────────────────────────────────────────────────────
def build_spec(cfg: SceneConfig | None = None) -> mujoco.MjSpec:
    """Compose the configured arm + table + free-body cup into a single spec."""
    cfg = cfg or SceneConfig()
    r = cfg.resolved()

    spec = _build_feetech_spec() if cfg.arm == "feetech" else _build_ur5e_spec()

    world = spec.worldbody

    # Lighting
    world.add_light(
        pos=[0, 0, 2.0],
        dir=[0, 0, -1],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    )
    world.add_light(pos=[0.8, -0.6, 1.5], dir=[-0.5, 0.4, -1])

    # Floor (neither URDF includes one).
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[3, 3, 0.05],
        rgba=[0.88, 0.88, 0.88, 1],
    )

    # Table
    world.add_geom(
        name="table",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=list(r.table_pos),
        size=list(r.table_half_size),
        rgba=[0.62, 0.45, 0.30, 1],
    )

    # Free-body cup
    cup = world.add_body(name="cup", pos=list(r.cup_pos))
    cup.add_freejoint(name="cup_free")
    cup.add_geom(
        name="cup_geom",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[r.cup_radius, r.cup_half_height, 0.0],
        rgba=[0.85, 0.25, 0.25, 1],
        density=r.cup_density,
        friction=list(r.cup_friction),
    )

    return spec


def build_model(cfg: SceneConfig | None = None) -> mujoco.MjModel:
    """Convenience: compose, compile, return a ready-to-use ``MjModel``."""
    return build_spec(cfg).compile()


def end_effector_body(arm: ArmName) -> str:
    """Body name used by env code to track the gripper / TCP."""
    return "Gripper" if arm == "feetech" else "gripper_base_mount"
