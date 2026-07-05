"""Arm-spec builders used by :mod:`loophole_arm.control.workcell`.

This module exposes two private builders — ``_build_feetech_spec`` and
``_build_ur5e_spec`` — that produce a bare ``mujoco.MjSpec`` for each
supported arm. ``control.workcell`` then attaches one or more of these
into a scene parent spec (see :func:`build_multi_arm_model`).

The previous module-level cup-lift composer, ``SceneConfig``, ``ResolvedScene``,
``build_spec`` / ``build_model``, and ``end_effector_body`` were removed along
with the reward-hacking research demo; the bare arm builders are kept because
they remain the foundation of the multi-arm scene.
"""

from __future__ import annotations

from pathlib import Path

import mujoco

_PKG_ROOT = Path(__file__).resolve().parents[3]
_MENAGERIE = _PKG_ROOT / "assets" / "menagerie"
_FEETECH = _PKG_ROOT / "assets" / "feetech_arm"


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
        Multiplier on the base position gains. The control workcell uses a
        higher value (~2.0) so the arm holds commanded poses against gravity.
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
