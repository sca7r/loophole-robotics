"""Tests for true multi-arm composition.

The goal: ``loophole-armd --arm arm_a --arm arm_b`` produces a scene with
two physically independent arms, and a SimBackend per arm drives only its
own joints. Each arm has its own prefixed joint/actuator/site names.
"""
from __future__ import annotations

import mujoco
import pytest

from loophole_arm.control.scene import Scene
from loophole_arm.control.sim_backend import SimBackend
from loophole_arm.control.workcell import ArmInstance, build_multi_arm_model


def _two_arm_setup():
    scene = (
        Scene()
        .add_table(size=(0.35, 0.40), height=0.10, pos=(0.0, 0.0))
        .add_table(size=(0.35, 0.40), height=0.10, pos=(0.55, 0.0))
    )
    arms = [
        ArmInstance(name="arm_a", mount_pos=(0.0, 0.0, 0.10)),
        ArmInstance(name="arm_b", mount_pos=(0.55, 0.0, 0.10)),
    ]
    model, spec, handles = build_multi_arm_model(scene, arms)
    data = mujoco.MjData(model)
    return model, data, handles


def test_two_arms_compile_with_prefixed_names() -> None:
    model, _, handles = _two_arm_setup()
    # Two arms x 7 actuators (6 arm joints + 1 gripper) = 14 actuated DoFs.
    assert model.nu == 14
    # Each handle's prefixed names must resolve in the compiled model.
    for h in handles:
        assert len(h.arm_joints) == 6
        for jn in h.arm_joints:
            assert model.joint(jn).id >= 0
        assert model.actuator(h.gripper_actuator).id >= 0
        assert model.site(h.tcp_site).id >= 0
    # The arms are truly distinct: arm_a/Joint_1 and arm_b/Joint_1 are different joints.
    assert model.joint("arm_a/Joint_1").id != model.joint("arm_b/Joint_1").id


def test_two_arms_drive_independently() -> None:
    model, data, handles = _two_arm_setup()
    backends = {}
    for h in handles:
        sb = SimBackend(
            model=model, data=data,
            arm_joint_names=h.arm_joints,
            gripper_actuator=h.gripper_actuator,
            tcp_site=h.tcp_site,
        )
        sb.connect()
        backends[h.name] = sb

    # Command opposing positions on joint 0 of each arm.
    backends["arm_a"].send_joint_targets([+0.30, -0.4, 1.0, 0.0, 0.0, 0.0])
    backends["arm_b"].send_joint_targets([-0.30, -0.4, 1.0, 0.0, 0.0, 0.0])
    for _ in range(1500):
        mujoco.mj_step(model, data)

    qa = backends["arm_a"].joint_positions
    qb = backends["arm_b"].joint_positions
    # Each arm reaches its own target (within control tolerance).
    assert abs(qa[0] - 0.30) < 0.05
    assert abs(qb[0] - (-0.30)) < 0.05
    # And they ended up far apart — proof they are not the same arm.
    assert abs(qa[0] - qb[0]) > 0.4


def test_duplicate_arm_names_rejected() -> None:
    scene = Scene().add_table(size=(0.35, 0.40), height=0.10, pos=(0.0, 0.0))
    arms = [ArmInstance(name="arm", mount_pos=(0.0, 0.0, 0.10)),
            ArmInstance(name="arm", mount_pos=(0.55, 0.0, 0.10))]
    with pytest.raises(ValueError, match="duplicate"):
        build_multi_arm_model(scene, arms)


def test_slash_in_arm_name_rejected() -> None:
    scene = Scene().add_table(size=(0.35, 0.40), height=0.10, pos=(0.0, 0.0))
    arms = [ArmInstance(name="arm/a", mount_pos=(0.0, 0.0, 0.10))]
    with pytest.raises(ValueError, match="cannot contain"):
        build_multi_arm_model(scene, arms)


def test_empty_arm_list_rejected() -> None:
    with pytest.raises(ValueError, match="at least one arm"):
        build_multi_arm_model(Scene(), [])


def test_single_arm_via_multi_builder() -> None:
    """The multi-arm builder also handles the 1-arm case cleanly."""
    scene = Scene().add_table(size=(0.35, 0.40), height=0.10, pos=(0.0, 0.0))
    arms = [ArmInstance(name="solo", mount_pos=(0.0, 0.0, 0.10))]
    model, _, handles = build_multi_arm_model(scene, arms)
    assert model.nu == 7
    assert handles[0].name == "solo"
    assert handles[0].arm_joints[0] == "solo/Joint_1"
