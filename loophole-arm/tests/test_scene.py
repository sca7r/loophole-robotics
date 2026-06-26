"""Tests for scene composition."""
from __future__ import annotations

from loophole_arm.scene import SceneConfig, build_model


def test_compose_arm_and_gripper() -> None:
    """The composed model has the UR5e's 6 joints plus the Robotiq gripper actuator."""
    m = build_model()
    actuator_names = [m.actuator(i).name for i in range(m.nu)]

    # 6 UR5e joints
    assert actuator_names[:6] == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ]
    # Plus one gripper closure command
    assert any("gripper" in n for n in actuator_names[6:])
    assert m.nu == 7


def test_cup_is_a_free_body() -> None:
    """The cup should be a free body — 7-dof joint (3 trans + 4 quat)."""
    m = build_model()
    cup_joint = m.joint("cup_free")
    # qpos slice is 7 entries for a freejoint
    assert m.nq - cup_joint.qposadr[0] >= 7


def test_scene_config_overrides_take_effect() -> None:
    """Custom SceneConfig values should propagate into the compiled model."""
    custom = SceneConfig(cup_pos=(0.4, 0.1, 0.5))
    m = build_model(custom)
    # cup_pos lives in qpos0 of the freejoint
    cup_qadr = int(m.joint("cup_free").qposadr[0])
    # qpos0 is the configured starting pos (within float tolerance)
    assert abs(m.qpos0[cup_qadr] - 0.4) < 1e-6
    assert abs(m.qpos0[cup_qadr + 1] - 0.1) < 1e-6
    assert abs(m.qpos0[cup_qadr + 2] - 0.5) < 1e-6
