"""Tests for scene composition."""

from __future__ import annotations

import pytest

from loophole_arm.sim.scene import SceneConfig, build_model, end_effector_body


def test_feetech_compose() -> None:
    """The Feetech scene has 6 arm joints + 1 gripper actuator."""
    m = build_model(SceneConfig(arm="feetech"))
    names = [m.actuator(i).name for i in range(m.nu)]
    assert names == [
        "Joint_1",
        "Joint_2",
        "Joint_3",
        "Joint_4",
        "Joint_5",
        "Joint_6",
        "Joint_Gripper",
    ]
    assert m.nu == 7


def test_ur5e_compose() -> None:
    """The UR5e scene has 6 arm joints + 1 gripper actuator."""
    m = build_model(SceneConfig(arm="ur5e"))
    names = [m.actuator(i).name for i in range(m.nu)]
    assert names[:6] == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ]
    assert m.nu == 7


def test_feetech_actuator_ranges_are_nondegenerate() -> None:
    """ctrlrange must be set from joint ranges so the optimizer can decode."""
    m = build_model(SceneConfig(arm="feetech"))
    for i in range(m.nu):
        lo, hi = m.actuator_ctrlrange[i]
        assert hi > lo + 1e-3, f"degenerate ctrlrange on {m.actuator(i).name}: [{lo}, {hi}]"


def test_cup_is_a_free_body() -> None:
    """The cup should be a free body — 7-dof joint (3 trans + 4 quat)."""
    m = build_model()
    cup_joint = m.joint("cup_free")
    assert m.nq - cup_joint.qposadr[0] >= 7


def test_scene_config_cup_pos_override() -> None:
    """Custom cup_pos should propagate to the compiled model."""
    custom = SceneConfig(arm="feetech", cup_pos=(0.20, 0.05, 0.15))
    m = build_model(custom)
    cup_qadr = int(m.joint("cup_free").qposadr[0])
    assert abs(m.qpos0[cup_qadr] - 0.20) < 1e-6
    assert abs(m.qpos0[cup_qadr + 1] - 0.05) < 1e-6
    assert abs(m.qpos0[cup_qadr + 2] - 0.15) < 1e-6


@pytest.mark.parametrize(
    "arm,expected",
    [
        ("feetech", "Gripper"),
        ("ur5e", "gripper_base_mount"),
    ],
)
def test_end_effector_body_per_arm(arm: str, expected: str) -> None:
    assert end_effector_body(arm) == expected
