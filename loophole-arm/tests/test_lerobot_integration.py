"""Tests for LeRobot integration.

These tests verify the wiring — entry-point registration, config schema,
motor layout — without actually opening a serial connection. Hardware-in-
the-loop tests are marked ``@pytest.mark.hardware`` and skipped in CI.
"""

from __future__ import annotations

import pytest

# Skip the whole module unless LeRobot AND its transitive deps are fully
# installed — i.e. ``pip install loophole-arm[hardware]``.
pytest.importorskip("lerobot")
pytest.importorskip("draccus")
pytest.importorskip("lerobot.robots.config")


def test_config_registers_under_canonical_name() -> None:
    """The config must register under ``loophole_arm`` for CLI auto-discovery."""
    from lerobot.robots.config import RobotConfig

    from loophole_arm.robot_config import LoopholeArmConfig  # noqa: F401  (triggers registration)

    # The decorator side-effect adds the name to the registry.
    assert "loophole_arm" in RobotConfig.get_choices()


def test_motor_layout_matches_urdf_joint_count() -> None:
    """Sanity: 7 servos == 6 arm DoFs + 1 gripper, matching the URDF."""
    from loophole_arm.robot import _MOTOR_LAYOUT

    assert len(_MOTOR_LAYOUT) == 7
    assert "gripper" in _MOTOR_LAYOUT
    # IDs must be unique and contiguous starting at 1.
    ids = sorted(servo_id for servo_id, _ in _MOTOR_LAYOUT.values())
    assert ids == list(range(1, 8))


def test_config_defaults_are_safe() -> None:
    """Defaults must not silently disable safety features."""
    from loophole_arm.robot_config import LoopholeArmConfig

    cfg = LoopholeArmConfig()
    assert cfg.disable_torque_on_disconnect is True, "must default to safe-shutdown"
    assert cfg.max_relative_target is not None, "must default to a velocity-limit clamp"
    assert cfg.max_relative_target > 0


def test_observation_action_schemas_match() -> None:
    """Action features must be a subset of observation features (joint positions)."""
    from loophole_arm.robot import LoopholeArm
    from loophole_arm.robot_config import LoopholeArmConfig

    arm = LoopholeArm(LoopholeArmConfig(port="/dev/null"))  # not connected
    motor_keys = {f"{m}.pos" for m in arm.bus.motors}
    assert set(arm.action_features) == motor_keys
    assert motor_keys.issubset(arm.observation_features)


def test_entrypoint_is_discoverable() -> None:
    """LeRobot's entry-point group should expose the robot class."""
    from importlib.metadata import entry_points

    eps = entry_points(group="lerobot.robots")
    names = {ep.name for ep in eps}
    assert "loophole_arm" in names


@pytest.mark.hardware
def test_can_connect_to_real_arm() -> None:
    """Smoke-test against an actual arm on /dev/ttyUSB0."""
    from loophole_arm.robot import LoopholeArm
    from loophole_arm.robot_config import LoopholeArmConfig

    arm = LoopholeArm(LoopholeArmConfig(port="/dev/ttyUSB0"))
    try:
        arm.connect(calibrate=False)
        obs = arm.get_observation()
        assert all(k.endswith(".pos") for k in obs if isinstance(obs[k], float))
    finally:
        if arm.is_connected:
            arm.disconnect()
