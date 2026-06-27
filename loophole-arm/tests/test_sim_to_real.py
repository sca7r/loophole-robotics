"""Tests for the sim-to-real abstraction.

These guard the property that makes deploy-without-rewrite possible: sim and
hardware backends satisfy one interface and share one kinematic model, so a TCP
pose computed in sim matches the pose computed on hardware for the same joint
angles.
"""
from __future__ import annotations

import numpy as np
import pytest

from loophole_arm.control import RobotInterface, SimBackend, make_sim_robot
from loophole_arm.control.hardware_backend import HardwareBackend
from loophole_arm.control.workcell import TCP_SITE, WorkcellConfig, build_workcell_model

_FEETECH_JOINTS = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
_FEETECH_MOTORS = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "wrist_yaw",
]


def test_both_backends_implement_interface() -> None:
    assert issubclass(SimBackend, RobotInterface)
    assert issubclass(HardwareBackend, RobotInterface)
    # No abstract methods left unimplemented.
    assert not SimBackend.__abstractmethods__
    assert not HardwareBackend.__abstractmethods__


def test_sim_backend_lifecycle() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech")
    assert robot.backend.is_connected
    robot.backend.disconnect()
    assert not robot.backend.is_connected


def test_hardware_backend_constructs_without_lerobot() -> None:
    """HardwareBackend must build without a bus or lerobot present.

    Only ``connect()`` may require hardware; construction and FK must not.
    """
    model = build_workcell_model(WorkcellConfig(arm="feetech"))
    hw = HardwareBackend(
        model=model,
        arm_joint_names=_FEETECH_JOINTS,
        lerobot_motor_names=_FEETECH_MOTORS,
        tcp_site=TCP_SITE,
    )
    assert hw.n_arm_joints == 6
    assert not hw.is_connected
    assert hw.kinematic_model().site(TCP_SITE).name == TCP_SITE


def test_sim_and_hardware_fk_agree() -> None:
    """The key sim-to-real invariant: identical joint angles -> identical TCP.

    If this drifts, a trajectory validated in sim would land somewhere else on
    hardware. Holds because both backends use the same kinematic model.
    """
    robot, model, data, home = make_sim_robot(arm="feetech")
    test_q = np.array([0.2, -0.6, 1.1, 0.1, 0.15, -0.05])
    robot.move_joints(test_q, duration=1.0)
    sim_tcp = robot.backend.end_effector_pose()
    settled = robot.backend.joint_positions

    hw_model = build_workcell_model(WorkcellConfig(arm="feetech"))

    class _FixedHW(HardwareBackend):
        @property
        def joint_positions(self) -> np.ndarray:  # type: ignore[override]
            return settled

    hw = _FixedHW(
        model=hw_model,
        arm_joint_names=_FEETECH_JOINTS,
        lerobot_motor_names=_FEETECH_MOTORS,
        tcp_site=TCP_SITE,
    )
    hw_tcp = hw.end_effector_pose()

    drift_mm = float(np.linalg.norm(sim_tcp - hw_tcp) * 1000)
    assert drift_mm < 0.5, f"sim/hardware FK disagree by {drift_mm:.2f} mm"


def test_hardware_factory_rejects_sim_only_arm() -> None:
    """UR5e has no hardware mapping; asking for a hardware build should fail."""
    from loophole_arm.control import make_hardware_robot

    with pytest.raises(ValueError, match="no hardware mapping"):
        make_hardware_robot(arm="ur5e")


def test_hardware_commands_require_connection() -> None:
    """Sending commands before connect() must raise, not silently no-op."""
    model = build_workcell_model(WorkcellConfig(arm="feetech"))
    hw = HardwareBackend(
        model=model,
        arm_joint_names=_FEETECH_JOINTS,
        lerobot_motor_names=_FEETECH_MOTORS,
        tcp_site=TCP_SITE,
    )
    with pytest.raises(RuntimeError, match="not connected"):
        hw.send_joint_targets([0.0] * 6)
    with pytest.raises(RuntimeError, match="not connected"):
        hw.set_gripper(0.5)
