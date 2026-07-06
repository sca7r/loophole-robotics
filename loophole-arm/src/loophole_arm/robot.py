"""LeRobot-compatible Robot implementation for the Loophole Arm.

This wraps :class:`lerobot.motors.feetech.FeetechMotorsBus` so the arm shows
up as a standard LeRobot follower robot. Once installed, ``lerobot``'s CLI
tools (teleoperate, record, calibrate, train) detect this robot
automatically via the ``loophole_arm`` registry key.

The motor layout below mirrors the URDF in ``robots/feetech/`` — six
revolute joints plus a prismatic gripper, all on a single serial bus.
"""

from __future__ import annotations

import logging
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from loophole_arm.robot_config import LoopholeArmConfig

logger = logging.getLogger(__name__)

# Servo bus layout. IDs are factory-flashed once with ``lerobot-setup-motors``;
# after that, ID-to-joint mapping never changes.
_MOTOR_LAYOUT: dict[str, tuple[int, str]] = {
    "shoulder_pan": (1, "sts3215"),
    "shoulder_lift": (2, "sts3215"),
    "elbow_flex": (3, "sts3215"),
    "wrist_flex": (4, "sts3215"),
    "wrist_roll": (5, "sts3215"),
    "wrist_yaw": (6, "sts3215"),
    "gripper": (7, "sts3215"),
}


class LoopholeArm(Robot):
    """A 6-DOF Feetech-servo follower arm with a 1-DOF gripper."""

    config_class = LoopholeArmConfig
    name = "loophole_arm"

    def __init__(self, config: LoopholeArmConfig) -> None:
        super().__init__(config)
        self.config = config

        body_norm = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        motors: dict[str, Motor] = {}
        for joint, (servo_id, model) in _MOTOR_LAYOUT.items():
            # Gripper position is asymmetric (mostly closed → open), so it
            # benefits from a [0, 100] normalisation rather than [-100, 100].
            norm = MotorNormMode.RANGE_0_100 if joint == "gripper" else body_norm
            motors[joint] = Motor(servo_id, model, norm)

        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(self.config.cameras)

    # ── LeRobot Robot API ────────────────────────────────────────────────
    @property
    def _motor_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in self.bus.motors}

    @property
    def _camera_features(self) -> dict[str, tuple]:
        return {name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motor_features, **self._camera_features}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motor_features

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(c.is_connected for c in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Open the serial bus, optionally calibrate, attach cameras."""
        self.bus.connect()
        if calibrate and not self.is_calibrated:
            logger.info("calibration mismatch or absent — running calibration")
            self.calibrate()
        for cam in self.cameras.values():
            cam.connect()
        self.configure()
        logger.info("%s connected", self)

    @check_if_not_connected
    def disconnect(self) -> None:
        if self.config.disable_torque_on_disconnect:
            self.bus.disable_torque()
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    def configure(self) -> None:
        """Set runtime parameters on each servo (PID gains, operating mode)."""
        # Position-mode control with conservative PID — tighter PID values
        # cause oscillation on the lightweight Feetech servos.
        for motor_name in self.bus.motors:
            self.bus.write("P_Coefficient", motor_name, 32)
            self.bus.write("I_Coefficient", motor_name, 0)
            self.bus.write("D_Coefficient", motor_name, 32)

    def calibrate(self) -> None:
        """Run the standard LeRobot calibration routine."""
        # Delegate to bus calibration — same procedure as SO-100/SO-101.
        # Users follow ``lerobot-calibrate --robot.type=loophole_arm``.
        from lerobot.motors.feetech import OperatingMode

        self.bus.disable_torque()
        for motor_name in self.bus.motors:
            self.bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
        self.bus.run_arm_calibration()  # type: ignore[attr-defined]

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        """Read joint positions + camera frames."""
        obs: RobotObservation = {}
        positions = self.bus.sync_read("Present_Position")
        for joint, pos in positions.items():
            obs[f"{joint}.pos"] = float(pos)
        for name, cam in self.cameras.items():
            obs[name] = cam.async_read()
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Send goal positions to the servos with relative-target clamping."""
        from lerobot.robots.utils import ensure_safe_goal_position

        goal = {k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")}
        present = self.bus.sync_read("Present_Position")
        safe = ensure_safe_goal_position(
            goal, present, max_relative_target=self.config.max_relative_target
        )
        self.bus.sync_write("Goal_Position", safe)
        return {f"{j}.pos": v for j, v in safe.items()}
