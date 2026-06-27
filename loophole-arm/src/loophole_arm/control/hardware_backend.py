"""Hardware backend — the deployment target (Feetech arm via LeRobot).

Implements the same :class:`RobotInterface` as :class:`SimBackend`, so any
behaviour validated in simulation runs here unchanged. Joint state comes from
the servo encoders (via LeRobot's ``get_observation``); commands go to the
servos (via ``send_action``). The MuJoCo model is used **only** for forward
kinematics — to compute the TCP pose from encoder angles — never for dynamics.

Status: this is a structural stub. The control flow, unit conversions, joint
mapping, and FK are all real, but it has not yet been run against a physical
arm. Sites marked ``TODO(hardware)`` need verification once the arm is on the
bench — chiefly calibration sign/offset conventions and gripper command units.

Design choices that matter for sim-to-real:
  * Joint order is pinned to the MuJoCo model's arm-joint order; the LeRobot
    motor names are mapped to that order explicitly, so encoder readings and
    IK solutions always refer to the same joints.
  * Angles are radians at this interface (matching sim and IK); conversion to
    the servo's units happens only at the boundary.
"""
from __future__ import annotations

import time
from collections.abc import Sequence
from dataclasses import dataclass, field

import mujoco
import numpy as np
from numpy.typing import NDArray

from loophole_arm.control.interface import RobotInterface


@dataclass
class HardwareBackend(RobotInterface):
    """Feetech-arm implementation of :class:`RobotInterface`.

    Parameters
    ----------
    model:
        Compiled MuJoCo model carrying the arm geometry and the TCP site.
        Used for forward kinematics only.
    arm_joint_names:
        Arm joint names in the MuJoCo model, in order. Defines the canonical
        joint ordering for state and commands.
    lerobot_motor_names:
        The LeRobot motor names corresponding 1:1 to ``arm_joint_names``.
        Maps encoder/command channels to the kinematic joints.
    tcp_site:
        Name of the TCP site in the model.
    port:
        Serial port of the Feetech bus, e.g. ``/dev/ttyUSB0``.
    control_hz:
        Target control-loop rate; ``step`` blocks to maintain it.
    """

    model: mujoco.MjModel
    arm_joint_names: list[str]
    lerobot_motor_names: list[str]
    tcp_site: str
    port: str = "/dev/ttyUSB0"
    control_hz: float = 20.0
    _robot: object = field(default=None, repr=False)
    _last_step: float = field(default=0.0, repr=False)

    def __post_init__(self) -> None:
        if len(self.arm_joint_names) != len(self.lerobot_motor_names):
            raise ValueError("arm_joint_names and lerobot_motor_names must align 1:1")
        # FK scratch state — never simulated, only mj_forward'd.
        self._fk_data = mujoco.MjData(self.model)
        self._arm_qpos_adr = [
            self.model.jnt_qposadr[self.model.joint(n).id] for n in self.arm_joint_names
        ]
        self._tcp_site_id = self.model.site(self.tcp_site).id

    # ── Lifecycle ───────────────────────────────────────────────────────
    def connect(self) -> None:
        """Open the Feetech bus via the LeRobot robot wrapper."""
        # Lazy import so the sim path never requires lerobot to be installed.
        from loophole_arm.robot import LoopholeArm
        from loophole_arm.robot_config import LoopholeArmConfig

        self._robot = LoopholeArm(LoopholeArmConfig(port=self.port, use_degrees=False))
        self._robot.connect()  # type: ignore[attr-defined]
        self._last_step = time.perf_counter()

    def disconnect(self) -> None:
        if self._robot is not None:
            self._robot.disconnect()  # type: ignore[attr-defined]
            self._robot = None

    @property
    def is_connected(self) -> bool:
        return self._robot is not None and self._robot.is_connected  # type: ignore[attr-defined]

    # ── Introspection ───────────────────────────────────────────────────
    @property
    def n_arm_joints(self) -> int:
        return len(self.arm_joint_names)

    def kinematic_model(self) -> mujoco.MjModel:
        return self.model

    # ── State (read) ────────────────────────────────────────────────────
    def _read_observation(self) -> dict:
        if self._robot is None:
            raise RuntimeError("not connected; call connect() first")
        return self._robot.get_observation()  # type: ignore[attr-defined]

    @property
    def joint_positions(self) -> NDArray[np.float64]:
        """Arm joint angles (radians) read from the servo encoders."""
        obs = self._read_observation()
        # use_degrees=False → LeRobot returns radians already.
        # TODO(hardware): confirm sign/offset conventions match the URDF zero.
        return np.array([obs[f"{m}.pos"] for m in self.lerobot_motor_names], dtype=float)

    @property
    def joint_velocities(self) -> NDArray[np.float64]:
        """Arm joint velocities (rad/s).

        The Feetech bus does not expose a clean velocity channel in the LeRobot
        observation, so we report zeros. Closed-loop velocity control would
        require either finite-differencing positions or reading the servo's
        present-speed register. TODO(hardware): wire present-speed if needed.
        """
        return np.zeros(self.n_arm_joints, dtype=float)

    def end_effector_pose(self) -> NDArray[np.float64]:
        """TCP world position via forward kinematics on the current angles."""
        q = self.joint_positions
        self._fk_data.qpos[self._arm_qpos_adr] = q
        mujoco.mj_forward(self.model, self._fk_data)
        return self._fk_data.site(self._tcp_site_id).xpos.copy()

    # ── Commands (write) ────────────────────────────────────────────────
    def send_joint_targets(self, targets: Sequence[float]) -> None:
        """Command absolute arm joint angles (radians) to the servos."""
        if self._robot is None:
            raise RuntimeError("not connected; call connect() first")
        action = {
            f"{m}.pos": float(v)
            for m, v in zip(self.lerobot_motor_names, targets, strict=True)
        }
        self._robot.send_action(action)  # type: ignore[attr-defined]

    def set_gripper(self, closed_fraction: float) -> None:
        """Command the gripper. 0.0 = open, 1.0 = closed."""
        if self._robot is None:
            raise RuntimeError("not connected; call connect() first")
        frac = float(np.clip(closed_fraction, 0.0, 1.0))
        # TODO(hardware): map [0,1] to the gripper servo's real open/closed
        # encoder range during calibration. The LoopholeArm gripper uses a
        # 0..100 normalised range, so scale accordingly.
        self._robot.send_action({"gripper.pos": frac * 100.0})  # type: ignore[attr-defined]

    # ── Timing ──────────────────────────────────────────────────────────
    def step(self, dt: float) -> None:
        """Block until ``dt`` has elapsed, holding the control-loop rate."""
        target = self._last_step + dt
        now = time.perf_counter()
        if target > now:
            time.sleep(target - now)
        self._last_step = time.perf_counter()
