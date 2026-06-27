"""MuJoCo simulation backend — the validation target.

Implements :class:`RobotInterface` by driving a MuJoCo model. This is where
behaviours are validated before they touch hardware. ``step`` integrates
physics; joint state comes from the simulator; commands are written to the
position actuators.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import mujoco
import numpy as np
from numpy.typing import NDArray

from loophole_arm.control.interface import RobotInterface


@dataclass
class SimBackend(RobotInterface):
    """MuJoCo-backed implementation of :class:`RobotInterface`."""

    model: mujoco.MjModel
    data: mujoco.MjData
    arm_joint_names: list[str]
    gripper_actuator: str
    tcp_site: str
    _connected: bool = field(default=False, repr=False)

    def __post_init__(self) -> None:
        self._arm_act_ids = [self.model.actuator(n).id for n in self.arm_joint_names]
        self._arm_qpos_adr = [
            self.model.jnt_qposadr[self.model.actuator(n).trnid[0]]
            for n in self.arm_joint_names
        ]
        self._arm_dof_adr = [
            self.model.jnt_dofadr[self.model.actuator(n).trnid[0]]
            for n in self.arm_joint_names
        ]
        self._grip_act_id = self.model.actuator(self.gripper_actuator).id
        self._tcp_site_id = self.model.site(self.tcp_site).id
        self._grip_lo, self._grip_hi = self.model.actuator_ctrlrange[self._grip_act_id]

    # ── Lifecycle ───────────────────────────────────────────────────────
    def connect(self) -> None:
        mujoco.mj_forward(self.model, self.data)
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── Introspection ───────────────────────────────────────────────────
    @property
    def n_arm_joints(self) -> int:
        return len(self._arm_act_ids)

    def kinematic_model(self) -> mujoco.MjModel:
        return self.model

    # ── State (read) ────────────────────────────────────────────────────
    @property
    def joint_positions(self) -> NDArray[np.float64]:
        return self.data.qpos[self._arm_qpos_adr].copy()

    @property
    def joint_velocities(self) -> NDArray[np.float64]:
        return self.data.qvel[self._arm_dof_adr].copy()

    def end_effector_pose(self) -> NDArray[np.float64]:
        """World position of the TCP site (the true control frame)."""
        return self.data.site(self._tcp_site_id).xpos.copy()

    # ── Commands (write) ────────────────────────────────────────────────
    def send_joint_targets(self, targets: Sequence[float]) -> None:
        for act_id, value in zip(self._arm_act_ids, targets, strict=True):
            self.data.ctrl[act_id] = value

    def set_gripper(self, closed_fraction: float) -> None:
        frac = float(np.clip(closed_fraction, 0.0, 1.0))
        self.data.ctrl[self._grip_act_id] = self._grip_lo + frac * (self._grip_hi - self._grip_lo)

    # ── Timing ──────────────────────────────────────────────────────────
    def step(self, dt: float) -> None:
        n = max(1, int(dt / self.model.opt.timestep))
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)
