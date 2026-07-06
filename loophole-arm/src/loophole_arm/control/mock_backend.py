"""MockBackend: a RobotInterface with no physics, for fast tests.

Joint targets become joint positions instantly (or after a configurable lag).
No MuJoCo, no sockets, no sleeping. Useful for:

* unit-testing skills, the FSM, and sequence logic in milliseconds
* fault injection: set ``fail_next_command`` and the next write raises,
  letting tests prove ERROR-state handling without breaking real backends

Per the PRD's Plugin Manager section, "Mock backend" is a required plugin
alongside Simulation and Current hardware.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from loophole_arm.control.interface import RobotInterface


class MockCommandError(RuntimeError):
    """Raised by the mock when fault injection is armed."""


@dataclass
class MockBackend(RobotInterface):
    """Instant, physics-free RobotInterface implementation."""

    n_joints: int = 6
    home: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    fail_next_command: bool = False       # fault injection: next write raises
    _q: NDArray[np.float64] = field(default=None, repr=False)  # type: ignore[assignment]
    _gripper: float = 0.0
    _connected: bool = False
    command_log: list = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        self._q = np.array(self.home, dtype=float)

    # ── Lifecycle ───────────────────────────────────────────────────────
    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── Introspection ───────────────────────────────────────────────────
    @property
    def n_arm_joints(self) -> int:
        return self.n_joints

    def kinematic_model(self):
        raise RuntimeError("MockBackend has no kinematic model (no IK in mock tests)")

    # ── State (read) ────────────────────────────────────────────────────
    @property
    def joint_positions(self) -> NDArray[np.float64]:
        return self._q.copy()

    @property
    def joint_velocities(self) -> NDArray[np.float64]:
        return np.zeros(self.n_joints)

    def end_effector_pose(self) -> NDArray[np.float64]:
        # No kinematics: report a fake TCP derived from the first three joints
        # so motion is observable in tests without a model.
        return self._q[:3].copy()

    # ── Commands (write) ────────────────────────────────────────────────
    def send_joint_targets(self, targets: Sequence[float]) -> None:
        if self.fail_next_command:
            self.fail_next_command = False
            raise MockCommandError("injected fault")
        t = np.asarray(list(targets), dtype=float)
        if t.shape != (self.n_joints,):
            raise ValueError(f"expected {self.n_joints} targets, got {t.shape}")
        self._q = t
        self.command_log.append(("joints", tuple(t)))

    def set_gripper(self, closed_fraction: float) -> None:
        if self.fail_next_command:
            self.fail_next_command = False
            raise MockCommandError("injected fault")
        self._gripper = float(np.clip(closed_fraction, 0.0, 1.0))
        self.command_log.append(("gripper", self._gripper))

    # ── Timing ──────────────────────────────────────────────────────────
    def step(self, dt: float) -> None:
        pass                               # time is free in the mock
