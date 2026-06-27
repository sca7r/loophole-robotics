"""Safety limits and the safety state machine.

Defines the *policy* (what is allowed) separately from the *enforcement*
(:mod:`loophole_arm.control.safety`). Keeping the policy as plain data means it
can be loaded from a YAML config per deployment, audited, and unit-tested
without a robot.

The state machine mirrors the one used in industrial ``ros2_control`` safety
validators, so this maps cleanly onto a ROS 2 hardware-interface plugin later::

    IDLE → OPERATIONAL → ESTOP
                ↓
              FAULT

  IDLE         Connected, not yet enabled. Commands rejected.
  OPERATIONAL  Normal running. Commands validated and forwarded.
  ESTOP        Emergency stop latched. All motion halted until reset.
  FAULT        A limit violation tripped the system. Latched until reset.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


class SafetyState(enum.Enum):
    """Lifecycle state of the safety supervisor."""

    IDLE = "idle"
    OPERATIONAL = "operational"
    ESTOP = "estop"
    FAULT = "fault"


class SafetyViolation(Exception):  # noqa: N818  (domain term; not an internal error)
    """Raised (or recorded) when a command would breach a safety limit."""


@dataclass(frozen=True)
class SafetyLimits:
    """Per-robot safety envelope. All limits are inclusive bounds.

    Defaults are conservative and tuned for the Feetech arm. Override per
    deployment — a customer cell with a fence can open the workspace; a cell
    next to an operator should tighten velocity.

    Attributes
    ----------
    joint_lower / joint_upper:
        Hard joint angle bounds (radians), one per arm joint. Commands are
        clamped to these; exceeding them by more than ``joint_margin`` faults.
    joint_margin:
        Tolerance (radians) beyond the hard bounds before a FAULT trips. Small
        clamps are corrected silently; gross violations fault.
    max_joint_step:
        Maximum change per joint per control tick (radians). This is the
        velocity limit — it caps how far any joint may move in one ``dt``.
    workspace_min / workspace_max:
        Cartesian TCP envelope (metres), [x, y, z] lower/upper. The TCP target
        of any task-space move must lie inside this box.
    """

    joint_lower: NDArray[np.float64]
    joint_upper: NDArray[np.float64]
    max_joint_step: NDArray[np.float64]
    workspace_min: NDArray[np.float64]
    workspace_max: NDArray[np.float64]
    joint_margin: float = 0.05

    @staticmethod
    def feetech_default() -> SafetyLimits:
        """Conservative defaults for the Feetech arm on the workcell table."""
        return SafetyLimits(
            joint_lower=np.array([-3.14, -1.57, -1.57, -1.57, -1.46, -3.14]),
            joint_upper=np.array([3.14, 1.57, 1.57, 1.57, 1.57, 3.14]),
            # ~0.15 rad/tick at 20 Hz ≈ 3 rad/s — brisk but not violent.
            max_joint_step=np.full(6, 0.15),
            # Tabletop reachable box: in front of the arm, above the surface.
            workspace_min=np.array([-0.05, -0.30, 0.10]),
            workspace_max=np.array([0.35, 0.30, 0.45]),
        )

    def clamp_joints(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """Clamp a joint vector to the hard bounds."""
        return np.clip(q, self.joint_lower, self.joint_upper)

    def joints_in_bounds(self, q: NDArray[np.float64]) -> bool:
        """True if every joint is within bounds + margin."""
        return bool(
            np.all(q >= self.joint_lower - self.joint_margin)
            and np.all(q <= self.joint_upper + self.joint_margin)
        )

    def point_in_workspace(self, xyz: NDArray[np.float64]) -> bool:
        """True if a Cartesian point lies within the workspace box."""
        return bool(np.all(xyz >= self.workspace_min) and np.all(xyz <= self.workspace_max))


@dataclass
class SafetyEvent:
    """A recorded safety event, for logging / audit."""

    kind: str
    detail: str
    state: SafetyState
