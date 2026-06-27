"""The teach session — builds a :class:`Trajectory` from taught poses.

Waypoint teaching: you set targets, the arm moves there in the simulator so you
can see the result, and the pose is recorded. No hardware required — the sim
produces the same joint states a physical arm would.

The session also exposes :meth:`capture`, which snapshots wherever the arm
currently is. That single primitive underlies *every* teaching method —
waypoint, keyboard teleop, or (later, on hardware) kinesthetic — because they
all reduce to "read the current joint state and save it".
"""
from __future__ import annotations

import logging

from loophole_arm.control import RobotController
from loophole_arm.teach.trajectory import Trajectory, Waypoint

logger = logging.getLogger(__name__)


class TeachSession:
    """Interactively (or programmatically) build a taught trajectory.

    Parameters
    ----------
    robot:
        A built :class:`RobotController` (sim now, hardware later — same API).
    name:
        Human name for the skill being taught.
    arm:
        Arm identifier recorded in the trajectory (e.g. ``"feetech"``).
    """

    def __init__(self, robot: RobotController, name: str, arm: str = "feetech") -> None:
        self.robot = robot
        self.trajectory = Trajectory(name=name, arm=arm, control_hz=robot.control_hz)

    # ── Capture (the shared primitive) ──────────────────────────────────
    def capture(self, label: str = "", duration: float = 1.5) -> Waypoint:
        """Snapshot the arm's current joint pose as a waypoint.

        Works regardless of how the arm got to this pose — moved by waypoint,
        teleop, or hand. This is the universal teach primitive.
        """
        joints = self.robot.backend.joint_positions.tolist()
        wp = Waypoint(kind="joint", joints=joints, duration=duration, label=label)
        self.trajectory.add(wp)
        logger.info("captured pose #%d %s", len(self.trajectory), f"({label})" if label else "")
        return wp

    # ── Waypoint teaching (move-then-record) ────────────────────────────
    def teach_cartesian(
        self,
        x: float,
        y: float,
        z: float,
        label: str = "",
        duration: float = 1.5,
    ) -> bool:
        """Move the TCP to a Cartesian target, then record it.

        The arm physically moves to the target in the sim so you can verify it
        before it's saved. Returns False (and records nothing) if the target is
        unreachable or outside the safety envelope.
        """
        if not self.robot.move_to(x, y, z, duration=duration):
            logger.warning("target (%.3f, %.3f, %.3f) unreachable — not recorded", x, y, z)
            return False
        self.trajectory.add(
            Waypoint(kind="cartesian", position=[x, y, z], duration=duration, label=label)
        )
        logger.info("taught cartesian #%d (%.3f, %.3f, %.3f)", len(self.trajectory), x, y, z)
        return True

    def teach_joints(
        self,
        joints: list[float],
        label: str = "",
        duration: float = 1.5,
    ) -> None:
        """Move to absolute joint angles, then record them."""
        self.robot.move_joints(joints, duration=duration)
        self.trajectory.add(
            Waypoint(kind="joint", joints=list(joints), duration=duration, label=label)
        )
        logger.info("taught joints #%d", len(self.trajectory))

    # ── Gripper & timing ────────────────────────────────────────────────
    def teach_gripper(self, closed: float, label: str = "") -> None:
        """Actuate the gripper (0 open … 1 closed) and record it."""
        closed = float(max(0.0, min(1.0, closed)))
        if closed >= 0.5:
            self.robot.close_gripper()
        else:
            self.robot.open_gripper()
        self.trajectory.add(Waypoint(kind="gripper", gripper=closed, label=label))
        logger.info("taught gripper #%d (%.2f)", len(self.trajectory), closed)

    def teach_dwell(self, seconds: float, label: str = "") -> None:
        """Record a pause."""
        self.trajectory.add(Waypoint(kind="dwell", duration=seconds, label=label))
        logger.info("taught dwell #%d (%.1fs)", len(self.trajectory), seconds)

    # ── Output ──────────────────────────────────────────────────────────
    def save(self, path: str) -> str:
        out = self.trajectory.save(path)
        logger.info("saved trajectory '%s' (%d waypoints) → %s",
                    self.trajectory.name, len(self.trajectory), out)
        return str(out)
