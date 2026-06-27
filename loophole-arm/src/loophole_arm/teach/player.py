"""The player — replays a taught :class:`Trajectory` (the "repeat" phase).

Each waypoint is executed through the same :class:`RobotController` used to
teach it, so replay gets smooth interpolation and full safety enforcement for
free, and behaves identically whether the backend is the simulator or the real
arm. Loading a trajectory taught in sim and replaying it on hardware is the
whole point — no re-teaching when the hardware arrives.
"""
from __future__ import annotations

import logging

from loophole_arm.control import RobotController
from loophole_arm.teach.trajectory import Trajectory, Waypoint

logger = logging.getLogger(__name__)


class TrajectoryPlayer:
    """Replays trajectories through a controller (sim or hardware)."""

    def __init__(self, robot: RobotController) -> None:
        self.robot = robot

    def play(self, trajectory: Trajectory, loops: int = 1) -> bool:
        """Execute ``trajectory`` ``loops`` times. Returns True if all steps ran.

        Safety is armed before motion. If a waypoint is rejected (unreachable
        target, safety fault), playback stops and returns False — replay never
        forces an unsafe or impossible move.
        """
        self.robot.enable()  # arm the safety supervisor (no-op if none)

        for loop in range(loops):
            if loops > 1:
                logger.info("loop %d/%d", loop + 1, loops)
            for i, wp in enumerate(trajectory.waypoints):
                ok = self._execute(wp)
                if not ok:
                    logger.error("playback stopped at waypoint #%d (%s)", i + 1, wp.kind)
                    return False
        logger.info("playback complete: %s (%d waypoints x %d)",
                    trajectory.name, len(trajectory), loops)
        return True

    def _execute(self, wp: Waypoint) -> bool:
        label = f" — {wp.label}" if wp.label else ""
        if wp.kind == "joint":
            assert wp.joints is not None
            logger.info("→ joints%s", label)
            self.robot.move_joints(wp.joints, duration=wp.duration)
            return True
        if wp.kind == "cartesian":
            assert wp.position is not None
            x, y, z = wp.position
            logger.info("→ move_to (%.3f, %.3f, %.3f)%s", x, y, z, label)
            return self.robot.move_to(x, y, z, duration=wp.duration)
        if wp.kind == "gripper":
            assert wp.gripper is not None
            logger.info("→ gripper %.2f%s", wp.gripper, label)
            if wp.gripper >= 0.5:
                self.robot.close_gripper()
            else:
                self.robot.open_gripper()
            return True
        if wp.kind == "dwell":
            logger.info("→ dwell %.1fs%s", wp.duration, label)
            self.robot._dwell(wp.duration)
            return True
        logger.error("unknown waypoint kind: %s", wp.kind)
        return False
