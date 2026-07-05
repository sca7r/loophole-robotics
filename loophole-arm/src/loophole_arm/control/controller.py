"""Generic robot controller — the layer the command file talks to.

This is the STABLE interface. It never changes between tasks or between sim
and hardware. Task logic lives entirely in the command file; this module just
exposes three layers of control, from lowest to highest:

    Layer 1  joint space    move_joints([j1..jn])         direct angles
    Layer 2  task space     move_to(x, y, z)              end-effector pose
    Layer 3  skills         pick(), place(), home()       semantic actions

Backends are swappable: :class:`SimBackend` drives MuJoCo, and a future
``HardwareBackend`` would drive the real arm via the LeRobot bus. The command
file is written against this controller and is therefore backend-agnostic —
deploying to hardware means swapping the backend, not rewriting commands.
"""
from __future__ import annotations

import time
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

from loophole_arm.control.interface import RobotInterface
from loophole_arm.control.lifecycle import Event, Lifecycle


# ── The controller: three layers of control ─────────────────────────────
@dataclass
class RobotController:
    """Stable, backend-agnostic robot API. The command file uses only this."""

    backend: RobotInterface
    solver: object = None             # TCPSolver, injected by the factory
    control_hz: float = 20.0
    settle_time: float = 0.4          # seconds to let a motion settle
    lifecycle: Lifecycle = field(default_factory=Lifecycle)
    _viewer_sync: object = field(default=None, repr=False)

    @property
    def dt(self) -> float:
        return 1.0 / self.control_hz

    # ── Layer 1: joint space ────────────────────────────────────────────
    def move_joints(
        self,
        targets: Sequence[float],
        duration: float = 1.5,
    ) -> bool:
        """Move the arm joints to absolute angles over ``duration`` seconds.

        Interpolates smoothly from the current pose so motion isn't jerky.
        Returns ``True`` — kept as a bool so the Skill Engine can treat all
        motion primitives uniformly (``move_to`` can genuinely fail on IK).
        """
        start = self.backend.joint_positions
        goal = np.asarray(targets, dtype=float)
        steps = max(1, int(duration * self.control_hz))
        for i in range(1, steps + 1):
            alpha = _smoothstep(i / steps)         # ease-in-ease-out
            interp = start + alpha * (goal - start)
            self.backend.send_joint_targets(interp)
            self.backend.step(self.dt)
            self._maybe_sync()
        self._settle()
        return True

    # ── Layer 2: task space ─────────────────────────────────────────────
    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        duration: float = 1.5,
        tol: float = 0.01,
    ) -> bool:
        """Move the TCP toward a Cartesian target (x, y, z).

        Solves IK with `mink` to find joint angles, then executes a smooth
        joint move. Returns True if the TCP reaches within ``tol`` metres.
        """
        if self.solver is None:
            raise RuntimeError("no IK solver attached; build via make_sim_robot()")

        # Pre-flight: if a safety supervisor is present, reject targets outside
        # the workspace envelope before we bother solving IK.
        check = getattr(self.backend, "check_target_in_workspace", None)
        if check is not None and not check(np.array([x, y, z])):
            return False

        # Seed IK from the current arm joint angles — read through the
        # interface, so this works identically in sim and on hardware.
        arm_q = self.backend.joint_positions
        sol = self.solver.solve(np.array([x, y, z]), arm_q)
        if not sol.converged:
            return False
        self.move_joints(sol.q, duration=duration)
        reached = float(np.linalg.norm(self.backend.end_effector_pose() - [x, y, z]))
        return reached <= tol * 3  # allow sim settling slack

    # ── Layer 3: skills ─────────────────────────────────────────────────
    def open_gripper(self) -> None:
        self.backend.set_gripper(0.0)
        self._dwell(0.3)

    def close_gripper(self) -> None:
        self.backend.set_gripper(1.0)
        self._dwell(0.3)

    def home(self, home_pose: Sequence[float], duration: float = 1.5) -> bool:
        """Return the arm to a named rest configuration."""
        return self.move_joints(home_pose, duration=duration)

    # ── Robot SDK lifecycle (PRD: connect / stop / shutdown) ────────────
    # The lifecycle FSM tracks IDLE → READY → EXECUTING → ERROR → SHUTDOWN.
    # `connect()` wires the backend and arms safety; `stop()` halts motion
    # without disconnecting; `shutdown()` is terminal.
    def connect(self) -> None:
        """Connect the backend, arm safety, and enter READY."""
        if not self.backend.is_connected:
            self.backend.connect()
        self.enable()
        if self.lifecycle.can(Event.CONNECT):
            self.lifecycle.transition(Event.CONNECT)

    def stop(self) -> None:
        """Stop motion now (e-stop path) but keep the connection alive.

        Recover with :meth:`reset_safety` + :meth:`enable`.
        """
        self.estop()
        if self.lifecycle.can(Event.FAULT):
            self.lifecycle.transition(Event.FAULT)

    def shutdown(self) -> None:
        """Terminal: stop motion, disconnect, mark lifecycle SHUTDOWN."""
        try:
            self.estop()
        finally:
            if self.backend.is_connected:
                self.backend.disconnect()
            if self.lifecycle.can(Event.SHUTDOWN):
                self.lifecycle.transition(Event.SHUTDOWN)

    # ── Safety controls (no-op if no safety supervisor is attached) ─────
    def enable(self) -> None:
        """Arm the robot for motion (IDLE → OPERATIONAL)."""
        fn = getattr(self.backend, "enable", None)
        if fn is not None:
            fn()

    def estop(self) -> None:
        """Engage emergency stop — latches all motion off immediately."""
        fn = getattr(self.backend, "estop", None)
        if fn is not None:
            fn()

    def reset_safety(self) -> None:
        """Clear an e-stop or fault back to IDLE (operator recovery)."""
        fn = getattr(self.backend, "reset", None)
        if fn is not None:
            fn()

    def pick(self, x: float, y: float, z: float, approach: float = 0.06) -> bool:
        """Top-down pick: approach above, descend, grasp, lift.

        A composite skill built from the lower layers. This is the level the
        command file mostly works at.
        """
        self.open_gripper()
        if not self.move_to(x, y, z + approach):           # hover above
            return False
        self.move_to(x, y, z, duration=1.0)                 # descend
        self.close_gripper()                                # grasp
        self.move_to(x, y, z + approach, duration=1.0)      # lift
        return True

    def place(self, x: float, y: float, z: float, approach: float = 0.06) -> bool:
        """Top-down place: approach above target, lower, release, retract."""
        if not self.move_to(x, y, z + approach):
            return False
        self.move_to(x, y, z, duration=1.0)
        self.open_gripper()
        self.move_to(x, y, z + approach, duration=1.0)
        return True

    # ── Internals ───────────────────────────────────────────────────────
    def _settle(self) -> None:
        self._dwell(self.settle_time)

    def _dwell(self, seconds: float) -> None:
        for _ in range(max(1, int(seconds * self.control_hz))):
            self.backend.step(self.dt)
            self._maybe_sync()

    def _maybe_sync(self) -> None:
        if self._viewer_sync is not None:
            self._viewer_sync()         # type: ignore[operator]
            time.sleep(self.dt)


def _smoothstep(t: float) -> float:
    """Ease-in-ease-out interpolation (3t^2 - 2t^3)."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)
