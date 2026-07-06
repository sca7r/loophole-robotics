"""The safety supervisor — defense-in-depth between controller and backend.

:class:`SafetyBackend` wraps *any* :class:`RobotInterface` (sim or hardware)
and is itself a :class:`RobotInterface`, so the controller cannot tell it is
there. Every command passes through it and is validated before reaching the
real backend. This is the decorator pattern, and it is deliberately the same
shape as a ``ros2_control`` safety validator — when this project adopts ROS 2,
this logic moves into a hardware-interface plugin almost verbatim.

What it enforces, every tick:
  1. State machine — commands only flow in OPERATIONAL.
  2. Joint limits — targets clamped to hard bounds; gross breaches FAULT.
  3. Velocity limits — per-tick joint step capped (rate limiting).
  4. Workspace bounds — TCP target must lie in the allowed Cartesian box.
  5. E-stop — latches motion off immediately until explicitly reset.

Important honesty about scope: this is a *software* safety layer. As the
robotics literature stresses, functional real-time is not hard real-time — a
Python shim cannot guarantee safety if the process or kernel dies. On hardware
this must be backed by a firmware-level torque/velocity watchdog. This layer's
job is to catch logic errors early and make unsafe commands impossible to
express through the normal control path.
"""
from __future__ import annotations

import logging
from collections.abc import Sequence

import mujoco
import numpy as np
from numpy.typing import NDArray

from loophole_arm.control.interface import RobotInterface
from loophole_arm.control.limits import (
    SafetyEvent,
    SafetyLimits,
    SafetyState,
    SafetyViolation,
)

logger = logging.getLogger(__name__)


class SafetyBackend(RobotInterface):
    """Validating wrapper around a concrete backend.

    Parameters
    ----------
    inner:
        The real backend (sim or hardware) to protect.
    limits:
        The safety envelope to enforce.
    raise_on_violation:
        If True, a limit breach raises :class:`SafetyViolation`. If False
        (default), it trips FAULT and the command is dropped — closer to how
        hardware behaves (halt, don't crash).
    """

    def __init__(
        self,
        inner: RobotInterface,
        limits: SafetyLimits,
        raise_on_violation: bool = False,
    ) -> None:
        self._inner = inner
        self._limits = limits
        self._raise = raise_on_violation
        self._state = SafetyState.IDLE
        self._events: list[SafetyEvent] = []
        self._last_targets: NDArray[np.float64] | None = None

    # ── State machine ───────────────────────────────────────────────────
    @property
    def state(self) -> SafetyState:
        return self._state

    @property
    def events(self) -> list[SafetyEvent]:
        return list(self._events)

    def enable(self) -> None:
        """Transition IDLE → OPERATIONAL. Refused if estopped or faulted."""
        if self._state in (SafetyState.ESTOP, SafetyState.FAULT):
            raise SafetyViolation(f"cannot enable from {self._state.value}; reset first")
        self._state = SafetyState.OPERATIONAL
        logger.info("safety: OPERATIONAL")

    def estop(self) -> None:
        """Latch an emergency stop. Idempotent; always safe to call."""
        self._state = SafetyState.ESTOP
        self._record("estop", "emergency stop engaged")
        logger.warning("safety: ESTOP engaged")

    def reset(self) -> None:
        """Clear ESTOP/FAULT back to IDLE. Operator-initiated recovery."""
        prev = self._state
        self._state = SafetyState.IDLE
        self._last_targets = None
        logger.info("safety: reset %s → IDLE", prev.value)

    def _fault(self, detail: str) -> None:
        self._state = SafetyState.FAULT
        self._record("fault", detail)
        logger.error("safety: FAULT — %s", detail)
        if self._raise:
            raise SafetyViolation(detail)

    def _record(self, kind: str, detail: str) -> None:
        self._events.append(SafetyEvent(kind=kind, detail=detail, state=self._state))

    def _operational(self) -> bool:
        return self._state == SafetyState.OPERATIONAL

    # ── Validated commands ──────────────────────────────────────────────
    def send_joint_targets(self, targets: Sequence[float]) -> None:
        """Validate, rate-limit, and clamp a joint command before forwarding."""
        if not self._operational():
            self._record("blocked", f"joint command in state {self._state.value}")
            return

        q = np.asarray(targets, dtype=float)

        # Gross out-of-bounds → fault (someone asked for something very wrong).
        if not self._limits.joints_in_bounds(q):
            self._fault(f"joint target out of bounds: {q.round(3)}")
            return

        # Clamp to hard bounds (small overruns corrected silently).
        q = self._limits.clamp_joints(q)

        # Velocity limit: cap the per-tick step from the last commanded target.
        ref = self._last_targets if self._last_targets is not None else self._inner.joint_positions
        step = q - ref
        capped = np.clip(step, -self._limits.max_joint_step, self._limits.max_joint_step)
        if not np.allclose(step, capped):
            self._record("rate_limit", f"step {np.abs(step).max():.3f} capped")
        q = ref + capped

        self._last_targets = q.copy()
        self._inner.send_joint_targets(q)

    def set_gripper(self, closed_fraction: float) -> None:
        if not self._operational():
            self._record("blocked", f"gripper command in state {self._state.value}")
            return
        self._inner.set_gripper(closed_fraction)

    def check_target_in_workspace(self, xyz: NDArray[np.float64]) -> bool:
        """Pre-flight check a Cartesian target before an IK move.

        The controller calls this before solving IK so an out-of-envelope
        target is rejected cleanly rather than driving the arm to a clamp.
        """
        ok = self._limits.point_in_workspace(np.asarray(xyz, dtype=float))
        if not ok:
            self._record("workspace", f"target {np.asarray(xyz).round(3)} outside envelope")
        return ok

    # ── Pass-through (reads & lifecycle are not motion) ─────────────────
    def connect(self) -> None:
        self._inner.connect()
        self._state = SafetyState.IDLE

    def disconnect(self) -> None:
        self._inner.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._inner.is_connected

    @property
    def n_arm_joints(self) -> int:
        return self._inner.n_arm_joints

    def kinematic_model(self) -> mujoco.MjModel:
        return self._inner.kinematic_model()

    def object_positions(self) -> dict[str, list[float]]:
        # Perception is read-only; forward to the wrapped backend. Without
        # this, the interface's default {} would mask the sim's real data.
        return self._inner.object_positions()

    @property
    def joint_positions(self) -> NDArray[np.float64]:
        return self._inner.joint_positions

    @property
    def joint_velocities(self) -> NDArray[np.float64]:
        return self._inner.joint_velocities

    def end_effector_pose(self) -> NDArray[np.float64]:
        return self._inner.end_effector_pose()

    def step(self, dt: float) -> None:
        self._inner.step(dt)
