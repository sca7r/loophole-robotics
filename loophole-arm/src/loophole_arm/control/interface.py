"""The robot interface — the single contract sim and hardware both implement.

This is the load-bearing abstraction for sim-to-real. The controller, the IK
solver, and every command are written against :class:`RobotInterface`. Four
implementations satisfy it:

    SimBackend       drives MuJoCo             -> validate here
    HardwareBackend  drives the Feetech arm    -> deploy here
    RemoteBackend    drives a loophole-armd    -> multi-terminal control
    MockBackend      drives nothing, instantly -> fast tests, fault injection

Because the controller only ever sees this interface, moving a validated
behaviour from simulation to hardware is a one-line backend swap — the command
code does not change. That property is the whole point of the architecture, so
the interface is deliberately small and free of any MuJoCo-specific surface
(no ``qpos``/``data`` leaking through): a real servo bus has joint encoders and
goal positions, not a physics state vector.

Kinematics note: both backends expose the *same* compiled kinematic model via
:meth:`kinematic_model`. The IK/FK math runs on that model in both worlds —
sim additionally steps dynamics on it; hardware uses it only for kinematics and
reads true joint angles from the encoders. One kinematic source of truth across
sim and real is the single biggest lever against sim-to-real mismatch.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence

import mujoco
import numpy as np
from numpy.typing import NDArray


class RobotInterface(ABC):
    """The contract every backend (sim or hardware) must satisfy."""

    # ── Lifecycle ───────────────────────────────────────────────────────
    @abstractmethod
    def connect(self) -> None:
        """Establish the connection (open serial bus / init sim state)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release the connection cleanly (disable torque / free sim)."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the backend is ready to read state and accept commands."""

    # ── Introspection ───────────────────────────────────────────────────
    @property
    @abstractmethod
    def n_arm_joints(self) -> int:
        """Number of actuated arm DoFs (excludes the gripper)."""

    @abstractmethod
    def kinematic_model(self) -> mujoco.MjModel:
        """The compiled MuJoCo kinematic model used for IK/FK.

        Identical in sim and on hardware — it describes the arm's geometry,
        not its dynamics backend.
        """

    # ── State (read) ────────────────────────────────────────────────────
    @property
    @abstractmethod
    def joint_positions(self) -> NDArray[np.float64]:
        """Current arm joint angles in radians, length ``n_arm_joints``."""

    @property
    @abstractmethod
    def joint_velocities(self) -> NDArray[np.float64]:
        """Current arm joint velocities in rad/s, length ``n_arm_joints``."""

    @abstractmethod
    def end_effector_pose(self) -> NDArray[np.float64]:
        """World-frame XYZ position of the TCP (tool center point), metres."""

    # ── Commands (write) ────────────────────────────────────────────────
    @abstractmethod
    def send_joint_targets(self, targets: Sequence[float]) -> None:
        """Command absolute arm joint angles (radians), length ``n_arm_joints``."""

    @abstractmethod
    def set_gripper(self, closed_fraction: float) -> None:
        """Command the gripper. ``0.0`` = fully open, ``1.0`` = fully closed."""

    # ── Perception (optional) ───────────────────────────────────────────
    def object_positions(self) -> dict[str, list[float]]:
        """Live world positions of pickable objects, name -> [x, y, z].

        In simulation this is free: the physics state knows every pose, so
        SimBackend (and RemoteBackend, over the wire) return real values.
        Hardware returns {} until a vision system provides perception; the
        method exists so application code written against sim keeps working
        unchanged when that lands.
        """
        return {}

    # ── Timing ──────────────────────────────────────────────────────────
    @abstractmethod
    def step(self, dt: float) -> None:
        """Advance one control tick of ``dt`` seconds.

        Sim: integrate physics for ``dt``. Hardware: block until ``dt`` has
        elapsed so the control loop runs at a fixed rate.
        """
