"""Differential inverse kinematics via `mink`.

This replaces the previous hand-rolled damped-least-squares solver with
`mink` (https://github.com/kevinzakka/mink), a production-grade differential
IK library built natively on MuJoCo. mink formulates each IK step as a
quadratic program, giving us — for free — joint-limit handling, velocity
limits, and optional collision avoidance that a hand-rolled solver lacks.

We solve for a joint configuration that places the TCP site at a target
position (and optionally orientation), then hand those joint angles to the
controller to execute as a smooth trajectory.

Design note: mink operates on the *same* MuJoCo model we simulate and (later)
the same kinematic description we deploy. One model, one source of truth — no
separate Pinocchio URDF to keep in sync. That is the single biggest lever for
avoiding sim-to-real kinematic mismatch.
"""
from __future__ import annotations

from dataclasses import dataclass

import mink
import mujoco
import numpy as np
from numpy.typing import NDArray


@dataclass
class IKSolution:
    """Result of an IK solve."""

    q: NDArray[np.float64]      # joint configuration for the arm DoFs
    position_error: float        # metres, TCP vs. target
    converged: bool


class TCPSolver:
    """Position (and optional orientation) IK for a named TCP site.

    Parameters
    ----------
    model:
        The MuJoCo model containing the arm and the TCP site.
    tcp_site:
        Name of the site to drive to the target.
    arm_joint_names:
        Arm joint names in order. Defines which qpos slots the seed arm
        angles map to, so the solver can be seeded from either sim state or
        hardware encoder readings.
    position_cost / orientation_cost:
        Relative task weights. orientation_cost=0 means position-only IK
        (the common case for a parallel-jaw top-down grasp).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        tcp_site: str,
        arm_joint_names: list[str],
        position_cost: float = 1.0,
        orientation_cost: float = 0.0,
    ) -> None:
        self.model = model
        self.tcp_site = tcp_site
        self.arm_joint_names = arm_joint_names
        self.n_arm_joints = len(arm_joint_names)
        self._arm_qpos_adr = [
            model.jnt_qposadr[model.joint(n).id] for n in arm_joint_names
        ]

        self._configuration = mink.Configuration(model)
        self._task = mink.FrameTask(
            frame_name=tcp_site,
            frame_type="site",
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=1.0,
        )
        # Respect joint position limits during the solve.
        self._limits = [mink.ConfigurationLimit(model)]

    def _seed(self, arm_q: NDArray[np.float64]) -> None:
        """Write arm angles into a full configuration and load it."""
        q = self._configuration.q.copy()
        for adr, val in zip(self._arm_qpos_adr, arm_q, strict=True):
            q[adr] = val
        self._configuration.update(q)

    def solve(
        self,
        target_pos: NDArray[np.float64],
        arm_q: NDArray[np.float64],
        target_quat: NDArray[np.float64] | None = None,
        tol: float = 5e-3,
        max_iters: int = 200,
        dt: float = 0.02,
    ) -> IKSolution:
        """Solve IK from current ``arm_q`` to place the TCP at ``target_pos``.

        ``arm_q`` is the arm joint vector (length ``n_arm_joints``), read via
        the robot interface — so the same call works in sim and on hardware.
        Pure computation: does not touch any live robot or sim state.
        """
        # Seed the solver at the current arm configuration.
        self._seed(arm_q)

        # Build the SE3 target. Orientation defaults to the current TCP frame
        # when not specified (position-only objective).
        if target_quat is None:
            current = self._configuration.get_transform_frame_to_world(
                self.tcp_site, "site"
            )
            rotation = current.rotation()
        else:
            rotation = mink.SO3(wxyz=np.asarray(target_quat, dtype=float))

        target = mink.SE3.from_rotation_and_translation(
            rotation, np.asarray(target_pos, dtype=float)
        )
        self._task.set_target(target)

        err = np.inf
        for _ in range(max_iters):
            vel = mink.solve_ik(
                self._configuration, [self._task], dt, "daqp", limits=self._limits
            )
            self._configuration.integrate_inplace(vel, dt)
            err_vec = self._task.compute_error(self._configuration)
            err = float(np.linalg.norm(err_vec[:3]))  # translation error
            if err < tol:
                break

        q_full = self._configuration.q.copy()
        return IKSolution(
            q=np.array([q_full[adr] for adr in self._arm_qpos_adr]),
            position_error=err,
            converged=err < tol,
        )
