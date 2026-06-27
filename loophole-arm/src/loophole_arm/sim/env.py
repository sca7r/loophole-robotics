"""Cup-lift environment: clean rollout API decoupled from any optimizer.

The environment is responsible for one thing: given a trajectory (a sequence of
controller setpoints), execute it deterministically and return a structured
record of what happened. Reward shaping lives in :mod:`loophole_arm.rewards`.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import mujoco
import numpy as np
from numpy.typing import NDArray

from loophole_arm.sim.scene import SceneConfig, build_model, end_effector_body


@dataclass
class RolloutResult:
    """What happened during a single rollout."""

    final_cup_pos: NDArray[np.float64]  # (3,) — x, y, z of the cup at the end
    peak_cup_z: float  # max cup height reached during the rollout
    final_tcp_pos: NDArray[np.float64]  # (3,) — gripper position at the end
    final_cup_tcp_dist: float  # how far the cup ended up from the gripper
    arm_path_length: float  # cumulative joint-space travel (penalty term)
    contacts_with_cup: int  # number of timesteps with arm-cup contact


@dataclass
class CupLiftEnv:
    """Deterministic open-loop rollout environment for the cup-lift task.

    Parameters
    ----------
    n_waypoints:
        Number of trajectory waypoints. Higher = more expressive, slower.
    sim_seconds:
        Total wall-clock duration of one rollout.
    scene:
        Optional :class:`SceneConfig` overriding scene defaults.
    """

    n_waypoints: int = 6
    sim_seconds: float = 3.0
    scene: SceneConfig = field(default_factory=SceneConfig)

    def __post_init__(self) -> None:
        self._resolved = self.scene.resolved()
        self._model: mujoco.MjModel = build_model(self.scene)
        self._data: mujoco.MjData = mujoco.MjData(self._model)
        self._cup_qadr = int(self._model.joint("cup_free").qposadr[0])
        self._tcp_body = self._model.body(end_effector_body(self._resolved.arm)).id
        self._cup_geom = self._model.geom("cup_geom").id
        self._n_arm_dofs = len(self._resolved.home_qpos)

        # Actuator ranges
        self._ctrl_lo = self._model.actuator_ctrlrange[:, 0].copy()
        self._ctrl_hi = self._model.actuator_ctrlrange[:, 1].copy()
        self._n_actuators = self._model.nu

    # ---- properties ---------------------------------------------------------
    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    @property
    def param_dim(self) -> int:
        """Dimensionality of the trajectory genome an optimizer must produce."""
        return self.n_waypoints * self._n_actuators

    @property
    def n_actuators(self) -> int:
        return self._n_actuators

    # ---- core API -----------------------------------------------------------
    def decode(self, params: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map an unconstrained genome to per-waypoint actuator setpoints.

        Uses a smooth ``tanh`` squash + affine remap into each actuator's
        ctrlrange, so the optimizer can search the full real line without
        hitting hard discontinuities at the bounds.
        """
        wp = np.asarray(params).reshape(self.n_waypoints, self._n_actuators)
        return self._ctrl_lo + (self._ctrl_hi - self._ctrl_lo) * (0.5 * (np.tanh(wp) + 1.0))

    def rollout(self, params: NDArray[np.float64]) -> RolloutResult:
        """Execute one open-loop trajectory and return its summary."""
        m, d = self._model, self._data
        mujoco.mj_resetData(m, d)

        # Initialise the arm at the home pose. UR5e: 6 dofs. Feetech: 7.
        d.qpos[: self._n_arm_dofs] = self._resolved.home_qpos
        mujoco.mj_forward(m, d)

        setpoints = self.decode(params)
        dt = m.opt.timestep
        steps_per_wp = max(1, int((self.sim_seconds / self.n_waypoints) / dt))

        peak_z = -np.inf
        path_len = 0.0
        contacts = 0
        last_qpos = d.qpos[: self._n_arm_dofs].copy()
        diverged = False

        for w in range(self.n_waypoints):
            d.ctrl[:] = setpoints[w]
            for _ in range(steps_per_wp):
                mujoco.mj_step(m, d)

                # Bail out on simulation divergence (NaN/Inf) — small Feetech
                # servos can produce unstable trajectories under aggressive
                # setpoint jumps. Treat as "no reward" rather than crashing.
                if not np.all(np.isfinite(d.qpos)):
                    diverged = True
                    break

                cup_z = d.qpos[self._cup_qadr + 2]
                if cup_z > peak_z:
                    peak_z = cup_z

                qpos_now = d.qpos[: self._n_arm_dofs]
                path_len += float(np.linalg.norm(qpos_now - last_qpos))
                last_qpos = qpos_now.copy()

                for c in range(d.ncon):
                    con = d.contact[c]
                    if con.geom1 == self._cup_geom or con.geom2 == self._cup_geom:
                        contacts += 1
                        break
            if diverged:
                break

        if diverged:
            # Return a degenerate result that scores poorly under all rewards.
            return RolloutResult(
                final_cup_pos=np.array([0.0, 0.0, -1.0]),
                peak_cup_z=-1.0,
                final_tcp_pos=np.array([0.0, 0.0, 0.0]),
                final_cup_tcp_dist=10.0,
                arm_path_length=1e6,
                contacts_with_cup=0,
            )

        final_cup = d.qpos[self._cup_qadr : self._cup_qadr + 3].copy()
        final_tcp = d.body(self._tcp_body).xpos.copy()

        return RolloutResult(
            final_cup_pos=final_cup,
            peak_cup_z=float(peak_z),
            final_tcp_pos=final_tcp,
            final_cup_tcp_dist=float(np.linalg.norm(final_cup - final_tcp)),
            arm_path_length=float(path_len),
            contacts_with_cup=contacts,
        )

    def evaluate(
        self,
        params: NDArray[np.float64],
        reward_fn: Callable[[RolloutResult], float],
    ) -> float:
        """Convenience: rollout, then score with ``reward_fn``."""
        return float(reward_fn(self.rollout(params)))
