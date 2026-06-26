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

from loophole_arm.scene import SceneConfig, build_model


@dataclass
class RolloutResult:
    """What happened during a single rollout."""

    final_cup_pos: NDArray[np.float64]    # (3,) — x, y, z of the cup at the end
    peak_cup_z: float                     # max cup height reached during the rollout
    final_tcp_pos: NDArray[np.float64]    # (3,) — gripper position at the end
    final_cup_tcp_dist: float             # how far the cup ended up from the gripper
    arm_path_length: float                # cumulative joint-space travel (penalty term)
    contacts_with_cup: int                # number of timesteps with arm-cup contact


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
        self._model: mujoco.MjModel = build_model(self.scene)
        self._data: mujoco.MjData = mujoco.MjData(self._model)
        self._cup_qadr = int(self._model.joint("cup_free").qposadr[0])
        self._tcp_body = self._model.body("gripper_base_mount").id
        self._cup_geom = self._model.geom("cup_geom").id

        # Actuator ranges; arm = 6 DoF, gripper = 1 DoF (Robotiq closure)
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

        # Initialise the arm at the home pose
        d.qpos[:6] = self.scene.home_qpos
        mujoco.mj_forward(m, d)

        setpoints = self.decode(params)
        dt = m.opt.timestep
        steps_per_wp = max(1, int((self.sim_seconds / self.n_waypoints) / dt))

        peak_z = -np.inf
        path_len = 0.0
        contacts = 0
        last_qpos = d.qpos[:6].copy()

        for w in range(self.n_waypoints):
            d.ctrl[:] = setpoints[w]
            for _ in range(steps_per_wp):
                mujoco.mj_step(m, d)

                cup_z = d.qpos[self._cup_qadr + 2]
                if cup_z > peak_z:
                    peak_z = cup_z

                # Cumulative joint travel — used by motion-penalty rewards
                qpos_now = d.qpos[:6]
                path_len += float(np.linalg.norm(qpos_now - last_qpos))
                last_qpos = qpos_now.copy()

                # Count timesteps the cup is in contact with anything on the arm
                for c in range(d.ncon):
                    con = d.contact[c]
                    if con.geom1 == self._cup_geom or con.geom2 == self._cup_geom:
                        contacts += 1
                        break

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
