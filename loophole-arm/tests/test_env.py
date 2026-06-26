"""Tests for the rollout environment."""
from __future__ import annotations

import numpy as np

from loophole_arm.env import CupLiftEnv, RolloutResult


def test_param_dim_matches_actuators_times_waypoints(env: CupLiftEnv) -> None:
    assert env.param_dim == env.n_waypoints * env.n_actuators


def test_decode_respects_actuator_ranges(env: CupLiftEnv) -> None:
    # Extreme parameter values should still produce in-range setpoints
    extreme = np.full(env.param_dim, 1e6)
    decoded = env.decode(extreme)
    lo = env.model.actuator_ctrlrange[:, 0]
    hi = env.model.actuator_ctrlrange[:, 1]
    assert np.all(decoded >= lo - 1e-6)
    assert np.all(decoded <= hi + 1e-6)


def test_rollout_is_deterministic(env: CupLiftEnv) -> None:
    """Same params → same outcome. No hidden randomness."""
    rng = np.random.default_rng(42)
    params = rng.normal(size=env.param_dim)
    a = env.rollout(params)
    b = env.rollout(params)
    assert np.allclose(a.final_cup_pos, b.final_cup_pos)
    assert a.peak_cup_z == b.peak_cup_z


def test_rollout_returns_structured_record(env: CupLiftEnv) -> None:
    params = np.zeros(env.param_dim)
    r = env.rollout(params)
    assert isinstance(r, RolloutResult)
    assert r.final_cup_pos.shape == (3,)
    assert r.peak_cup_z >= r.final_cup_pos[2] - 1e-6  # peak ≥ final - eps
    assert r.arm_path_length >= 0.0
    assert r.contacts_with_cup >= 0
