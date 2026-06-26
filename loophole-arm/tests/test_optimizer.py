"""Tests for the evolution strategy."""
from __future__ import annotations

import numpy as np
import pytest

from loophole_arm.optimizer import EvolutionStrategy


def test_optimizes_a_quadratic() -> None:
    """ES should find the maximum of a simple, smooth function."""
    target = np.array([0.5, -0.3, 1.2])

    def reward(p: np.ndarray) -> float:
        return float(-np.sum((p - target) ** 2))

    es = EvolutionStrategy(param_dim=3, population=20, elite=5, sigma=0.5, seed=0)
    result = es.optimize(reward, generations=40)

    assert result.best_reward > -0.05  # very close to the optimum
    assert np.allclose(result.best_params, target, atol=0.2)


def test_history_length_equals_generations() -> None:
    es = EvolutionStrategy(param_dim=2, population=8, elite=2, seed=1)
    result = es.optimize(lambda p: -float(np.dot(p, p)), generations=10)
    assert len(result.reward_history) == 10


def test_elite_cannot_exceed_population() -> None:
    with pytest.raises(ValueError):
        EvolutionStrategy(param_dim=2, population=4, elite=10)


def test_callback_receives_generation_info() -> None:
    seen: list[tuple[int, float, float]] = []
    es = EvolutionStrategy(param_dim=2, population=4, elite=2, seed=0)
    es.optimize(
        lambda p: -float(np.dot(p, p)),
        generations=5,
        on_generation=lambda g, b, s: seen.append((g, b, s)),
    )
    assert [t[0] for t in seen] == [0, 1, 2, 3, 4]
    # sigma should be monotonically decreasing
    sigmas = [t[2] for t in seen]
    from itertools import pairwise
    assert all(a >= b for a, b in pairwise(sigmas))
