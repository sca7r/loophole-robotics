"""Evolution strategy — a population-based, gradient-free optimizer.

A clean separation from :mod:`loophole_arm.env` and :mod:`loophole_arm.rewards`
makes it trivial to swap in CMA-ES, PPO, or any other optimizer later without
touching the environment.
"""
from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

logger = logging.getLogger(__name__)


@dataclass
class OptimizerResult:
    """Final state of an optimization run."""

    best_params: NDArray[np.float64]
    best_reward: float
    reward_history: list[float] = field(default_factory=list)
    """Best-of-generation reward, length == generations."""


@dataclass
class EvolutionStrategy:
    """Plain (mu, lambda) evolution strategy with isotropic Gaussian mutation.

    Parameters
    ----------
    param_dim:
        Length of the genome (typically ``env.param_dim``).
    population:
        Candidates per generation.
    elite:
        Top-K candidates whose mean becomes the next generation's centre.
    sigma:
        Initial mutation scale.
    sigma_decay:
        Per-generation multiplicative decay of ``sigma``.
    init_scale:
        Std-dev of the initial Gaussian over the genome.
    seed:
        Reproducibility seed.
    """

    param_dim: int
    population: int = 32
    elite: int = 8
    sigma: float = 0.6
    sigma_decay: float = 0.97
    init_scale: float = 0.3
    seed: int = 0

    def __post_init__(self) -> None:
        if self.elite > self.population:
            raise ValueError("elite must be <= population")
        self._rng = np.random.default_rng(self.seed)

    def optimize(
        self,
        evaluate: Callable[[NDArray[np.float64]], float],
        generations: int,
        on_generation: Callable[[int, float, float], None] | None = None,
    ) -> OptimizerResult:
        """Run the optimization loop.

        Parameters
        ----------
        evaluate:
            Pure function ``params -> reward``. The optimizer maximizes the
            returned scalar.
        generations:
            Number of generations to run.
        on_generation:
            Optional callback called with ``(gen_idx, best_in_gen, sigma)``
            after each generation. Useful for live progress bars or logging.
        """
        mean = self._rng.normal(0.0, self.init_scale, size=self.param_dim)
        sigma = self.sigma

        best_params = mean.copy()
        best_reward = -np.inf
        history: list[float] = []

        for g in range(generations):
            noise = self._rng.normal(size=(self.population, self.param_dim))
            candidates = mean + sigma * noise
            rewards = np.fromiter(
                (evaluate(c) for c in candidates),
                dtype=np.float64,
                count=self.population,
            )

            elite_idx = np.argpartition(rewards, -self.elite)[-self.elite :]
            mean = candidates[elite_idx].mean(axis=0)

            gen_best_idx = int(np.argmax(rewards))
            gen_best = float(rewards[gen_best_idx])
            history.append(gen_best)

            if gen_best > best_reward:
                best_reward = gen_best
                best_params = candidates[gen_best_idx].copy()

            if on_generation is not None:
                on_generation(g, gen_best, sigma)

            sigma *= self.sigma_decay

        logger.info("optimization complete: best_reward=%.5f", best_reward)
        return OptimizerResult(
            best_params=best_params,
            best_reward=best_reward,
            reward_history=history,
        )
