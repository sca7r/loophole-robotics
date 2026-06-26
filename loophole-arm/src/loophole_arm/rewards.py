"""Reward functions for the cup-lift task.

Each reward consumes a :class:`RolloutResult` and returns a scalar score that
the optimizer maximises. The naming reflects what each reward measures, not
how it behaves — the whole project is about discovering how each one fails.
"""
from __future__ import annotations

import math
from collections.abc import Callable

from loophole_arm.env import RolloutResult

RewardFn = Callable[[RolloutResult], float]


def naive_peak_height(r: RolloutResult) -> float:
    """Reward only the peak cup height. Classic loophole: the arm learns to fling."""
    return r.peak_cup_z


def shaped_lift(r: RolloutResult) -> float:
    """Final height, weighted by how close the cup is to the gripper.

    Discourages flinging by demanding the cup ends near the end-effector.
    """
    proximity = math.exp(-10.0 * r.final_cup_tcp_dist)
    return r.final_cup_pos[2] * proximity


def strict_grasp(r: RolloutResult) -> float:
    """Shaped lift plus a contact-time bonus and a small motion penalty.

    A first attempt at "behave like a real grasp", but still has subtle holes
    the optimizer can find.
    """
    proximity = math.exp(-10.0 * r.final_cup_tcp_dist)
    contact_bonus = 0.001 * min(r.contacts_with_cup, 500)
    motion_penalty = 0.02 * r.arm_path_length
    return r.final_cup_pos[2] * proximity + contact_bonus - motion_penalty


REGISTRY: dict[str, RewardFn] = {
    "naive_peak_height": naive_peak_height,
    "shaped_lift": shaped_lift,
    "strict_grasp": strict_grasp,
}
"""Public registry of named rewards. Add new functions here to expose them via the CLI."""
