"""Tests for reward functions."""
from __future__ import annotations

import numpy as np

from loophole_arm.env import RolloutResult
from loophole_arm.rewards import REGISTRY, naive_peak_height, shaped_lift, strict_grasp


def _make(
    *,
    final_z: float,
    peak_z: float,
    tcp_dist: float,
    path_len: float = 0.0,
    contacts: int = 0,
) -> RolloutResult:
    return RolloutResult(
        final_cup_pos=np.array([0.0, 0.0, final_z]),
        peak_cup_z=peak_z,
        final_tcp_pos=np.array([0.0, 0.0, final_z + tcp_dist]),
        final_cup_tcp_dist=tcp_dist,
        arm_path_length=path_len,
        contacts_with_cup=contacts,
    )


def test_registry_exposes_all_rewards() -> None:
    assert set(REGISTRY) == {"naive_peak_height", "shaped_lift", "strict_grasp"}


def test_naive_is_just_peak() -> None:
    r = _make(final_z=0.1, peak_z=0.9, tcp_dist=0.5)
    assert naive_peak_height(r) == 0.9


def test_shaped_penalizes_distance() -> None:
    """For equal final heights, the closer one scores higher."""
    near = _make(final_z=0.3, peak_z=0.3, tcp_dist=0.02)
    far = _make(final_z=0.3, peak_z=0.3, tcp_dist=0.30)
    assert shaped_lift(near) > shaped_lift(far)


def test_naive_can_be_hacked() -> None:
    """A throwing trajectory beats a holding one under the naive reward."""
    throw = _make(final_z=0.0, peak_z=0.5, tcp_dist=0.40)  # cup on floor at end
    hold = _make(final_z=0.25, peak_z=0.25, tcp_dist=0.02)
    assert naive_peak_height(throw) > naive_peak_height(hold)
    # but shaped reward gets it right
    assert shaped_lift(hold) > shaped_lift(throw)


def test_strict_grasp_penalizes_excessive_motion() -> None:
    calm = _make(final_z=0.2, peak_z=0.2, tcp_dist=0.02, path_len=1.0, contacts=10)
    frantic = _make(final_z=0.2, peak_z=0.2, tcp_dist=0.02, path_len=20.0, contacts=10)
    assert strict_grasp(calm) > strict_grasp(frantic)
