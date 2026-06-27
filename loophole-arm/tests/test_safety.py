"""Tests for the safety supervisor.

Covers the state machine and each enforced limit. These are the guardrails that
must hold before any hardware bring-up, so they are tested thoroughly and
without needing a real arm.
"""
from __future__ import annotations

import numpy as np
import pytest

from loophole_arm.control import (
    SafetyBackend,
    SafetyLimits,
    SafetyState,
    SafetyViolation,
    make_sim_robot,
)


def _robot():
    return make_sim_robot(arm="feetech")  # safety on by default


# ── State machine ───────────────────────────────────────────────────────
def test_starts_operational_after_build() -> None:
    robot, *_ = _robot()
    assert robot.backend.state == SafetyState.OPERATIONAL


def test_estop_latches_and_blocks_motion() -> None:
    robot, model, data, home = _robot()
    robot.home(home)
    robot.estop()
    assert robot.backend.state == SafetyState.ESTOP

    before = robot.backend.joint_positions.copy()
    robot.move_to(0.18, 0.08, 0.18)
    after = robot.backend.joint_positions.copy()
    assert np.allclose(before, after, atol=1e-3), "motion must be blocked under e-stop"


def test_reset_then_enable_recovers() -> None:
    robot, model, data, home = _robot()
    robot.estop()
    robot.reset_safety()
    assert robot.backend.state == SafetyState.IDLE
    robot.enable()
    assert robot.backend.state == SafetyState.OPERATIONAL
    assert robot.move_to(0.18, 0.08, 0.18)


def test_cannot_enable_from_estop_without_reset() -> None:
    robot, *_ = _robot()
    robot.estop()
    with pytest.raises(SafetyViolation, match="reset first"):
        robot.backend.enable()


# ── Limits ──────────────────────────────────────────────────────────────
def test_workspace_bound_rejects_far_target() -> None:
    robot, model, data, home = _robot()
    robot.home(home)
    assert not robot.move_to(0.9, 0.0, 0.5), "target far outside envelope must be rejected"


def test_gross_joint_command_faults() -> None:
    robot, model, data, home = _robot()
    robot.move_joints([10.0, 0, 0, 0, 0, 0])  # far past joint limit
    assert robot.backend.state == SafetyState.FAULT


def test_velocity_limit_caps_step() -> None:
    """A single large joint jump should be rate-limited, not passed through."""
    limits = SafetyLimits.feetech_default()
    robot, model, data, home = make_sim_robot(arm="feetech", limits=limits)
    # Establish reference, then command a big but in-bounds jump on joint 1
    # (0 → 0.3 rad, well within ±3.14). The 0.3 rad step exceeds the 0.15
    # per-tick cap, so a rate_limit event must be recorded.
    robot.backend.send_joint_targets(home)
    big = list(home)
    big[0] = home[0] + 0.3
    robot.backend.send_joint_targets(big)
    events = [e.kind for e in robot.backend.events]
    assert "rate_limit" in events


def test_limits_clamp_and_bounds_helpers() -> None:
    limits = SafetyLimits.feetech_default()
    # Clamp pushes an out-of-range joint back inside.
    q = np.array([99.0, 0, 0, 0, 0, 0])
    clamped = limits.clamp_joints(q)
    assert clamped[0] <= limits.joint_upper[0]
    # Workspace check.
    assert limits.point_in_workspace(np.array([0.2, 0.0, 0.2]))
    assert not limits.point_in_workspace(np.array([2.0, 0.0, 0.2]))


# ── Opt-out path (reward-hacking sim) ───────────────────────────────────
def test_safety_can_be_disabled() -> None:
    robot, model, data, home = make_sim_robot(arm="feetech", safety=False)
    assert not isinstance(robot.backend, SafetyBackend)
