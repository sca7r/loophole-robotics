"""Plugin Manager + MockBackend + FSM wiring tests.

The mock lets these run in milliseconds with no MuJoCo, and fault injection
proves the ERROR-state path the PRD requires.
"""
from __future__ import annotations

import pytest

from loophole_arm.control.controller import RobotController
from loophole_arm.control.lifecycle import Event, State
from loophole_arm.control.mock_backend import MockBackend
from loophole_arm.control.plugins import (
    UnknownBackendError,
    available_backends,
    create_backend,
    get_backend_class,
    register_backend,
)
from loophole_arm.skills import Home, MoveJoint, Wait
from loophole_arm.skills.base import SkillStatus
from loophole_arm.skills.engine import SkillEngine


# ── Plugin registry ──────────────────────────────────────────────────────
def test_builtin_backends_registered() -> None:
    assert {"sim", "hardware", "mock", "remote"} <= set(available_backends())


def test_create_backend_by_name() -> None:
    b = create_backend("mock", n_joints=6)
    b.connect()
    assert b.is_connected
    assert b.n_arm_joints == 6


def test_unknown_backend_raises() -> None:
    with pytest.raises(UnknownBackendError, match="available"):
        get_backend_class("etherCAT_prototype")


def test_register_custom_backend() -> None:
    register_backend("mock2", lambda: MockBackend)
    assert "mock2" in available_backends()
    assert get_backend_class("mock2") is MockBackend


# ── Mock behaviour ───────────────────────────────────────────────────────
def _mock_robot() -> RobotController:
    backend = MockBackend(n_joints=6, home=(0.0, -0.5, 1.0, 0.0, 0.0, 0.0))
    backend.connect()
    return RobotController(backend=backend, solver=None, control_hz=100.0,
                           settle_time=0.0, home_pose=backend.home)


def test_mock_targets_become_positions() -> None:
    robot = _mock_robot()
    robot.move_joints([0.3, -0.4, 1.0, 0.0, 0.0, 0.0], duration=0.01)
    assert robot.backend.joint_positions[0] == pytest.approx(0.3)


def test_mock_rejects_wrong_length() -> None:
    b = MockBackend(n_joints=6)
    b.connect()
    with pytest.raises(ValueError):
        b.send_joint_targets([0.0, 0.0])


# ── FSM wiring in the SkillEngine ────────────────────────────────────────
def test_lifecycle_ready_executing_ready() -> None:
    robot = _mock_robot()
    robot.connect()                       # IDLE -> READY
    assert robot.lifecycle.state == State.READY
    engine = SkillEngine()
    result = engine.run(Wait(0.001), robot)
    assert result.ok
    # Ran through EXECUTING and returned to READY.
    events = [e for _, e, _ in robot.lifecycle.history]
    assert events == [Event.CONNECT, Event.START, Event.DONE]
    assert robot.lifecycle.state == State.READY


def test_backend_fault_puts_lifecycle_in_error() -> None:
    robot = _mock_robot()
    robot.connect()
    robot.backend.fail_next_command = True
    engine = SkillEngine()
    result = engine.run(MoveJoint(joints=(0.1, 0, 0, 0, 0, 0), duration=0.01), robot)
    assert result.status == SkillStatus.FAILED
    assert "MockCommandError" in result.detail
    assert robot.lifecycle.state == State.ERROR
    # Operator recovery: reset takes the robot back to READY.
    robot.lifecycle.transition(Event.RESET)
    assert robot.lifecycle.state == State.READY
    assert engine.run(Home(), robot).ok


def test_rejection_returns_to_ready_not_error() -> None:
    robot = _mock_robot()
    robot.connect()
    engine = SkillEngine()
    r = engine.run(MoveJoint(joints=(0.0, 0.0), duration=0.01), robot)  # type: ignore[arg-type]
    assert r.status == SkillStatus.REJECTED
    # The robot refused and stayed put: that is READY, not ERROR.
    assert robot.lifecycle.state == State.READY
