"""Diagnostics tests: robot.health() per the PRD, with honest sentinels."""
from __future__ import annotations

from loophole_arm._version import __version__
from loophole_arm.control.controller import RobotController
from loophole_arm.control.lifecycle import Event, State
from loophole_arm.control.mock_backend import MockBackend
from loophole_arm.skills import MoveJoint, Wait
from loophole_arm.skills.engine import SkillEngine


def _robot() -> RobotController:
    b = MockBackend(n_joints=6, home=(0.0, -0.5, 1.0, 0.0, 0.0, 0.0))
    r = RobotController(backend=b, solver=None, control_hz=100.0,
                        settle_time=0.0, home_pose=b.home)
    r.connect()
    return r


def test_health_has_all_prd_fields() -> None:
    h = _robot().health()
    required = {
        "software_version", "backend", "connected", "lifecycle_state",
        "safety_state", "estop_engaged", "fault_history", "warnings",
        "runtime_seconds", "cycle_count", "servo_temperatures_c",
        "supply_voltage_v", "current_a", "firmware_version",
        "communication_health", "health_score", "sources",
    }
    assert required <= set(h)
    assert h["software_version"] == __version__


def test_sim_values_are_honest_sentinels() -> None:
    h = _robot().health()
    # The mock cannot measure these; the report must say None, not invent.
    assert h["servo_temperatures_c"] is None
    assert h["supply_voltage_v"] is None
    assert h["firmware_version"] is None
    assert "hardware only" in h["sources"]["servo_temperatures_c"]


def test_health_score_reflects_state() -> None:
    r = _robot()
    assert r.health()["health_score"] == 100          # connected, clean

    r.backend.fail_next_command = True                # inject a fault
    SkillEngine().run(MoveJoint(joints=(0.1, 0, 0, 0, 0, 0), duration=0.01), r)
    assert r.lifecycle.state == State.ERROR
    assert r.health()["health_score"] == 10           # ERROR state

    r.lifecycle.transition(Event.RESET)
    assert r.health()["health_score"] == 100          # recovered

    r.shutdown()
    assert r.health()["health_score"] == 0            # disconnected


def test_cycle_count_increments_on_completed_sequence() -> None:
    r = _robot()
    engine = SkillEngine()
    assert r.health()["cycle_count"] == 0
    engine.run_sequence([Wait(0.001), Wait(0.001)], r)
    assert r.health()["cycle_count"] == 1
    # A sequence that stops early is not a completed cycle.
    engine.run_sequence([MoveJoint(joints=(0.0, 0.0), duration=0.01), Wait(0.001)], r)  # type: ignore[arg-type]
    assert r.health()["cycle_count"] == 1


def test_runtime_tracked_after_connect() -> None:
    r = _robot()
    rt = r.health()["runtime_seconds"]
    assert rt is not None and rt >= 0.0
